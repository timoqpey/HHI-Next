

#include "BinEncoder.h"

#include "CommonLib/Rom.h"
#include "CommonLib/dtrace_next.h"

BinCounter::BinCounter()
  : m_CtxBinsCodedBuffer( Ctx::NumberOfContexts )
  , m_NumBinsCtx        ( m_CtxBinsCodedBuffer.data() )
  , m_NumBinsEP         ( 0 )
  , m_NumBinsTrm        ( 0 )
{}


void BinCounter::reset()
{
  for( std::size_t k = 0; k < m_CtxBinsCodedBuffer.size(); k++ )
  {
    m_NumBinsCtx[k] = 0;
  }
  m_NumBinsEP       = 0;
  m_NumBinsTrm      = 0;
}


uint32_t BinCounter::getAll() const
{
  uint32_t  count = m_NumBinsEP + m_NumBinsTrm;
  for( std::size_t k = 0; k < m_CtxBinsCodedBuffer.size(); k++ )
  {
    count += m_NumBinsCtx[k];
  }
  return count;
}





template <class BinProbModel>
BinEncoderBase::BinEncoderBase( const BinProbModel* dummy )
  : BinEncIf          ( dummy )
  , m_Bitstream       ( 0 )
  , m_Low             ( 0 )
  , m_Range           ( 0 )
  , m_bufferedByte    ( 0 )
  , m_numBufferedBytes( 0 )
  , m_bitsLeft        ( 0 )
{}

void BinEncoderBase::init( OutputBitstream* bitstream )
{
  m_Bitstream = bitstream;
}

void BinEncoderBase::uninit()
{
  m_Bitstream = 0;
}

void BinEncoderBase::start()
{
  m_Low               = 0;
  m_Range             = 510;
  m_bufferedByte      = 0xff;
  m_numBufferedBytes  = 0;
  m_bitsLeft          = 23;
  BinCounter::reset();
  m_BinStore. reset();
}

void BinEncoderBase::finish()
{
  if( m_Low >> ( 32 - m_bitsLeft ) )
  {
    m_Bitstream->write( m_bufferedByte + 1, 8 );
    while( m_numBufferedBytes > 1 )
    {
      m_Bitstream->write( 0x00, 8 );
      m_numBufferedBytes--;
    }
    m_Low -= 1 << ( 32 - m_bitsLeft );
  }
  else
  {
    if( m_numBufferedBytes > 0 )
    {
      m_Bitstream->write( m_bufferedByte, 8 );
    }
    while( m_numBufferedBytes > 1 )
    {
      m_Bitstream->write( 0xff, 8 );
      m_numBufferedBytes--;
    }
  }
  m_Bitstream->write( m_Low >> 8, 24 - m_bitsLeft );
}

void BinEncoderBase::restart()
{
  m_Low               = 0;
  m_Range             = 510;
  m_bufferedByte      = 0xff;
  m_numBufferedBytes  = 0;
  m_bitsLeft          = 23;
}

void BinEncoderBase::reset( int qp, int initId )
{
  Ctx::init( qp, initId );
  start();
}

void BinEncoderBase::resetBits()
{
  m_Low               = 0;
  m_bufferedByte      = 0xff;
  m_numBufferedBytes  = 0;
  m_bitsLeft          = 23;
  BinCounter::reset();
}

void BinEncoderBase::encodeBinEP( unsigned bin )
{
  DTRACE( g_trace_ctx, D_CABAC, "%d" "  " "%d" "  EP=%d \n", DTRACE_GET_COUNTER( g_trace_ctx, D_CABAC ), m_Range, bin );

  BinCounter::addEP();
  m_Low <<= 1;
  if( bin )
  {
    m_Low += m_Range;
  }
  m_bitsLeft--;
  if( m_bitsLeft < 12 )
  {
    writeOut();
  }
}

void BinEncoderBase::encodeBinsEP( unsigned bins, unsigned numBins )
{
  for(Int i = 0; i < numBins; i++)
  {
    DTRACE( g_trace_ctx, D_CABAC, "%d" "  " "%d" "  EP=%d \n", DTRACE_GET_COUNTER( g_trace_ctx, D_CABAC ), m_Range, ( bins >> ( numBins - 1 - i ) ) & 1 );
  }

  BinCounter::addEP( numBins );
  if( m_Range == 256 )
  {
    encodeAlignedBinsEP( bins, numBins );
    return;
  }
  while( numBins > 8 )
  {
    numBins          -= 8;
    unsigned pattern  = bins >> numBins;
    m_Low           <<= 8;
    m_Low            += m_Range * pattern;
    bins             -= pattern << numBins;
    m_bitsLeft       -= 8;
    if( m_bitsLeft < 12 )
    {
      writeOut();
    }
  }
  m_Low     <<= numBins;
  m_Low      += m_Range * bins;
  m_bitsLeft -= numBins;
  if( m_bitsLeft < 12 )
  {
    writeOut();
  }
}

void BinEncoderBase::encodeRemAbsEP( unsigned bins, unsigned goRicePar, bool useLimitedPrefixLength, int maxLog2TrDynamicRange, bool altRC )
{
  const unsigned threshold = ( altRC ? g_auiGoRiceRange[ goRicePar ] : COEF_REMAIN_BIN_REDUCTION ) << goRicePar;
  if( bins < threshold )
  {
    const unsigned bitMask  = ( 1 << goRicePar ) - 1;
    const unsigned length   = ( bins >> goRicePar ) + 1;
    encodeBinsEP( ( 1 << length ) - 2,  length );
    encodeBinsEP( bins & bitMask,       goRicePar);
  }
  else if (useLimitedPrefixLength)
  {
    const unsigned  maxPrefixLength = 32 - COEF_REMAIN_BIN_REDUCTION - maxLog2TrDynamicRange;
    unsigned        prefixLength    = 0;
    unsigned        codeValue       = ( bins >> goRicePar ) - COEF_REMAIN_BIN_REDUCTION;
    unsigned        suffixLength;
    if( codeValue >=  ( ( 1 << maxPrefixLength ) - 1 ) )
    {
      prefixLength = maxPrefixLength;
      suffixLength = maxLog2TrDynamicRange;
    }
    else
    {
      while( codeValue > ( ( 2 << prefixLength ) - 2 ) )
      {
        prefixLength++;
      }
      suffixLength = prefixLength + goRicePar + 1; //+1 for the separator bit
    }
    const unsigned totalPrefixLength  = prefixLength + COEF_REMAIN_BIN_REDUCTION;
    const unsigned bitMask            = ( 1 << goRicePar ) - 1;
    const unsigned prefix             = ( 1 << totalPrefixLength ) - 1;
    const unsigned suffix             = ( ( codeValue - ( (1 << prefixLength ) - 1 ) ) << goRicePar ) | ( bins & bitMask );
    encodeBinsEP( prefix, totalPrefixLength ); //prefix
    encodeBinsEP( suffix, suffixLength      ); //separator, suffix, and rParam bits
  }
  else
  {
    unsigned length = goRicePar;
    unsigned delta  = 1 << length;
    bins           -= threshold;
    while (bins >= delta )
    {
      bins -= delta;
      delta = 1 << (++length);
    }
    unsigned numBin = ( altRC ? g_auiGoRiceRange[ goRicePar ] : COEF_REMAIN_BIN_REDUCTION ) + length + 1 - goRicePar;
    encodeBinsEP( ( 1 << numBin ) - 2, numBin );
    encodeBinsEP( bins,                length );
  }
}

void BinEncoderBase::encodeBinTrm( unsigned bin )
{
  BinCounter::addTrm();
  m_Range -= 2;
  if( bin )
  {
    m_Low      += m_Range;
    m_Low     <<= 7;
    m_Range     = 2 << 7;
    m_bitsLeft -= 7;
  }
  else if( m_Range >= 256 )
  {
    return;
  }
  else
  {
    m_Low     <<= 1;
    m_Range   <<= 1;
    m_bitsLeft--;
  }
  if( m_bitsLeft < 12 )
  {
    writeOut();
  }
}

void BinEncoderBase::encodeBinsPCM( unsigned bins, unsigned numBins )
{
  m_Bitstream->write( bins, numBins );
}

void BinEncoderBase::align()
{
  m_Range = 256;
}

void BinEncoderBase::pcmAlignBits()
{
  finish();
  m_Bitstream->write( 1, 1 );
  m_Bitstream->writeAlignZero(); // pcm align zero
}

void BinEncoderBase::encodeAlignedBinsEP( unsigned bins, unsigned numBins )
{
  unsigned remBins = numBins;
  while( remBins > 0 )
  {
    //The process of encoding an EP bin is the same as that of coding a normal
    //bin where the symbol ranges for 1 and 0 are both half the range:
    //
    //  low = (low + range/2) << 1       (to encode a 1)
    //  low =  low            << 1       (to encode a 0)
    //
    //  i.e.
    //  low = (low + (bin * range/2)) << 1
    //
    //  which is equivalent to:
    //
    //  low = (low << 1) + (bin * range)
    //
    //  this can be generalised for multiple bins, producing the following expression:
    //
    unsigned binsToCode = std::min<unsigned>( remBins, 8); //code bytes if able to take advantage of the system's byte-write function
    unsigned binMask    = ( 1 << binsToCode ) - 1;
    unsigned newBins    = ( bins >> ( remBins - binsToCode ) ) & binMask;
    m_Low               = ( m_Low << binsToCode ) + ( newBins << 8 ); //range is known to be 256
    remBins            -= binsToCode;
    m_bitsLeft         -= binsToCode;
    if( m_bitsLeft < 12 )
    {
      writeOut();
    }
  }
}

void BinEncoderBase::writeOut()
{
  unsigned leadByte = m_Low >> ( 24 - m_bitsLeft );
  m_bitsLeft       += 8;
  m_Low            &= 0xffffffffu >> m_bitsLeft;
  if( leadByte == 0xff )
  {
    m_numBufferedBytes++;
  }
  else
  {
    if( m_numBufferedBytes > 0 )
    {
      unsigned carry  = leadByte >> 8;
      unsigned byte   = m_bufferedByte + carry;
      m_bufferedByte  = leadByte & 0xff;
      m_Bitstream->write( byte, 8 );
      byte            = ( 0xff + carry ) & 0xff;
      while( m_numBufferedBytes > 1 )
      {
        m_Bitstream->write( byte, 8 );
        m_numBufferedBytes--;
      }
    }
    else
    {
      m_numBufferedBytes  = 1;
      m_bufferedByte      = leadByte;
    }
  }
}



template <class BinProbModel>
TBinEncoder<BinProbModel>::TBinEncoder()
  : BinEncoderBase( static_cast<const BinProbModel*>    ( nullptr ) )
  , m_Ctx         ( static_cast<CtxStore<BinProbModel>&>( *this   ) )
{}

template <class BinProbModel>
void TBinEncoder<BinProbModel>::encodeBin( unsigned bin, unsigned ctxId )
{
  BinCounter::addCtx( ctxId );
  BinProbModel& rcProbModel = m_Ctx[ctxId];
  uint32_t      LPS         = rcProbModel.getLPS( m_Range );

  DTRACE( g_trace_ctx, D_CABAC, "%d" "  " "%d" "  " "[%d:%d]" "  " "%2d(MPS=%d)"  "  " "  -  " "%d" "\n", DTRACE_GET_COUNTER( g_trace_ctx, D_CABAC ), m_Range, m_Range - LPS, LPS, ( unsigned int ) ( rcProbModel.state() ), bin == rcProbModel.mps(), bin );

  m_Range   -=  LPS;
  if( bin != rcProbModel.mps() )
  {
    int numBits   = rcProbModel.getRenormBitsLPS( LPS );
    m_bitsLeft   -= numBits;
    m_Low        += m_Range;
    m_Low         = m_Low << numBits;
    m_Range       = LPS   << numBits;
    if( m_bitsLeft < 12 )
    {
      writeOut();
    }
  }
  else
  {
    if( m_Range < 256 )
    {
      int numBits   = rcProbModel.getRenormBitsRange( m_Range );
      m_bitsLeft   -= numBits;
      m_Low       <<= numBits;
      m_Range     <<= numBits;
      if( m_bitsLeft < 12 )
      {
        writeOut();
      }
    }
  }
  rcProbModel.update( bin );
  BinEncoderBase::m_BinStore.addBin( bin, ctxId );
}

template <class BinProbModel>
BinEncIf* TBinEncoder<BinProbModel>::getTestBinEncoder() const
{
  BinEncIf* testBinEncoder = 0;
  if( m_BinStore.inUse() )
  {
    testBinEncoder = new TBinEncoder<BinProbModel>();
  }
  return testBinEncoder;
}





template <class BinProbModel>
BitEstimatorBase::BitEstimatorBase( const BinProbModel* dummy )
  : BinEncIf      ( dummy )
{ 
  m_EstFracBits = 0; 
}

void BitEstimatorBase::encodeRemAbsEP( unsigned bins, unsigned goRicePar, bool useLimitedPrefixLength, int maxLog2TrDynamicRange, bool altRC )
{
  const unsigned threshold = ( altRC ? g_auiGoRiceRange[ goRicePar ] : COEF_REMAIN_BIN_REDUCTION ) << goRicePar;
  if( bins < threshold )
  {
    m_EstFracBits += BinProbModelBase::estFracBitsEP( ( bins >> goRicePar ) + 1 + goRicePar );
  }
  else if (useLimitedPrefixLength)
  {
    const unsigned  maxPrefixLength = 32 - COEF_REMAIN_BIN_REDUCTION - maxLog2TrDynamicRange;
    unsigned        prefixLength    = 0;
    unsigned        codeValue       = ( bins >> goRicePar ) - COEF_REMAIN_BIN_REDUCTION;
    unsigned        suffixLength;
    if( codeValue >=  ( ( 1 << maxPrefixLength ) - 1 ) )
    {
      prefixLength = maxPrefixLength;
      suffixLength = maxLog2TrDynamicRange;
    }
    else
    {
      while( codeValue > ( ( 2 << prefixLength ) - 2 ) )
      {
        prefixLength++;
      }
      suffixLength = prefixLength + goRicePar + 1; //+1 for the separator bit
    }
    m_EstFracBits += BinProbModelBase::estFracBitsEP( COEF_REMAIN_BIN_REDUCTION + prefixLength + suffixLength );
  }
  else
  {
    unsigned length = goRicePar;
    unsigned delta  = 1 << length;
    bins           -= threshold;
    while (bins >= delta )
    {
      bins -= delta;
      delta = 1 << (++length);
    }
    m_EstFracBits += BinProbModelBase::estFracBitsEP( ( altRC ? g_auiGoRiceRange[ goRicePar ] : COEF_REMAIN_BIN_REDUCTION ) + 1 + ( length << 1 ) - goRicePar );
  }
}

void BitEstimatorBase::align()
{
  static const uint64_t add   = BinProbModelBase::estFracBitsEP() - 1;
  static const uint64_t mask  = ~add;
  m_EstFracBits += add;
  m_EstFracBits &= mask;
}

void BitEstimatorBase::pcmAlignBits()
{
  uint64_t  numCurrBits = ( m_EstFracBits >> SCALE_BITS );
  uint64_t  filledBytes = ( numCurrBits + 8 ) >> 3; // including aligned_one_bit and aligned_zero_bits
  unsigned  bitsToAdd   = unsigned( ( filledBytes << 3 ) - numCurrBits );
  m_EstFracBits        += BinProbModelBase::estFracBitsEP( bitsToAdd );
}





template <class BinProbModel>
TBitEstimator<BinProbModel>::TBitEstimator()
  : BitEstimatorBase  ( static_cast<const BinProbModel*>    ( nullptr) )
  , m_Ctx             ( static_cast<CtxStore<BinProbModel>&>( *this  ) )
{}



template class TBinEncoder<BinProbModel_Std>;
template class TBinEncoder<BinProbModel_JMP>;
template class TBinEncoder<BinProbModel_JAW>;
template class TBinEncoder<BinProbModel_JMPAW>;

template class TBitEstimator<BinProbModel_Std>;
template class TBitEstimator<BinProbModel_JMP>;
template class TBitEstimator<BinProbModel_JAW>;
template class TBitEstimator<BinProbModel_JMPAW>;

