

#include "BinDecoder.h"
#include "CommonLib/Rom.h"
#if RExt__DECODER_DEBUG_BIT_STATISTICS
#include "CommonLib/CodingStatistics.h"
#endif

#include "CommonLib/dtrace_next.h"

#define CNT_OFFSET 0



template <class BinProbModel>
BinDecoderBase::BinDecoderBase( const BinProbModel* dummy )
  : Ctx         ( dummy )
  , m_Bitstream ( 0 )
  , m_Range     ( 0 )
  , m_Value     ( 0 )
  , m_bitsNeeded( 0 )
{}


void BinDecoderBase::init( InputBitstream* bitstream )
{
  m_Bitstream = bitstream;
}


void BinDecoderBase::uninit()
{
  m_Bitstream = 0;
}


void BinDecoderBase::start()
{
  CHECK( m_Bitstream->getNumBitsUntilByteAligned(), "Bitstream is not byte aligned." );
#if RExt__DECODER_DEBUG_BIT_STATISTICS
  CodingStatistics::UpdateCABACStat(STATS__CABAC_INITIALISATION, 512, 510, 0);
#endif
  m_Range       = 510;
  m_Value       = ( m_Bitstream->readByte() << 8 ) + m_Bitstream->readByte();
  m_bitsNeeded  = -8;
}


void BinDecoderBase::finish()
{
  unsigned lastByte;
  m_Bitstream->peekPreviousByte( lastByte );
  CHECK( ( ( lastByte << ( 8 + m_bitsNeeded ) ) & 0xff ) != 0x80,
        "No proper stop/alignment pattern at end of CABAC stream." );
}


void BinDecoderBase::reset( int qp, int initId )
{
  Ctx::init( qp, initId );
  start();
}


unsigned BinDecoderBase::decodeBinEP()
{
  m_Value            += m_Value;
  if( ++m_bitsNeeded >= 0 )
  {
    m_Value          += m_Bitstream->readByte();
    m_bitsNeeded      = -8;
  }
  unsigned bin = 0;
  unsigned SR  = m_Range << 7;
  if( m_Value >= SR )
  {
    m_Value   -= SR;
    bin        = 1;
  }
#if RExt__DECODER_DEBUG_BIT_STATISTICS
  CodingStatistics::IncrementStatisticEP( *ptype, 1, int(bin) );
#endif
  DTRACE( g_trace_ctx, D_CABAC, "%d" "  " "%d" "  EP=%d \n",  DTRACE_GET_COUNTER( g_trace_ctx, D_CABAC ), m_Range, bin );
  return bin;
}


unsigned BinDecoderBase::decodeBinsEP( unsigned numBins )
{
#if ENABLE_TRACING
  Int numBinsOrig = numBins;
#endif

  if( m_Range == 256 )
  {
    return decodeAlignedBinsEP( numBins );
  }
  unsigned remBins = numBins;
  unsigned bins    = 0;
  while(   remBins > 8 )
  {
    m_Value     = ( m_Value << 8 ) + ( m_Bitstream->readByte() << ( 8 + m_bitsNeeded ) );
    unsigned SR =   m_Range << 15;
    for( Int i = 0; i < 8; i++ )
    {
      bins += bins;
      SR  >>= 1;
      if( m_Value >= SR )
      {
        bins    ++;
        m_Value -= SR;
      }
    }
    remBins -= 8;
  }
  m_bitsNeeded   += remBins;
  m_Value       <<= remBins;
  if( m_bitsNeeded >= 0 )
  {
    m_Value      += m_Bitstream->readByte() << m_bitsNeeded;
    m_bitsNeeded -= 8;
  }
  unsigned SR = m_Range << ( remBins + 7 );
  for ( Int i = 0; i < remBins; i++ )
  {
    bins += bins;
    SR  >>= 1;
    if( m_Value >= SR )
    {
      bins    ++;
      m_Value -= SR;
    }
  }
#if RExt__DECODER_DEBUG_BIT_STATISTICS
  CodingStatistics::IncrementStatisticEP( *ptype, numBins, int(bins) );
#endif
#if ENABLE_TRACING
  for( Int i = 0; i < numBinsOrig; i++ )
  {
    DTRACE( g_trace_ctx, D_CABAC, "%d" "  " "%d" "  EP=%d \n", DTRACE_GET_COUNTER( g_trace_ctx, D_CABAC ), m_Range, ( bins >> ( numBinsOrig - 1 - i ) ) & 1 );
  }
#endif
  return bins;
}

unsigned BinDecoderBase::decodeRemAbsEP( unsigned goRicePar, bool useLimitedPrefixLength, int maxLog2TrDynamicRange, bool altRC )
{
  unsigned cutoff = altRC ? g_auiGoRiceRange[ goRicePar ] : COEF_REMAIN_BIN_REDUCTION;
  unsigned prefix = 0;
  if( useLimitedPrefixLength )
  {
    const unsigned  maxPrefix = 32 - maxLog2TrDynamicRange;
    unsigned        codeWord  = 0;
    do
    {
      prefix++;
      codeWord = decodeBinEP();
    }
    while( codeWord && prefix < maxPrefix );
    prefix -= 1 - codeWord;
  }
  else
  {
    while( decodeBinEP() )
    {
      prefix++;
    }
  }
  unsigned length = goRicePar, offset;
  if( prefix < cutoff )
  {
    offset    = prefix << goRicePar;
  }
  else
  {
    offset    = ( ( ( 1 << ( prefix - cutoff ) ) + cutoff - 1 ) << goRicePar );
    if( useLimitedPrefixLength )
    {
      length += ( prefix == ( 32 - maxLog2TrDynamicRange ) ? maxLog2TrDynamicRange - goRicePar : prefix - COEF_REMAIN_BIN_REDUCTION );
    }
    else
    {
      length += ( prefix - cutoff );
    }
  }
  return offset + decodeBinsEP( length );
}


unsigned BinDecoderBase::decodeBinTrm()
{
  m_Range    -= 2;
  unsigned SR = m_Range << 7;
  if( m_Value >= SR )
  {
#if RExt__DECODER_DEBUG_BIT_STATISTICS
    CodingStatistics::UpdateCABACStat     ( STATS__CABAC_TRM_BITS,       m_Range+2, 2, 1 );
    CodingStatistics::IncrementStatisticEP( STATS__BYTE_ALIGNMENT_BITS, -m_bitsNeeded, 0 );
#endif
    return 1;
  }
  else
  {
#if RExt__DECODER_DEBUG_BIT_STATISTICS
    CodingStatistics::UpdateCABACStat ( STATS__CABAC_TRM_BITS, m_Range+2, m_Range, 0 );
#endif
    if( m_Range < 256 )
    {
      m_Range += m_Range;
      m_Value += m_Value;
      if( ++m_bitsNeeded == 0 )
      {
        m_Value      += m_Bitstream->readByte();
        m_bitsNeeded  = -8;
      }
    }
    return 0;
  }
}


unsigned BinDecoderBase::decodeBinsPCM( unsigned numBins )
{
  unsigned bins = 0;
  m_Bitstream->read( numBins, bins );
#if RExt__DECODER_DEBUG_BIT_STATISTICS
  CodingStatistics::IncrementStatisticEP( STATS__CABAC_PCM_CODE_BITS, numBins, int(bins) );
#endif
  return bins;
}


void BinDecoderBase::align()
{
#if RExt__DECODER_DEBUG_BIT_STATISTICS
  CodingStatistics::UpdateCABACStat( STATS__CABAC_EP_BIT_ALIGNMENT, m_Range, 256, 0 );
#endif
  m_Range = 256;
}


unsigned BinDecoderBase::decodeAlignedBinsEP( unsigned numBins )
{
#if ENABLE_TRACING
  Int numBinsOrig = numBins;
#endif
  unsigned remBins = numBins;
  unsigned bins    = 0;
  while(   remBins > 0 )
  {
    // The MSB of m_Value is known to be 0 because range is 256. Therefore:
    //   > The comparison against the symbol range of 128 is simply a test on the next-most-significant bit
    //   > "Subtracting" the symbol range if the decoded bin is 1 simply involves clearing that bit.
    //  As a result, the required bins are simply the <binsToRead> next-most-significant bits of m_Value
    //  (m_Value is stored MSB-aligned in a 16-bit buffer - hence the shift of 15)
    //
    //    m_Value = |0|V|V|V|V|V|V|V|V|B|B|B|B|B|B|B|        (V = usable bit, B = potential buffered bit (buffer refills when m_bitsNeeded >= 0))
    //
    unsigned binsToRead = std::min<unsigned>( remBins, 8 ); //read bytes if able to take advantage of the system's byte-read function
    unsigned binMask    = ( 1 << binsToRead ) - 1;
    unsigned newBins    = ( m_Value >> (15 - binsToRead) ) & binMask;
    bins                = ( bins    << binsToRead) | newBins;
    m_Value             = ( m_Value << binsToRead) & 0x7FFF;
    remBins            -= binsToRead;
    m_bitsNeeded       += binsToRead;
    if( m_bitsNeeded >= 0 )
    {
      m_Value          |= m_Bitstream->readByte() << m_bitsNeeded;
      m_bitsNeeded     -= 8;
    }
  }
#if RExt__DECODER_DEBUG_BIT_STATISTICS
  CodingStatistics::IncrementStatisticEP( *ptype, numBins, int(bins) );
#endif
#if ENABLE_TRACING
  for( Int i = 0; i < numBinsOrig; i++ )
  {
    DTRACE( g_trace_ctx, D_CABAC, "%d" "  " "%d" "  " "EP=%d \n", DTRACE_GET_COUNTER( g_trace_ctx, D_CABAC ), m_Range, ( bins >> ( numBinsOrig - 1 - i ) ) & 1 );
  }
#endif
  return bins;
}




template <class BinProbModel>
TBinDecoder<BinProbModel>::TBinDecoder()
  : BinDecoderBase( static_cast<const BinProbModel*>    ( nullptr ) )
  , m_Ctx         ( static_cast<CtxStore<BinProbModel>&>( *this   ) )
{}


template <class BinProbModel>
unsigned TBinDecoder<BinProbModel>::decodeBin( unsigned ctxId )
{
  BinProbModel& rcProbModel = m_Ctx[ctxId];
  unsigned      bin         = rcProbModel.mps();
  uint32_t      LPS         = rcProbModel.getLPS( m_Range );

  DTRACE( g_trace_ctx, D_CABAC, "%d" "  " "%d" "  " "[%d:%d]" "  " "%2d(MPS=%d)"  "  " , DTRACE_GET_COUNTER( g_trace_ctx, D_CABAC ), m_Range, m_Range-LPS, LPS, ( unsigned int )( rcProbModel.state() ), m_Value < ( ( m_Range - LPS ) << 7 ) );

  m_Range   -=  LPS;
  uint32_t      SR          = m_Range << 7;
  if( m_Value < SR )
  {
#if RExt__DECODER_DEBUG_BIT_STATISTICS
    CodingStatistics::UpdateCABACStat( *ptype, m_Range+LPS, m_Range, int( bin ) );
#endif
    // MPS path
    if( m_Range < 256 )
    {
      int numBits   = rcProbModel.getRenormBitsRange( m_Range );
      m_Range     <<= numBits;
      m_Value     <<= numBits;
      m_bitsNeeded += numBits;
      if( m_bitsNeeded >= 0 )
      {
        m_Value      += m_Bitstream->readByte() << m_bitsNeeded;
        m_bitsNeeded -= 8;
      }
    }
  }
  else
  {
    bin = 1 - bin;
#if RExt__DECODER_DEBUG_BIT_STATISTICS
    CodingStatistics::UpdateCABACStat( *ptype, m_Range+LPS, LPS, int( bin ) );
#endif
    // LPS path
    int numBits   = rcProbModel.getRenormBitsLPS( LPS );
    m_Value      -= SR;
    m_Value       = m_Value << numBits;
    m_Range       = LPS     << numBits;
    m_bitsNeeded += numBits;
    if( m_bitsNeeded >= 0 )
    {
      m_Value      += m_Bitstream->readByte() << m_bitsNeeded;
      m_bitsNeeded -= 8;
    }
  }
  rcProbModel.update( bin );
  //DTRACE_DECR_COUNTER( g_trace_ctx, D_CABAC );
  DTRACE_WITHOUT_COUNT( g_trace_ctx, D_CABAC, "  -  " "%d" "\n", bin );
  return  bin;
}



template class TBinDecoder<BinProbModel_Std>;
template class TBinDecoder<BinProbModel_JMP>;
template class TBinDecoder<BinProbModel_JAW>;
template class TBinDecoder<BinProbModel_JMPAW>;

