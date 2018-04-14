

#pragma once

#include "CommonLib/Contexts.h"
#include "CommonLib/BitStream.h"
#include "CommonLib/dtrace_next.h"



class BinStore
{
public:
  BinStore () : m_inUse(false), m_allocated(false)  {}
  ~BinStore()                                       {}

  void  reset   ()
  {
    if( m_inUse )
    {
      for( unsigned n = 0; n < Ctx::NumberOfContexts; n++ )
      {
        m_binBuffer[n].clear();
      }
    }
  }
  void  addBin  ( unsigned bin, unsigned ctxId )
  {
    if( m_inUse )
    {
      std::vector<bool>& binBuffer = m_binBuffer[ctxId];
      if( binBuffer.size() < m_maxNumBins )
      {
        binBuffer.push_back( bin == 1 );
      }
    }
  }

  void                      setUse      ( bool useStore )         { m_inUse = useStore; if(m_inUse){xCheckAlloc();} }
  bool                      inUse       ()                  const { return m_inUse; }
  const std::vector<bool>&  getBinVector( unsigned ctxId )  const { return m_binBuffer[ctxId]; }

private:
  void  xCheckAlloc()
  {
    if( !m_allocated )
    {
      m_binBuffer.resize( Ctx::NumberOfContexts );
      for( unsigned n = 0; n < Ctx::NumberOfContexts; n++ )
      {
        m_binBuffer[n].reserve( m_maxNumBins );
      }
      m_allocated = true;
    }
  }

private:
  static const std::size_t          m_maxNumBins = 100000;
  bool                              m_inUse;
  bool                              m_allocated;
  std::vector< std::vector<bool> >  m_binBuffer;
};


class BinEncIf : public Ctx
{
protected:
  template <class BinProbModel>
  BinEncIf( const BinProbModel* dummy ) : Ctx( dummy ) {}
public:
  virtual ~BinEncIf() {}
public:
  virtual void      init              ( OutputBitstream* bitstream )        = 0;
  virtual void      uninit            ()                                    = 0;
  virtual void      start             ()                                    = 0;
  virtual void      finish            ()                                    = 0;
  virtual void      restart           ()                                    = 0;
  virtual void      reset             ( int qp, int initId )                = 0;
public:
  virtual void      resetBits         ()                                    = 0;
  virtual uint64_t  getEstFracBits    ()                              const = 0;
  virtual unsigned  getNumBins        ( unsigned    ctxId )           const = 0;
public:
  virtual void      encodeBin         ( unsigned bin,   unsigned ctxId    ) = 0;
  virtual void      encodeBinEP       ( unsigned bin                      ) = 0;
  virtual void      encodeBinsEP      ( unsigned bins,  unsigned numBins  ) = 0;
  virtual void      encodeRemAbsEP    ( unsigned bins,
                                        unsigned goRicePar,
                                        bool     useLimitedPrefixLength,
                                        int      maxLog2TrDynamicRange,
                                        bool     altResiComp = false      ) = 0;
  virtual void      encodeBinTrm      ( unsigned bin                      ) = 0;
  virtual void      encodeBinsPCM     ( unsigned bins,  unsigned numBins  ) = 0;
  virtual void      align             ()                                    = 0;
  virtual void      pcmAlignBits      ()                                    = 0;
public:
  virtual uint32_t  getNumBins        ()                                    = 0;
  virtual bool      isEncoding        ()                                    = 0;
  virtual unsigned  getNumWrittenBits ()                                    = 0;
public:
  virtual void            setBinStorage     ( bool b )                      = 0;
  virtual const BinStore* getBinStore       ()                        const = 0;
  virtual BinEncIf*       getTestBinEncoder ()                        const = 0;
};



class BinCounter
{
public:
  BinCounter();
  ~BinCounter() {}
public:
  void      reset   ();
  void      addCtx  ( unsigned ctxId )          { m_NumBinsCtx[ctxId]++; }
  void      addEP   ( unsigned num   )          { m_NumBinsEP+=num; }
  void      addEP   ()                          { m_NumBinsEP++; }
  void      addTrm  ()                          { m_NumBinsTrm++; }
  uint32_t  getAll  ()                  const;
  uint32_t  getCtx  ( unsigned ctxId )  const   { return m_NumBinsCtx[ctxId]; }
  uint32_t  getEP   ()                  const   { return m_NumBinsEP; }
  uint32_t  getTrm  ()                  const   { return m_NumBinsTrm; }
private:
  std::vector<uint32_t> m_CtxBinsCodedBuffer;
  uint32_t*             m_NumBinsCtx;
  uint32_t              m_NumBinsEP;
  uint32_t              m_NumBinsTrm;
};



class BinEncoderBase : public BinEncIf, public BinCounter
{
protected:
  template <class BinProbModel>
  BinEncoderBase ( const BinProbModel* dummy );
public:
  ~BinEncoderBase() {}
public:
  void      init    ( OutputBitstream* bitstream );
  void      uninit  ();
  void      start   ();
  void      finish  ();
  void      restart ();
  void      reset   ( int qp, int initId );
public:
  void      resetBits           ();
  uint64_t  getEstFracBits      ()                    const { THROW( "not supported" ); return 0; }
  unsigned  getNumBins          ( unsigned ctxId )    const { return BinCounter::getCtx(ctxId); }
public:
  void      encodeBinEP         ( unsigned bin                      );
  void      encodeBinsEP        ( unsigned bins,  unsigned numBins  );
  void      encodeRemAbsEP      ( unsigned bins,
                                  unsigned goRicePar,
                                  bool     useLimitedPrefixLength,
                                  int      maxLog2TrDynamicRange,
                                  bool     altResiComp = false      );
  void      encodeBinTrm        ( unsigned bin                      );
  void      encodeBinsPCM       ( unsigned bins,  unsigned numBins  );
  void      align               ();
  void      pcmAlignBits        ();
  unsigned  getNumWrittenBits   () { return ( m_Bitstream->getNumberOfWrittenBits() + 8 * m_numBufferedBytes + 23 - m_bitsLeft ); }
public:
  uint32_t  getNumBins          ()                          { return BinCounter::getAll(); }
  bool      isEncoding          ()                          { return true; }
protected:
  void      encodeAlignedBinsEP ( unsigned bins,  unsigned numBins  );
  void      writeOut            ();
protected:
  OutputBitstream*        m_Bitstream;
  uint32_t                m_Low;
  uint32_t                m_Range;
  uint32_t                m_bufferedByte;
  int32_t                 m_numBufferedBytes;
  int32_t                 m_bitsLeft;
  BinStore                m_BinStore;
};



template <class BinProbModel>
class TBinEncoder : public BinEncoderBase
{
public:
  TBinEncoder ();
  ~TBinEncoder() {}
  void  encodeBin   ( unsigned bin, unsigned ctxId );
public:
  void            setBinStorage     ( bool b )          { m_BinStore.setUse(b); }
  const BinStore* getBinStore       ()          const   { return &m_BinStore; }
  BinEncIf*       getTestBinEncoder ()          const;
private:
  CtxStore<BinProbModel>& m_Ctx;
};





class BitEstimatorBase : public BinEncIf
{
protected:
  template <class BinProbModel>
  BitEstimatorBase ( const BinProbModel* dummy );
public:
  ~BitEstimatorBase() {}
public:
  void      init                ( OutputBitstream* bitstream )        {}
  void      uninit              ()                                    {}
  void      start               ()                                    { m_EstFracBits = 0; }
  void      finish              ()                                    {}
  void      restart             ()                                    { m_EstFracBits = (m_EstFracBits >> SCALE_BITS) << SCALE_BITS; }
  void      reset               ( int qp, int initId )                { Ctx::init( qp, initId ); m_EstFracBits = 0;}
public:
  void      resetBits           ()                                    { m_EstFracBits = 0; }

  uint64_t  getEstFracBits      ()                              const { return m_EstFracBits; }
  unsigned  getNumBins          ( unsigned ctxId )              const { THROW( "not supported for BitEstimator" ); return 0; }
public:
  void      encodeBinEP         ( unsigned bin                      ) { m_EstFracBits += BinProbModelBase::estFracBitsEP (); }
  void      encodeBinsEP        ( unsigned bins,  unsigned numBins  ) { m_EstFracBits += BinProbModelBase::estFracBitsEP ( numBins ); }
  void      encodeRemAbsEP      ( unsigned bins,
                                  unsigned goRicePar,
                                  bool     useLimitedPrefixLength,
                                  int      maxLog2TrDynamicRange,
                                  bool     altResiComp = false      );
  void      encodeBinsPCM       ( unsigned bins,  unsigned numBins  ) { m_EstFracBits += BinProbModelBase::estFracBitsEP ( numBins ); }
  void      align               ();
  void      pcmAlignBits        ();
public:
  uint32_t  getNumBins          ()                                      { THROW("Not supported"); return 0; }
  bool      isEncoding          ()                                      { return false; }
  unsigned  getNumWrittenBits   ()                                      { /*THROW( "Not supported" );*/ return (UInt)( 0/*m_EstFracBits*//* >> SCALE_BITS*/ ); }

protected:
  uint64_t                m_EstFracBits;
};



template <class BinProbModel>
class TBitEstimator : public BitEstimatorBase
{
public:
  TBitEstimator ();
  ~TBitEstimator() {}
  void encodeBin    ( unsigned bin, unsigned ctxId )  { m_Ctx[ctxId].estFracBitsUpdate( bin, m_EstFracBits ); }
  void encodeBinTrm ( unsigned bin )                  { m_EstFracBits += BinProbModel::estFracBitsTrm( bin ); }
  void            setBinStorage     ( bool b )        {}
  const BinStore* getBinStore       ()          const { return 0; }
  BinEncIf*       getTestBinEncoder ()          const { return 0; }
private:
  CtxStore<BinProbModel>& m_Ctx;
};



typedef TBinEncoder  <BinProbModel_Std>   BinEncoder_Std;
typedef TBinEncoder  <BinProbModel_JMP>   BinEncoder_JMP;
typedef TBinEncoder  <BinProbModel_JAW>   BinEncoder_JAW;
typedef TBinEncoder  <BinProbModel_JMPAW> BinEncoder_JMPAW;
typedef TBinEncoder  <BinProbModel_MP>    BinEncoder_MP;
typedef TBinEncoder  <BinProbModel_MPI>   BinEncoder_MPI;
typedef TBinEncoder  <BinProbModel_MPCW>  BinEncoder_MPCW;

typedef TBitEstimator<BinProbModel_Std>   BitEstimator_Std;
typedef TBitEstimator<BinProbModel_JMP>   BitEstimator_JMP;
typedef TBitEstimator<BinProbModel_JAW>   BitEstimator_JAW;
typedef TBitEstimator<BinProbModel_JMPAW> BitEstimator_JMPAW;
typedef TBitEstimator<BinProbModel_MP>    BitEstimator_MP;
typedef TBitEstimator<BinProbModel_MPI>   BitEstimator_MPI;
typedef TBitEstimator<BinProbModel_MPCW>  BitEstimator_MPCW;


