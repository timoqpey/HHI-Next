

#pragma once


#include "CommonLib/Contexts.h"
#include "CommonLib/BitStream.h"


#if RExt__DECODER_DEBUG_BIT_STATISTICS
class CodingStatisticsClassType;
#endif



class BinDecoderBase : public Ctx
{
protected:
  template <class BinProbModel>
  BinDecoderBase ( const BinProbModel* dummy );
public:
  ~BinDecoderBase() {}
public:
  void      init    ( InputBitstream* bitstream );
  void      uninit  ();
  void      start   ();
  void      finish  ();
  void      reset   ( int qp, int initId );
#if RExt__DECODER_DEBUG_BIT_STATISTICS
  void      set     ( const CodingStatisticsClassType& type) { ptype = &type; }
#endif

public:
  virtual unsigned  decodeBin           ( unsigned ctxId    ) = 0;

public:
  unsigned          decodeBinEP         ();
  unsigned          decodeBinsEP        ( unsigned numBins  );
  unsigned          decodeRemAbsEP      ( unsigned goRicePar, bool useLimitedPrefixLength, int maxLog2TrDynamicRange, bool altRC = false );
  unsigned          decodeBinTrm        ();
  unsigned          decodeBinsPCM       ( unsigned numBins  );
  void              align               ();
  unsigned          getNumBitsRead      () { return m_Bitstream->getNumBitsRead() + m_bitsNeeded; }
private:
  unsigned          decodeAlignedBinsEP ( unsigned numBins  );
protected:
  InputBitstream*   m_Bitstream;
  uint32_t          m_Range;
  uint32_t          m_Value;
  int32_t           m_bitsNeeded;
#if RExt__DECODER_DEBUG_BIT_STATISTICS
  const CodingStatisticsClassType* ptype;
#endif
};



template <class BinProbModel>
class TBinDecoder : public BinDecoderBase
{
public:
  TBinDecoder ();
  ~TBinDecoder() {}
  unsigned decodeBin ( unsigned ctxId );
private:
  CtxStore<BinProbModel>& m_Ctx;
};



typedef TBinDecoder<BinProbModel_Std>   BinDecoder_Std;
typedef TBinDecoder<BinProbModel_JMP>   BinDecoder_JMP;
typedef TBinDecoder<BinProbModel_JAW>   BinDecoder_JAW;
typedef TBinDecoder<BinProbModel_JMPAW> BinDecoder_JMPAW;


