/* The copyright in this software is being made available under the BSD
* License, included below. This software may be subject to other third party
* and contributor rights, including patent rights, and no such rights are
* granted under this license.
*
* Copyright (c) 2010-2017, ITU/ISO/IEC
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*  * Redistributions of source code must retain the above copyright notice,
*    this list of conditions and the following disclaimer.
*  * Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*  * Neither the name of the ITU/ISO/IEC nor the names of its contributors may
*    be used to endorse or promote products derived from this software without
*    specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
* BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
* THE POSSIBILITY OF SUCH DAMAGE.
*/

/** \file     CABACWriter.cpp
 *  \brief    Writer for low level syntax
 */

#include "CommonLib/Contexts.h"
#include "CABACWriter.h"

#include "EncLib.h"

#include "CommonLib/UnitTools.h"
#include "CommonLib/dtrace_buffer.h"
#include "CommonLib/BinaryDecisionTree.h"

#include <map>
#include <algorithm>
#include <limits>


//! \ingroup EncoderLib
//! \{

void CABACWriter::initCtxModels( const Slice& slice, const CABACDataStore* cabacDataStore )
{
  Int       qp                = slice.getSliceQp();
  SliceType sliceType         = slice.getSliceType();
  SliceType encCABACTableIdx  = slice.getEncCABACTableIdx();
  if( !slice.isIntra() && (encCABACTableIdx==B_SLICE || encCABACTableIdx==P_SLICE) && slice.getPPS()->getCabacInitPresentFlag() )
  {
    sliceType = encCABACTableIdx;
  }
  m_BinEncoder.reset( qp, (int)sliceType );
  if( cabacDataStore )
  {
    m_BinEncoder.setWinSizes( cabacDataStore->getWinSizes(&slice) );
  }
}



template <class BinProbModel>
SliceType xGetCtxInitId( const Slice& slice, const BinEncIf& binEncoder, Ctx& ctxTest )
{
  const CtxStore<BinProbModel>& ctxStoreTest = static_cast<const CtxStore<BinProbModel>&>( ctxTest );
  const CtxStore<BinProbModel>& ctxStoreRef  = static_cast<const CtxStore<BinProbModel>&>( binEncoder.getCtx() );
  int qp = slice.getSliceQp();
  if( !slice.isIntra() )
  {
    SliceType aSliceTypeChoices[] = { B_SLICE, P_SLICE };
    uint64_t  bestCost            = std::numeric_limits<uint64_t>::max();
    SliceType bestSliceType       = aSliceTypeChoices[0];
    for (UInt idx=0; idx<2; idx++)
    {
      uint64_t  curCost           = 0;
      SliceType curSliceType      = aSliceTypeChoices[idx];
      ctxTest.init( qp, (int)curSliceType );
      for( int k = 0; k < Ctx::NumberOfContexts; k++ )
      {
        if( binEncoder.getNumBins(k) > 0 )
        {
          curCost += uint64_t( binEncoder.getNumBins(k) ) * ctxStoreRef[k].estFracExcessBits( ctxStoreTest[k] );
        }
      }
      if (curCost < bestCost)
      {
        bestSliceType = curSliceType;
        bestCost      = curCost;
      }
    }
    return bestSliceType;
  }
  else
  {
    return I_SLICE;
  }
}


SliceType CABACWriter::getCtxInitId( const Slice& slice )
{
  switch( m_TestCtx.getBPMType() )
  {
  case BPM_Std:   return  xGetCtxInitId<BinProbModel_Std>   ( slice, m_BinEncoder, m_TestCtx );
  case BPM_JMP:   return  xGetCtxInitId<BinProbModel_JMP>   ( slice, m_BinEncoder, m_TestCtx );
  case BPM_JAW:   return  xGetCtxInitId<BinProbModel_JAW>   ( slice, m_BinEncoder, m_TestCtx );
  case BPM_JMPAW: return  xGetCtxInitId<BinProbModel_JMPAW> ( slice, m_BinEncoder, m_TestCtx );
  case BPM_MP:    return  xGetCtxInitId<BinProbModel_MP>    ( slice, m_BinEncoder, m_TestCtx );
  case BPM_MPI:   return  xGetCtxInitId<BinProbModel_MPI>   ( slice, m_BinEncoder, m_TestCtx );
  case BPM_MPCW:  return  xGetCtxInitId<BinProbModel_MPCW>  ( slice, m_BinEncoder, m_TestCtx );
  default:        return  NUMBER_OF_SLICE_TYPES;
  }
}



unsigned estBits( BinEncIf& binEnc, const std::vector<bool>& bins, const Ctx& ctx, const int ctxId, const uint8_t winSize )
{
  binEnc.initCtxAndWinSize( ctxId, ctx, winSize );
  binEnc.start();
  const std::size_t numBins   = bins.size();
  unsigned          startBits = binEnc.getNumWrittenBits();
  for( std::size_t binId = 0; binId < numBins; binId++ )
  {
    unsigned  bin = ( bins[binId] ? 1 : 0 );
    binEnc.encodeBin( bin, ctxId );
  }
  unsigned endBits    = binEnc.getNumWrittenBits();
  unsigned codedBits  = endBits - startBits;
  return   codedBits;
}

uint8_t estWinSize( BinEncIf& binEnc, const BinStore& binStore, const Ctx& ctx, const int ctxId )
{
  const uint8_t             numWinSizesToTest = 4;
  const uint8_t             minWinSize        = 4;
  const uint8_t             maxWinSize        = minWinSize + numWinSizesToTest - uint8_t(1);
  const uint8_t             defWinSize        = binEnc.getDefaultWindowSize();
  const std::vector<bool>&  bins              = binStore.getBinVector( ctxId );
  if( !bins.size() )
  {
    return defWinSize;
  }
  uint8_t         bestWinSize       = defWinSize;
  unsigned        minBits           = estBits( binEnc, bins, ctx, ctxId, bestWinSize );
  for( uint8_t testWinSize = minWinSize; testWinSize <= maxWinSize; testWinSize++ )
  {
    if( testWinSize != defWinSize )
    {
      unsigned testBits = estBits( binEnc, bins, ctx, ctxId, testWinSize );
      if( testBits < minBits )
      {
        minBits     = testBits;
        bestWinSize = testWinSize;
      }
    }
  }
  return bestWinSize;
}


void CABACWriter::estWinSizes( const Slice& slice, CABACDataStore& cabacDataStore ) const
{
  const BinStore* binStore          = m_BinEncoder.getBinStore();
  if( !binStore || !binStore->inUse() )
  {
    return;
  }

  OutputBitstream       testBitStream;
  BinEncIf*             testBinEncoder  = m_BinEncoder.getTestBinEncoder();
  CHECKD( !testBinEncoder, "intern error" );
  testBinEncoder->init( &testBitStream );
  std::vector<uint8_t>& winBuffer       = cabacDataStore.getWinSizeBuffer( &slice );
  const std::size_t     numCodeIds      = cabacDataStore.getNumWSizeCodeIds();
  Ctx                   initCtx         = getCtx();

  if( slice.getSPS()->getSpsNext().getCIPFMode() == 1 ) // backward encoder compatibility to JEM [not required]
  {
    cabacDataStore.loadCtxStates( &slice, initCtx, 0 );
  }

  winBuffer.resize( 0 );
  winBuffer.resize( Ctx::NumberOfContexts, 0 );
  for( std::size_t codeId = 0; codeId < numCodeIds; codeId++ )
  {
    const int ctxId = cabacDataStore.getCtxIdFromWSizeCodeId( codeId );
    if( ctxId >= 0 )
    {
      winBuffer[ctxId] = estWinSize( *testBinEncoder, *binStore, initCtx, ctxId );
    }
  }
  cabacDataStore.setWSizeSetValid(&slice);
  delete testBinEncoder;
}

void CABACWriter::enableBinStore( const Slice& slice, CABACDataStore& cabacDataStore )
{
  if( slice.getSPS()->getSpsNext().getCABACEngineMode() == 2 || slice.getSPS()->getSpsNext().getCABACEngineMode() == 3 )
  {
    m_BinEncoder.setBinStorage(!cabacDataStore.validWinSizes(&slice));
  }
}




//================================================================================
//  clause 7.3.8.1
//--------------------------------------------------------------------------------
//    void  end_of_slice()
//================================================================================

void CABACWriter::end_of_slice()
{
  m_BinEncoder.encodeBinTrm ( 1 );
  m_BinEncoder.finish       ();
}




//================================================================================
//  clause 7.3.8.2
//--------------------------------------------------------------------------------
//    bool  coding_tree_unit( cs, area, qp, ctuRsAddr, skipSao )
//================================================================================

void CABACWriter::coding_tree_unit( CodingStructure& cs, const UnitArea& area, int (&qps)[2], unsigned ctuRsAddr, bool skipSao /* = false */ )
{
  CUCtx cuCtx( qps[CH_L] );
  Partitioner *partitioner = PartitionerFactory::get( *cs.slice );

  partitioner->initCtu( area, CH_L, *cs.slice );

  if( !skipSao )
  {
    sao( *cs.slice, ctuRsAddr );
  }
  coding_tree( cs, *partitioner, cuCtx );
  qps[CH_L] = cuCtx.qp;
  if( CS::isDualITree( cs ) && cs.pcv->chrFormat != CHROMA_400 )
  {
    CUCtx cuCtxChroma( qps[CH_C] );
    partitioner->initCtu( area, CH_C, *cs.slice );
    coding_tree( cs, *partitioner, cuCtxChroma );
    qps[CH_C] = cuCtxChroma.qp;
  }

  delete partitioner;
}





//================================================================================
//  clause 7.3.8.3
//--------------------------------------------------------------------------------
//    void  sao             ( slice, ctuRsAddr )
//    void  sao_block_pars  ( saoPars, bitDepths, sliceEnabled, leftMergeAvail, aboveMergeAvail, onlyEstMergeInfo )
//    void  sao_offset_pars ( ctbPars, compID, sliceEnabled, bitDepth )
//================================================================================

void CABACWriter::sao( const Slice& slice, unsigned ctuRsAddr )
{
  const SPS& sps = *slice.getSPS();
  if( !sps.getUseSAO() )
  {
    return;
  }

  CodingStructure&     cs                     = *slice.getPic()->cs;
  const PreCalcValues& pcv                    = *cs.pcv;
  const SAOBlkParam&  sao_ctu_pars            = cs.picture->getSAO()[ctuRsAddr];
  bool                slice_sao_luma_flag     = ( slice.getSaoEnabledFlag( CHANNEL_TYPE_LUMA ) );
  bool                slice_sao_chroma_flag   = ( slice.getSaoEnabledFlag( CHANNEL_TYPE_CHROMA ) && sps.getChromaFormatIdc() != CHROMA_400 );
  if( !slice_sao_luma_flag && !slice_sao_chroma_flag )
  {
    return;
  }

  bool                sliceEnabled[3]         = { slice_sao_luma_flag, slice_sao_chroma_flag, slice_sao_chroma_flag };
  int                 frame_width_in_ctus     = pcv.widthInCtus;
  int                 ry                      = ctuRsAddr      / frame_width_in_ctus;
  int                 rx                      = ctuRsAddr - ry * frame_width_in_ctus;
  const Position      pos                     ( rx * cs.pcv->maxCUWidth, ry * cs.pcv->maxCUHeight );
  const unsigned      curSliceIdx             = slice.getIndependentSliceIdx();
  const unsigned      curTileIdx              = cs.picture->tileMap->getTileIdxMap( pos );
  bool                leftMergeAvail          = cs.getCURestricted( pos.offset( -(Int)pcv.maxCUWidth, 0  ), curSliceIdx, curTileIdx, CH_L ) ? true : false;
  bool                aboveMergeAvail         = cs.getCURestricted( pos.offset( 0, -(Int)pcv.maxCUHeight ), curSliceIdx, curTileIdx, CH_L ) ? true : false;
  sao_block_pars( sao_ctu_pars, sps.getBitDepths(), sliceEnabled, leftMergeAvail, aboveMergeAvail, false );
}


void CABACWriter::sao_block_pars( const SAOBlkParam& saoPars, const BitDepths& bitDepths, bool* sliceEnabled, bool leftMergeAvail, bool aboveMergeAvail, bool onlyEstMergeInfo )
{
  bool isLeftMerge  = false;
  bool isAboveMerge = false;
  if( leftMergeAvail )
  {
    // sao_merge_left_flag
    isLeftMerge   = ( saoPars[COMPONENT_Y].modeIdc == SAO_MODE_MERGE && saoPars[COMPONENT_Y].typeIdc == SAO_MERGE_LEFT );
    m_BinEncoder.encodeBin( (isLeftMerge), Ctx::SaoMergeFlag() );
  }
  if( aboveMergeAvail && !isLeftMerge )
  {
    // sao_merge_above_flag
    isAboveMerge  = ( saoPars[COMPONENT_Y].modeIdc == SAO_MODE_MERGE && saoPars[COMPONENT_Y].typeIdc == SAO_MERGE_ABOVE );
    m_BinEncoder.encodeBin( (isAboveMerge), Ctx::SaoMergeFlag() );
  }
  if( onlyEstMergeInfo )
  {
    return; //only for RDO
  }
  if( !isLeftMerge && !isAboveMerge )
  {
    // explicit parameters
    for( int compIdx=0; compIdx < MAX_NUM_COMPONENT; compIdx++ )
    {
      sao_offset_pars( saoPars[compIdx], ComponentID(compIdx), sliceEnabled[compIdx], bitDepths.recon[ toChannelType(ComponentID(compIdx)) ] );
    }
  }
}


void CABACWriter::sao_offset_pars( const SAOOffset& ctbPars, ComponentID compID, bool sliceEnabled, int bitDepth )
{
  if( !sliceEnabled )
  {
    CHECK( ctbPars.modeIdc != SAO_MODE_OFF, "Sao must be off, if it is disabled on slice level" );
    return;
  }
  const bool isFirstCompOfChType = ( getFirstComponentOfChannel( toChannelType(compID) ) == compID );

  if( isFirstCompOfChType )
  {
    // sao_type_idx_luma / sao_type_idx_chroma
    if( ctbPars.modeIdc == SAO_MODE_OFF )
    {
      m_BinEncoder.encodeBin  ( 0, Ctx::SaoTypeIdx() );
    }
    else if( ctbPars.typeIdc == SAO_TYPE_BO )
    {
      m_BinEncoder.encodeBin  ( 1, Ctx::SaoTypeIdx() );
      m_BinEncoder.encodeBinEP( 0 );
    }
    else
    {
      CHECK(!( ctbPars.typeIdc < SAO_TYPE_START_BO ), "Unspecified error");
      m_BinEncoder.encodeBin  ( 1, Ctx::SaoTypeIdx() );
      m_BinEncoder.encodeBinEP( 1 );
    }
  }

  if( ctbPars.modeIdc == SAO_MODE_NEW )
  {
    const int maxOffsetQVal = SampleAdaptiveOffset::getMaxOffsetQVal( bitDepth );
    int       numClasses    = ( ctbPars.typeIdc == SAO_TYPE_BO ? 4 : NUM_SAO_EO_CLASSES );
    int       k             = 0;
    int       offset[4];
    for( int i = 0; i < numClasses; i++ )
    {
      if( ctbPars.typeIdc != SAO_TYPE_BO && i == SAO_CLASS_EO_PLAIN )
      {
        continue;
      }
      int classIdx = ( ctbPars.typeIdc == SAO_TYPE_BO ? ( ctbPars.typeAuxInfo + i ) % NUM_SAO_BO_CLASSES : i );
      offset[k++]  = ctbPars.offset[classIdx];
    }

    // sao_offset_abs
    for( int i = 0; i < 4; i++ )
    {
      unsigned absOffset = ( offset[i] < 0 ? -offset[i] : offset[i] );
      unary_max_eqprob( absOffset, maxOffsetQVal );
    }

    // band offset mode
    if( ctbPars.typeIdc == SAO_TYPE_BO )
    {
      // sao_offset_sign
      for( int i = 0; i < 4; i++ )
      {
        if( offset[i] )
        {
          m_BinEncoder.encodeBinEP( (offset[i] < 0) );
        }
      }
      // sao_band_position
      m_BinEncoder.encodeBinsEP( ctbPars.typeAuxInfo, NUM_SAO_BO_CLASSES_LOG2 );
    }
    // edge offset mode
    else
    {
      if( isFirstCompOfChType )
      {
        // sao_eo_class_luma / sao_eo_class_chroma
        CHECK( ctbPars.typeIdc - SAO_TYPE_START_EO < 0, "sao edge offset class is outside valid range" );
        m_BinEncoder.encodeBinsEP( ctbPars.typeIdc - SAO_TYPE_START_EO, NUM_SAO_EO_TYPES_LOG2 );
      }
    }
  }
}

Int CABACWriter::alf_lengthGolomb(int coeffVal, int k)
{
  int m = 2 << (k - 1);
  int q = coeffVal / m;
  if(coeffVal != 0)
    return(q + 2 + k);
  else
    return(q + 1 + k);
}

Void CABACWriter::codeAlfUvlc( UInt uiCode )
{
  Int i;
  if ( uiCode == 0 )
  {
    m_BinEncoder.encodeBinEP(0);
  }
  else
  {
    m_BinEncoder.encodeBinEP(1);
    for ( i=0; i<uiCode-1; i++ )
    {
      m_BinEncoder.encodeBinEP(1);
    }
    m_BinEncoder.encodeBinEP(0);
  }
}

Void CABACWriter::codeAlfSvlc( Int iCode )
{
  Int i;
  if ( iCode == 0 )
    {
    m_BinEncoder.encodeBinEP(0);
    }
    else
    {
      m_BinEncoder.encodeBinEP(1);

      // write sign
      if ( iCode > 0 )
      {
        m_BinEncoder.encodeBinEP(0);
      }
      else
      {
        m_BinEncoder.encodeBinEP(1);
        iCode = -iCode;
      }

      // write magnitude
      for ( i=0; i<iCode-1; i++ )
      {
        m_BinEncoder.encodeBinEP(1);
      }
      m_BinEncoder.encodeBinEP(0);
    }
}

Void CABACWriter::xWriteTruncBinCode(UInt uiSymbol, UInt uiMaxSymbol)
{
  UInt uiThresh;
  if (uiMaxSymbol > 256)
  {
    UInt uiThreshVal = 1 << 8;
    uiThresh = 8;
    while (uiThreshVal <= uiMaxSymbol)
    {
      uiThresh++;
      uiThreshVal <<= 1;
    }
    uiThresh--;
  }
  else
  {
    uiThresh = g_NonMPM[uiMaxSymbol];
  }

  UInt uiVal = 1 << uiThresh;
  assert(uiVal <= uiMaxSymbol);
  assert((uiVal << 1) > uiMaxSymbol);
  assert(uiSymbol < uiMaxSymbol);
  UInt b = uiMaxSymbol - uiVal;
  assert(b < uiVal);
  if (uiSymbol < uiVal - b)
  {
    m_BinEncoder.encodeBinsEP(uiSymbol, uiThresh);
  }
  else
  {
    uiSymbol += uiVal - b;
    assert(uiSymbol < (uiVal << 1));
    assert((uiSymbol >> 1) >= uiVal - b);
    m_BinEncoder.encodeBinsEP(uiSymbol, uiThresh + 1);
  }
}

#if JVET_C0038_NO_PREV_FILTERS
Void CABACWriter::xWriteEpExGolomb(UInt uiSymbol, UInt uiCount)
{
  UInt bins = 0;
  Int numBins = 0;

  while (uiSymbol >= (UInt)(1 << uiCount))
  {
    bins = 2 * bins + 1;
    numBins++;
    uiSymbol -= 1 << uiCount;
    uiCount++;
  }
  bins = 2 * bins + 0;
  numBins++;

  bins = (bins << uiCount) | uiSymbol;
  numBins += uiCount;

  assert(numBins <= 32);
  m_BinEncoder.encodeBinsEP(bins, numBins);
}
#endif

Void CABACWriter::alfGolombEncode(int coeff, int k)
{
  int q, i, m;
  int symbol = abs(coeff);

  m = (int)pow(2.0, k);
  q = symbol / m;

  for (i = 0; i < q; i++)
    m_BinEncoder.encodeBinEP(1);
  m_BinEncoder.encodeBinEP(0);
  // write one zero

  for(i = 0; i < k; i++)
  {
    m_BinEncoder.encodeBinEP(symbol & 0x01);
    symbol >>= 1;
  }

  if(coeff != 0)
  {
    int sign = (coeff > 0)? 1: 0;
    m_BinEncoder.encodeBinEP(sign);
  }
}

void CABACWriter::alf( const Slice& slice , const ALFParam& alfParam )
{
  const SPSNext& spsNext = slice.getSPS()->getSpsNext();
  if( !spsNext.getALFEnabled() )
  {
    return;
  }
  alf( alfParam, slice.getSliceType(), spsNext.getGALFEnabled() );
}

void CABACWriter::alf( const ALFParam& alfParam, SliceType sliceType, bool isGALF )
{
  //AdaptiveLoopFilter::printALFParam( &alfParam, false );
  m_BinEncoder.encodeBinEP( alfParam.alf_flag );
  if( !alfParam.alf_flag )
  {
    return;
  }

#if MCALF
  codeAlfUvlc(alfParam.classifier);
#endif

#if COM16_C806_ALF_TEMPPRED_NUM
  if( sliceType != I_SLICE )
  {
    m_BinEncoder.encodeBinEP( alfParam.temporalPredFlag );
  }
  else
  {
    CHECK( alfParam.temporalPredFlag != 0, "ALF: Temporal prediction for I-Slices not allowed" );
  }
#endif
  if( !alfParam.temporalPredFlag )
  {
    alf_aux    ( alfParam, isGALF );
    alf_filter ( alfParam, isGALF );
  }
  else
  {
    codeAlfUvlc( alfParam.prevIdx );
  }

#if MCALF
  if (alfParam.classifier == 1 || alfParam.classifier == 3)
  {
    CHECK(alfParam.numClasses > 80, "numClasses does not exeed 80!");
    for (Int i = 0; i < alfParam.numClasses; i++)
    {
      codeAlfUvlc(alfParam.selectedBins[i]);
    }
  }
#endif


  if( isGALF )
  {
    codeAlfUvlc(alfParam.chroma_idc);

    if (alfParam.chroma_idc && !alfParam.temporalPredFlag)
    {
      alf_filter(alfParam, isGALF, true);
    }
  }
  else
  {
    alf_chroma ( alfParam );
  }

  alf_cu_ctrl( alfParam );
}

Void CABACWriter::alf_aux( const ALFParam& alfParam, bool isGALF )
{
  Int iNoVarBins = AdaptiveLoopFilter::m_NO_VAR_BINS;
#if GALF
  xWriteTruncBinCode((UInt)(alfParam.filters_per_group - 1), (UInt)iNoVarBins);
#endif

  //FilterType
  codeAlfUvlc( alfParam.filterType );

  //FilterMode
  if( isGALF )
  {
    //Int iNoVarBins = AdaptiveLoopFilter::m_NO_VAR_BINS;
    //xWriteTruncBinCode((UInt)(alfParam.filters_per_group - 1), (UInt)iNoVarBins);
    if (alfParam.filters_per_group > 1)
    {
#if MCALF
      Int maxPat = iNoVarBins;
      if (alfParam.classifier == 2)
      {
        maxPat = iNoVarBins - 2;
      }
      for (Int i = 0; i < maxPat; i++)
#else
      for (Int i = 0; i < iNoVarBins; i++)
#endif
      {
        xWriteTruncBinCode((UInt)alfParam.filterPattern[i], (UInt)alfParam.filters_per_group);
      }
    }

#if JVET_C0038_NO_PREV_FILTERS
    Int i;
    Int availableFilters = alfParam.iAvailableFilters;
    UChar codetab_pred[3] = { 1, 0, 2 };

    if (availableFilters > 0)
    {
      //codeAlfUvlc(codetab_pred[alfParam.iPredPattern]); //correct way to encode ??
      xWriteEpExGolomb(codetab_pred[alfParam.iPredPattern],0); //0: all zero, no pred from pre-defined filters; 1: all are predicted but could be different values; 2: some predicted and some not
      if (alfParam.iPredPattern == 2)
      {
        for (i = 0; i < iNoVarBins; i++)
        {
          m_BinEncoder.encodeBinEP(alfParam.PrevFiltIdx[i]>0 ? 1 : 0);
        }
      }
      if (alfParam.iPredPattern > 0 && availableFilters > 1)
      {
        for (i = 0; i < iNoVarBins; i++)
        {
          if (alfParam.PrevFiltIdx[i] > 0)
          {
            xWriteTruncBinCode((UChar)alfParam.PrevFiltIdx[i] - 1, availableFilters);
          }
        }
      }
    }
#endif
  }
  else
  {
    codeAlfUvlc( alfParam.filterMode );
    if( alfParam.filterMode == ALF_TWO_FILTERS )
    {
      codeAlfUvlc( alfParam.startSecondFilter );
    }
    else if ( alfParam.filterMode == ALF_MULTIPLE_FILTERS )
    {
      for( Int i=1; i< AdaptiveLoopFilter::m_NO_VAR_BINS; i++)
      {
        m_BinEncoder.encodeBinEP( alfParam.filterPattern[i] );
      }
    }
  }
}

Void CABACWriter::alf_filter( const ALFParam& alfParam, bool isGALF, bool bChroma )
{
  Int filters_per_group;
  Int sqrFiltLength;
  Int filtType;
  if( isGALF )
  {
    filters_per_group = bChroma ? 1: alfParam.filters_per_group;
    sqrFiltLength = bChroma ? alfParam.num_coeff_chroma : alfParam.num_coeff;
    sqrFiltLength = sqrFiltLength - 1;
    filtType = bChroma ? (alfParam.tap_chroma == 5 ? 0 : (alfParam.tap_chroma == 7 ? 1 : 2)) : alfParam.filterType;

    if (!bChroma)
    {
  #if FORCE0
      m_BinEncoder.encodeBinEP(alfParam.forceCoeff0);
      if (!alfParam.forceCoeff0)
      {
  #endif
        if (filters_per_group > 1)
        {
          m_BinEncoder.encodeBinEP(alfParam.predMethod);
        }
  #if FORCE0
      }
  #endif
    }
  }
  else
  {
    filters_per_group  = alfParam.filters_per_group;
    sqrFiltLength          = alfParam.num_coeff;
    filtType               = alfParam.filterType;

    if( filters_per_group > 1 )
    {
      m_BinEncoder.encodeBinEP( alfParam.predMethod );
    }
  }

  int i, k, kMin, kStart, minBits, ind, scanPos, maxScanVal, coeffVal;

  int kMinTab[ AdaptiveLoopFilter::m_MAX_SQR_FILT_LENGTH], bitsCoeffScan[ AdaptiveLoopFilter::m_MAX_SCAN_VAL][AdaptiveLoopFilter::m_MAX_EXP_GOLOMB],
      minKStart, minBitsKStart, bitsKStart;

  maxScanVal = 0;
  const Int* pDepthInt = AdaptiveLoopFilter::m_pDepthIntTab[filtType];
  for(i = 0; i < sqrFiltLength; i++ )
  {
    maxScanVal = max(maxScanVal, pDepthInt[i]);
  }
  // vlc for all
  memset( bitsCoeffScan, 0, AdaptiveLoopFilter::m_MAX_SCAN_VAL * AdaptiveLoopFilter::m_MAX_EXP_GOLOMB * sizeof(int));
  for(ind=0; ind<filters_per_group; ++ind)
  {
#if FORCE0
    if (bChroma || alfParam.codedVarBins[ind] || !alfParam.forceCoeff0)
    {
#endif
      for (i = 0; i < sqrFiltLength; i++)
      {
        scanPos = pDepthInt[i] - 1;
        coeffVal = abs((isGALF && bChroma) ? alfParam.coeff_chroma[i] : alfParam.coeffmulti[ind][i]);

        for (k = 1; k < 15; k++)
        {
          bitsCoeffScan[scanPos][k] += alf_lengthGolomb(coeffVal, k);
        }
      }
#if FORCE0
    }
#endif
  }
  minBitsKStart =  0;
  minKStart     = -1;
  for(k = 1; k < 8; k++)
  {
    bitsKStart = 0;
    kStart = k;
    for(scanPos = 0; scanPos < maxScanVal; scanPos++)
    {
      kMin = kStart;
      minBits = bitsCoeffScan[scanPos][kMin];

      if(bitsCoeffScan[scanPos][kStart+1] < minBits)
      {
        kMin = kStart + 1;
        minBits = bitsCoeffScan[scanPos][kMin];
      }
      kStart = kMin;
      bitsKStart += minBits;
    }
    if((bitsKStart < minBitsKStart) || (k == 1))
    {
      minBitsKStart = bitsKStart;
      minKStart = k;
    }
  }

  kStart = minKStart;
  for(scanPos = 0; scanPos < maxScanVal; scanPos++)
  {
    kMin = kStart;
    minBits = bitsCoeffScan[scanPos][kMin];

    if(bitsCoeffScan[scanPos][kStart+1] < minBits)
    {
      kMin = kStart + 1;
      minBits = bitsCoeffScan[scanPos][kMin];
    }

    kMinTab[scanPos] = kMin;
    kStart = kMin;
  }

  // Golomb parameters
  codeAlfUvlc( minKStart -1 );
  int golombIndexBit;
  kMin = minKStart;
  for( scanPos = 0; scanPos < maxScanVal; scanPos++)
  {
     golombIndexBit = (kMinTab[scanPos] != kMin)? 1: 0;
     CHECK( !(kMinTab[scanPos] <= kMin + 1), "ALF Golomb parameter not consistent" );
     m_BinEncoder.encodeBinEP(golombIndexBit);
     kMin = kMinTab[scanPos];
  }
#if FORCE0
  if (!bChroma)
  {
    if (alfParam.forceCoeff0)
    {
      for (ind = 0; ind < filters_per_group; ++ind)
      {
        m_BinEncoder.encodeBinEP(alfParam.codedVarBins[ind]);
      }
    }
  }
#endif
  // Filter coefficients
  if( isGALF )
  {
    if (bChroma)
    {
      for (i = 0; i < sqrFiltLength; i++)
      {
        scanPos = pDepthInt[i] - 1;
        alfGolombEncode(alfParam.coeff_chroma[i], kMinTab[scanPos]);
      }
    }
    else
    {
      for (ind = 0; ind < alfParam.filters_per_group; ++ind)
      {
  #if FORCE0
        if ((alfParam.codedVarBins[ind] == 0) && alfParam.forceCoeff0 )
        {
          continue;
        }
  #endif
        for (i = 0; i < sqrFiltLength; i++)
        {
          scanPos = pDepthInt[i] - 1;
          alfGolombEncode(alfParam.coeffmulti[ind][i], kMinTab[scanPos]);
        }
      }
    }
  }
  else
  {
    for (ind = 0; ind < alfParam.filters_per_group; ++ind)
    {
      for (i = 0; i < sqrFiltLength; i++)
      {
        scanPos = pDepthInt[i] - 1;
        alfGolombEncode(alfParam.coeffmulti[ind][i], kMinTab[scanPos]);
      }
    }
  }
}

Void CABACWriter::alf_cu_ctrl( const ALFParam& alfParam )
{
  m_BinEncoder.encodeBinEP( alfParam.cu_control_flag );
  if( alfParam.cu_control_flag)
  {
    unary_max_symbol( alfParam.alf_max_depth, Ctx::AlfUvlcSCModel(0),Ctx::AlfUvlcSCModel(1), alfParam.maxCodingDepth-1);
    UInt uiLength = 0;

    UInt maxValue = ( alfParam.num_ctus_in_frame << (alfParam.alf_max_depth*2) );
    UInt minValue = alfParam.num_ctus_in_frame;
    CHECK( maxValue < alfParam.num_alf_cu_flag, "ALF: wrong max value for number of CUs" );
    CHECK( minValue > alfParam.num_alf_cu_flag, "ALF: wrong min value for number of CUs" );
    UInt temp = maxValue - minValue;
    for(UInt i=0; i<32; i++)
    {
      if(temp&0x1)
      {
        uiLength = i+1;
      }
      temp = (temp >> 1);
    }
    UInt uiSymbol = alfParam.num_alf_cu_flag - minValue;
    if(uiLength)
    {
      while( uiLength-- )
      {
        m_BinEncoder.encodeBinEP( (uiSymbol>>uiLength) & 0x1 );
      }
    }

    DTRACE( g_trace_ctx, D_SYNTAX, "alf_cu_ctrl() max_depth=%d max_alf_depth=%d num_cu_flags=%d\n", alfParam.maxCodingDepth, alfParam.alf_max_depth, alfParam.num_alf_cu_flag );

    for(UInt i=0; i< alfParam.num_alf_cu_flag; i++)
    {
      m_BinEncoder.encodeBin( alfParam.alf_cu_flag[i], Ctx::AlfCUCtrlFlags( 0 ) );

      DTRACE( g_trace_ctx, D_SYNTAX, "alf_cu_ctrl() blk=%d alf_cu_flag[blk]=%d\n", i, alfParam.alf_cu_flag[i] );
    }
  }
  else
  {
    DTRACE( g_trace_ctx, D_SYNTAX, "alf_cu_ctrl() off\n" );
  }
}

Void CABACWriter::alf_chroma( const ALFParam& alfParam )
{
  codeAlfUvlc( alfParam.chroma_idc );
  if( alfParam.chroma_idc && !alfParam.temporalPredFlag )
  {
    codeAlfUvlc((alfParam.tap_chroma-5)/2);
    // filter coefficients for chroma
    for(Int pos=0; pos<alfParam.num_coeff_chroma; pos++)
    {
      codeAlfSvlc( alfParam.coeff_chroma[pos] );
    }
  }
}



//================================================================================
//  clause 7.3.8.4
//--------------------------------------------------------------------------------
//    void  coding_tree       ( cs, partitioner, cuCtx )
//    void  split_cu_flag     ( split, cs, partitioner )
//    void  split_cu_mode_mt  ( split, cs, partitioner )
//================================================================================

void CABACWriter::coding_tree( const CodingStructure& cs, Partitioner& partitioner, CUCtx& cuCtx )
{
  const PPS      &pps         = *cs.pps;
  const UnitArea &currArea    = partitioner.currArea();
  const CodingUnit &cu        = *cs.getCU( currArea.blocks[partitioner.chType], partitioner.chType );

  // Reset delta QP coding flag and ChromaQPAdjustemt coding flag
  if( pps.getUseDQP() && partitioner.currDepth <= pps.getMaxCuDQPDepth() )
  {
    cuCtx.isDQPCoded          = false;
  }
  if( cs.slice->getUseChromaQpAdj() && partitioner.currDepth <= pps.getPpsRangeExtension().getDiffCuChromaQpOffsetDepth() )
  {
    cuCtx.isChromaQpAdjCoded  = false;
  }

  const PartSplit implicitSplit = partitioner.getImplicitSplit( cs );

  // QT
  bool canQtSplit = partitioner.canSplit( CU_QUAD_SPLIT, cs );

  if( canQtSplit )
  {
    // split_cu_flag
    bool qtSplit = implicitSplit == CU_QUAD_SPLIT;

    if( !qtSplit && implicitSplit != CU_QUAD_SPLIT )
    {
      qtSplit = ( cu.qtDepth > partitioner.currQtDepth );
      split_cu_flag( qtSplit, cs, partitioner );
    }

    // quad-tree split
    if( qtSplit )
    {
      partitioner.splitCurrArea( CU_QUAD_SPLIT, cs );

      do
      {
        if( cs.picture->blocks[partitioner.chType].contains( partitioner.currArea().blocks[partitioner.chType].pos() ) )
        {
          coding_tree( cs, partitioner, cuCtx );
        }
      } while( partitioner.nextPart( cs ) );

      partitioner.exitCurrSplit();
      return;
    }
  }

  if( cs.sps->getSpsNext().getUseGenBinSplit() )
  {
    const PartSplit split = CU::getSplitAtDepth( cu, partitioner.currDepth );

    if( implicitSplit != CU_DONT_SPLIT )
    {
      CHECK( split != implicitSplit, "Incorrect split - not the implicitly provided type" );
    }
    else
    {
      gen_bin_split_mode( split, cs, partitioner );
      if( split != CU_DONT_SPLIT )
      {
        gen_bin_split_mod( split, cs, partitioner );
      }
    }

    if( split != CU_DONT_SPLIT )
    {
      partitioner.splitCurrArea( split, cs );
      do
      {
        if( cs.picture->Y().contains( partitioner.currArea().lumaPos() ) )
        {
          coding_tree( cs, partitioner, cuCtx );
        }
      } while( partitioner.nextPart( cs ) );

      partitioner.exitCurrSplit();
      return;
    }
  }
  else
  {
    //// MT
    //bool mtSplit = partitioner.canSplit( CU_MT_SPLIT, cs );

    //if( mtSplit )
    // MT
    bool mtSplit = partitioner.canSplit( CU_MT_SPLIT, cs );

    if( mtSplit )
    {
      const PartSplit splitMode = CU::getSplitAtDepth( cu, partitioner.currDepth );

      CHECK( implicitSplit != CU_DONT_SPLIT && implicitSplit != splitMode, "Different split found than the implicit split" );

      if( implicitSplit == CU_DONT_SPLIT )
      {
        split_cu_mode_mt( splitMode, cs, partitioner );
      }

      if( splitMode != CU_DONT_SPLIT )
      {
        partitioner.splitCurrArea( splitMode, cs );
        do
        {
          if( cs.picture->blocks[partitioner.chType].contains( partitioner.currArea().blocks[partitioner.chType].pos() ) )
          {
            coding_tree( cs, partitioner, cuCtx );
          }
        } while( partitioner.nextPart( cs ) );

        partitioner.exitCurrSplit();
        return;
      }
    }
  }

  // Predict QP on start of quantization group
  if( pps.getUseDQP() && !cuCtx.isDQPCoded && CU::isQGStart( cu ) )
  {
    cuCtx.qp = CU::predictQP( cu, cuCtx.qp );
  }

  // coding unit
  coding_unit( cu, partitioner, cuCtx );

  DTRACE_COND( ( isEncoding() ), g_trace_ctx, D_QP, "x=%d, y=%d, w=%d, h=%d, qp=%d\n", cu.Y().x, cu.Y().y, cu.Y().width, cu.Y().height, cu.qp );
  DTRACE_BLOCK_REC_COND( ( !isEncoding() ), cs.picture->getRecoBuf( cu ), cu, cu.predMode );
}

void CABACWriter::gen_bin_split_mode( const PartSplit split, const CodingStructure& cs, Partitioner& partitioner )
{
  const unsigned width      = partitioner.currArea().lumaSize().width;
  const unsigned height     = partitioner.currArea().lumaSize().height;
  auto &gbsPartitioner      = *dynamic_cast<GenBinSplitPartitioner*>( &partitioner );

  DecisionTree dt( g_genBinSplitDTT );

  dt.setAvail( DTT_GBS_PERP_SPLIT, gbsPartitioner.canSplit( CU_PERPENDICULAR_SPLIT, cs ) );
  dt.setAvail( DTT_GBS_PARL_SPLIT, gbsPartitioner.canSplit( CU_PARALLEL_SPLIT,      cs ) );
  dt.setAvail( DTT_GBS_DONT_SPLIT, gbsPartitioner.canSplit( CU_DONT_SPLIT,          cs ) );

  unsigned btSCtxId = width == height ? 1 : ( width > height ? 0 : 2 );
  if( gbsPartitioner.getActualSplitType( CU_PERPENDICULAR_SPLIT ) == CU_VERT_SPLIT )
  {
    btSCtxId = 2 - btSCtxId;
  }
  unsigned btCtxIdx = DeriveCtx::CtxBTsplit( cs, partitioner );

  dt.setCtxId( DTT_GBS_DO_SPLIT,      Ctx::GBSSplitFlag(     btCtxIdx ) );
  dt.setCtxId( DTT_GBS_DO_PERP_SPLIT, Ctx::GBSSplitFlag( 3 + btSCtxId ) );

  const PartSplit pseudoSplit = gbsPartitioner.getPseudoSplitType( split );

  GeneralizedBinSplitDecisionTree splitId = DTT_GBS_DONT_SPLIT;

  if( pseudoSplit == CU_PERPENDICULAR_SPLIT ) splitId = DTT_GBS_PERP_SPLIT;
  else if( pseudoSplit == CU_PARALLEL_SPLIT ) splitId = DTT_GBS_PARL_SPLIT;
  else                                        splitId = DTT_GBS_DONT_SPLIT;

  encode_sparse_dt( dt, splitId );

  DTRACE( g_trace_ctx, D_SYNTAX, "gen_bin_split_mode() %dx%d at [%d,%d], btSCtx = %d, btCtx = %d, split = %d\n", width, height, partitioner.currArea().lx(), partitioner.currArea().ly(), btSCtxId, btCtxIdx, splitId );
}

void CABACWriter::gen_bin_split_mod( const PartSplit split, const CodingStructure& cs, Partitioner& partitioner )
{
  const SplitModifier mod = getModifier ( split );
  const PartSplit base    = getBaseSplit( split );
  auto &gbsPartitioner    = *dynamic_cast<GenBinSplitPartitioner*>( &partitioner );
  const unsigned width    = partitioner.currArea().lumaSize().width;
  const unsigned height   = partitioner.currArea().lumaSize().height;

  DecisionTree dt( g_splitModifierDTT );

  dt.setAvail( DTT_SM_12, gbsPartitioner.canModify( base, SPLIT_MOD_12, cs ) );
  dt.setAvail( DTT_SM_14, gbsPartitioner.canModify( base, SPLIT_MOD_14, cs ) );
  dt.setAvail( DTT_SM_34, gbsPartitioner.canModify( base, SPLIT_MOD_34, cs ) );
  dt.setAvail( DTT_SM_38, gbsPartitioner.canModify( base, SPLIT_MOD_38, cs ) );
  dt.setAvail( DTT_SM_58, gbsPartitioner.canModify( base, SPLIT_MOD_58, cs ) );
  dt.setAvail( DTT_SM_13, gbsPartitioner.canModify( base, SPLIT_MOD_13, cs ) );
  dt.setAvail( DTT_SM_23, gbsPartitioner.canModify( base, SPLIT_MOD_23, cs ) );
  dt.setAvail( DTT_SM_15, gbsPartitioner.canModify( base, SPLIT_MOD_15, cs ) );
  dt.setAvail( DTT_SM_25, gbsPartitioner.canModify( base, SPLIT_MOD_25, cs ) );
  dt.setAvail( DTT_SM_35, gbsPartitioner.canModify( base, SPLIT_MOD_35, cs ) );
  dt.setAvail( DTT_SM_45, gbsPartitioner.canModify( base, SPLIT_MOD_45, cs ) );

  unsigned dirCtx       = DeriveCtx::CtxSplitMod( cs, partitioner, base );
  unsigned btSCtxId     = width == height ? 1 : ( width > height ? 0 : 2 );
  if( gbsPartitioner.getActualSplitType( CU_PERPENDICULAR_SPLIT ) == CU_VERT_SPLIT )
  {
    btSCtxId   = 2 - btSCtxId;
  }

  dt.setCtxId( DTT_SM_IS_ASYM,  Ctx::GBSSplitMod(     btSCtxId ) );          //  0 -  2
  dt.setCtxId( DTT_SM_IS_x3,    Ctx::GBSSplitMod( 3            ) );          //  3
  dt.setCtxId( DTT_SM_IS_x5,    Ctx::GBSSplitMod( 3            ) );          //  3
  dt.setCtxId( DTT_SM_IS_QUART, Ctx::GBSSplitMod( 4 + btSCtxId ) );          //  4 -  6

  dt.setCtxId( DTT_SM_IS_14,    Ctx::GBSSplitMod( dirCtx + 7   ) );          //  7 -  9
  dt.setCtxId( DTT_SM_IS_38,    Ctx::GBSSplitMod( dirCtx + 7   ) );          //  7 -  9
  dt.setCtxId( DTT_SM_IS_13,    Ctx::GBSSplitMod( dirCtx + 7   ) );          //  7 -  9
  dt.setCtxId( DTT_SM_IS_15,    Ctx::GBSSplitMod( dirCtx + 7   ) );          //  7 -  9
  dt.setCtxId( DTT_SM_IS_25,    Ctx::GBSSplitMod( dirCtx + 7   ) );          //  7 -  9
  dt.setCtxId( DTT_SM_IS_235,   Ctx::GBSSplitMod( dirCtx == 1 ? 10 : 11 ) ); // 10 - 11

  encode_sparse_dt( dt, mod );

  DTRACE( g_trace_ctx, D_SYNTAX, "gen_bin_split_mod() %dx%d at [%d,%d] qtd=%d d=%d base=%d dirCtx=%d, mod = %d\n", partitioner.currArea().lwidth(), partitioner.currArea().lheight(), partitioner.currArea().lx(), partitioner.currArea().ly(), partitioner.currQtDepth, partitioner.currDepth, base, dirCtx, mod );
}

void CABACWriter::split_cu_flag( bool split, const CodingStructure& cs, Partitioner& partitioner )
{
  unsigned maxQTDepth = ( cs.sps->getSpsNext().getUseGenBinSplit() || cs.sps->getSpsNext().getUseQTBT()
    ? g_aucLog2[cs.sps->getSpsNext().getCTUSize()] - g_aucLog2[cs.sps->getSpsNext().getMinQTSize( cs.slice->getSliceType(), partitioner.chType )]
    : cs.sps->getLog2DiffMaxMinCodingBlockSize() );
//#else
//  unsigned maxQTDepth = ( cs.sps->getSpsNext().getUseQTBT()
//    ? g_aucLog2[cs.sps->getSpsNext().getCTUSize()] - g_aucLog2[cs.sps->getSpsNext().getMinQTSize( cs.slice->getSliceType(), partitioner.chType )]
//    : cs.sps->getLog2DiffMaxMinCodingBlockSize() );
//#endif
  if( partitioner.currDepth == maxQTDepth )
  {
    return;
  }
  unsigned  ctxId = DeriveCtx::CtxCUsplit( cs, partitioner );
  m_BinEncoder.encodeBin( (split), Ctx::SplitFlag(ctxId) );

  DTRACE( g_trace_ctx, D_SYNTAX, "split_cu_flag() ctx=%d split=%d\n", ctxId, split ? 1 : 0 );
}

void CABACWriter::split_cu_mode_mt(const PartSplit split, const CodingStructure& cs, Partitioner& partitioner)
{
  unsigned ctxIdBT = DeriveCtx::CtxBTsplit( cs, partitioner );

  unsigned width   = partitioner.currArea().lumaSize().width;
  unsigned height  = partitioner.currArea().lumaSize().height;

  DecisionTree dt( g_mtSplitDTT );

  dt.setAvail( DTT_SPLIT_BT_HORZ, partitioner.canSplit( CU_HORZ_SPLIT, cs ) );
  dt.setAvail( DTT_SPLIT_BT_VERT, partitioner.canSplit( CU_VERT_SPLIT, cs ) );

  dt.setAvail( DTT_SPLIT_TT_HORZ, partitioner.canSplit( CU_TRIH_SPLIT,    cs ) );
  dt.setAvail( DTT_SPLIT_TT_VERT, partitioner.canSplit( CU_TRIV_SPLIT,    cs ) );

  dt.setAvail( DTT_SPLIT_BT_H_14, partitioner.canSplit( CU_HORZ_SPLIT_14, cs ) );
  dt.setAvail( DTT_SPLIT_BT_H_34, partitioner.canSplit( CU_HORZ_SPLIT_34, cs ) );
  dt.setAvail( DTT_SPLIT_BT_V_14, partitioner.canSplit( CU_VERT_SPLIT_14, cs ) );
  dt.setAvail( DTT_SPLIT_BT_V_34, partitioner.canSplit( CU_VERT_SPLIT_34, cs ) );

  unsigned btSCtxId = width == height ? 0 : ( width > height ? 1 : 2 );
  dt.setCtxId( DTT_SPLIT_DO_SPLIT_DECISION,   Ctx::BTSplitFlag( ctxIdBT ) );
  dt.setCtxId( DTT_SPLIT_HV_DECISION,         Ctx::BTSplitFlag( 3 + btSCtxId ) );

  dt.setCtxId( DTT_SPLIT_H_IS_BT_12_DECISION, Ctx::BTSplitFlag( 6 + btSCtxId ) );
  dt.setCtxId( DTT_SPLIT_V_IS_BT_12_DECISION, Ctx::BTSplitFlag( 9 + btSCtxId ) );


  encode_sparse_dt( dt, split == CU_DONT_SPLIT ? ( unsigned ) DTT_SPLIT_NO_SPLIT : ( unsigned ) split );

  DTRACE(g_trace_ctx, D_SYNTAX, "split_cu_mode_mt() ctx=%d split=%d\n", ctxIdBT, split);
}


//================================================================================
//  clause 7.3.8.5
//--------------------------------------------------------------------------------
//    void  coding_unit               ( cu, partitioner, cuCtx )
//    void  cu_transquant_bypass_flag ( cu )
//    void  cu_skip_flag              ( cu )
//    void  pred_mode                 ( cu )
//    void  part_mode                 ( cu )
//    void  pcm_flag                  ( cu )
//    void  pcm_samples               ( tu )
//    void  cu_pred_data              ( pus )
//    void  cu_lic_flag               ( cu )
//    void  intra_luma_pred_modes     ( pus )
//    void  intra_chroma_pred_mode    ( pu )
//    void  cu_residual               ( cu, partitioner, cuCtx )
//    void  rqt_root_cbf              ( cu )
//    void  end_of_ctu                ( cu, cuCtx )
//================================================================================

void CABACWriter::coding_unit( const CodingUnit& cu, Partitioner& partitioner, CUCtx& cuCtx )
{
  CodingStructure& cs = *cu.cs;

  // transquant bypass flag
  if( cs.pps->getTransquantBypassEnabledFlag() )
  {
    cu_transquant_bypass_flag( cu );
  }

  // skip flag
  if( !cs.slice->isIntra() )
  {
    cu_skip_flag( cu );
  }

  if( cu.slice->getSPS()->getSpsNext().getDiffusionFilterMode() >> 2 )
  {
    CHECK( cu.partSize == SIZE_2Nx2N && cu.firstPU->mergeFlag && !cu.rootCbf &&cu.diffFilterIdx != 0, "pu.mergeFlag && !cu.rootCbf && cu.diffFilterIdx != 0 ist nicht erlaubt" );
  }

  // skip data
  if( cu.skip )
  {
    CHECK( !cu.firstPU->mergeFlag, "Merge flag has to be on!" );
    PredictionUnit&   pu = *cu.firstPU;
    prediction_unit ( pu );
    cu_lic_flag     ( cu );
    if (cu.slice->getSPS()->getSpsNext().getDiffusionFilterMode() >> 2)
    {
      CHECK(cu.diffFilterIdx != 0, " cu.diffFilterIdx != 0 && skip ist nihct erlaubt");
    }

    CHECK( pu.frucMrgMode != FRUC_MERGE_OFF && cu.diffFilterIdx != 0, "PDE selected in FRUC SKIP" );
    end_of_ctu      ( cu, cuCtx );
    return;
  }

  // prediction mode and partitioning data
  pred_mode ( cu );
  mrl_idx   ( cu );
  pdpc_flag ( cu );
  part_mode ( cu );

  // pcm samples
  if( CU::isIntra(cu) && cu.partSize == SIZE_2Nx2N )
  {
    pcm_data( cu );
    if( cu.ipcm )
    {
      end_of_ctu( cu, cuCtx );
      return;
    }
  }

  // prediction data ( intra prediction modes / reference indexes + motion vectors )
  cu_pred_data( cu );
  if( CU::isDiffIdxPresent( cu ) )
  {
    cu_diffusion_filter_idx( cu );
  }
  else
  {
    if( cu.slice->getSPS()->getSpsNext().getDiffusionFilterMode() >> 2 )
    {
      CHECK( cu.diffFilterIdx, "cu.diffFilterIdx !=0" );
    }
  }

  // residual data ( coded block flags + transform coefficient levels )
  if( cu.mode1dPartitions )
  {
    TU1dPartitioner partitionerFor1dTus( partitioner );
    cu_residual( cu, partitionerFor1dTus, cuCtx );
  }
  else
  {
    cu_residual( cu, partitioner, cuCtx );
  }

  // end of cu
  end_of_ctu( cu, cuCtx );
}


void CABACWriter::cu_transquant_bypass_flag( const CodingUnit& cu )
{
  m_BinEncoder.encodeBin( (cu.transQuantBypass), Ctx::TransquantBypassFlag() );
}


void CABACWriter::cu_skip_flag( const CodingUnit& cu )
{
  CHECK( cu.skip && cu.firstPU->numMergedAddHyps != cu.firstPU->addHypData.size(), "Multi Hyp: cu.skip && cu.firstPU->numMergedAddHyps != cu.firstPU->addHypData.size()" );
  CHECK( cu.skip && !cu.firstPU->mergeFlag, "merge_flag has to be true for skipped CUs" );
  unsigned ctxId = DeriveCtx::CtxSkipFlag( cu );
  m_BinEncoder.encodeBin( ( cu.skip ), Ctx::SkipFlag( ctxId ) );

  DTRACE( g_trace_ctx, D_SYNTAX, "cu_skip_flag() ctx=%d skip=%d\n", ctxId, cu.skip ? 1 : 0 );
}


void CABACWriter::pred_mode( const CodingUnit& cu )
{
  if( cu.cs->slice->isIntra() )
  {
    return;
  }
  m_BinEncoder.encodeBin( ( CU::isIntra( cu ) ), Ctx::PredMode() );
}

void CABACWriter::part_mode( const CodingUnit& cu )
{
  if( cu.cs->pcv->only2Nx2N )
  {
    CHECK( cu.partSize != SIZE_2Nx2N, "No CU sub-partitionining allowed with QTBT" );
    return;
  }

  const SPS&      sps       = *cu.cs->sps;
  const unsigned  cuWidth   = cu.lumaSize().width;
  const unsigned  cuHeight  = cu.lumaSize().height;
  const int       log2DiffMaxMinCodingBlockSize = sps.getLog2DiffMaxMinCodingBlockSize();
  const PartSize  partSize  = cu.partSize;

  DecisionTree dt( g_partSizeDTT );

  dt.setCtxId( DTT_PS_IS_2Nx2N, Ctx::PartSize() );

  if( CU::isIntra( cu ) )
  {
    dt.setAvail( DTT_PS_nLx2N, false );
    dt.setAvail( DTT_PS_2NxN,  false );
    dt.setAvail( DTT_PS_Nx2N,  false );
    dt.setAvail( DTT_PS_nRx2N, false );
    dt.setAvail( DTT_PS_2NxnU, false );
    dt.setAvail( DTT_PS_2NxnD, false );
    dt.setAvail( DTT_PS_NxN,   cu.qtDepth == log2DiffMaxMinCodingBlockSize );
  }
  else
  {
    const bool isAmpAvail = sps.getUseAMP() && cu.qtDepth < log2DiffMaxMinCodingBlockSize;

    dt.setAvail( DTT_PS_2NxN,  true );
    dt.setAvail( DTT_PS_Nx2N,  true );
    dt.setAvail( DTT_PS_nLx2N, isAmpAvail );
    dt.setAvail( DTT_PS_nRx2N, isAmpAvail );
    dt.setAvail( DTT_PS_2NxnU, isAmpAvail );
    dt.setAvail( DTT_PS_2NxnD, isAmpAvail );
    dt.setAvail( DTT_PS_NxN,   cu.qtDepth == log2DiffMaxMinCodingBlockSize && !( cuWidth == 8 && cuHeight == 8 ) );

    dt.setCtxId( DTT_PS_IS_2Nx,     Ctx::PartSize( 1 ) );
    dt.setCtxId( DTT_PS_IS_2NxN,    Ctx::PartSize( 3 ) );
    dt.setCtxId( DTT_PS_IS_NOT_NxN, Ctx::PartSize( 2 ) );
    dt.setCtxId( DTT_PS_IS_Nx2N,    Ctx::PartSize( 3 ) );
  }

  encode_sparse_dt( dt, partSize );
}

void CABACWriter::pcm_data( const CodingUnit& cu )
{
  pcm_flag( cu );
  if( cu.ipcm )
  {
    m_BinEncoder.pcmAlignBits();
    pcm_samples( *cu.firstTU );
  }
}

void CABACWriter::mrl_idx( const CodingUnit& cu )
{
  if ( !cu.cs->sps->getSpsNext().getUseIntraMRL() || cu.predMode != MODE_INTRA || !isLuma( cu.chType ) )
  {
    return;
  }

  // BB: TODO fast modes
  const int log2CuHeight  = g_aucLog2[cu.lumaSize().height];
  const int log2CuWidth   = g_aucLog2[cu.lumaSize().width];
  CHECK( ( abs( log2CuHeight - log2CuWidth ) > 1 || log2CuHeight < MRL_FAST_LOG2_MIN_SIZE || log2CuWidth < MRL_FAST_LOG2_MIN_SIZE ) && cu.mrlIdx, "mrlIdx > 0 for partitions with one side more than twice as large than the other, e.g. 16x4!" );
  if( abs( log2CuHeight - log2CuWidth ) > 1  || log2CuHeight < MRL_FAST_LOG2_MIN_SIZE || log2CuWidth < MRL_FAST_LOG2_MIN_SIZE )
  {
    return;
  }

  m_BinEncoder.encodeBin( cu.mrlIdx > 0 ? 1 : 0, Ctx::MrlIdx( 0 ) );
  if ( cu.mrlIdx != 0 )
  {
    m_BinEncoder.encodeBin( cu.mrlIdx > 1 ? 1 : 0, Ctx::MrlIdx( 1 ) );
  }
  DTRACE( g_trace_ctx, D_SYNTAX, "mrl_idx() value=%d pos=(%d,%d)\n", cu.mrlIdx, cu.lumaPos().x, cu.lumaPos().y );
}

void CABACWriter::pdpc_flag( const CodingUnit& cu )
{
  if (!cu.cs->sps->getSpsNext().isIntraPDPC() || cu.predMode == MODE_INTER)
  {
    return;
  }
  if ( cu.mrlIdx != 0 )
  {
    return;
  }

  m_BinEncoder.encodeBin( cu.pdpc, Ctx::PdpcFlag() );
}

void CABACWriter::pcm_flag( const CodingUnit& cu )
{
  const SPS& sps = *cu.cs->sps;
  if( !sps.getUsePCM() || cu.lumaSize().width > (1 << sps.getPCMLog2MaxSize()) || cu.lumaSize().width < (1 << sps.getPCMLog2MinSize()) )
  {
    return;
  }
  m_BinEncoder.encodeBinTrm( cu.ipcm );
}


void CABACWriter::cu_pred_data( const CodingUnit& cu )
{
  if( CU::isIntra( cu ) )
  {
    intra_luma_pred_modes  ( cu );
    intra_chroma_pred_modes( cu );
    return;
  }
  for( auto &pu : CU::traversePUs( cu ) )
  {
    prediction_unit( pu );
  }

  imv_mode   ( cu );
  obmc_flag  ( cu );
  cu_lic_flag( cu );
}

void CABACWriter::cu_lic_flag( const CodingUnit& cu )
{
  if( CU::isLICFlagPresent( cu ) )
  {
    m_BinEncoder.encodeBin( cu.LICFlag ? 1 : 0, Ctx::LICFlag() );
    DTRACE( g_trace_ctx, D_SYNTAX, "cu_lic_flag() lic_flag=%d\n", cu.LICFlag?1:0 );
  }
}

void CABACWriter::obmc_flag( const CodingUnit& cu )
{
  if( !cu.cs->sps->getSpsNext().getUseOBMC() )
  {
    return;
  }

  Bool bCoded = CU::isObmcFlagCoded ( cu );

  if ( bCoded )
  {
    m_BinEncoder.encodeBin ( cu.obmcFlag ? 1 : 0, Ctx::ObmcFlag () );
  }

  DTRACE ( g_trace_ctx, D_SYNTAX, "obmc_flag() obmc=%d pos=(%d,%d)\n", cu.obmcFlag ? 1 : 0, cu.lumaPos ().x, cu.lumaPos ().y );
}

void CABACWriter::nn_flag( const CodingUnit& cu )
{
  if( !cu.cs->sps->getSpsNext().getUseIntra_NN() )
  {
    return;
  }
  if( !cu.Y().valid() )
  {
    return;
  }
  if( cu.pdpc )
  {
    CHECK( cu.intra_NN, "Error" );
    return;
  }
  if( cu.lwidth() > g_max_size_NNs )
  {
    return;
  }
  if( cu.lheight() > g_max_size_NNs )
  {
    return;
  }
  if( cu.mrlIdx )
  {
    CHECK( cu.intra_NN, "Error" );
    return;
  }

  const int padded_width  = g_PaddedSizes[cu.lwidth()];
  const int padded_height = g_PaddedSizes[cu.lheight()];
  const int w = g_aucLog2[padded_width];
  const int h = g_aucLog2[padded_height];
  bool intraModelActive = ( g_numIntraModes_NN[w][h] > 0 );

  if( (padded_width <= g_max_size_NNs) && (padded_height <= g_max_size_NNs) )
  {
    intraModelActive = true;
  }

  if( !intraModelActive )
  {
    return;
  }
  unsigned ctxId = DeriveCtx::CtxIntraNNModel( cu );
  m_BinEncoder.encodeBin( ( cu.intra_NN > 0 ), Ctx::IntraNNFlag( ctxId ) );
  nn_subsampling_flag( cu );
}

void CABACWriter::code_unary_fixed( unsigned symbol, unsigned ctxId, unsigned unary_max, unsigned fixed )
{
  bool unary = ( symbol <= unary_max );
  m_BinEncoder.encodeBin( unary, ctxId );
  if( unary )
  {
    unary_max_eqprob( symbol, unary_max );
  }
  else
  {
    m_BinEncoder.encodeBinsEP( symbol - unary_max - 1, fixed );
  }
}

void CABACWriter::intra_luma_pred_modes_NN( const CodingUnit& cu )
{
  if( !cu.Y().valid() )
  {
    return;
  }
  for( const auto &pu : CU::traversePUs( cu ) )
  {
    intra_luma_pred_mode_NN( pu );
  }
}

void CABACWriter::intra_luma_pred_mode_NN( const PredictionUnit& pu )
{
  const int padded_width = g_PaddedSizes[pu.lwidth()];
  const int padded_height = g_PaddedSizes[pu.lheight()];
  const int width_for_NN = pu.cu->intra_NN_Use_Sampling ? ( padded_width >> 1 ) : padded_width;
  const int height_for_NN = pu.cu->intra_NN_Use_Sampling ? ( padded_height >> 1 ) : padded_height;
  const int w = g_aucLog2[width_for_NN];
  const int h = g_aucLog2[height_for_NN];

  unsigned idx = pu.intraDir[CHANNEL_TYPE_LUMA];
  CHECK( !( idx < g_numIntraModes_NN[w][h] ), "intra mode index out of range" );
  int unaryMax = g_numIntraMPM_NN[w][h] - 1;
  int fixedLength = g_numIntraEPBins_NN[w][h];
  code_unary_fixed( idx, Ctx::IntraPredModeANN[CHANNEL_TYPE_LUMA]( 0 ), unaryMax, fixedLength );
  DTRACE( g_trace_ctx, D_SYNTAX, "intra_luma_pred_mode_model() pos=(%d,%d) mode=%d\n", pu.lumaPos().x, pu.lumaPos().y, pu.intraDir[CHANNEL_TYPE_LUMA] );
}

void CABACWriter::nn_subsampling_flag( const CodingUnit& cu )
{
  if( !cu.intra_NN )
  {
    CHECK( cu.intra_NN_Use_Sampling != 0, "Error" );
    return;
  }
  if( cu.lwidth() == 64 && cu.lheight() == 64 )
  {
    CHECK( cu.intra_NN_Use_Sampling != 1, "Error" );
    return;
  }
  if( !subSamplingAllowed( cu.lwidth(), cu.lheight() ) )
  {
    CHECK( cu.intra_NN_Use_Sampling != 0, "Error" );
    return;
  }
  m_BinEncoder.encodeBinEP( cu.intra_NN_Use_Sampling );
}

void CABACWriter::cu_diffusion_filter_idx( const CodingUnit& cu )
{
  CHECK( !cu.cs->sps->getSpsNext().getDiffusionFilterEnabled(), "PDE is not enabled" );
  CHECK( !cu.Y().valid(), "luma not valid" );

  const unsigned  isInter = ( CU::isInter(cu) ? 1 : 0 );

  unsigned  numFilt = (isInter ? cu.cs->sps->getSpsNext().getNumDiffusionFiltersInter() : cu.cs->sps->getSpsNext().getNumDiffusionFiltersIntra());

  bool Min_Sign = true;
  if ((cu.cs->sps->getSpsNext().getRestrDiffusionMode() >> 1) & 1)
  {
    if ((cu.Y().width == 8) || (cu.Y().height == 8))
    {
      Min_Sign = false;
    }
  }

  CHECK( numFilt == 0, "diffusion not allowed" );
  unsigned  ctxIdx  = Ctx::DiffusionFilterIdx( isInter << 3 );  // offset = BaseIdx + [0 (intra) | 8 (inter)]
  const unsigned  enabled = ( cu.diffFilterIdx ? 1 : 0 );
  if (isInter) {
    ctxIdx = Ctx::DiffusionFilterIdx((1 << 3) + (cu.firstPU->mergeFlag << 3));  // offset = BaseIdx + [ 8 (inter)] + [0 (nonmerge) | 8 (merge)]
  }
  m_BinEncoder.encodeBin( enabled, ctxIdx );

  if( enabled )
  {
    const unsigned  fIdx    = cu.diffFilterIdx - 1;
    const unsigned  nonlin  = ( fIdx >> 1 );
    const unsigned  odd = ( fIdx  & 1 );

    ctxIdx++; // [B+1]
    if( numFilt > 2 )
    {
      m_BinEncoder.encodeBin( nonlin, ctxIdx );
    }
    ctxIdx+= 1+nonlin; // [B+2|B+3]

    if( ( ( nonlin && numFilt > 3 ) || ( ( !nonlin && numFilt > 1 ) && Min_Sign ) ) )
    {
      m_BinEncoder.encodeBin( odd, ctxIdx );
    }
  }

  DTRACE( g_trace_ctx, D_SYNTAX, "cu_diffusion_filter_idx() fidx=%d\n", cu.diffFilterIdx );
}

void CABACWriter::intra_luma_pred_modes( const CodingUnit& cu )
{
  if( !cu.Y().valid() )
  {
    return;
  }

  nn_flag( cu );
  if( cu.intra_NN > 0 )
  {
    intra_luma_pred_modes_NN( cu );
    return;
  }

  unsigned numMPMs   = cu.cs->pcv->numMPMs;
  int      numBlocks = CU::getNumPUs( cu );
  unsigned *mpm_preds  [4];
  unsigned mpm_idxs    [4];
  unsigned ipred_modes [4];

  const PredictionUnit* pu = cu.firstPU;
  const bool use65Ang = cu.cs->sps->getSpsNext().getUseIntra65Ang();

  for( int k = 0; k < numBlocks; k++ )
  {
    if( cu.cs->sps->getSpsNext().getUseIntraFTM() && !cu.mrlIdx )
    {
      const bool FTMFlag = pu->FTMRegIdx > 0;
      m_BinEncoder.encodeBin( FTMFlag ? 1 : 0, Ctx::FtmFlag() );
      if( FTMFlag )
      {
        unsigned FTMIdx = pu->FTMRegIdx - 1;
        CHECK( !( FTMIdx >= 0 && FTMIdx < NUM_FTM_REG ), "Invalid FTMRegIdx" );
        m_BinEncoder.encodeBin( FTMIdx ? 1 : 0, Ctx::FtmReg[0]() );
        if( FTMIdx )
        {
          unsigned Data = FTMIdx - 1;
          unsigned B1   = Data / 2;
          unsigned B2   = Data % 2;
          m_BinEncoder.encodeBin( B1, Ctx::FtmReg[1]( 0 ) );
          m_BinEncoder.encodeBin( B2, Ctx::FtmReg[1]( 1 ) );
        }

        DTRACE( g_trace_ctx, D_SYNTAX, "intra_luma_pred_modes() idx=%d pos=(%d,%d) mode=FTM:%d\n", k, pu->lumaPos().x, pu->lumaPos().y, pu->FTMRegIdx );
      }
    }
    else
    {
      CHECK( pu->FTMRegIdx > 0 && pu->cu->mrlIdx > 0, "Invalid combination of FTMRegIdx > 0 and mrlIdx > 0" );
    }
    pu = pu->next;
  }
  pu = cu.firstPU;

  // prev_intra_luma_pred_flag
  for( int k = 0; k < numBlocks; k++ )
  {
    if( pu->FTMRegIdx != 0 )
    {
      pu = pu->next;
      continue;
    }
    unsigned*& mpm_pred   = mpm_preds[k];
    unsigned&  mpm_idx    = mpm_idxs[k];
    unsigned&  ipred_mode = ipred_modes[k];

    mpm_pred = ( unsigned* ) alloca( numMPMs * sizeof( unsigned ) );
    PU::getIntraMPMs( *pu, mpm_pred );

    ipred_mode = pu->intraDir[0];
    mpm_idx    = numMPMs;
    for( unsigned idx = 0; idx < numMPMs; idx++ )
    {
      if( ipred_mode == mpm_pred[idx] )
      {
        mpm_idx = idx;
        break;
      }
    }
    m_BinEncoder.encodeBin( mpm_idx < numMPMs, Ctx::IPredMode[0]() );

    pu = pu->next;
  }

  pu = cu.firstPU;

  // mpm_idx / rem_intra_luma_pred_mode
  for( int k = 0; k < numBlocks; k++ )
  {
    if( pu->FTMRegIdx != 0 )
    {
      pu = pu->next;
      continue;
    }
    const unsigned& mpm_idx = mpm_idxs[k];
    if( mpm_idx < numMPMs )
    {
      if( use65Ang )
      {
        DecisionTree dt( g_intraLumaMpmDTT );

        dt.setCtxId( DTT_INTRA_MPM_ISGT_0, Ctx::IPredMode[0]( mpmCtx[mpm_preds[k][0]] ) );
        dt.setCtxId( DTT_INTRA_MPM_ISGT_1, Ctx::IPredMode[0]( mpmCtx[mpm_preds[k][1]] ) );
        dt.setCtxId( DTT_INTRA_MPM_ISGT_2, Ctx::IPredMode[0]( mpmCtx[mpm_preds[k][2]] ) );

        encode_sparse_dt( dt, mpm_idx );
      }
      else
      {
        m_BinEncoder.encodeBinEP( mpm_idx > 0 );
        if( mpm_idx )
        {
          m_BinEncoder.encodeBinEP( mpm_idx > 1 );
        }
      }
    }
    else
    {
      unsigned* mpm_pred   = mpm_preds[k];
      unsigned  ipred_mode = ipred_modes[k];

      // sorting of MPMs
      std::sort( mpm_pred, mpm_pred + numMPMs );

      if( use65Ang )
      {
        for( int idx = int( numMPMs ) - 1; idx >= 0; idx-- )
        {
          if( ipred_mode > mpm_pred[idx] )
          {
            ipred_mode--;
          }
        }

        int RealNumIntraMode = cu.cs->sps->getSpsNext().getRealNumIntraMode();
        if( ipred_mode < ( RealNumIntraMode - 8 ) ) { m_BinEncoder.encodeBinsEP( ipred_mode,      6 ); }
        else                                        { m_BinEncoder.encodeBinsEP( ipred_mode >> 2, 4 ); }
      }
      else
      {
        CHECK( g_intraMode33to65AngMapping[g_intraMode65to33AngMapping[ipred_mode]] != ipred_mode, "Using an extended intra mode, although not enabled" );

        ipred_mode = g_intraMode65to33AngMapping[ipred_mode];
        for( int idx = int( numMPMs ) - 1; idx >= 0; idx-- )
        {
          if( ipred_mode > g_intraMode65to33AngMapping[mpm_pred[idx]] )
          {
            ipred_mode--;
          }
        }

        CHECK( ipred_mode >= 32, "Incorrect mode" );

        m_BinEncoder.encodeBinsEP( ipred_mode, 5 );
      }
    }

    DTRACE( g_trace_ctx, D_SYNTAX, "intra_luma_pred_modes() idx=%d pos=(%d,%d) mode=%d\n", k, pu->lumaPos().x, pu->lumaPos().y, pu->intraDir[0] );
    pu = pu->next;
  }
}


void CABACWriter::intra_luma_pred_mode( const PredictionUnit& pu )
{
  if( pu.cu->intra_NN != 0 )
  {
    intra_luma_pred_mode_NN( pu );
    return;
  }

  // prev_intra_luma_pred_flag
  unsigned  numMPMs  = pu.cs->pcv->numMPMs;
  unsigned *mpm_pred = ( unsigned* ) alloca( numMPMs * sizeof( unsigned ) );

  const bool use65Ang = pu.cs->sps->getSpsNext().getUseIntra65Ang();

  PU::getIntraMPMs( pu, mpm_pred );

  unsigned ipred_mode = pu.intraDir[0];
  if( pu.cs->sps->getSpsNext().getUseIntraFTM() )
  {
    const bool FTMFlag = pu.FTMRegIdx > 0;
    m_BinEncoder.encodeBin( FTMFlag ? 1 : 0, Ctx::FtmFlag() );
    if( FTMFlag )
    {
      unsigned FTMRegIdx = pu.FTMRegIdx - 1;
      CHECK( !( FTMRegIdx >= 0 && FTMRegIdx < NUM_FTM_REG ), "Invalid FTMRegIdx" );
      m_BinEncoder.encodeBin( ( FTMRegIdx ? 1 : 0 ), Ctx::FtmReg[0]() );
      if( FTMRegIdx )
      {
        unsigned Data = FTMRegIdx - 1;
        unsigned B1   = Data / 2;
        unsigned B2   = Data % 2;
        m_BinEncoder.encodeBin( B1, Ctx::FtmReg[1]( 0 ) );
        m_BinEncoder.encodeBin( B2, Ctx::FtmReg[1]( 1 ) );
      }
      return;
    }
  }
  unsigned mpm_idx = numMPMs;

  for( unsigned idx = 0; idx < numMPMs; idx++ )
  {
    if( ipred_mode == mpm_pred[idx] )
    {
      mpm_idx = idx;
      break;
    }
  }
  m_BinEncoder.encodeBin( mpm_idx < numMPMs, Ctx::IPredMode[0]() );

  // mpm_idx / rem_intra_luma_pred_mode
  if( mpm_idx < numMPMs )
  {
    if( use65Ang )
    {
      DecisionTree dt( g_intraLumaMpmDTT );

      dt.setCtxId( DTT_INTRA_MPM_ISGT_0, Ctx::IPredMode[0]( mpmCtx[mpm_pred[0]] ) );
      dt.setCtxId( DTT_INTRA_MPM_ISGT_1, Ctx::IPredMode[0]( mpmCtx[mpm_pred[1]] ) );
      dt.setCtxId( DTT_INTRA_MPM_ISGT_2, Ctx::IPredMode[0]( mpmCtx[mpm_pred[2]] ) );

      encode_sparse_dt( dt, mpm_idx );
    }
    else
    {
      m_BinEncoder.encodeBinEP( mpm_idx > 0 );
      if( mpm_idx )
      {
        m_BinEncoder.encodeBinEP( mpm_idx > 1 );
      }
    }
  }
  else
  {
    std::sort( mpm_pred, mpm_pred + numMPMs );
    if( use65Ang )
    {
      for( int idx = int( numMPMs ) - 1; idx >= 0; idx-- )
      {
        if( ipred_mode > mpm_pred[idx] )
        {
          ipred_mode--;
        }
      }
      int RealNumIntraMode = pu.cs->sps->getSpsNext().getRealNumIntraMode();
      if( ipred_mode < ( RealNumIntraMode - 8 ) ) { m_BinEncoder.encodeBinsEP( ipred_mode, 6 ); }
      else                                        { m_BinEncoder.encodeBinsEP( ( ipred_mode >> 2 ), 4 ); }
    }
    else
    {
      CHECK( g_intraMode33to65AngMapping[g_intraMode65to33AngMapping[ipred_mode]] != ipred_mode, "Using an extended intra mode, although not enabled" );

      ipred_mode = g_intraMode65to33AngMapping[ipred_mode];
      for( int idx = int( numMPMs ) - 1; idx >= 0; idx-- )
      {
        if( ipred_mode > g_intraMode65to33AngMapping[mpm_pred[idx]] )
        {
          ipred_mode--;
        }
      }

      m_BinEncoder.encodeBinsEP( ipred_mode, 5 );
    }
  }
}


void CABACWriter::intra_chroma_pred_modes( const CodingUnit& cu )
{
  if( cu.chromaFormat == CHROMA_400 || ( CS::isDualITree( *cu.cs ) && cu.chType == CHANNEL_TYPE_LUMA ) )
  {
    return;
  }

  int numBlocks = enable4ChromaPUsInIntraNxNCU( cu.chromaFormat ) ? CU::getNumPUs( cu ) : 1;

  const PredictionUnit* pu = cu.firstPU;

  for( int k = 0; k < numBlocks; k++ )
  {
    intra_chroma_pred_mode( *pu );
    pu = pu->next;
  }

  assert( numBlocks == 1 || pu == nullptr );
}

void CABACWriter::intra_chroma_lmc_mode( const PredictionUnit& pu )
{
  const unsigned intraDir = pu.intraDir[1];

  if ( pu.cs->sps->getSpsNext().getUseMDMS() )
  {
    m_BinEncoder.encodeBin( PU::isLMCMode( intraDir ) ? 0 : 1, Ctx::IPredMode[1]( 0 ) );
    if( PU::isLMCMode( intraDir ) )
    {
      unsigned ctxId = 6;
      if ( PU::isMMLMEnabled( pu ) )
      {
        m_BinEncoder.encodeBin( intraDir == MMLM_CHROMA_IDX ? 1 : 0, Ctx::IPredMode[1]( ctxId++ ) );
      }
      if ( PU::isMFLMEnabled( pu ) )
      {
        if ( intraDir != MMLM_CHROMA_IDX )
        {
          m_BinEncoder.encodeBin( intraDir == LM_CHROMA_IDX ? 1 : 0, Ctx::IPredMode[1]( ctxId++ ) );
          if ( intraDir != LM_CHROMA_IDX )
          {
            int candId = intraDir - LM_CHROMA_F1_IDX;
            m_BinEncoder.encodeBin( (candId >> 1) & 1, Ctx::IPredMode[1]( ctxId++ ) );
            m_BinEncoder.encodeBin(  candId       & 1, Ctx::IPredMode[1]( ctxId++ ) );
          }
        }
      }
    }
  }
  else
  {
    int lmModeList[10];
    int maxSymbol = PU::getLMSymbolList( pu, lmModeList );
    int symbol    = -1;
    for ( int k = 0; k < LM_SYMBOL_NUM; k++ )
    {
      if ( lmModeList[k] == intraDir || ( lmModeList[k] == -1 && intraDir < LM_CHROMA_IDX ) )
      {
        symbol = k;
        break;
      }
    }
    CHECK( symbol < 0, "invalid symbol found" );

    unary_max_symbol( symbol, Ctx::IPredMode[1]( 2 ), Ctx::IPredMode[1]( 3 ), maxSymbol - 1 );
  }
}

void CABACWriter::intra_chroma_pred_mode( const PredictionUnit& pu )
{
  const unsigned intraDir = pu.intraDir[1];

  // DM chroma index
  if ( pu.cs->sps->getSpsNext().getUseMDMS() )
  {
  }
  else
  {
    if( intraDir == DM_CHROMA_IDX )
    {
      m_BinEncoder.encodeBin( 0, Ctx::IPredMode[1]( 1 ) );
      return;
    }
    m_BinEncoder.encodeBin( 1, Ctx::IPredMode[1]( 1 ) );
  }

  // LM chroma mode
  if( pu.cs->sps->getSpsNext().getUseLMChroma() )
  {
    intra_chroma_lmc_mode( pu );
    if ( PU::isLMCMode( intraDir ) )
    {
      return;
    }
  }

  // chroma candidate index
  unsigned chromaCandModes[ NUM_CHROMA_MODE ];
  PU::getIntraChromaCandModes( pu, chromaCandModes );

  int candId = 0;
  for ( ; candId < NUM_CHROMA_MODE; candId++ )
  {
    if( intraDir == chromaCandModes[ candId ] )
    {
      break;
    }
  }

  CHECK( candId >= NUM_CHROMA_MODE, "Chroma prediction mode index out of bounds" );
  CHECK( chromaCandModes[ candId ] == DM_CHROMA_IDX, "The intra dir cannot be DM_CHROMA for this path" );
  CHECK( PU::isLMCMode( chromaCandModes[ candId ] ), "The intra dir cannot be LM_CHROMA for this path" );

  if ( pu.cs->sps->getSpsNext().getUseMDMS() )
  {
    candId -= NUM_LMC_MODE;

    const unsigned lastId = NUM_DM_MODES - 1;
    const bool codeLast = ( lastId > candId );

    unsigned ctxId = 1;
    while ( candId > 0 )
    {
      m_BinEncoder.encodeBin( 1, Ctx::IPredMode[1]( ctxId++ ) );
      candId -= 1;
    }
    if ( codeLast )
    {
      m_BinEncoder.encodeBin( 0, Ctx::IPredMode[1]( ctxId ) );
    }
  }
  else
  {
    m_BinEncoder.encodeBinsEP( candId, 2 );
  }
}


void CABACWriter::cu_residual( const CodingUnit& cu, Partitioner& partitioner, CUCtx& cuCtx )
{
  if( CU::isInter( cu ) )
  {
    PredictionUnit& pu = *cu.firstPU;
    bool testMergeDF = true;

    if( cu.slice->getSPS()->getSpsNext().getDiffusionFilterMode() >> 2 )
    {
      if( !( cu.diffFilterIdx == 0 ) )
      {
        testMergeDF = false;
      }
    }
    if( !( ( cu.cs->pcv->noRQT || cu.partSize == SIZE_2Nx2N ) && pu.mergeFlag && testMergeDF
      && (
          pu.frucMrgMode == FRUC_MERGE_OFF ||
          cu.diffFilterIdx == 0 )
      && pu.mergeRefPicList == REF_PIC_LIST_X
      && pu.numMergedAddHyps == pu.addHypData.size() ) )
    {
      rqt_root_cbf( cu );
    }
    else 
    {
      if (cu.slice->getSPS()->getSpsNext().getDiffusionFilterMode() >> 2)
      {
        CHECK(!cu.rootCbf, "!cu.rootCbf");
      }
    }

    if( !cu.rootCbf )
    {
      return;
    }
  }

  mode_1d_partitions( cu );

  cuCtx.quadtreeTULog2MinSizeInCU = CU::getQuadtreeTULog2MinSizeInCU( cu );
  ChromaCbfs chromaCbfs;
  transform_tree( *cu.cs, partitioner, cuCtx, chromaCbfs );
  residual_nsst_mode( cu, cuCtx );

#if !HM_EMT_NSST_AS_IN_JEM
  cu_emt_noqrt_idx( cu );
#endif
}

#if !HM_EMT_NSST_AS_IN_JEM
void CABACWriter::cu_emt_noqrt_idx( const CodingUnit& cu )
{
  if( !cu.cs->pcv->noRQT || !isLuma( cu.chType ) || cu.nsstIdx != 0 ||
      cu.intraNNTrafoIdx != 0 ||
      !( cu.cs->sps->getSpsNext().getUseIntraEMT() || cu.cs->sps->getSpsNext().getUseInterEMT() ) || !cu.firstTU->cbf[COMPONENT_Y] || cu.firstTU->transformSkip[COMPONENT_Y] )
  {
    return;
  }

  emt_cu_flag( cu );

  if( cu.emtFlag )
  {
    if( CU::isIntra( cu ) )
    {
      if( TU::getNumNonZeroCoeffsNonTS( *cu.firstTU, true, false ) > g_EmtSigNumThr )
      {
        emt_tu_index( *cu.firstTU );
      }
      else
      {
        CHECK( cu.firstTU->emtIdx != 0, "If the number of significant coefficients is <= g_EmtSigNumThr, then the tu index must be 0" );
      }
    }
    else
    {
      emt_tu_index( *cu.firstTU );
    }
  }
}
#endif

void CABACWriter::rqt_root_cbf( const CodingUnit& cu )
{
  m_BinEncoder.encodeBin( cu.rootCbf, Ctx::QtRootCbf() );

  DTRACE( g_trace_ctx, D_SYNTAX, "rqt_root_cbf() ctx=0 root_cbf=%d pos=(%d,%d)\n", cu.rootCbf ? 1 : 0, cu.lumaPos().x, cu.lumaPos().y );
}


void CABACWriter::end_of_ctu( const CodingUnit& cu, CUCtx& cuCtx )
{
  const Slice*  slice             = cu.cs->slice;
  const TileMap& tileMap          = *cu.cs->picture->tileMap;
  const int     currentCTUTsAddr  = tileMap.getCtuRsToTsAddrMap( CU::getCtuAddr( cu ) );
  const bool    isLastSubCUOfCtu  = CU::isLastSubCUOfCtu( cu );

  if ( isLastSubCUOfCtu
    && ( !CS::isDualITree( *cu.cs ) || cu.chromaFormat == CHROMA_400 || isChroma( cu.chType ) )
      )
  {
    cuCtx.isDQPCoded = ( cu.cs->pps->getUseDQP() && !cuCtx.isDQPCoded );

    // The 1-terminating bit is added to all streams, so don't add it here when it's 1.
    // i.e. when the slice segment CurEnd CTU address is the current CTU address+1.
    if( slice->getSliceSegmentCurEndCtuTsAddr() != currentCTUTsAddr + 1 )
    {
      m_BinEncoder.encodeBinTrm( 0 );
    }
  }
}





//================================================================================
//  clause 7.3.8.6
//--------------------------------------------------------------------------------
//    void  prediction_unit ( pu );
//    void  merge_flag      ( pu );
//    void  merge_idx       ( pu );
//    void  inter_pred_idc  ( pu );
//    void  ref_idx         ( pu, refList );
//    void  mvp_flag        ( pu, refList );
//================================================================================

void CABACWriter::prediction_unit( const PredictionUnit& pu ) 
{
#if HHI_SPLIT_PARALLELISM || HHI_WPP_PARALLELISM
  CHECK( pu.cacheUsed, "Processing a PU that should be in cache!" );
  CHECK( pu.cu->cacheUsed, "Processing a CU that should be in cache!" );

#endif
  if( pu.cu->skip )
  {
    CHECK( !pu.mergeFlag, "merge_flag must be true for skipped CUs" );
  }
  else
  {
    merge_flag( pu );
  }
  if( pu.mergeFlag )
  {
    fruc_mrg_mode( pu );
    affine_flag  ( *pu.cu );
    merge_idx    ( pu );
    if( !pu.cu->skip && pu.cu->partSize == SIZE_2Nx2N )
    {
      CHECK( pu.numMergedAddHyps > pu.addHypData.size(), "wrong number of additional hypotheseis in mergemode" );
      CHECK( pu.cu->imv != 0, "Multi Hyp: pu.cu->imv != 0" );
      mh_pred_data( pu );
    }
    else
    {
      CHECK( pu.numMergedAddHyps != pu.addHypData.size(), "wrong number of additional hypotheseis in merge mode" );
    }
  }
  else
  {
    inter_pred_idc( pu );
    affine_flag   ( *pu.cu );
    if( pu.interDir != 2 /* PRED_L1 */ )
    {
      ref_idx     ( pu, REF_PIC_LIST_0 );
      if( pu.cu->affine )
      {
        CMotionBuf mb = pu.getMotionBuf();
        mvd_coding( mb.at(          0, 0 ).mvdAffi[REF_PIC_LIST_0], 0 );
        mvd_coding( mb.at( mb.width-1, 0 ).mvdAffi[REF_PIC_LIST_0], 0 );
      }
      else
      {
        mvd_coding( pu.mvd[REF_PIC_LIST_0], pu.cu->imv );
      }
      mvp_flag    ( pu, REF_PIC_LIST_0 );
    }
    if( pu.interDir != 1 /* PRED_L0 */ )
    {
      ref_idx     ( pu, REF_PIC_LIST_1 );
      if( !pu.cs->slice->getMvdL1ZeroFlag() || pu.interDir != 3 /* PRED_BI */ )
      {
        if( pu.cu->affine )
        {
          CMotionBuf mb = pu.getMotionBuf();
          mvd_coding( mb.at(          0, 0 ).mvdAffi[REF_PIC_LIST_1], 0 );
          mvd_coding( mb.at( mb.width-1, 0 ).mvdAffi[REF_PIC_LIST_1], 0 );
        }
        else
        {
          mvd_coding( pu.mvd[REF_PIC_LIST_1], pu.cu->imv );
        }
      }
      mvp_flag    ( pu, REF_PIC_LIST_1 );
    }
    mh_pred_data( pu );
  }
}

void CABACWriter::affine_flag( const CodingUnit& cu )
{
  if( cu.cs->slice->isIntra() || !cu.cs->sps->getSpsNext().getUseAffine() || cu.partSize != SIZE_2Nx2N || cu.firstPU->frucMrgMode )
  {
    return;
  }

  if( !cu.firstPU->mergeFlag && !( cu.lumaSize().width > 8 && cu.lumaSize().height > 8 ) )
  {
    return;
  }

  if( cu.firstPU->mergeFlag && !PU::isAffineMrgFlagCoded( *cu.firstPU ) )
  {
    return;
  }

  CHECK( !cu.cs->pcv->rectCUs && cu.lumaSize().width != cu.lumaSize().height, "CU width and height are not equal for QTBT off." );

  unsigned ctxId = DeriveCtx::CtxAffineFlag( cu );
  m_BinEncoder.encodeBin( cu.affine, Ctx::AffineFlag( ctxId ) );
  DTRACE( g_trace_ctx, D_COMMON, " (%d) affine_flag() affine=%d\n", DTRACE_GET_COUNTER(g_trace_ctx, D_COMMON), cu.affine ? 1 : 0 );

  DTRACE( g_trace_ctx, D_SYNTAX, "affine_flag() affine=%d ctx=%d pos=(%d,%d)\n", cu.affine ? 1 : 0, ctxId, cu.Y().x, cu.Y().y );
}

void CABACWriter::merge_flag( const PredictionUnit& pu )
{
  m_BinEncoder.encodeBin( pu.mergeFlag, Ctx::MergeFlag() );

  DTRACE( g_trace_ctx, D_SYNTAX, "merge_flag() merge=%d pos=(%d,%d) size=%dx%d\n", pu.mergeFlag ? 1 : 0, pu.lumaPos().x, pu.lumaPos().y, pu.lumaSize().width, pu.lumaSize().height );
}

void CABACWriter::imv_mode( const CodingUnit& cu )
{
  const SPSNext& spsNext = cu.cs->sps->getSpsNext();

  if( !spsNext.getUseIMV() )
  {
    return;
  }

  Bool bNonZeroMvd = CU::hasSubCUNonZeroMVd( cu );
  if( !bNonZeroMvd )
  {
    return;
  }

  unsigned ctxId = DeriveCtx::CtxIMVFlag( cu );
  m_BinEncoder.encodeBin( ( cu.imv > 0 ), Ctx::ImvFlag( ctxId ) );
  DTRACE( g_trace_ctx, D_SYNTAX, "imv_mode() value=%d ctx=%d\n", (cu.imv > 0), ctxId );

  if( spsNext.getImvMode() == IMV_4PEL && cu.imv > 0 )
  {
    m_BinEncoder.encodeBin( ( cu.imv > 1 ), Ctx::ImvFlag( 3 ) );
    DTRACE( g_trace_ctx, D_SYNTAX, "imv_mode() value=%d ctx=%d\n", ( cu.imv > 1 ), 3 );
  }

  DTRACE( g_trace_ctx, D_SYNTAX, "imv_mode() IMVFlag=%d\n", cu.imv );
}

void CABACWriter::merge_idx( const PredictionUnit& pu )
{
  if( pu.frucMrgMode || pu.cu->affine )
  {
    return;
  }

  merge_ref_pic_list( pu );

  int numCandminus1 = int( pu.cs->slice->getMaxNumMergeCand() ) - 1;
  if( numCandminus1 > 0 )
  {
    if( pu.mergeIdx == 0 )
    {
      m_BinEncoder.encodeBin( 0, Ctx::MergeIdx() );
      DTRACE( g_trace_ctx, D_SYNTAX, "merge_idx() merge_idx=%d\n", pu.mergeIdx );
      return;
    }
    else
    {
      bool useExtCtx = pu.cs->sps->getSpsNext().getUseSubPuMvp();
      m_BinEncoder.encodeBin( 1, Ctx::MergeIdx() );
      for( unsigned idx = 1; idx < numCandminus1; idx++ )
      {
        if( useExtCtx )
        {
          m_BinEncoder.encodeBin( pu.mergeIdx == idx ? 0 : 1, Ctx::MergeIdx( std::min<int>( idx, NUM_MERGE_IDX_EXT_CTX - 1 ) ) );
        }
        else
        {
          m_BinEncoder.encodeBinEP( pu.mergeIdx == idx ? 0 : 1 );
        }
        if( pu.mergeIdx == idx )
        {
          break;
        }
      }
    }
  }
  DTRACE( g_trace_ctx, D_SYNTAX, "merge_idx() merge_idx=%d\n", pu.mergeIdx );
}

void CABACWriter::merge_ref_pic_list( const PredictionUnit& pu )
{
  const bool available = pu.cs->slice->getSPS()->getSpsNext().getUseRestrictedMerge();
  if( !available || pu.cu->skip || pu.cu->partSize != SIZE_2Nx2N || !pu.cs->slice->isInterB() )
  {
    CHECK( pu.mergeRefPicList != REF_PIC_LIST_X, "wrong merge list" );
    return;
  }

  const bool restrictedMerge = pu.mergeRefPicList != REF_PIC_LIST_X;
  m_BinEncoder.encodeBin( restrictedMerge ? 1 : 0, Ctx::MergeList() );
  if( restrictedMerge )
  {
    m_BinEncoder.encodeBin( unsigned(pu.mergeRefPicList), Ctx::MergeList(1) );
  }
}

void CABACWriter::inter_pred_idc( const PredictionUnit& pu )
{
  if( !pu.cs->slice->isInterB() )
  {
    return;
  }
  if( pu.cu->partSize == SIZE_2Nx2N || pu.cs->sps->getSpsNext().getUseSubPuMvp() || pu.cu->lumaSize().width != 8 )
  {
    unsigned ctxId = DeriveCtx::CtxInterDir(pu);
    if( pu.interDir == 3 )
    {
      m_BinEncoder.encodeBin( 1, Ctx::InterDir(ctxId) );
      DTRACE( g_trace_ctx, D_SYNTAX, "inter_pred_idc() ctx=%d value=%d pos=(%d,%d)\n", ctxId, pu.interDir, pu.lumaPos().x, pu.lumaPos().y );
      return;
    }
    else
    {
      m_BinEncoder.encodeBin( 0, Ctx::InterDir(ctxId) );
    }
  }
  m_BinEncoder.encodeBin( ( pu.interDir == 2 ), Ctx::InterDir( 4 ) );
  DTRACE( g_trace_ctx, D_SYNTAX, "inter_pred_idc() ctx=4 value=%d pos=(%d,%d)\n", pu.interDir, pu.lumaPos().x, pu.lumaPos().y );
}


void CABACWriter::ref_idx( const PredictionUnit& pu, RefPicList eRefList )
{
  int numRef  = pu.cs->slice->getNumRefIdx(eRefList);
  if( numRef <= 1 )
  {
    return;
  }
  int refIdx  = pu.refIdx[eRefList];
  m_BinEncoder.encodeBin( (refIdx > 0), Ctx::RefPic() );
  if( numRef <= 2 || refIdx == 0 )
  {
    DTRACE( g_trace_ctx, D_SYNTAX, "ref_idx() value=%d pos=(%d,%d)\n", refIdx, pu.lumaPos().x, pu.lumaPos().y );
    return;
  }
  m_BinEncoder.encodeBin( (refIdx > 1), Ctx::RefPic(1) );
  if( numRef <= 3 || refIdx == 1 )
  {
    DTRACE( g_trace_ctx, D_SYNTAX, "ref_idx() value=%d pos=(%d,%d)\n", refIdx, pu.lumaPos().x, pu.lumaPos().y );
    return;
  }
  for( int idx = 3; idx < numRef; idx++ )
  {
    if( refIdx > idx - 1 )
    {
      m_BinEncoder.encodeBinEP( 1 );
      DTRACE( g_trace_ctx, D_SYNTAX, "ref_idx() value=%d ctxId=%d pos=(%d,%d)\n", 1, 0, pu.lumaPos().x, pu.lumaPos().y );
    }
    else
    {
      m_BinEncoder.encodeBinEP( 0 );
      DTRACE( g_trace_ctx, D_SYNTAX, "ref_idx() value=%d ctxId=%d pos=(%d,%d)\n", 0, 0, pu.lumaPos().x, pu.lumaPos().y );
      break;
    }
  }
  DTRACE( g_trace_ctx, D_SYNTAX, "ref_idx() value=%d pos=(%d,%d)\n", refIdx, pu.lumaPos().x, pu.lumaPos().y );
}

void CABACWriter::ref_idx_mh( const int numRef, const int refIdx )
{
  if( numRef <= 1 )
  {
    return;
  }
  m_BinEncoder.encodeBin( (refIdx > 0), Ctx::MHRefPic() );
  if( numRef <= 2 || refIdx == 0 )
  {
    return;
  }
  m_BinEncoder.encodeBin( (refIdx > 1), Ctx::MHRefPic(1) );
  if( numRef <= 3 || refIdx == 1 )
  {
    return;
  }
  for( int idx = 3; idx < numRef; idx++ )
  {
    if( refIdx > idx - 1 )
    {
      m_BinEncoder.encodeBinEP( 1 );
    }
    else
    {
      m_BinEncoder.encodeBinEP( 0 );
      break;
    }
  }
}

void CABACWriter::mvp_flag( const PredictionUnit& pu, RefPicList eRefList )
{
  m_BinEncoder.encodeBin( pu.mvpIdx[eRefList], Ctx::MVPIdx() );
  DTRACE( g_trace_ctx, D_SYNTAX, "mvp_flag() value=%d pos=(%d,%d)\n", pu.mvpIdx[eRefList], pu.lumaPos().x, pu.lumaPos().y );
  DTRACE( g_trace_ctx, D_SYNTAX, "mvpIdx(refList:%d)=%d\n", eRefList, pu.mvpIdx[eRefList] );
}

void CABACWriter::mh_pred_data( const PredictionUnit& pu )
{
  if( !pu.cs->sps->getSpsNext().getUseInterMultiHyp() || !pu.cs->slice->isInterB() )
  {
    CHECK( !pu.addHypData.empty(), "Multi Hyp: !pu.addHypData.empty()");
    return;
  }
  const int numMHRef = int(pu.cs->slice->getMultiHypRefPicList().size());
  CHECK(numMHRef <= 0, "Multi Hyp: numMHRef <= 0");
  const size_t maxNumAddHyps = pu.cs->sps->getSpsNext().getMaxNumAddHyps();
  CHECK(pu.addHypData.size() > maxNumAddHyps, "Multi Hyp: pu.addHypData.size() > maxNumAddHyps");
  int hypIdx = 0;
  for( int i = pu.numMergedAddHyps; i < pu.addHypData.size(); i++ )
  {
    m_BinEncoder.encodeBin( 1, Ctx::MultiHypothesisFlag(hypIdx) );
    if( hypIdx < 1 )
      hypIdx++;
    const MultiHypPredictionData &mhData = pu.addHypData[i];

    CHECK(mhData.refIdx < 0, "Multi Hyp: mhData.refIdx < 0");
    CHECK(mhData.refIdx >= numMHRef, "Multi Hyp: mhData.refIdx >= numMHRef");
    ref_idx_mh( numMHRef, mhData.refIdx );
    mvd_coding( mhData.mvd, pu.cu->imv );
    m_BinEncoder.encodeBin( mhData.mvpIdx, Ctx::MVPIdx() );
    CHECK(mhData.weightIdx < 0, "Multi Hyp: mhData.weightIdx < 0");
    CHECK(mhData.weightIdx >= pu.cs->sps->getSpsNext().getNumAddHypWeights(), "Multi Hyp: mhData.weightIdx >= pu.cs->sps->getSpsNext().getNumAddHypWeights()");
    unary_max_symbol( mhData.weightIdx, Ctx::MHWeight(), Ctx::MHWeight(1), pu.cs->sps->getSpsNext().getNumAddHypWeights() - 1 );
  }

  if( (pu.addHypData.size()-pu.numMergedAddHyps) < maxNumAddHyps )
    m_BinEncoder.encodeBin( 0, Ctx::MultiHypothesisFlag(hypIdx) );
}

void CABACWriter::fruc_mrg_mode( const PredictionUnit& pu )
{
  if( !pu.cs->slice->getSPS()->getSpsNext().getUseFRUCMrgMode() )
    return;

  unsigned uiFirstBin = pu.frucMrgMode != FRUC_MERGE_OFF;
  unsigned flag_idx   = DeriveCtx::CtxFrucFlag( pu );

  m_BinEncoder.encodeBin( uiFirstBin, Ctx::FrucFlag(flag_idx) );

  if( uiFirstBin )
  {
    if( pu.cs->slice->isInterP() )
    {
      CHECK( pu.frucMrgMode != FRUC_MERGE_TEMPLATE, "wrong fruc mode" );
    }
    else
    {
      unsigned uiSecondBin = pu.frucMrgMode == FRUC_MERGE_BILATERALMV;
      unsigned mode_idx    = DeriveCtx::CtxFrucMode( pu );

      m_BinEncoder.encodeBin( uiSecondBin, Ctx::FrucMode(mode_idx) );
    }

    merge_ref_pic_list( pu );
  }

  DTRACE( g_trace_ctx, D_SYNTAX, "fruc_mrg_mode() fruc_mode=%d pos=(%d,%d) size: %dx%d\n", pu.frucMrgMode, pu.Y().x, pu.Y().y, pu.lumaSize().width, pu.lumaSize().height );
}


//================================================================================
//  clause 7.3.8.7
//--------------------------------------------------------------------------------
//    void  pcm_samples( tu )
//================================================================================

void CABACWriter::pcm_samples( const TransformUnit& tu )
{
  CHECK( !tu.cu->ipcm, "pcm mode expected" );

  const SPS&        sps       = *tu.cu->cs->sps;
  const ComponentID maxCompId = ( tu.chromaFormat == CHROMA_400 ? COMPONENT_Y : COMPONENT_Cr );
  for( ComponentID compID = COMPONENT_Y; compID <= maxCompId; compID = ComponentID(compID+1) )
  {
    const CPelBuf   samples     = tu.getPcmbuf( compID );
    const unsigned  sampleBits  = sps.getPCMBitDepth( toChannelType(compID) );
    for( unsigned y = 0; y < samples.height; y++ )
    {
      for( unsigned x = 0; x < samples.width; x++ )
      {
        m_BinEncoder.encodeBinsPCM( samples.at(x, y), sampleBits );
      }
    }
  }
  m_BinEncoder.restart();
}



//================================================================================
//  clause 7.3.8.8
//--------------------------------------------------------------------------------
//    void  transform_tree      ( cs, area, cuCtx, chromaCbfs )
//    bool  split_transform_flag( split, depth )
//    bool  cbf_comp            ( cbf, area, depth )
//================================================================================

void CABACWriter::transform_tree( const CodingStructure& cs, Partitioner& partitioner, CUCtx& cuCtx, ChromaCbfs& chromaCbfs )
{
  const UnitArea&       area          = partitioner.currArea();


  const TransformUnit&  tu            = *cs.getTU( area.blocks[partitioner.chType].pos(), partitioner.chType );
  const CodingUnit&     cu            = *tu.cu;
  const SPS&            sps           = *cs.sps;
  const unsigned        log2TrafoSize = g_aucLog2[area.lumaSize().width];
  const unsigned        trDepth       = partitioner.currTrDepth;
  const bool            split         = ( tu.depth > trDepth );

  // split_transform_flag
  if( cs.pcv->noRQT )
  {
    if( !( cu.mode1dPartitions && ( tu.blocks[partitioner.chType].height == 1 || tu.blocks[partitioner.chType].width == 1 ) ) )
    {
      CHECK( split, "transform split not allowed with QTBT" );
    }
  }
  else if( CU::isIntra(cu) && cu.partSize == SIZE_NxN && trDepth == 0 )
  {
    CHECK( !split, "transform split must be true for Intra_NxN" );
  }
  else if( sps.getQuadtreeTUMaxDepthInter() == 1 && CU::isInter(cu) && cu.partSize != SIZE_2Nx2N && trDepth == 0 )
  {
    if( log2TrafoSize > cuCtx.quadtreeTULog2MinSizeInCU )
    {
      CHECK( !split, "transform split must be true for inferred split (for units greater than the minimum transform size)" );
    }
    else
    {
      CHECK( split,  "transform split must be false for inferred split (for units smaller than or equal to minimum transform size)" );
    }
  }
  else if( log2TrafoSize > sps.getQuadtreeTULog2MaxSize() )
  {
    CHECK( !split, "transform split must be true for units greater than the maximum transform size" );
  }
  else if( log2TrafoSize == sps.getQuadtreeTULog2MinSize() )
  {
    CHECK( split,  "transform split must be false for units equal to the minimum transform size" );
  }
  else if( log2TrafoSize == cuCtx.quadtreeTULog2MinSizeInCU )
  {
    CHECK( split,  "transform split must be false for maximum split depth" );
  }
  else
  {
    CHECK( log2TrafoSize <= cuCtx.quadtreeTULog2MinSizeInCU, "block cannot be split in multiple TUs" );

    if( sps.getSpsNext().nextToolsEnabled() )
    {
      split_transform_flag( split, sps.getQuadtreeTULog2MaxSize() - log2TrafoSize );
    }
    else
    {
      split_transform_flag( split, 5 - log2TrafoSize );
    }
  }

  // cbf_cb & cbf_cr
  if( area.chromaFormat != CHROMA_400 && area.blocks[COMPONENT_Cb].valid() && ( !CS::isDualITree( cs ) || partitioner.chType == CHANNEL_TYPE_CHROMA ) )
  {
    const bool firstCbfOfCU   = ( trDepth == 0 );
    const bool allQuadrants = TU::isProcessingAllQuadrants( area );
    {
      if( firstCbfOfCU || ( allQuadrants && chromaCbfs.Cb ) )
      {
        chromaCbfs.Cb = TU::getCbfAtDepth( tu, COMPONENT_Cb, trDepth );
        cbf_comp( cs, chromaCbfs.Cb, area.blocks[COMPONENT_Cb], trDepth );
      }
      else
      {
        CHECK( TU::getCbfAtDepth( tu, COMPONENT_Cb, trDepth ) != chromaCbfs.Cb, "incorrect Cb cbf" );
      }
      if( firstCbfOfCU || ( allQuadrants && chromaCbfs.Cr ) )
      {
        chromaCbfs.Cr = TU::getCbfAtDepth( tu, COMPONENT_Cr,   trDepth );
        cbf_comp( cs, chromaCbfs.Cr, area.blocks[COMPONENT_Cr], trDepth, false, chromaCbfs.Cb );
      }
      else
      {
        CHECK( TU::getCbfAtDepth( tu, COMPONENT_Cr, trDepth ) != chromaCbfs.Cr, "incorrect Cr cbf" );
      }
    }
  }
  else if( CS::isDualITree( cs ) )
  {
    chromaCbfs = ChromaCbfs( false );
  }

  if( split )
  {
    if( area.chromaFormat != CHROMA_400 )
    {
      chromaCbfs.Cb        = TU::getCbfAtDepth( tu, COMPONENT_Cb,  trDepth );
      chromaCbfs.Cr        = TU::getCbfAtDepth( tu, COMPONENT_Cr,  trDepth );
    }

    if( cu.mode1dPartitions )
    {
      partitioner.splitCurrArea( CU::select1dPartitionType( cu, CS::isDualITree( cs ) ? getFirstComponentOfChannel( partitioner.chType ) : COMPONENT_Y ), cs );
    }
    else
    {
      if( trDepth == 0 ) emt_cu_flag( cu );
      partitioner.splitCurrArea( TU_QUAD_SPLIT, cs );
    }

    do
    {
      ChromaCbfs subChromaCbfs = chromaCbfs;
      transform_tree( cs, partitioner, cuCtx, subChromaCbfs );
    } while( partitioner.nextPart( cs ) );

    partitioner.exitCurrSplit();
  }
  else
  {
    DTRACE( g_trace_ctx, D_SYNTAX, "transform_unit() pos=(%d,%d) depth=%d trDepth=%d\n", tu.lumaPos().x, tu.lumaPos().y, cu.depth, partitioner.currTrDepth );

    if( !isChroma( partitioner.chType ) )
    {
      if( !CU::isIntra( cu ) && trDepth == 0 && !chromaCbfs.sigChroma( area.chromaFormat ) )
      {
        CHECK( !TU::getCbfAtDepth( tu, COMPONENT_Y, trDepth ), "Luma cbf must be true for inter units with no chroma coeffs" );
      }
      else
      {
        bool previousCbf = false;
        if( cu.mode1dPartitions )
        {
          previousCbf = TU::getPrevTuCbfAtDepth( tu, trDepth, COMPONENT_Y );
        }
        cbf_comp( cs, TU::getCbfAtDepth( tu, COMPONENT_Y, trDepth ), tu.Y(), trDepth, cu.mode1dPartitions, previousCbf );
      }
    }

#if HM_EMT_NSST_AS_IN_JEM
    if( trDepth == 0 && TU::getCbfAtDepth( tu, COMPONENT_Y, 0 ) ) emt_cu_flag( cu );
#else
    if( trDepth == 0 && TU::getCbfAtDepth( tu, COMPONENT_Y, 0 ) && !cu.cs->pcv->noRQT )
    {
      emt_cu_flag( cu );
    }
#endif

    transform_unit( tu, cuCtx, chromaCbfs );
  }
}

void CABACWriter::split_transform_flag( bool split, unsigned depth )
{
  m_BinEncoder.encodeBin( split, Ctx::TransSubdivFlag( depth ) );
  DTRACE( g_trace_ctx, D_SYNTAX, "split_transform_flag() ctx=%d split=%d\n", depth, split );
}

void CABACWriter::cbf_comp( const CodingStructure& cs, bool cbf, const CompArea& area, unsigned depth, bool isUsingMode1dPartitions, bool previousCbf )
{
  if( area.compID == COMPONENT_Cr && !isUsingMode1dPartitions && cs.sps->getSpsNext().getAltResiCompId() == 2 )
  {
    m_BinEncoder.encodeBin( cbf, Ctx::QtCbf[2]( previousCbf ? 3 : 2 ) );
    return;
  }

  const unsigned  ctxId   = DeriveCtx::CtxQtCbf(area.compID, depth, isUsingMode1dPartitions && canUse1dPartitions( area.compID ), previousCbf);
  const CtxSet&   ctxSet  = Ctx::QtCbf[toChannelType(area.compID) << (cs.sps->getSpsNext().getAltResiCompId() == 2 ? 1 : 0)];

  if( !canUse1dPartitions( area.compID ) || !isUsingMode1dPartitions )
  {
    m_BinEncoder.encodeBin( cbf, ctxSet( ctxId ) );
    DTRACE( g_trace_ctx, D_SYNTAX, "cbf_comp() etype=%d pos=(%d,%d) ctx=%d cbf=%d\n", area.compID, area.x, area.y, ctxId, cbf );
  }
  else
  {
    m_BinEncoder.encodeBin( cbf, ctxSet( ctxId ) );
    DTRACE( g_trace_ctx, D_SYNTAX, "cbf_comp() etype=%d pos=(%d,%d) ctx=%d cbf=%d\n", area.compID, area.x, area.y, previousCbf, cbf );
  }

}





//================================================================================
//  clause 7.3.8.9
//--------------------------------------------------------------------------------
//    void  mvd_coding( pu, refList )
//================================================================================

void CABACWriter::mvd_coding( const Mv &rMvd, UChar imv )
{
  int       horMvd = rMvd.getHor();
  int       verMvd = rMvd.getVer();
  if( imv )
  {
    CHECK( (horMvd % 4) != 0 && (verMvd % 4) != 0, "IMV: MVD is not a multiple of 4" );
    horMvd >>= 2;
    verMvd >>= 2;
    if( imv == 2 )//IMV_4PEL
    {
      CHECK( (horMvd % 4) != 0 && (verMvd % 4) != 0, "IMV: MVD is not a multiple of 8" );
      horMvd >>= 2;
      verMvd >>= 2;
    }
  }
  unsigned  horAbs  = unsigned( horMvd < 0 ? -horMvd : horMvd );
  unsigned  verAbs  = unsigned( verMvd < 0 ? -verMvd : verMvd );

  if( rMvd.highPrec )
  {
    CHECK( horAbs & ((1<<VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE)-1), "mvd-x has high precision fractional part." );
    CHECK( verAbs & ((1<<VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE)-1), "mvd-y has high precision fractional part." );
    horAbs >>= VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE;
    verAbs >>= VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE;
  }

  // abs_mvd_greater0_flag[ 0 | 1 ]
  m_BinEncoder.encodeBin( (horAbs > 0), Ctx::Mvd() );
  m_BinEncoder.encodeBin( (verAbs > 0), Ctx::Mvd() );

  // abs_mvd_greater1_flag[ 0 | 1 ]
  if( horAbs > 0 )
  {
    m_BinEncoder.encodeBin( (horAbs > 1), Ctx::Mvd(1) );
  }
  if( verAbs > 0 )
  {
    m_BinEncoder.encodeBin( (verAbs > 1), Ctx::Mvd(1) );
  }

  // abs_mvd_minus2[ 0 | 1 ] and mvd_sign_flag[ 0 | 1 ]
  if( horAbs > 0 )
  {
    if( horAbs > 1 )
    {
      exp_golomb_eqprob( horAbs - 2, 1 );
    }
    m_BinEncoder.encodeBinEP( (horMvd < 0) );
  }
  if( verAbs > 0 )
  {
    if( verAbs > 1 )
    {
      exp_golomb_eqprob( verAbs - 2, 1 );
    }
    m_BinEncoder.encodeBinEP( (verMvd < 0) );
  }
}




//================================================================================
//  clause 7.3.8.10
//--------------------------------------------------------------------------------
//    void  transform_unit      ( tu, cuCtx, chromaCbfs )
//    void  cu_qp_delta         ( cu )
//    void  cu_chroma_qp_offset ( cu )
//================================================================================

void CABACWriter::transform_unit( const TransformUnit& tu, CUCtx& cuCtx, ChromaCbfs& chromaCbfs )
{
  CodingUnit& cu        = *tu.cu;
  int         currDepth = tu.depth;
  bool        lumaOnly  = ( cu.chromaFormat == CHROMA_400 || !tu.blocks[COMPONENT_Cb].valid() );
  bool        cbf[3]    = { TU::getCbfAtDepth( tu, COMPONENT_Y,  currDepth ), chromaCbfs.Cb, chromaCbfs.Cr };
  bool        cbfLuma   = ( cbf[ COMPONENT_Y ] != 0 );
  bool        cbfChroma = false;

#if THRESHOLDING
#if THR_ALLOW_ZERO_CBF
  thresholding(tu);
#endif
#endif

  if( cu.chromaFormat != CHROMA_400 )
  {
    if( tu.blocks[COMPONENT_Cb].valid() )
    {
      cbf   [ COMPONENT_Cb  ] = TU::getCbfAtDepth( tu, COMPONENT_Cb,  currDepth );
      cbf   [ COMPONENT_Cr  ] = TU::getCbfAtDepth( tu, COMPONENT_Cr,  currDepth );
    }
    cbfChroma = ( cbf[ COMPONENT_Cb ] || cbf[ COMPONENT_Cr ] );
  }
  if( cbfLuma || cbfChroma )
  {
    if( cu.cs->pps->getUseDQP() && !cuCtx.isDQPCoded )
    {
      cu_qp_delta( cu, cuCtx.qp, cu.qp );
      cuCtx.qp         = cu.qp;
      cuCtx.isDQPCoded = true;
    }
    if( cu.cs->slice->getUseChromaQpAdj() && cbfChroma && !cu.transQuantBypass && !cuCtx.isChromaQpAdjCoded )
    {
      cu_chroma_qp_offset( cu );
      cuCtx.isChromaQpAdjCoded = true;
    }
    if( cbfLuma )
    {
#if THRESHOLDING
#if !THR_ALLOW_ZERO_CBF
      thresholding( tu );
#endif
#endif
      residual_coding( tu, COMPONENT_Y );
    }
    if( !lumaOnly )
    {
      for( ComponentID compID = COMPONENT_Cb; compID <= COMPONENT_Cr; compID = ComponentID( compID + 1 ) )
      {
        if( TU::hasCrossCompPredInfo( tu, compID ) )
        {
          cross_comp_pred( tu, compID );
        }
        if( cbf[ compID ] )
        {
          residual_coding( tu, compID );
        }
      }
    }
  }
}


void CABACWriter::cu_qp_delta( const CodingUnit& cu, int predQP, const SChar qp )
{
  CHECK(!( predQP != std::numeric_limits<int>::max()), "Unspecified error");
  int       DQp         = qp - predQP;
  int       qpBdOffsetY = cu.cs->sps->getQpBDOffset( CHANNEL_TYPE_LUMA );
  DQp                   = ( DQp + 78 + qpBdOffsetY + ( qpBdOffsetY / 2 ) ) % ( 52 + qpBdOffsetY ) - 26 - ( qpBdOffsetY / 2 );
  unsigned  absDQP      = unsigned( DQp < 0 ? -DQp : DQp );
  unsigned  unaryDQP    = std::min<unsigned>( absDQP, CU_DQP_TU_CMAX );

  if (cu.cs->sps->getSpsNext().getUseNextDQP() && cu.cs->sps->getSpsNext().getSkipDQPinOddPOCs() && (cu.cs->picture->getPOC() & 1) && cu.cs->slice->isInterB())
  {
    CHECK (absDQP > 0, "all delta-QP values must be zero in B slices with an odd POC when AltDQPCoding is used");
  }
  else
  unary_max_symbol( unaryDQP, Ctx::DeltaQP(), Ctx::DeltaQP(1), CU_DQP_TU_CMAX );
  if( absDQP >= CU_DQP_TU_CMAX )
  {
    exp_golomb_eqprob( absDQP - CU_DQP_TU_CMAX, CU_DQP_EG_k );
  }
  if( absDQP > 0 )
  {
    m_BinEncoder.encodeBinEP( DQp < 0 );
  }

  DTRACE_COND( ( isEncoding() ), g_trace_ctx, D_DQP, "x=%d, y=%d, d=%d, pred_qp=%d, DQp=%d, qp=%d\n", cu.blocks[cu.chType].lumaPos().x, cu.blocks[cu.chType].lumaPos().y, cu.qtDepth, predQP, DQp, qp );
}


void CABACWriter::cu_chroma_qp_offset( const CodingUnit& cu )
{
  // cu_chroma_qp_offset_flag
  unsigned qpAdj = cu.chromaQpAdj;
  if( qpAdj == 0 )
  {
    m_BinEncoder.encodeBin( 0, Ctx::ChromaQpAdjFlag() );
  }
  else
  {
    m_BinEncoder.encodeBin( 1, Ctx::ChromaQpAdjFlag() );
    int length = cu.cs->pps->getPpsRangeExtension().getChromaQpOffsetListLen();
    if( length > 1 )
    {
      unary_max_symbol( qpAdj-1, Ctx::ChromaQpAdjIdc(), Ctx::ChromaQpAdjIdc(), length-1 );
    }
  }
}





//================================================================================
//  clause 7.3.8.11
//--------------------------------------------------------------------------------
//    void        residual_coding         ( tu, compID )
//    void        transform_skip_flag     ( tu, compID )
//    void        explicit_rdpcm_mode     ( tu, compID )
//    void        last_sig_coeff          ( coeffCtx )
//    void        residual_coding_subblock( coeffCtx )
//================================================================================

void CABACWriter::residual_coding( const TransformUnit& tu, ComponentID compID )
{
  const CodingUnit& cu    = *tu.cu;
  const CompArea&   rRect = tu.blocks[compID];
  DTRACE( g_trace_ctx, D_SYNTAX, "residual_coding() etype=%d pos=(%d,%d) size=%dx%d predMode=%d\n", rRect.compID, rRect.x, rRect.y, rRect.width, rRect.height, cu.predMode );

  // code transform skip and explicit rdpcm mode
  transform_skip_flag( tu, compID );
  explicit_rdpcm_mode( tu, compID );

  // determine sign hiding
  bool signHiding  = ( cu.cs->pps->getSignDataHidingEnabledFlag() && !cu.transQuantBypass && tu.rdpcm[compID] == RDPCM_OFF );
  if(  signHiding && CU::isIntra(cu) && CU::isRDPCMEnabled(cu) && tu.transformSkip[compID] )
  {
    const ChannelType chType    = toChannelType( compID );
    const unsigned    intraMode = PU::getFinalIntraMode( *cu.cs->getPU( rRect.pos(), chType ), chType );
    if( intraMode == HOR_IDX || intraMode == VER_IDX )
    {
      signHiding = false;
    }
  }

  // init coeff coding context
  CoeffCodingContext  cctx    ( tu, compID, signHiding );
  const TCoeff*       coeff   = tu.getCoeffs( compID ).buf;
  unsigned&           GRStats = m_BinEncoder.getCtx().getGRAdaptStats( TU::getGolombRiceStatisticsIndex( tu, compID ) );
  unsigned            numSig  = 0;

  // determine and set last coeff position and sig group flags
  int                      scanPosLast = -1;
  std::bitset<MLS_GRP_NUM> sigGroupFlags;
  for( int scanPos = 0; scanPos < cctx.maxNumCoeff(); scanPos++)
  {
    unsigned blkPos = cctx.blockPos( scanPos );
    if( coeff[blkPos] )
    {
      scanPosLast = scanPos;
      sigGroupFlags.set( scanPos >> cctx.log2CGSize() );
    }
  }
  CHECK( scanPosLast < 0, "Coefficient coding called for empty TU" );
  cctx.setScanPosLast(scanPosLast);

  // code last coeff position
  last_sig_coeff( cctx );

  // code subblocks
  cctx.setGoRiceStats( GRStats );
  bool useEmt = ( cu.cs->sps->getSpsNext().getUseIntraEMT() && cu.predMode == MODE_INTRA ) || ( cu.cs->sps->getSpsNext().getUseInterEMT() && cu.predMode != MODE_INTRA );
  useEmt = useEmt && isLuma(compID);
  useEmt = ( canUse1dPartitions( compID ) && cu.mode1dPartitions ) ? false : useEmt;
#if HM_EMT_NSST_AS_IN_JEM
  if( cu.cs->sps->getSpsNext().getAltResiCompId() == 2 )
  {
    cctx.setIsEMT( useEmt && tu.cu->emtFlag && !tu.transformSkip[compID] );
  }
#endif

  const bool tcqEnabled = tu.cs->sps->getSpsNext().getUseTCQ();
  CHECK( tcqEnabled != tu.tcq[compID], "Expected tcq_flag equal to " << tcqEnabled );

  if( tu.tcq[compID] )
  {
    int state = 0;
    for( int subSetId = ( cctx.scanPosLast() >> cctx.log2CGSize() ); subSetId >= 0; subSetId--)
    {
      cctx.initSubblock           ( subSetId, sigGroupFlags[subSetId] );
      residual_coding_subblock_tcq( cctx, coeff, state );
      if (useEmt)
      {
        numSig += cctx.emtNumSigCoeff();
        cctx.setEmtNumSigCoeff( 0 );
      }
    }
  }
  else
  {
  for( int subSetId = ( cctx.scanPosLast() >> cctx.log2CGSize() ); subSetId >= 0; subSetId--)
  {
    cctx.initSubblock       ( subSetId, sigGroupFlags[subSetId] );
    residual_coding_subblock( cctx, coeff );

    if (useEmt)
    {
      numSig += cctx.emtNumSigCoeff();
      cctx.setEmtNumSigCoeff( 0 );
    }
  }
  }
  GRStats = cctx.currGoRiceStats();

#if !HM_EMT_NSST_AS_IN_JEM
  if( tu.cs->pcv->noRQT )
  {
    return;
  }
#endif

  if( useEmt && !tu.transformSkip[compID] && compID == COMPONENT_Y && tu.cu->emtFlag )
  {
    if( CU::isIntra( *tu.cu ) )
    {
      if( numSig > g_EmtSigNumThr )
      {
        emt_tu_index( tu );
      }
      else
      {
        CHECK( tu.emtIdx != 0, "If the number of significant coefficients is <= g_EmtSigNumThr, then the tu index must be 0" );
      }
    }
    else
    {
      emt_tu_index( tu );
    }
  }
}


void CABACWriter::transform_skip_flag( const TransformUnit& tu, ComponentID compID )
{
  if( canUse1dPartitions( compID ) && tu.cu->mode1dPartitions && ( tu.blocks[compID].width == 1 || tu.blocks[compID].height == 1 ) )
  {
    return;
  }
#if HM_EMT_NSST_AS_IN_JEM
  if( !tu.cu->cs->pps->getUseTransformSkip() || tu.cu->transQuantBypass || !TU::hasTransformSkipFlag( *tu.cs, tu.blocks[compID] ) || ( isLuma( compID ) && tu.cu->emtFlag ) )
#else
  if( !tu.cu->cs->pps->getUseTransformSkip() || tu.cu->transQuantBypass || !TU::hasTransformSkipFlag( *tu.cs, tu.blocks[compID] ) )
#endif
  {
    return;
  }
  m_BinEncoder.encodeBin( tu.transformSkip[compID], Ctx::TransformSkipFlag(toChannelType(compID)) );

  DTRACE( g_trace_ctx, D_SYNTAX, "transform_skip_flag() etype=%d pos=(%d,%d) trSkip=%d\n", compID, tu.blocks[compID].x, tu.blocks[compID].y, (int)tu.transformSkip[compID] );
}

Void CABACWriter::emt_tu_index( const TransformUnit& tu )
{
  int maxSizeEmtIntra, maxSizeEmtInter;
  if( tu.cs->pcv->noRQT )
  {
    maxSizeEmtIntra = EMT_INTRA_MAX_CU_WITH_QTBT;
    maxSizeEmtInter = EMT_INTER_MAX_CU_WITH_QTBT;
  }
  else
  {
    maxSizeEmtIntra = EMT_INTRA_MAX_CU;
    maxSizeEmtInter = EMT_INTER_MAX_CU;
  }
  if( CU::isIntra( *tu.cu ) && ( tu.cu->Y().width <= maxSizeEmtIntra ) && ( tu.cu->Y().height <= maxSizeEmtIntra ) )
  {
    UChar trIdx = tu.emtIdx;
    m_BinEncoder.encodeBin( ( trIdx & 1 ) ? 1 : 0, Ctx::EMTTuIndex( 0 ) );
    m_BinEncoder.encodeBin( ( trIdx / 2 ) ? 1 : 0, Ctx::EMTTuIndex( 1 ) );
    DTRACE( g_trace_ctx, D_SYNTAX, "emt_tu_index() etype=%d pos=(%d,%d) emtTrIdx=%d\n", COMPONENT_Y, tu.blocks[COMPONENT_Y].x, tu.blocks[COMPONENT_Y].y, ( int ) tu.emtIdx );
  }
  if( !CU::isIntra( *tu.cu ) && ( tu.cu->Y().width <= maxSizeEmtInter ) && ( tu.cu->Y().height <= maxSizeEmtInter ) )
  {
    UChar trIdx = tu.emtIdx;
    m_BinEncoder.encodeBin( ( trIdx & 1 ) ? 1 : 0, Ctx::EMTTuIndex( 2 ) );
    m_BinEncoder.encodeBin( ( trIdx / 2 ) ? 1 : 0, Ctx::EMTTuIndex( 3 ) );
    DTRACE( g_trace_ctx, D_SYNTAX, "emt_tu_index() etype=%d pos=(%d,%d) emtTrIdx=%d\n", COMPONENT_Y, tu.blocks[COMPONENT_Y].x, tu.blocks[COMPONENT_Y].y, ( int ) tu.emtIdx );
  }
}

//Void CABACWriter::emt_cu_flag(const CodingUnit& cu, UInt depth, bool codeCuFlag, const int tuWidth,const int tuHeight)
Void CABACWriter::emt_cu_flag( const CodingUnit& cu )
{
  if( cu.mode1dPartitions )
  {
    return;
  }

  const CodingStructure& cs = *cu.cs;

  if( CU::isIntra( cu ) && cs.sps->getSpsNext().getRestrIntraDiffusionMode() && cu.diffFilterIdx )
  {
    return;
  }
  if( cu.cs->sps->getSpsNext().getUseIntraFTM() && cu.firstPU->FTMRegIdx > 0 )
  {
    return;
  }
  if( !( ( cs.sps->getSpsNext().getUseIntraEMT() && CU::isIntra( cu ) ) || ( cs.sps->getSpsNext().getUseInterEMT() && CU::isInter( cu ) ) ) || isChroma( cu.chType ) )
  {
    return;
  }

  unsigned depth          = cu.qtDepth;
  const unsigned cuWidth  = cu.lwidth();
  const unsigned cuHeight = cu.lheight();

  int maxSizeEmtIntra, maxSizeEmtInter;

  if( cu.cs->pcv->noRQT )
  {
    if( depth >= NUM_EMT_CU_FLAG_CTX )
    {
      depth = NUM_EMT_CU_FLAG_CTX - 1;
    }
    maxSizeEmtIntra = EMT_INTRA_MAX_CU_WITH_QTBT;
    maxSizeEmtInter = EMT_INTER_MAX_CU_WITH_QTBT;
  }
  else
  {
    maxSizeEmtIntra = EMT_INTRA_MAX_CU;
    maxSizeEmtInter = EMT_INTER_MAX_CU;
    CHECK( depth >= NUM_EMT_CU_FLAG_CTX, "Depth exceeds limit." );
  }

  const unsigned maxSizeEmt = CU::isIntra( cu ) ? maxSizeEmtIntra : maxSizeEmtInter;

  if( cuWidth <= maxSizeEmt && cuHeight <= maxSizeEmt )
  {
    m_BinEncoder.encodeBin( cu.emtFlag, Ctx::EMTCuFlag( depth ) );
    DTRACE( g_trace_ctx, D_SYNTAX, "emt_cu_flag() etype=%d pos=(%d,%d) emtCuFlag=%d\n", COMPONENT_Y, cu.lx(), cu.ly(), ( int ) cu.emtFlag );
  }
}

void CABACWriter::mode_1d_partitions( const CodingUnit& cu )
{
  if( !cu.cs->sps->getSpsNext().getUseMode1dPartitions() )
  {
    return;
  }


  if( !CU::isIntra( cu ) || cu.pdpc || cu.intra_NN != 0 || cu.diffFilterIdx != 0 || !canUse1dPartitions( cu.chType ) || cu.mrlIdx != 0 )
  {
    CHECK( cu.mode1dPartitions != NO_1D_PARTITION, "error: cu.mode1dPartitions != 0" );
    DTRACE( g_trace_ctx, D_SYNTAX, "mode_1d_partitions() etype=%d pos=(%d,%d) mode1dPartitionsIndex=%d\n", cu.chType, cu.blocks[cu.chType].x, cu.blocks[cu.chType].y, ( int ) cu.mode1dPartitions );
    return;
  }

  if( cu.mode1dPartitions == NO_1D_PARTITION )
  {
    m_BinEncoder.encodeBin( 0, Ctx::Mode1dPartitions( 0 ) );
  }
  else
  {
    m_BinEncoder.encodeBin( 1, Ctx::Mode1dPartitions( 0 ) );

    const PredictionUnit &pu          = *cu.firstPU;
    const UInt uiChFinalMode          = PU::getFinalIntraMode( pu, cu.chType );

    UInt intraModeRegion              = ( uiChFinalMode <= DC_IDX || uiChFinalMode > VDIA_IDX ) ? 0 : uiChFinalMode < DIA_IDX ? 1 : 2;
    intraModeRegion                   = cu.cs->sps->getSpsNext().getUseIntraFTM() && pu.FTMRegIdx != 0 ? 0 : intraModeRegion;
    const UInt mode1dPartitionsValue  = cu.mode1dPartitions - 1;

    m_BinEncoder.encodeBin( mode1dPartitionsValue, Ctx::Mode1dPartitions( 1 + intraModeRegion ) );
  }

  DTRACE( g_trace_ctx, D_SYNTAX, "mode_1d_partitions() etype=%d pos=(%d,%d) mode1dPartitionsIndex=%d\n", cu.chType, cu.blocks[cu.chType].x, cu.blocks[cu.chType].y, ( int ) cu.mode1dPartitions );
}


void CABACWriter::explicit_rdpcm_mode( const TransformUnit& tu, ComponentID compID )
{
  const CodingUnit& cu = *tu.cu;
  if( !CU::isIntra(cu) && CU::isRDPCMEnabled(cu) && ( tu.transformSkip[compID] || cu.transQuantBypass ) )
  {
    ChannelType chType = toChannelType( compID );
    switch( tu.rdpcm[compID] )
    {
    case RDPCM_VER:
      m_BinEncoder.encodeBin( 1, Ctx::RdpcmFlag(chType) );
      m_BinEncoder.encodeBin( 1, Ctx::RdpcmDir (chType) );
      break;
    case RDPCM_HOR:
      m_BinEncoder.encodeBin( 1, Ctx::RdpcmFlag(chType) );
      m_BinEncoder.encodeBin( 0, Ctx::RdpcmDir (chType) );
      break;
    default: // RDPCM_OFF
      m_BinEncoder.encodeBin( 0, Ctx::RdpcmFlag(chType) );
    }
  }
}

void CABACWriter::residual_nsst_mode( const CodingUnit& cu, CUCtx& cuCtx )
{
  if( cu.cs->sps->getSpsNext().getUseIntraFTM() && cu.firstPU->FTMRegIdx > 0 )
  {
    return;
  }

  if( CU::isIntra( cu ) && cu.cs->sps->getSpsNext().getRestrIntraDiffusionMode() && cu.diffFilterIdx )
  {
    return;
  }

  if( cu.mode1dPartitions )
  {
    return;
  }

  if( cu.cs->sps->getSpsNext().getUseSetOfTrafos() && CU::useRedTrafoSet( cu ) )
  {
    return;
  }

  if( cu.mrlIdx )
  {
    CHECK( cu.nsstIdx && cu.mrlIdx, "mrlIdx > 0 and nsstIdx > 0!" );
    CHECK( cu.intraNNTrafoIdx && cu.mrlIdx, "mrlIdx > 0 and intraNNTrafoIdx > 0!" );
    return;
  }

  if( CS::isDualITree( *cu.cs ) && cu.chType == CHANNEL_TYPE_CHROMA && std::min( cu.blocks[1].width, cu.blocks[1].height ) < 4 )
  {
    return;
  }

#if !HM_EMT_NSST_AS_IN_JEM
  CHECK( cu.cs->pcv->noRQT && cu.nsstIdx != 0 && cu.emtFlag, "EMT cannot be applied if NSST is on!" );
  CHECK( cu.cs->pcv->noRQT && cu.intraNNTrafoIdx != 0 && cu.emtFlag, "EMT cannot be applied if intra NN transform is on!" );
#endif

  if( ( cu.cs->sps->getSpsNext().getUseNSST() || cu.cs->sps->getSpsNext().getUseIntraNNTrafos() ) && CU::isIntra( cu ) && !CU::isLosslessCoded( cu ) && !cu.pdpc
    )
  {
    bool nonZeroCoeffNonTs;
    if( cu.cs->pcv->noRQT )
    {
      const int nonZeroCoeffThr = CS::isDualITree( *cu.cs ) ? ( isLuma( cu.chType ) ? NSST_SIG_NZ_LUMA : NSST_SIG_NZ_CHROMA ) : NSST_SIG_NZ_LUMA + NSST_SIG_NZ_CHROMA;
      cuCtx.numNonZeroCoeffNonTs = CU::getNumNonZeroCoeffNonTs( cu );
      nonZeroCoeffNonTs = cuCtx.numNonZeroCoeffNonTs > nonZeroCoeffThr;
    }
    else
    {
      nonZeroCoeffNonTs = CU::hasNonTsCodedBlock( cu );
    }
    if( !nonZeroCoeffNonTs )
    {
      return;
    }
  }
  else
  {
    return;
  }

  if( cu.intra_NN )
  {
    const UInt idxROT = cu.intraNNTrafoIdx;
    CHECK( idxROT > NUM_NSST_TRAFOS_PER_NN_MODE, "Error" );
    CHECK( NUM_NSST_TRAFOS_PER_NN_MODE > 4, "Error" );

    m_BinEncoder.encodeBin( idxROT ? 1 : 0, Ctx::NSST_For_NN_Ctxts( 0 ) );
    if( idxROT )
    {
      CHECK( cu.lwidth() <= 8 && cu.lheight() <= 8 && cu.emtFlag, "Error" );
      if( NUM_NSST_TRAFOS_PER_NN_MODE == 2 )
      {
        m_BinEncoder.encodeBin( ( idxROT > 1 ) ? 1 : 0, Ctx::NSST_For_NN_Ctxts( 1 ) );
      }
      if( NUM_NSST_TRAFOS_PER_NN_MODE == 3 )
      {
        m_BinEncoder.encodeBin( ( idxROT > 1 ) ? 1 : 0, Ctx::NSST_For_NN_Ctxts( 1 ) );
        if( idxROT > 1 )
        {
          m_BinEncoder.encodeBin( ( idxROT > 2 ) ? 1 : 0, Ctx::NSST_For_NN_Ctxts( 2 ) ); // 2 or 3
        }
      }
      if( NUM_NSST_TRAFOS_PER_NN_MODE == 4 )
      {
        m_BinEncoder.encodeBin( ( idxROT > 2 ) ? 1 : 0, Ctx::NSST_For_NN_Ctxts( 1 ) ); // 1, 2 or 3, 4
        if( idxROT > 2 )
        {
          m_BinEncoder.encodeBin( ( idxROT > 3 ) ? 1 : 0, Ctx::NSST_For_NN_Ctxts( 2 ) ); // 3 or 4
        }
        else
        {
          m_BinEncoder.encodeBin( ( idxROT > 1 ) ? 1 : 0, Ctx::NSST_For_NN_Ctxts( 3 ) ); // 1 or 2
        }
      }
    }
    return;
  }
  CHECK( cu.nsstIdx >= 4, "Error" );

  Bool bUseThreeNSSTPasses = false;

  if( cu.partSize == SIZE_2Nx2N )
  {
    int intraMode = cu.firstPU->intraDir[cu.chType];
    if( intraMode == DM_CHROMA_IDX )
    {
      intraMode = CS::isDualITree( *cu.cs ) ? cu.cs->picture->cs->getPU( cu.blocks[cu.chType].lumaPos(), CHANNEL_TYPE_LUMA )->intraDir[0] : cu.firstPU->intraDir[0];
    }
    else if( PU::isLMCMode( intraMode ) )
    {
      intraMode = PLANAR_IDX;
    }

    bUseThreeNSSTPasses = ( intraMode <= DC_IDX );
  }

  if( bUseThreeNSSTPasses )
  {
    const UInt idxROT = cu.nsstIdx;
    assert( idxROT < 3 );
    m_BinEncoder.encodeBin( idxROT ? 1 : 0, Ctx::NSSTIdx( 1 ) );
    if( idxROT )
    {
      m_BinEncoder.encodeBin( ( idxROT - 1 ) ? 1 : 0, Ctx::NSSTIdx( 3 ) );
    }
  }
  else
  {
    const UInt idxROT = cu.nsstIdx;
    assert( idxROT < 4 );
    m_BinEncoder.encodeBin( idxROT ? 1 : 0, Ctx::NSSTIdx( 0 ) );
    if( idxROT )
    {
      m_BinEncoder.encodeBin( ( idxROT - 1 ) ? 1 : 0, Ctx::NSSTIdx( 2 ) );
      if( idxROT > 1 )
      {
        m_BinEncoder.encodeBin( ( idxROT - 2 ) ? 1 : 0, Ctx::NSSTIdx( 4 ) );
      }
    }
  }

  DTRACE( g_trace_ctx, D_SYNTAX, "residual_nsst_mode() etype=%d pos=(%d,%d) mode=%d\n", COMPONENT_Y, cu.lx(), cu.ly(), ( int ) cu.nsstIdx );
}

void CABACWriter::last_sig_coeff( CoeffCodingContext& cctx )
{
  unsigned blkPos = cctx.blockPos( cctx.scanPosLast() );
  unsigned posX, posY;
  bool codePosX = true, codePosY = true;
  if( ( cctx.width() == 1 ) )// && cctx.scanType() != SCAN_VER) || (cctx.height() == 1 && cctx.scanType() == SCAN_VER))
  {
    posX = 0;
    posY = blkPos;// -(posX << cctx.log2BlockWidth());
    codePosX = false;
  }
  else if( ( cctx.height() == 1 ) )// && cctx.scanType() != SCAN_VER) || (cctx.width() == 1 && cctx.scanType() == SCAN_VER))
  {
    posY = 0;
    posX = blkPos;// >> cctx.log2BlockWidth();
    codePosY = false;
  }
  else
  {
    if( cctx.scanType() == SCAN_VER )
    {
      posX = blkPos / cctx.width();
      posY = blkPos - ( posX * cctx.width() );
    }
    else
    {
      posY = blkPos / cctx.width();
      posX = blkPos - ( posY * cctx.width() );
    }
  }

  unsigned CtxLast;
  unsigned GroupIdxX = g_uiGroupIdx[ posX ];
  unsigned GroupIdxY = g_uiGroupIdx[ posY ];

  if (codePosX)
  {
  for( CtxLast = 0; CtxLast < GroupIdxX; CtxLast++ )
  {
    m_BinEncoder.encodeBin( 1, cctx.lastXCtxId( CtxLast ) );
  }
  if( GroupIdxX < cctx.maxLastPosX() )
  {
    m_BinEncoder.encodeBin( 0, cctx.lastXCtxId( CtxLast ) );
  }
  }
  if (codePosY)
  {
  for( CtxLast = 0; CtxLast < GroupIdxY; CtxLast++ )
  {
    m_BinEncoder.encodeBin( 1, cctx.lastYCtxId( CtxLast ) );
  }
  if( GroupIdxY < cctx.maxLastPosY() )
  {
    m_BinEncoder.encodeBin( 0, cctx.lastYCtxId( CtxLast ) );
  }
  }
  if (codePosX)
  {
  if( GroupIdxX > 3 )
  {
    posX -= g_uiMinInGroup[ GroupIdxX ];
    for (Int i = ( ( GroupIdxX - 2 ) >> 1 ) - 1 ; i >= 0; i-- )
    {
      m_BinEncoder.encodeBinEP( ( posX >> i ) & 1 );
    }
  }
  }
  if (codePosY)
  {
  if( GroupIdxY > 3 )
  {
    posY -= g_uiMinInGroup[ GroupIdxY ];
    for ( Int i = ( ( GroupIdxY - 2 ) >> 1 ) - 1 ; i >= 0; i-- )
    {
      m_BinEncoder.encodeBinEP( ( posY >> i ) & 1 );
    }
  }
  }
}



static const int tab_nextState[ 4 ][ 2 ]  = { {0,2}, {2,0}, {1,3}, {3,1} };
static const int tab_nextQSet [ 4 ]       = {  0,     0,     1,     1    };

void CABACWriter::residual_coding_subblock_tcq( CoeffCodingContext& cctx, const TCoeff* coeff, int& state )
{
  //===== init =====
  const int   minSubPos   = cctx.minSubPos();
  const bool  isLast      = cctx.isLast();
  int         nextSigPos  = ( isLast ? cctx.scanPosLast() : cctx.maxSubPos() );

  //===== encode significant_coeffgroup_flag =====
  if( !isLast && cctx.isNotFirst() )
  {
    if( cctx.isSigGroup() )
    {
      m_BinEncoder.encodeBin( 1, cctx.sigGroupCtxId() );
    }
    else
    {
      m_BinEncoder.encodeBin( 0, cctx.sigGroupCtxId() );
      return;
    }
  }

  //===== encode significant_coeff_flag's =====
  const int inferSigPos   = nextSigPos != cctx.scanPosLast() ? ( cctx.isNotFirst() ? minSubPos : -1 ) : nextSigPos;
  int       remAbsLevel   = -1;
  int       numNonZero    =  0;
  unsigned  signPattern   =  0;

  for( ; nextSigPos >= minSubPos; nextSigPos-- )
  {
    const TCoeff  Coeff       = coeff[ cctx.blockPos( nextSigPos ) ];
    unsigned      sigFlag     = ( Coeff != 0 );
    bool          escapeData  = false;
    if( numNonZero || nextSigPos != inferSigPos )
    {
      m_BinEncoder.encodeBin( sigFlag, cctx.sigCtxIdAbsTcq( state, nextSigPos, coeff ) );
      DTRACE( g_trace_ctx, D_SYNTAX_RESI, "sig_bin() bin=%d ctx=%d\n", sigFlag, cctx.blockPos( nextSigPos ) );
    }
    if( sigFlag )
    {
      const unsigned gt1CtxId = cctx.greater1CtxIdTcq(state);
      numNonZero++;
      remAbsLevel = abs( Coeff ) - 1;

      if( nextSigPos != cctx.scanPosLast() ) signPattern <<= 1;
      if( Coeff < 0 )                        signPattern++;

      if( remAbsLevel > 0 )
      {
        m_BinEncoder.encodeBin( 1, gt1CtxId );
        DTRACE( g_trace_ctx, D_SYNTAX_RESI, "gt1_flag() bin=%d ctx=%d\n", 1, gt1CtxId );

        remAbsLevel -= 1;

        const unsigned gtxCtxId = cctx.greaterXCtxIdTcq();

        if( remAbsLevel > 0 )
        {
          m_BinEncoder.encodeBin( 1, gtxCtxId );
          DTRACE( g_trace_ctx, D_SYNTAX_RESI, "gt2_flag() bin=%d ctx=%d\n", 1, gtxCtxId );

          remAbsLevel -= 1;

          if( remAbsLevel > 0 )
          {
            m_BinEncoder.encodeBin( 1, gtxCtxId );
            DTRACE( g_trace_ctx, D_SYNTAX_RESI, "gt3_flag() bin=%d ctx=%d\n", 1, gtxCtxId );

            remAbsLevel -= 1;

            if( remAbsLevel > 0 )
            {
              m_BinEncoder.encodeBin( 1, gtxCtxId );
              DTRACE( g_trace_ctx, D_SYNTAX_RESI, "gt4_flag() bin=%d ctx=%d\n", 1, gtxCtxId );

              remAbsLevel -= 1;

              escapeData = true;
            }
            else
            {
              m_BinEncoder.encodeBin( 0, gtxCtxId );
              DTRACE( g_trace_ctx, D_SYNTAX_RESI, "gt4_flag() bin=%d ctx=%d\n", 0, gtxCtxId );
            }

          }
          else
          {
            m_BinEncoder.encodeBin( 0, gtxCtxId );
            DTRACE( g_trace_ctx, D_SYNTAX_RESI, "gt3_flag() bin=%d ctx=%d\n", 0, gtxCtxId );
          }
        }
        else
        {
          m_BinEncoder.encodeBin( 0, gtxCtxId );
          DTRACE( g_trace_ctx, D_SYNTAX_RESI, "gt2_flag() bin=%d ctx=%d\n", 0, gtxCtxId );
        }

      }
      else
      {
        m_BinEncoder.encodeBin( 0, gt1CtxId );
        DTRACE( g_trace_ctx, D_SYNTAX_RESI, "gt1_flag() bin=%d ctx=%d\n", 0, gt1CtxId );
      }

      if( escapeData )
      {
        m_BinEncoder.encodeRemAbsEP( remAbsLevel, cctx.GoRicePar(), cctx.extPrec(), cctx.maxLog2TrDRange(), true );
        DTRACE( g_trace_ctx, D_SYNTAX_RESI, "rem_val() bin=%d ctx=%d\n", remAbsLevel, cctx.GoRicePar() );
      }
    }

    state = tab_nextState[ state ][ Coeff & 1 ];
  }

  //===== encode sign's =====
  m_BinEncoder.encodeBinsEP( signPattern, numNonZero );
  cctx.setEmtNumSigCoeff(numNonZero);
}


void CABACWriter::residual_coding_subblock( CoeffCodingContext& cctx, const TCoeff* coeff )
{
  //===== init =====
  const int   maxSbbSize  = 1 << cctx.log2CGSize();
  const int   minSubPos   = cctx.minSubPos();
  const bool  isLast      = cctx.isLast();
  int         nextSigPos  = ( isLast ? cctx.scanPosLast() : cctx.maxSubPos() );

  //===== encode significant_coeffgroup_flag =====
  if( !isLast && cctx.isNotFirst() )
  {
    if( cctx.isSigGroup() )
    {
      m_BinEncoder.encodeBin( 1, cctx.sigGroupCtxId() );
    }
    else
    {
      m_BinEncoder.encodeBin( 0, cctx.sigGroupCtxId() );
      return;
    }
  }

  if( cctx.altResiCompId() == 2 )
  {
    //===== encode significant_coeff_flag's =====
    const int inferSigPos   = nextSigPos != cctx.scanPosLast() ? ( cctx.isNotFirst() ? minSubPos : -1 ) : nextSigPos;
    int       firstNZPos    = nextSigPos;
    int       lastNZPos     = -1;
    int       remAbsLevel   = -1;
    int       numGtXFlags   =  0;
    int       currLevelFlag =  0;
    int       numNonZero    =  0;
    unsigned  signPattern   =  0;

    for( ; nextSigPos >= minSubPos; nextSigPos-- )
    {
      TCoeff    Coeff      = coeff[ cctx.blockPos( nextSigPos ) ];
      unsigned  sigFlag    = ( Coeff != 0 );
      bool      escapeData = false;
      if( numNonZero || nextSigPos != inferSigPos )
      {
        m_BinEncoder.encodeBin( sigFlag, cctx.sigCtxIdAbs( nextSigPos, coeff ) );
        DTRACE( g_trace_ctx, D_SYNTAX_RESI, "sig_bin() bin=%d ctx=%d\n", sigFlag, cctx.blockPos( nextSigPos ) );
      }
      if( sigFlag )
      {
        numNonZero++;
        currLevelFlag++;
        firstNZPos  = nextSigPos;
        lastNZPos   = std::max<int>( lastNZPos, nextSigPos );
        remAbsLevel = abs( Coeff ) - 1;

        if( nextSigPos != cctx.scanPosLast() ) signPattern <<= 1;
        if( Coeff < 0 )                        signPattern++;

        if( numGtXFlags < CXFLAG_NUMBER )
        {
          numGtXFlags++;

          if( remAbsLevel > 0 )
          {
            m_BinEncoder.encodeBin( 1, cctx.greaterXCtxId( currLevelFlag ) );
            DTRACE( g_trace_ctx, D_SYNTAX_RESI, "gt1_flag() bin=%d ctx=%d\n", 1, cctx.greaterXCtxId( currLevelFlag ) );

            remAbsLevel -= 1;
            currLevelFlag++;

            if( remAbsLevel > 0 )
            {
              m_BinEncoder.encodeBin( 1, cctx.greaterXCtxId( currLevelFlag ) );
              DTRACE( g_trace_ctx, D_SYNTAX_RESI, "gt2_flag() bin=%d ctx=%d\n", 1, cctx.greaterXCtxId( currLevelFlag ) );

              remAbsLevel -= 1;
              currLevelFlag++;

              if( remAbsLevel > 0 )
              {
                m_BinEncoder.encodeBin( 1, cctx.greaterXCtxId( currLevelFlag ) );
                DTRACE( g_trace_ctx, D_SYNTAX_RESI, "gt3_flag() bin=%d ctx=%d\n", 1, cctx.greaterXCtxId( currLevelFlag ) );

                remAbsLevel -= 1;
                currLevelFlag++;

                if( remAbsLevel > 0 )
                {
                  m_BinEncoder.encodeBin( 1, cctx.greaterXCtxId( currLevelFlag ) );
                  DTRACE( g_trace_ctx, D_SYNTAX_RESI, "gt4_flag() bin=%d ctx=%d\n", 1, cctx.greaterXCtxId( currLevelFlag ) );

                  remAbsLevel -= 1;
                  currLevelFlag++;

                  escapeData = true;
                }
                else
                {
                  m_BinEncoder.encodeBin( 0, cctx.greaterXCtxId( currLevelFlag ) );
                  DTRACE( g_trace_ctx, D_SYNTAX_RESI, "gt4_flag() bin=%d ctx=%d\n", 0, cctx.greaterXCtxId( currLevelFlag ) );
                }

              }
              else
              {
                m_BinEncoder.encodeBin( 0, cctx.greaterXCtxId( currLevelFlag ) );
                DTRACE( g_trace_ctx, D_SYNTAX_RESI, "gt3_flag() bin=%d ctx=%d\n", 0, cctx.greaterXCtxId( currLevelFlag ) );
              }

            }
            else
            {
              m_BinEncoder.encodeBin( 0, cctx.greaterXCtxId( currLevelFlag ) );
              DTRACE( g_trace_ctx, D_SYNTAX_RESI, "gt2_flag() bin=%d ctx=%d\n", 0, cctx.greaterXCtxId( currLevelFlag ) );
            }

          }
          else
          {
            m_BinEncoder.encodeBin( 0, cctx.greaterXCtxId( currLevelFlag ) );
            DTRACE( g_trace_ctx, D_SYNTAX_RESI, "gt1_flag() bin=%d ctx=%d\n", 0, cctx.greaterXCtxId( currLevelFlag ) );
          }
        }
        else
        {
          escapeData = true;
        }

        if( escapeData )
        {
          m_BinEncoder.encodeRemAbsEP( remAbsLevel, cctx.GoRicePar(), cctx.extPrec(), cctx.maxLog2TrDRange(), true );
          DTRACE( g_trace_ctx, D_SYNTAX_RESI, "rem_val() bin=%d ctx=%d\n", remAbsLevel, cctx.GoRicePar() );
        }
      }
    }

    //===== decode sign's =====
    unsigned numSigns = numNonZero;
    if( cctx.hideSign( firstNZPos, lastNZPos ) )
    {
      numSigns    --;
      signPattern >>= 1;
    }
    m_BinEncoder.encodeBinsEP( signPattern, numSigns );
    cctx.setEmtNumSigCoeff(numNonZero);
  }
  else
  if( cctx.altResiCompId() == 1 )
  {
    //===== encode significant_coeff_flag's =====
    const int inferSigPos = ( cctx.isNotFirst() ? minSubPos : -1 );
    unsigned  numNonZero = 0;
    int       firstNZPos = maxSbbSize;
    int       lastNZPos = -1;
    int       pos     [ 1 << MLS_CG_SIZE ];
    int       absCoeff[ 1 << MLS_CG_SIZE ];
    unsigned  signPattern = 0;

    if( isLast )
    {
      firstNZPos = nextSigPos;
      lastNZPos = std::max<int>( lastNZPos, nextSigPos );
      pos   [ numNonZero ] = nextSigPos;
      TCoeff    Coeff = coeff[ cctx.blockPos( nextSigPos-- ) ];
      absCoeff[ numNonZero++ ] = ( Coeff > 0 ? Coeff : ( signPattern++, -Coeff ) );
    }
    for( ; nextSigPos >= minSubPos; nextSigPos-- )
    {
      TCoeff    Coeff = coeff[ cctx.blockPos( nextSigPos ) ];
      unsigned  sigFlag = ( Coeff != 0 );
      if( numNonZero || nextSigPos != inferSigPos )
      {
        m_BinEncoder.encodeBin( sigFlag, cctx.sigCtxId( nextSigPos, coeff ) );
      }
      if( sigFlag )
      {
        firstNZPos = nextSigPos;
        lastNZPos = std::max<int>( lastNZPos, nextSigPos );
        signPattern <<= 1;
        pos   [ numNonZero ] = nextSigPos;
        absCoeff[ numNonZero++ ] = ( Coeff > 0 ? Coeff : ( signPattern++, -Coeff ) );
      }
    }


    //===== decode abs_greater1_flag's =====
    const unsigned  numGt1Flags = std::min<unsigned>( numNonZero, C1FLAG_NUMBER );
    int             gt2FlagIdx = maxSbbSize;
    bool            escapeData = false;
    unsigned        ctxG1      = cctx.greater1CtxId( 0 );
    for( unsigned k = 0; k < numGt1Flags; k++ )
    {
      if( k || !cctx.isLast() )
      {
        ctxG1 = cctx.greater1CtxId( pos[ k ], coeff );
      }

      if( absCoeff[ k ] > 1 )
      {
        m_BinEncoder.encodeBin( 1, ctxG1 );
        if( gt2FlagIdx < maxSbbSize )
        {
          escapeData = true;
        }
        else
        {
          gt2FlagIdx = k;
        }
      }
      else
      {
        m_BinEncoder.encodeBin( 0, ctxG1 );
      }
    }
    escapeData = escapeData || ( numGt1Flags < numNonZero );


    //===== decode abs_greater2_flag =====
    if( gt2FlagIdx < maxSbbSize )
    {
      unsigned ctxG2 = cctx.greater1CtxId( 0 );
      if( gt2FlagIdx || !cctx.isLast() )
      {
        ctxG2 = cctx.greater2CtxId( pos[ gt2FlagIdx ], coeff );
      }

      if( absCoeff[ gt2FlagIdx ] > 2 )
      {
        m_BinEncoder.encodeBin( 1, ctxG2 );
        escapeData = true;
      }
      else
      {
        m_BinEncoder.encodeBin( 0, ctxG2 );
      }
    }


    //===== align data =====
    if( escapeData && cctx.alignFlag() )
    {
      m_BinEncoder.align();
    }


    //===== decode remaining absolute values =====
    if( escapeData )
    {
      bool      updateGoRiceStats = cctx.updGoRiceStats();
      unsigned  GoRicePar = cctx.currGoRiceStats() >> 2;
      unsigned  MaxGoRicePar = ( updateGoRiceStats ? std::numeric_limits<unsigned>::max() : 4 );
      int       baseLevel = 3;
      for( int k = 0; k < numNonZero; k++ )
      {
        if( absCoeff[ k ] >= baseLevel )
        {
          int remAbs = absCoeff[ k ] - baseLevel;
          if( !updateGoRiceStats || !cctx.isLast() )
          {
            GoRicePar = cctx.GoRicePar( pos[ k ], coeff );
          }
          m_BinEncoder.encodeRemAbsEP( remAbs, GoRicePar, cctx.extPrec(), cctx.maxLog2TrDRange(), true );

          // update rice parameter
          if( absCoeff[ k ] >( 3 << GoRicePar ) )
          {
            GoRicePar = std::min<unsigned>( MaxGoRicePar, GoRicePar + 1 );
          }
          if( updateGoRiceStats )
          {
            unsigned initGoRicePar = cctx.currGoRiceStats() >> 2;
            if( remAbs >= ( 3 << initGoRicePar ) )
            {
              cctx.incGoRiceStats();
            }
            else if( cctx.currGoRiceStats() > 0 && ( remAbs << 1 ) < ( 1 << initGoRicePar ) )
            {
              cctx.decGoRiceStats();
            }
            updateGoRiceStats = false;
          }
        }
        if( k > C1FLAG_NUMBER - 2 )
        {
          baseLevel = 1;
        }
        else if( baseLevel == 3 && absCoeff[ k ] > 1 )
        {
          baseLevel = 2;
        }
      }
    }

    //===== decode sign's =====
    unsigned numSigns = numNonZero;
    if( cctx.hideSign( firstNZPos, lastNZPos ) )
    {
      numSigns--;
      signPattern >>= 1;
    }
    m_BinEncoder.encodeBinsEP( signPattern, numSigns );
    cctx.setEmtNumSigCoeff(numNonZero);
  }
  else
  {
    //===== encode significant_coeff_flag's =====
    const int inferSigPos = ( cctx.isNotFirst() ? minSubPos : -1 );
    unsigned  numNonZero  = 0;
    int       firstNZPos  = maxSbbSize;
    int       lastNZPos   = -1;
    int       absCoeff    [ 1 << MLS_CG_SIZE ];
    unsigned  signPattern = 0;
    if( isLast )
    {
      firstNZPos                    = nextSigPos;
      lastNZPos                     = std::max<int>( lastNZPos, nextSigPos );
      TCoeff    Coeff               = coeff[ cctx.blockPos( nextSigPos-- ) ];
      absCoeff[ numNonZero++ ]      = ( Coeff > 0 ? Coeff : ( signPattern++, -Coeff ) );
    }
    for( ; nextSigPos >= minSubPos; nextSigPos-- )
    {
      TCoeff    Coeff               = coeff[ cctx.blockPos( nextSigPos ) ];
      unsigned  sigFlag             = ( Coeff != 0 );
      if( numNonZero || nextSigPos != inferSigPos )
      {
        m_BinEncoder.encodeBin( sigFlag, cctx.sigCtxId( nextSigPos ) );
      }
      if( sigFlag )
      {
        firstNZPos                = nextSigPos;
        lastNZPos                 = std::max<int>( lastNZPos, nextSigPos );
        signPattern             <<= 1;
        absCoeff[ numNonZero++ ]  = ( Coeff > 0 ? Coeff : ( signPattern++, -Coeff ) );
      }
    }


    //===== decode abs_greater1_flag's =====
    const unsigned  numGt1Flags = std::min<unsigned>( numNonZero, C1FLAG_NUMBER );
    int             gt2FlagIdx  = maxSbbSize;
    bool            escapeData  = false;
    uint16_t        ctxGt1Id    = 1;
    for( unsigned k = 0; k < numGt1Flags; k++ )
    {
      if( absCoeff[ k ] > 1 )
      {
        m_BinEncoder.encodeBin( 1, cctx.greater1CtxId( ctxGt1Id ) );
        ctxGt1Id      = 0;
        if( gt2FlagIdx < maxSbbSize )
        {
          escapeData  = true;
        }
        else
        {
          gt2FlagIdx  = k;
        }
      }
      else
      {
        m_BinEncoder.encodeBin( 0, cctx.greater1CtxId( ctxGt1Id ) );
        if( ctxGt1Id && ctxGt1Id < 3 )
        {
          ctxGt1Id++;
        }
      }
    }
    escapeData = escapeData || ( numGt1Flags < numNonZero );
    cctx.setGt2Flag( ctxGt1Id == 0 );


    //===== decode abs_greater2_flag =====
    if( gt2FlagIdx < maxSbbSize )
    {
      if( absCoeff[ gt2FlagIdx ] > 2 )
      {
        m_BinEncoder.encodeBin( 1, cctx.greater2CtxId() );
        escapeData  = true;
      }
      else
      {
        m_BinEncoder.encodeBin( 0, cctx.greater2CtxId() );
      }
    }


    //===== align data =====
    if( escapeData && cctx.alignFlag() )
    {
      m_BinEncoder.align();
    }


    //===== decode sign's =====
    unsigned numSigns = numNonZero;
    if( cctx.hideSign( firstNZPos, lastNZPos ) )
    {
      numSigns    --;
      signPattern >>= 1;
    }
    m_BinEncoder.encodeBinsEP( signPattern, numSigns );


    //===== decode remaining absolute values =====
    if( escapeData )
    {
      bool      updateGoRiceStats = cctx.updGoRiceStats();
      unsigned  GoRicePar         = cctx.currGoRiceStats() >> 2;
      unsigned  MaxGoRicePar      = ( updateGoRiceStats ? std::numeric_limits<unsigned>::max() : 4 );
      int       baseLevel         = 3;
      for( int k = 0; k < numNonZero; k++ )
      {
        if( absCoeff[ k ] >= baseLevel )
        {
          int remAbs    = absCoeff[ k ] - baseLevel;
          m_BinEncoder.encodeRemAbsEP( remAbs, GoRicePar, cctx.extPrec(), cctx.maxLog2TrDRange() );

          // update rice parameter
          if( absCoeff[ k ] > ( 3 << GoRicePar ) )
          {
            GoRicePar = std::min<unsigned>( MaxGoRicePar, GoRicePar + 1 );
          }
          if( updateGoRiceStats )
          {
            unsigned initGoRicePar = cctx.currGoRiceStats() >> 2;
            if( remAbs >= ( 3 << initGoRicePar) )
            {
              cctx.incGoRiceStats();
            }
            else if( cctx.currGoRiceStats() > 0 && ( remAbs << 1 ) < ( 1 << initGoRicePar ) )
            {
              cctx.decGoRiceStats();
            }
            updateGoRiceStats = false;
          }
        }
        if( k > C1FLAG_NUMBER - 2 )
        {
          baseLevel = 1;
        }
        else if( baseLevel == 3 && absCoeff[ k ] > 1 )
        {
          baseLevel = 2;
        }
      }
    }

    cctx.setEmtNumSigCoeff(numNonZero);
  }
}





//================================================================================
//  clause 7.3.8.12
//--------------------------------------------------------------------------------
//    void  cross_comp_pred( tu, compID )
//================================================================================

void CABACWriter::cross_comp_pred( const TransformUnit& tu, ComponentID compID )
{
  CHECK(!( !isLuma( compID ) ), "Unspecified error");
  signed char alpha   = tu.compAlpha[compID];
  unsigned    ctxBase = ( compID == COMPONENT_Cr ? 5 : 0 );
  if( alpha == 0 )
  {
    m_BinEncoder.encodeBin( 0, Ctx::CrossCompPred( ctxBase ) );
    DTRACE( g_trace_ctx, D_SYNTAX, "cross_comp_pred() etype=%d pos=(%d,%d) alpha=%d\n", compID, tu.blocks[compID].x, tu.blocks[compID].y, tu.compAlpha[compID] );
    return;
  }

  static const unsigned log2AbsAlphaMinus1Table[8] = { 0, 1, 1, 2, 2, 2, 3, 3 };
  unsigned sign = ( alpha < 0 );
  if( sign )
  {
    alpha = -alpha;
  }
  CHECK(!( alpha <= 8 ), "Unspecified error");
  m_BinEncoder.encodeBin( 1, Ctx::CrossCompPred(ctxBase) );
  if( alpha > 1)
  {
     m_BinEncoder.encodeBin( 1, Ctx::CrossCompPred(ctxBase+1) );
     unary_max_symbol( log2AbsAlphaMinus1Table[alpha-1]-1, Ctx::CrossCompPred(ctxBase+2), Ctx::CrossCompPred(ctxBase+3), 2 );
  }
  else
  {
     m_BinEncoder.encodeBin( 0, Ctx::CrossCompPred(ctxBase+1) );
  }
  m_BinEncoder.encodeBin( sign, Ctx::CrossCompPred(ctxBase+4) );

  DTRACE( g_trace_ctx, D_SYNTAX, "cross_comp_pred() etype=%d pos=(%d,%d) alpha=%d\n", compID, tu.blocks[compID].x, tu.blocks[compID].y, tu.compAlpha[compID] );
}

#if THRESHOLDING
void CABACWriter::thresholding( const TransformUnit& tu )
{
  unsigned numThresholdingSizes = Thresholding::getNumberValidSize( tu.Y(), tu.cs->slice->getSliceType(), tu.chType, &tu.cs->sps->getSpsNext() );

  if( !Thresholding::allowThresholding( tu ) )
  {
    numThresholdingSizes = 0;
  }

  if( 0 == numThresholdingSizes )
  {
    return;
  }

  m_BinEncoder.encodeBin( tu.thresholding ? 1 : 0, Ctx::Thresholding( 0 ) );

  if( tu.thresholding )
  {
    const SliceType sliceType = tu.cs->slice->isIntra() ? I_SLICE : B_SLICE;
    const int maxSize = numThresholdingSizes - 1;
    const int maxThrs = tu.cs->sps->getSpsNext().getThresholdingMaxThrs( sliceType ) - 1;

    unsigned ctxBse = 1;
    unsigned ctxIdx = ctxBse;

    unsigned binsToWrite = min( tu.thresholdingSize + 1, maxSize );
    for( int binsWritten = 0; binsWritten < binsToWrite; ++binsWritten )
    {
      unsigned nextBin = tu.thresholdingSize > binsWritten ? 1 : 0;
      m_BinEncoder.encodeBin( nextBin, Ctx::Thresholding( ctxIdx ) );
      ctxIdx = std::min( ctxIdx + 1, ctxBse + 4 );
    }

    ctxBse = 6;
    ctxIdx = ctxBse;

    binsToWrite = min( tu.thresholdingThrs + 1, maxThrs );
    for( int binsWritten = 0; binsWritten < binsToWrite; ++binsWritten )
    {
      unsigned nextBin = tu.thresholdingThrs > binsWritten ? 1 : 0;
      m_BinEncoder.encodeBin( nextBin, Ctx::Thresholding( ctxIdx ) );
      ctxIdx = std::min( ctxIdx + 1, ctxBse + 10 );
    }
  }

  DTRACE( g_trace_ctx, D_SYNTAX, "thresholding() usage=%d size=%d thrs=%d\n", tu.thresholding, tu.thresholdingSize, tu.thresholdingThrs );
}
#endif



//================================================================================
//  helper functions
//--------------------------------------------------------------------------------
//    void  unary_max_symbol  ( symbol, ctxId0, ctxIdN, maxSymbol )
//    void  unary_max_eqprob  ( symbol,                 maxSymbol )
//    void  exp_golomb_eqprob ( symbol, count )
//================================================================================

void CABACWriter::unary_max_symbol( unsigned symbol, unsigned ctxId0, unsigned ctxIdN, unsigned maxSymbol )
{
  CHECK( symbol > maxSymbol, "symbol > maxSymbol" );
  const unsigned totalBinsToWrite = std::min( symbol + 1, maxSymbol );
  for( unsigned binsWritten = 0; binsWritten < totalBinsToWrite; ++binsWritten )
  {
    const unsigned nextBin = symbol > binsWritten;
    m_BinEncoder.encodeBin( nextBin, binsWritten == 0 ? ctxId0 : ctxIdN );
  }
}


void CABACWriter::unary_max_eqprob( unsigned symbol, unsigned maxSymbol )
{
  if( maxSymbol == 0 )
  {
    return;
  }
  bool     codeLast = ( maxSymbol > symbol );
  unsigned bins     = 0;
  unsigned numBins  = 0;
  while( symbol-- )
  {
    bins   <<= 1;
    bins   ++;
    numBins++;
  }
  if( codeLast )
  {
    bins  <<= 1;
    numBins++;
  }
  CHECK(!( numBins <= 32 ), "Unspecified error");
  m_BinEncoder.encodeBinsEP( bins, numBins );
}


void CABACWriter::exp_golomb_eqprob( unsigned symbol, unsigned count )
{
  unsigned bins    = 0;
  unsigned numBins = 0;
  while( symbol >= (unsigned)(1<<count) )
  {
    bins <<= 1;
    bins++;
    numBins++;
    symbol -= 1 << count;
    count++;
  }
  bins <<= 1;
  numBins++;
  bins = (bins << count) | symbol;
  numBins += count;
  CHECK(!( numBins <= 32 ), "Unspecified error");
  m_BinEncoder.encodeBinsEP( bins, numBins );
}

void CABACWriter::encode_sparse_dt( DecisionTree& dt, unsigned toCodeId )
{
  // propagate the sparsity information from end-nodes to intermediate nodes
  dt.reduce();

  unsigned depth  = dt.dtt.depth;
  unsigned offset = 0;

  const unsigned encElPos = dt.dtt.mapping[toCodeId];

  while( dt.dtt.hasSub[offset] )
  {
    CHECKD( depth == 0, "Depth is '0' for a decision node in a decision tree" );

    const unsigned posRight = offset + 1;
    const unsigned posLeft  = offset + ( 1u << depth );

    const bool isLeft = encElPos >= posLeft;

    if( dt.isAvail[posRight] && dt.isAvail[posLeft] )
    {
      // encode the decision as both sub-paths are available
      const unsigned ctxId = dt.ctxId[offset];

      if( ctxId > 0 )
      {
        DTRACE( g_trace_ctx, D_DECISIONTREE, "Decision coding using context %d\n", ctxId - 1 );
        m_BinEncoder.encodeBin( isLeft ? 0 : 1, ctxId - 1 );
      }
      else
      {
        DTRACE( g_trace_ctx, D_DECISIONTREE, "Decision coding as an EP bin\n" );
        m_BinEncoder.encodeBinEP( isLeft ? 0 : 1 );
      }
    }

    DTRACE( g_trace_ctx, D_DECISIONTREE, "Following the tree to the %s sub-node\n", isLeft ? "left" : "right" );

    offset = isLeft ? posLeft : posRight;
    depth--;
  }

  CHECKD( offset != encElPos,             "Encoded a different element than assigned" );
  CHECKD( dt.dtt.ids[offset] != toCodeId, "Encoded a different element than assigned" );
  CHECKD( dt.isAvail[offset] == false,    "The encoded element is not available" );
  DTRACE( g_trace_ctx, D_DECISIONTREE,    "Found an end-node of the tree\n" );
  return;
}

//! \}
