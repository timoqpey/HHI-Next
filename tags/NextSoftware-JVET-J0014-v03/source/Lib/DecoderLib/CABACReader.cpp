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

/** \file     CABACReader.cpp
 *  \brief    Reader for low level syntax
 */

#include "CABACReader.h"

#include "CommonLib/CodingStructure.h"
#include "CommonLib/TrQuant.h"
#include "CommonLib/UnitTools.h"
#include "CommonLib/SampleAdaptiveOffset.h"
#include "CommonLib/AdaptiveLoopFilter.h"
#include "CommonLib/dtrace_next.h"
#include "CommonLib/Picture.h"
#include "CommonLib/IntraNNRom.h"
#if THRESHOLDING
#include "CommonLib/Thresholding.h"
#endif

#if RExt__DECODER_DEBUG_BIT_STATISTICS
#include "CommonLib/CodingStatistics.h"
#endif

#if RExt__DECODER_DEBUG_BIT_STATISTICS
#define RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET(x)           const CodingStatisticsClassType CSCT(x);                       m_BinDecoder.set( CSCT )
#define RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET2(x,y)        const CodingStatisticsClassType CSCT(x,y);                     m_BinDecoder.set( CSCT )
#define RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE(x,s)    const CodingStatisticsClassType CSCT(x, s.width, s.height);    m_BinDecoder.set( CSCT )
#define RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2(x,s,z) const CodingStatisticsClassType CSCT(x, s.width, s.height, z); m_BinDecoder.set( CSCT )
#define RExt__DECODER_DEBUG_BIT_STATISTICS_SET(x)                  m_BinDecoder.set( x );
#else
#define RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET(x)
#define RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET2(x,y)
#define RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE(x,s)
#define RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2(x,s,z)
#define RExt__DECODER_DEBUG_BIT_STATISTICS_SET(x)
#endif

#define BITCNT_START(_stage)
#define BITCNT_STOP()
#define BITCNT_ACCUM(_stage)
#define BITCNT_GET_BITS(_info,_trafo,_other)

void CABACReader::initCtxModels( Slice& slice, CABACDataStore* cabacDataStore )
{
  SliceType sliceType  = slice.getSliceType();
  Int       qp         = slice.getSliceQp();
  if( slice.getPPS()->getCabacInitPresentFlag() && slice.getCabacInitFlag() )
  {
    switch( sliceType )
    {
    case P_SLICE:           // change initialization table to B_SLICE initialization
      sliceType = B_SLICE;
      break;
    case B_SLICE:           // change initialization table to P_SLICE initialization
      sliceType = P_SLICE;
      break;
    default     :           // should not occur
      THROW( "Invalid slice type" );
      break;
    }
  }
  m_BinDecoder.reset( qp, (int)sliceType );
  if( cabacDataStore )
  {
    m_BinDecoder.setWinSizes( cabacDataStore->getWinSizes( &slice ) );
  }
}


//================================================================================
//  clause 7.3.8.1
//--------------------------------------------------------------------------------
//    bool  terminating_bit()
//    void  remaining_bytes( noTrailingBytesExpected )
//================================================================================

bool CABACReader::terminating_bit()
{
  if( m_BinDecoder.decodeBinTrm() )
  {
    m_BinDecoder.finish();
#if RExt__DECODER_DEBUG_BIT_STATISTICS
    CodingStatistics::IncrementStatisticEP( STATS__TRAILING_BITS, m_Bitstream->readOutTrailingBits(), 0 );
#else
    m_Bitstream->readOutTrailingBits();
#endif
    return true;
  }
  return false;
}

void CABACReader::remaining_bytes( bool noTrailingBytesExpected )
{
  if( noTrailingBytesExpected )
  {
    CHECK( 0 != m_Bitstream->getNumBitsLeft(), "Bits left when not supposed" );
  }
  else
  {
    while( m_Bitstream->getNumBitsLeft() )
    {
      unsigned trailingNullByte = m_Bitstream->readByte();
      if( trailingNullByte != 0 )
      {
        THROW( "Trailing byte should be '0', but has a value of " << std::hex << trailingNullByte << std::dec << "\n" );
      }
    }
  }
}





//================================================================================
//  clause 7.3.8.2
//--------------------------------------------------------------------------------
//    bool  coding_tree_unit( cs, area, qpL, qpC, ctuRsAddr )
//================================================================================

bool CABACReader::coding_tree_unit( CodingStructure& cs, const UnitArea& area, int (&qps)[2], unsigned ctuRsAddr )
{
  CUCtx cuCtx( qps[CH_L] );
  Partitioner *partitioner = PartitionerFactory::get( *cs.slice );

  partitioner->initCtu( area, CH_L, *cs.slice );

  BITCNT_START( BS_SIGNALING );

  sao( cs, ctuRsAddr );

  BITCNT_ACCUM( BS_OTHER );
  bool isLast = coding_tree( cs, *partitioner, cuCtx );
  qps[CH_L] = cuCtx.qp;
  if( !isLast && CS::isDualITree( cs ) && cs.pcv->chrFormat != CHROMA_400 )
  {
    CUCtx cuCtxChroma( qps[CH_C] );
    partitioner->initCtu( area, CH_C, *cs.slice );
    isLast = coding_tree( cs, *partitioner, cuCtxChroma );
    qps[CH_C] = cuCtxChroma.qp;
  }

  DTRACE_COND( ctuRsAddr == 0, g_trace_ctx, D_QP_PER_CTU, "\n%4d %2d", cs.picture->poc, cs.slice->getSliceQpBase() );
  DTRACE     (                 g_trace_ctx, D_QP_PER_CTU, " %3d",           qps[CH_L] - cs.slice->getSliceQpBase() );

  delete partitioner;
  return isLast;
}





//================================================================================
//  clause 7.3.8.3
//--------------------------------------------------------------------------------
//    void  sao( slice, ctuRsAddr )
//================================================================================

void CABACReader::sao( CodingStructure& cs, unsigned ctuRsAddr )
{
  const SPS&   sps   = *cs.sps;

  if( !sps.getUseSAO() )
  {
    return;
  }

  const Slice& slice                        = *cs.slice;
  SAOBlkParam&      sao_ctu_pars            = cs.picture->getSAO()[ctuRsAddr];
  bool              slice_sao_luma_flag     = ( slice.getSaoEnabledFlag( CHANNEL_TYPE_LUMA ) );
  bool              slice_sao_chroma_flag   = ( slice.getSaoEnabledFlag( CHANNEL_TYPE_CHROMA ) && sps.getChromaFormatIdc() != CHROMA_400 );
  sao_ctu_pars[ COMPONENT_Y  ].modeIdc      = SAO_MODE_OFF;
  sao_ctu_pars[ COMPONENT_Cb ].modeIdc      = SAO_MODE_OFF;
  sao_ctu_pars[ COMPONENT_Cr ].modeIdc      = SAO_MODE_OFF;
  if( !slice_sao_luma_flag && !slice_sao_chroma_flag )
  {
    return;
  }

  // merge
  int             frame_width_in_ctus     = cs.pcv->widthInCtus;
  int             ry                      = ctuRsAddr      / frame_width_in_ctus;
  int             rx                      = ctuRsAddr - ry * frame_width_in_ctus;
  int             sao_merge_type          = -1;
  const Position  pos( rx * cs.pcv->maxCUWidth, ry * cs.pcv->maxCUHeight );
  const unsigned  curSliceIdx = cs.slice->getIndependentSliceIdx();
  const unsigned  curTileIdx  = cs.picture->tileMap->getTileIdxMap( pos );

  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET( STATS__CABAC_BITS__SAO );

  if( cs.getCURestricted( pos.offset(-(Int)cs.pcv->maxCUWidth, 0), curSliceIdx, curTileIdx, CH_L ) )
  {
    // sao_merge_left_flag
    sao_merge_type  += int( m_BinDecoder.decodeBin( Ctx::SaoMergeFlag() ) );
  }

  if( sao_merge_type < 0 && cs.getCURestricted( pos.offset(0, -(Int)cs.pcv->maxCUHeight), curSliceIdx, curTileIdx, CH_L ) )
  {
    // sao_merge_above_flag
    sao_merge_type  += int( m_BinDecoder.decodeBin( Ctx::SaoMergeFlag() ) ) << 1;
  }
  if( sao_merge_type >= 0 )
  {
    if( slice_sao_luma_flag || slice_sao_chroma_flag )
    {
      sao_ctu_pars[ COMPONENT_Y  ].modeIdc  = SAO_MODE_MERGE;
      sao_ctu_pars[ COMPONENT_Y  ].typeIdc  = sao_merge_type;
    }
    if( slice_sao_chroma_flag )
    {
      sao_ctu_pars[ COMPONENT_Cb ].modeIdc  = SAO_MODE_MERGE;
      sao_ctu_pars[ COMPONENT_Cr ].modeIdc  = SAO_MODE_MERGE;
      sao_ctu_pars[ COMPONENT_Cb ].typeIdc  = sao_merge_type;
      sao_ctu_pars[ COMPONENT_Cr ].typeIdc  = sao_merge_type;
    }
    return;
  }

  // explicit parameters
  ComponentID firstComp = ( slice_sao_luma_flag   ? COMPONENT_Y  : COMPONENT_Cb );
  ComponentID lastComp  = ( slice_sao_chroma_flag ? COMPONENT_Cr : COMPONENT_Y  );
  for( ComponentID compID = firstComp; compID <= lastComp; compID = ComponentID( compID + 1 ) )
  {
    SAOOffset& sao_pars = sao_ctu_pars[ compID ];

    // sao_type_idx_luma / sao_type_idx_chroma
    if( compID != COMPONENT_Cr )
    {
      if( m_BinDecoder.decodeBin( Ctx::SaoTypeIdx() ) )
      {
        if( m_BinDecoder.decodeBinEP( ) )
        {
          // edge offset
          sao_pars.modeIdc = SAO_MODE_NEW;
          sao_pars.typeIdc = SAO_TYPE_START_EO;
        }
        else
        {
          // band offset
          sao_pars.modeIdc = SAO_MODE_NEW;
          sao_pars.typeIdc = SAO_TYPE_START_BO;
        }
      }
    }
    else //Cr, follow Cb SAO type
    {
      sao_pars.modeIdc = sao_ctu_pars[ COMPONENT_Cb ].modeIdc;
      sao_pars.typeIdc = sao_ctu_pars[ COMPONENT_Cb ].typeIdc;
    }
    if( sao_pars.modeIdc == SAO_MODE_OFF )
    {
      continue;
    }

    // sao_offset_abs
    int       offset[4];
    const int maxOffsetQVal = SampleAdaptiveOffset::getMaxOffsetQVal( sps.getBitDepth( toChannelType(compID) ) );
    offset    [0]           = (int)unary_max_eqprob( maxOffsetQVal );
    offset    [1]           = (int)unary_max_eqprob( maxOffsetQVal );
    offset    [2]           = (int)unary_max_eqprob( maxOffsetQVal );
    offset    [3]           = (int)unary_max_eqprob( maxOffsetQVal );

    // band offset mode
    if( sao_pars.typeIdc == SAO_TYPE_START_BO )
    {
      // sao_offset_sign
      for( int k = 0; k < 4; k++ )
      {
        if( offset[k] && m_BinDecoder.decodeBinEP( ) )
        {
          offset[k] = -offset[k];
        }
      }
      // sao_band_position
      sao_pars.typeAuxInfo = m_BinDecoder.decodeBinsEP( NUM_SAO_BO_CLASSES_LOG2 );
      for( int k = 0; k < 4; k++ )
      {
        sao_pars.offset[ ( sao_pars.typeAuxInfo + k ) % MAX_NUM_SAO_CLASSES ] = offset[k];
      }
      continue;
    }

    // edge offset mode
    sao_pars.typeAuxInfo = 0;
    if( compID != COMPONENT_Cr )
    {
      // sao_eo_class_luma / sao_eo_class_chroma
      sao_pars.typeIdc += m_BinDecoder.decodeBinsEP( NUM_SAO_EO_TYPES_LOG2 );
    }
    else
    {
      sao_pars.typeIdc  = sao_ctu_pars[ COMPONENT_Cb ].typeIdc;
    }
    sao_pars.offset[ SAO_CLASS_EO_FULL_VALLEY ] =  offset[0];
    sao_pars.offset[ SAO_CLASS_EO_HALF_VALLEY ] =  offset[1];
    sao_pars.offset[ SAO_CLASS_EO_PLAIN       ] =  0;
    sao_pars.offset[ SAO_CLASS_EO_HALF_PEAK   ] = -offset[2];
    sao_pars.offset[ SAO_CLASS_EO_FULL_PEAK   ] = -offset[3];
  }
}



UInt CABACReader::parseAlfUvlc ()
{
  UInt uiCode;
  Int  i;

  uiCode = m_BinDecoder.decodeBinEP();
  if ( uiCode == 0 )
  {
    return 0;
  }

  i=1;
  while (1)
  {
    uiCode = m_BinDecoder.decodeBinEP();
    if ( uiCode == 0 ) break;
    i++;
  }
  return i;
}

Int CABACReader::parseAlfSvlc()
{
  UInt uiCode;
  Int  iSign;
  Int  i;

  uiCode = m_BinDecoder.decodeBinEP();
  if ( uiCode == 0 )
  {
    return 0;
  }

  // read sign
  uiCode = m_BinDecoder.decodeBinEP();

  if ( uiCode == 0 ) iSign =  1;
  else               iSign = -1;

  // read magnitude
  i=1;
  while (1)
  {
    uiCode = m_BinDecoder.decodeBinEP();
    if ( uiCode == 0 ) break;
    i++;
  }
  return i*iSign;
}

Void CABACReader::xReadTruncBinCode(UInt& ruiSymbol, UInt uiMaxSymbol)
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
  UInt b = uiMaxSymbol - uiVal;
  ruiSymbol = m_BinDecoder.decodeBinsEP(uiThresh);
  if (ruiSymbol >= uiVal - b)
  {
    UInt uiSymbol;
    uiSymbol = m_BinDecoder.decodeBinEP();
    ruiSymbol <<= 1;
    ruiSymbol += uiSymbol;
    ruiSymbol -= (uiVal - b);
  }
}

UInt CABACReader::xReadEpExGolomb(UInt uiCount)
{
  UInt uiSymbol = 0;
  UInt uiBit = 1;

  while (uiBit)
  {
    uiBit = m_BinDecoder.decodeBinEP();
    uiSymbol += uiBit << uiCount++;
  }

  if (--uiCount)
  {
    UInt bins;
    bins = m_BinDecoder.decodeBinsEP(uiCount);
    uiSymbol += bins;
  }

  return uiSymbol;
}

Int CABACReader::alfGolombDecode(Int k)
{
  UInt uiSymbol;
  Int q = -1;
  Int nr = 0;
  Int m = (Int)pow(2.0, k);
  Int a;

  uiSymbol = 1;
  while (uiSymbol)
  {
    uiSymbol = m_BinDecoder.decodeBinEP();
    q++;
  }
  for(a = 0; a < k; ++a)          // read out the sequential log2(M) bits
  {
    uiSymbol = m_BinDecoder.decodeBinEP();
    if(uiSymbol)
      nr += 1 << a;
  }
  nr += q * m;                    // add the bits and the multiple of M
  if(nr != 0)
  {
    uiSymbol = m_BinDecoder.decodeBinEP();
    nr = (uiSymbol)? nr: -nr;
  }
  return nr;
}



void CABACReader::alf( CodingStructure& cs )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET( STATS__CABAC_BITS__ALF );

  const SPSNext& spsNext = cs.sps->getSpsNext();
  if( !spsNext.getALFEnabled() )
  {
    return;
  }
  ALFParam& alfParam = cs.picture->getALFParam();
  {
    CHECK( alfParam.num_ctus_in_frame != cs.pcv->sizeInCtus, "inconsistent" );
    alfParam.reset();
  }

  UInt alfFlag = m_BinDecoder.decodeBinEP();
  if( !alfFlag )
  {
    return;
  }

  alfParam.alf_flag = 1;

#if MCALF
  alfParam.classifier = (AlfFilterType)parseAlfUvlc();
#endif

#if COM16_C806_ALF_TEMPPRED_NUM
  if (cs.slice->getSliceType() == I_SLICE)
#else
  if (cs.slice->getSliceType() == I_SLICE )
#endif
  {
    alfParam.temporalPredFlag = 0;
  }
  else
  {
    alfParam.temporalPredFlag = m_BinDecoder.decodeBinEP();
    if( alfParam.temporalPredFlag )
    {
      alfParam.prevIdx = parseAlfUvlc();
    }
  }
  if( alfParam.temporalPredFlag == 0 )
  {
    alf_aux   ( alfParam, spsNext.getGALFEnabled() );
    alf_filter( alfParam, spsNext.getGALFEnabled() );
  }

#if MCALF
  if (alfParam.classifier == 1)
  {
    alfParam.numClasses = 20;
    for (Int i = 0; i < alfParam.numClasses; i++)
    {
      alfParam.selectedBins[i] = parseAlfUvlc();
    }
  }
  else if (alfParam.classifier == 3)
  {
    alfParam.numClasses = 9;
    for (Int i = 0; i < alfParam.numClasses; i++)
    {
      alfParam.selectedBins[i] = parseAlfUvlc();
    }
  }
  else
  {
    alfParam.numClasses = 0;
    ::memset(alfParam.selectedBins, 0, sizeof(Int) * 80);
  }
#endif

  if( spsNext.getGALFEnabled() )
  {
    alfParam.chroma_idc = parseAlfUvlc();
  #if COM16_C806_ALF_TEMPPRED_NUM
    if (!alfParam.temporalPredFlag && alfParam.chroma_idc)
  #else
    if (alfParam.chroma_idc)
  #endif
    {
      alfParam.num_coeff_chroma = ((alfParam.tap_chroma*alfParam.tap_chroma) >> 2) + 1;
      alf_filter(alfParam, spsNext.getGALFEnabled(), true);
    }
  }
  else
  {
    alf_chroma( alfParam );
  }
  alf_cu_ctrl( alfParam );
}

Void CABACReader::alf_aux( ALFParam& alfParam, bool isGALF )
{
  Int FiltTab[3] = {5, 7, 9};
  Int sqrFiltLengthTab[3] = {AdaptiveLoopFilter::m_SQR_FILT_LENGTH_5SYM, AdaptiveLoopFilter::m_SQR_FILT_LENGTH_7SYM, AdaptiveLoopFilter::m_SQR_FILT_LENGTH_9SYM };

  UInt uiSymbol;
  const int iNoVarBins = AdaptiveLoopFilter::m_NO_VAR_BINS;
  int i;
  if( isGALF )
  {
    memset(alfParam.filterPattern, 0, sizeof(Int)*AdaptiveLoopFilter::m_NO_VAR_BINS);
    //number of total filters
    xReadTruncBinCode(uiSymbol, iNoVarBins);
    alfParam.filters_per_group = uiSymbol + 1;
  }

  AlfFilterType filtType = (AlfFilterType) parseAlfUvlc();
  alfParam.filterType =                   filtType;
  alfParam.tapH       =          FiltTab[ filtType ];
  alfParam.num_coeff  = sqrFiltLengthTab[ filtType ];

  if( isGALF )
  {
    //filter set index for each class
    if (alfParam.filters_per_group > 1)
    {
#if MCALF
      Int maxPat = iNoVarBins;
      if (alfParam.classifier == 2)
      {
        maxPat = iNoVarBins - 2;
      }
      for (i = 0; i < maxPat; i++)
#else
      for (i = 0; i< iNoVarBins; i++)
#endif
      {
        xReadTruncBinCode(uiSymbol, (UInt)alfParam.filters_per_group);
        alfParam.filterPattern[i] = (Int)uiSymbol;
      }
    }
    else
    {
      memset(alfParam.filterPattern, 0, iNoVarBins* sizeof(Int));
    }

#if JVET_C0038_NO_PREV_FILTERS
    Int decodetab_pred[3] = { 1, 0, 2 };
    memset(alfParam.PrevFiltIdx, 0, sizeof(Int)*AdaptiveLoopFilter::m_NO_VAR_BINS);

    if (alfParam.iAvailableFilters > 0)
    {
      // prediction pattern
      //0: all zero, no pred from pre-defined filters; 1: all are predicted but could be different values; 2: some predicted and some not

      alfParam.iPredPattern = decodetab_pred[xReadEpExGolomb(0)];

      if (alfParam.iPredPattern == 0)
      {
        memset(alfParam.PrevFiltIdx, 0, sizeof(Int)*AdaptiveLoopFilter::m_NO_VAR_BINS);
      }
      else
      {
        if (alfParam.iPredPattern == 2)
        {
          //on/off flags
          for (i = 0; i < iNoVarBins; i++)
          {
            alfParam.PrevFiltIdx[i] = m_BinDecoder.decodeBinEP();
          }
        }
        else
        {
          CHECK(alfParam.iPredPattern != 1, "iPredPattern has to be equal 1");
          for (i = 0; i < iNoVarBins; i++)
          {
            alfParam.PrevFiltIdx[i] = 1;
          }
        }
        if (alfParam.iAvailableFilters > 1)
        {
          CHECK(alfParam.iPredPattern <= 0, "iPredPattern has to be greater 0");
          for (i = 0; i < iNoVarBins; i++)
          {
            if (alfParam.PrevFiltIdx[i] > 0)
            {
              xReadTruncBinCode(uiSymbol, (UInt)alfParam.iAvailableFilters);
              alfParam.PrevFiltIdx[i] = (uiSymbol + 1);
            }
          }
        }
      }
    }
#endif
  }
  else
  {
    alfParam.filters_per_group = 0;
    memset ( alfParam.filterPattern, 0 , sizeof(Int)*AdaptiveLoopFilter::m_NO_VAR_BINS);

    AlfFilterMode fMode = (AlfFilterMode) parseAlfUvlc();

    alfParam.filterMode = fMode;

    if( fMode == ALF_ONE_FILTER )
    {
      alfParam.filters_per_group = 1;
    }
    else if( fMode == ALF_TWO_FILTERS )
    {
      alfParam.filters_per_group = 2;
      UInt symbol = parseAlfUvlc();
      alfParam.startSecondFilter = symbol;
      alfParam.filterPattern[ symbol ] = 1;
    }
    else if( fMode == ALF_MULTIPLE_FILTERS )
    {
      alfParam.filters_per_group = 1;
      for( int i=1; i< AdaptiveLoopFilter::m_NO_VAR_BINS; i++)
      {
        UInt symbol = m_BinDecoder.decodeBinEP();
        alfParam.filterPattern[i]   = symbol;
        alfParam.filters_per_group += symbol;
      }
    }
    else
    {
      THROW( "ALF: unknown filter mode" );
    }
  }

 memset( alfParam.mapClassToFilter, 0, AdaptiveLoopFilter::m_NO_VAR_BINS * sizeof(int));
 for(Int i = 1; i < AdaptiveLoopFilter::m_NO_VAR_BINS; ++i)
 {
   if( alfParam.filterPattern[i])
   {
     alfParam.mapClassToFilter[i] = alfParam.mapClassToFilter[i-1] + 1;
   }
   else
   {
     alfParam.mapClassToFilter[i] = alfParam.mapClassToFilter[i-1];
   }
 }

}

Void CABACReader::alf_filter( ALFParam& alfParam, bool isGALF, bool bChroma )
{
  UInt uiSymbol;
  int ind, scanPos, i;
  int golombIndexBit;
  int kMin;
  int maxScanVal = 0;
  const Int* pDepthInt = nullptr;

#if FORCE0
  if (!bChroma)
  {
    alfParam.forceCoeff0 = m_BinDecoder.decodeBinEP();
    if (!alfParam.forceCoeff0)
    {
#endif
      for (int i = 0; i < AdaptiveLoopFilter::m_NO_VAR_BINS; i++)
      {
        alfParam.codedVarBins[i] = 1;
      }
      if (alfParam.filters_per_group > 1)
      {
#if !FORCE0
        alfParam.forceCoeff0 = 0;
#endif
        alfParam.predMethod = m_BinDecoder.decodeBinEP();
      }
      else
      {
        alfParam.forceCoeff0 = 0;
        alfParam.predMethod = 0;
      }
#if FORCE0
    }
    else
    {
      alfParam.predMethod = 0;
    }
  }
#endif

  //Determine maxScanVal
  if( bChroma)
  {
    pDepthInt = AdaptiveLoopFilter::m_pDepthIntTab[alfParam.tap_chroma == 5 ? 0 : (alfParam.tap_chroma == 7 ? 1 : 2)];
  }
  else
  {
    pDepthInt = AdaptiveLoopFilter::m_pDepthIntTab[alfParam.filterType];
  }
  int maxIdx = (bChroma ? alfParam.num_coeff_chroma : alfParam.num_coeff) - !!isGALF;
  for( int i = 0; i < maxIdx; i++ )
  {
    maxScanVal = std::max(maxScanVal, pDepthInt[i]);
  }

  uiSymbol = parseAlfUvlc();
  alfParam.minKStart = 1 + uiSymbol;

  kMin = alfParam.minKStart;
  for(scanPos = 0; scanPos < maxScanVal; scanPos++)
  {
     golombIndexBit = m_BinDecoder.decodeBinEP();
     if( golombIndexBit)
     {
       alfParam.kMinTab[scanPos] = kMin + 1;
     }
     else
     {
       alfParam.kMinTab[scanPos] = kMin;
     }
     kMin = alfParam.kMinTab[scanPos];
  }

#if FORCE0
  if (!bChroma)
  {
    if (alfParam.forceCoeff0)
    {
      for (ind = 0; ind < alfParam.filters_per_group; ++ind)
      {
        alfParam.codedVarBins[ind] = m_BinDecoder.decodeBinEP();
      }
    }
  }
#endif
  if( bChroma && isGALF)
  {
    Int iNumCoeffMinus1 = alfParam.num_coeff_chroma - 1;
    for (i = 0; i < iNumCoeffMinus1; i++)
    {
      scanPos = pDepthInt[i] - 1;
      alfParam.coeff_chroma[i] = alfGolombDecode(alfParam.kMinTab[scanPos]);
    }
  }
  else
  {
    // Filter coefficients
    for (ind = 0; ind < alfParam.filters_per_group; ++ind)
    {
      if( isGALF )
      {
#if FORCE0
        if (!alfParam.codedVarBins[ind] && alfParam.forceCoeff0)
        {
          continue;
        }
#endif
      }
      int maxIdx = alfParam.num_coeff  - !!isGALF;
      for( int i = 0; i < maxIdx; i++ )
      {
        scanPos = pDepthInt[i] - 1;
        alfParam.coeffmulti[ind][i] = alfGolombDecode(alfParam.kMinTab[scanPos]);
      }
    }
  }
}

Void CABACReader::alf_cu_ctrl( ALFParam& alfParam )
{
  alfParam.cu_control_flag = m_BinDecoder.decodeBinEP();
  if( alfParam.cu_control_flag )
  {
    alfParam.alf_max_depth = unary_max_symbol( Ctx::AlfUvlcSCModel( 0 ), Ctx::AlfUvlcSCModel( 1 ), alfParam.maxCodingDepth - 1 );

    UInt uiLength = 0;
    UInt minValue = alfParam.num_ctus_in_frame;
    UInt maxValue = ( minValue << ( alfParam.alf_max_depth * 2) );
    UInt temp = maxValue - minValue;
    for( UInt i = 0; i < 32; i++ )
    {
      if( temp & 0x1 )
      {
        uiLength = i + 1;
      }
      temp = ( temp >> 1 );
    }
    UInt numOfFlags = 0;
    UInt uiBit;
    if( uiLength )
    {
      while( uiLength-- )
      {
        uiBit = m_BinDecoder.decodeBinEP();
        numOfFlags += uiBit << uiLength;
      }
    }
    numOfFlags += minValue;
    alfParam.num_alf_cu_flag = numOfFlags;

    DTRACE( g_trace_ctx, D_SYNTAX, "alf_cu_ctrl() max_depth=%d max_alf_depth=%d num_cu_flags=%d\n", alfParam.maxCodingDepth, alfParam.alf_max_depth, alfParam.num_alf_cu_flag );

    for( UInt i = 0; i < alfParam.num_alf_cu_flag; i++ )
    {
      alfParam.alf_cu_flag[i] = m_BinDecoder.decodeBin( Ctx::AlfCUCtrlFlags( 0 ) );

      DTRACE( g_trace_ctx, D_SYNTAX, "alf_cu_ctrl() blk=%d alf_cu_flag[blk]=%d\n", i, alfParam.alf_cu_flag[i] );
    }
  }
  else
  {
    DTRACE( g_trace_ctx, D_SYNTAX, "alf_cu_ctrl() off\n" );
  }
}


Void CABACReader::alf_chroma( ALFParam& alfParam )
{

  //alfParam.chroma_idc = unary_max_eqprob( 3 );
  alfParam.chroma_idc = parseAlfUvlc();
  if(alfParam.chroma_idc && !alfParam.temporalPredFlag )
  {
    //alfParam.tap_chroma = (unary_max_eqprob( 3 ) <<1 ) + 5;
    alfParam.tap_chroma = (parseAlfUvlc() <<1 ) + 5;
    alfParam.num_coeff_chroma = ((alfParam.tap_chroma*alfParam.tap_chroma+1) >> 1) + 1;
    // filter coefficients for chroma
    for(Int pos=0; pos<alfParam.num_coeff_chroma; pos++)
    {
      alfParam.coeff_chroma[pos] = parseAlfSvlc();
    }
  }
}



//================================================================================
//  clause 7.3.8.4
//--------------------------------------------------------------------------------
//    bool  coding_tree       ( cs, partitioner, cuCtx )
//    bool  split_cu_flag     ( cs, partitioner )
//    split split_cu_mode_mt  ( cs, partitioner )
//================================================================================

bool CABACReader::coding_tree( CodingStructure& cs, Partitioner& partitioner, CUCtx& cuCtx )
{
  const PPS      &pps         = *cs.pps;
  const UnitArea &currArea    = partitioner.currArea();
  bool           lastSegment  = false;

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
    // force QT split enabling on the edges and if the current area exceeds maximum transformation size
    bool qtSplit = implicitSplit == CU_QUAD_SPLIT;

    // split_cu_flag
    if( !qtSplit && implicitSplit != CU_QUAD_SPLIT )
    {
      qtSplit = split_cu_flag( cs, partitioner );
    }

    // quad-tree split
    if( qtSplit )
    {
      partitioner.splitCurrArea( CU_QUAD_SPLIT, cs );

      do
      {
        if( !lastSegment && cs.area.blocks[partitioner.chType].contains( partitioner.currArea().blocks[partitioner.chType].pos() ) )
        {
          lastSegment = coding_tree( cs, partitioner, cuCtx );
        }
      } while( partitioner.nextPart( cs ) );

      partitioner.exitCurrSplit();
      return lastSegment;
    }
  }

  if( cs.sps->getSpsNext().getUseGenBinSplit() )
  {
    PartSplit split = implicitSplit;

    if( split == CU_DONT_SPLIT )
    {
      split = gen_bin_split_mode( cs, partitioner );

      if( split != CU_DONT_SPLIT )
      {
        const SplitModifier mod = gen_bin_split_mod( cs, partitioner, split );

        split = applyModifier( split, mod );
      }
    }

    if( split != CU_DONT_SPLIT )
    {
      partitioner.splitCurrArea( split, cs );

      do
      {
        if( !lastSegment && cs.area.blocks[partitioner.chType].contains( partitioner.currArea().blocks[partitioner.chType].pos() ) )
        {
          lastSegment = coding_tree( cs, partitioner, cuCtx );
        }
      } while( partitioner.nextPart( cs ) );

      partitioner.exitCurrSplit();
      return lastSegment;
    }
  }
  else
  {
    // MT
    bool mtSplit = partitioner.canSplit( CU_MT_SPLIT, cs );

    if( mtSplit )
    {
      const PartSplit splitMode = implicitSplit != CU_DONT_SPLIT ? implicitSplit : split_cu_mode_mt( cs, partitioner );

      if( splitMode != CU_DONT_SPLIT )
      {
        partitioner.splitCurrArea( splitMode, cs );

        do
        {
          if( !lastSegment && cs.area.blocks[partitioner.chType].contains( partitioner.currArea().blocks[partitioner.chType].pos() ) )
          {
            lastSegment = coding_tree( cs, partitioner, cuCtx );
          }
        } while( partitioner.nextPart( cs ) );

        partitioner.exitCurrSplit();
        return lastSegment;
      }
    }
  }


  CodingUnit& cu = cs.addCU( CS::getArea( cs, currArea, partitioner.chType ), partitioner.chType );

  partitioner.setCUData( cu );
  cu.slice   = cs.slice;
  cu.tileIdx = cs.picture->tileMap->getTileIdxMap( currArea.lumaPos() );

  // Predict QP on start of quantization group
  if( pps.getUseDQP() && !cuCtx.isDQPCoded && CU::isQGStart( cu ) )
  {
    cuCtx.qp = CU::predictQP( cu, cuCtx.qp );
  }

  cu.qp          = cuCtx.qp;        //NOTE: CU QP can be changed by deltaQP signaling at TU level
  cu.chromaQpAdj = cs.chromaQpAdj;  //NOTE: CU chroma QP adjustment can be changed by adjustment signaling at TU level

  // coding unit
  bool isLastCtu = coding_unit( cu, partitioner, cuCtx );

  DTRACE( g_trace_ctx, D_QP, "x=%d, y=%d, w=%d, h=%d, qp=%d\n", cu.Y().x, cu.Y().y, cu.Y().width, cu.Y().height, cu.qp );
  return isLastCtu;
}

PartSplit CABACReader::gen_bin_split_mode( CodingStructure& cs, Partitioner& partitioner )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET( STATS__CABAC_BITS__GBS );

  const unsigned width      = partitioner.currArea().lumaSize().width;
  const unsigned height     = partitioner.currArea().lumaSize().height;
  auto &gbsPartitioner      = *dynamic_cast<GenBinSplitPartitioner*>( &partitioner );

  DecisionTree dt( g_genBinSplitDTT );

  dt.setAvail( DTT_GBS_PERP_SPLIT, partitioner.canSplit( CU_PERPENDICULAR_SPLIT, cs ) );
  dt.setAvail( DTT_GBS_PARL_SPLIT, partitioner.canSplit( CU_PARALLEL_SPLIT,      cs ) );
  dt.setAvail( DTT_GBS_DONT_SPLIT, partitioner.canSplit( CU_DONT_SPLIT,          cs ) );

  unsigned btSCtxId = width == height ? 1 : ( width > height ? 0 : 2 );
  if( gbsPartitioner.getActualSplitType( CU_PERPENDICULAR_SPLIT ) == CU_VERT_SPLIT )
  {
    btSCtxId = 2 - btSCtxId;
  }
  unsigned btCtxIdx = DeriveCtx::CtxBTsplit( cs, partitioner );

  dt.setCtxId( DTT_GBS_DO_SPLIT,      Ctx::GBSSplitFlag(     btCtxIdx ) );
  dt.setCtxId( DTT_GBS_DO_PERP_SPLIT, Ctx::GBSSplitFlag( 3 + btSCtxId ) );

  unsigned id = decode_sparse_dt( dt );

  DTRACE( g_trace_ctx, D_SYNTAX, "gen_bin_split_mode() %dx%d at [%d,%d], btSCtx = %d, btCtx = %d, split = %d\n", width, height, partitioner.currArea().lx(), partitioner.currArea().ly(), btSCtxId, btCtxIdx, id );

  if( id == DTT_GBS_PARL_SPLIT )      return gbsPartitioner.getActualSplitType( CU_PARALLEL_SPLIT      );
  else if( id == DTT_GBS_PERP_SPLIT ) return gbsPartitioner.getActualSplitType( CU_PERPENDICULAR_SPLIT );
  else                                return gbsPartitioner.getActualSplitType( CU_DONT_SPLIT          );
}

SplitModifier CABACReader::gen_bin_split_mod( CodingStructure& cs, Partitioner& partitioner, const PartSplit baseSplit )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET( STATS__CABAC_BITS__GBS );

  const PartSplit base    = getBaseSplit( baseSplit );
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

  SplitModifier mod = SplitModifier( decode_sparse_dt( dt ) );

  DTRACE( g_trace_ctx, D_SYNTAX, "gen_bin_split_mod() %dx%d at [%d,%d] qtd=%d d=%d base=%d dirCtx=%d, mod = %d\n", partitioner.currArea().lwidth(), partitioner.currArea().lheight(), partitioner.currArea().lx(), partitioner.currArea().ly(), partitioner.currQtDepth, partitioner.currDepth, base, dirCtx, mod );

  return mod;
}
PartSplit CABACReader::split_cu_mode_mt( CodingStructure& cs, Partitioner &partitioner )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET( STATS__CABAC_BITS__SPLIT_FLAG );

  PartSplit mode      = CU_DONT_SPLIT;

  unsigned ctxIdBT    = DeriveCtx::CtxBTsplit( cs, partitioner );

  unsigned width      = partitioner.currArea().lumaSize().width;
  unsigned height     = partitioner.currArea().lumaSize().height;

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
  dt.setCtxId( DTT_SPLIT_DO_SPLIT_DECISION,   Ctx::BTSplitFlag( ctxIdBT ) );      // 0- 2
  dt.setCtxId( DTT_SPLIT_HV_DECISION,         Ctx::BTSplitFlag( 3 + btSCtxId ) ); // 3- 5

  dt.setCtxId( DTT_SPLIT_H_IS_BT_12_DECISION, Ctx::BTSplitFlag( 6 + btSCtxId ) ); // 6- 8
  dt.setCtxId( DTT_SPLIT_V_IS_BT_12_DECISION, Ctx::BTSplitFlag( 9 + btSCtxId ) ); // 9-11

  unsigned id = decode_sparse_dt( dt );

  mode = id == DTT_SPLIT_NO_SPLIT ? CU_DONT_SPLIT : PartSplit( id );

  DTRACE( g_trace_ctx, D_SYNTAX, "split_cu_mode_mt() ctx=%d split=%d\n", ctxIdBT, mode );

  return mode;

}

bool CABACReader::split_cu_flag( CodingStructure& cs, Partitioner &partitioner )
{
  // TODO: make maxQTDepth a slice parameter
  unsigned maxQTDepth = ( cs.sps->getSpsNext().getUseGenBinSplit() || cs.sps->getSpsNext().getUseQTBT()
    ? g_aucLog2[cs.sps->getSpsNext().getCTUSize()] - g_aucLog2[cs.sps->getSpsNext().getMinQTSize( cs.slice->getSliceType(), partitioner.chType )]
    : cs.sps->getLog2DiffMaxMinCodingBlockSize() );
  if( partitioner.currDepth == maxQTDepth )
  {
    return false;
  }

  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE( STATS__CABAC_BITS__SPLIT_FLAG, partitioner.currArea().lumaSize() );

  unsigned  ctxId = DeriveCtx::CtxCUsplit( cs, partitioner );
  bool      split = ( m_BinDecoder.decodeBin( Ctx::SplitFlag( ctxId ) ) );
  DTRACE( g_trace_ctx, D_SYNTAX, "split_cu_flag() ctx=%d split=%d\n", ctxId, split ? 1 : 0 );
  return split;
}


//================================================================================
//  clause 7.3.8.5
//--------------------------------------------------------------------------------
//    bool  coding_unit               ( cu, partitioner, cuCtx )
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
//    bool  end_of_ctu                ( cu, cuCtx )
//================================================================================

bool CABACReader::coding_unit( CodingUnit &cu, Partitioner &partitioner, CUCtx& cuCtx )
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

  // skip data
  if( cu.skip )
  {
    cs.addTU         ( cu, partitioner.chType );
    PredictionUnit&    pu = cs.addPU( cu, partitioner.chType );
    MergeCtx           mrgCtx;
    prediction_unit  ( pu, mrgCtx );
    cu.obmcFlag      = cu.cs->sps->getSpsNext().getUseOBMC();
    cu_lic_flag      ( cu );
    CHECK( cu.diffFilterIdx != 0, "cu.skip && cu.diffFilterIdx != 0" );
    return end_of_ctu( cu, cuCtx );
  }

  // prediction mode and partitioning data
  pred_mode ( cu );
  mrl_idx   ( cu );
  pdpc_flag ( cu );
  part_mode ( cu );

  // --> create PUs
  CU::addPUs( cu );

  // pcm samples
  if( CU::isIntra(cu) && cu.partSize == SIZE_2Nx2N )
  {
    pcm_flag( cu );
    if( cu.ipcm )
    {
      TransformUnit& tu = cs.addTU( cu, partitioner.chType );
      pcm_samples( tu );
      return end_of_ctu( cu, cuCtx );
    }
  }

  // prediction data ( intra prediction modes / reference indexes + motion vectors )
  cu_pred_data( cu );
  if( CU::isDiffIdxPresent( cu ) )
  {
    cu_diffusion_filter_idx( cu );
  }

  // residual data ( coded block flags + transform coefficient levels )
  cu_residual( cu, partitioner, cuCtx );

  if( cs.sps->getSpsNext().getUseIntraNNTrafos() && cu.intra_NN )
  {
    cu.intraNNTrafoIdx = cu.nsstIdx; // Here intra NN transform index is branched from NSST path.
    cu.nsstIdx = 0;
  }
  else
  {
    cu.intraNNTrafoIdx = 0;
  }
  CHECK( cu.intra_NN && cu.nsstIdx, "Intra NN is incompatible with NSST." );
  CHECK( !cu.intra_NN && cu.intraNNTrafoIdx, "Intra NN trafos are only allowed for intra NN." );

  // check end of cu
  return end_of_ctu( cu, cuCtx );
}


void CABACReader::cu_transquant_bypass_flag( CodingUnit& cu )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET( STATS__CABAC_BITS__TQ_BYPASS_FLAG );

  cu.transQuantBypass = ( m_BinDecoder.decodeBin( Ctx::TransquantBypassFlag() ) );
}


void CABACReader::cu_skip_flag( CodingUnit& cu )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET( STATS__CABAC_BITS__SKIP_FLAG );

  unsigned ctxId  = DeriveCtx::CtxSkipFlag(cu);
  unsigned skip   = m_BinDecoder.decodeBin( Ctx::SkipFlag(ctxId) );

  DTRACE( g_trace_ctx, D_SYNTAX, "cu_skip_flag() ctx=%d skip=%d\n", ctxId, skip ? 1 : 0 );

  if( skip )
  {
    cu.skip     = true;
    cu.rootCbf  = false;
    cu.predMode = MODE_INTER;
    cu.partSize = SIZE_2Nx2N;
  }
}

void CABACReader::imv_mode( CodingUnit& cu, MergeCtx& mrgCtx )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET( STATS__CABAC_BITS__OTHER );

  if( !cu.cs->sps->getSpsNext().getUseIMV() )
  {
    return;
  }

  Bool bNonZeroMvd = CU::hasSubCUNonZeroMVd( cu );
  if( !bNonZeroMvd )
  {
    return;
  }

  const SPSNext& spsNext = cu.cs->sps->getSpsNext();

  unsigned value = 0;
  unsigned ctxId = DeriveCtx::CtxIMVFlag( cu );
  value = m_BinDecoder.decodeBin( Ctx::ImvFlag( ctxId ) );
  DTRACE( g_trace_ctx, D_SYNTAX, "imv_mode() value=%d ctx=%d\n", value, ctxId );

  if( spsNext.getImvMode() == IMV_4PEL && value )
  {
    value = m_BinDecoder.decodeBin( Ctx::ImvFlag( 3 ) );
    DTRACE( g_trace_ctx, D_SYNTAX, "imv_mode() value=%d ctx=%d\n", value, 3 );
    value++;
  }

  cu.imv = value;
  DTRACE( g_trace_ctx, D_SYNTAX, "imv_mode() IMVFlag=%d\n", cu.imv );
}

void CABACReader::pred_mode( CodingUnit& cu )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET( STATS__CABAC_BITS__PRED_MODE );

  if( cu.cs->slice->isIntra() || m_BinDecoder.decodeBin( Ctx::PredMode() ) )
  {
    cu.predMode = MODE_INTRA;
  }
  else
  {
    cu.predMode = MODE_INTER;
  }
}

void CABACReader::part_mode( CodingUnit& cu )
{
  if( cu.cs->pcv->only2Nx2N )
  {
    cu.partSize = SIZE_2Nx2N;
    return;
  }

  const SPS&      sps                           = *cu.cs->sps;
  const unsigned  cuWidth                       = cu.lumaSize().width;
  const unsigned  cuHeight                      = cu.lumaSize().height;
  const int       log2DiffMaxMinCodingBlockSize = sps.getLog2DiffMaxMinCodingBlockSize();

  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE( STATS__CABAC_BITS__PART_SIZE, cu.blocks[cu.chType].lumaSize() );

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

  unsigned id = decode_sparse_dt( dt );
  cu.partSize = PartSize( id );
}

void CABACReader::mrl_idx( CodingUnit& cu )
{
  if ( !cu.cs->sps->getSpsNext().getUseIntraMRL() || cu.predMode != MODE_INTRA || !isLuma( cu.chType ) )
  {
    cu.mrlIdx = 0;
    return;
  }

  const int log2CuHeight  = g_aucLog2[cu.lumaSize().height];
  const int log2CuWidth   = g_aucLog2[cu.lumaSize().width];
  if( abs( log2CuHeight - log2CuWidth ) > 1  || log2CuHeight < MRL_FAST_LOG2_MIN_SIZE || log2CuWidth < MRL_FAST_LOG2_MIN_SIZE )
  {
    cu.mrlIdx = 0;
    return;
  }

  if ( !m_BinDecoder.decodeBin( Ctx::MrlIdx(0) ) )
  {
    cu.mrlIdx = 0;
  }
  else
  {
    if ( !m_BinDecoder.decodeBin( Ctx::MrlIdx(1) ) )
    {
      cu.mrlIdx = 1;
    }
    else
    {
      cu.mrlIdx = 3;
    }

  }
  DTRACE( g_trace_ctx, D_SYNTAX, "mrl_idx() value=%d pos=(%d,%d)\n", cu.mrlIdx, cu.lumaPos().x, cu.lumaPos().y );
}

void CABACReader::pdpc_flag( CodingUnit& cu )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET( STATS__CABAC_BITS__INTRA_PDPC_FLAG );

  if (!cu.cs->sps->getSpsNext().isIntraPDPC() || cu.predMode == MODE_INTER)
  {
    cu.pdpc = false;
    return;
  }
  if( cu.mrlIdx != 0 )
  {
    cu.pdpc = false;
    return;
  }

  cu.pdpc = ( m_BinDecoder.decodeBin( Ctx::PdpcFlag() ) );
}

void CABACReader::pcm_flag( CodingUnit& cu )
{
  const SPS& sps = *cu.cs->sps;
  if( !sps.getUsePCM() || cu.lumaSize().width > (1 << sps.getPCMLog2MaxSize()) || cu.lumaSize().width < (1 << sps.getPCMLog2MinSize()) )
  {
    cu.ipcm = false;
    return;
  }
  cu.ipcm = ( m_BinDecoder.decodeBinTrm() );
}


void CABACReader::cu_pred_data( CodingUnit &cu )
{
  if( CU::isIntra( cu ) )
  {
    intra_luma_pred_modes( cu );
    intra_chroma_pred_modes( cu );
    return;
  }

  MergeCtx mrgCtx;

  for( auto &pu : CU::traversePUs( cu ) )
  {
    prediction_unit( pu, mrgCtx );
  }

  imv_mode   ( cu, mrgCtx );
  obmc_flag  ( cu );
  cu_lic_flag( cu ); // local illumination compensation

  for( auto &pu : CU::traversePUs( cu ) )
  {
    PU::spanLICFlags( pu, cu.LICFlag );
  }
}


void CABACReader::cu_lic_flag( CodingUnit& cu )
{
  if( CU::isLICFlagPresent( cu ) )
  {
    RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET( STATS__CABAC_BITS__LIC_FLAG );

    cu.LICFlag = ( m_BinDecoder.decodeBin( Ctx::LICFlag() ) );
    DTRACE( g_trace_ctx, D_SYNTAX, "cu_lic_flag() lic_flag=%d\n", cu.LICFlag?1:0 );
  }
}

Void CABACReader::obmc_flag( CodingUnit& cu )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET( STATS__CABAC_BITS__OTHER );

  cu.obmcFlag = cu.cs->sps->getSpsNext().getUseOBMC();
  if( !cu.obmcFlag )
  {
    return;
  }

  Bool bCoded = CU::isObmcFlagCoded( cu );

  if( bCoded )
  {
    unsigned obmc = m_BinDecoder.decodeBin( Ctx::ObmcFlag() );
    cu.obmcFlag = ( Bool ) obmc;
  }

  DTRACE( g_trace_ctx, D_SYNTAX, "obmc_flag() obmc=%d pos=(%d,%d)\n", cu.obmcFlag ? 1 : 0, cu.lumaPos().x, cu.lumaPos().y );
}

void CABACReader::nn_flag( CodingUnit& cu )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET( STATS__CABAC_BITS__OTHER );

  if( !cu.Y().valid() )
  {
    return;
  }
  if( !( cu.cs->sps->getSpsNext().getUseIntra_NN() ) )
  {
    cu.intra_NN = ( UChar ) 0;
    return;
  }
  if( cu.pdpc )
  {
    cu.intra_NN = ( UChar ) 0;
    return;
  }
  if (cu.lwidth() > g_max_size_NNs)
  {
    return;
  }
  if (cu.lheight() > g_max_size_NNs)
  {
    return;
  }
  if( cu.mrlIdx )
  {
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
    cu.intra_NN = UChar( 0 );
    return;
  }
  unsigned ctxId = DeriveCtx::CtxIntraNNModel( cu );
  unsigned flagIntraNN = m_BinDecoder.decodeBin( Ctx::IntraNNFlag( ctxId ) );
  cu.intra_NN = UChar( flagIntraNN );
  nn_subsampling_flag( cu );
}

unsigned CABACReader::code_unary_fixed( unsigned ctxId, unsigned unary_max, unsigned fixed )
{
  unsigned idx;
  bool unary = m_BinDecoder.decodeBin( ctxId );
  if( unary )
  {
    idx = unary_max_eqprob( unary_max );
  }
  else
  {
    idx = unary_max + 1 + m_BinDecoder.decodeBinsEP( fixed );
  }
  return idx;
}

void CABACReader::intra_luma_pred_modes_NN( CodingUnit &cu )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET( STATS__CABAC_BITS__OTHER );

  if( !cu.Y().valid() )
  {
    return;
  }
  for( auto &pu : CU::traversePUs( cu ) )
  {
    intra_luma_pred_mode_NN( pu );
  }
}

void CABACReader::intra_luma_pred_mode_NN( PredictionUnit &pu )
{
  CHECK( pu.lwidth()  > g_max_size_NNs || pu.lheight() > g_max_size_NNs, "Error: block size not supported" );

  const int padded_width = g_PaddedSizes[pu.lwidth()];
  const int padded_height = g_PaddedSizes[pu.lheight()];
  const int width_for_NN = pu.cu->intra_NN_Use_Sampling ? ( padded_width >> 1 ) : padded_width;
  const int height_for_NN = pu.cu->intra_NN_Use_Sampling ? ( padded_height >> 1 ) : padded_height;
  const int w = g_aucLog2[width_for_NN];
  const int h = g_aucLog2[height_for_NN];

  int unaryMax = g_numIntraMPM_NN[w][h] - 1;
  int fixedLength = g_numIntraEPBins_NN[w][h];
  unsigned predIdx = code_unary_fixed( Ctx::IntraPredModeANN[CHANNEL_TYPE_LUMA]( 0 ), unaryMax, fixedLength );
  CHECK( !( predIdx < g_numIntraModes_NN[w][h] && predIdx >= 0 ), "predIdx not in range" );
  pu.intraDir[CHANNEL_TYPE_LUMA] = predIdx;
  DTRACE( g_trace_ctx, D_SYNTAX, "intra_luma_pred_mode_model() pos=(%d,%d) mode=%d\n", pu.lumaPos().x, pu.lumaPos().y, pu.intraDir[CHANNEL_TYPE_LUMA] );
}

void CABACReader::nn_subsampling_flag( CodingUnit& cu )
{
  if( !( cu.intra_NN ) )
  {
    cu.intra_NN_Use_Sampling = UChar( 0 );
    return;
  }
  if( ( cu.lwidth() == 64 ) && ( cu.lheight() == 64 ) )
  {
    cu.intra_NN_Use_Sampling = UChar( 1 );
    return;
  }
  if( !subSamplingAllowed( cu.lwidth(), cu.lheight() ) )
  {
    cu.intra_NN_Use_Sampling = UChar( 0 );
    return;
  }
  unsigned subsampling = m_BinDecoder.decodeBinEP();
  cu.intra_NN_Use_Sampling = UChar( subsampling );
}

void CABACReader::cu_diffusion_filter_idx( CodingUnit& cu )
{
  CHECK( !cu.cs->sps->getSpsNext().getDiffusionFilterEnabled(), "PDE is not enabled" );
  CHECK( !cu.Y().valid(), "luma not valid" );

  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET( STATS__CABAC_BITS__DIFFUSION_FILTER_IDX );
  const unsigned  isInter = ( CU::isInter( cu ) ? 1 : 0 );
  unsigned  numFilt = ( isInter ? cu.cs->sps->getSpsNext().getNumDiffusionFiltersInter() : cu.cs->sps->getSpsNext().getNumDiffusionFiltersIntra() );

  bool Min_Sign = true;
  if( ( cu.cs->sps->getSpsNext().getRestrDiffusionMode() >> 1 ) & 1 )
  {
    if( ( cu.Y().width == 8 ) || ( cu.Y().height == 8 ) )
    {
      Min_Sign = false;
    }
  }

  CHECK( numFilt == 0, "diffusion not allowed" );
  unsigned  ctxIdx = Ctx::DiffusionFilterIdx( isInter << 3 );  // offset = BaseIdx + [0 (intra) | 8 (inter)]
  if( isInter )
  {
    ctxIdx = Ctx::DiffusionFilterIdx( ( 1 << 3 ) + ( cu.firstPU->mergeFlag << 3 ) );  // offset = BaseIdx + [ 8 (inter)] + [0 (nonmerge) | 8 (merge)]
  }
  const bool enabled = m_BinDecoder.decodeBin( ctxIdx );
  cu.diffFilterIdx = enabled;

  if( cu.diffFilterIdx )
  {
    ctxIdx++; // [B+1]
    unsigned  nonlin = 0;
    if( numFilt > 2 )
    {
      nonlin = m_BinDecoder.decodeBin( ctxIdx );
    }

    ctxIdx += 1 + nonlin; // [B+2|B+3]
    cu.diffFilterIdx = 1 + ( nonlin << 1 );
    if( ( ( nonlin && numFilt > 3 ) || ( ( !nonlin && numFilt > 1 ) && Min_Sign ) ) )
    {
      cu.diffFilterIdx += m_BinDecoder.decodeBin( ctxIdx );
    }
  }

  DTRACE( g_trace_ctx, D_SYNTAX, "cu_diffusion_filter_idx() fidx=%d\n", cu.diffFilterIdx );
}

void CABACReader::intra_luma_pred_modes( CodingUnit &cu )
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


  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2( STATS__CABAC_BITS__INTRA_DIR_ANG, cu.lumaSize(), CHANNEL_TYPE_LUMA );

  const UInt numMPMs = cu.cs->pcv->numMPMs;

  // prev_intra_luma_pred_flag
  int numBlocks = CU::getNumPUs( cu );
  PredictionUnit *pu = cu.firstPU;
  for( int k = 0; k < numBlocks; k++ )
  {
    if( cu.cs->sps->getSpsNext().getUseIntraFTM() && !cu.mrlIdx )
    {
      unsigned FTMFlag = m_BinDecoder.decodeBin( Ctx::FtmFlag() );
      if( FTMFlag )
      {
        unsigned FTMIdx = m_BinDecoder.decodeBin( Ctx::FtmReg[0]() );
        if( FTMIdx )
        {
          unsigned B1 = 0, B2 = 0;
          B1     = m_BinDecoder.decodeBin( Ctx::FtmReg[1]( 0 ) );
          B2     = m_BinDecoder.decodeBin( Ctx::FtmReg[1]( 1 ) );
          FTMIdx = B1 * 2 + B2;
          FTMIdx++;
        }
        pu->intraDir[0] = DC_IDX;
        pu->FTMRegIdx   = ( FTMIdx + 1 );

        DTRACE( g_trace_ctx, D_SYNTAX, "intra_luma_pred_modes() idx=%d pos=(%d,%d) mode=FTM:%d\n", k, pu->lumaPos().x, pu->lumaPos().y, pu->FTMRegIdx );
      }
      else
      {
        pu->FTMRegIdx = 0;
      }
    }
    else
    {
      pu->FTMRegIdx = 0;
    }
    pu = pu->next;
  }
  pu = cu.firstPU;
  int mpmFlag[4];
  for( int k = 0; k < numBlocks; k++ )
  {
    if( pu->FTMRegIdx != 0 )
    {
      pu = pu->next;
      continue;
    }
    mpmFlag[k] = m_BinDecoder.decodeBin( Ctx::IPredMode[0]() );
    pu = pu->next;
  }

  pu = cu.firstPU;
  const bool use65Ang = cu.cs->sps->getSpsNext().getUseIntra65Ang();

  // mpm_idx / rem_intra_luma_pred_mode
  for( int k = 0; k < numBlocks; k++ )
  {
    if( pu->FTMRegIdx != 0 )
    {
      pu = pu->next;
      continue;
    }
    unsigned *mpm_pred = ( unsigned* ) alloca( numMPMs * sizeof( unsigned ) );
    PU::getIntraMPMs( *pu, mpm_pred );

    if( mpmFlag[k] )
    {
      unsigned ipred_idx = 0;
      if( use65Ang )
      {
        DecisionTree dt( g_intraLumaMpmDTT );

        dt.setCtxId( DTT_INTRA_MPM_ISGT_0, Ctx::IPredMode[0]( mpmCtx[mpm_pred[0]] ) );
        dt.setCtxId( DTT_INTRA_MPM_ISGT_1, Ctx::IPredMode[0]( mpmCtx[mpm_pred[1]] ) );
        dt.setCtxId( DTT_INTRA_MPM_ISGT_2, Ctx::IPredMode[0]( mpmCtx[mpm_pred[2]] ) );

        ipred_idx = decode_sparse_dt( dt );
      }
      else
      {
        ipred_idx = m_BinDecoder.decodeBinEP();
        if( ipred_idx )
        {
          ipred_idx += m_BinDecoder.decodeBinEP();
        }
      }
      pu->intraDir[0] = mpm_pred[ipred_idx];
    }
    else
    {
      unsigned ipred_mode = 0;

      if( use65Ang )
      {
        ipred_mode    = m_BinDecoder.decodeBinsEP( 4 );
        ipred_mode  <<= 2;
        int RealNumIntraMode = cu.cs->sps->getSpsNext().getRealNumIntraMode();
        if( ipred_mode < ( RealNumIntraMode - 8 ) )
        {
          ipred_mode += m_BinDecoder.decodeBinsEP( 2 );
        }
      }
      else
      {
        ipred_mode = m_BinDecoder.decodeBinsEP( 5 );
      }
      //postponed sorting of MPMs (only in remaining branch)
      std::sort( mpm_pred, mpm_pred + cu.cs->pcv->numMPMs );

      for( unsigned i = 0; i < cu.cs->pcv->numMPMs; i++ )
      {
        ipred_mode += use65Ang ? ( ipred_mode >= mpm_pred[i] ) : ( ipred_mode >= g_intraMode65to33AngMapping[mpm_pred[i]] );
      }

      pu->intraDir[0] = use65Ang ? ipred_mode : g_intraMode33to65AngMapping[ipred_mode];
    }

    DTRACE( g_trace_ctx, D_SYNTAX, "intra_luma_pred_modes() idx=%d pos=(%d,%d) mode=%d\n", k, pu->lumaPos().x, pu->lumaPos().y, pu->intraDir[0] );
    pu = pu->next;
  }
}

void CABACReader::intra_chroma_pred_modes( CodingUnit& cu )
{
  if( cu.chromaFormat == CHROMA_400 || ( CS::isDualITree( *cu.cs ) && cu.chType == CHANNEL_TYPE_LUMA ) )
  {
    return;
  }

  int numBlocks = enable4ChromaPUsInIntraNxNCU( cu.chromaFormat ) ? CU::getNumPUs( cu ) : 1;

  PredictionUnit *pu = cu.firstPU;

  for( int k = 0; k < numBlocks; k++ )
  {
    CHECK( pu->cu != &cu, "Inkonsistent PU-CU mapping" );
    intra_chroma_pred_mode( *pu );
    pu = pu->next;
  }
}

bool CABACReader::intra_chroma_lmc_mode( PredictionUnit& pu )
{
  if ( pu.cs->sps->getSpsNext().getUseMDMS() )
  {
    if ( m_BinDecoder.decodeBin( Ctx::IPredMode[1]( 0 ) ) == 0 )
    {
      unsigned ctxId = 6;
      if ( PU::isMMLMEnabled( pu ) )
      {
        if ( m_BinDecoder.decodeBin( Ctx::IPredMode[1]( ctxId++ ) ) == 1 )
        {
          pu.intraDir[1] = MMLM_CHROMA_IDX;
          return true;
        }
      }
      if ( PU::isMFLMEnabled( pu ) )
      {
        if ( m_BinDecoder.decodeBin( Ctx::IPredMode[1]( ctxId++ ) ) == 1 )
        {
          pu.intraDir[1] = LM_CHROMA_IDX;
          return true;
        }
        int candId = m_BinDecoder.decodeBin( Ctx::IPredMode[1]( ctxId++ ) ) << 1;
        candId    += m_BinDecoder.decodeBin( Ctx::IPredMode[1]( ctxId++ ) );
        pu.intraDir[1] = LM_CHROMA_F1_IDX + candId;
        return true;
      }
      pu.intraDir[1] = LM_CHROMA_IDX;
      return true;
    }
  }
  else
  {
    int lmModeList[10];
    int maxSymbol = PU::getLMSymbolList(pu, lmModeList);
    int symbol    = unary_max_symbol( Ctx::IPredMode[1]( 2 ), Ctx::IPredMode[1]( 3 ), maxSymbol - 1 );
    if ( lmModeList[ symbol ] != -1 )
    {
      pu.intraDir[1] = lmModeList[ symbol ];
      return true;
    }
  }

  return false;
}

void CABACReader::intra_chroma_pred_mode( PredictionUnit& pu )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2( STATS__CABAC_BITS__INTRA_DIR_ANG, pu.cu->blocks[pu.chType].lumaSize(), CHANNEL_TYPE_CHROMA );

  // DM chroma index
  if( pu.cs->sps->getSpsNext().getUseMDMS() )
  {
  }
  else
  {
    if( m_BinDecoder.decodeBin( Ctx::IPredMode[1]( 1 ) ) == 0 )
    {
      pu.intraDir[1] = DM_CHROMA_IDX;
      return;
    }
  }

  // LM chroma mode
  if( pu.cs->sps->getSpsNext().getUseLMChroma() )
  {
    if( intra_chroma_lmc_mode( pu ) )
    {
      return;
    }
  }

  // chroma candidate index
  unsigned candId = 0;
  if( pu.cs->sps->getSpsNext().getUseMDMS() )
  {
    const unsigned lastId = NUM_DM_MODES - 1;

    unsigned ctxId = 1;
    while( candId < lastId && m_BinDecoder.decodeBin( Ctx::IPredMode[1]( ctxId++ ) ) )
    {
      candId += 1;
    }
    candId += NUM_LMC_MODE;
  }
  else
  {
    candId = m_BinDecoder.decodeBinsEP( 2 );
  }

  unsigned chromaCandModes[ NUM_CHROMA_MODE ];
  PU::getIntraChromaCandModes( pu, chromaCandModes );

  CHECK( candId >= NUM_CHROMA_MODE, "Chroma prediction mode index out of bounds" );
  CHECK( PU::isLMCMode( chromaCandModes[ candId ] ), "The intra dir cannot be LM_CHROMA for this path" );
  CHECK( chromaCandModes[ candId ] == DM_CHROMA_IDX, "The intra dir cannot be DM_CHROMA for this path" );

  pu.intraDir[1] = chromaCandModes[ candId ];
}

void CABACReader::cu_residual( CodingUnit& cu, Partitioner &partitioner, CUCtx& cuCtx )
{
  if( CU::isInter( cu ) )
  {
    PredictionUnit& pu = *cu.firstPU;
    bool testMergeDF = true;

    if( cu.slice->getSPS()->getSpsNext().getDiffusionFilterMode() >> 2 )
    {
      if( !( pu.frucMrgMode == FRUC_MERGE_OFF || cu.diffFilterIdx == 0 ) )
      {
        testMergeDF = false;
      }
    }
    else
    {
      CHECK( pu.mergeFlag && pu.frucMrgMode == FRUC_MERGE_OFF && cu.diffFilterIdx != 0, "Diffusion filter idx set for non-FRUC merge mode" );
    }
    if( !( ( cu.cs->pcv->noRQT || cu.partSize == SIZE_2Nx2N ) && pu.mergeFlag && cu.diffFilterIdx == 0
      && pu.mergeRefPicList == REF_PIC_LIST_X
      && pu.numMergedAddHyps == pu.addHypData.size()
      && testMergeDF ) )
    {
      rqt_root_cbf( cu );
    }
    else
    {
      cu.rootCbf = true;
    }
    if( !cu.rootCbf )
    {
      TransformUnit& tu = cu.cs->addTU(cu, partitioner.chType);
      tu.depth = 0;
      for( unsigned c = 0; c < tu.blocks.size(); c++ )
      {
        tu.cbf[c]             = 0;
        ComponentID   compID  = ComponentID(c);
        tu.getCoeffs( compID ).fill( 0 );
        tu.getPcmbuf( compID ).fill( 0 );
      }
      return;
    }
  }
  mode_1d_partitions( cu );

  cuCtx.quadtreeTULog2MinSizeInCU = CU::getQuadtreeTULog2MinSizeInCU( cu );
  ChromaCbfs chromaCbfs;

  if( cu.mode1dPartitions )
  {
    TU1dPartitioner partitionerFor1dTus( partitioner );
    transform_tree( *cu.cs, partitionerFor1dTus, cuCtx, chromaCbfs );
  }
  else
  {
    transform_tree( *cu.cs, partitioner, cuCtx, chromaCbfs );
  }

  residual_nsst_mode( cu );

#if !HM_EMT_NSST_AS_IN_JEM
  cu_emt_noqrt_idx( cu );
#endif
}

#if !HM_EMT_NSST_AS_IN_JEM
void CABACReader::cu_emt_noqrt_idx( CodingUnit& cu )
{
  if( !cu.cs->pcv->noRQT || !isLuma( cu.chType ) || cu.nsstIdx != 0 || !( cu.cs->sps->getSpsNext().getUseIntraEMT() || cu.cs->sps->getSpsNext().getUseInterEMT() ) || !cu.firstTU->cbf[COMPONENT_Y] || cu.firstTU->transformSkip[COMPONENT_Y] )
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

void CABACReader::rqt_root_cbf( CodingUnit& cu )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET( STATS__CABAC_BITS__QT_ROOT_CBF );

  cu.rootCbf = ( m_BinDecoder.decodeBin( Ctx::QtRootCbf() ) );

  DTRACE( g_trace_ctx, D_SYNTAX, "rqt_root_cbf() ctx=0 root_cbf=%d pos=(%d,%d)\n", cu.rootCbf ? 1 : 0, cu.lumaPos().x, cu.lumaPos().y );
}


bool CABACReader::end_of_ctu( CodingUnit& cu, CUCtx& cuCtx )
{
  const SPS     &sps   = *cu.cs->sps;
  const Position rbPos = recalcPosition( cu.chromaFormat, cu.chType, CHANNEL_TYPE_LUMA, cu.blocks[cu.chType].bottomRight().offset( 1, 1 ) );

  if ( ( ( rbPos.x & cu.cs->pcv->maxCUWidthMask  ) == 0 || rbPos.x == sps.getPicWidthInLumaSamples () )
    && ( ( rbPos.y & cu.cs->pcv->maxCUHeightMask ) == 0 || rbPos.y == sps.getPicHeightInLumaSamples() )
    && ( !CS::isDualITree( *cu.cs ) || cu.chromaFormat == CHROMA_400 || isChroma( cu.chType ) )
      )
  {
    cuCtx.isDQPCoded = ( cu.cs->pps->getUseDQP() && !cuCtx.isDQPCoded );

    return terminating_bit();
  }

  return false;
}



//================================================================================
//  clause 7.3.8.6
//--------------------------------------------------------------------------------
//    void  prediction_unit ( pu, mrgCtx );
//    void  merge_flag      ( pu );
//    void  merge_data      ( pu, mrgCtx );
//    void  merge_idx       ( pu );
//    void  inter_pred_idc  ( pu );
//    void  ref_idx         ( pu, refList );
//    void  mvp_flag        ( pu, refList );
//================================================================================

void CABACReader::prediction_unit( PredictionUnit& pu, MergeCtx& mrgCtx )
{
  if( pu.cu->skip )
  {
    pu.mergeFlag = true;
  }
  else
  {
    merge_flag( pu );
  }
  if( pu.mergeFlag )
  {
    fruc_mrg_mode( pu );
    affine_flag  ( *pu.cu );
    merge_data   ( pu );
    if( !pu.cu->skip && pu.cu->partSize == SIZE_2Nx2N )
    {
      CHECK( pu.cu->imv != 0, "Multi Hyp: pu.cu->imv != 0" );
      mh_pred_data( pu );
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
        Mv affLT, affRT;
        mvd_coding( affLT );
        mvd_coding( affRT );

        PU::setAllAffineMvd( pu.getMotionBuf(), affLT, affRT, REF_PIC_LIST_0, pu.cs->pcv->rectCUs );
      }
      else
      {
        mvd_coding( pu.mvd[REF_PIC_LIST_0] );
      }
      mvp_flag    ( pu, REF_PIC_LIST_0 );
    }
    else if( pu.cu->affine )
    {
      PU::setAllAffineMv( pu, Mv(), Mv(), Mv(), REF_PIC_LIST_0 ); // done in JEM, but maybe unnecessary
    }
    if( pu.interDir != 1 /* PRED_L0 */ )
    {
      ref_idx     ( pu, REF_PIC_LIST_1 );
      if( pu.cu->cs->slice->getMvdL1ZeroFlag() && pu.interDir == 3 /* PRED_BI */ )
      {
        pu.mvd[ REF_PIC_LIST_1 ] = Mv();
      }
      else if( pu.cu->affine )
      {
        Mv affLT, affRT;
        mvd_coding( affLT );
        mvd_coding( affRT );

        PU::setAllAffineMvd( pu.getMotionBuf(), affLT, affRT, REF_PIC_LIST_1, pu.cs->pcv->rectCUs );
      }
      else
      {
        mvd_coding( pu.mvd[REF_PIC_LIST_1] );
      }
      mvp_flag    ( pu, REF_PIC_LIST_1 );
    }
    else if( pu.cu->affine )
    {
      PU::setAllAffineMv( pu, Mv(), Mv(), Mv(), REF_PIC_LIST_1 ); // done in JEM, but maybe not necessary
    }
    mh_pred_data( pu );
  }
  CHECK(PU::isBipredRestriction(pu) && !pu.addHypData.empty(), "additional hypotheseis signalled in restricted bi-pred mode");
  if( pu.interDir == 3 /* PRED_BI */ && PU::isBipredRestriction(pu) )
  {
    pu.mv    [REF_PIC_LIST_1] = Mv(0, 0);
    pu.refIdx[REF_PIC_LIST_1] = -1;
    pu.interDir               =  1;
  }

  PU::spanMotionInfo( pu, mrgCtx );
}

void CABACReader::affine_flag( CodingUnit& cu )
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

  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET( STATS__CABAC_BITS__AFFINE_FLAG );

  unsigned ctxId = DeriveCtx::CtxAffineFlag( cu );
  cu.affine = m_BinDecoder.decodeBin( Ctx::AffineFlag( ctxId ) );

  DTRACE( g_trace_ctx, D_SYNTAX, "affine_flag() affine=%d ctx=%d pos=(%d,%d)\n", cu.affine ? 1 : 0, ctxId, cu.Y().x, cu.Y().y );
}

void CABACReader::merge_flag( PredictionUnit& pu )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET( STATS__CABAC_BITS__MERGE_FLAG );

  pu.mergeFlag = ( m_BinDecoder.decodeBin( Ctx::MergeFlag() ) );

  DTRACE( g_trace_ctx, D_SYNTAX, "merge_flag() merge=%d pos=(%d,%d) size=%dx%d\n", pu.mergeFlag ? 1 : 0, pu.lumaPos().x, pu.lumaPos().y, pu.lumaSize().width, pu.lumaSize().height );
}


void CABACReader::merge_data( PredictionUnit& pu )
{
  if( pu.frucMrgMode || pu.cu->affine )
  {
    return;
  }

  merge_ref_pic_list( pu );
  merge_idx( pu );
}

void CABACReader::merge_ref_pic_list( PredictionUnit& pu )
{
  CHECK( pu.mergeRefPicList != REF_PIC_LIST_X, "pu is bad initialized" );
  const bool available = pu.cs->slice->getSPS()->getSpsNext().getUseRestrictedMerge();
  if( !available || pu.cu->skip || pu.cu->partSize != SIZE_2Nx2N || !pu.cs->slice->isInterB() )
    return;

  const bool restrictedMerge = m_BinDecoder.decodeBin( Ctx::MergeList() ) == 1;
  if( restrictedMerge )
  {
    pu.mergeRefPicList = RefPicList( m_BinDecoder.decodeBin( Ctx::MergeList(1) ) );
  }
}

void CABACReader::merge_idx( PredictionUnit& pu )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET( STATS__CABAC_BITS__MERGE_INDEX );

  int numCandminus1 = int( pu.cs->slice->getMaxNumMergeCand() ) - 1;
  pu.mergeIdx       = 0;
  if( numCandminus1 > 0 )
  {
    if( m_BinDecoder.decodeBin( Ctx::MergeIdx() ) )
    {
      bool useExtCtx = pu.cs->sps->getSpsNext().getUseSubPuMvp();
      pu.mergeIdx++;
      for( ; pu.mergeIdx < numCandminus1; pu.mergeIdx++ )
      {
        if( useExtCtx )
        {
          if( !m_BinDecoder.decodeBin( Ctx::MergeIdx( std::min<int>( pu.mergeIdx, NUM_MERGE_IDX_EXT_CTX - 1 ) ) ) )
          {
            break;
          }
        }
        else
        {
          if( !m_BinDecoder.decodeBinEP() )
          {
            break;
          }
        }
      }
    }
  }
  DTRACE( g_trace_ctx, D_SYNTAX, "merge_idx() merge_idx=%d\n", pu.mergeIdx );
}


void CABACReader::inter_pred_idc( PredictionUnit& pu )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET( STATS__CABAC_BITS__INTER_DIR );

  if( pu.cs->slice->isInterP() )
  {
    pu.interDir = 1;
    return;
  }
  if( pu.cu->partSize == SIZE_2Nx2N || pu.cs->sps->getSpsNext().getUseSubPuMvp() || pu.cu->lumaSize().width != 8 )
  {
    unsigned ctxId = DeriveCtx::CtxInterDir(pu);
    if( m_BinDecoder.decodeBin( Ctx::InterDir(ctxId) ) )
    {
      DTRACE( g_trace_ctx, D_SYNTAX, "inter_pred_idc() ctx=%d value=%d pos=(%d,%d)\n", ctxId, 3, pu.lumaPos().x, pu.lumaPos().y );
      pu.interDir = 3;
      return;
    }
  }
  if( m_BinDecoder.decodeBin( Ctx::InterDir(4) ) )
  {
    DTRACE( g_trace_ctx, D_SYNTAX, "inter_pred_idc() ctx=4 value=%d pos=(%d,%d)\n", 2, pu.lumaPos().x, pu.lumaPos().y );
    pu.interDir = 2;
    return;
  }
  DTRACE( g_trace_ctx, D_SYNTAX, "inter_pred_idc() ctx=4 value=%d pos=(%d,%d)\n", 1, pu.lumaPos().x, pu.lumaPos().y );
  pu.interDir = 1;
  return;
}


void CABACReader::ref_idx( PredictionUnit &pu, RefPicList eRefList )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET( STATS__CABAC_BITS__REF_FRM_IDX );

  int numRef  = pu.cs->slice->getNumRefIdx(eRefList);
  if( numRef <= 1 || !m_BinDecoder.decodeBin( Ctx::RefPic() ) )
  {
    if( numRef > 1 )
    {
      DTRACE( g_trace_ctx, D_SYNTAX, "ref_idx() value=%d pos=(%d,%d)\n", 0, pu.lumaPos().x, pu.lumaPos().y );
    }
    pu.refIdx[eRefList] = 0;
    return;
  }
  if( numRef <= 2 || !m_BinDecoder.decodeBin( Ctx::RefPic(1) ) )
  {
    DTRACE( g_trace_ctx, D_SYNTAX, "ref_idx() value=%d pos=(%d,%d)\n", 1, pu.lumaPos().x, pu.lumaPos().y );
    pu.refIdx[eRefList] = 1;
    return;
  }
  for( int idx = 3; ; idx++ )
  {
    if( numRef <= idx || !m_BinDecoder.decodeBinEP() )
    {
      pu.refIdx[eRefList] = (signed char)( idx - 1 );
      DTRACE( g_trace_ctx, D_SYNTAX, "ref_idx() value=%d pos=(%d,%d)\n", idx-1, pu.lumaPos().x, pu.lumaPos().y );
      return;
    }
  }
}

int CABACReader::ref_idx_mh( const int numRef )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET( STATS__CABAC_BITS__REF_FRM_IDX );

  if( numRef <= 1 || !m_BinDecoder.decodeBin( Ctx::MHRefPic() ) )
  {
    return 0;
  }
  if( numRef <= 2 || !m_BinDecoder.decodeBin( Ctx::MHRefPic(1) ) )
  {
    return 1;
  }
  for( int idx = 3; ; idx++ )
  {
    if( numRef <= idx || !m_BinDecoder.decodeBinEP() )
    {
      return( idx - 1 );
    }
  }
}


void CABACReader::mvp_flag( PredictionUnit& pu, RefPicList eRefList )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET( STATS__CABAC_BITS__MVP_IDX );

  unsigned mvp_idx = m_BinDecoder.decodeBin( Ctx::MVPIdx() );
  DTRACE( g_trace_ctx, D_SYNTAX, "mvp_flag() value=%d pos=(%d,%d)\n", mvp_idx, pu.lumaPos().x, pu.lumaPos().y );
  pu.mvpIdx [eRefList] = mvp_idx;
  DTRACE( g_trace_ctx, D_SYNTAX, "mvpIdx(refList:%d)=%d\n", eRefList, mvp_idx );
}

void CABACReader::mh_pred_data( PredictionUnit& pu )
{
  CHECK(!pu.addHypData.empty(), "Multi Hyp: !pu.addHypData.empty()");
  if( !pu.cs->sps->getSpsNext().getUseInterMultiHyp() || !pu.cs->slice->isInterB() )
    return;
  const int numMHRef = int(pu.cs->slice->getMultiHypRefPicList().size());
  CHECK(numMHRef <= 0, "Multi Hyp: numMHRef <= 0");
  int hypIdx = 0;
  //RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET( STATS__CABAC_BITS__ADDHYP_FLAG );
  const size_t maxNumAddHyps = pu.cs->sps->getSpsNext().getMaxNumAddHyps();
  while( pu.addHypData.size() < maxNumAddHyps &&
         m_BinDecoder.decodeBin( Ctx::MultiHypothesisFlag(hypIdx) ) == 1 )
  {
    if( hypIdx < 1 )
      hypIdx++;
    MultiHypPredictionData mhData;
    mhData.refIdx = ref_idx_mh( numMHRef );
    CHECK(mhData.refIdx < 0, "Multi Hyp: mhData.refIdx < 0");
    CHECK(mhData.refIdx >= numMHRef, "Multi Hyp: mhData.refIdx >= numMHRef");
    mvd_coding( mhData.mvd );
    //RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET( STATS__CABAC_BITS__MVP_IDX );
    mhData.mvpIdx = m_BinDecoder.decodeBin( Ctx::MVPIdx() );
    //RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET( STATS__CABAC_BITS__WEIGHT_INDEX );
    mhData.weightIdx = unary_max_symbol( Ctx::MHWeight(), Ctx::MHWeight(1), pu.cs->sps->getSpsNext().getNumAddHypWeights() - 1 );
    pu.addHypData.push_back( mhData );
    //RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET( STATS__CABAC_BITS__ADDHYP_FLAG );
  }
}

void CABACReader::fruc_mrg_mode( PredictionUnit& pu )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET( STATS__CABAC_BITS__OTHER );

  if( !pu.cs->slice->getSPS()->getSpsNext().getUseFRUCMrgMode() )
    return;

  unsigned fruc_mode  = FRUC_MERGE_OFF;
  unsigned flag_idx   = DeriveCtx::CtxFrucFlag( pu );

  if( m_BinDecoder.decodeBin( Ctx::FrucFlag(flag_idx) ) )
  {
    if( pu.cs->slice->isInterP() )
    {
      fruc_mode = FRUC_MERGE_TEMPLATE;
    }
    else
    {
      unsigned mode_idx   = DeriveCtx::CtxFrucMode( pu );
      unsigned second_bin = m_BinDecoder.decodeBin( Ctx::FrucMode(mode_idx) );
      fruc_mode = second_bin ? FRUC_MERGE_BILATERALMV : FRUC_MERGE_TEMPLATE;
    }

    merge_ref_pic_list( pu );
  }

  pu.frucMrgMode = fruc_mode;

  DTRACE( g_trace_ctx, D_SYNTAX, "fruc_mrg_mode() fruc_mode=%d pos=(%d,%d) size: %dx%d\n", fruc_mode, pu.Y().x, pu.Y().y, pu.lumaSize().width, pu.lumaSize().height );
}

//================================================================================
//  clause 7.3.8.7
//--------------------------------------------------------------------------------
//    void  pcm_samples( tu )
//================================================================================

void CABACReader::pcm_samples( TransformUnit& tu )
{
  CHECK( !tu.cu->ipcm, "pcm mode expected" );

  const SPS&        sps       = *tu.cu->cs->sps;
  const ComponentID maxCompId = ( tu.chromaFormat == CHROMA_400 ? COMPONENT_Y : COMPONENT_Cr );
  tu.depth                    = 0;
  for( ComponentID compID = COMPONENT_Y; compID <= maxCompId; compID = ComponentID(compID+1) )
  {
    PelBuf          samples     = tu.getPcmbuf( compID );
    const unsigned  sampleBits  = sps.getPCMBitDepth( toChannelType(compID) );
    for( unsigned y = 0; y < samples.height; y++ )
    {
      for( unsigned x = 0; x < samples.width; x++ )
      {
        samples.at(x, y) = m_BinDecoder.decodeBinsPCM( sampleBits );
      }
    }
  }
  m_BinDecoder.start();
}





//================================================================================
//  clause 7.3.8.8
//--------------------------------------------------------------------------------
//    void  transform_tree      ( cs, area, cuCtx, chromaCbfs )
//    bool  split_transform_flag( depth )
//    bool  cbf_comp            ( area, depth )
//================================================================================

void CABACReader::transform_tree( CodingStructure &cs, Partitioner &partitioner, CUCtx& cuCtx, ChromaCbfs& chromaCbfs )
{
  const UnitArea& area          = partitioner.currArea();


  CodingUnit&     cu            = *cs.getCU( area.blocks[partitioner.chType], partitioner.chType );
  const unsigned  trDepth       = partitioner.currTrDepth;
  const SPS&      sps           = *cs.sps;
  const unsigned  log2TrafoSize = g_aucLog2[area.lumaSize().width];

  // split_transform_flag
  bool split = false;
  if( cu.cs->pcv->noRQT )
  {
    split = false;
  }
  else if( CU::isIntra( cu ) && cu.partSize == SIZE_NxN && trDepth == 0 )
  {
    split = true;
  }
  else if( sps.getQuadtreeTUMaxDepthInter() == 1 && CU::isInter( cu ) && cu.partSize != SIZE_2Nx2N && trDepth == 0 )
  {
    split = ( log2TrafoSize > cuCtx.quadtreeTULog2MinSizeInCU );
  }
  else if( log2TrafoSize > sps.getQuadtreeTULog2MaxSize() )
  {
    split = true;
  }
  else if( log2TrafoSize == sps.getQuadtreeTULog2MinSize() )
  {
    split = false;
  }
  else if( log2TrafoSize == cuCtx.quadtreeTULog2MinSizeInCU )
  {
    split = false;
  }
  else
  {
    CHECK( log2TrafoSize <= cuCtx.quadtreeTULog2MinSizeInCU, "block cannot be split in multiple TUs" );

    if( sps.getSpsNext().nextToolsEnabled() )
    {
      split = split_transform_flag( sps.getQuadtreeTULog2MaxSize() - log2TrafoSize );
    }
    else
    {
      split = split_transform_flag( 5 - log2TrafoSize );
    }
  }

  if( CU::isIntra( cu ) && cu.mode1dPartitions && ( ( CU::divideTuInRows( cu ) && area.lumaSize().height != 1 ) || ( !CU::divideTuInRows( cu ) && area.lumaSize().width != 1 ) ) )
  {
    split = true;
  }

  // cbf_cb & cbf_cr
  if( area.chromaFormat != CHROMA_400 && area.blocks[COMPONENT_Cb].valid() && ( !CS::isDualITree( cs ) || partitioner.chType == CHANNEL_TYPE_CHROMA ) )
  {
    const bool firstCbfOfCU = ( trDepth == 0 );
    const bool allQuadrants = TU::isProcessingAllQuadrants(area);
    if( firstCbfOfCU || allQuadrants )
    {
      {
        if( chromaCbfs.Cb )
        {
          chromaCbfs.Cb &= cbf_comp( cs, area.blocks[COMPONENT_Cb], trDepth );
        }
        if( chromaCbfs.Cr )
        {
          chromaCbfs.Cr &= cbf_comp(cs, area.blocks[COMPONENT_Cr], trDepth, false, chromaCbfs.Cb);
        }
      }
    }
  }
  else if( CS::isDualITree( cs ) )
  {
    chromaCbfs = ChromaCbfs( false );
  }

  if( split )
  {
    if( cu.mode1dPartitions )
    {
      ComponentID compID = partitioner.chType == CHANNEL_TYPE_LUMA ? COMPONENT_Y : COMPONENT_Cb;
      partitioner.splitCurrArea( CU::select1dPartitionType( cu, CS::isDualITree( cs ) ? compID : COMPONENT_Y ), cs );
    }
    else
    {
      if( trDepth == 0 ) emt_cu_flag( cu );

      partitioner.splitCurrArea( TU_QUAD_SPLIT, cs );
    }

    do
    {
      ChromaCbfs subCbfs = chromaCbfs;
      transform_tree( cs, partitioner, cuCtx, subCbfs );
    } while( partitioner.nextPart( cs ) );

    partitioner.exitCurrSplit();

    const UnitArea &currArea  = partitioner.currArea();
    const unsigned  currDepth = partitioner.currTrDepth;
    const unsigned numTBlocks = getNumberValidTBlocks( *cs.pcv );

    unsigned        compCbf[3] = { 0, 0, 0 };

    for( auto &currTU : cs.traverseTUs( currArea, partitioner.chType ) )
    {
      for( unsigned ch = 0; ch < numTBlocks; ch++ )
      {
        compCbf[ch] |= ( TU::getCbfAtDepth( currTU, ComponentID( ch ), currDepth + 1 ) ? 1 : 0 );
      }
    }

    {

      for( auto &currTU : cs.traverseTUs( currArea, partitioner.chType ) )
      {
        TU::setCbfAtDepth( currTU, COMPONENT_Y, currDepth, compCbf[COMPONENT_Y] );
        if( currArea.chromaFormat != CHROMA_400 )
        {
          TU::setCbfAtDepth( currTU, COMPONENT_Cb, currDepth, compCbf[COMPONENT_Cb] );
          TU::setCbfAtDepth( currTU, COMPONENT_Cr, currDepth, compCbf[COMPONENT_Cr] );
        }
      }
    }
  }
  else
  {
    TransformUnit &tu = cs.addTU( CS::getArea( cs, area, partitioner.chType ), partitioner.chType );
    unsigned numBlocks = ::getNumberValidTBlocks( *cs.pcv );

    for( unsigned compID = COMPONENT_Y; compID < numBlocks; compID++ )
    {
      if( tu.blocks[compID].valid() )
      {
        tu.getCoeffs( ComponentID( compID ) ).fill( 0 );
        tu.getPcmbuf( ComponentID( compID ) ).fill( 0 );
      }
    }
    tu.depth = trDepth;
    DTRACE( g_trace_ctx, D_SYNTAX, "transform_unit() pos=(%d,%d) depth=%d trDepth=%d\n", tu.lumaPos().x, tu.lumaPos().y, cu.depth, partitioner.currTrDepth );

    if( !isChroma( partitioner.chType ) )
    {
      if( !CU::isIntra( cu ) && trDepth == 0 && !chromaCbfs.sigChroma( area.chromaFormat ) )
      {
        TU::setCbfAtDepth( tu, COMPONENT_Y, trDepth, 1 );
      }
      else
      {
        bool previousCbf = false;
        if( cu.mode1dPartitions )
        {
          previousCbf = TU::getPrevTuCbfAtDepth( tu, trDepth, COMPONENT_Y );
        }
        Bool cbfY = cbf_comp( cs, tu.Y(), trDepth, cu.mode1dPartitions, previousCbf );
        TU::setCbfAtDepth( tu, COMPONENT_Y, trDepth, ( cbfY ? 1 : 0 ) );
      }
    }
    if( area.chromaFormat != CHROMA_400 )
    {
      TU::setCbfAtDepth( tu, COMPONENT_Cb, trDepth, ( chromaCbfs.Cb ? 1 : 0 ) );
      TU::setCbfAtDepth( tu, COMPONENT_Cr, trDepth, ( chromaCbfs.Cr ? 1 : 0 ) );
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

bool CABACReader::split_transform_flag( unsigned depth )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE( STATS__CABAC_BITS__TRANSFORM_SUBDIV_FLAG, Size( 1 << ( 5 - depth ), 1 << ( 5 - depth ) ) );

  unsigned split = m_BinDecoder.decodeBin( Ctx::TransSubdivFlag( depth ) );
  DTRACE( g_trace_ctx, D_SYNTAX, "split_transform_flag() ctx=%d split=%d\n", depth, split );
  return ( split );
}

bool CABACReader::cbf_comp( CodingStructure& cs, const CompArea& area, unsigned depth, bool useMode1dPartitions, bool previousCbf )
{
  if( area.compID == COMPONENT_Cr && !useMode1dPartitions && cs.sps->getSpsNext().getAltResiCompId() == 2 )
  {
    return Bool( m_BinDecoder.decodeBin( Ctx::QtCbf[2]( previousCbf ? 3 : 2 ) ) );
  }

  const unsigned  ctxId   = DeriveCtx::CtxQtCbf( area.compID, depth, useMode1dPartitions && canUse1dPartitions( area.compID ), previousCbf );
  const CtxSet&   ctxSet  = Ctx::QtCbf[toChannelType( area.compID ) << ( cs.sps->getSpsNext().getAltResiCompId() == 2 ? 1 : 0 )];

  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2( STATS__CABAC_BITS__QT_CBF, area.size(), area.compID );

  if( !canUse1dPartitions( area.compID ) || !useMode1dPartitions )
  {
    const unsigned  cbf = m_BinDecoder.decodeBin( ctxSet( ctxId ) );

    DTRACE( g_trace_ctx, D_SYNTAX, "cbf_comp() etype=%d pos=(%d,%d) ctx=%d cbf=%d\n", area.compID, area.x, area.y, ctxId, cbf );
    return cbf;
  }
  else
  {
    //const unsigned  cbf = m_BinDecoder.decodeBin(ctxSet(2 + int(previousCbf)));
    const unsigned  cbf = m_BinDecoder.decodeBin( ctxSet( ctxId ) );

    DTRACE( g_trace_ctx, D_SYNTAX, "cbf_comp() etype=%d pos=(%d,%d) ctx=%d cbf=%d\n", area.compID, area.x, area.y, previousCbf, cbf );
    return cbf;
  }
}





//================================================================================
//  clause 7.3.8.9
//--------------------------------------------------------------------------------
//    void  mvd_coding( pu, refList )
//================================================================================

void CABACReader::mvd_coding( Mv &rMvd )
{
#if RExt__DECODER_DEBUG_BIT_STATISTICS
  CodingStatisticsClassType ctype_mvd    ( STATS__CABAC_BITS__MVD );
  CodingStatisticsClassType ctype_mvd_ep ( STATS__CABAC_BITS__MVD_EP );
#endif

  RExt__DECODER_DEBUG_BIT_STATISTICS_SET( ctype_mvd );

  // abs_mvd_greater0_flag[ 0 | 1 ]
  int horAbs = (int)m_BinDecoder.decodeBin(Ctx::Mvd());
  int verAbs = (int)m_BinDecoder.decodeBin(Ctx::Mvd());

  // abs_mvd_greater1_flag[ 0 | 1 ]
  if (horAbs)
  {
    horAbs += (int)m_BinDecoder.decodeBin(Ctx::Mvd(1));
  }
  if (verAbs)
  {
    verAbs += (int)m_BinDecoder.decodeBin(Ctx::Mvd(1));
  }

  RExt__DECODER_DEBUG_BIT_STATISTICS_SET( ctype_mvd_ep );

  // abs_mvd_minus2[ 0 | 1 ] and mvd_sign_flag[ 0 | 1 ]
  if (horAbs)
  {
    if (horAbs > 1)
    {
      horAbs += exp_golomb_eqprob(1 );
    }
    if (m_BinDecoder.decodeBinEP())
    {
      horAbs = -horAbs;
    }
  }
  if (verAbs)
  {
    if (verAbs > 1)
    {
      verAbs += exp_golomb_eqprob(1 );
    }
    if (m_BinDecoder.decodeBinEP())
    {
      verAbs = -verAbs;
    }
  }
  rMvd = Mv(horAbs, verAbs);
}


//================================================================================
//  clause 7.3.8.10
//--------------------------------------------------------------------------------
//    void  transform_unit      ( tu, cuCtx, chromaCbfs )
//    void  cu_qp_delta         ( cu )
//    void  cu_chroma_qp_offset ( cu )
//================================================================================

void CABACReader::transform_unit( TransformUnit& tu, CUCtx& cuCtx, ChromaCbfs& chromaCbfs )
{
  CodingUnit& cu         = *tu.cu;
  bool        lumaOnly   = ( cu.chromaFormat == CHROMA_400 || !tu.blocks[COMPONENT_Cb].valid() );
  bool        cbfLuma    = ( tu.cbf[ COMPONENT_Y ] != 0 );
  bool        cbfChroma  = ( cu.chromaFormat == CHROMA_400 ? false : ( chromaCbfs.Cb || chromaCbfs.Cr ) );


#if THRESHOLDING
#if THR_ALLOW_ZERO_CBF
  thresholding( tu );
#endif
#endif

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
        if( tu.cbf[ compID ] )
        {
          residual_coding( tu, compID );
        }
      }
    }
  }
}


void CABACReader::cu_qp_delta( CodingUnit& cu, int predQP, SChar& qp )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET( STATS__CABAC_BITS__DELTA_QP_EP );

  CHECK( predQP == std::numeric_limits<int>::max(), "Invalid predicted QP" );
  int qpY = predQP;
  int DQp = (cu.cs->sps->getSpsNext().getUseNextDQP() && cu.cs->sps->getSpsNext().getSkipDQPinOddPOCs() && (cu.cs->picture->getPOC() & 1) && cu.cs->slice->isInterB())
            ? 0 // don't read any symbols, assume a delta-QP value of zero in B slices with an odd POC
            : unary_max_symbol (Ctx::DeltaQP(), Ctx::DeltaQP(1), CU_DQP_TU_CMAX);
  if( DQp >= CU_DQP_TU_CMAX )
  {
    DQp += exp_golomb_eqprob( CU_DQP_EG_k  );
  }
  if( DQp > 0 )
  {
    if( m_BinDecoder.decodeBinEP( ) )
    {
      DQp = -DQp;
    }
    int     qpBdOffsetY = cu.cs->sps->getQpBDOffset( CHANNEL_TYPE_LUMA );
    qpY = ( ( predQP + DQp + 52 + 2*qpBdOffsetY ) % (52 + qpBdOffsetY) ) - qpBdOffsetY;
  }
  qp = (SChar)qpY;

  DTRACE( g_trace_ctx, D_DQP, "x=%d, y=%d, d=%d, pred_qp=%d, DQp=%d, qp=%d\n", cu.blocks[cu.chType].lumaPos().x, cu.blocks[cu.chType].lumaPos().y, cu.qtDepth, predQP, DQp, qp );
}


void CABACReader::cu_chroma_qp_offset( CodingUnit& cu )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2( STATS__CABAC_BITS__CHROMA_QP_ADJUSTMENT, cu.blocks[cu.chType].lumaSize(), CHANNEL_TYPE_CHROMA );

  // cu_chroma_qp_offset_flag
  int       length  = cu.cs->pps->getPpsRangeExtension().getChromaQpOffsetListLen();
  unsigned  qpAdj   = m_BinDecoder.decodeBin( Ctx::ChromaQpAdjFlag() );
  if( qpAdj && length > 1 )
  {
    // cu_chroma_qp_offset_idx
    qpAdj += unary_max_symbol( Ctx::ChromaQpAdjIdc(), Ctx::ChromaQpAdjIdc(), length-1 );
  }
  /* NB, symbol = 0 if outer flag is not set,
   *              1 if outer flag is set and there is no inner flag
   *              1+ otherwise */
  cu.chromaQpAdj = cu.cs->chromaQpAdj = qpAdj;
}





//================================================================================
//  clause 7.3.8.11
//--------------------------------------------------------------------------------
//    void        residual_coding         ( tu, compID )
//    bool        transform_skip_flag     ( tu, compID )
//    RDPCMMode   explicit_rdpcm_mode     ( tu, compID )
//    int         last_sig_coeff          ( coeffCtx )
//    void        residual_coding_subblock( coeffCtx )
//================================================================================

void CABACReader::residual_coding( TransformUnit& tu, ComponentID compID )
{
  const CodingUnit& cu    = *tu.cu;
  const CompArea&   rRect = tu.blocks[compID];
  DTRACE( g_trace_ctx, D_SYNTAX, "residual_coding() etype=%d pos=(%d,%d) size=%dx%d predMode=%d\n", rRect.compID, rRect.x, rRect.y, rRect.width, rRect.height, cu.predMode );

  // parse transform skip and explicit rdpcm mode
  transform_skip_flag( tu, compID );
  explicit_rdpcm_mode( tu, compID );

  BITCNT_ACCUM( BS_TEXTURE );

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
  TCoeff*             coeff   = tu.getCoeffs( compID ).buf;
  unsigned&           GRStats = m_BinDecoder.getCtx().getGRAdaptStats( TU::getGolombRiceStatisticsIndex( tu, compID ) );
  unsigned            numSig  = 0;

  // parse last coeff position
  cctx.setScanPosLast( last_sig_coeff( cctx ) );

  // parse subblocks
  cctx.setGoRiceStats( GRStats );

  bool useEmt = ( cu.cs->sps->getSpsNext().getUseIntraEMT() && cu.predMode == MODE_INTRA ) || ( cu.cs->sps->getSpsNext().getUseInterEMT() && cu.predMode != MODE_INTRA );
  useEmt = useEmt && isLuma(compID);
  useEmt = ( canUse1dPartitions( compID ) && cu.mode1dPartitions && ( rRect.width == 1 || rRect.height == 1 ) ) ? false : useEmt;
#if HM_EMT_NSST_AS_IN_JEM
  if( cu.cs->sps->getSpsNext().getAltResiCompId() == 2 )
  {
    cctx.setIsEMT( useEmt && tu.cu->emtFlag && !tu.transformSkip[compID] );
  }
#endif

  const bool tcqEnabled = tu.cs->sps->getSpsNext().getUseTCQ();
  tu.tcq[compID] = tcqEnabled;

  if( tu.tcq[compID] )
  {
    int state = 0;
    for( int subSetId = ( cctx.scanPosLast() >> cctx.log2CGSize() ); subSetId >= 0; subSetId--)
    {
      cctx.initSubblock       ( subSetId );
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
      cctx.initSubblock       ( subSetId );
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
    BITCNT_ACCUM( BS_SIGNALING );
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
        tu.emtIdx = 0; //default transform
      }
    }
    else
    {
      emt_tu_index( tu );
    }
  }

  BITCNT_ACCUM( BS_SIGNALING );
}


void CABACReader::transform_skip_flag( TransformUnit& tu, ComponentID compID )
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
    tu.transformSkip[compID] = false;
    return;
  }
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET2( STATS__CABAC_BITS__TRANSFORM_SKIP_FLAGS, compID );

  bool tskip = m_BinDecoder.decodeBin( Ctx::TransformSkipFlag( toChannelType( compID ) ) );
  DTRACE( g_trace_ctx, D_SYNTAX, "transform_skip_flag() etype=%d pos=(%d,%d) trSkip=%d\n", compID, tu.blocks[compID].x, tu.blocks[compID].y, (int)tskip );
  tu.transformSkip[compID] = tskip;
}

Void CABACReader::emt_tu_index( TransformUnit& tu )
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

  UChar trIdx = 0;
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET( STATS__CABAC_BITS__EMT_TU_INDEX );

  if( CU::isIntra( *tu.cu ) && ( tu.cu->Y().width <= maxSizeEmtIntra ) && ( tu.cu->Y().height <= maxSizeEmtIntra ) )
  {
    bool uiSymbol1 = m_BinDecoder.decodeBin( Ctx::EMTTuIndex( 0 ) );
    bool uiSymbol2 = m_BinDecoder.decodeBin( Ctx::EMTTuIndex( 1 ) );

    trIdx = ( uiSymbol2 << 1 ) | ( int ) uiSymbol1;

    DTRACE( g_trace_ctx, D_SYNTAX, "emt_tu_index() etype=%d pos=(%d,%d) emtTrIdx=%d\n", COMPONENT_Y, tu.lx(), tu.ly(), ( int ) trIdx );
  }
  if( !CU::isIntra( *tu.cu ) && ( tu.cu->Y().width <= maxSizeEmtInter ) && ( tu.cu->Y().height <= maxSizeEmtInter ) )
  {
    bool uiSymbol1 = m_BinDecoder.decodeBin( Ctx::EMTTuIndex( 2 ) );
    bool uiSymbol2 = m_BinDecoder.decodeBin( Ctx::EMTTuIndex( 3 ) );

    trIdx = ( uiSymbol2 << 1 ) | ( int ) uiSymbol1;

    DTRACE( g_trace_ctx, D_SYNTAX, "emt_tu_index() etype=%d pos=(%d,%d) emtTrIdx=%d\n", COMPONENT_Y, tu.lx(), tu.ly(), ( int ) trIdx );
  }

  tu.emtIdx = trIdx;
}

Void CABACReader::emt_cu_flag( CodingUnit& cu )
{
  if( CU::isIntra( cu ) && cu.mode1dPartitions && ( cu.firstTU->lwidth() == 1 || cu.firstTU->lheight() == 1 ) )
  {
    return;
  }
  const CodingStructure &cs = *cu.cs;
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

  unsigned       depth      = cu.qtDepth;
  const unsigned cuWidth    = cu.lwidth();
  const unsigned cuHeight   = cu.lheight();

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

  cu.emtFlag = 0;

  const unsigned maxSizeEmt = CU::isIntra( cu ) ? maxSizeEmtIntra : maxSizeEmtInter;

  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET( STATS__CABAC_BITS__EMT_CU_FLAG );
  if( cuWidth <= maxSizeEmt && cuHeight <= maxSizeEmt )
  {
    bool uiCuFlag = m_BinDecoder.decodeBin( Ctx::EMTCuFlag( depth ) );
    cu.emtFlag = uiCuFlag;
    DTRACE( g_trace_ctx, D_SYNTAX, "emt_cu_flag() etype=%d pos=(%d,%d) emtCuFlag=%d\n", COMPONENT_Y, cu.lx(), cu.ly(), ( int ) cu.emtFlag );
  }
}

void CABACReader::mode_1d_partitions( CodingUnit& cu )
{
  if( !cu.cs->sps->getSpsNext().getUseMode1dPartitions() )
  {
    return;
  }


  if( !CU::isIntra( cu ) || cu.pdpc || cu.intra_NN != 0 || cu.diffFilterIdx != 0 || !canUse1dPartitions( cu.chType ) || cu.mrlIdx != 0 )
  {
    cu.mode1dPartitions = NO_1D_PARTITION;
    DTRACE( g_trace_ctx, D_SYNTAX, "mode_1d_partitions() etype=%d pos=(%d,%d) mode1dPartitionsIndex=%d\n", cu.chType, cu.blocks[cu.chType].x, cu.blocks[cu.chType].y, ( int ) cu.mode1dPartitions );
    return;
  }

  UInt uiMode1dPartitionsIndex = NO_1D_PARTITION;
  UInt symbol = m_BinDecoder.decodeBin( Ctx::Mode1dPartitions( 0 ) );

  if( symbol )
  {
    const PredictionUnit &pu  = *cu.firstPU;
    const UInt uiChFinalMode  = PU::getFinalIntraMode( pu, pu.chType );
    UInt intraModeRegion      = ( uiChFinalMode <= DC_IDX || uiChFinalMode > VDIA_IDX ) ? 0 : uiChFinalMode < DIA_IDX ? 1 : 2;
    intraModeRegion           = cu.cs->sps->getSpsNext().getUseIntraFTM() && pu.FTMRegIdx != 0 ? 0 : intraModeRegion;
    symbol                    = m_BinDecoder.decodeBin( Ctx::Mode1dPartitions( 1 + intraModeRegion ) );
    uiMode1dPartitionsIndex   = 1 + symbol;
    cu.emtFlag                = 0;
    cu.nsstIdx                = 0;
  }

  cu.mode1dPartitions = uiMode1dPartitionsIndex;
  DTRACE( g_trace_ctx, D_SYNTAX, "mode_1d_partitions() etype=%d pos=(%d,%d) mode1dPartitionsIndex=%d\n", cu.chType, cu.blocks[cu.chType].x, cu.blocks[cu.chType].y, ( int ) cu.mode1dPartitions );
}

void CABACReader::explicit_rdpcm_mode( TransformUnit& tu, ComponentID compID )
{
  const CodingUnit& cu = *tu.cu;

  tu.rdpcm[compID] = RDPCM_OFF;

  if( !CU::isIntra(cu) && CU::isRDPCMEnabled(cu) && ( tu.transformSkip[compID] || cu.transQuantBypass ) )
  {
    RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE( STATS__EXPLICIT_RDPCM_BITS, tu.blocks[tu.chType].lumaSize() );

    ChannelType chType = toChannelType( compID );
    if( m_BinDecoder.decodeBin( Ctx::RdpcmFlag( chType ) ) )
    {
      if( m_BinDecoder.decodeBin( Ctx::RdpcmDir( chType ) ) )
      {
        tu.rdpcm[compID] = RDPCM_VER;
      }
      else
      {
        tu.rdpcm[compID] = RDPCM_HOR;
      }
    }
  }
}

void CABACReader::residual_nsst_mode( CodingUnit& cu )
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
    cu.nsstIdx = 0;
    return;
  }

  if( CS::isDualITree( *cu.cs ) && cu.chType == CHANNEL_TYPE_CHROMA && std::min( cu.blocks[1].width, cu.blocks[1].height ) < 4 )
  {
    return;
  }

  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET( STATS__CABAC_BITS__NSST );
  if( ( cu.cs->sps->getSpsNext().getUseNSST() || cu.cs->sps->getSpsNext().getUseIntraNNTrafos() ) && CU::isIntra( cu ) && !CU::isLosslessCoded( cu ) && !cu.pdpc
    )
  {
    bool nonZeroCoeffNonTs;
    if( cu.cs->pcv->noRQT )
    {
      const int nonZeroCoeffThr = CS::isDualITree( *cu.cs ) ? ( isLuma( cu.chType ) ? NSST_SIG_NZ_LUMA : NSST_SIG_NZ_CHROMA ) : NSST_SIG_NZ_LUMA + NSST_SIG_NZ_CHROMA;
      nonZeroCoeffNonTs = CU::getNumNonZeroCoeffNonTs( cu ) > nonZeroCoeffThr;
    }
    else
    {
      nonZeroCoeffNonTs = CU::hasNonTsCodedBlock( cu );
    }
    if( !nonZeroCoeffNonTs )
    {
      cu.nsstIdx = 0;
      return;
    }
  }
  else
  {
    cu.nsstIdx = 0;
    return;
  }

  if( cu.intra_NN )
  {
    bool rot_possible_for_NN = !((cu.lwidth() <= MAX_ONE_D_DIMENSION_FOR_NN_TRAFO) && (cu.lheight() <= MAX_ONE_D_DIMENSION_FOR_NN_TRAFO) && cu.emtFlag);

    UInt idxROT = rot_possible_for_NN ? m_BinDecoder.decodeBin( Ctx::NSST_For_NN_Ctxts( 0 ) ) : 0;
    if( idxROT )
    {
      if( NUM_NSST_TRAFOS_PER_NN_MODE == 2 )
      {
        idxROT += m_BinDecoder.decodeBin( Ctx::NSST_For_NN_Ctxts( 1 ) );
      }
      if( NUM_NSST_TRAFOS_PER_NN_MODE == 3 )
      {
        if( m_BinDecoder.decodeBin( Ctx::NSST_For_NN_Ctxts( 1 ) ) )
        {
          idxROT++;
          idxROT += m_BinDecoder.decodeBin( Ctx::NSST_For_NN_Ctxts( 2 ) );
        }
      }
      if( NUM_NSST_TRAFOS_PER_NN_MODE == 4 )
      {
        if( m_BinDecoder.decodeBin( Ctx::NSST_For_NN_Ctxts( 1 ) ) )
        {
          idxROT += 2;
          idxROT += m_BinDecoder.decodeBin( Ctx::NSST_For_NN_Ctxts( 2 ) );
        }
        else
        {
          idxROT += m_BinDecoder.decodeBin( Ctx::NSST_For_NN_Ctxts( 3 ) );// 1 or 2
        }
      }
    }

    cu.nsstIdx = idxROT;
    return;
  }

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
    UInt idxROT = m_BinDecoder.decodeBin( Ctx::NSSTIdx( 1 ) );
    if( idxROT )
    {
      idxROT += m_BinDecoder.decodeBin( Ctx::NSSTIdx( 3 ) );
    }
    cu.nsstIdx = idxROT;
  }
  else
  {
    UInt idxROT = m_BinDecoder.decodeBin( Ctx::NSSTIdx( 0 ) );
    if( idxROT )
    {
      UInt uiSymbol = m_BinDecoder.decodeBin( Ctx::NSSTIdx( 2 ) );
      if( uiSymbol )
      {
        idxROT += 1 + m_BinDecoder.decodeBin( Ctx::NSSTIdx( 4 ) );
      }
    }
    cu.nsstIdx = idxROT;
  }

  DTRACE( g_trace_ctx, D_SYNTAX, "residual_nsst_mode() etype=%d pos=(%d,%d) mode=%d\n", COMPONENT_Y, cu.lx(), cu.ly(), ( int ) cu.nsstIdx );
}

int CABACReader::last_sig_coeff( CoeffCodingContext& cctx )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2( STATS__CABAC_BITS__LAST_SIG_X_Y, Size( cctx.width(), cctx.height() ), cctx.compID() );

  unsigned PosLastX = 0, PosLastY = 0;
  bool decodePosX, decodePosY;
  if( ( cctx.width() == 1 ) )
  {
    decodePosY = true;
    decodePosX = false;
  }
  else if( ( cctx.height() == 1 ) )
  {
    decodePosX = true;
    decodePosY = false;
  }
  else
  {
    decodePosX = true;
    decodePosY = true;
  }
  if( decodePosX )
  {
  for( ; PosLastX < cctx.maxLastPosX(); PosLastX++ )
  {
    if( ! m_BinDecoder.decodeBin( cctx.lastXCtxId( PosLastX ) ) )
    {
      break;
    }
  }
  }
  if( decodePosY )
  {
  for( ; PosLastY < cctx.maxLastPosY(); PosLastY++ )
  {
    if( ! m_BinDecoder.decodeBin( cctx.lastYCtxId( PosLastY ) ) )
    {
      break;
    }
  }
  }
  if( decodePosX )
  {
  if( PosLastX > 3 )
  {
    UInt uiTemp  = 0;
    UInt uiCount = ( PosLastX - 2 ) >> 1;
    for ( Int i = uiCount - 1; i >= 0; i-- )
    {
      uiTemp += m_BinDecoder.decodeBinEP( ) << i;
    }
    PosLastX = g_uiMinInGroup[ PosLastX ] + uiTemp;
  }
  }
  if( decodePosY )
  {
  if( PosLastY > 3 )
  {
    UInt uiTemp  = 0;
    UInt uiCount = ( PosLastY - 2 ) >> 1;
    for ( Int i = uiCount - 1; i >= 0; i-- )
    {
      uiTemp += m_BinDecoder.decodeBinEP( ) << i;
    }
    PosLastY = g_uiMinInGroup[ PosLastY ] + uiTemp;
  }
  }

  int blkPos;
  if( cctx.scanType() == SCAN_VER )
  {
    blkPos = PosLastY + ( PosLastX * cctx.width() );
  }
  else
  {
    blkPos = PosLastX + ( PosLastY * cctx.width() );
  }
  if( ( cctx.width() == 1 ) )// && cctx.scanType() != SCAN_VER) || (cctx.height() == 1 && cctx.scanType() == SCAN_VER))
  {
    blkPos = PosLastY;
  }
  else if( ( cctx.height() == 1 ) )// && cctx.scanType() != SCAN_VER) || (cctx.width() == 1 && cctx.scanType() == SCAN_VER))
  {
    blkPos = PosLastX;
  }

  int scanPos = 0;
  for( ; scanPos < cctx.maxNumCoeff() - 1; scanPos++ )
  {
    if( blkPos == cctx.blockPos( scanPos ) )
    {
      break;
    }
  }
  return scanPos;
}


static const int tab_nextState[ 4 ][ 2 ]  = { {0,2}, {2,0}, {1,3}, {3,1} };
static const int tab_nextQSet [ 4 ]       = {  0,     0,     1,     1    };

void CABACReader::residual_coding_subblock_tcq( CoeffCodingContext& cctx, TCoeff* coeff, int& state )
{
  // NOTE: All coefficients of the subblock must be set to zero before calling this function
#if RExt__DECODER_DEBUG_BIT_STATISTICS
  CodingStatisticsClassType ctype_group ( STATS__CABAC_BITS__SIG_COEFF_GROUP_FLAG,  cctx.width(), cctx.height(), cctx.compID() );
  CodingStatisticsClassType ctype_map   ( STATS__CABAC_BITS__SIG_COEFF_MAP_FLAG,    cctx.width(), cctx.height(), cctx.compID() );
  CodingStatisticsClassType ctype_gt1   ( STATS__CABAC_BITS__GT1_FLAG,              cctx.width(), cctx.height(), cctx.compID() );
  CodingStatisticsClassType ctype_gt2   ( STATS__CABAC_BITS__GT2_FLAG,              cctx.width(), cctx.height(), cctx.compID() );
  CodingStatisticsClassType ctype_gt3   ( STATS__CABAC_BITS__GT3_FLAG,              cctx.width(), cctx.height(), cctx.compID() );
  CodingStatisticsClassType ctype_gt4   ( STATS__CABAC_BITS__GT4_FLAG,              cctx.width(), cctx.height(), cctx.compID() );
#endif
  RExt__DECODER_DEBUG_BIT_STATISTICS_SET( ctype_group );

  //===== init =====
  const int   minSubPos   = cctx.minSubPos();
  const bool  isLast      = cctx.isLast();
  int         nextSigPos  = ( isLast ? cctx.scanPosLast() : cctx.maxSubPos() );

  //===== decode significant_coeffgroup_flag =====
  bool sigGroup = ( isLast || !minSubPos );
  if( !sigGroup )
  {
    sigGroup = m_BinDecoder.decodeBin( cctx.sigGroupCtxId() );
  }
  if( sigGroup )
  {
    cctx.setSigGroup();
  }
  else
  {
    return;
  }

  //===== decode significant_coeff_flag's =====
  const int inferSigPos   = nextSigPos != cctx.scanPosLast() ? ( cctx.isNotFirst() ? minSubPos : -1 ) : nextSigPos;
  int       numNonZero    =  0;
  int       sigBlkPos[ 1 << MLS_CG_SIZE ];

#if RExt__DECODER_DEBUG_BIT_STATISTICS
  CodingStatisticsClassType ctype_escs ( STATS__CABAC_BITS__ESCAPE_BITS, cctx.width(), cctx.height(), cctx.compID() );
#endif

  for( ; nextSigPos >= minSubPos; nextSigPos-- )
  {
    int       blkPos      = cctx.blockPos( nextSigPos );
    unsigned  sigFlag     = ( !numNonZero && nextSigPos == inferSigPos );
    bool      escapeData  = false;
    if( !sigFlag )
    {
      RExt__DECODER_DEBUG_BIT_STATISTICS_SET( ctype_map );
      sigFlag = m_BinDecoder.decodeBin( cctx.sigCtxIdAbsTcq( state, nextSigPos, coeff ) );
      DTRACE( g_trace_ctx, D_SYNTAX_RESI, "sig_bin() bin=%d ctx=%d\n", sigFlag, cctx.blockPos( nextSigPos ) );
    }

    if( sigFlag )
    {
      const unsigned gt1CtxId = cctx.greater1CtxIdTcq(state);
      sigBlkPos[ numNonZero++ ] = blkPos;
      coeff[ blkPos ] = 1;

      RExt__DECODER_DEBUG_BIT_STATISTICS_SET( ctype_gt1 );
      sigFlag = m_BinDecoder.decodeBin( gt1CtxId );
      DTRACE( g_trace_ctx, D_SYNTAX_RESI, "gt1_flag() bin=%d ctx=%d\n", sigFlag, gt1CtxId );

      const unsigned gtxCtxId = cctx.greaterXCtxIdTcq();

      if( sigFlag )
      {
        coeff[blkPos]++;

        RExt__DECODER_DEBUG_BIT_STATISTICS_SET( ctype_gt2 );
        sigFlag = m_BinDecoder.decodeBin( gtxCtxId );
        DTRACE( g_trace_ctx, D_SYNTAX_RESI, "gt2_flag() bin=%d ctx=%d\n", sigFlag, gtxCtxId );

        if( sigFlag )
        {
          coeff[blkPos]++;

          RExt__DECODER_DEBUG_BIT_STATISTICS_SET( ctype_gt3 );
          sigFlag = m_BinDecoder.decodeBin( gtxCtxId );
          DTRACE( g_trace_ctx, D_SYNTAX_RESI, "gt3_flag() bin=%d ctx=%d\n", sigFlag, gtxCtxId );

          if( sigFlag )
          {
            coeff[blkPos]++;

            RExt__DECODER_DEBUG_BIT_STATISTICS_SET( ctype_gt4 );
            sigFlag = m_BinDecoder.decodeBin( gtxCtxId );
            DTRACE( g_trace_ctx, D_SYNTAX_RESI, "gt4_flag() bin=%d ctx=%d\n", sigFlag, gtxCtxId );

            if( sigFlag )
            {
              coeff[blkPos]++;
              escapeData = true;
            }
          }
        }
      }

      if( escapeData )
      {
        RExt__DECODER_DEBUG_BIT_STATISTICS_SET( ctype_escs );
        int remAbsLevel = m_BinDecoder.decodeRemAbsEP( cctx.GoRicePar(), cctx.extPrec(), cctx.maxLog2TrDRange(), true );
        DTRACE( g_trace_ctx, D_SYNTAX_RESI, "rem_val() bin=%d ctx=%d\n", remAbsLevel, cctx.GoRicePar() );
        coeff[ blkPos ] += remAbsLevel;
      }
    }

    state = tab_nextState[ state ][ coeff[ blkPos ] & 1 ];
  }

  //===== decode sign's =====
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2( STATS__CABAC_BITS__SIGN_BIT, Size( cctx.width(), cctx.height() ), cctx.compID() );
  unsigned        signPattern = m_BinDecoder.decodeBinsEP( numNonZero ) << ( 32 - numNonZero );

  //===== set final coefficents =====
  int sumAbs = 0;
  for( unsigned k = 0; k < numNonZero; k++ )
  {
    int AbsCoeff          = coeff[ sigBlkPos[ k ] ];
    sumAbs               += AbsCoeff;
    coeff[ sigBlkPos[k] ] = ( signPattern & ( 1u << 31 ) ? -AbsCoeff : AbsCoeff );
    signPattern         <<= 1;
  }
  cctx.setEmtNumSigCoeff(numNonZero);
}


void CABACReader::residual_coding_subblock( CoeffCodingContext& cctx, TCoeff* coeff )
{
  // NOTE: All coefficients of the subblock must be set to zero before calling this function
#if RExt__DECODER_DEBUG_BIT_STATISTICS
  CodingStatisticsClassType ctype_group ( STATS__CABAC_BITS__SIG_COEFF_GROUP_FLAG,  cctx.width(), cctx.height(), cctx.compID() );
  CodingStatisticsClassType ctype_map   ( STATS__CABAC_BITS__SIG_COEFF_MAP_FLAG,    cctx.width(), cctx.height(), cctx.compID() );
  CodingStatisticsClassType ctype_gt1   ( STATS__CABAC_BITS__GT1_FLAG,              cctx.width(), cctx.height(), cctx.compID() );
  CodingStatisticsClassType ctype_gt2   ( STATS__CABAC_BITS__GT2_FLAG,              cctx.width(), cctx.height(), cctx.compID() );
  CodingStatisticsClassType ctype_gt3   ( STATS__CABAC_BITS__GT3_FLAG,              cctx.width(), cctx.height(), cctx.compID() );
  CodingStatisticsClassType ctype_gt4   ( STATS__CABAC_BITS__GT4_FLAG,              cctx.width(), cctx.height(), cctx.compID() );
#endif
  RExt__DECODER_DEBUG_BIT_STATISTICS_SET( ctype_group );

  //===== init =====
  const int   maxSbbSize  = 1 << cctx.log2CGSize();
  const int   minSubPos   = cctx.minSubPos();
  const bool  isLast      = cctx.isLast();
  int         nextSigPos  = ( isLast ? cctx.scanPosLast() : cctx.maxSubPos() );

  //===== decode significant_coeffgroup_flag =====
  bool sigGroup = ( isLast || !minSubPos );
  if( !sigGroup )
  {
    sigGroup = m_BinDecoder.decodeBin( cctx.sigGroupCtxId() );
  }
  if( sigGroup )
  {
    cctx.setSigGroup();
  }
  else
  {
    return;
  }

  if( cctx.altResiCompId() == 2 )
  {
  //===== decode significant_coeff_flag's =====
    const int inferSigPos   = nextSigPos != cctx.scanPosLast() ? ( cctx.isNotFirst() ? minSubPos : -1 ) : nextSigPos;
    int       firstNZPos    = nextSigPos;
    int       lastNZPos     = -1;
    int       numGtXFlags   =  0;
    int       currLevelFlag =  0;
    int       numNonZero    =  0;
    int       sigBlkPos[ 1 << MLS_CG_SIZE ];

#if RExt__DECODER_DEBUG_BIT_STATISTICS
    CodingStatisticsClassType ctype_escs ( STATS__CABAC_BITS__ESCAPE_BITS, cctx.width(), cctx.height(), cctx.compID() );
#endif

    for( ; nextSigPos >= minSubPos; nextSigPos-- )
    {
      int      blkPos     = cctx.blockPos( nextSigPos );
      unsigned sigFlag    = ( !numNonZero && nextSigPos == inferSigPos );
      bool     escapeData = false;
      if( !sigFlag )
      {
        RExt__DECODER_DEBUG_BIT_STATISTICS_SET( ctype_map );
        sigFlag = m_BinDecoder.decodeBin( cctx.sigCtxIdAbs( nextSigPos, coeff ) );
        DTRACE( g_trace_ctx, D_SYNTAX_RESI, "sig_bin() bin=%d ctx=%d\n", sigFlag, cctx.blockPos( nextSigPos ) );
      }

      if( sigFlag )
      {
        currLevelFlag++;
        sigBlkPos[ numNonZero++ ] = blkPos;
        firstNZPos      = nextSigPos;
        lastNZPos       = std::max<int>( lastNZPos, nextSigPos );
        coeff[ blkPos ] = 1;

        if (numGtXFlags < CXFLAG_NUMBER)
        {
          numGtXFlags++;
          RExt__DECODER_DEBUG_BIT_STATISTICS_SET( ctype_gt1 );
          sigFlag = m_BinDecoder.decodeBin( cctx.greaterXCtxId( currLevelFlag ) );
          DTRACE( g_trace_ctx, D_SYNTAX_RESI, "gt1_flag() bin=%d ctx=%d\n", sigFlag, cctx.greaterXCtxId( currLevelFlag ) );

          if( sigFlag )
          {
            coeff[blkPos]++;
            currLevelFlag++;

            RExt__DECODER_DEBUG_BIT_STATISTICS_SET( ctype_gt2 );
            sigFlag = m_BinDecoder.decodeBin( cctx.greaterXCtxId( currLevelFlag ) );
            DTRACE( g_trace_ctx, D_SYNTAX_RESI, "gt2_flag() bin=%d ctx=%d\n", sigFlag, cctx.greaterXCtxId( currLevelFlag ) );

            if( sigFlag )
            {
              coeff[blkPos]++;
              currLevelFlag++;

              RExt__DECODER_DEBUG_BIT_STATISTICS_SET( ctype_gt3 );
              sigFlag = m_BinDecoder.decodeBin( cctx.greaterXCtxId( currLevelFlag ) );
              DTRACE( g_trace_ctx, D_SYNTAX_RESI, "gt3_flag() bin=%d ctx=%d\n", sigFlag, cctx.greaterXCtxId( currLevelFlag ) );

              if( sigFlag )
              {
                coeff[blkPos]++;
                currLevelFlag++;

                RExt__DECODER_DEBUG_BIT_STATISTICS_SET( ctype_gt4 );
                sigFlag = m_BinDecoder.decodeBin( cctx.greaterXCtxId( currLevelFlag ) );
                DTRACE( g_trace_ctx, D_SYNTAX_RESI, "gt4_flag() bin=%d ctx=%d\n", sigFlag, cctx.greaterXCtxId( currLevelFlag ) );

                if( sigFlag )
                {
                  coeff[blkPos]++;
                  escapeData = true;
                }
              }
            }

          }
        }
        else
        {
          escapeData = true;
        }

        if( escapeData )
        {
          RExt__DECODER_DEBUG_BIT_STATISTICS_SET( ctype_escs );
          int remAbsLevel = m_BinDecoder.decodeRemAbsEP( cctx.GoRicePar(), cctx.extPrec(), cctx.maxLog2TrDRange(), true );
          DTRACE( g_trace_ctx, D_SYNTAX_RESI, "rem_val() bin=%d ctx=%d\n", remAbsLevel, cctx.GoRicePar() );
          coeff[ blkPos ] += remAbsLevel;
        }
      }
    }

    //===== decode sign's =====
    RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2( STATS__CABAC_BITS__SIGN_BIT, Size( cctx.width(), cctx.height() ), cctx.compID() );
    const unsigned  numSigns    = ( cctx.hideSign( firstNZPos, lastNZPos ) ? numNonZero - 1 : numNonZero );
    unsigned        signPattern = m_BinDecoder.decodeBinsEP( numSigns ) << ( 32 - numSigns );

    //===== set final coefficents =====
    int sumAbs = 0;
    for( unsigned k = 0; k < numSigns; k++ )
    {
      int AbsCoeff          = coeff[ sigBlkPos[ k ] ];
      sumAbs               += AbsCoeff;
      coeff[ sigBlkPos[k] ] = ( signPattern & ( 1u << 31 ) ? -AbsCoeff : AbsCoeff );
      signPattern         <<= 1;
    }
    if( numNonZero > numSigns )
    {
      int k                 = numSigns;
      int AbsCoeff          = coeff[ sigBlkPos[ k ] ];
      sumAbs               += AbsCoeff;
      coeff[ sigBlkPos[k] ] = ( sumAbs & 1 ? -AbsCoeff : AbsCoeff );
    }
    cctx.setEmtNumSigCoeff(numNonZero);
  }
  else
  if( cctx.altResiCompId() == 1 )
  {
    //===== decode significant_coeff_flag's =====
    const int inferSigPos = ( cctx.isNotFirst() ? minSubPos : -1 );
    unsigned  numNonZero  = 0;
    int       firstNZPos  = maxSbbSize;
    int       lastNZPos   = -1;
    int       sigBlkPos   [ 1 << MLS_CG_SIZE ];
    int       scanPos     [ 1 << MLS_CG_SIZE ];
    if( isLast )
    {
      firstNZPos                = nextSigPos;
      lastNZPos                 = nextSigPos;
      sigBlkPos[ numNonZero   ] = cctx.blockPos( nextSigPos );
      scanPos  [ numNonZero++ ] = nextSigPos--;
      coeff[ sigBlkPos[ 0 ] ]   = 1;
    }
    for( ; nextSigPos >= minSubPos; nextSigPos-- )
    {
      unsigned sigFlag = ( !numNonZero && nextSigPos == inferSigPos );
      if( !sigFlag )
      {
        sigFlag = m_BinDecoder.decodeBin( cctx.sigCtxId( nextSigPos, coeff ) );
      }
      if( sigFlag )
      {
        sigBlkPos [ numNonZero ]    = cctx.blockPos( nextSigPos );
        scanPos   [ numNonZero++ ]  = nextSigPos;
        coeff[ cctx.blockPos( nextSigPos ) ] = 1;
        firstNZPos                  = nextSigPos;
        lastNZPos                   = std::max<int>( lastNZPos, nextSigPos );
      }
    }

    RExt__DECODER_DEBUG_BIT_STATISTICS_SET( ctype_gt1 );

    //===== decode abs_greater1_flag's =====
    const unsigned  numGt1Flags = std::min<unsigned>( numNonZero, C1FLAG_NUMBER );
    int             gt2FlagIdx  = maxSbbSize;
    bool            escapeData  = false;
    uint16_t        ctxGt1Id    = cctx.greater1CtxId( 0 );
    for( unsigned k = 0; k < numGt1Flags; k++ )
    {
      if( k || !cctx.isLast() )
      {
        ctxGt1Id = cctx.greater1CtxId( scanPos[ k ], coeff );
      }

      if( m_BinDecoder.decodeBin( ctxGt1Id ) )
      {
        coeff[ sigBlkPos[ k ] ] = 2;
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
        coeff[ sigBlkPos[ k ] ] = 1;
      }
    }
    for( unsigned k = numGt1Flags; k < numNonZero; k++ )
    {
      coeff[ sigBlkPos[ k ] ] = 1;
      escapeData              = true;
    }
    cctx.setGt2Flag( ctxGt1Id == 0 );

    RExt__DECODER_DEBUG_BIT_STATISTICS_SET( ctype_gt2 );

    //===== decode abs_greater2_flag =====
    if( gt2FlagIdx < maxSbbSize )
    {
      uint16_t ctxGt2Id = cctx.greater1CtxIdOfs();
      if( gt2FlagIdx || !cctx.isLast() )
      {
        ctxGt2Id = cctx.greater2CtxId( scanPos[ gt2FlagIdx ], coeff );
      }

      if( m_BinDecoder.decodeBin( ctxGt2Id ) )
      {
        coeff[ sigBlkPos[ gt2FlagIdx ] ]++;
        escapeData = true;
      }
    }

    //===== align data =====
    if( escapeData && cctx.alignFlag() )
    {
      m_BinDecoder.align();
    }

#if RExt__DECODER_DEBUG_BIT_STATISTICS
    const bool alignGroup = escapeData && cctx.alignFlag();
    CodingStatisticsClassType ctype_signs( ( alignGroup ? STATS__CABAC_BITS__ALIGNED_SIGN_BIT    : STATS__CABAC_BITS__SIGN_BIT    ), cctx.width(), cctx.height(), cctx.compID() );
    CodingStatisticsClassType ctype_escs ( ( alignGroup ? STATS__CABAC_BITS__ALIGNED_ESCAPE_BITS : STATS__CABAC_BITS__ESCAPE_BITS ), cctx.width(), cctx.height(), cctx.compID() );
#endif

    RExt__DECODER_DEBUG_BIT_STATISTICS_SET( ctype_signs );

    //===== decode remaining absolute values =====
    if( escapeData )
    {
      RExt__DECODER_DEBUG_BIT_STATISTICS_SET( ctype_escs );

      bool      updateGoRiceStats = cctx.updGoRiceStats();
      unsigned  GoRicePar         = cctx.currGoRiceStats() >> 2;
      unsigned  MaxGoRicePar      = ( updateGoRiceStats ? std::numeric_limits<unsigned>::max() : 4 );
      int       baseLevel         = 3;
      for( int k = 0; k < numNonZero; k++ )
      {
        if( coeff[ sigBlkPos[ k ] ] == baseLevel )
        {
          if( !updateGoRiceStats || !cctx.isLast() )
          {
            GoRicePar = cctx.GoRicePar( scanPos[ k ], coeff );
          }

          int remAbs    = m_BinDecoder.decodeRemAbsEP( GoRicePar, cctx.extPrec(), cctx.maxLog2TrDRange(), true );
          coeff[ sigBlkPos[ k ] ] = baseLevel + remAbs;

          // update rice parameter
          if( coeff[ sigBlkPos[ k ] ] > ( 3 << GoRicePar ) && GoRicePar < MaxGoRicePar )
          {
            GoRicePar++;
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
        else if( baseLevel == 3 && coeff[ sigBlkPos[ k ] ] > 1 )
        {
          baseLevel = 2;
        }
      }
    }

    //===== decode sign's =====
    const unsigned  numSigns    = ( cctx.hideSign( firstNZPos, lastNZPos ) ? numNonZero - 1 : numNonZero );
    unsigned        signPattern = m_BinDecoder.decodeBinsEP( numSigns ) << ( 32 - numSigns );

    //===== set final coefficents =====
    int sumAbs = 0;
    for( unsigned k = 0; k < numSigns; k++ )
    {
      int AbsCoeff          = coeff[ sigBlkPos[ k ] ];
      sumAbs               += AbsCoeff;
      coeff[ sigBlkPos[k] ] = ( signPattern & ( 1u << 31 ) ? -AbsCoeff : AbsCoeff );
      signPattern         <<= 1;
    }
    if( numNonZero > numSigns )
    {
      int k                 = numSigns;
      int AbsCoeff          = coeff[ sigBlkPos[ k ] ];
      sumAbs               += AbsCoeff;
      coeff[ sigBlkPos[k] ] = ( sumAbs & 1 ? -AbsCoeff : AbsCoeff );
    }

    cctx.setEmtNumSigCoeff( numNonZero );
  }
  else
  {
    //===== decode significant_coeff_flag's =====
    RExt__DECODER_DEBUG_BIT_STATISTICS_SET( ctype_map );
    const int inferSigPos = ( cctx.isNotFirst() ? minSubPos : -1 );
    unsigned  numNonZero  = 0;
    int       firstNZPos  = maxSbbSize;
    int       lastNZPos   = -1;
    int       sigBlkPos   [ 1 << MLS_CG_SIZE ];
    if( isLast )
    {
      firstNZPos                = nextSigPos;
      lastNZPos                 = nextSigPos;
      sigBlkPos[ numNonZero++ ] = cctx.blockPos( nextSigPos-- );
    }
    for( ; nextSigPos >= minSubPos; nextSigPos-- )
    {
      unsigned sigFlag = ( !numNonZero && nextSigPos == inferSigPos );
      if( !sigFlag )
      {
        sigFlag = m_BinDecoder.decodeBin( cctx.sigCtxId( nextSigPos ) );
      }
      if( sigFlag )
      {
        sigBlkPos [ numNonZero++ ]  = cctx.blockPos( nextSigPos );
        firstNZPos                  = nextSigPos;
        lastNZPos                   = std::max<int>( lastNZPos, nextSigPos );
      }
    }

    RExt__DECODER_DEBUG_BIT_STATISTICS_SET( ctype_gt1 );

    //===== decode abs_greater1_flag's =====
    int             absCoeff    [ 1 << MLS_CG_SIZE ];
    const unsigned  numGt1Flags = std::min<unsigned>( numNonZero, C1FLAG_NUMBER );
    int             gt2FlagIdx  = maxSbbSize;
    bool            escapeData  = false;
    uint16_t        ctxGt1Id    = 1;
    for( unsigned k = 0; k < numGt1Flags; k++ )
    {
      if( m_BinDecoder.decodeBin( cctx.greater1CtxId( ctxGt1Id ) ) )
      {
        absCoeff[ k ] = 2;
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
        absCoeff[ k ] = 1;
        if( ctxGt1Id && ctxGt1Id < 3 )
        {
          ctxGt1Id++;
        }
      }
    }
    for( unsigned k = numGt1Flags; k < numNonZero; k++ )
    {
      absCoeff[ k ] = 1;
      escapeData    = true;
    }
    cctx.setGt2Flag( ctxGt1Id == 0 );

    RExt__DECODER_DEBUG_BIT_STATISTICS_SET( ctype_gt2 );

    //===== decode abs_greater2_flag =====
    if( gt2FlagIdx < maxSbbSize )
    {
      if( m_BinDecoder.decodeBin( cctx.greater2CtxId() ) )
      {
        absCoeff[ gt2FlagIdx ]++;
        escapeData = true;
      }
    }

    //===== align data =====
    if( escapeData && cctx.alignFlag() )
    {
      m_BinDecoder.align();
    }

  #if RExt__DECODER_DEBUG_BIT_STATISTICS
    const bool alignGroup = escapeData && cctx.alignFlag();
    CodingStatisticsClassType ctype_signs( ( alignGroup ? STATS__CABAC_BITS__ALIGNED_SIGN_BIT    : STATS__CABAC_BITS__SIGN_BIT    ), cctx.width(), cctx.height(), cctx.compID() );
    CodingStatisticsClassType ctype_escs ( ( alignGroup ? STATS__CABAC_BITS__ALIGNED_ESCAPE_BITS : STATS__CABAC_BITS__ESCAPE_BITS ), cctx.width(), cctx.height(), cctx.compID() );
  #endif

    RExt__DECODER_DEBUG_BIT_STATISTICS_SET( ctype_signs );

    //===== decode sign's =====
    const unsigned  numSigns    = ( cctx.hideSign( firstNZPos, lastNZPos ) ? numNonZero - 1 : numNonZero );
    unsigned        signPattern = m_BinDecoder.decodeBinsEP( numSigns ) << ( 32 - numSigns );

    //===== decode remaining absolute values =====
    if( escapeData )
    {
      RExt__DECODER_DEBUG_BIT_STATISTICS_SET( ctype_escs );

      bool      updateGoRiceStats = cctx.updGoRiceStats();
      unsigned  GoRicePar         = cctx.currGoRiceStats() >> 2;
      unsigned  MaxGoRicePar      = ( updateGoRiceStats ? std::numeric_limits<unsigned>::max() : 4 );
      int       baseLevel         = 3;
      for( int k = 0; k < numNonZero; k++ )
      {
        if( absCoeff[ k ] == baseLevel )
        {
          int remAbs    = m_BinDecoder.decodeRemAbsEP( GoRicePar, cctx.extPrec(), cctx.maxLog2TrDRange() );
          absCoeff[ k ] = baseLevel + remAbs;

          // update rice parameter
          if( absCoeff[ k ] > ( 3 << GoRicePar ) && GoRicePar < MaxGoRicePar )
          {
            GoRicePar++;
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

    //===== set final coefficents =====
    int sumAbs = 0;
    for( unsigned k = 0; k < numSigns; k++ )
    {
      int AbsCoeff          = absCoeff[k];
      sumAbs               += AbsCoeff;
      coeff[ sigBlkPos[k] ] = ( signPattern & ( 1u << 31 ) ? -AbsCoeff : AbsCoeff );
      signPattern         <<= 1;
    }
    if( numNonZero > numSigns )
    {
      int k                 = numSigns;
      int AbsCoeff          = absCoeff[k];
      sumAbs               += AbsCoeff;
      coeff[ sigBlkPos[k] ] = ( sumAbs & 1 ? -AbsCoeff : AbsCoeff );
    }

    cctx.setEmtNumSigCoeff( numNonZero );
  }
}





//================================================================================
//  clause 7.3.8.12
//--------------------------------------------------------------------------------
//    void  cross_comp_pred( tu, compID )
//================================================================================

void CABACReader::cross_comp_pred( TransformUnit& tu, ComponentID compID )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2(STATS__CABAC_BITS__CROSS_COMPONENT_PREDICTION, tu.blocks[compID], compID);

  signed char alpha   = 0;
  unsigned    ctxBase = ( compID == COMPONENT_Cr ? 5 : 0 );
  unsigned    symbol  = m_BinDecoder.decodeBin( Ctx::CrossCompPred(ctxBase) );
  if( symbol )
  {
    // Cross-component prediction alpha is non-zero.
    symbol = m_BinDecoder.decodeBin( Ctx::CrossCompPred(ctxBase+1) );
    if( symbol )
    {
      // alpha is 2 (symbol=1), 4(symbol=2) or 8(symbol=3).
      // Read up to two more bits
      symbol += unary_max_symbol( Ctx::CrossCompPred(ctxBase+2), Ctx::CrossCompPred(ctxBase+3), 2 );
    }
    alpha = ( 1 << symbol );
    if( m_BinDecoder.decodeBin( Ctx::CrossCompPred(ctxBase+4) ) )
    {
      alpha = -alpha;
    }
  }
  DTRACE( g_trace_ctx, D_SYNTAX, "cross_comp_pred() etype=%d pos=(%d,%d) alpha=%d\n", compID, tu.blocks[compID].x, tu.blocks[compID].y, tu.compAlpha[compID] );
  tu.compAlpha[compID] = alpha;
}

#if THRESHOLDING
void CABACReader::thresholding( TransformUnit& tu )
{
  unsigned numThresholdingSizes = Thresholding::getNumberValidSize( tu.Y(), tu.cs->slice->getSliceType(), tu.chType, &tu.cs->sps->getSpsNext() );

  if( !Thresholding::allowThresholding( tu ) )
  {
    numThresholdingSizes = 0;
  }

  if( 0 == numThresholdingSizes )
  {
    tu.thresholdingSize = 0;
    return;
  }

  unsigned ctxIdx = Ctx::Thresholding( 0 );
  tu.thresholding = m_BinDecoder.decodeBin( ctxIdx );

  if( tu.thresholding )
  {
    const SliceType sliceType = tu.cs->slice->isIntra() ? I_SLICE : B_SLICE;
    const int maxSize = numThresholdingSizes - 1;
    const int maxThrs = tu.cs->sps->getSpsNext().getThresholdingMaxThrs( sliceType ) - 1;

    unsigned ctxBse = 1;
    unsigned ctxIdx = ctxBse;

    unsigned onesRead = 0;
    while( onesRead < maxSize && m_BinDecoder.decodeBin( Ctx::Thresholding( ctxIdx ) ) == 1 )
    {
      ++onesRead;
      ctxIdx = std::min( ctxIdx + 1, ctxBse + 4 );
    }
    tu.thresholdingSize = onesRead;

    ctxBse = 6;
    ctxIdx = ctxBse;

    onesRead = 0;
    while( onesRead < maxThrs && m_BinDecoder.decodeBin( Ctx::Thresholding( ctxIdx ) ) == 1 )
    {
      ++onesRead;
      ctxIdx = std::min( ctxIdx + 1, ctxBse + 10 );
    }
    tu.thresholdingThrs = onesRead;
  }

  DTRACE( g_trace_ctx, D_SYNTAX, "thresholding() usage=%d size=%d thrs=%d\n", tu.thresholding, tu.thresholdingSize, tu.thresholdingThrs, tu.cu->predMode );
}
#endif


//================================================================================
//  helper functions
//--------------------------------------------------------------------------------
//    unsigned  unary_max_symbol ( ctxId0, ctxId1, maxSymbol )
//    unsigned  unary_max_eqprob (                 maxSymbol )
//    unsigned  exp_golomb_eqprob( count )
//================================================================================

unsigned CABACReader::unary_max_symbol( unsigned ctxId0, unsigned ctxIdN, unsigned maxSymbol  )
{
  unsigned onesRead = 0;
  while( onesRead < maxSymbol && m_BinDecoder.decodeBin( onesRead == 0 ? ctxId0 : ctxIdN ) == 1 )
  {
    ++onesRead;
  }
  return onesRead;
}


unsigned CABACReader::unary_max_eqprob( unsigned maxSymbol )
{
  for( unsigned k = 0; k < maxSymbol; k++ )
  {
    if( !m_BinDecoder.decodeBinEP() )
    {
      return k;
    }
  }
  return maxSymbol;
}


unsigned CABACReader::exp_golomb_eqprob( unsigned count )
{
  unsigned symbol = 0;
  unsigned bit    = 1;
  while( bit )
  {
    bit     = m_BinDecoder.decodeBinEP( );
    symbol += bit << count++;
  }
  if( --count )
  {
    symbol += m_BinDecoder.decodeBinsEP( count );
  }
  return symbol;
}


unsigned CABACReader::decode_sparse_dt( DecisionTree& dt )
{
  dt.reduce();

  unsigned depth  = dt.dtt.depth;
  unsigned offset = 0;

  while( dt.dtt.hasSub[offset] )
  {
    CHECKD( depth == 0, "Depth is '0' for a decision node in a decision tree" );

    const unsigned posRight = offset + 1;
    const unsigned posLeft  = offset + ( 1u << depth );

    bool isLeft = true;

    if( dt.isAvail[posRight] && dt.isAvail[posLeft] )
    {
      // encode the decision as both sub-paths are available
      const unsigned ctxId = dt.ctxId[offset];

      if( ctxId > 0 )
      {
        DTRACE( g_trace_ctx, D_DECISIONTREE, "Decision coding using context %d\n", ctxId - 1 );
        isLeft = m_BinDecoder.decodeBin( ctxId - 1 ) == 0;
      }
      else
      {
        DTRACE( g_trace_ctx, D_DECISIONTREE, "Decision coding as an EP bin\n" );
        isLeft = m_BinDecoder.decodeBinEP() == 0;
      }
    }
    else if( dt.isAvail[posRight] )
    {
      isLeft = false;
    }

    DTRACE( g_trace_ctx, D_DECISIONTREE, "Following the tree to the %s sub-node\n", isLeft ? "left" : "right" );

    offset = isLeft ? posLeft : posRight;
    depth--;
  }

  CHECKD( dt.isAvail[offset] == false, "The decoded element is not available" );
  DTRACE( g_trace_ctx, D_DECISIONTREE, "Found an end-node of the tree\n" );
  return dt.dtt.ids[offset];
}

