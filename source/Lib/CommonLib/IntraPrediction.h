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

/** \file     IntraPrediction.h
    \brief    prediction class (header)
*/

#ifndef __INTRAPREDICTION__
#define __INTRAPREDICTION__


// Include files
#include "Unit.h"
#include "Buffer.h"
#include "Picture.h"

#include "IntraPrediction_NN.h"
#include "IntraNNRom.h"

#include <functional>

//! \ingroup CommonLib
//! \{

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// prediction class
enum PredBuf
{
  PRED_BUF_UNFILTERED = 0,
  PRED_BUF_FILTERED   = 1,
  NUM_PRED_BUF        = 2
};

static const UInt MAX_INTRA_FILTER_DEPTHS=8;

class IntraPrediction
{
private:

  Pel* m_piYuvExt[MAX_NUM_COMPONENT][NUM_PRED_BUF];
  Int  m_iYuvExtSize;

  Pel* m_piYuvExtMrl[MAX_NUM_COMPONENT][NUM_PRED_BUF];
  Int  m_iYuvExtMrlSize;

  static const UChar m_aucIntraFilter[MAX_NUM_CHANNEL_TYPE][MAX_INTRA_FILTER_DEPTHS];

  unsigned m_auShiftLM[32]; // Table for substituting division operation by multiplication

  Pel* m_piTemp;
  static const int m_intraBiFiWeights[177][5];
  Pel*   m_pLumaRecBufferMul[LM_FILTER_NUM];

  struct pu_Identifier_NNS
  {
    pu_Identifier_NNS() : pu_area(), cu_split_series( 0 ), subsampling_flag( ( UChar ) ( 0 ) ), poc( 0 ) {}
    pu_Identifier_NNS(CompArea pu_area_curr, SplitSeries cu_split_series_curr, UChar subsampling_flag_curr) : pu_area(pu_area_curr), cu_split_series(cu_split_series_curr), subsampling_flag(subsampling_flag_curr), poc( 0 ) {}
    CompArea    pu_area;
    SplitSeries cu_split_series;
    UChar       subsampling_flag;
    unsigned    poc;
  };
  pu_Identifier_NNS      m_pu_Identifier_NNS;

  IntraPrediction_NN     m_IntraPrediction_NN;
  std::vector<int>       m_praedSignalNN;
  std::vector<int>       m_outputJointHiddenLayer;

  struct pu_Identifier_Diff
  {
    pu_Identifier_Diff() : pu_area(), cu_split_series(0), m_intraNN(0), m_intra_NN_Use_Sampling(0), m_numFullRDModes(0), m_RdModeList() {}
    pu_Identifier_Diff(CompArea pu_area_curr, SplitSeries cu_split_series_curr, UChar intraNN, UChar intra_NN_Use_Subsampling, UInt numFullRDModes,
      static_vector< UInt, FAST_UDI_MAX_RDMODE_NUM > m_RdModeList) : 
                                pu_area(pu_area_curr),    cu_split_series(cu_split_series_curr), m_intraNN(intraNN), m_intra_NN_Use_Sampling(intra_NN_Use_Subsampling), 
                                m_numFullRDModes(numFullRDModes), m_RdModeList() {}
    CompArea    pu_area;
    SplitSeries cu_split_series;
    UChar m_intraNN;
    UChar m_intra_NN_Use_Sampling;
    UInt m_numFullRDModes;
    static_vector< UInt, FAST_UDI_MAX_RDMODE_NUM > m_RdModeList;
  };
  pu_Identifier_Diff m_pu_Identifier_DiffFilter;

  // copy unfiltered ref. samples to line buffer
  Pel                    m_piTempRef[4 * MAX_CU_SIZE + 1];
  Pel                    m_piFiltRef[4 * MAX_CU_SIZE + 1];

protected:

  ChromaFormat  m_currChromaFormat;

  // prediction
  Void xPredIntraPlanarBasic1D    ( const CPelBuf &pSrc, PelBuf &pDst, const SPS& sps );
  Void xPredIntraPlanar           ( const CPelBuf &pSrc, const int mrlOffset, PelBuf &pDst,                                                                                                            const SPS& sps );
  Void xPredIntraDc               ( const CPelBuf &pSrc, const int mrlOffset, PelBuf &pDst, const ChannelType &channelType,                                                                                            const bool &enableBoundaryFilter = true );
  Void xPredIntraAng              ( const CPelBuf &pSrc, const int mrlOffset, PelBuf &pDst, const ChannelType &channelType, const UInt &dirMode, const ClpRng& clpRng, const Bool &bEnableEdgeFilters, const SPS& sps, const bool &enableBoundaryFilter = true );
  Pel  xGetPredValDc              ( const CPelBuf &pSrc, const int mrlOffset, const Size &dstSize );
  // Fast Template Matching Intra Prediction
  Bool xCheckFTMSearch          (const PredictionUnit &pu, const Area &searchArea);
  Void xPredIntraFTMReg         (const PredictionUnit &pu, const ComponentID &compID, FTMInfo &Info, Pel* Test, Pel* temp, const Int  DstStride, UInt RegType, Bool DownSample);
  Void xFTMLoop                 (FTMInfo &Info, const Int DstStride, Int LoopX, Int LoopY, Int end, const bool isLoopX, const Pel* Test, const Pel* tempCpy, const int sampling = 1);
  Void xFTMRedefineBoundary     (const PredictionUnit &pu, FTMBorderInfo &Border, Bool InLoop, Int LoopX = 0, Int LoopY = 0);
  Void xInitializeFTMInfo       (const Area &puArea, FTMInfo &Info);
  Void xObtainFTMstartInfo      (const PredictionUnit &pu, FTMBorderInfo &Border, UInt uiIntraTMMode);
  Void xFTMRandomPrediction     (PelBuf &pDst, const PredictionUnit &pu, const ComponentID &compID);
  Void xFTMAdaptivePrediction   (PelBuf &pDst, const PredictionUnit &pu, const ComponentID &compID, FTMPredInfo &Pred);
  Void xFTM3PredWeightedAverage (PelBuf &pDst, const PredictionUnit &pu, const ComponentID &compID, FTMPredInfo &Pred);
  Void xFTM2PredAverage         (PelBuf &pDst, const PredictionUnit &pu, const ComponentID &compID, FTMPredInfo &Pred);
  Void xFTM1PredDirect          (PelBuf &pDst, const PredictionUnit &pu, const ComponentID &compID, FTMPredInfo &Pred);
  void xPredIntraLumaNNModel    ( PelBuf &pDst, const PredictionUnit &pu, UInt uiTrueNNMode );
  void xGenerateNNModeList      (const PredictionUnit &pu, const CPelBuf& src, const int bitDepth, std::function<bool(Position)> available);
  void xGenerateOutputJointHiddenLayer
                                (const PredictionUnit &pu, const CPelBuf& src, const int bitDepth, std::function<bool(Position)> available);
  void set_pu_Identifier_NNS    (const PredictionUnit &pu);
  bool need_recalculation_for_NN(const PredictionUnit &pu);

  std::vector<UInt>                            m_modeListNN;

  std::vector<UInt>                            m_modeListDiffFilter;
  void set_pu_Identifier_diffFilter(const PredictionUnit &pu, const static_vector< UInt, FAST_UDI_MAX_RDMODE_NUM > m_RdModeList);
  bool need_recalculation_for_diffFilter(const PredictionUnit &pu, const static_vector< UInt, FAST_UDI_MAX_RDMODE_NUM > m_RdModeList);

  void xFillReferenceSamples      ( const CPelBuf &recoBuf,      Pel* refBufUnfiltered, const CompArea &area, const CodingUnit &cu );
  void xFilterReferenceSamples    ( const Pel* refBufUnfiltered, Pel* refBufFiltered, const CompArea &area, const SPS &sps, const CodingUnit &cu );

  // filtering (intra boundary filter)
  Void xIntraPredFilteringModeDGL ( const CPelBuf &pSrc, PelBuf &pDst, UInt uiMode );
  Void xIntraPredFilteringMode34  ( const CPelBuf &pSrc, PelBuf &pDst );
  Void xIntraPredFilteringMode02  ( const CPelBuf &pSrc, PelBuf &pDst );

  // dc filtering
  Void xDCPredFiltering           ( const CPelBuf &pSrc, PelBuf &pDst, const ChannelType &channelType );

  Void xReferenceFilter           ( const int doubleSize, const int origWeight, const int filterOrder, Pel *piRefVector, Pel *piLowPassRef );

  Void destroy                    ();

  Void xFilterGroup               ( Pel* pMulDst[], Int i, Pel const* const piSrc, Int iRecStride, Bool bAboveAvaillable, Bool bLeftAvaillable);

  struct MMLM_parameter
  {
    Int Inf;  // Inferio boundary
    Int Sup;  // Superior bounday
    Int a;
    Int b;
    Int shift;
  };

  Int xCalcLMParametersGeneralized(Int x, Int y, Int xx, Int xy, Int count, Int bitDepth, Int &a, Int &b, Int &iShift);
  Int xLMSampleClassifiedTraining (Int count, Int LumaSamples[], Int ChrmSamples[], Int GroupNum, Int bitDepth, MMLM_parameter parameters[]);
  Int xGetMMLMParameters          (const PredictionUnit &pu, const ComponentID compID, const CompArea& chromaArea, Int &numClass, MMLM_parameter parameters[]);
  Void xGetLMParameters           (const PredictionUnit &pu, const ComponentID compID, const CompArea& chromaArea, Int iPredType, Int& a, Int& b, Int& iShift);

public:
  IntraPrediction();
  virtual ~IntraPrediction();

  Void init                       (ChromaFormat chromaFormatIDC, const unsigned bitDepthY);

  // Angular Intra
  void predIntraNNModel           ( const ComponentID compId, PelBuf &piPred, const PredictionUnit &pu );
  void predIntraAng               ( const ComponentID compId, PelBuf &piPred, const PredictionUnit &pu, const bool &useFilteredPredSamples, bool isFirst1dPartition = true );
  Pel*  getPredictorPtr           (const ComponentID compID, const Bool bUseFilteredPredictions) { return m_piYuvExt[compID][bUseFilteredPredictions?PRED_BUF_FILTERED:PRED_BUF_UNFILTERED]; }
  Pel*  getPredictorPtrMrl        (const ComponentID compID, const Bool bUseFilteredPredictions) { return m_piYuvExtMrl[compID][bUseFilteredPredictions?PRED_BUF_FILTERED:PRED_BUF_UNFILTERED]; }
  // Fast Template Matching Intra
  Void predIntraFastTM            (PelBuf &pDst, const PredictionUnit &pu, const ComponentID &compID, UInt dstX, UInt dstY );
  // Cross-component Chroma
  Void predIntraChromaLM          (const ComponentID compID, PelBuf &piPred, const PredictionUnit &pu, const CompArea& chromaArea, Int intraDir);
  Void xGetLumaRecPixels          (const PredictionUnit &pu, CompArea chromaArea);
  Void addCrossColorResi          (const ComponentID compID, PelBuf &piPred, const TransformUnit &tu, const CPelBuf &pResiCb);

  /// set parameters from CU data for accessing intra data
  Void initIntraPatternChType     (const CodingUnit &cu, const CompArea &area, const Bool bFilterRefSamples);
  void jointPreparationForNNIntraPrediction
                                  (const PredictionUnit &pu);
  UInt getFinalNNMode             (const UInt finalModeIdx);

  static bool useFilteredIntraRefSamples( const ComponentID &compID, const PredictionUnit &pu, bool modeSpecific, const UnitArea &tuArea );
  static bool useBilateralFilter        ( const CompArea &area );
  static Bool useDPCMForFirstPassIntraEstimation(const PredictionUnit &pu, const UInt &uiDirMode);
};

//! \}

#endif // __INTRAPREDICTION__
