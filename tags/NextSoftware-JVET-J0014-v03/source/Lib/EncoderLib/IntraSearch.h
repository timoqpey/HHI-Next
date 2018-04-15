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

/** \file     IntraSearch.h
    \brief    intra search class (header)
*/

#ifndef __INTRASEARCH__
#define __INTRASEARCH__

// Include files

#include "CABACWriter.h"
#include "EncCfg.h"

#include "CommonLib/IntraPrediction.h"
#include "CommonLib/CrossCompPrediction.h"
#include "CommonLib/TrQuant.h"
#include "CommonLib/Unit.h"
#include "CommonLib/RdCost.h"
#include "CommonLib/BilateralFilter.h"
#if THRESHOLDING
#include "CommonLib/ThresholdingSearch.h"
#endif
#include "CommonLib/DiffusionFilter.h"

//! \ingroup EncoderLib
//! \{

// ====================================================================================================================
// Class definition
// ====================================================================================================================

class EncModeCtrl;

/// encoder search class
class IntraSearch : public IntraPrediction, CrossComponentPrediction
{
private:
  EncModeCtrl    *m_modeCtrl; //we need this to call the saveLoadTag functions for the EMT
  Pel*            m_pSharedPredTransformSkip[MAX_NUM_TBLOCKS];

  XUCache         m_unitCache;

  CodingStructure ****m_pSplitCS;
  CodingStructure ****m_pFullCS;

  CodingStructure ***m_pTempCS;
  CodingStructure ***m_pBestCS;

  CodingStructure **m_pSaveCS;

  Pel             **m_pLMMFPredSaved;

  static_vector<UInt, FAST_UDI_MAX_RDMODE_NUM> m_ModeList_NNS;

  struct tu_Diff_Datum
  {
    tu_Diff_Datum() : listnr(-1), intraModeIdx(-1), intraMode(-1), diffFilter(-1) {}
    tu_Diff_Datum(UInt listnr_curr, UInt intramodeIdx_curr, UInt intra_mode_curr,  UInt diffFilter_curr) : listnr(listnr_curr), intraModeIdx(intramodeIdx_curr), intraMode(intra_mode_curr), diffFilter(diffFilter_curr) {}
    Int listnr;
    Int intraModeIdx;
    Int intraMode;
    Int diffFilter;
  };
  std::vector<tu_Diff_Datum> m_DiffFilterList;
  std::vector<tu_Diff_Datum> bestDiffOpt;

  static_vector< UInt, FAST_UDI_MAX_RDMODE_NUM > uiRdModeListFirst;
  bool b_StoreFirstDir;

  //cost variables for the EMT algorithm and new modes list
  Double m_bestModeCostStore[4];                                    // RD cost of the best mode for each PU using DCT2
  Double m_modeCostStore    [4][NUM_LUMA_MODE];                         // RD cost of each mode for each PU using DCT2
  UInt   m_savedRdModeList  [4][NUM_LUMA_MODE], m_savedNumRdModes[4];

  static_vector<UInt,   FAST_UDI_MAX_RDMODE_NUM> m_savedRdModeListFor1dPartitions;
  static_vector<Double, FAST_UDI_MAX_RDMODE_NUM> m_savedRdModesCostsFor1dPartitions;
  Double best2dDct2Cost;
  static_vector< Double, FAST_UDI_MAX_RDMODE_NUM > uiRdModeListCosts;
  UInt numberOfLinesCompleted;
  static_vector<UInt, FAST_UDI_MAX_RDMODE_NUM> m_bestNumberOfLinesPerIntraModeGroup;
  static_vector<UInt, FAST_UDI_MAX_RDMODE_NUM> m_intraModeGroup;

  static_vector<UInt,   FAST_UDI_MAX_RDMODE_NUM> m_uiSavedRdModeListNSST;
  UInt                                           m_uiSavedNumRdModesNSST;
  static_vector<UInt,   FAST_UDI_MAX_RDMODE_NUM> m_uiSavedHadModeListNSST;
  static_vector<Double, FAST_UDI_MAX_RDMODE_NUM> m_dSavedModeCostNSST;
  static_vector<Double, FAST_UDI_MAX_RDMODE_NUM> m_dSavedHadListNSST;

protected:
  // interface to option
  EncCfg*         m_pcEncCfg;

  // interface to classes
  TrQuant*        m_pcTrQuant;
  RdCost*         m_pcRdCost;
#if THRESHOLDING
  ThresholdingSearch*
                  m_pcThresholding;
#endif
  DiffusionFilter*
                  m_DiffusionFilter;

  BilateralFilter*
                  m_bilateralFilter;

  // RD computation
  CABACWriter*    m_CABACEstimator;
  CtxCache*       m_CtxCache;

  Bool            m_isInitialized;

public:

  IntraSearch();
  ~IntraSearch();

  Void init                       ( EncCfg*        pcEncCfg,
                                    DiffusionFilter*  diffusionFilter,
#if THRESHOLDING
                                    ThresholdingSearch* pcThresholding,
#endif
                                    TrQuant*       pcTrQuant,
                                    RdCost*        pcRdCost,
                                    BilateralFilter*
                                                   bilateralFilter,
                                    CABACWriter*   CABACEstimator,
                                    CtxCache*      ctxCache,
                                    const UInt     maxCUWidth,
                                    const UInt     maxCUHeight,
                                    const UInt     maxTotalCUDepth
                                  );

  Void destroy                    ();

  CodingStructure****getSplitCSBuf() { return m_pSplitCS; }
  CodingStructure****getFullCSBuf () { return m_pFullCS; }
  CodingStructure  **getSaveCSBuf () { return m_pSaveCS; }

  void setModeCtrl                (EncModeCtrl *modeCtrl) { m_modeCtrl = modeCtrl; }

public:

  Void estIntraPredLumaQT         ( CodingUnit &cu, Partitioner& pm, double bestCostSoFar = MAX_DOUBLE );
  void FastDiffFilter(PredictionUnit &pu, const static_vector<UInt, FAST_UDI_MAX_RDMODE_NUM> uiRdModeList, static_vector<Double, FAST_UDI_MAX_RDMODE_NUM>& CandHadList, const TempCtx& ctxStartIntraMode, const double sqrtLambdaForFirstPass, std::vector<tu_Diff_Datum> & bestDiffOpt );
  double bestCostnoDiff;

  Void setStoreFirstDir(bool b) { b_StoreFirstDir = b; }
  Bool getStoreFirstDir() { return b_StoreFirstDir; }
  Void estIntraPredChromaQT       (CodingUnit &cu, Partitioner& pm);
  Void IPCMSearch                 (CodingStructure &cs, Partitioner& partitioner);

protected:

  // -------------------------------------------------------------------------------------------------------------------
  // T & Q & Q-1 & T-1
  // -------------------------------------------------------------------------------------------------------------------

  Void xEncPCM                    (CodingStructure &cs, Partitioner& partitioner, const ComponentID &compID);

  // -------------------------------------------------------------------------------------------------------------------
  // Intra search
  // -------------------------------------------------------------------------------------------------------------------

  Void xEncIntraHeader            (CodingStructure &cs, Partitioner& pm, const Bool &bLuma, const Bool &bChroma);
  Void xEncSubdivCbfQT            (CodingStructure &cs, Partitioner& pm, const Bool &bLuma, const Bool &bChroma);
  UInt64 xGetIntraFracBitsQT      (CodingStructure &cs, Partitioner& pm, const Bool &bLuma, const Bool &bChroma);

  UInt64 xGetIntraFracBitsQTChroma(TransformUnit& tu, const ComponentID &compID);
  Void xEncCoeffQT                (CodingStructure &cs, Partitioner& pm, const ComponentID &compID);

  UInt64 xFracModeBitsIntra       (PredictionUnit &pu, const UInt &uiMode, const ChannelType &compID);
  UInt64 xFracModeBitsIntraWithDiffLuma (PredictionUnit &pu, const UInt &uiMode);

  Void xIntraCodingTUBlock        (TransformUnit &tu, const ComponentID &compID, const Bool &checkCrossCPrediction, Distortion& ruiDist, const Int &default0Save1Load2 = 0, UInt* numSig = nullptr );

  ChromaCbfs xRecurIntraChromaCodingQT  (CodingStructure &cs, Partitioner& pm);

#if HHI_RQT_INTRA_SPEEDUP
  Void xRecurIntraCodingLumaQT    ( CodingStructure &cs, Partitioner& pm, const Bool &checkFirst, int savedEmtIndex = -1, double bestCostSoFar = MAX_DOUBLE );
#else
  Void xRecurIntraCodingLumaQT    ( CodingStructure &cs, Partitioner& pm, double bestCostSoFar = MAX_DOUBLE );
#endif


  void encPredIntraDPCM( const ComponentID &compID, PelBuf &pOrg, PelBuf &pDst, const UInt &uiDirMode );
  static bool useDPCMForFirstPassIntraEstimation( const PredictionUnit &pu, const UInt &uiDirMode );
};// END CLASS DEFINITION EncSearch

//! \}

#endif // __ENCSEARCH__
