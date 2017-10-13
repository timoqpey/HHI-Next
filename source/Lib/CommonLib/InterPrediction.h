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

/** \file     InterPrediction.h
    \brief    inter prediction class (header)
*/

#ifndef __INTERPREDICTION__
#define __INTERPREDICTION__


// Include files
#include "InterpolationFilter.h"
#include "WeightPrediction.h"

#include "Buffer.h"
#include "Unit.h"
#include "Picture.h"

#include "RdCost.h"
#include "ContextModelling.h"

// forward declaration
class Mv;

//! \ingroup CommonLib
//! \{

// ====================================================================================================================
// Class definition
// ====================================================================================================================

class InterPrediction : public WeightPrediction
{
private:
  static const int  m_LICShift      = 5;
  static const int  m_LICRegShift   = 7;
  static const int  m_LICShiftDiff  = 12;
  int               m_LICMultApprox[64];

protected:
  Pel*                 m_acYuvPred            [NUM_REF_PIC_LIST_01][MAX_NUM_COMPONENT];
  Pel*                 m_filteredBlock        [LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][MAX_NUM_COMPONENT];
  Pel*                 m_filteredBlockTmp     [LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][MAX_NUM_COMPONENT];

  InterpolationFilter  m_if;

  ChromaFormat         m_currChromaFormat;

  ComponentID          m_maxCompIDToPred;      ///< tells the predictor to only process the components up to (inklusive) this one - useful to skip chroma components during RD-search

  Pel*                 m_pGradX0;
  Pel*                 m_pGradY0;
  Pel*                 m_pGradX1;
  Pel*                 m_pGradY1;
  Int                  m_iRefListIdx;

  PelStorage           m_tmpObmcBuf;

  Pel*                 m_cYuvPredTempDMVR[MAX_NUM_COMPONENT];

  UInt                 m_uiaBIOShift[64];

  // motion compensation functions
#define BIO_FILTER_LENGTH                 6
#define BIO_FILTER_LENGTH_MINUS_1         (BIO_FILTER_LENGTH-1)
#define BIO_FILTER_HALF_LENGTH_MINUS_1    ((BIO_FILTER_LENGTH>>1)-1)

  Void          xGradFilterX    ( const Pel* piRefY, Int iRefStride, Pel*  piDstY, Int iDstStride, Int iWidth, Int iHeight, Int iMVyFrac, Int iMVxFrac, const Int bitDepth );
  Void          xGradFilterY    ( const Pel* piRefY, Int iRefStride, Pel*  piDstY, Int iDstStride, Int iWidth, Int iHeight, Int iMVyFrac, Int iMVxFrac, const Int bitDepth );
  inline Void   gradFilter2DVer ( const Pel* piSrc, Int iSrcStride, Int iWidth, Int iHeight, Int iDstStride, Pel*& rpiDst, Int iMv, const Int iShift );
  inline Void   gradFilter2DHor ( const Pel* piSrc, Int iSrcStride, Int iWidth, Int iHeight, Int iDstStride, Pel*& rpiDst, Int iMV, const Int iShift );
  inline Void   fracFilter2DHor ( const Pel* piSrc, Int iSrcStride, Int iWidth, Int iHeight, Int iDstStride, Pel*& rpiDst, Int iMV, const Int iShift );
  inline Void   fracFilter2DVer ( const Pel* piSrc, Int iSrcStride, Int iWidth, Int iHeight, Int iDstStride, Pel*& rpiDst, Int iMv, const Int iShift );
  inline Void   gradFilter1DHor ( const Pel* piSrc, Int iSrcStride, Int iWidth, Int iHeight, Int iDstStride, Pel*& rpiDst, Int iMV, const Int iShift );
  inline Void   gradFilter1DVer ( const Pel* piSrc, Int iSrcStride, Int iWidth, Int iHeight, Int iDstStride, Pel*& rpiDst, Int iMV, const Int iShift );

  inline Int64  divide64        ( Int64 numer, Int64 denom);
  inline Void   calcBlkGradient ( Int sx, Int sy, Int64 *arraysGx2, Int64 *arraysGxGy, Int64 *arraysGxdI, Int64 *arraysGy2, Int64 *arraysGydI, Int64 &sGx2, Int64 &sGy2, Int64 &sGxGy, Int64 &sGxdI, Int64 &sGydI, Int iWidth, Int iHeight);

  Pel  optical_flow_averaging   ( Int64 s1, Int64 s2, Int64 s3, Int64 s5, Int64 s6,
                                  Pel pGradX0, Pel pGradX1, Pel pGradY0, Pel pGradY1, Pel pSrcY0Temp, Pel pSrcY1Temp,
                                  const int shiftNum, const int offset, const Int64 limit, const Int64 denom_min_1, const Int64 denom_min_2, const ClpRng& clpRng );
  void applyBiOptFlow           ( const PredictionUnit &pu, const CPelUnitBuf &pcYuvSrc0, const CPelUnitBuf &pcYuvSrc1, const Int &iRefIdx0, const Int &iRefIdx1, PelUnitBuf &pcYuvDst, const BitDepths &clipBitDepths);

  Void xPredInterUni            ( PredictionUnit& pu, const RefPicList &eRefPicList, PelUnitBuf &pcYuvPred, const Bool &bi = false, const Bool &bBIOApplied = false );
  Void xPredInterBi             ( PredictionUnit& pu, PelUnitBuf &pcYuvPred, Bool obmc = false );
  Void xPredInterBlk            ( const ComponentID &compID, const PredictionUnit& pu, const Picture* refPic, const Mv & mv, PelUnitBuf &dstPic, const Bool &bi, const ClpRng& clpRng, const Bool &bBIOApplied = false, Int nFRUCMode = FRUC_MERGE_OFF, Bool doLic = true );
  Void xPredAffineBlk           ( const ComponentID &compID, const PredictionUnit& pu, const Picture* refPic, const Mv *_mv, PelUnitBuf &dstPic, const Bool &bi, const ClpRng& clpRng, const Bool &bBIOApplied = false );
  void xGetLICParams            ( const CodingUnit& cu, const ComponentID compID, const Picture& refPic, const Mv& mv, int& shift, int& scale, int& offset );
  void xLocalIlluComp           ( const PredictionUnit& pu, const ComponentID compID, const Picture& refPic, const Mv& mv, const bool biPred, PelBuf& dstBuf );

  Void xWeightedAverage         ( const PredictionUnit &pu, const CPelUnitBuf &pcYuvSrc0, const CPelUnitBuf &pcYuvSrc1, PelUnitBuf &pcYuvDst, const BitDepths &clipBitDepths, const ClpRngs& clpRngs, const Bool &bBIOApplied );

  static Bool xCheckIdenticalMotion( const PredictionUnit& pu );

  Void xSubPuMC                 ( PredictionUnit& pu, PelUnitBuf& predBuf, const RefPicList &eRefPicList = REF_PIC_LIST_X );
  Void xSubblockOBMC            ( const ComponentID eComp, PredictionUnit &pu, PelUnitBuf &pcYuvPredDst, PelUnitBuf &pcYuvPredSrc, Int iDir, Bool bOBMCSimp );
  Void xSubtractOBMC            ( PredictionUnit &pu, PelUnitBuf &pcYuvPredDst, PelUnitBuf &pcYuvPredSrc, Int iDir, Bool bOBMCSimp );
  Void xSubBlockMotionCompensation( PredictionUnit &pu, PelUnitBuf &pcYuvPred );

  Void destroy();

  MotionInfo      m_SubPuMiBuf   [( MAX_CU_SIZE * MAX_CU_SIZE ) >> ( MIN_CU_LOG2 << 1 )];
  MotionInfo      m_SubPuExtMiBuf[( MAX_CU_SIZE * MAX_CU_SIZE ) >> ( MIN_CU_LOG2 << 1 )];

  RdCost m_cFRUCRDCost;

  std::list<MvField> m_listMVFieldCand[2];
  RefPicList m_bilatBestRefPicList;
  Pel*   m_acYuvPredFrucTemplate[2][MAX_NUM_COMPONENT];   //0: top, 1: left
  Bool   m_bFrucTemplateAvailabe[2];

  Bool xFrucFindBlkMv           (PredictionUnit& pu, MergeCtx& mergeCtx );
  Bool xFrucRefineSubBlkMv      (PredictionUnit& pu, MergeCtx  mergeCtx, Bool bTM);

  Void xFrucCollectBlkStartMv   (PredictionUnit& pu, MergeCtx& mergeCtx, RefPicList eTargetRefList = REF_PIC_LIST_0, Int nTargetRefIdx = -1, AMVPInfo* pInfo = NULL);
  Void xFrucCollectSubBlkStartMv(PredictionUnit& pu, MergeCtx  mergeCtx, RefPicList eRefPicList , const MvField& rMvStart , Int nSubBlkWidth , Int nSubBlkHeight, Position basePuPos);
  UInt xFrucFindBestMvFromList  (MvField* pBestMvField, RefPicList& rBestRefPicList, PredictionUnit& pu, const MvField& rMvStart, Int nBlkWidth, Int nBlkHeight, Bool bTM, Bool bMvCost);
  UInt xFrucRefineMv            (MvField* pBestMvField, RefPicList eCurRefPicList, UInt uiMinCost, Int nSearchMethod, PredictionUnit& pu, const MvField& rMvStart, Int nBlkWidth, Int nBlkHeight, Bool bTM, Bool bMvCostZero = false);
  template<Int SearchPattern>
  UInt xFrucRefineMvSearch      (MvField* pBestMvField, RefPicList eCurRefPicList, PredictionUnit& pu, const MvField& rMvStart, Int nBlkWidth, Int nBlkHeight, UInt uiMinDist, Bool bTM, Int nSearchStepShift, UInt uiMaxSearchRounds = MAX_UINT, Bool bMvCostZero = false);

  UInt xFrucGetMvCost           (const Mv& rMvStart, const Mv& rMvCur, Int nSearchRange, Int nWeighting, UInt precShift );
  UInt xFrucGetBilaMatchCost    (PredictionUnit& pu, Int nWidth, Int nHeight, RefPicList eCurRefPicList, const MvField& rCurMvField, MvField& rPairMVField, UInt uiMVCost );
  UInt xFrucGetTempMatchCost    (PredictionUnit& pu, Int nWidth, Int nHeight, RefPicList eCurRefPicList, const MvField& rCurMvField, UInt uiMVCost );
  Void xFrucUpdateTemplate      (PredictionUnit& pu, Int nWidth, Int nHeight, RefPicList eCurRefPicList, const MvField& rCurMvField );


  Void xFrucInsertMv2StartList  (const MvField & rMvField, std::list<MvField> & rList,Bool setHighPrec);
  Bool xFrucIsInList            (const MvField & rMvField, std::list<MvField> & rList);

  Bool xFrucGetCurBlkTemplate   (PredictionUnit& pu, Int nCurBlkWidth , Int nCurBlkHeight);
  Bool xFrucIsTopTempAvailable  (PredictionUnit& pu);
  Bool xFrucIsLeftTempAvailable (PredictionUnit& pu);
  Int  xFrucGetSubBlkSize       (PredictionUnit& pu, Int nBlkWidth, Int nBlkHeight);

  Void xBIPMVRefine             (PredictionUnit& pu, RefPicList eRefPicList, Int iWidth, Int iHeight, const CPelUnitBuf &pcYuvOrg, UInt uiMaxSearchRounds, UInt nSearchStepShift, UInt& uiMinCost, Bool fullPel = true);
  UInt xDirectMCCost            (Int iBitDepth, Pel* pRef, UInt uiRefStride, const Pel* pOrg, UInt uiOrgStride, Int iWidth, Int iHeight);
  Void xPredInterLines          (const PredictionUnit& pu, const Picture* refPic, Mv &mv, PelUnitBuf &dstPic, const Bool &bi, const ClpRng& clpRng, Int offset );
  Void xFillPredBlckAndBorder   (const PredictionUnit& pu, RefPicList eRefPicList, Int iWidth, Int iHeight, PelBuf &cTmpY);
  Void xProcessDMVR             (      PredictionUnit& pu, PelUnitBuf &pcYuvDst, const ClpRngs &clpRngs, const bool bBIOApplied);

public:

  InterPrediction();
  virtual ~InterPrediction();

  Void    init                (ChromaFormat chromaFormatIDC);

  // inter
  Void    motionCompensation  (PredictionUnit &pu, PelUnitBuf& predBuf, const RefPicList &eRefPicList = REF_PIC_LIST_X);
  Void    motionCompensation  (PredictionUnit &pu, const RefPicList &eRefPicList = REF_PIC_LIST_X);
  Void    motionCompensation  (CodingUnit &cu,     const RefPicList &eRefPicList = REF_PIC_LIST_X);

  Void    subBlockOBMC        (CodingUnit      &cu);
  Void    subBlockOBMC        (PredictionUnit  &pu, PelUnitBuf *pDst = nullptr, Bool bOBMC4ME = false);

  Bool    deriveFRUCMV        (PredictionUnit &pu);
  Bool    frucFindBlkMv4Pred  (PredictionUnit& pu, RefPicList eTargetRefPicList, const Int nTargetRefIdx, AMVPInfo* pInfo = NULL);
};

//! \}

#endif // __INTERPREDICTION__
