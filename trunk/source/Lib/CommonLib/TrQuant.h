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

/** \file     TrQuant.h
    \brief    transform and quantization class (header)
*/

#ifndef __TRQUANT__
#define __TRQUANT__

#include "CommonDef.h"
#include "Unit.h"
#include "ChromaFormat.h"
#include "Contexts.h"
#include "ContextModelling.h"

#include "UnitPartitioner.h"

//! \ingroup CommonLib
//! \{

// ====================================================================================================================
// Constants
// ====================================================================================================================

#define QP_BITS                 15

typedef void FwdTrans(const TCoeff*, TCoeff*, Int, Int, Int, Int, Int);
typedef void InvTrans(const TCoeff*, TCoeff*, Int, Int, Int, Int, Int, const TCoeff, const TCoeff);

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// QP struct
struct QpParam
{
  Int Qp;
  Int per;
  Int rem;

private:

  QpParam(const Int           qpy,
          const ChannelType   chType,
          const Int           qpBdOffset,
          const Int           chromaQPOffset,
          const ChromaFormat  chFmt,
          const int           dqp );

public:

  QpParam(const TransformUnit& tu, const ComponentID &compID);

}; // END STRUCT DEFINITION QpParam

/// transform and quantization class
class TrQuant
{
public:
  TrQuant();
  ~TrQuant();

  // initialize class
  Void init                 ( UInt uiMaxTrSize,
                              Bool useRDOQ                = false,
                              Bool useRDOQTS              = false,
#if T0196_SELECTIVE_RDOQ
                              Bool useSelectiveRDOQ       = false,
#endif
                              Bool bEnc                   = false,
                              Bool useTransformSkipFast   = false,
                              bool use65IntraModes        = false,
                              bool rectTUs                = false
                              );

  UChar getEmtTrIdx( TransformUnit tu, const ComponentID compID );
  UChar getEmtMode ( TransformUnit tu, const ComponentID compID );

  Void FwdNsstNxN( Int* src, const UInt uiMode, const UInt uiIndex, const UInt uiSize );
  Void InvNsstNxN( Int* src, const UInt uiMode, const UInt uiIndex, const UInt uiSize );

protected:

  Void xFwdNsst( const TransformUnit &tu, const ComponentID compID );
  Void xInvNsst( const TransformUnit &tu, const ComponentID compID );

public:

  Void invTransformNxN  (TransformUnit &tu, const ComponentID &compID, PelBuf &pResi, const QpParam &cQPs);

  Void transformNxN     (TransformUnit &tu, const ComponentID &compID, const QpParam &cQP, TCoeff &uiAbsSum, const Ctx &ctx);
  Void rdpcmNxN         (TransformUnit &tu, const ComponentID &compID, const QpParam &cQP, TCoeff &uiAbsSum,       RDPCMMode &rdpcmMode);
  Void applyForwardRDPCM(TransformUnit &tu, const ComponentID &compID, const QpParam &cQP, TCoeff &uiAbsSum, const RDPCMMode &rdpcmMode);

  Void transformSkipQuantOneSample(TransformUnit &tu, const ComponentID &compID, const TCoeff &resiDiff, TCoeff &coeff,    const UInt &uiPos, const QpParam &cQP, const Bool bUseHalfRoundingPoint);
  Void invTrSkipDeQuantOneSample  (TransformUnit &tu, const ComponentID &compID, const TCoeff &pcCoeff,  Pel &reconSample, const UInt &uiPos, const QpParam &cQP);

  Void invRdpcmNxN(TransformUnit& tu, const ComponentID &compID, PelBuf &pcResidual);

  Void setLambdas(const Double lambdas[MAX_NUM_COMPONENT]) { for (UInt component = 0; component < MAX_NUM_COMPONENT; component++) m_lambdas[component] = lambdas[component]; }
  Void selectLambda(const ComponentID compIdx) { m_dLambda = m_lambdas[ MAP_CHROMA(compIdx) ]; }
  Void setLambda(const Double dLambda) { m_dLambda = dLambda; }
  Double getlambda() const { return m_dLambda; }
  Void setRDOQOffset( UInt uiRDOQOffset ) { m_uiRDOQOffset = uiRDOQOffset; }

  Void initScalingList                  ();
  Void destroyScalingList               ();
  Void setErrScaleCoeff                 ( UInt list, UInt sizeX, UInt sizeY, Int qp, const Int maxLog2TrDynamicRange[MAX_NUM_CHANNEL_TYPE], const BitDepths &bitDepths );
  Double* getErrScaleCoeff              ( UInt list, UInt sizeX, UInt sizeY, Int qp ) { return m_errScale             [sizeX][sizeY][list][qp]; };  //!< get Error Scale Coefficent
  Double& getErrScaleCoeffNoScalingList ( UInt list, UInt sizeX, UInt sizeY, Int qp ) { return m_errScaleNoScalingList[sizeX][sizeY][list][qp]; };  //!< get Error Scale Coefficent
  Int* getQuantCoeff                    ( UInt list, Int qp, UInt sizeX, UInt sizeY ) { return m_quantCoef            [sizeX][sizeY][list][qp]; };  //!< get Quant Coefficent
  Int* getDequantCoeff                  ( UInt list, Int qp, UInt sizeX, UInt sizeY ) { return m_dequantCoef          [sizeX][sizeY][list][qp]; };  //!< get DeQuant Coefficent
  Void setUseScalingList    ( Bool bUseScalingList){ m_scalingListEnabledFlag = bUseScalingList; };
  Bool getUseScalingList    (const UInt width, const UInt height, const Bool isTransformSkip){ return m_scalingListEnabledFlag && (!isTransformSkip || ((width == 4) && (height == 4))); };
  Void setFlatScalingList   (const Int maxLog2TrDynamicRange[MAX_NUM_CHANNEL_TYPE], const BitDepths &bitDepths);
  Void xsetFlatScalingList  ( UInt list, UInt sizeX, UInt sizeY, Int qp);
  Void xSetScalingListEnc   ( ScalingList *scalingList, UInt list, UInt size, Int qp);
  Void xSetScalingListDec   ( const ScalingList &scalingList, UInt list, UInt size, Int qp);
  Void setScalingList       ( ScalingList *scalingList, const Int maxLog2TrDynamicRange[MAX_NUM_CHANNEL_TYPE], const BitDepths &bitDepths);
  Void setScalingListDec    ( const ScalingList &scalingList);
  Void processScalingListEnc( Int *coeff, Int *quantcoeff, Int quantScales, UInt height, UInt width, UInt ratio, Int sizuNum, UInt dc);
  Void processScalingListDec( const Int *coeff, Int *dequantcoeff, Int invQuantScales, UInt height, UInt width, UInt ratio, Int sizuNum, UInt dc);

protected:
  TCoeff*  m_plTempCoeff;
#if RDOQ_CHROMA_LAMBDA
  Double   m_lambdas[MAX_NUM_COMPONENT];
#endif
  Double   m_dLambda;
  UInt     m_uiRDOQOffset;
  UInt     m_uiMaxTrSize;
  Bool     m_bEnc;
  Bool     m_useRDOQ;
  Bool     m_useRDOQTS;
#if T0196_SELECTIVE_RDOQ
  Bool     m_useSelectiveRDOQ;
#endif
  Bool     m_useTransformSkipFast;

  bool     m_use65IntraModes;
  bool     m_rectTUs;

  Bool     m_scalingListEnabledFlag;

  Int      *m_quantCoef            [SCALING_LIST_SIZE_NUM][SCALING_LIST_SIZE_NUM][SCALING_LIST_NUM][SCALING_LIST_REM_NUM]; ///< array of quantization matrix coefficient 4x4
  Int      *m_dequantCoef          [SCALING_LIST_SIZE_NUM][SCALING_LIST_SIZE_NUM][SCALING_LIST_NUM][SCALING_LIST_REM_NUM]; ///< array of dequantization matrix coefficient 4x4
  Double   *m_errScale             [SCALING_LIST_SIZE_NUM][SCALING_LIST_SIZE_NUM][SCALING_LIST_NUM][SCALING_LIST_REM_NUM]; ///< array of quantization matrix coefficient 4x4
  Double    m_errScaleNoScalingList[SCALING_LIST_SIZE_NUM][SCALING_LIST_SIZE_NUM][SCALING_LIST_NUM][SCALING_LIST_REM_NUM]; ///< array of quantization matrix coefficient 4x4

private:

  // temporary buffers for RDOQ
  Double m_pdCostCoeff        [MAX_TU_SIZE * MAX_TU_SIZE];
  Double m_pdCostSig          [MAX_TU_SIZE * MAX_TU_SIZE];
  Double m_pdCostCoeff0       [MAX_TU_SIZE * MAX_TU_SIZE];
  Double m_pdCostCoeffGroupSig[(MAX_TU_SIZE * MAX_TU_SIZE) >> MLS_CG_SIZE]; // even if CG size is 2 (if one of the sides is 2) instead of 4, there should be enough space
  Int    m_rateIncUp          [MAX_TU_SIZE * MAX_TU_SIZE];
  Int    m_rateIncDown        [MAX_TU_SIZE * MAX_TU_SIZE];
  Int    m_sigRateDelta       [MAX_TU_SIZE * MAX_TU_SIZE];
  TCoeff m_deltaU             [MAX_TU_SIZE * MAX_TU_SIZE];

  // needed for NSST
  TCoeff m_tempMatrix         [64];

  // forward Transform
  Void xT        ( const Int channelBitDepth, Bool useDST, const Pel* piBlkResi, UInt uiStride, TCoeff* psCoeff, Int iWidth, Int iHeight, const Int maxLog2TrDynamicRange, UChar ucMode, UChar ucTrIdx );

  void (*m_fTr ) ( const int bitDepth, const Pel *residual, size_t stride, TCoeff *coeff, size_t width, size_t height, bool useDST, const int maxLog2TrDynamicRange );
  void (*m_fITr) ( const int bitDepth, const TCoeff *coeff, Pel *residual, size_t stride, size_t width, size_t height, bool useDST, const int maxLog2TrDynamicRange );

  Void signBitHidingHDQ (TCoeff* pQCoef, const TCoeff* pCoef, TCoeff* deltaU, const CoeffCodingContext& cctx, const Int maxLog2TrDynamicRange);

  // skipping Transform
  Void xTransformSkip   (const TransformUnit &tu, const ComponentID &compID, const CPelBuf &resi, TCoeff* psCoeff);

  // quantization
  Void xQuant           (TransformUnit &tu, const ComponentID &compID, const CCoeffBuf &pSrc, TCoeff &uiAbsSum, const QpParam &cQP, const Ctx& ctx);

#if T0196_SELECTIVE_RDOQ
  Bool xNeedRDOQ        (TransformUnit &tu, const ComponentID &compID, const CCoeffBuf &pSrc, const QpParam &cQP);
#endif

  // RDOQ functions
  Void xRateDistOptQuant(TransformUnit &tu, const ComponentID &compID, const CCoeffBuf &pSrc, TCoeff &uiAbsSum, const QpParam &cQP, const Ctx &ctx);


  inline UInt xGetCodedLevel  ( Double&             rd64CodedCost,
                                Double&             rd64CodedCost0,
                                Double&             rd64CodedCostSig,
                                Intermediate_Int    lLevelDouble,
                                UInt                uiMaxAbsLevel,
                                const BinFracBits*  fracBitsSig,
                                const BinFracBits&  fracBitsOne,
                                const BinFracBits&  fracBitsAbs,
                                const bool          useAltRC,
                                UShort              ui16AbsGoRice,
                                UInt                c1Idx,
                                UInt                c2Idx,
                                Int                 iQBits,
                                Double              errorScale,
                                Bool                bLast,
                                Bool                useLimitedPrefixLength,
                                const Int           maxLog2TrDynamicRange
                              ) const;
  inline Int xGetICRate  ( const UInt         uiAbsLevel,
                           const BinFracBits& fracBitsOne,
                           const BinFracBits& fracBitsAbs,
                           const bool         useAltRC,
                           const UShort       ui16AbsGoRice,
                           const UInt         c1Idx,
                           const UInt         c2Idx,
                           const Bool         useLimitedPrefixLength,
                           const Int          maxLog2TrDynamicRange
                          ) const;

  inline Double xGetRateLast         ( const int* lastBitsX, const int* lastBitsY,
                                       unsigned        PosX, unsigned   PosY                              ) const;

  inline Double xGetRateSigCoeffGroup( const BinFracBits& fracBitsSigCG,   unsigned uiSignificanceCoeffGroup ) const;

  inline Double xGetRateSigCoef      ( const BinFracBits& fracBitsSig,     unsigned uiSignificance           ) const;

  inline Double xGetICost            ( Double dRate                                                      ) const;
  inline Double xGetIEPRate          (                                                                   ) const;


  // dequantization
  Void xDeQuant( const TransformUnit &tu,
                       CoeffBuf      &dstCoeff,
                 const ComponentID   &compID,
                 const QpParam       &cQP      );

  // inverse transform
  Void xIT(      const Int           &channelBitDepth,
                 const Bool          &useDST,
                 const CCoeffBuf     &pCoeff,
                       PelBuf        &pResidual,
                 const Int           &maxLog2TrDynamicRange,
                 const UChar          ucMode,
                 const UChar          ucTrIdx);

  // inverse skipping transform
  Void xITransformSkip(
                 const CCoeffBuf     &plCoef,
                       PelBuf        &pResidual,
                 const TransformUnit &tu,
                 const ComponentID   &component);


#ifdef TARGET_SIMD_X86
  template<X86_VEXT vext>
  Void _initTrQuantX86();
  Void initTrQuantX86();
#endif
};// END CLASS DEFINITION TrQuant

//! \}

#endif // __TRQUANT__
