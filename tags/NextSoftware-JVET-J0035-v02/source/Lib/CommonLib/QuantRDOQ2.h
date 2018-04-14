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

/** \file     QuantRDOQ2.h
    \brief    RDOQ class (header)
*/

#ifndef __QUANTRDOQ2__
#define __QUANTRDOQ2__

#include "CommonDef.h"
#include "Unit.h"
#include "ChromaFormat.h"
#include "Contexts.h"
#include "ContextModelling.h"

#include "UnitPartitioner.h"

#include "Quant.h"

//! \ingroup CommonLib
//! \{

// ====================================================================================================================
// Class definition
// ====================================================================================================================

class QuantErrScale
{
public:
  QuantErrScale() : m_numWidths( 0 ), m_numHeights( 0 ) { memset( m_errScaleNoScalingList, 0, sizeof( m_errScaleNoScalingList ) ); }
  ~QuantErrScale(){ uninit(); }
  int  init( SizeType numWidths, SizeType numHeights );
  void uninit();

  Int& getErrScaleNoScalingList( SizeType wIdx, SizeType hIdx, UInt list, Int qp ) { CHECKD( !(wIdx < m_numWidths && hIdx < m_numHeights), "" ); return m_errScaleNoScalingList[list][qp][hIdx * m_numWidths + wIdx]; }
private:
  Int* m_errScaleNoScalingList[SCALING_LIST_NUM][SCALING_LIST_REM_NUM];
  int  m_numWidths;
  int  m_numHeights;
};

/// transform and quantization class
class QuantRDOQ2 : public Quant
{
public:
  QuantRDOQ2( const Quant* other );
  ~QuantRDOQ2();

  virtual Void init( UInt uiMaxTrSize,
                     Bool useRDOQ = false,
                     Bool useRDOQTS = false
#if T0196_SELECTIVE_RDOQ
                     , Bool useSelectiveRDOQ = false
#endif
                     );

public:
  Void setFlatScalingList              ( const Int maxLog2TrDynamicRange[MAX_NUM_CHANNEL_TYPE], const BitDepths &bitDepths );
  Void setScalingList                  ( ScalingList *scalingList, const Int maxLog2TrDynamicRange[MAX_NUM_CHANNEL_TYPE], const BitDepths &bitDepths);
  Void quant                           ( TransformUnit &tu, const ComponentID &compID, const CCoeffBuf &pSrc, TCoeff &uiAbsSum, const QpParam &cQP, const Ctx& ctx );

private:
  Int* xGetErrScaleCoeff               ( UInt list, UInt sizeX, UInt sizeY, Int qp ) { return m_errScale[sizeX][sizeY][list][qp]; };  //!< get Error Scale Coefficent
  //Int& xGetErrScaleCoeffNoScalingList  ( UInt list, UInt sizeX, UInt sizeY, Int qp ) { return m_errScaleNoScalingList[sizeX][sizeY][list][qp]; };  //!< get Error Scale Coefficent

  Void xInitScalingList                ( const QuantRDOQ2* other );
  Void xDestroyScalingList             ();
  Void xSetErrScaleCoeff               ( UInt list, UInt sizeX, UInt sizeY, Int qp, const Int maxLog2TrDynamicRange[MAX_NUM_CHANNEL_TYPE], const BitDepths &bitDepths );
  Void xSetErrScaleCoeffNoScalingList  ( UInt list, UInt wIdx, UInt hIdx, Int qp, const Int maxLog2TrDynamicRange[MAX_NUM_CHANNEL_TYPE], const BitDepths &bitDepths );
  Void xInitLastPosBitsTab             ( const CoeffCodingContext& cctx, const UInt uiWidth, const UInt uiHeight, const ChannelType chType, const FracBitsAccess& fracBits );

  inline Int64 xiGetICost              ( Int iRate ) const;
  inline Int   xGetIEPRate             () const;
  inline Int64 xiGetICRateCost         ( const UInt uiAbsLevel, const UInt ui16CtxNumOne, const UInt ui16CtxNumAbs, const UInt ui16AbsGoRice, const UInt c1Idx, const UInt c2Idx, const Bool useLimitedPrefixLength, const Int maxLog2TrDynamicRange, const FracBitsAccess& fracBits ) const;
  inline Int64 xiGetCostSigCoeffGroup  ( const UInt uiSignificanceCoeffGroup, const UInt ui16CtxNumSig, const FracBitsAccess& fracBits ) const;
  inline Int64 xiGetCostLast           ( const UInt uiPosX, const UInt uiPosY, const ChannelType chType ) const;
  inline Int64 xiGetCostSigCoef        ( const UInt uiSignificance, const UInt ui16CtxNumSig, const FracBitsAccess& fracBits ) const;

  template< Bool bSBH, Bool bUseScalingList >
  Int xRateDistOptQuantFast( TransformUnit &tu, const ComponentID &compID, const CCoeffBuf &pSrc, TCoeff &uiAbsSum, const QpParam &cQP, const Ctx &ctx );
  Int xRateDistOptQuant    ( TransformUnit &tu, const ComponentID &compID, const CCoeffBuf &pSrc, TCoeff &uiAbsSum, const QpParam &cQP, const Ctx &ctx, bool bUseScalingList );

#if HHI_SPLIT_PARALLELISM
  void copyState           ( const QuantRDOQ2& other );
#endif

private:
  Bool     m_bSBH;
  Int64    m_iLambda;

  QuantErrScale m_quantErrScale;

  Int m_lastBitsX[MAX_NUM_CHANNEL_TYPE][LAST_SIGNIFICANT_GROUPS];
  Int m_lastBitsY[MAX_NUM_CHANNEL_TYPE][LAST_SIGNIFICANT_GROUPS];
  Int *m_errScale[SCALING_LIST_SIZE_NUM][SCALING_LIST_SIZE_NUM][SCALING_LIST_NUM][SCALING_LIST_REM_NUM]; ///< array of quantization matrix coefficient 4x4

  Bool m_isErrListOwner;

};// END CLASS DEFINITION QuantRDOQ2
//! \}

#endif // __QUANTRDOQ2__
