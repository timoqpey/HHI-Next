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

/** \file     QuantRDOQ2.cpp
    \brief    transform and quantization class
*/

#include "QuantRDOQ2.h"
#include "TrQuant_EMT.h"

#include "UnitTools.h"
#include "ContextModelling.h"
#include "CodingStructure.h"
#include "CrossCompPrediction.h"

#include "dtrace_buffer.h"

#include <stdlib.h>
#include <limits>
#include <memory.h>


#include <assert.h>

// ====================================================================================================================
// Constants
// ====================================================================================================================

#define COEFF_ERR_SCALE_PRECISION_BITS 20

static const UChar nextc1[4] = {0, 2, 3, 3};
static const UChar nextGoRice[5] = {1, 2, 3, 4, 4};

//! \}


int QuantErrScale::init( SizeType numWidths, SizeType numHeights )
{
  const Int minimumQp = 0;
  const Int maximumQp = SCALING_LIST_REM_NUM;

  for( UInt list = 0; list < SCALING_LIST_NUM; list++ )
  {
    for( Int qp = minimumQp; qp < maximumQp; qp++ )
    {
      m_errScaleNoScalingList[list][qp] = new Int[numWidths * numHeights];
    }
  }
  m_numWidths = numWidths;
  m_numHeights = numHeights;
  return 0;
}

void QuantErrScale::uninit()
{
  const Int minimumQp = 0;
  const Int maximumQp = SCALING_LIST_REM_NUM;

  for( UInt list = 0; list < SCALING_LIST_NUM; list++ )
  {
    for( Int qp = minimumQp; qp < maximumQp; qp++ )
    {
      if( m_errScaleNoScalingList[list][qp] )
        delete[] m_errScaleNoScalingList[list][qp];
    }
  }
}


QuantRDOQ2::QuantRDOQ2() : m_bSBH( true ), m_iLambda( 0 )
{
  xInitScalingList();
}

QuantRDOQ2::~QuantRDOQ2()
{
  xDestroyScalingList();
}


Void QuantRDOQ2::init( UInt uiMaxTrSize, Bool useRDOQ, Bool useRDOQTS, UInt uiAltResiCompId,
#if T0196_SELECTIVE_RDOQ
                       Bool useSelectiveRDOQ
#endif
                       )
{
#if T0196_SELECTIVE_RDOQ
  Quant::init( uiMaxTrSize, useRDOQ, useRDOQTS, uiAltResiCompId, useSelectiveRDOQ );
#else
  Quant::init( uiMaxTrSize, useRDOQ, useRDOQTS, uiAltResiCompId );
#endif

  assert( gp_sizeIdxInfo );
  m_quantErrScale.init( gp_sizeIdxInfo->numWidths(), gp_sizeIdxInfo->numHeights() );
}


Void QuantRDOQ2::setFlatScalingList( const Int maxLog2TrDynamicRange[MAX_NUM_CHANNEL_TYPE], const BitDepths &bitDepths )
{
  Quant::setFlatScalingList( maxLog2TrDynamicRange, bitDepths );

  const Int minimumQp = 0;
  const Int maximumQp = SCALING_LIST_REM_NUM;

  for( UInt sizeX = 0; sizeX < gp_sizeIdxInfo->numWidths(); sizeX++ )
  {
    for( UInt sizeY = 0; sizeY < gp_sizeIdxInfo->numHeights(); sizeY++ )
    {
      for( UInt list = 0; list < SCALING_LIST_NUM; list++ )
      {
        for( Int qp = minimumQp; qp < maximumQp; qp++ )
        {
          xSetErrScaleCoeffNoScalingList( list, sizeX, sizeY, qp, maxLog2TrDynamicRange, bitDepths );
        }
      }
    }
  }
}


Void QuantRDOQ2::setScalingList( ScalingList *scalingList, const Int maxLog2TrDynamicRange[MAX_NUM_CHANNEL_TYPE], const BitDepths &bitDepths )
{
  Quant::setScalingList( scalingList, maxLog2TrDynamicRange, bitDepths );

  const Int minimumQp = 0;
  const Int maximumQp = SCALING_LIST_REM_NUM;

  for( UInt size = 0; size < SCALING_LIST_SIZE_NUM; size++ )
  {
    for( UInt list = 0; list < SCALING_LIST_NUM; list++ )
    {
      for( Int qp = minimumQp; qp < maximumQp; qp++ )
      {
        xSetErrScaleCoeff( list, size, size, qp, maxLog2TrDynamicRange, bitDepths );
      }
    }
  }
  setFlatScalingList( maxLog2TrDynamicRange, bitDepths );
}


Void QuantRDOQ2::quant( TransformUnit &tu, const ComponentID &compID, const CCoeffBuf &pSrc, TCoeff &uiAbsSum, const QpParam &cQP, const Ctx& ctx )
{
  const CompArea &rect = tu.blocks[compID];
  const UInt uiWidth   = rect.width;
  const UInt uiHeight  = rect.height;
  const Bool useTransformSkip = tu.transformSkip[compID];

  Bool useRDOQ = useTransformSkip ? m_useRDOQTS : m_useRDOQ;
  useRDOQ &= uiWidth > 2;
  useRDOQ &= uiHeight > 2;
  if( useRDOQ && ( isLuma( compID ) || RDOQ_CHROMA ) )
  {
#if T0196_SELECTIVE_RDOQ
    const CCoeffBuf &piCoef = pSrc;
    if( !m_useSelectiveRDOQ || xNeedRDOQ( tu, compID, piCoef, cQP ) )
    {
#endif
      Bool enableScalingLists = getUseScalingList( uiWidth, uiHeight, tu.transformSkip[compID] );
      xRateDistOptQuant( tu, compID, pSrc, uiAbsSum, cQP, ctx, enableScalingLists );
#if T0196_SELECTIVE_RDOQ
    }
    else
    {
      CoeffBuf piQCoef = tu.getCoeffs( compID );
      piQCoef.fill( 0 );
      uiAbsSum = 0;
    }
#endif
  }
  else
  {
    Quant::quant( tu, compID, pSrc, uiAbsSum, cQP, ctx );
  }
}

/**
* \brief Calculating quant. error scaling factors for default quant. parameter
*/

Void QuantRDOQ2::xSetErrScaleCoeffNoScalingList( UInt list, UInt wIdx, UInt hIdx, Int qp, const Int maxLog2TrDynamicRange[MAX_NUM_CHANNEL_TYPE], const BitDepths &bitDepths )
{
  const Int width               = gp_sizeIdxInfo->sizeFrom( wIdx );
  const Int height              = gp_sizeIdxInfo->sizeFrom( hIdx );
  const ChannelType channelType = ( ( list == 0 ) || ( list == MAX_NUM_COMPONENT ) ) ? CHANNEL_TYPE_LUMA : CHANNEL_TYPE_CHROMA;
  const Int channelBitDepth     = bitDepths.recon[channelType];
  const Int iTransformShift     = getTransformShift( channelBitDepth, Size( width, height ), maxLog2TrDynamicRange[channelType] );
  const Bool needsSrqt2         = needsBlockSizeQuantScale( Size( width, height ) );

  Double dTransShift = (Double)iTransformShift + ( needsSrqt2 ? -0.5 : 0.0 );
  Double dErrScale   = (Double)( 1 << SCALE_BITS );                   // Compensate for scaling of bitcount in Lagrange cost function
  dErrScale          = dErrScale*pow( 2.0, ( -/*2.0**/( dTransShift ) ) );   // Compensate for scaling through forward transform

  Int QStep = ( needsSrqt2 ? ( ( g_quantScales[qp] * 181 ) >> 7 ) : g_quantScales[qp] );

  Double errScale = dErrScale / QStep /*/ QStep*/ / ( 1 << DISTORTION_PRECISION_ADJUSTMENT( /*2 **/ ( bitDepths.recon[channelType] - 8 ) ) );
  m_quantErrScale.getErrScaleNoScalingList( wIdx, hIdx, list, qp ) = (Int)( errScale * (Double)( 1 << COEFF_ERR_SCALE_PRECISION_BITS ) );
}

/**
* \brief Calculating quant. error scaling factors from scaling list
*        NOTE!!! Err.Scale init from scaling list works only for HEVC mode
*/
Void QuantRDOQ2::xSetErrScaleCoeff( UInt list, UInt sizeX, UInt sizeY, Int qp, const Int maxLog2TrDynamicRange[MAX_NUM_CHANNEL_TYPE], const BitDepths &bitDepths )
{
  const Int width               = g_scalingListSizeX[sizeX];
  const Int height              = g_scalingListSizeX[sizeY];
  const ChannelType channelType = ( ( list == 0 ) || ( list == MAX_NUM_COMPONENT ) ) ? CHANNEL_TYPE_LUMA : CHANNEL_TYPE_CHROMA;
  const Int channelBitDepth     = bitDepths.recon[channelType];
  const Int iTransformShift     = getTransformShift( channelBitDepth, Size( width, height ), maxLog2TrDynamicRange[channelType] );
  const Double dTransShift      = (Double)iTransformShift;
  const UInt uiMaxNumCoeff      = g_scalingListSizeX[sizeX] * g_scalingListSizeX[sizeY];
  const Int *piQuantCoeff       = getQuantCoeff( list, qp, sizeX, sizeY );
  Int *piErrScale               = xGetErrScaleCoeff( list, sizeX, sizeY, qp );

  Double dErrScale = (Double)( 1 << SCALE_BITS );                   // Compensate for scaling of bitcount in Lagrange cost function
  dErrScale = dErrScale*pow( 2.0, ( -/*2.0**/( dTransShift ) ) );   // Compensate for scaling through forward transform

  for( UInt i = 0; i < uiMaxNumCoeff; i++ )
  {
    Int QStep = piQuantCoeff[i];
    Double errScale = dErrScale / QStep /*/ QStep*/ / ( 1 << DISTORTION_PRECISION_ADJUSTMENT( /*2 **/ ( bitDepths.recon[channelType] - 8 ) ) );
    piErrScale[i] = (Int)( errScale * (Double)( 1 << COEFF_ERR_SCALE_PRECISION_BITS ) );
  }
}


/** initialization process of scaling list array
*/
Void QuantRDOQ2::xInitScalingList()
{
  for( UInt sizeIdX = 0; sizeIdX < SCALING_LIST_SIZE_NUM; sizeIdX++ )
  {
    for( UInt sizeIdY = 0; sizeIdY < SCALING_LIST_SIZE_NUM; sizeIdY++ )
    {
      for( UInt qp = 0; qp < SCALING_LIST_REM_NUM; qp++ )
      {
        for( UInt listId = 0; listId < SCALING_LIST_NUM; listId++ )
        {
          m_errScale[sizeIdX][sizeIdY][listId][qp] = new Int[g_scalingListSizeX[sizeIdX] * g_scalingListSizeX[sizeIdY]];
        } // listID loop
      }
    }
  }
}

/** destroy quantization matrix array
*/
Void QuantRDOQ2::xDestroyScalingList()
{
  for( UInt sizeIdX = 0; sizeIdX < SCALING_LIST_SIZE_NUM; sizeIdX++ )
  {
    for( UInt sizeIdY = 0; sizeIdY < SCALING_LIST_SIZE_NUM; sizeIdY++ )
    {
      for( UInt listId = 0; listId < SCALING_LIST_NUM; listId++ )
      {
        for( UInt qp = 0; qp < SCALING_LIST_REM_NUM; qp++ )
        {
          if( m_errScale[sizeIdX][sizeIdY][listId][qp] )
          {
            delete[] m_errScale[sizeIdX][sizeIdY][listId][qp];
          }
        }
      }
    }
  }
}


inline Int64 QuantRDOQ2::xiGetICost( Int iRate ) const
{
  return m_iLambda * iRate;
}

inline Int QuantRDOQ2::xGetIEPRate() const
{
  return 32768;
}

#define LEVEL_FRAC_BITS(_id,_b) fracBits.getFracBitsArray(_id).intBits[_b]

/** Calculates the cost for specific absolute transform level
* \param uiAbsLevel scaled quantized level
* \param ui16CtxNumOne current ctxInc for coeff_abs_level_greater1 (1st bin of coeff_abs_level_minus1 in AVC)
* \param ui16CtxNumAbs current ctxInc for coeff_abs_level_greater2 (remaining bins of coeff_abs_level_minus1 in AVC)
* \param ui16AbsGoRice Rice parameter for coeff_abs_level_minus3
* \returns cost of given absolute transform level
*/
template< UInt altResiCompId >
inline Int64 QuantRDOQ2::xiGetICRateCost( const UInt uiAbsLevel,
                                          const UInt ui16CtxNumOne,
                                          const UInt ui16CtxNumAbs,
                                          const UInt ui16AbsGoRice,
                                          const UInt c1Idx,
                                          const UInt c2Idx,
                                          const Bool useLimitedPrefixLength,
                                          const Int  maxLog2TrDynamicRange,
                                          const FracBitsAccess& fracBits
                                          ) const
{
  Int iRate = xGetIEPRate();
  UInt baseLevel = ( c1Idx < C1FLAG_NUMBER ) ? ( 2 + ( c2Idx < C2FLAG_NUMBER ) ) : 1;

  if( uiAbsLevel >= baseLevel ) {
    Int symbol = uiAbsLevel - baseLevel;
    UInt length;

    int threshold;
    if( altResiCompId )
      threshold = g_auiGoRiceRange[ui16AbsGoRice];
    else
      threshold = COEF_REMAIN_BIN_REDUCTION;

    if( symbol < ( threshold << ui16AbsGoRice ) )
    {
      length = symbol >> ui16AbsGoRice;
      iRate += ( length + 1 + ui16AbsGoRice ) << 15;
    }
    else if( useLimitedPrefixLength )
    {
      const UInt maximumPrefixLength = ( 32 - ( COEF_REMAIN_BIN_REDUCTION + maxLog2TrDynamicRange ) );

      UInt prefixLength = 0;
      UInt suffix = ( symbol >> ui16AbsGoRice ) - COEF_REMAIN_BIN_REDUCTION;

      while( ( prefixLength < maximumPrefixLength ) && ( suffix >( ( 2 << prefixLength ) - 2 ) ) )
      {
        prefixLength++;
      }

      const UInt suffixLength = ( prefixLength == maximumPrefixLength ) ? ( maxLog2TrDynamicRange - ui16AbsGoRice ) : ( prefixLength + 1/*separator*/ );

      iRate += ( COEF_REMAIN_BIN_REDUCTION + prefixLength + suffixLength + ui16AbsGoRice ) << 15;
    }
    else
    {
      length = ui16AbsGoRice;
      symbol = symbol - ( threshold << ui16AbsGoRice );
      while( symbol >= ( 1 << length ) )
      {
        symbol -= ( 1 << ( length++ ) );
      }
      iRate += ( threshold + length + 1 - ui16AbsGoRice + length ) << 15;
    }
    if( c1Idx < C1FLAG_NUMBER ){
      iRate += LEVEL_FRAC_BITS( ui16CtxNumOne, 1 );

      if( c2Idx < C2FLAG_NUMBER ){
        iRate += LEVEL_FRAC_BITS( ui16CtxNumAbs, 1 );
      }
    }
  }
  else {
    if( uiAbsLevel == 1 ) {
      iRate += LEVEL_FRAC_BITS( ui16CtxNumOne, 0 );
    }
    else if( uiAbsLevel == 2 ) {
      iRate += LEVEL_FRAC_BITS( ui16CtxNumOne, 1 );
      iRate += LEVEL_FRAC_BITS( ui16CtxNumAbs, 0 );
    }
    else {
      assert( 0 );
    }
  }
  return xiGetICost( iRate );
}

inline Int64 QuantRDOQ2::xiGetCostSigCoeffGroup( const UInt uiSignificanceCoeffGroup, const UInt ui16CtxNumSig, const FracBitsAccess& fracBits ) const
{
  return xiGetICost( fracBits.getFracBitsArray( ui16CtxNumSig ).intBits[uiSignificanceCoeffGroup] );
}


Void QuantRDOQ2::xInitLastPosBitsTab( const CoeffCodingContext& cctx, const UInt uiWidth, const UInt uiHeight, const ChannelType chType, const FracBitsAccess& fracBits )
{
  int dim1 = ( cctx.scanType() == SCAN_VER ? uiHeight : uiWidth );
  int dim2 = ( cctx.scanType() == SCAN_VER ? uiWidth : uiHeight );
  int bitsX = 0;
  int bitsY = 0;
  int ctxId;

  //X-coordinate
  for( ctxId = 0; ctxId < g_uiGroupIdx[dim1 - 1]; ctxId++ )
  {
    const BinFracBits fB = fracBits.getFracBitsArray( cctx.lastXCtxId( ctxId ) );
    m_lastBitsX[chType][ctxId] = bitsX + fB.intBits[0];
    bitsX += fB.intBits[1];
  }
  m_lastBitsX[chType][ctxId] = bitsX;

  //Y-coordinate
  for( ctxId = 0; ctxId < g_uiGroupIdx[dim2 - 1]; ctxId++ )
  {
    const BinFracBits fB = fracBits.getFracBitsArray( cctx.lastYCtxId( ctxId ) );
    m_lastBitsY[chType][ctxId] = bitsY + fB.intBits[0];
    bitsY += fB.intBits[1];
  }
  m_lastBitsY[chType][ctxId] = bitsY;
}


/** Calculates the cost of signaling the last significant coefficient in the block
* \param uiPosX X coordinate of the last significant coefficient
* \param uiPosY Y coordinate of the last significant coefficient
* \returns cost of last significant coefficient
*/
/*
* \param uiWidth width of the transform unit (TU)
*/
inline Int64 QuantRDOQ2::xiGetCostLast( const UInt uiPosX, const UInt uiPosY, const ChannelType chType ) const
{
  UInt uiCtxX = g_uiGroupIdx[uiPosX];
  UInt uiCtxY = g_uiGroupIdx[uiPosY];

  UInt uiCost = m_lastBitsX[chType][uiCtxX] + m_lastBitsY[chType][uiCtxY];

  if( uiCtxX > 3 )
  {
    uiCost += xGetIEPRate() * ( ( uiCtxX - 2 ) >> 1 );
  }
  if( uiCtxY > 3 )
  {
    uiCost += xGetIEPRate() * ( ( uiCtxY - 2 ) >> 1 );
  }
  return xiGetICost( (Int)uiCost );
}

/** Calculates the cost for specific absolute transform level
* \param uiAbsLevel scaled quantized level
* \param ui16CtxNumOne current ctxInc for coeff_abs_level_greater1 (1st bin of coeff_abs_level_minus1 in AVC)
* \param ui16CtxNumAbs current ctxInc for coeff_abs_level_greater2 (remaining bins of coeff_abs_level_minus1 in AVC)
* \param ui16CtxBase current global offset for coeff_abs_level_greater1 and coeff_abs_level_greater2
* \returns cost of given absolute transform level
*/
inline Int64 QuantRDOQ2::xiGetCostSigCoef( const UInt uiSignificance, const UInt ui16CtxNumSig, const FracBitsAccess& fracBits ) const
{
  return xiGetICost( fracBits.getFracBitsArray( ui16CtxNumSig ).intBits[uiSignificance] );
}



template< Bool bSBH, Bool bUseScalingList, UInt altResiCompId >
Int QuantRDOQ2::xRateDistOptQuantFast( TransformUnit &tu, const ComponentID &compID, const CCoeffBuf &pSrc, TCoeff &uiAbsSum, const QpParam &cQP, const Ctx &ctx )
{
  CoeffCodingContext cctx( tu, compID, bSBH );

  const SPS &sps                            = *tu.cs->sps;
  const CompArea &rect                      = tu.blocks[compID];
  const UInt uiWidth                        = rect.width;
  const UInt uiHeight                       = rect.height;
  const UInt uiLog2BlockWidth               = g_aucLog2[uiWidth];
  const UInt uiLog2BlockHeight              = g_aucLog2[uiHeight];
  const ChannelType chType                  = toChannelType( compID );
  const UInt log2CGSize                     = cctx.log2CGSize();
  const UInt uiMaxNumCoeff                  = rect.area();
  const UInt uiLog2TrSize                   = ( uiLog2BlockWidth + uiLog2BlockHeight ) >> 1;

  const Bool extendedPrecision                 = sps.getSpsRangeExtension().getExtendedPrecisionProcessingFlag();
  const Bool bUseGolombRiceParameterAdaptation = sps.getSpsRangeExtension().getPersistentRiceAdaptationEnabledFlag();
  const UInt initialGolombRiceParameter        = ctx.getGRAdaptStats( TU::getGolombRiceStatisticsIndex( tu, compID ) ) >> 2;

  const Int  maxLog2TrDynamicRange          = sps.getMaxLog2TrDynamicRange(toChannelType(compID));
  const PredMode ePredMode                  = tu.cu->predMode;
  const CoeffScanType eScanIdx              = CoeffScanType( TU::getCoefScanIdx( tu, compID ) );
  const FracBitsAccess& fracBits            = ctx.getFracBitsAcess();
  m_iLambda                                 = static_cast<Int64>( getLambda() * ( 1 << SCALE_BITS ) + 0.5 );


  if( compID != COMPONENT_Cr )
    xInitLastPosBitsTab( cctx, uiWidth, uiHeight, chType, fracBits );

  /* for 422 chroma blocks, the effective scaling applied during transformation is not a power of 2, hence it cannot be
  * implemented as a bit-shift (the quantised result will be sqrt(2) * larger than required). Alternatively, adjust the
  * uiLog2TrSize applied in iTransformShift, such that the result is 1/sqrt(2) the required result (i.e. smaller)
  * Then a QP+3 (sqrt(2)) or QP-3 (1/sqrt(2)) method could be used to get the required result
  */

  // Quantization parameters
  Int iTransformShift = getTransformShift( sps.getBitDepth( chType ), rect.size(), maxLog2TrDynamicRange);

  if (tu.transformSkip[compID] && extendedPrecision)
  {
    iTransformShift = std::max<Int>(0, iTransformShift);
  }
  Int iQBits                  = QUANT_SHIFT + cQP.per + iTransformShift;                   // Right shift of non-RDOQ quantizer;  level = (coeff*uiQ + offset)>>q_bits
  Int iQOffset                = 1 << ( iQBits - 1 );
  Int scalingListType         = ( ePredMode == MODE_INTRA ? 0 : 3 ) + (Int)compID; assert( scalingListType < SCALING_LIST_NUM );

  Int quantScale              = ( uiLog2BlockWidth + uiLog2BlockHeight ) & 0x01 ? ( g_quantScales[cQP.rem] * 181 ) >> 7 : g_quantScales[cQP.rem];
  const Int iErrCoef0TrOffset = ( uiLog2BlockWidth + uiLog2BlockHeight ) & 0x01 ? 1: 0;
        Int iErrScale         = m_quantErrScale.getErrScaleNoScalingList( gp_sizeIdxInfo->idxFrom( uiWidth ), gp_sizeIdxInfo->idxFrom( uiHeight ), scalingListType, cQP.rem );
  const Int *piErrScale       = m_errScale[uiLog2BlockWidth - 2][uiLog2BlockHeight - 2][scalingListType][cQP.rem];
  const Int *piQCoef          = getQuantCoeff( scalingListType, cQP.rem, uiLog2BlockWidth - 2, uiLog2BlockHeight - 2 );
  const Int iErrScaleShift    = COEFF_ERR_SCALE_PRECISION_BITS;


  const TCoeff *plSrcCoeff = pSrc.buf;
        TCoeff *piDstCoeff = tu.getCoeffs( compID ).buf;

#define CG_SIZE iCGSize
  Int64 piCostCoeff   [16];
  Int64 piCostSig     [16];
  Int64 piCostCoeff0  [16];
  Int64 piCostDeltaSBH[16];
  Int   piAddSBH      [16];

  Int c1 = 1;

  Int64 iCodedCostBlock   = 0;
  Int64 iUncodedCostBlock = 0;
  Int   iLastScanPos      = -1;
  Int   iCGLastScanPos    = -1;


#if ENABLE_TRACING
  Bool  bFirstNZSeen = false;
  DTRACE( g_trace_ctx, D_RDOQ, "%d: %3d, %3d, %dx%d, comp=%d\n", DTRACE_GET_COUNTER( g_trace_ctx, D_RDOQ ), rect.x, rect.y, rect.width, rect.height, compID );
#endif

  //////////////////////////////////////////////////////////////////////////
  //  Loop other subblocks (coefficient groups)
  //////////////////////////////////////////////////////////////////////////

  uiAbsSum = 0;

  const Int iCGSize = 1 << log2CGSize;
  Int iScanPos = uiWidth * uiHeight - 1;

  // Find first non-zero coeff
  for( ; iScanPos > 0; iScanPos-- ){
    UInt uiBlkPos = cctx.blockPos( iScanPos );
    if( plSrcCoeff[uiBlkPos] )
      break;
    piDstCoeff[uiBlkPos] = 0;
  }

  Int subSetId = iScanPos >> log2CGSize;

  for( ; subSetId >= 0; subSetId-- )
  {
    cctx.initSubblock( subSetId );
    
    Int   iNZbeforePos0  = 0;
    Int   uiAbsSumCG     = 0;
    Int64 iCodedCostCG   = 0;
    Int64 iUncodedCostCG = 0;

    UInt c1Idx           = 0;
    UInt c2Idx           = 0;
    UInt uiGoRiceParam   = initialGolombRiceParameter;
    Int  c2 = 0;
         c1 = 1;

    //Int iScanPosinCG = iCGSize - 1;
    Int iScanPosinCG = iScanPos & ( iCGSize - 1 );
    if( iLastScanPos < 0 )
    {
    findlast2:
      // Fast loop to find last-pos.
      // No need to add distortion to cost as it would be added to both the coded and uncoded cost
      for( ; iScanPosinCG >= 0; iScanPosinCG--, iScanPos-- ){
        UInt uiBlkPos = cctx.blockPos( iScanPos );
        if( bUseScalingList )
          quantScale = piQCoef[uiBlkPos];

        UInt uiMaxAbsLevel = ( abs( (Int)plSrcCoeff[uiBlkPos] ) * quantScale + iQOffset ) >> iQBits;

        if( uiMaxAbsLevel ){
          iLastScanPos   = iScanPos;
          iCGLastScanPos = subSetId;
          break;
        }
        piDstCoeff[uiBlkPos] = 0;
#if ENABLE_TRACING
        if( bFirstNZSeen )
        {
          DTRACE( g_trace_ctx, D_RDOQ, "[%d][%d][%2d:%2d][%2d:%2d]", iScanPos, cctx.blockPos( iScanPos ), cctx.cgPosX(), cctx.cgPosY(), cctx.posX( iScanPos ), cctx.posY( iScanPos ) );
          DTRACE( g_trace_ctx, D_RDOQ, " Lev=%d \n", 0 );
        }
#endif
      }
    }

    //////////////////////////////////////////////////////////////////////////
    //  Loop other coefficients
    //////////////////////////////////////////////////////////////////////////

    for( ; iScanPosinCG >= 0; iScanPosinCG--, iScanPos-- ){
      UInt uiBlkPos = cctx.blockPos( iScanPos );

      //===== quantization =====
      if( bUseScalingList )
      {
        quantScale = piQCoef[uiBlkPos];
        iErrScale  = piErrScale[uiBlkPos];
      }

      Int64 iScaledLevel = (Int64)abs( plSrcCoeff[uiBlkPos] ) * quantScale;
      Int64 iAbsLevel    = ( iScaledLevel + iQOffset ) >> iQBits;

      // Set context models
      UInt uiCtxSig, uiOneCtx, uiAbsCtx;
      if( altResiCompId )
      {
        {
          if( iScanPos == iLastScanPos )
          {
            uiOneCtx = uiAbsCtx = cctx.greater1CtxIdOfs();
            uiCtxSig = cctx.sigCtxId( iScanPos, piDstCoeff );
          }
          else
            cctx.getAltResiCtxSet( piDstCoeff, iScanPos, uiCtxSig, uiOneCtx, uiAbsCtx, uiGoRiceParam );
        }
      }
      else
      {
        uiCtxSig = cctx.sigCtxId( iScanPos );
        uiOneCtx = cctx.greater1CtxId( c1 );
        uiAbsCtx = cctx.greater2CtxId();
      }

#if ENABLE_TRACING
      DTRACE( g_trace_ctx, D_RDOQ, "[%d][%d][%2d:%2d][%2d:%2d]", iScanPos, cctx.blockPos( iScanPos ), cctx.cgPosX(), cctx.cgPosY(), cctx.posX( iScanPos ), cctx.posY( iScanPos ) );
      DTRACE_COND( ( iAbsLevel != 0 ), g_trace_ctx, D_RDOQ_MORE, " One=%d Abs=%d", uiOneCtx, uiAbsCtx );
      DTRACE_COND( ( ( iScanPos != iLastScanPos || bFirstNZSeen ) && iAbsLevel != 0 ), g_trace_ctx, D_RDOQ_MORE, " uiCtxSig=%d", uiCtxSig );
      bFirstNZSeen = true;
#endif

      // Cost for zero coeff
      piCostCoeff0[iScanPosinCG] = ( (Int64)plSrcCoeff[uiBlkPos] * plSrcCoeff[uiBlkPos] ) << ( 16 + iErrCoef0TrOffset + 2 * uiLog2TrSize );
      if( iAbsLevel == 0 )
      {
        piDstCoeff [uiBlkPos]     = 0;
        piCostSig  [iScanPosinCG] = xiGetCostSigCoef( 0, uiCtxSig, fracBits );
        piCostCoeff[iScanPosinCG] = piCostCoeff0[iScanPosinCG] + piCostSig[iScanPosinCG];

        if( bSBH ){
          Int64 iErr1        = iScaledLevel - ( (Int64)1 << iQBits );
          Int64 iSqrtErrCost = ( iErr1*iErrScale ) >> iErrScaleShift;
          Int64 iCost1       = iSqrtErrCost*iSqrtErrCost + xiGetCostSigCoef( 1, uiCtxSig, fracBits ) + xiGetICRateCost<altResiCompId>( 1, uiOneCtx, uiAbsCtx, uiGoRiceParam, c1Idx, c2Idx, extendedPrecision, maxLog2TrDynamicRange, fracBits );

          piCostDeltaSBH[iScanPosinCG] = iCost1 - piCostCoeff[iScanPosinCG];
          piAddSBH      [iScanPosinCG] = 1;
        }
        DTRACE( g_trace_ctx, D_RDOQ, " Lev=%d \n", 0 );
      }
      else
      {
        //===== coefficient level estimation =====
        UInt uiLevel;
        Int iFloor = (Int)( iScaledLevel >> iQBits );
        Int iCeil  = iFloor + 1;

        if( iScanPos == iLastScanPos )
        {
          piCostSig[iScanPosinCG] = 0;
          Int64 iCurrCostF = piCostCoeff0[iScanPosinCG];

          if( iFloor ){
            Int64 iErrF         = iScaledLevel - ( iFloor << iQBits );
            Int64 iSqrtErrCostF = ( iErrF*iErrScale ) >> iErrScaleShift;
            iCurrCostF          = iSqrtErrCostF*iSqrtErrCostF + xiGetICRateCost<altResiCompId>( iFloor, uiOneCtx, uiAbsCtx, uiGoRiceParam, c1Idx, c2Idx, extendedPrecision, maxLog2TrDynamicRange, fracBits );
          }

          Int64 iErrC         = iScaledLevel - ( iCeil << iQBits );
          Int64 iSqrtErrCostC = ( iErrC*iErrScale ) >> iErrScaleShift;
          Int64 iCurrCostC    = iSqrtErrCostC*iSqrtErrCostC + xiGetICRateCost<altResiCompId>( iCeil, uiOneCtx, uiAbsCtx, uiGoRiceParam, c1Idx, c2Idx, extendedPrecision, maxLog2TrDynamicRange, fracBits );

          if( iCurrCostC < iCurrCostF ){
            uiLevel                   = iCeil;
            piCostCoeff[iScanPosinCG] = iCurrCostC;
            if( bSBH ){
              piCostDeltaSBH[iScanPosinCG] = iCurrCostF - iCurrCostC;
              piAddSBH      [iScanPosinCG] = -1;
            }
          }
          else
          {
            if( iFloor == 0 )
            {
              DTRACE( g_trace_ctx, D_RDOQ, " Lev=%d \n", 0 );
              DTRACE( g_trace_ctx, D_RDOQ, " CostC0=%d\n", piCostCoeff0[iScanPosinCG] >> 15 );
              DTRACE( g_trace_ctx, D_RDOQ, " CostC =%d\n", iCurrCostC >> 15 );
              piCostCoeff0[iScanPosinCG] = 0;
              piDstCoeff[uiBlkPos]       = 0;
              iLastScanPos               = -1;
              iCGLastScanPos             = -1;
              iScanPos--;
              iScanPosinCG--;
              goto findlast2;
            }
            uiLevel = iFloor;
            piCostCoeff[iScanPosinCG] = iCurrCostF;

            if( bSBH ){
              piCostDeltaSBH[iScanPosinCG] = iCurrCostC - iCurrCostF;
              piAddSBH      [iScanPosinCG] = 1;
            }
          }
        }
        else {
          Int64 iCostSig1 = xiGetCostSigCoef( 1, uiCtxSig, fracBits );
          if( iCeil < 3 ){
            Int64 iCostSig0    = xiGetCostSigCoef( 0, uiCtxSig, fracBits );
            Int64 iBestCost    = piCostCoeff0[iScanPosinCG] + iCostSig0;
            Int64 iBestCostSig = iCostSig0;
            Int64 iCostF       = iBestCost;

            uiLevel = 0;
            if( iFloor ){
              Int64 iErrF         = iScaledLevel - ( iFloor << iQBits );
              Int64 iSqrtErrCostF = ( iErrF*iErrScale ) >> iErrScaleShift;
              iCostF              = iSqrtErrCostF*iSqrtErrCostF + iCostSig1 + xiGetICRateCost<altResiCompId>( iFloor, uiOneCtx, uiAbsCtx, uiGoRiceParam, c1Idx, c2Idx, extendedPrecision, maxLog2TrDynamicRange, fracBits );

              if( iCostF < iBestCost ){
                uiLevel      = iFloor;
                iBestCost    = iCostF;
                iBestCostSig = iCostSig1;
                if( bSBH ){
                  piCostDeltaSBH[iScanPosinCG] = iBestCost - iCostF;
                  piAddSBH      [iScanPosinCG] = -1;
                }
              }
              else {
                if( bSBH ){
                  piCostDeltaSBH[iScanPosinCG] = iCostF - iBestCost;
                  piAddSBH      [iScanPosinCG] = 1;
                }
              }
            }

            Int64 iErrC         = iScaledLevel - ( iCeil << iQBits );
            Int64 iSqrtErrCostC = ( iErrC*iErrScale ) >> iErrScaleShift;
            Int64 iCostC        = iSqrtErrCostC*iSqrtErrCostC + iCostSig1 + xiGetICRateCost<altResiCompId>( iCeil, uiOneCtx, uiAbsCtx, uiGoRiceParam, c1Idx, c2Idx, extendedPrecision, maxLog2TrDynamicRange, fracBits );

            if( iCostC < iBestCost ){
              uiLevel                   = iCeil;
              piCostCoeff[iScanPosinCG] = iCostC;
              piCostSig[iScanPosinCG]   = iCostSig1;
              if( bSBH ){
                piCostDeltaSBH[iScanPosinCG] = iCostF - iCostC;
                piAddSBH[iScanPosinCG]       = -1;
              }
            }
            else {
              piCostCoeff[iScanPosinCG] = iBestCost;
              piCostSig[iScanPosinCG] = iBestCostSig;
              if( bSBH ){
                piCostDeltaSBH[iScanPosinCG] = iCostC - iCostF;
                piAddSBH      [iScanPosinCG] = 1;
              }
            }
          }
          else
          {
            Int64 iErrF             = iScaledLevel - ( iFloor << iQBits );
            Int64 iSqrtErrCostF     = ( iErrF*iErrScale ) >> iErrScaleShift;
            Int64 iCostF            = iSqrtErrCostF*iSqrtErrCostF + iCostSig1 + xiGetICRateCost<altResiCompId>( iFloor, uiOneCtx, uiAbsCtx, uiGoRiceParam, c1Idx, c2Idx, extendedPrecision, maxLog2TrDynamicRange, fracBits );

            Int64 iErrC             = iScaledLevel - ( iCeil << iQBits );
            Int64 iSqrtErrCostC     = ( iErrC*iErrScale ) >> iErrScaleShift;
            Int64 iCostC            = iSqrtErrCostC*iSqrtErrCostC + iCostSig1 + xiGetICRateCost<altResiCompId>( iCeil, uiOneCtx, uiAbsCtx, uiGoRiceParam, c1Idx, c2Idx, extendedPrecision, maxLog2TrDynamicRange, fracBits );

            piCostSig[iScanPosinCG] = iCostSig1;
            if( iCostC < iCostF ){
              uiLevel = iCeil;
              piCostCoeff[iScanPosinCG] = iCostC;
              if( bSBH ){
                piCostDeltaSBH[iScanPosinCG] = iCostF - iCostC;
                piAddSBH[iScanPosinCG]       = -1;
              }
            }
            else {
              uiLevel = iFloor;
              piCostCoeff[iScanPosinCG] = iCostF;

              if( bSBH ){
                piCostDeltaSBH[iScanPosinCG] = iCostC - iCostF;
                piAddSBH[iScanPosinCG] = 1;
              }
            }
          }
        }
        piDstCoeff[uiBlkPos] = uiLevel;

        DTRACE( g_trace_ctx, D_RDOQ, " Lev=%d \n", uiLevel );
        DTRACE( g_trace_ctx, D_RDOQ, " CostC0=%d\n", piCostCoeff0[iScanPosinCG] >> 15 );
        DTRACE( g_trace_ctx, D_RDOQ, " CostC =%d\n", piCostCoeff[iScanPosinCG] >> 15 );

        if( uiLevel )
        {
          uiAbsSumCG += uiLevel;
          if( altResiCompId )
          {
            c1Idx++;
            if( uiLevel > 1 )
            {
              c2Idx++;
            }
          }
          else
          {
            UInt baseLevel = (UInt)( c1Idx < C1FLAG_NUMBER ) ? ( 2 + ( c2Idx < C2FLAG_NUMBER ) ) : 1;
            if( uiLevel >= baseLevel ){
              if( uiLevel > ( UInt )3 * ( 1 << uiGoRiceParam ) ) {
                uiGoRiceParam = bUseGolombRiceParameterAdaptation ? ( uiGoRiceParam + 1 ): nextGoRice[uiGoRiceParam]; //min<UInt>(uiGoRiceParam+ 1, 4);
              }
            }

            c1Idx++;
            //===== update bin model =====
            if( uiLevel > 1 ){
              c1 = 0;
              c2 += ( c2 < 2 );
              c2Idx++;
            }
            else{
              c1 = nextc1[c1];
            }
          }
          cctx.setSigGroup();

          // hack-> just add instead of checking iScanPosinCG >0 and increment
          iNZbeforePos0 += iScanPosinCG;
        }
      }

      iUncodedCostCG += piCostCoeff0[iScanPosinCG];
      iCodedCostCG += piCostCoeff[iScanPosinCG];
    } // for (iScanPosinCG)

    cctx.setGt2Flag( c1 == 0 );

    //===== estimate cost last position =====
    if( iCGLastScanPos == subSetId )
    {
      Int64 i64BestCost    = std::numeric_limits<Int64>::max();
      Int iBestLastIdxP1   = 0;
      Int64 iBestCostLast  = std::numeric_limits<Int64>::max();
      for( Int iScanPosTmp = iLastScanPos; iScanPosTmp >= iCGLastScanPos*iCGSize; iScanPosTmp-- ) {
        UInt iScanPosinCGTmp = iScanPosTmp%CG_SIZE;
        UInt uiBlkPos = cctx.blockPos( iScanPosTmp );

        if( piDstCoeff[uiBlkPos] ) 
        {
          UInt uiPosY = uiBlkPos >> uiLog2BlockWidth;
          UInt uiPosX = uiBlkPos - ( uiPosY << uiLog2BlockWidth );

          const UInt  pos_major = eScanIdx == SCAN_VER ? uiPosY : uiPosX;
          const UInt  pos_minor = eScanIdx == SCAN_VER ? uiPosX : uiPosY;
          const Int64 iCostLast = xiGetCostLast( pos_major, pos_minor, chType );
          const Int64 totalCost = iCodedCostCG + iCostLast - piCostSig[iScanPosinCGTmp];

          if( totalCost < i64BestCost ){
            iBestLastIdxP1 = iScanPosTmp + 1;
            i64BestCost    = totalCost;
            iBestCostLast  = iCostLast;
          }
          if( piDstCoeff[uiBlkPos] > 1 ){
            break;
          }
        }
        iCodedCostCG -= piCostCoeff[iScanPosinCGTmp];
        iCodedCostCG += piCostCoeff0[iScanPosinCGTmp];
      } //end for

      iCodedCostCG = i64BestCost;
      for( Int iScanPosTmp = iBestLastIdxP1; iScanPosTmp <= iLastScanPos; iScanPosTmp++ ){
        uiAbsSumCG -= piDstCoeff[cctx.blockPos( iScanPosTmp )];
        piDstCoeff[cctx.blockPos( iScanPosTmp )] = 0;
      }
      iLastScanPos = iBestLastIdxP1 - 1;

      if( bSBH )
      {
        // take into account reduction of sig bins and last-rate changes
        if( ( piDstCoeff[cctx.blockPos( iLastScanPos )] == 1 ) && ( piAddSBH[iLastScanPos%CG_SIZE] == -1 ) ){
          // arbitrary offset taken over from HM SBH (used to be (4<<15), likely 4 ep sig bins)
          //piCostDeltaSBH[iLastScanPos%CG_SIZE] -= 4*m_iLambda*xGetIEPRate();

          Int iScanPosTmp = iLastScanPos - 1;
          for( ; iScanPosTmp >= iCGLastScanPos*iCGSize; iScanPosTmp-- ) {
            piCostDeltaSBH[iLastScanPos%CG_SIZE] -= piCostSig[iScanPosTmp%CG_SIZE];
            if( piDstCoeff[cctx.blockPos( iScanPosTmp )] != 0 ){
              break;
            }
          }
          if( iScanPosTmp >= 0 && iScanPosTmp < iLastScanPos - 1 ){
            UInt iLastScanPosTmp = iLastScanPos%CG_SIZE;
            UInt uiBlkPos        = cctx.blockPos( iScanPosTmp );
            UInt uiPosY          = uiBlkPos >> uiLog2BlockWidth;
            UInt uiPosX          = uiBlkPos - ( uiPosY << uiLog2BlockWidth );

            piCostDeltaSBH[iLastScanPosTmp] -= iBestCostLast;

            const UInt pos_major = eScanIdx == SCAN_VER ? uiPosY : uiPosX;
            const UInt pos_minor = eScanIdx == SCAN_VER ? uiPosX : uiPosY;
            piCostDeltaSBH[iLastScanPosTmp] += xiGetCostLast( pos_major, pos_minor, chType );

          }
        }
      }
    }

    if( bSBH && uiAbsSumCG >= 2 )
    {
      if( cctx.isSigGroup()/* uiSigCoeffGroupFlag[uiCGBlkPos] == 1*/ ){
        Int iSubPos         = subSetId*iCGSize;
        Int iLastNZPosInCG  = -1;
        Int iFirstNZPosInCG = CG_SIZE;

        for( Int n = 0; n <CG_SIZE; n++ ) {
          if( piDstCoeff[ cctx.blockPos( n + iSubPos ) ] ) {
            iFirstNZPosInCG = n;
            break;
          }
        }
        if( iCGLastScanPos == subSetId ){
          iLastNZPosInCG = iLastScanPos%CG_SIZE;
        }
        else{
          for( Int n = CG_SIZE - 1; n >= 0; n-- ) {
            if( piDstCoeff[ cctx.blockPos( n + iSubPos ) ] ) {
              iLastNZPosInCG = n;
              break;
            }
          }
        }
        if( iLastNZPosInCG - iFirstNZPosInCG >= SBH_THRESHOLD )
        {
          iCodedCostCG -= xiGetICost( xGetIEPRate() ); //subtract cost for one sign bin
          Bool bSign    = plSrcCoeff[ cctx.blockPos( iSubPos + iFirstNZPosInCG) ] < 0;

          if( bSign != ( uiAbsSumCG & 0x1 ) ) {
            Int iLastPosInCG    = ( iCGLastScanPos == subSetId ) ? iLastNZPosInCG : CG_SIZE - 1;
            Int64 iMinCostDelta = std::numeric_limits<Int64>::max();
            Int iMinCostPos     = -1;

            if( piDstCoeff[ cctx.blockPos( iFirstNZPosInCG + iSubPos ) ] >1 ){
              iMinCostDelta = piCostDeltaSBH[iFirstNZPosInCG];
              iMinCostPos   = iFirstNZPosInCG;
            }

            for( Int n = 0; n<iFirstNZPosInCG; n++ ){
              if( ( plSrcCoeff[ cctx.blockPos( iSubPos + n ) ] < 0 ) == bSign ){
                if( piCostDeltaSBH[n] < iMinCostDelta ){
                  iMinCostDelta = piCostDeltaSBH[n];
                  iMinCostPos   = n;
                }
              }
            }

            for( Int n = iFirstNZPosInCG + 1; n <= iLastPosInCG; n++ ){
              if( piCostDeltaSBH[n] < iMinCostDelta ){
                iMinCostDelta = piCostDeltaSBH[n];
                iMinCostPos   = n;
              }
            }
            piDstCoeff[ cctx.blockPos( iMinCostPos + iSubPos ) ] += piAddSBH[iMinCostPos];
            uiAbsSumCG   += piAddSBH[iMinCostPos];
            iCodedCostCG += iMinCostDelta;
          }
        }
      }
    }

    DTRACE( g_trace_ctx, D_RDOQ_COST, "UncodedCG=%d\n", iUncodedCostCG >> 15 );
    DTRACE( g_trace_ctx, D_RDOQ_COST, "CodedCG  =%d\n", iCodedCostCG >> 15 );

    // Group sig. flag
    if( iCGLastScanPos >= 0 )
    {
      UInt uiCtxSig = cctx.sigGroupCtxId();
      Int64 iCostCoeffGroupSig0 = xiGetCostSigCoeffGroup( 0, uiCtxSig, fracBits );

      // if no coeff in CG
      if( !cctx.isSigGroup() )
      {
        iCodedCostCG = iUncodedCostCG + iCostCoeffGroupSig0;
      }
      else
      {
        // if not topleft CG
        if( subSetId )
        {
          Int64 iCostCoeffGroupSig1 = xiGetCostSigCoeffGroup( 1, uiCtxSig, fracBits );
          // if only one coeff in CG
          if( !iNZbeforePos0 ){
            iCodedCostCG -= piCostSig[0];
          }
          if( iCGLastScanPos != subSetId ){
            iCodedCostCG += iCostCoeffGroupSig1;
            iUncodedCostCG += iCostCoeffGroupSig0;
          }

          // if we can save cost, change this block to all-zero block
          if( iUncodedCostCG < iCodedCostCG )
          {
            cctx.resetSigGroup();

            iCodedCostCG = iUncodedCostCG;
            // reset coeffs to 0 in this block
            for( iScanPosinCG = iCGSize - 1; iScanPosinCG >= 0; iScanPosinCG-- ){
              Int iScanPosTmp = subSetId*iCGSize + iScanPosinCG;
              UInt uiBlkPos = cctx.blockPos( iScanPosTmp );
              piDstCoeff[uiBlkPos] = 0;
            }
            uiAbsSumCG = 0;
            if( iCGLastScanPos == subSetId ){
              iCodedCostCG   = 0;
              iUncodedCostCG = 0;
              iLastScanPos   = -1;
              iCGLastScanPos = -1;
            }
          }
        }
      }
    }

    iCodedCostBlock += iCodedCostCG;
    iUncodedCostBlock += iUncodedCostCG;
    uiAbsSum += uiAbsSumCG;
    DTRACE( g_trace_ctx, D_RDOQ_COST, "%d: [%2d:%2d]\n", DTRACE_GET_COUNTER( g_trace_ctx, D_RDOQ_COST ), cctx.cgPosX(), cctx.cgPosY() );
    DTRACE( g_trace_ctx, D_RDOQ_COST, "Uncoded=%d\n", iUncodedCostBlock >> 15 );
    DTRACE( g_trace_ctx, D_RDOQ_COST, "Coded  =%d\n", iCodedCostBlock >> 15 );
  } //end for (iCGScanPos)

  if( iLastScanPos < 0 )
    return 0;


  if( !CU::isIntra( *tu.cu ) && isLuma( compID ) && tu.depth == 0 )
  {
    const BinFracBits fracBitsQtRootCbf = fracBits.getFracBitsArray( Ctx::QtRootCbf() );
    iUncodedCostBlock += xiGetICost( fracBitsQtRootCbf.intBits[0] );
    iCodedCostBlock   += xiGetICost( fracBitsQtRootCbf.intBits[1] );
  }
  else
  {
    const BinFracBits fracBitsQtCbf = fracBits.getFracBitsArray( Ctx::QtCbf[chType]( DeriveCtx::CtxQtCbf( rect.compID, tu.depth ) ) );
    iUncodedCostBlock += xiGetICost( fracBitsQtCbf.intBits[0] );
    iCodedCostBlock   += xiGetICost( fracBitsQtCbf.intBits[1] );
  }

  DTRACE( g_trace_ctx, D_RDOQ_COST, "%d: %3d, %3d, %dx%d, comp=%d\n", DTRACE_GET_COUNTER( g_trace_ctx, D_RDOQ ), rect.x, rect.y, rect.width, rect.height, compID );
  DTRACE( g_trace_ctx, D_RDOQ_COST, "Uncoded=%d\n", iUncodedCostBlock >> 15 );
  DTRACE( g_trace_ctx, D_RDOQ_COST, "Coded  =%d\n", iCodedCostBlock >> 15 );

  if( iCodedCostBlock > iUncodedCostBlock )
  {
    uiAbsSum = 0;
    ::memset( piDstCoeff, 0, uiMaxNumCoeff*sizeof( TCoeff ) );
  }
  else
  {
    for( Int scanPos = 0; scanPos <= iLastScanPos; scanPos++ ){
      Int blkPos = cctx.blockPos( scanPos );
      Int level = piDstCoeff[blkPos];
      Int iSign = plSrcCoeff[blkPos] >> 31;
      piDstCoeff[blkPos] = ( iSign^level ) - iSign;
    }
  }

  return 0;
}

Int QuantRDOQ2::xRateDistOptQuant( TransformUnit &tu, const ComponentID &compID, const CCoeffBuf &pSrc, TCoeff &uiAbsSum, const QpParam &cQP, const Ctx &ctx, bool bUseScalingList )
{
  if( m_altResiCompId == 0 )
  {
    if( m_bSBH )
    {
      if( bUseScalingList ) return xRateDistOptQuantFast<true, true,  0>( tu, compID, pSrc, uiAbsSum, cQP, ctx );
      else                  return xRateDistOptQuantFast<true, false, 0>( tu, compID, pSrc, uiAbsSum, cQP, ctx );
    }
    else
    {
      if( bUseScalingList ) return xRateDistOptQuantFast<false, true,  0>( tu, compID, pSrc, uiAbsSum, cQP, ctx );
      else                  return xRateDistOptQuantFast<false, false, 0>( tu, compID, pSrc, uiAbsSum, cQP, ctx );
    }
  }
  else if( m_altResiCompId == 1 )
  {
    if( m_bSBH )
    {
      if( bUseScalingList ) return xRateDistOptQuantFast<true, true,  1>( tu, compID, pSrc, uiAbsSum, cQP, ctx );
      else                  return xRateDistOptQuantFast<true, false, 1>( tu, compID, pSrc, uiAbsSum, cQP, ctx );
    }
    else
    {
      if( bUseScalingList ) return xRateDistOptQuantFast<false, true,  1>( tu, compID, pSrc, uiAbsSum, cQP, ctx );
      else                  return xRateDistOptQuantFast<false, false, 1>( tu, compID, pSrc, uiAbsSum, cQP, ctx );
    }
  }
  else
  {
    THROW( "Wrong AltResiCompId mode!" );
  }

}
