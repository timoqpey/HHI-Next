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

/** \file     TrQuant.cpp
    \brief    transform and quantization class
*/

#include "TrQuant.h"
#include "TrQuant_EMT.h"

#include "UnitTools.h"
#include "ContextModelling.h"
#include "CodingStructure.h"
#include "CrossCompPrediction.h"

#include "dtrace_buffer.h"

#include <stdlib.h>
#include <limits>
#include <memory.h>


struct coeffGroupRDStats
{
  Int    iNNZbeforePos0;
  Double d64CodedLevelandDist; // distortion and level cost only
  Double d64UncodedDist;    // all zero coded block distortion
  Double d64SigCost;
  Double d64SigCost_0;
};

FwdTrans *fastFwdTrans[5][7] =
{
  { fastForwardDCT2_B2, fastForwardDCT2_B4, fastForwardDCT2_B8, fastForwardDCT2_B16, fastForwardDCT2_B32, fastForwardDCT2_B64, fastForwardDCT2_B128 },
  { NULL,               fastForwardDCT5_B4, fastForwardDCT5_B8, fastForwardDCT5_B16, fastForwardDCT5_B32, fastForwardDCT5_B64, fastForwardDCT5_B128 },
  { NULL,               fastForwardDCT8_B4, fastForwardDCT8_B8, fastForwardDCT8_B16, fastForwardDCT8_B32, fastForwardDCT8_B64, fastForwardDCT8_B128 },
  { NULL,               fastForwardDST1_B4, fastForwardDST1_B8, fastForwardDST1_B16, fastForwardDST1_B32, fastForwardDST1_B64, fastForwardDST1_B128 },
  { NULL,               fastForwardDST7_B4, fastForwardDST7_B8, fastForwardDST7_B16, fastForwardDST7_B32, fastForwardDST7_B64, fastForwardDST7_B128 },
};

InvTrans *fastInvTrans[5][7] =
{
  { fastInverseDCT2_B2, fastInverseDCT2_B4, fastInverseDCT2_B8, fastInverseDCT2_B16, fastInverseDCT2_B32, fastInverseDCT2_B64, fastInverseDCT2_B128 },
  { NULL,               fastInverseDCT5_B4, fastInverseDCT5_B8, fastInverseDCT5_B16, fastInverseDCT5_B32, fastInverseDCT5_B64, fastInverseDCT5_B128 },
  { NULL,               fastInverseDCT8_B4, fastInverseDCT8_B8, fastInverseDCT8_B16, fastInverseDCT8_B32, fastInverseDCT8_B64, fastInverseDCT8_B128 },
  { NULL,               fastInverseDST1_B4, fastInverseDST1_B8, fastInverseDST1_B16, fastInverseDST1_B32, fastInverseDST1_B64, fastInverseDST1_B128 },
  { NULL,               fastInverseDST7_B4, fastInverseDST7_B8, fastInverseDST7_B16, fastInverseDST7_B32, fastInverseDST7_B64, fastInverseDST7_B128 },
};

//! \ingroup CommonLib
//! \{

// ====================================================================================================================
// Constants
// ====================================================================================================================

#define RDOQ_CHROMA                 1           ///< use of RDOQ in chroma

// ====================================================================================================================
// QpParam constructor
// ====================================================================================================================

QpParam::QpParam(const Int           qpy,
                 const ChannelType   chType,
                 const Int           qpBdOffset,
                 const Int           chromaQPOffset,
                 const ChromaFormat  chFmt,
                 const int           dqp )
{
  Int baseQp;

  if(isLuma(chType))
  {
    baseQp = qpy + qpBdOffset;
  }
  else
  {
    baseQp = Clip3( -qpBdOffset, (chromaQPMappingTableSize - 1), qpy + chromaQPOffset );

    if(baseQp < 0)
    {
      baseQp = baseQp + qpBdOffset;
    }
    else
    {
      baseQp = getScaledChromaQP(baseQp, chFmt) + qpBdOffset;
    }
  }

  baseQp = Clip3( 0, MAX_QP+qpBdOffset, baseQp + dqp );

  Qp =baseQp;
  per=baseQp/6;
  rem=baseQp%6;
}

QpParam::QpParam(const TransformUnit& tu, const ComponentID &compIDX)
{
  Int chromaQpOffset = 0;
  ComponentID compID = MAP_CHROMA(compIDX);

  if (isChroma(compID))
  {
    chromaQpOffset += tu.cs->pps->getQpOffset( compID );
    chromaQpOffset += tu.cs->slice->getSliceChromaQpDelta( compID );
    chromaQpOffset += tu.cs->pps->getPpsRangeExtension().getChromaQpOffsetListEntry( tu.cu->chromaQpAdj ).u.offset[Int( compID ) - 1];
  }

  int dqp = 0;

  *this = QpParam(tu.cu->qp, toChannelType(compID), tu.cs->sps->getQpBDOffset(toChannelType(compID)), chromaQpOffset, tu.chromaFormat, dqp);
}




static bool needsSqrt2Scale( const Size& size )
{
  return ( ( ( g_aucLog2[size.width] + g_aucLog2[size.height] ) & 1 ) == 1 );
}

static bool needsBlockSizeQuantScale( const Size& size )
{
  return needsSqrt2Scale( size );
}

// ====================================================================================================================
// TrQuant class member functions
// ====================================================================================================================
void xTrMxN ( const int bitDepth, const Pel *residual, size_t stride, TCoeff *coeff, size_t width, size_t height, bool useDST, const int maxLog2TrDynamicRange );
void xITrMxN( const int bitDepth, const TCoeff *coeff, Pel *residual, size_t stride, size_t width, size_t height, bool useDST, const int maxLog2TrDynamicRange );


TrQuant::TrQuant() : m_fTr(xTrMxN), m_fITr(xITrMxN)
{
  // allocate temporary buffers
  m_plTempCoeff = (TCoeff*) xMalloc( TCoeff, MAX_CU_SIZE * MAX_CU_SIZE );

  initScalingList();

}

TrQuant::~TrQuant()
{
  // delete temporary buffers
  if ( m_plTempCoeff )
  {
    xFree( m_plTempCoeff );
    m_plTempCoeff = nullptr;
  }
  destroyScalingList();
}


void xTrMxN_EMT( const Int bitDepth, const Pel *residual, size_t stride, TCoeff *coeff, Int iWidth, Int iHeight, Bool useDST, const Int maxLog2TrDynamicRange, UChar ucMode, UChar ucTrIdx, bool use65intraModes, bool useQTBT )
{
  const Int TRANSFORM_MATRIX_SHIFT = g_transformMatrixShift[TRANSFORM_FORWARD];
  const Int shift_1st              = ( ( g_aucLog2[iWidth ] - 2 + MIN_CU_LOG2 ) + bitDepth + TRANSFORM_MATRIX_SHIFT ) - maxLog2TrDynamicRange + COM16_C806_TRANS_PREC;
  const Int shift_2nd              = (   g_aucLog2[iHeight] - 2 + MIN_CU_LOG2 )            + TRANSFORM_MATRIX_SHIFT                           + COM16_C806_TRANS_PREC;
  const UInt nLog2WidthMinus1  = g_aucLog2[iWidth]  - 2 + MIN_CU_LOG2 - 1;  //nLog2WidthMinus1, since transform start from 2-point
  const UInt nLog2HeightMinus1 = g_aucLog2[iHeight] - 2 + MIN_CU_LOG2 - 1;  //nLog2HeightMinus1, since transform start from 2-point

  Int iSkipWidth = 0, iSkipHeight = 0;

  if( useQTBT )
  {
    iSkipWidth  = ( iWidth  > JVET_C0024_ZERO_OUT_TH ? iWidth  - JVET_C0024_ZERO_OUT_TH : 0 );
    iSkipHeight = ( iHeight > JVET_C0024_ZERO_OUT_TH ? iHeight - JVET_C0024_ZERO_OUT_TH : 0 );
  }
  else if( ( ( ucMode == INTER_MODE_IDX || iWidth == 64 ) && ucTrIdx != DCT2_EMT && iWidth >= JVET_C0024_ZERO_OUT_TH ) || ( ucTrIdx == DCT2_EMT && iWidth == 64 ) )
  {
    iSkipWidth  = iWidth  >> 1;
    iSkipHeight = iHeight >> 1;
  }

  CHECK( shift_1st < 0, "Negative shift" );
  CHECK( shift_2nd < 0, "Negative shift" );

  ALIGN_DATA( MEMORY_ALIGN_DEF_SIZE, TCoeff block[MAX_TU_SIZE * MAX_TU_SIZE] );

  for( Int y = 0; y < iHeight; y++ )
  {
    for( Int x = 0; x < iWidth; x++ )
    {
      block[( y * iWidth ) + x] = residual[( y * stride ) + x];
    }
  }

  TCoeff *tmp = ( TCoeff * ) alloca( iWidth * iHeight * sizeof( TCoeff ) );

  UInt  nTrIdxHor = DCT2, nTrIdxVer = DCT2;
  if( ucMode != INTER_MODE_IDX && ucTrIdx != DCT2_EMT )
  {
    UInt  nTrSubsetHor, nTrSubsetVer;
    if( use65intraModes )
    {
      nTrSubsetHor = g_aucTrSetHorz[ucMode];
      nTrSubsetVer = g_aucTrSetVert[ucMode];
    }
    else //we use only 35 intra modes
    {
      nTrSubsetHor = g_aucTrSetHorz35[ucMode];
      nTrSubsetVer = g_aucTrSetVert35[ucMode];
    }
    nTrIdxHor = g_aiTrSubsetIntra[nTrSubsetHor][ucTrIdx  & 1];
    nTrIdxVer = g_aiTrSubsetIntra[nTrSubsetVer][ucTrIdx >> 1];
  }
  if( ucMode == INTER_MODE_IDX && ucTrIdx != DCT2_EMT )
  {
    nTrIdxHor = g_aiTrSubsetInter[ucTrIdx  & 1];
    nTrIdxVer = g_aiTrSubsetInter[ucTrIdx >> 1];
  }

  fastFwdTrans[nTrIdxHor][nLog2WidthMinus1]( block, tmp,  shift_1st, iHeight,          0, iSkipWidth,  1 );
  fastFwdTrans[nTrIdxVer][nLog2HeightMinus1]( tmp, coeff, shift_2nd, iWidth,  iSkipWidth, iSkipHeight, 1 );
}

/** MxN inverse transform (2D)
*  \param bitDepth              [in]  bit depth
*  \param coeff                 [in]  transform coefficients
*  \param residual              [out] residual block
*  \param stride                [in]  stride of the residual block
*  \param iWidth                [in]  width of transform
*  \param iHeight               [in]  height of transform
*  \param uiSkipWidth           [in]
*  \param uiSkipHeight          [in]
*  \param useDST                [in]
*  \param maxLog2TrDynamicRange [in]
*  \param ucMode                [in]
*  \param ucTrIdx               [in]
*  \param use65intraModes       [in]
*/

void xITrMxN_EMT(const Int bitDepth, const TCoeff *coeff, Pel *residual, size_t stride, Int iWidth, Int iHeight, UInt uiSkipWidth, UInt uiSkipHeight, Bool useDST, const Int maxLog2TrDynamicRange, UChar ucMode, UChar ucTrIdx, bool use65intraModes)
{
  const Int TRANSFORM_MATRIX_SHIFT = g_transformMatrixShift[TRANSFORM_INVERSE];
  const Int shift_1st              =   TRANSFORM_MATRIX_SHIFT + 1 + COM16_C806_TRANS_PREC; //1 has been added to shift_1st at the expense of shift_2nd
  const Int shift_2nd              = ( TRANSFORM_MATRIX_SHIFT + maxLog2TrDynamicRange - 1 ) - bitDepth + COM16_C806_TRANS_PREC;
  const TCoeff clipMinimum         = -( 1 << maxLog2TrDynamicRange );
  const TCoeff clipMaximum         =  ( 1 << maxLog2TrDynamicRange ) - 1;

  const UInt nLog2WidthMinus1  = g_aucLog2[iWidth]  - 2 + MIN_CU_LOG2 - 1;  //nLog2WidthMinus1, since transform start from 2-point
  const UInt nLog2HeightMinus1 = g_aucLog2[iHeight] - 2 + MIN_CU_LOG2 - 1;  //nLog2HeightMinus1, since transform start from 2-point

  CHECK( shift_1st < 0, "Negative shift" );
  CHECK( shift_2nd < 0, "Negative shift" );

  TCoeff *tmp   = ( TCoeff * ) alloca( iWidth * iHeight * sizeof( TCoeff ) );
  TCoeff *block = ( TCoeff * ) alloca( iWidth * iHeight * sizeof( TCoeff ) );

  UInt  nTrIdxHor = DCT2, nTrIdxVer = DCT2;
  if( ucMode != INTER_MODE_IDX && ucTrIdx != DCT2_EMT )
  {
    UInt  nTrSubsetHor, nTrSubsetVer;
    if( use65intraModes )
    {
      nTrSubsetHor = g_aucTrSetHorz[ucMode];
      nTrSubsetVer = g_aucTrSetVert[ucMode];
    }
    else //we use only 35 intra modes
    {
      nTrSubsetHor = g_aucTrSetHorz35[ucMode];
      nTrSubsetVer = g_aucTrSetVert35[ucMode];
    }
    nTrIdxHor = g_aiTrSubsetIntra[nTrSubsetHor][ucTrIdx  & 1];
    nTrIdxVer = g_aiTrSubsetIntra[nTrSubsetVer][ucTrIdx >> 1];
  }
  if( ucMode == INTER_MODE_IDX && ucTrIdx != DCT2_EMT )
  {
    nTrIdxHor = g_aiTrSubsetInter[ucTrIdx  & 1];
    nTrIdxVer = g_aiTrSubsetInter[ucTrIdx >> 1];
  }

  fastInvTrans[nTrIdxVer][nLog2HeightMinus1]   ( coeff, tmp, shift_1st, iWidth, uiSkipWidth, uiSkipHeight, 1, clipMinimum, clipMaximum );
  fastInvTrans[nTrIdxHor][nLog2WidthMinus1]    ( tmp, block, shift_2nd, iHeight,          0,  uiSkipWidth, 1, clipMinimum, clipMaximum );

  for( Int y = 0; y < iHeight; y++ )
  {
    for( Int x = 0; x < iWidth; x++ )
    {
      residual[( y * stride ) + x] = Pel( block[( y * iWidth ) + x] );
    }
  }
}

/** MxN forward transform (2D)
*  \param bitDepth              [in]  bit depth
*  \param residual              [in]  residual block
*  \param stride                [in]  stride of residual block
*  \param coeff                 [out] transform coefficients
*  \param width                 [in]  width of transform
*  \param height                [in]  height of transform
*  \param useDST                [in]
*  \param maxLog2TrDynamicRange [in]

*/
void xTrMxN( const int bitDepth, const Pel *residual, size_t stride, TCoeff *coeff, size_t width, size_t height, bool useDST, const int maxLog2TrDynamicRange )
{
  const int iWidth  = (int)width;
  const int iHeight = (int)height;

  const Int TRANSFORM_MATRIX_SHIFT = g_transformMatrixShift[TRANSFORM_FORWARD];

  const Int shift_1st = (g_aucLog2[iWidth] +  bitDepth + TRANSFORM_MATRIX_SHIFT) - maxLog2TrDynamicRange;
  const Int shift_2nd = g_aucLog2[iHeight] + TRANSFORM_MATRIX_SHIFT;

  UInt iSkipWidth  = ( iWidth  > JVET_C0024_ZERO_OUT_TH ? iWidth  - JVET_C0024_ZERO_OUT_TH : 0 );
  UInt iSkipHeight = ( iHeight > JVET_C0024_ZERO_OUT_TH ? iHeight - JVET_C0024_ZERO_OUT_TH : 0 );

  CHECK( shift_1st < 0, "Negative shift" );
  CHECK( shift_2nd < 0, "Negative shift" );

  ALIGN_DATA( MEMORY_ALIGN_DEF_SIZE, TCoeff block[MAX_TU_SIZE * MAX_TU_SIZE] );
  ALIGN_DATA( MEMORY_ALIGN_DEF_SIZE, TCoeff   tmp[MAX_TU_SIZE * MAX_TU_SIZE] );

  for( Int y = 0; y < iHeight; y++ )
  {
    for( Int x = 0; x < iWidth; x++ )
    {
      block[( y * iWidth ) + x] = residual[( y * stride ) + x];
    }
  }

  {
    switch (iWidth)
    {
    case 2:     fastForwardDCT2_B2(block, tmp, shift_1st, iHeight, 0, iSkipWidth, 0);  break;
    case 4:
      {
        if ((iHeight == 4) && useDST)    // Check for DCT or DST
        {
          fastForwardDST7_B4(block, tmp, shift_1st, iHeight, 0, iSkipWidth, 0);
        }
        else
        {
          fastForwardDCT2_B4(block, tmp, shift_1st, iHeight, 0, iSkipWidth, 0);
        }
      }
      break;

    case 8:     fastForwardDCT2_B8  (block, tmp, shift_1st, iHeight, 0, iSkipWidth, 0);  break;
    case 16:    fastForwardDCT2_B16 (block, tmp, shift_1st, iHeight, 0, iSkipWidth, 0);  break;
    case 32:    fastForwardDCT2_B32 (block, tmp, shift_1st, iHeight, 0, iSkipWidth, 0);  break;
    case 64:    fastForwardDCT2_B64 (block, tmp, shift_1st+ COM16_C806_TRANS_PREC, iHeight, 0, iSkipWidth, 0);  break;
    case 128:   fastForwardDCT2_B128(block, tmp, shift_1st+ COM16_C806_TRANS_PREC, iHeight, 0, iSkipWidth, 0);  break;
    default:
      THROW( "Unsupported transformation size" ); break;
    }
  }

  {
    switch (iHeight)
    {
    case 2:     fastForwardDCT2_B2(tmp, coeff, shift_2nd, iWidth, iSkipWidth, iSkipHeight, 0);  break;
    case 4:
      {
        if ((iWidth == 4) && useDST)    // Check for DCT or DST
        {
          fastForwardDST7_B4(tmp, coeff, shift_2nd, iWidth, iSkipWidth, iSkipHeight, 0);
        }
        else
        {
          fastForwardDCT2_B4(tmp, coeff, shift_2nd, iWidth, iSkipWidth, iSkipHeight, 0);
        }
      }
      break;

    case 8:     fastForwardDCT2_B8  (tmp, coeff, shift_2nd, iWidth, iSkipWidth, iSkipHeight, 0);  break;
    case 16:    fastForwardDCT2_B16 (tmp, coeff, shift_2nd, iWidth, iSkipWidth, iSkipHeight, 0);  break;
    case 32:    fastForwardDCT2_B32 (tmp, coeff, shift_2nd, iWidth, iSkipWidth, iSkipHeight, 0);  break;
    case 64:    fastForwardDCT2_B64 (tmp, coeff, shift_2nd+ COM16_C806_TRANS_PREC, iWidth, iSkipWidth, iSkipHeight, 0);  break;
    case 128:   fastForwardDCT2_B128(tmp, coeff, shift_2nd+ COM16_C806_TRANS_PREC, iWidth, iSkipWidth, iSkipHeight, 0);  break;
    default:
      THROW( "Unsupported transformation size" ); break;
    }
  }
}


/** MxN inverse transform (2D)
*  \param bitDepth              [in]  bit depth
*  \param coeff                 [in]  transform coefficients
*  \param residual              [out] residual block
*  \param stride                [out] stride of the residual block
*  \param width                 [in]  width of transform
*  \param height                [in]  height of transform
*  \param useDST                [in]
*  \param maxLog2TrDynamicRange [in]
*/
void xITrMxN( const int bitDepth, const TCoeff *coeff, Pel *residual, size_t stride, size_t width, size_t height, bool useDST, const int maxLog2TrDynamicRange )
{
  const Int TRANSFORM_MATRIX_SHIFT = g_transformMatrixShift[TRANSFORM_INVERSE];
  const int iWidth  = (int)width;
  const int iHeight = (int)height;


  Int shift_1st = TRANSFORM_MATRIX_SHIFT + 1; //1 has been added to shift_1st at the expense of shift_2nd
  Int shift_2nd = (TRANSFORM_MATRIX_SHIFT + maxLog2TrDynamicRange - 1) - bitDepth;
  const TCoeff clipMinimum = -(1 << maxLog2TrDynamicRange);
  const TCoeff clipMaximum =  (1 << maxLog2TrDynamicRange) - 1;

  UInt uiSkipWidth  = ( iWidth  > JVET_C0024_ZERO_OUT_TH ? iWidth  - JVET_C0024_ZERO_OUT_TH : 0 );
  UInt uiSkipHeight = ( iHeight > JVET_C0024_ZERO_OUT_TH ? iHeight - JVET_C0024_ZERO_OUT_TH : 0 );

  CHECK( shift_1st < 0, "Negative shift" );
  CHECK( shift_2nd < 0, "Negative shift" );

  ALIGN_DATA( MEMORY_ALIGN_DEF_SIZE, TCoeff block[MAX_TU_SIZE * MAX_TU_SIZE] );
  ALIGN_DATA( MEMORY_ALIGN_DEF_SIZE, TCoeff   tmp[MAX_TU_SIZE * MAX_TU_SIZE] );

  {
    switch (iHeight)
    {
    case 2: fastInverseDCT2_B2(coeff, tmp, shift_1st, iWidth, uiSkipWidth, uiSkipHeight, 0, clipMinimum, clipMaximum); break;
    case 4:
      {
        if ((iWidth == 4) && useDST)    // Check for DCT or DST
        {
          fastInverseDST7_B4(coeff, tmp, shift_1st, iWidth, uiSkipWidth, uiSkipHeight, 0, clipMinimum, clipMaximum);
        }
        else
        {
          fastInverseDCT2_B4(coeff, tmp, shift_1st, iWidth, uiSkipWidth, uiSkipHeight, 0, clipMinimum, clipMaximum);
        }
      }
      break;

    case   8: fastInverseDCT2_B8  (coeff, tmp, shift_1st, iWidth, uiSkipWidth, uiSkipHeight, 0, clipMinimum, clipMaximum); break;
    case  16: fastInverseDCT2_B16 (coeff, tmp, shift_1st, iWidth, uiSkipWidth, uiSkipHeight, 0, clipMinimum, clipMaximum); break;
    case  32: fastInverseDCT2_B32 (coeff, tmp, shift_1st, iWidth, uiSkipWidth, uiSkipHeight, 0, clipMinimum, clipMaximum); break;
    case  64: fastInverseDCT2_B64 (coeff, tmp, shift_1st + COM16_C806_TRANS_PREC, iWidth, uiSkipWidth, uiSkipHeight, 0, clipMinimum, clipMaximum); break;
    case 128: fastInverseDCT2_B128(coeff, tmp, shift_1st + COM16_C806_TRANS_PREC, iWidth, uiSkipWidth, uiSkipHeight, 0, clipMinimum, clipMaximum); break;
    default:
      THROW( "Unsupported transformation size" ); break;
    }
  }

  {
    switch (iWidth)
    {
    case 2: fastInverseDCT2_B2(tmp, block, shift_2nd, iHeight,  0, uiSkipWidth, 0, std::numeric_limits<Pel>::min(), std::numeric_limits<Pel>::max()); break;
    // Clipping here is not in the standard, but is used to protect the "Pel" data type into which the inverse-transformed samples will be copied
    case 4:
      {
        if ((iHeight == 4) && useDST)    // Check for DCT or DST
        {
          fastInverseDST7_B4(tmp, block, shift_2nd, iHeight, 0, uiSkipWidth, 0, std::numeric_limits<Pel>::min(), std::numeric_limits<Pel>::max());
        }
        else
        {
          fastInverseDCT2_B4(tmp, block, shift_2nd, iHeight, 0, uiSkipWidth, 0, std::numeric_limits<Pel>::min(), std::numeric_limits<Pel>::max());
        }
      }
      break;

    case   8: fastInverseDCT2_B8  (tmp, block, shift_2nd, iHeight, 0, uiSkipWidth, 0, std::numeric_limits<Pel>::min(), std::numeric_limits<Pel>::max()); break;
    case  16: fastInverseDCT2_B16 (tmp, block, shift_2nd, iHeight, 0, uiSkipWidth, 0, std::numeric_limits<Pel>::min(), std::numeric_limits<Pel>::max()); break;
    case  32: fastInverseDCT2_B32 (tmp, block, shift_2nd, iHeight, 0, uiSkipWidth, 0, std::numeric_limits<Pel>::min(), std::numeric_limits<Pel>::max()); break;
    case  64: fastInverseDCT2_B64 (tmp, block, shift_2nd + COM16_C806_TRANS_PREC, iHeight, 0, uiSkipWidth, 0, std::numeric_limits<Pel>::min(), std::numeric_limits<Pel>::max()); break;
    case 128: fastInverseDCT2_B128(tmp, block, shift_2nd + COM16_C806_TRANS_PREC, iHeight, 0, uiSkipWidth, 0, std::numeric_limits<Pel>::min(), std::numeric_limits<Pel>::max()); break;
    default:
      THROW( "Unsupported transformation size" );
      break;
    }
  }

  for (Int y = 0; y < iHeight; y++)
  {
    for (Int x = 0; x < iWidth; x++)
    {
      residual[(y * stride)+x] = Pel(block[(y * width) + x]);
    }
  }
}

#if MATRIX_MULT
/** NxN forward transform (2D) using brute force matrix multiplication (3 nested loops)
 *  \param bitDepth              [in]  bit depth
 *  \param residual              [in]  residual block
 *  \param stride                [in]  stride of the residual block
 *  \param coeff                 [out]  transform coefficients
 *  \param width                 [in]  width of transform
 *  \param height                [in]  height of transform
 */
void xTr( const int bitDepth, const Pel *residual, size_t stride, TCoeff *coeff, size_t width, size_t height, bool useDST, const int maxLog2TrDynamicRange )
{
  const int iWidth  = ( int ) width;
  const int iHeight = ( int ) height;

  const int TRANSFORM_MATRIX_SHIFT = g_transformMatrixShift[TRANSFORM_FORWARD];

  const int shift_1st = ( g_aucLog2[iWidth] + bitDepth + TRANSFORM_MATRIX_SHIFT ) - maxLog2TrDynamicRange;
  const int shift_2nd =   g_aucLog2[iHeight] + TRANSFORM_MATRIX_SHIFT;

  const int add_1st = ( shift_1st > 0 ) ? ( 1 << ( shift_1st - 1 ) ) : 0;
  const int add_2nd = 1 << ( shift_2nd - 1 );

  CHECK( shift_1st < 0, "Negative shift" );
  CHECK( shift_2nd < 0, "Negative shift" );

  TCoeff tmp[MAX_TU_SIZE * MAX_TU_SIZE];

  unsigned i, j, k;
  TCoeff iSum;
  const TMatrixCoeff *iT;

  /* Horizontal transform */
  if( iWidth == 2 )
  {
    iT = g_aiT2[TRANSFORM_FORWARD][0];
  }
  else if( iWidth == 4 )
  {
    iT = ( useDST && iHeight == 4 ? g_as_DST_MAT_4[TRANSFORM_FORWARD][0] : g_aiT4[TRANSFORM_FORWARD][0] );
  }
  else if( iWidth == 8 )
  {
    iT = g_aiT8[TRANSFORM_FORWARD][0];
  }
  else if( iWidth == 16 )
  {
    iT = g_aiT16[TRANSFORM_FORWARD][0];
  }
  else if( iWidth == 32 )
  {
    iT = g_aiT32[TRANSFORM_FORWARD][0];
  }
  else if( iWidth == 64 )
  {
    iT = g_aiT64[TRANSFORM_FORWARD][0];
  }
  else if( iWidth == 128 )
  {
    iT = g_aiT128[TRANSFORM_FORWARD][0];
  }
  else
  {
    THROW( "Unsupported matrix size" );
  }

  for( i = 0; i < iWidth; i++ )
  {
    for( j = 0; j < iHeight; j++ )
    {
      iSum = 0;
      for( k = 0; k < iWidth; k++ )
      {
        iSum += iT[i*iWidth + k] * residual[j*stride + k];
      }
      tmp[i*iHeight + j] = ( iSum + add_1st ) >> shift_1st;
    }
  }

  /* Vertical transform */
  if( iHeight == 2 )
  {
    iT = g_aiT2[TRANSFORM_FORWARD][0];
  }
  else if( iHeight == 4 )
  {
    iT = ( useDST && iWidth == 4 ? g_as_DST_MAT_4[TRANSFORM_FORWARD][0] : g_aiT4[TRANSFORM_FORWARD][0] );
  }
  else if( iHeight == 8 )
  {
    iT = g_aiT8[TRANSFORM_FORWARD][0];
  }
  else if( iHeight == 16 )
  {
    iT = g_aiT16[TRANSFORM_FORWARD][0];
  }
  else if( iHeight == 32 )
  {
    iT = g_aiT32[TRANSFORM_FORWARD][0];
  }
  else if( iHeight == 64 )
  {
    iT = g_aiT64[TRANSFORM_FORWARD][0];
  }
  else if( iHeight == 128 )
  {
    iT = g_aiT128[TRANSFORM_FORWARD][0];
  }
  else
  {
    THROW( "Unsupported matrix size" );
  }

  for( i = 0; i < iHeight; i++ )
  {
    for( j = 0; j < iWidth; j++ )
    {
      iSum = 0;
      for( k = 0; k < iHeight; k++ )
      {
        iSum += iT[i*iHeight + k] * tmp[j*iHeight + k];
      }
      coeff[i*iWidth + j] = ( iSum + add_2nd ) >> shift_2nd;
    }
  }
}

/** NxN inverse transform (2D) using brute force matrix multiplication (3 nested loops)
 *  \param bitDepth              [in]  bit depth
 *  \param coeff                 [in]  transform coefficients
 *  \param residual              [out]  residual block
 *  \param stride                [in]  stride of the residual block
 *  \param width                 [in]  width of transform
 *  \param height                [in]  height of transform
 */
void xITr( const int bitDepth, const TCoeff *coeff, Pel *residual, size_t stride, size_t width, size_t height, bool useDST, const int maxLog2TrDynamicRange )
{
  const Int TRANSFORM_MATRIX_SHIFT = g_transformMatrixShift[TRANSFORM_INVERSE];
  const int iWidth  = (int)width;
  const int iHeight = (int)height;

  const int shift_1st = TRANSFORM_MATRIX_SHIFT + 1; //1 has been added to shift_1st at the expense of shift_2nd
  const int shift_2nd = (TRANSFORM_MATRIX_SHIFT + maxLog2TrDynamicRange - 1) - bitDepth;

  const TCoeff clipMinimum = -(1 << maxLog2TrDynamicRange);
  const TCoeff clipMaximum =  (1 << maxLog2TrDynamicRange) - 1;

  const int add_1st = 1 << ( shift_1st - 1 );
  const int add_2nd = ( shift_2nd > 0 ) ? ( 1 << ( shift_2nd - 1 ) ) : 0;

  CHECK( shift_1st < 0, "Negative shift" );
  CHECK( shift_2nd < 0, "Negative shift" );

  TCoeff tmp[MAX_TU_SIZE * MAX_TU_SIZE];

  unsigned i, j, k;
  TCoeff iSum = 0;
  const TMatrixCoeff *iT = nullptr;

  /* Horizontal transform */
  if( iHeight == 2 )
  {
    iT = g_aiT2[TRANSFORM_INVERSE][0];
  }
  else if( iHeight == 4 )
  {
    iT = ( iWidth == 4 && useDST ? g_as_DST_MAT_4[TRANSFORM_INVERSE][0] : g_aiT4[TRANSFORM_INVERSE][0] );
  }
  else if( iHeight == 8 )
  {
    iT = g_aiT8[TRANSFORM_INVERSE][0];
  }
  else if( iHeight == 16 )
  {
    iT = g_aiT16[TRANSFORM_INVERSE][0];
  }
  else if( iHeight == 32 )
  {
    iT = g_aiT32[TRANSFORM_INVERSE][0];
  }
  else if( iHeight == 64 )
  {
    iT = g_aiT64[TRANSFORM_INVERSE][0];
  }
  else if( iHeight == 128 )
  {
    iT = g_aiT64[TRANSFORM_INVERSE][0];
  }
  else
  {
    THROW( "Unsupported matrix size" );
  }

  for( i = 0; i < iHeight; i++ )
  {
    for( j = 0; j < iWidth; j++ )
    {
      iSum = 0;
      for( k = 0; k < iHeight; k++ )
      {
        iSum += iT[k*iHeight + i] * coeff[k*iWidth + j];
      }

      // Clipping here is not in the standard, but is used to protect the "Pel" data type into which the inverse-transformed samples will be copied

      // TODO: Is it a bug? Its look like for a stride should be used iHeight and not iWidth
      tmp[i*iWidth + j] = Clip3<TCoeff>( clipMinimum, clipMaximum, ( iSum + add_1st ) >> shift_1st );
    }
  }

  /* Vertical transform */
  if( iWidth == 2 )
  {
    iT = g_aiT2[TRANSFORM_INVERSE][0];
  }
  else if( iWidth == 4 )
  {
    iT = ( iHeight == 4 && useDST ? g_as_DST_MAT_4[TRANSFORM_INVERSE][0] : g_aiT4[TRANSFORM_INVERSE][0] );
  }
  else if( iWidth == 8 )
  {
    iT = g_aiT8[TRANSFORM_INVERSE][0];
  }
  else if( iWidth == 16 )
  {
    iT = g_aiT16[TRANSFORM_INVERSE][0];
  }
  else if( iWidth == 32 )
  {
    iT = g_aiT32[TRANSFORM_INVERSE][0];
  }
  else if( iWidth == 64 )
  {
    iT = g_aiT64[TRANSFORM_INVERSE][0];
  }
  else if( iWidth == 128 )
  {
    iT = g_aiT64[TRANSFORM_INVERSE][0];
  }
  else
  {
    THROW( "Unsupported matrix size" );
  }

  for( i = 0; i < iHeight; i++ )
  {
    for( j = 0; j < iWidth; j++ )
    {
      iSum = 0;
      for( k = 0; k < iWidth; k++ )
      {
        iSum += iT[k*iWidth + j] * tmp[i*iWidth + k];
      }

      residual[i*stride + j] = Clip3<TCoeff>( std::numeric_limits<Pel>::min(), std::numeric_limits<Pel>::max(), ( iSum + add_2nd ) >> shift_2nd );
    }
  }
}

Void xTr_EMT(Int bitDepth, Pel *block, TCoeff *coeff, UInt uiStride, UInt uiTrSize, Bool useDST, const Int maxLog2TrDynamicRange, UChar ucMode, UChar ucTrIdx)
{
  UInt i, j, k;
  TCoeff iSum;
  TCoeff tmp[MAX_TU_SIZE * MAX_TU_SIZE];
  const TMatrixCoeff *iTH, *iTV;
  UInt uiLog2TrSize = g_aucConvertToBit[uiTrSize] + 2;

  Int zo = ((ucMode == INTER_MODE_IDX && uiTrSize >= 32 && ucTrIdx != DCT2_EMT) || uiTrSize == 64) ? 1 : 0;

  UInt  nTrIdxHor = DCT2, nTrIdxVer = DCT2;
  if (ucMode != INTER_MODE_IDX && ucTrIdx != DCT2_EMT)
  {
    UInt  nTrSubsetHor = g_aucTrSetHorz[ucMode];
    UInt  nTrSubsetVer = g_aucTrSetVert[ucMode];
    nTrIdxHor = g_aiTrSubsetIntra[nTrSubsetHor][ucTrIdx & 1];
    nTrIdxVer = g_aiTrSubsetIntra[nTrSubsetVer][ucTrIdx >> 1];
  }
  if (ucMode == INTER_MODE_IDX && ucTrIdx != DCT2_EMT)
  {
    nTrIdxHor = g_aiTrSubsetInter[ucTrIdx & 1];
    nTrIdxVer = g_aiTrSubsetInter[ucTrIdx >> 1];
  }

  if (uiTrSize == 4)
  {
    iTH = g_aiTr4[nTrIdxHor][0];
    iTV = g_aiTr4[nTrIdxVer][0];
  }
  else if (uiTrSize == 8)
  {
    iTH = g_aiTr8[nTrIdxHor][0];
    iTV = g_aiTr8[nTrIdxVer][0];
  }
  else if (uiTrSize == 16)
  {
    iTH = g_aiTr16[nTrIdxHor][0];
    iTV = g_aiTr16[nTrIdxVer][0];
  }
  else if (uiTrSize == 32)
  {
    iTH = g_aiTr32[nTrIdxHor][0];
    iTV = g_aiTr32[nTrIdxVer][0];
  }
  else if (uiTrSize == 64)
  {
    assert(ucTrIdx == DCT2_EMT);
    iTH = iTV = g_aiTr64[DCT2][0];
  }
  else
  {
    assert(0);
  }

  const Int TRANSFORM_MATRIX_SHIFT = g_transformMatrixShift[TRANSFORM_FORWARD];

  Int shift_1st = (uiLog2TrSize + bitDepth + TRANSFORM_MATRIX_SHIFT) - maxLog2TrDynamicRange + COM16_C806_TRANS_PREC;
  Int shift_2nd = uiLog2TrSize + TRANSFORM_MATRIX_SHIFT + COM16_C806_TRANS_PREC;
  const Int add_1st = (shift_1st>0) ? (1 << (shift_1st - 1)) : 0;
  const Int add_2nd = 1 << (shift_2nd - 1);

  /* Horizontal transform */
  for (i = 0; i<(uiTrSize >> zo); i++)
  {
    for (j = 0; j<uiTrSize; j++)
    {
      iSum = 0;
      for (k = 0; k<uiTrSize; k++)
      {
        iSum += iTH[i*uiTrSize + k] * block[j*uiStride + k];
      }
      tmp[i*uiTrSize + j] = (iSum + add_1st) >> shift_1st;
    }
  }

  /* Vertical transform */
  for (i = 0; i<(uiTrSize >> zo); i++)
  {
    for (j = 0; j<(uiTrSize >> zo); j++)
    {
      iSum = 0;
      for (k = 0; k<uiTrSize; k++)
      {
        iSum += iTV[i*uiTrSize + k] * tmp[j*uiTrSize + k];
      }
      coeff[i*uiTrSize + j] = (iSum + add_2nd) >> shift_2nd;
    }
  }
  if (zo)
  {
    memset(coeff + uiTrSize*uiTrSize / 2, 0, sizeof(TCoeff)*uiTrSize*uiTrSize / 2);
    coeff += uiTrSize / 2;
    for (j = 0; j<uiTrSize / 2; j++)
    {
      memset(coeff, 0, sizeof(TCoeff)*uiTrSize / 2);
      coeff += uiTrSize;
    }
  }
}

Void xITr_EMT(Int bitDepth, TCoeff *coeff, Pel *block, UInt uiStride, UInt uiTrSize, Bool useDST, const Int maxLog2TrDynamicRange, UChar ucMode, UChar ucTrIdx)
{
  UInt i, j, k;
  TCoeff iSum;
  TCoeff tmp[MAX_TU_SIZE * MAX_TU_SIZE];
  const TMatrixCoeff *iTH, *iTV;

  Int zo = ((ucMode == INTER_MODE_IDX && uiTrSize >= 32 && ucTrIdx != DCT2_EMT) || uiTrSize == 64) ? 1 : 0;

  UInt  nTrIdxHor = DCT2, nTrIdxVer = DCT2;
  if (ucMode != INTER_MODE_IDX && ucTrIdx != DCT2_EMT)
  {
    UInt  nTrSubsetHor = g_aucTrSetHorz[ucMode];
    UInt  nTrSubsetVer = g_aucTrSetVert[ucMode];
    nTrIdxHor = g_aiTrSubsetIntra[nTrSubsetHor][ucTrIdx & 1];
    nTrIdxVer = g_aiTrSubsetIntra[nTrSubsetVer][ucTrIdx >> 1];
  }
  if (ucMode == INTER_MODE_IDX && ucTrIdx != DCT2_EMT)
  {
    nTrIdxHor = g_aiTrSubsetInter[ucTrIdx & 1];
    nTrIdxVer = g_aiTrSubsetInter[ucTrIdx >> 1];
  }

  if (uiTrSize == 4)
  {
    iTH = g_aiTr4[nTrIdxHor][0];
    iTV = g_aiTr4[nTrIdxVer][0];
  }
  else if (uiTrSize == 8)
  {
    iTH = g_aiTr8[nTrIdxHor][0];
    iTV = g_aiTr8[nTrIdxVer][0];
  }
  else if (uiTrSize == 16)
  {
    iTH = g_aiTr16[nTrIdxHor][0];
    iTV = g_aiTr16[nTrIdxVer][0];
  }
  else if (uiTrSize == 32)
  {
    iTH = g_aiTr32[nTrIdxHor][0];
    iTV = g_aiTr32[nTrIdxVer][0];
  }
  else if (uiTrSize == 64)
  {
    assert(ucTrIdx == DCT2_EMT);
    iTH = iTV = g_aiTr64[DCT2][0];
  }
  else
  {
    assert(0);
  }

  const Int TRANSFORM_MATRIX_SHIFT = g_transformMatrixShift[TRANSFORM_INVERSE];


  Int shift_1st = TRANSFORM_MATRIX_SHIFT + 1 + COM16_C806_TRANS_PREC; //1 has been added to shift_1st at the expense of shift_2nd
  Int shift_2nd = (TRANSFORM_MATRIX_SHIFT + maxLog2TrDynamicRange - 1) - bitDepth + COM16_C806_TRANS_PREC;
  const TCoeff clipMinimum = -(1 << maxLog2TrDynamicRange);
  const TCoeff clipMaximum = (1 << maxLog2TrDynamicRange) - 1;
  assert(shift_2nd >= 0);
  const Int add_1st = 1 << (shift_1st - 1);
  const Int add_2nd = (shift_2nd>0) ? (1 << (shift_2nd - 1)) : 0;

  /* Horizontal transform */
  for (i = 0; i<uiTrSize; i++)
  {
    for (j = 0; j<uiTrSize >> zo; j++)
    {
      iSum = 0;
      for (k = 0; k<uiTrSize >> zo; k++)
      {
        iSum += iTV[k*uiTrSize + i] * coeff[k*uiTrSize + j];
      }

      // Clipping here is not in the standard, but is used to protect the "Pel" data type into which the inverse-transformed samples will be copied
      tmp[i*uiTrSize + j] = Clip3<TCoeff>(clipMinimum, clipMaximum, (iSum + add_1st) >> shift_1st);
    }
  }

  /* Vertical transform */
  for (i = 0; i<uiTrSize; i++)
  {
    for (j = 0; j<uiTrSize; j++)
    {
      iSum = 0;
      for (k = 0; k<uiTrSize >> zo; k++)
      {
        iSum += iTH[k*uiTrSize + j] * tmp[i*uiTrSize + k];
      }

      block[i*uiStride + j] = Clip3<TCoeff>(std::numeric_limits<Pel>::min(), std::numeric_limits<Pel>::max(), (iSum + add_2nd) >> shift_2nd);
    }
  }
}
#endif //MATRIX_MULT

// To minimize the distortion only. No rate is considered.
Void TrQuant::signBitHidingHDQ( TCoeff* pQCoef, const TCoeff* pCoef, TCoeff* deltaU, const CoeffCodingContext& cctx, const Int maxLog2TrDynamicRange )
{
  const UInt width     = cctx.width();
  const UInt height    = cctx.height();
  const UInt groupSize = 1 << cctx.log2CGSize();

  const TCoeff entropyCodingMinimum = -(1 << maxLog2TrDynamicRange);
  const TCoeff entropyCodingMaximum =  (1 << maxLog2TrDynamicRange) - 1;

  Int lastCG = -1;
  Int absSum = 0 ;
  Int n ;

  for( Int subSet = (width*height-1) >> cctx.log2CGSize(); subSet >= 0; subSet-- )
  {
    Int  subPos = subSet << cctx.log2CGSize();
    Int  firstNZPosInCG=groupSize , lastNZPosInCG=-1 ;
    absSum = 0 ;

    for(n = groupSize-1; n >= 0; --n )
    {
      if( pQCoef[ cctx.blockPos( n + subPos ) ] )
      {
        lastNZPosInCG = n;
        break;
      }
    }

    for(n = 0; n <groupSize; n++ )
    {
      if( pQCoef[ cctx.blockPos( n + subPos ) ] )
      {
        firstNZPosInCG = n;
        break;
      }
    }

    for(n = firstNZPosInCG; n <=lastNZPosInCG; n++ )
    {
      absSum += Int(pQCoef[ cctx.blockPos( n + subPos ) ]);
    }

    if(lastNZPosInCG>=0 && lastCG==-1)
    {
      lastCG = 1 ;
    }

    if( lastNZPosInCG-firstNZPosInCG>=SBH_THRESHOLD )
    {
      UInt signbit = (pQCoef[cctx.blockPos(subPos+firstNZPosInCG)]>0?0:1) ;
      if( signbit!=(absSum&0x1) )  //compare signbit with sum_parity
      {
        TCoeff curCost    = std::numeric_limits<TCoeff>::max();
        TCoeff minCostInc = std::numeric_limits<TCoeff>::max();
        Int minPos =-1, finalChange=0, curChange=0;

        for( n = (lastCG==1?lastNZPosInCG:groupSize-1) ; n >= 0; --n )
        {
          UInt blkPos   = cctx.blockPos( n+subPos );
          if(pQCoef[ blkPos ] != 0 )
          {
            if(deltaU[blkPos]>0)
            {
              curCost = - deltaU[blkPos];
              curChange=1 ;
            }
            else
            {
              //curChange =-1;
              if(n==firstNZPosInCG && abs(pQCoef[blkPos])==1)
              {
                curCost = std::numeric_limits<TCoeff>::max();
              }
              else
              {
                curCost = deltaU[blkPos];
                curChange =-1;
              }
            }
          }
          else
          {
            if(n<firstNZPosInCG)
            {
              UInt thisSignBit = (pCoef[blkPos]>=0?0:1);
              if(thisSignBit != signbit )
              {
                curCost = std::numeric_limits<TCoeff>::max();
              }
              else
              {
                curCost = - (deltaU[blkPos])  ;
                curChange = 1 ;
              }
            }
            else
            {
              curCost = - (deltaU[blkPos])  ;
              curChange = 1 ;
            }
          }

          if( curCost<minCostInc)
          {
            minCostInc = curCost ;
            finalChange = curChange ;
            minPos = blkPos ;
          }
        } //CG loop

        if(pQCoef[minPos] == entropyCodingMaximum || pQCoef[minPos] == entropyCodingMinimum)
        {
          finalChange = -1;
        }

        if(pCoef[minPos]>=0)
        {
          pQCoef[minPos] += finalChange ;
        }
        else
        {
          pQCoef[minPos] -= finalChange ;
        }
      } // Hide
    }
    if(lastCG==1)
    {
      lastCG=0 ;
    }
  } // TU loop

  return;
}

Void TrQuant::xDeQuant(const TransformUnit &tu,
                             CoeffBuf      &dstCoeff,
                       const ComponentID   &compID,
                       const QpParam       &cQP)
{
  const SPS            *sps                = tu.cs->sps;
  const CompArea       &area               = tu.blocks[compID];
  const UInt            uiWidth            = area.width;
  const UInt            uiHeight           = area.height;
  const TCoeff   *const piQCoef            = tu.getCoeffs(compID).buf;
        TCoeff   *const piCoef             = dstCoeff.buf;
  const UInt            numSamplesInBlock  = uiWidth * uiHeight;
  const Int             maxLog2TrDynamicRange = sps->getMaxLog2TrDynamicRange(toChannelType(compID));
  const TCoeff          transformMinimum   = -(1 << maxLog2TrDynamicRange);
  const TCoeff          transformMaximum   =  (1 << maxLog2TrDynamicRange) - 1;
  const Bool            enableScalingLists = getUseScalingList(uiWidth, uiHeight, (tu.transformSkip[compID] != 0));
  const Int             scalingListType    = getScalingListType(tu.cu->predMode, compID);
  const Int             channelBitDepth    = sps->getBitDepth(toChannelType(compID));

  CHECK(scalingListType >= SCALING_LIST_NUM, "Invalid scaling list");
  CHECK(uiWidth > m_uiMaxTrSize, "Unsupported transformation size");

  // Represents scaling through forward transform
  const Bool bClipTransformShiftTo0 = (tu.transformSkip[compID] != 0) && sps->getSpsRangeExtension().getExtendedPrecisionProcessingFlag();
  const Int  originalTransformShift = getTransformShift(channelBitDepth, area.size(), maxLog2TrDynamicRange);
  const Int  iTransformShift        = bClipTransformShiftTo0 ? std::max<Int>(0, originalTransformShift) : originalTransformShift;

  const Int QP_per = cQP.per;
  const Int QP_rem = cQP.rem;

  const Bool needsScalingCorrection = needsBlockSizeQuantScale( tu.block( compID ) );
  const Bool needsSqrt2 = TU::needsSqrt2Scale( tu.blocks[compID] );
  const Int  NEScale    = ( needsSqrt2 ? 181 : 1 );
  const Int  rightShift = (needsScalingCorrection ?   8 : 0 ) + (IQUANT_SHIFT - (iTransformShift + QP_per)) + (enableScalingLists ? LOG2_SCALING_LIST_NEUTRAL_VALUE : 0);

  if(enableScalingLists)
  {
    //from the dequantization equation:
    //iCoeffQ                         = ((Intermediate_Int(clipQCoef) * piDequantCoef[deQuantIdx]) + iAdd ) >> rightShift
    //(sizeof(Intermediate_Int) * 8)  =              inputBitDepth    +    dequantCoefBits                   - rightShift
    const UInt             dequantCoefBits     = 1 + IQUANT_SHIFT + SCALING_LIST_BITS;
    const UInt             targetInputBitDepth = std::min<UInt>((maxLog2TrDynamicRange + 1), (((sizeof(Intermediate_Int) * 8) + rightShift) - dequantCoefBits));

    const Intermediate_Int inputMinimum        = -(1 << (targetInputBitDepth - 1));
    const Intermediate_Int inputMaximum        =  (1 << (targetInputBitDepth - 1)) - 1;

    const UInt uiLog2TrWidth  = g_aucLog2[uiWidth];
    const UInt uiLog2TrHeight = g_aucLog2[uiHeight];
    Int *piDequantCoef = getDequantCoeff(scalingListType, QP_rem, uiLog2TrWidth - 1, uiLog2TrHeight - 1);

    if(rightShift > 0)
    {
      const Intermediate_Int iAdd = 1 << (rightShift - 1);

      for( Int n = 0; n < numSamplesInBlock; n++ )
      {
        const TCoeff           clipQCoef = TCoeff(Clip3<Intermediate_Int>(inputMinimum, inputMaximum, piQCoef[n]));
        const Intermediate_Int iCoeffQ   = ((Intermediate_Int(clipQCoef) * piDequantCoef[n] * NEScale) + iAdd ) >> rightShift;

        piCoef[n] = TCoeff(Clip3<Intermediate_Int>(transformMinimum,transformMaximum,iCoeffQ));
      }
    }
    else
    {
      const Int leftShift = -rightShift;

      for( Int n = 0; n < numSamplesInBlock; n++ )
      {
        const TCoeff           clipQCoef = TCoeff(Clip3<Intermediate_Int>(inputMinimum, inputMaximum, piQCoef[n]));
        const Intermediate_Int iCoeffQ   = (Intermediate_Int(clipQCoef) * piDequantCoef[n] * NEScale) << leftShift;

        piCoef[n] = TCoeff(Clip3<Intermediate_Int>(transformMinimum,transformMaximum,iCoeffQ));
      }
    }
  }
  else
  {
    const Int scale     = g_invQuantScales[QP_rem] * NEScale;
    const Int scaleBits = ( IQUANT_SHIFT + 1 );

    //from the dequantisation equation:
    //iCoeffQ                         = Intermediate_Int((Int64(clipQCoef) * scale + iAdd) >> rightShift);
    //(sizeof(Intermediate_Int) * 8)  =                    inputBitDepth   + scaleBits      - rightShift
    const UInt             targetInputBitDepth = std::min<UInt>((maxLog2TrDynamicRange + 1), (((sizeof(Intermediate_Int) * 8) + rightShift) - scaleBits));
    const Intermediate_Int inputMinimum        = -(1 << (targetInputBitDepth - 1));
    const Intermediate_Int inputMaximum        =  (1 << (targetInputBitDepth - 1)) - 1;

    if (rightShift > 0)
    {
      const Intermediate_Int iAdd = 1 << (rightShift - 1);

      for( Int n = 0; n < numSamplesInBlock; n++ )
      {
        const TCoeff           clipQCoef = TCoeff(Clip3<Intermediate_Int>(inputMinimum, inputMaximum, piQCoef[n]));
        const Intermediate_Int iCoeffQ   = (Intermediate_Int(clipQCoef) * scale + iAdd) >> rightShift;

        piCoef[n] = TCoeff(Clip3<Intermediate_Int>(transformMinimum,transformMaximum,iCoeffQ));
      }
    }
    else
    {
      const Int leftShift = -rightShift;

      for( Int n = 0; n < numSamplesInBlock; n++ )
      {
        const TCoeff           clipQCoef = TCoeff(Clip3<Intermediate_Int>(inputMinimum, inputMaximum, piQCoef[n]));
        const Intermediate_Int iCoeffQ   = (Intermediate_Int(clipQCoef) * scale) << leftShift;

        piCoef[n] = TCoeff(Clip3<Intermediate_Int>(transformMinimum,transformMaximum,iCoeffQ));
      }
    }
  }
}

Void TrQuant::init(       UInt uiMaxTrSize,
                          Bool bUseRDOQ,
                          Bool bUseRDOQTS,
#if T0196_SELECTIVE_RDOQ
                          Bool useSelectiveRDOQ,
#endif
                          Bool bEnc,
                          Bool useTransformSkipFast,
                          Bool use65IntraModes,
                          Bool rectTUs
                       )
{
  m_uiMaxTrSize  = uiMaxTrSize;
  m_bEnc         = bEnc;
  m_useRDOQ      = bUseRDOQ;
  m_useRDOQTS    = bUseRDOQTS;
#if T0196_SELECTIVE_RDOQ
  m_useSelectiveRDOQ     = useSelectiveRDOQ;
#endif
  m_useTransformSkipFast = useTransformSkipFast;
  m_use65IntraModes      = use65IntraModes;
  m_rectTUs              = rectTUs;

}

Void TrQuant::FwdNsstNxN( Int* src, const UInt uiMode, const UInt uiIndex, const UInt uiSize )
{
  const int   rnd = uiSize >> 1;
  const int   shl = 5;
  const int * par = (uiSize > 4) ? g_nsstHyGTPar8x8[uiMode][uiIndex] : /*((uiSize > 2) ?*/ g_nsstHyGTPar4x4[uiMode][uiIndex] /*: g_nsstHyGTPar2x2[uiMode][uiIndex])*/;
  const int   cof = 1 << (shl + 9);
  const int  kMax = (Int)(uiSize * uiSize);
  const int  kLog = g_aucLog2[kMax];
  const int  iMax = kMax >> 1;

  CHECK( uiIndex >= 4, "Invalid NSST index" );
  CHECK( /*uiSize != 2 &&*/ uiSize != 4 && uiSize != 8, "Invalid NSST size" );

  for (int k = 0; k < kMax; k++) src[k] <<= shl;

  for (int r = 0, q = (kLog * rnd - 1); r < rnd; r++)
  {
    for (int d = 0; d < kLog; d++, q--)
    {
      const int   s = 1 << d;
      const int * p = par + ((r * kLog + d) * iMax);
      if (q > 0)
      {
        for (int i = 0; i < iMax; i++)
        {
          const tabSinCos   &t = g_tabSinCos[*p++];
          const register int j = i + (i & -s);
          const register int a = src[j];
          const register int b = src[j + s];
          src[j]     = (t.c * a - t.s * b + 512) >> 10;
          src[j + s] = (t.c * b + t.s * a + 512) >> 10;
        }
      }
      else
      {
        for (int i = 0; i < iMax; i++)
        {
          const tabSinCos   &t = g_tabSinCos[*p++];
          const register int j = i + (i & -s);
          const register int a = src[j];
          const register int b = src[j + s];
          src[j]     = (t.c * a - t.s * b + cof) >> (10 + shl);
          src[j + s] = (t.c * b + t.s * a + cof) >> (10 + shl);
        }
      }
    }
  }
}

Void TrQuant::InvNsstNxN( Int* src, const UInt uiMode, const UInt uiIndex, const UInt uiSize )
{
  const int   rnd = uiSize >> 1;
  const int   shl = 5;
  const int * par = (uiSize > 4) ? g_nsstHyGTPar8x8[uiMode][uiIndex] : /*((uiSize > 2) ?*/ g_nsstHyGTPar4x4[uiMode][uiIndex] /*: g_nsstHyGTPar2x2[uiMode][uiIndex])*/;
  const int   cof = 1 << (shl + 9);
  const int  kMax = (Int)(uiSize * uiSize);
  const int  kLog = g_aucLog2[kMax];
  const int  iMax = kMax >> 1;

  CHECK( uiIndex >= 4, "Invalid NSST index" );
  CHECK( /*uiSize != 2 &&*/ uiSize != 4 && uiSize != 8, "Invalid NSST size" );

  for (int k = 0; k < kMax; k++) src[k] <<= shl;

  for (int r = rnd, q = (kLog * rnd - 1); --r >= 0; )
  {
    for (int d = kLog; --d >= 0; q--)
    {
      const int   s = 1 << d;
      const int * p = par + ((r * kLog + d) * iMax);
      if (q > 0)
      {
        for (int i = 0; i < iMax; i++)
        {
          const tabSinCos   &t = g_tabSinCos[*p++];
          const register int j = i + (i & -s);
          const register int a = src[j];
          const register int b = src[j + s];
          src[j]     = (t.c * a + t.s * b + 512) >> 10;
          src[j + s] = (t.c * b - t.s * a + 512) >> 10;
        }
      }
      else
      {
        for (int i = 0; i < iMax; i++)
        {
          const tabSinCos   &t = g_tabSinCos[*p++];
          const register int j = i + (i & -s);
          const register int a = src[j];
          const register int b = src[j + s];
          src[j]     = (t.c * a + t.s * b + cof) >> (10 + shl);
          src[j + s] = (t.c * b - t.s * a + cof) >> (10 + shl);
        }
      }
    }
  }
}

Void TrQuant::xInvNsst( const TransformUnit &tu, const ComponentID compID )
{
  const CompArea& area   = tu.blocks[compID];
  const UInt width       = area.width;
  const UInt height      = area.height;

  if( tu.cu->nsstIdx && !tu.transformSkip[compID] && width >= 4 && height >= 4 && ( width & 3 ) == 0 && ( height & 3 ) == 0 )
  {
    const UInt uiNSSTIdx    = tu.cu->nsstIdx;
    const UInt uiScanIdx    = TU::getCoefScanIdx( tu, compID );
    const bool whge3        = width >= 8 && height >= 8;
    const bool whgt3        = width >  8 && height >  8;
    const UInt *scan        = ( ( tu.cs->pcv->rectCUs && whge3 ) || whgt3 ) ? g_auiCoefTopLeftDiagScan8x8[gp_sizeIdxInfo->idxFrom( width )] : g_scanOrder[SCAN_GROUPED_4x4][uiScanIdx][gp_sizeIdxInfo->idxFrom( width )][gp_sizeIdxInfo->idxFrom( height )];
    UInt uiIntraMode        = PU::getFinalIntraMode( *tu.cs->getPU( area.pos(), toChannelType( compID ) ), toChannelType( compID ) );

    if( PU::isLMCMode( tu.cs->getPU( area.pos(), toChannelType( compID ) )->intraDir[ toChannelType( compID ) ] ) )
    {
      uiIntraMode = PLANAR_IDX;
    }

    CHECK( uiIntraMode >= NUM_INTRA_MODE - 1, "Invalid intra mode" );

    if( uiNSSTIdx < ( uiIntraMode <= DC_IDX ? 3 : 4 ) )
    {
      const Int iLog2SbSize = ( width >= 8 && height >= 8 ) ? 3 : 2;
      const Int iSbSize     = ( width >= 8 && height >= 8 ) ? 8 : 4;
      const Int iSubGrpXMax = Clip3( 1, 8, ( Int ) width  ) >> iLog2SbSize;
      const Int iSubGrpYMax = Clip3( 1, 8, ( Int ) height ) >> iLog2SbSize;
      const UChar * permut  = ( iSbSize == 4 ) ? g_nsstHyGTPermut4x4[g_NsstLut[uiIntraMode]][uiNSSTIdx - 1] : g_nsstHyGTPermut8x8[g_NsstLut[uiIntraMode]][uiNSSTIdx - 1];
      TCoeff * NSST_MATRIX  = m_tempMatrix;
      TCoeff * piNsstTemp   = NSST_MATRIX;
      TCoeff * piCoeffTemp  = m_plTempCoeff;

      for( Int iSubGroupX = 0; iSubGroupX < iSubGrpXMax; iSubGroupX++ )
      {
        for( Int iSubGroupY = 0; iSubGroupY < iSubGrpYMax; iSubGroupY++ )
        {
          const Int iOffsetX = iSbSize * iSubGroupX;
          const Int iOffsetY = iSbSize * iSubGroupY * width;
          Int y;
          piNsstTemp  = NSST_MATRIX; // inverse spectral rearrangement
          piCoeffTemp = m_plTempCoeff + iOffsetX + iOffsetY;

          for( y = 0; y < iSbSize * iSbSize; y++ )
          {
            piNsstTemp[permut[y]] = piCoeffTemp[scan[y]];
          }

          InvNsstNxN( NSST_MATRIX, g_NsstLut[uiIntraMode], uiNSSTIdx - 1, iSbSize );

          piNsstTemp  = NSST_MATRIX; // inverse Hyper-Givens transform
          piCoeffTemp = m_plTempCoeff + iOffsetX + iOffsetY;

          for( y = 0; y < iSbSize; y++ )
          {
            if( uiIntraMode > DIA_IDX )
            {
              if( iSbSize == 4 )
              {
                piCoeffTemp[0] = piNsstTemp[ 0];  piCoeffTemp[1] = piNsstTemp[ 4];
                piCoeffTemp[2] = piNsstTemp[ 8];  piCoeffTemp[3] = piNsstTemp[12];
              }
              else // ( iSbSize == 8 )
              {
                piCoeffTemp[0] = piNsstTemp[ 0];  piCoeffTemp[1] = piNsstTemp[ 8];
                piCoeffTemp[2] = piNsstTemp[16];  piCoeffTemp[3] = piNsstTemp[24];
                piCoeffTemp[4] = piNsstTemp[32];  piCoeffTemp[5] = piNsstTemp[40];
                piCoeffTemp[6] = piNsstTemp[48];  piCoeffTemp[7] = piNsstTemp[56];
              }
              piNsstTemp++;
            }
            else
            {
              ::memcpy( piCoeffTemp, piNsstTemp, iSbSize * sizeof( TCoeff ) );
              piNsstTemp += iSbSize;
            }
            piCoeffTemp += width;
          }
        }
      } // iSubGroupX
    }
  }
}


Void TrQuant::xFwdNsst( const TransformUnit &tu, const ComponentID compID )
{
  const CompArea& area   = tu.blocks[compID];
  const UInt width       = area.width;
  const UInt height      = area.height;

  if( tu.cu->nsstIdx && !tu.transformSkip[compID] && width >= 4 && height >= 4 && ( width & 3 ) == 0 && ( height & 3 ) == 0 )
  {
    const UInt uiNSSTIdx    = tu.cu->nsstIdx;
    const UInt uiScanIdx    = TU::getCoefScanIdx( tu, compID );
    const bool whge3        = width >= 8 && height >= 8;
    const bool whgt3        = width >  8 && height >  8;
    const UInt *scan        = ( ( tu.cs->pcv->rectCUs && whge3 ) || whgt3 ) ? g_auiCoefTopLeftDiagScan8x8[gp_sizeIdxInfo->idxFrom(width)] : g_scanOrder[SCAN_GROUPED_4x4][uiScanIdx][gp_sizeIdxInfo->idxFrom(width)][gp_sizeIdxInfo->idxFrom(height)];
    UInt uiIntraMode        = PU::getFinalIntraMode( *tu.cs->getPU( area.pos(), toChannelType( compID ) ), toChannelType( compID ) );

    if( PU::isLMCMode( tu.cs->getPU( area.pos(), toChannelType( compID ) )->intraDir[ toChannelType( compID ) ] ) )
    {
      uiIntraMode = PLANAR_IDX;
    }

    CHECK( ( isLuma( compID ) || isChroma( tu.cs->chType ) ) && uiNSSTIdx == ( uiIntraMode <= DC_IDX ? 3 : 4 ), "nsst evaluated" );
    CHECK( uiIntraMode >= NUM_INTRA_MODE - 1, "Invalid intra mode" );

    if( uiNSSTIdx < ( uiIntraMode <= DC_IDX ? 3 : 4 ) )
    {
      const Int iLog2SbSize = ( width >= 8 && height >= 8 ) ? 3 : 2;
      const Int iSbSize     = ( width >= 8 && height >= 8 ) ? 8 : 4;
      const Int iSubGrpXMax = Clip3( 1, 8, ( Int ) width  ) >> iLog2SbSize;
      const Int iSubGrpYMax = Clip3( 1, 8, ( Int ) height ) >> iLog2SbSize;
      const UChar * permut  = ( iSbSize == 4 ) ? g_nsstHyGTPermut4x4[g_NsstLut[uiIntraMode]][uiNSSTIdx - 1] : g_nsstHyGTPermut8x8[g_NsstLut[uiIntraMode]][uiNSSTIdx - 1];
      TCoeff * NSST_MATRIX  = m_tempMatrix;
      TCoeff * piNsstTemp   = NSST_MATRIX;
      TCoeff * piCoeffTemp  = m_plTempCoeff;

      for( Int iSubGroupX = 0; iSubGroupX < iSubGrpXMax; iSubGroupX++ )
      {
        for( Int iSubGroupY = 0; iSubGroupY < iSubGrpYMax; iSubGroupY++ )
        {
          const Int iOffsetX = iSbSize * iSubGroupX;
          const Int iOffsetY = iSbSize * iSubGroupY * width;
          Int y;
          piNsstTemp  = NSST_MATRIX; // forward Hyper-Givens transform
          piCoeffTemp = m_plTempCoeff + iOffsetX + iOffsetY;

          for( y = 0; y < iSbSize; y++ )
          {
            if( uiIntraMode > DIA_IDX )
            {
              if( iSbSize == 4 )
              {
                piNsstTemp[ 0] = piCoeffTemp[0];  piNsstTemp[ 4] = piCoeffTemp[1];
                piNsstTemp[ 8] = piCoeffTemp[2];  piNsstTemp[12] = piCoeffTemp[3];
              }
              else // ( iSbSize == 8 )
              {
                piNsstTemp[ 0] = piCoeffTemp[0];  piNsstTemp[ 8] = piCoeffTemp[1];
                piNsstTemp[16] = piCoeffTemp[2];  piNsstTemp[24] = piCoeffTemp[3];
                piNsstTemp[32] = piCoeffTemp[4];  piNsstTemp[40] = piCoeffTemp[5];
                piNsstTemp[48] = piCoeffTemp[6];  piNsstTemp[56] = piCoeffTemp[7];
              }
              piNsstTemp++;
            }
            else
            {
              ::memcpy( piNsstTemp, piCoeffTemp, iSbSize * sizeof( TCoeff ) );
              piNsstTemp += iSbSize;
            }
            piCoeffTemp += width;
          }

          FwdNsstNxN( NSST_MATRIX, g_NsstLut[uiIntraMode], uiNSSTIdx - 1, iSbSize );

          piNsstTemp  = NSST_MATRIX; // forward spectral rearrangement
          piCoeffTemp = m_plTempCoeff + iOffsetX + iOffsetY;

          for( y = 0; y < iSbSize * iSbSize; y++ )
          {
            piCoeffTemp[scan[y]] = piNsstTemp[permut[y]];
          }
        }
      } // iSubGroupX
    }
  }
}

Void TrQuant::invTransformNxN(       TransformUnit &tu,
                               const ComponentID   &compID,
                                     PelBuf        &pResi,
                               const QpParam       &cQP     )
{
  const CompArea &area    = tu.blocks[compID];
  const UInt uiWidth      = area.width;
  const UInt uiHeight     = area.height;

  if (tu.cu->transQuantBypass)
  {
    // where should this logic go?
    const Bool rotateResidual = TU::isNonTransformedResidualRotated(tu, compID);
    const CCoeffBuf pCoeff    = tu.getCoeffs(compID);

    for (UInt y = 0, coefficientIndex = 0; y < uiHeight; y++)
    {
      for (UInt x = 0; x < uiWidth; x++, coefficientIndex++)
      {
        pResi.at(x, y) = rotateResidual ? pCoeff.at(pCoeff.width - x - 1, pCoeff.height - y - 1) : pCoeff.at(x, y);
      }
    }
  }
  else
  {
    CoeffBuf tmpCoeff = CoeffBuf(m_plTempCoeff, tu.blocks[compID]);
    xDeQuant(tu, tmpCoeff, compID, cQP);

    DTRACE_COEFF_BUF( D_TCOEFF, tmpCoeff, tu, tu.cu->predMode, compID );
    if( tu.cs->sps->getSpsNext().getUseNSST() )
    {
      xInvNsst( tu, compID );
    }

    if( tu.transformSkip[compID] )
    {
      xITransformSkip( tmpCoeff, pResi, tu, compID );
    }
    else
    {
      const Int channelBitDepth = tu.cs->sps->getBitDepth( toChannelType( compID ) );

      xIT( channelBitDepth, TU::useDST( tu, compID ), tmpCoeff, pResi, tu.cs->sps->getMaxLog2TrDynamicRange( toChannelType( compID ) ), getEmtMode( tu, compID ), getEmtTrIdx( tu, compID ) );
    }
  }

  //DTRACE_BLOCK_COEFF(tu.getCoeffs(compID), tu, tu.cu->predMode, compID);
  DTRACE_PEL_BUF( D_RESIDUALS, pResi, tu, tu.cu->predMode, compID);
  invRdpcmNxN(tu, compID, pResi);
}

Void TrQuant::invRdpcmNxN(TransformUnit& tu, const ComponentID &compID, PelBuf &pcResidual)
{
  const CompArea &area    = tu.blocks[compID];

  if (CU::isRDPCMEnabled(*tu.cu) && ((tu.transformSkip[compID] != 0) || tu.cu->transQuantBypass))
  {
    const UInt uiWidth  = area.width;
    const UInt uiHeight = area.height;

    RDPCMMode rdpcmMode = RDPCM_OFF;

    if (tu.cu->predMode == MODE_INTRA)
    {
      const ChannelType chType = toChannelType(compID);
      const UInt uiChFinalMode = PU::getFinalIntraMode(*tu.cs->getPU(area.pos(), chType), chType);

      if (uiChFinalMode == VER_IDX || uiChFinalMode == HOR_IDX)
      {
        rdpcmMode = (uiChFinalMode == VER_IDX) ? RDPCM_VER : RDPCM_HOR;
      }
    }
    else  // not intra case
    {
      rdpcmMode = RDPCMMode(tu.rdpcm[compID]);
    }

    const TCoeff pelMin = (TCoeff) std::numeric_limits<Pel>::min();
    const TCoeff pelMax = (TCoeff) std::numeric_limits<Pel>::max();

    if (rdpcmMode == RDPCM_VER)
    {
      for (UInt uiX = 0; uiX < uiWidth; uiX++)
      {
        TCoeff accumulator = pcResidual.at(uiX, 0); // 32-bit accumulator

        for (UInt uiY = 1; uiY < uiHeight; uiY++)
        {
          accumulator            += pcResidual.at(uiX, uiY);
          pcResidual.at(uiX, uiY) = (Pel) Clip3<TCoeff>(pelMin, pelMax, accumulator);
        }
      }
    }
    else if (rdpcmMode == RDPCM_HOR)
    {
      for (UInt uiY = 0; uiY < uiHeight; uiY++)
      {
        TCoeff accumulator = pcResidual.at(0, uiY);

        for (UInt uiX = 1; uiX < uiWidth; uiX++)
        {
          accumulator            += pcResidual.at(uiX, uiY);
          pcResidual.at(uiX, uiY) = (Pel) Clip3<TCoeff>(pelMin, pelMax, accumulator);
        }
      }
    }
  }
}

// ------------------------------------------------------------------------------------------------
// Logical transform
// ------------------------------------------------------------------------------------------------

/** Wrapper function between HM interface and core NxN forward transform (2D)
 */
Void TrQuant::xT( const Int channelBitDepth, Bool useDST, const Pel* piBlkResi, UInt uiStride, TCoeff* psCoeff, Int iWidth, Int iHeight, const Int maxLog2TrDynamicRange, UChar ucMode, UChar ucTrIdx )
{
#if MATRIX_MULT
  if( ucTrIdx != DCT2_HEVC )
  {
    CHECK( iWidth != iHeight, "The matrix multiplication operation is only implemented for the case width = height when EMT is active!" );
    xTr_EMT( channelBitDepth, piBlkResi, psCoeff, uiStride, ( UInt ) iWidth, useDST, maxLog2TrDynamicRange, ucMode, ucTrIdx );
  }
  else
  {
    xTr    ( channelBitDepth, piBlkResi, uiStride, psCoeff, iWidth, iHeight, useDST, maxLog2TrDynamicRange );
  }
#else
  if( ucTrIdx != DCT2_HEVC )
  {
    xTrMxN_EMT( channelBitDepth, piBlkResi, uiStride, psCoeff, iWidth, iHeight, useDST, maxLog2TrDynamicRange, ucMode, ucTrIdx, m_use65IntraModes, m_rectTUs );
  }
  else
  {
    m_fTr     ( channelBitDepth, piBlkResi, uiStride, psCoeff, iWidth, iHeight, useDST, maxLog2TrDynamicRange );
  }
#endif
}

/** Wrapper function between HM interface and core NxN inverse transform (2D)
 */
Void TrQuant::xIT(const Int       &channelBitDepth,
                  const Bool      &useDST,
                  const CCoeffBuf &pCoeff,
                        PelBuf    &pResidual,
                  const Int       &maxLog2TrDynamicRange,
                        UChar      ucMode,
                        UChar      ucTrIdx
)
{
#if MATRIX_MULT
  if (ucTrIdx != DCT2_HEVC)
  {
    CHECK( iWidth != iHeight, "The matrix multiplication operation is only implemented for the case width = height when EMT is active!" );
    xITr_EMT( channelBitDepth, pCoeff.buf, pResidual.buf, pResidual.stride, pCoeff.width, pCoeff.height, useDST, maxLog2TrDynamicRange, ucMode, ucTrIdx );
  }
  else
  {
    xITr    ( channelBitDepth, pCoeff.buf, pResidual.buf, pResidual.stride, pCoeff.width, pCoeff.height, useDST, maxLog2TrDynamicRange );
  }
#else
  if( ucTrIdx != DCT2_HEVC )
  {
    Int iSkipWidth = 0, iSkipHeight = 0;

    if( m_rectTUs )
    {
      iSkipWidth  = ( pCoeff.width  > JVET_C0024_ZERO_OUT_TH ? pCoeff.width  - JVET_C0024_ZERO_OUT_TH : 0 );
      iSkipHeight = ( pCoeff.height > JVET_C0024_ZERO_OUT_TH ? pCoeff.height - JVET_C0024_ZERO_OUT_TH : 0 );
    }
    else if( ( ( ucMode == INTER_MODE_IDX || pCoeff.width == 64 ) && ucTrIdx != DCT2_EMT && pCoeff.width >= JVET_C0024_ZERO_OUT_TH ) || ( ucTrIdx == DCT2_EMT && pCoeff.width == 64 ) )
    {
      iSkipWidth  = pCoeff.width  >> 1;
      iSkipHeight = pCoeff.height >> 1;
    }

    xITrMxN_EMT( channelBitDepth, pCoeff.buf, pResidual.buf, pResidual.stride, pCoeff.width, pCoeff.height, iSkipWidth, iSkipHeight, useDST, maxLog2TrDynamicRange, ucMode, ucTrIdx, m_use65IntraModes );
  }
  else
  {
    m_fITr     ( channelBitDepth, pCoeff.buf, pResidual.buf, pResidual.stride, pCoeff.width, pCoeff.height,                          useDST, maxLog2TrDynamicRange );
  }
#endif
}

/** Wrapper function between HM interface and core NxN transform skipping
 */
Void TrQuant::xITransformSkip(const CCoeffBuf     &pCoeff,
                                    PelBuf        &pResidual,
                              const TransformUnit &tu,
                              const ComponentID   &compID)
{
  const CompArea &area      = tu.blocks[compID];
  const Int width           = area.width;
  const Int height          = area.height;
  const Int maxLog2TrDynamicRange = tu.cs->sps->getMaxLog2TrDynamicRange(toChannelType(compID));
  const Int channelBitDepth = tu.cs->sps->getBitDepth(toChannelType(compID));

  Int iTransformShift = getTransformShift(channelBitDepth, area.size(), maxLog2TrDynamicRange);
  if( tu.cs->sps->getSpsRangeExtension().getExtendedPrecisionProcessingFlag() )
  {
    iTransformShift = std::max<Int>( 0, iTransformShift );
  }

  Int iWHScale = 1;
  if( needsBlockSizeQuantScale( area ) )
  {
    iTransformShift += ADJ_QUANT_SHIFT;
    iWHScale = 181;
  }

  const Bool rotateResidual = TU::isNonTransformedResidualRotated( tu, compID );

  if( iTransformShift >= 0 )
  {
    const TCoeff offset = iTransformShift == 0 ? 0 : ( 1 << ( iTransformShift - 1 ) );

    for( UInt y = 0; y < height; y++ )
    {
      for( UInt x = 0; x < width; x++ )
      {
        pResidual.at( x, y ) = Pel( ( ( rotateResidual ? pCoeff.at( pCoeff.width - x - 1, pCoeff.height - y - 1 ) : pCoeff.at( x, y ) ) * iWHScale + offset ) >> iTransformShift );
      }
    }
  }
  else //for very high bit depths
  {
    iTransformShift = -iTransformShift;

    for( UInt y = 0; y < height; y++ )
    {
      for( UInt x = 0; x < width; x++ )
      {
        pResidual.at( x, y ) = Pel( ( rotateResidual ? pCoeff.at( pCoeff.width - x - 1, pCoeff.height - y - 1 ) : pCoeff.at( x, y ) )  * iWHScale << iTransformShift );
      }
    }
  }
}

/** Get the best level in RD sense
 *
 * \returns best quantized transform level for given scan position
 *
 * This method calculates the best quantized transform level for a given scan position.
 */
inline UInt TrQuant::xGetCodedLevel  ( Double&            rd64CodedCost,
                                       Double&            rd64CodedCost0,
                                       Double&            rd64CodedCostSig,
                                       Intermediate_Int   lLevelDouble,
                                       UInt               uiMaxAbsLevel,
                                       const BinFracBits* fracBitsSig,
                                       const BinFracBits& fracBitsOne,
                                       const BinFracBits& fracBitsAbs,
                                       const bool         useAltRC,
                                       UShort             ui16AbsGoRice,
                                       UInt               c1Idx,
                                       UInt               c2Idx,
                                       Int                iQBits,
                                       Double             errorScale,
                                       Bool               bLast,
                                       Bool               useLimitedPrefixLength,
                                       const Int          maxLog2TrDynamicRange
                                     ) const
{
  Double dCurrCostSig   = 0;
  UInt   uiBestAbsLevel = 0;

  if( !bLast && uiMaxAbsLevel < 3 )
  {
    rd64CodedCostSig    = xGetRateSigCoef( *fracBitsSig, 0 );
    rd64CodedCost       = rd64CodedCost0 + rd64CodedCostSig;
    if( uiMaxAbsLevel == 0 )
    {
      return uiBestAbsLevel;
    }
  }
  else
  {
    rd64CodedCost       = MAX_DOUBLE;
  }

  if( !bLast )
  {
    dCurrCostSig        = xGetRateSigCoef( *fracBitsSig, 1 );
  }

  UInt uiMinAbsLevel    = ( uiMaxAbsLevel > 1 ? uiMaxAbsLevel - 1 : 1 );
  for( Int uiAbsLevel  = uiMaxAbsLevel; uiAbsLevel >= uiMinAbsLevel ; uiAbsLevel-- )
  {
    Double dErr         = Double( lLevelDouble  - ( Intermediate_Int(uiAbsLevel) << iQBits ) );
    Double dCurrCost    = dErr * dErr * errorScale + xGetICost( xGetICRate( uiAbsLevel, fracBitsOne, fracBitsAbs, useAltRC, ui16AbsGoRice, c1Idx, c2Idx, useLimitedPrefixLength, maxLog2TrDynamicRange ) );
    dCurrCost          += dCurrCostSig;

    if( dCurrCost < rd64CodedCost )
    {
      uiBestAbsLevel    = uiAbsLevel;
      rd64CodedCost     = dCurrCost;
      rd64CodedCostSig  = dCurrCostSig;
    }
  }

  return uiBestAbsLevel;
}

/** Calculates the cost for specific absolute transform level
 * \param uiAbsLevel scaled quantized level
 * \param ui16CtxNumOne current ctxInc for coeff_abs_level_greater1 (1st bin of coeff_abs_level_minus1 in AVC)
 * \param ui16CtxNumAbs current ctxInc for coeff_abs_level_greater2 (remaining bins of coeff_abs_level_minus1 in AVC)
 * \param ui16AbsGoRice Rice parameter for coeff_abs_level_minus3
 * \param c1Idx
 * \param c2Idx
 * \param useLimitedPrefixLength
 * \param maxLog2TrDynamicRange
 * \returns cost of given absolute transform level
 */
inline Int TrQuant::xGetICRate  ( const UInt   uiAbsLevel,
                                  const BinFracBits& fracBitsOne,
                                  const BinFracBits& fracBitsAbs,
                                  const bool   useAltRC,
                                  const UShort ui16AbsGoRice,
                                  const UInt   c1Idx,
                                  const UInt   c2Idx,
                                  const Bool   useLimitedPrefixLength,
                                  const Int maxLog2TrDynamicRange
                                ) const
{
  Int  iRate      = Int(xGetIEPRate()); // cost of sign bit
  UInt baseLevel  = (c1Idx < C1FLAG_NUMBER) ? (2 + (c2Idx < C2FLAG_NUMBER)) : 1;

  if ( uiAbsLevel >= baseLevel )
  {
    UInt symbol     = uiAbsLevel - baseLevel;
    UInt length;
    const int threshold = useAltRC ? g_auiGoRiceRange[ ui16AbsGoRice ] : COEF_REMAIN_BIN_REDUCTION;
    if( symbol < ( threshold << ui16AbsGoRice ) )
    {
      length = symbol>>ui16AbsGoRice;
      iRate += (length+1+ui16AbsGoRice)<< SCALE_BITS;
    }
    else if (useLimitedPrefixLength)
    {
      const UInt maximumPrefixLength = (32 - (COEF_REMAIN_BIN_REDUCTION + maxLog2TrDynamicRange));

      UInt prefixLength = 0;
      UInt suffix       = (symbol >> ui16AbsGoRice) - COEF_REMAIN_BIN_REDUCTION;

      while ((prefixLength < maximumPrefixLength) && (suffix > ((2 << prefixLength) - 2)))
      {
        prefixLength++;
      }

      const UInt suffixLength = (prefixLength == maximumPrefixLength) ? (maxLog2TrDynamicRange - ui16AbsGoRice) : (prefixLength + 1/*separator*/);

      iRate += (COEF_REMAIN_BIN_REDUCTION + prefixLength + suffixLength + ui16AbsGoRice) << SCALE_BITS;
    }
    else
    {
      length = ui16AbsGoRice;
      symbol  = symbol - ( threshold << ui16AbsGoRice);
      while (symbol >= (1<<length))
      {
        symbol -=  (1<<(length++));
      }
      iRate += (threshold+length+1-ui16AbsGoRice+length)<< SCALE_BITS;
    }

    if (c1Idx < C1FLAG_NUMBER)
    {
      iRate += fracBitsOne.intBits[1];

      if (c2Idx < C2FLAG_NUMBER)
      {
        iRate += fracBitsAbs.intBits[1];
      }
    }
  }
  else if( uiAbsLevel == 1 )
  {
    iRate += fracBitsOne.intBits[0];
  }
  else if( uiAbsLevel == 2 )
  {
    iRate += fracBitsOne.intBits[1];
    iRate += fracBitsAbs.intBits[0];
  }
  else
  {
    iRate = 0;
  }

  return  iRate;
}


inline Double TrQuant::xGetRateSigCoeffGroup( const BinFracBits& fracBitsSigCG, unsigned uiSignificanceCoeffGroup ) const
{
  return xGetICost( fracBitsSigCG.intBits[uiSignificanceCoeffGroup] );
}

/** Calculates the cost of signaling the last significant coefficient in the block
 * \param uiPosX X coordinate of the last significant coefficient
 * \param uiPosY Y coordinate of the last significant coefficient
 * \param component colour component ID
 * \returns cost of last significant coefficient
 */
/*
 * \param uiWidth width of the transform unit (TU)
*/
inline Double TrQuant::xGetRateLast( const int* lastBitsX, const int* lastBitsY, unsigned PosX, unsigned PosY ) const
{
  UInt    CtxX  = g_uiGroupIdx[PosX];
  UInt    CtxY  = g_uiGroupIdx[PosY];
  Double  Cost  = lastBitsX[ CtxX ] + lastBitsY[ CtxY ];
  if( CtxX > 3 )
  {
    Cost += xGetIEPRate() * ((CtxX-2)>>1);
  }
  if( CtxY > 3 )
  {
    Cost += xGetIEPRate() * ((CtxY-2)>>1);
  }
  return xGetICost( Cost );
}


inline Double TrQuant::xGetRateSigCoef( const BinFracBits& fracBitsSig, unsigned uiSignificance ) const
{
  return xGetICost( fracBitsSig.intBits[uiSignificance] );
}

/** Get the cost for a specific rate
 * \param dRate rate of a bit
 * \returns cost at the specific rate
 */
inline Double TrQuant::xGetICost        ( Double                          dRate         ) const
{
  return m_dLambda * dRate;
}

/** Get the cost of an equal probable bit
 * \returns cost of equal probable bit
 */
inline Double TrQuant::xGetIEPRate      (                                               ) const
{
  return 32768;
}



/** set quantized matrix coefficient for encode
 * \param scalingList            quantized matrix address
 * \param format                 chroma format
 * \param maxLog2TrDynamicRange
 * \param bitDepths              reference to bit depth array for all channels
 */
Void TrQuant::setScalingList(ScalingList *scalingList, const Int maxLog2TrDynamicRange[MAX_NUM_CHANNEL_TYPE], const BitDepths &bitDepths)
{
  const Int minimumQp = 0;
  const Int maximumQp = SCALING_LIST_REM_NUM;

  for(UInt size = 0; size < SCALING_LIST_SIZE_NUM; size++)
  {
    for(UInt list = 0; list < SCALING_LIST_NUM; list++)
    {
      for(Int qp = minimumQp; qp < maximumQp; qp++)
      {
        xSetScalingListEnc(scalingList,list,size,qp);
        xSetScalingListDec(*scalingList,list,size,qp);
        setErrScaleCoeff(list,size, size,qp,maxLog2TrDynamicRange, bitDepths);
      }
    }
  }
}
/** set quantized matrix coefficient for decode
 * \param scalingList quantized matrix address
 * \param format      chroma format
 */
Void TrQuant::setScalingListDec(const ScalingList &scalingList)
{
  const Int minimumQp = 0;
  const Int maximumQp = SCALING_LIST_REM_NUM;

  for(UInt size = 0; size < SCALING_LIST_SIZE_NUM; size++)
  {
    for(UInt list = 0; list < SCALING_LIST_NUM; list++)
    {
      for(Int qp = minimumQp; qp < maximumQp; qp++)
      {
        xSetScalingListDec(scalingList,list,size,qp);
      }
    }
  }
}


Double calcErrScaleCoeffNoScalingList( SizeType width, SizeType height, Int qp, const Int maxLog2TrDynamicRange, const Int channelBitDepth )
{
  const Int iTransformShift = getTransformShift( channelBitDepth, Size( width, height ), maxLog2TrDynamicRange );
  bool   needsSrqt2 = needsBlockSizeQuantScale( Size( width, height ) );// ( ( (sizeX+sizeY) & 1 ) !=0 );
  Double dErrScale = (Double)( 1 << SCALE_BITS );                                // Compensate for scaling of bitcount in Lagrange cost function

  Double dTransShift = (Double)iTransformShift + ( needsSrqt2 ? -0.5 : 0.0 );
  dErrScale = dErrScale*pow( 2.0, ( -2.0*dTransShift ) );                     // Compensate for scaling through forward transform

  Int QStep = ( needsSrqt2 ? ( ( g_quantScales[qp] * 181 ) >> 7 ) : g_quantScales[qp] );
  return dErrScale / QStep / QStep / ( 1 << DISTORTION_PRECISION_ADJUSTMENT( 2 * ( channelBitDepth - 8 ) ) );
}

/** set error scale coefficients
 * \param list                   list ID
 * \param size
 * \param qp                     quantization parameter
 * \param maxLog2TrDynamicRange
 * \param bitDepths              reference to bit depth array for all channels
 */
Void TrQuant::setErrScaleCoeff( UInt list, UInt sizeX, UInt sizeY, Int qp, const Int maxLog2TrDynamicRange[MAX_NUM_CHANNEL_TYPE], const BitDepths &bitDepths )
{
  const ChannelType channelType = ( ( list == 0 ) || ( list == MAX_NUM_COMPONENT ) ) ? CHANNEL_TYPE_LUMA : CHANNEL_TYPE_CHROMA;

  const Int channelBitDepth = bitDepths.recon[channelType];
  const Int iTransformShift = getTransformShift( channelBitDepth, Size( g_scalingListSizeX[sizeX], g_scalingListSizeX[sizeY] ), maxLog2TrDynamicRange[channelType] );  // Represents scaling through forward transform

  UInt i, uiMaxNumCoeff = g_scalingListSizeX[sizeX] * g_scalingListSizeX[sizeY];
  Int *piQuantcoeff;
  Double *pdErrScale;
  piQuantcoeff = getQuantCoeff( list, qp, sizeX, sizeY );
  pdErrScale   = getErrScaleCoeff( list, sizeX, sizeY, qp );

  Double dErrScale = (Double)( 1 << SCALE_BITS );                                // Compensate for scaling of bitcount in Lagrange cost function
  bool   needsSrqt2 = needsBlockSizeQuantScale( Size( g_scalingListSizeX[sizeX], g_scalingListSizeX[sizeY] ) );// ( ( (sizeX+sizeY) & 1 ) !=0 );
  Double dTransShift = (Double)iTransformShift + ( needsSrqt2 ? -0.5 : 0.0 );
  dErrScale = dErrScale*pow( 2.0, ( -2.0*dTransShift ) );                     // Compensate for scaling through forward transform

  for( i = 0; i < uiMaxNumCoeff; i++ )
  {
    pdErrScale[i] = dErrScale / piQuantcoeff[i] / piQuantcoeff[i] / ( 1 << DISTORTION_PRECISION_ADJUSTMENT( 2 * ( bitDepths.recon[channelType] - 8 ) ) );
  }
  Int QStep = ( needsSrqt2 ? ( ( g_quantScales[qp] * 181 ) >> 7 ) : g_quantScales[qp] );
  getErrScaleCoeffNoScalingList( list, sizeX, sizeY, qp ) = dErrScale / QStep / QStep / ( 1 << DISTORTION_PRECISION_ADJUSTMENT( 2 * ( bitDepths.recon[channelType] - 8 ) ) );
}

/** set quantized matrix coefficient for encode
 * \param scalingList quantized matrix address
 * \param listId List index
 * \param sizeId size index
 * \param qp Quantization parameter
 * \param format chroma format
 */
Void TrQuant::xSetScalingListEnc(ScalingList *scalingList, UInt listId, UInt sizeId, Int qp)
{
  UInt width  = g_scalingListSizeX[sizeId];
  UInt height = g_scalingListSizeX[sizeId];
  UInt ratio  = g_scalingListSizeX[sizeId]/std::min(MAX_MATRIX_SIZE_NUM,(Int)g_scalingListSizeX[sizeId]);
  Int *quantcoeff;
  Int *coeff  = scalingList->getScalingListAddress(sizeId,listId);
  quantcoeff  = getQuantCoeff(listId, qp, sizeId, sizeId);

  Int quantScales = g_quantScales[qp];

  processScalingListEnc(coeff,
                        quantcoeff,
                        (quantScales << LOG2_SCALING_LIST_NEUTRAL_VALUE),
                        height, width, ratio,
                        std::min(MAX_MATRIX_SIZE_NUM, (Int)g_scalingListSizeX[sizeId]),
                        scalingList->getScalingListDC(sizeId,listId));
}

/** set quantized matrix coefficient for decode
 * \param scalingList quantaized matrix address
 * \param listId List index
 * \param sizeId size index
 * \param qp Quantization parameter
 * \param format chroma format
 */
Void TrQuant::xSetScalingListDec(const ScalingList &scalingList, UInt listId, UInt sizeId, Int qp)
{
  UInt width  = g_scalingListSizeX[sizeId];
  UInt height = g_scalingListSizeX[sizeId];
  UInt ratio  = g_scalingListSizeX[sizeId]/std::min(MAX_MATRIX_SIZE_NUM,(Int)g_scalingListSizeX[sizeId]);
  Int *dequantcoeff;
  const Int *coeff  = scalingList.getScalingListAddress(sizeId,listId);

  dequantcoeff = getDequantCoeff(listId, qp, sizeId, sizeId);

  Int invQuantScale = g_invQuantScales[qp];

  processScalingListDec(coeff,
                        dequantcoeff,
                        invQuantScale,
                        height, width, ratio,
                        std::min(MAX_MATRIX_SIZE_NUM, (Int)g_scalingListSizeX[sizeId]),
                        scalingList.getScalingListDC(sizeId,listId));
}

/** set flat matrix value to quantized coefficient
 */
Void TrQuant::setFlatScalingList(const Int maxLog2TrDynamicRange[MAX_NUM_CHANNEL_TYPE], const BitDepths &bitDepths)
{
  const Int minimumQp = 0;
  const Int maximumQp = SCALING_LIST_REM_NUM;

  for(UInt sizeX = 0; sizeX < SCALING_LIST_SIZE_NUM; sizeX++)
  {
    for(UInt sizeY = 0; sizeY < SCALING_LIST_SIZE_NUM; sizeY++)
    {
      for(UInt list = 0; list < SCALING_LIST_NUM; list++)
      {
        for(Int qp = minimumQp; qp < maximumQp; qp++)
        {
          xsetFlatScalingList( list, sizeX, sizeY, qp );
          setErrScaleCoeff( list, sizeX, sizeY, qp, maxLog2TrDynamicRange, bitDepths );
        }
      }
    }
  }
}

/** set flat matrix value to quantized coefficient
 * \param list List ID
 * \param size size index
 * \param qp Quantization parameter
 * \param format chroma format
 */
Void TrQuant::xsetFlatScalingList(UInt list, UInt sizeX, UInt sizeY, Int qp)
{
  UInt i,num = g_scalingListSizeX[sizeX]*g_scalingListSizeX[sizeY];
  Int *quantcoeff;
  Int *dequantcoeff;

  Int quantScales    = g_quantScales   [qp];
  Int invQuantScales = g_invQuantScales[qp] << 4;

  quantcoeff   = getQuantCoeff(list, qp, sizeX, sizeY);
  dequantcoeff = getDequantCoeff(list, qp, sizeX, sizeY);

  for(i=0;i<num;i++)
  {
    *quantcoeff++ = quantScales;
    *dequantcoeff++ = invQuantScales;
  }
}

/** set quantized matrix coefficient for encode
 * \param coeff quantaized matrix address
 * \param quantcoeff quantaized matrix address
 * \param quantScales Q(QP%6)
 * \param height height
 * \param width width
 * \param ratio ratio for upscale
 * \param sizuNum matrix size
 * \param dc dc parameter
 */
Void TrQuant::processScalingListEnc( Int *coeff, Int *quantcoeff, Int quantScales, UInt height, UInt width, UInt ratio, Int sizuNum, UInt dc)
{
  for(UInt j=0;j<height;j++)
  {
    for(UInt i=0;i<width;i++)
    {
      quantcoeff[j*width + i] = quantScales / coeff[sizuNum * (j / ratio) + i / ratio];
    }
  }

  if(ratio > 1)
  {
    quantcoeff[0] = quantScales / dc;
  }
}

/** set quantized matrix coefficient for decode
 * \param coeff quantaized matrix address
 * \param dequantcoeff quantaized matrix address
 * \param invQuantScales IQ(QP%6))
 * \param height height
 * \param width width
 * \param ratio ratio for upscale
 * \param sizuNum matrix size
 * \param dc dc parameter
 */
Void TrQuant::processScalingListDec( const Int *coeff, Int *dequantcoeff, Int invQuantScales, UInt height, UInt width, UInt ratio, Int sizuNum, UInt dc)
{
  for(UInt j=0;j<height;j++)
  {
    for(UInt i=0;i<width;i++)
    {
      dequantcoeff[j*width + i] = invQuantScales * coeff[sizuNum * (j / ratio) + i / ratio];
    }
  }

  if(ratio > 1)
  {
    dequantcoeff[0] = invQuantScales * dc;
  }
}

/** initialization process of scaling list array
 */
Void TrQuant::initScalingList()
{
  for(UInt sizeIdX = 0; sizeIdX < SCALING_LIST_SIZE_NUM; sizeIdX++)
  {
    for(UInt sizeIdY = 0; sizeIdY < SCALING_LIST_SIZE_NUM; sizeIdY++)
    {
      for(UInt qp = 0; qp < SCALING_LIST_REM_NUM; qp++)
      {
        for(UInt listId = 0; listId < SCALING_LIST_NUM; listId++)
        {
          m_quantCoef   [sizeIdX][sizeIdY][listId][qp] = new Int    [g_scalingListSizeX[sizeIdX]*g_scalingListSizeX[sizeIdY]];
          m_dequantCoef [sizeIdX][sizeIdY][listId][qp] = new Int    [g_scalingListSizeX[sizeIdX]*g_scalingListSizeX[sizeIdY]];
          m_errScale    [sizeIdX][sizeIdY][listId][qp] = new Double [g_scalingListSizeX[sizeIdX]*g_scalingListSizeX[sizeIdY]];
        } // listID loop
      }
    }
  }
}

/** destroy quantization matrix array
 */
Void TrQuant::destroyScalingList()
{
  for(UInt sizeIdX = 0; sizeIdX < SCALING_LIST_SIZE_NUM; sizeIdX++)
  {
    for(UInt sizeIdY = 0; sizeIdY < SCALING_LIST_SIZE_NUM; sizeIdY++)
    {
      for(UInt listId = 0; listId < SCALING_LIST_NUM; listId++)
      {
        for(UInt qp = 0; qp < SCALING_LIST_REM_NUM; qp++)
        {
          if(m_quantCoef[sizeIdX][sizeIdY][listId][qp])
          {
            delete [] m_quantCoef[sizeIdX][sizeIdY][listId][qp];
          }
          if(m_dequantCoef[sizeIdX][sizeIdY][listId][qp])
          {
            delete [] m_dequantCoef[sizeIdX][sizeIdY][listId][qp];
          }
          if(m_errScale[sizeIdX][sizeIdY][listId][qp])
          {
            delete [] m_errScale[sizeIdX][sizeIdY][listId][qp];
          }
        }
      }
    }
  }
}

Void TrQuant::xQuant(TransformUnit &tu, const ComponentID &compID, const CCoeffBuf &pSrc, TCoeff &uiAbsSum, const QpParam &cQP, const Ctx& ctx)
{
  const SPS &sps            = *tu.cs->sps;
  const PPS &pps            = *tu.cs->pps;
  const CompArea &rect      = tu.blocks[compID];
  const UInt uiWidth        = rect.width;
  const UInt uiHeight       = rect.height;
  const Int channelBitDepth = sps.getBitDepth(toChannelType(compID));

  const CCoeffBuf &piCoef   = pSrc;
        CoeffBuf   piQCoef  = tu.getCoeffs(compID);

  const Bool useTransformSkip      = tu.transformSkip[compID];
  const Int  maxLog2TrDynamicRange = sps.getMaxLog2TrDynamicRange(toChannelType(compID));

  Bool useRDOQ = useTransformSkip ? m_useRDOQTS : m_useRDOQ;
  useRDOQ &= uiWidth  > 2;
  useRDOQ &= uiHeight > 2;
  if (useRDOQ && (isLuma(compID) || RDOQ_CHROMA))
  {
#if T0196_SELECTIVE_RDOQ
    if (!m_useSelectiveRDOQ || xNeedRDOQ(tu, compID, piCoef, cQP))
    {
#endif
      xRateDistOptQuant( tu, compID, pSrc, uiAbsSum, cQP, ctx );
#if T0196_SELECTIVE_RDOQ
    }
    else
    {
      piQCoef.fill(0);
      uiAbsSum = 0;
    }
#endif
  }
  else
  {
    CoeffCodingContext cctx(tu, compID, pps.getSignDataHidingEnabledFlag());

    const TCoeff entropyCodingMinimum = -(1 << maxLog2TrDynamicRange);
    const TCoeff entropyCodingMaximum =  (1 << maxLog2TrDynamicRange) - 1;

    TCoeff deltaU[MAX_TU_SIZE * MAX_TU_SIZE];

    Int scalingListType = getScalingListType(tu.cu->predMode, compID);
    CHECK(scalingListType >= SCALING_LIST_NUM, "Invalid scaling list");
    const UInt uiLog2TrWidth = g_aucLog2[uiWidth];
    const UInt uiLog2TrHeight = g_aucLog2[uiHeight];
    Int *piQuantCoeff = getQuantCoeff(scalingListType, cQP.rem, uiLog2TrWidth-1, uiLog2TrHeight-1);

    const Bool enableScalingLists             = getUseScalingList(uiWidth, uiHeight, useTransformSkip);
    const Int  defaultQuantisationCoefficient = g_quantScales[cQP.rem];

    /* for 422 chroma blocks, the effective scaling applied during transformation is not a power of 2, hence it cannot be
     * implemented as a bit-shift (the quantised result will be sqrt(2) * larger than required). Alternatively, adjust the
     * uiLog2TrSize applied in iTransformShift, such that the result is 1/sqrt(2) the required result (i.e. smaller)
     * Then a QP+3 (sqrt(2)) or QP-3 (1/sqrt(2)) method could be used to get the required result
     */
    // Represents scaling through forward transform
    Int iTransformShift = getTransformShift(channelBitDepth, rect.size(), maxLog2TrDynamicRange);

    if (useTransformSkip && sps.getSpsRangeExtension().getExtendedPrecisionProcessingFlag())
    {
      iTransformShift = std::max<Int>(0, iTransformShift);
    }

    Int iWHScale = 1;
    if( needsBlockSizeQuantScale( rect ) )
    {
      iTransformShift += ADJ_QUANT_SHIFT;
      iWHScale = 181;
    }

    const Int iQBits = QUANT_SHIFT + cQP.per + iTransformShift;
    // QBits will be OK for any internal bit depth as the reduction in transform shift is balanced by an increase in Qp_per due to QpBDOffset

    const Int64 iAdd = Int64(tu.cs->slice->getSliceType() == I_SLICE ? 171 : 85) << Int64(iQBits - 9);
    const Int qBits8 = iQBits - 8;

    for (Int uiBlockPos = 0; uiBlockPos < piQCoef.area(); uiBlockPos++)
    {
      const TCoeff iLevel   = piCoef.buf[uiBlockPos];
      const TCoeff iSign    = (iLevel < 0 ? -1: 1);

      const Int64  tmpLevel = (Int64)abs(iLevel) * (enableScalingLists ? piQuantCoeff[uiBlockPos] : defaultQuantisationCoefficient);

      const TCoeff quantisedMagnitude = TCoeff((tmpLevel * iWHScale + iAdd ) >> iQBits);
      deltaU[uiBlockPos] = (TCoeff)((tmpLevel * iWHScale - ((Int64)quantisedMagnitude<<iQBits) )>> qBits8);

      uiAbsSum += quantisedMagnitude;
      const TCoeff quantisedCoefficient = quantisedMagnitude * iSign;

      piQCoef.buf[uiBlockPos] = Clip3<TCoeff>( entropyCodingMinimum, entropyCodingMaximum, quantisedCoefficient );
    } // for n

    if( cctx.signHiding() && uiWidth>=4 && uiHeight>=4 )
    {
      if(uiAbsSum >= 2) //this prevents TUs with only one coefficient of value 1 from being tested
      {
        signBitHidingHDQ(piQCoef.buf, piCoef.buf, deltaU, cctx, maxLog2TrDynamicRange);
      }
    }
  } //if RDOQ
  //return;
}

Bool TrQuant::xNeedRDOQ(TransformUnit &tu, const ComponentID &compID, const CCoeffBuf &pSrc, const QpParam &cQP)
{
  const SPS &sps            = *tu.cs->sps;
  const CompArea &rect      = tu.blocks[compID];
  const UInt uiWidth        = rect.width;
  const UInt uiHeight       = rect.height;
  const Int channelBitDepth = sps.getBitDepth(toChannelType(compID));

  const CCoeffBuf piCoef    = pSrc;

  const Bool useTransformSkip      = tu.transformSkip[compID];
  const Int  maxLog2TrDynamicRange = sps.getMaxLog2TrDynamicRange(toChannelType(compID));

  Int scalingListType = getScalingListType(tu.cu->predMode, compID);
  CHECK(scalingListType >= SCALING_LIST_NUM, "Invalid scaling list");

  const UInt uiLog2TrWidth  = g_aucLog2[uiWidth];
  const UInt uiLog2TrHeight = g_aucLog2[uiHeight];
  Int *piQuantCoeff = getQuantCoeff(scalingListType, cQP.rem, uiLog2TrWidth-1, uiLog2TrHeight-1);

  const Bool enableScalingLists             = getUseScalingList(uiWidth, uiHeight, (useTransformSkip != 0));
  const Int  defaultQuantisationCoefficient = g_quantScales[cQP.rem];

  /* for 422 chroma blocks, the effective scaling applied during transformation is not a power of 2, hence it cannot be
    * implemented as a bit-shift (the quantised result will be sqrt(2) * larger than required). Alternatively, adjust the
    * uiLog2TrSize applied in iTransformShift, such that the result is 1/sqrt(2) the required result (i.e. smaller)
    * Then a QP+3 (sqrt(2)) or QP-3 (1/sqrt(2)) method could be used to get the required result
    */

  // Represents scaling through forward transform
  Int iTransformShift = getTransformShift(channelBitDepth, rect.size(), maxLog2TrDynamicRange);

  if (useTransformSkip && sps.getSpsRangeExtension().getExtendedPrecisionProcessingFlag())
  {
    iTransformShift = std::max<Int>(0, iTransformShift);
  }

  Int iWHScale = 1;
  if( needsBlockSizeQuantScale( rect ) )
  {
    iTransformShift += ADJ_QUANT_SHIFT;
    iWHScale = 181;
  }

  const Int iQBits = QUANT_SHIFT + cQP.per + iTransformShift;
  // QBits will be OK for any internal bit depth as the reduction in transform shift is balanced by an increase in Qp_per due to QpBDOffset

  // iAdd is different from the iAdd used in normal quantization
  const Int64 iAdd = Int64(compID == COMPONENT_Y ? 171 : 256) << (iQBits - 9);

  for (Int uiBlockPos = 0; uiBlockPos < rect.area(); uiBlockPos++)
  {
    const TCoeff iLevel   = piCoef.buf[uiBlockPos];
    const Int64  tmpLevel = (Int64)abs(iLevel) * (enableScalingLists ? piQuantCoeff[uiBlockPos] : defaultQuantisationCoefficient);
    const TCoeff quantisedMagnitude = TCoeff((tmpLevel * iWHScale + iAdd ) >> iQBits);

    if (quantisedMagnitude != 0)
    {
      return true;
    }
  } // for n
  return false;
}

UChar TrQuant::getEmtTrIdx(TransformUnit tu, const ComponentID compID)
{
  UChar ucTrIdx = DCT2_HEVC;

  if( compID == COMPONENT_Y )
  {
    if( CU::isIntra( *tu.cu ) && tu.cs->sps->getSpsNext().getUseIntraEMT() )
    {
      ucTrIdx = tu.cu->emtFlag ? tu.emtIdx : DCT2_EMT;
    }
    if( !CU::isIntra( *tu.cu ) && tu.cs->sps->getSpsNext().getUseInterEMT() )
    {
      ucTrIdx = tu.cu->emtFlag ? tu.emtIdx : DCT2_EMT;
    }
  }
  else
  {
    if( CU::isIntra( *tu.cu ) && tu.cs->sps->getSpsNext().getUseIntraEMT() )
    {
      ucTrIdx = DCT2_EMT;
    }
    if( !CU::isIntra( *tu.cu ) && tu.cs->sps->getSpsNext().getUseInterEMT() )
    {
      ucTrIdx = DCT2_EMT;
    }
  }

  return ucTrIdx;
}

UChar TrQuant::getEmtMode( TransformUnit tu, const ComponentID compID )
{
  UChar ucMode = 0;

  if( isLuma( compID ) )
  {
    if( CU::isIntra( *tu.cu ) )
    {
      CodingStructure &cs      = *tu.cs;
      const PredictionUnit &pu = *cs.getPU( tu.blocks[compID].pos(), toChannelType( compID ) );
      const UInt uiChFinalMode = PU::getFinalIntraMode( pu, toChannelType( compID ) );
      ucMode                   = m_use65IntraModes ? uiChFinalMode : g_intraMode65to33AngMapping[uiChFinalMode];
    }
    else
    {
      ucMode = INTER_MODE_IDX;
    }
  }

  return ucMode;
}


Void TrQuant::transformNxN(TransformUnit &tu, const ComponentID &compID, const QpParam &cQP, TCoeff &uiAbsSum, const Ctx &ctx)
{
        CodingStructure &cs = *tu.cs;
  const SPS &sps            = *cs.sps;
  const CompArea &rect      = tu.blocks[compID];
  const UInt uiWidth        = rect.width;
  const UInt uiHeight       = rect.height;

  const CPelBuf resiBuf     = cs.getResiBuf(rect);
        CoeffBuf rpcCoeff   = tu.getCoeffs(compID);

  RDPCMMode rdpcmMode = RDPCM_OFF;
  rdpcmNxN(tu, compID, cQP, uiAbsSum, rdpcmMode);

  if (rdpcmMode == RDPCM_OFF)
  {
    uiAbsSum = 0;

    // transform and quantize
    if (CU::isLosslessCoded(*tu.cu))
    {
      const Bool rotateResidual = TU::isNonTransformedResidualRotated( tu, compID );

      for( UInt y = 0; y < uiHeight; y++ )
      {
        for( UInt x = 0; x < uiWidth; x++ )
        {
          const Pel currentSample = resiBuf.at( x, y );

          if( rotateResidual )
          {
            rpcCoeff.at( uiWidth - x - 1, uiHeight - y - 1 ) = currentSample;
          }
          else
          {
            rpcCoeff.at( x, y ) = currentSample;
          }

          uiAbsSum += TCoeff( abs( currentSample ) );
        }
      }
    }
    else
    {
      CHECK( sps.getMaxTrSize() < uiWidth, "Unsupported transformation size" );

      CoeffBuf tempCoeff( m_plTempCoeff, rect );

      DTRACE_PEL_BUF( D_RESIDUALS, resiBuf, tu, tu.cu->predMode, compID );

      if( tu.transformSkip[compID] )
      {
        xTransformSkip( tu, compID, resiBuf, tempCoeff.buf );
      }
      else
      {
        const Int channelBitDepth = sps.getBitDepth( toChannelType( compID ) );

        xT( channelBitDepth, TU::useDST( tu, compID ), resiBuf.buf, resiBuf.stride, m_plTempCoeff, uiWidth, uiHeight, sps.getMaxLog2TrDynamicRange( toChannelType( compID ) ), getEmtMode( tu, compID ), getEmtTrIdx( tu, compID ) );
      }

      if (sps.getSpsNext().getUseNSST())
      {
        xFwdNsst( tu, compID );
      }

      DTRACE_COEFF_BUF( D_TCOEFF, tempCoeff, tu, tu.cu->predMode, compID );

      xQuant( tu, compID, tempCoeff, uiAbsSum, cQP, ctx );

      DTRACE_COEFF_BUF( D_TCOEFF, tu.getCoeffs( compID ), tu, tu.cu->predMode, compID );
    }
  }

  // set coded block flag (CBF)
  TU::setCbfAtDepth (tu, compID, tu.depth, uiAbsSum > 0);
}

Void TrQuant::applyForwardRDPCM(TransformUnit &tu, const ComponentID &compID, const QpParam &cQP, TCoeff &uiAbsSum, const RDPCMMode &mode)
{
  const Bool bLossless      = tu.cu->transQuantBypass;
  const UInt uiWidth        = tu.blocks[compID].width;
  const UInt uiHeight       = tu.blocks[compID].height;
  const Bool rotateResidual = TU::isNonTransformedResidualRotated(tu, compID);
  const UInt uiSizeMinus1   = (uiWidth * uiHeight) - 1;

  const CPelBuf pcResidual  = tu.cs->getResiBuf(tu.blocks[compID]);
  const CoeffBuf pcCoeff    = tu.getCoeffs(compID);

  UInt uiX = 0;
  UInt uiY = 0;

  UInt &majorAxis            = (mode == RDPCM_VER) ? uiX      : uiY;
  UInt &minorAxis            = (mode == RDPCM_VER) ? uiY      : uiX;
  const UInt  majorAxisLimit = (mode == RDPCM_VER) ? uiWidth  : uiHeight;
  const UInt  minorAxisLimit = (mode == RDPCM_VER) ? uiHeight : uiWidth;

  const Bool bUseHalfRoundingPoint = (mode != RDPCM_OFF);

  uiAbsSum = 0;

  for (majorAxis = 0; majorAxis < majorAxisLimit; majorAxis++)
  {
    TCoeff accumulatorValue = 0; // 32-bit accumulator

    for (minorAxis = 0; minorAxis < minorAxisLimit; minorAxis++)
    {
      const UInt sampleIndex        = (uiY * uiWidth) + uiX;
      const UInt coefficientIndex   = (rotateResidual ? (uiSizeMinus1-sampleIndex) : sampleIndex);
      const Pel  currentSample      = pcResidual.at(uiX, uiY);
      const TCoeff encoderSideDelta = TCoeff(currentSample) - accumulatorValue;

      Pel reconstructedDelta;

      if (bLossless)
      {
        pcCoeff.buf[coefficientIndex] = encoderSideDelta;
        reconstructedDelta            = (Pel) encoderSideDelta;
      }
      else
      {
        transformSkipQuantOneSample(tu, compID, encoderSideDelta, pcCoeff.buf[coefficientIndex],   coefficientIndex, cQP, bUseHalfRoundingPoint);
        invTrSkipDeQuantOneSample  (tu, compID, pcCoeff.buf[coefficientIndex], reconstructedDelta, coefficientIndex, cQP);
      }

      uiAbsSum += abs(pcCoeff.buf[coefficientIndex]);

      if (mode != RDPCM_OFF)
      {
        accumulatorValue += reconstructedDelta;
      }
    }
  }
}

Void TrQuant::rdpcmNxN(TransformUnit &tu, const ComponentID &compID, const QpParam &cQP, TCoeff &uiAbsSum, RDPCMMode &rdpcmMode)
{
  if (!CU::isRDPCMEnabled(*tu.cu) || (!tu.transformSkip[compID] && !tu.cu->transQuantBypass))
  {
    rdpcmMode = RDPCM_OFF;
  }
  else if (CU::isIntra(*tu.cu))
  {
    const ChannelType chType = toChannelType(compID);
    const UInt uiChFinalMode = PU::getFinalIntraMode(*tu.cs->getPU(tu.blocks[compID].pos(), chType), chType);

    if (uiChFinalMode == VER_IDX || uiChFinalMode == HOR_IDX)
    {
      rdpcmMode = (uiChFinalMode == VER_IDX) ? RDPCM_VER : RDPCM_HOR;

      applyForwardRDPCM(tu, compID, cQP, uiAbsSum, rdpcmMode);
    }
    else
    {
      rdpcmMode = RDPCM_OFF;
    }
  }
  else // not intra, need to select the best mode
  {
    const CompArea &area = tu.blocks[compID];
    const UInt uiWidth   = area.width;
    const UInt uiHeight  = area.height;

    RDPCMMode bestMode = NUMBER_OF_RDPCM_MODES;
    TCoeff    bestAbsSum = std::numeric_limits<TCoeff>::max();
    TCoeff    bestCoefficients[MAX_TU_SIZE * MAX_TU_SIZE];

    for (UInt modeIndex = 0; modeIndex < NUMBER_OF_RDPCM_MODES; modeIndex++)
    {
      const RDPCMMode mode = RDPCMMode(modeIndex);

      TCoeff currAbsSum = 0;

      applyForwardRDPCM(tu, compID, cQP, uiAbsSum, rdpcmMode);

      if (currAbsSum < bestAbsSum)
      {
        bestMode = mode;
        bestAbsSum = currAbsSum;

        if (mode != RDPCM_OFF)
        {
          CoeffBuf(bestCoefficients, uiWidth, uiHeight).copyFrom(tu.getCoeffs(compID));
        }
      }
    }

    rdpcmMode = bestMode;
    uiAbsSum = bestAbsSum;

    if (rdpcmMode != RDPCM_OFF) //the TU is re-transformed and quantized if DPCM_OFF is returned, so there is no need to preserve it here
    {
      tu.getCoeffs(compID).copyFrom(CoeffBuf(bestCoefficients, uiWidth, uiHeight));
    }
  }

  tu.rdpcm[compID] = rdpcmMode;
}

Void TrQuant::xTransformSkip(const TransformUnit &tu, const ComponentID &compID, const CPelBuf &resi, TCoeff* psCoeff)
{
  const SPS &sps            = *tu.cs->sps;
  const CompArea &rect      = tu.blocks[compID];
  const UInt width          = rect.width;
  const UInt height         = rect.height;
  const ChannelType chType  = toChannelType(compID);
  const Int channelBitDepth = sps.getBitDepth(chType);
  const Int maxLog2TrDynamicRange = sps.getMaxLog2TrDynamicRange(chType);
  Int iTransformShift       = getTransformShift(channelBitDepth, rect.size(), maxLog2TrDynamicRange);

  if( sps.getSpsRangeExtension().getExtendedPrecisionProcessingFlag() )
  {
    iTransformShift = std::max<Int>( 0, iTransformShift );
  }

  Int iWHScale = 1;
  if( needsBlockSizeQuantScale( rect ) )
  {
    iTransformShift -= ADJ_DEQUANT_SHIFT;
    iWHScale = 181;
  }

  const Bool rotateResidual = TU::isNonTransformedResidualRotated( tu, compID );
  const UInt uiSizeMinus1 = ( width * height ) - 1;

  if( iTransformShift >= 0 )
  {
    for( UInt y = 0, coefficientIndex = 0; y < height; y++ )
    {
      for( UInt x = 0; x < width; x++, coefficientIndex++ )
      {
        psCoeff[rotateResidual ? uiSizeMinus1 - coefficientIndex : coefficientIndex] = ( TCoeff( resi.at( x, y ) ) * iWHScale ) << iTransformShift;
      }
    }
  }
  else //for very high bit depths
  {
    iTransformShift = -iTransformShift;
    const TCoeff offset = 1 << ( iTransformShift - 1 );

    for( UInt y = 0, coefficientIndex = 0; y < height; y++ )
    {
      for( UInt x = 0; x < width; x++, coefficientIndex++ )
      {
        psCoeff[rotateResidual ? uiSizeMinus1 - coefficientIndex : coefficientIndex] = ( TCoeff( resi.at( x, y ) ) * iWHScale + offset ) >> iTransformShift;
      }
    }
  }
}



Void TrQuant::xRateDistOptQuant(TransformUnit &tu, const ComponentID &compID, const CCoeffBuf &pSrc, TCoeff &uiAbsSum, const QpParam &cQP, const Ctx &ctx)
{
  const FracBitsAccess& fracBits = ctx.getFracBitsAcess();

  const SPS &sps            = *tu.cs->sps;
  const PPS &pps            = *tu.cs->pps;
  const CompArea &rect      = tu.blocks[compID];
  const UInt uiWidth        = rect.width;
  const UInt uiHeight       = rect.height;
  const ChannelType chType  = toChannelType(compID);
  const Int channelBitDepth = sps.getBitDepth( chType );

  const Bool extendedPrecision     = sps.getSpsRangeExtension().getExtendedPrecisionProcessingFlag();
  const Int  maxLog2TrDynamicRange = sps.getMaxLog2TrDynamicRange(chType);

  /* for 422 chroma blocks, the effective scaling applied during transformation is not a power of 2, hence it cannot be
  * implemented as a bit-shift (the quantised result will be sqrt(2) * larger than required). Alternatively, adjust the
  * uiLog2TrSize applied in iTransformShift, such that the result is 1/sqrt(2) the required result (i.e. smaller)
  * Then a QP+3 (sqrt(2)) or QP-3 (1/sqrt(2)) method could be used to get the required result
  */

  // Represents scaling through forward transform
  Int iTransformShift = getTransformShift(channelBitDepth, rect.size(), maxLog2TrDynamicRange);

  if (tu.transformSkip[compID] && extendedPrecision)
  {
    iTransformShift = std::max<Int>(0, iTransformShift);
  }

  const Bool bUseGolombRiceParameterAdaptation = sps.getSpsRangeExtension().getPersistentRiceAdaptationEnabledFlag();
  const UInt initialGolombRiceParameter        = ctx.getGRAdaptStats(TU::getGolombRiceStatisticsIndex(tu, compID)) >> 2;
  UInt uiGoRiceParam                           = initialGolombRiceParameter;
  Double     d64BlockUncodedCost               = 0;
  const UInt uiLog2BlockWidth                  = g_aucLog2[uiWidth];
  const UInt uiLog2BlockHeight                 = g_aucLog2[uiHeight];
  const UInt uiMaxNumCoeff                     = rect.area();

  CHECK(compID >= MAX_NUM_TBLOCKS, "Invalid component ID");

  Int scalingListType = getScalingListType(tu.cu->predMode, compID);

  CHECK(scalingListType >= SCALING_LIST_NUM, "Invalid scaling list");

  const TCoeff *plSrcCoeff = pSrc.buf;
        TCoeff *piDstCoeff = tu.getCoeffs(compID).buf;

  Double *pdCostCoeff  = m_pdCostCoeff;
  Double *pdCostSig    = m_pdCostSig;
  Double *pdCostCoeff0 = m_pdCostCoeff0;
  Int    *rateIncUp    = m_rateIncUp;
  Int    *rateIncDown  = m_rateIncDown;
  Int    *sigRateDelta = m_sigRateDelta;
  TCoeff *deltaU       = m_deltaU;

  memset( m_pdCostCoeff,  0, sizeof( Double ) *  uiMaxNumCoeff );
  memset( m_pdCostSig,    0, sizeof( Double ) *  uiMaxNumCoeff );
  memset( m_rateIncUp,    0, sizeof( Int    ) *  uiMaxNumCoeff );
  memset( m_rateIncDown,  0, sizeof( Int    ) *  uiMaxNumCoeff );
  memset( m_sigRateDelta, 0, sizeof( Int    ) *  uiMaxNumCoeff );
  memset( m_deltaU,       0, sizeof( TCoeff ) *  uiMaxNumCoeff );

  const unsigned altResiCompId = tu.cs->sps->getSpsNext().getAltResiCompId();

  const Int iQBits = QUANT_SHIFT + cQP.per + iTransformShift;                   // Right shift of non-RDOQ quantizer;  level = (coeff*uiQ + offset)>>q_bits
  const Double *const pdErrScale = getErrScaleCoeff(scalingListType, (uiLog2BlockWidth-1), (uiLog2BlockHeight-1), cQP.rem);
  const Int    *const piQCoef    = getQuantCoeff(scalingListType, cQP.rem, (uiLog2BlockWidth-1), (uiLog2BlockHeight-1));

  const Bool   enableScalingLists             = getUseScalingList(uiWidth, uiHeight, tu.transformSkip[compID]);
  const Int    defaultQuantisationCoefficient = ( needsSqrt2Scale( rect ) ? ( g_quantScales[cQP.rem] * 181 ) >> 7 : g_quantScales[cQP.rem] );
  const Double defaultErrorScale              = getErrScaleCoeffNoScalingList(scalingListType, (uiLog2BlockWidth-1), (uiLog2BlockHeight-1), cQP.rem);

  const TCoeff entropyCodingMinimum = -(1 << maxLog2TrDynamicRange);
  const TCoeff entropyCodingMaximum =  (1 << maxLog2TrDynamicRange) - 1;

  CoeffCodingContext cctx(tu, compID, pps.getSignDataHidingEnabledFlag());
  const Int    iCGSizeM1 = (1 << cctx.log2CGSize()) - 1;

  Int iCGLastScanPos = -1;
  Int     c1                  = 1;
  Int     c2                  = 0;
  Double  d64BaseCost         = 0;
  Int     iLastScanPos        = -1;

  UInt    c1Idx     = 0;
  UInt    c2Idx     = 0;
  Int     baseLevel;

  Double *pdCostCoeffGroupSig = m_pdCostCoeffGroupSig;
  memset( pdCostCoeffGroupSig, 0, ( uiMaxNumCoeff >> cctx.log2CGSize() ) * sizeof( Double ) );

  const int iCGNum  = uiWidth * uiHeight >> cctx.log2CGSize();
  Int iScanPos;
  coeffGroupRDStats rdStats;

  for (int subSetId = iCGNum - 1; subSetId >= 0; subSetId--)
  {
    cctx.initSubblock( subSetId );

    memset( &rdStats, 0, sizeof (coeffGroupRDStats));

    for (Int iScanPosinCG = iCGSizeM1; iScanPosinCG >= 0; iScanPosinCG--)
    {
      iScanPos = cctx.minSubPos() + iScanPosinCG;
      //===== quantization =====
      UInt    uiBlkPos          = cctx.blockPos(iScanPos);
      // set coeff

      const Int    quantisationCoefficient = (enableScalingLists) ? piQCoef   [uiBlkPos]               : defaultQuantisationCoefficient;
      const Double errorScale              = (enableScalingLists) ? pdErrScale[uiBlkPos]               : defaultErrorScale;

      const Int64  tmpLevel                = Int64(abs(plSrcCoeff[ uiBlkPos ])) * quantisationCoefficient;

      const Intermediate_Int lLevelDouble  = (Intermediate_Int)std::min<Int64>(tmpLevel, std::numeric_limits<Intermediate_Int>::max() - (Intermediate_Int(1) << (iQBits - 1)));

      UInt uiMaxAbsLevel        = std::min<UInt>(UInt(entropyCodingMaximum), UInt((lLevelDouble + (Intermediate_Int(1) << (iQBits - 1))) >> iQBits));

      const Double dErr         = Double( lLevelDouble );
      pdCostCoeff0[ iScanPos ]  = dErr * dErr * errorScale;
      d64BlockUncodedCost      += pdCostCoeff0[ iScanPos ];
      piDstCoeff[ uiBlkPos ]    = uiMaxAbsLevel;

      if ( uiMaxAbsLevel > 0 && iLastScanPos < 0 )
      {
        iLastScanPos            = iScanPos;
        iCGLastScanPos          = cctx.subSetId();
      }

      if ( iLastScanPos >= 0 )
      {
        //===== coefficient level estimation =====
        unsigned ctxIdSig = 0;
        if( iScanPos != iLastScanPos )
        {
          ctxIdSig = altResiCompId ? cctx.sigCtxId( iScanPos, piDstCoeff ) : cctx.sigCtxId( iScanPos );
        }
        UInt  uiLevel;
        const BinFracBits fracBitsOne = fracBits.getFracBitsArray( altResiCompId ? ( iScanPos == iLastScanPos ? cctx.greater1CtxIdOfs() : cctx.greater1CtxId( iScanPos, piDstCoeff ) ) : cctx.greater1CtxId( c1 ) );
        const BinFracBits fracBitsAbs = fracBits.getFracBitsArray( altResiCompId ? ( iScanPos == iLastScanPos ? cctx.greater1CtxIdOfs() : cctx.greater2CtxId( iScanPos, piDstCoeff ) ) : cctx.greater2CtxId()     );
                        uiGoRiceParam = altResiCompId ? cctx.GoRicePar( iScanPos, piDstCoeff ) : uiGoRiceParam;
        if( iScanPos == iLastScanPos )
        {
          uiLevel              = xGetCodedLevel( pdCostCoeff[ iScanPos ], pdCostCoeff0[ iScanPos ], pdCostSig[ iScanPos ],
                                                 lLevelDouble, uiMaxAbsLevel, nullptr, fracBitsOne, fracBitsAbs, altResiCompId, uiGoRiceParam,
                                                 c1Idx, c2Idx, iQBits, errorScale, 1, extendedPrecision, maxLog2TrDynamicRange );
        }
        else
        {
          const BinFracBits fracBitsSig = fracBits.getFracBitsArray( ctxIdSig );
          uiLevel              = xGetCodedLevel( pdCostCoeff[ iScanPos ], pdCostCoeff0[ iScanPos ], pdCostSig[ iScanPos ],
                                                lLevelDouble, uiMaxAbsLevel, &fracBitsSig, fracBitsOne, fracBitsAbs, altResiCompId, uiGoRiceParam,
                                                c1Idx, c2Idx, iQBits, errorScale, 0, extendedPrecision, maxLog2TrDynamicRange );
          sigRateDelta[ uiBlkPos ] = fracBitsSig.intBits[1] - fracBitsSig.intBits[0];
        }

        deltaU[ uiBlkPos ]        = TCoeff((lLevelDouble - (Intermediate_Int(uiLevel) << iQBits)) >> (iQBits-8));

        if( uiLevel > 0 )
        {
          Int rateNow              = xGetICRate( uiLevel,   fracBitsOne, fracBitsAbs, altResiCompId, uiGoRiceParam, c1Idx, c2Idx, extendedPrecision, maxLog2TrDynamicRange );
          rateIncUp   [ uiBlkPos ] = xGetICRate( uiLevel+1, fracBitsOne, fracBitsAbs, altResiCompId, uiGoRiceParam, c1Idx, c2Idx, extendedPrecision, maxLog2TrDynamicRange ) - rateNow;
          rateIncDown [ uiBlkPos ] = xGetICRate( uiLevel-1, fracBitsOne, fracBitsAbs, altResiCompId, uiGoRiceParam, c1Idx, c2Idx, extendedPrecision, maxLog2TrDynamicRange ) - rateNow;
        }
        else // uiLevel == 0
        {
          rateIncUp   [ uiBlkPos ] = fracBitsOne.intBits[ 0 ];
        }
        piDstCoeff[ uiBlkPos ] = uiLevel;
        d64BaseCost           += pdCostCoeff [ iScanPos ];

        baseLevel = (c1Idx < C1FLAG_NUMBER) ? (2 + (c2Idx < C2FLAG_NUMBER)) : 1;
        if( uiLevel >= baseLevel )
        {
          if (uiLevel > 3*(1<<uiGoRiceParam))
          {
            uiGoRiceParam = bUseGolombRiceParameterAdaptation ? (uiGoRiceParam + 1) : (std::min<UInt>((uiGoRiceParam + 1), 4));
          }
        }
        if ( uiLevel >= 1)
        {
          c1Idx ++;
        }

        //===== update bin model =====
        if( uiLevel > 1 )
        {
          c1 = 0;
          c2 += (c2 < 2);
          c2Idx ++;
        }
        else if( (c1 < 3) && (c1 > 0) && uiLevel)
        {
          c1++;
        }

        //===== context set update =====
        if( ( (iScanPos & iCGSizeM1) == 0 ) && ( iScanPos > 0 ) )
        {
          cctx.setGt2Flag( c1 == 0 );
          c1                = 1;
          c2                = 0;
          c1Idx             = 0;
          c2Idx             = 0;
          uiGoRiceParam     = initialGolombRiceParameter;
        }
      }
      else
      {
        d64BaseCost    += pdCostCoeff0[ iScanPos ];
      }
      rdStats.d64SigCost += pdCostSig[ iScanPos ];
      if (iScanPosinCG == 0 )
      {
        rdStats.d64SigCost_0 = pdCostSig[ iScanPos ];
      }
      if (piDstCoeff[ uiBlkPos ] )
      {
        cctx.setSigGroup();
        rdStats.d64CodedLevelandDist += pdCostCoeff[ iScanPos ] - pdCostSig[ iScanPos ];
        rdStats.d64UncodedDist += pdCostCoeff0[ iScanPos ];
        if ( iScanPosinCG != 0 )
        {
          rdStats.iNNZbeforePos0++;
        }
      }
    } //end for (iScanPosinCG)

    if (iCGLastScanPos >= 0)
    {
      if( cctx.subSetId() )
      {
        if( !cctx.isSigGroup() )
        {
          const BinFracBits fracBitsSigGroup = fracBits.getFracBitsArray( cctx.sigGroupCtxId() );
          d64BaseCost += xGetRateSigCoeffGroup(fracBitsSigGroup, 0) - rdStats.d64SigCost;
          pdCostCoeffGroupSig[ cctx.subSetId() ] = xGetRateSigCoeffGroup(fracBitsSigGroup, 0);
        }
        else
        {
          if (cctx.subSetId() < iCGLastScanPos) //skip the last coefficient group, which will be handled together with last position below.
          {
            if ( rdStats.iNNZbeforePos0 == 0 )
            {
              d64BaseCost -= rdStats.d64SigCost_0;
              rdStats.d64SigCost -= rdStats.d64SigCost_0;
            }
            // rd-cost if SigCoeffGroupFlag = 0, initialization
            Double d64CostZeroCG = d64BaseCost;

            const BinFracBits fracBitsSigGroup = fracBits.getFracBitsArray( cctx.sigGroupCtxId() );

            if (cctx.subSetId() < iCGLastScanPos)
            {
              d64BaseCost  += xGetRateSigCoeffGroup(fracBitsSigGroup,1);
              d64CostZeroCG += xGetRateSigCoeffGroup(fracBitsSigGroup,0);
              pdCostCoeffGroupSig[ cctx.subSetId() ] = xGetRateSigCoeffGroup(fracBitsSigGroup,1);
            }

            // try to convert the current coeff group from non-zero to all-zero
            d64CostZeroCG += rdStats.d64UncodedDist;  // distortion for resetting non-zero levels to zero levels
            d64CostZeroCG -= rdStats.d64CodedLevelandDist;   // distortion and level cost for keeping all non-zero levels
            d64CostZeroCG -= rdStats.d64SigCost;     // sig cost for all coeffs, including zero levels and non-zerl levels

                                                     // if we can save cost, change this block to all-zero block
            if ( d64CostZeroCG < d64BaseCost )
            {
              cctx.resetSigGroup();
              d64BaseCost = d64CostZeroCG;
              if (cctx.subSetId() < iCGLastScanPos)
              {
                pdCostCoeffGroupSig[ cctx.subSetId() ] = xGetRateSigCoeffGroup(fracBitsSigGroup,0);
              }
              // reset coeffs to 0 in this block
              for (Int iScanPosinCG = iCGSizeM1; iScanPosinCG >= 0; iScanPosinCG--)
              {
                iScanPos      = cctx.minSubPos() + iScanPosinCG;
                UInt uiBlkPos = cctx.blockPos( iScanPos );

                if (piDstCoeff[ uiBlkPos ])
                {
                  piDstCoeff [ uiBlkPos ] = 0;
                  pdCostCoeff[ iScanPos ] = pdCostCoeff0[ iScanPos ];
                  pdCostSig  [ iScanPos ] = 0;
                }
              }
            } // end if ( d64CostAllZeros < d64BaseCost )
          }
        } // end if if (uiSigCoeffGroupFlag[ uiCGBlkPos ] == 0)
      }
      else
      {
        cctx.setSigGroup();
      }
    }
  } //end for (cctx.subSetId)

  //===== estimate last position =====
  if ( iLastScanPos < 0 )
  {
    return;
  }

  Double  d64BestCost         = 0;
  Int     iBestLastIdxP1      = 0;

  if( !CU::isIntra(*tu.cu) && isLuma(compID) && tu.depth == 0 )
  {
    const BinFracBits fracBitsQtRootCbf = fracBits.getFracBitsArray( Ctx::QtRootCbf() );
    d64BestCost  = d64BlockUncodedCost + xGetICost( fracBitsQtRootCbf.intBits[ 0 ] );
    d64BaseCost += xGetICost( fracBitsQtRootCbf.intBits[ 1 ] );
  }
  else
  {
    const BinFracBits fracBitsQtCbf = fracBits.getFracBitsArray( Ctx::QtCbf[chType](DeriveCtx::CtxQtCbf(rect.compID, tu.depth)) );
    d64BestCost  = d64BlockUncodedCost + xGetICost( fracBitsQtCbf.intBits[ 0 ] );
    d64BaseCost += xGetICost( fracBitsQtCbf.intBits[ 1 ] );
  }

  Int lastBitsX[LAST_SIGNIFICANT_GROUPS] = { 0 };
  Int lastBitsY[LAST_SIGNIFICANT_GROUPS] = { 0 };
  {
    int dim1  = ( cctx.scanType() == SCAN_VER ? uiHeight : uiWidth  );
    int dim2  = ( cctx.scanType() == SCAN_VER ? uiWidth  : uiHeight );
    int bitsX = 0;
    int bitsY = 0;
    int ctxId;
    //X-coordinate
    for ( ctxId = 0; ctxId < g_uiGroupIdx[dim1-1]; ctxId++)
    {
      const BinFracBits fB = fracBits.getFracBitsArray( cctx.lastXCtxId(ctxId) );
      lastBitsX[ ctxId ]   = bitsX + fB.intBits[ 0 ];
      bitsX               +=         fB.intBits[ 1 ];
    }
    lastBitsX[ctxId] = bitsX;
    //Y-coordinate
    for ( ctxId = 0; ctxId < g_uiGroupIdx[dim2-1]; ctxId++)
    {
      const BinFracBits fB = fracBits.getFracBitsArray( cctx.lastYCtxId(ctxId) );
      lastBitsY[ ctxId ]   = bitsY + fB.intBits[ 0 ];
      bitsY               +=         fB.intBits[ 1 ];
    }
    lastBitsY[ctxId] = bitsY;
  }


  Bool bFoundLast = false;
  for (Int iCGScanPos = iCGLastScanPos; iCGScanPos >= 0; iCGScanPos--)
  {
    d64BaseCost -= pdCostCoeffGroupSig [ iCGScanPos ];
    if (cctx.isSigGroup( iCGScanPos ) )
    {
      for (Int iScanPosinCG = iCGSizeM1; iScanPosinCG >= 0; iScanPosinCG--)
      {
        iScanPos = iCGScanPos * (iCGSizeM1 + 1) + iScanPosinCG;

        if (iScanPos > iLastScanPos)
        {
          continue;
        }
        UInt   uiBlkPos     = cctx.blockPos( iScanPos );

        if( piDstCoeff[ uiBlkPos ] )
        {
          // UInt   uiPosY       = cctx.posY( iScanPos );
          // UInt   uiPosX       = cctx.posX( iScanPos );
          UInt   uiPosY = uiBlkPos >> uiLog2BlockWidth;
          UInt   uiPosX = uiBlkPos - ( uiPosY << uiLog2BlockWidth );
          Double d64CostLast  = ( cctx.scanType() == SCAN_VER ? xGetRateLast( lastBitsX, lastBitsY, uiPosY, uiPosX ) : xGetRateLast( lastBitsX, lastBitsY, uiPosX, uiPosY ) );
          Double totalCost = d64BaseCost + d64CostLast - pdCostSig[ iScanPos ];

          if( totalCost < d64BestCost )
          {
            iBestLastIdxP1  = iScanPos + 1;
            d64BestCost     = totalCost;
          }
          if( piDstCoeff[ uiBlkPos ] > 1 )
          {
            bFoundLast = true;
            break;
          }
          d64BaseCost      -= pdCostCoeff[ iScanPos ];
          d64BaseCost      += pdCostCoeff0[ iScanPos ];
        }
        else
        {
          d64BaseCost      -= pdCostSig[ iScanPos ];
        }
      } //end for
      if (bFoundLast)
      {
        break;
      }
    } // end if (uiSigCoeffGroupFlag[ uiCGBlkPos ])
  } // end for


  for ( Int scanPos = 0; scanPos < iBestLastIdxP1; scanPos++ )
  {
    Int blkPos = cctx.blockPos( scanPos );
    TCoeff level = piDstCoeff[ blkPos ];
    uiAbsSum += level;
    piDstCoeff[ blkPos ] = ( plSrcCoeff[ blkPos ] < 0 ) ? -level : level;
  }

  //===== clean uncoded coefficients =====
  for ( Int scanPos = iBestLastIdxP1; scanPos <= iLastScanPos; scanPos++ )
  {
    piDstCoeff[ cctx.blockPos( scanPos ) ] = 0;
  }

  if( cctx.signHiding() && uiAbsSum>=2)
  {
    const Double inverseQuantScale = Double(g_invQuantScales[cQP.rem]);
    Int64 rdFactor = (Int64)(inverseQuantScale * inverseQuantScale * (1 << (2 * cQP.per))
                             / m_dLambda / 16 / (1 << (2 * DISTORTION_PRECISION_ADJUSTMENT(channelBitDepth - 8)))
                             + 0.5);

    Int lastCG = -1;
    Int absSum = 0 ;
    Int n ;

    for( Int subSet = (uiWidth*uiHeight-1) >> cctx.log2CGSize(); subSet >= 0; subSet-- )
    {
      Int  subPos         = subSet << cctx.log2CGSize();
      Int  firstNZPosInCG = iCGSizeM1 + 1, lastNZPosInCG = -1;
      absSum = 0 ;

      for( n = iCGSizeM1; n >= 0; --n )
      {
        if( piDstCoeff[ cctx.blockPos( n + subPos )] )
        {
          lastNZPosInCG = n;
          break;
        }
      }

      for( n = 0; n <= iCGSizeM1; n++ )
      {
        if( piDstCoeff[ cctx.blockPos( n + subPos )] )
        {
          firstNZPosInCG = n;
          break;
        }
      }

      for( n = firstNZPosInCG; n <= lastNZPosInCG; n++ )
      {
        absSum += Int(piDstCoeff[ cctx.blockPos( n + subPos )]);
      }

      if(lastNZPosInCG>=0 && lastCG==-1)
      {
        lastCG = 1;
      }

      if( lastNZPosInCG-firstNZPosInCG>=SBH_THRESHOLD )
      {
        UInt signbit = (piDstCoeff[cctx.blockPos(subPos+firstNZPosInCG)]>0?0:1);
        if( signbit!=(absSum&0x1) )  // hide but need tune
        {
          // calculate the cost
          Int64 minCostInc = std::numeric_limits<Int64>::max(), curCost = std::numeric_limits<Int64>::max();
          Int minPos = -1, finalChange = 0, curChange = 0;

          for( n = (lastCG == 1 ? lastNZPosInCG : iCGSizeM1); n >= 0; --n )
          {
            UInt uiBlkPos   = cctx.blockPos( n + subPos );
            if(piDstCoeff[ uiBlkPos ] != 0 )
            {
              Int64 costUp   = rdFactor * ( - deltaU[uiBlkPos] ) + rateIncUp[uiBlkPos];
              Int64 costDown = rdFactor * (   deltaU[uiBlkPos] ) + rateIncDown[uiBlkPos]
                -   ((abs(piDstCoeff[uiBlkPos]) == 1) ? sigRateDelta[uiBlkPos] : 0);

              if(lastCG==1 && lastNZPosInCG==n && abs(piDstCoeff[uiBlkPos])==1)
              {
                costDown -= (4<<SCALE_BITS);
              }

              if(costUp<costDown)
              {
                curCost = costUp;
                curChange =  1;
              }
              else
              {
                curChange = -1;
                if(n==firstNZPosInCG && abs(piDstCoeff[uiBlkPos])==1)
                {
                  curCost = std::numeric_limits<Int64>::max();
                }
                else
                {
                  curCost = costDown;
                }
              }
            }
            else
            {
              curCost = rdFactor * ( - (abs(deltaU[uiBlkPos])) ) + (1<<SCALE_BITS) + rateIncUp[uiBlkPos] + sigRateDelta[uiBlkPos] ;
              curChange = 1 ;

              if(n<firstNZPosInCG)
              {
                UInt thissignbit = (plSrcCoeff[uiBlkPos]>=0?0:1);
                if(thissignbit != signbit )
                {
                  curCost = std::numeric_limits<Int64>::max();
                }
              }
            }

            if( curCost<minCostInc)
            {
              minCostInc = curCost;
              finalChange = curChange;
              minPos = uiBlkPos;
            }
          }

          if(piDstCoeff[minPos] == entropyCodingMaximum || piDstCoeff[minPos] == entropyCodingMinimum)
          {
            finalChange = -1;
          }

          if(plSrcCoeff[minPos]>=0)
          {
            piDstCoeff[minPos] += finalChange ;
          }
          else
          {
            piDstCoeff[minPos] -= finalChange ;
          }
        }
      }

      if(lastCG==1)
      {
        lastCG=0 ;
      }
    }
  }
}

Void TrQuant::transformSkipQuantOneSample(TransformUnit &tu, const ComponentID &compID, const TCoeff &resiDiff, TCoeff &coeff, const UInt &uiPos, const QpParam &cQP, const Bool bUseHalfRoundingPoint)
{
  const SPS           &sps = *tu.cs->sps;
  const CompArea      &rect                           = tu.blocks[compID];
  const UInt           uiWidth                        = rect.width;
  const UInt           uiHeight                       = rect.height;
  const Int            maxLog2TrDynamicRange          = sps.getMaxLog2TrDynamicRange(toChannelType(compID));
  const Int            channelBitDepth                = sps.getBitDepth(toChannelType(compID));
  const Int            iTransformShift                = getTransformShift(channelBitDepth, rect.size(), maxLog2TrDynamicRange);
  const Int            scalingListType                = getScalingListType(tu.cu->predMode, compID);
  const Bool           enableScalingLists             = getUseScalingList(uiWidth, uiHeight, true);
  const Int            defaultQuantisationCoefficient = g_quantScales[cQP.rem];

  CHECK( scalingListType >= SCALING_LIST_NUM, "Invalid scaling list" );

  const UInt uiLog2TrWidth  = g_aucLog2[uiWidth];
  const UInt uiLog2TrHeight = g_aucLog2[uiHeight];
  const Int *const piQuantCoeff = getQuantCoeff(scalingListType, cQP.rem, uiLog2TrWidth-1, uiLog2TrHeight-1);

  /* for 422 chroma blocks, the effective scaling applied during transformation is not a power of 2, hence it cannot be
  * implemented as a bit-shift (the quantised result will be sqrt(2) * larger than required). Alternatively, adjust the
  * uiLog2TrSize applied in iTransformShift, such that the result is 1/sqrt(2) the required result (i.e. smaller)
  * Then a QP+3 (sqrt(2)) or QP-3 (1/sqrt(2)) method could be used to get the required result
  */

  const Int iQBits = QUANT_SHIFT + cQP.per + iTransformShift;
  // QBits will be OK for any internal bit depth as the reduction in transform shift is balanced by an increase in Qp_per due to QpBDOffset

  const Int iAdd = Int64(bUseHalfRoundingPoint ? 256 : (tu.cs->slice->getSliceType() == I_SLICE ? 171 : 85)) << Int64(iQBits - 9);

  TCoeff transformedCoefficient;

  // transform-skip
  if (iTransformShift >= 0)
  {
    transformedCoefficient = resiDiff << iTransformShift;
  }
  else // for very high bit depths
  {
    const Int iTrShiftNeg  = -iTransformShift;
    const Int offset       = 1 << (iTrShiftNeg - 1);
    transformedCoefficient = ( resiDiff + offset ) >> iTrShiftNeg;
  }

  // quantization
  const TCoeff iSign = (transformedCoefficient < 0 ? -1: 1);

  const Int quantisationCoefficient = enableScalingLists ? piQuantCoeff[uiPos] : defaultQuantisationCoefficient;

  const Int64 tmpLevel = (Int64)abs(transformedCoefficient) * quantisationCoefficient;

  const TCoeff quantisedCoefficient = (TCoeff((tmpLevel + iAdd ) >> iQBits)) * iSign;

  const TCoeff entropyCodingMinimum = -(1 << maxLog2TrDynamicRange);
  const TCoeff entropyCodingMaximum =  (1 << maxLog2TrDynamicRange) - 1;
  coeff = Clip3<TCoeff>( entropyCodingMinimum, entropyCodingMaximum, quantisedCoefficient );
}

Void TrQuant::invTrSkipDeQuantOneSample(TransformUnit &tu, const ComponentID &compID, const TCoeff &inSample, Pel &reconSample, const UInt &uiPos, const QpParam &cQP)
{
  const SPS           &sps                    = *tu.cs->sps;
  const CompArea      &rect                   = tu.blocks[compID];
  const UInt           uiWidth                = rect.width;
  const UInt           uiHeight               = rect.height;
  const Int            QP_per                 = cQP.per;
  const Int            QP_rem                 = cQP.rem;
  const Int            maxLog2TrDynamicRange  = sps.getMaxLog2TrDynamicRange(toChannelType(compID));
  const Int            channelBitDepth        = sps.getBitDepth(toChannelType(compID));
  const Int            iTransformShift        = getTransformShift(channelBitDepth, rect.size(), maxLog2TrDynamicRange);
  const Int            scalingListType        = getScalingListType(tu.cu->predMode, compID);
  const Bool           enableScalingLists     = getUseScalingList(uiWidth, uiHeight, true);

  CHECK(scalingListType >= SCALING_LIST_NUM, "Invalid scaling list");

  const Int rightShift = (IQUANT_SHIFT - (iTransformShift + QP_per)) + (enableScalingLists ? LOG2_SCALING_LIST_NEUTRAL_VALUE : 0);

  const TCoeff transformMinimum = -(1 << maxLog2TrDynamicRange);
  const TCoeff transformMaximum =  (1 << maxLog2TrDynamicRange) - 1;

  // De-quantisation

  TCoeff dequantisedSample;

  if (enableScalingLists)
  {
    const UInt             dequantCoefBits = 1 + IQUANT_SHIFT + SCALING_LIST_BITS;
    const UInt             targetInputBitDepth = std::min<UInt>((maxLog2TrDynamicRange + 1), (((sizeof(Intermediate_Int) * 8) + rightShift) - dequantCoefBits));

    const Intermediate_Int inputMinimum = -(1 << (targetInputBitDepth - 1));
    const Intermediate_Int inputMaximum =  (1 << (targetInputBitDepth - 1)) - 1;

    const UInt uiLog2TrWidth  = g_aucLog2[uiWidth];
    const UInt uiLog2TrHeight = g_aucLog2[uiHeight];
    Int *piDequantCoef = getDequantCoeff(scalingListType,QP_rem,uiLog2TrWidth-1, uiLog2TrHeight-1);

    if (rightShift > 0)
    {
      const Intermediate_Int iAdd = 1 << (rightShift - 1);
      const TCoeff           clipQCoef = TCoeff(Clip3<Intermediate_Int>(inputMinimum, inputMaximum, inSample));
      const Intermediate_Int iCoeffQ = ((Intermediate_Int(clipQCoef) * piDequantCoef[uiPos]) + iAdd) >> rightShift;

      dequantisedSample = TCoeff(Clip3<Intermediate_Int>(transformMinimum, transformMaximum, iCoeffQ));
    }
    else
    {
      const Int              leftShift = -rightShift;
      const TCoeff           clipQCoef = TCoeff(Clip3<Intermediate_Int>(inputMinimum, inputMaximum, inSample));
      const Intermediate_Int iCoeffQ = (Intermediate_Int(clipQCoef) * piDequantCoef[uiPos]) << leftShift;

      dequantisedSample = TCoeff(Clip3<Intermediate_Int>(transformMinimum, transformMaximum, iCoeffQ));
    }
  }
  else
  {
    const Int scale = g_invQuantScales[QP_rem];
    const Int scaleBits = (IQUANT_SHIFT + 1);

    const UInt             targetInputBitDepth = std::min<UInt>((maxLog2TrDynamicRange + 1), (((sizeof(Intermediate_Int) * 8) + rightShift) - scaleBits));
    const Intermediate_Int inputMinimum = -(1 << (targetInputBitDepth - 1));
    const Intermediate_Int inputMaximum = (1 << (targetInputBitDepth - 1)) - 1;

    if (rightShift > 0)
    {
      const Intermediate_Int iAdd = 1 << (rightShift - 1);
      const TCoeff           clipQCoef = TCoeff(Clip3<Intermediate_Int>(inputMinimum, inputMaximum, inSample));
      const Intermediate_Int iCoeffQ = (Intermediate_Int(clipQCoef) * scale + iAdd) >> rightShift;

      dequantisedSample = TCoeff(Clip3<Intermediate_Int>(transformMinimum, transformMaximum, iCoeffQ));
    }
    else
    {
      const Int              leftShift = -rightShift;
      const TCoeff           clipQCoef = TCoeff(Clip3<Intermediate_Int>(inputMinimum, inputMaximum, inSample));
      const Intermediate_Int iCoeffQ = (Intermediate_Int(clipQCoef) * scale) << leftShift;

      dequantisedSample = TCoeff(Clip3<Intermediate_Int>(transformMinimum, transformMaximum, iCoeffQ));
    }
  }

  // Inverse transform-skip

  if (iTransformShift >= 0)
  {
    const TCoeff offset = iTransformShift == 0 ? 0 : (1 << (iTransformShift - 1));
    reconSample = Pel((dequantisedSample + offset) >> iTransformShift);
  }
  else //for very high bit depths
  {
    const Int iTrShiftNeg = -iTransformShift;
    reconSample = Pel(dequantisedSample << iTrShiftNeg);
  }
}


//! \}
