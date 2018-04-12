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

/** \file     TrQuant_NonSepBF.h
    \brief    transform and quantization class (header)
*/

#include "TrQuant_NonSepBF.h"

#include "Rom.h"
#include "IntraNNRom.h"

#include "Trafos_NN_SOT.h"


const TMatrixCoeff (*trafosMapLog2NN[4][4]) =
{
      {  &g_aiNNT4x4[0], &g_aiNNT4x8[0],  &g_aiNNT4x16[0],  &g_aiNNT4x32[0] },
      {  &g_aiNNT8x4[0], &g_aiNNT8x8[0],  &g_aiNNT8x16[0],  &g_aiNNT8x32[0] },
      { &g_aiNNT16x4[0],&g_aiNNT16x8[0], &g_aiNNT16x16[0], &g_aiNNT16x32[0] },
      { &g_aiNNT32x4[0],&g_aiNNT32x8[0], &g_aiNNT32x16[0], &g_aiNNT32x32[0] },
};



const TMatrixCoeff (*trafosMapLog[4][4]) =
{
      {  &g_aiSOTT4x4[0], &g_aiSOTT4x8[0],  &g_aiSOTT4x16[0],  &g_aiSOTT4x32[0] },
      {  &g_aiSOTT8x4[0], &g_aiSOTT8x8[0],  &g_aiSOTT8x16[0],  &g_aiSOTT8x32[0] },
      { &g_aiSOTT16x4[0],&g_aiSOTT16x8[0], &g_aiSOTT16x16[0], &g_aiSOTT16x32[0] },
      { &g_aiSOTT32x4[0],&g_aiSOTT32x8[0], &g_aiSOTT32x16[0], &g_aiSOTT32x32[0] },
};

const TMatrixCoeff (*trafosMapNonLog[2]) =
{
    &g_aiSOTT4x12[0],
    &g_aiSOTT12x4[0]
};

const TMatrixCoeff* getTrafoPointer( const TransformUnit &tu, ComponentID compID  )
{
  if(  tu.cu->intra_NN )
  {
    const int trafo_idx = tu.cu->intraNNTrafoIdx - 1;
    const int true_NN_mode     = tu.cu->cs->getPU(tu.lumaPos(), CHANNEL_TYPE_LUMA)->intraNN_Mode_True;
    const int tu_width_padded  = g_PaddedSizes[tu.lwidth()];
    const int tu_height_padded = g_PaddedSizes[tu.lheight()];
    const int width_for_trafo  = tu.cu->intra_NN_Use_Sampling ? (tu_width_padded  >> 1) : tu_width_padded;
    const int height_for_trafo = tu.cu->intra_NN_Use_Sampling ? (tu_height_padded >> 1) : tu_height_padded;

    const int idx0 = g_aucLog2[height_for_trafo] - 2;
    const int idx1 = g_aucLog2[width_for_trafo ] - 2;

    const int secTrHeight = std::min<int>( MAX_ONE_D_DIMENSION_FOR_NN_TRAFO, tu.blocks[compID].height );
    const int secTrWidth  = std::min<int>( MAX_ONE_D_DIMENSION_FOR_NN_TRAFO, tu.blocks[compID].width  );
    const int trDim       = secTrHeight * secTrWidth;
    const int trIdxInMat  = true_NN_mode * NUM_NSST_TRAFOS_PER_NN_MODE + trafo_idx;

    CHECK( idx0 > 3 || idx1 > 3, "trafo size not implemted" );
    const TMatrixCoeff* trafo  = trafosMapLog2NN[idx0][idx1] + trIdxInMat * (trDim*trDim);
    return trafo;
  }

  const int  tuHeight = tu.blocks[compID].height;
  const int  tuWidth  = tu.blocks[compID].width;

  const bool is4x12 = (tuHeight == 4 && tuWidth == 12) || (tuHeight == 12 && tuWidth == 4);
  const int  trHeight = is4x12 ? tuHeight : std::min<int>( 8, tuHeight );
  const int  trWidth  = is4x12 ? tuWidth  : std::min<int>( 8, tuWidth  );
  const bool useSecTr = !is4x12 && (trHeight != tuHeight || trWidth != tuWidth);

  const UInt trIdx = useSecTr ? TU::getNonSepSecIdx( tu, compID ) : TU::getNonSepPrimIdx( tu, compID );

  if( is4x12 )
  {
    const int idx0 = tuHeight == 4 ? 0 : 1;
    return trafosMapNonLog[idx0] + trIdx * ((trWidth*trHeight)*(trHeight*trWidth));
  }

  const int idx0 = g_aucLog2[tuHeight] - 2;
  const int idx1 = g_aucLog2[tuWidth ] - 2;
  CHECK( idx0 > 3 || idx1 > 3, "trafo size not implemted" );
  return trafosMapLog[idx0][idx1] + trIdx * ((trWidth*trHeight)*(trHeight*trWidth));
}

void xTr_NonSepBF( const TransformUnit &tu, ComponentID compID, CoeffBuf& coeffBuf, const CPelBuf& resBuf )
{

  CHECK( CU::isInter(*tu.cu), "should not happen" );
  const int maxLog2TrDynamicRange = tu.cs->sps->getMaxLog2TrDynamicRange(toChannelType(compID));
  const int channelBitDepth       = tu.cs->sps->getBitDepth(toChannelType(compID));
  const int iHeight = (int) coeffBuf.height;
  const int iWidth  = (int) coeffBuf.width;

  const int TRANSFORM_MATRIX_SHIFT = g_transformMatrixShift[TRANSFORM_FORWARD] + COM16_C806_TRANS_PREC;

  //((1 << TRANSFORM_MATRIX_SHIFT ) * sqrt(trDim));
  //( 1 << (maxLog2TrDynamicRange - channelBitDepth)) / sqrt(iWidth*iHeight);
  const int shift = ( g_aucLog2OfPowerOf2Part[iHeight] + g_aucLog2OfPowerOf2Part[iWidth] + g_aucCeilOfLog2OfNonPowerOf2Part[iHeight] + g_aucCeilOfLog2OfNonPowerOf2Part[iWidth] + TRANSFORM_MATRIX_SHIFT ) - (maxLog2TrDynamicRange - channelBitDepth);

  const int add   = ( shift > 0 ) ? ( 1 << ( shift - 1 ) ) : 0;

  CHECK( shift < 0, "Negative shift" );

  const TMatrixCoeff *iT = getTrafoPointer( tu, compID );
  const int trStride =iHeight*iWidth;

  const Pel* residual = resBuf.buf;
  const int resStride = resBuf.stride;

  TCoeff* coeff = coeffBuf.buf;

  Int64  iSum;
  unsigned i, j, k;
  for( k = 0; k < iHeight*iWidth; k++ )
  {
    iSum = 0;
    for( i = 0; i < iHeight; i++ )
    {
      for( j=0; j< iWidth; j++ )
      {
        iSum += iT       [k*trStride + i*iWidth+j] * residual[ i*resStride + j];
      }
    }
    coeff[k]    = TCoeff( ( iSum  + add ) >> shift );
  }
}


void xITr_NonSepBF( const TransformUnit &tu, ComponentID compID, const CCoeffBuf& coeffBuf, PelBuf& resBuf )
{
  CHECK( CU::isInter(*tu.cu), "should not happen" );
  const Int TRANSFORM_MATRIX_SHIFT = g_transformMatrixShift[TRANSFORM_INVERSE] + COM16_C806_TRANS_PREC;

  const int maxLog2TrDynamicRange = tu.cs->sps->getMaxLog2TrDynamicRange(toChannelType(compID));
  const int channelBitDepth       = tu.cs->sps->getBitDepth(toChannelType(compID));
  const int iHeight = (int) coeffBuf.height;
  const int iWidth  = (int) coeffBuf.width;

  const int dim   = iWidth*iHeight;

   const int shift = TRANSFORM_MATRIX_SHIFT + g_aucCeilOfLog2OfNonPowerOf2Part[iHeight] + g_aucCeilOfLog2OfNonPowerOf2Part[iWidth] + (maxLog2TrDynamicRange - channelBitDepth);

  CHECK( shift < 0, "Negative shift" );
  const int add   = 1 << ( shift - 1 );

  const TCoeff clipMinimum         = -( 1 << maxLog2TrDynamicRange );
  const TCoeff clipMaximum         =  ( 1 << maxLog2TrDynamicRange ) - 1;

  const TMatrixCoeff *iT = getTrafoPointer( tu, compID );
  const int trStride = dim;

  const TCoeff* coeff = coeffBuf.buf;

  Pel*      residual  = resBuf.buf;
  const int resStride = resBuf.stride;

  Int64  iSum;
  unsigned i, j, k;
  for( i = 0; i < iHeight; i++ )
  {
    for( j=0; j< iWidth; j++ )
    {
      iSum = 0;
      for( k = 0; k < iHeight*iWidth; k++ )
      {
        iSum +=        iT[ k *trStride + i*iWidth+j] * coeff[k];
      }
      residual[i*resStride+j] = (Pel) Clip3<Int64>( clipMinimum, clipMaximum, (iSum + add) >> shift );
    }
  }
}


void xTrSec_NonSepBF( const TransformUnit &tu, ComponentID compID, CoeffBuf& coeffBuf, CoeffBuf& tmpBuf  )
{
  const int tuHeight = (int) coeffBuf.height;
  const int tuWidth  = (int) coeffBuf.width;

  const int secTrHeight = std::min<int>( 8, tuHeight );
  const int secTrWidth  = std::min<int>( 8, tuWidth  );

  const int dim   = secTrWidth*secTrHeight;
  CHECK( dim > 64, "secondary trafo only supported up to dim 64" );

  const int inpShift = 5;
  const int TRANSFORM_MATRIX_SHIFT = 10; //g_transformMatrixShift[TRANSFORM_FORWARD] + COM16_C806_TRANS_PREC;

  const int shift = ( TRANSFORM_MATRIX_SHIFT + inpShift );

  const int add   = ( shift > 0 ) ? ( 1 << ( shift - 1 ) ) : 0;
  CHECK( shift < 0, "Negative shift" );

  const TMatrixCoeff *T = getTrafoPointer( tu, compID );
  const int trStride = dim;

  const TCoeff *coeff   = coeffBuf.buf;
  const int coeffStride = coeffBuf.stride;

  TCoeff* tmpIter = tmpBuf.buf;
  Int64 iSum;
  unsigned i, j, k;
  for( k = 0; k < dim; k++ )
  {
    iSum = 0;
    for( i = 0; i < secTrHeight; i++ )
    {
      for( j=0; j< secTrWidth; j++ )
      {
        iSum += T[ k*trStride + i*secTrWidth+j ] * (coeff[ i * coeffStride + j ] << inpShift );
      }
    }
    tmpIter[k]  = TCoeff( ( iSum  + add ) >> shift );
  }

  TCoeff* dstIter = coeffBuf.buf;
  TCoeff* srcIter = tmpBuf.buf;
  for( int y=0; y < secTrHeight; y++ )
  {
    ::memcpy( dstIter, srcIter, secTrWidth * sizeof( TCoeff ) );
    dstIter += coeffStride;
    srcIter += secTrWidth;
  }
}


void xITrSec_NonSepBF( const TransformUnit &tu, ComponentID compID, CoeffBuf& coeffBuf, CoeffBuf& tmpBuf)
{
  CHECK( !TU::useSecTrafo( tu, compID) , "should not be used" );
  const int tuHeight = (int) coeffBuf.height;
  const int tuWidth  = (int) coeffBuf.width;
  const int secTrHeight = std::min<int>( 8, tuHeight );
  const int secTrWidth  = std::min<int>( 8, tuWidth  );

  const int dim   = secTrWidth*secTrHeight;
  CHECK( dim > 64, "secondary trafo only supported up to dim 64" );

  const int inpShift = 5;
  const Int TRANSFORM_MATRIX_SHIFT = 10; //g_transformMatrixShift[TRANSFORM_INVERSE] + COM16_C806_TRANS_PREC;

  const int shift = ( TRANSFORM_MATRIX_SHIFT + inpShift );

  const int add   = 1 << ( shift - 1 );

  const TCoeff clipMinimum  = std::numeric_limits<TCoeff>::min();
  const TCoeff clipMaximum  = std::numeric_limits<TCoeff>::max();

  const TMatrixCoeff *T = getTrafoPointer( tu, compID );
  const int trStride = dim;

  const TCoeff *coeff   = coeffBuf.buf;
  const int coeffStride = coeffBuf.stride;

  TCoeff* tmpIter = tmpBuf.buf;
  Int64 iSum;
  unsigned i, j, k, kx, ky;
  for( i = 0; i < secTrHeight; i++ )
  {
    for( j=0; j< secTrWidth; j++ )
    {
      iSum = 0;
      for( ky = 0; ky < secTrHeight; ky++ )
      {
        for( kx = 0; kx < secTrWidth; kx++ )
        {
          k     = ky*secTrWidth + kx;
          iSum += T[ k*trStride + i*secTrWidth+j ] * (coeff[ ky* coeffStride + kx ] << inpShift );
        }
      }
      tmpIter[j] = ( TCoeff ) Clip3<Int64>( clipMinimum, clipMaximum,  (iSum + add) >> shift  );
    }
    tmpIter += secTrWidth;
  }

  TCoeff* dstIter = coeffBuf.buf;
  TCoeff* srcIter = tmpBuf.buf;
  for( int y=0; y < secTrHeight; y++ )
  {
    ::memcpy( dstIter, srcIter, secTrWidth * sizeof( TCoeff ) );
    dstIter += coeffStride;
    srcIter += secTrWidth;
  }
}

