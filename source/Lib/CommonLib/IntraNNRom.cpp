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

/** \file     IntraNNRom.cpp
 *  \brief    IntraNNRom
 */

#include "IntraNNRom.h"
#include "CommonDef.h"
#include "Rom.h"
#include "TrQuant_EMT.h"
#include "IntraNNIntRom_static.h"


int getNumModesNNPadded( const int width, const int height )
{
  const int w = int( g_aucLog2[g_PaddedSizes[width]] );
  const int h = int( g_aucLog2[g_PaddedSizes[height]] );
  return g_numIntraModes_NN[w][h];
}

//------------------------------------------------------------------------------
// Integer implementation
//------------------------------------------------------------------------------
void xGetEluTable( int* &elu_table, const int layer_idx )
{
  switch( layer_idx )
  {
  case 0: { elu_table = &g_EluTableBlockZerothLayer[0]; } break;
  case 1: { elu_table = &g_EluTableBlockFirstLayer [0]; } break;
  case 2: { elu_table = &g_EluTableBlockSecondLayer[0]; } break;
  case 3: { elu_table = &g_EluTableBlockThirdLayer [0]; } break;
  default: THROW("Error: wrong layer index");
  }
}

void getWeightPointersMode( const int width, const int height, const int layer_pos, int* &weightsMatrix, int* &weightsBias )
{
  switch( width )
  {
  case 4:  { getWeightPointerModeWidth4 ( height, layer_pos, weightsMatrix, weightsBias ); } break;
  case 8:  { getWeightPointerModeWidth8 ( height, layer_pos, weightsMatrix, weightsBias ); } break;
  case 16: { getWeightPointerModeWidth16( height, layer_pos, weightsMatrix, weightsBias ); } break;
  case 32: { getWeightPointerModeWidth32( height, layer_pos, weightsMatrix, weightsBias ); } break;
  default: THROW( "Error: weights not available" );
  }
}

void getWeightPointersBlock( const int width, const int height, const int layer_pos, int*&weightsMatrix, int*&weightsBias )
{
  switch( width )
  {
  case 4:  { getWeightPointerBlockWidth4 ( height, layer_pos, weightsMatrix, weightsBias ); } break;
  case 8:  { getWeightPointerBlockWidth8 ( height, layer_pos, weightsMatrix, weightsBias ); } break;
  case 16: { getWeightPointerBlockWidth16( height, layer_pos, weightsMatrix, weightsBias ); } break;
  case 32: { getWeightPointerBlockWidth32( height, layer_pos, weightsMatrix, weightsBias ); } break;
  default: THROW( "Error: weights not available" );
  }
}

void getWeightPointersBlockLastLayer( const int width, const int height, const int mode_idx, short* &weightsMatrix, int* &weightsBias )
{
  switch( width )
  {
  case 4:  { getWeightPointerBlockLastLayerWidth4 ( height, mode_idx, weightsMatrix, weightsBias ); } break;
  case 8:  { getWeightPointerBlockLastLayerWidth8 ( height, mode_idx, weightsMatrix, weightsBias ); } break;
  case 16: { getWeightPointerBlockLastLayerWidth16( height, mode_idx, weightsMatrix, weightsBias ); } break;
  case 32: { getWeightPointerBlockLastLayerWidth32( height, mode_idx, weightsMatrix, weightsBias ); } break;
  default: THROW( "Error: weights not available" );
  }
}

void getWeightPointerModeWidth4( const int height, const int layer_pos, int* &weightsMatrix, int* &weightsBias )
{
  switch( height )
  {
  case 4:
    weightsMatrix = &g_MatrEntr_Mode_NN_4x4_INT[layer_pos][0];
    weightsBias   = &g_BiasEntr_Mode_NN_4x4_INT[layer_pos][0];
    break;

  case 8:
    weightsMatrix = &g_MatrEntr_Mode_NN_4x8_INT[layer_pos][0];
    weightsBias   = &g_BiasEntr_Mode_NN_4x8_INT[layer_pos][0];
    break;

  case 16:
    weightsMatrix = &g_MatrEntr_Mode_NN_4x16_INT[layer_pos][0];
    weightsBias   = &g_BiasEntr_Mode_NN_4x16_INT[layer_pos][0];
    break;

  case 32:
    weightsMatrix = &g_MatrEntr_Mode_NN_4x32_INT[layer_pos][0];
    weightsBias   = &g_BiasEntr_Mode_NN_4x32_INT[layer_pos][0];
    break;

  default: THROW( "Error: weights not available" );
  }
}

void getWeightPointerBlockWidth4( const int height, const int layer_pos, int*&weightsMatrix, int*&weightsBias )
{
  switch( height )
  {
  case 4:
    weightsMatrix = &g_MatrEntr_Block_NN_4x4_INT[layer_pos][0];
    weightsBias   = &g_BiasEntr_Block_NN_4x4_INT[layer_pos][0];
    break;

  case 8:
    weightsMatrix = &g_MatrEntr_Block_NN_4x8_INT[layer_pos][0];
    weightsBias   = &g_BiasEntr_Block_NN_4x8_INT[layer_pos][0];
    break;

  case 16:
    weightsMatrix = &g_MatrEntr_Block_NN_4x16_INT[layer_pos][0];
    weightsBias   = &g_BiasEntr_Block_NN_4x16_INT[layer_pos][0];
    break;

  case 32:
    weightsMatrix = &g_MatrEntr_Block_NN_4x32_INT[layer_pos][0];
    weightsBias   = &g_BiasEntr_Block_NN_4x32_INT[layer_pos][0];
    break;

  default: THROW( "Error: weights not available" );
  }
}

void getWeightPointerBlockLastLayerWidth4( const int height, const int mode_idx, short* &weightsMatrix, int* &weightsBias )
{
  switch( height )
  {
  case 4:
    weightsMatrix = &g_MatrEntr_Block_NN_LastLayer_4x4_INT[mode_idx][0];
    weightsBias   = &g_BiasEntr_Block_NN_LastLayer_4x4_INT[mode_idx][0];
    break;

  case 8:
    weightsMatrix = &g_MatrEntr_Block_NN_LastLayer_4x8_INT[mode_idx][0];
    weightsBias   = &g_BiasEntr_Block_NN_LastLayer_4x8_INT[mode_idx][0];
    break;

  case 16:
    weightsMatrix = &g_MatrEntr_Block_NN_LastLayer_4x16_INT[mode_idx][0];
    weightsBias   = &g_BiasEntr_Block_NN_LastLayer_4x16_INT[mode_idx][0];
    break;

  case 32:
    weightsMatrix = &g_MatrEntr_Block_NN_LastLayer_4x32_INT[mode_idx][0];
    weightsBias   = &g_BiasEntr_Block_NN_LastLayer_4x32_INT[mode_idx][0];
    break;

  default: THROW( "Error: weights not available" );
  }
}

void getWeightPointerModeWidth8( const int height, const int layer_pos, int* &weightsMatrix, int* &weightsBias )
{
  switch( height )
  {
  case 4:
    weightsMatrix = &g_MatrEntr_Mode_NN_8x4_INT[layer_pos][0];
    weightsBias   = &g_BiasEntr_Mode_NN_8x4_INT[layer_pos][0];
    break;

  case 8:
    weightsMatrix = &g_MatrEntr_Mode_NN_8x8_INT[layer_pos][0];
    weightsBias   = &g_BiasEntr_Mode_NN_8x8_INT[layer_pos][0];
    break;

  case 16:
    weightsMatrix = &g_MatrEntr_Mode_NN_8x16_INT[layer_pos][0];
    weightsBias   = &g_BiasEntr_Mode_NN_8x16_INT[layer_pos][0];
    break;

  case 32:
    weightsMatrix = &g_MatrEntr_Mode_NN_8x32_INT[layer_pos][0];
    weightsBias   = &g_BiasEntr_Mode_NN_8x32_INT[layer_pos][0];
    break;

  default: THROW( "Error: weights not available" );
  }
}

void getWeightPointerBlockWidth8( const int height, const int layer_pos, int*&weightsMatrix, int*&weightsBias )
{
  switch( height )
  {
  case 4:
    weightsMatrix = &g_MatrEntr_Block_NN_8x4_INT[layer_pos][0];
    weightsBias   = &g_BiasEntr_Block_NN_8x4_INT[layer_pos][0];
    break;

  case 8:
    weightsMatrix = &g_MatrEntr_Block_NN_8x8_INT[layer_pos][0];
    weightsBias   = &g_BiasEntr_Block_NN_8x8_INT[layer_pos][0];
    break;

  case 16:
    weightsMatrix = &g_MatrEntr_Block_NN_8x16_INT[layer_pos][0];
    weightsBias   = &g_BiasEntr_Block_NN_8x16_INT[layer_pos][0];
    break;

  case 32:
    weightsMatrix = &g_MatrEntr_Block_NN_8x32_INT[layer_pos][0];
    weightsBias   = &g_BiasEntr_Block_NN_8x32_INT[layer_pos][0];
    break;

  default: THROW( "Error: weights not available" );
  }
}

void getWeightPointerBlockLastLayerWidth8( const int height, const int mode_idx, short* &weightsMatrix, int* &weightsBias )
{
  switch( height )
  {
  case 4:
    weightsMatrix = &g_MatrEntr_Block_NN_LastLayer_8x4_INT[mode_idx][0];
    weightsBias   = &g_BiasEntr_Block_NN_LastLayer_8x4_INT[mode_idx][0];
    break;

  case 8:
    weightsMatrix = &g_MatrEntr_Block_NN_LastLayer_8x8_INT[mode_idx][0];
    weightsBias   = &g_BiasEntr_Block_NN_LastLayer_8x8_INT[mode_idx][0];
    break;

  case 16:
    weightsMatrix = &g_MatrEntr_Block_NN_LastLayer_8x16_INT[mode_idx][0];
    weightsBias   = &g_BiasEntr_Block_NN_LastLayer_8x16_INT[mode_idx][0];
    break;

  case 32:
    weightsMatrix = &g_MatrEntr_Block_NN_LastLayer_8x32_INT[mode_idx][0];
    weightsBias   = &g_BiasEntr_Block_NN_LastLayer_8x32_INT[mode_idx][0];
    break;

  default: THROW( "Error: weights not available" );
  }
}

void getWeightPointerModeWidth16( const int height, const int layer_pos, int* &weightsMatrix, int* &weightsBias )
{
  switch( height )
  {
  case 4:
    weightsMatrix = &g_MatrEntr_Mode_NN_16x4_INT[layer_pos][0];
    weightsBias   = &g_BiasEntr_Mode_NN_16x4_INT[layer_pos][0];
    break;

  case 8:
    weightsMatrix = &g_MatrEntr_Mode_NN_16x8_INT[layer_pos][0];
    weightsBias   = &g_BiasEntr_Mode_NN_16x8_INT[layer_pos][0];
    break;

  case 16:
    weightsMatrix = &g_MatrEntr_Mode_NN_16x16_INT[layer_pos][0];
    weightsBias   = &g_BiasEntr_Mode_NN_16x16_INT[layer_pos][0];
    break;

  case 32:
    weightsMatrix = &g_MatrEntr_Mode_NN_16x32_INT[layer_pos][0];
    weightsBias   = &g_BiasEntr_Mode_NN_16x32_INT[layer_pos][0];
    break;

  default: THROW( "Error: weights not available" );
  }
}

void getWeightPointerBlockWidth16( const int height, const int layer_pos, int*&weightsMatrix, int*&weightsBias )
{
  switch( height )
  {
  case 4:
    weightsMatrix = &g_MatrEntr_Block_NN_16x4_INT[layer_pos][0];
    weightsBias   = &g_BiasEntr_Block_NN_16x4_INT[layer_pos][0];
    break;

  case 8:
    weightsMatrix = &g_MatrEntr_Block_NN_16x8_INT[layer_pos][0];
    weightsBias   = &g_BiasEntr_Block_NN_16x8_INT[layer_pos][0];
    break;

  case 16:
    weightsMatrix = &g_MatrEntr_Block_NN_16x16_INT[layer_pos][0];
    weightsBias   = &g_BiasEntr_Block_NN_16x16_INT[layer_pos][0];
    break;

  case 32:
    weightsMatrix = &g_MatrEntr_Block_NN_16x32_INT[layer_pos][0];
    weightsBias   = &g_BiasEntr_Block_NN_16x32_INT[layer_pos][0];
    break;

  default: THROW( "Error: weights not available" );
  }
}

void getWeightPointerBlockLastLayerWidth16( const int height, const int mode_idx, short* &weightsMatrix, int* &weightsBias )
{
  switch( height )
  {
  case 4:
    weightsMatrix = &g_MatrEntr_Block_NN_LastLayer_16x4_INT[mode_idx][0];
    weightsBias   = &g_BiasEntr_Block_NN_LastLayer_16x4_INT[mode_idx][0];
    break;

  case 8:
    weightsMatrix = &g_MatrEntr_Block_NN_LastLayer_16x8_INT[mode_idx][0];
    weightsBias   = &g_BiasEntr_Block_NN_LastLayer_16x8_INT[mode_idx][0];
    break;

  case 16:
    weightsMatrix = &g_MatrEntr_Block_NN_LastLayer_16x16_INT[mode_idx][0];
    weightsBias   = &g_BiasEntr_Block_NN_LastLayer_16x16_INT[mode_idx][0];
    break;

  case 32:
    weightsMatrix = &g_MatrEntr_Block_NN_LastLayer_16x32_INT[mode_idx][0];
    weightsBias   = &g_BiasEntr_Block_NN_LastLayer_16x32_INT[mode_idx][0];
    break;

  default: THROW( "Error: weights not available" );
  }
}

void getWeightPointerModeWidth32( const int height, const int layer_pos, int* &weightsMatrix, int* &weightsBias )
{
  switch( height )
  {
  case 32:
    weightsMatrix = &g_MatrEntr_Mode_NN_32x32_INT[layer_pos][0];
    weightsBias   = &g_BiasEntr_Mode_NN_32x32_INT[layer_pos][0];
    break;

  default: THROW( "Error: weights not available" );
  }
}

void getWeightPointerBlockWidth32( const int height, const int layer_pos, int*&weightsMatrix, int*&weightsBias )
{
  switch( height )
  {
  case 32:
    weightsMatrix = &g_MatrEntr_Block_NN_32x32_INT[layer_pos][0];
    weightsBias   = &g_BiasEntr_Block_NN_32x32_INT[layer_pos][0];
    break;

  default: THROW( "Error: weights not available" );
  }
}

void getWeightPointerBlockLastLayerWidth32( const int height, const int mode_idx, short* &weightsMatrix, int* &weightsBias )
{
  switch( height )
  {
  case 32:
    weightsMatrix = &g_MatrEntr_Block_NN_LastLayer_32x32_INT[mode_idx][0];
    weightsBias   = &g_BiasEntr_Block_NN_LastLayer_32x32_INT[mode_idx][0];
    break;

  default: THROW( "Error: weights not available" );
  }
}

bool subSamplingAllowed( const int width, const int height )
{
  return (width == height && width >= MIN_SIZE_FOR_UPSAMPLING);
}

bool isTransposed( const int width, const int height ) 
{ 
  return (height < width && width == 32); 
}

UChar* getMaskNonZeroRows( const int log2width, const int log2height, const int mode_idx )
{
  if( log2width == 2 )
  {
    if( log2height == 3 ) { return &g_MaskNonZeroRowsBlock_4x8  [mode_idx][0]; }
    if( log2height == 4 ) { return &g_MaskNonZeroRowsBlock_4x16 [mode_idx][0]; }
    if( log2height == 5 ) { return &g_MaskNonZeroRowsBlock_4x32 [mode_idx][0]; }
  }
  if( log2width == 3 )
  {
    if( log2height == 2 ) { return &g_MaskNonZeroRowsBlock_8x4  [mode_idx][0]; }
    if( log2height == 3 ) { return &g_MaskNonZeroRowsBlock_8x8  [mode_idx][0]; }
    if( log2height == 4 ) { return &g_MaskNonZeroRowsBlock_8x16 [mode_idx][0]; }
    if( log2height == 5 ) { return &g_MaskNonZeroRowsBlock_8x32 [mode_idx][0]; }
  }
  if( log2width == 4 )
  {
    if( log2height == 2 ) { return &g_MaskNonZeroRowsBlock_16x4 [mode_idx][0]; }
    if( log2height == 3 ) { return &g_MaskNonZeroRowsBlock_16x8 [mode_idx][0]; }
    if( log2height == 4 ) { return &g_MaskNonZeroRowsBlock_16x16[mode_idx][0]; }
    if( log2height == 5 ) { return &g_MaskNonZeroRowsBlock_16x32[mode_idx][0]; }
  }
  if( log2width == 5 )
  {
    if( log2height == 5 ) { return &g_MaskNonZeroRowsBlock_32x32[mode_idx][0]; }
  }
  THROW( "Error" );
}

UChar workInTrafoDomain( const int log2width, const int log2height )
{
  return g_WorkInTrafoDomain[log2width][log2height];
}


void computIntegerInverseDCT( const int* weights, int* weightsTransformed, const int width, const int height )
{
  std::vector<int> tmp( width * height );

  const int maxLog2TrDynamicRange = 15;
  const int TRANSFORM_MATRIX_SHIFT = g_transformMatrixShift[TRANSFORM_INVERSE];
  const int shift_1st =  TRANSFORM_MATRIX_SHIFT + 1 + COM16_C806_TRANS_PREC; //1 has been added to shift_1st at the expense of shift_2nd
  const int shift_2nd = (TRANSFORM_MATRIX_SHIFT + maxLog2TrDynamicRange - 1) - g_assumed_standard_bitdepth + COM16_C806_TRANS_PREC;
  const TCoeff clipMinimum = -(1 << maxLog2TrDynamicRange);
  const TCoeff clipMaximum =  (1 << maxLog2TrDynamicRange) - 1;

  switch( height )
  {
  case 4:  { fastInverseDCT2_B4 ( weights, &tmp[0], shift_1st, width, 0, 0, 1, clipMinimum, clipMaximum ); } break;
  case 8:  { fastInverseDCT2_B8 ( weights, &tmp[0], shift_1st, width, 0, 0, 1, clipMinimum, clipMaximum ); } break;
  case 16: { fastInverseDCT2_B16( weights, &tmp[0], shift_1st, width, 0, 0, 1, clipMinimum, clipMaximum ); } break;
  case 32: { fastInverseDCT2_B32( weights, &tmp[0], shift_1st, width, 0, 0, 1, clipMinimum, clipMaximum ); } break;
  default: THROW( "Error: block size not supported" );
  }
  switch( width )
  {
  case 4:  { fastInverseDCT2_B4 ( &tmp[0], &weightsTransformed[0], shift_2nd, height, 0, 0, 1, clipMinimum, clipMaximum ); } break;
  case 8:  { fastInverseDCT2_B8 ( &tmp[0], &weightsTransformed[0], shift_2nd, height, 0, 0, 1, clipMinimum, clipMaximum ); } break;
  case 16: { fastInverseDCT2_B16( &tmp[0], &weightsTransformed[0], shift_2nd, height, 0, 0, 1, clipMinimum, clipMaximum ); } break;
  case 32: { fastInverseDCT2_B32( &tmp[0], &weightsTransformed[0], shift_2nd, height, 0, 0, 1, clipMinimum, clipMaximum ); } break;
  default: THROW( "Error: block size not supported" );
  }
}

int getNumNonZeroRowsBlock( const int log2width, const int log2height, const int mode_idx )
{
  return g_numNonZeroRowsBlock[log2width][log2height][mode_idx];
}

