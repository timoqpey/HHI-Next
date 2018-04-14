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

/** \file     IntraPrediction_NN.cpp
 *  \brief    IntraPrediction_NN
 */

#include "IntraPrediction_NN.h"
#include "IntraNNRom.h"

IntraPrediction_NN::IntraPrediction_NN() : m_fApplyMatrixShort( applyMatrix ), m_fApplyMatrixInt( applyMatrix )
{
#if HHI_SIMD_OPT_INTRA_PRED_NN
#ifdef TARGET_SIMD_X86
  initIntraPredNNX86();
#endif
#endif
}

void IntraPrediction_NN::applyMatrixPadded( std::vector<int>&output, const std::vector<int>&input, const short* weights_matrix, const int input_size, const int output_size, 
                                            const int output_width, const int output_height, const int padded_output_width )
{
  CHECK( output_size != (output_width * output_height), "Error" );
  output.resize( output_size );
  int idx = 0;
  for( int row_in_output = 0; row_in_output < output_height; row_in_output++ )
  {
    for( int col_in_output = 0; col_in_output < output_width; col_in_output++ )
    {
      const int pos_in_matrx = (row_in_output * padded_output_width + col_in_output) * input_size;
      int val = 0;
      int j = 0;
      for( ; j < input_size - 3; j += 4 )
      {
        val +=   weights_matrix[pos_in_matrx + j]     * input[j]
               + weights_matrix[pos_in_matrx + j + 1] * input[j + 1]
               + weights_matrix[pos_in_matrx + j + 2] * input[j + 2]
               + weights_matrix[pos_in_matrx + j + 3] * input[j + 3];
      }
      for( ; j < input_size; j++ )
      {
        val += weights_matrix[pos_in_matrx + j] * input[j];
      }
      output[idx] = val;
      idx++;
    }
  }
}

template<typename T>
void applyMatrix( int* output, const int* input, const T* weights_matrix, const int input_size, const int output_size )
{
  int pos_in_matrx = 0;
  for( int i = 0; i < output_size; i++ )
  {
    int val = 0;
    int j = 0;
    for( ; j < input_size - 3; j += 4 )
    {
      val +=   weights_matrix[pos_in_matrx + j]     * input[j]
             + weights_matrix[pos_in_matrx + j + 1] * input[j + 1]
             + weights_matrix[pos_in_matrx + j + 2] * input[j + 2]
             + weights_matrix[pos_in_matrx + j + 3] * input[j + 3];
    }
    for( ; j < input_size; j++ )
    {
      val += weights_matrix[pos_in_matrx + j] * input[j];
    }
    pos_in_matrx += input_size;
    output[i] = val;
  }
}

void IntraPrediction_NN::applyMatrixWithMask( std::vector<int> &output, const std::vector<int> &input, const short* weights_matrix, const int input_size, const int output_size, UChar* Mask, const int num_nonzero_rows )
{
  const int upper_bound = (num_nonzero_rows % 4 != 0) ? (4 - num_nonzero_rows % 4) : 0;
  const int size_output_nonzeros = num_nonzero_rows + upper_bound;
  std::vector<int> output_reduced( size_output_nonzeros, 0 );
  m_fApplyMatrixShort( &output_reduced[0], &input[0], weights_matrix, input_size, size_output_nonzeros );

  output.resize( output_size, 0 );
  int pos_output_reduced = 0;
  for( int i = 0; i < output_size; i++ )
  {
    if( Mask[i] )
    {
      output[i] = output_reduced[pos_output_reduced];
      pos_output_reduced++;
    }
  }
}

template<typename T>
void IntraPrediction_NN::applyBias( std::vector<T> &inoutput, const T* weights_bias )
{
  int i = 0;
  for( ; i < inoutput.size() - 3; i += 4 )
  {
    inoutput[i]     += weights_bias[i];
    inoutput[i + 1] += weights_bias[i + 1];
    inoutput[i + 2] += weights_bias[i + 2];
    inoutput[i + 3] += weights_bias[i + 3];
  }
  for( ; i < inoutput.size(); i++ )
  {
    inoutput[i] += weights_bias[i];
  }
}
template<typename T>
void IntraPrediction_NN::applyBiasPadded( std::vector<T> &inoutput, const T* weights_bias, const int output_width, const int output_height, const int padded_output_width )
{
  for( int row_in_output = 0; row_in_output < output_height; row_in_output++ )
  {
    int pos_in_bias = row_in_output*padded_output_width;
    int pos_in_output = row_in_output*output_width;
    int col_in_output = 0;
    for( ; col_in_output < output_width - 3; col_in_output += 4 )
    {
      inoutput[pos_in_output + col_in_output]     += weights_bias[pos_in_bias + col_in_output];
      inoutput[pos_in_output + col_in_output + 1] += weights_bias[pos_in_bias + col_in_output + 1];
      inoutput[pos_in_output + col_in_output + 2] += weights_bias[pos_in_bias + col_in_output + 2];
      inoutput[pos_in_output + col_in_output + 3] += weights_bias[pos_in_bias + col_in_output + 3];
    }
    for( ; col_in_output < output_width; col_in_output++ )
    {
      inoutput[pos_in_output + col_in_output] += weights_bias[pos_in_bias + col_in_output];
    }
  }
}

void IntraPrediction_NN::applyLayerEluInt( std::vector<int> &output, const std::vector<int> &input,
                                           const int* weights_matrix, const int* weights_bias,
                                           const int input_size, const int output_size, 
                                           const int w, const int h, const int layer_idx )
{
  output.resize( output_size );
  if( output_size % 4 == 0 ) { m_fApplyMatrixInt( &output[0], &input[0], weights_matrix, input_size, output_size ); }
  else                       { applyMatrix      ( &output[0], &input[0], weights_matrix, input_size, output_size ); }
  rightShiftResultInt( output, g_Internal_Right_Shifts_wo_Bias[w][h][layer_idx] );

  applyBias( output, weights_bias );
  rightShiftResultInt( output, g_Internal_Right_Shifts_Matrix_Bias[w][h][layer_idx] );

  applyEluInt( output, w, h, layer_idx );
}

void IntraPrediction_NN::applyLayerInt( std::vector<int> &output, const std::vector<int> &input,
                                        const short* weights_matrix, const int* weights_bias,
                                        const int input_size, const int output_size,
                                        const int output_width, const int output_height, 
                                        const int padded_output_width,
                                        const int w, const int h, const int mode_idx )
{
  if( workInTrafoDomain( w, h ) )
  {
    std::vector<int> outputDctDomain;
    applyMatrixWithMask( outputDctDomain, input, weights_matrix, input_size, output_size, getMaskNonZeroRows( w, h, mode_idx ), getNumNonZeroRowsBlock( w, h, mode_idx ) );
    rightShiftResultInt( outputDctDomain, g_Right_Shifts_ResultNN_wo_Bias[w][h][mode_idx] );

    applyBias( outputDctDomain, weights_bias );
    rightShiftResultInt( outputDctDomain, g_Right_Shifts_ResultNN_Matrix_Bias[w][h][mode_idx] );

    const int padded_width  = 1 << w;
    const int padded_height = 1 << h;
    output.resize( output_width*output_height );
    if( padded_width == output_width && padded_height == output_height )
    {
      computIntegerInverseDCT( &outputDctDomain[0], &output[0], padded_width, padded_height );
    }
    else
    {
      std::vector<int> padded_output( padded_width*padded_height );
      computIntegerInverseDCT( &outputDctDomain[0], &padded_output[0], padded_width, padded_height );
      restrictPaddedOutput( output, padded_output, padded_width, padded_height, output_width, output_height );
    }
  }
  else
  {
    applyMatrixPadded( output, input, weights_matrix, input_size, output_size, output_width, output_height, padded_output_width );
    rightShiftResultInt( output, g_Right_Shifts_ResultNN_wo_Bias[w][h][mode_idx] );

    applyBiasPadded( output, weights_bias, output_width, output_height, padded_output_width );
    rightShiftResultInt( output, g_Right_Shifts_ResultNN_Matrix_Bias[w][h][mode_idx] );
  }
}

void IntraPrediction_NN::applyEluInt( std::vector<int> &in_output, const int w, const int h, const int layer_idx )
{
  int * elu_table;
  xGetEluTable( elu_table, layer_idx );
  for( int i = 0; i < (int)in_output.size(); i++ )
  {
    if( in_output[i] < 0 )
    {
      const int right_shift_neg_input = (layer_idx + 1)*g_KeptPrecisionsJoint[w][h] - PRECISION_ELU_TABLE;
      int input_val = std::min( (right_shift_neg_input >= 0) ? (std::abs( in_output[i] ) >> right_shift_neg_input) : (std::abs( in_output[i] ) << (-right_shift_neg_input)), g_EluTableSizes[layer_idx] - 1 );
      in_output[i]  = elu_table[input_val];
    }
  }
}

void IntraPrediction_NN::applyLayerEluInt_Mode( std::vector<int> &output, const std::vector<int> &input,
                                                const int* weights_matrix, const int* weights_bias,
                                                const int input_size, const int output_size, 
                                                const int w, const int h, const int layer_idx )
{
  output.resize( output_size );
  if( output_size % 4 == 0 ) { m_fApplyMatrixInt( &output[0], &input[0], weights_matrix, input_size, output_size ); }
  else                       { applyMatrix      ( &output[0], &input[0], weights_matrix, input_size, output_size ); }
  rightShiftResultInt( output, g_Internal_Right_Shifts_Mode_wo_Bias[w][h][layer_idx] );

  applyBias( output, weights_bias );
  rightShiftResultInt( output, g_Internal_Right_Shifts_Mode_Matrix_Bias[w][h][layer_idx] );

  if( layer_idx != g_NumLayers_Mode_NN[w][h] - 1 )
  {
    applyEluInt( output, w, h, layer_idx );
  }
}

void IntraPrediction_NN::restrictPaddedOutput( std::vector<int> &output, const std::vector<int> &padded_output,
                                               const int padded_output_width, const int padded_output_height,
                                               const int output_width, const int output_height )
{
  CHECK( padded_output_width  <  output_width,                              "Error" );
  CHECK( padded_output_height <  output_height,                             "Error" );
  CHECK( output.size()        != output_width*output_height,                "Error" );
  CHECK( padded_output.size() != padded_output_width* padded_output_height, "Error" );
  for( int y = 0; y < output_height; y++ )
  {
    for( int x = 0; x < output_width; x++ )
    {
      output[y * output_width + x] = padded_output[y * padded_output_width + x];
    }
  }
}


template<typename T>
void IntraPrediction_NN::rightShiftResultInt( std::vector<T> &inoutput, const int shift )
{
  if( shift == 0 ) return;

  CHECK( shift < 0, "Error: negative shift" );
  const T offset = (T( 1 ) << shift) - 1;

  for( int idx = 0; idx < inoutput.size(); idx++ )
  {
    int sign = inoutput[idx] < 0 ? -1 : 1;
    inoutput[idx] = sign*((std::abs( inoutput[idx] ) + offset) >> shift);
  }
}
template<typename T>
void IntraPrediction_NN::shiftOutputLastLayer( std::vector<T> &inoutput, const int bitDepth )
{
  const int offset = 1 << (bitDepth - 1);
  for( int idx = 0; idx < inoutput.size(); idx++ )
  {
    inoutput[idx] = inoutput[idx] + offset;
  }
}

template<typename T>
void IntraPrediction_NN::copyWithPadding(AreaBuf<T> dst, const CPelBuf& src, const Area srcArea, pos2Bool srcAvail, Pel defaultPadVal)
{
  CHECK(Size(dst) != srcArea.size(), "Error: mismatch between source and destination size");
  Pel padVal = defaultPadVal;
  for (int y = 0; y < srcArea.height; y++)
  {
    for (int x = 0; x < srcArea.width; x++)
    {
      Position posDst(x, y);
      Position posSrc = srcArea.offset(posDst);
      if (srcAvail(posSrc))
      {
        dst.at(posDst) = src.at(posSrc);
        padVal = src.at(posSrc);  // TODO_ANN: use a better padding method
      }
      else
      {
        dst.at(posDst) = (T)(padVal);
      }
    }
  }
}


void IntraPrediction_NN::computeLastLayerInt_Downsampling( std::vector<int> &predSignal, const std::vector<int> &outputJointHiddenLayers, const int width, const int height, 
                                                           const int mode_idx, const int bitDepth, const CPelBuf &src, const Area block, pos2Bool available )
{
  const int halfwidth = width >> 1;
  const int halfheight = height >> 1;
  const int w = g_aucLog2[halfwidth];
  const int h = g_aucLog2[halfheight];
  CHECK( mode_idx >= g_numIntraModes_NN[w][h], "Error" );

  const int last_layer_idx = g_NumLayers_Block_NN[w][h] - 1;
  short* weights_Matrix_Last_Layer;
  int*   weights_Bias_Last_Layer;
  getWeightPointersBlockLastLayer( halfwidth, halfheight, mode_idx, weights_Matrix_Last_Layer, weights_Bias_Last_Layer );

  const int input_size  = g_Architecture_Block_NN[w][h][last_layer_idx];
  const int output_size = g_Architecture_Block_NN[w][h][last_layer_idx + 1];
  predSignalDS.clear();
  applyLayerInt( predSignalDS, outputJointHiddenLayers, weights_Matrix_Last_Layer, weights_Bias_Last_Layer,
                 input_size, output_size, halfwidth, halfheight, halfwidth, w, h, mode_idx );

  shiftOutputLastLayer( predSignalDS, bitDepth );
  xLinInterpolRectangleFactorTwo( predSignalDS, predSignal, Size( halfwidth, halfheight ) );
}



void IntraPrediction_NN::computeJointHiddenOutputInt_Downsampling( std::vector<int> &hiddenLayerOutput, const CPelBuf &src, const Area block, const int bitDepth, pos2Bool available )
{
  input_curr.clear();

  const int halfwidth = block.width >> 1;
  const int halfheight = block.height >> 1;
  const int w = g_aucLog2[halfwidth];
  const int h = g_aucLog2[halfheight];
  const int sizeCtx = g_Context_Sizes_Block_NN[w][h];
  const int num_hidden_layers_curr = g_NumLayers_Block_NN[w][h] - 1;
  prepareInput_Downsampling( input_curr, src, block, available, bitDepth, sizeCtx );
  int*weights_Matrix_Shared_Layer;
  int*weights_Bias_Shared_Layer;
  for( int layer_idx = 0; layer_idx < num_hidden_layers_curr; layer_idx++ )
  {
    const int input_size = g_Architecture_Block_NN[w][h][layer_idx];
    const int output_size = g_Architecture_Block_NN[w][h][layer_idx + 1];
    getWeightPointersBlock( halfwidth, halfheight, layer_idx, weights_Matrix_Shared_Layer, weights_Bias_Shared_Layer );
    applyLayerEluInt( hiddenLayerOutput, input_curr, weights_Matrix_Shared_Layer, weights_Bias_Shared_Layer, input_size, output_size, w, h, layer_idx );
    input_curr = hiddenLayerOutput;
  }
  rightShiftResultInt( hiddenLayerOutput, g_RightShifts_For_Last_Layer[w][h] );
}

void IntraPrediction_NN::computeModeList_Downsampling( std::vector<UInt> &modeList, const CPelBuf &src, const Area block, const int bitDepth, pos2Bool available )
{
  input_Data.clear();

  CHECK( !subSamplingAllowed( block.width, block.height ), "Error: subsampling not allowed for this block size" );
  const int halfwidth  = block.width  >> 1;
  const int halfheight = block.height >> 1;
  const int w = g_aucLog2[halfwidth];
  const int h = g_aucLog2[halfheight];
  prepareInput_Downsampling( input_Data, src, block, available, bitDepth, g_Context_Sizes_Mode_NN[w][h] );

  loggits.clear();
  computeLoggitsModesInt( loggits, input_Data, halfwidth, halfheight );
  sortingPermutation(modeList, loggits);
  CHECK(modeList.size() != g_numIntraModes_NN[w][h], "Error: wrong mode list size");
}


template<typename T>
void IntraPrediction_NN::prepareInput_Transpose( std::vector<T>&input, const CPelBuf& src, const Area block, pos2Bool available, const int bitDepth, const int sizeCtx )
{
  if( sizeCtx == 0 )
  {
    return;
  }

  const Pel defaultPad = 1 << (bitDepth - 1);
  const int offset     = sizeCtx;
  const T   inital_val = T(0);

  // get above rectangle (incl. corner)
  const int rectAbove_width  = sizeCtx + block.width;
  const int rectAbove_height = sizeCtx;
  rectangle_above_for_TP.resize( rectAbove_width * rectAbove_height, inital_val );
  AreaBuf<T>     rectAbove_buffer( &rectangle_above_for_TP[0], SizeType(rectAbove_width), SizeType(rectAbove_height) );
  const Area     rectAbove_area( Position( block.x - offset, block.y - offset ), Size( rectAbove_width, rectAbove_height ) );
  generatePaddedRectangle( rectAbove_buffer, src, offset, rectAbove_area, available, defaultPad );

  // get left rectangle (incl. corner)
  const int rectLeft_width  = sizeCtx;
  const int rectLeft_height = block.height + sizeCtx;
  rectangle_left_for_TP.resize( rectLeft_width * rectLeft_height, inital_val );
  AreaBuf<T>     rectLeft_buffer( &rectangle_left_for_TP[0], SizeType(rectLeft_width), SizeType(rectLeft_height) );
  const Area     rectLeft_area( Position( block.x - offset, block.y - offset ), Size( rectLeft_width, rectLeft_height ) );
  generatePaddedRectangle( rectLeft_buffer, src, offset, rectLeft_area, available, defaultPad );

  // fill input
  input.clear();
  input.resize( sizeCtx*(sizeCtx + block.width + block.height) );

  int idx = 0;
  // Top = transposed left rectangle without corner
  for( int x = 0; x < sizeCtx; x++ )
  {
    for( int y = 0; y < block.height; y++ )
    {
      input[idx] = rectangle_left_for_TP[(y + sizeCtx) * sizeCtx + x];
      idx++;
    }
  }

  // Corner = transposed corner
  for( int x = 0; x < sizeCtx; x++ )
  {
    for( int y = 0; y < sizeCtx; y++ )
    {
      input[idx] = rectangle_left_for_TP[y * sizeCtx + x];
      idx++;
    }
}

  // Left = transposed above rectangle without corner
  for( int x = 0; x < block.width; x++ )
  {
    for( int y = 0; y < sizeCtx; y++ )
    {
      input[idx] = rectangle_above_for_TP[y * (block.width + sizeCtx) + x + sizeCtx];
      idx++;
    }
  }
}

template<typename T>
void IntraPrediction_NN::prepareInput_Downsampling( std::vector<T> &input, const CPelBuf &src, const Area block, pos2Bool available, const int bitDepth, const int sizeCtx )
{

  if (sizeCtx == 0)
  {
    return;
  }

  Pel defaultPad = 1 << (bitDepth - 1);
  const int  offset = 2 * sizeCtx;
  T inital_val = (T)(0);
  // above rectangle (incl. corner area)
  const int rectangle_above_for_DS_width = (2 * sizeCtx + block.width);
  const int rectangle_above_for_DS_height = 2 * sizeCtx;
  rectangle_above_for_DS.resize(rectangle_above_for_DS_width*rectangle_above_for_DS_height, inital_val);
  AreaBuf<T> rectangle_buffer_above( &rectangle_above_for_DS[0], SizeType(rectangle_above_for_DS_width), SizeType(rectangle_above_for_DS_height) );
  Position   rect_above_pos(block.x - offset, block.y - offset);
  Size       rect_above_size(rectangle_above_for_DS_width, rectangle_above_for_DS_height);
  const Area rect_above_for_DS_Area(rect_above_pos, rect_above_size);
  generatePaddedRectangle(rectangle_buffer_above, src, offset, rect_above_for_DS_Area, available, defaultPad);


  // left rectangle (incl. corner area)
  const int rectangle_left_for_DS_width = 2 * sizeCtx;
  const int rectangle_left_for_DS_height = block.height + 2 * sizeCtx;
  rectangle_left_for_DS.resize(rectangle_left_for_DS_width*rectangle_left_for_DS_height, inital_val);
  AreaBuf<T> rectangle_buffer_left( &rectangle_left_for_DS[0], SizeType(rectangle_left_for_DS_width), SizeType(rectangle_left_for_DS_height) );
  Position   rect_left_pos(block.x - offset, block.y - offset);
  Size       rect_left_size(rectangle_left_for_DS_width, rectangle_left_for_DS_height);
  const Area rect_left_for_DS_Area(rect_left_pos, rect_left_size);
  defaultPad = 1 << (bitDepth - 1);
  generatePaddedRectangle(rectangle_buffer_left, src, offset, rect_left_for_DS_Area, available, defaultPad);

  // downsampling of above rectangle
  xDownSamplingRectangleFactorTwo(rectangle_above_for_DS, rectangle_above_downsampled, rect_above_size);

  // downsampling of left rectangle
  xDownSamplingRectangleFactorTwo(rectangle_left_for_DS, rectangle_left_downsampled, rect_left_size);

  input.clear();
  const int halfwidth = block.width >> 1;
  const int halfheight = block.height >> 1;

  input.resize(sizeCtx*(sizeCtx + halfwidth + halfheight));
  int idx = 0;
  //top
  for (int y = 0; y < sizeCtx; y++)
  {
    for (int x = 0; x < halfwidth; x++)
    {
      input[idx] = rectangle_above_downsampled[y*(halfwidth + sizeCtx) + sizeCtx + x];
      idx++;
    }
  }

  //corner
  for (int y = 0; y < sizeCtx; y++)
  {
    for (int x = 0; x < sizeCtx; x++)
    {
      input[idx] = rectangle_above_downsampled[y*(halfwidth + sizeCtx) + x];
      idx++;
    }
  }

  //left
  for (int y = 0; y < halfheight; y++)
  {
    for (int x = 0; x < sizeCtx; x++)
    {
      input[idx] = rectangle_left_downsampled[(y + sizeCtx)*sizeCtx + x];
      idx++;
    }
  }
}

template<typename T>
void IntraPrediction_NN::generatePaddedRectangle(AreaBuf<T> dst, const CPelBuf& src, const int extension_size,
  const Area rectAreaInRecoPic, pos2Bool srcAvail, Pel defaultPadVal)
{
  Pel padVal = defaultPadVal;
  for (int y = 0; y < extension_size; y++)
  {
    for (int x = 0; x < rectAreaInRecoPic.width; x++)
    {
      Position posDst(x, y);
      Position posSrc = rectAreaInRecoPic.offset(posDst);
      if (srcAvail(posSrc))
      {
        dst.at(posDst) = src.at(posSrc);
        padVal = src.at(posSrc);
      }
      else
      {
        dst.at(posDst) = padVal;
      }
    }
  }
  for (int y = extension_size; y < rectAreaInRecoPic.height; y++)
  {
    for (int x = 0; x < extension_size; x++)
    {
      Position posDst(x, y);
      Position posSrc = rectAreaInRecoPic.offset(posDst);
      if (srcAvail(posSrc))
      {
        dst.at(posDst) = src.at(posSrc);
        padVal = src.at(posSrc);
      }
      else
      {
        dst.at(posDst) = padVal;
      }
    }
  }
}


template<typename T>
void IntraPrediction_NN::getInputDataPad(std::vector<T>& input_data, const CPelBuf& src, const Area block, pos2Bool available, const int bitDepth, const int sizeCtx)
{
  input_data.resize(sizeCtx*(block.height + sizeCtx + block.width));
  if (sizeCtx == 0)
  {
    return;
  }
  AreaBuf<T> top(&input_data[0], block.width, block.width, sizeCtx);
  AreaBuf<T> corner(&input_data[0] + sizeCtx*block.width, sizeCtx, sizeCtx, sizeCtx);
  AreaBuf<T> left(&input_data[0] + sizeCtx*(block.width + sizeCtx), sizeCtx, sizeCtx, block.height);

  Pel defaultPad = 1 << (bitDepth - 1);
  copyWithPadding(top, src, Area(block.x, block.y - sizeCtx, block.width, sizeCtx), available, defaultPad);
  copyWithPadding(corner, src, Area(block.x - sizeCtx, block.y - sizeCtx, sizeCtx, sizeCtx), available, defaultPad);
  copyWithPadding(left, src, Area(block.x - sizeCtx, block.y, sizeCtx, block.height), available, defaultPad);
}



void IntraPrediction_NN::computeLastLayerInt( std::vector<int> &predSignal, const std::vector<int> &outputJointHiddenLayers, const int width, const int height, const int mode_idx, const int bitDepth )
{
  int org_width  = width;
  int org_height = height;

  int padded_width  = g_PaddedSizes[width];
  int padded_height = g_PaddedSizes[height];
  int w = g_aucLog2[padded_width];
  int h = g_aucLog2[padded_height];
  if( isTransposed( padded_width, padded_height ) )
  {
    std::swap( w, h );
    std::swap( padded_width, padded_height );
    std::swap( org_width, org_height );
  }
  CHECK( mode_idx >= g_numIntraModes_NN[w][h], "Error" );
  const int last_layer_idx = g_NumLayers_Block_NN[w][h] - 1;
  short* weights_Matrix_Last_Layer;
  int*   weights_Bias_Last_Layer;
  getWeightPointersBlockLastLayer( padded_width, padded_height, mode_idx, weights_Matrix_Last_Layer, weights_Bias_Last_Layer );

  const int input_size  = g_Architecture_Block_NN[w][h][last_layer_idx];
  const int output_size = g_Architecture_Block_NN[w][h][last_layer_idx + 1];
  applyLayerInt( predSignal, outputJointHiddenLayers, weights_Matrix_Last_Layer, weights_Bias_Last_Layer,
                 input_size, output_size, org_width, org_height, padded_width, w, h, mode_idx );

  shiftOutputLastLayer( predSignal, bitDepth );

  CHECK( output_size != (padded_width * padded_height), "Error" );
  CHECK( predSignal.size() != (width * height),         "Error" );
}



void IntraPrediction_NN::computeJointHiddenOutputInt( std::vector<int> &hiddenLayerOutput, const CPelBuf &src, const Area block, const int bitDepth, pos2Bool available )
{
  const int w = g_aucLog2[block.width];
  const int h = g_aucLog2[block.height];
  const int num_hidden_layers_curr = g_NumLayers_Block_NN[w][h] - 1;

  input_curr2.clear();

  const int sizeCtx = g_Context_Sizes_Block_NN[w][h];
  getInputDataPad( input_curr2, src, block, available, bitDepth, sizeCtx );

  int* weights_Matrix_Shared_Layer;
  int* weights_Bias_Shared_Layer;
  for( int layer_idx = 0; layer_idx < num_hidden_layers_curr; layer_idx++ )
  {
    const int input_size = g_Architecture_Block_NN[w][h][layer_idx];
    const int output_size = g_Architecture_Block_NN[w][h][layer_idx + 1];
    getWeightPointersBlock( block.width, block.height, layer_idx, weights_Matrix_Shared_Layer, weights_Bias_Shared_Layer );
    applyLayerEluInt( hiddenLayerOutput, input_curr2, weights_Matrix_Shared_Layer, weights_Bias_Shared_Layer, input_size, output_size, w, h, layer_idx );
    input_curr2 = hiddenLayerOutput;
  }

  rightShiftResultInt( hiddenLayerOutput, g_RightShifts_For_Last_Layer[w][h] );
}

void IntraPrediction_NN::computeJointHiddenOutputInt_Transpose( std::vector<int> &hiddenLayerOutput, const CPelBuf &src, const Area block, const int bitDepth, pos2Bool available )
{
  const int w = g_aucLog2[block.height]; // transposed
  const int h = g_aucLog2[block.width];  // transposed
  const int num_hidden_layers_curr = g_NumLayers_Block_NN[w][h] - 1;

  input_curr2.clear();

  const int sizeCtx = g_Context_Sizes_Block_NN[w][h];
  prepareInput_Transpose( input_curr2, src, block, available, bitDepth, sizeCtx );
  int* weights_Matrix_Shared_Layer;
  int* weights_Bias_Shared_Layer;
  for( int layer_idx = 0; layer_idx < num_hidden_layers_curr; layer_idx++ )
  {
    const int input_size = g_Architecture_Block_NN[w][h][layer_idx];
    const int output_size = g_Architecture_Block_NN[w][h][layer_idx + 1];
    getWeightPointersBlock( block.height, block.width, layer_idx, weights_Matrix_Shared_Layer, weights_Bias_Shared_Layer ); // transposed
    applyLayerEluInt( hiddenLayerOutput, input_curr2, weights_Matrix_Shared_Layer, weights_Bias_Shared_Layer, input_size, output_size, w, h, layer_idx );
    input_curr2 = hiddenLayerOutput;
  }

  rightShiftResultInt( hiddenLayerOutput, g_RightShifts_For_Last_Layer[w][h] );
}

void IntraPrediction_NN::computeModeList( std::vector<UInt>&modeList, const CPelBuf& src, const Area block, const int bitDepth, pos2Bool available )
{
  input_Data2.clear();

  const int w = g_aucLog2[block.width];
  const int h = g_aucLog2[block.height];
  getInputDataPad( input_Data2, src, block, available, bitDepth, g_Context_Sizes_Mode_NN[w][h] );

  loggits2.clear();

  computeLoggitsModesInt( loggits2, input_Data2, block.width, block.height );
  sortingPermutation(modeList, loggits2);
  CHECK(modeList.size() != g_numIntraModes_NN[w][h], "Error");
}

void IntraPrediction_NN::computeModeList_Transpose( std::vector<UInt>&modeList, const CPelBuf& src, const Area block, const int bitDepth, pos2Bool available )
{
  input_Data2.clear();

  const int w = g_aucLog2[block.height]; // transpose
  const int h = g_aucLog2[block.width];  // transpose
  prepareInput_Transpose( input_Data2, src, block, available, bitDepth, g_Context_Sizes_Mode_NN[w][h] );

  loggits2.clear();

  computeLoggitsModesInt( loggits2, input_Data2, block.height, block.width ); // transpose
  sortingPermutation( modeList, loggits2 );
  CHECK( modeList.size() != g_numIntraModes_NN[w][h], "Error: wrong mode list size" );
}

void IntraPrediction_NN::computeLoggitsModesInt( std::vector<int> &loggits, const std::vector<int> &input, const int width, const int height )
{
  const int w = g_aucLog2[width];
  const int h = g_aucLog2[height];

  input_curr3 = input;

  int* weights_Matrix;
  int* weights_Bias;
  for( int layer_idx = 0; layer_idx < g_NumLayers_Mode_NN[w][h]; layer_idx++ )
  {
    const int input_size  = g_Architecture_Mode_NN[w][h][layer_idx];
    const int output_size = g_Architecture_Mode_NN[w][h][layer_idx + 1];
    getWeightPointersMode( width, height, layer_idx, weights_Matrix, weights_Bias );
    applyLayerEluInt_Mode( loggits, input_curr3, weights_Matrix, weights_Bias, input_size, output_size, w, h, layer_idx );

    input_curr3 = loggits;
  }
}

template<class Vals>
void IntraPrediction_NN::sortingPermutation(std::vector<UInt>& v, const Vals& values)
{
  Int size = Int(values.size());
  v.clear();
  v.resize(size);
  for (int i = 0; i < size; ++i)
  {
    v[i] = ((UInt)i);
  }
  std::stable_sort(v.begin(), v.end(), [&values](int a, int b) -> bool { return values[b] < values[a]; });
}



template<typename T>
void IntraPrediction_NN::xDownSamplingRectangleFactorTwo(std::vector<T>& srcBuf, std::vector<T>& dstBuf, const Size& size)
{
  CHECK(size.area() != srcBuf.size(), "Error");

  Int SrcWidth = size.width;
  Int SrcHeight = size.height;
  Int SrcWidthHalf = SrcWidth >> 1;

  dstBuf.clear();
  dstBuf.resize((SrcWidth >> 1)*(SrcHeight >> 1));


  DownSamplingResultHor.clear();
  DownSamplingResultHor.resize(SrcWidthHalf*SrcHeight, 0);

  T* SrcForHor = &srcBuf[0];
  T* DstForHor = &DownSamplingResultHor[0];
  for( int y = 0; y < SrcHeight; y++ )
  {
    xOneDDownSamplingFactorTwo(SrcForHor, DstForHor, SrcWidth, 1, 1);
    SrcForHor += SrcWidth;
    DstForHor += SrcWidthHalf;
  }

  T *SrcForVer = &DownSamplingResultHor[0];
  T *DstForVer = &dstBuf[0];
  for( int x = 0; x < SrcWidthHalf; x++ )
  {
    xOneDDownSamplingFactorTwo(SrcForVer, DstForVer, SrcHeight, SrcWidthHalf, SrcWidthHalf);
    SrcForVer += 1;
    DstForVer += 1;
  }

  return;
}

template<typename T>
void IntraPrediction_NN::xOneDDownSamplingFactorTwo(T* srcPtr, T* dstPtr, Int SrcLength, Int SrcStride, Int DstStride)
{
  Int SrcLengthHalf = SrcLength >> 1;
  CHECK(2 * SrcLengthHalf != SrcLength, "Source size is not a multiple of two.");

  for( int i = 0; i < SrcLengthHalf - 1; i++ )
  {
    dstPtr[i*DstStride] = (srcPtr[2 * i*SrcStride] + 2 * srcPtr[(2 * i + 1)*SrcStride] + srcPtr[(2 * i + 2)*SrcStride]) / 4;
  }
  dstPtr[(SrcLengthHalf - 1)*DstStride] = (srcPtr[(SrcLength - 2)*SrcStride] + srcPtr[(SrcLength - 1)*SrcStride]) / 2;
}

template<typename T>
void IntraPrediction_NN::xLinInterpolRectangleFactorTwo(std::vector<T>& srcBuf, std::vector<T>& dstBuf, const Size& size_src)
{
  Int SrcWidth = size_src.width;
  Int SrcHeight = size_src.height;
  Int WidthInterpol = SrcWidth << 1;
  dstBuf.clear();
  dstBuf.resize((SrcWidth << 1)*(SrcHeight << 1));

  InterPolResultHor.clear();
  InterPolResultHor.resize(WidthInterpol*SrcHeight, 0);

  T* SrcForHor = &srcBuf[0];
  T* DstForHor = &InterPolResultHor[0];
  for (int y = 0; y < SrcHeight; y++) {
    xLinInterpolOneDFactorTwo(SrcForHor, DstForHor, SrcWidth, 1, 1);
    SrcForHor += SrcWidth;
    DstForHor += WidthInterpol;
  }

  T* SrcForVer = &InterPolResultHor[0];
  T* DstForVer = &dstBuf[0];
  for (int x = 0; x < WidthInterpol; x++) {
    xLinInterpolOneDFactorTwo(SrcForVer, DstForVer, SrcHeight, WidthInterpol, WidthInterpol);
    SrcForVer += 1;
    DstForVer += 1;
  }

  return;
}

template<typename T>
void IntraPrediction_NN::xLinInterpolOneDFactorTwo(T* srcPtr, T* dstPtr, Int SrcLength, Int SrcStride, Int DstStride)
{
  if (SrcLength >1) { dstPtr[0] = srcPtr[0] + (T)(double(srcPtr[0] - srcPtr[SrcStride]) / 2.0); }
  else              { dstPtr[0] = srcPtr[0]; }

  for( int i = 0; i < SrcLength; i++ )
  {
    dstPtr[(2 * i + 1)*DstStride] = srcPtr[i*SrcStride];
  }

  for( int i = 1; i < SrcLength; i++ )
  {
    dstPtr[2 * i * DstStride] = (T)(double(srcPtr[i * SrcStride] + srcPtr[(i - 1) * SrcStride]) / 2.0);
  }
}


