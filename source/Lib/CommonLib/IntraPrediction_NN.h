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

/** \file     IntraPrediction_NN.h
 *  \brief    IntraPrediction_NN
 */

#ifndef __IntraPrediction_NN__
#define __IntraPrediction_NN__

// Include files
#include "Rom.h"
#include "Common.h"
#include "Unit.h"
#include "Buffer.h"


#include <functional>
#include <cmath>

typedef std::function<bool(Position)> pos2Bool;

class IntraPrediction_NN
{

public:
  IntraPrediction_NN();
  ~IntraPrediction_NN() {}

  // functions for computing the predicted block
  void computeModeList             ( std::vector<UInt> &modeList, const CPelBuf &src, const Area block, const int bitDepth, pos2Bool available );
  void computeModeList_Transpose   ( std::vector<UInt> &modeList, const CPelBuf &src, const Area block, const int bitDepth, pos2Bool available );
  void computeModeList_Downsampling( std::vector<UInt> &modeList, const CPelBuf &src, const Area block, const int bitDepth, pos2Bool available );

  void computeLastLayerInt             ( std::vector<int> &predSignal, const std::vector<int> &outputJointHiddenLayers, const int width, const int height, const int mode_idx, const int bitDepth );
  void computeLastLayerInt_Downsampling( std::vector<int> &predSignal, const std::vector<int> &outputJointHiddenLayers, const int width, const int height, const int mode_idx, const int bitDepth, const CPelBuf& src, const Area block, pos2Bool available );

  void computeJointHiddenOutputInt             ( std::vector<int> &hiddenLayerOutput, const CPelBuf &src, const Area block, const int bitDepth, pos2Bool available );
  void computeJointHiddenOutputInt_Transpose   ( std::vector<int> &hiddenLayerOutput, const CPelBuf &src, const Area block, const int bitDepth, pos2Bool available );
  void computeJointHiddenOutputInt_Downsampling( std::vector<int> &hiddenLayerOutput, const CPelBuf &src, const Area block, const int bitDepth, pos2Bool available );


private:
  // functions for generating the NN input 
  template<typename T>
  void getInputDataPad(std::vector<T>& data, const CPelBuf& src, const Area block, pos2Bool available, const int bitDepth, const int sizeCtx);
  template<typename T>
  void copyWithPadding(AreaBuf<T> dst, const CPelBuf& src, const Area srcArea, pos2Bool srcAvail, Pel defaultPadVal);
  template<typename T>
  void generatePaddedRectangle( AreaBuf<T> dst, const CPelBuf &src, const int extension_size, const Area rectArea, pos2Bool srcAvail, Pel defaultPadVal);
  template<typename T>
  void prepareInput_Transpose   ( std::vector<T> &input, const CPelBuf &src, const Area block, pos2Bool available, const int bitDepth, const int sizeCtx );
  template<typename T>
  void prepareInput_Downsampling( std::vector<T> &input, const CPelBuf &src, const Area block, pos2Bool available, const int bitDepth, const int sizeCtx );

  // function for computing loggits
  void computeLoggitsModesInt( std::vector<int> &loggits, const std::vector<int> &input, const int width, const int height );

  // functions for applying a layer
  void applyLayerEluInt( std::vector<int> &output, const std::vector<int> &input, 
                         const int* weights_matrix, const int* weights_bias,
                         const int input_size, const int output_size, 
                         const int w, const int h, const int layer_idx );

  void applyLayerInt( std::vector<int> &output, const std::vector<int> &input,
                      const short* weights_matrix, const int* weights_bias,
                      const int input_size, const int output_size,
                      const int output_width, const int output_height, const int padded_output_width,
                      const int w, const int h, const int mode_idx );

  void applyLayerEluInt_Mode( std::vector<int> &output, const std::vector<int> &input,
                              const int* weights_matrix, const int* weights_bias,
                              const int input_size, const int output_size,
                              const int w, const int h, const int layer_idx );

  void restrictPaddedOutput( std::vector<int> &output, const std::vector<int> &padded_output,
                             const int padded_output_width, const int padded_output_height,
                             const int output_width, const int output_height );

  void applyEluInt( std::vector<int> &output, const int w, const int h, const int layer_idx );


  template<typename T>
  void rightShiftResultInt ( std::vector<T> &inoutput, const int shift );
  template<typename T>
  void shiftOutputLastLayer( std::vector<T> &inoutput, const int bitDepth );

  template<typename T>
  void applyBias      ( std::vector<T> &inoutput, const T* weights_bias );
  template<typename T>
  void applyBiasPadded( std::vector<T> &inoutput, const T* weights_bias, const int output_width, const int output_height, const int padded_output_width );

  void applyMatrixPadded( std::vector<int> &output, const std::vector<int> &input, const short* weights_matrix, const int input_size, const int output_size, const int output_width, const int output_height, const int padded_output_width );

  void applyMatrixWithMask( std::vector<int> &output, const std::vector<int> &input, const short* weights_matrix, const int input_size, const int output_size, UChar* Mask, const int num_nonzero_rows );

  // sort function for mode list
  template<class Vals>
  void sortingPermutation(std::vector<UInt>& v, const Vals& values);

  // up and down sampling
  template<typename T>
  void xDownSamplingRectangleFactorTwo(std::vector<T>& srcBuf, std::vector<T>& dstBuf, const Size& size);
  template<typename T>
  void xLinInterpolRectangleFactorTwo(std::vector<T>& srcBuf, std::vector<T>& dstBuf, const Size& size);
  template<typename T>
  void xOneDDownSamplingFactorTwo(T* srcPtr, T* dstPtr, Int SrcLength, Int SrcStride, Int DstStride);
  template<typename T>
  void xLinInterpolOneDFactorTwo(T* srcPtr, T* dstPtr, Int SrcLength, Int SrcStride, Int DstStride);

  // Temporary variables
  std::vector<int> predSignalDS;
  std::vector<int> input_curr;
  std::vector<int> input_curr2;
  std::vector<int> input_curr3;

  std::vector<int> rectangle_above_for_DS;
  std::vector<int> rectangle_left_for_DS;
  std::vector<int> rectangle_above_downsampled;
  std::vector<int> rectangle_left_downsampled;
  std::vector<int> DownSamplingResultHor;
  std::vector<int> InterPolResultHor;

  std::vector<int> rectangle_above_for_TP;
  std::vector<int> rectangle_left_for_TP;

  std::vector<int> input_Data;
  std::vector<int> loggits;
  std::vector<int> input_Data2;
  std::vector<int> loggits2;

#ifdef TARGET_SIMD_X86
  void( *m_fApplyMatrixShort ) ( int* output, const int* input, const short* weights_matrix, const int input_size, const int output_size );
  void( *m_fApplyMatrixInt   ) ( int* output, const int* input, const int*   weights_matrix, const int input_size, const int output_size );

  template<X86_VEXT vext>
  void _initIntraPredNNX86();
  void initIntraPredNNX86();
#endif
};

template<typename T>
void applyMatrix( int* output, const int* input, const T* weights_matrix, const int input_size, const int output_size );

#endif