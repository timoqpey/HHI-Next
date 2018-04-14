/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2012, ITU/ISO/IEC
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

/** \file     IntraPredNNX86.h
    \brief    SIMD functions for intra NN modes.
*/

//! \ingroup CommonLib
//! \{


#include "CommonLib/CommonDef.h"
#include "CommonDefX86.h"
#include "CommonLib/IntraPrediction_NN.h"


#ifdef TARGET_SIMD_X86


#if defined _MSC_VER
#include <tmmintrin.h>
#else
#include <immintrin.h>
#endif

static void apply_Matrix_SIMD_base( int* output, const int* input, const short* weights_matrix, const int input_size, const int output_size, const int stride)
{
  CHECK(output_size % 4 != 0, "failed");
  const __m128i vzero = _mm_setzero_si128();

  int* dst = output;
  const short* weights = weights_matrix;
  for( int y = 0; y < output_size; y += 4 )
  {
    // compute 4 destination pixel values in parallel
    __m128i vval[4] = { vzero, vzero, vzero, vzero };
    for( int j = 0; j < input_size; j += 4 )
    {
      __m128i vweight[4];
      __m128i vsrc;
      // load input data and weights
      vsrc = _mm_loadu_si128((__m128i*)&input[j]);
      vweight[0] = _mm_loadu_si128((__m128i*)&weights[j]);
      vweight[1] = _mm_loadu_si128((__m128i*)&weights[j + stride]);
      vweight[2] = _mm_loadu_si128((__m128i*)&weights[j + 2 * stride]);
      vweight[3] = _mm_loadu_si128((__m128i*)&weights[j + 3 * stride]);

      // mul input data and weights
      vweight[0] = _mm_mullo_epi32(vsrc, _mm_cvtepi16_epi32(vweight[0]));
      vweight[1] = _mm_mullo_epi32(vsrc, _mm_cvtepi16_epi32(vweight[1]));
      vweight[2] = _mm_mullo_epi32(vsrc, _mm_cvtepi16_epi32(vweight[2]));
      vweight[3] = _mm_mullo_epi32(vsrc, _mm_cvtepi16_epi32(vweight[3]));

      // accumulate product of input data and weights
      vval[0] = _mm_add_epi32(vval[0], vweight[0]);
      vval[1] = _mm_add_epi32(vval[1], vweight[1]);
      vval[2] = _mm_add_epi32(vval[2], vweight[2]);
      vval[3] = _mm_add_epi32(vval[3], vweight[3]);
    }

    // horizontal accumulate
    vval[0] = _mm_hadd_epi32(vval[0], vval[1]);
    vval[2] = _mm_hadd_epi32(vval[2], vval[3]);
    vval[0] = _mm_hadd_epi32(vval[0], vval[2]);

    // store 4 destination pixels
    _mm_storeu_si128((__m128i*)&dst[y], vval[0]);

    weights += 4 * stride;
  }
}


static void apply_Matrix_SIMD_base( int* output, const int* input, const int* weights_matrix, const int input_size, const int output_size, const int stride )
{
  CHECK( output_size % 4 != 0, "failed" );

  const __m128i vzero = _mm_setzero_si128();

  int* dst = output;
  const int* weights = weights_matrix;
  for( int y = 0; y < output_size; y += 4 )
  {
    __m128i vval[4] = { vzero, vzero, vzero, vzero };
    for( int j = 0; j < input_size; j += 4 )
    {
      __m128i vweight[4];
      __m128i vsrc;
      // load input data and weights
      vsrc = _mm_loadu_si128( ( __m128i* )&input[j] );
      vweight[0] = _mm_loadu_si128( ( __m128i* )&weights[j] );
      vweight[1] = _mm_loadu_si128( ( __m128i* )&weights[j +     stride] );
      vweight[2] = _mm_loadu_si128( ( __m128i* )&weights[j + 2 * stride] );
      vweight[3] = _mm_loadu_si128( ( __m128i* )&weights[j + 3 * stride] );

      // mul input data and weights
      vweight[0] = _mm_mullo_epi32( vsrc, vweight[0] );
      vweight[1] = _mm_mullo_epi32( vsrc, vweight[1] );
      vweight[2] = _mm_mullo_epi32( vsrc, vweight[2] );
      vweight[3] = _mm_mullo_epi32( vsrc, vweight[3] );

      // accumulate product of input data and weights
      vval[0] = _mm_add_epi32( vval[0], vweight[0] );
      vval[1] = _mm_add_epi32( vval[1], vweight[1] );
      vval[2] = _mm_add_epi32( vval[2], vweight[2] );
      vval[3] = _mm_add_epi32( vval[3], vweight[3] );
    }

    // horizontal accumulate
    vval[0] = _mm_hadd_epi32( vval[0], vval[1] );
    vval[2] = _mm_hadd_epi32( vval[2], vval[3] );
    vval[0] = _mm_hadd_epi32( vval[0], vval[2] );


    // store 4 destination pixels
    _mm_storeu_si128( ( __m128i* )&dst[y], vval[0] );

    weights += 4 * stride;
  }
}

template<typename T>
static void applyMatrixSIMD( int* output, const int* input, const T* weights_matrix, const int input_size, const int output_size )
{
  if( input_size % 4 != 0 )
  {
    int upper_bound_input_size = 4 * (input_size / 4);
    apply_Matrix_SIMD_base( output, input, weights_matrix, upper_bound_input_size, output_size, input_size );
    // Do the rest
    for( int y = 0; y < output_size; y++ )
    {
      int val = 0;
      for( int i = upper_bound_input_size; i < input_size; i++ )
      {
        val += weights_matrix[y * input_size + i] * input[i];
      }
      output[y] += val;
    }
  }
  else
  {
    apply_Matrix_SIMD_base( output, input, weights_matrix, input_size, output_size, input_size );
  }
}

template<X86_VEXT vext>
void IntraPrediction_NN::_initIntraPredNNX86()
{
  switch( vext ) 
  {
    case AVX2:
      m_fApplyMatrixShort = applyMatrixSIMD;
      m_fApplyMatrixInt   = applyMatrixSIMD;
      break;
    case SSE41:
    case AVX:
      m_fApplyMatrixShort = applyMatrixSIMD;
      m_fApplyMatrixInt   = applyMatrixSIMD;
      break;
    default:
      CHECK( m_fApplyMatrixShort == nullptr, "scv" );
      CHECK( m_fApplyMatrixInt   == nullptr, "scv" );
  }
}

template void IntraPrediction_NN::_initIntraPredNNX86<SIMDX86>();

#endif



