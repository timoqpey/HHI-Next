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

/**
 * \file
 * \brief Implementation of InterpolationFilter class
 */

// ====================================================================================================================
// Includes
// ====================================================================================================================

#include "Rom.h"
#include "InterpolationFilter.h"

#include "ChromaFormat.h"

//! \ingroup CommonLib
//! \{

// ====================================================================================================================
// Tables
// ====================================================================================================================

#if JEM_TOOLS
const TFilterCoeff InterpolationFilter::m_lumaFilter[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS << VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE][NTAPS_LUMA] =
#else
const TFilterCoeff InterpolationFilter::m_lumaFilter[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][NTAPS_LUMA] =
#endif
{
#if JEM_TOOLS
  {  0, 0,   0, 64,  0,   0,  0,  0 },
  {  0, 1,  -3, 63,  4,  -2,  1,  0 },
  { -1, 2,  -5, 62,  8,  -3,  1,  0 },
  { -1, 3,  -8, 60, 13,  -4,  1,  0 },
  { -1, 4, -10, 58, 17,  -5,  1,  0 },
  { -1, 4, -11, 52, 26,  -8,  3, -1 },
  { -1, 3,  -9, 47, 31, -10,  4, -1 },
  { -1, 4, -11, 45, 34, -10,  4, -1 },
  { -1, 4, -11, 40, 40, -11,  4, -1 },
  { -1, 4, -10, 34, 45, -11,  4, -1 },
  { -1, 4, -10, 31, 47,  -9,  3, -1 },
  { -1, 3,  -8, 26, 52, -11,  4, -1 },
  {  0, 1,  -5, 17, 58, -10,  4, -1 },
  {  0, 1,  -4, 13, 60,  -8,  3, -1 },
  {  0, 1,  -3,  8, 62,  -5,  2, -1 },
  {  0, 1,  -2,  4, 63,  -3,  1,  0 }
#else
  {  0, 0,   0, 64,  0,   0, 0,  0 },
  { -1, 4, -10, 58, 17,  -5, 1,  0 },
  { -1, 4, -11, 40, 40, -11, 4, -1 },
  {  0, 1,  -5, 17, 58, -10, 4, -1 }
#endif
};

#if JEM_TOOLS
const TFilterCoeff InterpolationFilter::m_chromaFilter[CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS << VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE][NTAPS_CHROMA] =
#else
const TFilterCoeff InterpolationFilter::m_chromaFilter[CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][NTAPS_CHROMA] =
#endif
{
#if JEM_TOOLS
  {  0, 64,  0,  0 },
  { -1, 63,  2,  0 },
  { -2, 62,  4,  0 },
  { -2, 60,  7, -1 },
  { -2, 58, 10, -2 },
  { -3, 57, 12, -2 },
  { -4, 56, 14, -2 },
  { -4, 55, 15, -2 },
  { -4, 54, 16, -2 },
  { -5, 53, 18, -2 },
  { -6, 52, 20, -2 },
  { -6, 49, 24, -3 },
  { -6, 46, 28, -4 },
  { -5, 44, 29, -4 },
  { -4, 42, 30, -4 },
  { -4, 39, 33, -4 },
  { -4, 36, 36, -4 },
  { -4, 33, 39, -4 },
  { -4, 30, 42, -4 },
  { -4, 29, 44, -5 },
  { -4, 28, 46, -6 },
  { -3, 24, 49, -6 },
  { -2, 20, 52, -6 },
  { -2, 18, 53, -5 },
  { -2, 16, 54, -4 },
  { -2, 15, 55, -4 },
  { -2, 14, 56, -4 },
  { -2, 12, 57, -3 },
  { -2, 10, 58, -2 },
  { -1,  7, 60, -2 },
  {  0,  4, 62, -2 },
  {  0,  2, 63, -1 },
#else
  {  0, 64,  0,  0 },
  { -2, 58, 10, -2 },
  { -4, 54, 16, -2 },
  { -6, 46, 28, -4 },
  { -4, 36, 36, -4 },
  { -4, 28, 46, -6 },
  { -2, 16, 54, -4 },
  { -2, 10, 58, -2 }
#endif
};

#if JEM_TOOLS
const TFilterCoeff InterpolationFilter::m_lumaFilterBilinear[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS << VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE][NTAPS_LUMA_FRUC] =
{
  { 64,  0, },
  { 60,  4, },
  { 56,  8, },
  { 52, 12, },
  { 48, 16, },
  { 44, 20, },
  { 40, 24, },
  { 36, 28, },
  { 32, 32, },
  { 28, 36, },
  { 24, 40, },
  { 20, 44, },
  { 16, 48, },
  { 12, 52, },
  {  8, 56, },
  {  4, 60, },
};
#endif
// ====================================================================================================================
// Private member functions
// ====================================================================================================================

InterpolationFilter::InterpolationFilter()
{
  m_filterHor[0][0][0] = filter<8, false, false, false>;
  m_filterHor[0][0][1] = filter<8, false, false, true>;
  m_filterHor[0][1][0] = filter<8, false, true, false>;
  m_filterHor[0][1][1] = filter<8, false, true, true>;

  m_filterHor[1][0][0] = filter<4, false, false, false>;
  m_filterHor[1][0][1] = filter<4, false, false, true>;
  m_filterHor[1][1][0] = filter<4, false, true, false>;
  m_filterHor[1][1][1] = filter<4, false, true, true>;

  m_filterHor[2][0][0] = filter<2, false, false, false>;
  m_filterHor[2][0][1] = filter<2, false, false, true>;
  m_filterHor[2][1][0] = filter<2, false, true, false>;
  m_filterHor[2][1][1] = filter<2, false, true, true>;

  m_filterVer[0][0][0] = filter<8, true, false, false>;
  m_filterVer[0][0][1] = filter<8, true, false, true>;
  m_filterVer[0][1][0] = filter<8, true, true, false>;
  m_filterVer[0][1][1] = filter<8, true, true, true>;

  m_filterVer[1][0][0] = filter<4, true, false, false>;
  m_filterVer[1][0][1] = filter<4, true, false, true>;
  m_filterVer[1][1][0] = filter<4, true, true, false>;
  m_filterVer[1][1][1] = filter<4, true, true, true>;

  m_filterVer[2][0][0] = filter<2, true, false, false>;
  m_filterVer[2][0][1] = filter<2, true, false, true>;
  m_filterVer[2][1][0] = filter<2, true, true, false>;
  m_filterVer[2][1][1] = filter<2, true, true, true>;

  m_filterCopy[0][0]   = filterCopy<false, false>;
  m_filterCopy[0][1]   = filterCopy<false, true>;
  m_filterCopy[1][0]   = filterCopy<true, false>;
  m_filterCopy[1][1]   = filterCopy<true, true>;

#if HHI_SIMD_OPT_MCIF
#ifdef TARGET_SIMD_X86
  initInterpolationFilterX86();
#endif
#endif
}


/**
 * \brief Apply unit FIR filter to a block of samples
 *
 * \param bitDepth   bitDepth of samples
 * \param src        Pointer to source samples
 * \param srcStride  Stride of source samples
 * \param dst        Pointer to destination samples
 * \param dstStride  Stride of destination samples
 * \param width      Width of block
 * \param height     Height of block
 * \param isFirst    Flag indicating whether it is the first filtering operation
 * \param isLast     Flag indicating whether it is the last filtering operation
 */
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// !!! NOTE !!!
//
//  This is the scalar version of the function.
//  If you change the functionality here, consider to switch off the SIMD implementation of this function.
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<Bool isFirst, Bool isLast>
Void InterpolationFilter::filterCopy( const ClpRng& clpRng, const Pel *src, Int srcStride, Pel *dst, Int dstStride, Int width, Int height )
{
  Int row, col;

  if ( isFirst == isLast )
  {
    for (row = 0; row < height; row++)
    {
      for (col = 0; col < width; col++)
      {
        dst[col] = ClipPel( src[col], clpRng );
      }

      src += srcStride;
      dst += dstStride;
    }
  }
  else if ( isFirst )
  {
    const Int shift = std::max<Int>(2, (IF_INTERNAL_PREC - clpRng.bd));

    for (row = 0; row < height; row++)
    {
      for (col = 0; col < width; col++)
      {
        Pel val = leftShift_round(src[col], shift);
        dst[col] = val - (Pel)IF_INTERNAL_OFFS;
      }

      src += srcStride;
      dst += dstStride;
    }
  }
  else
  {
    const Int shift = std::max<Int>(2, (IF_INTERNAL_PREC - clpRng.bd));

    for (row = 0; row < height; row++)
    {
      for (col = 0; col < width; col++)
      {
        Pel val = src[ col ];
        val = rightShift_round((val + IF_INTERNAL_OFFS), shift);

        dst[col] = ClipPel( val, clpRng );
      }

      src += srcStride;
      dst += dstStride;
    }
  }
}

/**
 * \brief Apply FIR filter to a block of samples
 *
 * \tparam N          Number of taps
 * \tparam isVertical Flag indicating filtering along vertical direction
 * \tparam isFirst    Flag indicating whether it is the first filtering operation
 * \tparam isLast     Flag indicating whether it is the last filtering operation
 * \param  bitDepth   Bit depth of samples
 * \param  src        Pointer to source samples
 * \param  srcStride  Stride of source samples
 * \param  dst        Pointer to destination samples
 * \param  dstStride  Stride of destination samples
 * \param  width      Width of block
 * \param  height     Height of block
 * \param  coeff      Pointer to filter taps
 */
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// !!! NOTE !!!
//
//  This is the scalar version of the function.
//  If you change the functionality here, consider to switch off the SIMD implementation of this function.
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<Int N, Bool isVertical, Bool isFirst, Bool isLast>
Void InterpolationFilter::filter(const ClpRng& clpRng, Pel const *src, Int srcStride, Pel *dst, Int dstStride, Int width, Int height, TFilterCoeff const *coeff)
{
  Int row, col;

  Pel c[8];
  c[0] = coeff[0];
  c[1] = coeff[1];
  if ( N >= 4 )
  {
    c[2] = coeff[2];
    c[3] = coeff[3];
  }
  if ( N >= 6 )
  {
    c[4] = coeff[4];
    c[5] = coeff[5];
  }
  if ( N == 8 )
  {
    c[6] = coeff[6];
    c[7] = coeff[7];
  }

  Int cStride = ( isVertical ) ? srcStride : 1;
  src -= ( N/2 - 1 ) * cStride;

  Int offset;
  Int headRoom = std::max<Int>(2, (IF_INTERNAL_PREC - clpRng.bd));
  Int shift    = IF_FILTER_PREC;
  // with the current settings (IF_INTERNAL_PREC = 14 and IF_FILTER_PREC = 6), though headroom can be
  // negative for bit depths greater than 14, shift will remain non-negative for bit depths of 8->20
  CHECK(shift < 0, "Negative shift");

  if ( isLast )
  {
    shift += (isFirst) ? 0 : headRoom;
    offset = 1 << (shift - 1);
    offset += (isFirst) ? 0 : IF_INTERNAL_OFFS << IF_FILTER_PREC;
  }
  else
  {
    shift -= (isFirst) ? headRoom : 0;
    offset = (isFirst) ? -IF_INTERNAL_OFFS << shift : 0;
  }

  for (row = 0; row < height; row++)
  {
    for (col = 0; col < width; col++)
    {
      Int sum;

      sum  = src[ col + 0 * cStride] * c[0];
      sum += src[ col + 1 * cStride] * c[1];
      if ( N >= 4 )
      {
        sum += src[ col + 2 * cStride] * c[2];
        sum += src[ col + 3 * cStride] * c[3];
      }
      if ( N >= 6 )
      {
        sum += src[ col + 4 * cStride] * c[4];
        sum += src[ col + 5 * cStride] * c[5];
      }
      if ( N == 8 )
      {
        sum += src[ col + 6 * cStride] * c[6];
        sum += src[ col + 7 * cStride] * c[7];
      }

      Pel val = ( sum + offset ) >> shift;
      if ( isLast )
      {
        val = ClipPel( val, clpRng );
      }
      dst[col] = val;
    }

    src += srcStride;
    dst += dstStride;
  }
}

/**
 * \brief Filter a block of samples (horizontal)
 *
 * \tparam N          Number of taps
 * \param  bitDepth   Bit depth of samples
 * \param  src        Pointer to source samples
 * \param  srcStride  Stride of source samples
 * \param  dst        Pointer to destination samples
 * \param  dstStride  Stride of destination samples
 * \param  width      Width of block
 * \param  height     Height of block
 * \param  isLast     Flag indicating whether it is the last filtering operation
 * \param  coeff      Pointer to filter taps
 */
template<Int N>
Void InterpolationFilter::filterHor(const ClpRng& clpRng, Pel const *src, Int srcStride, Pel *dst, Int dstStride, Int width, Int height, Bool isLast, TFilterCoeff const *coeff)
{
//#if HHI_SIMD_OPT_MCIF
  if( N == 8 )
  {
    m_filterHor[0][1][isLast]( clpRng, src, srcStride, dst, dstStride, width, height, coeff );
  }
  else if( N == 4 )
  {
    m_filterHor[1][1][isLast]( clpRng, src, srcStride, dst, dstStride, width, height, coeff );
  }
  else if( N == 2 )
  {
    m_filterHor[2][1][isLast]( clpRng, src, srcStride, dst, dstStride, width, height, coeff );
  }
  else
  {
    THROW( "Invalid tap number" );
  }
}

/**
 * \brief Filter a block of samples (vertical)
 *
 * \tparam N          Number of taps
 * \param  bitDepth   Bit depth
 * \param  src        Pointer to source samples
 * \param  srcStride  Stride of source samples
 * \param  dst        Pointer to destination samples
 * \param  dstStride  Stride of destination samples
 * \param  width      Width of block
 * \param  height     Height of block
 * \param  isFirst    Flag indicating whether it is the first filtering operation
 * \param  isLast     Flag indicating whether it is the last filtering operation
 * \param  coeff      Pointer to filter taps
 */
template<Int N>
Void InterpolationFilter::filterVer(const ClpRng& clpRng, Pel const *src, Int srcStride, Pel *dst, Int dstStride, Int width, Int height, Bool isFirst, Bool isLast, TFilterCoeff const *coeff)
{
//#if HHI_SIMD_OPT_MCIF
  if( N == 8 )
  {
    m_filterVer[0][isFirst][isLast]( clpRng, src, srcStride, dst, dstStride, width, height, coeff );
  }
  else if( N == 4 )
  {
    m_filterVer[1][isFirst][isLast]( clpRng, src, srcStride, dst, dstStride, width, height, coeff );
  }
  else if( N == 2 )
  {
    m_filterVer[2][isFirst][isLast]( clpRng, src, srcStride, dst, dstStride, width, height, coeff );
  }
  else{
    THROW( "Invalid tap number" );
  }
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

/**
 * \brief Filter a block of Luma/Chroma samples (horizontal)
 *
 * \param  compID     Chroma component ID
 * \param  src        Pointer to source samples
 * \param  srcStride  Stride of source samples
 * \param  dst        Pointer to destination samples
 * \param  dstStride  Stride of destination samples
 * \param  width      Width of block
 * \param  height     Height of block
 * \param  frac       Fractional sample offset
 * \param  isLast     Flag indicating whether it is the last filtering operation
 * \param  fmt        Chroma format
 * \param  bitDepth   Bit depth
 */
#if JEM_TOOLS
Void InterpolationFilter::filterHor( const ComponentID compID, Pel const *src, Int srcStride, Pel *dst, Int dstStride, Int width, Int height, Int frac, Bool isLast, const ChromaFormat fmt, const ClpRng& clpRng, Int nFilterIdx )
#else
Void InterpolationFilter::filterHor( const ComponentID compID, Pel const *src, Int srcStride, Pel *dst, Int dstStride, Int width, Int height, Int frac, Bool isLast, const ChromaFormat fmt, const ClpRng& clpRng )
#endif
{
  if( frac == 0 )
  {
    m_filterCopy[true][isLast]( clpRng, src, srcStride, dst, dstStride, width, height );
  }
  else if( isLuma( compID ) )
  {
#if JEM_TOOLS
    CHECK( frac < 0 || frac >= ( LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS << VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE ), "Invalid fraction" );
#else
    CHECK( frac < 0 || frac >= ( LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS ), "Invalid fraction" );
#endif
#if JEM_TOOLS
    if( nFilterIdx == 1 )
    {
      filterHor<NTAPS_LUMA_FRUC>( clpRng, src, srcStride, dst, dstStride, width, height, isLast, m_lumaFilterBilinear[frac] );
    }
    else
#endif
    {
      filterHor<NTAPS_LUMA>( clpRng, src, srcStride, dst, dstStride, width, height, isLast, m_lumaFilter[frac] );
    }
  }
  else
  {
    const UInt csx = getComponentScaleX( compID, fmt );
#if JEM_TOOLS
    CHECK( frac < 0 || csx >= 2 || ( frac << ( 1 - csx ) ) >= ( CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS << VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE ), "Invalid fraction" );
#else
    CHECK( frac < 0 || csx >= 2 || ( frac << ( 1 - csx ) ) >= ( CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS ), "Invalid fraction" );
#endif
    filterHor<NTAPS_CHROMA>( clpRng, src, srcStride, dst, dstStride, width, height, isLast, m_chromaFilter[frac << ( 1 - csx )] );
  }
}


/**
 * \brief Filter a block of Luma/Chroma samples (vertical)
 *
 * \param  compID     Colour component ID
 * \param  src        Pointer to source samples
 * \param  srcStride  Stride of source samples
 * \param  dst        Pointer to destination samples
 * \param  dstStride  Stride of destination samples
 * \param  width      Width of block
 * \param  height     Height of block
 * \param  frac       Fractional sample offset
 * \param  isFirst    Flag indicating whether it is the first filtering operation
 * \param  isLast     Flag indicating whether it is the last filtering operation
 * \param  fmt        Chroma format
 * \param  bitDepth   Bit depth
 */
#if JEM_TOOLS
Void InterpolationFilter::filterVer( const ComponentID compID, Pel const *src, Int srcStride, Pel *dst, Int dstStride, Int width, Int height, Int frac, Bool isFirst, Bool isLast, const ChromaFormat fmt, const ClpRng& clpRng, Int nFilterIdx )
#else
Void InterpolationFilter::filterVer( const ComponentID compID, Pel const *src, Int srcStride, Pel *dst, Int dstStride, Int width, Int height, Int frac, Bool isFirst, Bool isLast, const ChromaFormat fmt, const ClpRng& clpRng )
#endif
{
  if( frac == 0 )
  {
    m_filterCopy[isFirst][isLast]( clpRng, src, srcStride, dst, dstStride, width, height );
  }
  else if( isLuma( compID ) )
  {
#if JEM_TOOLS
    CHECK( frac < 0 || frac >= ( LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS << VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE ), "Invalid fraction" );
#else
    CHECK( frac < 0 || frac >= ( LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS ), "Invalid fraction" );
#endif
#if JEM_TOOLS
    if( nFilterIdx == 1 )
    {
      filterVer<NTAPS_LUMA_FRUC>( clpRng, src, srcStride, dst, dstStride, width, height, isFirst, isLast, m_lumaFilterBilinear[frac] );
    }
    else
#endif
    {
      filterVer<NTAPS_LUMA>( clpRng, src, srcStride, dst, dstStride, width, height, isFirst, isLast, m_lumaFilter[frac] );
    }
  }
  else
  {
    const UInt csy = getComponentScaleY( compID, fmt );
#if JEM_TOOLS
    CHECK( frac < 0 || csy >= 2 || ( frac << ( 1 - csy ) ) >= ( CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS << VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE ), "Invalid fraction" );
#else
    CHECK( frac < 0 || csy >= 2 || ( frac << ( 1 - csy ) ) >= ( CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS ), "Invalid fraction" );
#endif
    filterVer<NTAPS_CHROMA>( clpRng, src, srcStride, dst, dstStride, width, height, isFirst, isLast, m_chromaFilter[frac << ( 1 - csy )] );
  }
}

//! \}
