/* The copyright in this software is being made available under the BSD
* License, included below. This software may be subject to other third party
* and contributor rights, including patent rights, and no such rights are
* granted under this license.
*
* Copyright (c) 2010-2016, ITU/ISO/IEC
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

/** \file     Thresholding.cpp
    \brief    Thresholding implementation
*/

#include "Thresholding.h"
#include "dtrace_codingstruct.h"
#include "dtrace_buffer.h"
#include "Quant.h"

#if THRESHOLDING
inline int getNonLog2Factor( const SizeType& size )
{
  return ( size / ( 1 << ( g_aucLog2OfPowerOf2Part[ size ] ) ) );
}

Thresholding::Thresholding()
  : m_extThrsBuffer   ( MAX_TR_SIZE * MAX_TR_SIZE )
  , m_extThrsDctCoeffs( MAX_TR_SIZE * MAX_TR_SIZE )
{
}

Thresholding::~Thresholding()
{
}

Void Thresholding::init( const SPS* sps )
{
  m_bitDepth             [ 0 ] = sps->getBitDepth             ( CHANNEL_TYPE_LUMA   );
  m_maxLog2TrDynamicRange[ 0 ] = sps->getMaxLog2TrDynamicRange( CHANNEL_TYPE_LUMA   );
  m_bitDepth             [ 1 ] = sps->getBitDepth             ( CHANNEL_TYPE_CHROMA );
  m_maxLog2TrDynamicRange[ 1 ] = sps->getMaxLog2TrDynamicRange( CHANNEL_TYPE_CHROMA );
}

bool Thresholding::allowThresholding( const TransformUnit& tu )
{
  bool thresholding = true;

  if( tu.cu->mode1dPartitions != NO_1D_PARTITION )
  {
    thresholding = false;
  }

  return thresholding;
}

Void Thresholding::applyThresholding( const ComponentID compID, TransformUnit& tu, PelBuf& piPred )
{
  CHECK( !tu.thresholding, "Thresholding not enabled but applied" )
  CHECK( std::max( tu.blocks[ compID ].height, tu.blocks[ compID ].width ) > MAX_CU_SIZE, "TU is larger than maximum allowed size" );

  const Size finalSize = xGetTrafoSize( tu.blocks[ compID ], tu.thresholdingSize );

  CHECK( finalSize.area() == 0, "invalid transform size" );
  CHECK( std::max( finalSize.height, finalSize.width ) > MAX_TR_SIZE, "transform size larger than supported" );

  PelBuf thrsSavedBuffer( m_thrsPelBuffer[ tu.thresholdingThrs ], tu.blocks[ compID ] );
  piPred.copyFrom( thrsSavedBuffer );
}

UInt Thresholding::getNumberValidThrs( const ComponentID compID, TransformUnit& tu, PelBuf& piPred )
{
  CHECK( !tu.thresholding, "Thresholding not enabled but applied" )
  CHECK( std::max( tu.blocks[ compID ].height, tu.blocks[ compID ].width ) > MAX_CU_SIZE, "TU is larger than maximum allowed size" );

  const Size finalSize = xGetTrafoSize( tu.blocks[ compID ], tu.thresholdingSize );

  CHECK( finalSize.area() == 0, "invalid transform size" );
  CHECK( std::max( finalSize.height, finalSize.width ) > MAX_TR_SIZE, "transform size larger than supported" );

  m_candThrsVals    .clear();
  m_extThrsBuffer   .resize( finalSize.area() );
  m_extThrsDctCoeffs.resize( finalSize.area() );

  xGenerateThrs( compID, tu, piPred );

  PelBuf thrsSavedBuffer( m_thrsPelBuffer[ tu.thresholdingThrs ], tu.blocks[ compID ] );

  xThresholding( compID, tu, finalSize );
  xIDCT( compID, tu, thrsSavedBuffer, finalSize );

  return (UInt)m_candThrsVals.size();
}

UInt Thresholding::getNumberValidSize( const Area& area, const SliceType sliceType, const ChannelType chanType, const SPSNext* sps )
{
  if( !sps->getUseThresholding() || chanType != CHANNEL_TYPE_LUMA )
  {
    return 0;
  }

  Size     prevSize;
  unsigned numAllowedSizes      = 0;
  unsigned maxThresholdingSizes = sps->getThresholdingMaxSize( sliceType );

  if( area.width < 16 && area.height < 16 )
  {
    return 0;
  }

  if( area.width != area.height )
  {
    return 0;
  }

  for( unsigned thresholdingSize = 0; thresholdingSize < maxThresholdingSizes; thresholdingSize++ )
  {
    const Size finalSize = xGetTrafoSize( area, thresholdingSize );

    const int deltaWdt  = finalSize.width  - area.width;
    const int deltaHgt  = finalSize.height - area.height;
    const int trafoPosX = area.topLeft().x - deltaWdt;
    const int trafoPosY = area.topLeft().y - deltaHgt;

    if( !( trafoPosX < 0 || trafoPosY < 0 ) && prevSize != finalSize )
    {
      numAllowedSizes++;
      prevSize = finalSize;
    }
  }

  return numAllowedSizes;
}

Void Thresholding::xGenerateThrs( const ComponentID compID, TransformUnit& tu, PelBuf& piPred )
{
  const SliceType sliceType = tu.cs->slice->getSliceType();
  const Size      finalSize = xGetTrafoSize( tu.blocks[ compID ], tu.thresholdingSize );
        unsigned  maxThrs   = tu.cs->sps->getSpsNext().getThresholdingMaxThrs( sliceType );

  m_candThrsVals    .clear();
  m_extThrsBuffer   .resize( finalSize.area() );
  m_extThrsDctCoeffs.resize( finalSize.area() );

  xGenerateDCT( compID, tu, piPred, finalSize );

  const unsigned qStepSize         = ( tu.cs->slice->getSliceQpBase() / 2 );
  const int      bitDepth          = m_bitDepth[ (int)toChannelType( compID ) ];
  const TCoeff   scaleForThreshold = qStepSize * (TCoeff)floor( ( 1 << ( bitDepth - 8 ) ) * 32 * sqrt( 1. / finalSize.area() ) + .5 );

  for( int i = 1; i < finalSize.area(); i++ )
  {
    const unsigned threshold = abs( m_extThrsDctCoeffs[ i ] );
    const unsigned thrsScale = threshold / scaleForThreshold;

    if( 0 != thrsScale )
    {
      m_candThrsVals.push_back( thrsScale );
    }
  }

  std::sort   ( m_candThrsVals.begin(), m_candThrsVals.end() );
  std::reverse( m_candThrsVals.begin(), m_candThrsVals.end() );

  m_candThrsVals.erase( std::unique( m_candThrsVals.begin(), m_candThrsVals.end() ), m_candThrsVals.end() );

  std::reverse( m_candThrsVals.begin(), m_candThrsVals.end() );

  if( maxThrs < (unsigned)m_candThrsVals.size() )
  {
    m_candThrsVals.resize( maxThrs );
  }

  for( int i = 0; i < (int)m_candThrsVals.size(); i++ )
  {
    m_candThrsVals[ i ] *= scaleForThreshold;
  }
}

Size Thresholding::xGetTrafoSize( const Area& area, const unsigned sizeIdx )
{
  Size finalSize;
  const unsigned numTrafoSupported = 18;
  const unsigned thresholdingSizes[ numTrafoSupported ] = { 2, 4, 6, 8, 10, 12, 16, 20, 24, 32, 40, 48, 64, 80, 96, 128, 160, 192 };

  for( unsigned i = 0; i < numTrafoSupported; i++ )
  {
    if( thresholdingSizes[ i ] == area.width )
    {
      finalSize.width  = i;
    }

    if( thresholdingSizes[ i ] == area.height )
    {
      finalSize.height = i;
    }
  }

  finalSize.width  = thresholdingSizes[ std::min( finalSize.width  + sizeIdx + 1, numTrafoSupported - 1 ) ];
  finalSize.height = thresholdingSizes[ std::min( finalSize.height + sizeIdx + 1, numTrafoSupported - 1 ) ];

  return finalSize;
}

Void Thresholding::xGenerateDCT( const ComponentID compID, TransformUnit& tu, const PelBuf& piPred, const Size& finalSize )
{
  CHECK( std::max( finalSize.height, finalSize.width ) > MAX_TR_SIZE, "extended TU larger than maximum allowed size" );

  const Bool applyDST = isLuma( compID ) && tu.cu->predMode == MODE_INTRA;

  const Int offsetX = finalSize.width  - piPred.width;
  const Int offsetY = finalSize.height - piPred.height;
  CPelBuf recBorder = tu.cs->picture->getRecoBuf( CompArea( compID, tu.cu->chromaFormat, tu.blocks[ compID ].offset( -offsetX, -offsetY ), finalSize ) );

  xCopyTopLeftBorder( recBorder, m_extThrsBuffer, finalSize, offsetX, offsetY );
  xCopyFromPelBuf   ( piPred,    m_extThrsBuffer, finalSize, offsetX, offsetY );
  xFwdTransform     ( compID, &m_extThrsBuffer[ 0 ], &m_extThrsDctCoeffs[ 0 ], finalSize, applyDST );
}

Void Thresholding::xThresholding( const ComponentID compID, TransformUnit& tu, const Size& finalSize )
{
  const UInt   finalArea = finalSize.area();
  const TCoeff threshold = m_candThrsVals[ tu.thresholdingThrs ] + 1;

  for( unsigned i = 0; i < finalArea; i++ )
  {
    if( abs( m_extThrsDctCoeffs[ i ] ) < threshold )
    {
      m_extThrsDctCoeffs[ i ] = 0;
    }
  }
}

Void Thresholding::xIDCT( const ComponentID compID, TransformUnit& tu, PelBuf& thrPrdBuf, const Size& finalSize )
{
  const Bool applyDST = tu.cu->predMode == MODE_INTRA && isLuma( compID );

  xInvTransform( compID, &m_extThrsDctCoeffs[ 0 ], &m_extThrsBuffer[ 0 ], finalSize, applyDST );

  const Int offsetX = finalSize.width  - thrPrdBuf.width;
  const Int offsetY = finalSize.height - thrPrdBuf.height;

  xCopyFromVector( compID, m_extThrsBuffer, thrPrdBuf, finalSize, offsetX, offsetY );
}

Void Thresholding::xCopyTopLeftBorder( const CPelBuf& srcBorder, std::vector<Pel>& buf, const Size& size, const int offsetLeft, const int offsetTop )
{
  CHECK( size.area() != buf.size(), "input size does not match size of buf in copyTopLeftBorder" );

  Int width  = size.width;
  Int height = size.height;

  CHECK( width  != srcBorder.width,  "Incompatible width in copyTopLeftBorder"  );
  CHECK( height != srcBorder.height, "Incompatible height in copyTopLeftBorder" );

  UInt dstStride = width;

  const Pel* srcData   = srcBorder.bufAt( 0, 0 );
  const UInt srcStride = srcBorder.stride;

  // top
  for( int y = 0; y < offsetTop; y++, srcData += srcStride )
  {
    for( int x = 0; x < width; x++ )
    {
      buf[ x + y*dstStride ] = *( srcData + x );
    }
  }

  // left
  srcData = srcBorder.bufAt( 0, offsetTop );
  for( int y = offsetTop; y < height; y++, srcData += srcStride )
  {
    for( int x = 0; x < offsetLeft; x++ )
    {
      buf[ x + y*dstStride ] = *( srcData + x );
    }
  }
}

Void Thresholding::xCopyFromPelBuf( const PelBuf& orgPrdBuf, std::vector<Pel>& buf, const Size& size, const int offsetLeft, const int offsetTop )
{
  CHECK( size.area() != buf.size(), "input size does not match size of buf in copyFromPel" );

  Int width  = size.width;
  Int height = size.height;

  CHECK( width - offsetLeft != orgPrdBuf.width,  "Incompatible width in copyFromPel"  );
  CHECK( height - offsetTop != orgPrdBuf.height, "Incompatible height in copyFromPel" );

  UInt dstStride = width;

  const Pel* orgData = orgPrdBuf.bufAt( 0, 0 );
  const UInt orgStride = orgPrdBuf.stride;

  for( UInt y = offsetTop; y < height; y++, orgData += orgStride )
  {
    for( UInt x = offsetLeft; x < width; x++ )
    {
      buf[ x + y*dstStride ] = *( orgData + x - offsetLeft );
    }
  }
}

Void Thresholding::xCopyFromVector( const ComponentID compID, const std::vector<Pel>& buf, PelBuf& thrPrdBuf, const Size& size, const int offsetLeft, const int offsetTop )
{
  CHECK( size.area() != buf.size(), "input size does not match size of buf in copyFromFloat" );

  const int width    = size.width;
  const int height   = size.height;
  const int bitDepth = m_bitDepth[ (int)toChannelType( compID ) ];

  CHECK( width - offsetLeft != thrPrdBuf.width,  "Incompatible width in copyFromFloat"  );
  CHECK( height - offsetTop != thrPrdBuf.height, "Incompatible height in copyFromFloat" );

  UInt dstStride = width;

  Pel* thrData = thrPrdBuf.bufAt( 0, 0 );
  const UInt thrStride = thrPrdBuf.stride;

  for( UInt y = offsetTop; y < height; y++, thrData += thrStride )
  {
    for( UInt x = offsetLeft; x < width; x++ )
    {
      *( thrData + x - offsetLeft ) = ClipBD<Int>( buf[ x + y*dstStride ], bitDepth );
    }
  }
}

Void Thresholding::xFwdTransform( const ComponentID compID, const Pel *srcBuf, TCoeff *dstBuf, const Size& size, const Bool useDST )
{
  const int iWidth  = (int)size.width;
  const int iHeight = (int)size.height;

  const int bitDepth               = m_bitDepth             [ (int)toChannelType( compID ) ];
  const int maxLog2TrDynamicRange  = m_maxLog2TrDynamicRange[ (int)toChannelType( compID ) ];
  const int TRANSFORM_MATRIX_SHIFT = g_transformMatrixShift[ TRANSFORM_FORWARD ];

  const Int shift_1st = ( g_aucLog2OfPowerOf2Part[ iWidth ] + g_aucCeilOfLog2OfNonPowerOf2Part[ iWidth ] + bitDepth + TRANSFORM_MATRIX_SHIFT ) - maxLog2TrDynamicRange + ( isNonLog2Size( iWidth ) ? COM16_C806_TRANS_PREC : 0 );
  const Int shift_2nd = g_aucLog2OfPowerOf2Part[ iHeight ] + g_aucCeilOfLog2OfNonPowerOf2Part[ iHeight ] + TRANSFORM_MATRIX_SHIFT + ( isNonLog2Size( iHeight ) ? COM16_C806_TRANS_PREC : 0 );

  Int iSkipHeight = 0;
  Int iSkipWidth = ( iWidth > JVET_C0024_ZERO_OUT_TH ) ? ( iWidth - JVET_C0024_ZERO_OUT_TH ) : 0;
  if( iWidth > 128 )
  {
    iSkipWidth = 32;
  }
  else if( iHeight > 128 )
  {
    iSkipWidth = iSkipWidth / 2;
  }

  CHECK( shift_1st < 0, "Negative shift" );
  CHECK( shift_2nd < 0, "Negative shift" );

  ALIGN_DATA( MEMORY_ALIGN_DEF_SIZE, TCoeff block[ MAX_TR_SIZE * MAX_TR_SIZE ] );
  ALIGN_DATA( MEMORY_ALIGN_DEF_SIZE, TCoeff   tmp[ MAX_TR_SIZE * MAX_TR_SIZE ] );

  for( Int y = 0; y < iHeight; y++ )
  {
    for( Int x = 0; x < iWidth; x++ )
    {
      block[ ( y * iWidth ) + x ] = srcBuf[ ( y * iWidth ) + x ];
    }
  }

  switch( iWidth )
  {
  case 2: fastForwardDCT2_B2( block, tmp, shift_1st, iHeight, iSkipHeight, iSkipWidth, 0 ); break;
  case 4:
  {
    if( ( iHeight == 4 ) && useDST )    // Check for DCT or DST
    {
      fastForwardDST7_B4( block, tmp, shift_1st, iHeight, iSkipHeight, iSkipWidth, 0 );
    }
    else
    {
      fastForwardDCT2_B4( block, tmp, shift_1st, iHeight, iSkipHeight, iSkipWidth, 0 );
    }
  }
  break;

  case   8: fastForwardDCT2_B8  ( block, tmp, shift_1st, iHeight, iSkipHeight, iSkipWidth, 0 ); break;
  case  16: fastForwardDCT2_B16 ( block, tmp, shift_1st, iHeight, iSkipHeight, iSkipWidth, 0 ); break;
  case  32: fastForwardDCT2_B32 ( block, tmp, shift_1st, iHeight, iSkipHeight, iSkipWidth, 0 ); break;
  case  64: fastForwardDCT2_B64 ( block, tmp, shift_1st + COM16_C806_TRANS_PREC, iHeight, iSkipHeight, iSkipWidth, 0 ); break;
  case 128: fastForwardDCT2_B128( block, tmp, shift_1st + COM16_C806_TRANS_PREC, iHeight, iSkipHeight, iSkipWidth, 0 ); break;
  case   6: fastForwardDCT2_B6  ( block, tmp, shift_1st, iHeight, iSkipHeight, iSkipWidth, 1 ); break;
  case  10: fastForwardDCT2_B10 ( block, tmp, shift_1st, iHeight, iSkipHeight, iSkipWidth, 1 ); break;
  case  12: fastForwardDCT2_B12 ( block, tmp, shift_1st, iHeight, iSkipHeight, iSkipWidth, 1 ); break;
  case  20: fastForwardDCT2_B20 ( block, tmp, shift_1st, iHeight, iSkipHeight, iSkipWidth, 1 ); break;
  case  24: fastForwardDCT2_B24 ( block, tmp, shift_1st, iHeight, iSkipHeight, iSkipWidth, 1 ); break;
  case  40: fastForwardDCT2_B40 ( block, tmp, shift_1st, iHeight, iSkipHeight, iSkipWidth, 1 ); break;
  case  48: fastForwardDCT2_B48 ( block, tmp, shift_1st, iHeight, iSkipHeight, iSkipWidth, 1 ); break;
  case  80: fastForwardDCT2_B80 ( block, tmp, shift_1st, iHeight, iSkipHeight, iSkipWidth, 1 ); break;
  case  96: fastForwardDCT2_B96 ( block, tmp, shift_1st, iHeight, iSkipHeight, iSkipWidth, 1 ); break;
  case 160: fastForwardDCT2_B160( block, tmp, shift_1st, iHeight, iSkipHeight, iSkipWidth, 1 ); break;
  case 192: fastForwardDCT2_B192( block, tmp, shift_1st, iHeight, iSkipHeight, iSkipWidth, 1 ); break;
  default:
    THROW( "Unsupported transformation size" ); break;
  }

  iSkipHeight = ( iHeight > JVET_C0024_ZERO_OUT_TH ) ? ( iHeight - JVET_C0024_ZERO_OUT_TH ) : 0;
  if( iHeight > 128 )
  {
    iSkipHeight = 32;
  }
  else if( iWidth > 128 )
  {
    iSkipHeight = iSkipHeight / 2;
  }

  switch( iHeight )
  {
  case 2: fastForwardDCT2_B2( tmp, dstBuf, shift_2nd, iWidth, iSkipWidth, iSkipHeight, 0 ); break;
  case 4:
  {
    if( ( iWidth == 4 ) && useDST )    // Check for DCT or DST
    {
      fastForwardDST7_B4( tmp, dstBuf, shift_2nd, iWidth, iSkipWidth, iSkipHeight, 0 );
    }
    else
    {
      fastForwardDCT2_B4( tmp, dstBuf, shift_2nd, iWidth, iSkipWidth, iSkipHeight, 0 );
    }
  }
  break;

  case   8: fastForwardDCT2_B8  ( tmp, dstBuf, shift_2nd, iWidth, iSkipWidth, iSkipHeight, 0 ); break;
  case  16: fastForwardDCT2_B16 ( tmp, dstBuf, shift_2nd, iWidth, iSkipWidth, iSkipHeight, 0 ); break;
  case  32: fastForwardDCT2_B32 ( tmp, dstBuf, shift_2nd, iWidth, iSkipWidth, iSkipHeight, 0 ); break;
  case  64: fastForwardDCT2_B64 ( tmp, dstBuf, shift_2nd + COM16_C806_TRANS_PREC, iWidth, iSkipWidth, iSkipHeight, 0 ); break;
  case 128: fastForwardDCT2_B128( tmp, dstBuf, shift_2nd + COM16_C806_TRANS_PREC, iWidth, iSkipWidth, iSkipHeight, 0 ); break;
  case   6: fastForwardDCT2_B6  ( tmp, dstBuf, shift_2nd, iWidth, iSkipWidth, iSkipHeight, 1 ); break;
  case  10: fastForwardDCT2_B10 ( tmp, dstBuf, shift_2nd, iWidth, iSkipWidth, iSkipHeight, 1 ); break;
  case  12: fastForwardDCT2_B12 ( tmp, dstBuf, shift_2nd, iWidth, iSkipWidth, iSkipHeight, 1 ); break;
  case  20: fastForwardDCT2_B20 ( tmp, dstBuf, shift_2nd, iWidth, iSkipWidth, iSkipHeight, 1 ); break;
  case  24: fastForwardDCT2_B24 ( tmp, dstBuf, shift_2nd, iWidth, iSkipWidth, iSkipHeight, 1 ); break;
  case  40: fastForwardDCT2_B40 ( tmp, dstBuf, shift_2nd, iWidth, iSkipWidth, iSkipHeight, 1 ); break;
  case  48: fastForwardDCT2_B48 ( tmp, dstBuf, shift_2nd, iWidth, iSkipWidth, iSkipHeight, 1 ); break;
  case  80: fastForwardDCT2_B80 ( tmp, dstBuf, shift_2nd, iWidth, iSkipWidth, iSkipHeight, 1 ); break;
  case  96: fastForwardDCT2_B96 ( tmp, dstBuf, shift_2nd, iWidth, iSkipWidth, iSkipHeight, 1 ); break;
  case 160: fastForwardDCT2_B160( tmp, dstBuf, shift_2nd, iWidth, iSkipWidth, iSkipHeight, 1 ); break;
  case 192: fastForwardDCT2_B192( tmp, dstBuf, shift_2nd, iWidth, iSkipWidth, iSkipHeight, 1 ); break;
  default:
    THROW( "Unsupported transformation size" ); break;
  }
}

Void Thresholding::xInvTransform( const ComponentID compID, const TCoeff *srcBuf, Pel *dstBuf, const Size& size, const Bool useDST )
{
  const Int TRANSFORM_MATRIX_SHIFT = g_transformMatrixShift[ TRANSFORM_INVERSE ];
  const int iWidth  = (int)size.width;
  const int iHeight = (int)size.height;

  const int bitDepth              = m_bitDepth             [ (int)toChannelType( compID ) ];
  const int maxLog2TrDynamicRange = m_maxLog2TrDynamicRange[ (int)toChannelType( compID ) ];

  Int shift_1st = TRANSFORM_MATRIX_SHIFT + g_aucCeilOfLog2OfNonPowerOf2Part[ iHeight ] + 1 + ( isNonLog2Size( iHeight ) ? COM16_C806_TRANS_PREC : 0 ); //1 has been added to shift_1st at the expense of shift_2nd
  Int shift_2nd = ( TRANSFORM_MATRIX_SHIFT + g_aucCeilOfLog2OfNonPowerOf2Part[ iWidth ] + maxLog2TrDynamicRange - 1 ) - bitDepth + ( isNonLog2Size( iWidth ) ? COM16_C806_TRANS_PREC : 0 );

  const TCoeff clipMinimum = -( 1 << maxLog2TrDynamicRange );
  const TCoeff clipMaximum =  ( 1 << maxLog2TrDynamicRange ) - 1;
  TCoeff reScale = getNonLog2Factor( iWidth ) * getNonLog2Factor( iHeight );

  Int iSkipWidth  = ( iWidth > JVET_C0024_ZERO_OUT_TH )  ? ( iWidth - JVET_C0024_ZERO_OUT_TH )  : 0;
  Int iSkipHeight = ( iHeight > JVET_C0024_ZERO_OUT_TH ) ? ( iHeight - JVET_C0024_ZERO_OUT_TH ) : 0;

  if( iHeight > 128 )
  {
    iSkipHeight = 32;
  }
  else if( iWidth > 128 )
  {
    iSkipHeight = iSkipHeight / 2;
  }

  if( iWidth > 128 )
  {
    iSkipWidth = 32;
  }
  else if( iHeight > 128 )
  {
    iSkipWidth = iSkipWidth / 2;
  }

  CHECK( shift_1st < 0, "Negative shift" );
  CHECK( shift_2nd < 0, "Negative shift" );

  ALIGN_DATA( MEMORY_ALIGN_DEF_SIZE, TCoeff block[ MAX_TR_SIZE * MAX_TR_SIZE ] );
  ALIGN_DATA( MEMORY_ALIGN_DEF_SIZE, TCoeff   tmp[ MAX_TR_SIZE * MAX_TR_SIZE ] );

  switch( iHeight )
  {
  case 2: fastInverseDCT2_B2( srcBuf, tmp, shift_1st, iWidth, iSkipWidth, iSkipHeight, 0, clipMinimum, clipMaximum ); break;
  case 4:
  {
    if( ( iWidth == 4 ) && useDST )    // Check for DCT or DST
    {
      fastInverseDST7_B4( srcBuf, tmp, shift_1st, iWidth, iSkipWidth, iSkipHeight, 0, clipMinimum, clipMaximum );
    }
    else
    {
      fastInverseDCT2_B4( srcBuf, tmp, shift_1st, iWidth, iSkipWidth, iSkipHeight, 0, clipMinimum, clipMaximum );
    }
  }
  break;

  case   8: fastInverseDCT2_B8  ( srcBuf, tmp, shift_1st, iWidth, iSkipWidth, iSkipHeight, 0, clipMinimum, clipMaximum ); break;
  case  16: fastInverseDCT2_B16 ( srcBuf, tmp, shift_1st, iWidth, iSkipWidth, iSkipHeight, 0, clipMinimum, clipMaximum ); break;
  case  32: fastInverseDCT2_B32 ( srcBuf, tmp, shift_1st, iWidth, iSkipWidth, iSkipHeight, 0, clipMinimum, clipMaximum ); break;
  case  64: fastInverseDCT2_B64 ( srcBuf, tmp, shift_1st + COM16_C806_TRANS_PREC, iWidth, iSkipWidth, iSkipHeight, 0, clipMinimum, clipMaximum ); break;
  case 128: fastInverseDCT2_B128( srcBuf, tmp, shift_1st + COM16_C806_TRANS_PREC, iWidth, iSkipWidth, iSkipHeight, 0, clipMinimum, clipMaximum ); break;
  case  6:  fastInverseDCT2_B6  ( srcBuf, tmp, shift_1st, iWidth, iSkipWidth, iSkipHeight, 1, clipMinimum, clipMaximum ); break;
  case  10: fastInverseDCT2_B10 ( srcBuf, tmp, shift_1st, iWidth, iSkipWidth, iSkipHeight, 1, clipMinimum, clipMaximum ); break;
  case  12: fastInverseDCT2_B12 ( srcBuf, tmp, shift_1st, iWidth, iSkipWidth, iSkipHeight, 1, clipMinimum, clipMaximum ); break;
  case  20: fastInverseDCT2_B20 ( srcBuf, tmp, shift_1st, iWidth, iSkipWidth, iSkipHeight, 1, clipMinimum, clipMaximum ); break;
  case  24: fastInverseDCT2_B24 ( srcBuf, tmp, shift_1st, iWidth, iSkipWidth, iSkipHeight, 1, clipMinimum, clipMaximum ); break;
  case  40: fastInverseDCT2_B40 ( srcBuf, tmp, shift_1st, iWidth, iSkipWidth, iSkipHeight, 1, clipMinimum, clipMaximum ); break;
  case  48: fastInverseDCT2_B48 ( srcBuf, tmp, shift_1st, iWidth, iSkipWidth, iSkipHeight, 1, clipMinimum, clipMaximum ); break;
  case  80: fastInverseDCT2_B80 ( srcBuf, tmp, shift_1st, iWidth, iSkipWidth, iSkipHeight, 1, clipMinimum, clipMaximum ); break;
  case  96: fastInverseDCT2_B96 ( srcBuf, tmp, shift_1st, iWidth, iSkipWidth, iSkipHeight, 1, clipMinimum, clipMaximum ); break;
  case 160: fastInverseDCT2_B160( srcBuf, tmp, shift_1st, iWidth, iSkipWidth, iSkipHeight, 1, clipMinimum, clipMaximum ); break;
  case 192: fastInverseDCT2_B192( srcBuf, tmp, shift_1st, iWidth, iSkipWidth, iSkipHeight, 1, clipMinimum, clipMaximum ); break;
  default:
    THROW( "Unsupported transformation size" ); break;
  }

  iSkipHeight = 0;

  if( reScale != 1 )
  {
    for( Int y = 0; y < iHeight; y++ )
    {
      for( Int x = 0; x < iWidth; x++ )
      {
        tmp[ y * iWidth + x ] *= reScale;
      }
    }
  }

  switch( iWidth )
  {
  case 2: fastInverseDCT2_B2( tmp, block, shift_2nd, iHeight, iSkipHeight, iSkipWidth, 0, std::numeric_limits<Pel>::min(), std::numeric_limits<Pel>::max() ); break;
    // Clipping here is not in the standard, but is used to protect the "Pel" data type into which the inverse-transformed samples will be copied
  case 4:
  {
    if( ( iHeight == 4 ) && useDST )    // Check for DCT or DST
    {
      fastInverseDST7_B4( tmp, block, shift_2nd, iHeight, iSkipHeight, iSkipWidth, 0, std::numeric_limits<Pel>::min(), std::numeric_limits<Pel>::max() );
    }
    else
    {
      fastInverseDCT2_B4( tmp, block, shift_2nd, iHeight, iSkipHeight, iSkipWidth, 0, std::numeric_limits<Pel>::min(), std::numeric_limits<Pel>::max() );
    }
  }
  break;

  case   8: fastInverseDCT2_B8  ( tmp, block, shift_2nd, iHeight, iSkipHeight, iSkipWidth, 0, std::numeric_limits<Pel>::min(), std::numeric_limits<Pel>::max() ); break;
  case  16: fastInverseDCT2_B16 ( tmp, block, shift_2nd, iHeight, iSkipHeight, iSkipWidth, 0, std::numeric_limits<Pel>::min(), std::numeric_limits<Pel>::max() ); break;
  case  32: fastInverseDCT2_B32 ( tmp, block, shift_2nd, iHeight, iSkipHeight, iSkipWidth, 0, std::numeric_limits<Pel>::min(), std::numeric_limits<Pel>::max() ); break;
  case  64: fastInverseDCT2_B64 ( tmp, block, shift_2nd + COM16_C806_TRANS_PREC, iHeight, iSkipHeight, iSkipWidth, 0, std::numeric_limits<Pel>::min(), std::numeric_limits<Pel>::max() ); break;
  case 128: fastInverseDCT2_B128( tmp, block, shift_2nd + COM16_C806_TRANS_PREC, iHeight, iSkipHeight, iSkipWidth, 0, std::numeric_limits<Pel>::min(), std::numeric_limits<Pel>::max() ); break;
  case  6:  fastInverseDCT2_B6  ( tmp, block, shift_2nd, iHeight, iSkipHeight, iSkipWidth, 1, std::numeric_limits<Pel>::min(), std::numeric_limits<Pel>::max() ); break;
  case  10: fastInverseDCT2_B10 ( tmp, block, shift_2nd, iHeight, iSkipHeight, iSkipWidth, 1, std::numeric_limits<Pel>::min(), std::numeric_limits<Pel>::max() ); break;
  case  12: fastInverseDCT2_B12 ( tmp, block, shift_2nd, iHeight, iSkipHeight, iSkipWidth, 1, std::numeric_limits<Pel>::min(), std::numeric_limits<Pel>::max() ); break;
  case  20: fastInverseDCT2_B20 ( tmp, block, shift_2nd, iHeight, iSkipHeight, iSkipWidth, 1, std::numeric_limits<Pel>::min(), std::numeric_limits<Pel>::max() ); break;
  case  24: fastInverseDCT2_B24 ( tmp, block, shift_2nd, iHeight, iSkipHeight, iSkipWidth, 1, std::numeric_limits<Pel>::min(), std::numeric_limits<Pel>::max() ); break;
  case  40: fastInverseDCT2_B40 ( tmp, block, shift_2nd, iHeight, iSkipHeight, iSkipWidth, 1, std::numeric_limits<Pel>::min(), std::numeric_limits<Pel>::max() ); break;
  case  48: fastInverseDCT2_B48 ( tmp, block, shift_2nd, iHeight, iSkipHeight, iSkipWidth, 1, std::numeric_limits<Pel>::min(), std::numeric_limits<Pel>::max() ); break;
  case  80: fastInverseDCT2_B80 ( tmp, block, shift_2nd, iHeight, iSkipHeight, iSkipWidth, 1, std::numeric_limits<Pel>::min(), std::numeric_limits<Pel>::max() ); break;
  case  96: fastInverseDCT2_B96 ( tmp, block, shift_2nd, iHeight, iSkipHeight, iSkipWidth, 1, std::numeric_limits<Pel>::min(), std::numeric_limits<Pel>::max() ); break;
  case 160: fastInverseDCT2_B160( tmp, block, shift_2nd, iHeight, iSkipHeight, iSkipWidth, 1, std::numeric_limits<Pel>::min(), std::numeric_limits<Pel>::max() ); break;
  case 192: fastInverseDCT2_B192( tmp, block, shift_2nd, iHeight, iSkipHeight, iSkipWidth, 1, std::numeric_limits<Pel>::min(), std::numeric_limits<Pel>::max() ); break;
  default:
    THROW( "Unsupported transformation size" );
    break;
  }

  for( Int y = 0; y < iHeight; y++ )
  {
    for( Int x = 0; x < iWidth; x++ )
    {
      dstBuf[ ( y * iWidth ) + x ] = Pel( block[ ( y * iWidth ) + x ] );
    }
  }
}

#endif // THRESHOLDING
