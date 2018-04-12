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

/** \file     ThresholdingSearch.cpp
    \brief    ThresholdingSearch implementation
*/

#include "Thresholding.h"
#include "ThresholdingSearch.h"
#include "dtrace_codingstruct.h"
#include "dtrace_buffer.h"

#if THRESHOLDING
ThresholdingSearch::ThresholdingSearch()
{
}

ThresholdingSearch::~ThresholdingSearch()
{
}

UInt ThresholdingSearch::getNumberValidThrs( const ComponentID compID, TransformUnit& tu, PelBuf& piPred )
{
  CHECK( !tu.thresholding, "Thresholding not enabled but applied" )
  CHECK( std::max( tu.blocks[ compID ].height, tu.blocks[ compID ].width ) > MAX_CU_SIZE, "TU is larger than maximum allowed size" );

  const SliceType sliceType = tu.cs->slice->getSliceType();
  const Size      finalSize = xGetTrafoSize( tu.blocks[ compID ], tu.thresholdingSize );
        unsigned  maxThrs   = tu.cs->sps->getSpsNext().getThresholdingMaxThrs( sliceType );

  CHECK( finalSize.area() == 0, "invalid transform size" );
  CHECK( std::max( finalSize.height, finalSize.width ) > MAX_TR_SIZE, "transform size larger than supported" );

  if( 0 == tu.thresholdingThrs )
  {
    xGenerateThrs( compID, tu, piPred );

    maxThrs = (unsigned)m_candThrsVals.size();

    for( int thresholdingThrs = 0; thresholdingThrs < maxThrs; thresholdingThrs++ )
    {
      PelBuf thrsSavedBuffer( m_thrsPelBuffer[ thresholdingThrs ], tu.blocks[ compID ] );

      tu.thresholdingThrs = thresholdingThrs;

      xThresholding( compID, tu, finalSize );
      xIDCT( compID, tu, thrsSavedBuffer, finalSize );
    }

    tu.thresholdingThrs = 0;
  }

  return (UInt)m_candThrsVals.size();
}

Void ThresholdingSearch::initTestLoop( const TransformUnit& tu, Int& numTransformModes, Int& lastCheckId )
{
  const SPSNext&  spsNext   = tu.cs->slice->getSPS()->getSpsNext();
  const SliceType sliceType = tu.cs->slice->isIntra() ? I_SLICE : B_SLICE;

  m_thresholdingMaxSize = Thresholding::getNumberValidSize( tu.Y(), sliceType, tu.chType, &tu.cs->sps->getSpsNext() );

  if( !allowThresholding( tu ) )
  {
    m_thresholdingMaxSize = 0;
    m_thresholdingMaxThrs = 0;
  }
  else
  {
    m_thresholdingMaxThrs = spsNext.getThresholdingMaxThrs( sliceType );
  }

  m_noThrLastCheckId = lastCheckId;
  numTransformModes  = lastCheckId + 1;
  lastCheckId       += m_thresholdingMaxSize > 0 ? ( m_thresholdingMaxSize * m_thresholdingMaxThrs ) * numTransformModes : 0;

  m_numTransformModes = numTransformModes;
  m_transformMode     = 0;

  m_bestModeNoThrs       = -1;
  m_thresholdingTestSize =  0;
  m_thresholdingTestThrs =  0;

  m_invalidTransformModes.clear();
}

Void ThresholdingSearch::getTestParameters( Int modeId, Bool& skipTest, UChar& transformIndex, Bool& thresholding, UChar& thresholdingSize, UChar& thresholdingThrs )
{
  transformIndex = m_transformMode;

  // loop sizes
  // loop thresholds (load size data)
  // loop transforms (load threshold data)

  m_transformMode++;

  if( m_transformMode >= m_numTransformModes )
  {
    m_transformMode = 0;
  }

  thresholding     = modeId > m_noThrLastCheckId;
  thresholdingSize = m_thresholdingTestSize;
  thresholdingThrs = m_thresholdingTestThrs;

  skipTest = false;

  if( thresholding )
  {
    for( int i = 0; i < (int)m_invalidTransformModes.size(); i++ )
    {
      if( transformIndex == m_invalidTransformModes[ i ] )
      {
        skipTest = true;
        break;
      }
    }

    if( transformIndex != m_bestModeNoThrs )
    {
      skipTest = true;
    }

    if( m_transformMode == 0 )
    {
      m_thresholdingTestThrs++;

      if( m_thresholdingTestThrs >= m_thresholdingMaxThrs )
      {
        m_thresholdingTestSize++;
        m_thresholdingTestThrs = 0;
      }
    }
  }
}

Void ThresholdingSearch::addInvalidMode( UChar transformIndex )
{
  m_invalidTransformModes.push_back( transformIndex );
}

#endif // THRESHOLDING
