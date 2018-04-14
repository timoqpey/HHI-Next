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

/** \file     Thresholding.h
    \brief    Thresholding (header)
*/

#ifndef __THRESHOLDING__
#define __THRESHOLDING__


#include "CommonDef.h"
#include "Unit.h"
#include "Buffer.h"
#include "UnitTools.h"
#include "CodingStructure.h"
#include "Picture.h"
#include "TrQuant_EMT.h"
#include "RdCost.h"

#include <iostream>
#include <cmath>

#if THRESHOLDING
class Thresholding
{
public:
   Thresholding();
  ~Thresholding();

         Void init              ( const SPS* sps );
  static bool allowThresholding ( const TransformUnit& tu );
         Void applyThresholding ( const ComponentID compID, TransformUnit& tu, PelBuf& piPred );
         UInt getNumberValidThrs( const ComponentID compID, TransformUnit& tu, PelBuf& piPred );
  static UInt getNumberValidSize( const Area& area, const SliceType sliceType, const ChannelType chanType, const SPSNext* sps );

protected:
         Void xGenerateThrs     ( const ComponentID compID, TransformUnit& tu, PelBuf& piPred );
  static Size xGetTrafoSize     ( const Area& area, const unsigned sizeIdx );
         Void xGenerateDCT      ( const ComponentID compID, TransformUnit& tu, const PelBuf& piPred, const Size& finalSize );
         Void xThresholding     ( const ComponentID compID, TransformUnit& tu,                       const Size& finalSize );
         Void xIDCT             ( const ComponentID compID, TransformUnit& tu, PelBuf& thrPrdBuf,    const Size& finalSize );

         Void xCopyTopLeftBorder(                           const CPelBuf& srcBorder, std::vector<Pel>&  buf, const Size& size, const int offsetLeft = 1, const int offsetTop = 1 );
         Void xCopyFromPelBuf   (                           const PelBuf&  orgPrdBuf, std::vector<Pel>&  buf, const Size& size, const int offsetLeft = 1, const int offsetTop = 1 );
         Void xCopyFromVector   ( const ComponentID compID, const std::vector<Pel>&   buf, PelBuf& thrPrdBuf, const Size& size, const int offsetLeft = 1, const int offsetTop = 1 );

         Void xFwdTransform     ( const ComponentID compID, const Pel *srcBuf, TCoeff *dstBuf, const Size& size, const Bool useDST );
         Void xInvTransform     ( const ComponentID compID, const TCoeff *srcBuf, Pel *dstBuf, const Size& size, const Bool useDST );

  Int                 m_bitDepth[ 2 ];
  Int                 m_maxLog2TrDynamicRange[ 2 ];

  std::vector<TCoeff> m_candThrsVals;
  std::vector<Pel>    m_extThrsBuffer;
  std::vector<TCoeff> m_extThrsDctCoeffs;
  Pel                 m_thrsPelBuffer[ MAX_THRESHOLD_CAND ][ MAX_TR_SIZE * MAX_TR_SIZE ];
};
#endif // THRESHOLDING

#endif //__THRESHOLDING___
