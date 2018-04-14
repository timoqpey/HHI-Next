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

/** \file     LoopFilter.h
    \brief    deblocking filter (header)
*/

#ifndef __LOOPFILTER__
#define __LOOPFILTER__

#include "CommonDef.h"
#include "Unit.h"
#include "Picture.h"

//! \ingroup CommonLib
//! \{

#define DEBLOCK_SMALLEST_BLOCK  8

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// deblocking filter class
class LoopFilter
{
private:
  static_vector<char, MAX_NUM_PARTS_IN_CTU> m_aapucBS       [NUM_EDGE_DIR];         ///< Bs for [Ver/Hor][Y/U/V][Blk_Idx]
  static_vector<bool, MAX_NUM_PARTS_IN_CTU> m_aapbEdgeFilter[NUM_EDGE_DIR];
  LFCUParam m_stLFCUParam;                   ///< status structure

private:
  /// CU-level deblocking function
  void xDeblockCU                 (       CodingUnit& cu, const DeblockEdgeDir edgeDir );

  // set / get functions
  void xSetLoopfilterParam        ( const CodingUnit& cu );

  // filtering functions
  unsigned
  xGetBoundaryStrengthSingle      ( const CodingUnit& cu, const DeblockEdgeDir edgeDir, const Position& localPos ) const;

  void xSetEdgefilterMultiple     ( const CodingUnit&     cu,
                                    const DeblockEdgeDir  edgeDir,
                                    const Area&           area,
                                    const bool            bValue,
                                    const bool            EdgeIdx = false );
#if JEM_TOOLS
  Void xSetEdgefilterMultipleSubPu( const CodingUnit&    cu,
                                          DeblockEdgeDir edgeDir,
                                    const Area&          area,
                                    const Position       subPuPos,
                                          Bool           bValue );
#endif

  void xEdgeFilterLuma            ( const CodingUnit& cu, const DeblockEdgeDir edgeDir, const int iEdge );
  void xEdgeFilterChroma          ( const CodingUnit& cu, const DeblockEdgeDir edgeDir, const int iEdge );

  inline void xPelFilterLuma      ( Pel* piSrc, const int iOffset, const int tc, const bool sw, const bool bPartPNoFilter, const bool bPartQNoFilter, const int iThrCut, const bool bFilterSecondP, const bool bFilterSecondQ, const ClpRng& clpRng ) const;
  inline void xPelFilterChroma    ( Pel* piSrc, const int iOffset, const int tc,                const bool bPartPNoFilter, const bool bPartQNoFilter,                                                                          const ClpRng& clpRng ) const;

  inline bool xUseStrongFiltering ( Pel* piSrc, const int iOffset, const int d, const int beta, const int tc ) const;
  inline int xCalcDP              ( Pel* piSrc, const int iOffset ) const;
  inline int xCalcDQ              ( Pel* piSrc, const int iOffset ) const;

  static const UChar sm_tcTable[54];
  static const UChar sm_betaTable[52];

public:

  LoopFilter();
  ~LoopFilter();

  void  create                    ( const unsigned uiMaxCUDepth );
  void  destroy                   ();

  /// picture-level deblocking filter
  void loopFilterPic              ( CodingStructure& cs
                                    );

  static int getBeta              ( const int qp )
  {
    const int indexB = Clip3( 0, MAX_QP, qp );
    return sm_betaTable[ indexB ];
  }
};

//! \}

#endif
