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

/** \file     DiffusionFilter.h
    \brief    diffusion filter (header)
*/

#ifndef __DIFFUSION_FILTER__
#define __DIFFUSION_FILTER__


#include "CommonDef.h"
#include "Unit.h"
#include "Buffer.h"
#include "UnitTools.h"




struct DFPars
{
  bool  doDiffusion;
  bool  linear;
  int   numIter;
  int   GFuncId;

  bool  canContinueIter( const DFPars* prevPars ) const
  {
    if( prevPars == nullptr )                   return false;
    if( !doDiffusion )                          return false;
    if( prevPars->doDiffusion != doDiffusion )  return false;
    if( prevPars->linear      != linear )       return false;
    if( prevPars->GFuncId     != GFuncId )      return false;
    if( prevPars->numIter     >  numIter )      return false;
    return true;
  }
};


typedef double (*GenFunc)( const double );

class TabFunc
{
public:
  TabFunc() {}
  void  init        ( GenFunc dFunc, int inputShift, int outputShift, double maxInputValue, double outsideRangeInputValue = 1e30 );
  int   operator()  ( unsigned arg ) const { return ( arg >= tabVals.size() ? defVal : tabVals[arg] ); }
private:
  std::vector<int>  tabVals;
  int               defVal;
};


class DiffusionFilter
{
public:
  DiffusionFilter ();
  ~DiffusionFilter();

  void  init                ();
  void  applyDiffusion      ( const ComponentID compID, CodingUnit& cu, const Area& area, const CPelBuf& orgPrdBuf, PelBuf& dfPrdBuf, const int prevDFIdx = -1 );

protected:
  void  destroy                   ();
  void  linearDiffusion           ( const DFPars& dfPars, CPelBuf& recBorder, const CPelBuf& orgPrdBlk, PelBuf& dfPrdBlk );
  void  nonlinearDiffusion        ( const DFPars& dfPars, CPelBuf& recBorder, const CPelBuf& orgPrdBlk, PelBuf& dfPrdBlk );

  void  linearDiffusionAddIter    ( CPelBuf& recBorder, PelBuf& dfPrdBlk, const int numAddIter );
  void  nonlinearDiffusionAddIter ( CPelBuf& recBorder, PelBuf& dfPrdBlk, const int numAddIter );


  // border copying and extenson
public:
  static void  copyTopLeftBorder         ( const CPelBuf&  srcBorder,
                                         PelBuf&   dstBuf );
private:
  void  extendBottomRightBorder   (       PelBuf&   srcBuf );

  // basic diffusion steps
  void  linearDiffusionStep       (       PelBuf&   srcBuf,
                                          PelBuf&   dstBuf );
  void  nonlinearDiffusionStep    (       PelBuf&   srcBuf,
                                          PelBuf&   dstBuf );

  // functions for setting non-linear weights
  void  setNonlinearWeights       ( const int       wFuncId,
                                          PelBuf&   srcBuf );
  void  setCenterDifferencesHor   (       PelBuf&   srcBuf,
                                          PelBuf&   bufEW  );
  void  setCenterDifferencesVer   (       PelBuf&   srcBuf,
                                          PelBuf&   bufNS  );
  void  setGradientMatrices       (       PelBuf&   bufNS,
                                          PelBuf&   bufEW,
                                          IntBuf&   grdMatXX,
                                          IntBuf&   grdMatXY,
                                          IntBuf&   grdMatYY );
  void  setIntermediateWeights    ( const int       wFuncId,
                                          PelBuf&   bufNS,
                                          PelBuf&   bufEW,
                                          IntBuf&   grdMatXX,
                                          IntBuf&   grdMatXY,
                                          IntBuf&   grdMatYY,
                                          PelBuf&   auxWeightNS,
                                          PelBuf&   auxWeightEW,
                                          PelBuf&   auxOffsetNS,
                                          PelBuf&   auxOffsetEW );
  void  setFinalNonlinearWeights  (       PelBuf&   auxWeightNS,
                                          PelBuf&   auxWeightEW,
                                          PelBuf&   auxOffsetNS,
                                          PelBuf&   auxOffsetEW );

  void  filterIntBufferSym5tap    ( const int*      filterCoeff,
                                    const int       filterShift,
                                          IntBuf&   buf );
  void  filterIntBufferSym3tap    ( const int*      filterCoeff,
                                    const int       filterShift,
                                          IntBuf&   buf );
  void  setAuxWeights(const TabFunc&  wFunc,
                      const int wFuncId,
                      const int       dNS,
                      const int       dEW,
                      const int       gxx,
                      const int       gxy,
                      const int       gyy,
                      Pel&      auxWeightNS,
                      Pel&      auxWeightEW,
                      Pel&      auxOffsetNS,
                      Pel&      auxOffsetEW);

private:
  enum
  {
    PB_TEMP1_BLOCK = 0,
    PB_TEMP2_BLOCK,
    PB_CDIFF1_BLOCK,
    PB_CDIFF2_BLOCK,
    PB_AUXWEIGHT1_BLOCK,
    PB_AUXWEIGHT2_BLOCK,
    PB_AUXWEIGHT3_BLOCK,
    PB_AUXWEIGHT4_BLOCK,
    PB_WEIGHT1_BLOCK,
    PB_WEIGHT2_BLOCK,
    PB_WEIGHT3_BLOCK,
    PB_WEIGHT4_BLOCK,
    PB_WEIGHT5_BLOCK,
    PB_WEIGHT6_BLOCK,
    PB_WEIGHT7_BLOCK,
    PB_WEIGHT8_BLOCK,
    PB_NUM_BLOCK_TYPES
  };
  enum
  {
    IB_GRDMAT1_BLOCK = 0,
    IB_GRDMAT2_BLOCK,
    IB_GRDMAT3_BLOCK,
    IB_AUXFILTER_BLOCK,
    IB_NUM_BLOCK_TYPES
  };
  enum
  {
    COS_SIN_TAB_INPUT_SHIFT  = 8,
    COS_SIN_TAB_OUTPUT_SHIFT = 8,
    DIFF_FUNC_INPUT_SHIFT    = 2, // must be 2
    DIFF_FUNC_OUTPUT_SHIFT   = 8,
    DIFF_WEIGHT_SHIFT        = 8
  };

  Pel*      m_MemChunksPel[ PB_NUM_BLOCK_TYPES ];
  int*      m_MemChunksInt[ IB_NUM_BLOCK_TYPES ];

  TabFunc   m_GFunc[ 3 ];
  TabFunc   m_cc_t;
  TabFunc   m_cs_t;
  TabFunc   m_ta_t;
  TabFunc   m_cc_c;
  TabFunc   m_cs_c;
  TabFunc   m_ta_c;

  int       m_bitDepth;
  PelBuf    m_wN;
  PelBuf    m_wS;
  PelBuf    m_wE;
  PelBuf    m_wW;
  PelBuf    m_wO1;
  PelBuf    m_wO2;
  PelBuf    m_wO3;
  PelBuf    m_wO4;
};


#endif//__DIFFUSION_FILTER__
