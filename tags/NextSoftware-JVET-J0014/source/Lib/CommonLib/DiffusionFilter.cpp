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

/** \file     DiffusionFilter.cpp
    \brief    diffusion filter implementation
*/

#include "DiffusionFilter.h"
#include "CodingStructure.h"
#include "Picture.h"
#include <iostream>
#include "math.h"
#include "AdaptiveLoopFilter.h"
//#include "printmatfile.h"



static const DFPars dfParsArray[2][5] =
{
  { // intra ( doDiffusion, linear, numIter, GFuncId )
    { false, false, 0,  0 },  //                                //  NO_DIFFUSION
    { true,  true,  5,  0 },  //                                //  LINEAR_DIFF_5_ITER
    { true,  true,  15, 0 },  //                                //  LINEAR_DIFF_15_ITER
    { true,  false, 5,  0 },  // g0(x) = exp( -x / * )         //  NON_LINEAR_DIFF_EXP_5_ITER
    { true,  false, 20, 0 },  // g0(x) = exp( -x / * )         //  NON_LINEAR_DIFF_EXP_20_ITER
  },
  { // inter ( doDiffusion, linear, numIter, GFuncId )
    { false, false, 0,  0 },  //                                //  NO_DIFFUSION
    { true,  true,  5,  0 },  //                                //  LINEAR_DIFF_5_ITER
    { true,  true,  35, 0 },  //                                //  LINEAR_DIFF_35_ITER
    { true,  false, 4,  1 },  // g1(x) = 1 / ( 1 + x / * )      //  NON_LINEAR_DIFF_INV_4_ITER_COND_2
    { true,  false, 8,  1 }   // g1(x) = 1 / ( 1 + x / * )      //  NON_LINEAR_DIFF_INV_8_ITER_COND_2
  }
};


double f_g0_200(const double arg) { return exp(-arg / 200.); }
double f_g1_64(const double arg) { return 1. / (1. + arg / 64.); }
double f_g1_200(const double arg) { return 1. / (1. + arg / 550.); }

double cc_t( const double arg ) { double c = cos( 0.5 * atan(    arg ) ); return c*c; }
double cs_t( const double arg ) { double c = cos( 0.5 * atan(    arg ) );
                                  double s = sin( 0.5 * atan(    arg ) ); return c*s; }
double ta_t( const double arg ) { return     tan( 0.5 * atan(    arg ) ); }
double cc_c( const double arg ) { if( arg == 0.0 )                        return 0.5;
                                  double c = cos( 0.5 * atan( 1./arg ) ); return c*c; }
double cs_c( const double arg ) { if( arg == 0.0 )                        return 0.5;
                                  double c = cos( 0.5 * atan( 1./arg ) );
                                  double s = sin( 0.5 * atan( 1./arg ) ); return c*s; }
double ta_c( const double arg ) { if( arg == 0.0 )                        return 1.0;
                                  return     tan( 0.5 * atan( 1./arg ) ); }


void TabFunc::init( GenFunc dFunc, int inputShift, int outputShift, double maxInputValue, double outsideRangeInputValue /* = 1e30 */ )
{
  const double  inputScale  = 1. / double( 1 << inputShift  );
  const double  outputScale =      double( 1 << outputShift );
  const int     numVals     = 1  + lround( maxInputValue / inputScale );
  tabVals.resize( numVals );
  for( unsigned arg = 0; arg < numVals; arg++ )
  {
    tabVals[arg] = lround( outputScale * (*dFunc)( inputScale * double(arg) ) );
  }
  defVal = lround( outputScale * (*dFunc)( outsideRangeInputValue ) );
}


#define DIFFUSION_DEBUG_OUT   0

#define DIFFUSION_DEBUG(x)


DiffusionFilter::DiffusionFilter()
  : m_MemChunksPel { 0 }
  , m_MemChunksInt { 0 }
{
  m_GFunc[0].init( &f_g0_200, DIFF_FUNC_INPUT_SHIFT,    DIFF_FUNC_OUTPUT_SHIFT,   512.0 );
  m_GFunc[1].init( &f_g1_64, DIFF_FUNC_INPUT_SHIFT,    DIFF_FUNC_OUTPUT_SHIFT,   512.0 );
  m_GFunc[2].init(&f_g1_200, DIFF_FUNC_INPUT_SHIFT, DIFF_FUNC_OUTPUT_SHIFT, 512.0);
  m_cc_t    .init( &cc_t, COS_SIN_TAB_INPUT_SHIFT,  COS_SIN_TAB_OUTPUT_SHIFT, 1.0,  1.0 );
  m_cs_t    .init( &cs_t, COS_SIN_TAB_INPUT_SHIFT,  COS_SIN_TAB_OUTPUT_SHIFT, 1.0,  1.0 );
  m_ta_t    .init( &ta_t, COS_SIN_TAB_INPUT_SHIFT,  COS_SIN_TAB_OUTPUT_SHIFT, 1.0,  1.0 );
  m_cc_c    .init( &cc_c, COS_SIN_TAB_INPUT_SHIFT,  COS_SIN_TAB_OUTPUT_SHIFT, 1.0,  1.0 );
  m_cs_c    .init( &cs_c, COS_SIN_TAB_INPUT_SHIFT,  COS_SIN_TAB_OUTPUT_SHIFT, 1.0,  1.0 );
  m_ta_c    .init( &ta_c, COS_SIN_TAB_INPUT_SHIFT,  COS_SIN_TAB_OUTPUT_SHIFT, 1.0,  1.0 );
}

DiffusionFilter::~DiffusionFilter()
{
  destroy();
}


void DiffusionFilter::init()
{
  destroy();
  const int chunkSize = ( MAX_CU_SIZE + 2 ) * ( MAX_CU_SIZE + 2 );
  for( int k = 0; k < PB_NUM_BLOCK_TYPES; k++ )
  {
    m_MemChunksPel[k] = new Pel [ chunkSize ];
  }
  for( int k = 0; k < IB_NUM_BLOCK_TYPES; k++ )
  {
    m_MemChunksInt[k] = new int [ chunkSize ];
  }
}

void DiffusionFilter::destroy()
{
  for( int k = 0; k < PB_NUM_BLOCK_TYPES; k++ )
  {
    if( m_MemChunksPel[k] )
    {
      delete [] m_MemChunksPel[k];
      m_MemChunksPel[k] = 0;
    }
  }
  for( int k = 0; k < IB_NUM_BLOCK_TYPES; k++ )
  {
    if( m_MemChunksInt[k] )
    {
      delete [] m_MemChunksInt[k];
      m_MemChunksInt[k] = 0;
    }
  }
}


void DiffusionFilter::applyDiffusion( const ComponentID compID, CodingUnit& cu, const Area& area, const CPelBuf& orgPrdBuf, PelBuf& dfPrdBuf, const int prevDFIdx /*= -1*/ )
{
  const int     mode    = ( CU::isIntra( cu ) ? 0 : 1 );
  DFPars* dfPars = new DFPars[1];
  dfPars[0] = dfParsArray[mode][cu.diffFilterIdx];
  const bool    noDiff  = ( !dfPars->doDiffusion || area.x == 0 || area.y == 0 );
  if( noDiff )
  {
    dfPrdBuf.copyFrom( orgPrdBuf );
    delete[] dfPars;
    return;
  }
  DFPars* prvPars = new DFPars[1];
  if (prevDFIdx > 0)
    prvPars[0] = dfParsArray[mode][prevDFIdx];
  else {
    delete[] prvPars;
    prvPars = nullptr;
  }

  //GFuncId 0: f_g0_200, GFuncId 1: f_g1_64, GFuncId 2: f_g1_550
  //GFuncId ==0 for Intra, GFuncId==1,2 for Inter
  const int qp = cu.slice->getSliceQpBase();
  if( CU::isInter( cu ) )
  {
    if( !dfPars->linear && qp >= 33 )
    {
      dfPars->GFuncId = dfPars->GFuncId + 1; //switch from f_g1_64 --> f_g1_550   
      if( prvPars && !prvPars->linear )
        prvPars->GFuncId = prvPars->GFuncId + 1;
    }
  }

  // get reconstructed border
  CPelBuf recBorder = cu.cs->picture->getRecoBuf( CompArea( compID, cu.chromaFormat, area.offset(-1, -1), Size(1+orgPrdBuf.width, 1+orgPrdBuf.height) ) ); // accessor for reconstructed border
  m_bitDepth        = cu.cs->sps->getBitDepth( toChannelType( compID ) );

  // apply diffusion filter
  if( dfPars->linear )
  {
    if( dfPars->canContinueIter  ( prvPars ) )
    {
      linearDiffusionAddIter    ( recBorder, dfPrdBuf, dfPars->numIter - prvPars->numIter );
    }
    else
    {
      linearDiffusion           ( dfPars[0], recBorder, orgPrdBuf, dfPrdBuf );
    }
  }
  else
  {
    if( dfPars->canContinueIter  ( prvPars ) )
    {
      nonlinearDiffusionAddIter ( recBorder, dfPrdBuf, dfPars->numIter - prvPars->numIter );
    }
    else
    {
      nonlinearDiffusion        ( dfPars[0], recBorder, orgPrdBuf, dfPrdBuf );
    }
  }
  delete[] dfPars; delete[] prvPars;
}

void DiffusionFilter::linearDiffusion( const DFPars& dfPars, CPelBuf& recBorder, const CPelBuf& orgPrdBlk, PelBuf& dfPrdBlk )
{
  // create blocks of size (2+W)x(2+H)
  const Size  diffBlkSize = Size( 2 + orgPrdBlk.width, 2 + orgPrdBlk.height  );
  PelBuf      tempBlk1    ( m_MemChunksPel[ PB_TEMP1_BLOCK ], diffBlkSize );
  PelBuf      tempBlk2    ( m_MemChunksPel[ PB_TEMP2_BLOCK ], diffBlkSize );
  PelBuf*     dstBlk      = &tempBlk1;
  PelBuf*     srcBlk      = &tempBlk2;

  // copy reconstructed border and original prediction signal
  copyTopLeftBorder( recBorder, *srcBlk );
  copyTopLeftBorder( recBorder, *dstBlk );
  PelBuf( dstBlk->bufAt(1,1), dstBlk->stride, orgPrdBlk.width, orgPrdBlk.height ).copyFrom( orgPrdBlk );

  // iteration
  for( int iter = 0; iter < dfPars.numIter; iter++ )
  {
    std::swap          (  srcBlk,  dstBlk );
    linearDiffusionStep( *srcBlk, *dstBlk );
  }

  // copy result
  dfPrdBlk.copyFrom( PelBuf( dstBlk->bufAt(1,1), dstBlk->stride, orgPrdBlk.width, orgPrdBlk.height ) );
}


void DiffusionFilter::nonlinearDiffusion( const DFPars& dfPars, CPelBuf& recBorder, const CPelBuf& orgPrdBlk, PelBuf& dfPrdBlk )
{
  // create blocks of size (2+W)x(2+H)
  const Size  diffBlkSize = Size( 2 + orgPrdBlk.width, 2 + orgPrdBlk.height  );
  PelBuf      tempBlk1    ( m_MemChunksPel[ PB_TEMP1_BLOCK ], diffBlkSize );
  PelBuf      tempBlk2    ( m_MemChunksPel[ PB_TEMP2_BLOCK ], diffBlkSize );
  PelBuf*     dstBlk      = &tempBlk1;
  PelBuf*     srcBlk      = &tempBlk2;

  // copy reconstructed border and original prediction signal
  copyTopLeftBorder( recBorder, *srcBlk );
  copyTopLeftBorder( recBorder, *dstBlk );
  PelBuf( dstBlk->bufAt(1,1), dstBlk->stride, orgPrdBlk.width, orgPrdBlk.height ).copyFrom( orgPrdBlk );

  // get weights
  setNonlinearWeights( dfPars.GFuncId, *dstBlk );

  // iteration
  DIFFUSION_DEBUG( std::cout << "pred (iter 0) = " << std::endl << *dstBlk << std::endl );
  for( int iter = 0; iter < dfPars.numIter; iter++ )
  {
    std::swap             (  srcBlk,  dstBlk );
    nonlinearDiffusionStep( *srcBlk, *dstBlk );
    DIFFUSION_DEBUG( std::cout << "pred (iter " << iter+1 << ") = " << std::endl << *dstBlk << std::endl );
  }

  // copy result
  dfPrdBlk.copyFrom( PelBuf( dstBlk->bufAt(1,1), dstBlk->stride, orgPrdBlk.width, orgPrdBlk.height ) );
}


void DiffusionFilter::linearDiffusionAddIter( CPelBuf& recBorder, PelBuf& dfPrdBlk, const int numAddIter )
{
  // create blocks of size (2+W)x(2+H)
  const Size  diffBlkSize = Size( 2 + dfPrdBlk.width, 2 + dfPrdBlk.height  );
  PelBuf      tempBlk1    ( m_MemChunksPel[ PB_TEMP1_BLOCK ], diffBlkSize );
  PelBuf      tempBlk2    ( m_MemChunksPel[ PB_TEMP2_BLOCK ], diffBlkSize );
  PelBuf*     dstBlk      = &tempBlk1;
  PelBuf*     srcBlk      = &tempBlk2;

  // copy reconstructed border and original prediction signal
  copyTopLeftBorder( recBorder, *srcBlk );
  copyTopLeftBorder( recBorder, *dstBlk );
  PelBuf( dstBlk->bufAt(1,1), dstBlk->stride, dfPrdBlk.width, dfPrdBlk.height ).copyFrom( dfPrdBlk );

  // iteration
  for( int iter = 0; iter < numAddIter; iter++ )
  {
    std::swap          (  srcBlk,  dstBlk );
    linearDiffusionStep( *srcBlk, *dstBlk );
  }

  // copy result
  dfPrdBlk.copyFrom( PelBuf( dstBlk->bufAt(1,1), dstBlk->stride, dfPrdBlk.width, dfPrdBlk.height ) );
}


void DiffusionFilter::nonlinearDiffusionAddIter( CPelBuf& recBorder, PelBuf& dfPrdBlk, const int numAddIter )
{
  // create blocks of size (2+W)x(2+H)
  const Size  diffBlkSize = Size( 2 + dfPrdBlk.width, 2 + dfPrdBlk.height  );
  PelBuf      tempBlk1    ( m_MemChunksPel[ PB_TEMP1_BLOCK ], diffBlkSize );
  PelBuf      tempBlk2    ( m_MemChunksPel[ PB_TEMP2_BLOCK ], diffBlkSize );
  PelBuf*     dstBlk      = &tempBlk1;
  PelBuf*     srcBlk      = &tempBlk2;

  // copy reconstructed border and original prediction signal
  copyTopLeftBorder( recBorder, *srcBlk );
  copyTopLeftBorder( recBorder, *dstBlk );
  PelBuf( dstBlk->bufAt(1,1), dstBlk->stride, dfPrdBlk.width, dfPrdBlk.height ).copyFrom( dfPrdBlk );

  // iteration
  for( int iter = 0; iter < numAddIter; iter++ )
  {
    std::swap             (  srcBlk,  dstBlk );
    nonlinearDiffusionStep( *srcBlk, *dstBlk );
    DIFFUSION_DEBUG( std::cout << "pred (iter " << iter+1 << ") = " << std::endl << *dstBlk << std::endl );
  }

  // copy result
  dfPrdBlk.copyFrom( PelBuf( dstBlk->bufAt(1,1), dstBlk->stride, dfPrdBlk.width, dfPrdBlk.height ) );
}


void DiffusionFilter::copyTopLeftBorder( const CPelBuf& srcBorder, PelBuf& dstBuf )
{
  CHECKD( dstBuf.width < srcBorder.width || dstBuf.height < srcBorder.height, "target buffer is too small" );
  const Pel*  srcData  = srcBorder.bufAt( 0, 0 );
  Pel*        dstData  = dstBuf   .bufAt( 0, 0 );

  // top
  ::memcpy( dstData, srcData, srcBorder.width * sizeof(Pel) );

  // left
  srcData += srcBorder.stride;
  dstData += dstBuf   .stride;
  for( int y = 1; y < srcBorder.height; y++, srcData += srcBorder.stride, dstData += dstBuf.stride )
  {
    *dstData = *srcData;
  }
}



void DiffusionFilter::extendBottomRightBorder( PelBuf& srcBuf )
{
  const int xmax    = srcBuf.width  - 1;
  const int ymax    = srcBuf.height - 1;
  Pel*      right   = srcBuf.bufAt( xmax, 0 );
  for( int y = 0; y < ymax; y++, right += srcBuf.stride )
  {
    *right = right[-2];
  }
  Pel*      bottom  = srcBuf.bufAt( 0, ymax );
  ::memcpy( bottom, bottom-(srcBuf.stride<<1), srcBuf.width * sizeof(Pel) );
}


void DiffusionFilter::linearDiffusionStep( PelBuf& srcBuf, PelBuf& dstBuf )
{
  const int   xmax      = srcBuf.width  - 1;
  const int   ymax      = srcBuf.height - 1;
  const int   srcStride = srcBuf.stride;
  const int   dstStride = dstBuf.stride;

  // right and bottom border extension
  extendBottomRightBorder( srcBuf );

  // filtering
  const Pel*  srcData = srcBuf.bufAt( 0, 1 );
  Pel*        dstData = dstBuf.bufAt( 0, 1 );
  for( int y = 1; y < ymax; y++, srcData += srcStride, dstData += dstStride )
  {
    for( int x = 1; x < xmax; x++ )
    {
      dstData[x] = ( srcData[x-srcStride] + srcData[x-1] + srcData[x+1] + srcData[x+srcStride] + 2 ) >> 2;
    }
  }
}


void DiffusionFilter::nonlinearDiffusionStep( PelBuf& srcBuf, PelBuf& dstBuf )
{
  const int   xmax      = srcBuf.width  - 1;
  const int   ymax      = srcBuf.height - 1;
  const int   srcStride = srcBuf.stride;

  // right and bottom border extension
  extendBottomRightBorder( srcBuf );

  // filtering
  const Pel*  wN      = m_wN  .bufAt( 0, 1 );
  const Pel*  wS      = m_wS  .bufAt( 0, 1 );
  const Pel*  wE      = m_wE  .bufAt( 0, 1 );
  const Pel*  wW      = m_wW  .bufAt( 0, 1 );
  const Pel*  wO1      = m_wO1  .bufAt( 0, 1 );
  const Pel*  wO2 = m_wO2.bufAt(0, 1);
  const Pel*  wO3 = m_wO3.bufAt(0, 1);
  const Pel*  wO4 = m_wO4.bufAt(0, 1);
  const Pel*  srcData = srcBuf.bufAt( 0, 1 );
  Pel*        dstData = dstBuf.bufAt( 0, 1 );
  for( int y = 1; y < ymax; y++ )
  {
    for( int x = 1; x < xmax; x++ )
    {
      Pel curr   =      srcData[x];
      int wSum = int(wO1[x]) * int(srcData[x + 1 - srcStride]);
      wSum -=   int(wO2[x]) * int(srcData[x + 1 + srcStride]);
      wSum -= int(wO3[x]) * int(srcData[x - 1 - srcStride] );
      wSum += int(wO4[x]) * int(srcData[x - 1 + srcStride]);
      wSum      += int(      wN[x] ) * int( srcData[ x - srcStride ] - curr );
      wSum      += int(      wW[x] ) * int( srcData[ x -         1 ] - curr );
      wSum      += int(      wE[x] ) * int( srcData[ x +         1 ] - curr );
      wSum      += int(      wS[x] ) * int( srcData[ x + srcStride ] - curr );
      int val = int(curr) + (wSum >> (2+DIFF_WEIGHT_SHIFT));
      dstData[x] = Pel( ClipBD( val, m_bitDepth ) );
    }

    wN      += m_wN  .stride;
    wS      += m_wS  .stride;
    wE      += m_wE  .stride;
    wW      += m_wW  .stride;
    wO1      += m_wO1  .stride;
    wO2 += m_wO2.stride;
    wO3 += m_wO3.stride;
    wO4 += m_wO4.stride;
    srcData += srcBuf.stride;
    dstData += dstBuf.stride;
  }
}





void DiffusionFilter::setNonlinearWeights( const int wFuncId, PelBuf& srcBuf )
{
  // set temporary buffers
  PelBuf    diffNS      ( m_MemChunksPel[ PB_CDIFF1_BLOCK     ], srcBuf );
  PelBuf    diffEW      ( m_MemChunksPel[ PB_CDIFF2_BLOCK     ], srcBuf );
  IntBuf    grdMatXX    ( m_MemChunksInt[ IB_GRDMAT1_BLOCK    ], srcBuf );
  IntBuf    grdMatXY    ( m_MemChunksInt[ IB_GRDMAT2_BLOCK    ], srcBuf );
  IntBuf    grdMatYY    ( m_MemChunksInt[ IB_GRDMAT3_BLOCK    ], srcBuf );
  PelBuf    auxWeightNS ( m_MemChunksPel[ PB_AUXWEIGHT1_BLOCK ], srcBuf );
  PelBuf    auxWeightEW ( m_MemChunksPel[ PB_AUXWEIGHT2_BLOCK ], srcBuf );
  PelBuf    auxOffsetNS ( m_MemChunksPel[ PB_AUXWEIGHT3_BLOCK ], srcBuf );
  PelBuf    auxOffsetEW ( m_MemChunksPel[ PB_AUXWEIGHT4_BLOCK ], srcBuf );

  // get weights
  extendBottomRightBorder ( srcBuf );
  DIFFUSION_DEBUG         ( std::cout << "Src = " << std::endl << srcBuf << std::endl );
  setCenterDifferencesHor ( srcBuf, diffEW );
  setCenterDifferencesVer ( srcBuf, diffNS );
  DIFFUSION_DEBUG         ( std::cout << "diffNS = " << std::endl << diffNS << std::endl );
  DIFFUSION_DEBUG         ( std::cout << "diffEW = " << std::endl << diffEW << std::endl );
  setGradientMatrices     ( diffNS, diffEW,  grdMatXX, grdMatXY, grdMatYY );
  DIFFUSION_DEBUG         ( std::cout << "grdMatXX = " << std::endl << grdMatXX << std::endl );
  DIFFUSION_DEBUG         ( std::cout << "grdMatXY = " << std::endl << grdMatXY << std::endl );
  DIFFUSION_DEBUG         ( std::cout << "grdMatYY = " << std::endl << grdMatYY << std::endl );
  setIntermediateWeights  ( wFuncId, diffNS, diffEW, grdMatXX, grdMatXY, grdMatYY, auxWeightNS, auxWeightEW, auxOffsetNS, auxOffsetEW );
  DIFFUSION_DEBUG         ( std::cout << "auxWeightNS = " << std::endl << auxWeightNS << std::endl );
  DIFFUSION_DEBUG         ( std::cout << "auxWeightEW = " << std::endl << auxWeightEW << std::endl );
  DIFFUSION_DEBUG         ( std::cout << "auxOffsetNS = " << std::endl << auxOffsetNS << std::endl );
  DIFFUSION_DEBUG         ( std::cout << "auxOffsetEW = " << std::endl << auxOffsetEW << std::endl );
  setFinalNonlinearWeights( auxWeightNS, auxWeightEW, auxOffsetNS, auxOffsetEW );
  DIFFUSION_DEBUG         ( std::cout << "wN = " << std::endl << m_wN << std::endl );
  DIFFUSION_DEBUG         ( std::cout << "wS = " << std::endl << m_wS << std::endl );
  DIFFUSION_DEBUG         ( std::cout << "wW = " << std::endl << m_wW << std::endl );
  DIFFUSION_DEBUG         ( std::cout << "wE = " << std::endl << m_wE << std::endl );
  DIFFUSION_DEBUG         ( std::cout << "wO = " << std::endl << m_wO << std::endl );
}



void DiffusionFilter::setCenterDifferencesHor( PelBuf& srcBuf, PelBuf& bufEW )
{
  const int   xmax      = srcBuf.width - 1;
  const int   srcStride = srcBuf.stride;
  const Pel*  srcData   = srcBuf.bufAt( 0, 0 );
  Pel*        EW        = bufEW .bufAt( 0, 0 );

  for( int y = 0; y < srcBuf.height; y++, srcData += srcStride, EW += bufEW.stride )
  {
    for( int x = 1; x < xmax; x++ )
    {
      EW[x] = ( srcData[ x + 1 ] - srcData[ x - 1 ] );  // >> 1
    }

    // left and right border
    EW[0]     = -EW[2];
    EW[xmax]  = -EW[xmax-2];
  }
}


void DiffusionFilter::setCenterDifferencesVer( PelBuf& srcBuf, PelBuf& bufNS )
{
  const int   ymax      = srcBuf.height - 1;
  const int   srcStride = srcBuf.stride;
  const Pel*  srcData   = srcBuf.bufAt( 0, 1 );
  Pel*        NS        = bufNS .bufAt( 0, 1 );

  for( int y = 1; y < srcBuf.height - 1; y++, srcData += srcStride, NS += bufNS.stride )
  {
    for( int x = 0; x < srcBuf.width; x++ )
    {
      NS[x] = ( srcData[ x - srcStride ] - srcData[ x + srcStride ] );  // >> 1
    }
  }

  // top and bottom border
  Pel* line0  = bufNS.bufAt(0, 0);
  Pel* lineN  = bufNS.bufAt(0, ymax);
  Pel* line2  = line0 + (srcStride<<1);
  Pel* lineN2 = lineN - (srcStride<<1);
  for( int x = 0; x < srcBuf.width; x++ )
  {
    line0[x]  = -line2 [x];
    lineN[x]  = -lineN2[x];
  }
}


void DiffusionFilter::setGradientMatrices( PelBuf& bufNS, PelBuf& bufEW, IntBuf& grdMatXX, IntBuf& grdMatXY, IntBuf& grdMatYY )
{
  const int   height  = bufNS   .height;
  const int   width   = bufNS   .width;
  const Pel*  NS      = bufNS   .bufAt(0,0);
  const Pel*  EW      = bufEW   .bufAt(0,0);
  int*        gxx     = grdMatXX.bufAt(0,0);
  int*        gxy     = grdMatXY.bufAt(0,0);
  int*        gyy     = grdMatYY.bufAt(0,0);

  // initial
  for( int y = 0; y < height; y++ )
  {
    for( int x = 0; x < width; x++ )
    {
      gxx[x] = EW[x] * EW[x];   // >> 2
      gxy[x] = EW[x] * NS[x];   // >> 2
      gyy[x] = NS[x] * NS[x];   // >> 2
    }

    NS  += bufNS   .stride;
    EW  += bufEW   .stride;
    gxx += grdMatXX.stride;
    gxy += grdMatXY.stride;
    gyy += grdMatYY.stride;
  }

  // smoothing
  const int smoothingFilterCoeff[]  = { 2, 1 };
  const int smoothingFilterShift    = 2;          // log2( sum of filter coefficients )
  filterIntBufferSym3tap( smoothingFilterCoeff, smoothingFilterShift, grdMatXX );
  filterIntBufferSym3tap( smoothingFilterCoeff, smoothingFilterShift, grdMatXY );
  filterIntBufferSym3tap( smoothingFilterCoeff, smoothingFilterShift, grdMatYY );
}


void DiffusionFilter::setIntermediateWeights( const int wFuncId, PelBuf& bufNS, PelBuf& bufEW, IntBuf& grdMatXX, IntBuf& grdMatXY, IntBuf& grdMatYY, PelBuf& auxWeightNS, PelBuf& auxWeightEW, PelBuf& auxOffsetNS, PelBuf& auxOffsetEW )
{
  const TabFunc&  wFunc = m_GFunc[wFuncId];
  const int       height  = grdMatXX   .height;
  const int       width   = grdMatXX   .width;
  const Pel*      dNS     = bufNS      .bufAt(0,0);
  const Pel*      dEW     = bufEW      .bufAt(0,0);
  const int*      gxx     = grdMatXX   .bufAt(0,0);
  const int*      gxy     = grdMatXY   .bufAt(0,0);
  const int*      gyy     = grdMatYY   .bufAt(0,0);
  Pel*            wNS     = auxWeightNS.bufAt(0,0);
  Pel*            wEW     = auxWeightEW.bufAt(0,0);
  Pel*            oNS     = auxOffsetNS.bufAt(0,0);
  Pel*            oEW     = auxOffsetEW.bufAt(0,0);

  for( int y = 0; y < height; y++ )
  {
    for( int x = 0; x < width; x++ )
    {
      setAuxWeights(wFunc, wFuncId, dNS[x], dEW[x], gxx[x], gxy[x], gyy[x], wNS[x], wEW[x], oNS[x], oEW[x]);
    }

    dNS += bufNS      .stride;
    dEW += bufEW      .stride;
    gxx += grdMatXX   .stride;
    gxy += grdMatXY   .stride;
    gyy += grdMatYY   .stride;
    wNS += auxWeightNS.stride;
    wEW += auxWeightEW.stride;
    oNS += auxOffsetNS.stride;
    oEW += auxOffsetEW.stride;
  }
}


void DiffusionFilter::setFinalNonlinearWeights( PelBuf& auxWeightNS, PelBuf& auxWeightEW, PelBuf& auxOffsetNS, PelBuf& auxOffsetEW )
{
  // set weight buffers
  m_wN = PelBuf( m_MemChunksPel[ PB_WEIGHT1_BLOCK ], auxWeightNS );
  m_wS = PelBuf( m_MemChunksPel[ PB_WEIGHT2_BLOCK ], auxWeightNS );
  m_wE = PelBuf( m_MemChunksPel[ PB_WEIGHT3_BLOCK ], auxWeightNS );
  m_wW = PelBuf( m_MemChunksPel[ PB_WEIGHT4_BLOCK ], auxWeightNS );
  m_wO1 = PelBuf( m_MemChunksPel[ PB_WEIGHT5_BLOCK ], auxWeightNS );
  m_wO2 = PelBuf(m_MemChunksPel[PB_WEIGHT6_BLOCK], auxWeightNS);
  m_wO3 = PelBuf(m_MemChunksPel[PB_WEIGHT7_BLOCK], auxWeightNS);
  m_wO4 = PelBuf(m_MemChunksPel[PB_WEIGHT8_BLOCK], auxWeightNS);

  DIFFUSION_DEBUG( m_wN.fill(0) );
  DIFFUSION_DEBUG( m_wS.fill(0) );
  DIFFUSION_DEBUG( m_wE.fill(0) );
  DIFFUSION_DEBUG( m_wW.fill(0) );
  DIFFUSION_DEBUG( m_wO1.fill(0) );

  // set weights (note: only for inside range)
  const int   xmax  = auxWeightNS .width  - 1;
  const int   ymax  = auxWeightNS .height - 1;
  const int   stwNS = auxWeightNS .stride;
  const int   stoEW = auxOffsetEW .stride;
  const Pel*  wNS   = auxWeightNS .bufAt(0, 1);
  const Pel*  wEW   = auxWeightEW .bufAt(0, 1);
  const Pel*  oNS   = auxOffsetNS .bufAt(0, 1);
  const Pel*  oEW   = auxOffsetEW .bufAt(0, 1);
  Pel*        wN    = m_wN        .bufAt(0, 1);
  Pel*        wS    = m_wS        .bufAt(0, 1);
  Pel*        wE    = m_wE        .bufAt(0, 1);
  Pel*        wW    = m_wW        .bufAt(0, 1);
  Pel*        wO1    = m_wO1        .bufAt(0, 1);
  Pel*        wO2 = m_wO2.bufAt(0, 1);
  Pel*        wO3 = m_wO3.bufAt(0, 1);
  Pel*        wO4 = m_wO4.bufAt(0, 1);
  for( int y = 1; y < ymax; y++ )
  {
    for( int x = 1; x < xmax; x++ )
    {
      wN[ x ] = ( wNS[ x         ] + wNS[ x - stwNS ] + 1 ) >> 1;
      wS[ x ] = ( wNS[ x         ] + wNS[ x + stwNS ] + 1 ) >> 1;
      wW[ x ] = ( wEW[ x         ] + wEW[ x - 1     ] + 1 ) >> 1;
      wE[ x ] = ( wEW[ x         ] + wEW[ x + 1     ] + 1 ) >> 1;
      wO1[x] = (oEW[x + 1] + oEW[x - stoEW] + 2) >> 2;
      wO2[x] = (oEW[x + 1] + oEW[x + stoEW] + 2) >> 2;
      wO3[x] = (oEW[x - 1] + oEW[x - stoEW] + 2) >> 2;
      wO4[x] = (oEW[x - 1] + oEW[x + stoEW] + 2) >> 2;
    }

    wNS += auxWeightNS.stride;
    wEW += auxWeightEW.stride;
    oNS += auxOffsetNS.stride;
    oEW += auxOffsetEW.stride;
    wN  += m_wN       .stride;
    wS  += m_wS       .stride;
    wE  += m_wE       .stride;
    wW  += m_wW       .stride;
    wO1  += m_wO1       .stride;
    wO2 += m_wO2.stride;
    wO3 += m_wO3.stride;
    wO4 += m_wO4.stride;
  }
}








void DiffusionFilter::filterIntBufferSym5tap( const int* filterCoeff, const int filterShift, IntBuf& buf )
{
  IntBuf aux( m_MemChunksInt[ IB_AUXFILTER_BLOCK ], buf );

  const int   a         = filterCoeff[0];
  const int   b         = filterCoeff[1];
  const int   c         = filterCoeff[2];
  const int   o         = ( 1 << filterShift ) >> 1;
  const int   w         = buf.width;
  const int   h         = buf.height;
  const int   srcStride = buf.stride;
  const int   auxStride = aux.stride;
  const int*  src       = buf.bufAt(0,0);
  int*        des       = aux.bufAt(0,0);

  // vertical
  for( int y = 0; y < h; y++, src += srcStride, des += auxStride )
  {
    const int*  srcB0 = ( y > 0   ? src -  srcStride     : src   );
    const int*  srcB1 = ( y < h-1 ? src +  srcStride     : src   );
    const int*  srcC0 = ( y > 1   ? src - (srcStride<<1) : srcB0 );
    const int*  srcC1 = ( y < h-2 ? src + (srcStride<<1) : srcB1 );

    for( int x = 0; x < w; x++ )
    {
      des[x] = ( a * src[x] + b * ( srcB0[x] + srcB1[x] ) + c * ( srcC0[x] + srcC1[x] ) + o ) >> filterShift;
    }
  }

  // horizontal
  src = aux.bufAt(0,0);
  des = buf.bufAt(0,0);
  for( int y = 0; y < h; y++, src += auxStride, des += srcStride )
  {
    des[0]    = ( a * src[0]   + b * ( src[0]   + src[1]   ) + c * ( src[0]   + src[2]   ) + o ) >> filterShift;
    des[1]    = ( a * src[1]   + b * ( src[0]   + src[2]   ) + c * ( src[0]   + src[3]   ) + o ) >> filterShift;
    for( int x = 2; x < w-2; x++ )
    {
      des[x]  = ( a * src[x]   + b * ( src[x-1] + src[x+1] ) + c * ( src[x-2] + src[x+2] ) + o ) >> filterShift;
    }
    des[w-2]  = ( a * src[w-2] + b * ( src[w-3] + src[w-1] ) + c * ( src[w-4] + src[w-1] ) + o ) >> filterShift;
    des[w-1]  = ( a * src[w-1] + b * ( src[w-2] + src[w-1] ) + c * ( src[w-3] + src[w-1] ) + o ) >> filterShift;
  }
}


void DiffusionFilter::filterIntBufferSym3tap( const int* filterCoeff, const int filterShift, IntBuf& buf )
{
  IntBuf aux( m_MemChunksInt[ IB_AUXFILTER_BLOCK ], buf );

  const int   a         = filterCoeff[0];
  const int   b         = filterCoeff[1];
  const int   o         = ( 1 << filterShift ) >> 1;
  const int   w         = buf.width;
  const int   h         = buf.height;
  const int   srcStride = buf.stride;
  const int   auxStride = aux.stride;
  const int*  src       = buf.bufAt(0,0);
  int*        des       = aux.bufAt(0,0);

  // vertical
  for( int y = 0; y < h; y++, src += srcStride, des += auxStride )
  {
    const int*  srcB0 = ( y > 0   ? src - srcStride : src );
    const int*  srcB1 = ( y < h-1 ? src + srcStride : src );

    for( int x = 0; x < w; x++ )
    {
      des[x] = ( a * src[x] + b * ( srcB0[x] + srcB1[x] ) + o ) >> filterShift;
    }
  }

  // horizontal
  src = aux.bufAt(0,0);
  des = buf.bufAt(0,0);
  for( int y = 0; y < h; y++, src += auxStride, des += srcStride )
  {
    des[0]    = ( a * src[0]   + b * ( src[0]   + src[1]   ) + o ) >> filterShift;
    for( int x = 1; x < w-1; x++ )
    {
      des[x]  = ( a * src[x]   + b * ( src[x-1] + src[x+1] ) + o ) >> filterShift;
    }
    des[w-1]  = ( a * src[w-1] + b * ( src[w-2] + src[w-1] ) + o ) >> filterShift;
  }
}

void DiffusionFilter::setAuxWeights(const TabFunc& wFunc, const int wFuncId, const int dNS, const int dEW, const int gxx, const int gxy, const int gyy, Pel& auxWeightNS, Pel& auxWeightEW, Pel& auxOffsetNS, Pel& auxOffsetEW)
{
  // [A] basic algorithm:
  //
  //   (1)  eigen decomposition
  //         [ ev1  0  ] = [  c -s ] [ gxx gxy ] [  c  s ]    with  c = cos(t)
  //         [  0  ev2 ]   [  s  c ] [ gxy gyy ] [ -s  c ]          s = sin(t)
  //
  //   (2)  apply function to eigenvalues
  //         [ g1   0  ] = [ g( ev1 )     0     ]
  //         [  0  g2  ]   [    0      g( ev2 ) ]
  //
  //   (3)  transform back
  //         [ axx axy ] = [  c  s ] [ g1   0 ] [  c -s ]
  //         [ axy ayy ]   [ -s  c ] [  0  g2 ] [  s  c ]
  //
  //
  // [B] If the angle t is known (and with ss = 1 - cc), we have
  //        ev1 = cc * gxx + ss * gyy - 2 * cs * gxy          with cc = cos(t) * cos(t)
  //        ev2 = cc * gyy + ss * gxx + 2 * cs * gxy               ss = sin(t) * sin(t) = 1 - cc
  //                                                               cs = cos(t) * sin(t)
  //     and
  //        axx = cc * g1 + ss * g2
  //        ayy = cc * g2 + ss * g1
  //        axy = cs * ( g2 - g1 )
  //
  //
  // [C] Determination of theta:
  //
  //     According to (1), we need
  //          0 = cs * ( gxx - gyy ) + ( cc - ss ) * gxy
  //
  //     With sin(2t) = 2 cos(t) sin(t)  and  cos(2t) = cos^2(t) - sin^2(t), we get
  //          0 = (1/2) sin(2t) * ( gxx - gyy ) + cos(2t) * gxy
  //
  //     Yielding
  //          tan(2t) = 2 * gxy / ( gyy - gxx )
  //
  //
  // [D] Using tan(t) = s / c = ss / cs = ss ( cc + ss ) / cs = ( 2 cs cs - cc ss + ss ss ) /cs
  //                  = 2 cs - ss ( cc - ss ) / cs
  //                  = 2 cs - ss ( gyy - gxx ) / gxy     (see first equation in [C])
  //     we can simplify
  //        ev1 = gxx - gxy * tan(t)
  //        ev2 = gyy + gxy * tan(t)
  //
  //
  // [E] Final algorithm:
  //
  //     [precalculate]  Tabulate the following functions for 0 <= x <= 1 (with sufficient accuracy)
  //          cc_t(x) = cos( 0.5 * atan( x ) ) * cos( 0.5 * atan( x ) )
  //          cs_t(x) = cos( 0.5 * atan( x ) ) * sin( 0.5 * atan( x ) )
  //          ta_t(x) = tan( 0.5 * atan( x ) )
  //          cc_c(x) = cos( 0.5 * atan(1/x) ) * cos( 0.5 * atan(1/x) )
  //          cs_c(x) = cos( 0.5 * atan(1/x) ) * sin( 0.5 * atan(1/x) )
  //          ta_c(x) = tan( 0.5 * atan(1/x) )
  //
  //     (1) Determine cc, ss, cs, ta
  //
  //           if( gxy == 0 )  { cc = 1; cs = 0; ta = 0; }
  //           else {
  //             a   = abs( 2 * gxy )
  //             b   = abs( gyy - gxx )
  //             sgn = sgn( gxy ) * sgn( gyy - gxx )
  //             if( a <= b )  { cc = cc_t(a/b); cs = sgn * cs_t(a/b); ta = sgn * ta_t(a/b); }
  //             else          { cc = cc_c(b/a); cs = sgn * cs_c(b/a); ta = sgn * ta_c(b/a);  }
  //           }
  //           ss = 1 - cc
  //
  //     (2) Determine eigenvalues
  //
  //           ev1 = gxx - gxy * ta
  //           ev2 = gyy + gxy * ta
  //
  //     (3) Apply function
  //
  //           g1 = g(ev1)
  //           g2 = g(ev2)
  //
  //     (4) Back transform
  //
  //           axx = cc * g1 + ss * g2
  //           ayy = cc * g2 + ss * g1
  //           axy = cs * ( g2 - g1 )


  // input:  [ gxx gxy ]                                                                        //  >> 2
  //         [ gxy gyy ]

  // determine cc, ss, cs and ta
  const int           two_gxy   = gxy << 1;                                                     //  >> 2
  const int           gyy_gxx   = gyy - gxx;                                                    //  >> 2
  int                 cc        = ( 1 << COS_SIN_TAB_OUTPUT_SHIFT );                            //  >> CSOS
  int                 cs        = 0;
  int                 ta        = 0;
  if( two_gxy != 0 )
  {
    const int         a         = abs( two_gxy );
    const int         b         = abs( gyy_gxx );
    const int         sgn       = ( two_gxy < 0 ? -1 : 1 ) * ( gyy_gxx < 0 ? -1 : 1 );
    if( a <= b )
    {
      const unsigned  arg       = unsigned( ( ( a << COS_SIN_TAB_INPUT_SHIFT ) + ( b >> 1 ) ) / b );
      cc                        = m_cc_t( arg );                                                //  >> CSOS
      cs                        = m_cs_t( arg ) * sgn;                                          //  >> CSOS
      ta                        = m_ta_t( arg ) * sgn;                                          //  >> CSOS
    }
    else
    {
      const unsigned  arg       = unsigned( ( ( b << COS_SIN_TAB_INPUT_SHIFT ) + ( a >> 1 ) ) / a );
      cc                        = m_cc_c( arg );                                                //  >> CSOS
      cs                        = m_cs_c( arg ) * sgn;                                          //  >> CSOS
      ta                        = m_ta_c( arg ) * sgn;                                          //  >> CSOS
    }
  }
  const int           ss        = ( 1 << COS_SIN_TAB_OUTPUT_SHIFT ) - cc;                       //  >> CSOS

  // get eigenvalues and diffusion function results
  CHECKD( DIFF_FUNC_INPUT_SHIFT != 2, "expected shift equal to 2" );
  const int           evOffs    = ( ta * gxy + ( ( 1 << COS_SIN_TAB_OUTPUT_SHIFT ) >> 1 ) ) >> COS_SIN_TAB_OUTPUT_SHIFT;
  const unsigned      ev1       = (unsigned)std::max( 0, gxx - evOffs );                        //  >> DFIS
  const unsigned      ev2       = (unsigned)std::max( 0, gyy + evOffs );                        //  >> DFIS
  int g1; int g2;
  g1 = wFunc(ev1);
  g2 = wFunc(ev2);
  // back transform
  const int           axx       = cc * g1 + ss * g2;                                            //  >> ( CSOS + DFOS )
  const int           ayy       = cc * g2 + ss * g1;                                            //  >> ( CSOS + DFOS )
  const int           axy       = cs * ( g2 - g1 );                                             //  >> ( CSOS + DFOS )

  // get weights
  const int           shift = COS_SIN_TAB_OUTPUT_SHIFT + DIFF_FUNC_OUTPUT_SHIFT - DIFF_WEIGHT_SHIFT;
  const int           add = (1 << shift) >> 1;
  auxWeightNS = Pel((ayy + add) >> shift);
  auxWeightEW = Pel((axx + add) >> shift);
  auxOffsetNS = Pel((axy + add) >> shift);
  auxOffsetEW = Pel((axy + add) >> shift);
}



