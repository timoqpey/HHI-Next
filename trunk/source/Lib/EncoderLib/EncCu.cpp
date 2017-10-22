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

/** \file     EncCu.cpp
    \brief    Coding Unit (CU) encoder class
*/

#include "EncCu.h"

#include "EncLib.h"
#include "Analyze.h"
#include "AQp.h"

#include "CommonLib/dtrace_codingstruct.h"
#include "CommonLib/Picture.h"
#include "CommonLib/UnitTools.h"

#include "CommonLib/dtrace_buffer.h"

#include <stdio.h>
#include <cmath>
#include <algorithm>

using namespace std;


//! \ingroup EncoderLib
//! \{

// ====================================================================================================================
// Constructor / destructor / create / destroy
// ====================================================================================================================

void EncCu::create( EncCfg* encCfg )
{
  unsigned      uiMaxWidth    = encCfg->getMaxCUWidth();
  unsigned      uiMaxHeight   = encCfg->getMaxCUHeight();
  ChromaFormat  chromaFormat  = encCfg->getChromaFormatIdc();
  bool          BTnoRQT       = encCfg->getQTBT();
  unsigned      numWidths     = gp_sizeIdxInfo->numWidths();
  unsigned      numHeights    = gp_sizeIdxInfo->numHeights();
  unsigned      maxMEPart     = BTnoRQT ? 1 : NUMBER_OF_PART_SIZES;

  m_pTempCS = new CodingStructure**  [numWidths];
  m_pBestCS = new CodingStructure**  [numWidths];

  for( unsigned w = 0; w < numWidths; w++ )
  {
    m_pTempCS[w] = new CodingStructure*  [numHeights];
    m_pBestCS[w] = new CodingStructure*  [numHeights];

    for( unsigned h = 0; h < numHeights; h++ )
    {
      unsigned width  = gp_sizeIdxInfo->sizeFrom( w );
      unsigned height = gp_sizeIdxInfo->sizeFrom( h );

      if( ( BTnoRQT || w == h ) && gp_sizeIdxInfo->isCuSize( width ) && gp_sizeIdxInfo->isCuSize( height ) )
      {
        m_pTempCS[w][h] = new CodingStructure( m_unitCache.cuCache, m_unitCache.puCache, m_unitCache.tuCache );
        m_pBestCS[w][h] = new CodingStructure( m_unitCache.cuCache, m_unitCache.puCache, m_unitCache.tuCache );

        m_pTempCS[w][h]->create( chromaFormat, Area( 0, 0, width, height ) );
        m_pBestCS[w][h]->create( chromaFormat, Area( 0, 0, width, height ) );
      }
      else
      {
        m_pTempCS[w][h] = nullptr;
        m_pBestCS[w][h] = nullptr;
      }
    }
  }

  // WIA: only the weight==height case is relevant without QTBT
  m_pImvTempCS = nullptr;

  if( IMV_OFF != encCfg->getIMV() && !BTnoRQT )
  {
    m_pImvTempCS = new CodingStructure**[numWidths];

    for( unsigned w = 0; w < numWidths; w++ )
    {
      unsigned width  = gp_sizeIdxInfo->sizeFrom( w );
      unsigned height = gp_sizeIdxInfo->sizeFrom( w );

      m_pImvTempCS[w] = new CodingStructure*[maxMEPart];

      for( unsigned p = 0; p < maxMEPart; p++ )
      {
        if( gp_sizeIdxInfo->isCuSize( width ) )
        {
          m_pImvTempCS[w][p] = new CodingStructure( m_unitCache.cuCache, m_unitCache.puCache, m_unitCache.tuCache );
          m_pImvTempCS[w][p]->create( chromaFormat, Area( 0, 0, width, height ) );
        }
        else
        {
          m_pImvTempCS[w][p] = nullptr;
        }
      }
    }
  }

  m_pTempCUWoOBMC  = nullptr;
  m_pPredBufWoOBMC = nullptr;

  if( encCfg->getUseOBMC() )
  {
    m_pTempCUWoOBMC  = new CodingStructure**[numWidths];
    m_pPredBufWoOBMC = new PelStorage*[numWidths];

    for( unsigned w = 0; w < numWidths; w++ )
    {
      m_pTempCUWoOBMC [w] = new CodingStructure*[numHeights];
      m_pPredBufWoOBMC[w] = new PelStorage[numHeights];

      for( unsigned h = 0; h < numHeights; h++ )
      {
        UInt width  = gp_sizeIdxInfo->sizeFrom( w );
        UInt height = gp_sizeIdxInfo->sizeFrom( h );

        if( ( BTnoRQT || w == h ) && gp_sizeIdxInfo->isCuSize( width ) && gp_sizeIdxInfo->isCuSize( height ) )
        {

          m_pTempCUWoOBMC[w][h] = new CodingStructure( m_unitCache.cuCache, m_unitCache.puCache, m_unitCache.tuCache );
          m_pTempCUWoOBMC[w][h]->create( chromaFormat, Area( 0, 0, width, height ) );

          m_pPredBufWoOBMC[w][h].create( UnitArea( chromaFormat, Area( 0, 0, width, height ) ) );
        }
      }
    }
  }

  m_cuChromaQpOffsetIdxPlus1 = 0;

  unsigned maxDepth = numWidths + numHeights;

  if( encCfg->getQTBT() )
  {
    m_modeCtrl = new EncModeCtrlMTnoRQT();
  }
  else
  {
    m_modeCtrl = new EncModeCtrlQTwithRQT();
  }

  for( unsigned ui = 0; ui < MRG_MAX_NUM_CANDS; ui++ )
  {
    m_acMergeBuffer[ui].create( chromaFormat, Area(  0, 0, uiMaxWidth, uiMaxHeight ) );
  }

  m_CtxBuffer.resize( maxDepth );
  m_CurrCtx = 0;
}


void EncCu::destroy()
{
  bool          BTnoRQT       = m_pcEncCfg->getQTBT();
  unsigned      maxMEPart     = BTnoRQT ? 1 : NUMBER_OF_PART_SIZES;

  unsigned numWidths  = gp_sizeIdxInfo->numWidths();
  unsigned numHeights = gp_sizeIdxInfo->numHeights();

  for( unsigned w = 0; w < numWidths; w++ )
  {
    for( unsigned h = 0; h < numHeights; h++ )
    {
      if( BTnoRQT || w == h )
      {
        if( m_pBestCS[w][h] ) m_pBestCS[w][h]->destroy();
        if( m_pTempCS[w][h] ) m_pTempCS[w][h]->destroy();

        delete m_pBestCS[w][h];
        delete m_pTempCS[w][h];
      }
    }

    delete[] m_pTempCS[w];
    delete[] m_pBestCS[w];
  }

  delete[] m_pBestCS; m_pBestCS = nullptr;
  delete[] m_pTempCS; m_pTempCS = nullptr;

  delete m_modeCtrl;
  m_modeCtrl = nullptr;

  // WIA: only the weight==height case is relevant without QTBT
  if( m_pImvTempCS )
  {
    for( unsigned w = 0; w < numWidths; w++ )
    {
      for( unsigned p = 0; p < maxMEPart; p++ )
      {
        if( m_pImvTempCS[w][p] ) m_pImvTempCS[w][p]->destroy();
        delete m_pImvTempCS[w][p];
      }
      delete[] m_pImvTempCS[w];
    }

    delete[] m_pImvTempCS;
    m_pImvTempCS = nullptr;
  }

  if( m_pTempCUWoOBMC )
  {
    for( unsigned w = 0; w < numWidths; w++ )
    {
      for( unsigned h = 0; h < numHeights; h++ )
      {
        if( ( BTnoRQT || w == h ) && gp_sizeIdxInfo->isCuSize( gp_sizeIdxInfo->sizeFrom( w ) ) && gp_sizeIdxInfo->isCuSize( gp_sizeIdxInfo->sizeFrom( h ) ) )
        {
          m_pTempCUWoOBMC[w][h]->destroy();
          delete m_pTempCUWoOBMC[w][h];

          m_pPredBufWoOBMC[w][h].destroy();
        }
      }
      delete[] m_pTempCUWoOBMC[w];
      delete[] m_pPredBufWoOBMC[w];
    }

    delete[] m_pTempCUWoOBMC;
    m_pTempCUWoOBMC = nullptr;

    delete[] m_pPredBufWoOBMC;
    m_pPredBufWoOBMC = nullptr;
  }

  for( unsigned ui = 0; ui < MRG_MAX_NUM_CANDS; ui++ )
  {
    m_acMergeBuffer[ui].destroy();
  }
}



EncCu::~EncCu()
{
}



/** \param    pcEncLib      pointer of encoder class
 */
void EncCu::init( EncLib* pcEncLib, const SPS& sps )
{
  m_pcEncCfg           = pcEncLib;
  m_pcIntraSearch      = pcEncLib->getIntraSearch();
  m_pcInterSearch      = pcEncLib->getInterSearch();
  m_pcTrQuant          = pcEncLib->getTrQuant();
  m_pcRdCost           = pcEncLib->getRdCost();
  m_CABACEstimator     = pcEncLib->getCABACEncoder()->getCABACEstimator( &sps );
  m_CtxCache           = pcEncLib->getCtxCache();
  m_pcRateCtrl         = pcEncLib->getRateCtrl();

  m_modeCtrl->init( m_pcEncCfg, m_pcRateCtrl, m_pcRdCost );

  m_pcInterSearch->setModeCtrl( m_modeCtrl );
  m_pcIntraSearch->setModeCtrl( m_modeCtrl );
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

void EncCu::compressCtu( CodingStructure& cs, const UnitArea& area, unsigned ctuRsAddr )
{
  m_modeCtrl->initCTUEncoding( *cs.slice );

  // init the partitioning manager
  Partitioner *partitioner = PartitionerFactory::get( *cs.slice );
  partitioner->initCtu( area );

  // init current context pointer
  m_CurrCtx = m_CtxBuffer.data();

  CodingStructure *tempCS = m_pTempCS[gp_sizeIdxInfo->idxFrom( area.lumaSize().width )][gp_sizeIdxInfo->idxFrom( area.lumaSize().height )];
  CodingStructure *bestCS = m_pBestCS[gp_sizeIdxInfo->idxFrom( area.lumaSize().width )][gp_sizeIdxInfo->idxFrom( area.lumaSize().height )];

  cs.initSubStructure( *tempCS, partitioner->currArea() );
  cs.initSubStructure( *bestCS, partitioner->currArea() );

  if( !cs.slice->isIntra() )
  {
    if( m_pImvTempCS )
    {
      const unsigned maxMEPart = cs.pcv->only2Nx2N ? 1 : NUMBER_OF_PART_SIZES;
      for( unsigned p = 0; p < maxMEPart; p++ )
      {
        cs.initSubStructure( *m_pImvTempCS[gp_sizeIdxInfo->idxFrom( tempCS->area.lwidth() )][p], area );
      }
    }

    if( m_pTempCUWoOBMC )
    {
      cs.initSubStructure( *m_pTempCUWoOBMC[gp_sizeIdxInfo->idxFrom( tempCS->area.lwidth() )][gp_sizeIdxInfo->idxFrom( tempCS->area.lheight() )], partitioner->currArea() );
    }
  }

  xCompressCU( tempCS, bestCS, *partitioner );

  // all signals were already copied during compression if the CTU was split - at this point only the structures are copied to the top level CS
  const bool copyUnsplitCTUSignals = bestCS->cus.size() == 1 && KEEP_PRED_AND_RESI_SIGNALS;
  cs.useSubStructure( *bestCS, CS::getArea( *bestCS, area ), copyUnsplitCTUSignals, false, false, copyUnsplitCTUSignals );

  if( !cs.pcv->ISingleTree && cs.slice->isIntra() && cs.pcv->chrFormat != CHROMA_400 )
  {
    m_CABACEstimator->getCtx() = m_CurrCtx->start;

    partitioner->initCtu( area );

    cs.chType = CHANNEL_TYPE_CHROMA;
    cs.initSubStructure( *tempCS, partitioner->currArea(), false, CHANNEL_TYPE_CHROMA );
    cs.initSubStructure( *bestCS, partitioner->currArea(), false, CHANNEL_TYPE_CHROMA );

    xCompressCU( tempCS, bestCS, *partitioner );

    const bool copyUnsplitCTUSignals = bestCS->cus.size() == 1 && KEEP_PRED_AND_RESI_SIGNALS;
    cs.useSubStructure( *bestCS, CS::getArea( *bestCS, area ), copyUnsplitCTUSignals, false, false, copyUnsplitCTUSignals );
    cs.chType = CHANNEL_TYPE_LUMA;
  }

  // reset context states and uninit context pointer
  m_CABACEstimator->getCtx() = m_CurrCtx->start;
  m_CurrCtx                  = 0;
  delete partitioner;

  // Ensure that a coding was found
  // Selected mode's RD-cost must be not MAX_DOUBLE.
  CHECK( bestCS->cus.empty()                                   , "No possible encoding found" );
  CHECK( bestCS->cus[0]->partSize == NUMBER_OF_PART_SIZES      , "No possible encoding found" );
  CHECK( bestCS->cus[0]->predMode == NUMBER_OF_PREDICTION_MODES, "No possible encoding found" );
  CHECK( bestCS->cost             == MAX_DOUBLE                , "No possible encoding found" );
}

// ====================================================================================================================
// Protected member functions
// ====================================================================================================================

static int xCalcHADs8x8_ISlice(const Pel *piOrg, const Int iStrideOrg)
{
  Int k, i, j, jj;
  Int diff[64], m1[8][8], m2[8][8], m3[8][8], iSumHad = 0;

  for (k = 0; k < 64; k += 8)
  {
    diff[k + 0] = piOrg[0];
    diff[k + 1] = piOrg[1];
    diff[k + 2] = piOrg[2];
    diff[k + 3] = piOrg[3];
    diff[k + 4] = piOrg[4];
    diff[k + 5] = piOrg[5];
    diff[k + 6] = piOrg[6];
    diff[k + 7] = piOrg[7];

    piOrg += iStrideOrg;
  }

  //horizontal
  for (j = 0; j < 8; j++)
  {
    jj = j << 3;
    m2[j][0] = diff[jj    ] + diff[jj + 4];
    m2[j][1] = diff[jj + 1] + diff[jj + 5];
    m2[j][2] = diff[jj + 2] + diff[jj + 6];
    m2[j][3] = diff[jj + 3] + diff[jj + 7];
    m2[j][4] = diff[jj    ] - diff[jj + 4];
    m2[j][5] = diff[jj + 1] - diff[jj + 5];
    m2[j][6] = diff[jj + 2] - diff[jj + 6];
    m2[j][7] = diff[jj + 3] - diff[jj + 7];

    m1[j][0] = m2[j][0] + m2[j][2];
    m1[j][1] = m2[j][1] + m2[j][3];
    m1[j][2] = m2[j][0] - m2[j][2];
    m1[j][3] = m2[j][1] - m2[j][3];
    m1[j][4] = m2[j][4] + m2[j][6];
    m1[j][5] = m2[j][5] + m2[j][7];
    m1[j][6] = m2[j][4] - m2[j][6];
    m1[j][7] = m2[j][5] - m2[j][7];

    m2[j][0] = m1[j][0] + m1[j][1];
    m2[j][1] = m1[j][0] - m1[j][1];
    m2[j][2] = m1[j][2] + m1[j][3];
    m2[j][3] = m1[j][2] - m1[j][3];
    m2[j][4] = m1[j][4] + m1[j][5];
    m2[j][5] = m1[j][4] - m1[j][5];
    m2[j][6] = m1[j][6] + m1[j][7];
    m2[j][7] = m1[j][6] - m1[j][7];
  }

  //vertical
  for (i = 0; i < 8; i++)
  {
    m3[0][i] = m2[0][i] + m2[4][i];
    m3[1][i] = m2[1][i] + m2[5][i];
    m3[2][i] = m2[2][i] + m2[6][i];
    m3[3][i] = m2[3][i] + m2[7][i];
    m3[4][i] = m2[0][i] - m2[4][i];
    m3[5][i] = m2[1][i] - m2[5][i];
    m3[6][i] = m2[2][i] - m2[6][i];
    m3[7][i] = m2[3][i] - m2[7][i];

    m1[0][i] = m3[0][i] + m3[2][i];
    m1[1][i] = m3[1][i] + m3[3][i];
    m1[2][i] = m3[0][i] - m3[2][i];
    m1[3][i] = m3[1][i] - m3[3][i];
    m1[4][i] = m3[4][i] + m3[6][i];
    m1[5][i] = m3[5][i] + m3[7][i];
    m1[6][i] = m3[4][i] - m3[6][i];
    m1[7][i] = m3[5][i] - m3[7][i];

    m2[0][i] = m1[0][i] + m1[1][i];
    m2[1][i] = m1[0][i] - m1[1][i];
    m2[2][i] = m1[2][i] + m1[3][i];
    m2[3][i] = m1[2][i] - m1[3][i];
    m2[4][i] = m1[4][i] + m1[5][i];
    m2[5][i] = m1[4][i] - m1[5][i];
    m2[6][i] = m1[6][i] + m1[7][i];
    m2[7][i] = m1[6][i] - m1[7][i];
  }

  for (i = 0; i < 8; i++)
  {
    for (j = 0; j < 8; j++)
    {
      iSumHad += abs(m2[i][j]);
    }
  }
  iSumHad -= abs(m2[0][0]);
  iSumHad = (iSumHad + 2) >> 2;
  return(iSumHad);
}

int  EncCu::updateCtuDataISlice(const CPelBuf buf)
{
  Int  xBl, yBl;
  const Int iBlkSize = 8;
  const Pel* pOrgInit = buf.buf;
  Int  iStrideOrig = buf.stride;

  Int iSumHad = 0;
  for( yBl = 0; ( yBl + iBlkSize ) <= buf.height; yBl += iBlkSize )
  {
    for( xBl = 0; ( xBl + iBlkSize ) <= buf.width; xBl += iBlkSize )
    {
      const Pel* pOrg = pOrgInit + iStrideOrig*yBl + xBl;
      iSumHad += xCalcHADs8x8_ISlice( pOrg, iStrideOrig );
    }
  }
  return( iSumHad );
}

void EncCu::xCheckBestMode( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode )
{
  if( !tempCS->cus.empty() )
  {
    DTRACE_BEST_MODE( tempCS, bestCS, m_pcRdCost->getLambda() );

    //Position bottomRight = clipArea( bestCS->area.Y(), bestCS->picture->Y() ).bottomRight();
    //CHECK( !tempCS->getCU( bottomRight ), "No possible encoding found" );

    if( m_modeCtrl->useModeResult( encTestMode, tempCS, partitioner ) )
    {
      if( tempCS->cus.size() == 1 )
      {
        // if tempCS is not a split-mode
        CodingUnit &cu = *tempCS->cus.front();

        if( CU::isLosslessCoded( cu ) && !cu.ipcm )
        {
          xFillPCMBuffer( cu );
        }
      }


      std::swap( tempCS, bestCS );
      // store temp best CI for next CU coding
      m_CurrCtx->best = m_CABACEstimator->getCtx();
    }
  }

  // reset context states
  m_CABACEstimator->getCtx() = m_CurrCtx->start;
}

void EncCu::xCompressCU( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner )
{
  const Slice &slice  = *tempCS->slice;
  const PPS &pps      = *tempCS->pps;
  const SPS &sps      = *tempCS->sps;
  const UInt uiLPelX  = tempCS->area.Y().lumaPos().x;
  const UInt uiTPelY  = tempCS->area.Y().lumaPos().y;

  m_CurrCtx->start    = m_CABACEstimator->getCtx();

  m_modeCtrl->initCULevel( partitioner, *tempCS );

#if SHARP_LUMA_DELTA_QP
  Double totalSplitCost = 0;
#endif

  m_cuChromaQpOffsetIdxPlus1 = 0;

  if( slice.getUseChromaQpAdj() )
  {
    Int lgMinCuSize = sps.getLog2MinCodingBlockSize() +
      std::max<Int>( 0, sps.getLog2DiffMaxMinCodingBlockSize() - Int( pps.getPpsRangeExtension().getDiffCuChromaQpOffsetDepth() ) );
    m_cuChromaQpOffsetIdxPlus1 = ( ( uiLPelX >> lgMinCuSize ) + ( uiTPelY >> lgMinCuSize ) ) % ( pps.getPpsRangeExtension().getChromaQpOffsetListLen() + 1 );
  }

  if( !m_modeCtrl->anyMode() )
  {
    m_modeCtrl->finishCULevel( partitioner );
    return;
  }

  DTRACE( g_trace_ctx, D_COMMON, "@(%4d,%4d) [%2dx%2d]\n", tempCS->area.lx(), tempCS->area.ly(), tempCS->area.lwidth(), tempCS->area.lheight() );


  do
  {
    const EncTestMode currTestMode = m_modeCtrl->currTestMode();

    if( currTestMode.type == ETM_INTER_ME )
    {
      bool tryObmc = true;

      if( ( currTestMode.opts & ETO_IMV ) != 0 )
      {
        tryObmc = xCheckRDCostInterIMV( tempCS, bestCS, partitioner, currTestMode );
      }
      else
      {
        xCheckRDCostInter( tempCS, bestCS, partitioner, currTestMode );
      }

      if( tryObmc )
      {
        xCheckRDCostInterWoOBMC( tempCS, bestCS, partitioner, currTestMode );
      }
    }
    else if( currTestMode.type == ETM_AFFINE )
    {
      xCheckRDCostAffineMerge2Nx2N( tempCS, bestCS, partitioner, currTestMode );
    }
    else if( currTestMode.type == ETM_MERGE_SKIP )
    {
      xCheckRDCostMerge2Nx2N( tempCS, bestCS, partitioner, currTestMode );
    }
    else if( currTestMode.type == ETM_MERGE_FRUC )
    {
      xCheckRDCostMerge2Nx2NFRUC( tempCS, bestCS, partitioner, currTestMode );
    }
    else if( currTestMode.type == ETM_INTRA )
    {
      xCheckRDCostIntra( tempCS, bestCS, partitioner, currTestMode );
    }
    else if( currTestMode.type == ETM_IPCM )
    {
      xCheckIntraPCM( tempCS, bestCS, partitioner, currTestMode );
    }
    else if( isModeSplit( currTestMode ) )
    {

#if SHARP_LUMA_DELTA_QP
      xCheckModeSplit( tempCS, bestCS, partitioner, currTestMode, totalSplitCost );
#else
      xCheckModeSplit( tempCS, bestCS, partitioner, currTestMode );
#endif
    }
    else
    {
      THROW( "Don't know how to handle mode: type = " << currTestMode.type << ", size = " << currTestMode.partSize << ", options = " << currTestMode.opts );
    }
  } while( m_modeCtrl->nextMode( *tempCS, partitioner ) );

  //////////////////////////////////////////////////////////////////////////
  // Finishing CU

  // set context states
  m_CABACEstimator->getCtx() = m_CurrCtx->best;

  // QP from last processed CU for further processing
  bestCS->prevQP[bestCS->chType] = bestCS->cus.back()->qp;

  const UnitArea currCsArea = CS::getArea( *bestCS, bestCS->area );
  bestCS->picture->getRecoBuf( currCsArea ).copyFrom( bestCS->getRecoBuf( currCsArea ) );

  m_modeCtrl->finishCULevel( partitioner );

  // Assert if Best prediction mode is NONE
  // Selected mode's RD-cost must be not MAX_DOUBLE.
  CHECK( bestCS->cus.empty()                                   , "No possible encoding found" );
  CHECK( bestCS->cus[0]->partSize == NUMBER_OF_PART_SIZES      , "No possible encoding found" );
  CHECK( bestCS->cus[0]->predMode == NUMBER_OF_PREDICTION_MODES, "No possible encoding found" );
  CHECK( bestCS->cost             == MAX_DOUBLE                , "No possible encoding found" );
}

#if SHARP_LUMA_DELTA_QP
void EncCu::xCheckModeSplit(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode , Double& splitTotalCost)
#else
void EncCu::xCheckModeSplit(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode)
#endif
{
  const Int qp                = encTestMode.qp;
  const PPS &pps              = *tempCS->pps;
  const Slice &slice          = *tempCS->slice;
  const Bool bIsLosslessMode  = false; // False at this level. Next level down may set it to true.
  const int oldPrevQp         = tempCS->prevQP[tempCS->chType];
  const UInt currDepth        = partitioner.currDepth;

  const PartSplit split = getPartSplit( encTestMode );

  CHECK( split == CU_DONT_SPLIT, "No proper split provided!" );

  tempCS->initStructData( qp, bIsLosslessMode );

  partitioner.splitCurrArea( split, *tempCS );

  m_CABACEstimator->getCtx() = m_CurrCtx->start;
  m_CurrCtx++;

  tempCS->getRecoBuf().fill( 0 );

  do
  {
    const auto &subCUArea  = partitioner.currArea();

    if( tempCS->picture->Y().contains( subCUArea.lumaPos() ) )
    {
      const unsigned wIdx    = gp_sizeIdxInfo->idxFrom( subCUArea.lwidth () );
      const unsigned hIdx    = gp_sizeIdxInfo->idxFrom( subCUArea.lheight() );

      CodingStructure *tempSubCS = m_pTempCS[wIdx][hIdx];
      CodingStructure *bestSubCS = m_pBestCS[wIdx][hIdx];

      tempCS->initSubStructure( *tempSubCS, subCUArea );
      tempCS->initSubStructure( *bestSubCS, subCUArea );

      if( m_pImvTempCS && !slice.isIntra() )
      {
        const unsigned maxMEPart = tempCS->pcv->only2Nx2N ? 1 : NUMBER_OF_PART_SIZES;
        for( unsigned p = 0; p < maxMEPart; p++ )
        {
          tempCS->initSubStructure( *m_pImvTempCS[wIdx][p], subCUArea );
        }
      }

      if( m_pTempCUWoOBMC && !slice.isIntra() )
      {
        tempCS->initSubStructure( *m_pTempCUWoOBMC[wIdx][hIdx], subCUArea );
      }

      xCompressCU( tempSubCS, bestSubCS, partitioner );

      if( bestSubCS->cost == MAX_DOUBLE )
      {
        CHECK( split == CU_QUAD_SPLIT, "Split decision reusing cannot skip quad split" );
        tempCS->cost = MAX_DOUBLE;
        m_CurrCtx--;
        partitioner.exitCurrSplit();
        xCheckBestMode( tempCS, bestCS, partitioner, encTestMode );
        return;
      }

      tempCS->useSubStructure( *bestSubCS, CS::getArea( *tempCS, subCUArea ), KEEP_PRED_AND_RESI_SIGNALS, true, KEEP_PRED_AND_RESI_SIGNALS, KEEP_PRED_AND_RESI_SIGNALS );

      if(currDepth < pps.getMaxCuDQPDepth())
      {
        tempCS->prevQP[tempCS->chType] = bestSubCS->prevQP[bestSubCS->chType];
      }

#if SHARP_LUMA_DELTA_QP
      if ( m_pcEncCfg->getLumaLevelToDeltaQPMapping().isEnabled() && pps.getMaxCuDQPDepth() >= 1 )
      {
        splitTotalCost += bestSubCS->cost;
      }
#endif

      tempSubCS->releaseIntermediateData();
      bestSubCS->releaseIntermediateData();
    }
  } while( partitioner.nextPart( *tempCS ) );

  partitioner.exitCurrSplit();

  m_CurrCtx--;

  // Finally, generate split-signaling bits for RD-cost check
  const PartSplit implicitSplit = partitioner.getImplicitSplit( *tempCS );

  {
    bool enforceQT = implicitSplit == CU_QUAD_SPLIT;


    if( !enforceQT )
    {
      m_CABACEstimator->resetBits();

      if( partitioner.canSplit( CU_QUAD_SPLIT, *tempCS ) )
      {
        m_CABACEstimator->split_cu_flag( split == CU_QUAD_SPLIT, *tempCS, partitioner );
      }
      if( split != CU_QUAD_SPLIT && implicitSplit == CU_DONT_SPLIT )
      {
        m_CABACEstimator->split_cu_mode_mt( split, *tempCS, partitioner );
      }

#if SHARP_LUMA_DELTA_QP
      if( m_pcEncCfg->getLumaLevelToDeltaQPMapping().isEnabled() && pps.getMaxCuDQPDepth() >= 1 )
      {
        splitTotalCost += m_pcRdCost->calcRdCost( m_CABACEstimator->getEstFracBits(), 0 );
      }
#endif

      tempCS->fracBits += m_CABACEstimator->getEstFracBits(); // split bits
    }
  }

#if SHARP_LUMA_DELTA_QP
  if ( m_pcEncCfg->getLumaLevelToDeltaQPMapping().isEnabled() && pps.getMaxCuDQPDepth() >= 1 )
  {
    tempCS->cost = splitTotalCost;
  }
  else
  {
#endif
    tempCS->cost = m_pcRdCost->calcRdCost( tempCS->fracBits, tempCS->dist );
#if SHARP_LUMA_DELTA_QP
  }
#endif

  // Check Delta QP bits for splitted structure
  if( pps.getUseDQP() && currDepth == pps.getMaxCuDQPDepth() )
  {
    xCheckDQP( *tempCS, true );
  }

  // If the configuration being tested exceeds the maximum number of bytes for a slice / slice-segment, then
  // a proper RD evaluation cannot be performed. Therefore, termination of the
  // slice/slice-segment must be made prior to this CTU.
  // This can be achieved by forcing the decision to be that of the rpcTempCU.
  // The exception is each slice / slice-segment must have at least one CTU.
  if (bestCS->cost != MAX_DOUBLE)
  {
    const TileMap& tileMap = *tempCS->picture->tileMap;
    const UInt CtuAddr = CU::getCtuAddr( *bestCS->getCU() );
    const Bool isEndOfSlice        =    slice.getSliceMode() == FIXED_NUMBER_OF_BYTES
                                      && ((slice.getSliceBits() + CS::getEstBits(*bestCS)) > slice.getSliceArgument() << 3)
                                      && CtuAddr != tileMap.getCtuTsToRsAddrMap(slice.getSliceCurStartCtuTsAddr())
                                      && CtuAddr != tileMap.getCtuTsToRsAddrMap(slice.getSliceSegmentCurStartCtuTsAddr());

    const Bool isEndOfSliceSegment =    slice.getSliceSegmentMode() == FIXED_NUMBER_OF_BYTES
                                      && ((slice.getSliceSegmentBits() + CS::getEstBits(*bestCS)) > slice.getSliceSegmentArgument() << 3)
                                      && CtuAddr != tileMap.getCtuTsToRsAddrMap(slice.getSliceSegmentCurStartCtuTsAddr());
                                          // Do not need to check slice condition for slice-segment since a slice-segment is a subset of a slice.
    if (isEndOfSlice || isEndOfSliceSegment)
    {
      bestCS->cost = MAX_DOUBLE;
    }
  }


  // RD check for sub partitioned coding structure.
  xCheckBestMode( tempCS, bestCS, partitioner, encTestMode );

  tempCS->releaseIntermediateData();

  tempCS->prevQP[tempCS->chType] = oldPrevQp;
}


void EncCu::xCheckRDCostIntra( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode )
{
  double bestInterCost     = m_modeCtrl->getBestInterCost(), costSize2Nx2NemtFirstPass = m_modeCtrl->getEmtSize2Nx2NFirstPassCost(), costSizeNxNemtFirstPass = MAX_DOUBLE;
  bool isAllIntra          = m_pcEncCfg->getIntraPeriod() == 1, skipSecondEmtPass = m_modeCtrl->getSkipSecondEMTPass();
  auto slsCtrl             = dynamic_cast<SaveLoadEncInfoCtrl*>( m_modeCtrl );
  const CodingUnit *bestCU = bestCS->getCU();

  const SPS &sps = *tempCS->sps;
  const PPS &pps = *tempCS->pps;

  const int maxSizeEMT              = pps.pcv->noRQT ? EMT_INTRA_MAX_CU_WITH_QTBT : EMT_INTRA_MAX_CU;
  const UChar considerEmtSecondPass = sps.getSpsNext().getUseIntraEMT() && isLuma( tempCS->chType ) && partitioner.currArea().lwidth() <= maxSizeEMT && partitioner.currArea().lheight() <= maxSizeEMT ? 1 : 0;

  const int nsstIdx  = ( encTestMode.opts & ETO_NSST ) >> ETO_NSST_SHIFT;
  const bool usePDPC =  sps.getSpsNext().isPlanarPDPC() ? false : ((encTestMode.opts & ETO_PDPC) != 0);

  Distortion interHad = m_modeCtrl->getInterHad();

  for( UChar emtCuFlag = 0; emtCuFlag <= considerEmtSecondPass; emtCuFlag++ )
  {
    //Possible early EMT tests interruptions
    //1) saveLoadTag code for EMT
    if( sps.getSpsNext().getUseQTBT() && slsCtrl && m_pcEncCfg->getUseSaveLoadEncInfo() )
    {
      if( m_pcEncCfg->getIntraEMT() && LOAD_ENC_INFO == slsCtrl->getSaveLoadTag( tempCS->area ) && ( emtCuFlag > 0 ) != slsCtrl->getSaveLoadEmtCuFlag( tempCS->area ) )
      {
        continue;
      }
    }
    //2) Second EMT pass. This "if clause" is necessary because of the NSST and PDPC "for loops".
    if( emtCuFlag && skipSecondEmtPass )
    {
      continue;
    }
    //3) if interHad is 0, only try further modes if some intra mode was already better than inter
    if( m_pcEncCfg->getUsePbIntraFast() && !tempCS->slice->isIntra() && CU::isInter( *bestCS->getCU() ) && interHad == 0 )
    {
      continue;
    }

    tempCS->initStructData( encTestMode.qp, encTestMode.lossless );

    CodingUnit &cu      = tempCS->addCU( CS::getArea( *tempCS, tempCS->area ) );

    partitioner.setCUData( cu );
    cu.slice            = tempCS->slice;
    cu.tileIdx          = tempCS->picture->tileMap->getTileIdxMap( tempCS->area.lumaPos() );
    cu.skip             = false;
    cu.partSize         = encTestMode.partSize;
    cu.predMode         = MODE_INTRA;
    cu.transQuantBypass = encTestMode.lossless;
    cu.pdpc             = usePDPC;
    cu.chromaQpAdj      = cu.transQuantBypass ? 0 : m_cuChromaQpOffsetIdxPlus1;
    cu.qp               = encTestMode.qp;
  //cu.ipcm             = false;
    cu.nsstIdx          = nsstIdx;
    cu.emtFlag          = emtCuFlag;

    CU::addPUs( cu );

    tempCS->interHad    = interHad;

    if( isLuma( tempCS->chType ) )
    {
      m_pcIntraSearch->estIntraPredLumaQT( cu, partitioner );

      if( m_pcEncCfg->getUsePbIntraFast() && tempCS->dist == MAX_UINT )
      {
        // JEM assumes only perfect reconstructions can from now on beat the inter mode
        interHad = 0;
        m_modeCtrl->enforceInterHad( interHad );
        continue;
      }

      if( !CS::isDoubleITree( *tempCS ) )
      {
        cu.cs->picture->getRecoBuf( cu.Y() ).copyFrom( cu.cs->getRecoBuf( COMPONENT_Y ) );
      }
    }

    if( tempCS->area.chromaFormat != CHROMA_400 && ( tempCS->chType == CHANNEL_TYPE_CHROMA || !CS::isDoubleITree( *tempCS ) ) )
    {
      m_pcIntraSearch->estIntraPredChromaQT( cu, partitioner );
    }

    cu.rootCbf = false;

    for( UInt t = 0; t < getNumberValidTBlocks( *cu.cs->pcv ); t++ )
    {
      cu.rootCbf |= cu.firstTU->cbf[t] != 0;
    }

    m_CABACEstimator->resetBits();

    if( pps.getTransquantBypassEnabledFlag() )
    {
      m_CABACEstimator->cu_transquant_bypass_flag( cu );
    }

    // Get total bits for current mode: encode CU
    m_CABACEstimator->resetBits();
    if( !cu.cs->slice->isIntra() )
    {
      m_CABACEstimator->cu_skip_flag ( cu );
    }
    m_CABACEstimator->pred_mode      ( cu );
    m_CABACEstimator->pdpc_flag      ( cu );
    m_CABACEstimator->part_mode      ( cu );
    m_CABACEstimator->cu_pred_data   ( cu );
    m_CABACEstimator->pcm_data       ( cu );


    // Encode Coefficients
    CUCtx cuCtx;
    cuCtx.isDQPCoded = true;
    cuCtx.isChromaQpAdjCoded = true;
    m_CABACEstimator->cu_residual( cu, partitioner, cuCtx );

    tempCS->fracBits = m_CABACEstimator->getEstFracBits();
    tempCS->cost     = m_pcRdCost->calcRdCost(tempCS->fracBits, tempCS->dist);

    xEncodeDontSplit( *tempCS, partitioner );

    if( pps.getUseDQP() && ( cu.depth ) <= pps.getMaxCuDQPDepth() )
    {
      xCheckDQP( *tempCS );
    }

    // Check if secondary transform (NSST) is too expensive
    const int nonZeroCoeffThr = CS::isDoubleITree( *tempCS ) ? ( isLuma( tempCS->chType ) ? NSST_SIG_NZ_LUMA : NSST_SIG_NZ_CHROMA ) : NSST_SIG_NZ_LUMA + NSST_SIG_NZ_CHROMA;
    if( nsstIdx && tempCS->pcv->noRQT && cuCtx.numNonZeroCoeffNonTs <= nonZeroCoeffThr )
    {
      Bool isMDIS = false;
      if (sps.getSpsNext().isPlanarPDPC() )
      {
        for (auto &pu : CU::traversePUs(cu))
        {
          isMDIS = IntraPrediction::useFilteredIntraRefSamples(COMPONENT_Y, pu, true, pu);
        }
      }

      if (cuCtx.numNonZeroCoeffNonTs > 0 || isMDIS)
      {
        CHECKD( !sps.getSpsNext().getUseNSST(), "Expecting NSST mode" );
        tempCS->cost = MAX_DOUBLE;
      }
    }

    // we save the cost of the modes for the first EMT pass
    if( !emtCuFlag ) static_cast< double& >( cu.partSize == SIZE_2Nx2N ? costSize2Nx2NemtFirstPass : costSizeNxNemtFirstPass ) = tempCS->cost;

    DTRACE_MODE_COST( *tempCS, m_pcRdCost->getLambda() );
    xCheckBestMode( tempCS, bestCS, partitioner, encTestMode );

    //now we check whether the second pass of SIZE_2Nx2N and the whole Intra SIZE_NxN should be skipped or not
    if( !emtCuFlag && !tempCS->slice->isIntra() && bestCU->predMode != MODE_INTRA && cu.partSize == SIZE_2Nx2N && m_pcEncCfg->getFastInterEMT() && ( m_pcEncCfg->getUseSaveLoadEncInfo() ? ( bestInterCost < MAX_DOUBLE ) : true ) )
    {
      const double thEmtInterFastSkipIntra = 1.4; // Skip checking Intra if "2Nx2N using DCT2" is worse than best Inter mode
      if( costSize2Nx2NemtFirstPass > thEmtInterFastSkipIntra * bestInterCost )
      {
        skipSecondEmtPass = true;
        m_modeCtrl->setSkipSecondEMTPass( true );
        break;
      }
    }
    //now we check whether the second pass of EMT with SIZE_NxN should be skipped or not
    if( !emtCuFlag && isAllIntra && cu.partSize == SIZE_NxN && m_pcEncCfg->getFastIntraEMT() )
    {
      costSize2Nx2NemtFirstPass = m_modeCtrl->getEmtSize2Nx2NFirstPassCost();
      const double thEmtIntraFastSkipNxN = 1.2; // Skip checking "NxN using EMT" if "NxN using DCT2" is worse than "2Nx2N using DCT2"
      if( costSizeNxNemtFirstPass > thEmtIntraFastSkipNxN * costSize2Nx2NemtFirstPass )
      {
        break;
      }
    }
  } //for emtCuFlag
}

void EncCu::xCheckIntraPCM(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode )
{
  tempCS->initStructData( encTestMode.qp, encTestMode.lossless );

  CodingUnit &cu      = tempCS->addCU( tempCS->area );

  partitioner.setCUData( cu );
  cu.slice            = tempCS->slice;
  cu.tileIdx          = tempCS->picture->tileMap->getTileIdxMap( tempCS->area.lumaPos() );
  cu.skip             = false;
  cu.partSize         = SIZE_2Nx2N;
  cu.predMode         = MODE_INTRA;
  cu.transQuantBypass = encTestMode.lossless;
  cu.pdpc             = false;
  cu.chromaQpAdj      = cu.transQuantBypass ? 0 : m_cuChromaQpOffsetIdxPlus1;
  cu.qp               = encTestMode.qp;
  cu.ipcm             = true;

  tempCS->addPU(tempCS->area);

  TransformUnit & tu  = tempCS->addTU(tempCS->area);
  tu.depth            = 0;

  m_pcIntraSearch->IPCMSearch(*tempCS);

  m_CABACEstimator->getCtx() = m_CurrCtx->start;

  m_CABACEstimator->resetBits();

  if( tempCS->pps->getTransquantBypassEnabledFlag() )
  {
    m_CABACEstimator->cu_transquant_bypass_flag( cu );
  }

  if( !cu.cs->slice->isIntra() )
  {
    m_CABACEstimator->cu_skip_flag ( cu );
  }
  m_CABACEstimator->pred_mode      ( cu );
  m_CABACEstimator->part_mode      ( cu );

  m_CABACEstimator->pcm_data       ( cu );


  tempCS->fracBits = m_CABACEstimator->getEstFracBits();
  tempCS->cost     = m_pcRdCost->calcRdCost(tempCS->fracBits, tempCS->dist);

  xEncodeDontSplit( *tempCS, partitioner );

  if( tempCS->pps->getUseDQP() && ( cu.depth ) <= tempCS->pps->getMaxCuDQPDepth() )
  {
    xCheckDQP( *tempCS );
  }

  DTRACE_MODE_COST( *tempCS, m_pcRdCost->getLambda() );
  xCheckBestMode( tempCS, bestCS, partitioner, encTestMode );
}

void EncCu::xCheckDQP( CodingStructure& cs, bool bKeepCtx )
{
  CHECK( !cs.getCU(), "No CU available" );

  bool hasResidual = false;
  for( const auto &cu : cs.cus )
  {
    if( cu->rootCbf )
    {
      hasResidual = true;
      break;
    }
  }

  int predQP = CU::predictQP( *cs.getCU(), cs.prevQP[cs.chType] );

  if( hasResidual )
  {
    TempCtx ctxTemp( m_CtxCache );
    if( !bKeepCtx ) ctxTemp = SubCtx( Ctx::DeltaQP, m_CABACEstimator->getCtx() );

    m_CABACEstimator->resetBits();
    m_CABACEstimator->cu_qp_delta(*cs.getCU(), predQP);

    cs.fracBits += m_CABACEstimator->getEstFracBits(); // dQP bits
    cs.cost      = m_pcRdCost->calcRdCost(cs.fracBits, cs.dist);


    if( !bKeepCtx ) m_CABACEstimator->getCtx() = SubCtx( Ctx::DeltaQP, ctxTemp );

    // NOTE: reset QPs for CUs without residuals up to first coded CU
    for( const auto &cu : cs.cus )
    {
      if( cu->rootCbf )
      {
        break;
      }
      cu->qp = predQP;
    }
  }
  else
  {
    // No residuals: reset CU QP to predicted value
    for( const auto &cu : cs.cus )
    {
      cu->qp = predQP;
    }
  }
}

void EncCu::xFillPCMBuffer( CodingUnit &cu )
{
  const ChromaFormat format        = cu.chromaFormat;
  const UInt numberValidComponents = getNumberValidComponents(format);

  for( auto &tu : CU::traverseTUs( cu ) )
  {
    for( UInt ch = 0; ch < numberValidComponents; ch++ )
    {
      const ComponentID compID = ComponentID( ch );

      const CompArea &compArea = tu.blocks[ compID ];

      const CPelBuf source      = tu.cs->getOrgBuf( compArea );
             PelBuf destination = tu.getPcmbuf( compID );

      destination.copyFrom( source );
    }
  }
}


void EncCu::xCheckRDCostMerge2Nx2N( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode )
{
  const Slice &slice = *tempCS->slice;

  CHECK( slice.getSliceType() == I_SLICE, "Merge modes not available for I-slices" );

  tempCS->initStructData( encTestMode.qp, encTestMode.lossless );

  MergeCtx mergeCtx;
  const SPS &sps = *tempCS->sps;

  if( sps.getSpsNext().getUseSubPuMvp() )
  {
    Size bufSize = g_miScaling.scale( tempCS->area.lumaSize() );
    mergeCtx.subPuMvpMiBuf    = MotionBuf( m_SubPuMiBuf,    bufSize );
    mergeCtx.subPuMvpExtMiBuf = MotionBuf( m_SubPuExtMiBuf, bufSize );
  }

  {
    // first get merge candidates
    CodingUnit cu( tempCS->area );
    cu.cs       = tempCS;
    cu.partSize = SIZE_2Nx2N;
    cu.predMode = MODE_INTER;
    cu.slice    = tempCS->slice;
    cu.tileIdx  = tempCS->picture->tileMap->getTileIdxMap(tempCS->area.lumaPos());

    PredictionUnit pu( tempCS->area );
    pu.cu = &cu;
    pu.cs = tempCS;

    PU::getInterMergeCandidates(pu, mergeCtx);
  }


  Int candHasNoResidual[MRG_MAX_NUM_CANDS];
  for (UInt ui = 0; ui < mergeCtx.numValidMergeCand; ui++)
  {
    candHasNoResidual[ui] = 0;
  }

  bool                                        bestIsSkip       = false;
  unsigned                                    uiNumMrgSATDCand = mergeCtx.numValidMergeCand;
  PelUnitBuf                                  acMergeBuffer    [ MRG_MAX_NUM_CANDS ];
  static_vector<unsigned, MRG_MAX_NUM_CANDS>  RdModeList       ( MRG_MAX_NUM_CANDS );
  bool                                        mrgTempBufSet    = false;

  for( unsigned i = 0; i < MRG_MAX_NUM_CANDS; i++ )
  {
    RdModeList[i] = i;
  }

  if( m_pcEncCfg->getUseFastMerge() )
  {
    uiNumMrgSATDCand = NUM_MRG_SATD_CAND;
    bestIsSkip       = false;

    if( auto blkCache = dynamic_cast<CacheBlkInfoCtrl*>( m_modeCtrl ) )
    {
      bestIsSkip = blkCache->isSkip( tempCS->area );
    }

    static_vector<double,   MRG_MAX_NUM_CANDS>  candCostList( MRG_MAX_NUM_CANDS, MAX_DOUBLE );

    // 1. Pass: get SATD-cost for selected candidates and reduce their count
    if( !bestIsSkip )
    {
      mrgTempBufSet       = true;
      const double sqrtLambdaForFirstPass = m_pcRdCost->getMotionLambda( encTestMode.lossless );

      CodingUnit &cu      = tempCS->addCU( tempCS->area );

      partitioner.setCUData( cu );
      cu.slice            = tempCS->slice;
      cu.tileIdx          = tempCS->picture->tileMap->getTileIdxMap( tempCS->area.lumaPos() );
      cu.skip             = false;
      cu.partSize         = SIZE_2Nx2N;
    //cu.affine
      cu.predMode         = MODE_INTER;
    //cu.LICFlag
      cu.transQuantBypass = encTestMode.lossless;
      cu.chromaQpAdj      = cu.transQuantBypass ? 0 : m_cuChromaQpOffsetIdxPlus1;
      cu.qp               = encTestMode.qp;
    //cu.emtFlag  is set below
      cu.obmcFlag         = sps.getSpsNext().getUseOBMC();

      DistParam distParam;
      const Bool bUseHadamard= !encTestMode.lossless;
      m_pcRdCost->setDistParam (distParam, tempCS->getOrgBuf().Y(), m_acMergeBuffer[0].Y(), sps.getBitDepth (CHANNEL_TYPE_LUMA), COMPONENT_Y, bUseHadamard);

      const UnitArea localUnitArea( tempCS->area.chromaFormat, Area( 0, 0, tempCS->area.Y().width, tempCS->area.Y().height) );

      for( UInt uiMergeCand = 0; uiMergeCand < mergeCtx.numValidMergeCand; uiMergeCand++ )
      {
        acMergeBuffer[uiMergeCand] = m_acMergeBuffer[uiMergeCand].getBuf( localUnitArea );
        PredictionUnit &pu = tempCS->addPU( cu );

        mergeCtx.setMergeInfo( pu, uiMergeCand );

        PU::spanMotionInfo( pu, mergeCtx );

        distParam.cur = acMergeBuffer[uiMergeCand].Y();

        pu.mvRefine = true;
        m_pcInterSearch->motionCompensation( pu,  acMergeBuffer[uiMergeCand] );
        pu.mvRefine = false;

        m_pcInterSearch->subBlockOBMC      ( pu, &acMergeBuffer[uiMergeCand], false );
/*
        DTRACE( g_trace_ctx, D_TMP, "TMP: MV0(%7d %7d %7d) MV1(%7d %7d %7d)\n",
          mergeCtx.mvFieldNeighbours[0 + 2*uiMergeCand].mv.getHor(), mergeCtx.mvFieldNeighbours[0 + 2*uiMergeCand].mv.getVer(), mergeCtx.mvFieldNeighbours[0 + 2*uiMergeCand].refIdx,
          mergeCtx.mvFieldNeighbours[1 + 2*uiMergeCand].mv.getHor(), mergeCtx.mvFieldNeighbours[1 + 2*uiMergeCand].mv.getVer(), mergeCtx.mvFieldNeighbours[1 + 2*uiMergeCand].refIdx);

        DTRACE_CRC( g_trace_ctx, D_TMP, tempCS, acMergeBuffer[uiMergeCand] );
*/
        if( mergeCtx.interDirNeighbours[uiMergeCand] == 3 && mergeCtx.mrgTypeNeighnours[uiMergeCand] == MRG_TYPE_DEFAULT_N )
        {
          mergeCtx.mvFieldNeighbours[2*uiMergeCand].mv   = pu.mv[0];
          mergeCtx.mvFieldNeighbours[2*uiMergeCand+1].mv = pu.mv[1];
        }

        UInt uiSad      = distParam.distFunc( distParam );
        UInt uiBitsCand = uiMergeCand + 1;
        if( uiMergeCand == tempCS->slice->getMaxNumMergeCand() - 1 )
        {
          uiBitsCand--;
        }
        Double cost     = (Double)uiSad + (Double)uiBitsCand * sqrtLambdaForFirstPass;

        updateCandList( uiMergeCand, cost, RdModeList, candCostList, uiNumMrgSATDCand );
      }

      // Try to limit number of candidates using SATD-costs
      for( UInt i = 1; i < uiNumMrgSATDCand; i++ )
      {
        if( candCostList[i] > MRG_FAST_RATIO*candCostList[0] )
        {
          uiNumMrgSATDCand = i;
          break;
        }
      }

      tempCS->initStructData( encTestMode.qp, encTestMode.lossless );
    }
  }

  const UInt iteration = encTestMode.lossless ? 1 : 2;

  // 2. Pass: check candidates using full RD test
  for( UInt uiNoResidualPass = 0; uiNoResidualPass < iteration; uiNoResidualPass++ )
  {
    for( UInt uiMrgHADIdx = 0; uiMrgHADIdx < uiNumMrgSATDCand; uiMrgHADIdx++ )
    {
      UInt uiMergeCand = RdModeList[uiMrgHADIdx];

      if( !( uiNoResidualPass == 1 && candHasNoResidual[uiMergeCand] == 1 ) )
      {
        if( !( bestIsSkip && ( uiNoResidualPass == 0 ) ) )
        {
          const int maxSizeEMT        = tempCS->pcv->noRQT ? EMT_INTER_MAX_CU_WITH_QTBT : EMT_INTER_MAX_CU;
          double bestCost             = bestCS->cost;
          UChar considerEmtSecondPass = sps.getSpsNext().getUseInterEMT() && partitioner.currArea().lwidth() <= maxSizeEMT && partitioner.currArea().lheight() <= maxSizeEMT ? 1 : 0;
          bool skipSecondEmtPass      = false;
          bool hasResidual[2]         = { false, false };
          double emtCost[2]           = { MAX_DOUBLE, MAX_DOUBLE };

          // CU-level optimization
          for( UChar emtCuFlag = 0; emtCuFlag <= considerEmtSecondPass; emtCuFlag++ )
          {
            if( m_pcEncCfg->getFastInterEMT() && emtCuFlag && skipSecondEmtPass )
            {
              continue;
            }

            // first get merge candidates
            CodingUnit &cu      = tempCS->addCU( tempCS->area );

            partitioner.setCUData( cu );
            cu.slice            = tempCS->slice;
            cu.tileIdx          = tempCS->picture->tileMap->getTileIdxMap( tempCS->area.lumaPos() );
            cu.skip             = false;
            cu.partSize         = SIZE_2Nx2N;
          //cu.affine
            cu.predMode         = MODE_INTER;
          //cu.LICFlag
            cu.transQuantBypass = encTestMode.lossless;
            cu.chromaQpAdj      = cu.transQuantBypass ? 0 : m_cuChromaQpOffsetIdxPlus1;
            cu.qp               = encTestMode.qp;
            cu.emtFlag          = emtCuFlag;
            cu.obmcFlag         = sps.getSpsNext().getUseOBMC();

            PredictionUnit &pu  = tempCS->addPU( cu );

            mergeCtx.setMergeInfo( pu, uiMergeCand );

            PU::spanMotionInfo( pu, mergeCtx );

            if( mrgTempBufSet )
            {
              tempCS->getPredBuf().copyFrom( acMergeBuffer[ uiMergeCand ]);
            }
            else
            {
              pu.mvRefine = true;
              m_pcInterSearch->motionCompensation( pu );
              pu.mvRefine = false;

              m_pcInterSearch->subBlockOBMC      ( pu );

              if( mergeCtx.interDirNeighbours[uiMergeCand] == 3 && mergeCtx.mrgTypeNeighnours[uiMergeCand] == MRG_TYPE_DEFAULT_N )
              {
                mergeCtx.mvFieldNeighbours[2*uiMergeCand].mv   = pu.mv[0];
                mergeCtx.mvFieldNeighbours[2*uiMergeCand+1].mv = pu.mv[1];
              }
            }
            // estimate residual and encode everything
            m_CABACEstimator->getCtx() = m_CurrCtx->start;

            m_pcInterSearch->encodeResAndCalcRdInterCU( *tempCS, partitioner, ( uiNoResidualPass != 0 ) );

            xEncodeDontSplit( *tempCS, partitioner );

            if( tempCS->pps->getUseDQP() && ( cu.depth ) <= tempCS->pps->getMaxCuDQPDepth() )
            {
              xCheckDQP( *tempCS );
            }

            //now we check whether the second pass should be skipped or not
            if( !emtCuFlag && m_pcEncCfg->getFastInterEMT() && considerEmtSecondPass )
            {
              if( !cu.firstTU->cbf[COMPONENT_Y] )
              {
                skipSecondEmtPass = true;   // Skip checking EMT transforms
              }
              else
              {
                static const double thresholdToSkipEmtSecondPass = 1.1;
                if( tempCS->cost > bestCost * thresholdToSkipEmtSecondPass )
                {
                  skipSecondEmtPass = true; // Skip checking EMT transforms
                }
              }
            }

            hasResidual[emtCuFlag] = cu.rootCbf;
            emtCost    [emtCuFlag] = tempCS->cost;

            DTRACE_MODE_COST( *tempCS, m_pcRdCost->getLambda() );
            xCheckBestMode( tempCS, bestCS, partitioner, encTestMode );

            tempCS->initStructData( encTestMode.qp, encTestMode.lossless );
          }

          if( uiNoResidualPass == 0 && ( emtCost[0] <= emtCost[1] ? !hasResidual[0] : !hasResidual[1] ) )
          {
            // If no residual when allowing for one, then set mark to not try case where residual is forced to 0
            candHasNoResidual[uiMergeCand] = 1;
          }

          if( m_pcEncCfg->getUseFastDecisionForMerge() && !bestIsSkip )
          {
            bestIsSkip = bestCS->getCU()->rootCbf == 0;
          }
        }
      }
    }

    if( uiNoResidualPass == 0 && m_pcEncCfg->getUseEarlySkipDetection() )
    {
      const CodingUnit &bestCU = *bestCS->getCU();
      const PredictionUnit &bestPU = *bestCS->getPU();

      if( bestCU.rootCbf == 0 )
      {
        if( bestPU.mergeFlag )
        {
          m_modeCtrl->setEarlySkipDetected();
        }
        else if( m_pcEncCfg->getMotionEstimationSearchMethod() != MESEARCH_SELECTIVE )
        {
          Int absolute_MV = 0;

          for( UInt uiRefListIdx = 0; uiRefListIdx < 2; uiRefListIdx++ )
          {
            if( slice.getNumRefIdx( RefPicList( uiRefListIdx ) ) > 0 )
            {
              Int iHor = bestPU.mvd[uiRefListIdx].getAbsHor();
              Int iVer = bestPU.mvd[uiRefListIdx].getAbsVer();

              absolute_MV += iHor + iVer;
            }
          }

          if( absolute_MV == 0 )
          {
            m_modeCtrl->setEarlySkipDetected();
          }
        }
      }
    }
  }
}

void EncCu::xCheckRDCostAffineMerge2Nx2N( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode )
{
  if( m_modeCtrl->getFastDeltaQp() )
  {
    return;
  }

  if( tempCS->pcv->rectCUs && bestCS->area.lumaSize().area() < 64 )
  {
    return;
  }

  MvField       affineMvField[2][3];
  unsigned char interDirNeighbours;
  int           numValidMergeCand;

  const UInt iteration = encTestMode.lossless ? 1 : 2;
  const SPS &sps       = *tempCS->sps;

  for( UInt uiNoResidualPass = 0; uiNoResidualPass < iteration; uiNoResidualPass++ )
  {
    const int maxSizeEMT        = tempCS->pcv->noRQT ? EMT_INTER_MAX_CU_WITH_QTBT : EMT_INTER_MAX_CU;
    double bestCost             = bestCS->cost;
    UChar considerEmtSecondPass = sps.getSpsNext().getUseInterEMT() && partitioner.currArea().lwidth() <= maxSizeEMT && partitioner.currArea().lheight() <= maxSizeEMT ? 1 : 0;
    bool skipSecondEmtPass      = false;
    bool hasResidual[2]         = { false, false };
    double emtCost[2]           = { MAX_DOUBLE, MAX_DOUBLE };

    // CU-level optimization
    for( UChar emtCuFlag = 0; emtCuFlag <= considerEmtSecondPass; emtCuFlag++ )
    {
      if( m_pcEncCfg->getFastInterEMT() && emtCuFlag && skipSecondEmtPass )
      {
        continue;
      }

      tempCS->initStructData( encTestMode.qp, encTestMode.lossless );

      CodingUnit &cu      = tempCS->addCU( tempCS->area );

      partitioner.setCUData( cu );
      cu.slice            = tempCS->slice;
      cu.tileIdx          = tempCS->picture->tileMap->getTileIdxMap( tempCS->area.lumaPos() );
      cu.skip             = false;
      cu.partSize         = encTestMode.partSize;
      cu.affine           = true;
      cu.predMode         = MODE_INTER;
      cu.LICFlag          = false;
      cu.transQuantBypass = encTestMode.lossless;
      cu.chromaQpAdj      = cu.transQuantBypass ? 0 : m_cuChromaQpOffsetIdxPlus1;
      cu.qp               = encTestMode.qp;
      cu.emtFlag          = emtCuFlag;
      cu.obmcFlag         = sps.getSpsNext().getUseOBMC();

      CU::addPUs( cu );

      cu.firstPU->mergeFlag = true;
      cu.firstPU->mergeIdx  = 0;

      if( uiNoResidualPass == 0 )
      {
        PU::getAffineMergeCand( *cu.firstPU, affineMvField, interDirNeighbours, numValidMergeCand );
        if( numValidMergeCand == -1 )
        {
          return;
        }
      }

      cu.firstPU->interDir = interDirNeighbours;
      PU::setAllAffineMvField( *cu.firstPU, affineMvField[REF_PIC_LIST_0], REF_PIC_LIST_0 );
      PU::setAllAffineMvField( *cu.firstPU, affineMvField[REF_PIC_LIST_1], REF_PIC_LIST_1 );

      PU::spanMotionInfo( *cu.firstPU );

      m_pcInterSearch->motionCompensation( cu ); //TODO: consider doing MC only once like in merge and fruc est
      m_pcInterSearch->subBlockOBMC      ( cu );

      m_CABACEstimator->getCtx() = m_CurrCtx->start;
      m_pcInterSearch->encodeResAndCalcRdInterCU( *tempCS, partitioner, ( uiNoResidualPass != 0 ) );

      xEncodeDontSplit( *tempCS, partitioner );

      if( tempCS->pps->getUseDQP() && ( cu.qtDepth + cu.btDepth ) <= tempCS->pps->getMaxCuDQPDepth() )
      {
        xCheckDQP( *tempCS );
      }

      //now we check whether the second pass should be skipped or not
      if( !emtCuFlag && m_pcEncCfg->getFastInterEMT() && considerEmtSecondPass )
      {
        if( !cu.firstTU->cbf[COMPONENT_Y] )
        {
          skipSecondEmtPass = true;   // Skip checking EMT transforms
        }
        else
        {
          static const double thresholdToSkipEmtSecondPass = 1.1;
          if( tempCS->cost > bestCost * thresholdToSkipEmtSecondPass )
          {
            skipSecondEmtPass = true; // Skip checking EMT transforms
          }
        }
      }

      hasResidual[emtCuFlag] = cu.rootCbf;
      emtCost    [emtCuFlag] = tempCS->cost;
      cu.skip = ( cu.rootCbf == 0 );
      DTRACE_MODE_COST( *tempCS, m_pcRdCost->getLambda() );
      xCheckBestMode( tempCS, bestCS, partitioner, encTestMode );

      tempCS->initStructData( encTestMode.qp, encTestMode.lossless ); // could be unnecessary here
    }

    if( uiNoResidualPass == 0 && ( emtCost[0] <= emtCost[1] ? !hasResidual[0] : !hasResidual[1] ) )
    {
      // If no residual when allowing for one, then set mark to not try case where residual is forced to 0
      uiNoResidualPass++;
    }
  }
}

void EncCu::xCheckRDCostInter( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode )
{
  tempCS->initStructData( encTestMode.qp, encTestMode.lossless );

  const SPS &sps      = *tempCS->sps;
  CodingUnit &cu      = tempCS->addCU( tempCS->area );

  partitioner.setCUData( cu );
  cu.slice            = tempCS->slice;
  cu.tileIdx          = tempCS->picture->tileMap->getTileIdxMap( tempCS->area.lumaPos() );
  cu.skip             = false;
  cu.partSize         = encTestMode.partSize;
//cu.affine
  cu.predMode         = MODE_INTER;
  cu.LICFlag          = ( ( encTestMode.opts & ETO_LIC ) != 0 );
  cu.transQuantBypass = encTestMode.lossless;
  cu.chromaQpAdj      = cu.transQuantBypass ? 0 : m_cuChromaQpOffsetIdxPlus1;
  cu.qp               = encTestMode.qp;
//cu.emtFlag  is set below
  cu.obmcFlag         = sps.getSpsNext().getUseOBMC();

  CU::addPUs( cu );

#if AMP_MRG
  m_pcInterSearch->predInterSearch( cu, partitioner, ( encTestMode.opts & ETO_FORCE_MERGE ) );
#else
  m_pcInterSearch->predInterSearch( cu, partitioner );
#endif

  if( m_pTempCUWoOBMC )
  {
    Int wIdx = gp_sizeIdxInfo->idxFrom( tempCS->area.lwidth () );
    Int hIdx = gp_sizeIdxInfo->idxFrom( tempCS->area.lheight() );

    m_pTempCUWoOBMC[wIdx][hIdx]->clearCUs();
    m_pTempCUWoOBMC[wIdx][hIdx]->clearPUs();
    m_pTempCUWoOBMC[wIdx][hIdx]->clearTUs();
    m_pTempCUWoOBMC[wIdx][hIdx]->copyStructure( *tempCS );

    m_pPredBufWoOBMC[wIdx][hIdx].copyFrom( tempCS->getPredBuf( cu ) );

    m_pcInterSearch->subBlockOBMC( cu );

    m_pTempCUWoOBMC[wIdx][hIdx]->getPredBuf( cu ).copyFrom( tempCS->getPredBuf( cu ) );
  }

  const int maxSizeEMT            = tempCS->pcv->noRQT ? EMT_INTER_MAX_CU_WITH_QTBT : EMT_INTER_MAX_CU;
  double    bestCost              = bestCS->cost;
  UChar     considerEmtSecondPass = sps.getSpsNext().getUseInterEMT() && partitioner.currArea().lwidth() <= maxSizeEMT && partitioner.currArea().lheight() <= maxSizeEMT ? 1 : 0;
  bool      skipSecondEmtPass     = false;
  double    emtFirstPassCost      = MAX_DOUBLE;

  // CU-level optimization
  for( UChar emtCuFlag = 0; emtCuFlag <= considerEmtSecondPass; emtCuFlag++ )
  {
    if( m_pcEncCfg->getFastInterEMT() && emtCuFlag && skipSecondEmtPass )
    {
      continue;
    }

    tempCS->getCU()->emtFlag = emtCuFlag;

    m_pcInterSearch->encodeResAndCalcRdInterCU( *tempCS, partitioner, false );

    if( m_pcEncCfg->getFastInterEMT() )
    {
      emtFirstPassCost = (!emtCuFlag) ? tempCS->cost : emtFirstPassCost;
    }

    xEncodeDontSplit( *tempCS, partitioner );

    if( tempCS->pps->getUseDQP() && ( cu.depth ) <= tempCS->pps->getMaxCuDQPDepth() )
    {
      xCheckDQP( *tempCS );
    }

    if( m_pImvTempCS )
    {
      CodingStructure* imvCS = m_pImvTempCS[gp_sizeIdxInfo->idxFrom( tempCS->area.lwidth() )][encTestMode.partSize];

      if( tempCS->cost < imvCS->cost )
      {
        if( imvCS->cost != MAX_DOUBLE )
        {
          imvCS->initStructData( encTestMode.qp, encTestMode.lossless );
        }
        imvCS->copyStructure( *tempCS );
      }
    }

    DTRACE_MODE_COST( *tempCS, m_pcRdCost->getLambda() );
    xCheckBestMode( tempCS, bestCS, partitioner, encTestMode );

    //now we check whether the second pass should be skipped or not
    if( !emtCuFlag && considerEmtSecondPass )
    {
      static const double thresholdToSkipEmtSecondPass = 1.1; // Skip checking EMT transforms
      if( m_pcEncCfg->getFastInterEMT() && ( !cu.firstTU->cbf[COMPONENT_Y] || emtFirstPassCost > bestCost * thresholdToSkipEmtSecondPass ) )
      {
        skipSecondEmtPass = true;
      }
      else //EMT will be checked
      {
        if( bestCost == bestCS->cost ) //The first EMT pass didn't become the bestCS, so we clear the TUs generated
        {
          tempCS->clearTUs();
        }
        else
        {
          tempCS->initStructData( bestCS->currQP[bestCS->chType], bestCS->isLossless );
          tempCS->copyStructure ( *bestCS );
          tempCS->getPredBuf().copyFrom( bestCS->getPredBuf() );
        }

        //we need to restart the distortion for the new tempCS, the bit count and the cost
        tempCS->dist     = 0;
        tempCS->fracBits = 0;
        tempCS->cost     = MAX_DOUBLE;
      }
    }
  }
}


void EncCu::xCheckRDCostInterWoOBMC( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode )
{
  if( !tempCS->sps->getSpsNext().getUseOBMC() )
  {
    return;
  }

  if( m_modeCtrl->getFastDeltaQp() )
  {
    if( encTestMode.partSize != SIZE_2Nx2N || tempCS->area.lumaSize().width > tempCS->pcv->fastDeltaQPCuMaxSize )
    {
      return; // only check necessary 2Nx2N Inter in fast deltaqp mode
    }
  }

  tempCS->initStructData( encTestMode.qp, encTestMode.lossless );

  CodingStructure* CSWoOBMC = nullptr;
  const SPS &sps            = *tempCS->sps;

  Int wIdx = gp_sizeIdxInfo->idxFrom( tempCS->area.lwidth () );
  Int hIdx = gp_sizeIdxInfo->idxFrom( tempCS->area.lheight() );

  CSWoOBMC = m_pTempCUWoOBMC[wIdx][hIdx];

  CodingUnit *cu = CSWoOBMC->getCU();

  if( !cu->obmcFlag )
  {
    return;
  }

  Bool bCheckOBMCOff = true;

  const Double dOBMCThOff = 1.0;

  UInt uiSADOBMCOff = m_pcRdCost->getDistPart( tempCS->getOrgBuf( cu->Y() ), m_pPredBufWoOBMC[wIdx][hIdx].Y(), sps.getBitDepth( CHANNEL_TYPE_LUMA ), COMPONENT_Y, DF_SAD_FULL_NBIT );
  UInt uiSADOBMCOn  = m_pcRdCost->getDistPart( tempCS->getOrgBuf( cu->Y() ), CSWoOBMC->getPredBuf( cu->Y() ),  sps.getBitDepth( CHANNEL_TYPE_LUMA ), COMPONENT_Y, DF_SAD_FULL_NBIT );

  bCheckOBMCOff = uiSADOBMCOff * dOBMCThOff < uiSADOBMCOn;

  if( CU::isObmcFlagCoded( *cu ) && bCheckOBMCOff )
  {
    tempCS->copyStructure( *CSWoOBMC );
    tempCS->getPredBuf( *cu ).copyFrom( m_pPredBufWoOBMC[wIdx][hIdx] );
    cu           = tempCS->getCU();
    cu->obmcFlag = false;
    CHECK( cu->firstPU->mergeFlag && cu->partSize == SIZE_2Nx2N, "Merge2Nx2Ns is on" );
  }
  else
  {
    return;
  }

  const int maxSizeEMT            = tempCS->pcv->noRQT ? EMT_INTER_MAX_CU_WITH_QTBT : EMT_INTER_MAX_CU;
  double    bestCost              = bestCS->cost;
  UChar     considerEmtSecondPass = sps.getSpsNext().getUseInterEMT() && partitioner.currArea().lwidth() <= maxSizeEMT && partitioner.currArea().lheight() <= maxSizeEMT ? 1 : 0;
  bool      skipSecondEmtPass     = m_pcEncCfg->getFastInterEMT(); // skip emt if OBMC is allowed but checking without obmc
  double    emtFirstPassCost      = MAX_DOUBLE;

  // CU-level optimization
  for( UChar emtCuFlag = 0; emtCuFlag <= considerEmtSecondPass; emtCuFlag++ )
  {
    if( m_pcEncCfg->getFastInterEMT() && emtCuFlag && skipSecondEmtPass )
    {
      continue;
    }

    cu->emtFlag = emtCuFlag;

    m_pcInterSearch->encodeResAndCalcRdInterCU( *tempCS, partitioner, false );

    xEncodeDontSplit( *tempCS, partitioner );

    if( tempCS->pps->getUseDQP() && ( cu->depth ) <= tempCS->pps->getMaxCuDQPDepth() )
    {
      xCheckDQP( *tempCS );
    }

    if( m_pImvTempCS )
    {
      CodingStructure* imvCS = m_pImvTempCS[wIdx][encTestMode.partSize];

      if( tempCS->cost < imvCS->cost )
      {
        if( imvCS->cost != MAX_DOUBLE )
        {
          imvCS->initStructData( encTestMode.qp, encTestMode.lossless );
        }
        imvCS->copyStructure( *tempCS );
      }
    }

    DTRACE_MODE_COST( *tempCS, m_pcRdCost->getLambda() );
    xCheckBestMode( tempCS, bestCS, partitioner, encTestMode );

    //now we check whether the second pass should be skipped or not
    if( !emtCuFlag && !skipSecondEmtPass && considerEmtSecondPass )
    {
      static const double thresholdToSkipEmtSecondPass = 1.1; // Skip checking EMT transforms
      if( m_pcEncCfg->getFastInterEMT() && ( !cu->firstTU->cbf[COMPONENT_Y] || emtFirstPassCost > bestCost * thresholdToSkipEmtSecondPass ) )
      {
        skipSecondEmtPass = true;
      }
      else //EMT will be checked
      {
        if( bestCost == bestCS->cost ) //The first EMT pass didn't become the bestCS, so we clear the TUs generated
        {
          tempCS->clearTUs();
        }
        else
        {
          tempCS->initStructData( bestCS->currQP[bestCS->chType], bestCS->isLossless );
          tempCS->copyStructure ( *CSWoOBMC );
          tempCS->getPredBuf().copyFrom( m_pPredBufWoOBMC[wIdx][hIdx] );
          cu           = tempCS->getCU();
          cu->obmcFlag = false;
        }

        //we need to restart the distortion for the new tempCS, the bit count and the cost
        tempCS->dist     = 0;
        tempCS->fracBits = 0;
        tempCS->cost     = MAX_DOUBLE;
      }
    }
  }
}

void EncCu::xCheckRDCostMerge2Nx2NFRUC( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode )
{
  const Slice &slice = *tempCS->slice;
  auto slsCtrl = dynamic_cast< SaveLoadEncInfoCtrl* >( m_modeCtrl );

  if( !slice.getSPS()->getSpsNext().getUseFRUCMrgMode() )
  {
    return;
  }

  CHECK( slice.getSliceType() == I_SLICE, "Merge modes not available for I-slices" );

  if( m_modeCtrl->getFastDeltaQp() )
  {
    return;   // never check merge in fast delta-qp mode
  }

  tempCS->initStructData( encTestMode.qp, encTestMode.lossless );

  Bool transQuantBypass = tempCS->isLossless;

  MergeCtx mergeCtx;
  const SPS &sps = *tempCS->sps;
  Size bufSize = g_miScaling.scale( tempCS->area.lumaSize() );
  mergeCtx.subPuFrucMiBuf = MotionBuf( m_SubPuFrucBuf, bufSize );

  const UChar uhFRUCME[2] = { FRUC_MERGE_BILATERALMV, FRUC_MERGE_TEMPLATE };

  PelUnitBuf acMergeBuffer[2];

  for( Int nME = 0; nME < 2; nME++ )
  {
    if( slsCtrl && m_pcEncCfg->getUseSaveLoadEncInfo() && LOAD_ENC_INFO == slsCtrl->getSaveLoadTag( tempCS->area ) && uhFRUCME[nME] != slsCtrl->getSaveLoadFrucMode( tempCS->area ) )
    {
      continue;
    }

    Bool bAvailable        = false;

    CodingUnit &cu         = tempCS->addCU( tempCS->area );

    partitioner.setCUData( cu );
    cu.slice               = tempCS->slice;
    cu.tileIdx             = tempCS->picture->tileMap->getTileIdxMap( tempCS->area.lumaPos() );
  //cu.skip                = false;
    cu.partSize            = SIZE_2Nx2N;
  //cu.affine
    cu.predMode            = MODE_INTER;
    cu.LICFlag             = ( ( encTestMode.opts & ETO_LIC ) != 0 );
    cu.obmcFlag            = sps.getSpsNext().getUseOBMC();

    PredictionUnit &puFruc = tempCS->addPU( cu );
    puFruc.mergeFlag       = true;
    puFruc.mergeIdx        = 0;
    puFruc.frucMrgMode     = uhFRUCME[nME];
    puFruc.mergeType       = MRG_TYPE_FRUC;

    bAvailable = m_pcInterSearch->deriveFRUCMV( puFruc );

    if( bAvailable )
    {
      const UnitArea localUnitArea( tempCS->area.chromaFormat, Area( 0, 0, tempCS->area.lwidth(), tempCS->area.lheight() ) );
      acMergeBuffer[nME] = m_acMergeBuffer[nME].getBuf( localUnitArea );

      m_pcInterSearch->motionCompensation( puFruc,  acMergeBuffer[nME] );
      m_pcInterSearch->subBlockOBMC      ( puFruc, &acMergeBuffer[nME], false );

      CHECK( mergeCtx.subPuFrucMiBuf.area() == 0 || !mergeCtx.subPuFrucMiBuf.buf, "Buffer not initialized" );
      mergeCtx.subPuFrucMiBuf.fill( MotionInfo() );
      mergeCtx.subPuFrucMiBuf.copyFrom( puFruc.getMotionBuf() );

      const UInt iteration = encTestMode.lossless ? 1 : 2;

      for( UInt uiNoResidualPass = 0; uiNoResidualPass < iteration; uiNoResidualPass++ )
      {
        const int maxSizeEMT        = tempCS->pcv->noRQT ? EMT_INTER_MAX_CU_WITH_QTBT : EMT_INTER_MAX_CU;
        double bestCost             = bestCS->cost;
        UChar considerEmtSecondPass = sps.getSpsNext().getUseInterEMT() && partitioner.currArea().lwidth() <= maxSizeEMT && partitioner.currArea().lheight() <= maxSizeEMT ? 1 : 0;
        bool skipSecondEmtPass      = false;
        bool hasResidual[2]         = { false, false };
        double emtCost[2]           = { MAX_DOUBLE, MAX_DOUBLE };

        // CU-level optimization
        for( UChar emtCuFlag = 0; emtCuFlag <= considerEmtSecondPass; emtCuFlag++ )
        {
          if( m_pcEncCfg->getFastInterEMT() && emtCuFlag && skipSecondEmtPass )
          {
            continue;
          }

          tempCS->initStructData( encTestMode.qp, encTestMode.lossless );
          CodingUnit &cu      = tempCS->addCU( tempCS->area );

          partitioner.setCUData( cu );
          cu.slice            = tempCS->slice;
          cu.tileIdx          = tempCS->picture->tileMap->getTileIdxMap( tempCS->area.lumaPos() );
          cu.skip             = false;
          cu.partSize         = SIZE_2Nx2N;
        //cu.affine
          cu.predMode         = MODE_INTER;
          cu.LICFlag          = ( ( encTestMode.opts & ETO_LIC ) != 0 );
          cu.transQuantBypass = transQuantBypass;
          cu.chromaQpAdj      = transQuantBypass ? 0 : m_cuChromaQpOffsetIdxPlus1;
          cu.qp               = tempCS->currQP[tempCS->chType];
          cu.emtFlag          = emtCuFlag;
          cu.obmcFlag         = sps.getSpsNext().getUseOBMC();

          PredictionUnit &pu  = tempCS->addPU( cu );
          pu.mergeFlag        = true;
          pu.mergeIdx         = 0;
          pu.frucMrgMode      = uhFRUCME[nME];

          pu.mergeType        = MRG_TYPE_FRUC_SET;
          PU::spanMotionInfo( pu, mergeCtx );
          pu.mergeType        = MRG_TYPE_FRUC;

          tempCS->getPredBuf().copyFrom( acMergeBuffer[ nME ]);

          // estimate residual and encode everything
          m_CABACEstimator->getCtx() = m_CurrCtx->start;

          m_pcInterSearch->encodeResAndCalcRdInterCU( *tempCS, partitioner, ( uiNoResidualPass != 0 ) );

          xEncodeDontSplit( *tempCS, partitioner );

          if( tempCS->pps->getUseDQP() && ( cu.qtDepth + cu.btDepth ) <= tempCS->pps->getMaxCuDQPDepth() )
          {
            xCheckDQP( *tempCS );
          }

          //now we check whether the second pass should be skipped or not
          if( !emtCuFlag && m_pcEncCfg->getFastInterEMT() && considerEmtSecondPass )
          {
            if( !cu.firstTU->cbf[COMPONENT_Y] )
            {
              skipSecondEmtPass = true;   // Skip checking EMT transforms
            }
            else
            {
              static const double thresholdToSkipEmtSecondPass = 1.1;
              if( tempCS->cost > bestCost * thresholdToSkipEmtSecondPass )
              {
                skipSecondEmtPass = true; // Skip checking EMT transforms
              }
            }
          }

          hasResidual[emtCuFlag] = cu.rootCbf;
          emtCost    [emtCuFlag] = tempCS->cost;

          DTRACE_MODE_COST( *tempCS, m_pcRdCost->getLambda() );
          xCheckBestMode( tempCS, bestCS, partitioner, encTestMode );
        }

        if( uiNoResidualPass == 0 && ( emtCost[0] <= emtCost[1] ? !hasResidual[0] : !hasResidual[1] ) )
        {
          // If no residual when allowing for one, then set mark to not try case where residual is forced to 0
          uiNoResidualPass++;
        }
      }
    }
  }
}



bool EncCu::xCheckRDCostInterIMV( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode )
{
  int iIMV = int( ( encTestMode.opts & ETO_IMV ) >> ETO_IMV_SHIFT );
  // Only Int-Pel, 4-Pel and fast 4-Pel allowed
  CHECK( iIMV != 1 && iIMV != 2 && iIMV != 3, "Unsupported IMV Mode" );
  // Fast 4-Pel Mode

  EncTestMode encTestModeBase = encTestMode;                                        // copy for clearing non-IMV options
  encTestModeBase.opts        = EncTestModeOpts( encTestModeBase.opts & ETO_IMV );  // clear non-IMV options (is that intended?)

  tempCS->initStructData( encTestMode.qp, encTestMode.lossless );

  CodingStructure* pcCUInfo2Reuse = nullptr;
  const SPS &sps                  = *tempCS->sps;

  if( m_pImvTempCS && encTestMode.partSize != SIZE_2Nx2N && ( ( encTestMode.opts & ETO_FORCE_MERGE ) == 0 ) )
  {
    pcCUInfo2Reuse = m_pImvTempCS[gp_sizeIdxInfo->idxFrom( tempCS->area.lwidth() )][encTestMode.partSize];

    if( pcCUInfo2Reuse != nullptr )
    {
      CHECK( tempCS->area != pcCUInfo2Reuse->area, " mismatch" );
      tempCS->copyStructure( *pcCUInfo2Reuse );
      tempCS->fracBits = 0;
      tempCS->dist     = 0;
      tempCS->cost     = MAX_DOUBLE;
    }
  }

  CodingUnit &cu = ( pcCUInfo2Reuse != nullptr ) ? *tempCS->getCU() : tempCS->addCU( tempCS->area );

  if( pcCUInfo2Reuse == nullptr )
  {
    partitioner.setCUData( cu );
    cu.slice            = tempCS->slice;
    cu.tileIdx          = tempCS->picture->tileMap->getTileIdxMap( tempCS->area.lumaPos() );
    cu.skip             = false;
    cu.partSize         = encTestMode.partSize;
  //cu.affine
    cu.predMode         = MODE_INTER;
    cu.LICFlag          = ( ( encTestMode.opts & ETO_FORCE_MERGE ) ? false : ( ( encTestMode.opts & ETO_LIC ) != 0 ) );
    cu.transQuantBypass = encTestMode.lossless;
    cu.chromaQpAdj      = cu.transQuantBypass ? 0 : m_cuChromaQpOffsetIdxPlus1;
    cu.qp               = encTestMode.qp;
  //cu.emtFlag  is set below
  //cu.obmcFlag is set below

    CU::addPUs( cu );
  }
  else
  {
    CHECK( cu.partSize != encTestMode.partSize,    "Mismatch" );
    CHECK( cu.skip,                                "Mismatch" );
    CHECK( cu.qtDepth  != partitioner.currQtDepth, "Mismatch" );
    CHECK( cu.btDepth  != partitioner.currBtDepth, "Mismatch" );
    CHECK( cu.depth    != partitioner.currDepth,   "Mismatch" );
  }

  cu.imv      = iIMV > 1 ? 2 : 1;
  cu.emtFlag  = false;
  cu.obmcFlag = sps.getSpsNext().getUseOBMC();

  if( pcCUInfo2Reuse != nullptr )
  {
    // reuse the motion info from pcCUInfo2Reuse
    CU::resetMVDandMV2Int( cu, m_pcInterSearch );

    if( !CU::hasSubCUNonZeroMVd( cu ) )
    {
      m_modeCtrl->useModeResult( encTestModeBase, tempCS, partitioner );
      return false;
    }
    else
    {
      m_pcInterSearch->motionCompensation( cu );
    }
  }
  else
  {
#if AMP_MRG
    m_pcInterSearch->predInterSearch( cu, partitioner, false );
#else
    m_pcInterSearch->predInterSearch( cu );
#endif
  }

  if( !CU::hasSubCUNonZeroMVd( cu ) )
  {
    m_modeCtrl->useModeResult( encTestModeBase, tempCS, partitioner );
    return false;
  }

  if( m_pTempCUWoOBMC )
  {
    Int wIdx = gp_sizeIdxInfo->idxFrom( tempCS->area.lwidth () );
    Int hIdx = gp_sizeIdxInfo->idxFrom( tempCS->area.lheight() );

    m_pTempCUWoOBMC[wIdx][hIdx]->clearCUs();
    m_pTempCUWoOBMC[wIdx][hIdx]->clearPUs();
    m_pTempCUWoOBMC[wIdx][hIdx]->clearTUs();
    m_pTempCUWoOBMC[wIdx][hIdx]->copyStructure( *tempCS );

    m_pPredBufWoOBMC[wIdx][hIdx].copyFrom( tempCS->getPredBuf( cu ) );

    m_pcInterSearch->subBlockOBMC( cu );

    m_pTempCUWoOBMC[wIdx][hIdx]->getPredBuf( cu ).copyFrom( tempCS->getPredBuf( cu ) );
  }

  const int maxSizeEMT            = tempCS->pcv->noRQT ? EMT_INTER_MAX_CU_WITH_QTBT : EMT_INTER_MAX_CU;
  double    bestCost              = bestCS->cost;
  UChar     considerEmtSecondPass = sps.getSpsNext().getUseInterEMT() && partitioner.currArea().lwidth() <= maxSizeEMT && partitioner.currArea().lheight() <= maxSizeEMT ? 1 : 0;
  bool      skipSecondEmtPass     = false;
  double    emtFirstPassCost      = MAX_DOUBLE;

  // CU-level optimization
  for( UChar emtCuFlag = 0; emtCuFlag <= considerEmtSecondPass; emtCuFlag++ )
  {
    if( m_pcEncCfg->getFastInterEMT() && emtCuFlag && skipSecondEmtPass )
    {
      continue;
    }

    tempCS->getCU()->emtFlag = emtCuFlag;

    m_pcInterSearch->encodeResAndCalcRdInterCU( *tempCS, partitioner, false );

    if( m_pcEncCfg->getFastInterEMT() )
    {
      emtFirstPassCost = tempCS->cost;
    }

    xEncodeDontSplit( *tempCS, partitioner );

    if( tempCS->pps->getUseDQP() && ( cu.depth ) <= tempCS->pps->getMaxCuDQPDepth() )
    {
      xCheckDQP( *tempCS );
    }

    DTRACE_MODE_COST( *tempCS, m_pcRdCost->getLambda() );
    xCheckBestMode( tempCS, bestCS, partitioner, encTestModeBase );

    //now we check whether the second pass should be skipped or not
    if( !emtCuFlag && considerEmtSecondPass )
    {
      static const double thresholdToSkipEmtSecondPass = 1.1; // Skip checking EMT transforms
      if( m_pcEncCfg->getFastInterEMT() && ( !cu.firstTU->cbf[COMPONENT_Y] || emtFirstPassCost > bestCost * thresholdToSkipEmtSecondPass ) )
      {
        skipSecondEmtPass = true;
      }
      else //EMT will be checked
      {
        if( bestCost == bestCS->cost ) //The first EMT pass didn't become the bestCS, so we clear the TUs generated
        {
          tempCS->clearTUs();
        }
        else
        {
          tempCS->initStructData( bestCS->currQP[bestCS->chType], bestCS->isLossless );
          tempCS->copyStructure( *bestCS );
          tempCS->getPredBuf().copyFrom( bestCS->getPredBuf() );
        }

        //we need to restart the distortion for the new tempCS, the bit count and the cost
        tempCS->dist     = 0;
        tempCS->fracBits = 0;
        tempCS->cost     = MAX_DOUBLE;
      }
    }
  }

  return true;
}


void EncCu::xEncodeDontSplit( CodingStructure &cs, Partitioner &partitioner )
{
  m_CABACEstimator->resetBits();

  {
    if( partitioner.canSplit( CU_QUAD_SPLIT, cs ) )
    {
      m_CABACEstimator->split_cu_flag( false, cs, partitioner );
    }
    if( partitioner.canSplit( CU_BT_SPLIT, cs ) )
    {
      m_CABACEstimator->split_cu_mode_mt( CU_DONT_SPLIT, cs, partitioner );
    }
  }

  cs.fracBits += m_CABACEstimator->getEstFracBits(); // split bits
  cs.cost      = m_pcRdCost->calcRdCost( cs.fracBits, cs.dist );

}

//! \}
