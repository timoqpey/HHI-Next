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
#include "IntraNNRom.h"

#include "CommonLib/dtrace_codingstruct.h"
#include "CommonLib/Picture.h"
#include "CommonLib/UnitTools.h"


#include "CommonLib/dtrace_buffer.h"

#include <stdio.h>
#include <cmath>
#include <algorithm>
#if HHI_WPP_PARALLELISM
#include <mutex>
extern std::recursive_mutex g_cache_mutex;
#endif



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
  bool          BTnoRQT       = encCfg->getGenBinSplit() || encCfg->getQTBT();

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

        m_pTempCS[w][h]->create( chromaFormat, Area( 0, 0, width, height ), false );
        m_pBestCS[w][h]->create( chromaFormat, Area( 0, 0, width, height ), false );
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
          m_pImvTempCS[w][p]->create( chromaFormat, Area( 0, 0, width, height ), false );
        }
        else
        {
          m_pImvTempCS[w][p] = nullptr;
        }
      }
    }
  }

  m_pBestInters2Nx2NCS = nullptr;
  if( encCfg->getMaxNumAddHyps() )
  {
    m_pBestInters2Nx2NCS = new CodingStructure***[numWidths]();

    for( unsigned w = 0; w < numWidths; w++ )
    {
      m_pBestInters2Nx2NCS[w] = new CodingStructure**[numHeights]();

      for( unsigned h = 0; h < numHeights; h++ )
      {
        const unsigned width  = gp_sizeIdxInfo->sizeFrom( w );
        const unsigned height = gp_sizeIdxInfo->sizeFrom( h );

        if( ( BTnoRQT || w == h ) && gp_sizeIdxInfo->isCuSize( width ) && gp_sizeIdxInfo->isCuSize( height ) )
        {
          const int howMany = encCfg->getAddHypTries();
          m_pBestInters2Nx2NCS[w][h] = new CodingStructure*[howMany]();
          for( int i = 0; i < howMany; ++i )
          {
            m_pBestInters2Nx2NCS[w][h][i] = new CodingStructure( m_unitCache.cuCache, m_unitCache.puCache, m_unitCache.tuCache );
            m_pBestInters2Nx2NCS[w][h][i]->create( chromaFormat, Area( 0, 0, width, height ), false );
          }
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
          m_pTempCUWoOBMC[w][h]->create( chromaFormat, Area( 0, 0, width, height ), false );
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
  if( encCfg->getGenBinSplit() )
  {
    m_modeCtrl = new EncModeCtrlGBS();
  }
  else
  {
    m_modeCtrl = new EncModeCtrlQTwithRQT();
  }

  for( unsigned ui = 0; ui < MRG_MAX_NUM_CANDS; ui++ )
  {
    m_acMergeBuffer[ui].create( chromaFormat, Area( 0, 0, uiMaxWidth, uiMaxHeight ) );
  }

  m_CtxBuffer.resize( maxDepth );
  m_CurrCtx = 0;
  for (unsigned i = 0; i < 3; i++)
  {
    m_DiffuFiltBuf[i].create(chromaFormat, Area(0, 0, uiMaxWidth, uiMaxHeight));
  }
}


void EncCu::destroy()
{
  bool          BTnoRQT   = m_pcEncCfg->getGenBinSplit() || m_pcEncCfg->getQTBT();
  unsigned      maxMEPart = BTnoRQT ? 1 : NUMBER_OF_PART_SIZES;

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

  if( m_pBestInters2Nx2NCS )
  {
    for( unsigned w = 0; w < numWidths; w++ )
    {
      for( unsigned h = 0; h < numHeights; h++ )
      {
        if( m_pBestInters2Nx2NCS[w][h] )
        {
          const int howMany = m_pcEncCfg->getAddHypTries();
          for( int i = 0; i < howMany; ++i )
          {
            m_pBestInters2Nx2NCS[w][h][i]->destroy();
            delete m_pBestInters2Nx2NCS[w][h][i];
          }
          delete[] m_pBestInters2Nx2NCS[w][h];
        }
      }
      delete[] m_pBestInters2Nx2NCS[w];
    }
    delete[] m_pBestInters2Nx2NCS;
    m_pBestInters2Nx2NCS = nullptr;
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
  m_DiffuFiltBuf[0].destroy();
  m_DiffuFiltBuf[1].destroy();
}



EncCu::~EncCu()
{
}



/** \param    pcEncLib      pointer of encoder class
 */
void EncCu::init( EncLib* pcEncLib, const SPS& sps PARL_PARAM( const int tId ) )
{
  m_pcEncCfg           = pcEncLib;
  m_pcIntraSearch      = pcEncLib->getIntraSearch( PARL_PARAM0( tId ) );
  m_pcInterSearch      = pcEncLib->getInterSearch( PARL_PARAM0( tId ) );
  m_DiffusionFilter    = pcEncLib->getDiffusionFilter( PARL_PARAM0( tId ) );
#if THRESHOLDING
  m_pcThresholding     = pcEncLib->getThresholding( PARL_PARAM0( tId ) );
#endif
  m_pcTrQuant          = pcEncLib->getTrQuant( PARL_PARAM0( tId ) );
  m_pcRdCost           = pcEncLib->getRdCost ( PARL_PARAM0( tId ) );
  m_CABACEstimator     = pcEncLib->getCABACEncoder( PARL_PARAM0( tId ) )->getCABACEstimator( &sps );
  m_CtxCache           = pcEncLib->getCtxCache( PARL_PARAM0( tId ) );
  m_pcRateCtrl         = pcEncLib->getRateCtrl();
  m_pcSliceEncoder     = pcEncLib->getSliceEncoder();
#if HHI_SPLIT_PARALLELISM || HHI_WPP_PARALLELISM
  m_pcEncLib           = pcEncLib;
  m_dataId             = tId;
#endif

  m_modeCtrl->init( m_pcEncCfg, m_pcRateCtrl, m_pcRdCost );

  m_pcInterSearch->setModeCtrl( m_modeCtrl );
  m_pcIntraSearch->setModeCtrl( m_modeCtrl );
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

void EncCu::compressCtu( CodingStructure& cs, const UnitArea& area, const unsigned ctuRsAddr, const int prevQP[], const int currQP[] )
{
  m_modeCtrl->initCTUEncoding( *cs.slice );

#if HHI_SPLIT_PARALLELISM
  if( m_pcEncCfg->getNumSplitThreads() > 1 )
  {
    for( int jId = 1; jId < NUM_RESERVERD_SPLIT_JOBS; jId++ )
    {
      EncCu*            jobEncCu  = m_pcEncLib->getCuEncoder( cs.picture->scheduler.getSplitDataId( jId ) );
      CacheBlkInfoCtrl* cacheCtrl = dynamic_cast< CacheBlkInfoCtrl* >( jobEncCu->m_modeCtrl );
      if( cacheCtrl )
      {
        cacheCtrl->init( *cs.slice );
      }
    }
  }

  if( auto* cacheCtrl = dynamic_cast<CacheBlkInfoCtrl*>( m_modeCtrl ) ) { cacheCtrl->tick(); }
#endif
  // init the partitioning manager
  Partitioner *partitioner = PartitionerFactory::get( *cs.slice );
  partitioner->initCtu( area, CH_L, *cs.slice );

  // init current context pointer
  m_CurrCtx = m_CtxBuffer.data();

  CodingStructure *tempCS = m_pTempCS[gp_sizeIdxInfo->idxFrom( area.lumaSize().width )][gp_sizeIdxInfo->idxFrom( area.lumaSize().height )];
  CodingStructure *bestCS = m_pBestCS[gp_sizeIdxInfo->idxFrom( area.lumaSize().width )][gp_sizeIdxInfo->idxFrom( area.lumaSize().height )];

  cs.initSubStructure( *tempCS, partitioner->chType, partitioner->currArea(), false );
  cs.initSubStructure( *bestCS, partitioner->chType, partitioner->currArea(), false );
  tempCS->currQP[CH_L] = bestCS->currQP[CH_L] =
  tempCS->baseQP       = bestCS->baseQP       = currQP[CH_L];
  tempCS->prevQP[CH_L] = bestCS->prevQP[CH_L] = prevQP[CH_L];

  xCompressCU( tempCS, bestCS, *partitioner );


  // all signals were already copied during compression if the CTU was split - at this point only the structures are copied to the top level CS
  const bool copyUnsplitCTUSignals = bestCS->cus.size() == 1 && KEEP_PRED_AND_RESI_SIGNALS;
  cs.useSubStructure( *bestCS, partitioner->chType, CS::getArea( *bestCS, area, partitioner->chType ), copyUnsplitCTUSignals, false, false, copyUnsplitCTUSignals );

  if( !cs.pcv->ISingleTree && cs.slice->isIntra() && cs.pcv->chrFormat != CHROMA_400 )
  {
    m_CABACEstimator->getCtx() = m_CurrCtx->start;

    partitioner->initCtu( area, CH_C, *cs.slice );

    cs.initSubStructure( *tempCS, partitioner->chType, partitioner->currArea(), false );
    cs.initSubStructure( *bestCS, partitioner->chType, partitioner->currArea(), false );
    tempCS->currQP[CH_C] = bestCS->currQP[CH_C] =
    tempCS->baseQP       = bestCS->baseQP       = currQP[CH_C];
    tempCS->prevQP[CH_C] = bestCS->prevQP[CH_C] = prevQP[CH_C];

    xCompressCU( tempCS, bestCS, *partitioner );

    const bool copyUnsplitCTUSignals = bestCS->cus.size() == 1 && KEEP_PRED_AND_RESI_SIGNALS;
    cs.useSubStructure( *bestCS, partitioner->chType, CS::getArea( *bestCS, area, partitioner->chType ), copyUnsplitCTUSignals, false, false, copyUnsplitCTUSignals );
  }

  // reset context states and uninit context pointer
  m_CABACEstimator->getCtx() = m_CurrCtx->start;
  m_CurrCtx                  = 0;
  delete partitioner;

#if HHI_SPLIT_PARALLELISM && HHI_WPP_PARALLELISM
  if( m_pcEncCfg->getNumSplitThreads() > 1 && m_pcEncCfg->getNumWppThreads() > 1 )
  {
    cs.picture->finishCtuPart( area );
  }
#endif

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
    if( tempCS->cus.size() == 1 )
    {
      const CodingUnit& cu = *tempCS->cus.front();
      CHECK( cu.skip && !cu.firstPU->mergeFlag, "Skip flag without a merge flag is not allowed!" );
    }

    DTRACE_BEST_MODE( tempCS, bestCS, m_pcRdCost->getLambda() );

    if( tempCS->sps->getSpsNext().getUseInterMultiHyp() && tempCS->slice->isInterB() )
      xSaveBestInterResultsForMultiHyp( *tempCS, partitioner, encTestMode );

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
#if HHI_SPLIT_PARALLELISM
  CHECK( m_dataId != tempCS->picture->scheduler.getDataId(), "Working in the wrong dataId!" );

  if( m_pcEncCfg->getNumSplitThreads() != 1 && tempCS->picture->scheduler.getSplitJobId() == 0 )
  {
    if( m_modeCtrl->isParallelSplit( *tempCS, partitioner ) )
    {
      m_modeCtrl->setParallelSplit( true );
      xCompressCUParallel( tempCS, bestCS, partitioner );
      return;
    }
  }

#endif

  Slice&   slice      = *tempCS->slice;
  const PPS &pps      = *tempCS->pps;
  const SPS &sps      = *tempCS->sps;
  const UInt uiLPelX  = tempCS->area.Y().lumaPos().x;
  const UInt uiTPelY  = tempCS->area.Y().lumaPos().y;

  const unsigned wIdx = gp_sizeIdxInfo->idxFrom( partitioner.currArea().lwidth()  );
  const unsigned hIdx = gp_sizeIdxInfo->idxFrom( partitioner.currArea().lheight() );

  const UnitArea currCsArea = clipArea( CS::getArea( *bestCS, bestCS->area, partitioner.chType ), *tempCS->picture );
  if( m_pImvTempCS && !slice.isIntra() )
  {
    const unsigned maxMEPart = tempCS->pcv->only2Nx2N ? 1 : NUMBER_OF_PART_SIZES;
    for( unsigned p = 0; p < maxMEPart; p++ )
    {
      tempCS->initSubStructure( *m_pImvTempCS[wIdx][p], partitioner.chType, partitioner.currArea(), false );
    }
  }

  if( m_pBestInters2Nx2NCS && !slice.isIntra() )
  {
    CodingStructure ** const best2Nx2NSubCS = m_pBestInters2Nx2NCS[wIdx][hIdx];
    const int howMany = m_pcEncCfg->getAddHypTries();
    for( int i = 0; i < howMany; ++i )
    {
      tempCS->initSubStructure( *best2Nx2NSubCS[i], partitioner.chType, partitioner.currArea(), false );
    }
  }


  if( m_pTempCUWoOBMC && !slice.isIntra() )
  {
    tempCS->initSubStructure( *m_pTempCUWoOBMC[wIdx][hIdx], partitioner.chType, partitioner.currArea(), false );
  }

  m_modeCtrl->initCULevel( partitioner, *tempCS );

  m_CurrCtx->start = m_CABACEstimator->getCtx();

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

  DTRACE_UPDATE( g_trace_ctx, std::make_pair( "cux", uiLPelX ) );
  DTRACE_UPDATE( g_trace_ctx, std::make_pair( "cuy", uiTPelY ) );
  DTRACE_UPDATE( g_trace_ctx, std::make_pair( "cuw", tempCS->area.lwidth() ) );
  DTRACE_UPDATE( g_trace_ctx, std::make_pair( "cuh", tempCS->area.lheight() ) );
  DTRACE( g_trace_ctx, D_COMMON, "@(%4d,%4d) [%2dx%2d]\n", tempCS->area.lx(), tempCS->area.ly(), tempCS->area.lwidth(), tempCS->area.lheight() );

  do
  {
    const EncTestMode currTestMode = m_modeCtrl->currTestMode();

#if SHARP_LUMA_DELTA_QP
    if( m_pcEncCfg->getLumaLevelToDeltaQPMapping().isEnabled() && partitioner.currDepth <= pps.getMaxCuDQPDepth() )
    {
#if HHI_SPLIT_PARALLELISM
      CHECK( tempCS->picture->scheduler.getSplitJobId() > 0, "Changing lambda is only allowed in the master thread!" );
#endif
      updateLambda( &slice, currTestMode.qp );
    }
#endif

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
      if( sps.getSpsNext().getUseRestrictedMerge() && slice.isInterB() )
      {
        xCheckRDCostMerge2Nx2N( tempCS, bestCS, partitioner, currTestMode, REF_PIC_LIST_0 );
        xCheckRDCostMerge2Nx2N( tempCS, bestCS, partitioner, currTestMode, REF_PIC_LIST_1 );
      }
    }
    else if( currTestMode.type == ETM_MERGE_FRUC )
    {
      xCheckRDCostMerge2Nx2NFRUC( tempCS, bestCS, partitioner, currTestMode );
      if( sps.getSpsNext().getUseRestrictedMerge() && slice.isInterB() )
      {
        xCheckRDCostMerge2Nx2NFRUC( tempCS, bestCS, partitioner, currTestMode, REF_PIC_LIST_0 );
        xCheckRDCostMerge2Nx2NFRUC( tempCS, bestCS, partitioner, currTestMode, REF_PIC_LIST_1 );
      }
    }
    else if( currTestMode.type == ETM_INTER_MULTIHYP )
    {
      for( int i = 0; i < m_pcEncCfg->getAddHypTries(); ++i )
        xCheckRDCostInterMultiHyp2Nx2N( tempCS, bestCS, partitioner, currTestMode, i );
    }
    else if( currTestMode.type == ETM_MOTION_VECTOR_REESTIMATION )
    {
      xCheckRDCostMotionVectorReestimation( tempCS, bestCS, partitioner, currTestMode );
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

      xCheckModeSplit( tempCS, bestCS, partitioner, currTestMode );
    }
    else
    {
      THROW( "Don't know how to handle mode: type = " << currTestMode.type << ", size = " << currTestMode.partSize << ", options = " << currTestMode.opts );
    }
  } while( m_modeCtrl->nextMode( *tempCS, partitioner ) );

  //////////////////////////////////////////////////////////////////////////
  // Finishing CU
#if HHI_SPLIT_PARALLELISM
  if( bestCS->cus.empty() )
  {
    CHECK( bestCS->cost != MAX_DOUBLE, "Cost should be maximal if no encoding found" );
    CHECK( bestCS->picture->scheduler.getSplitJobId() == 0, "Should always get a result in serial case" );

    m_modeCtrl->finishCULevel( partitioner );
    return;
  }

#endif
  // set context states
  m_CABACEstimator->getCtx() = m_CurrCtx->best;

  // QP from last processed CU for further processing
  bestCS->prevQP[partitioner.chType] = bestCS->cus.back()->qp;

  bestCS->picture->getRecoBuf( currCsArea ).copyFrom( bestCS->getRecoBuf( currCsArea ) );
  m_modeCtrl->finishCULevel( partitioner );

#if HHI_SPLIT_PARALLELISM
  if( tempCS->picture->scheduler.getSplitJobId() == 0 && m_pcEncCfg->getNumSplitThreads() != 1 )
  {
    tempCS->picture->finishParallelPart( currCsArea );
  }

#endif
  // Assert if Best prediction mode is NONE
  // Selected mode's RD-cost must be not MAX_DOUBLE.
  CHECK( bestCS->cus.empty()                                   , "No possible encoding found" );
  CHECK( bestCS->cus[0]->partSize == NUMBER_OF_PART_SIZES      , "No possible encoding found" );
  CHECK( bestCS->cus[0]->predMode == NUMBER_OF_PREDICTION_MODES, "No possible encoding found" );
  CHECK( bestCS->cost             == MAX_DOUBLE                , "No possible encoding found" );
}

#if SHARP_LUMA_DELTA_QP
Void EncCu::updateLambda( Slice* slice, Double dQP )
{
  Int iQP = (Int)dQP;
  const Double oldQP     = (Double)slice->getSliceQpBase();
  const Double oldLambda = m_pcSliceEncoder->calculateLambda (slice, m_pcSliceEncoder->getGopId(), slice->getDepth(), oldQP, oldQP, iQP);
  const Double newLambda = oldLambda * pow (2.0, (dQP - oldQP) / 3.0);
#if RDOQ_CHROMA_LAMBDA
  const Double chromaLambda = newLambda / m_pcRdCost->getChromaWeight();
  const Double lambdaArray[MAX_NUM_COMPONENT] = {newLambda, chromaLambda, chromaLambda};
  m_pcTrQuant->setLambdas (lambdaArray);
#else
  m_pcTrQuant->setLambda (newLambda);
#endif
  m_pcRdCost->setLambda( newLambda, slice->getSPS()->getBitDepths() );
}
#endif

#if HHI_SPLIT_PARALLELISM
//#undef DEBUG_PARALLEL_TIMINGS
//#define DEBUG_PARALLEL_TIMINGS 1
void EncCu::xCompressCUParallel( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner )
{
  const unsigned wIdx = gp_sizeIdxInfo->idxFrom( partitioner.currArea().lwidth() );
  const unsigned hIdx = gp_sizeIdxInfo->idxFrom( partitioner.currArea().lheight() );

  Picture* picture = tempCS->picture;

  int numJobs = m_modeCtrl->getNumParallelJobs( *bestCS, partitioner );

  bool    jobUsed                            [NUM_RESERVERD_SPLIT_JOBS];
  std::fill( jobUsed, jobUsed + NUM_RESERVERD_SPLIT_JOBS, false );

  const UnitArea currArea = CS::getArea( *tempCS, partitioner.currArea(), partitioner.chType );
#if HHI_WPP_PARALLELISM
  const int      wppTId   = picture->scheduler.getWppThreadId();
#endif
  const bool doParallel   = !m_pcEncCfg->getForceSingleSplitThread();
#if _MSC_VER && HHI_WPP_PARALLELISM
#pragma omp parallel for schedule(dynamic,1) num_threads(NUM_SPLIT_THREADS_IF_MSVC) if(doParallel)
#else
  omp_set_num_threads( m_pcEncCfg->getNumSplitThreads() );

#pragma omp parallel for schedule(dynamic,1) if(doParallel)
#endif
  for( int jId = 1; jId <= numJobs; jId++ )
  {
    // thread start
#if HHI_WPP_PARALLELISM
    picture->scheduler.setWppThreadId( wppTId );
#endif
    picture->scheduler.setSplitThreadId();
    picture->scheduler.setSplitJobId( jId );

    Partitioner* jobPartitioner = PartitionerFactory::get( *tempCS->slice );
    EncCu*       jobCuEnc       = m_pcEncLib->getCuEncoder( picture->scheduler.getSplitDataId( jId ) );
    auto*        jobBlkCache    = dynamic_cast<CacheBlkInfoCtrl*>( jobCuEnc->m_modeCtrl );

    jobPartitioner->copyState( partitioner );
    jobCuEnc      ->copyState( this, *jobPartitioner, currArea, true );

    if( jobBlkCache )
    {
      jobBlkCache->tick();
    }

    CodingStructure *&jobBest = jobCuEnc->m_pBestCS[wIdx][hIdx];
    CodingStructure *&jobTemp = jobCuEnc->m_pTempCS[wIdx][hIdx];

    jobUsed[jId] = true;

    jobCuEnc->xCompressCU( jobTemp, jobBest, *jobPartitioner );

    delete jobPartitioner;

    picture->scheduler.setSplitJobId( 0 );
    // thread stop
  }
  picture->scheduler.setSplitThreadId( 0 );

  int    bestJId  = 0;
  double bestCost = bestCS->cost;
  for( int jId = 1; jId <= numJobs; jId++ )
  {
    EncCu* jobCuEnc = m_pcEncLib->getCuEncoder( picture->scheduler.getSplitDataId( jId ) );

    if( jobUsed[jId] && jobCuEnc->m_pBestCS[wIdx][hIdx]->cost < bestCost )
    {
      bestCost = jobCuEnc->m_pBestCS[wIdx][hIdx]->cost;
      bestJId  = jId;
    }
  }

  if( bestJId > 0 )
  {
    copyState( m_pcEncLib->getCuEncoder( picture->scheduler.getSplitDataId( bestJId ) ), partitioner, currArea, false );
    m_CurrCtx->best = m_CABACEstimator->getCtx();

    tempCS = m_pTempCS[wIdx][hIdx];
    bestCS = m_pBestCS[wIdx][hIdx];
  }

  const int      bitDepthY = tempCS->sps->getBitDepth( CH_L );
  const UnitArea clipdArea = clipArea( currArea, *picture );

  CHECK( calcCheckSum( picture->getRecoBuf( clipdArea.Y() ), bitDepthY ) != calcCheckSum( bestCS->getRecoBuf( clipdArea.Y() ), bitDepthY ), "Data copied incorrectly!" );

  picture->finishParallelPart( currArea );

  if( auto *blkCache = dynamic_cast<CacheBlkInfoCtrl*>( m_modeCtrl ) )
  {
    for( int jId = 1; jId <= numJobs; jId++ )
    {
      if( !jobUsed[jId] || jId == bestJId ) continue;

      auto *jobBlkCache = dynamic_cast<CacheBlkInfoCtrl*>( m_pcEncLib->getCuEncoder( picture->scheduler.getSplitDataId( jId ) )->m_modeCtrl );
      CHECK( !jobBlkCache, "If own mode controller has blk info cache capability so should all other mode controllers!" );
      blkCache->CacheBlkInfoCtrl::copyState( *jobBlkCache, partitioner.currArea() );
    }

    blkCache->tick();
  }

}

void EncCu::copyState( EncCu* other, Partitioner& partitioner, const UnitArea& currArea, const bool isDist )
{
  const unsigned wIdx = gp_sizeIdxInfo->idxFrom( partitioner.currArea().lwidth () );
  const unsigned hIdx = gp_sizeIdxInfo->idxFrom( partitioner.currArea().lheight() );

  if( isDist )
  {
    other->m_pBestCS[wIdx][hIdx]->initSubStructure( *m_pBestCS[wIdx][hIdx], partitioner.chType, partitioner.currArea(), false );
    other->m_pTempCS[wIdx][hIdx]->initSubStructure( *m_pTempCS[wIdx][hIdx], partitioner.chType, partitioner.currArea(), false );
  }
  else
  {
          CodingStructure* dst =        m_pBestCS[wIdx][hIdx];
    const CodingStructure *src = other->m_pBestCS[wIdx][hIdx];
    bool keepResi = KEEP_PRED_AND_RESI_SIGNALS;

    dst->useSubStructure( *src, partitioner.chType, currArea, KEEP_PRED_AND_RESI_SIGNALS, true, keepResi, keepResi );
    dst->cost           =  src->cost;
    dst->dist           =  src->dist;
    dst->fracBits       =  src->fracBits;
    dst->features       =  src->features;
  }

  if( isDist )
  {
    m_CurrCtx = m_CtxBuffer.data();
  }

  m_pcInterSearch->copyState( *other->m_pcInterSearch );
  m_modeCtrl     ->copyState( *other->m_modeCtrl, partitioner.currArea() );
  m_pcRdCost     ->copyState( *other->m_pcRdCost );
  m_pcTrQuant    ->copyState( *other->m_pcTrQuant );

  m_CABACEstimator->getCtx() = other->m_CABACEstimator->getCtx();
}
#endif

void EncCu::xCheckModeSplit(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode)
{
  const Int qp                = encTestMode.qp;
  const PPS &pps              = *tempCS->pps;
  const Slice &slice          = *tempCS->slice;
  const Bool bIsLosslessMode  = false; // False at this level. Next level down may set it to true.
  const int oldPrevQp         = tempCS->prevQP[partitioner.chType];
  const UInt currDepth        = partitioner.currDepth;

  PartSplit split = CU_DONT_SPLIT;

  if( auto gbsPartitioner = dynamic_cast<GenBinSplitPartitioner*>( &partitioner ) )
  {
    const PartSplit baseSplit = gbsPartitioner->getActualSplitType( getPartSplit( encTestMode ) );
    const SplitModifier mod   = SplitModifier( ( encTestMode.opts & ETO_SM ) >> ETO_SM_SHIFT );
    split                     = applyModifier( baseSplit, mod );
  }
  else
  {
    split = getPartSplit( encTestMode );
  }

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

      tempCS->initSubStructure( *tempSubCS, partitioner.chType, subCUArea, false );
      tempCS->initSubStructure( *bestSubCS, partitioner.chType, subCUArea, false );

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

      bool keepResi = KEEP_PRED_AND_RESI_SIGNALS;
      tempCS->useSubStructure( *bestSubCS, partitioner.chType, CS::getArea( *tempCS, subCUArea, partitioner.chType ), KEEP_PRED_AND_RESI_SIGNALS, true, keepResi, keepResi );

      if(currDepth < pps.getMaxCuDQPDepth())
      {
        tempCS->prevQP[partitioner.chType] = bestSubCS->prevQP[partitioner.chType];
      }

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
        if( tempCS->sps->getSpsNext().getUseGenBinSplit() )
        {
          m_CABACEstimator->gen_bin_split_mode( split, *tempCS, partitioner );
          m_CABACEstimator->gen_bin_split_mod ( split, *tempCS, partitioner );
        }
        else
          m_CABACEstimator->split_cu_mode_mt(split, *tempCS, partitioner);
      }

      tempCS->fracBits += m_CABACEstimator->getEstFracBits(); // split bits
    }
  }

  tempCS->cost = m_pcRdCost->calcRdCost( tempCS->fracBits, tempCS->dist );

  // Check Delta QP bits for splitted structure
  xCheckDQP( *tempCS, partitioner, true );

  // If the configuration being tested exceeds the maximum number of bytes for a slice / slice-segment, then
  // a proper RD evaluation cannot be performed. Therefore, termination of the
  // slice/slice-segment must be made prior to this CTU.
  // This can be achieved by forcing the decision to be that of the rpcTempCU.
  // The exception is each slice / slice-segment must have at least one CTU.
  if (bestCS->cost != MAX_DOUBLE)
  {
    const TileMap& tileMap = *tempCS->picture->tileMap;
    const UInt CtuAddr = CU::getCtuAddr( *bestCS->getCU( partitioner.chType ) );
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

  tempCS->prevQP[partitioner.chType] = oldPrevQp;
}


void EncCu::xCheckRDCostIntra( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode )
{
  double bestInterCost        = m_modeCtrl->getBestInterCost();
  bool isAllIntra             = m_pcEncCfg->getIntraPeriod() == 1;
  double costSize2Nx2NemtFirstPass = m_modeCtrl->getEmtSize2Nx2NFirstPassCost();
  double costSizeNxNemtFirstPass   = MAX_DOUBLE;
  bool skipSecondEmtPass           = m_modeCtrl->getSkipSecondEMTPass();
  auto slsCtrl                = dynamic_cast<SaveLoadEncInfoCtrl*>( m_modeCtrl );
  const SPS &sps              = *tempCS->sps;
  const PPS &pps              = *tempCS->pps;
  const CodingUnit *bestCU    = bestCS->getCU( partitioner.chType );
  const int nsstIdx           = ( encTestMode.opts & ETO_NSST ) >> ETO_NSST_SHIFT;
  const bool usePDPC          = ( encTestMode.opts & ETO_PDPC ) != 0;
  const int mode1dTus         = ( encTestMode.opts & ETO_1D_PARTITIONS ) >> ETO_1D_PARTITIONS_SHIFT;
  const bool use1dTus         = mode1dTus != 0;
  const int mrlIdx            = ( encTestMode.opts & ETO_MRL )  >> ETO_MRL_SHIFT;
  const int maxSizeEMT        = pps.pcv->noRQT ? EMT_INTRA_MAX_CU_WITH_QTBT : EMT_INTRA_MAX_CU;
  int num_passes_Intra_NN     = (!isLuma( partitioner.chType ) || usePDPC || use1dTus || !sps.getSpsNext().getUseIntra_NN()) ? 1 : (getNumModesNNPadded( tempCS->area.lwidth(), tempCS->area.lheight() ) > 0 ? 2 : 1);

  if( nsstIdx > NUM_NSST_TRAFOS_PER_NN_MODE )
  {
    num_passes_Intra_NN = 1;
  }


  UChar considerEmtSecondPass = ( sps.getSpsNext().getUseIntraEMT() && isLuma( partitioner.chType ) && partitioner.currArea().lwidth() <= maxSizeEMT && partitioner.currArea().lheight() <= maxSizeEMT && ( nsstIdx == 0 || !tempCS->pcv->noRQT ) ) ? 1 : 0;
  CHECK( usePDPC && sps.getSpsNext().isPlanarPDPC(), "PDPC cannot be on with Planar-PDPC" );

  if( !usePDPC && subSamplingAllowed( tempCS->area.lwidth(), tempCS->area.lheight() ) && isLuma( partitioner.chType ) && sps.getSpsNext().getUseIntra_NN() && tempCS->area.lheight() <= 64 && tempCS->area.lwidth() <= 64 )
  {
    num_passes_Intra_NN = 3;
    if( tempCS->area.lwidth() != 32 && tempCS->area.lwidth() != 64 )
    {
      num_passes_Intra_NN = 1;
    }
  }

  if( use1dTus )
  {
    considerEmtSecondPass = 0;
    num_passes_Intra_NN   = 1;
  }


  TU1dPartitioner partitionerFor1dTus( partitioner );

  const UChar mLearningIdxStart = ( !sps.getSpsNext().getUseNSST() && nsstIdx != 0 ) ? 1 : 0; // Skip testing non-NN intra NSST if NSST is disabled.

  if( !sps.getSpsNext().getUseIntraNNTrafos() && nsstIdx != 0 )                               // Skip testing intra NN transforms if they are disabled.
  {
    num_passes_Intra_NN = 1;
  }

  for( UChar mLearningIdx = mLearningIdxStart; mLearningIdx < num_passes_Intra_NN; mLearningIdx++ )
  {
    Distortion interHad = m_modeCtrl->getInterHad(mLearningIdx);

    if( mLearningIdx == 1 && getNumModesNNPadded( tempCS->area.lwidth(), tempCS->area.lheight() ) == 0 )
    {
      continue;
    }

    if( m_pcEncCfg->getUseSetOfTrafos() && nsstIdx && isLuma( partitioner.chType ) && mLearningIdx == 0 )
    {
      continue;
    }

    if( mLearningIdx == 0 && nsstIdx > 3 )
    {
      continue;
    }
    if( mLearningIdx > 0 && mrlIdx > 0 )
    {
      continue;
    }


  for( UChar emtCuFlag = 0; emtCuFlag <= considerEmtSecondPass; emtCuFlag++ )
  {
    //Possible early EMT tests interruptions
    //1) saveLoadTag code for EMT
    if( sps.getSpsNext().getUseQTBT() && slsCtrl && m_pcEncCfg->getUseSaveLoadEncInfo() && !use1dTus )
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
    if( m_pcEncCfg->getUsePbIntraFast() && !tempCS->slice->isIntra() && bestCU && CU::isInter( *bestCS->getCU( partitioner.chType ) ) && interHad == 0 )
    {
      continue;
    }

    bool b_minTestDifFilter = (mLearningIdx > 1)? false: true;
    if (b_minTestDifFilter)
    {
      if ((sps.getSpsNext().getRestrDiffusionMode() >> 1) & 1)
      {
        if (tempCS->area.lwidth() != tempCS->area.lheight())
        {
          if ((tempCS->area.lwidth() == 128) || (tempCS->area.lheight() == 128))
          {
            b_minTestDifFilter = false;
          }
        }
        if ((tempCS->area.lwidth() == 4) || (tempCS->area.lheight() == 4))
        {
          b_minTestDifFilter = false;
        }
      }
      if ((sps.getSpsNext().getRestrDiffusionMode() & 1) && (tempCS->area.Y().area() <= 32))
      {
        b_minTestDifFilter = false;
      }
    }
    UInt maxDiffusionFilterIdx = (sps.getSpsNext().getDiffusionFilterEnabled()
                                         && !use1dTus
                                         && !usePDPC
                                         && ( !sps.getSpsNext().getRestrIntraDiffusionMode() || ( nsstIdx == 0 && mrlIdx == 0 && emtCuFlag == 0 && mLearningIdx == 0 && tempCS->area.Y().area() > 256 ) )
                                         && partitioner.chType != CHANNEL_TYPE_CHROMA
                                         && b_minTestDifFilter
                                         && tempCS->area.lx() && tempCS->area.ly() ? 1 : 0);
    m_pcIntraSearch->bestCostnoDiff = MAX_DOUBLE;
    m_pcIntraSearch->setStoreFirstDir(maxDiffusionFilterIdx > 0);

    for( UInt diffFilterIdx = 0; diffFilterIdx <= maxDiffusionFilterIdx; diffFilterIdx++ )
    {
    tempCS->initStructData( encTestMode.qp, encTestMode.lossless );

    CodingUnit &cu      = tempCS->addCU( CS::getArea( *tempCS, tempCS->area, partitioner.chType ), partitioner.chType );

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
    cu.nsstIdx          = mLearningIdx ? 0 : nsstIdx; // Here intra NN transform index is branched from NSST path.
    cu.emtFlag          = emtCuFlag;
    cu.mrlIdx           = mrlIdx;
    cu.intra_NN         = mLearningIdx;
    cu.intra_NN_Use_Sampling
                        = mLearningIdx > 0 ? mLearningIdx - 1 : 0;
    cu.intraNNTrafoIdx  = mLearningIdx ? nsstIdx : 0;
    cu.mode1dPartitions = mode1dTus;
    cu.diffFilterIdx    = diffFilterIdx;

    CU::addPUs( cu );

    tempCS->interHad    = interHad;

    if( isLuma( partitioner.chType ) )
    {
      m_pcIntraSearch->estIntraPredLumaQT( cu, use1dTus ? partitionerFor1dTus : partitioner, use1dTus ? bestCS->cost : MAX_DOUBLE );

      if( use1dTus && cu.mode1dPartitions == 0 )
      {
        continue;
      }

      if( m_pcEncCfg->getUsePbIntraFast() && tempCS->dist == MAX_UINT && tempCS->interHad == 0 )
      {
        interHad = 0;
        maxDiffusionFilterIdx = 0;
        // JEM assumes only perfect reconstructions can from now on beat the inter mode
        m_modeCtrl->enforceInterHad( mLearningIdx, 0 );
        continue;
      }

      if( cu.cs->sps->getSpsNext().getUseIntraFTM() && cu.firstPU->FTMRegIdx == MAX_UINT )
      {
        cu.firstPU->FTMRegIdx = 0;
        continue;
      }

      if( sps.getSpsNext().getDiffusionFilterEnabled() && cu.diffFilterIdx > 0 && m_pcIntraSearch->bestCostnoDiff == MAX_DOUBLE )
        continue;

      if( !CS::isDualITree( *tempCS ) )
      {
        cu.cs->picture->getRecoBuf( cu.Y() ).copyFrom( cu.cs->getRecoBuf( COMPONENT_Y ) );
      }
    }

    if( tempCS->area.chromaFormat != CHROMA_400 && ( partitioner.chType == CHANNEL_TYPE_CHROMA || !CS::isDualITree( *tempCS ) ) )
    {
      m_pcIntraSearch->estIntraPredChromaQT( cu, ( !use1dTus || ( CS::isDualITree( *cu.cs ) && !canUse1dPartitions( CHANNEL_TYPE_CHROMA ) ) ) ? partitioner : partitionerFor1dTus );
    }

    cu.rootCbf = false;

    for( UInt t = 0; t < getNumberValidTBlocks( *cu.cs->pcv ); t++ )
    {
      cu.rootCbf |= cu.firstTU->cbf[t] != 0;
    }

    // Get total bits for current mode: encode CU
    m_CABACEstimator->resetBits();

    if( pps.getTransquantBypassEnabledFlag() )
    {
      m_CABACEstimator->cu_transquant_bypass_flag( cu );
    }

    if( !cu.cs->slice->isIntra() )
    {
      m_CABACEstimator->cu_skip_flag ( cu );
    }
    m_CABACEstimator->pred_mode      ( cu );
    m_CABACEstimator->mrl_idx        ( cu );
    m_CABACEstimator->pdpc_flag      ( cu );
    m_CABACEstimator->part_mode      ( cu );
    m_CABACEstimator->cu_pred_data   ( cu );
    if( CU::isDiffIdxPresent( cu ) )
    {
      m_CABACEstimator->cu_diffusion_filter_idx( cu );
    }
    m_CABACEstimator->pcm_data       ( cu );


    // Encode Coefficients
    CUCtx cuCtx;
    cuCtx.isDQPCoded = true;
    cuCtx.isChromaQpAdjCoded = true;
    m_CABACEstimator->cu_residual( cu, use1dTus ? partitionerFor1dTus : partitioner, cuCtx );

    tempCS->fracBits = m_CABACEstimator->getEstFracBits();
    tempCS->cost     = m_pcRdCost->calcRdCost(tempCS->fracBits, tempCS->dist);

    xEncodeDontSplit( *tempCS, partitioner );

    xCheckDQP( *tempCS, partitioner );

    // Check if secondary transform (NSST) is too expensive
    const int nonZeroCoeffThr = CS::isDualITree( *tempCS ) ? ( isLuma( partitioner.chType ) ? NSST_SIG_NZ_LUMA : NSST_SIG_NZ_CHROMA ) : NSST_SIG_NZ_LUMA + NSST_SIG_NZ_CHROMA;
    if( nsstIdx && tempCS->pcv->noRQT && cuCtx.numNonZeroCoeffNonTs <= nonZeroCoeffThr )
    {
      Bool isMDIS = false;
      if( sps.getSpsNext().isPlanarPDPC() )
      {
        CHECK( CU::getNumPUs( cu ) > 1, "PLanarPDPC: encoder MDIS condition not defined for multi PU" );
        const PredictionUnit* pu = cu.firstPU;
        isMDIS = IntraPrediction::useFilteredIntraRefSamples( COMPONENT_Y, *pu, true, *pu );
      }

      if( cuCtx.numNonZeroCoeffNonTs > 0 || isMDIS )
      {
        CHECKD( !sps.getSpsNext().getUseNSST() && !sps.getSpsNext().getUseIntraNNTrafos(), "Expecting NSST mode" );
        tempCS->cost = MAX_DOUBLE;
      }
    }
    if( nsstIdx && !tempCS->pcv->noRQT && cu.rootCbf == 0 )
    {
      tempCS->cost = MAX_DOUBLE;
    }

    // we save the cost of the modes for the first EMT pass
    if( !emtCuFlag ) static_cast< double& >( cu.partSize == SIZE_2Nx2N ? costSize2Nx2NemtFirstPass : costSizeNxNemtFirstPass ) = tempCS->cost;

    DTRACE_MODE_COST( *tempCS, m_pcRdCost->getLambda() );
    if( !tempCS->slice->isIntra() && diffFilterIdx == 0 && maxDiffusionFilterIdx != 0 && tempCS->cost > 1.4 * bestInterCost )
    {
      maxDiffusionFilterIdx = 0;
    }
    xCheckBestMode( tempCS, bestCS, partitioner, encTestMode );

    if( use1dTus )
    {
      // don't evaluate the rest of EMT speed-ups if testing 1d TUs
      goto outOfEmtLoop;
    }


    //now we check whether the second pass of SIZE_2Nx2N and the whole Intra SIZE_NxN should be skipped or not
    if( !emtCuFlag && !tempCS->slice->isIntra() && bestCU && bestCU->predMode != MODE_INTRA && cu.partSize == SIZE_2Nx2N && m_pcEncCfg->getFastInterEMT() && ( m_pcEncCfg->getUseSaveLoadEncInfo() ? ( bestInterCost < MAX_DOUBLE ) : true ) )
    {
      const double thEmtInterFastSkipIntra = 1.4; // Skip checking Intra if "2Nx2N using DCT2" is worse than best Inter mode
      if( costSize2Nx2NemtFirstPass > thEmtInterFastSkipIntra * bestInterCost )
      {
        skipSecondEmtPass = true;
        m_modeCtrl->setSkipSecondEMTPass( true );
        goto outOfEmtLoop;
      }
    }

    //now we check whether the second pass of EMT with SIZE_NxN should be skipped or not
    if( !emtCuFlag && isAllIntra && cu.partSize == SIZE_NxN && m_pcEncCfg->getFastIntraEMT() )
    {
      costSize2Nx2NemtFirstPass = m_modeCtrl->getEmtSize2Nx2NFirstPassCost();
      const double thEmtIntraFastSkipNxN = 1.2; // Skip checking "NxN using EMT" if "NxN using DCT2" is worse than "2Nx2N using DCT2"
      if( costSizeNxNemtFirstPass > thEmtIntraFastSkipNxN * costSize2Nx2NemtFirstPass )
      {
        goto outOfEmtLoop;
      }
    }
    const bool intraNoCoeff = CU::isIntra( *bestCS->cus[0] ) && !cu.rootCbf;
    if( diffFilterIdx < maxDiffusionFilterIdx && intraNoCoeff )
    {
      break;
    }

    } //for diffFilterIdx
  } //for emtCuFlag
outOfEmtLoop:;
  } //for MLearningFlag
}

void EncCu::xCheckIntraPCM(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode )
{
  tempCS->initStructData( encTestMode.qp, encTestMode.lossless );

  CodingUnit &cu      = tempCS->addCU( tempCS->area, partitioner.chType );

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
  cu.mrlIdx           = 0;

  tempCS->addPU(tempCS->area, partitioner.chType);
  
  TransformUnit & tu  = tempCS->addTU(tempCS->area, partitioner.chType);
  tu.depth            = 0;

  m_pcIntraSearch->IPCMSearch(*tempCS, partitioner);

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

  xCheckDQP( *tempCS, partitioner );

  DTRACE_MODE_COST( *tempCS, m_pcRdCost->getLambda() );
  xCheckBestMode( tempCS, bestCS, partitioner, encTestMode );
}

void EncCu::xCheckDQP( CodingStructure& cs, Partitioner& partitioner, bool bKeepCtx )
{
  CHECK( bKeepCtx && cs.cus.size() <= 1 && partitioner.getImplicitSplit( cs ) == CU_DONT_SPLIT, "bKeepCtx should only be set in split case" );
  CHECK( !bKeepCtx && cs.cus.size() > 1, "bKeepCtx should never be set for non-split case" );

  if( !cs.pps->getUseDQP() )
  {
    return;
  }

  if( bKeepCtx && partitioner.currDepth != cs.pps->getMaxCuDQPDepth() )
  {
    return;
  }

  if( !bKeepCtx && partitioner.currDepth > cs.pps->getMaxCuDQPDepth() )
  {
    return;
  }

  CodingUnit* cuFirst = cs.getCU( partitioner.chType );

  CHECK( !cuFirst, "No CU available" );

  bool hasResidual = false;
  for( const auto &cu : cs.cus )
  {
    if( cu->rootCbf )
    {
      hasResidual = true;
      break;
    }
  }

  int predQP = CU::predictQP( *cuFirst, cs.prevQP[partitioner.chType] );

  if( hasResidual )
  {
    TempCtx ctxTemp( m_CtxCache );
    if( !bKeepCtx ) ctxTemp = SubCtx( Ctx::DeltaQP, m_CABACEstimator->getCtx() );

    m_CABACEstimator->resetBits();
    m_CABACEstimator->cu_qp_delta( *cuFirst, predQP, cuFirst->qp );

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


void EncCu::xCheckRDCostMerge2Nx2N( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode, const RefPicList mergeRefPicList )
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
    PU::restrictInterMergeCandidatesForRefPicList( mergeCtx, mergeRefPicList, slice );
  }


  bool candHasNoResidual[MRG_MAX_NUM_CANDS];
  for (UInt ui = 0; ui < mergeCtx.numValidMergeCand; ui++)
  {
    candHasNoResidual[ui] = false;
  }

  bool                                        bestIsSkip       = false;
  unsigned                                    uiNumMrgSATDCand = mergeCtx.numValidMergeCand;
  PelUnitBuf                                  acMergeBuffer    [ MRG_MAX_NUM_CANDS ];
  static_vector<unsigned, MRG_MAX_NUM_CANDS>  RdModeList;
  bool                                        mrgTempBufSet    = false;

  for( unsigned i = 0; i < MRG_MAX_NUM_CANDS; i++ )
  {
    RdModeList.push_back( i );
  }

  if( m_pcEncCfg->getUseFastMerge() )
  {
    uiNumMrgSATDCand = NUM_MRG_SATD_CAND;
    bestIsSkip       = false;

    if( auto blkCache = dynamic_cast< CacheBlkInfoCtrl* >( m_modeCtrl ) )
    {
      bestIsSkip = blkCache->isSkip( tempCS->area );
      if( bestIsSkip && mergeRefPicList != REF_PIC_LIST_X )
      {
        return;
      }
    }

    static_vector<double, MRG_MAX_NUM_CANDS> candCostList;

    // 1. Pass: get SATD-cost for selected candidates and reduce their count
    if( !bestIsSkip )
    {
      RdModeList.clear();
      mrgTempBufSet       = true;
      const double sqrtLambdaForFirstPass = m_pcRdCost->getMotionLambda( encTestMode.lossless );

      CodingUnit &cu      = tempCS->addCU( tempCS->area, partitioner.chType );

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

      PredictionUnit &pu  = tempCS->addPU( cu, partitioner.chType );

      DistParam distParam;
      const Bool bUseHadamard= !encTestMode.lossless;
      m_pcRdCost->setDistParam (distParam, tempCS->getOrgBuf().Y(), m_acMergeBuffer[0].Y(), sps.getBitDepth (CHANNEL_TYPE_LUMA), COMPONENT_Y, bUseHadamard);

      const UnitArea localUnitArea( tempCS->area.chromaFormat, Area( 0, 0, tempCS->area.Y().width, tempCS->area.Y().height) );

      for( UInt uiMergeCand = 0; uiMergeCand < mergeCtx.numValidMergeCand; uiMergeCand++ )
      {
        acMergeBuffer[uiMergeCand] = m_acMergeBuffer[uiMergeCand].getBuf( localUnitArea );

        mergeCtx.setMergeInfo( pu, uiMergeCand );
        pu.mergeRefPicList = mergeRefPicList;

        PU::spanMotionInfo( pu, mergeCtx );

        distParam.cur = acMergeBuffer[uiMergeCand].Y();

        pu.mvRefine = true;
        m_pcInterSearch->motionCompensation( pu,  acMergeBuffer[uiMergeCand] );
        pu.mvRefine = false;

        m_pcInterSearch->subBlockOBMC      ( pu, &acMergeBuffer[uiMergeCand], false );
        
        if( !( cu.slice->getSPS()->getSpsNext().getDiffusionFilterMode() >> 2 ) )
        {
          if( cu.diffFilterIdx )
          {
            PelBuf predY = acMergeBuffer[uiMergeCand].Y();
            m_DiffusionFilter->applyDiffusion(COMPONENT_Y, cu, cu.blocks[COMPONENT_Y], predY, predY);
          }
        }
        if( mergeCtx.interDirNeighbours[uiMergeCand] == 3 && mergeCtx.mrgTypeNeighbours[uiMergeCand] == MRG_TYPE_DEFAULT_N )
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

        CHECK( std::min( uiMergeCand + 1, uiNumMrgSATDCand ) != RdModeList.size(), "" );
      }

      // Try to limit number of candidates using SATD-costs
      for( UInt i = 1; i < uiNumMrgSATDCand; i++ )
      {
        if( candCostList[i] > MRG_FAST_RATIO * candCostList[0] )
        {
          uiNumMrgSATDCand = i;
          break;
        }
      }

      tempCS->initStructData( encTestMode.qp, encTestMode.lossless );
    }
  }

  const UInt iteration = ( encTestMode.lossless || mergeRefPicList != REF_PIC_LIST_X ) ? 1 : 2;

  // 2. Pass: check candidates using full RD test
  for( UInt uiNoResidualPass = 0; uiNoResidualPass < iteration; uiNoResidualPass++ )
  {
    for( UInt uiMrgHADIdx = 0; uiMrgHADIdx < uiNumMrgSATDCand; uiMrgHADIdx++ )
    {
      UInt uiMergeCand = RdModeList[uiMrgHADIdx];

      if( ( (uiNoResidualPass != 0) && candHasNoResidual[uiMergeCand] )
       || ( (uiNoResidualPass == 0) && bestIsSkip ) )
      {
        continue;
      }

      // first get merge candidates
      CodingUnit &cu      = tempCS->addCU( tempCS->area, partitioner.chType );

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
      cu.obmcFlag         = sps.getSpsNext().getUseOBMC();
      PredictionUnit &pu  = tempCS->addPU( cu, partitioner.chType );

      mergeCtx.setMergeInfo( pu, uiMergeCand );
      pu.mergeRefPicList = mergeRefPicList;
      PU::spanMotionInfo( pu, mergeCtx );

#if MCTS_ENC_CHECK
      if( m_pcEncCfg->getTMCTSSEITileConstraint() && ( !( m_pcInterSearch->checkTMctsMv( pu ) ) ) )
      {
        tempCS->initStructData( encTestMode.qp, encTestMode.lossless );
        continue;
      }

#endif
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
        
        if( cu.diffFilterIdx )
        {
          PelBuf predY = tempCS->getPredBuf().Y();
          m_DiffusionFilter->applyDiffusion( COMPONENT_Y, cu, cu.blocks[COMPONENT_Y], predY, predY );
        }
      }

      xEncodeInterResidual( tempCS, bestCS, partitioner, encTestMode, uiNoResidualPass, NULL, true, ( ( uiNoResidualPass == 0 ) ? &candHasNoResidual[uiMergeCand] : NULL ), cu.slice->getSPS()->getSpsNext().getDiffusionFilterMode() >> 2 );
      if( m_pcEncCfg->getUseFastDecisionForMerge() && !bestIsSkip )
      {
#if MCTS_ENC_CHECK
        bestIsSkip = !bestCS->cus.empty() && bestCS->getCU( partitioner.chType )->rootCbf == 0;
#else
        bestIsSkip = bestCS->getCU( partitioner.chType )->rootCbf == 0;
#endif
      }
      tempCS->initStructData( encTestMode.qp, encTestMode.lossless );
    }// end loop uiMrgHADIdx

    if( mergeRefPicList == REF_PIC_LIST_X )
    if( uiNoResidualPass == 0 && m_pcEncCfg->getUseEarlySkipDetection() )
    {
      const CodingUnit     &bestCU = *bestCS->getCU( partitioner.chType );
      const PredictionUnit &bestPU = *bestCS->getPU( partitioner.chType );

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
              absolute_MV += bestPU.mvd[uiRefListIdx].getAbsHor() + bestPU.mvd[uiRefListIdx].getAbsVer();
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
  bool          hasNoResidual = false;

  const SPS &sps       = *tempCS->sps;

  tempCS->initStructData( encTestMode.qp, encTestMode.lossless );

  CodingUnit &cu      = tempCS->addCU( tempCS->area, partitioner.chType );

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
  cu.obmcFlag         = sps.getSpsNext().getUseOBMC();

  CU::addPUs( cu );

  cu.firstPU->mergeFlag = true;
  cu.firstPU->mergeIdx  = 0;

  PU::getAffineMergeCand( *cu.firstPU, affineMvField, interDirNeighbours, numValidMergeCand );
  if( numValidMergeCand == -1 )
  {
    return;
  }

  cu.firstPU->interDir = interDirNeighbours;
  PU::setAllAffineMvField( *cu.firstPU, affineMvField[REF_PIC_LIST_0], REF_PIC_LIST_0 );
  PU::setAllAffineMvField( *cu.firstPU, affineMvField[REF_PIC_LIST_1], REF_PIC_LIST_1 );

  PU::spanMotionInfo( *cu.firstPU );

  m_pcInterSearch->motionCompensation( cu );
  m_pcInterSearch->subBlockOBMC      ( cu );

  xEncodeInterResidual( tempCS, bestCS, partitioner, encTestMode, 0, NULL, true, &hasNoResidual, 0 );

  if( ! (encTestMode.lossless || hasNoResidual) )
  {
    tempCS->initStructData( encTestMode.qp, encTestMode.lossless );
    tempCS->copyStructure( *bestCS, partitioner.chType );
    tempCS->getPredBuf().copyFrom( bestCS->getPredBuf() );

    xEncodeInterResidual( tempCS, bestCS, partitioner, encTestMode, 1, NULL, true, &hasNoResidual, 0 );
  }
}

void EncCu::xCheckRDCostInter( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode )
{
  tempCS->initStructData( encTestMode.qp, encTestMode.lossless );

  const SPS &sps      = *tempCS->sps;
  CodingUnit &cu      = tempCS->addCU( tempCS->area, partitioner.chType );

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
  cu.obmcFlag         = sps.getSpsNext().getUseOBMC();
  CU::addPUs( cu );

#if AMP_MRG
  m_pcInterSearch->predInterSearch( cu, partitioner, ( encTestMode.opts & ETO_FORCE_MERGE ) );
#else
  m_pcInterSearch->predInterSearch( cu, partitioner );
#endif

  const unsigned wIdx = gp_sizeIdxInfo->idxFrom( tempCS->area.lwidth () );
  if( m_pTempCUWoOBMC )
  {
    const unsigned hIdx = gp_sizeIdxInfo->idxFrom( tempCS->area.lheight() );
    m_pTempCUWoOBMC[wIdx][hIdx]->clearCUs();
    m_pTempCUWoOBMC[wIdx][hIdx]->clearPUs();
    m_pTempCUWoOBMC[wIdx][hIdx]->clearTUs();
    m_pTempCUWoOBMC[wIdx][hIdx]->copyStructure( *tempCS, partitioner.chType );

    m_pPredBufWoOBMC[wIdx][hIdx].copyFrom( tempCS->getPredBuf( cu ) );

    m_pcInterSearch->subBlockOBMC( cu );

    m_pTempCUWoOBMC[wIdx][hIdx]->getPredBuf( cu ).copyFrom( tempCS->getPredBuf( cu ) );
  }

  xEncodeInterResidual( tempCS, bestCS, partitioner, encTestMode, 0, ( m_pImvTempCS ? m_pImvTempCS[wIdx][encTestMode.partSize] : NULL ) );
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

  const SPS &sps = *tempCS->sps;
  const unsigned wIdx = gp_sizeIdxInfo->idxFrom( tempCS->area.lwidth () );
  const unsigned hIdx = gp_sizeIdxInfo->idxFrom( tempCS->area.lheight() );

  CodingStructure* CSWoOBMC = m_pTempCUWoOBMC[wIdx][hIdx];
  CodingUnit *cu = CSWoOBMC->getCU( partitioner.chType );

  if( !cu->obmcFlag || !CU::isObmcFlagCoded( *cu ) )
  {
    return;
  }

  const UInt    uiSADOBMCOff = m_pcRdCost->getDistPart( tempCS->getOrgBuf( cu->Y() ), m_pPredBufWoOBMC[wIdx][hIdx].Y(), sps.getBitDepth( CHANNEL_TYPE_LUMA ), COMPONENT_Y, DF_SAD_FULL_NBIT );
  const UInt    uiSADOBMCOn  = m_pcRdCost->getDistPart( tempCS->getOrgBuf( cu->Y() ), CSWoOBMC->getPredBuf( cu->Y() ),  sps.getBitDepth( CHANNEL_TYPE_LUMA ), COMPONENT_Y, DF_SAD_FULL_NBIT );
  const Double    dOBMCThOff = 1.0;
  const Bool   bCheckOBMCOff = uiSADOBMCOff * dOBMCThOff < uiSADOBMCOn;

  if( !bCheckOBMCOff )
  {
    return;
  }

  tempCS->copyStructure( *CSWoOBMC,partitioner.chType );
  tempCS->getPredBuf( *cu ).copyFrom( m_pPredBufWoOBMC[wIdx][hIdx] );
  cu           = tempCS->getCU( partitioner.chType );
  cu->obmcFlag = false;
  CHECK( cu->firstPU->mergeFlag && cu->partSize == SIZE_2Nx2N, "Merge2Nx2Ns is on" );

  xEncodeInterResidual( tempCS, bestCS, partitioner, encTestMode, 0, ( m_pImvTempCS ? m_pImvTempCS[wIdx][encTestMode.partSize] : NULL ), !m_pcEncCfg->getFastInterEMT() );
}

void EncCu::xCheckRDCostMerge2Nx2NFRUC( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode, const RefPicList mergeRefPicList )
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

  Bool transQuantBypass = tempCS->isLossless;

  MergeCtx mergeCtx;
  const SPS &sps = *tempCS->sps;
  Size bufSize = g_miScaling.scale( tempCS->area.lumaSize() );
  mergeCtx.subPuFrucMiBuf = MotionBuf( m_SubPuFrucBuf, bufSize );

  const UChar uhFRUCME[2] = { FRUC_MERGE_BILATERALMV, FRUC_MERGE_TEMPLATE };

  PelUnitBuf acMergeBuffer[2];

  for( Int nME = 0; nME < 2; nME++ )
  {
    if( mergeRefPicList == REF_PIC_LIST_X )
    {
      MotionBuf( m_savedSubPuBuf[nME], bufSize ).fill( MotionInfo() );
    }
    if( slsCtrl && m_pcEncCfg->getUseSaveLoadEncInfo() && LOAD_ENC_INFO == slsCtrl->getSaveLoadTag( tempCS->area ) && uhFRUCME[nME] != slsCtrl->getSaveLoadFrucMode( tempCS->area ) )
    {
      continue;
    }

    tempCS->initStructData( encTestMode.qp, encTestMode.lossless );

    Bool bAvailable        = false;

    CodingUnit &cu         = tempCS->addCU( tempCS->area, partitioner.chType );

    partitioner.setCUData( cu );
    cu.slice               = tempCS->slice;
    cu.tileIdx             = tempCS->picture->tileMap->getTileIdxMap( tempCS->area.lumaPos() );
  //cu.skip                = false;
    cu.partSize            = SIZE_2Nx2N;
  //cu.affine
    cu.predMode            = MODE_INTER;
    cu.LICFlag             = ( ( encTestMode.opts & ETO_LIC ) != 0 );
    cu.obmcFlag            = sps.getSpsNext().getUseOBMC();

    PredictionUnit &puFruc = tempCS->addPU( cu, partitioner.chType );
    puFruc.mergeFlag       = true;
    puFruc.mergeIdx        = 0;
    puFruc.frucMrgMode     = uhFRUCME[nME];
    puFruc.mergeType       = MRG_TYPE_FRUC;

    CHECK( puFruc.mergeRefPicList != REF_PIC_LIST_X, "puFruc is bad initialized" );
    if( mergeRefPicList != REF_PIC_LIST_X )
    {
      const auto mb = MotionBuf( m_savedSubPuBuf[nME], bufSize );
      if( !mb.at(0,0).isInter )
      {
        // was not available before
        continue;
      }
      bAvailable = true;
      puFruc.getMotionBuf().copyFrom( mb );
      CHECK( !m_savedInterPredData[nME].mergeFlag, "mismatch" );
      CHECK( m_savedInterPredData[nME].mergeIdx != 0, "mismatch" );
      CHECK( m_savedInterPredData[nME].mergeType != MRG_TYPE_FRUC, "mismatch" );
      CHECK( m_savedInterPredData[nME].frucMrgMode != uhFRUCME[nME], "mismatch" );
      CHECK( m_savedInterPredData[nME].interDir < 1 || m_savedInterPredData[nME].interDir > 3, "mismatch" );
      puFruc = m_savedInterPredData[nME];
      puFruc.mergeRefPicList = mergeRefPicList;
      PU::restrictFRUCRefList( puFruc, m_savedMergeCtx );
    }
    else
    {
      bAvailable = m_pcInterSearch->deriveFRUCMV( puFruc, &m_savedMergeCtx );
      if( bAvailable )
      {
        m_savedInterPredData[nME] = puFruc;
        auto mb = MotionBuf( m_savedSubPuBuf[nME], bufSize );
        mb.copyFrom( puFruc.getMotionBuf() );
      }
    }

    if( bAvailable )
    {
      const UnitArea localUnitArea( tempCS->area.chromaFormat, Area( 0, 0, tempCS->area.lwidth(), tempCS->area.lheight() ) );
      acMergeBuffer[nME] = m_acMergeBuffer[nME].getBuf( localUnitArea );

      m_pcInterSearch->motionCompensation( puFruc,  acMergeBuffer[nME] );
      m_pcInterSearch->subBlockOBMC      ( puFruc, &acMergeBuffer[nME], false );

      CHECK( mergeCtx.subPuFrucMiBuf.area() == 0 || !mergeCtx.subPuFrucMiBuf.buf, "Buffer not initialized" );
      mergeCtx.subPuFrucMiBuf.fill( MotionInfo() );
      mergeCtx.subPuFrucMiBuf.copyFrom( puFruc.getMotionBuf() );

      UInt iteration = ( encTestMode.lossless || mergeRefPicList != REF_PIC_LIST_X ) ? 1 : 2;
      bool candHasNoResidual = false;
      // 2. Pass: check candidates using full RD test
      for( UInt uiNoResidualPass = 0; uiNoResidualPass < iteration; uiNoResidualPass++ )
      {
        if( (uiNoResidualPass != 0) && candHasNoResidual )
        {
          continue;
        }

        // first get merge candidates
        tempCS->initStructData( encTestMode.qp, encTestMode.lossless );
        CodingUnit &cu      = tempCS->addCU( tempCS->area, partitioner.chType );

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
        cu.qp               = encTestMode.qp;
        cu.obmcFlag         = sps.getSpsNext().getUseOBMC();

        PredictionUnit &pu  = tempCS->addPU( cu, partitioner.chType );
        pu.mergeFlag        = true;
        pu.mergeIdx         = 0;
        pu.frucMrgMode      = uhFRUCME[nME];
        pu.mergeRefPicList  = mergeRefPicList;

        pu.mergeType        = MRG_TYPE_FRUC_SET;
        PU::spanMotionInfo( pu, mergeCtx );
        pu.mergeType        = MRG_TYPE_FRUC;

        tempCS->getPredBuf().copyFrom( acMergeBuffer[ nME ]);

        xEncodeInterResidual( tempCS, bestCS, partitioner, encTestMode, uiNoResidualPass, NULL, true, ( ( uiNoResidualPass == 0 ) ? &candHasNoResidual : NULL ) );
      } // end loop uiNoResidualPass
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
      tempCS->copyStructure( *pcCUInfo2Reuse, partitioner.chType );
      tempCS->fracBits = 0;
      tempCS->dist     = 0;
      tempCS->cost     = MAX_DOUBLE;
    }
  }

  CodingUnit &cu = ( pcCUInfo2Reuse != nullptr ) ? *tempCS->getCU( partitioner.chType ) : tempCS->addCU( tempCS->area, partitioner.chType );

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

    CU::addPUs( cu );
  }
  else
  {
    CHECK( cu.partSize != encTestMode.partSize,    "Mismatch" );
    CHECK( cu.skip,                                "Mismatch" );
    CHECK( cu.qtDepth  != partitioner.currQtDepth, "Mismatch" );
    CHECK( cu.btDepth  != partitioner.currBtDepth, "Mismatch" );
    CHECK( cu.mtDepth  != partitioner.currMtDepth, "Mismatch" );
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
    m_pcInterSearch->predInterSearch( cu, partitioner );
#endif
  }

  if( !CU::hasSubCUNonZeroMVd( cu ) )
  {
    m_modeCtrl->useModeResult( encTestModeBase, tempCS, partitioner );
    return false;
  }

  if( m_pTempCUWoOBMC )
  {
    const unsigned wIdx = gp_sizeIdxInfo->idxFrom( tempCS->area.lwidth () );
    const unsigned hIdx = gp_sizeIdxInfo->idxFrom( tempCS->area.lheight() );

    m_pTempCUWoOBMC[wIdx][hIdx]->clearCUs();
    m_pTempCUWoOBMC[wIdx][hIdx]->clearPUs();
    m_pTempCUWoOBMC[wIdx][hIdx]->clearTUs();
    m_pTempCUWoOBMC[wIdx][hIdx]->copyStructure( *tempCS, partitioner.chType );

    m_pPredBufWoOBMC[wIdx][hIdx].copyFrom( tempCS->getPredBuf( cu ) );

    m_pcInterSearch->subBlockOBMC( cu );

    m_pTempCUWoOBMC[wIdx][hIdx]->getPredBuf( cu ).copyFrom( tempCS->getPredBuf( cu ) );
  }

  xEncodeInterResidual( tempCS, bestCS, partitioner, encTestModeBase, 0, NULL, true );

  return true;
}

void EncCu::xEncodeInterResidual( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode, int residualPass, CodingStructure* imvCS, int emtMode, bool* bestHasNonResi, bool testDiffusion )
{
  if( residualPass == 1 && encTestMode.lossless )
  {
    return;
  }

  double           bestCost        = bestCS->cost;
  const SPS&            sps        = *tempCS->sps;
  const int      maxSizeEMT        = tempCS->pcv->noRQT ? EMT_INTER_MAX_CU_WITH_QTBT : EMT_INTER_MAX_CU;
  CodingUnit*            cu        = tempCS->getCU( partitioner.chType );
  bool              swapped        = false; // avoid unwanted data copy
  bool             reloadCU        = false;
  double   bestCostInternal        = MAX_DOUBLE;
  const bool bMergeFruc            = (ETM_MERGE_SKIP == encTestMode.type ) || (ETM_MERGE_FRUC == encTestMode.type);
  const bool considerEmtSecondPass = (emtMode != 0) && sps.getSpsNext().getUseInterEMT() && partitioner.currArea().lwidth() <= maxSizeEMT && partitioner.currArea().lheight() <= maxSizeEMT;
  int minEMTMode = 0;
  int maxEMTMode = (considerEmtSecondPass?1:0);

  if( emtMode == 2 )
  {
    minEMTMode = maxEMTMode = (cu->emtFlag?1:0);
  }

  int resetDiffIdx = 0;
  if( cu->firstPU->mergeFlag && cu->firstPU->frucMrgMode == FRUC_MERGE_OFF )
  {
    resetDiffIdx = cu->diffFilterIdx;
  }

  for( int curEmtMode = minEMTMode; curEmtMode <= maxEMTMode; curEmtMode++ )
  {
    if( reloadCU )
    {
      if( bestCost == bestCS->cost ) //The first EMT pass didn't become the bestCS, so we clear the TUs generated
      {
        tempCS->clearTUs();
      }
      else if( false == swapped )
      {
        tempCS->initStructData( encTestMode.qp, encTestMode.lossless );
        tempCS->copyStructure( *bestCS, partitioner.chType );
        tempCS->getPredBuf().copyFrom( bestCS->getPredBuf() );
        bestCost = bestCS->cost;
        cu       = tempCS->getCU( partitioner.chType );
        swapped = true;
      }
      else
      {
        tempCS->clearTUs();
        bestCost = bestCS->cost;
        cu       = tempCS->getCU( partitioner.chType );
      }

      //we need to restart the distortion for the new tempCS, the bit count and the cost
      tempCS->dist     = 0;
      tempCS->fracBits = 0;
      tempCS->cost     = MAX_DOUBLE;
      cu->diffFilterIdx = resetDiffIdx;
    }

    reloadCU    = true; // enable cu reloading
    cu->skip    = false;
    cu->emtFlag = curEmtMode;

    const bool skipResidual = residualPass == 1;
    if( bMergeFruc )
    {
      m_CABACEstimator->getCtx() = m_CurrCtx->start;
    }
    m_pcInterSearch->encodeResAndCalcRdInterCU( *tempCS, partitioner, skipResidual );

    double emtFirstPassCost = tempCS->cost;

    xEncodeDontSplit( *tempCS, partitioner );

    xCheckDQP( *tempCS, partitioner );

    double emtFirstPassCostFruc = tempCS->cost;

    if( imvCS && (tempCS->cost < imvCS->cost) )
    {
      if( imvCS->cost != MAX_DOUBLE )
      {
        imvCS->initStructData( encTestMode.qp, encTestMode.lossless );
      }
      imvCS->copyStructure( *tempCS, partitioner.chType );
    }

    if( NULL != bestHasNonResi && (bestCostInternal > tempCS->cost) )
    {
      bestCostInternal = tempCS->cost;
      *bestHasNonResi  = !cu->rootCbf;
    }

    DTRACE_MODE_COST( *tempCS, m_pcRdCost->getLambda() );
    xCheckBestMode( tempCS, bestCS, partitioner, encTestMode );
    bool cbf = cu->firstTU->cbf[COMPONENT_Y];

    if (testDiffusion && cbf)
    {
      const double bestCostBeforDifusion = bestCS->cost;
      bool change = bestCost != bestCS->cost;
      xEncodeInterResidualWithDiffusionFilter(tempCS, bestCS, partitioner, encTestMode, skipResidual, curEmtMode ? nullptr : &emtFirstPassCost, change);
      if( bestCostBeforDifusion != bestCS->cost )
      {
        bestCostInternal = bestCS->cost;
        cbf = bestCS->getCU( partitioner.chType )->firstTU->cbf[COMPONENT_Y];
        if( NULL != bestHasNonResi )
        {
          *bestHasNonResi = !bestCS->getCU( partitioner.chType )->rootCbf;
        }

        if( bMergeFruc && ( sps.getSpsNext().getDiffusionFilterMode() >> 2 ) )
        {
          CHECK( cu->firstPU->mergeRefPicList == REF_PIC_LIST_X && ( !bestCS->getCU( partitioner.chType )->rootCbf ), "in case of (not restricted) merge pde hasResidual should be true" );
          if( ETM_MERGE_SKIP == encTestMode.type )
          {
            CHECK( bestCS->getCU( CHANNEL_TYPE_LUMA )->diffFilterIdx == 0, "error" );
          }
        }
      }
    }

    //now we check whether the second pass should be skipped or not
    if( 0 == curEmtMode && 0 != maxEMTMode )
    {
      const double thresholdToSkipEmtSecondPass = 1.1; // Skip checking EMT transforms
      const bool bCond1 = !cbf;
      const bool bCond2 = bMergeFruc && (emtFirstPassCostFruc > (bestCost * thresholdToSkipEmtSecondPass));
      const bool bCond3 = !bMergeFruc && (emtFirstPassCost > (bestCost * thresholdToSkipEmtSecondPass));

      if( m_pcEncCfg->getFastInterEMT() && (bCond1 || bCond2 || bCond3 ) ) 
      {
        maxEMTMode = 0; // do not test EMT
      }
    }
  }//end emt loop 
}


void EncCu::xEncodeDontSplit( CodingStructure &cs, Partitioner &partitioner )
{
  m_CABACEstimator->resetBits();

  {
    if( partitioner.canSplit( CU_QUAD_SPLIT, cs ) )
    {
      m_CABACEstimator->split_cu_flag( false, cs, partitioner );
    }
    if( cs.sps->getSpsNext().getUseGenBinSplit() )
    {
      m_CABACEstimator->gen_bin_split_mode( CU_DONT_SPLIT, cs, partitioner );
    }
    else 
    if( partitioner.canSplit( CU_MT_SPLIT, cs ) )
    {
      m_CABACEstimator->split_cu_mode_mt( CU_DONT_SPLIT, cs, partitioner );
    }
  }

  cs.fracBits += m_CABACEstimator->getEstFracBits(); // split bits
  cs.cost      = m_pcRdCost->calcRdCost( cs.fracBits, cs.dist );

}

void EncCu::xSaveBestInterResultsForMultiHyp( const CodingStructure &cs, Partitioner &partitioner, const EncTestMode& encTestMode )
{
  const bool isNonMultiHypInterMode = encTestMode.type == ETM_INTER_ME
                                      || encTestMode.type == ETM_AFFINE || encTestMode.type == ETM_MERGE_FRUC
                                      || encTestMode.type == ETM_MERGE_SKIP;
  
  if( !isNonMultiHypInterMode || encTestMode.partSize != SIZE_2Nx2N )
    return;

  CHECK( cs.cus.empty(), "current temp inter 2Nx2N mode has no CU" );
  CHECK( cs.pus.empty(), "current temp inter 2Nx2N mode has no PU" );

  const unsigned wIdx = gp_sizeIdxInfo->idxFrom( cs.area.lwidth() );
  const unsigned hIdx = gp_sizeIdxInfo->idxFrom( cs.area.lheight() );
  const int howMany   = m_pcEncCfg->getAddHypTries();
  CodingStructure ** const beginPos  = m_pBestInters2Nx2NCS[wIdx][hIdx];
  CodingStructure ** const endPos    = beginPos + howMany;
  CodingStructure ** const insertPos = std::upper_bound( beginPos, endPos, cs,
                                                         []( const CodingStructure &a, const CodingStructure * const b )
                                                           {
                                                             return a.cost < b->cost;
                                                           } );
  if( insertPos == endPos )
    return;

  std::rotate( insertPos, endPos-1, endPos );

  CodingStructure &insertCS = **insertPos;
  if( insertCS.cost != MAX_DOUBLE )
  {
    insertCS.initStructData( encTestMode.qp, encTestMode.lossless );
  }
  insertCS.copyStructure( cs, partitioner.chType );
}

void EncCu::xCheckRDCostInterMultiHyp2Nx2N (CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode, const int testIdx )
{
  const SPS &sps = *tempCS->sps;
  CHECK( !sps.getSpsNext().getUseInterMultiHyp(), "Multi Hyp is not active" );
  CHECK( !tempCS->slice->isInterB(), "Multi Hyp only allowed in B slices" );
  CHECK( encTestMode.partSize != SIZE_2Nx2N, "EncCu::xCheckRDCostInterMultiHyp2Nx2N() called for non-2Nx2N part size" );
  CHECK( encTestMode.opts != ETO_STANDARD, "unknown encoding option to EncCu::xCheckRDCostInterMultiHyp2Nx2N()" );

  if( m_modeCtrl->getFastDeltaQp() )
  {
    if( tempCS->area.lumaSize().width > tempCS->pcv->fastDeltaQPCuMaxSize )
    {
      return; // only check necessary 2Nx2N Inter in fast deltaqp mode
    }
  }

  const unsigned wIdx = gp_sizeIdxInfo->idxFrom( tempCS->area.lwidth() );
  const unsigned hIdx = gp_sizeIdxInfo->idxFrom( tempCS->area.lheight() );
  CodingStructure &bestInter2Nx2NCS = *m_pBestInters2Nx2NCS[wIdx][hIdx][testIdx];
  if( bestInter2Nx2NCS.cost == MAX_DOUBLE )
  {
    CHECK( testIdx == 0, "should not happen" );
    return;
  }
  if( bestInter2Nx2NCS.cost > 1.1 * bestCS->cost )
  {
    return;
  }
  CHECK( !bestInter2Nx2NCS.getCU( partitioner.chType ), "no CU found for best inter 2Nx2N mode" );
  CHECK( !bestInter2Nx2NCS.getPU( partitioner.chType ), "no PU found for best inter 2Nx2N mode" );
  CHECK( ( bestInter2Nx2NCS.getPU( partitioner.chType )->mergeFlag ? (bestInter2Nx2NCS.getPU( partitioner.chType )->numMergedAddHyps != bestInter2Nx2NCS.getPU( partitioner.chType )->addHypData.size() )
                                               : (!bestInter2Nx2NCS.getPU( partitioner.chType )->addHypData.empty()) ),
         "EncCu::xCheckRDCostInterMultiHyp2Nx2N() starts with filled addHypData" );

  if( PU::isBipredRestriction( *bestInter2Nx2NCS.getPU( partitioner.chType ) ) )
  {
    CHECK( !bestInter2Nx2NCS.getPU( partitioner.chType )->mergeFlag, "bi-pred restriction and not merge" );
    return;
  }

  tempCS->initStructData( encTestMode.qp, encTestMode.lossless );
  tempCS->copyStructure( bestInter2Nx2NCS, partitioner.chType );

  bool weHaveWonAtLeastOnce = false;
  int remainingTries = 1;
  const size_t maxNumAddHyps = sps.getSpsNext().getMaxNumAddHyps();
  while( remainingTries > 0 && tempCS->getPU( partitioner.chType )->addHypData.size() < maxNumAddHyps )
  {
    tempCS->clearTUs();
    tempCS->fracBits = 0;
    tempCS->dist     = 0;
    tempCS->cost     = MAX_DOUBLE;

    CodingUnit &cu = *tempCS->getCU( partitioner.chType );

    CHECK( cu.partSize != encTestMode.partSize,    "Mismatch" );
    cu.skip = false;
    CHECK( cu.qtDepth  != partitioner.currQtDepth, "Mismatch" );
    CHECK( cu.btDepth  != partitioner.currBtDepth, "Mismatch" );
    CHECK( cu.mtDepth  != partitioner.currMtDepth, "Mismatch" );

    PredictionUnit &pu = *tempCS->getPU( partitioner.chType );
    if( !( cu.slice->getSPS()->getSpsNext().getDiffusionFilterMode() >> 2 ) )
    {
      if( !( pu.mergeFlag && pu.frucMrgMode == FRUC_MERGE_OFF ) )
      {
        cu.diffFilterIdx = 0;
      }
    }
    else
    {
      cu.diffFilterIdx = 0;
    }
    m_pcInterSearch->predInterSearchAdditionalHypothesis( pu );
    m_pcInterSearch->motionCompensation( pu );
    m_pcInterSearch->subBlockOBMC      ( cu );

    if( !( cu.slice->getSPS()->getSpsNext().getDiffusionFilterMode() >> 2 ) )

    {
      if( cu.diffFilterIdx )
      {
        PelBuf predY = tempCS->getPredBuf().Y();
        m_DiffusionFilter->applyDiffusion( COMPONENT_Y, cu, cu.blocks[COMPONENT_Y], predY, predY );
      }
    }

    const auto bestCost = bestCS->cost;
    xEncodeInterResidual( tempCS, bestCS, partitioner, encTestMode, 0, NULL, 2 );

    CHECK( cu.skip, "SKIP mode detected in EncCu::xCheckRDCostInterMultiHyp2Nx2N()" );
    CHECK( bestCS->cost > bestCost, "should not happen" );

    const bool weHaveLost = ( bestCost == bestCS->cost );
    if( weHaveLost )
    {
      --remainingTries;
    }
    else
    {
      if( !weHaveWonAtLeastOnce )
      {
        weHaveWonAtLeastOnce = true;
        // we have a new tempCS
        tempCS->initStructData( encTestMode.qp, encTestMode.lossless );
        tempCS->copyStructure( bestInter2Nx2NCS, partitioner.chType );
      }
      tempCS->getPU( partitioner.chType )->addHypData = pu.addHypData;
    }
  }
}

void EncCu::xCheckRDCostMotionVectorReestimation( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode )
{
  CHECK( m_pcEncCfg->getMvReestIters() <= 0, "Mv re-estimation iters <= 0" );
  CHECK( m_pcEncCfg->getMvReestRange() <= 0, "Mv re-estimation range <= 0" );
  CHECK( encTestMode.partSize != SIZE_2Nx2N, "EncCu::xCheckRDCostMotionVectorReestimation() called for non-2Nx2N part size" );
  CHECK( encTestMode.opts != ETO_STANDARD, "unknown encoding option to EncCu::xCheckRDCostMotionVectorReestimation()" );
  if( m_modeCtrl->getFastDeltaQp() )
  {
    if( tempCS->area.lumaSize().width > tempCS->pcv->fastDeltaQPCuMaxSize )
    {
      return; // only check necessary 2Nx2N Inter in fast deltaqp mode
    }
  }

  CHECK( bestCS->cus.size() != 1, "wrong number of CUs" );
  if( CU::isIntra( *bestCS->getCU( partitioner.chType ) ) || bestCS->getCU( partitioner.chType )->partSize != SIZE_2Nx2N || bestCS->getCU( partitioner.chType )->affine )
    return;
  tempCS->initStructData( encTestMode.qp, encTestMode.lossless );
  tempCS->copyStructure( *bestCS, partitioner.chType );

  bool weHaveWonAtLeastOnce = false;
  InterPredictionData bestInterPredictionData = *bestCS->getPU( partitioner.chType );
  const int numHyps = int( bestInterPredictionData.addHypData.size() + 2 );
  const int numIters = m_pcEncCfg->getMvReestIters();
  const int range = m_pcEncCfg->getMvReestRange();
  for( int iter = 0; iter < numIters; ++iter )
  {
    auto motionVectorsHaveBeenChanged = false;
    for( int hyp = bestInterPredictionData.mergeFlag ? ( 2+bestInterPredictionData.numMergedAddHyps ) : 0; hyp < numHyps; ++hyp )
    {
      if( hyp == 1 && bestCS->slice->getMvdL1ZeroFlag() )
        continue;
      if( hyp < 2 && ( bestInterPredictionData.interDir & (1 << hyp) ) == 0 )
        continue;
      for( int dy = -range; dy <= range; ++dy )
      for( int dx = -range; dx <= range; ++dx )
      {
        const auto len = std::abs(dx) + std::abs(dy);
        if( (!motionVectorsHaveBeenChanged && len == 0) || len > range )
          continue;
        tempCS->clearTUs();
        tempCS->fracBits = 0;
        tempCS->dist     = 0;
        tempCS->cost     = MAX_DOUBLE;

        CodingUnit &cu = *tempCS->getCU( partitioner.chType );

        CHECK( cu.partSize != encTestMode.partSize,    "Mismatch" );
        cu.skip = false;
        CHECK( cu.qtDepth  != partitioner.currQtDepth, "Mismatch" );
        CHECK( cu.btDepth  != partitioner.currBtDepth, "Mismatch" );
        CHECK( cu.mtDepth  != partitioner.currMtDepth, "Mismatch" );

        PredictionUnit &pu = *tempCS->getPU( partitioner.chType );
        pu = bestInterPredictionData;

        if( !( cu.slice->getSPS()->getSpsNext().getDiffusionFilterMode() >> 2 ) )
        {
          if( !( pu.mergeFlag && pu.frucMrgMode == FRUC_MERGE_OFF ) )
          {
            cu.diffFilterIdx = 0;
          }
        }
        else
        {
          cu.diffFilterIdx = 0;
        }

        Mv &mv  = hyp < 2 ? pu.mv [hyp] : pu.addHypData[hyp-2].mv;
        Mv &mvd = hyp < 2 ? pu.mvd[hyp] : pu.addHypData[hyp-2].mvd;

        const int imvScale = 1 << ( cu.imv << 1 );
        mv.hor += dx*imvScale; mvd.hor += dx*imvScale;
        mv.ver += dy*imvScale; mvd.ver += dy*imvScale;

        if( cu.imv && !CU::hasSubCUNonZeroMVd( cu ) )
        {
          CHECK( pu.mergeFlag, "Multi Hyp: pu.mergeFlag" );
          // we would have to update all the mvps in this case, as they are currently rounded
          // whereas we would need the "unrounded" ones
          // (besides, we would have to set cu.imv=0)
          //   ==> avoid, and skip to next position
          continue;
        }
        if( !pu.mergeFlag )
          PU::spanMotionInfo( pu );
        m_pcInterSearch->motionCompensation( pu );
        m_pcInterSearch->subBlockOBMC      ( cu );
        if( !( cu.slice->getSPS()->getSpsNext().getDiffusionFilterMode() >> 2 ) )
        {
          if (cu.diffFilterIdx)
          {
            PelBuf predY = tempCS->getPredBuf().Y();
            m_DiffusionFilter->applyDiffusion( COMPONENT_Y, cu, cu.blocks[COMPONENT_Y], predY, predY );
          }
        }

        const double bestCost = bestCS->cost;
        xEncodeInterResidual( tempCS, bestCS, partitioner, encTestMode, 0, NULL, 2 );
        CHECK( cu.skip, "SKIP mode detected in EncCu::xCheckRDCostInterMultiHyp2Nx2N()" );

        const bool weHaveWon = bestCS->cost < bestCost;
        if( weHaveWon )
        {
          motionVectorsHaveBeenChanged = true;
          if( !weHaveWonAtLeastOnce )
          {
            weHaveWonAtLeastOnce = true;
            // we have a new tempCS
            tempCS->initStructData( encTestMode.qp, encTestMode.lossless );
            tempCS->copyStructure( *bestCS, partitioner.chType );
          }
        }
      }

      if( weHaveWonAtLeastOnce )
        bestInterPredictionData = *bestCS->getPU( partitioner.chType );
    }
    if( !motionVectorsHaveBeenChanged )
      break;
  }
}

void EncCu::xEncodeInterResidualWithDiffusionFilter( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode, const bool skipResidual, double *bestIntermediateCostForEMT, bool change )
{
  if (change)
  {
    if (!CU::isDiffIdxPresent(*bestCS->getCU(partitioner.chType)))
    return;
    CHECK(bestCS->getCU(partitioner.chType)->diffFilterIdx != 0, "diffFilterIdx already set");
  }
  else
  {
    if (!CU::isDiffIdxPresent(*tempCS->getCU(partitioner.chType)))
      return;
    CHECK(tempCS->getCU(partitioner.chType)->diffFilterIdx != 0, "diffFilterIdx already set");
  }

  if (skipResidual)
    return;

  if (!change && (tempCS->cost > 1.4*bestCS->cost))
    return;

  const SPS &sps      = *tempCS->sps;

  if (change)
  {
    tempCS->initStructData(encTestMode.qp, encTestMode.lossless);
    tempCS->copyStructure(*bestCS, partitioner.chType);
    tempCS->getPredBuf().copyFrom(bestCS->getPredBuf());
  }
  else
  {
    tempCS->clearTUs();
  }
  tempCS->fracBits      = 0;
  tempCS->dist          = 0;
  tempCS->cost          = MAX_DOUBLE;

  CodingUnit &cuTest    = *tempCS->getCU( partitioner.chType );

  const UnitArea localArea  = UnitArea( tempCS->area.chromaFormat, Area( 0, 0, tempCS->area.Y().width, tempCS->area.Y().height ) );
  PelUnitBuf     diffuBuf   = m_DiffuFiltBuf[1].getBuf( localArea );
  PelBuf bestPDEPredSignal  = m_DiffuFiltBuf[0].getBuf( localArea ).Y();

  PelUnitBuf StartpredSignal = m_DiffuFiltBuf[2].getBuf(localArea);
  StartpredSignal.copyFrom(tempCS->getPredBuf());

  Distortion bestPDEDist    = std::numeric_limits<Distortion>::max();
  unsigned bestDiffFilterIdx = 0;

  unsigned minDiffusionFilterIdx = 1;
  unsigned maxDiffusionFilterIdx = tempCS->sps->getSpsNext().getNumDiffusionFiltersInter();

  const auto bestOrgDist = m_pcRdCost->getDistPart(bestCS->getOrgBuf().Y(), bestCS->getPredBuf().Y(), sps.getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y, DF_HAD);
  std::vector<Bool> testDiffFilter;
  testDiffFilter.resize(maxDiffusionFilterIdx, 0);
  testDiffFilter[0] = 1; //first linear filter should be tested
  testDiffFilter[maxDiffusionFilterIdx >> 1] = 1; //first nonlinear filter should be tested

  bool b_minFilter = false;
  if ((tempCS->sps->getSpsNext().getRestrDiffusionMode() >> 1) & 1)
  {
    b_minFilter = true;
  }

  for( unsigned diffFilterIdx = minDiffusionFilterIdx; diffFilterIdx <= maxDiffusionFilterIdx; diffFilterIdx++ )
  {
    if( b_minFilter && ( diffFilterIdx == 2 ) )
    {
      if( ( ( tempCS->area.lwidth() < 16 ) || ( tempCS->area.lheight() < 16 ) ) )
      {
        diffFilterIdx += 1;
      }
    }

    if( !testDiffFilter[diffFilterIdx - 1] )
      continue;

    cuTest.diffFilterIdx  = diffFilterIdx;
    m_DiffusionFilter->applyDiffusion(COMPONENT_Y, cuTest, cuTest.blocks[COMPONENT_Y], StartpredSignal.Y(), diffuBuf.Y(), diffFilterIdx - 1);

    const auto currPDEDist = m_pcRdCost->getDistPart( tempCS->getOrgBuf().Y(), diffuBuf.Y(), sps.getBitDepth( CHANNEL_TYPE_LUMA ), COMPONENT_Y, DF_HAD );
    if( currPDEDist < bestPDEDist )
    {
      if( currPDEDist < bestOrgDist && diffFilterIdx < maxDiffusionFilterIdx )
        testDiffFilter[diffFilterIdx] = 1;

      bestPDEDist = currPDEDist;
      bestDiffFilterIdx = diffFilterIdx;
      bestPDEPredSignal.copyFrom( diffuBuf.Y() );
    }
  }

  cuTest.diffFilterIdx  = bestDiffFilterIdx;
  cuTest.skip           = false;

  tempCS->getPredBuf().Y ().copyFrom( bestPDEPredSignal );
  tempCS->getPredBuf().Cb().copyFrom(StartpredSignal.Cb());
  tempCS->getPredBuf().Cr().copyFrom(StartpredSignal.Cr());

  m_pcInterSearch->encodeResAndCalcRdInterCU( *tempCS, partitioner, skipResidual );

  tempCS->getPredBuf().Y().copyFrom(StartpredSignal.Y());

  bool b_testMergeDF = true;

  if( tempCS->slice->getSPS()->getSpsNext().getDiffusionFilterMode() >> 2 )
  {
    if( cuTest.rootCbf == 0 && cuTest.firstPU->mergeFlag )
    {
      b_testMergeDF = false;
    }
  }

  if (bestIntermediateCostForEMT && b_testMergeDF && tempCS->cost < *bestIntermediateCostForEMT)
    *bestIntermediateCostForEMT = tempCS->cost;

  xEncodeDontSplit( *tempCS, partitioner );

  xCheckDQP( *tempCS, partitioner );

  DTRACE_MODE_COST( *tempCS, m_pcRdCost->getLambda() );

  if( !( tempCS->slice->getSPS()->getSpsNext().getDiffusionFilterMode() >> 2 ) || ( !( cuTest.partSize == SIZE_2Nx2N && cuTest.rootCbf == 0 && cuTest.firstPU->mergeFlag ) ) )
  {
    xCheckBestMode( tempCS, bestCS, partitioner, encTestMode );
  }
}

//! \}
