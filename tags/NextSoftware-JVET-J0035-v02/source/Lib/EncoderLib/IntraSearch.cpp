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

/** \file     EncSearch.cpp
 *  \brief    encoder intra search class
 */

#include "IntraSearch.h"

#include "EncModeCtrl.h"

#include "CommonLib/CommonDef.h"
#include "CommonLib/Rom.h"
#include "CommonLib/Picture.h"
#include "CommonLib/UnitTools.h"

#include "CommonLib/dtrace_next.h"
#include "CommonLib/dtrace_buffer.h"

#include <math.h>
#include <limits>

 //! \ingroup EncoderLib
 //! \{

IntraSearch::IntraSearch()
  : m_modeCtrl      (nullptr)
  , m_pSplitCS      (nullptr)
  , m_pFullCS       (nullptr)
  , m_pBestCS       (nullptr)
  , m_pcEncCfg      (nullptr)
  , m_pcTrQuant     (nullptr)
  , m_pcRdCost      (nullptr)
  , m_CABACEstimator(nullptr)
  , m_CtxCache      (nullptr)
  , m_isInitialized (false)
{
  for( UInt ch = 0; ch < MAX_NUM_TBLOCKS; ch++ )
  {
    m_pSharedPredTransformSkip[ch] = nullptr;
  }
}


Void IntraSearch::destroy()
{
  CHECK( !m_isInitialized, "Not initialized" );

  if( m_pcEncCfg )
  {
    bool BTnoRQT = m_pcEncCfg->getGenBinSplit() || m_pcEncCfg->getQTBT();


    const UInt uiNumLayersToAllocateSplit = BTnoRQT ? 1 : m_pcEncCfg->getQuadtreeTULog2MaxSize() - m_pcEncCfg->getQuadtreeTULog2MinSize() + 1;
    const UInt uiNumLayersToAllocateFull  = BTnoRQT ? 1 : m_pcEncCfg->getQuadtreeTULog2MaxSize() - m_pcEncCfg->getQuadtreeTULog2MinSize() + 1;
    const int uiNumSaveLayersToAllocate = 2;

    for( UInt layer = 0; layer < uiNumSaveLayersToAllocate; layer++ )
    {
      m_pSaveCS[layer]->destroy();
      delete m_pSaveCS[layer];
    }

    UInt numWidths  = gp_sizeIdxInfo->numWidths();
    UInt numHeights = gp_sizeIdxInfo->numHeights();

    for( UInt width = 0; width < numWidths; width++ )
    {
      for( UInt height = 0; height < numHeights; height++ )
      {
        if( ( BTnoRQT || width == height ) && gp_sizeIdxInfo->isCuSize( gp_sizeIdxInfo->sizeFrom( width ) ) && gp_sizeIdxInfo->isCuSize( gp_sizeIdxInfo->sizeFrom( height ) ) )
        {
          for( UInt layer = 0; layer < uiNumLayersToAllocateSplit; layer++ )
          {
            m_pSplitCS[width][height][layer]->destroy();

            delete m_pSplitCS[width][height][layer];
          }

          for( UInt layer = 0; layer < uiNumLayersToAllocateFull; layer++ )
          {
            m_pFullCS[width][height][layer]->destroy();

            delete m_pFullCS[width][height][layer];
          }

          delete[] m_pSplitCS[width][height];
          delete[] m_pFullCS [width][height];

          m_pBestCS[width][height]->destroy();
          m_pTempCS[width][height]->destroy();

          delete m_pTempCS[width][height];
          delete m_pBestCS[width][height];
        }
      }

      delete[] m_pSplitCS[width];
      delete[] m_pFullCS [width];

      delete[] m_pTempCS[width];
      delete[] m_pBestCS[width];
    }

    delete[] m_pSplitCS;
    delete[] m_pFullCS;

    delete[] m_pBestCS;
    delete[] m_pTempCS;

    delete[] m_pSaveCS;
  }

  m_pSplitCS = m_pFullCS = nullptr;

  m_pBestCS = m_pTempCS = nullptr;

  m_pSaveCS = nullptr;

  for( UInt ch = 0; ch < MAX_NUM_TBLOCKS; ch++ )
  {
    delete[] m_pSharedPredTransformSkip[ch];
    m_pSharedPredTransformSkip[ch] = nullptr;
  }

  m_isInitialized = false;
}

IntraSearch::~IntraSearch()
{
  if( m_isInitialized )
  {
    destroy();
  }
}

Void IntraSearch::init( EncCfg*        pcEncCfg,
                        TrQuant*       pcTrQuant,
                        RdCost*        pcRdCost,
                        CABACWriter*   CABACEstimator,
                        CtxCache*      ctxCache,
                        const UInt     maxCUWidth,
                        const UInt     maxCUHeight,
                        const UInt     maxTotalCUDepth
)
{
  CHECK(m_isInitialized, "Already initialized");
  m_pcEncCfg                     = pcEncCfg;
  m_pcTrQuant                    = pcTrQuant;
  m_pcRdCost                     = pcRdCost;
  m_CABACEstimator               = CABACEstimator;
  m_CtxCache                     = ctxCache;

  const ChromaFormat cform = pcEncCfg->getChromaFormatIdc();

  IntraPrediction::init( cform, pcEncCfg->getBitDepth( CHANNEL_TYPE_LUMA ) );

  for( UInt ch = 0; ch < MAX_NUM_TBLOCKS; ch++ )
  {
    m_pSharedPredTransformSkip[ch] = new Pel[MAX_CU_SIZE * MAX_CU_SIZE];
  }

  UInt numWidths  = gp_sizeIdxInfo->numWidths();
  UInt numHeights = gp_sizeIdxInfo->numHeights();

  bool BTnoRQT = m_pcEncCfg->getGenBinSplit() || m_pcEncCfg->getQTBT();

  const UInt uiNumLayersToAllocateSplit = BTnoRQT ? 1 : pcEncCfg->getQuadtreeTULog2MaxSize() - pcEncCfg->getQuadtreeTULog2MinSize() + 1;
  const UInt uiNumLayersToAllocateFull  = BTnoRQT ? 1 : pcEncCfg->getQuadtreeTULog2MaxSize() - pcEncCfg->getQuadtreeTULog2MinSize() + 1;

  m_pBestCS = new CodingStructure**[numWidths];
  m_pTempCS = new CodingStructure**[numWidths];

  m_pFullCS  = new CodingStructure***[numWidths];
  m_pSplitCS = new CodingStructure***[numWidths];

  for( UInt width = 0; width < numWidths; width++ )
  {
    m_pBestCS[width] = new CodingStructure*[numHeights];
    m_pTempCS[width] = new CodingStructure*[numHeights];

    m_pFullCS [width] = new CodingStructure**[numHeights];
    m_pSplitCS[width] = new CodingStructure**[numHeights];

    for( UInt height = 0; height < numHeights; height++ )
    {
      if( ( BTnoRQT || width == height ) && gp_sizeIdxInfo->isCuSize( gp_sizeIdxInfo->sizeFrom( width ) ) && gp_sizeIdxInfo->isCuSize( gp_sizeIdxInfo->sizeFrom( height ) ) )
      {
        m_pBestCS[width][height] = new CodingStructure( m_unitCache.cuCache, m_unitCache.puCache, m_unitCache.tuCache );
        m_pTempCS[width][height] = new CodingStructure( m_unitCache.cuCache, m_unitCache.puCache, m_unitCache.tuCache );

        m_pBestCS[width][height]->create( m_pcEncCfg->getChromaFormatIdc(), Area( 0, 0, gp_sizeIdxInfo->sizeFrom( width ), gp_sizeIdxInfo->sizeFrom( height ) ), false );
        m_pTempCS[width][height]->create( m_pcEncCfg->getChromaFormatIdc(), Area( 0, 0, gp_sizeIdxInfo->sizeFrom( width ), gp_sizeIdxInfo->sizeFrom( height ) ), false );
        m_pFullCS [width][height] = new CodingStructure*[uiNumLayersToAllocateFull];
        m_pSplitCS[width][height] = new CodingStructure*[uiNumLayersToAllocateSplit];

        for( UInt layer = 0; layer < uiNumLayersToAllocateFull; layer++ )
        {
          m_pFullCS [width][height][layer] = new CodingStructure( m_unitCache.cuCache, m_unitCache.puCache, m_unitCache.tuCache );

          m_pFullCS [width][height][layer]->create( m_pcEncCfg->getChromaFormatIdc(), Area( 0, 0, gp_sizeIdxInfo->sizeFrom( width ), gp_sizeIdxInfo->sizeFrom( height ) ), false );
        }

        for( UInt layer = 0; layer < uiNumLayersToAllocateSplit; layer++ )
        {
          m_pSplitCS[width][height][layer] = new CodingStructure( m_unitCache.cuCache, m_unitCache.puCache, m_unitCache.tuCache );

          m_pSplitCS[width][height][layer]->create( m_pcEncCfg->getChromaFormatIdc(), Area( 0, 0, gp_sizeIdxInfo->sizeFrom( width ), gp_sizeIdxInfo->sizeFrom( height ) ), false );
        }
      }
      else
      {
        m_pBestCS[width][height] = nullptr;
        m_pTempCS[width][height] = nullptr;

        m_pFullCS [width][height] = nullptr;
        m_pSplitCS[width][height] = nullptr;
      }
    }
  }

  const int uiNumSaveLayersToAllocate = 2;

  m_pSaveCS = new CodingStructure*[uiNumSaveLayersToAllocate];

  for( UInt depth = 0; depth < uiNumSaveLayersToAllocate; depth++ )
  {
    m_pSaveCS[depth] = new CodingStructure( m_unitCache.cuCache, m_unitCache.puCache, m_unitCache.tuCache );
    m_pSaveCS[depth]->create( UnitArea( cform, Area( 0, 0, maxCUWidth, maxCUHeight ) ), false );
  }

  m_isInitialized = true;
}


//////////////////////////////////////////////////////////////////////////
// INTRA PREDICTION
//////////////////////////////////////////////////////////////////////////

Void IntraSearch::estIntraPredLumaQT( CodingUnit &cu, Partitioner &partitioner )
{
  CodingStructure       &cs            = *cu.cs;
  const SPS             &sps           = *cs.sps;
  const UInt             uiWidthBit    = cs.pcv->rectCUs ? g_aucLog2[partitioner.currArea().lwidth() ] : CU::getIntraSizeIdx(cu);
  const UInt             uiHeightBit   =                   g_aucLog2[partitioner.currArea().lheight()];

  // Lambda calculation at equivalent Qp of 4 is recommended because at that Qp, the quantization divisor is 1.
  const double sqrtLambdaForFirstPass = m_pcRdCost->getMotionLambda(cu.transQuantBypass) / double(1 << SCALE_BITS);


  //===== loop over partitions =====

  const TempCtx ctxStart          ( m_CtxCache, m_CABACEstimator->getCtx() );
  const TempCtx ctxStartIntraMode ( m_CtxCache, SubCtx( Ctx::IPredMode[CHANNEL_TYPE_LUMA],        m_CABACEstimator->getCtx() ) );

  CHECK( !cu.firstPU, "CU has no PUs" );
  const bool keepResi   = cs.pps->getPpsRangeExtension().getCrossComponentPredictionEnabledFlag() || KEEP_PRED_AND_RESI_SIGNALS;


  UInt extraModes = 0; // add two extra modes, which would be used after uiMode <= DC_IDX is removed for cu.nsstIdx == 3





  static_vector<UInt,   FAST_UDI_MAX_RDMODE_NUM> uiHadModeList;
  static_vector<Double, FAST_UDI_MAX_RDMODE_NUM> CandCostList;
  static_vector<Double, FAST_UDI_MAX_RDMODE_NUM> CandHadList;

  auto &pu = *cu.firstPU;

  {
    CandHadList.clear();
    CandCostList.clear();
    uiHadModeList.clear();

    CHECK(pu.cu != &cu, "PU is not contained in the CU");

    //===== determine set of modes to be tested (using prediction signal only) =====
    Int numModesAvailable = NUM_LUMA_MODE; // total number of Intra modes
    static_vector< UInt, FAST_UDI_MAX_RDMODE_NUM > uiRdModeList;

    Int numModesForFullRD = 3;
    if( sps.getSpsNext().getUseGenBinSplit() && isNonLog2BlockSize( pu.Y() ) )
    {
      if( std::max( pu.lwidth(), pu.lheight() ) / std::min( pu.lheight(), pu.lwidth() ) >= 2 )
      {
        numModesForFullRD = 2;
      }
      else
      {
        numModesForFullRD = 3;
      }
    }
    else if( cs.pcv->rectCUs )
    {
      numModesForFullRD = g_aucIntraModeNumFast_UseMPM_2D[uiWidthBit - MIN_CU_LOG2][uiHeightBit - MIN_CU_LOG2];
    }
    else
    {
      numModesForFullRD = m_pcEncCfg->getFastUDIUseMPMEnabled() ? g_aucIntraModeNumFast_UseMPM[uiWidthBit] : g_aucIntraModeNumFast_NotUseMPM[uiWidthBit];
    }

#if INTRA_FULL_SEARCH
    numModesForFullRD = numModesAvailable;
#endif


    {
      // this should always be true
      CHECK( !pu.Y().valid(), "PU is not valid" );

      //===== init pattern for luma prediction =====
      initIntraPatternChType( cu, pu.Y(), IntraPrediction::useFilteredIntraRefSamples( COMPONENT_Y, pu, false, pu ) );

      if( numModesForFullRD != numModesAvailable )
      {
        CHECK( numModesForFullRD >= numModesAvailable, "Too many modes for full RD search" );

        const CompArea &area = pu.Y();

        PelBuf piOrg         = cs.getOrgBuf(area);
        PelBuf piPred        = cs.getPredBuf(area);

        DistParam distParam;

        const Bool bUseHadamard = cu.transQuantBypass == 0;

        m_pcRdCost->setDistParam(distParam, piOrg, piPred, sps.getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y, bUseHadamard);

        distParam.applyWeight = false;

        bool bSatdChecked[NUM_INTRA_MODE];
        memset( bSatdChecked, 0, sizeof( bSatdChecked ) );

        {
          for( Int modeIdx = 0; modeIdx < numModesAvailable; modeIdx++ )
          {
            UInt       uiMode = modeIdx;
            Distortion uiSad  = 0;

            // Skip checking extended Angular modes in the first round of SATD
            if( uiMode > DC_IDX && ( uiMode & 1 ) )
            {
              continue;
            }

            bSatdChecked[uiMode] = true;

            pu.intraDir[0] = modeIdx;

            if( useDPCMForFirstPassIntraEstimation( pu, uiMode ) )
            {
              encPredIntraDPCM( COMPONENT_Y, piOrg, piPred, uiMode );
            }
            else
            {
              predIntraAng( COMPONENT_Y, piPred, pu, IntraPrediction::useFilteredIntraRefSamples( COMPONENT_Y, pu, true, pu ) );
            }
            // use Hadamard transform here
            uiSad += distParam.distFunc(distParam);

            // NB xFracModeBitsIntra will not affect the mode for chroma that may have already been pre-estimated.
            m_CABACEstimator->getCtx() = SubCtx( Ctx::IPredMode[CHANNEL_TYPE_LUMA], ctxStartIntraMode );

            UInt64 fracModeBits = xFracModeBitsIntra(pu, uiMode, CHANNEL_TYPE_LUMA);

            Double cost = ( Double ) uiSad + ( Double ) fracModeBits * sqrtLambdaForFirstPass;

            DTRACE( g_trace_ctx, D_INTRA_COST, "IntraHAD: %u, %llu, %f (%d)\n", uiSad, fracModeBits, cost, uiMode );

            updateCandList( uiMode, cost,  uiRdModeList, CandCostList, numModesForFullRD + extraModes );
            updateCandList( uiMode, uiSad, uiHadModeList, CandHadList, 3                 + extraModes );
          }
        } // NSSTFlag

        // forget the extra modes
        uiRdModeList.resize( numModesForFullRD );

        if( m_pcEncCfg->getFastUDIUseMPMEnabled() )
        {
          unsigned  numMPMs = pu.cs->pcv->numMPMs;
          unsigned *uiPreds = ( unsigned* ) alloca( numMPMs * sizeof( unsigned ) );

          const Int numCand = PU::getIntraMPMs( pu, uiPreds );

          for( Int j = 0; j < numCand; j++ )
          {
            Bool mostProbableModeIncluded = false;
            Int  mostProbableMode         = uiPreds[j];


            for( Int i = 0; i < numModesForFullRD; i++ )
            {
              mostProbableModeIncluded |= ( mostProbableMode == uiRdModeList[i] );
            }
            if( !mostProbableModeIncluded )
            {
              numModesForFullRD++;
              uiRdModeList.push_back( mostProbableMode );
            }
          }
        }
      }
      else
      {
        for( Int i = 0; i < numModesForFullRD; i++ )
        {
          uiRdModeList.push_back( i );
        }
      }
    }


    CHECK( numModesForFullRD != uiRdModeList.size(), "Inconsistent state!" );

    // after this point, don't use numModesForFullRD

    // PBINTRA fast
    if( m_pcEncCfg->getUsePbIntraFast() && !cs.slice->isIntra() && cu.partSize == SIZE_2Nx2N && uiRdModeList.size() < numModesAvailable )
    {
      if( CandHadList.size() < 3 || CandHadList[2] > cs.interHad * PBINTRA_RATIO )
      {
        uiRdModeList.resize( std::min<size_t>( uiRdModeList.size(), 2 ) );
      }
      if( CandHadList.size() < 2 || CandHadList[1] > cs.interHad * PBINTRA_RATIO )
      {
        uiRdModeList.resize( std::min<size_t>( uiRdModeList.size(), 1 ) );
      }
      if( CandHadList.size() < 1 || CandHadList[0] > cs.interHad * PBINTRA_RATIO )
      {
        cs.dist     = MAX_UINT;
        cs.interHad = 0;

        //===== reset context models =====
        m_CABACEstimator->getCtx() = SubCtx( Ctx::IPredMode       [CHANNEL_TYPE_LUMA], ctxStartIntraMode );

        return;
      }
    }

    //===== check modes (using r-d costs) =====
    UInt       uiBestPUMode  = 0;

    CodingStructure *csTemp = m_pTempCS[gp_sizeIdxInfo->idxFrom( cu.lwidth() )][gp_sizeIdxInfo->idxFrom( cu.lheight() )];
    CodingStructure *csBest = m_pBestCS[gp_sizeIdxInfo->idxFrom( cu.lwidth() )][gp_sizeIdxInfo->idxFrom( cu.lheight() )];

    csTemp->slice = cs.slice;
    csBest->slice = cs.slice;
    csTemp->initStructData();
    csBest->initStructData();

    // just to be sure
    numModesForFullRD = ( int ) uiRdModeList.size();
    for (UInt uiMode = 0; uiMode < numModesForFullRD; uiMode++)
    {
      // set luma prediction mode
      UInt uiOrgMode = uiRdModeList[uiMode];

      pu.intraDir[0] = uiOrgMode;


      // set context models
      m_CABACEstimator->getCtx() = ctxStart;

      // determine residual for partition
      cs.initSubStructure( *csTemp, partitioner.chType, cs.area, true );

      xRecurIntraCodingLumaQT( *csTemp, partitioner );



      DTRACE( g_trace_ctx, D_INTRA_COST, "IntraCost T %f (%d) \n", csTemp->cost, uiOrgMode );

      // check r-d cost
      if( csTemp->cost < csBest->cost )
      {
        std::swap( csTemp, csBest );


        uiBestPUMode  = uiOrgMode;

      }

      csTemp->releaseIntermediateData();
    } // Mode loop

    cs.useSubStructure( *csBest, partitioner.chType, pu.singleChan( CHANNEL_TYPE_LUMA ), KEEP_PRED_AND_RESI_SIGNALS, true, keepResi, keepResi );

    csBest->releaseIntermediateData();
    //=== update PU data ====
    pu.intraDir[0] = uiBestPUMode;
  }

  //===== reset context models =====
  m_CABACEstimator->getCtx() = ctxStart;
}

Void IntraSearch::estIntraPredChromaQT(CodingUnit &cu, Partitioner &partitioner)
{
  const ChromaFormat format   = cu.chromaFormat;
  const UInt    numberValidComponents = getNumberValidComponents(format);
  CodingStructure &cs = *cu.cs;
  const TempCtx ctxStart  ( m_CtxCache, m_CABACEstimator->getCtx() );

  cs.setDecomp( cs.area.Cb(), false );

  auto &pu = *cu.firstPU;

  {
    UInt       uiBestMode = 0;
    Distortion uiBestDist = 0;
    Double     dBestCost = MAX_DOUBLE;

    //----- init mode list ----
    {
      UInt  uiMinMode = 0;
      UInt  uiMaxMode = NUM_CHROMA_MODE;

      //----- check chroma modes -----
      UInt chromaCandModes[ NUM_CHROMA_MODE ];
      PU::getIntraChromaCandModes( pu, chromaCandModes );

      // create a temporary CS
      CodingStructure &saveCS = *m_pSaveCS[0];
      saveCS.pcv      = cs.pcv;
      saveCS.picture  = cs.picture;
      saveCS.area.repositionTo( cs.area );
      saveCS.clearTUs();

      if( CS::isDualITree( cs ) )
      {
        cs.addTU( CS::getArea( cs, partitioner.currArea(), partitioner.chType ), partitioner.chType );
      }

      std::vector<TransformUnit*> orgTUs;


      // create a store for the TUs
      for( const auto &ptu : cs.tus )
      {
        // for split TUs in HEVC, add the TUs without Chroma parts for correct setting of Cbfs
        if( pu.contains( *ptu, CHANNEL_TYPE_CHROMA ) || ( !cs.pcv->noRQT && !ptu->Cb().valid() && !ptu->Cr().valid() ) )
        {
          saveCS.addTU( *ptu, partitioner.chType );
          orgTUs.push_back( ptu );
        }
      }

      // save the dist
      Distortion baseDist = cs.dist;

      for (UInt uiMode = uiMinMode; uiMode < uiMaxMode; uiMode++)
      {
        const int chromaIntraMode = chromaCandModes[uiMode];

        cs.setDecomp( pu.Cb(), false );
        cs.dist = baseDist;
        //----- restore context models -----
        m_CABACEstimator->getCtx() = ctxStart;

        //----- chroma coding -----
        pu.intraDir[1] = chromaIntraMode;

        xRecurIntraChromaCodingQT( cs, partitioner );

        if (cs.pps->getUseTransformSkip())
        {
          m_CABACEstimator->getCtx() = ctxStart;
        }

        UInt64 fracBits   = xGetIntraFracBitsQT( cs, partitioner, false, true );
        Distortion uiDist = cs.dist;
        Double    dCost   = m_pcRdCost->calcRdCost( fracBits, uiDist - baseDist );

        //----- compare -----
        if( dCost < dBestCost )
        {
          for( UInt i = getFirstComponentOfChannel( CHANNEL_TYPE_CHROMA ); i < numberValidComponents; i++ )
          {
            const CompArea &area = pu.blocks[i];

            saveCS.getRecoBuf     ( area ).copyFrom( cs.getRecoBuf   ( area ) );
#if KEEP_PRED_AND_RESI_SIGNALS
            saveCS.getPredBuf     ( area ).copyFrom( cs.getPredBuf   ( area ) );
            saveCS.getResiBuf     ( area ).copyFrom( cs.getResiBuf   ( area ) );
#endif
            cs.picture->getRecoBuf( area ).copyFrom( cs.getRecoBuf( area ) );

            for( UInt j = 0; j < saveCS.tus.size(); j++ )
            {
              saveCS.tus[j]->copyComponentFrom( *orgTUs[j], area.compID );
            }
          }

          dBestCost  = dCost;
          uiBestDist = uiDist;
          uiBestMode = chromaIntraMode;
        }
      }

      for( UInt i = getFirstComponentOfChannel( CHANNEL_TYPE_CHROMA ); i < numberValidComponents; i++ )
      {
        const CompArea &area = pu.blocks[i];

        cs.getRecoBuf         ( area ).copyFrom( saveCS.getRecoBuf( area ) );
#if KEEP_PRED_AND_RESI_SIGNALS
        cs.getPredBuf         ( area ).copyFrom( saveCS.getPredBuf( area ) );
        cs.getResiBuf         ( area ).copyFrom( saveCS.getResiBuf( area ) );
#endif
        cs.picture->getRecoBuf( area ).copyFrom( cs.    getRecoBuf( area ) );

        for( UInt j = 0; j < saveCS.tus.size(); j++ )
        {
          orgTUs[ j ]->copyComponentFrom( *saveCS.tus[ j ], area.compID );
        }
      }
    }

    pu.intraDir[1] = uiBestMode;
    cs.dist        = uiBestDist;
  }

  //----- restore context models -----
  m_CABACEstimator->getCtx() = ctxStart;
}

Void IntraSearch::IPCMSearch(CodingStructure &cs, Partitioner& partitioner)
{
  for (UInt ch = 0; ch < getNumberValidTBlocks( *cs.pcv ); ch++)
  {
    const ComponentID compID = ComponentID(ch);

    xEncPCM(cs, partitioner, compID);
  }

  cs.getPredBuf().fill(0);
  cs.getResiBuf().fill(0);
  cs.getOrgResiBuf().fill(0);

  cs.dist     = 0;
  cs.fracBits = 0;
  cs.cost     = 0;

  cs.setDecomp(cs.area);
  cs.picture->getRecoBuf(cs.area).copyFrom(cs.getRecoBuf());
}

Void IntraSearch::xEncPCM(CodingStructure &cs, Partitioner& partitioner, const ComponentID &compID)
{
  TransformUnit &tu = *cs.getTU( partitioner.chType );

  const Int  channelBitDepth = cs.sps->getBitDepth(toChannelType(compID));
  const UInt uiPCMBitDepth = cs.sps->getPCMBitDepth(toChannelType(compID));

  const Int pcmShiftRight = (channelBitDepth - Int(uiPCMBitDepth));

  CompArea  area    = tu.blocks[compID];
  PelBuf    pcmBuf  = tu.getPcmbuf  (compID);
  PelBuf    recBuf  = cs.getRecoBuf ( area );
  CPelBuf   orgBuf  = cs.getOrgBuf  ( area );

  CHECK(pcmShiftRight < 0, "Negative shift");

  for (UInt uiY = 0; uiY < pcmBuf.height; uiY++)
  {
    for (UInt uiX = 0; uiX < pcmBuf.width; uiX++)
    {
      // Encode
      pcmBuf.at(uiX, uiY) = orgBuf.at(uiX, uiY) >> pcmShiftRight;
      // Reconstruction
      recBuf.at(uiX, uiY) = pcmBuf.at(uiX, uiY) << pcmShiftRight;
    }
  }
}

// -------------------------------------------------------------------------------------------------------------------
// Intra search
// -------------------------------------------------------------------------------------------------------------------

Void IntraSearch::xEncIntraHeader(CodingStructure &cs, Partitioner &partitioner, const Bool &bLuma, const Bool &bChroma)
{
  CodingUnit &cu = *cs.getCU( partitioner.chType );

  if (bLuma)
  {
    Bool isFirst = partitioner.currArea().lumaPos() == cs.area.lumaPos();

    // CU header
    if( isFirst )
    {
      if( !cs.slice->isIntra() )
      {
        if( cs.pps->getTransquantBypassEnabledFlag() )
        {
          m_CABACEstimator->cu_transquant_bypass_flag( cu );
        }
        m_CABACEstimator->cu_skip_flag( cu );
        m_CABACEstimator->pred_mode   ( cu );
      }
      if( CU::isIntra(cu) && cu.partSize == SIZE_2Nx2N )
      {
        m_CABACEstimator->pcm_data( cu );
        if( cu.ipcm )
        {
          return;
        }
      }
    }

    PredictionUnit &pu = *cs.getPU(partitioner.currArea().lumaPos(), partitioner.chType);

    // luma prediction mode
    if (cu.partSize == SIZE_2Nx2N)
    {
      if (isFirst)
      {
        m_CABACEstimator->intra_luma_pred_mode( pu );
      }
    }
  }

  if (bChroma)
  {
    Bool isFirst = partitioner.currArea().Cb().valid() && partitioner.currArea().chromaPos() == cs.area.chromaPos();

    PredictionUnit &pu = *cs.getPU( partitioner.currArea().chromaPos(), CHANNEL_TYPE_CHROMA );

    if( cu.partSize == SIZE_2Nx2N )
    {
      if( isFirst )
      {
        m_CABACEstimator->intra_chroma_pred_mode( pu );
      }
    }
  }
}

Void IntraSearch::xEncSubdivCbfQT(CodingStructure &cs, Partitioner &partitioner, const Bool &bLuma, const Bool &bChroma)
{
  const UnitArea &currArea  = partitioner.currArea();
  TransformUnit &currTU = *cs.getTU( currArea.blocks[partitioner.chType], partitioner.chType );

  if (bChroma)
  {
    const UInt numberValidComponents = getNumberValidComponents(currArea.chromaFormat);

    for (UInt ch = COMPONENT_Cb; ch < numberValidComponents; ch++)
    {
      const ComponentID compID = ComponentID(ch);

      m_CABACEstimator->cbf_comp( cs, TU::getCbf( currTU, compID ), currArea.blocks[compID] );
    }
  }

  {

    //===== Cbfs =====
    if (bLuma)
    {
      m_CABACEstimator->cbf_comp( cs, TU::getCbf( currTU, COMPONENT_Y), currTU.Y() );
    }
  }
}

Void IntraSearch::xEncCoeffQT(CodingStructure &cs, Partitioner &partitioner, const ComponentID &compID)
{
  const UnitArea &currArea  = partitioner.currArea();
  TransformUnit &currTU     = *cs.getTU( currArea.blocks[partitioner.chType], partitioner.chType );

  if( currArea.blocks[compID].valid() )
  {
    if( TU::hasCrossCompPredInfo( currTU, compID ) )
    {
      m_CABACEstimator->cross_comp_pred( currTU, compID );
    }
    if( TU::getCbf( currTU, compID ) )
    {
      m_CABACEstimator->residual_coding( currTU, compID );
    }
  }
}

UInt64 IntraSearch::xGetIntraFracBitsQT( CodingStructure &cs, Partitioner &partitioner, const Bool &bLuma, const Bool &bChroma )
{
  m_CABACEstimator->resetBits();

  xEncIntraHeader( cs, partitioner, bLuma, bChroma );
  xEncSubdivCbfQT( cs, partitioner, bLuma, bChroma );

  if( bLuma )
  {
    xEncCoeffQT( cs, partitioner, COMPONENT_Y );
  }
  if( bChroma )
  {
    xEncCoeffQT( cs, partitioner, COMPONENT_Cb );
    xEncCoeffQT( cs, partitioner, COMPONENT_Cr );
  }


  UInt64 fracBits = m_CABACEstimator->getEstFracBits();
  return fracBits;
}

UInt64 IntraSearch::xGetIntraFracBitsQTChroma(TransformUnit& currTU, const ComponentID &compID)
{
  m_CABACEstimator->resetBits();

  if( TU::hasCrossCompPredInfo( currTU, compID ) )
  {
    m_CABACEstimator->cross_comp_pred( currTU, compID );
  }
  if( TU::getCbf( currTU, compID ) )
  {
    m_CABACEstimator->residual_coding( currTU, compID );
  }

  UInt64 fracBits = m_CABACEstimator->getEstFracBits();
  return fracBits;
}

Void IntraSearch::xIntraCodingTUBlock(TransformUnit &tu, const ComponentID &compID, const Bool &checkCrossCPrediction, Distortion& ruiDist, const Int &default0Save1Load2, UInt* numSig )
{
  if (!tu.blocks[compID].valid())
  {
    return;
  }

  CodingStructure &cs                       = *tu.cs;

  const CompArea      &area                 = tu.blocks[compID];
  const SPS           &sps                  = *cs.sps;
  const PPS           &pps                  = *cs.pps;

  const ChannelType    chType               = toChannelType(compID);
  const Int            bitDepth             = sps.getBitDepth(chType);

  PelBuf         piOrg                      = cs.getOrgBuf    (area);
  PelBuf         piPred                     = cs.getPredBuf   (area);
  PelBuf         piResi                     = cs.getResiBuf   (area);
  PelBuf         piOrgResi                  = cs.getOrgResiBuf(area);
  PelBuf         piReco                     = cs.getRecoBuf   (area);


  const PredictionUnit &pu                  = *cs.getPU(area.pos(), chType);
#if ENABLE_TRACING
  const UInt           uiChFinalMode        = PU::getFinalIntraMode(pu, chType);

#endif
  const Bool           bUseCrossCPrediction = pps.getPpsRangeExtension().getCrossComponentPredictionEnabledFlag() && isChroma( compID ) && PU::isChromaIntraModeCrossCheckMode( pu ) && checkCrossCPrediction;
  const Bool           ccUseRecoResi        = m_pcEncCfg->getUseReconBasedCrossCPredictionEstimate();


  //===== init availability pattern =====
  PelBuf sharedPredTS( m_pSharedPredTransformSkip[compID], area );
  if( default0Save1Load2 != 2 )
  {
    const bool bUseFilteredPredictions = IntraPrediction::useFilteredIntraRefSamples( compID, pu, true, tu );
    initIntraPatternChType( *tu.cu, area, bUseFilteredPredictions );

    //===== get prediction signal =====
    {
      predIntraAng( compID, piPred, pu, bUseFilteredPredictions );
    }


    // save prediction
    if( default0Save1Load2 == 1 )
    {
      sharedPredTS.copyFrom( piPred );
    }
  }
  else
  {
    // load prediction
    piPred.copyFrom( sharedPredTS );
  }


  DTRACE( g_trace_ctx, D_PRED, "@(%4d,%4d) [%2dx%2d] IMode=%d\n", tu.lx(), tu.ly(), tu.lwidth(), tu.lheight(), uiChFinalMode );
  //DTRACE_PEL_BUF( D_PRED, piPred, tu, tu.cu->predMode, COMPONENT_Y );

  //===== get residual signal =====
  piResi.copyFrom( piOrg  );
  piResi.subtract( piPred );

  if (pps.getPpsRangeExtension().getCrossComponentPredictionEnabledFlag() && isLuma(compID))
  {
    piOrgResi.copyFrom (piResi);
  }

  if (bUseCrossCPrediction)
  {
    if (xCalcCrossComponentPredictionAlpha(tu, compID, ccUseRecoResi) == 0)
    {
      return;
    }
    CrossComponentPrediction::crossComponentPrediction(tu, compID, cs.getResiBuf(tu.Y()), piResi, piResi, false);
  }

  //===== transform and quantization =====
  //--- init rate estimation arrays for RDOQ ---
  //--- transform and quantization           ---
  TCoeff uiAbsSum = 0;

  const QpParam cQP(tu, compID);

#if RDOQ_CHROMA_LAMBDA
  m_pcTrQuant->selectLambda(compID);
#endif

  m_pcTrQuant->transformNxN(tu, compID, cQP, uiAbsSum, m_CABACEstimator->getCtx());


  DTRACE( g_trace_ctx, D_TU_ABS_SUM, "%d: comp=%d, abssum=%d\n", DTRACE_GET_COUNTER( g_trace_ctx, D_TU_ABS_SUM ), compID, uiAbsSum );


  //--- inverse transform ---
  if (uiAbsSum > 0)
  {
    m_pcTrQuant->invTransformNxN(tu, compID, piResi, cQP);
  }
  else
  {
    piResi.fill(0);
  }

  //===== reconstruction =====
  if (bUseCrossCPrediction)
  {
    CrossComponentPrediction::crossComponentPrediction(tu, compID, cs.getResiBuf(tu.Y()), piResi, piResi, true);
  }

  piReco.reconstruct(piPred, piResi, cs.slice->clpRng( compID ));

  //===== update distortion =====
  ruiDist += m_pcRdCost->getDistPart(piOrg, piReco, bitDepth, compID);
}

Void IntraSearch::xRecurIntraCodingLumaQT( CodingStructure &cs, Partitioner &partitioner )
{
  const UnitArea &currArea = partitioner.currArea();
  const CodingUnit &cu     = *cs.getCU(currArea.lumaPos(), partitioner.chType);
  const PPS &pps           = *cs.pps;
  const bool keepResi      = pps.getPpsRangeExtension().getCrossComponentPredictionEnabledFlag() || KEEP_PRED_AND_RESI_SIGNALS;
  Bool bCheckFull          = true;
  Bool bCheckSplit         = false;

  UInt    numSig           = 0;

  if( !cs.pcv->noRQT )
  {
  }

  Bool    checkInitTrDepth = false, checkInitTrDepthTransformSkipWinner = false;

  Double     dSingleCost                        = MAX_DOUBLE;
  Distortion uiSingleDistLuma                   = 0;
  UInt64     singleFracBits                     = 0;
  Bool       checkTransformSkip                 = pps.getUseTransformSkip();
  Int        bestModeId[MAX_NUM_COMPONENT]      = {0, 0, 0};
  UChar      nNumTransformCands                 = 1;

  UChar numTransformIndexCands                  = nNumTransformCands;

  const TempCtx ctxStart  ( m_CtxCache, m_CABACEstimator->getCtx() );
  TempCtx       ctxBest   ( m_CtxCache );

  CodingStructure *csFull  = nullptr;

  {
    csFull = &cs;
  }

  if( bCheckFull )
  {
    csFull->cost = 0.0;

    TransformUnit &tu = csFull->addTU( CS::getArea( *csFull, currArea, partitioner.chType ), partitioner.chType );

    checkTransformSkip &= TU::hasTransformSkipFlag( *tu.cs, tu.Y() );
    checkTransformSkip &= !cu.transQuantBypass;

    CHECK( !tu.Y().valid(), "Invalid TU" );

    //this prevents transformSkip from being checked because we already know it's not the best mode
    checkTransformSkip = ( checkInitTrDepth && !checkInitTrDepthTransformSkipWinner ) ? false : checkTransformSkip;


    CHECK( checkInitTrDepthTransformSkipWinner && !checkTransformSkip, "Transform Skip must be enabled if it was the winner in the previous call of xRecurIntraCodingLumaQT!" );

    CodingStructure &saveCS = *m_pSaveCS[0];

    TransformUnit *tmpTU = nullptr;

    Distortion singleDistTmpLuma = 0;
    UInt64     singleTmpFracBits = 0;
    Double     singleCostTmp     = 0;
    Int        firstCheckId      = 0;

    //we add the EMT candidates to the loop. TransformSkip will still be the last one to be checked (when modeId == lastCheckId) as long as checkTransformSkip is true
    Int        lastCheckId       = numTransformIndexCands - ( firstCheckId + 1 ) + ( int ) checkTransformSkip;
    bool isNotOnlyOneMode        = lastCheckId != firstCheckId && !checkInitTrDepthTransformSkipWinner;

    if( isNotOnlyOneMode )
    {
      saveCS.pcv     = cs.pcv;
      saveCS.picture = cs.picture;
      saveCS.area.repositionTo(cs.area);
      saveCS.clearTUs();
      tmpTU = &saveCS.addTU(currArea, partitioner.chType);
    }



    for( Int modeId = firstCheckId; modeId <= lastCheckId; modeId++ )
    {
      if( checkInitTrDepthTransformSkipWinner )
      {
        //If this is a full RQT call and the winner of the first call (checkFirst=true) was transformSkip, then we skip the first iteration of the loop, since transform skip always comes at the end
        if( modeId == firstCheckId )
        {
          continue;
        }
      }




      if ((modeId != firstCheckId) && isNotOnlyOneMode)
      {
        m_CABACEstimator->getCtx() = ctxStart;
      }

      Int default0Save1Load2 = 0;
      singleDistTmpLuma = 0;

      if (modeId == firstCheckId && modeId != lastCheckId && !checkInitTrDepthTransformSkipWinner )
      {
        default0Save1Load2 = 1;
      }
      else if (modeId != firstCheckId)
      {
        default0Save1Load2 = 2;
      }

      if( !checkTransformSkip )
      {
        tu.transformSkip[COMPONENT_Y] = false;
      }
      else
      {
        tu.transformSkip[COMPONENT_Y] = modeId == lastCheckId;
      }

      xIntraCodingTUBlock( tu, COMPONENT_Y, false, singleDistTmpLuma, default0Save1Load2, &numSig );

      //----- determine rate and r-d cost -----
      //the condition (transformIndex != DCT2_EMT) seems to be irrelevant, since DCT2_EMT=7 and the highest value of transformIndex is 4
      if( ( modeId == lastCheckId && checkTransformSkip && !TU::getCbf( tu, COMPONENT_Y ) ) )
      {
        //In order not to code TS flag when cbf is zero, the case for TS with cbf being zero is forbidden.
        singleCostTmp = MAX_DOUBLE;
      }
      else
      {
        singleTmpFracBits = xGetIntraFracBitsQT( *csFull, partitioner, true, false );
        singleCostTmp     = m_pcRdCost->calcRdCost( singleTmpFracBits, singleDistTmpLuma );
      }

      if (singleCostTmp < dSingleCost)
      {
        dSingleCost       = singleCostTmp;
        uiSingleDistLuma  = singleDistTmpLuma;
        singleFracBits    = singleTmpFracBits;

        bestModeId[COMPONENT_Y] = modeId;


        if( bestModeId[COMPONENT_Y] != lastCheckId )
        {
#if KEEP_PRED_AND_RESI_SIGNALS
          saveCS.getPredBuf( tu.Y() ).copyFrom( csFull->getPredBuf( tu.Y() ) );
#endif
          saveCS.getRecoBuf( tu.Y() ).copyFrom( csFull->getRecoBuf( tu.Y() ) );

          if( keepResi )
          {
            saveCS.getResiBuf   ( tu.Y() ).copyFrom( csFull->getResiBuf   ( tu.Y() ) );
            saveCS.getOrgResiBuf( tu.Y() ).copyFrom( csFull->getOrgResiBuf( tu.Y() ) );
          }

          tmpTU->copyComponentFrom( tu, COMPONENT_Y );

          ctxBest = m_CABACEstimator->getCtx();
        }
      }
    }

    if( bestModeId[COMPONENT_Y] != lastCheckId )
    {
#if KEEP_PRED_AND_RESI_SIGNALS
      csFull->getPredBuf( tu.Y() ).copyFrom( saveCS.getPredBuf( tu.Y() ) );
#endif
      csFull->getRecoBuf( tu.Y() ).copyFrom( saveCS.getRecoBuf( tu.Y() ) );

      if( keepResi )
      {
        csFull->getResiBuf   ( tu.Y() ).copyFrom( saveCS.getResiBuf   ( tu.Y() ) );
        csFull->getOrgResiBuf( tu.Y() ).copyFrom( saveCS.getOrgResiBuf( tu.Y() ) );
      }

      tu.copyComponentFrom( *tmpTU, COMPONENT_Y );

      if( !bCheckSplit )
      {
        m_CABACEstimator->getCtx() = ctxBest;
      }
    }
    else if( bCheckSplit )
    {
      ctxBest = m_CABACEstimator->getCtx();
    }

    csFull->cost     += dSingleCost;
    csFull->dist     += uiSingleDistLuma;
    csFull->fracBits += singleFracBits;
  }

  {
    {
      // otherwise this would've happened in useSubStructure
      cs.picture->getRecoBuf( currArea.Y() ).copyFrom( cs.getRecoBuf( currArea.Y() ) );
    }

    cs.cost = m_pcRdCost->calcRdCost( cs.fracBits, cs.dist );
  }
}

ChromaCbfs IntraSearch::xRecurIntraChromaCodingQT(CodingStructure &cs, Partitioner& partitioner)
{
  UnitArea currArea                   = partitioner.currArea();
  const bool keepResi                 = KEEP_PRED_AND_RESI_SIGNALS;
  if( !currArea.Cb().valid() ) return ChromaCbfs( false );


  TransformUnit &currTU               = *cs.getTU( currArea.chromaPos(), CHANNEL_TYPE_CHROMA );
  const PredictionUnit &pu            = *cs.getPU( currArea.chromaPos(), CHANNEL_TYPE_CHROMA );
  const TransformUnit &currTULuma     = CS::isDualITree( cs ) ? *cs.picture->cs->getTU( currArea.lumaPos(), CHANNEL_TYPE_LUMA ) : currTU;

  const PPS &pps                      = *cs.pps;
  ChromaCbfs cbfs                     ( false );

  {
    if (!currArea.Cb().valid() || !currArea.Cr().valid())
    {
      return cbfs;
    }

    Bool checkTransformSkip = pps.getUseTransformSkip();
    checkTransformSkip &= TU::hasTransformSkipFlag( *currTU.cs, partitioner.currArea().Cb() );

    if( m_pcEncCfg->getUseTransformSkipFast() )
    {
      checkTransformSkip &= TU::hasTransformSkipFlag( *currTU.cs, partitioner.currArea().Y() );

      if( checkTransformSkip && cs.pcv->noChroma2x2 )
      {
        Int nbLumaSkip = currTULuma.transformSkip[0] ? 1 : 0;

        {
          // the chroma blocks are co-located with the last luma block, so backwards references are needed
          nbLumaSkip += cs.getTU( currTULuma.Y().topLeft().offset( -1,  0 ), partitioner.chType )->transformSkip[0] ? 1 : 0;
          nbLumaSkip += cs.getTU( currTULuma.Y().topLeft().offset( -1, -1 ), partitioner.chType )->transformSkip[0] ? 1 : 0;
          nbLumaSkip += cs.getTU( currTULuma.Y().topLeft().offset(  0, -1 ), partitioner.chType )->transformSkip[0] ? 1 : 0;
        }

        checkTransformSkip &= ( nbLumaSkip > 0 );
      }
    }

    CodingStructure &saveCS = *m_pSaveCS[1];
    saveCS.pcv      = cs.pcv;
    saveCS.picture  = cs.picture;
    saveCS.area.repositionTo(cs.area);
    saveCS.initStructData();

    TransformUnit &tmpTU = saveCS.addTU(currArea, partitioner.chType);


    cs.setDecomp(currArea.Cb(), true); // set in advance (required for Cb2/Cr2 in 4:2:2 video)

    const unsigned      numTBlocks  = ::getNumberValidTBlocks( *cs.pcv );

    for( UInt c = COMPONENT_Cb; c < numTBlocks; c++)
    {
      const ComponentID compID  = ComponentID(c);
      const CompArea&   area    = currTU.blocks[compID];

      Double     dSingleCost    = MAX_DOUBLE;
      Int        bestModeId     = 0;
      Distortion singleDistC    = 0;
      Distortion singleDistCTmp = 0;
      Double     singleCostTmp  = 0;

      const Bool checkCrossComponentPrediction = PU::isChromaIntraModeCrossCheckMode( pu ) && pps.getPpsRangeExtension().getCrossComponentPredictionEnabledFlag() && TU::getCbf( currTU, COMPONENT_Y );

      const Int  crossCPredictionModesToTest = checkCrossComponentPrediction ? 2 : 1;
      const Int  transformSkipModesToTest    = checkTransformSkip ? 2 : 1;
      const Int  totalModesToTest            = crossCPredictionModesToTest * transformSkipModesToTest;
      const Bool isOneMode                   = (totalModesToTest == 1);

      Int currModeId = 0;
      Int default0Save1Load2 = 0;

      TempCtx ctxStart  ( m_CtxCache );
      TempCtx ctxBest   ( m_CtxCache );

      if (!isOneMode)
      {
        ctxStart = m_CABACEstimator->getCtx();
      }

      for (Int transformSkipModeId = 0; transformSkipModeId < transformSkipModesToTest; transformSkipModeId++)
      {
        for (Int crossCPredictionModeId = 0; crossCPredictionModeId < crossCPredictionModesToTest; crossCPredictionModeId++)
        {
          currTU.compAlpha    [compID] = 0;
          currTU.transformSkip[compID] = transformSkipModeId;

          currModeId++;

          const Bool isFirstMode = (currModeId == 1);
          const Bool isLastMode  = (currModeId == totalModesToTest); // currModeId is indexed from 1

          if (isOneMode)
          {
            default0Save1Load2 = 0;
          }
          else if (!isOneMode && (transformSkipModeId == 0) && (crossCPredictionModeId == 0))
          {
            default0Save1Load2 = 1; //save prediction on first mode
          }
          else
          {
            default0Save1Load2 = 2; //load it on subsequent modes
          }

          if (!isFirstMode) // if not first mode to be tested
          {
            m_CABACEstimator->getCtx() = ctxStart;
          }

          singleDistCTmp = 0;

          xIntraCodingTUBlock( currTU, compID, crossCPredictionModeId != 0, singleDistCTmp, default0Save1Load2 );

          if( ( ( crossCPredictionModeId == 1 ) && ( currTU.compAlpha[compID] == 0 ) ) || ( ( transformSkipModeId == 1 ) && !TU::getCbf( currTU, compID ) ) ) //In order not to code TS flag when cbf is zero, the case for TS with cbf being zero is forbidden.
          {
            singleCostTmp = MAX_DOUBLE;
          }
          else if( !isOneMode )
          {
            UInt64 fracBitsTmp = xGetIntraFracBitsQTChroma( currTU, compID );
            singleCostTmp = m_pcRdCost->calcRdCost( fracBitsTmp, singleDistCTmp );
          }

          if( singleCostTmp < dSingleCost )
          {
            dSingleCost = singleCostTmp;
            singleDistC = singleDistCTmp;
            bestModeId  = currModeId;

            if( !isLastMode )
            {
#if KEEP_PRED_AND_RESI_SIGNALS
              saveCS.getPredBuf   (area).copyFrom(cs.getPredBuf   (area));
              saveCS.getOrgResiBuf(area).copyFrom(cs.getOrgResiBuf(area));
#endif
              if( keepResi )
              {
                saveCS.getResiBuf (area).copyFrom(cs.getResiBuf   (area));
              }
              saveCS.getRecoBuf   (area).copyFrom(cs.getRecoBuf   (area));

              tmpTU.copyComponentFrom(currTU, compID);

              ctxBest = m_CABACEstimator->getCtx();
            }
          }
        }
      }

      if (bestModeId < totalModesToTest)
      {
#if KEEP_PRED_AND_RESI_SIGNALS
        cs.getPredBuf   (area).copyFrom(saveCS.getPredBuf   (area));
        cs.getOrgResiBuf(area).copyFrom(saveCS.getOrgResiBuf(area));
#endif
        if( keepResi )
        {
          cs.getResiBuf (area).copyFrom(saveCS.getResiBuf   (area));
        }
        cs.getRecoBuf   (area).copyFrom(saveCS.getRecoBuf   (area));

        currTU.copyComponentFrom(tmpTU, compID);

        m_CABACEstimator->getCtx() = ctxBest;
      }

      cs.picture->getRecoBuf(area).copyFrom(cs.getRecoBuf(area));

      cbfs.cbf(compID) = TU::getCbf(currTU, compID);

      cs.dist += singleDistC;
    }
  }

  return cbfs;
}

UInt64 IntraSearch::xFracModeBitsIntra(PredictionUnit &pu, const UInt &uiMode, const ChannelType &chType)
{
  UInt orgMode = uiMode;

  std::swap(orgMode, pu.intraDir[chType]);

  m_CABACEstimator->resetBits();

  if( isLuma( chType ) )
  {
    m_CABACEstimator->intra_luma_pred_mode( pu );
  }
  else
  {
    m_CABACEstimator->intra_chroma_pred_mode( pu );
  }

  std::swap(orgMode, pu.intraDir[chType]);

  return m_CABACEstimator->getEstFracBits();
}



void IntraSearch::encPredIntraDPCM( const ComponentID &compID, PelBuf &pOrg, PelBuf &pDst, const UInt &uiDirMode )
{
  CHECK( pOrg.buf == 0, "Encoder DPCM called without original buffer" );

  const int srcStride = (pDst.width + pDst.height + 1);
  CPelBuf   pSrc      = CPelBuf( getPredictorPtr( compID, false ), srcStride, srcStride );

  // Sample Adaptive intra-Prediction (SAP)
  if( uiDirMode == HOR_IDX )
  {
    // left column filled with reference samples, remaining columns filled with pOrg data
    for( int y = 0; y < pDst.height; y++ )
    {
      pDst.at( 0, y ) = pSrc.at( 0, 1 + y );
    }
    CPelBuf orgRest  = pOrg.subBuf( 0, 0, pOrg.width - 1, pOrg.height );
    PelBuf  predRest = pDst.subBuf( 1, 0, pDst.width - 1, pDst.height );

    predRest.copyFrom( orgRest );
  }
  else // VER_IDX
  {
    // top row filled with reference samples, remaining rows filled with pOrg data
    for( int x = 0; x < pDst.width; x++ )
    {
      pDst.at( x, 0 ) = pSrc.at( 1 + x, 0 );
    }
    CPelBuf orgRest  = pOrg.subBuf( 0, 0, pOrg.width, pOrg.height - 1 );
    PelBuf  predRest = pDst.subBuf( 0, 1, pDst.width, pDst.height - 1 );

    predRest.copyFrom( orgRest );
  }
}

bool IntraSearch::useDPCMForFirstPassIntraEstimation( const PredictionUnit &pu, const UInt &uiDirMode )
{
  return CU::isRDPCMEnabled( *pu.cu ) && pu.cu->transQuantBypass && (uiDirMode == HOR_IDX || uiDirMode == VER_IDX);
}
