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

/** \file     DecCu.cpp
    \brief    CU decoder class
*/

#include "DecCu.h"

#include "CommonLib/CrossCompPrediction.h"
#include "CommonLib/InterPrediction.h"
#include "CommonLib/IntraPrediction.h"
#include "CommonLib/Picture.h"
#include "CommonLib/UnitTools.h"
#include "CommonLib/DiffusionFilter.h"
#if THRESHOLDING
#include "CommonLib/Thresholding.h"
#endif

#include "CommonLib/dtrace_buffer.h"


//! \ingroup DecoderLib
//! \{

// ====================================================================================================================
// Constructor / destructor / create / destroy
// ====================================================================================================================

DecCu::DecCu()
{
}

DecCu::~DecCu()
{
  m_bilateralFilter.destroy();
}

#if THRESHOLDING
Void DecCu::init( TrQuant* pcTrQuant, IntraPrediction* pcIntra, InterPrediction* pcInter, DiffusionFilter* diffFilter, Thresholding* pcThresholding )
#else
Void DecCu::init( TrQuant* pcTrQuant, IntraPrediction* pcIntra, InterPrediction* pcInter, DiffusionFilter* diffFilter )
#endif
{
  m_pcTrQuant       = pcTrQuant;
  m_pcIntraPred     = pcIntra;
  m_pcInterPred     = pcInter;
  m_DiffusionFilter = diffFilter;
#if THRESHOLDING
  m_pcThresholding  = pcThresholding;
#endif

  m_bilateralFilter .create();
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

Void DecCu::decompressCtu( CodingStructure& cs, const UnitArea& ctuArea )
{
  const int maxNumChannelType = cs.pcv->chrFormat != CHROMA_400 && CS::isDualITree( cs ) ? 2 : 1;

  for( int ch = 0; ch < maxNumChannelType; ch++ )
  {
    const ChannelType chType = ChannelType( ch );

    for( auto &currCU : cs.traverseCUs( CS::getArea( cs, ctuArea, chType ), chType ) )
    {
      switch( currCU.predMode )
      {
      case MODE_INTER:
        xDeriveCUMV( currCU );
        xReconInter( currCU );
        break;
      case MODE_INTRA:
        xReconIntraQT( currCU );
        break;
      default:
        THROW( "Invalid prediction mode" );
        break;
      }

      if( CU::isLosslessCoded( currCU ) && !currCU.ipcm )
      {
        xFillPCMBuffer( currCU );
      }

      DTRACE_BLOCK_REC( cs.picture->getRecoBuf( currCU ), currCU, currCU.predMode );
    }
  }
}

// ====================================================================================================================
// Protected member functions
// ====================================================================================================================

Void DecCu::xIntraRecBlk( TransformUnit& tu, const ComponentID compID )
{
  if( !tu.blocks[ compID ].valid() )
  {
    return;
  }

        CodingStructure &cs = *tu.cs;
  const CompArea &area      = tu.blocks[compID];
  const SPS &sps            = *cs.sps;

  const ChannelType chType  = toChannelType( compID );

        PelBuf piPred       = cs.getPredBuf( area );

        PredictionUnit &pu  = *tu.cs->getPU( area.pos(), chType );
  const UInt uiChFinalMode  = PU::getFinalIntraMode( pu, chType );

  //===== init availability pattern =====

  const bool bUseFilteredPredictions = IntraPrediction::useFilteredIntraRefSamples( compID, pu, true, tu );
  m_pcIntraPred->initIntraPatternChType( *tu.cu, area, bUseFilteredPredictions );

  //===== get prediction signal =====
  if( compID != COMPONENT_Y && PU::isLMCMode( uiChFinalMode ) )
  {
    const PredictionUnit& pu = cs.pcv->noRQT && cs.pcv->only2Nx2N ? *tu.cu->firstPU : *tu.cs->getPU( tu.block( compID ), CHANNEL_TYPE_CHROMA );
    m_pcIntraPred->xGetLumaRecPixels( pu, area );
    m_pcIntraPred->predIntraChromaLM( compID, piPred, pu, area, uiChFinalMode );
  }
  else
  {
    if( sps.getSpsNext().getUseIntraFTM() && pu.FTMRegIdx > 0 && compID == COMPONENT_Y )
    {
      // FTM intra mode, available only for luma
      m_pcIntraPred->predIntraFastTM( piPred, pu, compID, area.x, area.y );
    }
    else if( sps.getSpsNext().getUseIntra_NN() && pu.cu->intra_NN && isLuma( compID ) )
    {
      m_pcIntraPred->jointPreparationForNNIntraPrediction( pu );
      pu.intraNN_Mode_True = m_pcIntraPred->getFinalNNMode( uiChFinalMode );
      m_pcIntraPred->predIntraNNModel( compID, piPred, pu );
    }
    else
    {
      const bool isFirstPartition = (tu.cu->mode1dPartitions && canUse1dPartitions( compID ) && (area.width == 1 || area.height == 1)) ? CU::isFirst1dPartition( *tu.cu, area, compID ) : true;

      m_pcIntraPred->predIntraAng( compID, piPred, pu, bUseFilteredPredictions, isFirstPartition );
    }
    if (pu.cu->diffFilterIdx && pu.cs->sps->getSpsNext().getDiffusionFilterEnabled() && compID == COMPONENT_Y)
    {
      CHECK(!CU::isDiffIdxPresent(*pu.cu), "!CU::isDiffIdxPresent(cu)");
      m_DiffusionFilter->applyDiffusion(compID, *pu.cu, tu.blocks[compID], piPred, piPred);
    }
#if THRESHOLDING
    if( tu.thresholding && compID == COMPONENT_Y )
    {
      m_pcThresholding->getNumberValidThrs( COMPONENT_Y, tu, piPred );
      m_pcThresholding->applyThresholding ( COMPONENT_Y, tu, piPred );
      DTRACE_CCRC(g_trace_ctx, D_CRC, cs, piPred, COMPONENT_Y, &(tu.blocks[compID]));
    }
#endif

    if( compID == COMPONENT_Cr && sps.getSpsNext().getUseLMChroma() )
    {
      const CPelBuf pResiCb = cs.getResiBuf( tu.Cb() );
      m_pcIntraPred->addCrossColorResi( compID, piPred, tu, pResiCb );
    }
  }

  //===== inverse transform =====
  PelBuf piResi = cs.getResiBuf( area );

  const QpParam cQP( tu, compID );

  if( TU::getCbf( tu, compID ) )
  {
    m_pcTrQuant->invTransformNxN( tu, compID, piResi, cQP );
  }
  else
  {
    piResi.fill( 0 );
  }

  //===== reconstruction =====
  if( isChroma(compID) && tu.compAlpha[compID] != 0 )
  {
    CrossComponentPrediction::crossComponentPrediction( tu, compID, cs.getResiBuf( tu.Y() ), piResi, piResi, true );
  }

  PelBuf pReco = cs.getRecoBuf( area );

  if( !tu.cu->mode1dPartitions || !canUse1dPartitions( compID ) )
  {
    cs.setDecomp( area );
  }
  else if( tu.cu->mode1dPartitions && canUse1dPartitions( compID ) && CU::isFirst1dPartition( *tu.cu, tu.blocks[compID], compID ) )
  {
    cs.setDecomp( tu.cu->blocks[compID] );
  }

#if KEEP_PRED_AND_RESI_SIGNALS
  pReco.reconstruct( piPred, piResi, tu.cu->cs->slice->clpRng( compID ) );
#else
  piPred.reconstruct( piPred, piResi, tu.cu->cs->slice->clpRng( compID ) );
#endif
  if( !tu.cu->mode1dPartitions || !canUse1dPartitions( compID ) )
  if( sps.getSpsNext().getUseBIF() && isLuma( compID ) && TU::getCbf( tu, compID ) && ( tu.cu->qp > 17 ) )
  {
#if KEEP_PRED_AND_RESI_SIGNALS
    m_bilateralFilter.bilateralFilterIntra( pReco, tu.cu->qp );
#else
    m_bilateralFilter.bilateralFilterIntra( piPred, tu.cu->qp );
#endif
  }

#if !KEEP_PRED_AND_RESI_SIGNALS
  pReco.copyFrom( piPred );
#endif
}

Void DecCu::xReconIntraQT( CodingUnit &cu )
{
  if( cu.ipcm )
  {
    xReconPCM( *cu.firstTU );
    return;
  }

  const UInt numChType = ::getNumberValidChannels( cu.chromaFormat );

  for( UInt chType = CHANNEL_TYPE_LUMA; chType < numChType; chType++ )
  {
    if( cu.blocks[chType].valid() )
    {
      xIntraRecQT( cu, ChannelType( chType ) );
    }
  }
}

/** Function for deriving reconstructed luma/chroma samples of a PCM mode CU.
* \param pcCU pointer to current CU
* \param uiPartIdx part index
* \param piPCM pointer to PCM code arrays
* \param piReco pointer to reconstructed sample arrays
* \param uiStride stride of reconstructed sample arrays
* \param uiWidth CU width
* \param uiHeight CU height
* \param compID colour component ID
* \returns Void
*/
Void DecCu::xDecodePCMTexture(TransformUnit &tu, const ComponentID compID)
{
  const CompArea &area         = tu.blocks[compID];
        PelBuf piPicReco       = tu.cs->getRecoBuf( area );
  const CPelBuf piPicPcm       = tu.getPcmbuf(compID);
  const SPS &sps               = *tu.cs->sps;
  const UInt uiPcmLeftShiftBit = sps.getBitDepth(toChannelType(compID)) - sps.getPCMBitDepth(toChannelType(compID));

  for (UInt uiY = 0; uiY < area.height; uiY++)
  {
    for (UInt uiX = 0; uiX < area.width; uiX++)
    {
      piPicReco.at(uiX, uiY) = (piPicPcm.at(uiX, uiY) << uiPcmLeftShiftBit);
    }
  }

  tu.cs->picture->getRecoBuf( area ).copyFrom( piPicReco );
  tu.cs->setDecomp( area );
}

/** Function for reconstructing a PCM mode CU.
* \param pcCU pointer to current CU
* \param uiDepth CU Depth
* \returns Void
*/
Void DecCu::xReconPCM(TransformUnit &tu)
{
  for (UInt ch = 0; ch < tu.blocks.size(); ch++)
  {
    ComponentID compID = ComponentID(ch);

    xDecodePCMTexture(tu, compID);
  }
}

/** Function for deriving reconstructed PU/CU chroma samples with QTree structure
* \param pcRecoYuv pointer to reconstructed sample arrays
* \param pcPredYuv pointer to prediction sample arrays
* \param pcResiYuv pointer to residue sample arrays
* \param chType    texture channel type (luma/chroma)
* \param rTu       reference to transform data
*
\ This function derives reconstructed PU/CU chroma samples with QTree recursive structure
*/

Void
DecCu::xIntraRecQT(CodingUnit &cu, const ChannelType chType)
{
  for( auto &currTU : CU::traverseTUs( cu ) )
  {
    if( isLuma( chType ) )
    {
      xIntraRecBlk( currTU, COMPONENT_Y );
    }
    else
    {
      const UInt numValidComp = getNumberValidComponents( cu.chromaFormat );

      for( UInt compID = COMPONENT_Cb; compID < numValidComp; compID++ )
      {
        xIntraRecBlk( currTU, ComponentID( compID ) );
      }
    }
  }
}

/** Function for filling the PCM buffer of a CU using its reconstructed sample array
* \param pCU   pointer to current CU
* \param depth CU Depth
*/
Void DecCu::xFillPCMBuffer(CodingUnit &cu)
{
  for( auto &currTU : CU::traverseTUs( cu ) )
  {
    for (const CompArea &area : currTU.blocks)
    {
      if( !area.valid() ) continue;;

      CPelBuf source      = cu.cs->getRecoBuf(area);
       PelBuf destination = currTU.getPcmbuf(area.compID);

      destination.copyFrom(source);
    }
  }
}

Void DecCu::xReconInter(CodingUnit &cu)
{
  // inter prediction
  m_pcInterPred->motionCompensation( cu );
  m_pcInterPred->subBlockOBMC      ( cu );

  if( cu.diffFilterIdx )
  {
    PelBuf predY = cu.cs->getPredBuf( cu ).Y();
    m_DiffusionFilter->applyDiffusion( COMPONENT_Y, cu, cu.blocks[COMPONENT_Y], predY, predY );
  }

#if THRESHOLDING    
  if( cu.firstTU->thresholding )
  {
    PelBuf predY = cu.cs->getPredBuf( cu ).Y();
    m_pcThresholding->getNumberValidThrs( COMPONENT_Y, *cu.firstTU, predY );
    m_pcThresholding->applyThresholding ( COMPONENT_Y, *cu.firstTU, predY );
  }
#endif
  // inter recon
  xDecodeInterTexture(cu);

  // clip for only non-zero cbf case
  CodingStructure &cs = *cu.cs;

  if (cu.rootCbf)
  {
#if KEEP_PRED_AND_RESI_SIGNALS
    cs.getRecoBuf( cu ).reconstruct( cs.getPredBuf( cu ), cs.getResiBuf( cu ), cs.slice->clpRngs() );
#else
    cs.getResiBuf( cu ).reconstruct( cs.getPredBuf( cu ), cs.getResiBuf( cu ), cs.slice->clpRngs() );
    cs.getRecoBuf( cu ).copyFrom( cs.getResiBuf( cu ) );
#endif
  }
  else
  {
    cs.getRecoBuf(cu).copyClip(cs.getPredBuf(cu), cs.slice->clpRngs());
  }

  cs.setDecomp(cu);
}

Void DecCu::xDecodeInterTU( TransformUnit & currTU, const ComponentID compID )
{
  if( !currTU.blocks[compID].valid() ) return;

  const CompArea &area = currTU.blocks[compID];

  CodingStructure& cs = *currTU.cs;

  //===== inverse transform =====
  PelBuf resiBuf  = cs.getResiBuf(area);

  const QpParam cQP(currTU, compID);

  if( TU::getCbf( currTU, compID ) )
  {
    m_pcTrQuant->invTransformNxN( currTU, compID, resiBuf, cQP );
    if( cs.sps->getSpsNext().getUseBIF() && isLuma(compID) && (currTU.cu->qp > 17) && (16 > std::min(currTU.lumaSize().width, currTU.lumaSize().height) ) )
    {
      const CPelBuf predBuf  = cs.getPredBuf(area);
      m_bilateralFilter.bilateralFilterInter( resiBuf, predBuf, currTU.cu->qp, cs.slice->clpRng(compID) );
    }
  }
  else
  {
    resiBuf.fill( 0 );
  }

  //===== reconstruction =====
  if( isChroma( compID ) && currTU.compAlpha[compID] != 0 )
  {
    CrossComponentPrediction::crossComponentPrediction( currTU, compID, cs.getResiBuf( currTU.Y() ), resiBuf, resiBuf, true );
  }
}

Void DecCu::xDecodeInterTexture(CodingUnit &cu)
{
  if( !cu.rootCbf )
  {
    return;
  }

  const UInt uiNumVaildComp = getNumberValidComponents(cu.chromaFormat);

  for (UInt ch = 0; ch < uiNumVaildComp; ch++)
  {
    const ComponentID compID = ComponentID(ch);

    for( auto& currTU : CU::traverseTUs( cu ) )
    {
      xDecodeInterTU( currTU, compID );
    }
  }
}

Void DecCu::xDeriveCUMV( CodingUnit &cu )
{
  for( auto &pu : CU::traversePUs( cu ) )
  {
    MergeCtx mrgCtx;

    const unsigned imvShift = pu.cu->imv << 1;
    for( auto &mhData : pu.addHypData )
    {
      mhData.mvd <<= imvShift;
      const Mv mvp = PU::getMultiHypMVP( pu, mhData );
      mhData.mv = mvp + mhData.mvd;
    }

    if( pu.mergeFlag )
    {
      CHECK(cu.skip && !pu.addHypData.empty(),"additional hypotheseis signalled in SKIP mode");
      // save signalled add hyp data, to put it at the end after merging
      auto addHypData = std::move( pu.addHypData );
      pu.addHypData.clear();
      if( pu.frucMrgMode )
      {
        pu.mergeType = MRG_TYPE_FRUC;

        Bool bAvailable = m_pcInterPred->deriveFRUCMV( pu );

        CHECK( !bAvailable, "fruc mode not availabe" );
        //normal merge data should be set already, to be checked
      }
      else
      {
        if( pu.cu->affine )
        {
          pu.mergeIdx = 0;
          MvField       affineMvField[2][3];
          unsigned char interDirNeighbours;
          int           numValidMergeCand;
          PU::getAffineMergeCand( pu, affineMvField, interDirNeighbours, numValidMergeCand );
          pu.interDir = interDirNeighbours;
          for( int i = 0; i < 2; ++i )
          {
            if( pu.cs->slice->getNumRefIdx( RefPicList( i ) ) > 0 )
            {
              MvField* mvField = affineMvField[i];

              pu.mvpIdx[i] = 0;
              pu.mvpNum[i] = 0;
              pu.mvd[i]    = Mv();
              PU::setAllAffineMvField( pu, mvField, RefPicList( i ) );
            }
          }
          PU::spanMotionInfo( pu, mrgCtx );
        }
        else
        {
          if( pu.cs->sps->getSpsNext().getUseSubPuMvp() )
          {
            Size bufSize = g_miScaling.scale( pu.lumaSize() );
            mrgCtx.subPuMvpMiBuf    = MotionBuf( m_SubPuMiBuf,    bufSize );
            mrgCtx.subPuMvpExtMiBuf = MotionBuf( m_SubPuExtMiBuf, bufSize );
          }

          if( cu.cs->pps->getLog2ParallelMergeLevelMinus2() && cu.partSize != SIZE_2Nx2N && cu.lumaSize().width <= 8 )
          {
            CHECK( pu.mergeRefPicList != REF_PIC_LIST_X, "Invalid merge reference list" );
            if( !mrgCtx.hasMergedCandList )
            {
              // temporarily set size to 2Nx2N
              PartSize                 tmpPS    = SIZE_2Nx2N;
              PredictionUnit           tmpPU    = pu;
              static_cast<UnitArea&> ( tmpPU )  = cu;
              std::swap( tmpPS, cu.partSize );
              PU::getInterMergeCandidates( tmpPU, mrgCtx );
              std::swap( tmpPS, cu.partSize );
              mrgCtx.hasMergedCandList          = true;
            }
          }
          else
          {
            PU::getInterMergeCandidates( pu, mrgCtx );
          }

          PU::restrictInterMergeCandidatesForRefPicList( mrgCtx, pu.mergeRefPicList, *cu.slice );
          mrgCtx.setMergeInfo( pu, pu.mergeIdx );

          if( pu.interDir == 3 /* PRED_BI */ && PU::isBipredRestriction(pu) )
          {
            pu.mv    [REF_PIC_LIST_1] = Mv(0, 0);
            pu.refIdx[REF_PIC_LIST_1] = -1;
            pu.interDir               =  1;
            pu.addHypData.clear();
          }

          PU::spanMotionInfo( pu, mrgCtx );
        }
      }
      // put saved additional hypotheseis to the end
      pu.addHypData.insert( pu.addHypData.end(), std::make_move_iterator(addHypData.begin()), std::make_move_iterator(addHypData.end()) );
    }
    else
    {
      if( cu.imv )
      {
        PU::applyImv( pu, mrgCtx, m_pcInterPred );
      }
      else
      {
        if( pu.cu->affine )
        {
          for ( UInt uiRefListIdx = 0; uiRefListIdx < 2; uiRefListIdx++ )
          {
            RefPicList eRefList = RefPicList( uiRefListIdx );
            if ( pu.cs->slice->getNumRefIdx( eRefList ) > 0 && ( pu.interDir & ( 1 << uiRefListIdx ) ) )
            {
              AffineAMVPInfo affineAMVPInfo;
              PU::fillAffineMvpCand( pu, eRefList, pu.refIdx[eRefList], affineAMVPInfo );

              const unsigned mvp_idx = pu.mvpIdx[eRefList];

              pu.mvpNum[eRefList] = affineAMVPInfo.numCand;

              //    Mv mv[3];
              CHECK( pu.refIdx[eRefList] < 0, "Unexpected negative refIdx." );

              Position posLT = pu.Y().topLeft();
              Position posRT = pu.Y().topRight();

              Mv mvLT = affineAMVPInfo.mvCandLT[mvp_idx] + pu.getMotionInfo( posLT ).mvdAffi[eRefList];
              Mv mvRT = affineAMVPInfo.mvCandRT[mvp_idx] + pu.getMotionInfo( posRT ).mvdAffi[eRefList];

              CHECK( !mvLT.highPrec, "unexpected lp mv" );
              CHECK( !mvRT.highPrec, "unexpected lp mv" );

              Int iWidth = pu.Y().width;
              Int iHeight = pu.Y().height;
              Int vx2 =  - ( mvRT.getVer() - mvLT.getVer() ) * iHeight / iWidth + mvLT.getHor();
              Int vy2 =    ( mvRT.getHor() - mvLT.getHor() ) * iHeight / iWidth + mvLT.getVer();
              //    Mv mvLB( vx2, vy2 );
              Mv mvLB( vx2, vy2, true );

              clipMv(mvLT, pu.cu->lumaPos(), *pu.cs->sps);
              clipMv(mvRT, pu.cu->lumaPos(), *pu.cs->sps);
              clipMv(mvLB, pu.cu->lumaPos(), *pu.cs->sps);
              PU::setAllAffineMv( pu, mvLT, mvRT, mvLB, eRefList );
            }
          }
        }
        else
        {
          for ( UInt uiRefListIdx = 0; uiRefListIdx < 2; uiRefListIdx++ )
          {
            RefPicList eRefList = RefPicList( uiRefListIdx );
            if ( pu.cs->slice->getNumRefIdx( eRefList ) > 0 && ( pu.interDir & ( 1 << uiRefListIdx ) ) )
            {
              AMVPInfo amvpInfo;
              PU::fillMvpCand( pu, eRefList, pu.refIdx[eRefList], amvpInfo, m_pcInterPred );
              pu.mvpNum [eRefList] = amvpInfo.numCand;
              pu.mv     [eRefList] = amvpInfo.mvCand[pu.mvpIdx [eRefList]] + pu.mvd[eRefList];

              if( pu.cs->sps->getSpsNext().getUseAffine() )
              {
                pu.mv[eRefList].setHighPrec();
              }
            }
          }
        }
        PU::spanMotionInfo( pu, mrgCtx );
      }
    }
#if MCTS_ENC_CHECK

    if( pu.cs->pps->getMctsOneRegionPerTileFlag() && !m_pcInterPred->checkTMctsMv( pu ) )
    {
      THROW( "pu motion vector across tile boundaries\n" );
    }
#endif
  }
}
//! \}