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

/** \file     EncSlice.cpp
    \brief    slice encoder class
*/

#include "EncSlice.h"

#include "EncLib.h"

#include "CommonLib/UnitTools.h"

#include <math.h>

//! \ingroup EncoderLib
//! \{

// ====================================================================================================================
// Constructor / destructor / create / destroy
// ====================================================================================================================

EncSlice::EncSlice()
 : m_encCABACTableIdx(I_SLICE)
#if HHI_HLM_USE_QPA
 , m_uEnerHpCtu (nullptr)
#endif
{
}

EncSlice::~EncSlice()
{
  destroy();
}

Void EncSlice::create( Int iWidth, Int iHeight, ChromaFormat chromaFormat, UInt iMaxCUWidth, UInt iMaxCUHeight, UChar uhTotalDepth )
{
#if HHI_HLM_USE_QPA
  const UInt L = ((iWidth + iMaxCUWidth - 1) / iMaxCUWidth) * ((iHeight + iMaxCUHeight - 1) / iMaxCUHeight);

  m_uEnerHpCtu = (double*) xMalloc (double, L);
#endif
}

Void EncSlice::destroy()
{
#if HHI_HLM_USE_QPA
  if (m_uEnerHpCtu) xFree (m_uEnerHpCtu);
  m_uEnerHpCtu = nullptr;
#endif
  // free lambda and QP arrays
  m_vdRdPicLambda.clear();
  m_vdRdPicQp.clear();
  m_viRdPicQp.clear();
}

Void EncSlice::init( EncLib* pcEncLib, const SPS& sps )
{
  m_pcCfg             = pcEncLib;
  m_pcListPic         = pcEncLib->getListPic();

  m_pcGOPEncoder      = pcEncLib->getGOPEncoder();
  m_pcCuEncoder       = pcEncLib->getCuEncoder();
  m_pcInterSearch     = pcEncLib->getInterSearch();
  m_CABACEncoder      = pcEncLib->getCABACEncoder();
  m_CABACWriter       = m_CABACEncoder->getCABACWriter   (&sps);
  m_CABACEstimator    = m_CABACEncoder->getCABACEstimator(&sps);
  m_pcTrQuant         = pcEncLib->getTrQuant();
  m_pcRdCost          = pcEncLib->getRdCost();

  // create lambda and QP arrays
  m_vdRdPicLambda.resize(m_pcCfg->getDeltaQpRD() * 2 + 1 );
  m_vdRdPicQp.resize(    m_pcCfg->getDeltaQpRD() * 2 + 1 );
  m_viRdPicQp.resize(    m_pcCfg->getDeltaQpRD() * 2 + 1 );
  m_pcRateCtrl        = pcEncLib->getRateCtrl();
}



#if SHARP_LUMA_DELTA_QP
Void EncSlice::updateLambda( Slice* slice, Double dQP)
{
  Int iQP = (Int)dQP;
  Double dLambda = calculateLambda(slice, m_gopID, slice->getDepth(), slice->getSliceQp(), dQP, iQP);

  setUpLambda(slice, dLambda, iQP);
}
#endif

Void
EncSlice::setUpLambda( Slice* slice, const Double dLambda, Int iQP)
{
  // store lambda
  m_pcRdCost ->setLambda( dLambda, slice->getSPS()->getBitDepths() );

  // for RDO
  // in RdCost there is only one lambda because the luma and chroma bits are not separated, instead we weight the distortion of chroma.
  Double dLambdas[MAX_NUM_COMPONENT] = { dLambda };
  for(UInt compIdx=1; compIdx<MAX_NUM_COMPONENT; compIdx++)
  {
    const ComponentID compID=ComponentID(compIdx);
    Int chromaQPOffset = slice->getPPS()->getQpOffset(compID) + slice->getSliceChromaQpDelta(compID);
    Int qpc=(iQP + chromaQPOffset < 0) ? iQP : getScaledChromaQP(iQP + chromaQPOffset, m_pcCfg->getChromaFormatIdc());
    Double tmpWeight = pow( 2.0, (iQP-qpc)/3.0 );  // takes into account of the chroma qp mapping and chroma qp Offset
    m_pcRdCost->setDistortionWeight(compID, tmpWeight);
    dLambdas[compIdx]=dLambda/tmpWeight;
  }

#if RDOQ_CHROMA_LAMBDA
// for RDOQ
  m_pcTrQuant->setLambdas( dLambdas );
#else
  m_pcTrQuant->setLambda( dLambda );
#endif

// For SAO
  slice->setLambdas( dLambdas );
}



/**
 - non-referenced frame marking
 - QP computation based on temporal structure
 - lambda computation based on QP
 - set temporal layer ID and the parameter sets
 .
 \param pcPic         picture class
 \param pocLast       POC of last picture
 \param pocCurr       current POC
 \param iNumPicRcvd   number of received pictures
 \param iGOPid        POC offset for hierarchical structure
 \param rpcSlice      slice header class
 \param isField       true for field coding
 */

Void EncSlice::initEncSlice( Picture* pcPic, const Int pocLast, const Int pocCurr, const Int iGOPid, Slice*& rpcSlice, const Bool isField )
{
  Double dQP;
  Double dLambda;

  rpcSlice = pcPic->slices[0];
  rpcSlice->setSliceBits(0);
  rpcSlice->setPic( pcPic );
  rpcSlice->initSlice();
  rpcSlice->setPicOutputFlag( true );
  rpcSlice->setPOC( pocCurr );
  rpcSlice->setUseLIC( false );

#if SHARP_LUMA_DELTA_QP
  pcPic->fieldPic = isField;
  m_gopID = iGOPid;
#endif

  // depth computation based on GOP size
  Int depth;
  {
    Int poc = rpcSlice->getPOC();
    if(isField)
    {
      poc = (poc/2) % (m_pcCfg->getGOPSize()/2);
    }
    else
    {
      poc = poc % m_pcCfg->getGOPSize();
    }

    if ( poc == 0 )
    {
      depth = 0;
    }
    else
    {
      Int step = m_pcCfg->getGOPSize();
      depth    = 0;
      for( Int i=step>>1; i>=1; i>>=1 )
      {
        for ( Int j=i; j<m_pcCfg->getGOPSize(); j+=step )
        {
          if ( j == poc )
          {
            i=0;
            break;
          }
        }
        step >>= 1;
        depth++;
      }
    }

    if(m_pcCfg->getHarmonizeGopFirstFieldCoupleEnabled() && poc != 0)
    {
      if (isField && ((rpcSlice->getPOC() % 2) == 1))
      {
        depth ++;
      }
    }
  }

  // slice type
  SliceType eSliceType;

  eSliceType=B_SLICE;
  if(!(isField && pocLast == 1) || !m_pcCfg->getEfficientFieldIRAPEnabled())
  {
    if(m_pcCfg->getDecodingRefreshType() == 3)
    {
      eSliceType = (pocLast == 0 || pocCurr % m_pcCfg->getIntraPeriod() == 0             || m_pcGOPEncoder->getGOPSize() == 0) ? I_SLICE : eSliceType;
    }
    else
    {
      eSliceType = (pocLast == 0 || (pocCurr - (isField ? 1 : 0)) % m_pcCfg->getIntraPeriod() == 0 || m_pcGOPEncoder->getGOPSize() == 0) ? I_SLICE : eSliceType;
    }
  }

  rpcSlice->setSliceType    ( eSliceType );

  // ------------------------------------------------------------------------------------------------------------------
  // Non-referenced frame marking
  // ------------------------------------------------------------------------------------------------------------------

  if(pocLast == 0)
  {
    rpcSlice->setTemporalLayerNonReferenceFlag(false);
  }
  else
  {
    rpcSlice->setTemporalLayerNonReferenceFlag(!m_pcCfg->getGOPEntry(iGOPid).m_refPic);
  }
  pcPic->referenced = true;

  // ------------------------------------------------------------------------------------------------------------------
  // QP setting
  // ------------------------------------------------------------------------------------------------------------------

#if X0038_LAMBDA_FROM_QP_CAPABILITY
  dQP = m_pcCfg->getQPForPicture(iGOPid, rpcSlice);
#else
  dQP = m_pcCfg->getQP();
  if(eSliceType!=I_SLICE)
  {
#if SHARP_LUMA_DELTA_QP
    if (!(( m_pcCfg->getMaxDeltaQP() == 0) && (!m_pcCfg->getLumaLevelToDeltaQPMapping().isEnabled()) && (dQP == -rpcSlice->getSPS()->getQpBDOffset(CHANNEL_TYPE_LUMA) ) && (rpcSlice->getPPS()->getTransquantBypassEnabledFlag())))
#else
    if (!(( m_pcCfg->getMaxDeltaQP() == 0 ) && (dQP == -rpcSlice->getSPS()->getQpBDOffset(CHANNEL_TYPE_LUMA) ) && (rpcSlice->getPPS()->getTransquantBypassEnabledFlag())))
#endif
    {
      dQP += m_pcCfg->getGOPEntry(iGOPid).m_QPOffset;
    }
  }

  // modify QP
  const Int* pdQPs = m_pcCfg->getdQPs();
  if ( pdQPs )
  {
    dQP += pdQPs[ rpcSlice->getPOC() ];
  }

  if (m_pcCfg->getCostMode()==COST_LOSSLESS_CODING)
  {
    dQP=LOSSLESS_AND_MIXED_LOSSLESS_RD_COST_TEST_QP;
    m_pcCfg->setDeltaQpRD(0);
  }
#endif

  // ------------------------------------------------------------------------------------------------------------------
  // Lambda computation
  // ------------------------------------------------------------------------------------------------------------------

#if X0038_LAMBDA_FROM_QP_CAPABILITY
  const Int temporalId=m_pcCfg->getGOPEntry(iGOPid).m_temporalId;
#if !SHARP_LUMA_DELTA_QP
  const std::vector<Double> &intraLambdaModifiers=m_pcCfg->getIntraLambdaModifier();
#endif
#endif
  Int iQP;
  Double dOrigQP = dQP;

  // pre-compute lambda and QP values for all possible QP candidates
  for ( Int iDQpIdx = 0; iDQpIdx < 2 * m_pcCfg->getDeltaQpRD() + 1; iDQpIdx++ )
  {
    // compute QP value
    dQP = dOrigQP + ((iDQpIdx+1)>>1)*(iDQpIdx%2 ? -1 : 1);
#if SHARP_LUMA_DELTA_QP
    dLambda = calculateLambda(rpcSlice, iGOPid, depth, dQP, dQP, iQP );
#else
    // compute lambda value
    Int    NumberBFrames = ( m_pcCfg->getGOPSize() - 1 );
    Int    SHIFT_QP = 12;

#if FULL_NBIT
    Int    bitdepth_luma_qp_scale = 6 * (rpcSlice->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA) - 8);
#else
    Int    bitdepth_luma_qp_scale = 0;
#endif
    Double qp_temp = (Double) dQP + bitdepth_luma_qp_scale - SHIFT_QP;
#if FULL_NBIT
    Double qp_temp_orig = (Double) dQP - SHIFT_QP;
#endif
    // Case #1: I or P-slices (key-frame)
    Double dQPFactor = m_pcCfg->getGOPEntry(iGOPid).m_QPFactor;
    if ( eSliceType==I_SLICE )
    {
      if (m_pcCfg->getIntraQpFactor()>=0.0 && m_pcCfg->getGOPEntry(iGOPid).m_sliceType != I_SLICE)
      {
        dQPFactor=m_pcCfg->getIntraQpFactor();
      }
      else
      {
#if X0038_LAMBDA_FROM_QP_CAPABILITY
        if(m_pcCfg->getLambdaFromQPEnable())
        {
          dQPFactor=0.57;
        }
        else
        {
#endif
        Double dLambda_scale = 1.0 - Clip3( 0.0, 0.5, 0.05*(Double)(isField ? NumberBFrames/2 : NumberBFrames) );

        dQPFactor=0.57*dLambda_scale;
#if X0038_LAMBDA_FROM_QP_CAPABILITY
        }
#endif
      }
    }
#if X0038_LAMBDA_FROM_QP_CAPABILITY
    else if( m_pcCfg->getLambdaFromQPEnable() )
    {
      dQPFactor=0.57;
    }
#endif

    dLambda = dQPFactor*pow( 2.0, qp_temp/3.0 );

#if X0038_LAMBDA_FROM_QP_CAPABILITY
    if(!m_pcCfg->getLambdaFromQPEnable() && depth>0)
#else
    if ( depth>0 )
#endif
    {
#if FULL_NBIT
        dLambda *= Clip3( 2.00, 4.00, (qp_temp_orig / 6.0) ); // (j == B_SLICE && p_cur_frm->layer != 0 )
#else
        dLambda *= Clip3( 2.00, 4.00, (qp_temp / 6.0) ); // (j == B_SLICE && p_cur_frm->layer != 0 )
#endif
    }

    // if hadamard is used in ME process
    if ( !m_pcCfg->getUseHADME() && rpcSlice->getSliceType( ) != I_SLICE )
    {
      dLambda *= 0.95;
    }

#if X0038_LAMBDA_FROM_QP_CAPABILITY
    Double lambdaModifier;
    if( rpcSlice->getSliceType( ) != I_SLICE || intraLambdaModifiers.empty())
    {
      lambdaModifier = m_pcCfg->getLambdaModifier( temporalId );
    }
    else
    {
      lambdaModifier = intraLambdaModifiers[ (temporalId < intraLambdaModifiers.size()) ? temporalId : (intraLambdaModifiers.size()-1) ];
    }
    dLambda *= lambdaModifier;
#endif

    iQP = max( -rpcSlice->getSPS()->getQpBDOffset(CHANNEL_TYPE_LUMA), min( MAX_QP, (Int) floor( dQP + 0.5 ) ) );
#endif

    m_vdRdPicLambda[iDQpIdx] = dLambda;
    m_vdRdPicQp    [iDQpIdx] = dQP;
    m_viRdPicQp    [iDQpIdx] = iQP;
  }

  // obtain dQP = 0 case
  dLambda = m_vdRdPicLambda[0];
  dQP     = m_vdRdPicQp    [0];
  iQP     = m_viRdPicQp    [0];

#if !X0038_LAMBDA_FROM_QP_CAPABILITY
  const Int temporalId=m_pcCfg->getGOPEntry(iGOPid).m_temporalId;
  const std::vector<Double> &intraLambdaModifiers=m_pcCfg->getIntraLambdaModifier();
#endif

#if W0038_CQP_ADJ
  if(rpcSlice->getPPS()->getSliceChromaQpFlag())
  {
    const Bool bUseIntraOrPeriodicOffset = rpcSlice->getSliceType()==I_SLICE || (m_pcCfg->getSliceChromaOffsetQpPeriodicity()!=0 && (rpcSlice->getPOC()%m_pcCfg->getSliceChromaOffsetQpPeriodicity())==0);
    Int cbQP = bUseIntraOrPeriodicOffset? m_pcCfg->getSliceChromaOffsetQpIntraOrPeriodic(false) : m_pcCfg->getGOPEntry(iGOPid).m_CbQPoffset;
    Int crQP = bUseIntraOrPeriodicOffset? m_pcCfg->getSliceChromaOffsetQpIntraOrPeriodic(true)  : m_pcCfg->getGOPEntry(iGOPid).m_CrQPoffset;

    cbQP = Clip3( -12, 12, cbQP + rpcSlice->getPPS()->getQpOffset(COMPONENT_Cb) ) - rpcSlice->getPPS()->getQpOffset(COMPONENT_Cb);
    crQP = Clip3( -12, 12, crQP + rpcSlice->getPPS()->getQpOffset(COMPONENT_Cr) ) - rpcSlice->getPPS()->getQpOffset(COMPONENT_Cr);
    rpcSlice->setSliceChromaQpDelta(COMPONENT_Cb, Clip3( -12, 12, cbQP));
    CHECK(!(rpcSlice->getSliceChromaQpDelta(COMPONENT_Cb)+rpcSlice->getPPS()->getQpOffset(COMPONENT_Cb)<=12 && rpcSlice->getSliceChromaQpDelta(COMPONENT_Cb)+rpcSlice->getPPS()->getQpOffset(COMPONENT_Cb)>=-12), "Unspecified error");
    rpcSlice->setSliceChromaQpDelta(COMPONENT_Cr, Clip3( -12, 12, crQP));
    CHECK(!(rpcSlice->getSliceChromaQpDelta(COMPONENT_Cr)+rpcSlice->getPPS()->getQpOffset(COMPONENT_Cr)<=12 && rpcSlice->getSliceChromaQpDelta(COMPONENT_Cr)+rpcSlice->getPPS()->getQpOffset(COMPONENT_Cr)>=-12), "Unspecified error");
  }
  else
  {
    rpcSlice->setSliceChromaQpDelta( COMPONENT_Cb, 0 );
    rpcSlice->setSliceChromaQpDelta( COMPONENT_Cr, 0 );
  }
#endif

#if !X0038_LAMBDA_FROM_QP_CAPABILITY
  Double lambdaModifier;
  if( rpcSlice->getSliceType( ) != I_SLICE || intraLambdaModifiers.empty())
  {
    lambdaModifier = m_pcCfg->getLambdaModifier( temporalId );
  }
  else
  {
    lambdaModifier = intraLambdaModifiers[ (temporalId < intraLambdaModifiers.size()) ? temporalId : (intraLambdaModifiers.size()-1) ];
  }

  dLambda *= lambdaModifier;
#endif

  setUpLambda(rpcSlice, dLambda, iQP);

  if (m_pcCfg->getFastMEForGenBLowDelayEnabled())
  {
    // restore original slice type

    if(!(isField && pocLast == 1) || !m_pcCfg->getEfficientFieldIRAPEnabled())
    {
      if(m_pcCfg->getDecodingRefreshType() == 3)
      {
        eSliceType = (pocLast == 0 || (pocCurr)                     % m_pcCfg->getIntraPeriod() == 0 || m_pcGOPEncoder->getGOPSize() == 0) ? I_SLICE : eSliceType;
      }
      else
      {
        eSliceType = (pocLast == 0 || (pocCurr - (isField ? 1 : 0)) % m_pcCfg->getIntraPeriod() == 0 || m_pcGOPEncoder->getGOPSize() == 0) ? I_SLICE : eSliceType;
      }
    }

    rpcSlice->setSliceType        ( eSliceType );
  }

  if (m_pcCfg->getUseRecalculateQPAccordingToLambda())
  {
    dQP = xGetQPValueAccordingToLambda( dLambda );
    iQP = max( -rpcSlice->getSPS()->getQpBDOffset(CHANNEL_TYPE_LUMA), min( MAX_QP, (Int) floor( dQP + 0.5 ) ) );
  }

  rpcSlice->setSliceQp           ( iQP );
  rpcSlice->setSliceQpDelta      ( 0 );
#if !W0038_CQP_ADJ
  rpcSlice->setSliceChromaQpDelta( COMPONENT_Cb, 0 );
  rpcSlice->setSliceChromaQpDelta( COMPONENT_Cr, 0 );
#endif
  rpcSlice->setUseChromaQpAdj( rpcSlice->getPPS()->getPpsRangeExtension().getChromaQpOffsetListEnabledFlag() );
  rpcSlice->setNumRefIdx(REF_PIC_LIST_0,m_pcCfg->getGOPEntry(iGOPid).m_numRefPicsActive);
  rpcSlice->setNumRefIdx(REF_PIC_LIST_1,m_pcCfg->getGOPEntry(iGOPid).m_numRefPicsActive);

  if ( m_pcCfg->getDeblockingFilterMetric() )
  {
    rpcSlice->setDeblockingFilterOverrideFlag(true);
    rpcSlice->setDeblockingFilterDisable(false);
    rpcSlice->setDeblockingFilterBetaOffsetDiv2( 0 );
    rpcSlice->setDeblockingFilterTcOffsetDiv2( 0 );
  }
  else if (rpcSlice->getPPS()->getDeblockingFilterControlPresentFlag())
  {
    rpcSlice->setDeblockingFilterOverrideFlag( rpcSlice->getPPS()->getDeblockingFilterOverrideEnabledFlag() );
    rpcSlice->setDeblockingFilterDisable( rpcSlice->getPPS()->getPPSDeblockingFilterDisabledFlag() );
    if ( !rpcSlice->getDeblockingFilterDisable())
    {
      if ( rpcSlice->getDeblockingFilterOverrideFlag() && eSliceType!=I_SLICE)
      {
        rpcSlice->setDeblockingFilterBetaOffsetDiv2( m_pcCfg->getGOPEntry(iGOPid).m_betaOffsetDiv2 + m_pcCfg->getLoopFilterBetaOffset()  );
        rpcSlice->setDeblockingFilterTcOffsetDiv2( m_pcCfg->getGOPEntry(iGOPid).m_tcOffsetDiv2 + m_pcCfg->getLoopFilterTcOffset() );
      }
      else
      {
        rpcSlice->setDeblockingFilterBetaOffsetDiv2( m_pcCfg->getLoopFilterBetaOffset() );
        rpcSlice->setDeblockingFilterTcOffsetDiv2( m_pcCfg->getLoopFilterTcOffset() );
      }
    }
  }
  else
  {
    rpcSlice->setDeblockingFilterOverrideFlag( false );
    rpcSlice->setDeblockingFilterDisable( false );
    rpcSlice->setDeblockingFilterBetaOffsetDiv2( 0 );
    rpcSlice->setDeblockingFilterTcOffsetDiv2( 0 );
  }

  rpcSlice->setDepth            ( depth );

  pcPic->layer =  temporalId;
  if(eSliceType==I_SLICE)
  {
    pcPic->layer = 0;
  }
  rpcSlice->setTLayer( pcPic->layer );

  rpcSlice->setSliceMode            ( m_pcCfg->getSliceMode()            );
  rpcSlice->setSliceArgument        ( m_pcCfg->getSliceArgument()        );
  rpcSlice->setSliceSegmentMode     ( m_pcCfg->getSliceSegmentMode()     );
  rpcSlice->setSliceSegmentArgument ( m_pcCfg->getSliceSegmentArgument() );
  rpcSlice->setMaxNumMergeCand      ( m_pcCfg->getMaxNumMergeCand()      );
  rpcSlice->setMaxBTSize            ( rpcSlice->isIntra() ? MAX_BT_SIZE : MAX_BT_SIZE_INTER );
}


#if SHARP_LUMA_DELTA_QP
Double EncSlice::calculateLambda( Slice* slice,
                                   const Int        GOPid, // entry in the GOP table
                                   const Int        depth, // slice GOP hierarchical depth.
                                   const Double     refQP, // initial slice-level QP
                                   const Double     dQP,   // initial double-precision QP
                                         Int       &iQP )  // returned integer QP.
{
  enum   SliceType eSliceType    = slice->getSliceType();
  const  Bool      isField       = slice->getPic()->fieldPic;
  const  Int       NumberBFrames = ( m_pcCfg->getGOPSize() - 1 );
  const  Int       SHIFT_QP      = 12;
#if X0038_LAMBDA_FROM_QP_CAPABILITY
  const Int temporalId=m_pcCfg->getGOPEntry(GOPid).m_temporalId;
  const std::vector<Double> &intraLambdaModifiers=m_pcCfg->getIntraLambdaModifier();
#endif

#if FULL_NBIT
  Int    bitdepth_luma_qp_scale = 6 * (slice->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA) - 8);
#else
  Int    bitdepth_luma_qp_scale = 0;
#endif
  Double qp_temp = dQP + bitdepth_luma_qp_scale - SHIFT_QP;
  // Case #1: I or P-slices (key-frame)
  Double dQPFactor = m_pcCfg->getGOPEntry(GOPid).m_QPFactor;
  if ( eSliceType==I_SLICE )
  {
    if (m_pcCfg->getIntraQpFactor()>=0.0 && m_pcCfg->getGOPEntry(GOPid).m_sliceType != I_SLICE)
    {
      dQPFactor=m_pcCfg->getIntraQpFactor();
    }
    else
    {
#if X0038_LAMBDA_FROM_QP_CAPABILITY
      if(m_pcCfg->getLambdaFromQPEnable())
      {
        dQPFactor=0.57;
      }
      else
      {
#endif
        Double dLambda_scale = 1.0 - Clip3( 0.0, 0.5, 0.05*(Double)(isField ? NumberBFrames/2 : NumberBFrames) );
        dQPFactor=0.57*dLambda_scale;
#if X0038_LAMBDA_FROM_QP_CAPABILITY
      }
#endif
    }
  }
#if X0038_LAMBDA_FROM_QP_CAPABILITY
  else if( m_pcCfg->getLambdaFromQPEnable() )
  {
    dQPFactor=0.57;
  }
#endif

  Double dLambda = dQPFactor*pow( 2.0, qp_temp/3.0 );

#if X0038_LAMBDA_FROM_QP_CAPABILITY
  if( !(m_pcCfg->getLambdaFromQPEnable()) && depth>0 )
#else
  if ( depth>0 )
#endif
  {
#if FULL_NBIT
      Double qp_temp_ref_orig = refQP - SHIFT_QP;
      dLambda *= Clip3( 2.00, 4.00, (qp_temp_ref_orig / 6.0) ); // (j == B_SLICE && p_cur_frm->layer != 0 )
#else
      Double qp_temp_ref = refQP + bitdepth_luma_qp_scale - SHIFT_QP;
      dLambda *= Clip3( 2.00, 4.00, (qp_temp_ref / 6.0) ); // (j == B_SLICE && p_cur_frm->layer != 0 )
#endif
  }

  // if hadamard is used in ME process
  if ( !m_pcCfg->getUseHADME() && slice->getSliceType( ) != I_SLICE )
  {
    dLambda *= 0.95;
  }

#if X0038_LAMBDA_FROM_QP_CAPABILITY
  Double lambdaModifier;
  if( eSliceType != I_SLICE || intraLambdaModifiers.empty())
  {
    lambdaModifier = m_pcCfg->getLambdaModifier( temporalId );
  }
  else
  {
    lambdaModifier = intraLambdaModifiers[ (temporalId < intraLambdaModifiers.size()) ? temporalId : (intraLambdaModifiers.size()-1) ];
  }
  dLambda *= lambdaModifier;
#endif

  iQP = max( -slice->getSPS()->getQpBDOffset(CHANNEL_TYPE_LUMA), min( MAX_QP, (Int) floor( dQP + 0.5 ) ) );

  // NOTE: the lambda modifiers that are sometimes applied later might be best always applied in here.
  return dLambda;
}
#endif

Void EncSlice::resetQP( Picture* pic, Int sliceQP, Double lambda )
{
  Slice* slice = pic->slices[0];

  // store lambda
  slice->setSliceQp( sliceQP );
  setUpLambda(slice, lambda, sliceQP);
}

#if HHI_HLM_USE_QPA
static inline Int apprI2Log2 (const double d)
{
  return d < 6.0e-20 ? -128 : Int(floor(2.0 * log(d) / log(2.0) + 0.5));
}
#endif


#if HHI_HLM_USE_QPA
#ifndef HLM_L1_NORM
  #define HLM_L1_NORM
#endif

static Int filterAndCalculateAverageEnergies (const Pel* pSrc,     const Int  iSrcStride,
                                              double &hpEner,      const Int  iHeight,   const Int iWidth,
                                              const Int  iPOC = 0, const bool bBypass = false)
{
  Int iHpValue;
  UInt uHpERow, uHpEner = 0;
  if (bBypass) {
    hpEner = 1.0;
    return 0;
  }
  // skip first row as there may be a black border frame
  pSrc += iSrcStride;
  // center rows
  for (Int y = 1; y < iHeight - 1; y++)
  {
    uHpERow = 0;
    // skip column as there may be a black border frame

    for (Int x = 1; x < iWidth - 1; x++) // and columns
    {
      iHpValue = 4 * (Int)pSrc[x] - (Int)pSrc[x-1] - (Int)pSrc[x+1] - (Int)pSrc[x-iSrcStride] - (Int)pSrc[x+iSrcStride];
#ifdef HLM_L1_NORM
      uHpERow += abs(iHpValue);
#else
      uHpERow += iHpValue * iHpValue;
#endif
    }
    // skip column as there may be a black border frame
#ifdef HLM_L1_NORM
    uHpEner += uHpERow;
#else
    uHpEner += (uHpERow + 64) >> 7; // avoids overflows
#endif
    pSrc += iSrcStride;
  }
  // skip last row as there may be a black border frame

  hpEner = double(uHpEner) / double((iWidth - 2) * (iHeight - 2));
  if (hpEner < 1.0) hpEner = 1.0;
#ifdef HLM_L1_NORM
    hpEner *= hpEner;
#endif
  if (iPOC  <= 0) return 0;
  return 1; // OK
}

#ifdef HLM_L1_NORM
  #undef HLM_L1_NORM
#endif
#endif

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

//! set adaptive search range based on poc difference
Void EncSlice::setSearchRange( Slice* pcSlice )
{
  Int iCurrPOC = pcSlice->getPOC();
  Int iRefPOC;
  Int iGOPSize = m_pcCfg->getGOPSize();
  Int iOffset = (iGOPSize >> 1);
  Int iMaxSR = m_pcCfg->getSearchRange();
  Int iNumPredDir = pcSlice->isInterP() ? 1 : 2;

  for (Int iDir = 0; iDir < iNumPredDir; iDir++)
  {
    RefPicList  e = ( iDir ? REF_PIC_LIST_1 : REF_PIC_LIST_0 );
    for (Int iRefIdx = 0; iRefIdx < pcSlice->getNumRefIdx(e); iRefIdx++)
    {
      iRefPOC = pcSlice->getRefPic(e, iRefIdx)->getPOC();
      Int newSearchRange = Clip3(m_pcCfg->getMinSearchWindow(), iMaxSR, (iMaxSR*ADAPT_SR_SCALE*abs(iCurrPOC - iRefPOC)+iOffset)/iGOPSize);
      m_pcInterSearch->setAdaptiveSearchRange(iDir, iRefIdx, newSearchRange);
    }
  }
}

/**
 Multi-loop slice encoding for different slice QP

 \param pcPic    picture class
 */
Void EncSlice::precompressSlice( Picture* pcPic )
{
  // if deltaQP RD is not used, simply return
  if ( m_pcCfg->getDeltaQpRD() == 0 )
  {
    return;
  }

  if ( m_pcCfg->getUseRateCtrl() )
  {
    THROW("\nMultiple QP optimization is not allowed when rate control is enabled." );
  }

  Slice* pcSlice        = pcPic->slices[getSliceSegmentIdx()];

  if (pcSlice->getDependentSliceSegmentFlag())
  {
    // if this is a dependent slice segment, then it was optimised
    // when analysing the entire slice.
    return;
  }

  if (pcSlice->getSliceMode()==FIXED_NUMBER_OF_BYTES)
  {
    // TODO: investigate use of average cost per CTU so that this Slice Mode can be used.
    THROW( "Unable to optimise Slice-level QP if Slice Mode is set to FIXED_NUMBER_OF_BYTES\n" );
  }

  Double     dPicRdCostBest = MAX_DOUBLE;
  UInt       uiQpIdxBest = 0;

  Double dFrameLambda;
#if FULL_NBIT
  Int    SHIFT_QP = 12 + 6 * (pcSlice->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA) - 8);
#else
  Int    SHIFT_QP = 12;
#endif

  // set frame lambda
  if (m_pcCfg->getGOPSize() > 1)
  {
    dFrameLambda = 0.68 * pow (2, (m_viRdPicQp[0]  - SHIFT_QP) / 3.0) * (pcSlice->isInterB()? 2 : 1);
  }
  else
  {
    dFrameLambda = 0.68 * pow (2, (m_viRdPicQp[0] - SHIFT_QP) / 3.0);
  }

  // for each QP candidate
  for ( UInt uiQpIdx = 0; uiQpIdx < 2 * m_pcCfg->getDeltaQpRD() + 1; uiQpIdx++ )
  {
    pcSlice       ->setSliceQp             ( m_viRdPicQp    [uiQpIdx] );
    setUpLambda(pcSlice, m_vdRdPicLambda[uiQpIdx], m_viRdPicQp    [uiQpIdx]);

    // try compress
    compressSlice   ( pcPic, true, m_pcCfg->getFastDeltaQp());

    UInt64 uiPicDist        = m_uiPicDist; // Distortion, as calculated by compressSlice.
    // NOTE: This distortion is the chroma-weighted SSE distortion for the slice.
    //       Previously a standard SSE distortion was calculated (for the entire frame).
    //       Which is correct?
#if W0038_DB_OPT
    // TODO: Update loop filter, SAO and distortion calculation to work on one slice only.
    // uiPicDist = m_pcGOPEncoder->preLoopFilterPicAndCalcDist( pcPic );
#endif
    // compute RD cost and choose the best
    double dPicRdCost = double( uiPicDist ) + dFrameLambda * double( m_uiPicTotalBits );

    if ( dPicRdCost < dPicRdCostBest )
    {
      uiQpIdxBest    = uiQpIdx;
      dPicRdCostBest = dPicRdCost;
    }
  }

  // set best values
  pcSlice       ->setSliceQp             ( m_viRdPicQp    [uiQpIdxBest] );
  setUpLambda(pcSlice, m_vdRdPicLambda[uiQpIdxBest], m_viRdPicQp    [uiQpIdxBest]);
}

Void EncSlice::calCostSliceI(Picture* pcPic) // TODO: this only analyses the first slice segment. What about the others?
{
  Double         iSumHadSlice      = 0;
  Slice * const  pcSlice           = pcPic->slices[getSliceSegmentIdx()];
  const TileMap &tileMap           = *pcPic->tileMap;
  const PreCalcValues& pcv         = *pcPic->cs->pcv;
  const SPS     &sps               = *(pcSlice->getSPS());
  const Int      shift             = sps.getBitDepth(CHANNEL_TYPE_LUMA)-8;
  const Int      offset            = (shift>0)?(1<<(shift-1)):0;

  pcSlice->setSliceSegmentBits(0);

  UInt startCtuTsAddr, boundingCtuTsAddr;
  xDetermineStartAndBoundingCtuTsAddr ( startCtuTsAddr, boundingCtuTsAddr, pcPic );

  for( UInt ctuTsAddr = startCtuTsAddr, ctuRsAddr = tileMap.getCtuTsToRsAddrMap( startCtuTsAddr);
       ctuTsAddr < boundingCtuTsAddr;
       ctuRsAddr = tileMap.getCtuTsToRsAddrMap(++ctuTsAddr) )
  {
    Position pos( (ctuRsAddr % pcv.widthInCtus) * pcv.maxCUWidth, (ctuRsAddr / pcv.widthInCtus) * pcv.maxCUHeight);

    const Int height  = std::min( pcv.maxCUHeight, pcv.lumaHeight - pos.y );
    const Int width   = std::min( pcv.maxCUWidth,  pcv.lumaWidth  - pos.x );
    const CompArea blk( COMPONENT_Y, pcv.chrFormat, pos, Size( width, height));
    Int iSumHad = m_pcCuEncoder->updateCtuDataISlice( pcPic->getOrigBuf( blk ) );

    (m_pcRateCtrl->getRCPic()->getLCU(ctuRsAddr)).m_costIntra=(iSumHad+offset)>>shift;
    iSumHadSlice += (m_pcRateCtrl->getRCPic()->getLCU(ctuRsAddr)).m_costIntra;

  }
  m_pcRateCtrl->getRCPic()->setTotalIntraCost(iSumHadSlice);
}

/** \param pcPic   picture class
 */
Void EncSlice::compressSlice( Picture* pcPic, const Bool bCompressEntireSlice, const Bool bFastDeltaQP )
{
  // if bCompressEntireSlice is true, then the entire slice (not slice segment) is compressed,
  //   effectively disabling the slice-segment-mode.

  Slice* const pcSlice    = pcPic->slices[getSliceSegmentIdx()];
  const TileMap&  tileMap = *pcPic->tileMap;
  UInt  startCtuTsAddr;
  UInt  boundingCtuTsAddr;
#if HHI_HLM_USE_QPA
  double hpEnerPic = 0.0;
  Int   iSrcOffset;
  const Int iQPThresh = m_pcCfg->getGOPSize() >> 1;
#endif

  pcSlice->setSliceSegmentBits(0);
  xDetermineStartAndBoundingCtuTsAddr ( startCtuTsAddr, boundingCtuTsAddr, pcPic );
  if (bCompressEntireSlice)
  {
    boundingCtuTsAddr = pcSlice->getSliceCurEndCtuTsAddr();
    pcSlice->setSliceSegmentCurEndCtuTsAddr(boundingCtuTsAddr);
  }

  // initialize cost values - these are used by precompressSlice (they should be parameters).
  m_uiPicTotalBits  = 0;
  m_uiPicDist       = 0;

  pcSlice->setSliceQpBase( pcSlice->getSliceQp() );

  m_CABACEncoder->updateBufferState(pcSlice);
  m_CABACEncoder->setSliceWinUpdateMode(pcSlice);

  m_CABACEstimator->initCtxModels( *pcSlice, m_CABACEncoder );

  m_pcCuEncoder->getModeCtrl()->setFastDeltaQp(bFastDeltaQP);
  m_pcCuEncoder->getModeCtrl()->initSlice( *pcSlice );

  //------------------------------------------------------------------------------
  //  Weighted Prediction parameters estimation.
  //------------------------------------------------------------------------------
  // calculate AC/DC values for current picture
  if( pcSlice->getPPS()->getUseWP() || pcSlice->getPPS()->getWPBiPred() )
  {
    xCalcACDCParamSlice(pcSlice);
  }

  const Bool bWp_explicit = (pcSlice->getSliceType()==P_SLICE && pcSlice->getPPS()->getUseWP()) || (pcSlice->getSliceType()==B_SLICE && pcSlice->getPPS()->getWPBiPred());

  if ( bWp_explicit )
  {
    //------------------------------------------------------------------------------
    //  Weighted Prediction implemented at Slice level. SliceMode=2 is not supported yet.
    //------------------------------------------------------------------------------
    if ( pcSlice->getSliceMode()==FIXED_NUMBER_OF_BYTES || pcSlice->getSliceSegmentMode()==FIXED_NUMBER_OF_BYTES )
    {
      EXIT("Weighted Prediction is not yet supported with slice mode determined by max number of bins.");
    }

    xEstimateWPParamSlice( pcSlice, m_pcCfg->getWeightedPredictionMethod() );
    pcSlice->initWpScaling(pcSlice->getSPS());

    // check WP on/off
    xCheckWPEnable( pcSlice );
  }

  if( pcSlice->getSliceSegmentIdx() == 0 )
  {
    pcSlice->setUseLICOnPicLevel( m_pcCfg->getFastPicLevelLIC() );
  }

  // Adjust initial state if this is the start of a dependent slice.
  {
    const UInt      ctuRsAddr               = tileMap.getCtuTsToRsAddrMap( startCtuTsAddr);
    const UInt      currentTileIdx          = tileMap.getTileIdxMap(ctuRsAddr);
    const Tile&     currentTile             = tileMap.tiles[currentTileIdx];
    const UInt      firstCtuRsAddrOfTile    = currentTile.getFirstCtuRsAddr();
    if( pcSlice->getDependentSliceSegmentFlag() && ctuRsAddr != firstCtuRsAddrOfTile )
    {
      // This will only occur if dependent slice-segments (m_entropyCodingSyncContextState=true) are being used.
      if( currentTile.getTileWidthInCtus() >= 2 || !m_pcCfg->getEntropyCodingSyncEnabledFlag() )
      {
        m_CABACEstimator->getCtx() = m_lastSliceSegmentEndContextState;
        m_CABACEstimator->start();
      }
    }
  }

  if(!pcSlice->getDependentSliceSegmentFlag())
  {
    pcPic->setPrevQP(pcSlice->getSliceQp());
  }
  CHECK(!(pcPic->getPrevQP() != std::numeric_limits<Int>::max()), "Unspecified error");


  CodingStructure&  cs          = *pcPic->cs;
  const PreCalcValues& pcv      = *cs.pcv;
  const UInt        widthInCtus = pcv.widthInCtus;

  cs.slice = pcSlice;
  if (startCtuTsAddr == 0) {
    cs.initStructData (pcSlice->getSliceQp(), pcSlice->getPPS()->getTransquantBypassEnabledFlag());
  }

  const Int iQPIndex = pcSlice->getSliceQp(); // initial QP index for current slice, used in following loops
  pcSlice->setSliceQpBase( iQPIndex );

#if HHI_HLM_USE_QPA
#if HHI_HLM_USE_QPA
  if (m_pcCfg->getUsePerceptQPA() && pcSlice->getPPS()->getUseDQP() && !m_pcCfg->getUseRateCtrl())
#endif
  {
    for (UInt ctuTsAddr = startCtuTsAddr; ctuTsAddr < boundingCtuTsAddr; ++ctuTsAddr)
    {
      const UInt     ctuRsAddr  = tileMap.getCtuTsToRsAddrMap(ctuTsAddr);
      const Position pos((ctuRsAddr % widthInCtus) * pcv.maxCUWidth, (ctuRsAddr / widthInCtus) * pcv.maxCUHeight);
      const CompArea subArea    = clipArea (CompArea (COMPONENT_Y, pcPic->chromaFormat, Area (pos.x, pos.y, pcv.maxCUWidth, pcv.maxCUHeight)), pcPic->Y());
      const CompArea fltArea    = clipArea (CompArea (COMPONENT_Y, pcPic->chromaFormat, Area (pos.x > 0 ? pos.x - 1 : 0, pos.y > 0 ? pos.y - 1 : 0, pcv.maxCUWidth + (pos.x > 0 ? 2 : 1), pcv.maxCUHeight + (pos.y > 0 ? 2 : 1))), pcPic->Y());
      const SizeType iSrcStride = pcPic->getOrigBuf (subArea).stride;
      const SizeType iFltHeight = pcPic->getOrigBuf (fltArea).height;
      const SizeType iFltWidth  = pcPic->getOrigBuf (fltArea).width;
      const UInt     uiCUIndex  = ctuTsAddr - startCtuTsAddr;
      double hpEner = 0.0;
      DTRACE_UPDATE (g_trace_ctx, std::make_pair ("ctu", ctuRsAddr));

      filterAndCalculateAverageEnergies (pcPic->getOrigBuf(fltArea).buf, iSrcStride,
                                         hpEner, iFltHeight, iFltWidth, pcPic->getPOC(),
#if X0038_LAMBDA_FROM_QP_CAPABILITY
                                         !pcSlice->isIntra() && (iQPIndex >= m_pcCfg->getQPForPicture(0, pcSlice) + iQPThresh));
#else
                                         !pcSlice->isIntra() && (iQPIndex >= m_pcCfg->getQP() + iQPThresh));
#endif

#if HHI_HLM_USE_QPA
      m_uEnerHpCtu[uiCUIndex] = hpEner;
      hpEnerPic += hpEner;
#endif

    } // end iteration over all CUs in current slice
  }

#if HHI_HLM_USE_QPA
  if (m_pcCfg->getUsePerceptQPA() && pcSlice->getPPS()->getUseDQP() && !m_pcCfg->getUseRateCtrl() && (boundingCtuTsAddr > startCtuTsAddr))
  {
    const double factor = 1.0 * (boundingCtuTsAddr - startCtuTsAddr);  // average across the current picture
    hpEnerPic  = factor / hpEnerPic; // speedup: multiply instead of divide below (1.0 for rate fine-tuning)
  }
#endif
#endif

  // for every CTU in the slice segment (may terminate sooner if there is a byte limit on the slice-segment)

  for( UInt ctuTsAddr = startCtuTsAddr; ctuTsAddr < boundingCtuTsAddr; ++ctuTsAddr )
  {
    const UInt ctuRsAddr = tileMap.getCtuTsToRsAddrMap(ctuTsAddr);

    // update CABAC state
    const UInt firstCtuRsAddrOfTile = tileMap.tiles[tileMap.getTileIdxMap(ctuRsAddr)].getFirstCtuRsAddr();
    const UInt tileXPosInCtus       = firstCtuRsAddrOfTile % widthInCtus;
    const UInt ctuXPosInCtus        = ctuRsAddr % widthInCtus;
    const UInt ctuYPosInCtus        = ctuRsAddr / widthInCtus;

    const Position pos (ctuXPosInCtus * pcv.maxCUWidth, ctuYPosInCtus * pcv.maxCUHeight);
    const UnitArea ctuArea (cs.area.chromaFormat, Area(pos.x, pos.y, pcv.maxCUWidth, pcv.maxCUHeight));

    DTRACE_UPDATE( g_trace_ctx, std::make_pair( "ctu", ctuRsAddr ) );

    if (ctuRsAddr == firstCtuRsAddrOfTile)
    {
      m_CABACEstimator->initCtxModels( *pcSlice, m_CABACEncoder );
      pcPic->setPrevQP(pcSlice->getSliceQp());
    }
    else if (ctuXPosInCtus == tileXPosInCtus && m_pcCfg->getEntropyCodingSyncEnabledFlag())
    {
      // reset and then update contexts to the state at the end of the top-right CTU (if within current slice and tile).
      m_CABACEstimator->initCtxModels( *pcSlice, m_CABACEncoder );
      if( cs.getCURestricted( pos.offset(pcv.maxCUWidth, -1), pcSlice->getIndependentSliceIdx(), tileMap.getTileIdxMap( pos ) ) )
      {
        // Top-right is available, we use it.
        m_CABACEstimator->getCtx() = m_entropyCodingSyncContextState;
      }
      pcPic->setPrevQP(pcSlice->getSliceQp());
    }

    // load CABAC context from previous frame
    if( ctuRsAddr == 0 )
    {
      m_CABACEncoder->loadCtxStates( pcSlice, m_CABACEstimator->getCtx() );
    }

    const Double oldLambda = m_pcRdCost->getLambda();
    if ( m_pcCfg->getUseRateCtrl() )
    {
      Int estQP        = pcSlice->getSliceQp();
      Double estLambda = -1.0;
      Double bpp       = -1.0;

      if( ( pcPic->slices[0]->getSliceType() == I_SLICE && m_pcCfg->getForceIntraQP() ) || !m_pcCfg->getLCULevelRC() )
      {
        estQP = pcSlice->getSliceQp();
      }
      else
      {
        bpp = m_pcRateCtrl->getRCPic()->getLCUTargetBpp(pcSlice->getSliceType());
        if ( pcPic->slices[0]->getSliceType() == I_SLICE)
        {
          estLambda = m_pcRateCtrl->getRCPic()->getLCUEstLambdaAndQP(bpp, pcSlice->getSliceQp(), &estQP);
        }
        else
        {
          estLambda = m_pcRateCtrl->getRCPic()->getLCUEstLambda( bpp );
          estQP     = m_pcRateCtrl->getRCPic()->getLCUEstQP    ( estLambda, pcSlice->getSliceQp() );
        }

        estQP     = Clip3( -pcSlice->getSPS()->getQpBDOffset(CHANNEL_TYPE_LUMA), MAX_QP, estQP );

        m_pcRdCost->setLambda(estLambda, pcSlice->getSPS()->getBitDepths());

#if RDOQ_CHROMA_LAMBDA
        // set lambda for RDOQ
        const Double chromaLambda = estLambda / m_pcRdCost->getChromaWeight();
        const Double lambdaArray[MAX_NUM_COMPONENT] = { estLambda, chromaLambda, chromaLambda };
        m_pcTrQuant->setLambdas( lambdaArray );
#else
        m_pcTrQuant->setLambda( estLambda );
#endif
      }

      m_pcRateCtrl->setRCQP( estQP );
    }
#if HHI_HLM_USE_QPA
    else if (m_pcCfg->getUsePerceptQPA() && pcSlice->getPPS()->getUseDQP())
    {
#if X0038_LAMBDA_FROM_QP_CAPABILITY
      if (!pcSlice->isIntra() && (iQPIndex >= m_pcCfg->getQPForPicture (0, pcSlice) + iQPThresh))
#else
      if (!pcSlice->isIntra() && (iQPIndex >= m_pcCfg->getQP() + iQPThresh))
#endif
      {
        pcSlice->setSliceQp (iQPIndex); // restore initial QP index and lambda values for the slice
#if RDOQ_CHROMA_LAMBDA
        const Double chromaLambda = oldLambda / m_pcRdCost->getChromaWeight();
        const Double lambdaArray[MAX_NUM_COMPONENT] = {oldLambda, chromaLambda, chromaLambda};
        m_pcTrQuant->setLambdas (lambdaArray);
#else
        m_pcTrQuant->setLambda (oldLambda);
#endif
        m_pcRdCost->setLambda (oldLambda, pcSlice->getSPS()->getBitDepths());
      }
      else { // apply CTU-wise perceptually motivated QP modification based on the luma input image
        const UInt uiCUIndex = ctuTsAddr - startCtuTsAddr;
        iSrcOffset = Clip3(0, MAX_QP, iQPIndex + apprI2Log2(m_uEnerHpCtu[uiCUIndex] * hpEnerPic));
        if (ctuTsAddr == 0) {
          cs.currQP = iSrcOffset; // avoid mismatch, see if (startCtuTsAddr == 0) cs.initStructData
        }
        pcSlice->setSliceQp (iSrcOffset); // update actual QP index and lambda values for the slice
        const Double newLambda = oldLambda * pow (2.0, Double(iSrcOffset - iQPIndex) / 3.0);
#if RDOQ_CHROMA_LAMBDA
        const Double chromaLambda = newLambda / m_pcRdCost->getChromaWeight();
        const Double lambdaArray[MAX_NUM_COMPONENT] = {newLambda, chromaLambda, chromaLambda};
        m_pcTrQuant->setLambdas (lambdaArray);
#else
        m_pcTrQuant->setLambda (newLambda);
#endif
        m_pcRdCost->setLambda (newLambda, pcSlice->getSPS()->getBitDepths());
      }
    }
#endif

    //////////////////////////////////////////////////////////////////////////
    // CTU estimation
    //////////////////////////////////////////////////////////////////////////

    cs.pcv    = pcSlice->getPPS()->pcv;
    cs.prevQP = pcPic->getPrevQP();
    cs.fracBits = 0;

    if( pcSlice->getSPS()->getSpsNext().getUseFRUCMrgMode() && !pcSlice->isIntra() )
    {
      CS::initFrucMvp( cs );
    }


    m_pcCuEncoder->compressCtu( cs, ctuArea, ctuRsAddr );
    m_CABACEstimator->resetBits();
    m_CABACEstimator->coding_tree_unit( cs, ctuArea, pcPic->getPrevQP(), ctuRsAddr, true );
    const int numberOfWrittenBits = int( m_CABACEstimator->getEstFracBits() >> SCALE_BITS );

    // Calculate if this CTU puts us over slice bit size.
    // cannot terminate if current slice/slice-segment would be 0 Ctu in size,
    const UInt validEndOfSliceCtuTsAddr = ctuTsAddr + (ctuTsAddr == startCtuTsAddr ? 1 : 0);
    // Set slice end parameter
    if(pcSlice->getSliceMode()==FIXED_NUMBER_OF_BYTES && pcSlice->getSliceBits()+numberOfWrittenBits > (pcSlice->getSliceArgument()<<3))
    {
      pcSlice->setSliceSegmentCurEndCtuTsAddr(validEndOfSliceCtuTsAddr);
      pcSlice->setSliceCurEndCtuTsAddr(validEndOfSliceCtuTsAddr);
      boundingCtuTsAddr=validEndOfSliceCtuTsAddr;
    }
    else if((!bCompressEntireSlice) && pcSlice->getSliceSegmentMode()==FIXED_NUMBER_OF_BYTES && pcSlice->getSliceSegmentBits()+numberOfWrittenBits > (pcSlice->getSliceSegmentArgument()<<3))
    {
      pcSlice->setSliceSegmentCurEndCtuTsAddr(validEndOfSliceCtuTsAddr);
      boundingCtuTsAddr=validEndOfSliceCtuTsAddr;
    }

    if (boundingCtuTsAddr <= ctuTsAddr)
    {
      break;
    }

    pcSlice->setSliceBits( (UInt)(pcSlice->getSliceBits() + numberOfWrittenBits) );
    pcSlice->setSliceSegmentBits(pcSlice->getSliceSegmentBits()+numberOfWrittenBits);

    // Store probabilities of second CTU in line into buffer - used only if wavefront-parallel-processing is enabled.
    if( ctuXPosInCtus == tileXPosInCtus + 1 && m_pcCfg->getEntropyCodingSyncEnabledFlag() )
    {
      m_entropyCodingSyncContextState = m_CABACEstimator->getCtx();
    }


    Int actualBits = int(cs.fracBits >> SCALE_BITS);
    if ( m_pcCfg->getUseRateCtrl() )
    {
      Int actualQP        = g_RCInvalidQPValue;
      Double actualLambda = m_pcRdCost->getLambda();
      Int numberOfEffectivePixels    = 0;

      for( auto &cu : cs.traverseCUs( ctuArea ) )
      {
        if( !cu.skip || cu.rootCbf )
        {
          numberOfEffectivePixels += cu.lumaSize().area();
          break;
        }
      }

      CodingUnit* cu = cs.getCU( ctuArea.lumaPos() );

      if ( numberOfEffectivePixels == 0 )
      {
        actualQP = g_RCInvalidQPValue;
      }
      else
      {
        actualQP = cu->qp;
      }
      m_pcRdCost->setLambda(oldLambda, pcSlice->getSPS()->getBitDepths());
      m_pcRateCtrl->getRCPic()->updateAfterCTU( m_pcRateCtrl->getRCPic()->getLCUCoded(), actualBits, actualQP, actualLambda,
                                                pcSlice->getSliceType() == I_SLICE ? 0 : m_pcCfg->getLCULevelRC() );
    }
#if HHI_HLM_USE_QPA
    else if (m_pcCfg->getUsePerceptQPA() && pcSlice->getPPS()->getUseDQP())
    {
      pcSlice->setSliceQp (iQPIndex);  // restore initial QP index and lambda values for this slice
#if RDOQ_CHROMA_LAMBDA
      const Double chromaLambda = oldLambda / m_pcRdCost->getChromaWeight();
      const Double lambdaArray[MAX_NUM_COMPONENT] = {oldLambda, chromaLambda, chromaLambda};
      m_pcTrQuant->setLambdas (lambdaArray);
#else
      m_pcTrQuant->setLambda (oldLambda);
#endif
      m_pcRdCost->setLambda (oldLambda, pcSlice->getSPS()->getBitDepths());
    }
#endif

    m_uiPicTotalBits += actualBits;
    m_uiPicDist       = cs.dist;
  }

  // store context state at the end of this slice-segment, in case the next slice is a dependent slice and continues using the CABAC contexts.
  if( pcSlice->getPPS()->getDependentSliceSegmentsEnabledFlag() )
  {
    m_lastSliceSegmentEndContextState = m_CABACEstimator->getCtx();//ctx end of dep.slice
  }

}

Void EncSlice::encodeSlice   ( Picture* pcPic, OutputBitstream* pcSubstreams, UInt &numBinsCoded )
{
  Slice *const pcSlice               = pcPic->slices[getSliceSegmentIdx()];
  const TileMap& tileMap             = *pcPic->tileMap;
  const UInt startCtuTsAddr          = pcSlice->getSliceSegmentCurStartCtuTsAddr();
  const UInt boundingCtuTsAddr       = pcSlice->getSliceSegmentCurEndCtuTsAddr();
  const Bool depSliceSegmentsEnabled = pcSlice->getPPS()->getDependentSliceSegmentsEnabledFlag();
  const Bool wavefrontsEnabled       = pcSlice->getPPS()->getEntropyCodingSyncEnabledFlag();

  m_CABACWriter->enableBinStore( *pcSlice, *m_CABACEncoder );

  // setup coding structure
  CodingStructure& cs = *pcPic->cs;
  cs.slice            = pcSlice;
  // initialise entropy coder for the slice
  m_CABACWriter->initCtxModels( *pcSlice, m_CABACEncoder );

  DTRACE( g_trace_ctx, D_HEADER, "=========== POC: %d ===========\n", pcSlice->getPOC() );

  if (depSliceSegmentsEnabled)
  {
    // modify initial contexts with previous slice segment if this is a dependent slice.
    const UInt ctuRsAddr            = tileMap.getCtuTsToRsAddrMap( startCtuTsAddr );
    const UInt currentTileIdx       = tileMap.getTileIdxMap(ctuRsAddr);
    const Tile& currentTile         = tileMap.tiles[currentTileIdx];
    const UInt firstCtuRsAddrOfTile = currentTile.getFirstCtuRsAddr();

    if( pcSlice->getDependentSliceSegmentFlag() && ctuRsAddr != firstCtuRsAddrOfTile )
    {
      if( currentTile.getTileWidthInCtus() >= 2 || !wavefrontsEnabled )
      {
        m_CABACWriter->getCtx() = m_lastSliceSegmentEndContextState;
      }
    }
  }

  if( !pcSlice->getDependentSliceSegmentFlag())
  {
    pcPic->setPrevQP(pcSlice->getSliceQp());
    cs.prevQP = pcSlice->getSliceQp();
  }

  const PreCalcValues& pcv = *cs.pcv;
  const UInt widthInCtus   = pcv.widthInCtus;

  // for every CTU in the slice segment...

  for( UInt ctuTsAddr = startCtuTsAddr; ctuTsAddr < boundingCtuTsAddr; ++ctuTsAddr )
  {
    const UInt ctuRsAddr            = tileMap.getCtuTsToRsAddrMap(ctuTsAddr);
    const Tile& currentTile         = tileMap.tiles[tileMap.getTileIdxMap(ctuRsAddr)];
    const UInt firstCtuRsAddrOfTile = currentTile.getFirstCtuRsAddr();
    const UInt tileXPosInCtus       = firstCtuRsAddrOfTile % widthInCtus;
    const UInt tileYPosInCtus       = firstCtuRsAddrOfTile / widthInCtus;
    const UInt ctuXPosInCtus        = ctuRsAddr % widthInCtus;
    const UInt ctuYPosInCtus        = ctuRsAddr / widthInCtus;
    const UInt uiSubStrm            = tileMap.getSubstreamForCtuAddr(ctuRsAddr, true, pcSlice);

    DTRACE_UPDATE( g_trace_ctx, std::make_pair( "ctu", ctuRsAddr ) );

    const Position pos (ctuXPosInCtus * pcv.maxCUWidth, ctuYPosInCtus * pcv.maxCUHeight);
    const UnitArea ctuArea (cs.area.chromaFormat, Area(pos.x, pos.y, pcv.maxCUWidth, pcv.maxCUHeight));

    m_CABACWriter->initBitstream( &pcSubstreams[uiSubStrm] );

    // set up CABAC contexts' state for this CTU
    if (ctuRsAddr == firstCtuRsAddrOfTile)
    {
      if (ctuTsAddr != startCtuTsAddr) // if it is the first CTU, then the entropy coder has already been reset
      {
        m_CABACWriter->initCtxModels( *pcSlice, m_CABACEncoder );
      }
    }
    else if (ctuXPosInCtus == tileXPosInCtus && wavefrontsEnabled)
    {
      // Synchronize cabac probabilities with upper-right CTU if it's available and at the start of a line.
      if (ctuTsAddr != startCtuTsAddr) // if it is the first CTU, then the entropy coder has already been reset
      {
        m_CABACWriter->initCtxModels( *pcSlice, m_CABACEncoder );
      }
      if( cs.getCURestricted( pos.offset(pcv.maxCUWidth, -1), pcSlice->getIndependentSliceIdx(), tileMap.getTileIdxMap( pos ) ) )
      {
        // Top-right is available, so use it.
        m_CABACWriter->getCtx() = m_entropyCodingSyncContextState;
      }
    }

    // load CABAC context from previous frame
    if( ctuRsAddr == 0 )
    {
      m_CABACEncoder->loadCtxStates( pcSlice, m_CABACWriter->getCtx() );
    }

    if( ctuRsAddr == 0 )
    {
      ALFParam& alfParam = cs.getALFParam();
      m_CABACWriter->alf( *pcSlice, alfParam );
    }

    m_CABACWriter->coding_tree_unit( cs, ctuArea, pcPic->getPrevQP(), ctuRsAddr );

    // store probabilities of second CTU in line into buffer
    if( ctuXPosInCtus == tileXPosInCtus + 1 && wavefrontsEnabled )
    {
      m_entropyCodingSyncContextState = m_CABACWriter->getCtx();
    }

    // store CABAC context to be used in next frames
    if ( pcSlice->getSPS()->getSpsNext().getUseCIPF() )
    {
      const unsigned storeCtuAddr = std::min<unsigned>( pcv.widthInCtus / 2 + pcv.sizeInCtus / 2, pcv.sizeInCtus - 1 );
      if ( ctuRsAddr == storeCtuAddr )
      {
        m_CABACEncoder->storeCtxStates( pcSlice, m_CABACWriter->getCtx() );
      }
    }

    // terminate the sub-stream, if required (end of slice-segment, end of tile, end of wavefront-CTU-row):
    if( ctuTsAddr + 1 == boundingCtuTsAddr ||
         (  ctuXPosInCtus + 1 == tileXPosInCtus + currentTile.getTileWidthInCtus () &&
          ( ctuYPosInCtus + 1 == tileYPosInCtus + currentTile.getTileHeightInCtus() || wavefrontsEnabled )
         )
       )
    {
      m_CABACWriter->end_of_slice();

      // Byte-alignment in slice_data() when new tile
      pcSubstreams[uiSubStrm].writeByteAlignment();

      // write sub-stream size
      if( ctuTsAddr + 1 != boundingCtuTsAddr )
      {
        pcSlice->addSubstreamSize( (pcSubstreams[uiSubStrm].getNumberOfWrittenBits() >> 3) + pcSubstreams[uiSubStrm].countStartCodeEmulations() );
      }
    }
  } // CTU-loop

  if( depSliceSegmentsEnabled )
  {
    m_lastSliceSegmentEndContextState = m_CABACWriter->getCtx();//ctx end of dep.slice
  }

  if (pcSlice->getPPS()->getCabacInitPresentFlag() && !pcSlice->getPPS()->getDependentSliceSegmentsEnabledFlag())
  {
    m_encCABACTableIdx = m_CABACWriter->getCtxInitId( *pcSlice );
  }
  else
  {
    m_encCABACTableIdx = pcSlice->getSliceType();
  }
  numBinsCoded = m_CABACWriter->getNumBins();

  m_CABACWriter->estWinSizes( *pcSlice, *m_CABACEncoder );
}

Void EncSlice::calculateBoundingCtuTsAddrForSlice(UInt &startCtuTSAddrSlice, UInt &boundingCtuTSAddrSlice, Bool &haveReachedTileBoundary,
                                                   Picture* pcPic, const Int sliceMode, const Int sliceArgument)
{
  Slice* pcSlice = pcPic->slices[getSliceSegmentIdx()];
  const TileMap& tileMap = *(pcPic->tileMap);
  const UInt numberOfCtusInFrame = pcPic->cs->pcv->sizeInCtus;
  const PPS &pps=*(pcSlice->getPPS());
  boundingCtuTSAddrSlice=0;
  haveReachedTileBoundary=false;

  switch (sliceMode)
  {
    case FIXED_NUMBER_OF_CTU:
      {
        UInt ctuAddrIncrement    = sliceArgument;
        boundingCtuTSAddrSlice  = ((startCtuTSAddrSlice + ctuAddrIncrement) < numberOfCtusInFrame) ? (startCtuTSAddrSlice + ctuAddrIncrement) : numberOfCtusInFrame;
      }
      break;
    case FIXED_NUMBER_OF_BYTES:
      boundingCtuTSAddrSlice  = numberOfCtusInFrame; // This will be adjusted later if required.
      break;
    case FIXED_NUMBER_OF_TILES:
      {
        const UInt tileIdx        = tileMap.getTileIdxMap( tileMap.getCtuTsToRsAddrMap(startCtuTSAddrSlice) );
        const UInt tileTotalCount = (pps.getNumTileColumnsMinus1()+1) * (pps.getNumTileRowsMinus1()+1);
        UInt ctuAddrIncrement   = 0;

        for(UInt tileIdxIncrement = 0; tileIdxIncrement < sliceArgument; tileIdxIncrement++)
        {
          if((tileIdx + tileIdxIncrement) < tileTotalCount)
          {
            UInt tileWidthInCtus    = tileMap.tiles[tileIdx + tileIdxIncrement].getTileWidthInCtus();
            UInt tileHeightInCtus   = tileMap.tiles[tileIdx + tileIdxIncrement].getTileHeightInCtus();
            ctuAddrIncrement       += (tileWidthInCtus * tileHeightInCtus);
          }
        }

        boundingCtuTSAddrSlice  = ((startCtuTSAddrSlice + ctuAddrIncrement) < numberOfCtusInFrame) ? (startCtuTSAddrSlice + ctuAddrIncrement) : numberOfCtusInFrame;
      }
      break;
    default:
      boundingCtuTSAddrSlice    = numberOfCtusInFrame;
      break;
  }

  // Adjust for tiles and wavefronts.
  const Bool wavefrontsAreEnabled = pps.getEntropyCodingSyncEnabledFlag();

  if ((sliceMode == FIXED_NUMBER_OF_CTU || sliceMode == FIXED_NUMBER_OF_BYTES) &&
      (pps.getNumTileRowsMinus1() > 0 || pps.getNumTileColumnsMinus1() > 0))
  {
    const UInt ctuRsAddr                   = tileMap.getCtuTsToRsAddrMap(startCtuTSAddrSlice);
    const UInt startTileIdx                = tileMap.getTileIdxMap(ctuRsAddr);
    const Tile& startingTile               = tileMap.tiles[startTileIdx];
    const UInt  tileStartTsAddr            = tileMap.getCtuRsToTsAddrMap(startingTile.getFirstCtuRsAddr());
    const UInt  tileStartWidth             = startingTile.getTileWidthInCtus();
    const UInt  tileStartHeight            = startingTile.getTileHeightInCtus();
    const UInt tileLastTsAddr_excl        = tileStartTsAddr + tileStartWidth*tileStartHeight;
    const UInt tileBoundingCtuTsAddrSlice = tileLastTsAddr_excl;
    const UInt ctuColumnOfStartingTile     = ((startCtuTSAddrSlice-tileStartTsAddr)%tileStartWidth);
    if (wavefrontsAreEnabled && ctuColumnOfStartingTile!=0)
    {
      // WPP: if a slice does not start at the beginning of a CTB row, it must end within the same CTB row
      const UInt numberOfCTUsToEndOfRow            = tileStartWidth - ctuColumnOfStartingTile;
      const UInt wavefrontTileBoundingCtuAddrSlice = startCtuTSAddrSlice + numberOfCTUsToEndOfRow;
      if (wavefrontTileBoundingCtuAddrSlice < boundingCtuTSAddrSlice)
      {
        boundingCtuTSAddrSlice = wavefrontTileBoundingCtuAddrSlice;
      }
    }

    if (tileBoundingCtuTsAddrSlice < boundingCtuTSAddrSlice)
    {
      boundingCtuTSAddrSlice = tileBoundingCtuTsAddrSlice;
      haveReachedTileBoundary = true;
    }
  }
  else if ((sliceMode == FIXED_NUMBER_OF_CTU || sliceMode == FIXED_NUMBER_OF_BYTES) && wavefrontsAreEnabled && ((startCtuTSAddrSlice % pcPic->cs->pcv->widthInCtus) != 0))
  {
    // Adjust for wavefronts (no tiles).
    // WPP: if a slice does not start at the beginning of a CTB row, it must end within the same CTB row
    boundingCtuTSAddrSlice = min(boundingCtuTSAddrSlice, startCtuTSAddrSlice - (startCtuTSAddrSlice % pcPic->cs->pcv->widthInCtus) + (pcPic->cs->pcv->widthInCtus));
  }
}

/** Determines the starting and bounding CTU address of current slice / dependent slice
 * \param [out] startCtuTsAddr
 * \param [out] boundingCtuTsAddr
 * \param [in]  pcPic

 * Updates startCtuTsAddr, boundingCtuTsAddr with appropriate CTU address
 */
Void EncSlice::xDetermineStartAndBoundingCtuTsAddr  ( UInt& startCtuTsAddr, UInt& boundingCtuTsAddr, Picture* pcPic )
{
  Slice* pcSlice                 = pcPic->slices[getSliceSegmentIdx()];

  // Non-dependent slice
  UInt startCtuTsAddrSlice           = pcSlice->getSliceCurStartCtuTsAddr();
  Bool haveReachedTileBoundarySlice  = false;
  UInt boundingCtuTsAddrSlice;
  calculateBoundingCtuTsAddrForSlice(startCtuTsAddrSlice, boundingCtuTsAddrSlice, haveReachedTileBoundarySlice, pcPic,
                                     m_pcCfg->getSliceMode(), m_pcCfg->getSliceArgument());
  pcSlice->setSliceCurEndCtuTsAddr(   boundingCtuTsAddrSlice );
  pcSlice->setSliceCurStartCtuTsAddr( startCtuTsAddrSlice    );

  // Dependent slice
  UInt startCtuTsAddrSliceSegment          = pcSlice->getSliceSegmentCurStartCtuTsAddr();
  Bool haveReachedTileBoundarySliceSegment = false;
  UInt boundingCtuTsAddrSliceSegment;
  calculateBoundingCtuTsAddrForSlice(startCtuTsAddrSliceSegment, boundingCtuTsAddrSliceSegment, haveReachedTileBoundarySliceSegment, pcPic,
                                     m_pcCfg->getSliceSegmentMode(), m_pcCfg->getSliceSegmentArgument());
  if (boundingCtuTsAddrSliceSegment>boundingCtuTsAddrSlice)
  {
    boundingCtuTsAddrSliceSegment = boundingCtuTsAddrSlice;
  }
  pcSlice->setSliceSegmentCurEndCtuTsAddr( boundingCtuTsAddrSliceSegment );
  pcSlice->setSliceSegmentCurStartCtuTsAddr(startCtuTsAddrSliceSegment);

  // Make a joint decision based on reconstruction and dependent slice bounds
  startCtuTsAddr    = max(startCtuTsAddrSlice   , startCtuTsAddrSliceSegment   );
  boundingCtuTsAddr = boundingCtuTsAddrSliceSegment;
}

Double EncSlice::xGetQPValueAccordingToLambda ( Double lambda )
{
  return 4.2005*log(lambda) + 13.7122;
}

//! \}
