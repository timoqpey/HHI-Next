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

/** \file     ContextModelling.cpp
    \brief    Classes providing probability descriptions and contexts
*/

#include "ContextModelling.h"
#include "UnitTools.h"
#include "CodingStructure.h"
#include "Picture.h"


CoeffCodingContext::CoeffCodingContext(const TransformUnit& tu, ComponentID component, bool signHide)
  : m_compID                    (component)
  , m_chType                    (toChannelType(m_compID))
  , m_width                     (tu.block(m_compID).width)
  , m_height                    (tu.block(m_compID).height)
  , m_log2CGWidth               ((m_width & 3) || (m_height & 3) ? 1 : 2)
  , m_log2CGHeight              ((m_width & 3) || (m_height & 3) ? 1 : 2)
  , m_log2CGSize                (m_log2CGWidth + m_log2CGHeight)
  , m_widthInGroups             (m_width  >> m_log2CGWidth)
  , m_heightInGroups            (m_height >> m_log2CGHeight)
  , m_log2BlockWidth            (g_aucLog2[m_width])
  , m_log2BlockHeight           (g_aucLog2[m_height])
  , m_log2WidthInGroups         (g_aucLog2[m_widthInGroups])
  , m_log2HeightInGroups        (g_aucLog2[m_heightInGroups])
  , m_log2BlockSize             ((m_log2BlockWidth + m_log2BlockHeight)>>1)
  , m_maxNumCoeff               (m_width * m_height)
  , m_AlignFlag                 (tu.cs->sps->getSpsRangeExtension().getCabacBypassAlignmentEnabledFlag())
  , m_signHiding                (signHide)
  , m_useGoRiceParAdapt         (tu.cs->sps->getSpsRangeExtension().getPersistentRiceAdaptationEnabledFlag())
  , m_extendedPrecision         (tu.cs->sps->getSpsRangeExtension().getExtendedPrecisionProcessingFlag())
  , m_maxLog2TrDynamicRange     (tu.cs->sps->getMaxLog2TrDynamicRange(m_chType))
  , m_scanType                  (CoeffScanType(TU::getCoefScanIdx( tu, m_compID)))
  , m_scan                      (g_scanOrder     [SCAN_GROUPED_4x4][m_scanType][gp_sizeIdxInfo->idxFrom(m_width        )][gp_sizeIdxInfo->idxFrom(m_height        )])
  , m_scanPosX                  (g_scanOrderPosXY[SCAN_GROUPED_4x4][m_scanType][gp_sizeIdxInfo->idxFrom(m_width        )][gp_sizeIdxInfo->idxFrom(m_height        )][0])
  , m_scanPosY                  (g_scanOrderPosXY[SCAN_GROUPED_4x4][m_scanType][gp_sizeIdxInfo->idxFrom(m_width        )][gp_sizeIdxInfo->idxFrom(m_height        )][1])
  , m_scanCG                    (g_scanOrder[SCAN_UNGROUPED  ][m_scanType][gp_sizeIdxInfo->idxFrom(m_widthInGroups)][gp_sizeIdxInfo->idxFrom(m_heightInGroups)])
  , m_CtxSetLastX               (Ctx::LastX[m_chType])
  , m_CtxSetLastY               (Ctx::LastY[m_chType])
  , m_maxLastPosX               (g_uiGroupIdx[m_width - 1])
  , m_maxLastPosY               (g_uiGroupIdx[m_height - 1])
  , m_lastOffsetX               (0)
  , m_lastOffsetY               (0)
  , m_lastShiftX                (0)
  , m_lastShiftY                (0)
  , m_TrafoBypass               (tu.cs->sps->getSpsRangeExtension().getTransformSkipContextEnabledFlag() &&  (tu.cu->transQuantBypass || tu.transformSkip[m_compID]))
  , m_SigBlockType              (m_TrafoBypass ? 0 : m_width == 4 && m_height == 4 ? 1 : m_width == 8 && m_height == 8 ? 2 : m_log2CGSize==2 ? 4 : 3 )
  , m_sigCtxSet                 (Ctx::SigFlag[m_chType])
  , m_scanPosLast               (-1)
  , m_subSetId                  (-1)
  , m_subSetPos                 (-1)
  , m_subSetPosX                (-1)
  , m_subSetPosY                (-1)
  , m_minSubPos                 (-1)
  , m_maxSubPos                 (-1)
  , m_sigGroupCtxId             (-1)
  , m_gt1FlagCtxSet             (0, 0)
  , m_gt2FlagCtxId              (-1)
  , m_currentGolombRiceStatistic(-1)
  , m_prevGt2                   (false)
  , m_sigCoeffGroupFlag         ()
  , m_altResiCompId             ( tu.cs->sps->getSpsNext().getAltResiCompId() )
  , m_emtNumSigCoeff            (0)
{
  // LOGTODO
  unsigned log2sizeX = m_log2BlockWidth;
  unsigned log2sizeY = m_log2BlockHeight;
  if (m_scanType == SCAN_VER)
  {
    std::swap(log2sizeX, log2sizeY);
    std::swap(const_cast<unsigned&>(m_maxLastPosX), const_cast<unsigned&>(m_maxLastPosY));
  }
  if (m_chType == CHANNEL_TYPE_CHROMA)
  {
    if( tu.cs->pcv->rectCUs )
    {
      const_cast<int&>(m_lastShiftX) = Clip3( 0, 2, int( ( m_scanType == SCAN_VER ? m_height : m_width  ) >> 3) );
      const_cast<int&>(m_lastShiftY) = Clip3( 0, 2, int( ( m_scanType == SCAN_VER ? m_width  : m_height ) >> 3) );
    }
    else
    {
      const_cast<int&>(m_lastShiftX) = log2sizeX - 2;
      const_cast<int&>(m_lastShiftY) = log2sizeY - 2;
    }
  }
  else
  {
    if( tu.cs->pcv->rectCUs )
    {
      static const int prefix_ctx[8]  = { 0, 0, 0, 3, 6, 10, 15, 21 };
      const_cast<int&>(m_lastOffsetX) = prefix_ctx[ log2sizeX ];
      const_cast<int&>(m_lastOffsetY) = prefix_ctx[ log2sizeY ];;
    }
    else
    {
      const_cast<int&>(m_lastOffsetX) = 3 * (log2sizeX - 2) + ((log2sizeX - 1) >> 2);
      const_cast<int&>(m_lastOffsetY) = 3 * (log2sizeY - 2) + ((log2sizeY - 1) >> 2);
    }
    const_cast<int&>(m_lastShiftX)  = (log2sizeX + 1) >> 2;
    const_cast<int&>(m_lastShiftY)  = (log2sizeY + 1) >> 2;
  }
  if( m_altResiCompId == 1 )
  {
    if( m_scanType && m_log2BlockWidth == 3 && m_log2BlockHeight == 3 )
    {
      m_scan     = g_scanOrder     [ SCAN_UNGROUPED ][ m_scanType ][ gp_sizeIdxInfo->idxFrom( m_width ) ][ gp_sizeIdxInfo->idxFrom( m_height ) ];
      m_scanPosX = g_scanOrderPosXY[ SCAN_UNGROUPED ][ m_scanType ][ gp_sizeIdxInfo->idxFrom( m_width ) ][ gp_sizeIdxInfo->idxFrom( m_height ) ][0];
      m_scanPosY = g_scanOrderPosXY[ SCAN_UNGROUPED ][ m_scanType ][ gp_sizeIdxInfo->idxFrom( m_width ) ][ gp_sizeIdxInfo->idxFrom( m_height ) ][1];
    }
    if( m_scanType == SCAN_VER && m_log2WidthInGroups == 1 && m_log2HeightInGroups == 1 )
    {
      m_scanCG = g_scanOrder[ SCAN_UNGROUPED ][ m_scanType - 1 ][ gp_sizeIdxInfo->idxFrom( m_widthInGroups ) ][ gp_sizeIdxInfo->idxFrom( m_heightInGroups ) ];
    }
  }
}

void CoeffCodingContext::initSubblock( int SubsetId, bool sigGroupFlag )
{
  m_subSetId                = SubsetId;
  m_subSetPos               = m_scanCG[ m_subSetId ];
  m_subSetPosY              = m_subSetPos / m_widthInGroups;
  m_subSetPosX              = m_subSetPos - ( m_subSetPosY * m_widthInGroups );
  m_minSubPos               = m_subSetId << m_log2CGSize;
  m_maxSubPos               = m_minSubPos + ( 1 << m_log2CGSize ) - 1;
  if( sigGroupFlag )
  {
    m_sigCoeffGroupFlag.set ( m_subSetPos );
  }
  unsigned  CGPosY   = 0;
  unsigned  CGPosX   = 0;
  unsigned  sigRight = 0;
  unsigned  sigLower = 0;
  if( m_altResiCompId == 1 )
  {
    unsigned widthInGroups  = m_widthInGroups;
    unsigned heightInGroups = m_heightInGroups;

    if( m_widthInGroups == 2 && m_heightInGroups )
    {
      if( m_scanType == SCAN_HOR )
      {
        widthInGroups  = 1;
        heightInGroups = 4;
      }
      else if( m_scanType == SCAN_VER )
      {
        widthInGroups  = 4;
        heightInGroups = 1;
      }
    }

//     CGPosY    = m_subSetPos >> m_log2WidthInGroups;
//     CGPosX    = m_subSetPos - ( CGPosY << m_log2WidthInGroups );
    CGPosY = m_subSetPosY;
    CGPosX = m_subSetPosX;

    bool hor8x8 = m_width == 8 && m_height == 8 && m_scanType == SCAN_HOR;
    bool ver8x8 = m_width == 8 && m_height == 8 && m_scanType == SCAN_VER;
    bool bonDiag8x8 = hor8x8 || ver8x8;
    if( bonDiag8x8 )
    {
      CGPosY = ( bonDiag8x8 ? m_subSetPos : 0 );
      CGPosX = ( bonDiag8x8 ? m_subSetPos : 0 );
    }

    sigRight  = unsigned( ( CGPosX + 1 ) < widthInGroups  ? m_sigCoeffGroupFlag[ m_subSetPos + 1             ] : false );
    sigLower  = unsigned( ( CGPosY + 1 ) < heightInGroups ? m_sigCoeffGroupFlag[ m_subSetPos + widthInGroups ] : false );
  }
  else
  {
//     CGPosY    = m_subSetPos >> m_log2WidthInGroups;
//     CGPosX    = m_subSetPos - ( CGPosY << m_log2WidthInGroups );
    CGPosY    = m_subSetPosY;
    CGPosX    = m_subSetPosX;
    sigRight  = unsigned( ( CGPosX + 1 ) < m_widthInGroups  ? m_sigCoeffGroupFlag[ m_subSetPos + 1               ] : false );
    sigLower  = unsigned( ( CGPosY + 1 ) < m_heightInGroups ? m_sigCoeffGroupFlag[ m_subSetPos + m_widthInGroups ] : false );
  }
  const unsigned  ctxSet    = m_prevGt2 + ( m_chType == CHANNEL_TYPE_LUMA ? ( m_subSetId > 0 ? 2 : 0 ) : 4 );
  m_sigGroupCtxId           = Ctx::SigCoeffGroup[m_chType]( sigRight | sigLower );
  m_gt1FlagCtxSet           = Ctx::GreaterOneFlag[ ctxSet ];
  m_gt2FlagCtxId            = Ctx::GreaterTwoFlag( ctxSet );
  m_sigCGPattern            = sigRight + ( sigLower << 1 );
  if( m_altResiCompId > 0 )
  {
    m_sigCtxSet     = Ctx::SigFlag       [ m_chType + 2 ];
    m_gt1FlagCtxSet = Ctx::GreaterOneFlag[ m_chType + 6 ];
    m_sigGroupCtxId = Ctx::SigCoeffGroup [ m_chType + 2 ]( sigRight | sigLower );
  }
}


unsigned CoeffCodingContext::sigCtxId( int scanPos ) const
{
  int offset = 0; // DC

  if( m_SigBlockType == 0 ) // bypass
  {
    offset = ( m_chType == CHANNEL_TYPE_LUMA ? 27 : 15 );
  }
  else if( scanPos )
  {
    const unsigned posY       = m_scanPosY[ scanPos ];
    const unsigned posX       = m_scanPosX[ scanPos ];

    if( m_SigBlockType == 1 ) // 4x4
    {
      //      const unsigned ctxIndMap4x4[16] = { 0, 1, 4, 5, 2, 3, 4, 5, 6, 6, 8, 8, 7, 7, 8, 8 };
      offset = ctxIndMap4x4[ ( posY << 2 ) + posX ];
    }
    else
    {
      int cnt = 0;
      switch( m_sigCGPattern )
      {
      case 0:
      {
        unsigned posIS  = ( posX & 3 ) + ( posY & 3 );
        cnt             = ( posIS >= 3 ? 0 : posIS >= 1 ? 1 : 2 );
      }
      break;
      case 1:
      {
        unsigned posIS  = ( posY & 3 );
        cnt             = ( posIS >= 2 ? 0 : posIS >= 1 ? 1 : 2 );
      }
      break;
      case 2:
      {
        unsigned posIS  = ( posX & 3 );
        cnt             = ( posIS >= 2 ? 0 : posIS >= 1 ? 1 : 2 );
      }
      break;
      case 3:
      {
        cnt             = 2;
      }
      break;
      default:
        THROW( "sig pattern must be in range [0;3]" );
      }
      offset    = ( m_chType == CHANNEL_TYPE_LUMA && ( posX > 3 || posY > 3 ) ? 3 : 0 ) + cnt;

      if( m_SigBlockType == 2 ) // 8x8
      {
        offset += ( m_scanType != SCAN_DIAG && m_chType == CHANNEL_TYPE_LUMA ? 15 : 9 );
      }
      else // NxN
      {
        offset += ( m_chType == CHANNEL_TYPE_LUMA ? 21 : 12 );
      }
    }
  }
  return m_sigCtxSet( offset );
}


void CoeffCodingContext::getAltResiCtxSet( const TCoeff* coeff,
                                           int   scanPos,
                                           UInt& sigCtxIdx,
                                           UInt& gt1CtxIdx,
                                           UInt& gt2CtxIdx,
                                           UInt& goRicePar,
                                           int   strd
                                           )
{
  const UInt posY = m_scanPosY[scanPos];
  const UInt posX = m_scanPosX[scanPos];

  strd = strd == 0 ? m_width : strd;
  const TCoeff *pData = coeff + posX + posY * strd;
  const Int   widthM1 = m_width - 1;
  const Int  heightM1 = m_height - 1;
  const Int      diag = posX + posY;


  Int sumAbs  = 0;
  Int numPos1 = 0;
  Int numPos2 = 0;
  Int numPosN = 0;

  if( posX < widthM1 )
  {
    sumAbs  += abs( pData[ 1 ] );
    numPos1 += abs( pData[ 1 ] ) > 1;
    numPos2 += abs( pData[ 1 ] ) > 2;
    numPosN +=      pData[ 1 ] != 0;
    if( posX < widthM1 - 1 )
    {
      sumAbs  += abs( pData[ 2 ] );
      numPos1 += abs( pData[ 2 ] ) > 1;
      numPos2 += abs( pData[ 2 ] ) > 2;
      numPosN +=      pData[ 2 ] != 0;
    }
    if( posY < heightM1 )
    {
      sumAbs  += abs( pData[ m_width + 1 ] );
      numPos1 += abs( pData[ m_width + 1 ] ) > 1;
      numPos2 += abs( pData[ m_width + 1 ] ) > 2;
      numPosN +=      pData[ m_width + 1 ] != 0;
    }
  }
  if( posY < heightM1 )
  {
    sumAbs  += abs( pData[ m_width ] );
    numPos1 += abs( pData[ m_width ] ) > 1;
    numPos2 += abs( pData[ m_width ] ) > 2;
    numPosN +=      pData[ m_width ] != 0;
    if( posY < heightM1 - 1 )
    {
      sumAbs  += abs( pData[ 2 * m_width ] );
      numPos1 += abs( pData[ 2 * m_width ] ) > 1;
      numPos2 += abs( pData[ 2 * m_width ] ) > 2;
      numPosN +=      pData[ 2 * m_width ] != 0;
    }
  }

  unsigned val = sumAbs - numPosN;
  unsigned order = 0;
  for( order = 0; order < MAX_GR_ORDER_RESIDUAL; order++ )
  {
    if( ( 1 << ( order + 3 ) ) >( val + 4 ) )
    {
      break;
    }
  }
  goRicePar = ( order == MAX_GR_ORDER_RESIDUAL ? ( MAX_GR_ORDER_RESIDUAL - 1 ) : order );

  const Int ctxIdx1 = std::min( numPos1, 4 ) + 1;
        Int ctxOfs1 = 0;

  const Int ctxIdx2 = std::min( numPos2, 4 ) + 1;
        Int ctxOfs2 = 0;

  const Int ctxIdxN = std::min( numPosN, 5 );
        Int ctxOfsN = diag < 2 ? 6 : 0;


  if( m_chType == CHANNEL_TYPE_LUMA )
  {
    ctxOfs1 += diag < 3 ? 10 : ( diag < 10 ? 5 : 0 );
    ctxOfs2 += diag < 3 ? 10 : ( diag < 10 ? 5 : 0 );
    ctxOfsN += diag < 5 ? 6 : 0;
  }

  if( m_log2BlockSize > 2 && m_chType == CHANNEL_TYPE_LUMA )
  {
    ctxOfsN += 18 << std::min( 1, ( (int)m_log2BlockSize - 3 ) );
  }

  gt1CtxIdx = m_gt1FlagCtxSet( ctxOfs1 + ctxIdx1 );
  gt2CtxIdx = m_gt1FlagCtxSet( ctxOfs2 + ctxIdx2 );
  sigCtxIdx = m_sigCtxSet    ( ctxOfsN + ctxIdxN );
     
}



unsigned DeriveCtx::CtxCUsplit( const CodingStructure& cs, Partitioner& partitioner )
{
  auto adPartitioner = dynamic_cast<AdaptiveDepthPartitioner*>( &partitioner );

  if( !adPartitioner )
  {
    return 0;
  }

  const Position pos         = partitioner.currArea().blocks[cs.chType];
  const unsigned curSliceIdx = cs.slice->getIndependentSliceIdx();
  const unsigned curTileIdx  = cs.picture->tileMap->getTileIdxMap( partitioner.currArea().lumaPos() );
  unsigned ctxId = 0;

  // get left depth
  const CodingUnit* cuLeft = cs.getCURestricted( pos.offset( -1, 0 ), curSliceIdx, curTileIdx, cs.chType );
  ctxId = ( cuLeft && cuLeft->qtDepth > partitioner.currQtDepth ) ? 1 : 0;

  // get above depth
  const CodingUnit* cuAbove = cs.getCURestricted( pos.offset( 0, -1 ), curSliceIdx, curTileIdx, cs.chType );
  ctxId += ( cuAbove && cuAbove->qtDepth > partitioner.currQtDepth ) ? 1 : 0;

  if( cs.sps->getSpsNext().getUseLargeCTU() )
  {
    unsigned minDepth = 0;
    unsigned maxDepth = 0;
    adPartitioner->setMaxMinDepth( minDepth, maxDepth, cs );
    if( partitioner.currDepth < minDepth )
    {
      ctxId = 3;
    }
    else if( partitioner.currDepth >= maxDepth + 1 )
    {
      ctxId = 4;
    }
  }

  return ctxId;
}

unsigned DeriveCtx::CtxQtCbf( const ComponentID compID, const unsigned trDepth )
{
  if( isChroma( compID ) )
  {
    return trDepth;
  }
  else
  {
    return ( trDepth == 0 ? 1 : 0 );
  }
}

unsigned DeriveCtx::CtxInterDir( const PredictionUnit& pu )
{
  if( pu.cs->sps->getSpsNext().getUseLargeCTU() )
  {
    if( pu.cs->pcv->rectCUs )
    {
      return Clip3( 0, 3, 7 - ( ( g_aucLog2[pu.lumaSize().width] + g_aucLog2[pu.lumaSize().height] + 1 ) >> 1 ) );    // VG-ASYMM DONE
    }
    return Clip3( 0, 3, 6 - g_aucLog2[pu.cu->lumaSize().width] );
  }
  return pu.cu->qtDepth;
}

unsigned DeriveCtx::CtxAffineFlag( const CodingUnit& cu )
{
  const CodingStructure *cs = cu.cs;
  unsigned ctxId = 0;

  const CodingUnit *cuLeft = cs->getCURestricted(cu.lumaPos().offset(-1, 0), cu);
  ctxId = (cuLeft && cuLeft->affine) ? 1 : 0;

  const CodingUnit *cuAbove= cs->getCURestricted(cu.lumaPos().offset(0, -1), cu);
  ctxId += (cuAbove && cuAbove->affine) ? 1 : 0;

  return ctxId;
}

unsigned DeriveCtx::CtxSkipFlag( const CodingUnit& cu )
{
  const CodingStructure *cs = cu.cs;
  unsigned ctxId = 0;

  // Get BCBP of left PU
  const CodingUnit *cuLeft = cs->getCURestricted(cu.lumaPos().offset(-1, 0), cu);
  ctxId = (cuLeft && cuLeft->skip) ? 1 : 0;

  // Get BCBP of above PU
  const CodingUnit *cuAbove= cs->getCURestricted(cu.lumaPos().offset(0, -1), cu);
  ctxId += (cuAbove && cuAbove->skip) ? 1 : 0;

  return ctxId;
}

unsigned DeriveCtx::CtxIMVFlag( const CodingUnit& cu )
{
  const CodingStructure *cs = cu.cs;
  unsigned ctxId = 0;

  // Get BCBP of left PU
  const CodingUnit *cuLeft = cs->getCURestricted(cu.lumaPos().offset(-1, 0), cu);
  ctxId = ( cuLeft && cuLeft->imv ) ? 1 : 0;

  // Get BCBP of above PU
  const CodingUnit *cuAbove = cs->getCURestricted(cu.lumaPos().offset(0, -1), cu);
  ctxId += ( cuAbove && cuAbove->imv ) ? 1 : 0;

  return ctxId;
}

unsigned DeriveCtx::CtxBTsplit(const CodingStructure& cs, Partitioner& partitioner)
{
  const Position pos          = partitioner.currArea().blocks[cs.chType];
  const unsigned curSliceIdx  = cs.slice->getIndependentSliceIdx();
  const unsigned curTileIdx   = cs.picture->tileMap->getTileIdxMap( pos );

  unsigned ctx                = 0;

  const CodingUnit *cuLeft    = cs.getCURestricted( pos.offset( -1,  0 ), curSliceIdx, curTileIdx, cs.chType );
  const CodingUnit *cuAbove   = cs.getCURestricted( pos.offset(  0, -1 ), curSliceIdx, curTileIdx, cs.chType );

  {
    const unsigned currDepth = partitioner.currQtDepth * 2 + partitioner.currBtDepth;

    if( cuLeft )  ctx += ( ( 2 * cuLeft->qtDepth  + cuLeft->btDepth  ) > currDepth ? 1 : 0 );
    if( cuAbove ) ctx += ( ( 2 * cuAbove->qtDepth + cuAbove->btDepth ) > currDepth ? 1 : 0 );
  }

  return ctx;
}

unsigned DeriveCtx::CtxFrucFlag( const PredictionUnit& pu )
{
  unsigned ctxId = 0;

  const CodingStructure &cs     = *pu.cs;
  const Position pos            = pu.lumaPos();

  const PredictionUnit *puLeft  = cs.getPURestricted( pos.offset( -1, 0 ), pu );
  ctxId  = ( puLeft ) ? puLeft->frucMrgMode > 0 : 0;

  const PredictionUnit *puAbove = cs.getPURestricted( pos.offset( 0, -1 ), pu );
  ctxId += ( puAbove ) ? puAbove->frucMrgMode > 0 : 0;

  return ctxId;
}

unsigned DeriveCtx::CtxFrucMode( const PredictionUnit& pu )
{
  unsigned ctxId = 0;

  const CodingStructure &cs     = *pu.cs;
  const Position pos            = pu.lumaPos();

  const PredictionUnit *puLeft  = cs.getPURestricted( pos.offset( -1, 0 ), pu );
  ctxId  = ( puLeft ) ? puLeft->frucMrgMode == FRUC_MERGE_BILATERALMV : 0;

  const PredictionUnit *puAbove = cs.getPURestricted( pos.offset( 0, -1 ), pu );
  ctxId += ( puAbove ) ? puAbove->frucMrgMode == FRUC_MERGE_BILATERALMV : 0;

  return ctxId;
}

Void MergeCtx::setMergeInfo( PredictionUnit& pu, int candIdx )
{
  CHECK( candIdx >= numValidMergeCand, "Merge candidate does not exist" );

  pu.mergeFlag               = true;
  pu.interDir                = interDirNeighbours[candIdx];
  pu.mergeIdx                = candIdx;
  pu.mergeType               = mrgTypeNeighnours[candIdx];
  pu.mv     [REF_PIC_LIST_0] = mvFieldNeighbours[(candIdx << 1) + 0].mv;
  pu.mv     [REF_PIC_LIST_1] = mvFieldNeighbours[(candIdx << 1) + 1].mv;
  pu.mvd    [REF_PIC_LIST_0] = Mv();
  pu.mvd    [REF_PIC_LIST_1] = Mv();
  pu.refIdx [REF_PIC_LIST_0] = mvFieldNeighbours[( candIdx << 1 ) + 0].refIdx;
  pu.refIdx [REF_PIC_LIST_1] = mvFieldNeighbours[( candIdx << 1 ) + 1].refIdx;
  pu.mvpIdx [REF_PIC_LIST_0] = NOT_VALID;
  pu.mvpIdx [REF_PIC_LIST_1] = NOT_VALID;
  pu.mvpNum [REF_PIC_LIST_0] = NOT_VALID;
  pu.mvpNum [REF_PIC_LIST_1] = NOT_VALID;

  if( pu.lumaSize() == pu.cu->lumaSize() )
  {
    pu.cu->LICFlag = ( pu.cs->slice->getUseLIC() ? LICFlags[candIdx] : false );
  }
}
