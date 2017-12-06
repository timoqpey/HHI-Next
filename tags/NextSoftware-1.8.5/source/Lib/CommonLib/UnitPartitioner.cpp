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

/** \file     UnitPartitioner.h
 *  \brief    Provides a class for partitioning management
 */

#include "UnitPartitioner.h"

#include "CodingStructure.h"
#include "Unit.h"
#include "Slice.h"
#include "UnitTools.h"
#include "Picture.h"


PartLevel::PartLevel()
: split               ( CU_DONT_SPLIT )
, parts               (               )
, idx                 ( 0u            )
, checkdIfImplicit    ( false         )
, isImplicit          ( false         )
, implicitSplit       ( CU_DONT_SPLIT )
, firstSubPartSplit   ( CU_DONT_SPLIT )
, canQtSplit          ( true          )
{
}

PartLevel::PartLevel( const PartSplit _split, const Partitioning& _parts )
: split               ( _split        )
, parts               ( _parts        )
, idx                 ( 0u            )
, checkdIfImplicit    ( false         )
, isImplicit          ( false         )
, implicitSplit       ( CU_DONT_SPLIT )
, firstSubPartSplit   ( CU_DONT_SPLIT )
, canQtSplit          ( true          )
{
}

PartLevel::PartLevel( const PartSplit _split, Partitioning&& _parts )
: split               ( _split                               )
, parts               ( std::forward<Partitioning>( _parts ) )
, idx                 ( 0u                                   )
, checkdIfImplicit    ( false                                )
, isImplicit          ( false                                )
, implicitSplit       ( CU_DONT_SPLIT                        )
, firstSubPartSplit   ( CU_DONT_SPLIT                        )
, canQtSplit          ( true                                 )
{
}

//////////////////////////////////////////////////////////////////////////
// Partitioner class
//////////////////////////////////////////////////////////////////////////

SplitSeries Partitioner::getSplitSeries() const
{
  SplitSeries splitSeries = 0;
  SplitSeries depth = 0;

  for( const auto &level : m_partStack )
  {
    if( level.split == CTU_LEVEL ) continue;
    else splitSeries += static_cast< SplitSeries >( level.split ) << ( depth * SPLIT_DMULT );

    depth++;
  }

  return splitSeries;
}

void Partitioner::setCUData( CodingUnit& cu )
{
  cu.depth       = currDepth;
  cu.btDepth     = currBtDepth;
  cu.qtDepth     = currQtDepth;
  cu.splitSeries = getSplitSeries();
}

//////////////////////////////////////////////////////////////////////////
// AdaptiveDepthPartitioner class
//////////////////////////////////////////////////////////////////////////

void AdaptiveDepthPartitioner::setMaxMinDepth( unsigned& minDepth, unsigned& maxDepth, const CodingStructure& cs ) const
{
  unsigned          stdMinDepth = 0;
  unsigned          stdMaxDepth = ( cs.sps->getSpsNext().getUseQTBT() ? g_aucLog2[cs.sps->getSpsNext().getCTUSize()] - g_aucLog2[cs.sps->getSpsNext().getMinQTSize( cs.slice->getSliceType(), cs.chType )] : cs.sps->getLog2DiffMaxMinCodingBlockSize() );
  const Position    pos         = currArea().blocks[cs.chType].pos();
  const unsigned    curSliceIdx = cs.slice->getIndependentSliceIdx();
  const unsigned    curTileIdx  = cs.picture->tileMap->getTileIdxMap( currArea().lumaPos() );

  const CodingUnit* cuLeft        = cs.getCURestricted( pos.offset( -1,                                                                           0 ), curSliceIdx, curTileIdx );
  const CodingUnit* cuBelowLeft   = cs.getCURestricted( pos.offset( -1, cs.pcv->minCUHeight >> getChannelTypeScaleY( cs.chType, cs.pcv->chrFormat ) ), curSliceIdx, curTileIdx );  // should use actual block size instead of minCU size
  const CodingUnit* cuAbove       = cs.getCURestricted( pos.offset(  0,                                                                          -1 ), curSliceIdx, curTileIdx );
  const CodingUnit* cuAboveRight  = cs.getCURestricted( pos.offset( cs.pcv->minCUWidth >> getChannelTypeScaleX( cs.chType, cs.pcv->chrFormat ),  -1 ), curSliceIdx, curTileIdx );  // should use actual block size instead of minCU size

  minDepth = stdMaxDepth;
  maxDepth = stdMinDepth;

  if( cuLeft )
  {
    minDepth = std::min<unsigned>( minDepth, cuLeft->qtDepth );
    maxDepth = std::max<unsigned>( maxDepth, cuLeft->qtDepth );
  }
  else
  {
    minDepth = stdMinDepth;
    maxDepth = stdMaxDepth;
  }

  if( cuBelowLeft )
  {
    minDepth = std::min<unsigned>( minDepth, cuBelowLeft->qtDepth );
    maxDepth = std::max<unsigned>( maxDepth, cuBelowLeft->qtDepth );
  }
  else
  {
    minDepth = stdMinDepth;
    maxDepth = stdMaxDepth;
  }

  if( cuAbove )
  {
    minDepth = std::min<unsigned>( minDepth, cuAbove->qtDepth );
    maxDepth = std::max<unsigned>( maxDepth, cuAbove->qtDepth );
  }
  else
  {
    minDepth = stdMinDepth;
    maxDepth = stdMaxDepth;
  }

  if( cuAboveRight )
  {
    minDepth = std::min<unsigned>( minDepth, cuAboveRight->qtDepth );
    maxDepth = std::max<unsigned>( maxDepth, cuAboveRight->qtDepth );
  }
  else
  {
    minDepth = stdMinDepth;
    maxDepth = stdMaxDepth;
  }

  minDepth = ( minDepth >= 1 ? minDepth - 1 : 0 );
  maxDepth = std::min<unsigned>( stdMaxDepth, maxDepth + 1 );
}

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
// HEVCPartitioner
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

Void HEVCPartitioner::initCtu( const UnitArea& ctuArea )
{
#if _DEBUG
  m_currArea  = ctuArea;
#endif
  currDepth   = 0;
  currTrDepth = 0;
  currBtDepth = 0;
  currQtDepth = 0;

  m_partStack.clear();
  m_partStack.push_back( PartLevel( CTU_LEVEL, Partitioning{ ctuArea } ) );
}

Void HEVCPartitioner::splitCurrArea( const PartSplit split, const CodingStructure& cs )
{
  CHECKD( !canSplit( split, cs ), "Trying to apply a prohibited split!" );

  switch( split )
  {
  case CU_QUAD_SPLIT:
    m_partStack.push_back( PartLevel( split, PartitionerImpl::getCUSubPartitions( currArea(), cs ) ) );
    break;
  case TU_QUAD_SPLIT:
    m_partStack.push_back( PartLevel( split, PartitionerImpl::getTUSubPartitions( currArea(), cs ) ) );
    break;
  default:
    THROW( "Unknown split mode" );
    break;
  }

  currDepth++;
#if _DEBUG
  m_currArea = m_partStack.back().parts.front();
#endif

  if( split == TU_QUAD_SPLIT )
  {
    currTrDepth++;
  }
  else
  {
    currTrDepth = 0;
  }

  currQtDepth++;
}

bool HEVCPartitioner::canSplit( const PartSplit split, const CodingStructure &cs )
{
  switch( split )
  {
  case CTU_LEVEL:
    THROW( "Checking if top level split is possible" );
    return true;
    break;
  case TU_QUAD_SPLIT:
    // not important, only for HM compatibility
    return true;
    break;
  case CU_QUAD_SPLIT:
    {
      unsigned minQtSize = cs.pcv->getMinQtSize( *cs.slice, cs.chType );
      if( currArea().lwidth() <=  minQtSize || currArea().lheight() <= minQtSize ) return false;

      return true;
    }
    break;
  case CU_HORZ_SPLIT:
  case CU_VERT_SPLIT:
  case CU_BT_SPLIT:
    return false;
    break;
  default:
    THROW( "Unknown split mode" );
    return false;
    break;
  }

  return true;
}

bool HEVCPartitioner::isSplitImplicit( const PartSplit split, const CodingStructure &cs )
{
  return split == getImplicitSplit( cs );
}

PartSplit HEVCPartitioner::getImplicitSplit( const CodingStructure &cs )
{
  if( m_partStack.back().checkdIfImplicit )
  {
    return m_partStack.back().implicitSplit;
  }

  PartSplit split = CU_DONT_SPLIT;

  if( !cs.picture->Y().contains( currArea().Y().bottomRight() ) )
  {
    split = CU_QUAD_SPLIT;
  }

  m_partStack.back().checkdIfImplicit = true;
  m_partStack.back().isImplicit       = split != CU_DONT_SPLIT;
  m_partStack.back().implicitSplit    = split;

  return split;
}

Void HEVCPartitioner::exitCurrSplit()
{
  m_partStack.pop_back();

  CHECK( currDepth == 0, "depth is '0', although a split was performed" );
  currDepth--;
#if _DEBUG
  m_currArea = m_partStack.back().parts[m_partStack.back().idx];
#endif

  if( currTrDepth > 0 ) currTrDepth--;

  CHECK( currQtDepth == 0, "QT depth is '0', although a QT split was performed" );
  currQtDepth--;
}

Bool HEVCPartitioner::nextPart(const CodingStructure &cs, bool autoPop /*= false*/)
{
  const Position &prevPos = currArea().blocks[cs.chType].pos();

  unsigned currIdx = ++m_partStack.back().idx;

  m_partStack.back().checkdIfImplicit = false;
  m_partStack.back().isImplicit       = false;

  if( currIdx == 1 )
  {
    const CodingUnit* prevCU = cs.getCU( prevPos );
    m_partStack.back().firstSubPartSplit = prevCU ? CU::getSplitAtDepth( *prevCU, currDepth ) : CU_DONT_SPLIT;
  }

  if( currIdx < m_partStack.back().parts.size() )
  {
#if _DEBUG
    m_currArea = m_partStack.back().parts[currIdx];
#endif
    return true;
  }
  else
  {
    if( autoPop ) exitCurrSplit();
    return false;
  }
}

bool HEVCPartitioner::hasNextPart()
{
  return ( ( m_partStack.back().idx + 1 ) < m_partStack.back().parts.size() );
}

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
// QTBTPartitioner
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

Void QTBTPartitioner::initCtu( const UnitArea& ctuArea )
{
#if _DEBUG
  m_currArea = ctuArea;
#endif
  currDepth   = 0;
  currTrDepth = 0;
  currBtDepth = 0;
  currQtDepth = 0;

  m_partStack.clear();
  m_partStack.push_back( PartLevel( CTU_LEVEL, Partitioning{ ctuArea } ) );
}

Void QTBTPartitioner::splitCurrArea( const PartSplit split, const CodingStructure& cs )
{
  CHECKD( !canSplit( split, cs ), "Trying to apply a prohibited split!" );

  bool canQtSplit = canSplit( CU_QUAD_SPLIT, cs );

  switch( split )
  {
  case CU_QUAD_SPLIT:
    m_partStack.push_back( PartLevel( split, PartitionerImpl::getCUSubPartitions( currArea(), cs ) ) );
    break;
  case TU_QUAD_SPLIT:
    m_partStack.push_back( PartLevel( split, PartitionerImpl::getTUSubPartitions( currArea(), cs ) ) );
    break;
  case CU_HORZ_SPLIT:
  case CU_VERT_SPLIT:
    CHECK( !cs.sps->getSpsNext().getUseQTBT(), "QTBT disabled" );
    m_partStack.push_back( PartLevel( split, PartitionerImpl::getCUSubPartitions( currArea(), cs, split ) ) );
    break;
  default:
    THROW( "Unknown split mode" );
    break;
  }

  currDepth++;
#if _DEBUG
  m_currArea = m_partStack.back().parts.front();
#endif

  if( split == TU_QUAD_SPLIT )
  {
    THROW( "QTBT does not allow for RQT" );
  }
  else
  {
    currTrDepth = 0;
  }

  if( split == CU_HORZ_SPLIT || split == CU_VERT_SPLIT )
  {
    currBtDepth++;

    m_partStack.back().canQtSplit = canQtSplit;
  }
  else
  {
    CHECK( currBtDepth > 0, "Cannot split a non-square area other than with a binary split" );
    currBtDepth = 0;
    currQtDepth++;
  }
}

bool QTBTPartitioner::canSplit( const PartSplit split, const CodingStructure &cs )
{
  const PartSplit implicitSplit = getImplicitSplit( cs );

  // the minimal and maximal sizes are given in luma samples
  const CompArea area           = currArea().Y();

  const unsigned maxBTD         = cs.pcv->getMaxBtDepth( *cs.slice, cs.chType );
  const unsigned maxBtSize      = cs.pcv->getMaxBtSize( *cs.slice, cs.chType );
  const unsigned minBtSize      = cs.pcv->getMinBtSize( *cs.slice, cs.chType );


  const PartSplit prevSplit     = m_partStack.back().firstSubPartSplit;
  const PartSplit lastSplit     = m_partStack.back().split;
  const PartSplit perpSplit     = lastSplit == CU_HORZ_SPLIT ? CU_VERT_SPLIT : CU_HORZ_SPLIT;

  switch( split )
  {
  case CTU_LEVEL:
    THROW( "Checking if top level split is possible" );
    return true;
    break;
  case TU_QUAD_SPLIT:
    return false;
    break;
  case CU_QUAD_SPLIT:
  {
    // don't allow QT-splitting below a BT split
    PartSplit lastSplit = m_partStack.back().split;
    if( lastSplit != CTU_LEVEL && lastSplit != CU_QUAD_SPLIT )                  return false;
    
    // allowing QT split even if a BT split is implied
    if( implicitSplit != CU_DONT_SPLIT )                                        return true;

    unsigned minQtSize = cs.pcv->getMinQtSize( *cs.slice, cs.chType );
    if( currArea().lwidth() <= minQtSize || currArea().lheight() <= minQtSize ) return false;

    return true;
  }
  break;
  // general check for BT split, specific checks are done in a separate switch
  case CU_HORZ_SPLIT:
  case CU_VERT_SPLIT:
  {
    if( !cs.slice->isIntra() && m_partStack.back().idx == 1 && implicitSplit == CU_DONT_SPLIT && ( lastSplit == CU_HORZ_SPLIT || lastSplit == CU_VERT_SPLIT ) )
    {
      if( split == perpSplit && prevSplit == perpSplit && ( ( lastSplit == CU_VERT_SPLIT && m_partStack.back().canQtSplit ) || lastSplit == CU_HORZ_SPLIT ) )
      {
        return false;
      }
    }
  }
  case CU_BT_SPLIT:
  {
    if( !cs.sps->getSpsNext().getUseQTBT() )                  return false;
    if( currBtDepth >= maxBTD )                               return false;
    if( area.width <= minBtSize && area.height <= minBtSize ) return false;
    if( area.width > maxBtSize || area.height > maxBtSize )   return false;
  }
  break;
  default:
    THROW( "Unknown split mode" );
    return false;
    break;
  }

  // specific check for BT splits
  switch( split )
  {
  case CU_HORZ_SPLIT:
    if( area.height <= minBtSize || area.height > maxBtSize )     return false;
    break;
  case CU_VERT_SPLIT:
    if( area.width <= minBtSize || area.width > maxBtSize )       return false;
    break;
  default:
    break;
  }

  return true;
}

bool QTBTPartitioner::isSplitImplicit( const PartSplit split, const CodingStructure &cs )
{
  return split == getImplicitSplit( cs );
}

PartSplit QTBTPartitioner::getImplicitSplit( const CodingStructure &cs )
{
  if( m_partStack.back().checkdIfImplicit )
  {
    return m_partStack.back().implicitSplit;
  }

  PartSplit split = CU_DONT_SPLIT;

  if( currArea().lwidth() > cs.sps->getMaxTrSize() || currArea().lheight() > cs.sps->getMaxTrSize() )
  {
    split = CU_QUAD_SPLIT;
  }

  if( split == CU_DONT_SPLIT )
  {
    const bool isBlInPic = cs.picture->Y().contains( currArea().Y().bottomLeft() );
    const bool isTrInPic = cs.picture->Y().contains( currArea().Y().topRight() );

    if( !isBlInPic || !isTrInPic )
    {
      split = CU_QUAD_SPLIT;
    }
  }

  m_partStack.back().checkdIfImplicit = true;
  m_partStack.back().isImplicit = split != CU_DONT_SPLIT;
  m_partStack.back().implicitSplit = split;

  return split;
}

Void QTBTPartitioner::exitCurrSplit()
{
  PartSplit currSplit = m_partStack.back().split;

  m_partStack.pop_back();

  CHECK( currDepth == 0, "depth is '0', although a split was performed" );
  currDepth--;
#if _DEBUG
  m_currArea = m_partStack.back().parts[m_partStack.back().idx];
#endif

  if( currSplit == CU_HORZ_SPLIT || currSplit == CU_VERT_SPLIT )
  {
    CHECK( !m_partStack.back().checkdIfImplicit, "Didn't check if the current split is implicit" );
    CHECK( currBtDepth == 0, "BT depth is '0', athough a BT split was performed" );
    currBtDepth--;

  }
  else
  {
    CHECK( currTrDepth > 0, "RQT found with QTBT partitioner" );

    CHECK( currQtDepth == 0, "QT depth is '0', although a QT split was performed" );
    currQtDepth--;
  }
}

Bool QTBTPartitioner::nextPart( const CodingStructure &cs, bool autoPop /*= false*/ )
{
  const Position &prevPos = currArea().blocks[cs.chType].pos();

  unsigned currIdx = ++m_partStack.back().idx;

  m_partStack.back().checkdIfImplicit = false;
  m_partStack.back().isImplicit = false;

  if( currIdx == 1 )
  {
    const CodingUnit* prevCU = cs.getCU( prevPos );
    m_partStack.back().firstSubPartSplit = prevCU ? CU::getSplitAtDepth( *prevCU, currDepth ) : CU_DONT_SPLIT;
  }

  if( currIdx < m_partStack.back().parts.size() )
  {
#if _DEBUG
    m_currArea = m_partStack.back().parts[currIdx];
#endif
    return true;
  }
  else
  {
    if( autoPop ) exitCurrSplit();
    return false;
  }
}

bool QTBTPartitioner::hasNextPart()
{
  return ( ( m_partStack.back().idx + 1 ) < m_partStack.back().parts.size() );
}


//////////////////////////////////////////////////////////////////////////
// PartitionerFactory
//////////////////////////////////////////////////////////////////////////

Partitioner* PartitionerFactory::get( const Slice& slice )
{
  if( slice.getSPS()->getSpsNext().getUseQTBT() )
  {
    return new QTBTPartitioner;
  }
  else
  {
    return new HEVCPartitioner;
  }
}

//////////////////////////////////////////////////////////////////////////
// Partitioner methods describing the actual partitioning logic
//////////////////////////////////////////////////////////////////////////

Partitioning PartitionerImpl::getPUPartitioning( const CodingUnit &cu )
{
  Partitioning pus;

  pus.reserve( 4 );

  switch( cu.partSize )
  {
  case SIZE_2Nx2N:
    pus.push_back( cu );
    break;
  case SIZE_2NxN:
  {
    CHECK( !CU::isInter( cu ), "2NxN split applied to non inter CU" );

    UnitArea pu( cu );

    for( UInt i = 0; i < pu.blocks.size(); i++ )
    {
      pu.blocks[i].height >>= 1;
    }

    pus.push_back( pu );

    for( UInt i = 0; i < pu.blocks.size(); i++ )
    {
      pu.blocks[i].y += pu.blocks[i].height;
    }

    pus.push_back( pu );
  }
  break;
  case SIZE_Nx2N:
  {
    CHECK( !CU::isInter( cu ), "Nx2N split applied to non inter CU" );

    UnitArea pu( cu );

    for( UInt i = 0; i < pu.blocks.size(); i++ )
    {
      pu.blocks[i].width >>= 1;
    }

    pus.push_back( pu );

    for( UInt i = 0; i < pu.blocks.size(); i++ )
    {
      pu.blocks[i].x += pu.blocks[i].width;
    }

    pus.push_back( pu );
  }
  break;
  case SIZE_NxN:
  {
    Bool splitChroma = true;

    if( CU::isIntra( cu ) )
    {
      CHECK( cu.lumaSize().width != ( cu.cs->sps->getMaxCUWidth() >> cu.cs->sps->getLog2DiffMaxMinCodingBlockSize() ), "NxN intra split performed at not maximal CU depth" );
      splitChroma = enable4ChromaPUsInIntraNxNCU(cu.chromaFormat);
    }

    for( UInt i = 0; i < 4; i++ )
    {
      UnitArea pu( cu.chromaFormat );

      // split the luma channel
      {
        const CompArea& Y = cu.Y();
        Position pos = Y;

        if ((i & 1) == 1) pos.x += Y.width  / 2;
        if (i > 1)        pos.y += Y.height / 2;

        pu.blocks.push_back(CompArea(COMPONENT_Y, cu.chromaFormat, pos, Size(Y.width / 2, Y.height / 2)));
      }

      if (!splitChroma)
      {
        if (i == 0)
        {
          // add the not-split chroma channel
          for (UInt c = 1; c < cu.blocks.size(); c++)
          {
            pu.blocks.push_back(cu.blocks[c]);
          }
        }
        else
        {
          for (UInt c = 1; c < cu.blocks.size(); c++)
          {
            pu.blocks.push_back(CompArea());
          }
        }
      }

      if (splitChroma)
      {
        // split the chroma channel
        for (UInt c = 1; c < cu.blocks.size(); c++)
        {
          const CompArea& area = cu.blocks[c];
          Position pos = area;

          if ((i & 1) == 1) pos.x += area.width  / 2;
          if (i > 1)        pos.y += area.height / 2;

          pu.blocks.push_back(CompArea(area.compID, cu.chromaFormat, pos, Size(area.width / 2, area.height / 2)));
        }
      }

      pus.push_back(pu);
    }
  }
  break;
  case SIZE_2NxnU:
  {
    CHECK( !CU::isInter( cu ), "2NxnU split applied to non inter CU" );

    UnitArea pu(cu);

    for (UInt i = 0; i < pu.blocks.size(); i++)
    {
      pu.blocks[i].height >>= 2;
    }

    pus.push_back(pu);

    for (UInt i = 0; i < pu.blocks.size(); i++)
    {
      pu.blocks[i].y      +=  pu.blocks[i].height;
      pu.blocks[i].height += (pu.blocks[i].height << 1);
    }

    pus.push_back(pu);
  }
  break;
  case SIZE_2NxnD:
  {
    CHECK( !CU::isInter( cu ), "2NxnD split applied to non inter CU" );

    UnitArea pu( cu );

    for( UInt i = 0; i < pu.blocks.size(); i++ )
    {
      pu.blocks[i].height >>= 1;
      pu.blocks[i].height  += ( pu.blocks[i].height >> 1 );
    }

    pus.push_back( pu );

    for( UInt i = 0; i < pu.blocks.size(); i++ )
    {
      pu.blocks[i].y      += pu.blocks[i].height;
      pu.blocks[i].height /= 3;
    }

    pus.push_back( pu );
  }
  break;
  case SIZE_nLx2N:
  {
    CHECK( !CU::isInter( cu ), "nLx2N split applied to non inter CU" );

    UnitArea pu( cu );

    for( UInt i = 0; i < pu.blocks.size(); i++ )
    {
      pu.blocks[i].width >>= 2;
    }

    pus.push_back(pu);

    for( UInt i = 0; i < pu.blocks.size(); i++ )
    {
      pu.blocks[i].x     +=  pu.blocks[i].width;
      pu.blocks[i].width += ( pu.blocks[i].width << 1 );
    }

    pus.push_back( pu );
  }
  break;
  case SIZE_nRx2N:
  {
    CHECK( !CU::isInter( cu ), "nRx2N split applied to non inter CU" );

    UnitArea pu( cu );

    for( UInt i = 0; i < pu.blocks.size(); i++ )
    {
      pu.blocks[i].width >>= 1;
      pu.blocks[i].width  += ( pu.blocks[i].width >> 1 );
    }

    pus.push_back(pu);

    for( UInt i = 0; i < pu.blocks.size(); i++ )
    {
      pu.blocks[i].x     += pu.blocks[i].width;
      pu.blocks[i].width /= 3;
    }

    pus.push_back( pu );
  }
  break;
  default:
    THROW( "Unknown PU partition mode" );
    break;
  }

  return std::move( pus );
}

Partitioning PartitionerImpl::getCUSubPartitions( const UnitArea &cuArea, const CodingStructure &cs, const PartSplit _splitType /*= CU_QUAD_SPLIT*/ )
{
  const PartSplit splitType = _splitType;

  if( splitType == CU_QUAD_SPLIT )
  {
    if( !cs.pcv->noChroma2x2 )
    {
      Partitioning sub;

      sub.resize( 4, cuArea );

      for( UInt i = 0; i < 4; i++ )
      {
        for( auto &blk : sub[i].blocks )
        {
          blk.height >>= 1;
          blk.width  >>= 1;
          if( i >= 2 ) blk.y += blk.height;
          if( i &  1 ) blk.x += blk.width;
        }

        CHECK( sub[i].lumaSize().height < MIN_TU_SIZE, "the split causes the block to be smaller than the minimal TU size" );
      }

      return sub;
    }
    else
    {
      const UInt minCUSize = ( cs.sps->getMaxCUWidth() >> cs.sps->getMaxCodingDepth() );

      Bool canSplit = cuArea.lumaSize().width > minCUSize && cuArea.lumaSize().height > minCUSize;

      Partitioning ret;

      if( cs.slice->getSliceType() == I_SLICE )
      {
        canSplit &= cuArea.lumaSize().width > cs.pcv->minCUWidth && cuArea.lumaSize().height > cs.pcv->minCUHeight;
      }

      if( canSplit )
      {
        ret.resize( 4 );

        if( cuArea.chromaFormat == CHROMA_400 )
        {
          CompArea  blkY = cuArea.Y();
          blkY.width >>= 1;
          blkY.height >>= 1;
          ret[0]  = UnitArea( cuArea.chromaFormat, blkY );
          blkY.x += blkY.width;
          ret[1]  = UnitArea( cuArea.chromaFormat, blkY );
          blkY.x -= blkY.width;
          blkY.y += blkY.height;
          ret[2]  = UnitArea( cuArea.chromaFormat, blkY );
          blkY.x += blkY.width;
          ret[3]  = UnitArea( cuArea.chromaFormat, blkY );
        }
        else
        {
          for( UInt i = 0; i < 4; i++ )
          {
            ret[i] = cuArea;

            CompArea &blkY  = ret[i].Y();
            CompArea &blkCb = ret[i].Cb();
            CompArea &blkCr = ret[i].Cr();

            blkY.width  /= 2;
            blkY.height /= 2;

            // TODO: get those params from SPS
            if( blkCb.width > 4 )
            {
              blkCb.width  /= 2;
              blkCb.height /= 2;
              blkCr.width  /= 2;
              blkCr.height /= 2;
            }
            else if( i > 0 )
            {
              blkCb = CompArea();
              blkCr = CompArea();
            }

            if( ( i & 1 ) == 1 )
            {
              blkY.x  += blkY .width;
              blkCb.x += blkCb.width;
              blkCr.x += blkCr.width;
            }

            if( i > 1 )
            {
              blkY.y  += blkY .height;
              blkCb.y += blkCb.height;
              blkCr.y += blkCr.height;
            }
          }
        }
      }

      return ret;
    }
  }
  else if( splitType == CU_HORZ_SPLIT )
  {
    Partitioning sub;

    sub.resize(2, cuArea);

    for (UInt i = 0; i < 2; i++)
    {
      for (auto &blk : sub[i].blocks)
      {
        blk.height >>= 1;
        if (i == 1) blk.y += blk.height;
      }

      CHECK(sub[i].lumaSize().height < MIN_TU_SIZE, "the cs split causes the block to be smaller than the minimal TU size");
    }

    return sub;
  }
  else if( splitType == CU_VERT_SPLIT )
  {
    Partitioning sub;

    sub.resize( 2, cuArea );

    for( UInt i = 0; i < 2; i++ )
    {
      for( auto &blk : sub[i].blocks )
      {
        blk.width >>= 1;
        if( i == 1 ) blk.x += blk.width;
      }

      CHECK( sub[i].lumaSize().width < MIN_TU_SIZE, "the split causes the block to be smaller than the minimal TU size" );
    }

    return sub;
  }
  else
  {
    THROW( "Unknown CU sub-partitioning" );
    return Partitioning();
  }
}


Partitioning PartitionerImpl::getTUSubPartitions(const UnitArea &tuArea, const CodingStructure &cs)
{
  Bool canSplit = tuArea.lumaSize().width > MIN_TU_SIZE && tuArea.lumaSize().height > MIN_TU_SIZE;

  Partitioning ret(4);

  if (canSplit)
  {
    if( tuArea.chromaFormat == CHROMA_400 )
    {
      CompArea  blkY  = tuArea.Y();
      blkY.width    >>= 1;
      blkY.height   >>= 1;
      ret[0] = UnitArea( tuArea.chromaFormat, blkY );
      blkY.x         += blkY.width;
      ret[1] = UnitArea( tuArea.chromaFormat, blkY );
      blkY.x         -= blkY.width;
      blkY.y         += blkY.height;
      ret[2] = UnitArea( tuArea.chromaFormat, blkY );
      blkY.x         += blkY.width;
      ret[3] = UnitArea( tuArea.chromaFormat, blkY );
    }
    else
    {
      Bool splitChroma = false;

      for (UInt i = 0; i < 4; i++)
      {
        ret[i] = tuArea;

        CompArea& blkY  = ret[i].blocks[ COMPONENT_Y  ];
        CompArea& blkCb = ret[i].blocks[ COMPONENT_Cb ];
        CompArea& blkCr = ret[i].blocks[ COMPONENT_Cr ];

        blkY.width  /= 2;
        blkY.height /= 2;

        if (blkCb.width > MIN_TU_SIZE)
        {
          blkCb.width  /= 2;
          blkCb.height /= 2;
          blkCr.width  /= 2;
          blkCr.height /= 2;
          splitChroma   = true;
        }
        else if (i < 3)
        {
          blkCb = CompArea();
          blkCr = CompArea();
        }

        if ((i & 1) == 1)
        {
          blkY. x += blkY. width;
          if (splitChroma) blkCb.x += blkCb.width;
          if (splitChroma) blkCr.x += blkCr.width;
        }

        if (i > 1)
        {
          blkY. y += blkY. height;
          if (splitChroma) blkCb.y += blkCb.height;
          if (splitChroma) blkCr.y += blkCr.height;
        }
      }
    }
  }
  return std::move(ret);
}
