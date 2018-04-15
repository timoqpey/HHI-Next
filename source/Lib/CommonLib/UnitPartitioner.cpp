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

PartSplit applyModifier( const PartSplit split, const SplitModifier mod )
{
  if( split == CU_QUAD_SPLIT )
  {
    CHECK( mod != SPLIT_MOD_12, "Only mod 1/2 is allowed for quad split" );
    return split;
  }

  if( split == CU_HORZ_SPLIT )
  {
    switch( mod )
    {
    case SPLIT_MOD_12: return CU_HORZ_SPLIT;            break;
    case SPLIT_MOD_14: return CU_HORZ_SPLIT_14;         break;
    case SPLIT_MOD_34: return CU_HORZ_SPLIT_34;         break;
    case SPLIT_MOD_38: return CU_HORZ_SPLIT_38;         break;
    case SPLIT_MOD_58: return CU_HORZ_SPLIT_58;         break;
    case SPLIT_MOD_13: return CU_HORZ_SPLIT_13;         break;
    case SPLIT_MOD_23: return CU_HORZ_SPLIT_23;         break;
    case SPLIT_MOD_15: return CU_HORZ_SPLIT_15;         break;
    case SPLIT_MOD_25: return CU_HORZ_SPLIT_25;         break;
    case SPLIT_MOD_35: return CU_HORZ_SPLIT_35;         break;
    case SPLIT_MOD_45: return CU_HORZ_SPLIT_45;         break;
    default: THROW( "Invalid split modifier " << mod ); break;
    }
  }
  else
  {
    CHECK( split != CU_VERT_SPLIT, "Only horizontal and vertical split can be modified" );

    switch( mod )
    {
    case SPLIT_MOD_12: return CU_VERT_SPLIT;            break;
    case SPLIT_MOD_14: return CU_VERT_SPLIT_14;         break;
    case SPLIT_MOD_34: return CU_VERT_SPLIT_34;         break;
    case SPLIT_MOD_38: return CU_VERT_SPLIT_38;         break;
    case SPLIT_MOD_58: return CU_VERT_SPLIT_58;         break;
    case SPLIT_MOD_13: return CU_VERT_SPLIT_13;         break;
    case SPLIT_MOD_23: return CU_VERT_SPLIT_23;         break;
    case SPLIT_MOD_15: return CU_VERT_SPLIT_15;         break;
    case SPLIT_MOD_25: return CU_VERT_SPLIT_25;         break;
    case SPLIT_MOD_35: return CU_VERT_SPLIT_35;         break;
    case SPLIT_MOD_45: return CU_VERT_SPLIT_45;         break;
    default: THROW( "Invalid split modifier " << mod ); break;
    }
  }
}

PartSplit getBaseSplit( const PartSplit split )
{
  if( split == CU_DONT_SPLIT || split == CTU_LEVEL ) return split;
  if( split == CU_QUAD_SPLIT ) return split;

  switch( split )
  {
  case CU_HORZ_SPLIT:
  case CU_HORZ_SPLIT_14:
  case CU_HORZ_SPLIT_34:
  case CU_HORZ_SPLIT_38:
  case CU_HORZ_SPLIT_58:
  case CU_HORZ_SPLIT_13:
  case CU_HORZ_SPLIT_23:
  case CU_HORZ_SPLIT_15:
  case CU_HORZ_SPLIT_25:
  case CU_HORZ_SPLIT_35:
  case CU_HORZ_SPLIT_45:
    return CU_HORZ_SPLIT;
    break;
  case CU_VERT_SPLIT:
  case CU_VERT_SPLIT_14:
  case CU_VERT_SPLIT_34:
  case CU_VERT_SPLIT_38:
  case CU_VERT_SPLIT_58:
  case CU_VERT_SPLIT_13:
  case CU_VERT_SPLIT_23:
  case CU_VERT_SPLIT_15:
  case CU_VERT_SPLIT_25:
  case CU_VERT_SPLIT_35:
  case CU_VERT_SPLIT_45:
    return CU_VERT_SPLIT;
    break;
  default:
    break;
  }

  THROW( "The split " << split << " has no bas split" );

  return CU_DONT_SPLIT;
}

SplitModifier getModifier( const PartSplit split )
{
  switch( split )
  {
  case CU_QUAD_SPLIT:
  case CU_HORZ_SPLIT:
  case CU_VERT_SPLIT:
    return SPLIT_MOD_12;
    break;
  case CU_HORZ_SPLIT_14:
  case CU_VERT_SPLIT_14:
    return SPLIT_MOD_14;
    break;
  case CU_HORZ_SPLIT_34:
  case CU_VERT_SPLIT_34:
    return SPLIT_MOD_34;
    break;
  case CU_HORZ_SPLIT_38:
  case CU_VERT_SPLIT_38:
    return SPLIT_MOD_38;
    break;
  case CU_HORZ_SPLIT_58:
  case CU_VERT_SPLIT_58:
    return SPLIT_MOD_58;
    break;
  case CU_HORZ_SPLIT_13:
  case CU_VERT_SPLIT_13:
    return SPLIT_MOD_13;
    break;
  case CU_HORZ_SPLIT_23:
  case CU_VERT_SPLIT_23:
    return SPLIT_MOD_23;
    break;
  case CU_HORZ_SPLIT_15:
  case CU_VERT_SPLIT_15:
    return SPLIT_MOD_15;
    break;
  case CU_HORZ_SPLIT_25:
  case CU_VERT_SPLIT_25:
    return SPLIT_MOD_25;
    break;
  case CU_HORZ_SPLIT_35:
  case CU_VERT_SPLIT_35:
    return SPLIT_MOD_35;
    break;
  case CU_HORZ_SPLIT_45:
  case CU_VERT_SPLIT_45:
    return SPLIT_MOD_45;
    break;
  default:
    THROW( "The split does not contain a modifier " << split );
    return NUM_SPLIT_MOD;
  }
}

PartSplit getSplitSide( const SplitModifier mod )
{
  switch( mod )
  {
  case SPLIT_MOD_13:
  case SPLIT_MOD_14:
  case SPLIT_MOD_15:
  case SPLIT_MOD_25:
  case SPLIT_MOD_38:
    return CU_L_SPLIT;
    break;
  case SPLIT_MOD_23:
  case SPLIT_MOD_34:
  case SPLIT_MOD_35:
  case SPLIT_MOD_45:
  case SPLIT_MOD_58:
    return CU_R_SPLIT;
    break;
  default:
    return  NUM_PART_SPLIT;
  }
}

SizeClass getSizeClass( const unsigned size )
{
  unsigned lastLog2 = 1 << g_aucLog2[size];

  if( size == lastLog2 )
  {
    return SIZE_LOG2;
  }
  else if( size == ( lastLog2 + ( lastLog2 >> 1 ) ) )
  {
    return SIZE_32_LOG2;
  }
  else if( size == ( lastLog2 + ( lastLog2 >> 2 ) ) )
  {
    return SIZE_54_LOG2;
  }
  else
  {
    THROW( "Invalid size: " << size );
    return NUM_SIZE_CLASSES;
  }
}

double getSplitRatio( const SplitModifier mod )
{
  switch( mod )
  {
  case SPLIT_MOD_12: return .5;   break;
  case SPLIT_MOD_14: return .25;  break;
  case SPLIT_MOD_34: return .75;  break;
  case SPLIT_MOD_38: return .375; break;
  case SPLIT_MOD_58: return .625; break;
  case SPLIT_MOD_13: return .334; break; // for round-to-0 compatibility
  case SPLIT_MOD_23: return .667; break; // for round-to-0 compatibility
  case SPLIT_MOD_15: return .2;   break;
  case SPLIT_MOD_25: return .4;   break;
  case SPLIT_MOD_35: return .6;   break;
  case SPLIT_MOD_45: return .8;   break;
  default: THROW( "Invalid split modifier " << mod ); break;
  }
  return 0.0;
}

bool isPseudoSplit( const PartSplit split )
{
  return split >= NUM_PART_SPLIT;
}

PartLevel::PartLevel()
: split               ( CU_DONT_SPLIT )
, parts               (               )
, idx                 ( 0u            )
, checkdIfImplicit    ( false         )
, isImplicit          ( false         )
, implicitSplit       ( CU_DONT_SPLIT )
, firstSubPartSplit   ( CU_DONT_SPLIT )
, canQtSplit          ( true          )
, pseudoSplit         ( CU_DONT_SPLIT )
{
  memset( canModify, 0, sizeof( canModify ) );
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
, pseudoSplit         ( CU_DONT_SPLIT )
{
  memset( canModify, 0, sizeof( canModify ) );
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
, pseudoSplit         ( CU_DONT_SPLIT                        )
{
  memset( canModify, 0, sizeof( canModify ) );
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
  cu.mtDepth     = currMtDepth;
  cu.qtDepth     = currQtDepth;
  cu.splitSeries = getSplitSeries();
}

void Partitioner::copyState( const Partitioner& other )
{
  m_partStack = other.m_partStack;
  currBtDepth = other.currBtDepth;
  currQtDepth = other.currQtDepth;
  currDepth   = other.currDepth;
  currMtDepth = other.currMtDepth;
  currGtDepth = other.currGtDepth;
  currTrDepth = other.currTrDepth;
#if !HM_QTBT_ONLY_QT_IMPLICIT
  currImplicitBtDepth
              = other.currImplicitBtDepth;
#endif
  chType      = other.chType;
#ifdef _DEBUG
  m_currArea  = other.m_currArea;
#endif
}

//////////////////////////////////////////////////////////////////////////
// AdaptiveDepthPartitioner class
//////////////////////////////////////////////////////////////////////////

void AdaptiveDepthPartitioner::setMaxMinDepth( unsigned& minDepth, unsigned& maxDepth, const CodingStructure& cs ) const
{
  unsigned          stdMinDepth = 0;
  unsigned          stdMaxDepth = ( ( cs.sps->getSpsNext().getUseGenBinSplit() || cs.sps->getSpsNext().getUseQTBT() )
                                        ? g_aucLog2[cs.sps->getSpsNext().getCTUSize()] - g_aucLog2[cs.sps->getSpsNext().getMinQTSize( cs.slice->getSliceType(), chType )]
                                        : cs.sps->getLog2DiffMaxMinCodingBlockSize() );
  const Position    pos         = currArea().blocks[chType].pos();
  const unsigned    curSliceIdx = cs.slice->getIndependentSliceIdx();
  const unsigned    curTileIdx  = cs.picture->tileMap->getTileIdxMap( currArea().lumaPos() );

  const CodingUnit* cuLeft        = cs.getCURestricted( pos.offset( -1,                                                                        0 ), curSliceIdx, curTileIdx, chType );
  const CodingUnit* cuBelowLeft   = cs.getCURestricted( pos.offset( -1, cs.pcv->minCUHeight >> getChannelTypeScaleY( chType, cs.pcv->chrFormat ) ), curSliceIdx, curTileIdx, chType );  // should use actual block size instead of minCU size
  const CodingUnit* cuAbove       = cs.getCURestricted( pos.offset(  0,                                                                       -1 ), curSliceIdx, curTileIdx, chType );
  const CodingUnit* cuAboveRight  = cs.getCURestricted( pos.offset( cs.pcv->minCUWidth >> getChannelTypeScaleX( chType, cs.pcv->chrFormat ),  -1 ), curSliceIdx, curTileIdx, chType );  // should use actual block size instead of minCU size

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

Void HEVCPartitioner::initCtu( const UnitArea& ctuArea, const ChannelType _chType, const Slice& slice )
{
#if _DEBUG
  m_currArea  = ctuArea;
#endif
  currDepth   = 0;
  currTrDepth = 0;
  currBtDepth = 0;
  currMtDepth = 0;
  currQtDepth = 0;
  chType      = _chType;

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
      unsigned minQtSize = cs.pcv->getMinQtSize( *cs.slice, chType );
      if( currArea().lwidth() <=  minQtSize || currArea().lheight() <= minQtSize ) return false;

      return true;
    }
    break;
  case CU_HORZ_SPLIT:
  case CU_VERT_SPLIT:
  case CU_BT_SPLIT:
  case CU_MT_SPLIT:
  case CU_TRIV_SPLIT:
  case CU_TRIH_SPLIT:
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
  const Position &prevPos = currArea().blocks[chType].pos();

  unsigned currIdx = ++m_partStack.back().idx;

  m_partStack.back().checkdIfImplicit = false;
  m_partStack.back().isImplicit       = false;

  if( currIdx == 1 )
  {
    const CodingUnit* prevCU = cs.getCU( prevPos, chType );
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

Void QTBTPartitioner::initCtu( const UnitArea& ctuArea, const ChannelType _chType, const Slice& slice )
{
#if _DEBUG
  m_currArea = ctuArea;
#endif
  currDepth   = 0;
  currTrDepth = 0;
  currBtDepth = 0;
  currMtDepth = 0;
  currQtDepth = 0;
#if !HM_QTBT_ONLY_QT_IMPLICIT
  currImplicitBtDepth = 0;
#endif
  chType      = _chType;

  m_partStack.clear();
  m_partStack.push_back( PartLevel( CTU_LEVEL, Partitioning{ ctuArea } ) );
}

Void QTBTPartitioner::splitCurrArea( const PartSplit split, const CodingStructure& cs )
{
  CHECKD( !canSplit( split, cs ), "Trying to apply a prohibited split!" );

#if !HM_QTBT_ONLY_QT_IMPLICIT
  bool isImplicit = isSplitImplicit( split, cs );
#endif
  bool canQtSplit = canSplit( CU_QUAD_SPLIT, cs );

  switch( split )
  {
  case CU_QUAD_SPLIT:
    m_partStack.push_back( PartLevel( split, PartitionerImpl::getCUSubPartitions( currArea(), cs ) ) );
    break;
  case CU_HORZ_SPLIT_14:
  case CU_VERT_SPLIT_14:
  case CU_HORZ_SPLIT_34:
  case CU_VERT_SPLIT_34:
    CHECK( ( cs.sps->getSpsNext().getMTTMode() & 2 ) != 2, "1/4 and 3/4 splits are not allowed" );
  case CU_HORZ_SPLIT:
  case CU_VERT_SPLIT:
    CHECK( !cs.sps->getSpsNext().getUseQTBT(), "QTBT disabled" );
    m_partStack.push_back( PartLevel( split, PartitionerImpl::getCUSubPartitions( currArea(), cs, split ) ) );
    break;
  case CU_TRIH_SPLIT:
  case CU_TRIV_SPLIT:
    CHECK( ( cs.sps->getSpsNext().getMTTMode() & 1 ) != 1, "Triple splits are not allowed" );
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

  if( split == CU_HORZ_SPLIT || split == CU_VERT_SPLIT || split == CU_TRIH_SPLIT || split == CU_TRIV_SPLIT || split == CU_HORZ_SPLIT_14 || split == CU_HORZ_SPLIT_34 || split == CU_VERT_SPLIT_14 || split == CU_VERT_SPLIT_34 )
  {
    currBtDepth++;
#if !HM_QTBT_ONLY_QT_IMPLICIT
    if( isImplicit ) currImplicitBtDepth++;
#endif
    currMtDepth++;

    if( split == CU_TRIH_SPLIT || split == CU_TRIV_SPLIT )
    {
      // first and last part of triple split are equivalent to double bt split
      currBtDepth++;
    }
    m_partStack.back().canQtSplit = canQtSplit;
  }
  else
  {
    CHECK( currBtDepth > 0, "Cannot split a non-square area other than with a binary split" );
    CHECK( currMtDepth > 0, "Cannot split a non-square area other than with a binary split" );
    currMtDepth = 0;
    currBtDepth = 0;
    currQtDepth++;
  }
}

bool QTBTPartitioner::canSplit( const PartSplit split, const CodingStructure &cs )
{
  const PartSplit implicitSplit = getImplicitSplit( cs );

  // the minimal and maximal sizes are given in luma samples
  const CompArea area           = currArea().Y();

#if !HM_QTBT_ONLY_QT_IMPLICIT
  const unsigned maxBTD         = cs.pcv->getMaxBtDepth( *cs.slice, chType ) + currImplicitBtDepth;
#else
  const unsigned maxBTD         = cs.pcv->getMaxBtDepth( *cs.slice, chType );
#endif
  const unsigned maxBtSize      = cs.pcv->getMaxBtSize( *cs.slice, chType );
  const unsigned minBtSize      = cs.pcv->getMinBtSize( *cs.slice, chType );
  const unsigned maxTtSize      = cs.pcv->getMaxTtSize( *cs.slice, chType );
  const unsigned minTtSize      = cs.pcv->getMinTtSize( *cs.slice, chType );

  const PartSplit prevSplit     = m_partStack.back().firstSubPartSplit;
  const PartSplit lastSplit     = m_partStack.back().split;
  const PartSplit perpSplit     = lastSplit == CU_HORZ_SPLIT ? CU_VERT_SPLIT : CU_HORZ_SPLIT;
  const PartSplit perpTriSp     = lastSplit == CU_HORZ_SPLIT ? CU_TRIV_SPLIT : CU_TRIH_SPLIT;
  const PartSplit parlSplit     = lastSplit == CU_TRIH_SPLIT ? CU_HORZ_SPLIT : CU_VERT_SPLIT;

  if( isNonLog2BlockSize( currArea().Y() ) )
  {
    return false;
  }

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

    unsigned minQtSize = cs.pcv->getMinQtSize( *cs.slice, chType );
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
    if( ( lastSplit == CU_TRIH_SPLIT || lastSplit == CU_TRIV_SPLIT ) && currPartIdx() == 1 && split == parlSplit )
    {
      return false;
    }
  }
  case CU_TRIH_SPLIT:
  case CU_TRIV_SPLIT:
  {
    if( !cs.slice->isIntra() && m_partStack.back().idx == 1 && implicitSplit == CU_DONT_SPLIT && ( lastSplit == CU_HORZ_SPLIT || lastSplit == CU_VERT_SPLIT ) )
    {
      if( split == perpTriSp && prevSplit == perpTriSp )
      {
        return false;
      }
    }
  }
  case CU_HORZ_SPLIT_14:
  case CU_HORZ_SPLIT_34:
  case CU_VERT_SPLIT_14:
  case CU_VERT_SPLIT_34:

#if !HM_QTBT_ONLY_QT_IMPLICIT
    if( implicitSplit == split )                                   return true;
    if( implicitSplit != CU_DONT_SPLIT && implicitSplit != split ) return false;
#endif

  case CU_MT_SPLIT:
  case CU_BT_SPLIT:
  {
    if( !cs.sps->getSpsNext().getUseQTBT() )                  return false;
    if( currMtDepth >= maxBTD )                               return false;
    if(      ( area.width <= minBtSize && area.height <= minBtSize )
        && ( ( area.width <= minTtSize && area.height <= minTtSize ) || cs.sps->getSpsNext().getMTTMode() == 0 ) ) return false;
    if(      ( area.width > maxBtSize || area.height > maxBtSize )
        && ( ( area.width > maxTtSize || area.height > maxTtSize ) || cs.sps->getSpsNext().getMTTMode() == 0 ) ) return false;
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
  case CU_HORZ_SPLIT_14:
  case CU_HORZ_SPLIT_34:
    if( ( cs.sps->getSpsNext().getMTTMode() & 2 ) != 2 )          return false;
    if( area.height <= 2 * minBtSize || area.height > maxBtSize ) return false;
    break;
  case CU_VERT_SPLIT_14:
  case CU_VERT_SPLIT_34:
    if( ( cs.sps->getSpsNext().getMTTMode() & 2 ) != 2 )          return false;
    if( area.width <= 2 * minBtSize || area.width > maxBtSize )   return false;
    break;
  case CU_TRIH_SPLIT:
    if( ( cs.sps->getSpsNext().getMTTMode() & 1 ) != 1 )          return false;
    if( area.height <= 2 * minTtSize || area.height > maxTtSize ) return false;
    break;
  case CU_TRIV_SPLIT:
    if( ( cs.sps->getSpsNext().getMTTMode() & 1 ) != 1 )          return false;
    if( area.width <= 2 * minTtSize || area.width > maxTtSize )   return false;
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

#if HM_QTBT_ONLY_QT_IMPLICIT
    if( !isBlInPic || !isTrInPic )
    {
      split = CU_QUAD_SPLIT;
    }
#else
    const CompArea& area      = currArea().Y();
    const unsigned maxBtSize  = cs.pcv->getMaxBtSize( *cs.slice, chType );
    const bool isBtAllowed    = area.width <= maxBtSize && area.height <= maxBtSize;

    if( !isBlInPic && !isTrInPic )
    {
      split = CU_QUAD_SPLIT;
    }
    else if( !isBlInPic && isBtAllowed )
    {
      split = CU_HORZ_SPLIT;
    }
    else if( !isTrInPic && isBtAllowed )
    {
      split = CU_VERT_SPLIT;
    }
    else if( !isBlInPic || !isTrInPic )
    {
      split = CU_QUAD_SPLIT;
    }
#endif
  }

  m_partStack.back().checkdIfImplicit = true;
  m_partStack.back().isImplicit = split != CU_DONT_SPLIT;
  m_partStack.back().implicitSplit = split;

  return split;
}

Void QTBTPartitioner::exitCurrSplit()
{
  PartSplit currSplit = m_partStack.back().split;
  unsigned  currIdx = m_partStack.back().idx;

  m_partStack.pop_back();

  CHECK( currDepth == 0, "depth is '0', although a split was performed" );
  currDepth--;
#if _DEBUG
  m_currArea = m_partStack.back().parts[m_partStack.back().idx];
#endif

  if( currSplit == CU_HORZ_SPLIT || currSplit == CU_VERT_SPLIT || currSplit == CU_TRIH_SPLIT || currSplit == CU_TRIV_SPLIT || currSplit == CU_HORZ_SPLIT_14 || currSplit == CU_HORZ_SPLIT_34 || currSplit == CU_VERT_SPLIT_14 || currSplit == CU_VERT_SPLIT_34 )
  {
    CHECK( !m_partStack.back().checkdIfImplicit, "Didn't check if the current split is implicit" );
    CHECK( currBtDepth == 0, "BT depth is '0', athough a BT split was performed" );
    CHECK( currMtDepth == 0, "MT depth is '0', athough a BT split was performed" );
    currMtDepth--;
#if !HM_QTBT_ONLY_QT_IMPLICIT
    if( m_partStack.back().isImplicit ) currImplicitBtDepth--;
#endif
    currBtDepth--;
    if( ( currSplit == CU_TRIH_SPLIT || currSplit == CU_TRIV_SPLIT ) && currIdx != 1 )
    {
      CHECK( currBtDepth == 0, "BT depth is '0', athough a TT split was performed" );
      currBtDepth--;
    }
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
  const Position &prevPos = currArea().blocks[chType].pos();

  unsigned currIdx = ++m_partStack.back().idx;

  m_partStack.back().checkdIfImplicit = false;
  m_partStack.back().isImplicit = false;

  if( currIdx == 1 )
  {
    const CodingUnit* prevCU = cs.getCU( prevPos, chType );
    m_partStack.back().firstSubPartSplit = prevCU ? CU::getSplitAtDepth( *prevCU, currDepth ) : CU_DONT_SPLIT;
  }

  if( currIdx < m_partStack.back().parts.size() )
  {
    if( m_partStack.back().split == CU_TRIH_SPLIT || m_partStack.back().split == CU_TRIV_SPLIT )
    {
      // adapt the current bt depth
      if( currIdx == 1 ) currBtDepth--;
      else               currBtDepth++;
    }
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
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
// GenBinSplitPartitioner
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////
// Structures defining prohibited part level combinations
//////////////////////////////////////////////////////////////////////////

#define PROHIBIT_ALL        -2
#define PROHIBIT_SAME_DIR   -3
#define PROHIBIT_PERP_DIR   -4
#define MAX_PROHIBIT_FRAMES  3
#define INTER_FRAME         -5

#define PROHIBIT_REDUNDANT_SPLITS 1

struct ProhibitedSplitFrame
{
  int split;
  int mod;
  int idx;
  int firstPartSplit;
  int firstPartMod;
};

struct ProhibitedSplit
{
  int sliceType;
  int split;
  int mod;

  static_vector<ProhibitedSplitFrame, MAX_PROHIBIT_FRAMES> splitStack;
};

static std::vector<ProhibitedSplit> prohibitedSplits;

static void initProhibitedSplits( const Slice& slice )
{
  const SPSNext& spsNext = slice.getSPS()->getSpsNext();

  if( spsNext.getMaxBTDepth() >= 3 )
  {
    prohibitedSplits.push_back( ProhibitedSplit{ INTER_FRAME,  PROHIBIT_PERP_DIR, SPLIT_MOD_12, { ProhibitedSplitFrame{ PROHIBIT_SAME_DIR, SPLIT_MOD_12, 1, PROHIBIT_PERP_DIR, SPLIT_MOD_12 } } } ); // is the same as 1/3 split followed by 1/2 split on the second part
    prohibitedSplits.push_back( ProhibitedSplit{ INTER_FRAME,  PROHIBIT_PERP_DIR, SPLIT_MOD_12, { ProhibitedSplitFrame{ PROHIBIT_PERP_DIR, SPLIT_MOD_12, 1, PROHIBIT_PERP_DIR, SPLIT_MOD_12 }, ProhibitedSplitFrame{ CU_QUAD_SPLIT, SPLIT_MOD_12, PROHIBIT_ALL, PROHIBIT_ALL, PROHIBIT_ALL } } } ); // is the same as 1/4 split followed by 2/3 split on the second part
  }

  if( spsNext.getGbsForceSplitToLog2() && !spsNext.getGbsNonLog2CUs() )
  {
    prohibitedSplits.push_back( ProhibitedSplit{ PROHIBIT_ALL, PROHIBIT_ALL, SPLIT_MOD_34, {} } );
  }

  // prohibit redundant parallel configurations
  prohibitedSplits.push_back( ProhibitedSplit{ PROHIBIT_ALL, PROHIBIT_SAME_DIR, SPLIT_MOD_12, { ProhibitedSplitFrame{ PROHIBIT_ALL, SPLIT_MOD_23, 0, PROHIBIT_ALL,      PROHIBIT_ALL } } } ); // is the same as 1/3 split followed by 1/2 split on the second part
  prohibitedSplits.push_back( ProhibitedSplit{ PROHIBIT_ALL, PROHIBIT_SAME_DIR, SPLIT_MOD_13, { ProhibitedSplitFrame{ PROHIBIT_ALL, SPLIT_MOD_34, 0, PROHIBIT_ALL,      PROHIBIT_ALL } } } ); // is the same as 1/4 split followed by 2/3 split on the second part
  prohibitedSplits.push_back( ProhibitedSplit{ PROHIBIT_ALL, PROHIBIT_SAME_DIR, SPLIT_MOD_23, { ProhibitedSplitFrame{ PROHIBIT_ALL, SPLIT_MOD_34, 0, PROHIBIT_ALL,      PROHIBIT_ALL } } } ); // is the same as 1/2 split followed by 1/2 split on the second part

  if( spsNext.getGbsAllowEights() )
  {
    prohibitedSplits.push_back( ProhibitedSplit{ PROHIBIT_ALL, PROHIBIT_SAME_DIR, SPLIT_MOD_13, { ProhibitedSplitFrame{ PROHIBIT_ALL, SPLIT_MOD_35, 0, PROHIBIT_ALL,      PROHIBIT_ALL } } } ); // is the same as 1/5 split followed by 1/2 split on the second part
    prohibitedSplits.push_back( ProhibitedSplit{ PROHIBIT_ALL, PROHIBIT_SAME_DIR, SPLIT_MOD_23, { ProhibitedSplitFrame{ PROHIBIT_ALL, SPLIT_MOD_35, 0, PROHIBIT_ALL,      PROHIBIT_ALL } } } ); // is the same as 2/5 split followed by 1/3 split on the second part
    prohibitedSplits.push_back( ProhibitedSplit{ PROHIBIT_ALL, PROHIBIT_SAME_DIR, SPLIT_MOD_12, { ProhibitedSplitFrame{ PROHIBIT_ALL, SPLIT_MOD_25, 0, PROHIBIT_ALL,      PROHIBIT_ALL } } } ); // is the same as 1/5 split followed by 1/4 split on the second part
    prohibitedSplits.push_back( ProhibitedSplit{ PROHIBIT_ALL, PROHIBIT_SAME_DIR, SPLIT_MOD_12, { ProhibitedSplitFrame{ PROHIBIT_ALL, SPLIT_MOD_45, 0, PROHIBIT_ALL,      PROHIBIT_ALL } } } ); // is the same as 2/5 split followed by 2/3 split on the second part
    prohibitedSplits.push_back( ProhibitedSplit{ PROHIBIT_ALL, PROHIBIT_SAME_DIR, SPLIT_MOD_45, { ProhibitedSplitFrame{ PROHIBIT_ALL, SPLIT_MOD_58, 0, PROHIBIT_ALL,      PROHIBIT_ALL } } } ); // is the same as 1/2 split followed by 1/4 split on the second part
    prohibitedSplits.push_back( ProhibitedSplit{ PROHIBIT_ALL, PROHIBIT_SAME_DIR, SPLIT_MOD_35, { ProhibitedSplitFrame{ PROHIBIT_ALL, SPLIT_MOD_58, 0, PROHIBIT_ALL,      PROHIBIT_ALL } } } ); // is the same as 3/8 split followed by 2/5 split on the second part
    prohibitedSplits.push_back( ProhibitedSplit{ PROHIBIT_ALL, PROHIBIT_SAME_DIR, SPLIT_MOD_14, { ProhibitedSplitFrame{ PROHIBIT_ALL, SPLIT_MOD_45, 0, PROHIBIT_ALL,      PROHIBIT_ALL } } } ); // is the same as 1/5 split followed by 3/4 split on the second part
    prohibitedSplits.push_back( ProhibitedSplit{ PROHIBIT_ALL, PROHIBIT_SAME_DIR, SPLIT_MOD_34, { ProhibitedSplitFrame{ PROHIBIT_ALL, SPLIT_MOD_45, 0, PROHIBIT_ALL,      PROHIBIT_ALL } } } ); // is the same as 3/5 split followed by 1/2 split on the second part

    prohibitedSplits.push_back( ProhibitedSplit{ PROHIBIT_ALL, PROHIBIT_SAME_DIR, SPLIT_MOD_15, { ProhibitedSplitFrame{ PROHIBIT_ALL, SPLIT_MOD_38, 1, PROHIBIT_ALL,      PROHIBIT_ALL } } } ); // is the same as 1/2 split followed by 3/4 split on the first part
  }

  prohibitedSplits.push_back( ProhibitedSplit{ PROHIBIT_ALL, PROHIBIT_SAME_DIR, SPLIT_MOD_13, { ProhibitedSplitFrame{ PROHIBIT_ALL, SPLIT_MOD_14, 1, PROHIBIT_ALL,      PROHIBIT_ALL } } } ); // is the same as 1/2 split followed by 1/2 split on the first part

  if( spsNext.getGbsAllowEights() )
  {
    prohibitedSplits.push_back( ProhibitedSplit{ PROHIBIT_ALL, PROHIBIT_SAME_DIR, SPLIT_MOD_34, { ProhibitedSplitFrame{ PROHIBIT_SAME_DIR, SPLIT_MOD_23, 0, PROHIBIT_ALL, PROHIBIT_ALL }, ProhibitedSplitFrame{ PROHIBIT_ALL, SPLIT_MOD_14, 1, PROHIBIT_ALL, PROHIBIT_ALL } } } ); // is the same as 5/8 split followed by 2/3 split on the second part and 2/5 in the first
  }

  prohibitedSplits.push_back( ProhibitedSplit{ PROHIBIT_ALL, PROHIBIT_SAME_DIR, SPLIT_MOD_13, { ProhibitedSplitFrame{ PROHIBIT_SAME_DIR, SPLIT_MOD_12, 1, PROHIBIT_ALL, PROHIBIT_ALL }, ProhibitedSplitFrame{ PROHIBIT_ALL, SPLIT_MOD_14, 1, PROHIBIT_ALL, PROHIBIT_ALL } } } ); // is the same as 3/4 split followed by 1/3 split on first part and then 3/4 on second
  prohibitedSplits.push_back( ProhibitedSplit{ PROHIBIT_ALL, PROHIBIT_SAME_DIR, SPLIT_MOD_34, { ProhibitedSplitFrame{ PROHIBIT_SAME_DIR, SPLIT_MOD_13, 1, PROHIBIT_ALL, PROHIBIT_ALL }, ProhibitedSplitFrame{ PROHIBIT_ALL, SPLIT_MOD_34, 0, PROHIBIT_ALL, PROHIBIT_ALL } } } ); // is the same as 1/4 split followed by 1/2 split on second part and then 1/3 on second
  prohibitedSplits.push_back( ProhibitedSplit{ PROHIBIT_ALL, PROHIBIT_SAME_DIR, SPLIT_MOD_34, { ProhibitedSplitFrame{ PROHIBIT_SAME_DIR, SPLIT_MOD_23, 0, PROHIBIT_ALL, PROHIBIT_ALL }, ProhibitedSplitFrame{ PROHIBIT_ALL, SPLIT_MOD_14, 1, PROHIBIT_ALL, PROHIBIT_ALL } } } ); // is the same as 3/4 split followed by 1/3 split on first part and then 3/4 on second


  // prohibit redundant perpendicular configuration

  // (1/2 comes before all other splits)
  prohibitedSplits.push_back( ProhibitedSplit{ PROHIBIT_ALL, PROHIBIT_PERP_DIR, SPLIT_MOD_12, { ProhibitedSplitFrame{ PROHIBIT_ALL, SPLIT_MOD_14, 1, PROHIBIT_PERP_DIR, SPLIT_MOD_12 } } } );
  prohibitedSplits.push_back( ProhibitedSplit{ PROHIBIT_ALL, PROHIBIT_PERP_DIR, SPLIT_MOD_12, { ProhibitedSplitFrame{ PROHIBIT_ALL, SPLIT_MOD_34, 1, PROHIBIT_PERP_DIR, SPLIT_MOD_12 } } } );

  prohibitedSplits.push_back( ProhibitedSplit{ PROHIBIT_ALL, PROHIBIT_PERP_DIR, SPLIT_MOD_12, { ProhibitedSplitFrame{ PROHIBIT_ALL, SPLIT_MOD_13, 1, PROHIBIT_PERP_DIR, SPLIT_MOD_12 } } } );
  prohibitedSplits.push_back( ProhibitedSplit{ PROHIBIT_ALL, PROHIBIT_PERP_DIR, SPLIT_MOD_12, { ProhibitedSplitFrame{ PROHIBIT_ALL, SPLIT_MOD_23, 1, PROHIBIT_PERP_DIR, SPLIT_MOD_12 } } } );

  if( spsNext.getGbsAllowEights() )
  {
    prohibitedSplits.push_back( ProhibitedSplit{ PROHIBIT_ALL, PROHIBIT_PERP_DIR, SPLIT_MOD_12, { ProhibitedSplitFrame{ PROHIBIT_ALL, SPLIT_MOD_15, 1, PROHIBIT_PERP_DIR, SPLIT_MOD_12 } } } );
    prohibitedSplits.push_back( ProhibitedSplit{ PROHIBIT_ALL, PROHIBIT_PERP_DIR, SPLIT_MOD_12, { ProhibitedSplitFrame{ PROHIBIT_ALL, SPLIT_MOD_25, 1, PROHIBIT_PERP_DIR, SPLIT_MOD_12 } } } );
    prohibitedSplits.push_back( ProhibitedSplit{ PROHIBIT_ALL, PROHIBIT_PERP_DIR, SPLIT_MOD_12, { ProhibitedSplitFrame{ PROHIBIT_ALL, SPLIT_MOD_35, 1, PROHIBIT_PERP_DIR, SPLIT_MOD_12 } } } );
    prohibitedSplits.push_back( ProhibitedSplit{ PROHIBIT_ALL, PROHIBIT_PERP_DIR, SPLIT_MOD_12, { ProhibitedSplitFrame{ PROHIBIT_ALL, SPLIT_MOD_45, 1, PROHIBIT_PERP_DIR, SPLIT_MOD_12 } } } );
  }

  // (1/4, 3/4 comes before  comes before all splits other than 1/2)
  prohibitedSplits.push_back( ProhibitedSplit{ PROHIBIT_ALL, PROHIBIT_PERP_DIR, SPLIT_MOD_14, { ProhibitedSplitFrame{ PROHIBIT_ALL, SPLIT_MOD_34, 1, PROHIBIT_PERP_DIR, SPLIT_MOD_14 } } } );

  prohibitedSplits.push_back( ProhibitedSplit{ PROHIBIT_ALL, PROHIBIT_PERP_DIR, SPLIT_MOD_14, { ProhibitedSplitFrame{ PROHIBIT_ALL, SPLIT_MOD_13, 1, PROHIBIT_PERP_DIR, SPLIT_MOD_14 } } } );
  prohibitedSplits.push_back( ProhibitedSplit{ PROHIBIT_ALL, PROHIBIT_PERP_DIR, SPLIT_MOD_14, { ProhibitedSplitFrame{ PROHIBIT_ALL, SPLIT_MOD_23, 1, PROHIBIT_PERP_DIR, SPLIT_MOD_14 } } } );

  if( spsNext.getGbsAllowEights() )
  {
    prohibitedSplits.push_back( ProhibitedSplit{ PROHIBIT_ALL, PROHIBIT_PERP_DIR, SPLIT_MOD_14, { ProhibitedSplitFrame{ PROHIBIT_ALL, SPLIT_MOD_15, 1, PROHIBIT_PERP_DIR, SPLIT_MOD_14 } } } );
    prohibitedSplits.push_back( ProhibitedSplit{ PROHIBIT_ALL, PROHIBIT_PERP_DIR, SPLIT_MOD_14, { ProhibitedSplitFrame{ PROHIBIT_ALL, SPLIT_MOD_25, 1, PROHIBIT_PERP_DIR, SPLIT_MOD_14 } } } );
    prohibitedSplits.push_back( ProhibitedSplit{ PROHIBIT_ALL, PROHIBIT_PERP_DIR, SPLIT_MOD_14, { ProhibitedSplitFrame{ PROHIBIT_ALL, SPLIT_MOD_35, 1, PROHIBIT_PERP_DIR, SPLIT_MOD_14 } } } );
    prohibitedSplits.push_back( ProhibitedSplit{ PROHIBIT_ALL, PROHIBIT_PERP_DIR, SPLIT_MOD_14, { ProhibitedSplitFrame{ PROHIBIT_ALL, SPLIT_MOD_45, 1, PROHIBIT_PERP_DIR, SPLIT_MOD_14 } } } );
  }

  prohibitedSplits.push_back( ProhibitedSplit{ PROHIBIT_ALL, PROHIBIT_PERP_DIR, SPLIT_MOD_34, { ProhibitedSplitFrame{ PROHIBIT_ALL, SPLIT_MOD_13, 1, PROHIBIT_PERP_DIR, SPLIT_MOD_34 } } } );
  prohibitedSplits.push_back( ProhibitedSplit{ PROHIBIT_ALL, PROHIBIT_PERP_DIR, SPLIT_MOD_34, { ProhibitedSplitFrame{ PROHIBIT_ALL, SPLIT_MOD_23, 1, PROHIBIT_PERP_DIR, SPLIT_MOD_34 } } } );

  if( spsNext.getGbsAllowEights() )
  {
    prohibitedSplits.push_back( ProhibitedSplit{ PROHIBIT_ALL, PROHIBIT_PERP_DIR, SPLIT_MOD_34, { ProhibitedSplitFrame{ PROHIBIT_ALL, SPLIT_MOD_15, 1, PROHIBIT_PERP_DIR, SPLIT_MOD_34 } } } );
    prohibitedSplits.push_back( ProhibitedSplit{ PROHIBIT_ALL, PROHIBIT_PERP_DIR, SPLIT_MOD_34, { ProhibitedSplitFrame{ PROHIBIT_ALL, SPLIT_MOD_25, 1, PROHIBIT_PERP_DIR, SPLIT_MOD_34 } } } );
    prohibitedSplits.push_back( ProhibitedSplit{ PROHIBIT_ALL, PROHIBIT_PERP_DIR, SPLIT_MOD_34, { ProhibitedSplitFrame{ PROHIBIT_ALL, SPLIT_MOD_35, 1, PROHIBIT_PERP_DIR, SPLIT_MOD_34 } } } );
    prohibitedSplits.push_back( ProhibitedSplit{ PROHIBIT_ALL, PROHIBIT_PERP_DIR, SPLIT_MOD_34, { ProhibitedSplitFrame{ PROHIBIT_ALL, SPLIT_MOD_45, 1, PROHIBIT_PERP_DIR, SPLIT_MOD_34 } } } );
  }

  // (1/3, 2/3 comes before  comes before all splits other than 1/2, 1/4 and 3/4)
  prohibitedSplits.push_back( ProhibitedSplit{ PROHIBIT_ALL, PROHIBIT_PERP_DIR, SPLIT_MOD_13, { ProhibitedSplitFrame{ PROHIBIT_ALL, SPLIT_MOD_23, 1, PROHIBIT_PERP_DIR, SPLIT_MOD_13 } } } );

  if( spsNext.getGbsAllowEights() )
  {
    prohibitedSplits.push_back( ProhibitedSplit{ PROHIBIT_ALL, PROHIBIT_PERP_DIR, SPLIT_MOD_13, { ProhibitedSplitFrame{ PROHIBIT_ALL, SPLIT_MOD_15, 1, PROHIBIT_PERP_DIR, SPLIT_MOD_13 } } } );
    prohibitedSplits.push_back( ProhibitedSplit{ PROHIBIT_ALL, PROHIBIT_PERP_DIR, SPLIT_MOD_13, { ProhibitedSplitFrame{ PROHIBIT_ALL, SPLIT_MOD_25, 1, PROHIBIT_PERP_DIR, SPLIT_MOD_13 } } } );
    prohibitedSplits.push_back( ProhibitedSplit{ PROHIBIT_ALL, PROHIBIT_PERP_DIR, SPLIT_MOD_13, { ProhibitedSplitFrame{ PROHIBIT_ALL, SPLIT_MOD_35, 1, PROHIBIT_PERP_DIR, SPLIT_MOD_13 } } } );
    prohibitedSplits.push_back( ProhibitedSplit{ PROHIBIT_ALL, PROHIBIT_PERP_DIR, SPLIT_MOD_13, { ProhibitedSplitFrame{ PROHIBIT_ALL, SPLIT_MOD_45, 1, PROHIBIT_PERP_DIR, SPLIT_MOD_13 } } } );

    prohibitedSplits.push_back( ProhibitedSplit{ PROHIBIT_ALL, PROHIBIT_PERP_DIR, SPLIT_MOD_23, { ProhibitedSplitFrame{ PROHIBIT_ALL, SPLIT_MOD_15, 1, PROHIBIT_PERP_DIR, SPLIT_MOD_23 } } } );
    prohibitedSplits.push_back( ProhibitedSplit{ PROHIBIT_ALL, PROHIBIT_PERP_DIR, SPLIT_MOD_23, { ProhibitedSplitFrame{ PROHIBIT_ALL, SPLIT_MOD_25, 1, PROHIBIT_PERP_DIR, SPLIT_MOD_23 } } } );
    prohibitedSplits.push_back( ProhibitedSplit{ PROHIBIT_ALL, PROHIBIT_PERP_DIR, SPLIT_MOD_23, { ProhibitedSplitFrame{ PROHIBIT_ALL, SPLIT_MOD_35, 1, PROHIBIT_PERP_DIR, SPLIT_MOD_23 } } } );
    prohibitedSplits.push_back( ProhibitedSplit{ PROHIBIT_ALL, PROHIBIT_PERP_DIR, SPLIT_MOD_23, { ProhibitedSplitFrame{ PROHIBIT_ALL, SPLIT_MOD_45, 1, PROHIBIT_PERP_DIR, SPLIT_MOD_23 } } } );

    // x/5 split ordering
    prohibitedSplits.push_back( ProhibitedSplit{ PROHIBIT_ALL, PROHIBIT_PERP_DIR, SPLIT_MOD_15, { ProhibitedSplitFrame{ PROHIBIT_ALL, SPLIT_MOD_25, 1, PROHIBIT_PERP_DIR, SPLIT_MOD_15 } } } );
    prohibitedSplits.push_back( ProhibitedSplit{ PROHIBIT_ALL, PROHIBIT_PERP_DIR, SPLIT_MOD_15, { ProhibitedSplitFrame{ PROHIBIT_ALL, SPLIT_MOD_35, 1, PROHIBIT_PERP_DIR, SPLIT_MOD_15 } } } );
    prohibitedSplits.push_back( ProhibitedSplit{ PROHIBIT_ALL, PROHIBIT_PERP_DIR, SPLIT_MOD_15, { ProhibitedSplitFrame{ PROHIBIT_ALL, SPLIT_MOD_45, 1, PROHIBIT_PERP_DIR, SPLIT_MOD_15 } } } );

    prohibitedSplits.push_back( ProhibitedSplit{ PROHIBIT_ALL, PROHIBIT_PERP_DIR, SPLIT_MOD_25, { ProhibitedSplitFrame{ PROHIBIT_ALL, SPLIT_MOD_35, 1, PROHIBIT_PERP_DIR, SPLIT_MOD_25 } } } );
    prohibitedSplits.push_back( ProhibitedSplit{ PROHIBIT_ALL, PROHIBIT_PERP_DIR, SPLIT_MOD_25, { ProhibitedSplitFrame{ PROHIBIT_ALL, SPLIT_MOD_45, 1, PROHIBIT_PERP_DIR, SPLIT_MOD_25 } } } );

    prohibitedSplits.push_back( ProhibitedSplit{ PROHIBIT_ALL, PROHIBIT_PERP_DIR, SPLIT_MOD_35, { ProhibitedSplitFrame{ PROHIBIT_ALL, SPLIT_MOD_45, 1, PROHIBIT_PERP_DIR, SPLIT_MOD_35 } } } );
  }
}

GenBinSplitPartitioner::GenBinSplitPartitioner()
{

}

bool GenBinSplitPartitioner::isProhibited( const PartSplit split, const SliceType sliceType ) const
{
  const PartSplit baseSplit = getBaseSplit( getActualSplitType( split ) );
  const SplitModifier mod   = getModifier( split );
  const PartSplit splitDir  = getPseudoSplitType( split );

  for( const auto &ps : prohibitedSplits )
  {
    bool doCheck = false;

    switch( ps.sliceType )
    {
    case PROHIBIT_ALL:
      doCheck = true;
      break;
    case INTER_FRAME:
      doCheck = sliceType != I_SLICE;
      break;
    default:
      doCheck = ps.sliceType == sliceType;
      break;
    }

    if( !doCheck ) continue;

    switch( ps.split )
    {
    case PROHIBIT_ALL:
      doCheck = true;
      break;
    case PROHIBIT_SAME_DIR:
      doCheck = splitDir == CU_PARALLEL_SPLIT;
      break;
    case PROHIBIT_PERP_DIR:
      doCheck = splitDir == CU_PERPENDICULAR_SPLIT;
      break;
    default:
      doCheck = ps.split == baseSplit;
      break;
    }

    if( !doCheck ) continue;

    switch( ps.mod )
    {
    case PROHIBIT_ALL:
      break;
    default:
      doCheck = ps.mod == mod;
      break;
    }

    if( !doCheck ) continue;

    const auto &prohibitedSplitStack = ps.splitStack;

    int offset = 1;

    bool isProhibited = offset < m_partStack.size();

    while( isProhibited && offset <= prohibitedSplitStack.size() && offset < m_partStack.size() )
    {
      const auto &prohibitedFrame = prohibitedSplitStack[offset - 1];
      const auto &splitFrame      = m_partStack         [m_partStack.size() - offset];

      switch( prohibitedFrame.split )
      {
      case PROHIBIT_ALL:
        isProhibited = true;
        break;
      case PROHIBIT_SAME_DIR:
        isProhibited = splitFrame.pseudoSplit == CU_PARALLEL_SPLIT;
        break;
      case PROHIBIT_PERP_DIR:
        isProhibited = splitFrame.pseudoSplit == CU_PERPENDICULAR_SPLIT;
        break;
      default:
        isProhibited = getBaseSplit( splitFrame.split ) == prohibitedFrame.split;
        break;
      }

      if( !isProhibited ) break;

      switch( ps.mod )
      {
      case PROHIBIT_ALL:
        break;
      default:
        isProhibited = getModifier( splitFrame.split ) == prohibitedFrame.mod;
        break;
      }

      if( !isProhibited ) break;

      if( prohibitedFrame.idx != PROHIBIT_ALL ) isProhibited = prohibitedFrame.idx == splitFrame.idx;

      if( splitFrame.idx > 0 )
      {
        if( !isProhibited ) break;

        const PartSplit subBaseSplit = getBaseSplit( splitFrame.firstSubPartSplit );
        const SplitModifier subMod   = subBaseSplit == CU_DONT_SPLIT ? NUM_SPLIT_MOD  : getModifier( splitFrame.firstSubPartSplit );
        const PartSplit subSplitDir  = subBaseSplit == CU_DONT_SPLIT ? NUM_PART_SPLIT : ( subBaseSplit == getBaseSplit( splitFrame.split ) ? CU_PARALLEL_SPLIT : CU_PERPENDICULAR_SPLIT );

        switch( prohibitedFrame.firstPartSplit )
        {
        case PROHIBIT_ALL:
          break;
        case PROHIBIT_PERP_DIR:
          isProhibited = subSplitDir == CU_PERPENDICULAR_SPLIT;
          break;
        case PROHIBIT_SAME_DIR:
          isProhibited = subSplitDir == CU_PARALLEL_SPLIT;
          break;
        default:
          isProhibited = subBaseSplit == prohibitedFrame.firstPartSplit;
          break;
        }

        if( !isProhibited ) break;

        switch( prohibitedFrame.firstPartMod )
        {
        case PROHIBIT_ALL:
          break;
        default:
          isProhibited = subMod == prohibitedFrame.firstPartMod;
          break;
        }
      }

      if( !isProhibited ) break;

      offset++;
    }

    if( isProhibited ) return true;
  }

  return false;
}

Void GenBinSplitPartitioner::initCtu( const UnitArea& ctuArea, const ChannelType _chType, const Slice& slice )
{
  if( prohibitedSplits.empty() )
  {
    initProhibitedSplits( slice );
  }
#if _DEBUG
  m_currArea = ctuArea;
#endif
  currDepth   = 0;
  currTrDepth = 0;
  currBtDepth = 0;
  currMtDepth = 0;
  currGtDepth = 0;
  currQtDepth = 0;
#if !HM_QTBT_ONLY_QT_IMPLICIT
  currImplicitBtDepth = 0;
#endif
  chType      = _chType;

  m_partStack.clear();
  m_partStack.push_back( PartLevel( CTU_LEVEL, Partitioning{ ctuArea } ) );

  m_isForceSplitToLog2 = slice.getSPS()->getSpsNext().getGbsForceSplitToLog2();
  m_isNonLog2CUs       = slice.getSPS()->getSpsNext().getGbsNonLog2CUs();
}

Void GenBinSplitPartitioner::splitCurrArea( const PartSplit split, const CodingStructure& cs )
{
  CHECKD( !canSplit( split, cs ), "Trying to apply a prohibited split!" );

#if !HM_QTBT_ONLY_QT_IMPLICIT
  bool isImplicit             = isSplitImplicit   ( split, cs );
#endif
  const PartSplit pseudoSplit = getPseudoSplitType( split );
  const PartSplit actualSplit = getActualSplitType( split );
  const SplitModifier mod     = getModifier       ( split );

  bool canQtSplit = canSplit( CU_QUAD_SPLIT, cs );

  switch( actualSplit )
  {
  case CU_HORZ_SPLIT:
  case CU_VERT_SPLIT:
  case CU_HORZ_SPLIT_14:
  case CU_VERT_SPLIT_14:
  case CU_HORZ_SPLIT_34:
  case CU_VERT_SPLIT_34:
  case CU_HORZ_SPLIT_38:
  case CU_VERT_SPLIT_38:
  case CU_HORZ_SPLIT_58:
  case CU_VERT_SPLIT_58:
  case CU_HORZ_SPLIT_13:
  case CU_VERT_SPLIT_13:
  case CU_HORZ_SPLIT_23:
  case CU_VERT_SPLIT_23:
  case CU_HORZ_SPLIT_15:
  case CU_VERT_SPLIT_15:
  case CU_HORZ_SPLIT_25:
  case CU_VERT_SPLIT_25:
  case CU_HORZ_SPLIT_35:
  case CU_VERT_SPLIT_35:
  case CU_HORZ_SPLIT_45:
  case CU_VERT_SPLIT_45:
    m_partStack.push_back( PartLevel( actualSplit, PartitionerImpl::getGBSPartitions( currArea(), actualSplit ) ) );
    m_partStack.back().pseudoSplit = pseudoSplit;
    break;
  case CU_QUAD_SPLIT:
    m_partStack.push_back( PartLevel( actualSplit, PartitionerImpl::getCUSubPartitions( currArea(), cs, split ) ) );
    m_partStack.back().pseudoSplit = pseudoSplit;
    break;
  default:
    THROW( "Unknown split mode" );
    break;
  }

  currDepth++;

#if _DEBUG
  m_currArea = m_partStack.back().parts.front();
#endif

  if( split == CU_QUAD_SPLIT )
  {
    currQtDepth++;
  }
  else
  {
    currMtDepth++;
    currBtDepth++;

    if( !m_isNonLog2CUs && isNonLog2BlockSize( currArea().Y() ) )
    {
      currMtDepth--;
      currBtDepth--;
    }

#if !HM_QTBT_ONLY_QT_IMPLICIT
    if( isImplicit ) currImplicitBtDepth++;
#endif
    if( mod != SPLIT_MOD_12 )
    {
      currGtDepth++;
    }

    m_partStack.back().canQtSplit = canQtSplit;
  }
}

bool GenBinSplitPartitioner::canSplit( const PartSplit split, const CodingStructure &cs )
{
  return canSplit( split, cs, true );
}

bool GenBinSplitPartitioner::canSplit( const PartSplit split, const CodingStructure &cs, const bool checkMods )
{
  const CompArea area = currArea().Y();
  const PartSplit implicitSplit = getImplicitSplit( cs );

  if( split == CU_QUAD_SPLIT )
  {
    // don't allow QT-splitting below a BT split
    PartSplit lastSplit = m_partStack.back().split;
    if( lastSplit != CTU_LEVEL && lastSplit != CU_QUAD_SPLIT )                  return false;
    // allowing QT split even if a BT split is implied
    if( implicitSplit != CU_DONT_SPLIT )                                        return true;
    // check minimal allowed QT size
    unsigned minQtSize = cs.pcv->getMinQtSize( *cs.slice, chType );
    if( currArea().lwidth() <= minQtSize || currArea().lheight() <= minQtSize ) return false;

    return true;
  }

#if !HM_QTBT_ONLY_QT_IMPLICIT
  const unsigned maxBTD       = cs.pcv->getMaxBtDepth( *cs.slice, chType ) + currImplicitBtDepth;
#else
  const unsigned maxBTD       = cs.pcv->getMaxBtDepth( *cs.slice, chType );
#endif
  const unsigned maxBtSize    = cs.pcv->getMaxBtSize ( *cs.slice, chType );
  const unsigned minBtSize    = cs.pcv->getMinBtSize ( *cs.slice, chType );

  const PartSplit actualSplit = getBaseSplit( getActualSplitType( split ) );
  const unsigned  splitSize   = actualSplit == CU_HORZ_SPLIT ? area.height : area.width;
  const unsigned  unsplitSize = actualSplit == CU_VERT_SPLIT ? area.height : area.width;

  if( area.width > maxBtSize || area.height > maxBtSize )
  {
    if( split == CU_DONT_SPLIT && implicitSplit == CU_DONT_SPLIT )
    {
      return area.width == area.height;
    }
    else
    {
      return false;
    }
  }

  if( implicitSplit != CU_DONT_SPLIT )
  {
    if( getBaseSplit( implicitSplit ) == actualSplit )
    {
      return true;
    }
    else
    {
      return false;
    }
  }

  if( split == CU_DONT_SPLIT )
  {
    if( m_isNonLog2CUs )
    {
      return true;
    }
    else
    {
      return !isNonLog2BlockSize( area );
    }
  }

  if( m_isForceSplitToLog2 )
  {
    if( isNonLog2Size( unsplitSize ) )
    {
      return false;
    }
  }

  if( !m_isNonLog2CUs )
  {
    if( isNonLog2Size( unsplitSize ) && !isNonLog2Size( splitSize ) && currBtDepth + 1 >= maxBTD )
    {
      return false;
    }
    else if( isNonLog2Size( splitSize ) && currBtDepth >= maxBTD )
    {
      return true;
    }
  }

  if( currBtDepth >= maxBTD )
  {
    return false;
  }

  if( splitSize <= minBtSize ) return false;

  if( split != CU_DONT_SPLIT && isNonLog2Size( splitSize ) )
  {
    if( getSizeClass( splitSize ) == SIZE_32_LOG2 )
    {
      if( ( splitSize / 3 ) < minBtSize ) return false;
    }
    else
    {
      CHECK( getSizeClass( splitSize ) != SIZE_54_LOG2, "Invalid size class" );
      if( ( splitSize / 5 ) < minBtSize ) return false;
    }
  }

  if( !checkMods ) return true;

  // check if at least on modifier is allowed
  for( int modId = 0; modId < NUM_SPLIT_MOD; modId++ )
  {
    const SplitModifier mod = SplitModifier( modId );
    if( canModify( split, mod, cs ) ) return true;
  }

  return false;
}

bool GenBinSplitPartitioner::canModify( const PartSplit split, const SplitModifier mod, const CodingStructure &cs )
{
  const PartSplit splitDir    = getPseudoSplitType( split );
  const int       splitDirIdx = splitDir == CU_PERPENDICULAR_SPLIT ? 0 : 1;

  if( m_partStack.back().canModify[splitDirIdx][mod] == 0 )
  {
    const bool ret = canModifyImpl( split, mod, cs );
    m_partStack.back().canModify[splitDirIdx][mod] = ret ? 2 : 1;
    return ret;
  }
  else
  {
    return m_partStack.back().canModify[splitDirIdx][mod] == 2;
  }
}

bool GenBinSplitPartitioner::canModifyImpl( const PartSplit split, const SplitModifier mod, const CodingStructure &cs )
{
  CHECKD( !canSplit( split, cs, false ), "Cannot apply a modifier to a prohibited split" );

  if( split == CU_QUAD_SPLIT )
  {
    return mod == SPLIT_MOD_12;
  }

  const PartSplit implicitSplit = getImplicitSplit( cs );

  const CompArea area           = currArea().Y();

#if !HM_QTBT_ONLY_QT_IMPLICIT
  const unsigned maxBTD         = cs.pcv->getMaxBtDepth ( *cs.slice, chType ) + currImplicitBtDepth;
#else
  const unsigned maxBTD         = cs.pcv->getMaxBtDepth ( *cs.slice, chType );
#endif
  const unsigned maxBtSize      = cs.pcv->getMaxBtSize  ( *cs.slice, chType );
  const unsigned minBtSize      = cs.pcv->getMinBtSize  ( *cs.slice, chType );
  const unsigned maxAsymSize    = cs.pcv->getMaxAsymSize( *cs.slice, chType );

  const PartSplit baseSplit     = getBaseSplit( getActualSplitType( split ) );
  const PartSplit splitDir      = getPseudoSplitType( split );
  const unsigned splitSize      = baseSplit == CU_HORZ_SPLIT ? area.height : area.width;

  if( area.width > maxBtSize || area.height > maxBtSize )
  {
    // simulate quad splits up to the maximal BT size
    return split == CU_QUAD_SPLIT && mod == SPLIT_MOD_12;
  }

  if( implicitSplit != CU_DONT_SPLIT )
  {
    return   mod == getModifier( implicitSplit ) && splitDir == getPseudoSplitType( implicitSplit );
  }

  switch( mod )
  {
  case SPLIT_MOD_38:
  case SPLIT_MOD_58:
    if( !cs.sps->getSpsNext().getGbsAllowEights() )   return false;
    if( ( area.height > maxAsymSize || area.width > maxAsymSize ) )
                                                      return false;
    if( !( splitSize >= 8 * minBtSize ) )             return false;
    if( getSizeClass( splitSize ) != SIZE_LOG2 )      return false;
    break;
  case SPLIT_MOD_14:
  case SPLIT_MOD_34:
    if( !cs.sps->getSpsNext().getGbsAllowFourths() )  return false;
    if( ( area.height > maxAsymSize || area.width > maxAsymSize ) )
                                                      return false;
    if( !( splitSize >= 4 * minBtSize ) )             return false;
    if( getSizeClass( splitSize ) != SIZE_LOG2 )      return false;
    break;
  case SPLIT_MOD_12:
    if( !( splitSize >= 2 * minBtSize ) )             return false;
    if( cs.sps->getSpsNext().getGbsNonLog2Halving() )
    {
      if( m_isForceSplitToLog2 && getSizeClass( splitSize ) != SIZE_LOG2 )
      {
        return false;
      }
      if( !m_isNonLog2CUs && currBtDepth + 1 >= maxBTD && getSizeClass( splitSize ) != SIZE_LOG2 )
      {
        return false;
      }
      if( ( splitSize >= 8 * minBtSize ) )
      {
        if( !( getSizeClass( splitSize / 2 ) == SIZE_54_LOG2 || getSizeClass( splitSize / 2 ) == SIZE_32_LOG2 || getSizeClass( splitSize / 2 ) == SIZE_LOG2 ) )
        {
          return false;
        }
      }
      else if( ( splitSize >= 4 * minBtSize ) )
      {
        if( !( getSizeClass( splitSize / 2 ) == SIZE_32_LOG2 || getSizeClass( splitSize / 2 ) == SIZE_LOG2 ) )
        {
          return false;
        }
      }
      else
      {
        if( getSizeClass( splitSize / 2 ) != SIZE_LOG2 ) return false;
      }
    }
    else if( getSizeClass( splitSize ) != SIZE_LOG2 )    return false;
    break;
  case SPLIT_MOD_13:
  case SPLIT_MOD_23:
    if( getSizeClass( splitSize ) != SIZE_32_LOG2 )      return false;
    break;
  case SPLIT_MOD_25:
  case SPLIT_MOD_35:
    // TODO: fix this to ensure the option does what its called!
    if(/* m_isForceSplitToLog2 ||*/ ( !m_isNonLog2CUs && currBtDepth + 1 >= maxBTD ) )
                                                         return false;
  case SPLIT_MOD_15:
  case SPLIT_MOD_45:
    if( getSizeClass( splitSize ) != SIZE_54_LOG2 )      return false;
    break;
  default:
    THROW( "Invalid modifier " << mod );
    break;
  }

  if( isProhibited( applyModifier( baseSplit, mod ), cs.slice->getSliceType() ) )
                                                         return false;

  return true;
}

Void GenBinSplitPartitioner::exitCurrSplit()
{
  const SplitModifier currMod     = getModifier       ( m_partStack.back().split );
  const bool wasNonLog2Area       = isNonLog2BlockSize( ( m_partStack.back().idx >= m_partStack.back().parts.size() ? m_partStack.back().parts.back() : currArea() ).Y() );

  m_partStack.pop_back();

  CHECK( currDepth == 0, "depth is '0', although a split was performed" );
  currDepth--;
#if _DEBUG
  m_currArea = m_partStack.back().parts[m_partStack.back().idx];
#endif

  if( currMtDepth != 0 || wasNonLog2Area )
  {
    if( !wasNonLog2Area || m_isNonLog2CUs )
    {
      currMtDepth--;
      currBtDepth--;
    }

    if( currMod != SPLIT_MOD_12 )
    {
      CHECK( currGtDepth == 0, "currGtDepth is not 0!" );
      currGtDepth--;
    }
#if !HM_QTBT_ONLY_QT_IMPLICIT
    if( m_partStack.back().isImplicit )
    {
      CHECK( currImplicitBtDepth == 0, "Exiting a BT split that was not performed" );
      currImplicitBtDepth--;
    }
#endif
  }
  else
  {
    CHECK( currQtDepth == 0, "Exiting QT split, but QTD=0 already!" );
    CHECK( currMtDepth != 0, "Exiting QT split, but BTD!=0!")
    currQtDepth--;
  }
}

bool GenBinSplitPartitioner::isSplitImplicit( const PartSplit split, const CodingStructure &cs )
{
  return split == getImplicitSplit( cs );
}

PartSplit GenBinSplitPartitioner::getImplicitSplit( const CodingStructure &cs )
{
  if( m_partStack.back().checkdIfImplicit )
  {
    return m_partStack.back().implicitSplit;
  }

  PartSplit split = CU_DONT_SPLIT;

  if( currArea().lwidth() > cs.sps->getMaxTrSize() || currArea().lheight() > cs.sps->getMaxTrSize() )
  {
    split = getActualSplitType( CU_PERPENDICULAR_SPLIT );
  }

  if( split == CU_DONT_SPLIT )
  {
    const bool isBlInPic = cs.picture->cs->area.Y().contains( currArea().Y().bottomLeft() );
    const bool isTrInPic = cs.picture->cs->area.Y().contains( currArea().Y().topRight() );

#if !HM_QTBT_ONLY_QT_IMPLICIT
    if( !isBlInPic && isTrInPic )
    {
      split = CU_HORZ_SPLIT;
    }
    else if( isBlInPic && !isTrInPic )
    {
       split = CU_VERT_SPLIT;
    }
#endif
    if( !isBlInPic || !isTrInPic )
    {
#if !HM_QTBT_ONLY_QT_IMPLICIT
      const unsigned maxBtSize = cs.pcv->getMaxBtSize( *cs.slice, chType );

      if( split != CU_DONT_SPLIT && currArea().lwidth() <= maxBtSize && currArea().lheight() <= maxBtSize )
      {
        // set those values to be able to evaluate canSplit and canModify with endless recursion
        m_partStack.back().isImplicit       = false;
        m_partStack.back().implicitSplit    = CU_DONT_SPLIT;
        m_partStack.back().checkdIfImplicit = true;

        if( !canSplit( getBaseSplit( split ), cs ) || !canModify( getBaseSplit( split ), getModifier( split ), cs ) )
        {
          if( getModifier( split ) != SPLIT_MOD_12 && canSplit( getBaseSplit( split ), cs ) && canModify( getBaseSplit( split ), SPLIT_MOD_12, cs ) )
          {
            split = getBaseSplit( split );
          }
          else
          {
            split = CU_QUAD_SPLIT;
          }
        }
      }
      else
#endif
      {
        split = CU_QUAD_SPLIT;
      }
    }
  }

  m_partStack.back().checkdIfImplicit = true;
  m_partStack.back().isImplicit       = split != CU_DONT_SPLIT;
  m_partStack.back().implicitSplit    = split;

  return split;
}

Bool GenBinSplitPartitioner::nextPart( const CodingStructure &cs, bool autoPop /*= false*/ )
{
  const Position &prevPos = currArea().blocks[chType].pos();

  const CodingUnit *prevCU = cs.getCU( prevPos, chType );

  const bool wasNonLog2 = isNonLog2BlockSize( currArea().Y() );

  unsigned currIdx = ++m_partStack.back().idx;

  m_partStack.back().checkdIfImplicit  = false;
  m_partStack.back().isImplicit        = false;
  m_partStack.back().firstSubPartSplit = prevCU ? CU::getSplitAtDepth( *prevCU, currDepth ) : CU_DONT_SPLIT;

  memset( m_partStack.back().canModify, 0, sizeof( m_partStack.back().canModify ) );

  if( currIdx < m_partStack.back().parts.size() )
  {
#if _DEBUG
    m_currArea = m_partStack.back().parts[currIdx];
#endif

    const bool isNonLog2 = isNonLog2BlockSize( currArea().Y() );

    if( !m_isNonLog2CUs )
    {
      if( wasNonLog2 && !isNonLog2 )
      {
        currMtDepth++;
        currBtDepth++;
      }
      else if( !wasNonLog2 && isNonLog2 )
      {
        currMtDepth--;
        currBtDepth--;
      }
    }

    return true;
  }
  else
  {
    if( autoPop ) exitCurrSplit();
    return false;
  }
}

bool GenBinSplitPartitioner::hasNextPart()
{
  return ( ( m_partStack.back().idx + 1 ) < m_partStack.back().parts.size() );
}

void GenBinSplitPartitioner::copyState( const Partitioner& other )
{
  Partitioner::copyState( other );

  const GenBinSplitPartitioner* gbsp = dynamic_cast<const GenBinSplitPartitioner*>( &other );

  m_isForceSplitToLog2 = gbsp->m_isForceSplitToLog2;
  m_isNonLog2CUs       = gbsp->m_isNonLog2CUs;
}

PartSplit GenBinSplitPartitioner::getPseudoSplitType( const PartSplit split ) const
{
  if( split == CU_QUAD_SPLIT )
  {
    return CU_QUAD_SPLIT;
  }

  if( isPseudoSplit( split ) )
  {
    return split;
  }

  if( split == CU_DONT_SPLIT || split == CTU_LEVEL )
  {
    return split;
  }

  if( getBaseSplit( m_partStack.back().split ) == CU_HORZ_SPLIT )
  {
    switch( split )
    {
    case CU_HORZ_SPLIT:
    case CU_HORZ_SPLIT_14:
    case CU_HORZ_SPLIT_34:
    case CU_HORZ_SPLIT_38:
    case CU_HORZ_SPLIT_58:
    case CU_HORZ_SPLIT_13:
    case CU_HORZ_SPLIT_23:
    case CU_HORZ_SPLIT_15:
    case CU_HORZ_SPLIT_25:
    case CU_HORZ_SPLIT_35:
    case CU_HORZ_SPLIT_45:
      return CU_PARALLEL_SPLIT;
    default:
      return CU_PERPENDICULAR_SPLIT;
    }
  }
  else
  {
    switch( split )
    {
    case CU_VERT_SPLIT:
    case CU_VERT_SPLIT_14:
    case CU_VERT_SPLIT_34:
    case CU_VERT_SPLIT_38:
    case CU_VERT_SPLIT_58:
    case CU_VERT_SPLIT_13:
    case CU_VERT_SPLIT_23:
    case CU_VERT_SPLIT_15:
    case CU_VERT_SPLIT_25:
    case CU_VERT_SPLIT_35:
    case CU_VERT_SPLIT_45:
      return CU_PARALLEL_SPLIT;
    default:
      return CU_PERPENDICULAR_SPLIT;
    }
  }

  THROW( "Reached an unreachable statement" );
}

PartSplit GenBinSplitPartitioner::getActualSplitType( const PartSplit split ) const
{
  if( split == CU_QUAD_SPLIT )
  {
    return CU_QUAD_SPLIT;
  }

  if( !isPseudoSplit( split ) )
  {
    return split;
  }

  if( split == CU_DONT_SPLIT )
  {
    return split;
  }

  if( split == CU_PARALLEL_SPLIT )
  {
    if( getBaseSplit( m_partStack.back().split ) == CU_HORZ_SPLIT )
    {
      return CU_HORZ_SPLIT;
    }
    else
    {
      return CU_VERT_SPLIT;
    }
  }
  else
  {
    if( getBaseSplit( m_partStack.back().split ) == CU_HORZ_SPLIT )
    {
      return CU_VERT_SPLIT;
    }
    else
    {
      return CU_HORZ_SPLIT;
    }
  }

  THROW( "Reached an unreachable statement" );
}

int GenBinSplitPartitioner::getSplitSize( const PartSplit split, const Size& area ) const
{
  const PartSplit splitDir = getBaseSplit( getActualSplitType( split ) );

  if( splitDir == CU_HORZ_SPLIT )
  {
    return area.height;
  }
  else
  {
    return area.width;
  }
}

int GenBinSplitPartitioner::getUnsplitSize( const PartSplit split, const Size& area ) const
{
  const PartSplit splitDir = getBaseSplit( getActualSplitType( split ) );

  if( splitDir == CU_HORZ_SPLIT )
  {
    return area.width;
  }
  else
  {
    return area.height;
  }
}


Void TU1dPartitioner::splitCurrArea( const PartSplit split, const CodingStructure& cs )
{
  switch( split )
  {
  case TU_1D_HORZ_SPLIT:
  case TU_1D_VERT_SPLIT:
  case TU_1D_VERT_SPLIT_REVERSE_ORDER:
  case TU_1D_HORZ_SPLIT_REVERSE_ORDER:
  {
    const UnitArea &area = currArea();
    m_partStack.push_back( PartLevel() );
    m_partStack.back().split = split;
    PartitionerImpl::getTU1DSubPartitions( m_partStack.back().parts, area, cs, split );
    break;
  }
  default:
    THROW( "Unknown 1-D split mode" );
    break;
  }

  currDepth  ++;
  currTrDepth++; // we need this to identify the level. since the 1d partitions are forbidden if the RQT is on, there area no compatibility issues

#if _DEBUG
  m_currArea = m_partStack.back().parts.front();
#endif

}

Void TU1dPartitioner::exitCurrSplit()
{
  PartSplit currSplit = m_partStack.back().split;

  m_partStack.pop_back();

  CHECK( currDepth == 0, "depth is '0', although a split was performed" );

  currDepth  --;
  currTrDepth--;

#if _DEBUG
  m_currArea = m_partStack.back().parts[m_partStack.back().idx];
#endif

  CHECK( !( currSplit == TU_1D_HORZ_SPLIT || currSplit == TU_1D_VERT_SPLIT || currSplit == TU_1D_HORZ_SPLIT_REVERSE_ORDER || currSplit == TU_1D_VERT_SPLIT_REVERSE_ORDER ), "Unknown 1D partition type!" );
}

Bool TU1dPartitioner::nextPart(const CodingStructure &cs, bool autoPop /*= false*/)
{
  unsigned currIdx = ++m_partStack.back().idx;

  m_partStack.back().checkdIfImplicit = false;
  m_partStack.back().isImplicit       = false;

  if( currIdx < m_partStack.back().parts.size() )
  {
#if _DEBUG
    m_currArea = m_partStack.back().parts[m_partStack.back().idx];
#endif
    return true;
  }
  else
  {
    if( autoPop ) exitCurrSplit();
    return false;
  }
}

bool TU1dPartitioner::hasNextPart()
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
  if( slice.getSPS()->getSpsNext().getUseGenBinSplit() )
  {
    return new GenBinSplitPartitioner;
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
  else if( splitType == CU_TRIH_SPLIT )
  {
    Partitioning sub;

    sub.resize( 3, cuArea );

    for( int i = 0; i < 3; i++ )
    {
      for( auto &blk : sub[i].blocks )
      {
        blk.height >>= 1;
        if( ( i + 1 ) & 1 ) blk.height >>= 1;
        if( i == 1 )        blk.y       +=     blk.height / 2;
        if( i == 2 )        blk.y       += 3 * blk.height;
      }

      CHECK( sub[i].lumaSize().height < MIN_TU_SIZE, "the cs split causes the block to be smaller than the minimal TU size" );
    }

    return sub;
  }
  else if( splitType == CU_TRIV_SPLIT )
  {
    Partitioning sub;

    sub.resize( 3, cuArea );

    for( int i = 0; i < 3; i++ )
    {
      for( auto &blk : sub[i].blocks )
      {
        blk.width >>= 1;

        if( ( i + 1 ) & 1 ) blk.width >>= 1;
        if( i == 1 )        blk.x      +=     blk.width / 2;
        if( i == 2 )        blk.x      += 3 * blk.width;
      }

      CHECK( sub[i].lumaSize().width < MIN_TU_SIZE, "the cs split causes the block to be smaller than the minimal TU size" );
    }

    return sub;
  }
  else if( splitType == CU_HORZ_SPLIT_14 )
  {
    Partitioning sub;

    sub.resize( 2, cuArea );

    for( UInt i = 0; i < 2; i++ )
    {
      for( auto &blk : sub[i].blocks )
      {
        blk.height >>= 2;

        if( i == 1 )
        {
          blk.y      += blk.height;
          blk.height *= 3;
        }
      }

      CHECK( sub[i].lumaSize().height < MIN_TU_SIZE, "the cs split causes the block to be smaller than the minimal TU size" );
    }

    return sub;
  }
  else if( splitType == CU_VERT_SPLIT_14 )
  {
    Partitioning sub;

    sub.resize( 2, cuArea );

    for( UInt i = 0; i < 2; i++ )
    {
      for( auto &blk : sub[i].blocks )
      {
        blk.width >>= 2;

        if( i == 1 )
        {
          blk.x     += blk.width;
          blk.width *= 3;
        }
      }

      CHECK( sub[i].lumaSize().width < MIN_TU_SIZE, "the split causes the block to be smaller than the minimal TU size" );
    }

    return sub;
  }
  else if( splitType == CU_HORZ_SPLIT_34 )
  {
    Partitioning sub;

    sub.resize( 2, cuArea );

    for( UInt i = 0; i < 2; i++ )
    {
      for( auto &blk : sub[i].blocks )
      {
        blk.height >>= 2;

        if( i == 0 ) blk.height *= 3;
        if( i == 1 ) blk.y      += 3 * blk.height;
      }

      CHECK( sub[i].lumaSize().height < MIN_TU_SIZE, "the cs split causes the block to be smaller than the minimal TU size" );
    }

    return sub;
  }
  else if( splitType == CU_VERT_SPLIT_34 )
  {
    Partitioning sub;

    sub.resize( 2, cuArea );

    for( UInt i = 0; i < 2; i++ )
    {
      for( auto &blk : sub[i].blocks )
      {
        blk.width >>= 2;

        if( i == 0 ) blk.width *= 3;
        if( i == 1 ) blk.x     += 3 * blk.width;
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

Partitioning PartitionerImpl::getGBSPartitions( const UnitArea &cuArea, const PartSplit _splitType )
{
  const PartSplit splitType = getBaseSplit( _splitType );
  const SplitModifier mod   = getModifier ( _splitType );

  unsigned blkDiv         = 2;
  unsigned firstPartMul   = 1;
  unsigned secondPartMul  = 1;

  switch( mod )
  {
  case SPLIT_MOD_12: blkDiv = 2; firstPartMul = 1; secondPartMul = 1; break;
  case SPLIT_MOD_14: blkDiv = 4; firstPartMul = 1; secondPartMul = 3; break;
  case SPLIT_MOD_34: blkDiv = 4; firstPartMul = 3; secondPartMul = 1; break;
  case SPLIT_MOD_38: blkDiv = 8; firstPartMul = 3; secondPartMul = 5; break;
  case SPLIT_MOD_58: blkDiv = 8; firstPartMul = 5; secondPartMul = 3; break;
  case SPLIT_MOD_13: blkDiv = 3; firstPartMul = 1; secondPartMul = 2; break;
  case SPLIT_MOD_23: blkDiv = 3; firstPartMul = 2; secondPartMul = 1; break;
  case SPLIT_MOD_15: blkDiv = 5; firstPartMul = 1; secondPartMul = 4; break;
  case SPLIT_MOD_25: blkDiv = 5; firstPartMul = 2; secondPartMul = 3; break;
  case SPLIT_MOD_35: blkDiv = 5; firstPartMul = 3; secondPartMul = 2; break;
  case SPLIT_MOD_45: blkDiv = 5; firstPartMul = 4; secondPartMul = 1; break;
  default:
    THROW( "Invalid modifier " << mod );
    break;
  }

  CHECK( blkDiv != firstPartMul + secondPartMul, "Invalid modifier values" );

  Partitioning sub( 2, cuArea );

  if( splitType == CU_HORZ_SPLIT )
  {
    for( UInt i = 0; i < 2; i++ )
    {
      for( auto &blk : sub[i].blocks )
      {
        blk.height /= blkDiv;

        if( i == 0 ) {                                     blk.height *= firstPartMul;  }
        if( i == 1 ) { blk.y += firstPartMul * blk.height; blk.height *= secondPartMul; }
      }

      CHECK( sub[i].lumaSize().height < MIN_TU_SIZE, "the cs split causes the block to be smaller than the minimal TU size" );
    }
  }
  else if( splitType == CU_VERT_SPLIT )
  {
    for( UInt i = 0; i < 2; i++ )
    {
      for( auto &blk : sub[i].blocks )
      {
        blk.width /= blkDiv;

        if( i == 0 ) {                                    blk.width *= firstPartMul;  }
        if( i == 1 ) { blk.x += firstPartMul * blk.width; blk.width *= secondPartMul; }
      }

      CHECK( sub[i].lumaSize().width < MIN_TU_SIZE, "the split causes the block to be smaller than the minimal TU size" );
    }
  }
  else
  {
    THROW( "Unknown CU sub-partitioning" );
    return Partitioning();
  }

  return sub;
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

void PartitionerImpl::getTU1DSubPartitions( Partitioning &sub, const UnitArea &tuArea, const CodingStructure &cs, const PartSplit splitType )
{
  bool canSplit = true;

  if( splitType == TU_1D_HORZ_SPLIT || splitType == TU_1D_HORZ_SPLIT_REVERSE_ORDER )
  {
    canSplit = tuArea.lumaSize().height > 1;
  }
  else if( splitType == TU_1D_VERT_SPLIT || splitType == TU_1D_VERT_SPLIT_REVERSE_ORDER )
  {
    canSplit = tuArea.lumaSize().width > 1;
  }
  else
  {
    THROW( "getTU1DSubPartitions can only use a split type equal to TU_1D_HORZ_SPLIT, TU_1D_VERT_SPLIT, TU_1D_HORZ_SPLIT_REVERSE_ORDER or TU_1D_VERT_SPLIT_REVERSE_ORDER" );
  }


  if( !canSplit )
  {
    sub.resize( 0 );
  }

  if( splitType == TU_1D_HORZ_SPLIT )
  {
    sub.resize( tuArea.lumaSize().height );

    for( UInt i = 0; i < tuArea.lumaSize().height; i++ )
    {
      sub[i] = tuArea;
      CompArea& blkY = sub[i].blocks[COMPONENT_Y];

      blkY.height = 1;
      blkY.y += i;

      //we only partition luma, so there is going to be only one chroma tu at the beginning
      if( i < tuArea.lumaSize().height - 1 )
      {
        CompArea& blkCb = sub[i].blocks[COMPONENT_Cb];
        CompArea& blkCr = sub[i].blocks[COMPONENT_Cr];
        blkCb = CompArea();
        blkCr = CompArea();
      }

      CHECK( sub[i].lumaSize().height < 1, "the cs split causes the block to be smaller than the minimal TU size" );
    }
  }
  else if( splitType == TU_1D_HORZ_SPLIT_REVERSE_ORDER )
  {
    sub.resize( tuArea.lumaSize().height );
    int offset = tuArea.lumaSize().height - 1;

    for( UInt i = 0; i < tuArea.lumaSize().height; i++ )
    {
      sub[i] = tuArea;
      CompArea& blkY = sub[i].blocks[COMPONENT_Y];

      blkY.height = 1;
      blkY.y += offset - i;

      //we only partition luma, so there is going to be only one chroma tu at the beginning
      if( i < tuArea.lumaSize().height - 1 )
      {
        CompArea& blkCb = sub[i].blocks[COMPONENT_Cb];
        CompArea& blkCr = sub[i].blocks[COMPONENT_Cr];
        blkCb = CompArea();
        blkCr = CompArea();
      }

      CHECK( sub[i].lumaSize().height < 1, "the cs split causes the block to be smaller than the minimal TU size" );
    }
  }
  else if( splitType == TU_1D_VERT_SPLIT )
  {
    sub.resize( tuArea.lumaSize().width );

    for( UInt i = 0; i < tuArea.lumaSize().width; i++ )
    {
      sub[i] = tuArea;
      CompArea& blkY = sub[i].blocks[COMPONENT_Y];

      blkY.width = 1;
      blkY.x += i;

      //we only partition luma, so there is going to be only one chroma tu at the beginning
      if( i < tuArea.lumaSize().width - 1 )
      {
        CompArea& blkCb = sub[i].blocks[COMPONENT_Cb];
        CompArea& blkCr = sub[i].blocks[COMPONENT_Cr];
        blkCb = CompArea();
        blkCr = CompArea();
      }

      CHECK( sub[i].lumaSize().width < 1, "the split causes the block to be smaller than the minimal TU size" );
    }
  }
  else if( splitType == TU_1D_VERT_SPLIT_REVERSE_ORDER )
  {
    sub.resize( tuArea.lumaSize().width );
    int offset = tuArea.lumaSize().width - 1;

    for( UInt i = 0; i < tuArea.lumaSize().width; i++ )
    {
      sub[i] = tuArea;
      CompArea& blkY = sub[i].blocks[COMPONENT_Y];

      blkY.width = 1;
      blkY.x += offset - i;

      //we only partition luma, so there is going to be only one chroma tu at the beginning
      if( i < tuArea.lumaSize().width - 1 )
      {
        CompArea& blkCb = sub[i].blocks[COMPONENT_Cb];
        CompArea& blkCr = sub[i].blocks[COMPONENT_Cr];
        blkCb = CompArea();
        blkCr = CompArea();
      }

      CHECK( sub[i].lumaSize().width < 1, "the split causes the block to be smaller than the minimal TU size" );
    }
  }
  else
  {
    THROW( "Unknown TU sub-partitioning" );
  }
}
