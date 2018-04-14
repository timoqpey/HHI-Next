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

#ifndef __UNITPARTITIONER__
#define __UNITPARTITIONER__

#include "Unit.h"

#include "CommonDef.h"

#include <vector>
typedef std::vector  <UnitArea>    Partitioning;

//////////////////////////////////////////////////////////////////////////
// PartManager class - manages the partitioning tree
//
// contains the currently processed partitioning area (currArea)
// as well as the all partitioning decisions that led to this area
// being processed (in m_partStack).
//////////////////////////////////////////////////////////////////////////

enum PartSplit
{
  CTU_LEVEL        = 0,
  CU_QUAD_SPLIT,

  CU_HORZ_SPLIT,
  CU_VERT_SPLIT,
  CU_TRIH_SPLIT,
  CU_TRIV_SPLIT,
  CU_HORZ_SPLIT_14,
  CU_HORZ_SPLIT_34,
  CU_VERT_SPLIT_14,
  CU_VERT_SPLIT_34,
  CU_HORZ_SPLIT_38,
  CU_HORZ_SPLIT_58,
  CU_VERT_SPLIT_38,
  CU_VERT_SPLIT_58,

  CU_HORZ_SPLIT_13,
  CU_HORZ_SPLIT_23,
  CU_VERT_SPLIT_13,
  CU_VERT_SPLIT_23,

  CU_HORZ_SPLIT_15,
  CU_HORZ_SPLIT_25,
  CU_HORZ_SPLIT_35,
  CU_HORZ_SPLIT_45,
  CU_VERT_SPLIT_15,
  CU_VERT_SPLIT_25,
  CU_VERT_SPLIT_35,
  CU_VERT_SPLIT_45,

  CU_L_SPLIT,
  CU_R_SPLIT,

  TU_QUAD_SPLIT,
  TU_LEVEL0,
  TU_1D_HORZ_SPLIT,
  TU_1D_VERT_SPLIT,
  TU_1D_HORZ_SPLIT_REVERSE_ORDER,
  TU_1D_VERT_SPLIT_REVERSE_ORDER,
  NUM_PART_SPLIT,
  CU_MT_SPLIT             = 1000, ///< dummy element to indicate the MT (multi-type-tree) split
  CU_BT_SPLIT             = 1001, ///< dummy element to indicate the BT split
  CU_PERPENDICULAR_SPLIT  = 1003,
  CU_PARALLEL_SPLIT       = 1002,
  CU_DONT_SPLIT           = 2000  ///< dummy element to indicate no splitting
};

enum SplitModifier
{
  SPLIT_MOD_12 = 0,
  SPLIT_MOD_14, // for log2 block sizes
  SPLIT_MOD_34,
  SPLIT_MOD_38, // for log2 block sizes
  SPLIT_MOD_58,
  SPLIT_MOD_13, // for splitting of 3/8 or 3/4 split log2 blocks (e.g. 12)
  SPLIT_MOD_23,
  SPLIT_MOD_15, // for splitting of 5/8 split log2 blocks (e.g. 20)
  SPLIT_MOD_25,
  SPLIT_MOD_35,
  SPLIT_MOD_45,
  NUM_SPLIT_MOD
};

enum SizeClass
{
  SIZE_LOG2         = 0,
  SIZE_32_LOG2,
  SIZE_54_LOG2,
  NUM_SIZE_CLASSES
};

PartSplit applyModifier( const PartSplit split, const SplitModifier mod );

PartSplit getBaseSplit( const PartSplit split );

SplitModifier getModifier( const PartSplit split );

PartSplit getSplitSide( const SplitModifier mod );

SizeClass getSizeClass( const unsigned size );

double getSplitRatio( const SplitModifier mod );

bool isPseudoSplit( const PartSplit split );



struct PartLevel
{
  PartSplit    split;
  Partitioning parts;
  unsigned     idx;
  bool         checkdIfImplicit;
  bool         isImplicit;
  PartSplit    implicitSplit;
  PartSplit    firstSubPartSplit;
  bool         canQtSplit;
  PartSplit    pseudoSplit;
  char         canModify[2][NUM_SPLIT_MOD];

  PartLevel();
  PartLevel( const PartSplit _split, const Partitioning&  _parts );
  PartLevel( const PartSplit _split,       Partitioning&& _parts );
};

// set depending on max QT / BT possibilities
typedef static_vector<PartLevel, 2 * MAX_CU_DEPTH + 1> PartitioningStack;

class Partitioner
{
protected:
  PartitioningStack m_partStack;
#if _DEBUG
  UnitArea          m_currArea;
#endif

public:
  unsigned currDepth;
  unsigned currQtDepth;
  unsigned currTrDepth;
  unsigned currBtDepth;
  unsigned currMtDepth;
  unsigned currGtDepth;

  unsigned currImplicitBtDepth;
  ChannelType chType;

  virtual ~Partitioner                    () { }

  const PartLevel& currPartLevel          () const { return m_partStack.back(); }
  const UnitArea&  currArea               () const { return currPartLevel().parts[currPartIdx()]; }
  const unsigned   currPartIdx            () const { return currPartLevel().idx; }
  const PartitioningStack& getPartStack   () const { return m_partStack; }

  SplitSeries getSplitSeries              () const;

  virtual void initCtu                    ( const UnitArea& ctuArea, const ChannelType _chType, const Slice& slice )    = 0;
  virtual void splitCurrArea              ( const PartSplit split, const CodingStructure &cs )                          = 0;
  virtual void exitCurrSplit              ()                                                                            = 0;
  virtual bool nextPart                   ( const CodingStructure &cs, bool autoPop = false )                           = 0;
  virtual bool hasNextPart                ()                                                                            = 0;
                                                                                                                        
  virtual void setCUData                  ( CodingUnit& cu );                                                           
                                                                                                                        
  virtual void copyState                  ( const Partitioner& other );
                                                                                                                        
public:                                                                                                                 
  virtual bool canSplit                   ( const PartSplit split,                          const CodingStructure &cs ) = 0;
  virtual bool isSplitImplicit            ( const PartSplit split,                          const CodingStructure &cs ) = 0;
  virtual PartSplit getImplicitSplit      (                                                 const CodingStructure &cs ) = 0;
};

class AdaptiveDepthPartitioner : public Partitioner
{
public:
  void setMaxMinDepth( unsigned& minDepth, unsigned& maxDepth, const CodingStructure& cs ) const;
};

class HEVCPartitioner : public AdaptiveDepthPartitioner
{
public:
  void initCtu                    ( const UnitArea& ctuArea, const ChannelType _chTyp, const Slice& slice );
  void splitCurrArea              ( const PartSplit split, const CodingStructure &cs );
  void exitCurrSplit              ();
  bool nextPart                   ( const CodingStructure &cs, bool autoPop = false );
  bool hasNextPart                ();

  bool canSplit                   ( const PartSplit split,                          const CodingStructure &cs );
  bool isSplitImplicit            ( const PartSplit split,                          const CodingStructure &cs );
  PartSplit getImplicitSplit      (                                                 const CodingStructure &cs );
};

class QTBTPartitioner : public AdaptiveDepthPartitioner
{
public:
  void initCtu                    ( const UnitArea& ctuArea, const ChannelType _chType, const Slice& slice );
  void splitCurrArea              ( const PartSplit split, const CodingStructure &cs );
  void exitCurrSplit              ();
  bool nextPart                   ( const CodingStructure &cs, bool autoPop = false );
  bool hasNextPart                ();

  bool canSplit                   ( const PartSplit split,                          const CodingStructure &cs );
  bool isSplitImplicit            ( const PartSplit split,                          const CodingStructure &cs );
  PartSplit getImplicitSplit      (                                                 const CodingStructure &cs );
};

class GenBinSplitPartitioner : public AdaptiveDepthPartitioner
{
public:

  GenBinSplitPartitioner();

  void  initCtu                   ( const UnitArea& ctuArea, const ChannelType _chType, const Slice& slice );
  void  splitCurrArea             ( const PartSplit split, const CodingStructure &cs );
  void  exitCurrSplit             ();
  bool  nextPart                  ( const CodingStructure &cs, bool autoPop = false );
  bool  hasNextPart               ();

  void  copyState                 ( const Partitioner& other );

  bool  canSplit                  ( const PartSplit split,                          const CodingStructure &cs );
  bool  canSplit                  ( const PartSplit split,                          const CodingStructure &cs, const bool checkMods );
  bool canModify                  ( const PartSplit split, const SplitModifier mod, const CodingStructure &cs );
  bool  isSplitImplicit           ( const PartSplit split,                          const CodingStructure &cs );
  PartSplit getImplicitSplit      (                                                 const CodingStructure &cs );

  PartSplit getPseudoSplitType    ( const PartSplit split ) const;
  PartSplit getActualSplitType    ( const PartSplit split ) const;

  int  getSplitSize               ( const PartSplit split )                   const { return getSplitSize( split, currArea().Y() ); }
  int  getSplitSize               ( const PartSplit split, const Size& size ) const;

  int  getUnsplitSize             ( const PartSplit split )                   const { return getUnsplitSize( split, currArea().Y() ); }
  int  getUnsplitSize             ( const PartSplit split, const Size& size ) const;

private:

  bool isProhibited               ( const PartSplit split, const SliceType sliceType ) const;
  bool canModifyImpl              ( const PartSplit split, const SplitModifier mod, const CodingStructure &cs );

  bool m_isForceSplitToLog2;
  bool m_isNonLog2CUs;
};

class TU1dPartitioner : public Partitioner
{
public:
  TU1dPartitioner( Partitioner& _initialState )
  {
    //we copy the input partitioner data
    m_partStack.push_back( PartLevel( TU_LEVEL0, { _initialState.currArea() } ) );
    currDepth            = _initialState.currDepth;
    currQtDepth          = _initialState.currQtDepth;
    currTrDepth          = _initialState.currTrDepth;
    currBtDepth          = _initialState.currBtDepth;
    currMtDepth          = _initialState.currMtDepth;
    currGtDepth          = _initialState.currGtDepth;
    currImplicitBtDepth  = _initialState.currImplicitBtDepth;
    chType               = _initialState.chType;
#if _DEBUG
    m_currArea           = _initialState.currArea();
#endif
  }

  void initCtu              ( const UnitArea& ctuArea, const ChannelType chType, const Slice& slice ) {}; // not needed
  void splitCurrArea        ( const PartSplit split, const CodingStructure &cs );
  void exitCurrSplit        ();
  bool nextPart             ( const CodingStructure &cs, bool autoPop = false );
  bool hasNextPart          ();
  bool canSplit             ( const PartSplit split, const CodingStructure &cs ) { return true; }; //not needed
  bool isSplitImplicit      ( const PartSplit split, const CodingStructure &cs ) { return false; }; //not needed
  PartSplit getImplicitSplit( const CodingStructure &cs ) { return CU_DONT_SPLIT; }; //not needed

};



namespace PartitionerFactory
{
  Partitioner* get( const Slice& slice );
};

//////////////////////////////////////////////////////////////////////////
// Partitioner namespace - contains methods calculating the actual splits
//////////////////////////////////////////////////////////////////////////

namespace PartitionerImpl
{
  Partitioning getPUPartitioning ( const CodingUnit &cu );
  Partitioning getCUSubPartitions( const UnitArea   &cuArea, const CodingStructure &cs, const PartSplit splitType = CU_QUAD_SPLIT );
  Partitioning getGBSPartitions  ( const UnitArea   &area, const PartSplit splitType );
  Partitioning getTUSubPartitions( const UnitArea   &tuArea, const CodingStructure &cs );
  void getTU1DSubPartitions      ( Partitioning &sub, const UnitArea   &tuArea, const CodingStructure &cs, const PartSplit splitType );
};

#endif
