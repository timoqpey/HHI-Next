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

/** \file     UnitTool.h
 *  \brief    defines operations for basic units
 */

#ifndef __UNITTOOLS__
#define __UNITTOOLS__

#include "Unit.h"
#include "UnitPartitioner.h"
#include "ContextModelling.h"
#include "InterPrediction.h"

// CS tools
namespace CS
{
  UInt64 getEstBits                   ( const CodingStructure &cs );
  UnitArea getArea                    ( const CodingStructure &cs, const UnitArea &area, const ChannelType chType );
  bool   isDualITree                  ( const CodingStructure &cs );
}



// CU tools
namespace CU
{
  bool isIntra                        (const CodingUnit &cu);
  bool isInter                        (const CodingUnit &cu);
  bool isRDPCMEnabled                 (const CodingUnit &cu);
  bool isLosslessCoded                (const CodingUnit &cu);
  UInt getIntraSizeIdx                (const CodingUnit &cu);

  bool isSameCtu                      (const CodingUnit &cu, const CodingUnit &cu2);
  bool isSameSlice                    (const CodingUnit &cu, const CodingUnit &cu2);
  bool isSameTile                     (const CodingUnit &cu, const CodingUnit &cu2);
  bool isSameSliceAndTile             (const CodingUnit &cu, const CodingUnit &cu2);
  bool isLastSubCUOfCtu               (const CodingUnit &cu);
  UInt getCtuAddr                     (const CodingUnit &cu);

  int  predictQP                      (const CodingUnit& cu, const int prevQP );
  bool isQGStart                      (const CodingUnit& cu); // check if start of a Quantization Group

  UInt getNumPUs                      (const CodingUnit& cu);
  void addPUs                         (      CodingUnit& cu);

  PartSplit getSplitAtDepth           (const CodingUnit& cu, const unsigned depth);

  bool hasNonTsCodedBlock             (const CodingUnit& cu);
  UInt getNumNonZeroCoeffNonTs        (const CodingUnit& cu);

  PUTraverser traversePUs             (      CodingUnit& cu);
  TUTraverser traverseTUs             (      CodingUnit& cu);
  cPUTraverser traversePUs            (const CodingUnit& cu);
  cTUTraverser traverseTUs            (const CodingUnit& cu);



}
// PU tools
namespace PU
{
  int  getIntraMPMs                   (const PredictionUnit &pu, unsigned *mpm, const ChannelType &channelType = CHANNEL_TYPE_LUMA);
  void getIntraChromaCandModes        (const PredictionUnit &pu, unsigned modeList[NUM_CHROMA_MODE]);
  UInt getFinalIntraMode              (const PredictionUnit &pu, const ChannelType &chType);

  void getInterMergeCandidates        (const PredictionUnit &pu, MergeCtx& mrgCtx, const int& mrgCandIdx = -1 );
  bool isDiffMER                      (const PredictionUnit &pu, const PredictionUnit &pu2);
  void fillMvpCand                    (      PredictionUnit &pu, const RefPicList &eRefPicList, const int &refIdx, AMVPInfo &amvpInfo);
  bool addMVPCandUnscaled             (const PredictionUnit &pu, const RefPicList &eRefPicList, const int &iRefIdx, const Position &pos, const MvpDir &eDir, AMVPInfo &amvpInfo);
  bool addMVPCandWithScaling          (const PredictionUnit &pu, const RefPicList &eRefPicList, const int &iRefIdx, const Position &pos, const MvpDir &eDir, AMVPInfo &amvpInfo);
  bool getColocatedMVP                (const PredictionUnit &pu, const RefPicList &eRefPicList, const Position &pos, Mv& rcMv, const int &refIdx );
  bool isBipredRestriction            (const PredictionUnit &pu);
  void spanMotionInfo                 (      PredictionUnit &pu, const MergeCtx &mrgCtx = MergeCtx() );
  bool isBiPredFromDifferentDir       (const PredictionUnit &pu);
  void restrictBiPredMergeCands       (const PredictionUnit &pu, MergeCtx& mrgCtx);
  bool isChromaIntraModeCrossCheckMode(const PredictionUnit &pu);
}

// TU tools
namespace TU
{
  UInt getNumNonZeroCoeffsNonTS       (const TransformUnit &tu, const bool bLuma = true, const bool bChroma = true);
  bool useDST                         (const TransformUnit &tu, const ComponentID &compID);
  bool isNonTransformedResidualRotated(const TransformUnit &tu, const ComponentID &compID);
  bool getCbf                         (const TransformUnit &tu, const ComponentID &compID);
  void setCbf                         (      TransformUnit &tu, const ComponentID &compID, const bool &cbf);
  bool hasTransformSkipFlag           (const CodingStructure& cs, const CompArea& area);
  UInt getGolombRiceStatisticsIndex   (const TransformUnit &tu, const ComponentID &compID);
  UInt getCoefScanIdx                 (const TransformUnit &tu, const ComponentID &compID);
  bool hasCrossCompPredInfo           (const TransformUnit &tu, const ComponentID &compID);

  bool needsSqrt2Scale                ( const Size& size );
  int  getBlockSizeTrafoScaleForQuant         ( const Size& size );
  int  getBlockSizeTrafoScaleForDeQuant       ( const Size& size );
  bool needsBlockSizeTrafoScale       ( const Size& size );
}

UInt getCtuAddr        (const Position& pos, const PreCalcValues &pcv);

template<typename T, size_t N>
UInt updateCandList( T uiMode, Double uiCost, static_vector<T, N>& candModeList, static_vector<double, N>& candCostList, size_t uiFastCandNum = N )
{
  CHECK( std::min( uiFastCandNum, candModeList.size() ) != std::min( uiFastCandNum, candCostList.size() ), "Sizes do not match!" );
  CHECK( uiFastCandNum > candModeList.capacity(), "The vector is to small to hold all the candidates!" );

  size_t i;
  size_t shift = 0;
  size_t currSize = std::min( uiFastCandNum, candCostList.size() );

  while( shift < uiFastCandNum && shift < currSize && uiCost < candCostList[currSize - 1 - shift] )
  {
    shift++;
  }

  if( candModeList.size() >= uiFastCandNum && shift != 0 )
  {
    for( i = 1; i < shift; i++ )
    {
      candModeList[currSize - i] = candModeList[currSize - 1 - i];
      candCostList[currSize - i] = candCostList[currSize - 1 - i];
    }
    candModeList[currSize - shift] = uiMode;
    candCostList[currSize - shift] = uiCost;
    return 1;
  }
  else if( currSize < uiFastCandNum )
  {
    candModeList.insert( candModeList.end() - shift, uiMode );
    candCostList.insert( candCostList.end() - shift, uiCost );
    return 1;
  }

  return 0;
}


#endif

#if MCTS_ENC_CHECK
Void getTilePosition( const PredictionUnit& pu, UInt &tileXPosInCtus, UInt &tileYPosInCtus, UInt &tileWidthtInCtus, UInt &tileHeightInCtus );
#endif