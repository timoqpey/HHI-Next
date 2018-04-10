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

/** \file     DecSlice.cpp
    \brief    slice decoder class
*/

#include "DecSlice.h"
#include "CommonLib/UnitTools.h"
#include "CommonLib/dtrace_next.h"

#include <vector>

//! \ingroup DecoderLib
//! \{

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

DecSlice::DecSlice()
{
}

DecSlice::~DecSlice()
{
}

Void DecSlice::create()
{
}

Void DecSlice::destroy()
{
}

#if JEM_TOOLS
Void DecSlice::init( CABACDataStore* cabacDataStore, CABACDecoder* cabacDecoder, DecCu* pcCuDecoder )
{
  m_CABACDataStore  = cabacDataStore;
  m_CABACDecoder    = cabacDecoder;
  m_pcCuDecoder     = pcCuDecoder;
}
#else
Void DecSlice::init( CABACDecoder* cabacDecoder, DecCu* pcCuDecoder )
{
  m_CABACDecoder    = cabacDecoder;
  m_pcCuDecoder     = pcCuDecoder;
}
#endif

Void DecSlice::decompressSlice( Slice* slice, InputBitstream* bitstream )
{
  //-- For time output for each slice
  slice->startProcessingTimer();

  const SPS*     sps          = slice->getSPS();
  Picture*       pic          = slice->getPic();
  const TileMap& tileMap      = *pic->tileMap;
#if JEM_TOOLS
  CABACReader&   cabacReader  = *m_CABACDecoder->getCABACReader( sps->getSpsNext().getCABACEngineMode() );
  m_CABACDataStore->updateBufferState( slice );
#else
  CABACReader&   cabacReader  = *m_CABACDecoder->getCABACReader( 0 );
#endif

  // setup coding structure
  CodingStructure& cs = *pic->cs;
  cs.slice            = slice;
  cs.sps              = sps;
  cs.pps              = slice->getPPS();
  cs.vps              = slice->getVPS();
  cs.pcv              = slice->getPPS()->pcv;
  cs.chromaQpAdj      = 0;

  cs.picture->resizeSAO(cs.pcv->sizeInCtus, 0);

  const unsigned numSubstreams = slice->getNumberOfSubstreamSizes() + 1;

  // init each couple {EntropyDecoder, Substream}
  // Table of extracted substreams.
  std::vector<InputBitstream*> ppcSubstreams( numSubstreams );
  for( unsigned idx = 0; idx < numSubstreams; idx++ )
  {
    ppcSubstreams[idx] = bitstream->extractSubstream( idx+1 < numSubstreams ? ( slice->getSubstreamSize(idx) << 3 ) : bitstream->getNumBitsLeft() );
  }

  const int       startCtuTsAddr          = slice->getSliceSegmentCurStartCtuTsAddr();
  const int       startCtuRsAddr          = tileMap.getCtuTsToRsAddrMap(startCtuTsAddr);
  const unsigned  numCtusInFrame          = cs.pcv->sizeInCtus;
  const unsigned  widthInCtus             = cs.pcv->widthInCtus;
  const bool      depSliceSegmentsEnabled = cs.pps->getDependentSliceSegmentsEnabledFlag();
  const bool      wavefrontsEnabled       = cs.pps->getEntropyCodingSyncEnabledFlag();

  cabacReader.initBitstream( ppcSubstreams[0] );
#if JEM_TOOLS
  cabacReader.initCtxModels( *slice, m_CABACDataStore );
#else
  cabacReader.initCtxModels( *slice );
#endif

  // Quantization parameter
  if(!slice->getDependentSliceSegmentFlag())
  {
    pic->m_prevQP[0] = pic->m_prevQP[1] = slice->getSliceQp();
  }
  CHECK( pic->m_prevQP[0] == std::numeric_limits<Int>::max(), "Invalid previous QP" );

  DTRACE( g_trace_ctx, D_HEADER, "=========== POC: %d ===========\n", slice->getPOC() );

  // The first CTU of the slice is the first coded substream, but the global substream number, as calculated by getSubstreamForCtuAddr may be higher.
  // This calculates the common offset for all substreams in this slice.
  const unsigned subStreamOffset = tileMap.getSubstreamForCtuAddr( startCtuRsAddr, true, slice );

  if( depSliceSegmentsEnabled )
  {
    // modify initial contexts with previous slice segment if this is a dependent slice.
    const unsigned  startTileIdx          = tileMap.getTileIdxMap(startCtuRsAddr);
    const Tile&     currentTile           = tileMap.tiles[startTileIdx];
    const unsigned  firstCtuRsAddrOfTile  = currentTile.getFirstCtuRsAddr();
    if( slice->getDependentSliceSegmentFlag() && startCtuRsAddr != firstCtuRsAddrOfTile )
    {
      if( currentTile.getTileWidthInCtus() >= 2 || !wavefrontsEnabled )
      {
        cabacReader.getCtx() = m_lastSliceSegmentEndContextState;
      }
    }
  }
  // for every CTU in the slice segment...
  bool isLastCtuOfSliceSegment = false;
  for( unsigned ctuTsAddr = startCtuTsAddr; !isLastCtuOfSliceSegment && ctuTsAddr < numCtusInFrame; ctuTsAddr++ )
  {
    const unsigned  ctuRsAddr             = tileMap.getCtuTsToRsAddrMap(ctuTsAddr);
    const Tile&     currentTile           = tileMap.tiles[ tileMap.getTileIdxMap(ctuRsAddr) ];
    const unsigned  firstCtuRsAddrOfTile  = currentTile.getFirstCtuRsAddr();
    const unsigned  tileXPosInCtus        = firstCtuRsAddrOfTile % widthInCtus;
    const unsigned  tileYPosInCtus        = firstCtuRsAddrOfTile / widthInCtus;
    const unsigned  ctuXPosInCtus         = ctuRsAddr % widthInCtus;
    const unsigned  ctuYPosInCtus         = ctuRsAddr / widthInCtus;
    const unsigned  subStrmId             = tileMap.getSubstreamForCtuAddr( ctuRsAddr, true, slice ) - subStreamOffset;
    const unsigned  maxCUSize             = sps->getMaxCUWidth();
#if JEM_TOOLS
    const CIPFSpec  cipf                  = getCIPFSpec( slice, ctuXPosInCtus, ctuYPosInCtus );
#endif
    Position pos( ctuXPosInCtus*maxCUSize, ctuYPosInCtus*maxCUSize) ;
    UnitArea ctuArea(cs.area.chromaFormat, Area( pos.x, pos.y, maxCUSize, maxCUSize ) );

    DTRACE_UPDATE( g_trace_ctx, std::make_pair( "ctu", ctuRsAddr ) );

    cabacReader.initBitstream( ppcSubstreams[subStrmId] );

    // set up CABAC contexts' state for this CTU
    if( ctuRsAddr == firstCtuRsAddrOfTile )
    {
      if( ctuTsAddr != startCtuTsAddr ) // if it is the first CTU, then the entropy coder has already been reset
      {
#if JEM_TOOLS
        cabacReader.initCtxModels( *slice, m_CABACDataStore );
#else
        cabacReader.initCtxModels( *slice );
#endif
      }
      pic->m_prevQP[0] = pic->m_prevQP[1] = slice->getSliceQp();
    }
    else if( ctuXPosInCtus == tileXPosInCtus && wavefrontsEnabled )
    {
      // Synchronize cabac probabilities with upper-right CTU if it's available and at the start of a line.
      if( ctuTsAddr != startCtuTsAddr ) // if it is the first CTU, then the entropy coder has already been reset
      {
#if JEM_TOOLS
        cabacReader.initCtxModels( *slice, m_CABACDataStore );
#else
        cabacReader.initCtxModels( *slice );
#endif
      }
      if( cs.getCURestricted( pos.offset(maxCUSize, -1), slice->getIndependentSliceIdx(), tileMap.getTileIdxMap( pos ), CH_L ) )
      {
        // Top-right is available, so use it.
        cabacReader.getCtx() = m_entropyCodingSyncContextState;
      }
      pic->m_prevQP[0] = pic->m_prevQP[1] = slice->getSliceQp();
    }

#if JEM_TOOLS
    // load ctx from previous frame
    if( cipf.loadCtx )
    {
      m_CABACDataStore->loadCtxStates( slice, cabacReader.getCtx(), cipf.ctxId );
    }
#endif
#if JEM_TOOLS

    if( ctuRsAddr == 0 )
    {
      cabacReader.alf( cs );
    }
#endif
    isLastCtuOfSliceSegment = cabacReader.coding_tree_unit( cs, ctuArea, pic->m_prevQP, ctuRsAddr );

    m_pcCuDecoder->decompressCtu( cs, ctuArea );


    if( ctuXPosInCtus == tileXPosInCtus+1 && wavefrontsEnabled )
    {
      m_entropyCodingSyncContextState = cabacReader.getCtx();
    }

#if JEM_TOOLS
    // store CABAC context to be used in next frames
    if( cipf.storeCtx )
    {
      m_CABACDataStore->storeCtxStates( slice, cabacReader.getCtx(), cipf.ctxId );
    }
#endif

    if( isLastCtuOfSliceSegment )
    {
#if DECODER_CHECK_SUBSTREAM_AND_SLICE_TRAILING_BYTES
      cabacReader.remaining_bytes( false );
#endif
      if( !slice->getDependentSliceSegmentFlag() )
      {
        slice->setSliceCurEndCtuTsAddr( ctuTsAddr+1 );
      }
      slice->setSliceSegmentCurEndCtuTsAddr( ctuTsAddr+1 );
    }
    else if( ( ctuXPosInCtus + 1 == tileXPosInCtus + currentTile.getTileWidthInCtus () ) &&
             ( ctuYPosInCtus + 1 == tileYPosInCtus + currentTile.getTileHeightInCtus() || wavefrontsEnabled ) )
    {
      // The sub-stream/stream should be terminated after this CTU.
      // (end of slice-segment, end of tile, end of wavefront-CTU-row)
      unsigned binVal = cabacReader.terminating_bit();
      CHECK( !binVal, "Expecting a terminating bit" );
#if DECODER_CHECK_SUBSTREAM_AND_SLICE_TRAILING_BYTES
      cabacReader.remaining_bytes( true );
#endif
    }
  }
  CHECK( !isLastCtuOfSliceSegment, "Last CTU of slice segment not signalled as such" );

  if( depSliceSegmentsEnabled )
  {
    m_lastSliceSegmentEndContextState = cabacReader.getCtx();  //ctx end of dep.slice
  }
  // deallocate all created substreams, including internal buffers.
  for( auto substr: ppcSubstreams )
  {
    delete substr;
  }
  slice->stopProcessingTimer();
}

//! \}