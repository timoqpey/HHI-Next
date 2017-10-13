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

/** \file     Picture.h
 *  \brief    Description of a coded picture
 */

#ifndef __PICTURE__
#define __PICTURE__

#include "CommonDef.h"

#include "Common.h"
#include "Unit.h"
#include "Buffer.h"
#include "Unit.h"
#include "Slice.h"
#include "CodingStructure.h"

#include <deque>


class SEI;
class AQpLayer;

typedef std::list<SEI*> SEIMessages;

class Tile
{
private:
  UInt      m_tileWidthInCtus;
  UInt      m_tileHeightInCtus;
  UInt      m_rightEdgePosInCtus;
  UInt      m_bottomEdgePosInCtus;
  UInt      m_firstCtuRsAddr;

public:
  Tile();
  virtual ~Tile();

  Void      setTileWidthInCtus     ( UInt i )            { m_tileWidthInCtus = i; }
  UInt      getTileWidthInCtus     () const              { return m_tileWidthInCtus; }
  Void      setTileHeightInCtus    ( UInt i )            { m_tileHeightInCtus = i; }
  UInt      getTileHeightInCtus    () const              { return m_tileHeightInCtus; }
  Void      setRightEdgePosInCtus  ( UInt i )            { m_rightEdgePosInCtus = i; }
  UInt      getRightEdgePosInCtus  () const              { return m_rightEdgePosInCtus; }
  Void      setBottomEdgePosInCtus ( UInt i )            { m_bottomEdgePosInCtus = i; }
  UInt      getBottomEdgePosInCtus () const              { return m_bottomEdgePosInCtus; }
  Void      setFirstCtuRsAddr      ( UInt i )            { m_firstCtuRsAddr = i; }
  UInt      getFirstCtuRsAddr      () const              { return m_firstCtuRsAddr; }
};


struct TileMap
{
  TileMap();

  Void create( const SPS& sps, const PPS& pps );
  Void destroy();

  UInt getTileIdxMap( UInt ctuRsAddr )       const { return *(tileIdxMap + ctuRsAddr); }
  UInt getTileIdxMap( const Position& pos )  const { return getTileIdxMap( ( pos.x / pcv->maxCUWidth ) + ( pos.y / pcv->maxCUHeight ) * pcv->widthInCtus ); };
  UInt getCtuTsToRsAddrMap( UInt ctuTsAddr ) const { return *(ctuTsToRsAddrMap + (ctuTsAddr>=pcv->sizeInCtus ? pcv->sizeInCtus : ctuTsAddr)); }
  UInt getCtuRsToTsAddrMap( UInt ctuRsAddr ) const { return *(ctuRsToTsAddrMap + (ctuRsAddr>=pcv->sizeInCtus ? pcv->sizeInCtus : ctuRsAddr)); }
  UInt getSubstreamForCtuAddr(const UInt ctuAddr, const Bool bAddressInRaster, Slice *pcSlice) const;

  const PreCalcValues* pcv;
  std::vector<Tile> tiles;
  UInt  numTiles;
  UInt  numTileColumns;
  UInt  numTileRows;
  UInt* tileIdxMap;
  UInt* ctuTsToRsAddrMap;
  UInt* ctuRsToTsAddrMap;

  void initTileMap( const SPS& sps, const PPS& pps );
  void initCtuTsRsAddrMap();
  UInt calculateNextCtuRSAddr( const UInt currCtuRsAddr ) const;
};


struct Picture : public UnitArea
{
  UInt margin;

  Picture();

  Void create(const ChromaFormat &_chromaFormat, const Size &size, const unsigned _maxCUSize, const unsigned margin, const bool bDecoder);

  Void destroy();

  Void createTempBuffers( const unsigned _maxCUSize );
  Void destroyTempBuffers();

         PelBuf     getOrigBuf(const CompArea &blk);
  const CPelBuf     getOrigBuf(const CompArea &blk) const;
         PelUnitBuf getOrigBuf(const UnitArea &unit);
  const CPelUnitBuf getOrigBuf(const UnitArea &unit) const;
         PelUnitBuf getOrigBuf();
  const CPelUnitBuf getOrigBuf() const;

         PelBuf     getPredBuf(const CompArea &blk);
  const CPelBuf     getPredBuf(const CompArea &blk) const;
         PelUnitBuf getPredBuf(const UnitArea &unit);
  const CPelUnitBuf getPredBuf(const UnitArea &unit) const;

         PelBuf     getResiBuf(const CompArea &blk);
  const CPelBuf     getResiBuf(const CompArea &blk) const;
         PelUnitBuf getResiBuf(const UnitArea &unit);
  const CPelUnitBuf getResiBuf(const UnitArea &unit) const;

         PelBuf     getRecoBuf(const CompArea &blk);
  const CPelBuf     getRecoBuf(const CompArea &blk) const;
         PelUnitBuf getRecoBuf(const UnitArea &unit);
  const CPelUnitBuf getRecoBuf(const UnitArea &unit) const;
         PelUnitBuf getRecoBuf();
  const CPelUnitBuf getRecoBuf() const;

         PelBuf     getBuf(const CompArea &blk,  const PictureType &type);
  const CPelBuf     getBuf(const CompArea &blk,  const PictureType &type) const;
         PelUnitBuf getBuf(const UnitArea &unit, const PictureType &type);
  const CPelUnitBuf getBuf(const UnitArea &unit, const PictureType &type) const;

  void extendPicBorder();
  void finalInit( const SPS& sps, const PPS& pps );

  int  getPOC()                               const { return poc; }
  Void setBorderExtension( bool bFlag)              { m_bIsBorderExtended = bFlag;}

  Void setPrevQP(Int qp)                            { m_prevQP = qp; }
  Int& getPrevQP()                                  { return m_prevQP; }

public:
  bool m_bIsBorderExtended;
  bool referenced;
  bool reconstructed;
  bool neededForOutput;
  bool usedByCurr;
  bool longTerm;
  bool topField;
  bool fieldPic;
  int  m_prevQP;

  Int  poc;
  UInt layer;
  UInt depth;

  PelStorage m_bufs[NUM_PIC_TYPES];

  CodingStructure*   cs;
  std::deque<Slice*> slices;
  SEIMessages        SEIs; 

  Void         allocateNewSlice();
  Slice        *swapSliceObject(Slice * p, UInt i);
  void         clearSliceBuffer();

  TileMap*     tileMap;
  std::vector<AQpLayer*> aqlayer;

};

class SEIDecodedPictureHash;
int calcAndPrintHashStatus(const CPelUnitBuf& pic, const SEIDecodedPictureHash* pictureHashSEI, const BitDepths &bitDepths, const MsgLevel msgl);

void smoothResidual( PelBuf& resBuf, const CPelBuf& orgBuf, const ClpRng& clpRng );
void smoothResidual( PelUnitBuf& resBuf, const CPelUnitBuf& orgBuf, const ClpRngs& clpRngs);

typedef std::list<Picture*> PicList;

#endif
