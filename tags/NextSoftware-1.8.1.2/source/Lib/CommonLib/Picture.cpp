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

/** \file     Picture.cpp
 *  \brief    Description of a coded picture
 */

#include "Picture.h"
#include "SEI.h"
#include "ChromaFormat.h"

// ---------------------------------------------------------------------------
// picture methods
// ---------------------------------------------------------------------------

Tile::Tile()
: m_tileWidthInCtus     (0)
, m_tileHeightInCtus    (0)
, m_rightEdgePosInCtus  (0)
, m_bottomEdgePosInCtus (0)
, m_firstCtuRsAddr      (0)
{
}

Tile::~Tile()
{
}


TileMap::TileMap()
  : pcv(nullptr)
  , tiles(0)
  , numTiles(0)
  , numTileColumns(0)
  , numTileRows(0)
  , tileIdxMap(nullptr)
  , ctuTsToRsAddrMap(nullptr)
  , ctuRsToTsAddrMap(nullptr)
{
}

Void TileMap::create( const SPS& sps, const PPS& pps )
{
  pcv = pps.pcv;

  numTileColumns = pps.getNumTileColumnsMinus1() + 1;
  numTileRows    = pps.getNumTileRowsMinus1() + 1;
  numTiles       = numTileColumns * numTileRows;
  tiles.resize( numTiles );

  const UInt numCtusInFrame = pcv->sizeInCtus;
  tileIdxMap       = new UInt[numCtusInFrame];
  ctuTsToRsAddrMap = new UInt[numCtusInFrame+1];
  ctuRsToTsAddrMap = new UInt[numCtusInFrame+1];

  initTileMap( sps, pps );
  initCtuTsRsAddrMap();
}

Void TileMap::destroy()
{
  tiles.clear();

  if ( tileIdxMap )
  {
    delete[] tileIdxMap;
    tileIdxMap = nullptr;
  }

  if ( ctuTsToRsAddrMap )
  {
    delete[] ctuTsToRsAddrMap;
    ctuTsToRsAddrMap = nullptr;
  }

  if ( ctuRsToTsAddrMap )
  {
    delete[] ctuRsToTsAddrMap;
    ctuRsToTsAddrMap = nullptr;
  }
}

void TileMap::initTileMap( const SPS& sps, const PPS& pps )
{
  const UInt frameWidthInCtus  = pcv->widthInCtus;
  const UInt frameHeightInCtus = pcv->heightInCtus;

  if( pps.getTileUniformSpacingFlag() )
  {
    //set width and height for each (uniform) tile
    for(Int row=0; row < numTileRows; row++)
    {
      for(Int col=0; col < numTileColumns; col++)
      {
        const Int tileIdx = row * numTileColumns + col;
        tiles[tileIdx].setTileWidthInCtus(  (col+1)*frameWidthInCtus/numTileColumns - (col*frameWidthInCtus)/numTileColumns );
        tiles[tileIdx].setTileHeightInCtus( (row+1)*frameHeightInCtus/numTileRows   - (row*frameHeightInCtus)/numTileRows );
      }
    }
  }
  else
  {
    //set the width for each tile
    for(Int row=0; row < numTileRows; row++)
    {
      Int cumulativeTileWidth = 0;
      for(Int col=0; col < numTileColumns - 1; col++)
      {
        tiles[row * numTileColumns + col].setTileWidthInCtus( pps.getTileColumnWidth(col) );
        cumulativeTileWidth += pps.getTileColumnWidth(col);
      }
      tiles[row * numTileColumns + numTileColumns - 1].setTileWidthInCtus( frameWidthInCtus-cumulativeTileWidth );
    }

    //set the height for each tile
    for(Int col=0; col < numTileColumns; col++)
    {
      Int cumulativeTileHeight = 0;
      for(Int row=0; row < numTileRows - 1; row++)
      {
        tiles[row * numTileColumns + col].setTileHeightInCtus( pps.getTileRowHeight(row) );
        cumulativeTileHeight += pps.getTileRowHeight(row);
      }
      tiles[(numTileRows - 1) * numTileColumns + col].setTileHeightInCtus( frameHeightInCtus-cumulativeTileHeight );
    }
  }

  // Tile size check
  Int minWidth  = 1;
  Int minHeight = 1;
  const Int profileIdc = sps.getPTL()->getGeneralPTL()->getProfileIdc();
  if (  profileIdc == Profile::MAIN || profileIdc == Profile::MAIN10)
  {
    if (pps.getTilesEnabledFlag())
    {
      minHeight = 64  / sps.getMaxCUHeight();
      minWidth  = 256 / sps.getMaxCUWidth();
    }
  }
  for(Int row=0; row < numTileRows; row++)
  {
    for(Int col=0; col < numTileColumns; col++)
    {
      const Int tileIdx = row * numTileColumns + col;
      if(tiles[tileIdx].getTileWidthInCtus() < minWidth)   { THROW("Invalid tile size"); }
      if(tiles[tileIdx].getTileHeightInCtus() < minHeight) { THROW("Invalid tile size"); }
    }
  }

  //initialize each tile of the current picture
  for( Int row=0; row < numTileRows; row++ )
  {
    for( Int col=0; col < numTileColumns; col++ )
    {
      const Int tileIdx = row * numTileColumns + col;

      //initialize the RightEdgePosInCU for each tile
      Int rightEdgePosInCTU = 0;
      for( Int i=0; i <= col; i++ )
      {
        rightEdgePosInCTU += tiles[row * numTileColumns + i].getTileWidthInCtus();
      }
      tiles[tileIdx].setRightEdgePosInCtus(rightEdgePosInCTU-1);

      //initialize the BottomEdgePosInCU for each tile
      Int bottomEdgePosInCTU = 0;
      for( Int i=0; i <= row; i++ )
      {
        bottomEdgePosInCTU += tiles[i * numTileColumns + col].getTileHeightInCtus();
      }
      tiles[tileIdx].setBottomEdgePosInCtus(bottomEdgePosInCTU-1);

      //initialize the FirstCUAddr for each tile
      tiles[tileIdx].setFirstCtuRsAddr( (tiles[tileIdx].getBottomEdgePosInCtus() - tiles[tileIdx].getTileHeightInCtus() + 1) * frameWidthInCtus +
                                         tiles[tileIdx].getRightEdgePosInCtus()  - tiles[tileIdx].getTileWidthInCtus()  + 1);
    }
  }

  Int  columnIdx = 0;
  Int  rowIdx = 0;

  //initialize the TileIdxMap
  const UInt numCtusInFrame = pcv->sizeInCtus;
  for( Int i=0; i<numCtusInFrame; i++)
  {
    for( Int col=0; col < numTileColumns; col++)
    {
      if(i % frameWidthInCtus <= tiles[col].getRightEdgePosInCtus())
      {
        columnIdx = col;
        break;
      }
    }
    for(Int row=0; row < numTileRows; row++)
    {
      if(i / frameWidthInCtus <= tiles[row*numTileColumns].getBottomEdgePosInCtus())
      {
        rowIdx = row;
        break;
      }
    }
    tileIdxMap[i] = rowIdx * numTileColumns + columnIdx;
  }
}

void TileMap::initCtuTsRsAddrMap()
{
  //generate the Coding Order Map and Inverse Coding Order Map
  const UInt numCtusInFrame = pcv->sizeInCtus;
  for(Int ctuTsAddr=0, ctuRsAddr=0; ctuTsAddr<numCtusInFrame; ctuTsAddr++, ctuRsAddr = calculateNextCtuRSAddr(ctuRsAddr))
  {
    ctuTsToRsAddrMap[ctuTsAddr] = ctuRsAddr;
    ctuRsToTsAddrMap[ctuRsAddr] = ctuTsAddr;
  }
  ctuTsToRsAddrMap[numCtusInFrame] = numCtusInFrame;
  ctuRsToTsAddrMap[numCtusInFrame] = numCtusInFrame;
}

UInt TileMap::calculateNextCtuRSAddr( const UInt currCtuRsAddr ) const
{
  const UInt frameWidthInCtus = pcv->widthInCtus;
  UInt  nextCtuRsAddr;

  //get the tile index for the current CTU
  const UInt uiTileIdx = getTileIdxMap(currCtuRsAddr);

  //get the raster scan address for the next CTU
  if( currCtuRsAddr % frameWidthInCtus == tiles[uiTileIdx].getRightEdgePosInCtus() && currCtuRsAddr / frameWidthInCtus == tiles[uiTileIdx].getBottomEdgePosInCtus() )
  //the current CTU is the last CTU of the tile
  {
    if(uiTileIdx+1 == numTiles)
    {
      nextCtuRsAddr = pcv->sizeInCtus;
    }
    else
    {
      nextCtuRsAddr = tiles[uiTileIdx+1].getFirstCtuRsAddr();
    }
  }
  else //the current CTU is not the last CTU of the tile
  {
    if( currCtuRsAddr % frameWidthInCtus == tiles[uiTileIdx].getRightEdgePosInCtus() )  //the current CTU is on the rightmost edge of the tile
    {
      nextCtuRsAddr = currCtuRsAddr + frameWidthInCtus - tiles[uiTileIdx].getTileWidthInCtus() + 1;
    }
    else
    {
      nextCtuRsAddr = currCtuRsAddr + 1;
    }
  }

  return nextCtuRsAddr;
}

UInt TileMap::getSubstreamForCtuAddr(const UInt ctuAddr, const Bool bAddressInRaster, Slice *pcSlice) const
{
  const bool bWPPEnabled = pcSlice->getPPS()->getEntropyCodingSyncEnabledFlag();
  UInt subStrm;

  if( (bWPPEnabled && pcv->heightInCtus > 1) || (numTiles > 1) ) // wavefronts, and possibly tiles being used.
  {
    const UInt ctuRsAddr = bAddressInRaster ? ctuAddr : getCtuTsToRsAddrMap(ctuAddr);
    const UInt tileIndex = getTileIdxMap(ctuRsAddr);

    if (bWPPEnabled)
    {
      const UInt firstCtuRsAddrOfTile     = tiles[tileIndex].getFirstCtuRsAddr();
      const UInt tileYInCtus              = firstCtuRsAddrOfTile / pcv->widthInCtus;
      const UInt ctuLine                  = ctuRsAddr / pcv->widthInCtus;
      const UInt startingSubstreamForTile = (tileYInCtus * numTileColumns) + (tiles[tileIndex].getTileHeightInCtus() * (tileIndex % numTileColumns));

      subStrm = startingSubstreamForTile + (ctuLine - tileYInCtus);
    }
    else
    {
      subStrm = tileIndex;
    }
  }
  else
  {
    subStrm = 0;
  }
  return subStrm;
}


Picture::Picture()
{
  tileMap              = nullptr;
  cs                   = nullptr;
  m_bIsBorderExtended  = false;
  usedByCurr           = false;
  longTerm             = false;
  reconstructed        = false;
  neededForOutput      = false;
  referenced           = false;
  layer                = std::numeric_limits<UInt>::max();
}


Void Picture::create(const ChromaFormat &_chromaFormat, const Size &size, const unsigned _maxCUSize, const unsigned _margin, const bool _decoder)
{
  UnitArea::operator=( UnitArea( _chromaFormat, Area( Position{ 0, 0 }, size ) ) );
  margin            =  _margin;

  const Area a      = Area( Position(), size );
  m_bufs[PIC_RECONSTRUCTION].create( _chromaFormat, a, _maxCUSize, _margin, MEMORY_ALIGN_DEF_SIZE );

  if( !_decoder )
  {
    m_bufs[PIC_ORIGINAL].    create( chromaFormat, a );
  }
}

Void Picture::destroy()
{
  for (UInt t = 0; t < NUM_PIC_TYPES; t++)
  {
    m_bufs[t].destroy();
  }

  if( cs )
  {
    cs->destroy();
    delete cs;
    cs = nullptr;
  }

  for( auto &ps : slices )
  {
    delete ps;
  }
  slices.clear();

  for( auto &psei : SEIs )
  {
    delete psei;
  }
  SEIs.clear();

  if ( tileMap )
  {
    tileMap->destroy();
    delete tileMap;
    tileMap = nullptr;
  }
}

Void Picture::createTempBuffers( const unsigned _maxCUSize )
{
  const Area a( Position{ 0, 0 }, lumaSize() );

  m_bufs[PIC_PREDICTION].create( chromaFormat, a, _maxCUSize );
  m_bufs[PIC_RESIDUAL].  create( chromaFormat, a, _maxCUSize );

  if( cs ) cs->rebindPicBufs();
}

Void Picture::destroyTempBuffers()
{
  for( UInt t = 0; t < NUM_PIC_TYPES; t++ )
  {
    if( t != PIC_RECONSTRUCTION && t != PIC_ORIGINAL ) m_bufs[t].destroy();
  }

  if( cs ) cs->rebindPicBufs();
}

       PelBuf     Picture::getOrigBuf(const CompArea &blk)        { return getBuf(blk,  PIC_ORIGINAL); }
const CPelBuf     Picture::getOrigBuf(const CompArea &blk)  const { return getBuf(blk,  PIC_ORIGINAL); }
       PelUnitBuf Picture::getOrigBuf(const UnitArea &unit)       { return getBuf(unit, PIC_ORIGINAL); }  
const CPelUnitBuf Picture::getOrigBuf(const UnitArea &unit) const { return getBuf(unit, PIC_ORIGINAL); }
       PelUnitBuf Picture::getOrigBuf()                           { return m_bufs[PIC_ORIGINAL]; }
const CPelUnitBuf Picture::getOrigBuf()                     const { return m_bufs[PIC_ORIGINAL]; }

       PelBuf     Picture::getPredBuf(const CompArea &blk)        { return getBuf(blk,  PIC_PREDICTION); }
const CPelBuf     Picture::getPredBuf(const CompArea &blk)  const { return getBuf(blk,  PIC_PREDICTION); }
       PelUnitBuf Picture::getPredBuf(const UnitArea &unit)       { return getBuf(unit, PIC_PREDICTION); }
const CPelUnitBuf Picture::getPredBuf(const UnitArea &unit) const { return getBuf(unit, PIC_PREDICTION); }

       PelBuf     Picture::getResiBuf(const CompArea &blk)        { return getBuf(blk,  PIC_RESIDUAL); }
const CPelBuf     Picture::getResiBuf(const CompArea &blk)  const { return getBuf(blk,  PIC_RESIDUAL); }
       PelUnitBuf Picture::getResiBuf(const UnitArea &unit)       { return getBuf(unit, PIC_RESIDUAL); }
const CPelUnitBuf Picture::getResiBuf(const UnitArea &unit) const { return getBuf(unit, PIC_RESIDUAL); }

       PelBuf     Picture::getRecoBuf(const CompArea &blk)        { return getBuf(blk,  PIC_RECONSTRUCTION); }
const CPelBuf     Picture::getRecoBuf(const CompArea &blk)  const { return getBuf(blk,  PIC_RECONSTRUCTION); }
       PelUnitBuf Picture::getRecoBuf(const UnitArea &unit)       { return getBuf(unit, PIC_RECONSTRUCTION); }
const CPelUnitBuf Picture::getRecoBuf(const UnitArea &unit) const { return getBuf(unit, PIC_RECONSTRUCTION); }
       PelUnitBuf Picture::getRecoBuf()                           { return m_bufs[PIC_RECONSTRUCTION]; }
const CPelUnitBuf Picture::getRecoBuf()                     const { return m_bufs[PIC_RECONSTRUCTION]; }



Void Picture::finalInit( const SPS& sps, const PPS& pps )
{
  for( auto &sei : SEIs )
  {
    delete sei;
  }
  SEIs.clear();
  clearSliceBuffer();

  if( tileMap )
  {
    tileMap->destroy();
    delete tileMap;
    tileMap = nullptr;
  }

  const ChromaFormat chromaFormatIDC = sps.getChromaFormatIdc();
  const Int          iWidth = sps.getPicWidthInLumaSamples();
  const Int          iHeight = sps.getPicHeightInLumaSamples();

  if( cs )
  {
    cs->initStructData();
  }
  else
  {
    cs = new CodingStructure;
    cs->sps = &sps;
    cs->create( chromaFormatIDC, Area( 0, 0, iWidth, iHeight ), true );
  }

  cs->picture = this;
  cs->slice   = nullptr;  // the slices for this picture have not been set at this point. update cs->slice after swapSliceObject()
  cs->pps     = &pps;
  cs->vps     = nullptr;
  cs->pcv     = pps.pcv;

  cs->chType  = CHANNEL_TYPE_LUMA;

  tileMap = new TileMap;
  tileMap->create( sps, pps );
}

Void Picture::allocateNewSlice()
{
  slices.push_back(new Slice);
  Slice& slice = *slices.back();

  slice.setPPS( cs->pps);
  slice.setSPS( cs->sps);
  if(slices.size()>=2)
  {
    slice.copySliceInfo( slices[slices.size()-2] );
    slice.initSlice();
  }
}

Slice *Picture::swapSliceObject(Slice * p, UInt i)
{
  p->setSPS(cs->sps);
  p->setPPS(cs->pps);

  Slice * pTmp = slices[i];
  slices[i] = p;
  pTmp->setSPS(0);
  pTmp->setPPS(0);
  return pTmp;
}

void Picture::clearSliceBuffer()
{
  for (UInt i = 0; i < UInt(slices.size()); i++)
  {
    delete slices[i];
  }
  slices.clear();
}

void Picture::extendPicBorder()
{
  if ( m_bIsBorderExtended )
  {
    return;
  }

  for(Int comp=0; comp<getNumberValidComponents( cs->area.chromaFormat ); comp++)
  {
    ComponentID compID = ComponentID( comp );
    PelBuf p = m_bufs[PIC_RECONSTRUCTION].get( compID );
    Pel *piTxt = p.bufAt(0,0);
    int xmargin = margin >> getComponentScaleX( compID, cs->area.chromaFormat );
    int ymargin = margin >> getComponentScaleY( compID, cs->area.chromaFormat );

    Pel*  pi = piTxt;
    // do left and right margins
    for (Int y = 0; y < p.height; y++)
    {
      for (Int x = 0; x < xmargin; x++ )
      {
        pi[ -xmargin + x ] = pi[0];
        pi[  p.width + x ] = pi[p.width-1];
      }
      pi += p.stride;
    }

    // pi is now the (0,height) (bottom left of image within bigger picture
    pi -= (p.stride + xmargin);
    // pi is now the (-marginX, height-1)
    for (Int y = 0; y < ymargin; y++ )
    {
      ::memcpy( pi + (y+1)*p.stride, pi, sizeof(Pel)*(p.width + (xmargin << 1)));
    }

    // pi is still (-marginX, height-1)
    pi -= ((p.height-1) * p.stride);
    // pi is now (-marginX, 0)
    for (Int y = 0; y < ymargin; y++ )
    {
      ::memcpy( pi - (y+1)*p.stride, pi, sizeof(Pel)*(p.width + (xmargin<<1)) );
    }
  }

  m_bIsBorderExtended = true;
}


PelBuf Picture::getBuf(const CompArea &blk, const PictureType &type)
{
  if (!blk.valid())
  {
    return PelBuf();
  }

  return m_bufs[type].getBuf(blk);
}

const CPelBuf Picture::getBuf(const CompArea &blk, const PictureType &type) const
{
  if (!blk.valid())
  {
    return PelBuf();
  }

  return m_bufs[type].getBuf(blk);
}

PelUnitBuf Picture::getBuf(const UnitArea &unit, const PictureType &type)
{
  if( chromaFormat == CHROMA_400)
  {
    return PelUnitBuf(chromaFormat, getBuf(unit.Y(), type));
  }
  else
  {
    return PelUnitBuf(chromaFormat, getBuf(unit.Y(), type), getBuf(unit.Cb(), type), getBuf(unit.Cr(), type));
  }
}

const CPelUnitBuf Picture::getBuf(const UnitArea &unit, const PictureType &type) const
{
  if (chromaFormat == CHROMA_400)
  {
    return CPelUnitBuf(chromaFormat, getBuf(unit.Y(), type));
  }
  else
  {
    return CPelUnitBuf(chromaFormat, getBuf(unit.Y(), type), getBuf(unit.Cb(), type), getBuf(unit.Cr(), type));
  }
}

template <int N>
Int average(const std::vector<Pel> &r,unsigned i0, unsigned j0, unsigned uiHeight, unsigned uiWidth) 
{
  Int s=0;
  for(int i=-N/2;i<=N/2;++i)
  {
    const int ii=(i0+i+uiHeight)%uiHeight;
    for(int j=-N/2;j<=N/2;++j)
    {
      const int jj=(j0+j+uiWidth)%uiWidth;
      s+=r[ii*uiWidth+jj];
    }
  }
  return (s+(N*N)/2)/(N*N);
}

void smoothResidual( std::vector<Pel> &res, const std::vector<char> &bmM, const ClpRng& clpRng, unsigned uiHeight, unsigned uiWidth) 
{
  // find boundaries of the res
  static std::vector<Pel> r;
  r=res;

  const int cptmax = 4;
  int  cpt = 0;
  bool cont=true;
  while( cpt < cptmax && cont) 
  {
    cont=false;
    for( unsigned i = 0; i < uiHeight; i++)
    {
      for( unsigned j = 0; j < uiWidth; j++) 
      {
        const unsigned k = i*uiWidth+j;
        if( bmM[k] == -1 ) 
        { // we can lower the res
          int s=average<3>(res,i,j,uiHeight,uiWidth);
          if( s < res[k] ) 
          {
            r[k]=s;
            cont=true;
          }
        } 
        else if( bmM[k] == 1 ) 
        { // we can inc the res
          int s=average<3>(res,i,j,uiHeight,uiWidth);
          if( s > res[k] ) 
          {
            r[k]=s;
            cont=true;
          }
        }
      }
    }
    ++cpt;
    if( cont )  
    {
      std::copy(r.begin(),r.end(),res.begin());
    }
  }
}

void smoothResidual( PelBuf& resBuf, const CPelBuf& orgBuf, const ClpRng& clpRng ) 
{
  const unsigned uiWidth      = orgBuf.width;
  const unsigned uiHeight     = orgBuf.height;
  const unsigned uiStrideOrg  = orgBuf.stride;
  const Pel*     piOrg        = orgBuf.buf;
  Pel*           piResi       = resBuf.buf;
  const unsigned uiStrideRes  = resBuf.stride;

  const unsigned areaSize     = uiWidth*uiHeight;
  const unsigned cpySize      = uiWidth*sizeof(Pel);

  static std::vector<char> bmM;
  bmM.resize( areaSize );
  memset( &bmM[0], 0, areaSize );

  bool activate=false;
  for( unsigned k = 0, h = 0; h < uiHeight; h++)
  {
    for( unsigned w = 0; w < uiWidth; w++, k++)
    {
      const Pel orgPel = piOrg[w+h*uiStrideOrg];

      if( orgPel <= clpRng.min)      { bmM[k]=-1; activate=true; }
      else if( orgPel >= clpRng.max) { bmM[k]=+1; activate=true; }
    }
  }

  if (activate) 
  {
    static std::vector<Pel> r; // avoid realloc
    r.resize( areaSize );

    for( unsigned h = 0; h < uiHeight; h++)
    {
      memcpy( &r[h*uiWidth], &piResi[h*uiStrideRes], cpySize);
    }

    smoothResidual( r, bmM, clpRng, uiHeight, uiWidth);

    // copy back
    for( unsigned h = 0; h < uiHeight; h++)
    {
      memcpy( &piResi[h*uiStrideRes], &r[h*uiWidth], cpySize);
    }
  }
}

void smoothResidual( PelUnitBuf& resBuf, const CPelUnitBuf& orgBuf, const ClpRngs& clpRngs) 
{
  for(size_t comp=0; comp< resBuf.bufs.size(); ++comp) 
  {
    const ComponentID compID = ComponentID(comp);
    smoothResidual( resBuf.get(compID), orgBuf.get(compID), clpRngs.comp[compID]);
  }
} // anom

