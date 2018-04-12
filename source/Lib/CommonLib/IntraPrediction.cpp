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

/** \file     Prediction.cpp
    \brief    prediction class
*/

#include "IntraPrediction.h"

#include "Unit.h"
#include "UnitTools.h"
#include "Buffer.h"

#include "dtrace_next.h"
#include "Rom.h"
#include "DiffusionFilter.h"

#include <memory.h>

//! \ingroup CommonLib
//! \{

// ====================================================================================================================
// Tables
// ====================================================================================================================

const UChar IntraPrediction::m_aucIntraFilter[MAX_NUM_CHANNEL_TYPE][MAX_INTRA_FILTER_DEPTHS] =
{
  { // Luma
    20, //   1xn
    20, //   2xn
    20, //   4xn
    14, //   8xn
    2,  //  16xn
    0,  //  32xn
    0,  //  64xn
    0,  // 128xn
  },
  { // Chroma
    40, //   1xn
    40, //   2xn
    40, //   4xn
    28, //   8xn
    4,  //  16xn
    0,  //  32xn
    0,  //  64xn
    0,  // 128xn
  }
};
const int IntraPrediction::m_intraBiFiWeights[177][5] =
{
  { 1024, 992, 904, 773, 621 },
  { 1024, 992, 903, 773, 621 },
  { 1023, 992, 903, 772, 620 },
  { 1022, 990, 902, 771, 620 },
  { 1020, 989, 900, 770, 619 },
  { 1018, 986, 898, 768, 617 },
  { 1015, 984, 896, 766, 616 },
  { 1012, 981, 893, 764, 614 },
  { 1008, 977, 890, 761, 611 },
  { 1004, 973, 886, 758, 609 },
  {  999, 969, 882, 754, 606 },
  {  994, 964, 877, 750, 603 },
  {  989, 958, 872, 746, 600 },
  {  983, 952, 867, 742, 596 },
  {  976, 946, 861, 737, 592 },
  {  969, 939, 855, 732, 588 },
  {  962, 932, 849, 726, 583 },
  {  954, 925, 842, 720, 579 },
  {  946, 917, 835, 714, 574 },
  {  938, 909, 827, 708, 569 },
  {  929, 900, 820, 701, 563 },
  {  919, 891, 811, 694, 558 },
  {  910, 882, 803, 687, 552 },
  {  900, 872, 794, 679, 546 },
  {  890, 862, 785, 672, 540 },
  {  879, 852, 776, 664, 533 },
  {  868, 841, 766, 655, 527 },
  {  857, 831, 756, 647, 520 },
  {  846, 820, 746, 638, 513 },
  {  834, 808, 736, 629, 506 },
  {  822, 797, 725, 620, 499 },
  {  810, 785, 715, 611, 491 },
  {  797, 773, 704, 602, 484 },
  {  785, 761, 693, 593, 476 },
  {  772, 748, 681, 583, 468 },
  {  759, 736, 670, 573, 461 },
  {  746, 723, 659, 563, 453 },
  {  733, 711, 647, 553, 445 },
  {  720, 698, 635, 543, 437 },
  {  706, 685, 623, 533, 428 },
  {  693, 672, 611, 523, 420 },
  {  679, 658, 599, 513, 412 },
  {  666, 645, 587, 502, 404 },
  {  652, 632, 575, 492, 395 },
  {  638, 619, 563, 482, 387 },
  {  625, 605, 551, 471, 379 },
  {  611, 592, 539, 461, 371 },
  {  597, 579, 527, 451, 362 },
  {  583, 566, 515, 440, 354 },
  {  570, 552, 503, 430, 346 },
  {  556, 539, 491, 420, 337 },
  {  543, 526, 479, 410, 329 },
  {  529, 513, 467, 399, 321 },
  {  516, 500, 455, 389, 313 },
  {  502, 487, 443, 379, 305 },
  {  489, 474, 432, 369, 297 },
  {  476, 462, 420, 359, 289 },
  {  463, 449, 409, 350, 281 },
  {  450, 437, 397, 340, 273 },
  {  438, 424, 386, 330, 266 },
  {  425, 412, 375, 321, 258 },
  {  413, 400, 364, 312, 250 },
  {  401, 388, 354, 302, 243 },
  {  389, 377, 343, 293, 236 },
  {  377, 365, 332, 284, 228 },
  {  365, 354, 322, 276, 221 },
  {  354, 343, 312, 267, 214 },
  {  342, 332, 302, 258, 208 },
  {  331, 321, 292, 250, 201 },
  {  320, 310, 283, 242, 194 },
  {  310, 300, 273, 234, 188 },
  {  299, 290, 264, 226, 181 },
  {  289, 280, 255, 218, 175 },
  {  279, 270, 246, 210, 169 },
  {  269, 261, 237, 203, 163 },
  {  259, 251, 229, 196, 157 },
  {  250, 242, 221, 189, 152 },
  {  241, 233, 213, 182, 146 },
  {  232, 225, 205, 175, 141 },
  {  223, 216, 197, 168, 135 },
  {  215, 208, 189, 162, 130 },
  {  206, 200, 182, 156, 125 },
  {  198, 192, 175, 150, 120 },
  {  190, 185, 168, 144, 116 },
  {  183, 177, 161, 138, 111 },
  {  175, 170, 155, 132, 106 },
  {  168, 163, 149, 127, 102 },
  {  161, 156, 142, 122,  98 },
  {  155, 150, 136, 117,  94 },
  {  148, 144, 131, 112,  90 },
  {  142, 137, 125, 107,  86 },
  {  136, 131, 120, 102,  82 },
  {  130, 126, 114,  98,  79 },
  {  124, 120, 109,  94,  75 },
  {  118, 115, 105,  89,  72 },
  {  113, 110, 100,  85,  69 },
  {  108, 105,  95,  81,  65 },
  {  103, 100,  91,  78,  62 },
  {   98,  95,  87,  74,  60 },
  {   94,  91,  83,  71,  57 },
  {   89,  86,  79,  67,  54 },
  {   85,  82,  75,  64,  51 },
  {   81,  78,  71,  61,  49 },
  {   77,  74,  68,  58,  47 },
  {   73,  71,  64,  55,  44 },
  {   69,  67,  61,  52,  42 },
  {   66,  64,  58,  50,  40 },
  {   63,  61,  55,  47,  38 },
  {   59,  58,  52,  45,  36 },
  {   56,  55,  50,  43,  34 },
  {   53,  52,  47,  40,  32 },
  {   51,  49,  45,  38,  31 },
  {   48,  46,  42,  36,  29 },
  {   45,  44,  40,  34,  27 },
  {   43,  42,  38,  32,  26 },
  {   41,  39,  36,  31,  25 },
  {   38,  37,  34,  29,  23 },
  {   36,  35,  32,  27,  22 },
  {   34,  33,  30,  26,  21 },
  {   32,  31,  28,  24,  20 },
  {   30,  30,  27,  23,  18 },
  {   29,  28,  25,  22,  17 },
  {   27,  26,  24,  20,  16 },
  {   25,  25,  22,  19,  15 },
  {   24,  23,  21,  18,  15 },
  {   23,  22,  20,  17,  14 },
  {   21,  21,  19,  16,  13 },
  {   20,  19,  18,  15,  12 },
  {   19,  18,  17,  14,  11 },
  {   18,  17,  16,  13,  11 },
  {   17,  16,  15,  12,  10 },
  {   16,  15,  14,  12,   9 },
  {   15,  14,  13,  11,   9 },
  {   14,  13,  12,  10,   8 },
  {   13,  12,  11,  10,   8 },
  {   12,  12,  11,   9,   7 },
  {   11,  11,  10,   8,   7 },
  {   10,  10,   9,   8,   6 },
  {   10,   9,   9,   7,   6 },
  {    9,   9,   8,   7,   6 },
  {    9,   8,   8,   6,   5 },
  {    8,   8,   7,   6,   5 },
  {    7,   7,   7,   6,   5 },
  {    7,   7,   6,   5,   4 },
  {    6,   6,   6,   5,   4 },
  {    6,   6,   5,   5,   4 },
  {    6,   5,   5,   4,   3 },
  {    5,   5,   5,   4,   3 },
  {    5,   5,   4,   4,   3 },
  {    5,   4,   4,   3,   3 },
  {    4,   4,   4,   3,   3 },
  {    4,   4,   3,   3,   2 },
  {    4,   4,   3,   3,   2 },
  {    3,   3,   3,   3,   2 },
  {    3,   3,   3,   2,   2 },
  {    3,   3,   3,   2,   2 },
  {    3,   3,   2,   2,   2 },
  {    2,   2,   2,   2,   2 },
  {    2,   2,   2,   2,   1 },
  {    2,   2,   2,   2,   1 },
  {    2,   2,   2,   1,   1 },
  {    2,   2,   2,   1,   1 },
  {    2,   2,   1,   1,   1 },
  {    2,   2,   1,   1,   1 },
  {    1,   1,   1,   1,   1 },
  {    1,   1,   1,   1,   1 },
  {    1,   1,   1,   1,   1 },
  {    1,   1,   1,   1,   1 },
  {    1,   1,   1,   1,   1 },
  {    1,   1,   1,   1,   1 },
  {    1,   1,   1,   1,   1 },
  {    1,   1,   1,   1,   0 },
  {    1,   1,   1,   1,   0 },
  {    1,   1,   1,   1,   0 },
  {    1,   1,   1,   0,   0 },
  {    1,   1,   1,   0,   0 },
  {    1,   1,   0,   0,   0 }
};

// ====================================================================================================================
// Constructor / destructor / initialize
// ====================================================================================================================

IntraPrediction::IntraPrediction()
:
  m_currChromaFormat( NUM_CHROMA_FORMAT )
{
  for (UInt ch = 0; ch < MAX_NUM_COMPONENT; ch++)
  {
    for (UInt buf = 0; buf < 2; buf++)
    {
      m_piYuvExt[ch][buf] = nullptr;
      m_piYuvExtMrl[ch][buf] = nullptr;
    }
  }

  m_piTemp = nullptr;

  for (Int i = 0; i < LM_FILTER_NUM; i++)
  {
    m_pLumaRecBufferMul[i] = nullptr;
  }
}

IntraPrediction::~IntraPrediction()
{
  destroy();
}

Void IntraPrediction::destroy()
{
  for (UInt ch = 0; ch < MAX_NUM_COMPONENT; ch++)
  {
    for (UInt buf = 0; buf < NUM_PRED_BUF; buf++)
    {
      delete[] m_piYuvExt[ch][buf];
      m_piYuvExt[ch][buf] = nullptr;
      delete[] m_piYuvExtMrl[ch][buf];
      m_piYuvExtMrl[ch][buf] = nullptr;
    }
  }

  delete[] m_piTemp;
  m_piTemp = nullptr;

  for (Int i = 0; i < LM_FILTER_NUM; i++)
  {
    if (m_pLumaRecBufferMul[i])
    {
      delete[] m_pLumaRecBufferMul[i];
      m_pLumaRecBufferMul[i] = nullptr;
    }
  }
}

Void IntraPrediction::init(ChromaFormat chromaFormatIDC, const unsigned bitDepthY)
{
  // if it has been initialised before, but the chroma format has changed, release the memory and start again.
  if (m_piYuvExt[COMPONENT_Y][PRED_BUF_UNFILTERED] != nullptr && m_currChromaFormat != chromaFormatIDC)
  {
    destroy();
  }

  m_currChromaFormat = chromaFormatIDC;

  if (m_piYuvExt[COMPONENT_Y][PRED_BUF_UNFILTERED] == nullptr) // check if first is null (in which case, nothing initialised yet)
  {
    m_iYuvExtSize = (MAX_CU_SIZE * 2 + 1) * (MAX_CU_SIZE * 2 + 1);
    m_iYuvExtMrlSize = (MAX_CU_SIZE * 2 + 1 + 2 * MAX_MRL_OFFSET) * (MAX_CU_SIZE * 2 + 1 + 2 * MAX_MRL_OFFSET);

    for (UInt ch = 0; ch < MAX_NUM_COMPONENT; ch++)
    {
      for (UInt buf = 0; buf < NUM_PRED_BUF; buf++)
      {
        m_piYuvExt[ch][buf] = new Pel[m_iYuvExtSize];
        m_piYuvExtMrl[ch][buf] = new Pel[m_iYuvExtMrlSize];
      }
    }
  }

  Int shift = bitDepthY + 4;
  for (Int i = 32; i < 64; i++)
  {
    m_auShiftLM[i - 32] = ((1 << shift) + i / 2) / i;
  }

  if (m_piTemp == nullptr)
  {
    const int MMLM_Lines = std::max<int>( 1, MMLM_SAMPLE_NEIGHBOR_LINES );
    m_piTemp = new Pel[( MAX_CU_SIZE + MMLM_Lines ) * ( MAX_CU_SIZE + MMLM_Lines )];
  }

  for (Int i = 0; i < LM_FILTER_NUM; i++)
  {
    if (!m_pLumaRecBufferMul[i])
    {
      m_pLumaRecBufferMul[i] = new Pel[ (MAX_CU_SIZE + MMLM_SAMPLE_NEIGHBOR_LINES) * (MAX_CU_SIZE + MMLM_SAMPLE_NEIGHBOR_LINES) ];
    }
  }
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

// Function for calculating DC value of the reference samples used in Intra prediction
//NOTE: Bit-Limit - 25-bit source
Pel IntraPrediction::xGetPredValDc( const CPelBuf &pSrc, const int mrlOffset, const Size &dstSize )
{
  CHECK( dstSize.width == 0 || dstSize.height == 0, "Empty area provided" );

  Int iInd, iSum = 0;
  Pel pDcVal;
  const int width  = dstSize.width  + mrlOffset;
  const int height = dstSize.height + mrlOffset;

  for( iInd = 0; iInd < width; iInd++ )
  {
    iSum += pSrc.at( 1 + iInd, 0 );
  }
  for( iInd = 0; iInd < height; iInd++ )
  {
    iSum += pSrc.at( 0, 1 + iInd );
  }

  pDcVal = ( iSum + ( ( width + height ) >> 1 ) ) / ( width + height );
  return pDcVal;
}

void IntraPrediction::predIntraAng( const ComponentID compId, PelBuf &piPred, const PredictionUnit &pu, const bool &useFilteredPredSamples, bool isFirst1dPartition )                   
{
  const ComponentID    compID       = MAP_CHROMA( compId );
  const ChannelType    channelType  = toChannelType( compID );
  const Int            iWidth       = piPred.width;
  const Int            iHeight      = piPred.height;
        UInt           uiDirMode    = PU::getFinalIntraMode( pu, channelType );

  // only use extended reference samples for luma
  const int mrlOffset     = ( compID == COMPONENT_Y ) ? pu.cu->mrlIdx : 0;
  const int log2CuHeight  = g_aucLog2[pu.cu->lumaSize().height];
  const int log2CuWidth   = g_aucLog2[pu.cu->lumaSize().width];
  CHECK( ( abs( log2CuHeight - log2CuWidth ) > 1 || log2CuHeight < MRL_FAST_LOG2_MIN_SIZE || log2CuWidth < MRL_FAST_LOG2_MIN_SIZE ) && pu.cu->mrlIdx, "mrlIdx > 0 for partitions with one side more than twice as large than the other, e.g. 16x4!" );

  CHECK( g_aucLog2[iWidth] < 2 && pu.cs->pcv->noChroma2x2, "Size not allowed" );
  CHECK( g_aucLog2[iWidth] > 7, "Size not allowed" );
  CHECK( iWidth != iHeight && !pu.cs->pcv->rectCUs, "Rectangular block are only allowed with QTBT" );

  const Int  srcStride = ( iWidth + iHeight + 1 + 2 * mrlOffset );

  const Bool enableEdgeFilters = !(CU::isRDPCMEnabled( *pu.cu ) && pu.cu->transQuantBypass);
  Pel *ptrSrc = getPredictorPtr( compID, useFilteredPredSamples );
  if( mrlOffset > 0 )
  {
    ptrSrc = getPredictorPtrMrl( compID, useFilteredPredSamples );
  }

  bool useModeWith1dPartitions = canUse1dPartitions( compID ) && pu.cu->mode1dPartitions != NO_1D_PARTITION && ( iWidth == 1 || iHeight == 1 );
  if( useModeWith1dPartitions && !isFirst1dPartition )
  {
    // mirror mode for angular pred.
    switch( CU::select1dPartitionType( *pu.cu, pu, compID ) )
    {
    case( TU_1D_HORZ_SPLIT_REVERSE_ORDER ): uiDirMode = (HOR_IDX << 1) - uiDirMode; break;
    case( TU_1D_VERT_SPLIT_REVERSE_ORDER ): uiDirMode = (VER_IDX << 1) - uiDirMode; break;
    default: break;
    }
  }

  bool pdpcCondition = (((pu.cs->sps->getSpsNext().isIntraPDPC()) && pu.cu->pdpc) || (pu.cs->sps->getSpsNext().isPlanarPDPC() && (uiDirMode == PLANAR_IDX) && (mrlOffset == 0))) && !useModeWith1dPartitions;

  if( pdpcCondition )
  {
    int idxW = std::min( 4, (int)g_aucLog2[iWidth]  - 1 );
    int idxH = std::min( 4, (int)g_aucLog2[iHeight] - 1 );
    if( !pu.cs->pcv->only2Nx2N )
    {
      CHECK( idxW != idxH, "Non-square partitions not supported by this config" );
      if( pu.cu->partSize == SIZE_NxN && idxW == 1 ) { idxW = idxH = 0; }
    }
    const int *pPdpcParWidth;
    const int *pPdpcParHeight;
    if( pu.cs->sps->getSpsNext().isPlanarPDPC() )
    {
      pPdpcParWidth = g_pdpcParam[idxW];
      pPdpcParHeight = g_pdpcParam[idxH];
    }
    else
    {
      pPdpcParWidth = g_pdpc_pred_param[idxW][g_intraMode65to33AngMapping[uiDirMode]];
      pPdpcParHeight = g_pdpc_pred_param[idxH][g_intraMode65to33AngMapping[uiDirMode]];
    }
    const int *pPdpcParMain   = (iWidth < iHeight) ? pPdpcParHeight : pPdpcParWidth;

    const int srcStride  = iWidth + iHeight + 1;
    const int doubleSize = iWidth + iHeight;

    Pel* piRefVector = m_piTempRef + doubleSize;
    Pel* piLowpRefer = m_piFiltRef + doubleSize;

    for( int j = 0; j <= doubleSize; j++ ) { piRefVector[ j] = ptrSrc[j]; }
    for( int i = 1; i <= doubleSize; i++ ) { piRefVector[-i] = ptrSrc[i*srcStride]; }

    if( pPdpcParMain[5] != 0 )
    {
      xReferenceFilter( doubleSize, pPdpcParMain[4], pPdpcParMain[5], piRefVector, piLowpRefer );

      // copy filtered ref. samples back to ref. buffer
      for( int j = 0; j <= doubleSize; j++ ) { ptrSrc[j]           = piLowpRefer[ j]; }
      for( int i = 1; i <= doubleSize; i++ ) { ptrSrc[i*srcStride] = piLowpRefer[-i]; }
    }

    const ClpRng& clpRng( pu.cu->cs->slice->clpRng(compID) );

    switch( uiDirMode )
    {
    case( PLANAR_IDX ): xPredIntraPlanar( CPelBuf( ptrSrc, srcStride, srcStride ), mrlOffset, piPred, *pu.cs->sps );         break;
    case( DC_IDX ):     xPredIntraDc    ( CPelBuf( ptrSrc, srcStride, srcStride ), mrlOffset, piPred, channelType, false );  break; // including DCPredFiltering
    default:            xPredIntraAng   ( CPelBuf( ptrSrc, srcStride, srcStride ), mrlOffset, piPred, channelType,
                                            uiDirMode, clpRng, enableEdgeFilters, *pu.cs->sps, false );                      break;
    }

    if( pPdpcParMain[5] != 0 )
    {
      // copy unfiltered ref. samples back to ref. buffer for weighted prediction
      for( int j = 0; j <= doubleSize; j++ ) { ptrSrc[j]           = piRefVector[ j]; }
      for( int i = 1; i <= doubleSize; i++ ) { ptrSrc[i*srcStride] = piRefVector[-i]; }
    }

    int scale     = (g_aucLog2[iWidth] + g_aucLog2[iHeight] < 10) ? 0 : 1;
    int parShift  = 6; //normalization factor
    int parScale  = 1 << parShift;
    int parOffset = 1 << (parShift - 1);

    for( int y = 0; y < iHeight; y++ )
    {
      int shiftRow     = y >> scale;
      int coeff_Top    = pPdpcParHeight[2] >> shiftRow;
      int coeff_offset = pPdpcParHeight[3] >> shiftRow;

      for( int x = 0; x < iWidth; x++ )
      {
        int shiftCol      = x >> scale;
        int coeff_Left    =  pPdpcParWidth[0] >> shiftCol;
        int coeff_TopLeft = (pPdpcParWidth[1] >> shiftCol) + coeff_offset;
        int coeff_Cur     = parScale - coeff_Left - coeff_Top + coeff_TopLeft;

        int sampleVal = (coeff_Left* piRefVector[-y - 1] + coeff_Top * piRefVector[x + 1] - coeff_TopLeft * piRefVector[0] + coeff_Cur * piPred.at( x, y ) + parOffset) >> parShift;
        piPred.at( x, y ) = ClipPel( sampleVal, clpRng );
      }
    }
  }
  else
  {
    if( uiDirMode == PLANAR_IDX && useModeWith1dPartitions )
    {
      xPredIntraPlanarBasic1D( CPelBuf( ptrSrc, srcStride, srcStride ), piPred, *pu.cs->sps );
    }
    else
    {
      switch( uiDirMode )
      {
      case( PLANAR_IDX ): xPredIntraPlanar( CPelBuf( ptrSrc, srcStride, srcStride ), mrlOffset, piPred, *pu.cs->sps );                           break;
      case( DC_IDX ):     xPredIntraDc    ( CPelBuf( ptrSrc, srcStride, srcStride ), mrlOffset, piPred, channelType, !useModeWith1dPartitions ); break; // including DCPredFiltering
      default:            xPredIntraAng   ( CPelBuf( ptrSrc, srcStride, srcStride ), mrlOffset, piPred, channelType, uiDirMode,
                                            pu.cs->slice->clpRng( compID ), enableEdgeFilters, *pu.cs->sps );                                    break;
      }
    }
  }
}

Void IntraPrediction::predIntraChromaLM(const ComponentID compID, PelBuf &piPred, const PredictionUnit &pu, const CompArea& chromaArea, Int intraDir)
{
  bool DO_ELM = false;
  if (pu.cs->sps->getSpsNext().isELMModeMMLM())
  {
    if (intraDir == MMLM_CHROMA_IDX)
    {
      DO_ELM = true;
    }
    if (pu.cs->sps->getSpsNext().isELMModeMFLM() && (intraDir >= LM_CHROMA_F1_IDX && intraDir < (LM_CHROMA_F1_IDX + LM_FILTER_NUM)))
    {
      DO_ELM = true;
    }
  }
  int MMLM_Lines = pu.cs->sps->getSpsNext().isELMModeMMLM() ? 2 : 1;
  if(DO_ELM)
  {
    Pel *pLumaSaved=0;
    if (pu.cs->sps->getSpsNext().isELMModeMFLM())
    {
      pLumaSaved = m_piTemp;

      if (intraDir >= LM_CHROMA_F1_IDX && intraDir < (LM_CHROMA_F1_IDX + LM_FILTER_NUM))
      {
        Int iLumaIdx = intraDir - LM_CHROMA_F1_IDX;
        m_piTemp = m_pLumaRecBufferMul[iLumaIdx];
      }
    }
    // LLS parameters estimation -->
    MMLM_parameter parameters[2];
    Int iGroupNum = 2;
    xGetMMLMParameters(pu, compID, chromaArea, iGroupNum, parameters);

    Int  iLumaStride = MAX_CU_SIZE + MMLM_Lines; //MMLM_SAMPLE_NEIGHBOR_LINES;
    PelBuf Temp = PelBuf(m_piTemp + (iLumaStride + 1) *MMLM_Lines, iLumaStride, Size(chromaArea));//MMLM_SAMPLE_NEIGHBOR_LINES;
    Pel  *pLuma = Temp.bufAt(0, 0);

    Pel*  pPred = piPred.bufAt(0, 0);
    Int  uiPredStride = piPred.stride;

    UInt uiCWidth = chromaArea.width;
    UInt uiCHeight = chromaArea.height;

    for (Int i = 0; i < uiCHeight; i++)
    {
      for (Int j = 0; j < uiCWidth; j++)
      {
        Int a, b, iShift;
        if (pLuma[j] <= parameters[0].Sup)
        {
          a = parameters[0].a;
          b = parameters[0].b;
          iShift = parameters[0].shift;
        }
        else
        {
          a = parameters[1].a;
          b = parameters[1].b;
          iShift = parameters[1].shift;
        }

        pPred[j] = ( Pel ) ClipPel( ( ( a * pLuma[j] ) >> iShift ) + b, pu.cs->slice->clpRng( compID ) );
      }

      pPred += uiPredStride;
      pLuma += iLumaStride;
    }

    if (pu.cs->sps->getSpsNext().isELMModeMFLM())
    {
      m_piTemp = pLumaSaved;
    }
  }
  else
  {
    Int  iLumaStride = 0;
    PelBuf Temp;
    if (pu.cs->sps->getSpsNext().isELMModeMMLM())
    {
      iLumaStride = MAX_CU_SIZE + MMLM_Lines; //MMLM_SAMPLE_NEIGHBOR_LINES;
      Temp = PelBuf(m_piTemp + (iLumaStride + 1) *MMLM_Lines, iLumaStride, Size(chromaArea)); //MMLM_SAMPLE_NEIGHBOR_LINES;
    }
    else
    {
      iLumaStride = MAX_CU_SIZE + 1;
      Temp = PelBuf(m_piTemp + (iLumaStride + 1) * 1, iLumaStride, Size(chromaArea));
    }

    Int a, b, iShift;
    Int iPredType = 0;
    xGetLMParameters(pu, compID, chromaArea, iPredType, a, b, iShift);

    ////// final prediction
    piPred.copyFrom(Temp);
    piPred.linearTransform(a, iShift, b, true, pu.cs->slice->clpRng(compID));
  }
}

Void IntraPrediction::addCrossColorResi(const ComponentID compID, PelBuf &piPred, const TransformUnit &tu, const CPelBuf &pResiCb)
{
  const CompArea& chromaArea = tu.block(compID);

  Int a, b, iShift;

  const PredictionUnit& pu = *(tu.cs->getPU(chromaArea.pos(), toChannelType(compID)));
  xGetLMParameters(pu, compID, chromaArea, Int(1), a, b, iShift);

  Int offset = 1 << (iShift - 1);

  if (a >= 0)
  {
    return;
  }

  const ClpRng& clpRng( tu.cu->cs->slice->clpRng(compID) );

  Pel*  pPred = piPred.buf;
  const Pel*  pResi = pResiCb.bufAt(0, 0);
  UInt uiPredStride = piPred.stride;
  UInt uiResiStride = pResiCb.stride;


  for (UInt uiY = 0; uiY < chromaArea.height; uiY++)
  {
    for (UInt uiX = 0; uiX < chromaArea.width; uiX++)
    {
      pPred[uiX] = ClipPel( pPred[uiX] + ((pResi[uiX] * a + offset) >> iShift), clpRng);
    }
    pPred += uiPredStride;
    pResi += uiResiStride;
  }
}


Void IntraPrediction::xFilterGroup(Pel* pMulDst[], Int i, Pel const * const piSrc, Int iRecStride, Bool bAboveAvaillable, Bool bLeftAvaillable)
{
  pMulDst[0][i] = (piSrc[1] + piSrc[iRecStride + 1] + 1) >> 1;

  pMulDst[1][i] = (piSrc[iRecStride] + piSrc[iRecStride + 1] + 1) >> 1;

  pMulDst[3][i] = (piSrc[0] + piSrc[1] + 1) >> 1;

  pMulDst[2][i] = (piSrc[0] + piSrc[1] + piSrc[iRecStride] + piSrc[iRecStride + 1] + 2) >> 2;

}



/** Function for deriving planar intra prediction. This function derives the prediction samples for planar mode (intra coding).
 */

//NOTE: Bit-Limit - 24-bit source
Void IntraPrediction::xPredIntraPlanar( const CPelBuf &pSrc, const int mrlOffset, PelBuf &pDst, const SPS& sps )
{
  const UInt width = pDst.width;
  const UInt height = pDst.height;

  Int leftColumn[MAX_CU_SIZE + 1], topRow[MAX_CU_SIZE + 1], bottomRow[MAX_CU_SIZE], rightColumn[MAX_CU_SIZE];
  const UInt offset = width * height;

  // Get left and above reference column and row
  for( Int k = 0; k < width + 1; k++ )
  {
    topRow[k] = pSrc.at( k + 1 + mrlOffset, 0 );
  }

  for( Int k = 0; k < height + 1; k++ )
  {
    leftColumn[k] = pSrc.at( 0, k + 1 + mrlOffset );
  }

  // Prepare intermediate variables used in interpolation
  Int bottomLeft = leftColumn[height];
  Int topRight = topRow[width];

  for( Int k = 0; k < width; k++ )
  {
    bottomRow[k] = bottomLeft - topRow[k];
    topRow[k]    = topRow[k] * height;
    //topRow[k] <<= shift1Dver;
  }

  for( Int k = 0; k < height; k++ )
  {
    rightColumn[k] = topRight - leftColumn[k];
    leftColumn[k]  = leftColumn[k] * width;
    //leftColumn[k] <<= shift1Dhor;
  }

  for( Int y = 0; y < height; y++ )
  {
    Int horPred = leftColumn[y];

    for( Int x = 0; x < width; x++ )
    {
      horPred += rightColumn[y];
      topRow[x] += bottomRow[x];

      Int vertPred = topRow[x];
      pDst.at( x, y ) = ( ( horPred * height ) + ( vertPred * width ) + offset ) / ( width * height * 2 );
    }
  }
}

//Fast Template Matching Intra Prediction
Void IntraPrediction::predIntraFastTM( PelBuf &pDst, const PredictionUnit &pu, const ComponentID &compID, UInt dstX, UInt dstY )
{
  // FTM available only for luma
  CHECK( compID != COMPONENT_Y,                                           "Invalid FTM mode" );
  if( !pu.cu->mode1dPartitions )
  {
    CHECK( ( ( pDst.width < 4 ) || ( pDst.width % 4 == 1 ) ),             "Invalid width for FTM" ); // code optimised for width with multiple of 4
  }

  // FTM does not have PDPC, EMT and NSST
  CHECK( ( pu.cs->sps->getSpsNext().getUseIntraPDPC() && pu.cu->pdpc ),   "Invalid TM_PDPC Mode" );
  CHECK( ( pu.cs->sps->getSpsNext().getUseIntraEMT() && pu.cu->emtFlag ), "Invalid TM_EMT Mode" );
  CHECK( ( pu.cs->sps->getSpsNext().getUseNSST() && pu.cu->nsstIdx ),     "Invalid TM_NSST Mode" );
  CHECK( ( pu.cs->sps->getSpsNext().getUseIntra_NN() && pu.cu->intra_NN ), "TM and intra NN are incompatible." );
  CHECK( pu.cu->diffFilterIdx, "Invalid TM_PDE Mode" );

  const Area puArea = Area(dstX, dstY, pDst.width, pDst.height);

  Bool TMSearch = xCheckFTMSearch(pu, puArea); // Checks if there is enough area for a search
  if (TMSearch == true)
  {
    FTMInfo Info;
    xInitializeFTMInfo(puArea, Info);

    Pel* pRec = pu.cs->picture->getRecoBuf().bufs[compID].buf; //start of the Rec picture for the current compID
    const Int  DstStride = pu.cs->picture->getRecoBuf().bufs[compID].stride;
    Pel* temp = pRec; //Points to start of the picture buffer
    Pel* Test = temp + (((Info.Border.uiTPelY - Info.Border.TemplateSize)* (Int)DstStride) + (Info.Border.uiLPelX - Info.Border.TemplateSize)); // Points to the Template
    //Redefining the search area based on the position of the current block w.r.t CTU position
    xFTMRedefineBoundary(pu, Info.Border, false);

    UInt uiIntraTMMode = pu.FTMRegIdx;

    //Obtain the start values
    xObtainFTMstartInfo(pu, Info.Border, uiIntraTMMode);
    if ((Info.Border.endX2 < Info.Border.startX) || (Info.Border.condX - 1 < Info.Border.startY)) //FTM impossible in this case
    {
      pDst.copyFrom (pu.cs->picture->getRecoBuf (CompArea (compID, pu.chromaFormat, Info.Pred[0].x, Info.Pred[0].y, pDst.width, pDst.height)));
      return;
    }
    temp += ((Info.Border.condX - 1)*DstStride + Info.Border.endX2); //start of the search

    UInt RegType = ((uiIntraTMMode == FTM_REG1) ? FTM_HORVER : ((uiIntraTMMode == FTM_REG2 || uiIntraTMMode == FTM_REG4) ? FTM_HOR : FTM_VER));
    xPredIntraFTMReg(pu, compID, Info, Test, temp, DstStride, RegType, ((uiIntraTMMode > FTM_REG3) ? pu.cs->sps->getSpsNext().getFTMDownSample() : false));


    // FTM Prediction
    xFTMAdaptivePrediction(pDst, pu, compID, Info.Pred);
  }
  else
  {
    // TM Search not possible in this case
    // Random Prediction
    xFTMRandomPrediction(pDst, pu, compID);
  }
}

Void IntraPrediction::xInitializeFTMInfo(const Area &puArea, FTMInfo &Info)
{
  Info.Border.uiLPelX = puArea.x;
  Info.Border.uiTPelY = puArea.y;
  Info.Border.TemplateSize = FTM_RATE;
  Info.Border.Tem_Height = puArea.height + Info.Border.TemplateSize;
  Info.Border.Tem_Width = puArea.width + Info.Border.TemplateSize;
  Info.Border.startX = 0;
  Info.Border.startY = 0;
  Info.Border.endY = Info.Border.uiTPelY - Info.Border.TemplateSize;
  Info.Border.endX1 = Info.Border.uiLPelX - Info.Border.TemplateSize;
  Info.Border.endX2 = Info.Border.uiLPelX - Info.Border.Tem_Width;
  Info.Border.condX = Info.Border.uiTPelY - Info.Border.Tem_Height + 1;
  Info.Pred.resize(3);
  Info.Pred[0] = { 0, 0, MAX_UINT - 2 };
  Info.Pred[1] = { 0, 0, MAX_UINT - 1 };
  Info.Pred[2] = { 0, 0, MAX_UINT - 0 };
}

Void IntraPrediction::xObtainFTMstartInfo(const PredictionUnit &pu, FTMBorderInfo &Border, UInt uiIntraTMMode)
{
  CHECK(((Border.condX - 1) < 0 || Border.endX2 < 0), "FTM Error: Search region  unavailable");

  UInt toCheck = 0, Checked = 0;
  if (uiIntraTMMode == FTM_REG1)
  {
    toCheck = pu.cs->sps->getSpsNext().getFTMAreaSize1(); // Search region size for Reg 1
  }
  else
  {
    Checked = pu.cs->sps->getSpsNext().getFTMAreaSize1(); // Search region size for Reg 1
    if (uiIntraTMMode > FTM_REG3)
    {
      UInt CheckedArea2 = pu.cs->sps->getSpsNext().getFTMAreaSize2(); // Search region size for Reg 2,3
      Checked += CheckedArea2;
      toCheck = pu.cs->sps->getSpsNext().getFTMAreaSize3(); // Search region size for Reg 4,5
    }
    else
    {
      toCheck = pu.cs->sps->getSpsNext().getFTMAreaSize2(); // Search region size for Reg 2,3
    }
    Border.condX -= Checked;
    Border.condX--; //shifting to the start of the search, excluding searched area
    if (Border.condX < 0)
    {
      Border.condX = 0;
    }
    Border.endX2 -= Checked;
    Border.endX2--; //shifting to the start of the search, excluding searched area
    if (Border.endX2 < 0)
    {
      Border.endX2 = 0;
    }
  }
  Border.startX = Border.endX2 - toCheck;
  Border.startY = Border.condX - 1 - toCheck;
  if (Border.startX < 0)
  {
    Border.startX = 0;
  }
  if (Border.startY < 0)
  {
    Border.startY = 0;
  }
}

Void IntraPrediction::xFTMRandomPrediction( PelBuf &pDst, const PredictionUnit &pu, const ComponentID &compID )
{
  ChannelType channelType = toChannelType( compID );
  const Int iDCValue      = 1 << ( pu.cu->cs->sps->getBitDepth( channelType ) - 1 );

  pDst.fill( iDCValue );
}


Void IntraPrediction::xFTMAdaptivePrediction(PelBuf &pDst, const PredictionUnit &pu, const ComponentID &compID, FTMPredInfo &Pred)
{
  UInt Thres = (Pred[0].ssd << 1); // 2 times the First Predictor error
  if (Pred[1].ssd <= Thres && Pred[2].ssd <= Thres)
  {
    xFTM3PredWeightedAverage(pDst, pu, compID, Pred);
  }
  else
    if (Pred[1].ssd <= Thres)
    {
      xFTM2PredAverage(pDst, pu, compID, Pred);
    }
    else
    {
      xFTM1PredDirect(pDst, pu, compID, Pred);
    }
}

Void IntraPrediction::xFTM3PredWeightedAverage(PelBuf &pDst, const PredictionUnit &pu, const ComponentID &compID, FTMPredInfo &Pred)
{
  const Int width = pDst.width;
  const Int height = pDst.height;

  const CPelBuf buf1 = pu.cs->picture->getRecoBuf(CompArea(compID, pu.chromaFormat, Pred[0].x, Pred[0].y, width, height));
  const CPelBuf buf2 = pu.cs->picture->getRecoBuf(CompArea(compID, pu.chromaFormat, Pred[1].x, Pred[1].y, width, height));
  const CPelBuf buf3 = pu.cs->picture->getRecoBuf(CompArea(compID, pu.chromaFormat, Pred[2].x, Pred[2].y, width, height));

  // Prediction
  if (pu.cu->mode1dPartitions)
  {
    for (int y = 0; y < height; y++)
    {
      for (int x = 0; x < width; x++)
      {
        pDst.at(x, y) = (Pel)(((buf1.at(x, y) << 1) + buf2.at(x, y) + buf3.at(x, y)) >> 2);
      }
    }
  }
  else
  {
    for (int y = 0; y < height; y++)
    {
      for (int x = 0; x < width; x += 4)
      {
        pDst.at(x + 0, y) = (Pel)(((buf1.at(x + 0, y) << 1) + buf2.at(x + 0, y) + buf3.at(x + 0, y)) >> 2);
        pDst.at(x + 1, y) = (Pel)(((buf1.at(x + 1, y) << 1) + buf2.at(x + 1, y) + buf3.at(x + 1, y)) >> 2);
        pDst.at(x + 2, y) = (Pel)(((buf1.at(x + 2, y) << 1) + buf2.at(x + 2, y) + buf3.at(x + 2, y)) >> 2);
        pDst.at(x + 3, y) = (Pel)(((buf1.at(x + 3, y) << 1) + buf2.at(x + 3, y) + buf3.at(x + 3, y)) >> 2);
      }
    }
  }
}

Void IntraPrediction::xFTM2PredAverage(PelBuf &pDst, const PredictionUnit &pu, const ComponentID &compID, FTMPredInfo &Pred)
{
  const Int width = pDst.width;
  const Int height = pDst.height;

  const CPelBuf buf1 = pu.cs->picture->getRecoBuf(CompArea(compID, pu.chromaFormat, Pred[0].x, Pred[0].y, width, height));
  const CPelBuf buf2 = pu.cs->picture->getRecoBuf(CompArea(compID, pu.chromaFormat, Pred[1].x, Pred[1].y, width, height));

  // Prediction
  if (pu.cu->mode1dPartitions)
  {
    for (int y = 0; y < height; y++)
    {
      for (int x = 0; x < width; x++)
      {
        pDst.at(x, y) = (Pel)((buf1.at(x, y) + buf2.at(x, y)) >> 1);
      }
    }
  }
  else
  {
    for (int y = 0; y < height; y++)
    {
      for (int x = 0; x < width; x += 4)
      {
        pDst.at(x + 0, y) = (Pel)((buf1.at(x + 0, y) + buf2.at(x + 0, y)) >> 1);
        pDst.at(x + 1, y) = (Pel)((buf1.at(x + 1, y) + buf2.at(x + 1, y)) >> 1);
        pDst.at(x + 2, y) = (Pel)((buf1.at(x + 2, y) + buf2.at(x + 2, y)) >> 1);
        pDst.at(x + 3, y) = (Pel)((buf1.at(x + 3, y) + buf2.at(x + 3, y)) >> 1);
      }
    }
  }
}

Void IntraPrediction::xFTM1PredDirect(PelBuf &pDst, const PredictionUnit &pu, const ComponentID &compID, FTMPredInfo &Pred)
{
  const Int width = pDst.width;
  const Int height = pDst.height;

  const CPelBuf buf1 = pu.cs->picture->getRecoBuf(CompArea(compID, pu.chromaFormat, Pred[0].x, Pred[0].y, width, height));

  // Prediction
  if (pu.cu->mode1dPartitions)
  {
    for (int y = 0; y < height; y++)
    {
      for (int x = 0; x < width; x++)
      {
        pDst.at(x, y) = (Pel)(buf1.at(x, y));
      }
    }
  }
  else
  {
    for (int y = 0; y < height; y++)
    {
      for (int x = 0; x < width; x += 4)
      {
        pDst.at(x + 0, y) = (Pel)(buf1.at(x + 0, y));
        pDst.at(x + 1, y) = (Pel)(buf1.at(x + 1, y));
        pDst.at(x + 2, y) = (Pel)(buf1.at(x + 2, y));
        pDst.at(x + 3, y) = (Pel)(buf1.at(x + 3, y));
      }
    }
  }
}

Void IntraPrediction::xFTMRedefineBoundary(const PredictionUnit &pu, FTMBorderInfo &Border, Bool InLoop, Int LoopX, Int LoopY)
{
  const UInt TMmode         = pu.FTMRegIdx;

  //Redefining the search area based on the position of the current block w.r.t CTU position
  CodingStructure &cs       = *pu.cs; // This is the CU
  PelUnitBuf  pRecPicTM     = cs.picture->getRecoBuf();   // start of the Rec picture buffer with luma and chroma
  const unsigned OrgWidth   = pRecPicTM.bufs[COMPONENT_Y].width; // width of the picture
  const unsigned OrgHeight  = pRecPicTM.bufs[COMPONENT_Y].height; // height of the picture

  Position pos(Border.uiLPelX & ~cs.pcv->maxCUWidthMask, Border.uiTPelY & ~cs.pcv->maxCUHeightMask );
  const UnitArea ctuArea( cs.area.chromaFormat, Area( pos.x, pos.y, cs.pcv->maxCUWidth, cs.pcv->maxCUHeight ) );
  const UInt CTU_Width      = ctuArea.lumaSize().width;
  const UInt CTU_Height     = ctuArea.lumaSize().height;
  const UInt uiXCtu         = ctuArea.lumaPos().x;
  const UInt uiYCtu         = ctuArea.lumaPos().y;
  const UInt XDiff          = Border.uiLPelX - uiXCtu;
  const UInt YDiff          = Border.uiTPelY - uiYCtu;
  const UInt XCtuLimit      = CTU_Width  * ( OrgWidth  / CTU_Width  );
  const UInt YCtuLimit      = CTU_Height * ( OrgHeight / CTU_Height );

  if( InLoop )
  {
    if( TMmode == 2 || TMmode == 4 )
    {
      if( uiXCtu != 0 && uiYCtu != 0 && uiXCtu < XCtuLimit ) // extra area along the width
      {
        if( XDiff != 0 && YDiff != 0 )
        {
          if( LoopY < ( uiYCtu - Border.Tem_Height ) )
          {
            Border.endX1 = uiXCtu + CTU_Height - Border.Tem_Width;
          }
        }
      }
    }
    else if( TMmode == 3 || TMmode == 5 )
    {
      if( uiXCtu != 0 && uiYCtu != 0 && uiYCtu < YCtuLimit ) // extra area down the height
      {
        if( XDiff != 0 && YDiff != 0 )
        {
          if( LoopX < ( uiXCtu - Border.Tem_Width ) )
          {
            Border.endY = uiYCtu + CTU_Width - Border.Tem_Height;
          }
        }
      }
    }
    else
    {
      if( uiXCtu != 0 && uiYCtu != 0 && uiXCtu < XCtuLimit && uiYCtu < YCtuLimit )
      {
        if( XDiff != 0 && YDiff != 0 )
        {
          if( LoopY < ( uiYCtu - Border.Tem_Height ) )
          {
            Border.endX1 = uiXCtu + CTU_Height - Border.Tem_Width;
          }
          if( LoopX < ( uiXCtu - Border.Tem_Width ) )
          {
            Border.endY = uiYCtu + CTU_Width - Border.Tem_Height;
          }
        }
      }
      else if( uiXCtu != 0 && uiYCtu != 0 && uiXCtu < XCtuLimit ) // extra area along the width
      {
        if( XDiff != 0 && YDiff != 0 )
        {
          if( LoopY < ( uiYCtu - Border.Tem_Height ) )
          {
            Border.endX1 = uiXCtu + CTU_Height - Border.Tem_Width;
          }
        }
      }
      else if( uiXCtu != 0 && uiYCtu != 0 && uiYCtu < YCtuLimit ) // extra area down the height
      {
        if( XDiff != 0 && YDiff != 0 )
        {
          if( LoopX < ( uiXCtu - Border.Tem_Width ) )
          {
            Border.endY = uiYCtu + CTU_Width - Border.Tem_Height;
          }
        }
      }
      else
      {
      }
    }
  }
  else
  {
    //InLoop == false
    if( TMmode == 2 || TMmode == 4 )
    {
      if( uiXCtu != 0 && uiYCtu != 0 && uiXCtu < XCtuLimit && uiYCtu < YCtuLimit )
      {
        if( XDiff == 0 && YDiff == 0 )
        {
          Border.endX1 = uiXCtu + CTU_Width - Border.Tem_Width;
        }
        else if( YDiff == 0 && XDiff != 0 )
        {
          CHECK( uiYCtu == 0, "Invalid uiYCtu" );
          Border.endX1 = uiXCtu + CTU_Width - Border.Tem_Width;
        }
        else
        {
          ; // Redefined in 'InLoop'
        }
      }
      else if( uiXCtu != 0 && uiYCtu != 0 && uiXCtu < XCtuLimit ) // extra area along the width
      {
        if( YDiff == 0 )
        {
          Border.endX1 = uiXCtu + CTU_Width - Border.Tem_Width;
        }
        else
        {
          ; // Redefined in 'InLoop'
        }
      }
      else
      {
        ;
      }
    }
    else if( TMmode == 3 || TMmode == 5 )
    {
      if( uiXCtu != 0 && uiYCtu != 0 && uiXCtu < XCtuLimit && uiYCtu < YCtuLimit )
      {
        if( XDiff == 0 && YDiff == 0 )
        {
          Border.endY = uiYCtu + CTU_Height - Border.Tem_Height;
        }
        else if( XDiff == 0 && YDiff != 0 )
        {
          CHECK( uiXCtu == 0, "Invalid uiXCtu" );
          Border.endY = uiYCtu + CTU_Height - Border.Tem_Height;
        }
        else
        {
          ; // Redefined in 'InLoop'
        }
      }
      else if( uiXCtu != 0 && uiYCtu != 0 && uiYCtu < YCtuLimit ) // extra area down the height
      {
        if( XDiff == 0 )
        {
          Border.endY = uiYCtu + CTU_Height - Border.Tem_Height;
        }
        else
        {
          ; // Redefined in 'InLoop'
        }
      }
      else
      {
        ;
      }
    }
    else
    {
      if( uiXCtu != 0 && uiYCtu != 0 && uiXCtu < XCtuLimit && uiYCtu < YCtuLimit )
      {
        if( XDiff == 0 && YDiff == 0 )
        {
          Border.endX1 = uiXCtu + CTU_Width - Border.Tem_Width;
          Border.endY = uiYCtu + CTU_Height - Border.Tem_Height;
        }
        else if( YDiff == 0 && XDiff != 0 )
        {
          CHECK( uiYCtu == 0, "Invalid uiYCtu" );
          Border.endX1 = uiXCtu + CTU_Width - Border.Tem_Width;
        }
        else if( XDiff == 0 && YDiff != 0 )
        {
          CHECK( uiXCtu == 0, "Invalid uiXCtu" );
          Border.endY = uiYCtu + CTU_Height - Border.Tem_Height;
        }
        else
        {
          ; // Redefined in 'InLoop'
        }
      }
      else if( uiXCtu != 0 && uiYCtu != 0 && uiXCtu < XCtuLimit ) // extra area along the width
      {
        if( YDiff == 0 )
        {
          Border.endX1 = uiXCtu + CTU_Width - Border.Tem_Width;
        }
        else
        {
          ; // Redefined in 'InLoop'
        }
      }
      else if( uiXCtu != 0 && uiYCtu != 0 && uiYCtu < YCtuLimit ) // extra area down the height
      {
        if( XDiff == 0 )
        {
          Border.endY = uiYCtu + CTU_Height - Border.Tem_Height;
        }
        else
        {
          ; // Redefined in 'InLoop'
        }
      }
      else
      {
        ;
      }
    }
  }
}

Void IntraPrediction::xFTMLoop( FTMInfo &Info, const Int DstStride, Int LoopX, Int LoopY, Int end, const bool isLoopX, const Pel* Test, const Pel* tempCpy, const int sampling )
{
  const bool remainingWidthIsMultipleOfFour = ( ( Info.Border.Tem_Width - 2 ) & 3 ) == 0;
  unsigned Sum = 0;
  int      d   = 0;

  for( int j = isLoopX ? LoopX : LoopY + 1; j <= end; j += sampling )
  {
    Sum = 0;
    const Pel* TestCpy    = Test;
    const Pel* CpytempCpy = tempCpy;

    for( unsigned y = 0; y < Info.Border.Tem_Height; y++ )
    {
      d = CpytempCpy[0] - TestCpy[0]; Sum += d * d;
      d = CpytempCpy[1] - TestCpy[1]; Sum += d * d;

      if( y < Info.Border.TemplateSize )
      {
        if( remainingWidthIsMultipleOfFour )
        {
          for( unsigned x2 = 2; x2 < Info.Border.Tem_Width; x2 += 4 )
          {
            d = CpytempCpy[x2    ] - TestCpy[x2    ]; Sum += d * d;
            d = CpytempCpy[x2 + 1] - TestCpy[x2 + 1]; Sum += d * d;
            d = CpytempCpy[x2 + 2] - TestCpy[x2 + 2]; Sum += d * d;
            d = CpytempCpy[x2 + 3] - TestCpy[x2 + 3]; Sum += d * d;

            if( Sum > Info.Pred[2].ssd ) break; // Comparing with ThirdBest, in order to skip unnecessary search
          }
        }
        else
        {
          for( unsigned x2 = 2; x2 < Info.Border.Tem_Width; x2++ )
          {
            d = CpytempCpy[x2] - TestCpy[x2]; Sum += d * d;
          }
        }
      }

      CpytempCpy += DstStride;
      TestCpy    += DstStride;

      if( Sum > Info.Pred[2].ssd ) break; // Comparing with ThirdBest, in order to skip unnecessary search
    }

    int insertAt = 3;

    if( Sum < Info.Pred[0].ssd )
    {
      insertAt = 0;
    }
    else if( Sum < Info.Pred[1].ssd )
    {
      insertAt = 1;
    }
    else if( Sum <= Info.Pred[2].ssd )
    {
      insertAt = 2;
    }

    if( insertAt < 3 )
    {
      Info.Pred.pop_back();
      Info.Pred.insert( Info.Pred.begin() + insertAt, isLoopX ? FTMPredCand{ ( unsigned ) j + Info.Border.TemplateSize, ( unsigned ) LoopY + Info.Border.TemplateSize, Sum } : FTMPredCand{ ( unsigned ) LoopX + Info.Border.TemplateSize, ( unsigned ) j + Info.Border.TemplateSize, Sum } );
    }

    tempCpy += sampling * ( isLoopX ? 1 : DstStride );
  }
}

Bool IntraPrediction::xCheckFTMSearch( const PredictionUnit &pu, const Area &searchArea )
{
  // Checks if there is enough area for TM search
  const UChar TemplateSize = FTM_RATE;

  Int endX2 = searchArea.x - searchArea.width  - TemplateSize;
  Int condX = searchArea.y - searchArea.height - TemplateSize + 1;

  return !( ( condX - 1 ) < 0 || endX2 < 0 );
}

Void IntraPrediction::xPredIntraFTMReg( const PredictionUnit &pu, const ComponentID &compID, FTMInfo &Info, Pel* Test, Pel* temp, const Int DstStride, UInt RegType, Bool DownSample )
{
  UInt SUMTest = 0;
  Pel* TestCpyThres = Test;
  for (UInt m = 0; m < Info.Border.Tem_Height; m++)
  {
    for (UInt n = 0; (m >= Info.Border.TemplateSize ? n < Info.Border.TemplateSize : n < Info.Border.Tem_Width); n++)
    {
      SUMTest += (TestCpyThres[n]);
    }
    TestCpyThres += DstStride;
  }
  if (SUMTest == 0)
  {
    SUMTest = 1; //Denominator of Per cannot be zero
  }
  UInt count = 0;
  Int LoopX = Info.Border.endX2, LoopY = Info.Border.condX - 1;
  while (LoopX >= Info.Border.startX && LoopY >= Info.Border.startY)
  {
    Pel *tempCpyX = temp, *tempCpyY = temp + DstStride;
    //Redefining the search area based on the position of the current block w.r.t CTU position
    xFTMRedefineBoundary(pu, Info.Border, true, LoopX, LoopY);
    switch (RegType)
    {
    case 1:
      // uiIntraTMMode == FTM_REG1, hence no downsampling
      xFTMLoop(Info, DstStride, LoopX, LoopY, Info.Border.endX1, true, Test, tempCpyX); // TMSearchLoop Horizontal
      xFTMLoop(Info, DstStride, LoopX, LoopY, Info.Border.endY, false, Test, tempCpyY); // TMSearchLoop Vertical
      break;
    case 2:
      xFTMLoop(Info, DstStride, LoopX, LoopY, Info.Border.endX1, true, Test, tempCpyX, DownSample ? FTM_RATE : 1); // TMSearchLoop Horizontal
      break;
    case 3:
      xFTMLoop(Info, DstStride, LoopX, LoopY, Info.Border.endY, false, Test, tempCpyY, DownSample ? FTM_RATE : 1); // TMSearchLoop Vertical
      break;
    default:
      THROW("Invalid FTM Reg type");
      break;
    }
    LoopX--;
    LoopY--;
    temp -= (DstStride + 1);
    if (pu.cs->sps->getSpsNext().getFTMMode() == 1)
    {
      count++;
      if (LoopX >= Info.Border.startX && LoopY >= Info.Border.startY && count >= FTM_COUNT4)
      {
        double Per = ((Info.Pred[2].ssd + SUMTest) / SUMTest);
        if (Per <= FTM_LOW_BOUND || Per > FTM_HIGH_BOUND) 
        {
          LoopX--;
          LoopY--;
          temp -= (DstStride + 1);
        }
      }
    }
  }
  // leftover lines from the while loop
  if (RegType == FTM_HORVER || RegType == FTM_VER)
  {
    if (LoopX >= Info.Border.startX)
    {
      CHECK((LoopY < (-1)), "FTM Error: Invalid LoopY value");
      temp += DstStride;

      while (LoopX >= Info.Border.startX)
      {
        Pel *tempCpyY = temp;
        xFTMRedefineBoundary(pu, Info.Border, true, LoopX, LoopY);
        xFTMLoop(Info, DstStride, LoopX, LoopY, Info.Border.endY, false, Test, tempCpyY, DownSample ? FTM_RATE : 1);
        LoopX--;
        temp--;
      }
    }
  }
  if (RegType == FTM_HORVER || RegType == FTM_HOR)
  {
    if (LoopY >= Info.Border.startY)
    {
      CHECK((LoopX < (-1)), "FTM Error: Invalid LoopX value");
      temp++;
      LoopX++;
      while (LoopY >= Info.Border.startY)
      {
        Pel *tempCpyX = temp;
        xFTMRedefineBoundary(pu, Info.Border, true, LoopX, LoopY);
        xFTMLoop(Info, DstStride, LoopX, LoopY, Info.Border.endX1, true, Test, tempCpyX, DownSample ? FTM_RATE : 1);
        LoopY--;
        temp -= DstStride;
      }
    }
  }
}

#define PLANAR_PRED_1D_GBS_BUG_FIX ( 1 && HHI_PRIVATE )

Void IntraPrediction::xPredIntraPlanarBasic1D( const CPelBuf &pSrc, PelBuf &pDst, const SPS& sps )
{
  const UInt width  = pDst.width;
  const UInt height = pDst.height;

  Int leftColumn[MAX_CU_SIZE + 1], topRow[MAX_CU_SIZE + 1];

  // Get left and above reference column and row
  for (Int k = 0; k < width + 1; k++)
  {
    topRow[k] = pSrc.at(k + 1, 0);
  }

  for (Int k = 0; k < height + 1; k++)
  {
    leftColumn[k] = pSrc.at(0, k + 1);
  }

  // Prepare intermediate variables used in interpolation
  Int bottomLeft = leftColumn[height];
  Int topRight = topRow[width];

  bool tuIsDividedInRows = height == 1;

  int horPred, verPred, nintSumValue;

  if (tuIsDividedInRows)
  {
    nintSumValue = width << 1;

    for (int x = 0; x < width; x++)
    {
      horPred = (width - 1 - x)*leftColumn[0] + (x + 1)*topRow[width];
      horPred = horPred << 1;
      verPred = bottomLeft + topRow[x];
      verPred = verPred*width;
      pDst.at(x, 0) = (horPred + verPred + nintSumValue)/(width<<2);
    }
  }
  else
  {
    nintSumValue = height << 1;

    for (int y = 0; y < height; y++)
    {
      horPred = topRight + leftColumn[y];
      horPred = horPred*height;
      verPred = (height - 1 - y)*topRow[0] + (y + 1)*leftColumn[height];
      verPred = verPred << 1;
      pDst.at(0, y) = (horPred + verPred + nintSumValue)/(height << 2);
    }
  }
}


void IntraPrediction::jointPreparationForNNIntraPrediction( const PredictionUnit &pu )
{
  if( !need_recalculation_for_NN( pu ) ) { return; }

  const int bitDepth = pu.cu->slice->getSPS()->getBitDepth( CHANNEL_TYPE_LUMA );
  const Area& picArea = pu.cs->picture->Y();
  std::function<bool( Position )> available( [ &picArea, &pu ]( Position p ) -> bool { return picArea.contains( p ) && pu.cs->isDecomp( p, CHANNEL_TYPE_LUMA ); } );
  xGenerateNNModeList( pu, pu.cs->picture->getRecoBuf( COMPONENT_Y ), bitDepth, available );
  xGenerateOutputJointHiddenLayer( pu, pu.cs->picture->getRecoBuf( COMPONENT_Y ), bitDepth, available );
}

UInt IntraPrediction::getFinalNNMode( const UInt finalModeIdx )
{
  return m_modeListNN[finalModeIdx];
}

void IntraPrediction::xGenerateNNModeList( const PredictionUnit &pu, const CPelBuf& src, const int bitDepth, std::function<bool( Position )> available )
{
  if( pu.cu->intra_NN_Use_Sampling )
  {
    m_IntraPrediction_NN.computeModeList_Downsampling( m_modeListNN, src, pu.Y(), bitDepth, available );
  }
  else
  {
    CHECK( pu.lwidth() > g_max_size_NNs || pu.lheight() > g_max_size_NNs, "Error: block size not supported" );
    const SizeType padded_width  = SizeType( g_PaddedSizes[pu.lwidth()] );
    const SizeType padded_height = SizeType( g_PaddedSizes[pu.lheight()] );
    const Area extended_area( pu.Y().pos(), Size( padded_width, padded_height ) );
    if( isTransposed( padded_width, padded_height ) ) 
    { 
      CHECK( pu.cu->intra_NN_Use_Sampling == 1, "Error" );
      m_IntraPrediction_NN.computeModeList_Transpose( m_modeListNN, src, extended_area, bitDepth, available ); 
    }
    else
    { 
      m_IntraPrediction_NN.computeModeList( m_modeListNN, src, extended_area, bitDepth, available ); 
    }
  }
}

void IntraPrediction::xGenerateOutputJointHiddenLayer( const PredictionUnit &pu, const CPelBuf& src, const int bitDepth, std::function<bool( Position )> available )
{
  if( pu.cu->intra_NN_Use_Sampling )
  {
    m_IntraPrediction_NN.computeJointHiddenOutputInt_Downsampling( m_outputJointHiddenLayer, src, pu.Y(), bitDepth, available );
  }
  else
  {
    CHECK( pu.lwidth() > g_max_size_NNs || pu.lheight() > g_max_size_NNs, "Error: block size not supported" );
    const SizeType padded_width  = SizeType(g_PaddedSizes[pu.lwidth()]);
    const SizeType padded_height = SizeType(g_PaddedSizes[pu.lheight()]);
    Area extended_area( pu.Y().pos(), Size( padded_width, padded_height ) );
    if( isTransposed( padded_width, padded_height ) ) 
    { 
      m_IntraPrediction_NN.computeJointHiddenOutputInt_Transpose( m_outputJointHiddenLayer, src, extended_area, bitDepth, available ); 
    }
    else
    { 
      m_IntraPrediction_NN.computeJointHiddenOutputInt          ( m_outputJointHiddenLayer, src, extended_area, bitDepth, available ); 
    }
  }
}

void IntraPrediction::predIntraNNModel( const ComponentID compId, PelBuf &piPred, const PredictionUnit &pu )
{
  CHECK( compId != COMPONENT_Y, "Error: chroma not supported" );
  xPredIntraLumaNNModel( piPred, pu, pu.intraNN_Mode_True );
}

void IntraPrediction::xPredIntraLumaNNModel( PelBuf &pDst, const PredictionUnit &pu, UInt uiTrueNNMode )
{
  m_praedSignalNN.clear();

  const int width  = pu.lwidth();
  const int height = pu.lheight();

  const int bitDepth = pu.cu->slice->getSPS()->getBitDepth( CHANNEL_TYPE_LUMA );

  if( pu.cu->intra_NN_Use_Sampling )
  {
    const Area& picArea = pu.cs->picture->Y();
    std::function<bool( Position )> available( [&picArea, &pu]( Position p ) -> bool { return picArea.contains( p ) && pu.cs->isDecomp( p, CHANNEL_TYPE_LUMA ); } );
    m_IntraPrediction_NN.computeLastLayerInt_Downsampling( m_praedSignalNN, m_outputJointHiddenLayer, width, height, uiTrueNNMode, bitDepth,
      pu.cs->picture->getRecoBuf( COMPONENT_Y ), pu.Y(), available );
  }
  else
  {
    m_IntraPrediction_NN.computeLastLayerInt( m_praedSignalNN, m_outputJointHiddenLayer, width, height, uiTrueNNMode, bitDepth );
  }

  CHECK( m_praedSignalNN.size() != (width * height), "Error" );
  if( isTransposed( width, height ) )
  {
    for( int y = 0; y < height; y++ )
    {
      for( int x = 0; x < width; x++ )
      {
        pDst.at( x, y ) = Pel( ClipBD( m_praedSignalNN[x * height + y], bitDepth ) );
      }
    }
  }
  else
  {
    for( int y = 0; y < height; y++ )
    {
      for( int x = 0; x < width; x++ )
      {
        pDst.at( x, y ) = Pel( ClipBD( m_praedSignalNN[y * width + x], bitDepth ) );
      }
    }
  }
}

void IntraPrediction::set_pu_Identifier_NNS( const PredictionUnit &pu )
{
  m_pu_Identifier_NNS.pu_area = pu.Y();
  m_pu_Identifier_NNS.cu_split_series = pu.cu->splitSeries;
  m_pu_Identifier_NNS.subsampling_flag = pu.cu->intra_NN_Use_Sampling;
  m_pu_Identifier_NNS.poc = pu.cs->picture->poc;
}

bool IntraPrediction::need_recalculation_for_NN( const PredictionUnit &pu )
{
  if( !pu.cs->pcv->isEncoder )
  {
    return true;
  }
  if( pu.Y() != m_pu_Identifier_NNS.pu_area )
  {
    return true;
  }
  if( pu.cu->intra_NN_Use_Sampling != m_pu_Identifier_NNS.subsampling_flag )
  {
    return true;
  }
  if( pu.cu->splitSeries != m_pu_Identifier_NNS.cu_split_series )
  {
    return true;
  }
  if( pu.cs->picture->poc != m_pu_Identifier_NNS.poc )
  {
    return true;
  }
  return false;
}
void IntraPrediction::set_pu_Identifier_diffFilter(const PredictionUnit &pu, const static_vector< UInt, FAST_UDI_MAX_RDMODE_NUM > RdModeList)
{
  m_pu_Identifier_DiffFilter.pu_area = pu.Y();
  m_pu_Identifier_DiffFilter.cu_split_series = pu.cu->splitSeries;

  m_pu_Identifier_DiffFilter.m_intraNN = pu.cu->intra_NN;
  m_pu_Identifier_DiffFilter.m_intra_NN_Use_Sampling = pu.cu->intra_NN_Use_Sampling;

  m_pu_Identifier_DiffFilter.m_numFullRDModes = (UInt)RdModeList.size();
  m_pu_Identifier_DiffFilter.m_RdModeList.clear();
  m_pu_Identifier_DiffFilter.m_RdModeList.resize(RdModeList.size());
  for (UInt i = 0; i < RdModeList.size(); i++) {
    m_pu_Identifier_DiffFilter.m_RdModeList[i] = RdModeList[i];
  }
}


bool IntraPrediction::need_recalculation_for_diffFilter(const PredictionUnit &pu, const static_vector< UInt, FAST_UDI_MAX_RDMODE_NUM > RdModeList)
{
  if (pu.Y() != m_pu_Identifier_DiffFilter.pu_area)
  {
    return true;
  }
  if (pu.cu->splitSeries != m_pu_Identifier_DiffFilter.cu_split_series)
  {
    return true;
  } 

  if (pu.cu->intra_NN != m_pu_Identifier_DiffFilter.m_intraNN)
    return true;

  if (pu.cu->intra_NN_Use_Sampling != m_pu_Identifier_DiffFilter.m_intra_NN_Use_Sampling)
    return true;

  if (RdModeList.size() != m_pu_Identifier_DiffFilter.m_numFullRDModes)
    //dies ist wichtig falls die List im Identifier groesser ist, auch dann muessen die besten 
    //Filtermodes nochmal berechnet werden, weil sie sich ja mischen und nur die oberste Teilmenge
    //genommen wird
    return true;

  for (UInt i = 0; i < RdModeList.size(); i++) {
    //es kann passieren, dass im Fall von Ang65 sich die (gespeicherten) Modes nochmal von nsstIdx off 
    //auf on veraendern: dann muessen die Filtermodes aktualisiert werden
    if (m_pu_Identifier_DiffFilter.m_RdModeList[i] != RdModeList[i])
      return true;
  }

  return false;
}

Void IntraPrediction::xPredIntraDc( const CPelBuf &pSrc, const int mrlOffset, PelBuf &pDst, const ChannelType &channelType, const bool &enableBoundaryFilter )
{
  const Pel dcval = xGetPredValDc( pSrc, mrlOffset, pDst );
  pDst.fill( dcval );

  if( enableBoundaryFilter && ( mrlOffset == 0 ) )
  {
    xDCPredFiltering( pSrc, pDst, channelType );
  }
}

/** Function for filtering intra DC predictor. This function performs filtering left and top edges of the prediction samples for DC mode (intra coding).
 */
Void IntraPrediction::xDCPredFiltering(const CPelBuf &pSrc, PelBuf &pDst, const ChannelType &channelType)
{
  UInt iWidth = pDst.width;
  UInt iHeight = pDst.height;
  Int x, y;

  if (isLuma(channelType) && (iWidth <= MAXIMUM_INTRA_FILTERED_WIDTH) && (iHeight <= MAXIMUM_INTRA_FILTERED_HEIGHT))
  {
    //top-left
    pDst.at(0, 0) = (Pel)((pSrc.at(1, 0) + pSrc.at(0, 1) + 2 * pDst.at(0, 0) + 2) >> 2);

    //top row (vertical filter)
    for ( x = 1; x < iWidth; x++ )
    {
      pDst.at(x, 0) = (Pel)((pSrc.at(x + 1, 0)  +  3 * pDst.at(x, 0) + 2) >> 2);
    }

    //left column (horizontal filter)
    for ( y = 1; y < iHeight; y++ )
    {
      pDst.at(0, y) = (Pel)((pSrc.at(0, y + 1) + 3 * pDst.at(0, y) + 2) >> 2);
    }
  }

  return;
}

// Function for deriving the angular Intra predictions

/** Function for deriving the simplified angular intra predictions.
*
* This function derives the prediction samples for the angular mode based on the prediction direction indicated by
* the prediction mode index. The prediction direction is given by the displacement of the bottom row of the block and
* the reference row above the block in the case of vertical prediction or displacement of the rightmost column
* of the block and reference column left from the block in the case of the horizontal prediction. The displacement
* is signalled at 1/32 pixel accuracy. When projection of the predicted pixel falls inbetween reference samples,
* the predicted value for the pixel is linearly interpolated from the reference samples. All reference samples are taken
* from the extended main reference.
*/
//NOTE: Bit-Limit - 25-bit source
Void IntraPrediction::xPredIntraAng( const CPelBuf &pSrc, const int mrlOffset, PelBuf &pDst, const ChannelType &channelType, const UInt &dirMode, const ClpRng& clpRng, const Bool &bEnableEdgeFilters, const SPS& sps, const bool &enableBoundaryFilter )
{
  Int width =Int(pDst.width);
  Int height=Int(pDst.height);

  CHECK( !( dirMode > DC_IDX && dirMode < NUM_LUMA_MODE ), "Invalid intra dir" );

  const Bool       bIsModeVer         = (dirMode >= DIA_IDX);
  const Int        intraPredAngleMode = (bIsModeVer) ? (Int)dirMode - VER_IDX :  -((Int)dirMode - HOR_IDX);
  const Int        absAngMode         = abs(intraPredAngleMode);
  const Int        signAng            = intraPredAngleMode < 0 ? -1 : 1;
  const Bool       edgeFilter         = bEnableEdgeFilters && isLuma(channelType) && (width <= MAXIMUM_INTRA_FILTERED_WIDTH) && (height <= MAXIMUM_INTRA_FILTERED_HEIGHT) && (width > 1) && (height > 1) && (mrlOffset == 0);

  // Set bitshifts and scale the angle parameter to block size

  static const Int angTable[17]    = { 0,    1,    2,    3,    5,    7,    9,   11,   13,   15,   17,   19,   21,   23,   26,   29,   32 };
  static const Int invAngTable[17] = { 0, 8192, 4096, 2731, 1638, 1170,  910,  745,  630,  546,  482,  431,  390,  356,  315,  282,  256 }; // (256 * 32) / Angle

  Int invAngle                    = invAngTable[absAngMode];
  Int absAng                      = angTable   [absAngMode];
  Int intraPredAngle              = signAng * absAng;

  Pel* refMain;
  Pel* refSide;

  Pel  refAbove[2 * MAX_CU_SIZE + 7];
  Pel  refLeft [2 * MAX_CU_SIZE + 7];

  // Initialize the Main and Left reference array.
  if (intraPredAngle < 0)
  {
    for( Int x = 0; x < width + 1 + mrlOffset; x++ )
    {
      refAbove[x + height - 1] = pSrc.at( x, 0 );
    }
    for( Int y = 0; y < height + 1 + mrlOffset; y++ )
    {
      refLeft[y + width - 1] = pSrc.at( 0, y );
    }
    refMain = (bIsModeVer ? refAbove + height : refLeft  + width ) - 1;
    refSide = (bIsModeVer ? refLeft  + width  : refAbove + height) - 1;

    // Extend the Main reference to the left.
    Int invAngleSum    = 128;       // rounding for (shift by 8)
    const Int refMainOffsetPreScale = bIsModeVer ? height : width;
    for( Int k = -1; k > (refMainOffsetPreScale * intraPredAngle) >> 5; k-- )
    {
      invAngleSum += invAngle;
      refMain[k] = refSide[invAngleSum>>8];
    }
  }
  else
  {
    for( Int x = 0; x < width + height + 1 + 2*mrlOffset; x++ )
    {
      refAbove[x] = pSrc.at(x, 0);
      refLeft[x]  = pSrc.at(0, x);
    }
    refMain = bIsModeVer ? refAbove : refLeft ;
    refSide = bIsModeVer ? refLeft  : refAbove;
  }

  // swap width/height if we are doing a horizontal mode:
  Pel tempArray[MAX_CU_SIZE*MAX_CU_SIZE];
  const Int dstStride = bIsModeVer ? pDst.stride : MAX_CU_SIZE;
  Pel *pDstBuf = bIsModeVer ? pDst.buf : tempArray;
  if (!bIsModeVer)
  {
    std::swap(width, height);
  }

  // compensate for mrlOffset in reference line buffers
  refMain += mrlOffset;
  refSide += mrlOffset;

  if( intraPredAngle == 0 )  // pure vertical or pure horizontal
  {
    for( Int y = 0; y < height; y++ )
    {
      for( Int x = 0; x < width; x++ )
      {
        pDstBuf[y*dstStride + x] = refMain[x + 1];
      }
    }

    if (edgeFilter)
    {
      for( Int y = 0; y < height; y++ )
      {
        pDstBuf[y*dstStride] = ClipPel( pDstBuf[y*dstStride] + ( ( refSide[y + 1] - refSide[0] ) >> 1 ), clpRng );
      }
    }
  }
  else
  {
    Pel *pDsty=pDstBuf;

    for (Int y=0, deltaPos=intraPredAngle*(1+mrlOffset); y<height; y++, deltaPos+=intraPredAngle, pDsty+=dstStride)
    {
      const Int deltaInt   = deltaPos >> 5;
      const Int deltaFract = deltaPos & (32 - 1);

      if( absAng < 32 )
      {
        if( sps.getSpsNext().getUseIntra4Tap() && width > 1 && height > 1 )
        {
          Int         p[4];
          const Bool  useCubicFilter = (width <= 8);
          const Int  *f              = (useCubicFilter) ? g_intraCubicFilter[deltaFract] : g_intraGaussFilter[deltaFract];
          Int         refMainIndex   = deltaInt + 1;

          for( Int x = 0; x < width; x++, refMainIndex++ )
          {
            p[1] = refMain[refMainIndex];
            p[2] = refMain[refMainIndex + 1];

            p[0] = x == 0 ? p[1] : refMain[refMainIndex - 1];
            p[3] = x == (width - 1) ? p[2] : refMain[refMainIndex + 2];

            pDstBuf[y*dstStride + x] =  (Pel)((f[0] * p[0] + f[1] * p[1] + f[2] * p[2] + f[3] * p[3] + 128) >> 8);

            if( useCubicFilter ) // only cubic filter has negative coefficients and requires clipping
            {
              pDstBuf[y*dstStride + x] = ClipPel( pDstBuf[y*dstStride + x], clpRng );
            }
          }
        }
        else
        {
          // Do linear filtering
          const Pel *pRM = refMain + deltaInt + 1;
          Int lastRefMainPel = *pRM++;
          for( Int x = 0; x < width; pRM++, x++ )
          {
            Int thisRefMainPel = *pRM;
            pDsty[x + 0] = ( Pel ) ( ( ( 32 - deltaFract )*lastRefMainPel + deltaFract*thisRefMainPel + 16 ) >> 5 );
            lastRefMainPel = thisRefMainPel;
          }
        }
      }
      else
      {
        // Just copy the integer samples
        for( Int x = 0; x < width; x++ )
        {
          pDsty[x] = refMain[x + deltaInt + 1];
        }
      }
    }
    if( edgeFilter && absAng <= 1 )
    {
      for( Int y = 0; y < height; y++ )
      {
        pDstBuf[y*dstStride] = ClipPel( pDstBuf[y*dstStride] + ((refSide[y + 1] - refSide[0]) >> 2), clpRng );
      }
    }
  }

  // Flip the block if this is the horizontal mode
  if( !bIsModeVer )
  {
    for( Int y = 0; y < height; y++ )
    {
      for( Int x = 0; x < width; x++ )
      {
        pDst.at( y, x ) = pDstBuf[x];
      }
      pDstBuf += dstStride;
    }
  }

  if( sps.getSpsNext().getUseIntraBoundaryFilter() && enableBoundaryFilter && isLuma( channelType ) && (width > 2) && (height > 2) && (mrlOffset == 0) )
  {
    if( dirMode == VDIA_IDX )
    {
      xIntraPredFilteringMode34( pSrc, pDst );
    }
    else  if( dirMode == 2 )
    {
      xIntraPredFilteringMode02( pSrc, pDst );
    }
    else if( ( dirMode <= 10 && dirMode > 2 ) || ( dirMode >= ( VDIA_IDX - 8 ) && dirMode < VDIA_IDX ) )
    {
      xIntraPredFilteringModeDGL( pSrc, pDst, dirMode );
    }
  }
}

Void IntraPrediction::xIntraPredFilteringMode34(const CPelBuf &pSrc, PelBuf &pDst)
{
  UInt iWidth  = pDst.width;
  UInt iHeight = pDst.height;

  Int y;

  for( y = 0; y < iHeight; y++ )
  {
    pDst.at( 0, y ) = (  8 * pDst.at( 0, y ) + 8 * pSrc.at( 0, y + 2 ) + 8 ) >> 4;
    pDst.at( 1, y ) = ( 12 * pDst.at( 1, y ) + 4 * pSrc.at( 0, y + 3 ) + 8 ) >> 4;

    if( iWidth > 2 )
    {
      pDst.at( 2, y ) = ( 14 * pDst.at( 2, y ) + 2 * pSrc.at( 0, y + 4 ) + 8 ) >> 4;
      pDst.at( 3, y ) = ( 15 * pDst.at( 3, y ) +     pSrc.at( 0, y + 5 ) + 8 ) >> 4;

    }
  }
}

Void IntraPrediction::xIntraPredFilteringMode02(const CPelBuf &pSrc, PelBuf &pDst)
{
  UInt iWidth  = pDst.width;
  UInt iHeight = pDst.height;

  Int x;

  for( x = 0; x < iWidth; x++ )
  {
    pDst.at( x, 0 ) = (  8 * pDst.at( x, 0 ) + 8 * pSrc.at( x + 2, 0 ) + 8 ) >> 4;
    pDst.at( x, 1 ) = ( 12 * pDst.at( x, 1 ) + 4 * pSrc.at( x + 3, 0 ) + 8 ) >> 4;

    if( iHeight > 2 )
    {
      pDst.at( x, 2 ) = ( 14 * pDst.at( x, 2 ) + 2 * pSrc.at( x + 4, 0 ) + 8 ) >> 4;
      pDst.at( x, 3 ) = ( 15 * pDst.at( x, 3 ) +     pSrc.at( x + 5, 0 ) + 8 ) >> 4;
    }
  }
}

Void IntraPrediction::xIntraPredFilteringModeDGL(const CPelBuf &pSrc, PelBuf &pDst, UInt uiMode)
{
  UInt iWidth = pDst.width;
  UInt iHeight = pDst.height;
  Int x, y;

  const Int aucAngPredFilterCoef[8][3] = {
    { 12, 3, 1 }, { 12, 3, 1 },
    { 12, 1, 3 }, { 12, 2, 2 },
    { 12, 2, 2 }, { 12, 3, 1 },
    {  8, 6, 2 }, {  8, 7, 1 },
  };
  const Int aucAngPredPosiOffset[8][2] = {
    { 2, 3 }, { 2, 3 },
    { 1, 2 }, { 1, 2 },
    { 1, 2 }, { 1, 2 },
    { 1, 2 }, { 1, 2 },
  };

  CHECK( !( ( uiMode >= ( VDIA_IDX - 8 ) && uiMode < VDIA_IDX ) || ( uiMode > 2 && uiMode <= ( 2 + 8 ) ) ), "Incorrect mode" );

  Bool bHorz    = (uiMode < DIA_IDX);
  UInt deltaAng = bHorz ? ((2 + 8) - uiMode) : (uiMode - (VDIA_IDX - 8));

  const Int *offset = aucAngPredPosiOffset[deltaAng];
  const Int *filter = aucAngPredFilterCoef[deltaAng];

  if (bHorz)
  {
    for (x = 0; x < iWidth; x++)
    {
      pDst.at( x, 0 ) = ( filter[0] * pDst.at( x                , 0 )
                        + filter[1] * pSrc.at( x + 1 + offset[0], 0 )
                        + filter[2] * pSrc.at( x + 1 + offset[1], 0 ) + 8 ) >> 4;
    }
  }
  else
  {
    for (y = 0; y < iHeight; y++)
    {
      pDst.at( 0, y ) = ( filter[0] * pDst.at( 0,   y )
                        + filter[1] * pSrc.at( 0, ( y + offset[0] + 1 ) )
                        + filter[2] * pSrc.at( 0, ( y + offset[1] + 1 ) ) + 8 ) >> 4;
    }
  }

  return;
}

void IntraPrediction::xReferenceFilter( const int doubleSize, const int origWeight, const int filterOrder, Pel *piRefVector, Pel *piLowPassRef )
{
  const int imCoeff[3][4] =
  {
    { 20, 15, 6, 1 },
    { 16, 14, 7, 3 },
    { 14, 12, 9, 4 }
  };

  const int * piFc;

  int binBuff[4 * MAX_CU_SIZE + 9];
  int * piTmp = &binBuff[2 * MAX_CU_SIZE + 4];   // to  use negative indexes
  Pel * piDat = piRefVector;
  Pel * piRes = piLowPassRef;

  for( int k = -doubleSize; k <= doubleSize; k++ )
    piTmp[k] = piDat[k];

  for( int n = 1; n <= 3; n++ )
  {
    piTmp[-doubleSize - n] = piTmp[-doubleSize - 1 + n];
    piTmp[ doubleSize + n] = piTmp[ doubleSize + 1 - n];
  }

  switch( filterOrder )
  {
  case 0:
    break;
  case 1:
    for( int k = -doubleSize; k <= doubleSize; k++ )
      piRes[k] = (Pel)(((piTmp[k] << 1) + piTmp[k - 1] + piTmp[k + 1] + 2) >> 2);
    break;
  case 2:
    for( int k = -doubleSize; k <= doubleSize; k++ )
      piRes[k] = (Pel)(((piTmp[k] << 1) + ((piTmp[k] + piTmp[k - 1] + piTmp[k + 1]) << 2) + piTmp[k - 2] + piTmp[k + 2] + 8) >> 4);
    break;
  case 3:
  case 5:
  case 7:
    piFc = imCoeff[(filterOrder - 3) >> 1];
    for( int k = -doubleSize; k <= doubleSize; k++ )
    {
      int s = 32 + piFc[0] * piTmp[k];
      for( int n = 1; n < 4; n++ )
        s += piFc[n] * (piTmp[k - n] + piTmp[k + n]);

      piRes[k] = (Pel)(s >> 6);
    }
    break;
  default:
    EXIT( "Invalid intra prediction reference filter order" );
  }

  int ParShift = 6; //normalization factor
  int ParScale = 1 << ParShift;
  int ParOffset = 1 << (ParShift - 1);

  if( origWeight != 0 )
  {
    int iCmptWeight = ParScale - origWeight;
    for( int k = -doubleSize; k <= doubleSize; k++ )
      piLowPassRef[k] = (origWeight * piRefVector[k] + iCmptWeight * piLowPassRef[k] + ParOffset) >> ParShift;
  }
}

Bool IntraPrediction::useDPCMForFirstPassIntraEstimation(const PredictionUnit &pu, const UInt &uiDirMode)
{
  return CU::isRDPCMEnabled(*pu.cu) && pu.cu->transQuantBypass && (uiDirMode == HOR_IDX || uiDirMode == VER_IDX);
}

inline Bool isAboveLeftAvailable  ( const CodingUnit &cu, const ChannelType &chType, const Position &posLT );
inline Int  isAboveAvailable      ( const CodingUnit &cu, const ChannelType &chType, const Position &posLT, const UInt uiNumUnitsInPU, const UInt unitWidth, Bool *validFlags );
inline Int  isLeftAvailable       ( const CodingUnit &cu, const ChannelType &chType, const Position &posLT, const UInt uiNumUnitsInPU, const UInt unitWidth, Bool *validFlags );
inline Int  isAboveRightAvailable ( const CodingUnit &cu, const ChannelType &chType, const Position &posRT, const UInt uiNumUnitsInPU, const UInt unitHeight, Bool *validFlags );
inline Int  isBelowLeftAvailable  ( const CodingUnit &cu, const ChannelType &chType, const Position &posLB, const UInt uiNumUnitsInPU, const UInt unitHeight, Bool *validFlags );

Void IntraPrediction::initIntraPatternChType(const CodingUnit &cu, const CompArea &area, const Bool bFilterRefSamples)
{
  const CodingStructure& cs   = *cu.cs;

  Pel *refBufUnfiltered   = m_piYuvExt[area.compID][PRED_BUF_UNFILTERED];
  Pel *refBufFiltered     = m_piYuvExt[area.compID][PRED_BUF_FILTERED];
  if ( ( cu.mrlIdx > 0 ) && ( area.compID == COMPONENT_Y ) )
  {
    refBufUnfiltered   = m_piYuvExtMrl[area.compID][PRED_BUF_UNFILTERED];
    refBufFiltered     = m_piYuvExtMrl[area.compID][PRED_BUF_FILTERED];
  }

  // ----- Step 1: unfiltered reference samples -----
  xFillReferenceSamples( cs.picture->getRecoBuf( area ), refBufUnfiltered, area, cu );

  // ----- Step 2: filtered reference samples -----
  if( bFilterRefSamples )
  {
    xFilterReferenceSamples( refBufUnfiltered, refBufFiltered, area, *cs.sps, cu );
  }
}

void IntraPrediction::xFillReferenceSamples( const CPelBuf &recoBuf, Pel* refBufUnfiltered, const CompArea &area, const CodingUnit &cu )
{
  const ChannelType      chType = toChannelType( area.compID );
  const CodingStructure &cs     = *cu.cs;
  const SPS             &sps    = *cs.sps;
  const PreCalcValues   &pcv    = *cs.pcv;

  const int  tuWidth            = area.width;
  const int  tuHeight           = area.height;
  // only fill extended reference samples for luma
  const int  mrlOffset          = ( area.compID == COMPONENT_Y ) ? cu.mrlIdx : 0;
  const int  predSize           = tuWidth + tuHeight + mrlOffset;
  const int  predStride         = predSize + 1 + mrlOffset;

  const bool noShift            = pcv.noChroma2x2 && area.width == 4; // don't shift on the lowest level (chroma not-split)
  const Int  unitWidth          = (cu.mode1dPartitions && (tuWidth == 1 || tuHeight == 1) && canUse1dPartitions(area.compID)) ? 1 : pcv.minCUWidth  >> (noShift ? 0 : getComponentScaleX(area.compID, sps.getChromaFormatIdc()));
  const Int  unitHeight         = (cu.mode1dPartitions && (tuWidth == 1 || tuHeight == 1) && canUse1dPartitions(area.compID)) ? 1 : pcv.minCUHeight >> (noShift ? 0 : getComponentScaleY(area.compID, sps.getChromaFormatIdc()));

  const int  totalAboveUnits    = (predSize + (unitWidth - 1)) / unitWidth;
  const int  totalLeftUnits     = (predSize + (unitHeight - 1)) / unitHeight;
  const int  totalUnits         = totalAboveUnits + totalLeftUnits + 1; //+1 for top-left
  const int  numAboveUnits      = std::max<int>( tuWidth / unitWidth, 1 );
  const int  numLeftUnits       = std::max<int>( tuHeight / unitHeight, 1 );
  const int  numAboveRightUnits = totalAboveUnits - numAboveUnits;
  const int  numLeftBelowUnits  = totalLeftUnits - numLeftUnits;

  CHECK( numAboveUnits <= 0 || numLeftUnits <= 0 || numAboveRightUnits <= 0 || numLeftBelowUnits <= 0, "Size not supported" );

  // ----- Step 1: analyze neighborhood -----
  const Position posLT          = area;
  const Position posRT          = area.topRight();
  const Position posLB          = area.bottomLeft();

  Bool  neighborFlags[MAX_TU_SIZE * MAX_NUM_PART_IDXS_IN_CTU_WIDTH + 1];
  int   numIntraNeighbor = 0;

  memset( neighborFlags, 0, totalUnits );

  neighborFlags[totalLeftUnits] = isAboveLeftAvailable( cu, chType, posLT );
  bool aboveLeftSampleIsAvailable   = neighborFlags[totalLeftUnits];
  int aux                           = 0;
  numIntraNeighbor                 += neighborFlags[totalLeftUnits] ? 1 : 0;
  aux                               = isAboveAvailable( cu, chType, posLT, numAboveUnits, unitWidth, ( neighborFlags + totalLeftUnits + 1 ) );
  bool aboveRowSamplesAreAvailable  = aux != 0;
  numIntraNeighbor                 += aux;
  aux                               = 0;
  aux                               = isAboveRightAvailable( cu, chType, posRT, numAboveRightUnits, unitWidth, ( neighborFlags + totalLeftUnits + 1 + numAboveUnits ) );
  bool aboveRightSamplesAreAvailable= aux != 0;
  numIntraNeighbor                 += aux;
  aux                               = 0;
  aux                               = isLeftAvailable( cu, chType, posLT, numLeftUnits, unitHeight, ( neighborFlags + totalLeftUnits - 1 ) );
  bool leftColSamplesAreAvailable   = aux != 0;
  numIntraNeighbor                 += aux;
  aux                               = 0;
  aux                               = isBelowLeftAvailable( cu, chType, posLB, numLeftBelowUnits, unitHeight, ( neighborFlags + totalLeftUnits - 1 - numLeftUnits ) );
  bool belowLeftSamplesAreAvailable = aux != 0;
  numIntraNeighbor                 += aux;

  // ----- Step 2: fill reference samples (depending on neighborhood) -----
  CHECK( (predStride-2*mrlOffset) * (predStride-2*mrlOffset) > m_iYuvExtSize, "Reference sample area not supported" );

  const Pel*  srcBuf    = recoBuf.buf;
  const int   srcStride = recoBuf.stride;
        Pel*  ptrDst    = refBufUnfiltered;
  const Pel*  ptrSrc;
  const Pel   valueDC   = 1 << (sps.getBitDepth( chType ) - 1);

  bool fillReferenceSamplesAsUsual = true;
  //we check if it is a partition that does not have the usual processing order
  if( canUse1dPartitions( area.compID ) && cu.mode1dPartitions != NO_1D_PARTITION && ( tuWidth == 1 || tuHeight == 1 ) )
  {
    //we check if it is a 1D partition that is coded backwards and it is at least the second TU
    PartSplit partitionType = CU::select1dPartitionType( cu, area.compID );
    if( partitionType == TU_1D_HORZ_SPLIT_REVERSE_ORDER && !CU::isFirst1dPartition( cu, area, area.compID ) )
    {
      fillReferenceSamplesAsUsual = false;
      //we substitute the above reference samples with the below reference samples
      int picTuOffset = srcStride;
      for( int K = 1; K < tuWidth + 1; K++ )
      {
        ptrDst[K] = srcBuf[picTuOffset + K - 1];
      }
      //bool leftColSamplesAreAvailable = area.x > 0;
      if( leftColSamplesAreAvailable )
      {
        ptrDst[0] = srcBuf[picTuOffset - 1];
        //we get the other left sample needed
        ptrDst[predStride] = srcBuf[-1];
      }
      else
      {
        Pel defaultValue = valueDC;
        ptrDst[0] = defaultValue;
        //we get the other left sample needed
        ptrDst[predStride] = defaultValue;
      }
    }
    else if( partitionType == TU_1D_VERT_SPLIT_REVERSE_ORDER && !CU::isFirst1dPartition( cu, area, area.compID ) )
    {
      fillReferenceSamplesAsUsual = false;
      //we substitute the left reference samples with the right reference samples
      int picTuOffset = srcStride;
      for( int K = 1; K < tuHeight + 1; K++ )
      {
        ptrDst[K*predStride] = srcBuf[( K - 1 )*picTuOffset + 1];
      }
      //bool aboveRowSamplesAreAvailable = area.y > 0;
      if( aboveRowSamplesAreAvailable )
      {
        ptrDst[0] = srcBuf[-picTuOffset + 1];
        //we get the other above sample needed
        ptrDst[1] = srcBuf[-picTuOffset];
      }
      else
      {
        Pel defaultValue = valueDC;
        ptrDst[0] = defaultValue;
        //we get the other above sample needed
        ptrDst[1] = defaultValue;
      }
    }
    else if( partitionType == TU_1D_HORZ_SPLIT )
    {
      fillReferenceSamplesAsUsual = false;
      int picTuOffset = srcStride;
      bool isFirstPartition = CU::isFirst1dPartition( cu, area, area.compID );
      //above-left sample
      ptrDst[0] = aboveLeftSampleIsAvailable ? srcBuf[-picTuOffset - 1] : aboveRowSamplesAreAvailable ? srcBuf[-picTuOffset] : leftColSamplesAreAvailable ? srcBuf[-1] : valueDC;
      //above row samples
      if( !isFirstPartition || aboveRowSamplesAreAvailable )
      {
        for( int K = 1; K < tuWidth + 1; K++ )
        {
          ptrDst[K] = srcBuf[-picTuOffset + K - 1];
        }
      }
      else
      {
        for( int K = 1; K < tuWidth + 1; K++ )
        {
          ptrDst[K] = ptrDst[0];
        }
      }
      //above-right sample
      ptrDst[tuWidth + 1] = isFirstPartition && aboveRightSamplesAreAvailable ? srcBuf[-picTuOffset + tuWidth] : ptrDst[tuWidth];
      //left col sample and below-left sample
      if( leftColSamplesAreAvailable )
      {
        //left sample
        ptrDst[predStride] = srcBuf[-1];
        //below-left sample
        ptrDst[predStride << 1] = !CU::isLast1dPartition( cu, area, area.compID ) || belowLeftSamplesAreAvailable ? srcBuf[picTuOffset - 1] : ptrDst[predStride];
      }
      else
      {
        //left sample
        ptrDst[predStride] = ptrDst[0];
        //below-left sample
        ptrDst[predStride << 1] = ptrDst[0];
      }
    }
    else if( partitionType == TU_1D_VERT_SPLIT )
    {
      fillReferenceSamplesAsUsual = false;
      int picTuOffset = srcStride;
      bool isFirstPartition = CU::isFirst1dPartition( cu, area, area.compID );
      //above-left sample
      ptrDst[0] = aboveLeftSampleIsAvailable ? srcBuf[-picTuOffset - 1] : aboveRowSamplesAreAvailable ? srcBuf[-picTuOffset] : leftColSamplesAreAvailable ? srcBuf[-1] : valueDC;
      if( aboveRowSamplesAreAvailable )
      {
        //above row sample
        ptrDst[1] = srcBuf[-picTuOffset];
        //above-right sample
        ptrDst[2] = !CU::isLast1dPartition( cu, area, area.compID ) || aboveRightSamplesAreAvailable ? srcBuf[-picTuOffset + 1] : ptrDst[1];
      }
      else
      {
        //above row sample
        ptrDst[1] = ptrDst[0];
        //above-right sample
        ptrDst[2] = ptrDst[0];
      }
      //left samples
      if( !isFirstPartition || leftColSamplesAreAvailable )
      {
        for( int K = 1; K < tuHeight + 1; K++ )
        {
          ptrDst[K*predStride] = srcBuf[( K - 1 )*picTuOffset - 1];
        }
      }
      else
      {
        for( int K = 1; K < tuHeight + 1; K++ )
        {
          ptrDst[K*predStride] = ptrDst[0];
        }
      }
      //below-left sample
      ptrDst[( tuHeight + 1 )*predStride] = isFirstPartition && belowLeftSamplesAreAvailable ? srcBuf[tuHeight*picTuOffset - 1] : ptrDst[tuHeight*predStride];
    }

    if( !fillReferenceSamplesAsUsual )
    {
      return;
    }
  }

  if( numIntraNeighbor == 0 )
  {
    // Fill border with DC value
    for( int j = 0; j <= predSize + mrlOffset; j++ ) { ptrDst[j]            = valueDC; }
    for( int i = 1; i <= predSize + mrlOffset; i++ ) { ptrDst[i*predStride] = valueDC; }
  }
  else if( numIntraNeighbor == totalUnits )
  {
    // Fill top-left border and top and top right with rec. samples
    ptrSrc = srcBuf - ( 1 + mrlOffset ) * srcStride - ( 1 + mrlOffset );
    for( int j = 0; j <= predSize + mrlOffset; j++ ) { ptrDst[j] = ptrSrc[j]; }
    // Fill left and below left border with rec. samples
    ptrSrc += srcStride;
    for( int i = 1; i <= predSize + mrlOffset; i++ ) { ptrDst[i*predStride] = *(ptrSrc); ptrSrc += srcStride; }
  }
  else // reference samples are partially available
  {
    // BB: new implementation using directly ptrDst
    // ---------------------------------------------

    // Copying of available reference samples
    //-----------------------------------------

    // Fill top-left sample(s) if available
    ptrSrc = srcBuf - ( 1 + mrlOffset ) * srcStride - ( 1 + mrlOffset );
    ptrDst = refBufUnfiltered;
    if( neighborFlags[totalLeftUnits] )
    {
      ptrDst[0] = ptrSrc[0];
      for ( int i = 1; i <= mrlOffset; i++ )
      {
        ptrDst[i]            = ptrSrc[i];
        ptrDst[i*predStride] = ptrSrc[i*srcStride];
      }
    }

    // Fill left & below-left samples if available (downwards)
    ptrSrc += ( 1 + mrlOffset ) * srcStride;
    ptrDst += ( 1 + mrlOffset ) * predStride;
    for( int unitIdx = totalLeftUnits - 1; unitIdx > 0; unitIdx-- )
    {
      if( neighborFlags[unitIdx] )
      {
        for( int i = 0; i < unitHeight; i++ )
        {
          ptrDst[i*predStride] = ptrSrc[i*srcStride];
        }
      }
      ptrSrc += unitHeight*srcStride;
      ptrDst += unitHeight*predStride;
    }
    // Fill last below-left sample(s)
    if( neighborFlags[0] )
    {
      // In case of MRL, last samples are not a full unit but just mrlOffset samples
      int lastSample = mrlOffset > 0 ? mrlOffset : unitHeight;
      for ( int i = 0; i < lastSample; i++ )
      {
        ptrDst[i*predStride] = ptrSrc[i*srcStride];
      }
    }

    // Fill above & above-right samples if available (left-to-right)
    ptrSrc = srcBuf - srcStride * ( 1 + mrlOffset );
    ptrDst = refBufUnfiltered + 1 + mrlOffset;
    for( int unitIdx = totalLeftUnits + 1; unitIdx < totalUnits - 1; unitIdx++ )
    {
      if( neighborFlags[unitIdx] )
      {
        for( int j = 0; j < unitWidth; j++ )
        {
          ptrDst[j] = ptrSrc[j];
        }
      }
      ptrSrc += unitWidth;
      ptrDst += unitWidth;
    }
    // Fill last above-right sample(s)
    if( neighborFlags[totalUnits - 1] )
    {
      // In case of MRL, last samples are not a full unit but just mrlOffset samples
      int lastSample = mrlOffset > 0 ? mrlOffset : unitHeight;
      for ( int j = 0; j < lastSample; j++ )
      {
        ptrDst[j] = ptrSrc[j];
      }
    }

    // Padding of unavailable reference samples
    //-----------------------------------------

    // last below-left is n.a.
    // pad from first available down to the last below-left
    ptrDst = refBufUnfiltered;
    int lastAvailUnit = 0;
    if( !neighborFlags[0] )
    {
      // determine first available unit
      int firstAvailUnit = 1;
      while( firstAvailUnit < totalUnits && !neighborFlags[firstAvailUnit] )
      {
        firstAvailUnit++;
      }

      // first available sample
      int firstAvailRow = 0;
      int firstAvailCol = 0;
      if ( firstAvailUnit < totalLeftUnits )              // is left
      {
        firstAvailRow = ( totalLeftUnits - firstAvailUnit ) * unitHeight + mrlOffset;
      }
      else if ( firstAvailUnit == totalLeftUnits )       // is top-left
      {
        firstAvailRow = mrlOffset;
      }
      else                                               // is above
      {
        firstAvailCol = ( firstAvailUnit - totalLeftUnits - 1 ) * unitWidth + 1 + mrlOffset;
      }
      const Pel firstAvailSample = ptrDst[firstAvailCol + firstAvailRow * predStride];

      // last sample below-left (n.a.)
      int lastRow = mrlOffset > 0 ? ( ( totalLeftUnits * unitHeight ) + ( 2 * mrlOffset - 4 ) ) : ( totalLeftUnits * unitHeight );

      // pad unavailable samples from last sample below-left with first available sample
      // fill left column
      for ( int i = lastRow; i > firstAvailRow; i-- )
      {
        ptrDst[i*predStride] = firstAvailSample;
      }
      // fill top row
      if ( firstAvailCol > 0 )
      {
        for ( int j = 0; j < firstAvailCol; j++ )
        {
          ptrDst[j] = firstAvailSample;
        }
      }
      // first available unit becomes last available unit for subsequent units that are n.a.
      lastAvailUnit = firstAvailUnit;
    }

    // pad all other reference samples.
    int currUnit = lastAvailUnit + 1;
    while( currUnit < totalUnits )
    {
      if( !neighborFlags[currUnit] ) // samples not available
      {
        // last available sample
        int lastAvailRow = 0;
        int lastAvailCol = 0;
        if ( lastAvailUnit < totalLeftUnits )             // is left
        {
          lastAvailRow = ( totalLeftUnits - lastAvailUnit - 1 ) * unitHeight + mrlOffset + 1;
        }
        else if ( lastAvailUnit == totalLeftUnits )       // is top-left
        {
          lastAvailCol = mrlOffset;
        }
        else                                              // is above
        {
          lastAvailCol = ( lastAvailUnit - totalLeftUnits ) * unitWidth + mrlOffset;
        }
        const Pel lastAvailSample = ptrDst[lastAvailCol + lastAvailRow * predStride];

        // fill current unit with last available sample
        if ( currUnit < totalLeftUnits )              // is left
        {
          for( int i = lastAvailRow - unitHeight; i < lastAvailRow; i++ )
          {
            ptrDst[i*predStride] = lastAvailSample;
          }
        }
        else if ( currUnit == totalLeftUnits )       // is top-left
        {
          for( int i = 1; i < mrlOffset + 1; i++ )
          {
            ptrDst[i*predStride] = lastAvailSample;
          }
          for( int j = 0; j < mrlOffset + 1; j++ )
          {
            ptrDst[j] = lastAvailSample;
          }
        }
        else                                         // is above
        {
          // in case of MRL, last right unit contains only mrlOffset reference samples
          int numSamplesInUnit = ( mrlOffset > 0 ) && ( currUnit == totalUnits - 1 ) ? mrlOffset : unitWidth;
          for( int j = lastAvailCol + 1; j <= lastAvailCol + numSamplesInUnit; j++ )
          {
            ptrDst[j] = lastAvailSample;
          }
        }
      }
      lastAvailUnit = currUnit;
      currUnit++;
    }
  }
}

void IntraPrediction::xFilterReferenceSamples( const Pel* refBufUnfiltered, Pel* refBufFiltered, const CompArea &area, const SPS &sps, const CodingUnit &cu )
{
  const int  tuWidth    = area.width;
  const int  tuHeight   = area.height;
  // only fill extended reference samples for luma
  const int  mrlOffset          = ( area.compID == COMPONENT_Y ) ? cu.mrlIdx : 0;
  const int  predSize   = tuWidth + tuHeight + 2 * mrlOffset;
  const int  predStride = predSize + 1;

  // Strong filter
  if( sps.getSpsNext().getUseIntraBiFi() && useBilateralFilter( area ) )
  {
    Pel tempRef[4 * MAX_CU_SIZE + 1 + 4 * MAX_MRL_OFFSET] = { 0 };
    Pel filtRef[4 * MAX_CU_SIZE + 1 + 4 * MAX_MRL_OFFSET] = { 0 };

    Pel* pRefVector = &tempRef[predSize];
    Pel* pFilVector = &filtRef[predSize];

    // copy unfiltered ref. samples to line buffer
    const Pel *ptrSrc = refBufUnfiltered;
    for( int j = 0; j <= predSize; j++ ) { pRefVector[ j] = pFilVector[ j] = ptrSrc[j]; }
    for( int i = 1; i <= predSize; i++ ) { pRefVector[-i] = pFilVector[-i] = ptrSrc[i*predStride]; }

    // bilateral filter
    bool strong = (area.width >= INTRA_BILATERAL_STRONG_WIDTH && area.height >= INTRA_BILATERAL_STRONG_HEIGHT);
    const int    numIter = (strong) ? 3 : 1;                          // number of filter iterations
    const int    cSize   = (strong) ? 4 : 3;                          // filtering relative positions [-cSize..cSize]
    CHECK( cSize > 4, "BiFi: lookup-table not defined for this filter size" );

    for( int iter = 0; iter < numIter; iter++ )
    {
      if( iter > 0 ) { std::swap( pRefVector, pFilVector ); }
      for( int k = (cSize - predSize); k <= (predSize - cSize); k++ )
      {
        int sumWI = 0;
        int sumW  = 0;
        for( int dPos = -cSize; dPos <= cSize; dPos++ )
        {
          int absDeltaI = abs( pRefVector[k + dPos] - pRefVector[k] );
          int weight    = m_intraBiFiWeights[std::min<int>( (strong) ? absDeltaI : (4 * absDeltaI), 176 )][abs( dPos )];
          sumWI += weight * pRefVector[k + dPos];
          sumW  += weight;
        }
        pFilVector[k] = static_cast<Pel>( (sumWI + (sumW>>1)) / sumW );
      }
    }

    // copy filtered ref. samples to dest. buffer
    Pel *ptrDst = refBufFiltered;
    for( int j = 0; j <= predSize; j++ ) { ptrDst[j]            = pFilVector[ j]; }
    for( int i = 1; i <= predSize; i++ ) { ptrDst[i*predStride] = pFilVector[-i]; }

    return;
  }

  // Strong intra smoothing
  ChannelType chType = toChannelType( area.compID );
  if( sps.getUseStrongIntraSmoothing() && isLuma( chType ) )
  {
    const Pel bottomLeft = refBufUnfiltered[predStride * predSize];
    const Pel topLeft    = refBufUnfiltered[0];
    const Pel topRight   = refBufUnfiltered[predSize];

    const int  threshold     = 1 << (sps.getBitDepth( chType ) - 5);
    const bool bilinearLeft  = abs( (bottomLeft + topLeft)  - (2 * refBufUnfiltered[predStride * ( tuHeight + mrlOffset )]) ) < threshold; //difference between the
    const bool bilinearAbove = abs( (topLeft    + topRight) - (2 * refBufUnfiltered[             ( tuWidth +  mrlOffset )]) ) < threshold; //ends and the middle

    if( (tuWidth >= 32) && bilinearLeft && bilinearAbove )
    {
      Pel *piDestPtr = refBufFiltered + (predStride * predSize); // bottom left

      // apply strong intra smoothing
      for( UInt i = 0; i < predSize; i++, piDestPtr -= predStride ) //left column (bottom to top)
      {
        *piDestPtr = (((predSize - i) * bottomLeft) + (i * topLeft) + predSize / 2) / predSize;
      }
      for( UInt i = 0; i <= predSize; i++, piDestPtr++ )            //full top row (left-to-right)
      {
        *piDestPtr = (((predSize - i) * topLeft) + (i * topRight) + predSize / 2) / predSize;
      }

      return;
    }
  }

  // Regular reference sample filter
  const Pel *piSrcPtr  = refBufUnfiltered + (predStride * predSize); // bottom left
        Pel *piDestPtr = refBufFiltered   + (predStride * predSize); // bottom left

  // bottom left (not filtered)
  *piDestPtr = *piSrcPtr;
  piDestPtr -= predStride;
  piSrcPtr  -= predStride;
  //left column (bottom to top)
  for( UInt i=1; i < predSize; i++, piDestPtr-=predStride, piSrcPtr-=predStride )
  {
    *piDestPtr = (piSrcPtr[predStride] + 2 * piSrcPtr[0] + piSrcPtr[-predStride] + 2) >> 2;
  }
  //top-left
  *piDestPtr = (piSrcPtr[predStride] + 2 * piSrcPtr[0] + piSrcPtr[1] + 2) >> 2;
  piDestPtr++;
  piSrcPtr++;
  //top row (left-to-right)
  for( UInt i=1; i < predSize; i++, piDestPtr++, piSrcPtr++ )
  {
    *piDestPtr = (piSrcPtr[1] + 2 * piSrcPtr[0] + piSrcPtr[-1] + 2) >> 2;
  }
  // top right (not filtered)
  *piDestPtr=*piSrcPtr;
}

bool IntraPrediction::useFilteredIntraRefSamples( const ComponentID &compID, const PredictionUnit &pu, bool modeSpecific, const UnitArea &tuArea )
{
  const SPS         &sps    = *pu.cs->sps;
  const ChannelType  chType = toChannelType( compID );

  if( ( pu.cu->mode1dPartitions && ( tuArea.blocks[compID].width == 1 || tuArea.blocks[compID].height == 1 ) && canUse1dPartitions( compID ) ) )
  {
    return false;
  }

  // high level conditions
  if( sps.getSpsRangeExtension().getIntraSmoothingDisabledFlag() )                                       { return false; }
  if( !isLuma( chType ) && pu.chromaFormat != CHROMA_444 )                                               { return false; }

  // PDPC related conditions
  if( sps.getSpsNext().isIntraPDPC() && (!(sps.getSpsNext().getUseIntraBiFi()) || pu.cu->pdpc))          { return false; }

  // NSST related conditions
  if( sps.getSpsNext().isPlanarPDPC() && (sps.getSpsNext().getUseNSST() && pu.cu->nsstIdx == 0) && !sps.getSpsNext().getUseIntraBiFi() )
                                                                                                         { return false; }

  // Intra NN related conditions
  if( sps.getSpsNext().getUseIntra_NN() && pu.cu->intra_NN )                                             { return false; }

  if( !modeSpecific )                                                                                    { return true; }

  // pred. mode related conditions
  const int dirMode = PU::getFinalIntraMode( pu, chType );
  if( dirMode == DC_IDX || (sps.getSpsNext().isPlanarPDPC() && dirMode == PLANAR_IDX) )                  { return false; }

  int diff = std::min<int>( abs( dirMode - HOR_IDX ), abs( dirMode - VER_IDX ) );
  if( sps.getSpsNext().getUseIntraBiFi()  && useBilateralFilter( pu.blocks[compID] ) && (diff > 1) )     { return true; }
  if( sps.getSpsNext().isPlanarPDPC() && (sps.getSpsNext().getUseNSST() && pu.cu->nsstIdx == 0) )        { return false; }

  int log2Size = ((g_aucLog2[tuArea.blocks[compID].width] + g_aucLog2[tuArea.blocks[compID].height]) >> 1);
  CHECK( log2Size >= MAX_INTRA_FILTER_DEPTHS, "Size not supported" );
  return (diff > m_aucIntraFilter[chType][log2Size]);
}

bool IntraPrediction::useBilateralFilter( const CompArea &area )
{
  return (isLuma( area.compID ) && area.width >= MINIMUM_INTRA_BILATERAL_WIDTH && area.height >= MINIMUM_INTRA_BILATERAL_HEIGHT);
}

Bool isAboveLeftAvailable(const CodingUnit &cu, const ChannelType &chType, const Position &posLT)
{
  const CodingStructure& cs = *cu.cs;
  const Position refPos = posLT.offset(-1, -1);
  const CodingUnit* pcCUAboveLeft = cs.isDecomp( refPos, chType ) ? cs.getCURestricted( refPos, cu, chType ) : nullptr;
  const Bool isConstrained = cs.pps->getConstrainedIntraPred();
  Bool bAboveLeftFlag;

  if (isConstrained)
  {
    bAboveLeftFlag = pcCUAboveLeft && CU::isIntra(*pcCUAboveLeft);
  }
  else
  {
    bAboveLeftFlag = (pcCUAboveLeft ? true : false);
  }

  return bAboveLeftFlag;
}

Int isAboveAvailable(const CodingUnit &cu, const ChannelType &chType, const Position &posLT, const UInt uiNumUnitsInPU, const UInt unitWidth, Bool *bValidFlags)
{
  const CodingStructure& cs = *cu.cs;
  const Bool isConstrained = cs.pps->getConstrainedIntraPred();
  Bool *pbValidFlags = bValidFlags;
  Int iNumIntra = 0;
  Int maxDx = uiNumUnitsInPU * unitWidth;

  for (UInt dx = 0; dx < maxDx; dx += unitWidth)
  {
    const Position refPos = posLT.offset(dx, -1);

    const CodingUnit* pcCUAbove = cs.isDecomp(refPos, chType) ? cs.getCURestricted(refPos, cu, chType) : nullptr;

    if( pcCUAbove && ( ( isConstrained && CU::isIntra( *pcCUAbove ) ) || !isConstrained ) )
    {
      iNumIntra++;
      *pbValidFlags = true;
    }
    else if( !pcCUAbove )
    {
      return iNumIntra;
    }

    pbValidFlags++;
  }
  return iNumIntra;
}

Int isLeftAvailable(const CodingUnit &cu, const ChannelType &chType, const Position &posLT, const UInt uiNumUnitsInPU, const UInt unitHeight, Bool *bValidFlags)
{
  const CodingStructure& cs = *cu.cs;
  const Bool isConstrained = cs.pps->getConstrainedIntraPred();
  Bool *pbValidFlags = bValidFlags;
  Int iNumIntra = 0;
  Int maxDy = uiNumUnitsInPU * unitHeight;

  for (UInt dy = 0; dy < maxDy; dy += unitHeight)
  {
    const Position refPos = posLT.offset(-1, dy);

    const CodingUnit* pcCULeft = cs.isDecomp(refPos, chType) ? cs.getCURestricted(refPos, cu, chType) : nullptr;

    if( pcCULeft && ( ( isConstrained && CU::isIntra( *pcCULeft ) ) || !isConstrained ) )
    {
      iNumIntra++;
      *pbValidFlags = true;
    }
    else if( !pcCULeft )
    {
      return iNumIntra;
    }

    pbValidFlags--; // opposite direction
  }

  return iNumIntra;
}

Int isAboveRightAvailable(const CodingUnit &cu, const ChannelType &chType, const Position &posRT, const UInt uiNumUnitsInPU, const UInt unitWidth, Bool *bValidFlags )
{
  const CodingStructure& cs = *cu.cs;
  const Bool isConstrained = cs.pps->getConstrainedIntraPred();
  Bool *pbValidFlags = bValidFlags;
  Int iNumIntra = 0;

  UInt maxDx = uiNumUnitsInPU * unitWidth;

  for (UInt dx = 0; dx < maxDx; dx += unitWidth)
  {
    const Position refPos = posRT.offset(unitWidth + dx, -1);

    const CodingUnit* pcCUAbove = cs.isDecomp(refPos, chType) ? cs.getCURestricted(refPos, cu, chType) : nullptr;

    if( pcCUAbove && ( ( isConstrained && CU::isIntra( *pcCUAbove ) ) || !isConstrained ) )
    {
      iNumIntra++;
      *pbValidFlags = true;
    }
    else if( !pcCUAbove )
    {
      return iNumIntra;
    }

    pbValidFlags++;
  }

  return iNumIntra;
}

Int isBelowLeftAvailable(const CodingUnit &cu, const ChannelType &chType, const Position &posLB, const UInt uiNumUnitsInPU, const UInt unitHeight, Bool *bValidFlags )
{
  const CodingStructure& cs = *cu.cs;
  const Bool isConstrained = cs.pps->getConstrainedIntraPred();
  Bool *pbValidFlags = bValidFlags;
  Int iNumIntra = 0;
  Int maxDy = uiNumUnitsInPU * unitHeight;

  for (UInt dy = 0; dy < maxDy; dy += unitHeight)
  {
    const Position refPos = posLB.offset(-1, unitHeight + dy);

    const CodingUnit* pcCULeft = cs.isDecomp(refPos, chType) ? cs.getCURestricted(refPos, cu, chType) : nullptr;

    if( pcCULeft && ( ( isConstrained && CU::isIntra( *pcCULeft ) ) || !isConstrained ) )
    {
      iNumIntra++;
      *pbValidFlags = true;
    }
    else if ( !pcCULeft )
    {
      return iNumIntra;
    }

    pbValidFlags--; // opposite direction
  }

  return iNumIntra;
}

// LumaRecPixels
Void IntraPrediction::xGetLumaRecPixels(const PredictionUnit &pu, CompArea chromaArea)
{
  Int iDstStride = 0;
  Pel* pDst0 = 0;
  int MMLM_Lines = pu.cs->sps->getSpsNext().isELMModeMMLM() ? 2 : 1;
  iDstStride = MAX_CU_SIZE + MMLM_Lines; //MMLM_SAMPLE_NEIGHBOR_LINES;
  pDst0 = m_piTemp + (iDstStride + 1) * MMLM_Lines; //MMLM_SAMPLE_NEIGHBOR_LINES;

  //assert 420 chroma subsampling
  CompArea lumaArea = CompArea( COMPONENT_Y, pu.chromaFormat, chromaArea.lumaPos(), recalcSize( pu.chromaFormat, CHANNEL_TYPE_CHROMA, CHANNEL_TYPE_LUMA, chromaArea.size() ) );//needed for correct pos/size (4x4 Tus)

  Pel *pMulDst0[LM_FILTER_NUM];
  Int  iBufStride = pu.cs->sps->getSpsNext().isELMModeMMLM() ? MAX_CU_SIZE + MMLM_Lines : MAX_CU_SIZE; //MMLM_SAMPLE_NEIGHBOR_LINES
  Pel* pMulDst[LM_FILTER_NUM];
  if (pu.cs->sps->getSpsNext().isELMModeMFLM())
  {
    for (Int i = 0; i < LM_FILTER_NUM; i++)
    {
      pMulDst0[i] = m_pLumaRecBufferMul[i] + (iBufStride + 1) * MMLM_Lines;
    }
  }

  CHECK( lumaArea.width  == chromaArea.width, "" );
  CHECK( lumaArea.height == chromaArea.height, "" );

  const SizeType uiCWidth = chromaArea.width;
  const SizeType uiCHeight = chromaArea.height;

  const CPelBuf Src = pu.cs->picture->getRecoBuf( lumaArea );
  Pel const* pRecSrc0   = Src.bufAt( 0, 0 );
  Int iRecStride        = Src.stride;
  Int iRecStride2       = iRecStride << 1;

  CodingStructure&      cs = *pu.cs;
  const CodingUnit& lumaCU = isChroma( pu.chType ) ? *pu.cs->picture->cs->getCU( lumaArea.pos(), CH_L ) : *pu.cu;
  const CodingUnit&     cu = *pu.cu;

  const CompArea& area = isChroma( pu.chType ) ? chromaArea : lumaArea;

  const SPS &sps = *cs.sps;

  const UInt uiTuWidth  = area.width;
  const UInt uiTuHeight = area.height;

  Int iBaseUnitSize = ( 1 << MIN_CU_LOG2 );

  if( !cs.pcv->rectCUs )
  {
    iBaseUnitSize = sps.getMaxCUWidth() >> sps.getMaxCodingDepth();
  }

  const Int  iUnitWidth       = iBaseUnitSize >> getComponentScaleX( area.compID, area.chromaFormat );
  const Int  iUnitHeight      = iBaseUnitSize >> getComponentScaleX( area.compID, area.chromaFormat );
  const Int  iTUWidthInUnits  = uiTuWidth  / iUnitWidth;
  const Int  iTUHeightInUnits = uiTuHeight / iUnitHeight;
  const Int  iAboveUnits      = iTUWidthInUnits;
  const Int  iLeftUnits       = iTUHeightInUnits;

  Bool  bNeighborFlags[4 * MAX_NUM_PART_IDXS_IN_CTU_WIDTH + 1];

  memset( bNeighborFlags, 0, 1 + iLeftUnits + iAboveUnits );
  Bool bAboveAvaillable, bLeftAvaillable;

  Int availlableUnit = isLeftAvailable( isChroma( pu.chType ) ? cu : lumaCU, toChannelType( area.compID ), area.pos(), iLeftUnits, iUnitHeight, ( bNeighborFlags + iLeftUnits - 1 ) );

  if( lumaCU.cs->pcv->rectCUs )
  {
    bLeftAvaillable = availlableUnit == iTUHeightInUnits;
  }
  else
  {
    bLeftAvaillable = availlableUnit == iTUWidthInUnits;
  }

  availlableUnit = isAboveAvailable( isChroma( pu.chType ) ? cu : lumaCU, toChannelType( area.compID ), area.pos(), iAboveUnits, iUnitWidth, ( bNeighborFlags + iLeftUnits + 1 ) );

  if( lumaCU.cs->pcv->rectCUs )
  {
    bAboveAvaillable = availlableUnit == iTUWidthInUnits;
  }
  else
  {
    bAboveAvaillable = availlableUnit == iTUHeightInUnits;
  }


  Pel*       pDst  = nullptr;
  Pel const* piSrc = nullptr;

  if( bAboveAvaillable )
  {
    pDst  = pDst0    - iDstStride;
    piSrc = pRecSrc0 - iRecStride2;

    for( Int i = 0; i < uiCWidth; i++ )
    {
      if( i == 0 && !bLeftAvaillable )
      {
        pDst[i] = ( piSrc[2 * i] + piSrc[2 * i + iRecStride] + 1 ) >> 1;
      }
      else
      {
        pDst[i] = ( ( ( piSrc[2 * i             ] * 2 ) + piSrc[2 * i - 1             ] + piSrc[2 * i + 1             ] )
                  + ( ( piSrc[2 * i + iRecStride] * 2 ) + piSrc[2 * i - 1 + iRecStride] + piSrc[2 * i + 1 + iRecStride] )
                  + 4 ) >> 3;
      }
    }

    if (pu.cs->sps->getSpsNext().isELMModeMMLM())
    {
      for (Int line = 2; line <= MMLM_Lines; line++)
      {
        pDst  = pDst0    - iDstStride  * line;
        piSrc = pRecSrc0 - iRecStride2 * line;

        for (Int i = 0; i < uiCWidth; i++)
        {
          if (i == 0 && !bLeftAvaillable)
          {
            pDst[i] = (piSrc[2 * i] + piSrc[2 * i + iRecStride] + 1) >> 1;
          }
          else
          {
            pDst[i] = ( ( (piSrc[2 * i             ] * 2 ) + piSrc[2 * i - 1             ] + piSrc[2 * i + 1             ] )
                      + ( (piSrc[2 * i + iRecStride] * 2 ) + piSrc[2 * i - 1 + iRecStride] + piSrc[2 * i + 1 + iRecStride] )
                      + 4 ) >> 3;
          }
        }
      }
    }

    if (pu.cs->sps->getSpsNext().isELMModeMFLM())
    {
      for (Int i = 0; i < LM_FILTER_NUM; i++)
      {
        pMulDst[i] = pMulDst0[i] - iDstStride;
      }

      piSrc = pRecSrc0 - iRecStride2;

      for (Int i = 0; i < uiCWidth; i++)
      {

        xFilterGroup(pMulDst, i, &piSrc[2 * i], iRecStride, bAboveAvaillable, i != 0 || bLeftAvaillable);
      }

      if (pu.cs->sps->getSpsNext().isELMModeMMLM())
      {
        for (int line = 2; line <= MMLM_Lines; line++)

        {
          for (Int i = 0; i < LM_FILTER_NUM; i++)
          {
            pMulDst[i] = pMulDst0[i] - iDstStride * line;
          }

          piSrc = pRecSrc0 - iRecStride2 * line;

          for (Int i = 0; i < uiCWidth; i++)
          {
            xFilterGroup(pMulDst, i, &piSrc[2 * i], iRecStride, bAboveAvaillable, i != 0 || bLeftAvaillable);
          }
        }
      }
    }
  }

  if( bLeftAvaillable )
  {
    pDst  = pDst0    - 1;
    piSrc = pRecSrc0 - 3;

    for( Int j = 0; j < uiCHeight; j++ )
    {
      pDst[0] = ( ( piSrc[1             ] * 2 + piSrc[0         ] + piSrc[2             ] )
                + ( piSrc[1 + iRecStride] * 2 + piSrc[iRecStride] + piSrc[2 + iRecStride] )
                + 4 ) >> 3;

      piSrc += iRecStride2;
      pDst  += iDstStride;
    }

    if (pu.cs->sps->getSpsNext().isELMModeMMLM())
    {
      for (Int line = 2; line <= MMLM_Lines; line++)
      {
        pDst  = pDst0    - line;
        piSrc = pRecSrc0 - 2 * line - 1;

        for (Int j = 0; j < uiCHeight; j++)
        {
          pDst[0] = ( ( piSrc[1             ] * 3 + piSrc[2             ] )
                    + ( piSrc[1 + iRecStride] * 3 + piSrc[2 + iRecStride] )
                    + 4 ) >> 3;
          piSrc += iRecStride2;
          pDst  += iDstStride;
        }
      }
    }

    if (pu.cs->sps->getSpsNext().isELMModeMFLM())
    {
      for (Int i = 0; i < LM_FILTER_NUM; i++)
      {
        pMulDst[i] = pMulDst0[i] - 1;
      }

      piSrc = pRecSrc0 - 2;
      for (Int j = 0; j < uiCHeight; j++)
      {
        //Filter group 1

        xFilterGroup(pMulDst, 0, piSrc, iRecStride, j != 0 || bAboveAvaillable, bLeftAvaillable);

        piSrc += iRecStride2;

        for (Int i = 0; i < LM_FILTER_NUM; i++)
        {
          pMulDst[i] += iDstStride;
        }
      }

      if (pu.cs->sps->getSpsNext().isELMModeMMLM())
      {
        for (int line = 2; line <= MMLM_Lines; line++)
        {
          for (Int i = 0; i < LM_FILTER_NUM; i++)
          {
            pMulDst[i] = pMulDst0[i] - line;
          }
          piSrc = pRecSrc0 - 2 * line;

          for (Int j = 0; j < uiCHeight; j++)
          {

            xFilterGroup(pMulDst, 0, piSrc, iRecStride, j != 0 || bAboveAvaillable, bLeftAvaillable);

            piSrc += iRecStride2;

            for (Int i = 0; i < LM_FILTER_NUM; i++)
            {
              pMulDst[i] += iDstStride;
            }
          }
        }
      }
    }
  }


  // inner part from reconstructed picture buffer
  for( Int j = 0; j < uiCHeight; j++ )
  {
    for( Int i = 0; i < uiCWidth; i++ )
    {
      if( i == 0 && !bLeftAvaillable )
      {
        pDst0[i] = ( pRecSrc0[2 * i] + pRecSrc0[2 * i + iRecStride] + 1 ) >> 1;
      }
      else
      {
        pDst0[i] = ( pRecSrc0[2 * i             ] * 2 + pRecSrc0[2 * i + 1             ] + pRecSrc0[2 * i - 1             ]
                   + pRecSrc0[2 * i + iRecStride] * 2 + pRecSrc0[2 * i + 1 + iRecStride] + pRecSrc0[2 * i - 1 + iRecStride]
                   + 4 ) >> 3;
      }
    }

    pDst0    += iDstStride;
    pRecSrc0 += iRecStride2;
  }

  if (pu.cs->sps->getSpsNext().isELMModeMFLM())
  {
    pRecSrc0 = Src.bufAt(0, 0);

    for (Int j = 0; j < uiCHeight; j++)
    {
      for (Int i = 0; i < uiCWidth; i++)
      {
        xFilterGroup(pMulDst0, i, &pRecSrc0[2 * i], iRecStride, j != 0 || bAboveAvaillable, i != 0 || bLeftAvaillable);
      }
      for (Int i = 0; i < LM_FILTER_NUM; i++)
      {
        pMulDst0[i] += iDstStride;
      }

      pRecSrc0 += iRecStride2;
    }
  }
}

static int GetFloorLog2( unsigned x )
{
  int bits = -1;
  while( x > 0 )
  {
    bits++;
    x >>= 1;
  }
  return bits;
}

Int IntraPrediction::xCalcLMParametersGeneralized(Int x, Int y, Int xx, Int xy, Int count, Int bitDepth, Int &a, Int &b, Int &iShift)
{

  UInt uiInternalBitDepth = bitDepth;
  if (count == 0)
  {
    a = 0;
    b = 1 << (uiInternalBitDepth - 1);
    iShift = 0;
    return -1;
  }
  CHECK(count > 512, "");

  Int avgX = (x * g_aiLMDivTableLow[count - 1] + 32768) >> 16;
  Int avgY = (y * g_aiLMDivTableLow[count - 1] + 32768) >> 16;
  avgX = (x * g_aiLMDivTableHigh[count - 1] + avgX) >> 16;
  avgY = (y * g_aiLMDivTableHigh[count - 1] + avgY) >> 16;


  Int RErrX = x - avgX * count;
  Int RErrY = y - avgY * count;

  Int iB = 7;
  iShift = 13 - iB;

  {
    Int a1 = xy - (avgX*avgY * count) - avgX*RErrY - avgY*RErrX;
    Int a2 = xx - (avgX*avgX * count) - 2 * avgX*RErrX;

    const Int iShiftA1 = uiInternalBitDepth - 2;
    const Int iShiftA2 = 5;
    const Int iAccuracyShift = uiInternalBitDepth + 4;

    Int iScaleShiftA2 = 0;
    Int iScaleShiftA1 = 0;
    Int a1s = a1;
    Int a2s = a2;

    iScaleShiftA1 = a1 == 0 ? 0 : GetFloorLog2(abs(a1)) - iShiftA1;
    iScaleShiftA2 = a2 == 0 ? 0 : GetFloorLog2(abs(a2)) - iShiftA2;

    if (iScaleShiftA1 < 0)
    {
      iScaleShiftA1 = 0;
    }

    if (iScaleShiftA2 < 0)
    {
      iScaleShiftA2 = 0;
    }

    Int iScaleShiftA = iScaleShiftA2 + iAccuracyShift - iShift - iScaleShiftA1;

    a2s = a2 >> iScaleShiftA2;

    a1s = a1 >> iScaleShiftA1;

    if (a2s >= 32)
    {
      UInt a2t = m_auShiftLM[a2s - 32];
      a = a1s * a2t;
    }
    else
    {
      a = 0;
    }

    if (iScaleShiftA < 0)
    {
      a = a << -iScaleShiftA;
    }
    else
    {
      a = a >> iScaleShiftA;
    }
    a = Clip3(-(1 << (15 - iB)), (1 << (15 - iB)) - 1, a);
    a = a << iB;

    Short n = 0;
    if (a != 0)
    {
      n = GetFloorLog2(abs(a) + ((a < 0 ? -1 : 1) - 1) / 2) - 5;
    }

    iShift = (iShift + iB) - n;
    a = a >> n;

    b = avgY - ((a * avgX) >> iShift);

    return 0;
  }
}


Int IntraPrediction::xLMSampleClassifiedTraining(Int count, Int LumaSamples[], Int ChrmSamples[], Int GroupNum, Int bitDepth, MMLM_parameter parameters[])
{

  //Initialize

  for (Int i = 0; i < GroupNum; i++)
  {
    parameters[i].Inf = 0;
    parameters[i].Sup = (1 << bitDepth) - 1;
    parameters[i].a = 0;
    parameters[i].b = 1 << (bitDepth - 1);
    parameters[i].shift = 0;
  }

  if (count < 4)//
  {
    return -1;
  }

  Int GroupTag[1024];

  Int GroupCount[3] = { 0, 0, 0 };

  Int mean = 0;
  Int meanC = 0;

  Int iMaxLuma = -1;
  Int iMinLuma = 0xffffff;
  for (int i = 0; i < count; i++)
  {
    mean += LumaSamples[i];
    meanC += ChrmSamples[i];
    if (LumaSamples[i] < iMinLuma)
    {
      iMinLuma = LumaSamples[i];
    }
    if (LumaSamples[i] > iMaxLuma)
    {
      iMaxLuma = LumaSamples[i];
    }
  }

  CHECK(count > 512, "");

  Int meand = (mean  * g_aiLMDivTableLow[count - 1] + 32768) >> 16;
  Int meanCd = (meanC * g_aiLMDivTableLow[count - 1] + 32768) >> 16;
  mean = (mean  * g_aiLMDivTableHigh[count - 1] + meand + 32768) >> 16;
  meanC = (meanC * g_aiLMDivTableHigh[count - 1] + meanCd + 32768) >> 16;


  Int meanDiff = meanC - mean;

  mean = std::max(1, mean);

  Int iTh[2] = { 0, 0 };

  if (GroupNum == 2)
  {
    iTh[0] = mean;

    parameters[0].Inf = 0;
    parameters[0].Sup = mean - 1;

    parameters[1].Inf = mean;
    parameters[1].Sup = (1 << bitDepth) - 1;

  }
  else if (GroupNum == 3)
  {
    iTh[0] = std::max(iMinLuma + 1, (iMinLuma + mean + 1) >> 1);
    iTh[1] = std::min(iMaxLuma - 1, (iMaxLuma + mean + 1) >> 1);

    parameters[0].Inf = 0;
    parameters[0].Sup = iTh[0] - 1;

    parameters[1].Inf = iTh[0];
    parameters[1].Sup = iTh[1] - 1;

    parameters[2].Inf = iTh[1];
    parameters[2].Sup = (1 << bitDepth) - 1;
  }
  else
  {
    CHECK(1, "");
  }
  for (Int i = 0; i < count; i++)
  {
    if (LumaSamples[i] < iTh[0])
    {
      GroupTag[i] = 0;
      GroupCount[0]++;
    }
    else if (LumaSamples[i] < iTh[1] || GroupNum == 2)
    {
      GroupTag[i] = 1;
      GroupCount[1]++;
    }
    else
    {
      GroupTag[i] = 2;
      GroupCount[2]++;
    }
  }
  Int iBiggestGroup = 0;
  for (Int i = 1; i < GroupNum; i++)
  {
    if (GroupCount[i] > iBiggestGroup)
    {
      iBiggestGroup = i;
    }
  }

  for (int group = 0; group < GroupNum; group++)
  {
    // If there is only 1 sample in a group, add the nearest value of the two neighboring pixels to the group.
    if (GroupCount[group] < 2)
    {
      for (int i = 0; i < count; i++)
      {
        if (GroupTag[i] == group)
        {
          for (Int k = 1; (i + k < count) || (i - k >= 0); k++)
          {
            if (i + k < count && GroupTag[i + k] == iBiggestGroup)
            {
              GroupTag[i + k] = group;
              GroupCount[group]++;
              GroupCount[iBiggestGroup]--;
              break;
            }
            if (i - k >= 0 && GroupTag[i - k] == iBiggestGroup)
            {
              GroupTag[i - k] = group;
              GroupCount[group]++;
              GroupCount[iBiggestGroup]--;
              break;
            }
          }
          break;
        }
      }
    }
  }


  Int x[3], y[3], xy[3], xx[3];
  for (Int group = 0; group < GroupNum; group++)
  {
    x[group] = y[group] = xy[group] = xx[group] = 0;
  }
  for (Int i = 0; i < count; i++)
  {
    Int group = GroupTag[i];
    x[group] += LumaSamples[i];
    y[group] += ChrmSamples[i];
    xx[group] += LumaSamples[i] * LumaSamples[i];
    xy[group] += LumaSamples[i] * ChrmSamples[i];
  }

  for (Int group = 0; group < GroupNum; group++)
  {
    Int a, b, iShift;
    if (GroupCount[group] > 1)
    {
      xCalcLMParametersGeneralized(x[group], y[group], xx[group], xy[group], GroupCount[group], bitDepth, a, b, iShift);

      parameters[group].a = a;
      parameters[group].b = b;
      parameters[group].shift = iShift;
    }
    else
    {
      parameters[group].a = 0;
      parameters[group].b = meanDiff;
      parameters[group].shift = 0;
    }
  }
  return 0;
}

Int IntraPrediction::xGetMMLMParameters(const PredictionUnit& pu, const ComponentID compID, const CompArea& chromaArea, Int &numClass, MMLM_parameter parameters[])
{
  CHECK(compID == COMPONENT_Y, "");

  const SizeType uiCWidth = chromaArea.width;
  const SizeType uiCHeight = chromaArea.height;

  const Position posLT = chromaArea;

  CodingStructure&  cs = *pu.cs;
  const CodingUnit& cu = *pu.cu;
  Bool bQTBT = cu.slice->getSPS()->getSpsNext().getUseQTBT();

  const SPS &sps = *cs.sps;
  const UInt uiTuWidth = chromaArea.width;
  const UInt uiTuHeight = chromaArea.height;
  const ChromaFormat nChromaFormat = sps.getChromaFormatIdc();

  const Int iBaseUnitSize = 1 << MIN_CU_LOG2;
  const Int  iUnitWidth = iBaseUnitSize >> getComponentScaleX(chromaArea.compID, nChromaFormat);
  const Int  iUnitHeight = iBaseUnitSize >> getComponentScaleX(chromaArea.compID, nChromaFormat);


  const Int  iTUWidthInUnits = uiTuWidth / iUnitWidth;
  const Int  iTUHeightInUnits = uiTuHeight / iUnitHeight;
  const Int  iAboveUnits = iTUWidthInUnits;
  const Int  iLeftUnits = iTUHeightInUnits;

  Bool  bNeighborFlags[4 * MAX_NUM_PART_IDXS_IN_CTU_WIDTH + 1];

  memset(bNeighborFlags, 0, 1 + iLeftUnits + iAboveUnits);

  Bool bAboveAvaillable, bLeftAvaillable;

  Int availlableUnit = isAboveAvailable(cu, CHANNEL_TYPE_CHROMA, posLT, iAboveUnits, iUnitWidth, (bNeighborFlags + iLeftUnits + 1));

  if (bQTBT)
  {
    bAboveAvaillable = availlableUnit == iTUWidthInUnits;
  }
  else
  {
    bAboveAvaillable = availlableUnit == iTUWidthInUnits;
  }

  availlableUnit = isLeftAvailable(cu, CHANNEL_TYPE_CHROMA, posLT, iLeftUnits, iUnitHeight, (bNeighborFlags + iLeftUnits - 1));

  if (bQTBT)
  {
    bLeftAvaillable = availlableUnit == iTUHeightInUnits;
  }
  else
  {
    bLeftAvaillable = availlableUnit == iTUHeightInUnits;
  }


  Pel *pSrcColor0, *pCurChroma0;
  Int  iSrcStride, iCurStride;

  UInt uiInternalBitDepth = sps.getBitDepth(CHANNEL_TYPE_CHROMA);

  int MMLM_Lines = pu.cs->sps->getSpsNext().isELMModeMMLM() ? 2 : 1;
  iSrcStride = (MAX_CU_SIZE)+MMLM_Lines;
  PelBuf Temp = PelBuf(m_piTemp + (iSrcStride + 1) * MMLM_Lines, iSrcStride, Size(chromaArea));

  iSrcStride = Temp.stride;
  pSrcColor0 = Temp.bufAt(0, 0);

  PelBuf recoC = pu.cs->picture->getRecoBuf(chromaArea);
  iCurStride = recoC.stride;
  pCurChroma0 = recoC.bufAt(0, 0);


  Int count = 0;
  Int LumaSamples[512];
  Int ChrmSamples[512];


  Int i, j;

  Pel *pSrc = pSrcColor0 - iSrcStride;
  Pel *pCur = pCurChroma0 - iCurStride;

  Bool bAdditionalLine = true;

  if (bAboveAvaillable)
  {
    for (j = 0; j < uiCWidth; j++)
    {
      LumaSamples[count] = pSrc[j];
      ChrmSamples[count] = pCur[j];
      count++;
    }
    if (bAdditionalLine)
    {
      for (int line = 2; line <= MMLM_Lines; line++)
      {
        pSrc = pSrcColor0 - line * iSrcStride;
        pCur = pCurChroma0 - line * iCurStride;

        for (j = 0; j < uiCWidth; j++)
        {
          LumaSamples[count] = pSrc[j];
          ChrmSamples[count] = pCur[j];
          count++;
        }
      }
    }
  }

  if (bLeftAvaillable)
  {
    pSrc = pSrcColor0 - 1;
    pCur = pCurChroma0 - 1;

    for (i = 0; i < uiCHeight; i++)
    {
      LumaSamples[count] = pSrc[0];
      ChrmSamples[count] = pCur[0];
      count++;

      pSrc += iSrcStride;
      pCur += iCurStride;
    }

    if (bAdditionalLine)
    {
      for (int line = 2; line <= MMLM_Lines; line++)
      {
        pSrc = pSrcColor0 - line;
        pCur = pCurChroma0 - line;

        for (i = 0; i < uiCHeight; i++)
        {
          LumaSamples[count] = pSrc[0];
          ChrmSamples[count] = pCur[0];
          count++;

          pSrc += iSrcStride;
          pCur += iCurStride;
        }
      }
    }
  }

  xLMSampleClassifiedTraining(count, LumaSamples, ChrmSamples, numClass, uiInternalBitDepth, parameters);
  return 2;
}

Void IntraPrediction::xGetLMParameters(const PredictionUnit &pu, const ComponentID compID, const CompArea& chromaArea, Int iPredType, Int& a, Int&  b, Int& iShift)
{
  CHECK( compID == COMPONENT_Y, "" );

  const SizeType uiCWidth  = chromaArea.width;
  const SizeType uiCHeight = chromaArea.height;

  const Position posLT = chromaArea;

  CodingStructure&  cs = *(pu.cs);
  const CodingUnit& cu = *(pu.cu);

  const SPS &sps        = *cs.sps;
  const UInt uiTuWidth  = chromaArea.width;
  const UInt uiTuHeight = chromaArea.height;
  const ChromaFormat nChromaFormat = sps.getChromaFormatIdc();

  const Int iBaseUnitSize = 1 << MIN_CU_LOG2;
  const Int  iUnitWidth   = iBaseUnitSize >> getComponentScaleX( chromaArea.compID, nChromaFormat );
  const Int  iUnitHeight  = iBaseUnitSize >> getComponentScaleX( chromaArea.compID, nChromaFormat );


  const Int  iTUWidthInUnits  = uiTuWidth / iUnitWidth;
  const Int  iTUHeightInUnits = uiTuHeight / iUnitHeight;
  const Int  iAboveUnits      = iTUWidthInUnits;
  const Int  iLeftUnits       = iTUHeightInUnits;

  Bool  bNeighborFlags[4 * MAX_NUM_PART_IDXS_IN_CTU_WIDTH + 1];

  memset( bNeighborFlags, 0, 1 + iLeftUnits + iAboveUnits );

  Bool bAboveAvaillable, bLeftAvaillable;

  Int availlableUnit = isAboveAvailable( cu, CHANNEL_TYPE_CHROMA, posLT, iAboveUnits, iUnitWidth, ( bNeighborFlags + iLeftUnits + 1 ) );
  bAboveAvaillable = availlableUnit == iTUWidthInUnits;

  availlableUnit = isLeftAvailable( cu, CHANNEL_TYPE_CHROMA, posLT, iLeftUnits, iUnitHeight, ( bNeighborFlags + iLeftUnits - 1 ) );
  bLeftAvaillable = availlableUnit == iTUHeightInUnits;

  Pel *pSrcColor0, *pCurChroma0;
  Int  iSrcStride,  iCurStride;

  if( iPredType == 0 ) //chroma from luma
  {
    PelBuf Temp;
    int MMLM_Lines = pu.cs->sps->getSpsNext().isELMModeMMLM() ? 2 : 1;
    iSrcStride = MAX_CU_SIZE + MMLM_Lines; //MMLM_SAMPLE_NEIGHBOR_LINES;
    Temp = PelBuf(m_piTemp + (iSrcStride + 1) * MMLM_Lines, iSrcStride, Size(chromaArea)); //MMLM_SAMPLE_NEIGHBOR_LINES
    pSrcColor0 = Temp.bufAt(0, 0);
    pCurChroma0   = getPredictorPtr( compID, false );
    iCurStride    = uiCWidth + uiCHeight + 1;
    pCurChroma0  += iCurStride + 1;
  }
  else
  {
    CHECK( !( compID == COMPONENT_Cr ), "called for incorrect color channel" );

    pSrcColor0   = getPredictorPtr( COMPONENT_Cb, false );
    pCurChroma0  = getPredictorPtr( COMPONENT_Cr, false );

    iSrcStride   = ( uiCWidth + uiCHeight + 1 );
    iCurStride   = iSrcStride;

    pSrcColor0  += iSrcStride + 1;
    pCurChroma0 += iCurStride + 1;
  }

  int x = 0, y = 0, xx = 0, xy = 0;
  int iCountShift = 0;
  unsigned uiInternalBitDepth = sps.getBitDepth( CHANNEL_TYPE_CHROMA );

  Pel *pSrc = pSrcColor0  - iSrcStride;
  Pel *pCur = pCurChroma0 - iCurStride;

  int       minDim        = bLeftAvaillable && bAboveAvaillable ? 1 << g_aucPrevLog2[std::min( uiCHeight, uiCWidth )] : 1 << g_aucPrevLog2[bLeftAvaillable ? uiCHeight : uiCWidth];
  int       minStep       = 1;
  int       numSteps      = cs.pcv->rectCUs ? minDim / minStep : minDim;

  if( bAboveAvaillable )
  {
    for( int j = 0; j < numSteps; j++ )
    {
      int idx = ( j * minStep * uiCWidth ) / minDim;

      x  += pSrc[idx];
      y  += pCur[idx];
      xx += pSrc[idx] * pSrc[idx];
      xy += pSrc[idx] * pCur[idx];
    }

    iCountShift = g_aucLog2[minDim / minStep];
  }

  if( bLeftAvaillable )
  {
    pSrc = pSrcColor0  - 1;
    pCur = pCurChroma0 - 1;

    for( int i = 0; i < numSteps; i++ )
    {
      int idx = ( i * uiCHeight * minStep ) / minDim;
      x  += pSrc[iSrcStride * idx];
      y  += pCur[iCurStride * idx];
      xx += pSrc[iSrcStride * idx] * pSrc[iSrcStride * idx];
      xy += pSrc[iSrcStride * idx] * pCur[iCurStride * idx];
    }

    iCountShift += bAboveAvaillable ? 1 : g_aucLog2[minDim / minStep];
  }

  if( !bLeftAvaillable && !bAboveAvaillable )
  {
    a = 0;
    if( iPredType == 0 )
    {
      b = 1 << ( uiInternalBitDepth - 1 );
    }
    else
    {
      b = 0;
    }
    iShift = 0;
    return;
  }

  Int iTempShift = uiInternalBitDepth + iCountShift - 15;

  if( iTempShift > 0 )
  {
    x  = ( x  + ( 1 << ( iTempShift - 1 ) ) ) >> iTempShift;
    y  = ( y  + ( 1 << ( iTempShift - 1 ) ) ) >> iTempShift;
    xx = ( xx + ( 1 << ( iTempShift - 1 ) ) ) >> iTempShift;
    xy = ( xy + ( 1 << ( iTempShift - 1 ) ) ) >> iTempShift;
    iCountShift -= iTempShift;
  }


  /////// xCalcLMParameters

  Int avgX = x >> iCountShift;
  Int avgY = y >> iCountShift;

  Int RErrX = x & ( ( 1 << iCountShift ) - 1 );
  Int RErrY = y & ( ( 1 << iCountShift ) - 1 );

  Int iB = 7;
  iShift = 13 - iB;


  if( iCountShift == 0 )
  {
    a = 0;
    b = 1 << ( uiInternalBitDepth - 1 );
    iShift = 0;
  }
  else
  {
    Int a1 = xy - ( avgX * avgY << iCountShift ) -     avgX * RErrY - avgY * RErrX;
    Int a2 = xx - ( avgX * avgX << iCountShift ) - 2 * avgX * RErrX;

    if( iPredType == 1 ) // Cr residual predicted from Cb residual, Cr from Cb
    {
      a1 += -1 * ( xx >> ( CR_FROM_CB_REG_COST_SHIFT + 1 ) );
      a2 +=        xx >>   CR_FROM_CB_REG_COST_SHIFT;
    }

    const Int iShiftA1 = uiInternalBitDepth - 2;
    const Int iShiftA2 = 5;
    const Int iAccuracyShift = uiInternalBitDepth + 4;

    Int iScaleShiftA2 = 0;
    Int iScaleShiftA1 = 0;
    Int a1s = a1;
    Int a2s = a2;

    iScaleShiftA1 = a1 == 0 ? 0 : GetFloorLog2( abs( a1 ) ) - iShiftA1;
    iScaleShiftA2 = a2 == 0 ? 0 : GetFloorLog2( abs( a2 ) ) - iShiftA2;

    if( iScaleShiftA1 < 0 )
    {
      iScaleShiftA1 = 0;
    }

    if( iScaleShiftA2 < 0 )
    {
      iScaleShiftA2 = 0;
    }

    Int iScaleShiftA = iScaleShiftA2 + iAccuracyShift - iShift - iScaleShiftA1;

    a2s = a2 >> iScaleShiftA2;

    a1s = a1 >> iScaleShiftA1;

    if( a2s >= 32 )
    {
      UInt a2t = m_auShiftLM[a2s - 32];
      a = a1s * a2t;
    }
    else
    {
      a = 0;
    }

    if( iScaleShiftA < 0 )
    {
      a = a << -iScaleShiftA;
    }
    else
    {
      a = a >> iScaleShiftA;
    }
    a = Clip3( -( 1 << ( 15 - iB ) ), ( 1 << ( 15 - iB ) ) - 1, a );
    a = a << iB;

    Short n = 0;
    if( a != 0 )
    {
      n = GetFloorLog2( abs( a ) + ( ( a < 0 ? -1 : 1 ) - 1 ) / 2 ) - 5;
    }

    iShift = ( iShift + iB ) - n;
    a = a >> n;

    b = avgY - ( ( a * avgX ) >> iShift );
  }
}


//! \}
