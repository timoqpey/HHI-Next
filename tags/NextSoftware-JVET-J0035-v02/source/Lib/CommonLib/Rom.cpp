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

/** \file     Rom.cpp
    \brief    global variables & functions
*/

#include "Rom.h"
#include "UnitTools.h"

#include <memory.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <iomanip>

// ====================================================================================================================
// Initialize / destroy functions
// ====================================================================================================================

#if ENABLE_TRACING
CDTrace *g_trace_ctx = NULL;
#endif


//! \ingroup CommonLib
//! \{

MsgLevel g_verbosity = VERBOSE;

const TChar* nalUnitTypeToString(NalUnitType type)
{
  switch (type)
  {
  case NAL_UNIT_CODED_SLICE_TRAIL_R:    return "TRAIL_R";
  case NAL_UNIT_CODED_SLICE_TRAIL_N:    return "TRAIL_N";
  case NAL_UNIT_CODED_SLICE_TSA_R:      return "TSA_R";
  case NAL_UNIT_CODED_SLICE_TSA_N:      return "TSA_N";
  case NAL_UNIT_CODED_SLICE_STSA_R:     return "STSA_R";
  case NAL_UNIT_CODED_SLICE_STSA_N:     return "STSA_N";
  case NAL_UNIT_CODED_SLICE_BLA_W_LP:   return "BLA_W_LP";
  case NAL_UNIT_CODED_SLICE_BLA_W_RADL: return "BLA_W_RADL";
  case NAL_UNIT_CODED_SLICE_BLA_N_LP:   return "BLA_N_LP";
  case NAL_UNIT_CODED_SLICE_IDR_W_RADL: return "IDR_W_RADL";
  case NAL_UNIT_CODED_SLICE_IDR_N_LP:   return "IDR_N_LP";
  case NAL_UNIT_CODED_SLICE_CRA:        return "CRA";
  case NAL_UNIT_CODED_SLICE_RADL_R:     return "RADL_R";
  case NAL_UNIT_CODED_SLICE_RADL_N:     return "RADL_N";
  case NAL_UNIT_CODED_SLICE_RASL_R:     return "RASL_R";
  case NAL_UNIT_CODED_SLICE_RASL_N:     return "RASL_N";
  case NAL_UNIT_VPS:                    return "VPS";
  case NAL_UNIT_SPS:                    return "SPS";
  case NAL_UNIT_PPS:                    return "PPS";
  case NAL_UNIT_ACCESS_UNIT_DELIMITER:  return "AUD";
  case NAL_UNIT_EOS:                    return "EOS";
  case NAL_UNIT_EOB:                    return "EOB";
  case NAL_UNIT_FILLER_DATA:            return "FILLER";
  case NAL_UNIT_PREFIX_SEI:             return "Prefix SEI";
  case NAL_UNIT_SUFFIX_SEI:             return "Suffix SEI";
  default:                              return "UNK";
  }
}

class ScanGenerator
{
private:
  UInt m_line, m_column;
  const UInt m_blockWidth, m_blockHeight;
  const UInt m_stride;
  const CoeffScanType m_scanType;

public:
  ScanGenerator(UInt blockWidth, UInt blockHeight, UInt stride, CoeffScanType scanType)
    : m_line(0), m_column(0), m_blockWidth(blockWidth), m_blockHeight(blockHeight), m_stride(stride), m_scanType(scanType)
  { }

  UInt GetCurrentX() const { return m_column; }
  UInt GetCurrentY() const { return m_line; }

  UInt GetNextIndex(UInt blockOffsetX, UInt blockOffsetY)
  {
    const UInt rtn = ((m_line + blockOffsetY) * m_stride) + m_column + blockOffsetX;

    //advance line and column to the next position
    switch (m_scanType)
    {
      //------------------------------------------------

      case SCAN_DIAG:

        if ((m_column == m_blockWidth - 1) || (m_line == 0)) //if we reach the end of a rank, go diagonally down to the next one
        {
          m_line += m_column + 1;
          m_column = 0;

          if (m_line >= m_blockHeight) //if that takes us outside the block, adjust so that we are back on the bottom row
          {
            m_column += m_line - (m_blockHeight - 1);
            m_line = m_blockHeight - 1;
          }
        }
        else
        {
          m_column++;
          m_line--;
        }
        break;

      //------------------------------------------------

      case SCAN_HOR:

        if (m_column == m_blockWidth - 1)
        {
          m_line++;
          m_column = 0;
        }
        else
        {
          m_column++;
        }
        break;

      //------------------------------------------------

      case SCAN_VER:

        if (m_line == m_blockHeight - 1)
        {
          m_column++;
          m_line = 0;
        }
        else
        {
          m_line++;
        }
        break;

      //------------------------------------------------

      default:

        THROW("ERROR: Unknown scan type \"" << m_scanType << "\"in ScanGenerator::GetNextIndex");
        break;
    }

    return rtn;
  }
};

// initialize ROM variables
Void initROM()
{
  Int i, c;

#if RExt__HIGH_BIT_DEPTH_SUPPORT
  {
    c = 64;
    const Double s = sqrt((Double)c) * (64 << COM16_C806_TRANS_PREC);


    for (Int k = 0; k < c; k++)
    {
      for (Int n = 0; n < c; n++)
      {
        Double w0, v;
        const Double PI = 3.14159265358979323846;

        // DCT-II
        w0 = k == 0 ? sqrt(0.5) : 1;
        v = cos(PI*(n + 0.5)*k / c) * w0 * sqrt(2.0 / c);
        short sv = (short)(s * v + (v > 0 ? 0.5 : -0.5));
        if (g_aiT64[0][0][c*c + k*c + n] != sv)
        {
          msg(WARNING, "trap");
        }
      }
    }
  }
#endif





  // g_aucConvertToBit[ x ]: log2(x/4), if x=4 -> 0, x=8 -> 1, x=16 -> 2, ...
  // g_aucLog2[ x ]: log2(x), if x=1 -> 0, x=2 -> 1, x=4 -> 2, x=8 -> 3, x=16 -> 4, ...
  ::memset(g_aucLog2, 0, sizeof(g_aucLog2));
  // Explanation for g_aucLog2ForNonPowerOfSizes[ x ]
  // 1) if x is a power of 2 -> g_aucLog2ForNonPowerOfSizes[ x ] = 0
  // 2) if x is not a power of 2 -> x=K*p, where K is a power of 2 and p is not. Then g_aucLog2ForNonPowerOfSizes[ x ] = ceil(log2(p))
  ::memset(g_aucCeilOfLog2OfNonPowerOf2Part, 0, sizeof(g_aucCeilOfLog2OfNonPowerOf2Part));
  // Explanation for g_aucLog2OfPowerOf2Part[ x ]
  // 1) if x is a power of 2 -> g_aucLog2ForNonPowerOfSizes[ x ] = log2(x)
  // 2) if x is not a power of 2 -> x=K*p, where K is a power of 2 and p is not. Then g_aucLog2ForNonPowerOfSizes[ x ] = log2(K)
  ::memset(g_aucLog2OfPowerOf2Part, 0, sizeof(g_aucLog2OfPowerOf2Part));
  c = 0;
  for( int i = 0, n = 0; i <= MAX_CU_SIZE; i++ )
  {
    g_aucNextLog2[i] = i <= 1 ? 0 : c + 1;

    if( i == ( 1 << n ) )
    {
      c = n;
      n++;
    }

    g_aucPrevLog2[i] = c;
    g_aucLog2    [i] = c;
    g_aucLog2OfPowerOf2Part[i] = c;
  }

  c = 2; //for the 2x2 transforms if QTBT is on

  const Double PI = 3.14159265358979323846;

  gp_sizeIdxInfo = new SizeIndexInfoWithNonLog2();
  gp_sizeIdxInfo->init(MAX_CU_SIZE);
  const int numberOfDifferentSizes = 16;//gp_sizeIdxInfo->numAllHeights();
  UInt lengths[numberOfDifferentSizes] = { 2, 4, 6, 8, 10, 12, 16, 20, 24, 32, 40, 48, 64, 80, 96, 128 };
  UInt nonPowerOf2FactorsOfLengths[numberOfDifferentSizes] = { 1, 1, 3, 1,  5,  3,  1,  5,  3,  1,  5,  3,  1,  5,  3,   1 };
  UInt lengthsPowerOf2Factor[numberOfDifferentSizes];
  for (int i = 0; i < numberOfDifferentSizes; i++)
  {
    lengthsPowerOf2Factor[i] = lengths[i] / nonPowerOf2FactorsOfLengths[i];
  }

  for (i = 0; i < numberOfDifferentSizes; i++)
  {
    //Parameters of the scaling (to make it easier to understand, in this explanation COM16_C806_TRANS_PREC is ignored)
    //N => size of the 1-D transform
    //N=K*p, where K is a power of 2 and p is a prime number. For example, for N=12 -> K=4 and p=3
    //Scaling for a power of 2 => 2^(6+0.5*log2(N))=sqrt(N)*64
    //If N is not a power of 2, then we need to change the scaling of the transform to avoid having later non-integer shifts that would require double divisions. Therefore:
    //Scaling for a non-power of 2 => 2^(6+0.5*log2(K)+ceil(log2(p))-0.5*log2(p))=sqrt(K/p)*64*2^ceil(log2(p))
    //As we can see, in the exponent we have subtracted the non-integer part log2(p) and we have added ceil(log2(p)), which is an integer. The scaling is hence bigger, so no precision is lost.
    c = lengths[i];
    const double K = lengthsPowerOf2Factor[i];
    const double p = nonPowerOf2FactorsOfLengths[i];
    const int ceilLog2ofp = (int)ceil(log2(p));
    TMatrixCoeff *iT = NULL;
    const Double s = sqrt(K / p) * (64 << (ceilLog2ofp + COM16_C806_TRANS_PREC)); //here we have added COM16_C806_TRANS_PREC for more precision

    //we update the g_aucCeilOfLog2OfNonPowerOf2Part for the non-power of 2 values of N, where we save ceil(log2(p))
    g_aucCeilOfLog2OfNonPowerOf2Part[c] = (p != 1) ? (SChar)(ceilLog2ofp) : 0;
    g_aucLog2OfPowerOf2Part[c] = (p != 1) ? (SChar)(log2(K)) : g_aucLog2[c];





    //now we fill the transforms arrays
    switch (i)
    {
    case 0:  iT = g_aiTr2[0][0]; break;
    case 1:  iT = g_aiTr4[0][0]; break;
    case 2:  iT = g_aiTr6[0][0]; break;
    case 3:  iT = g_aiTr8[0][0]; break;
    case 4:  iT = g_aiTr10[0][0]; break;
    case 5:  iT = g_aiTr12[0][0]; break;
    case 6:  iT = g_aiTr16[0][0]; break;
    case 7:  iT = g_aiTr20[0][0]; break;
    case 8:  iT = g_aiTr24[0][0]; break;
    case 9:  iT = g_aiTr32[0][0]; break;
    case 10: iT = g_aiTr40[0][0]; break;
    case 11: iT = g_aiTr48[0][0]; break;
    case 12: iT = g_aiTr64[0][0]; break;
    case 13: iT = g_aiTr80[0][0]; break;
    case 14: iT = g_aiTr96[0][0]; break;
    case 15: iT = g_aiTr128[0][0]; break;
    default: exit(0); break;
    }

    for (Int k = 0; k < c; k++)
    {
      for (Int n = 0; n < c; n++)
      {
        Double w0, w1, v;

        // DCT-II
        w0 = k == 0 ? sqrt(0.5) : 1;
        v = cos(PI*(n + 0.5)*k / c) * w0 * sqrt(2.0 / c);
        iT[DCT2*c*c + k*c + n] = (Short)(s * v + (v > 0 ? 0.5 : -0.5));

        // DCT-V
        w0 = (k == 0) ? sqrt(0.5) : 1.0;
        w1 = (n == 0) ? sqrt(0.5) : 1.0;
        v = cos(PI*n*k / (c - 0.5)) * w0 * w1 * sqrt(2.0 / (c - 0.5));
        iT[DCT5*c*c + k*c + n] = (Short)(s * v + (v > 0 ? 0.5 : -0.5));

        // DCT-VIII
        v = cos(PI*(k + 0.5)*(n + 0.5) / (c + 0.5)) * sqrt(2.0 / (c + 0.5));
        iT[DCT8*c*c + k*c + n] = (Short)(s * v + (v > 0 ? 0.5 : -0.5));

        // DST-I
        v = sin(PI*(n + 1)*(k + 1) / (c + 1)) * sqrt(2.0 / (c + 1));
        iT[DST1*c*c + k*c + n] = (Short)(s * v + (v > 0 ? 0.5 : -0.5));

        // DST-VII
        v = sin(PI*(k + 0.5)*(n + 1) / (c + 0.5)) * sqrt(2.0 / (c + 0.5));
        iT[DST7*c*c + k*c + n] = (Short)(s * v + (v > 0 ? 0.5 : -0.5));
      }

    }
  }


  generateTrafoBlockSizeScaling(*gp_sizeIdxInfo);

  SizeIndexInfoWithNonLog2 sizeInfo;
  sizeInfo.init(MAX_CU_SIZE);

  // initialize scan orders
  for (UInt blockHeightIdx = 0; blockHeightIdx < sizeInfo.numAllHeights(); blockHeightIdx++)
  {
    for (UInt blockWidthIdx = 0; blockWidthIdx < sizeInfo.numAllWidths(); blockWidthIdx++)
    {
      const UInt blockWidth  = sizeInfo.sizeFrom(blockWidthIdx);
      const UInt blockHeight = sizeInfo.sizeFrom(blockHeightIdx);
      const UInt totalValues = blockWidth * blockHeight;

      //--------------------------------------------------------------------------------------------------

      //non-grouped scan orders

      for (UInt scanTypeIndex = 0; scanTypeIndex < SCAN_NUMBER_OF_TYPES; scanTypeIndex++)
      {
        const CoeffScanType scanType = CoeffScanType(scanTypeIndex);

        g_scanOrder     [SCAN_UNGROUPED][scanType][blockWidthIdx][blockHeightIdx]    = new UInt[totalValues];
        g_scanOrderPosXY[SCAN_UNGROUPED][scanType][blockWidthIdx][blockHeightIdx][0] = new UInt[totalValues];
        g_scanOrderPosXY[SCAN_UNGROUPED][scanType][blockWidthIdx][blockHeightIdx][1] = new UInt[totalValues];

        ScanGenerator fullBlockScan(blockWidth, blockHeight, blockWidth, scanType);

        for (UInt scanPosition = 0; scanPosition < totalValues; scanPosition++)
        {
          const int rasterPos = fullBlockScan.GetNextIndex( 0, 0 );
          const int posY      = rasterPos / blockWidth;
          const int posX      = rasterPos - ( posY * blockWidth );
          g_scanOrder     [SCAN_UNGROUPED][scanType][blockWidthIdx][blockHeightIdx]   [scanPosition] = rasterPos;
          g_scanOrderPosXY[SCAN_UNGROUPED][scanType][blockWidthIdx][blockHeightIdx][0][scanPosition] = posX;
          g_scanOrderPosXY[SCAN_UNGROUPED][scanType][blockWidthIdx][blockHeightIdx][1][scanPosition] = posY;
        }
      }

      if( blockWidthIdx >= sizeInfo.numWidths() || blockHeightIdx >= sizeInfo.numHeights() )
      {
        // size indizes greater than numIdxs are sizes than are only used when grouping - they will never come up as a block size - thus they can be skipped at this point
        for( UInt scanTypeIndex = 0; scanTypeIndex < SCAN_NUMBER_OF_TYPES; scanTypeIndex++ )
        {
          g_scanOrder     [SCAN_GROUPED_4x4][scanTypeIndex][blockWidthIdx][blockHeightIdx]    = nullptr;
          g_scanOrderPosXY[SCAN_GROUPED_4x4][scanTypeIndex][blockWidthIdx][blockHeightIdx][0] = nullptr;
          g_scanOrderPosXY[SCAN_GROUPED_4x4][scanTypeIndex][blockWidthIdx][blockHeightIdx][1] = nullptr;

        }

        continue;
      }

      //--------------------------------------------------------------------------------------------------

      //grouped scan orders
      const UInt  log2CGWidth    = (blockWidth & 3) + (blockHeight & 3) > 0 ? 1 : 2;
      const UInt  log2CGHeight   = (blockWidth & 3) + (blockHeight & 3) > 0 ? 1 : 2;

      const UInt  groupWidth     = 1 << log2CGWidth;
      const UInt  groupHeight    = 1 << log2CGHeight;
      const UInt  widthInGroups  = blockWidth >> log2CGWidth;
      const UInt  heightInGroups = blockHeight >> log2CGHeight;

      const UInt  groupSize      = groupWidth    * groupHeight;
      const UInt  totalGroups    = widthInGroups * heightInGroups;

      for (UInt scanTypeIndex = 0; scanTypeIndex < SCAN_NUMBER_OF_TYPES; scanTypeIndex++)
      {
        const CoeffScanType scanType = CoeffScanType(scanTypeIndex);

        g_scanOrder     [SCAN_GROUPED_4x4][scanType][blockWidthIdx][blockHeightIdx]    = new UInt[totalValues];
        g_scanOrderPosXY[SCAN_GROUPED_4x4][scanType][blockWidthIdx][blockHeightIdx][0] = new UInt[totalValues];
        g_scanOrderPosXY[SCAN_GROUPED_4x4][scanType][blockWidthIdx][blockHeightIdx][1] = new UInt[totalValues];


        ScanGenerator fullBlockScan(widthInGroups, heightInGroups, groupWidth, scanType);

        for (UInt groupIndex = 0; groupIndex < totalGroups; groupIndex++)
        {
          const UInt groupPositionY  = fullBlockScan.GetCurrentY();
          const UInt groupPositionX  = fullBlockScan.GetCurrentX();
          const UInt groupOffsetX    = groupPositionX * groupWidth;
          const UInt groupOffsetY    = groupPositionY * groupHeight;
          const UInt groupOffsetScan = groupIndex     * groupSize;

          ScanGenerator groupScan(groupWidth, groupHeight, blockWidth, scanType);

          for (UInt scanPosition = 0; scanPosition < groupSize; scanPosition++)
          {
            const int rasterPos = groupScan.GetNextIndex( groupOffsetX, groupOffsetY );
            const int posY      = rasterPos / blockWidth;
            const int posX      = rasterPos - ( posY * blockWidth );

            g_scanOrder     [SCAN_GROUPED_4x4][scanType][blockWidthIdx][blockHeightIdx]   [groupOffsetScan + scanPosition] = rasterPos;
            g_scanOrderPosXY[SCAN_GROUPED_4x4][scanType][blockWidthIdx][blockHeightIdx][0][groupOffsetScan + scanPosition] = posX;
            g_scanOrderPosXY[SCAN_GROUPED_4x4][scanType][blockWidthIdx][blockHeightIdx][1][groupOffsetScan + scanPosition] = posY;
          }

          fullBlockScan.GetNextIndex(0, 0);
        }
      }

      //--------------------------------------------------------------------------------------------------
    }
  }

}

Void destroyROM()
{
  unsigned numWidths = gp_sizeIdxInfo->numAllWidths();
  unsigned numHeights = gp_sizeIdxInfo->numAllHeights();

  for (UInt groupTypeIndex = 0; groupTypeIndex < SCAN_NUMBER_OF_GROUP_TYPES; groupTypeIndex++)
  {
    for (UInt scanOrderIndex = 0; scanOrderIndex < SCAN_NUMBER_OF_TYPES; scanOrderIndex++)
    {
      for (UInt blockWidthIdx = 0; blockWidthIdx <= numWidths; blockWidthIdx++)
      {
        for (UInt blockHeightIdx = 0; blockHeightIdx <= numHeights; blockHeightIdx++)
        {
          delete[] g_scanOrder[groupTypeIndex][scanOrderIndex][blockWidthIdx][blockHeightIdx];
          g_scanOrder[groupTypeIndex][scanOrderIndex][blockWidthIdx][blockHeightIdx] = nullptr;

          delete[] g_scanOrderPosXY[groupTypeIndex][scanOrderIndex][blockWidthIdx][blockHeightIdx][0];
          g_scanOrderPosXY[groupTypeIndex][scanOrderIndex][blockWidthIdx][blockHeightIdx][0] = nullptr;

          delete[] g_scanOrderPosXY[groupTypeIndex][scanOrderIndex][blockWidthIdx][blockHeightIdx][1];
          g_scanOrderPosXY[groupTypeIndex][scanOrderIndex][blockWidthIdx][blockHeightIdx][1] = nullptr;

        }
      }
    }
  }

  delete gp_sizeIdxInfo;
  gp_sizeIdxInfo = nullptr;
}



void generateTrafoBlockSizeScaling(SizeIndexInfo& sizeIdxInfo)
{
  for (SizeType y = 0; y < sizeIdxInfo.numHeights(); y++)
  {
    for (SizeType x = 0; x < sizeIdxInfo.numWidths(); x++)
    {
      SizeType h = sizeIdxInfo.sizeFrom(y);
      SizeType w = sizeIdxInfo.sizeFrom(x);
      // pWidth and pHeight are the nonPowerOf2Parts of width and heigh respectively
      // a is 1 if log2(width)+log2(height) is an even number and 2 otherwise
      // realFactor for the quantization   => sqrt(a*pWidth*pHeight)
      // realFactor for the dequantization => sqrt(pWidth*pHeight/a)
      //the cases are implemented to minimize the number of costly math operations and truncation errors
      //NOTE => this only works for the widths and heights defined for the transforms in initROM!!!
      int pWidth = w >> g_aucLog2OfPowerOf2Part[w], pHeight = h >> g_aucLog2OfPowerOf2Part[h];
      int a = (g_aucLog2OfPowerOf2Part[h] + g_aucLog2OfPowerOf2Part[w]) % 2 == 0 ? 1 : 2;
      double sqrt_a = a != 1 ? sqrt(a) : 1;
      int log2a = g_aucLog2[a];
      double realFactorQuant, realFactorDeQuant;
      if (pWidth == pHeight)                //case sqrt(a*pWidth*pWidth)=sqrt(a*pWidth^2)=sqrt(a)*pWidth
      {
        realFactorQuant = pWidth*sqrt_a;
        realFactorDeQuant = pWidth / sqrt_a;
      }
      else if (pWidth == 1 && pHeight != 1) //case sqrt(a*pWidth*pWidth)=sqrt(a*1*pHeight)=sqrt(a*pHeight)
      {
        realFactorQuant = sqrt(pHeight << log2a);
        realFactorDeQuant = sqrt(pHeight >> log2a);
      }
      else if (pWidth != 1 && pHeight == 1) //case sqrt(a*pWidth*pWidth)=sqrt(a*pWidth*1)=sqrt(a*pWidth)
      {
        realFactorQuant = sqrt(pWidth << log2a);
        realFactorDeQuant = sqrt(pWidth >> log2a);
      }
      else                                  //general case sqrt(a*pWidth*pWidth)
      {
        realFactorQuant = sqrt((pWidth*pHeight) << log2a);
        realFactorDeQuant = sqrt((pWidth*pHeight) >> log2a);
      }
      //now we approximate the real factor to a fraction where the denominator can be implemented with shifts
      g_BlockSizeTrafoScale[h][w][0] = (pWidth == 1 && pHeight == 1 && a == 1) ? 1 : (int)round((double)((1 << ADJ_QUANT_SHIFT))*realFactorQuant);
      g_BlockSizeTrafoScale[h][w][1] = (pWidth == 1 && pHeight == 1 && a == 1) ? 1 : (int)round((double)((1 << ADJ_DEQUANT_SHIFT))*realFactorDeQuant);

    }
  }
}

// ====================================================================================================================
// Data structure related table & variable
// ====================================================================================================================

const Int g_quantScales[SCALING_LIST_REM_NUM] =
{
  26214,23302,20560,18396,16384,14564
};

const Int g_invQuantScales[SCALING_LIST_REM_NUM] =
{
  40,45,51,57,64,72
};

//--------------------------------------------------------------------------------------------------
//structures
//EMT transform coeficient variable
TMatrixCoeff g_aiTr2  [NUM_TRANS_TYPE][  2][  2];
TMatrixCoeff g_aiTr4  [NUM_TRANS_TYPE][  4][  4];
TMatrixCoeff g_aiTr8  [NUM_TRANS_TYPE][  8][  8];
TMatrixCoeff g_aiTr16 [NUM_TRANS_TYPE][ 16][ 16];
TMatrixCoeff g_aiTr32 [NUM_TRANS_TYPE][ 32][ 32];
TMatrixCoeff g_aiTr64 [NUM_TRANS_TYPE][ 64][ 64];
TMatrixCoeff g_aiTr128[NUM_TRANS_TYPE][128][128];

TMatrixCoeff g_aiTr6  [NUM_TRANS_TYPE][  6][  6];
TMatrixCoeff g_aiTr10 [NUM_TRANS_TYPE][ 10][ 10];
TMatrixCoeff g_aiTr12 [NUM_TRANS_TYPE][ 12][ 12];
TMatrixCoeff g_aiTr20 [NUM_TRANS_TYPE][ 20][ 20];
TMatrixCoeff g_aiTr24 [NUM_TRANS_TYPE][ 24][ 24];
TMatrixCoeff g_aiTr40 [NUM_TRANS_TYPE][ 40][ 40];
TMatrixCoeff g_aiTr48 [NUM_TRANS_TYPE][ 48][ 48];
TMatrixCoeff g_aiTr80 [NUM_TRANS_TYPE][ 80][ 80];
TMatrixCoeff g_aiTr96 [NUM_TRANS_TYPE][ 96][ 96];

//--------------------------------------------------------------------------------------------------
//coefficients
//--------------------------------------------------------------------------------------------------

const UChar g_aucChromaScale[NUM_CHROMA_FORMAT][chromaQPMappingTableSize] =
{
  //0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
#if MEDIATEK_CHROMA_SCALE
  { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,29,30,31,32,33,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,51,51,51,51 },
#else
  { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,29,30,31,32,33,33,34,34,35,35,36,36,37,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51 },
#endif
  { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,51,51,51,51,51,51 },
  { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,51,51,51,51,51,51 }
};

// ====================================================================================================================
// Intra prediction
// ====================================================================================================================

const UChar g_aucIntraModeNumFast_UseMPM_2D[7 - MIN_CU_LOG2 + 1][7 - MIN_CU_LOG2 + 1] =
{
  {3, 3, 3, 3, 2, 2},  //   4x4,   4x8,   4x16,   4x32,   4x64,   4x128,
  {3, 3, 3, 3, 3, 2},  //   8x4,   8x8,   8x16,   8x32,   8x64,   8x128,
  {3, 3, 3, 3, 3, 2},  //  16x4,  16x8,  16x16,  16x32,  16x64,  16x128,
  {3, 3, 3, 3, 3, 2},  //  32x4,  32x8,  32x16,  32x32,  32x64,  32x128,
  {2, 3, 3, 3, 3, 2},  //  64x4,  64x8,  64x16,  64x32,  64x64,  64x128,
  {2, 2, 2, 2, 2, 3},  // 128x4, 128x8, 128x16, 128x32, 128x64, 128x128,
};

const UChar g_aucIntraModeNumFast_UseMPM[MAX_CU_DEPTH] =
{
  3,  //   2x2
  8,  //   4x4
  8,  //   8x8
  3,  //  16x16
  3,  //  32x32
  3,  //  64x64
  3   // 128x128
};
const UChar g_aucIntraModeNumFast_NotUseMPM[MAX_CU_DEPTH] =
{
  3,  //   2x2
  9,  //   4x4
  9,  //   8x8
  4,  //  16x16   33
  4,  //  32x32   33
  5,  //  64x64   33
  5   // 128x128
};

const UChar g_chroma422IntraAngleMappingTable[NUM_INTRA_MODE] =
//                                                               H                                                               D                                                               V
//0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, DM
{ 0, 1, 2, 2, 2, 2, 2, 2, 2, 3,  4,  6,  8, 10, 12, 13, 14, 16, 18, 20, 22, 23, 24, 26, 28, 30, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 44, 44, 45, 46, 46, 46, 47, 48, 48, 48, 49, 50, 51, 52, 52, 52, 53, 54, 54, 54, 55, 56, 56, 56, 57, 58, 59, 60, DM_CHROMA_IDX };

extern const UChar  g_intraMode65to33AngMapping[NUM_INTRA_MODE] =
//                                                               H                                                               D                                                               V
//0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, DM
{ 0, 1, 2, 2, 3, 3, 4, 4, 5, 5,  6,  6,  7,  7,  8,  8,  9,  9, 10, 10, 11, 11, 12, 12, 13, 13, 14, 14, 15, 15, 16, 16, 17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 22, 23, 23, 24, 24, 25, 25, 26, 26, 27, 27, 28, 28, 29, 29, 30, 30, 31, 31, 32, 32, 33, 33, 34, DM_CHROMA_IDX };

extern const UChar g_intraMode33to65AngMapping[36] =
//                                   H                               D                               V
//0, 1, 2, 3, 4, 5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, DM
{ 0, 1, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44, 46, 48, 50, 52, 54, 56, 58, 60, 62, 64, 66, DM_CHROMA_IDX };


// ====================================================================================================================
// Decision tree templates
// ====================================================================================================================

const DecisionTreeTemplate g_mtSplitDTT = compile(
  decision( DTT_SPLIT_DO_SPLIT_DECISION,
  /*0*/ DTT_SPLIT_NO_SPLIT,
  /*1*/ decision( DTT_SPLIT_HV_DECISION,
        /*0*/ decision( DTT_SPLIT_H_IS_BT_12_DECISION,
              /*0*/ decision( DTT_SPLIT_H_IS_SYM,
                    /*0*/ decision( DTT_SPLIT_H_IS_14,
                          /*0*/ DTT_SPLIT_BT_H_34,
                          /*1*/ DTT_SPLIT_BT_H_14 ),
                    /*1*/ DTT_SPLIT_TT_HORZ ),
              /*1*/ DTT_SPLIT_BT_HORZ ),
        /*1*/ decision( DTT_SPLIT_V_IS_BT_12_DECISION,
              /*0*/ decision( DTT_SPLIT_V_IS_SYM,
                    /*0*/ decision( DTT_SPLIT_V_IS_14,
                          /*0*/ DTT_SPLIT_BT_V_34,
                          /*1*/ DTT_SPLIT_BT_V_14 ),
                    /*1*/ DTT_SPLIT_TT_VERT ),
              /*1*/ DTT_SPLIT_BT_VERT ) ) ) );


const DecisionTreeTemplate g_genBinSplitDTT = compile(
  decision( DTT_GBS_DO_SPLIT,
  /*0*/ DTT_GBS_DONT_SPLIT,
  /*1*/ decision( DTT_GBS_DO_PERP_SPLIT,
        /*0*/ DTT_GBS_PARL_SPLIT,
        /*1*/ DTT_GBS_PERP_SPLIT ) ) );

const DecisionTreeTemplate g_splitModifierDTT = compile(
  //////////////////////////////////////////////////////////////////////////
  // all but the 3/2 and 5/4 subtrees are used for log2 classes
  decision( DTT_SM_IS_ASYM,
  /*0*/ decision( DTT_SM_IS_x3,
        /*0*/ decision( DTT_SM_IS_x5,
              /*0*/ DTT_SM_12,
              //////////////////////////////////////////////////////////////////////////
              // the following subtree is used for 5/4 classes
              /*1*/ decision( DTT_SM_IS_235,
                    /*0*/ decision( DTT_SM_IS_15,
                          /*0*/ DTT_SM_45,
                          /*1*/ DTT_SM_15 ),
                    /*1*/ decision( DTT_SM_IS_25,
                          /*0*/ DTT_SM_35,
                          /*1*/ DTT_SM_25 ) ) ),
        //////////////////////////////////////////////////////////////////////////
        // the following subtree is used for 3/2 classes
        /*1*/ decision( DTT_SM_IS_13,
              /*0*/ DTT_SM_23,
              /*1*/ DTT_SM_13 ) ),
        //////////////////////////////////////////////////////////////////////////
        // back to log2 tree
  /*1*/ decision( DTT_SM_IS_QUART,
        /*0*/ decision( DTT_SM_IS_38,
              /*0*/ DTT_SM_58,
              /*1*/ DTT_SM_38 ),
        /*1*/ decision( DTT_SM_IS_14,
              /*0*/ DTT_SM_34,
              /*1*/ DTT_SM_14 ) ) ) );


// ====================================================================================================================
// Misc.
// ====================================================================================================================
SizeIndexInfo*           gp_sizeIdxInfo = NULL;
int                      g_BlockSizeTrafoScale[MAX_CU_SIZE + 1][MAX_CU_SIZE + 1][2];
SChar                    g_aucLog2    [MAX_CU_SIZE + 1];
SChar                    g_aucNextLog2[MAX_CU_SIZE + 1];
SChar                    g_aucPrevLog2[MAX_CU_SIZE + 1];

UnitScale g_miScaling( MIN_CU_LOG2, MIN_CU_LOG2 );

SChar                    g_aucCeilOfLog2OfNonPowerOf2Part[MAX_CU_SIZE + 1];
SChar                    g_aucLog2OfPowerOf2Part[MAX_CU_SIZE + 1];

// ====================================================================================================================
// Scanning order & context model mapping
// ====================================================================================================================

// scanning order table
UInt* g_scanOrder     [SCAN_NUMBER_OF_GROUP_TYPES][SCAN_NUMBER_OF_TYPES][MAX_CU_SIZE / 2 + 1][MAX_CU_SIZE / 2 + 1];
UInt* g_scanOrderPosXY[SCAN_NUMBER_OF_GROUP_TYPES][SCAN_NUMBER_OF_TYPES][MAX_CU_SIZE / 2 + 1][MAX_CU_SIZE / 2 + 1][2];

const UInt ctxIndMap4x4[4 * 4] =
{
  0, 1, 4, 5,
  2, 3, 4, 5,
  6, 6, 8, 8,
  7, 7, 8, 8
};


const UInt g_uiMinInGroup[LAST_SIGNIFICANT_GROUPS] = { 0,1,2,3,4,6,8,12,16,24,32,48,64,96 };
const UInt g_uiGroupIdx[MAX_TU_SIZE] = { 0,1,2,3,4,4,5,5,6,6,6,6,7,7,7,7,8,8,8,8,8,8,8,8,9,9,9,9,9,9,9,9, 10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11
,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12
,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13 };
const UInt g_auiGoRiceRange[MAX_GR_ORDER_RESIDUAL] =
{
  6, 5, 6, COEF_REMAIN_BIN_REDUCTION, COEF_REMAIN_BIN_REDUCTION, COEF_REMAIN_BIN_REDUCTION, COEF_REMAIN_BIN_REDUCTION, COEF_REMAIN_BIN_REDUCTION, COEF_REMAIN_BIN_REDUCTION, COEF_REMAIN_BIN_REDUCTION
};

const TChar *MatrixType[SCALING_LIST_SIZE_NUM][SCALING_LIST_NUM] =
{
  {
    "INTRA2X2_LUMA",
    "INTRA2X2_CHROMAU",
    "INTRA2X2_CHROMAV",
    "INTER2X2_LUMA",
    "INTER2X2_CHROMAU",
    "INTER2X2_CHROMAV"
  },
  {
    "INTRA4X4_LUMA",
    "INTRA4X4_CHROMAU",
    "INTRA4X4_CHROMAV",
    "INTER4X4_LUMA",
    "INTER4X4_CHROMAU",
    "INTER4X4_CHROMAV"
  },
  {
    "INTRA8X8_LUMA",
    "INTRA8X8_CHROMAU",
    "INTRA8X8_CHROMAV",
    "INTER8X8_LUMA",
    "INTER8X8_CHROMAU",
    "INTER8X8_CHROMAV"
  },
  {
    "INTRA16X16_LUMA",
    "INTRA16X16_CHROMAU",
    "INTRA16X16_CHROMAV",
    "INTER16X16_LUMA",
    "INTER16X16_CHROMAU",
    "INTER16X16_CHROMAV"
  },
  {
   "INTRA32X32_LUMA",
   "INTRA32X32_CHROMAU_FROM16x16_CHROMAU",
   "INTRA32X32_CHROMAV_FROM16x16_CHROMAV",
   "INTER32X32_LUMA",
   "INTER32X32_CHROMAU_FROM16x16_CHROMAU",
   "INTER32X32_CHROMAV_FROM16x16_CHROMAV"
  },
};

const TChar *MatrixType_DC[SCALING_LIST_SIZE_NUM][SCALING_LIST_NUM] =
{
  {
  },
  {
  },
  {
  },
  {
    "INTRA16X16_LUMA_DC",
    "INTRA16X16_CHROMAU_DC",
    "INTRA16X16_CHROMAV_DC",
    "INTER16X16_LUMA_DC",
    "INTER16X16_CHROMAU_DC",
    "INTER16X16_CHROMAV_DC"
  },
  {
    "INTRA32X32_LUMA_DC",
    "INTRA32X32_CHROMAU_DC_FROM16x16_CHROMAU",
    "INTRA32X32_CHROMAV_DC_FROM16x16_CHROMAV",
    "INTER32X32_LUMA_DC",
    "INTER32X32_CHROMAU_DC_FROM16x16_CHROMAU",
    "INTER32X32_CHROMAV_DC_FROM16x16_CHROMAV"
  },
};

const Int g_quantTSDefault4x4[4 * 4] =
{
  16,16,16,16,
  16,16,16,16,
  16,16,16,16,
  16,16,16,16
};

const Int g_quantIntraDefault8x8[8 * 8] =
{
  16,16,16,16,17,18,21,24,
  16,16,16,16,17,19,22,25,
  16,16,17,18,20,22,25,29,
  16,16,18,21,24,27,31,36,
  17,17,20,24,30,35,41,47,
  18,19,22,27,35,44,54,65,
  21,22,25,31,41,54,70,88,
  24,25,29,36,47,65,88,115
};

const Int g_quantInterDefault8x8[8 * 8] =
{
  16,16,16,16,17,18,20,24,
  16,16,16,17,18,20,24,25,
  16,16,17,18,20,24,25,28,
  16,17,18,20,24,25,28,33,
  17,18,20,24,25,28,33,41,
  18,20,24,25,28,33,41,54,
  20,24,25,28,33,41,54,71,
  24,25,28,33,41,54,71,91
};

const UInt g_scalingListSize [SCALING_LIST_SIZE_NUM] = { 4, 16, 64, 256, 1024, 4096, 16384 };
const UInt g_scalingListSizeX[SCALING_LIST_SIZE_NUM] = { 2,  4,  8,  16,   32,   64,   128 };

const UChar g_NonMPM[257] = { 0, 0, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4,
4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5,
5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6,
6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6,
6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 7, 7,
7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 8 };


//! \}
