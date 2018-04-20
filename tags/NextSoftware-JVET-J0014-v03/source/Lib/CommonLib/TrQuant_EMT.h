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

/** \file     TrQuant_EMT.h
    \brief    transform and quantization class (header)
*/

#ifndef __TRQUANT_EMT__
#define __TRQUANT_EMT__

#include "CommonDef.h"

////DCT-II transforms
void fastForwardDCT2_B2  (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use);
void fastInverseDCT2_B2  (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDCT2_B4  (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use);
void fastInverseDCT2_B4  (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDCT2_B8  (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use);
void fastInverseDCT2_B8  (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDCT2_B16 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use);
void fastInverseDCT2_B16 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDCT2_B32 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use);
void fastInverseDCT2_B32 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDCT2_B64 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use);
void fastInverseDCT2_B64 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDCT2_B128(const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use);
void fastInverseDCT2_B128(const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDCT2_B6  (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use);
void fastInverseDCT2_B6  (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDCT2_B10 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use);
void fastInverseDCT2_B10 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDCT2_B12 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use);
void fastInverseDCT2_B12 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDCT2_B20 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use);
void fastInverseDCT2_B20 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDCT2_B24 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use);
void fastInverseDCT2_B24 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDCT2_B40 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use);
void fastInverseDCT2_B40 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDCT2_B48 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use);
void fastInverseDCT2_B48 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDCT2_B80 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use);
void fastInverseDCT2_B80 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDCT2_B96 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use);
void fastInverseDCT2_B96 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum);
#if THRESHOLDING
void fastForwardDCT2_B160(const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use);
void fastInverseDCT2_B160(const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDCT2_B192(const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use);
void fastInverseDCT2_B192(const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum);
#endif

//DST-VII transforms (HEVC uses only the 4x4 ones and EMT uses all of them)
void fastForwardDST7_B4  (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use);
void fastInverseDST7_B4  (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDST7_B8  (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use);
void fastInverseDST7_B8  (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDST7_B16 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use);
void fastInverseDST7_B16 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDST7_B32 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use);
void fastInverseDST7_B32 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDST7_B64 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use);
void fastInverseDST7_B64 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDST7_B128(const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use);
void fastInverseDST7_B128(const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDST7_B6  (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use);
void fastInverseDST7_B6  (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDST7_B10 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use);
void fastInverseDST7_B10 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDST7_B12 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use);
void fastInverseDST7_B12 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDST7_B20 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use);
void fastInverseDST7_B20 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDST7_B24 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use);
void fastInverseDST7_B24 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDST7_B40 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use);
void fastInverseDST7_B40 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDST7_B48 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use);
void fastInverseDST7_B48 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDST7_B80 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use);
void fastInverseDST7_B80 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDST7_B96 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use);
void fastInverseDST7_B96 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum);

//DCT-V transforms (EMT)
void fastForwardDCT5_B4  (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use);
void fastInverseDCT5_B4  (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDCT5_B8  (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use);
void fastInverseDCT5_B8  (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDCT5_B16 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use);
void fastInverseDCT5_B16 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDCT5_B32 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use);
void fastInverseDCT5_B32 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDCT5_B64 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use);
void fastInverseDCT5_B64 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDCT5_B128(const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use);
void fastInverseDCT5_B128(const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDCT5_B6  (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use);
void fastInverseDCT5_B6  (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDCT5_B10 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use);
void fastInverseDCT5_B10 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDCT5_B12 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use);
void fastInverseDCT5_B12 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDCT5_B20 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use);
void fastInverseDCT5_B20 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDCT5_B24 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use);
void fastInverseDCT5_B24 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDCT5_B40 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use);
void fastInverseDCT5_B40 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDCT5_B48 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use);
void fastInverseDCT5_B48 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDCT5_B80 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use);
void fastInverseDCT5_B80 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDCT5_B96 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use);
void fastInverseDCT5_B96 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum);

//DCT-VIII transforms (EMT)
void fastForwardDCT8_B4  (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use);
void fastInverseDCT8_B4  (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDCT8_B8  (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use);
void fastInverseDCT8_B8  (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDCT8_B16 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use);
void fastInverseDCT8_B16 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDCT8_B32 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use);
void fastInverseDCT8_B32 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDCT8_B64 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use);
void fastInverseDCT8_B64 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDCT8_B128(const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use);
void fastInverseDCT8_B128(const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDCT8_B6  (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use);
void fastInverseDCT8_B6  (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDCT8_B10 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use);
void fastInverseDCT8_B10 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDCT8_B12 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use);
void fastInverseDCT8_B12 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDCT8_B20 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use);
void fastInverseDCT8_B20 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDCT8_B24 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use);
void fastInverseDCT8_B24 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDCT8_B40 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use);
void fastInverseDCT8_B40 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDCT8_B48 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use);
void fastInverseDCT8_B48 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDCT8_B80 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use);
void fastInverseDCT8_B80 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDCT8_B96 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use);
void fastInverseDCT8_B96 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum);

//DST-I transforms (EMT)
void fastForwardDST1_B4  (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use);
void fastInverseDST1_B4  (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDST1_B8  (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use);
void fastInverseDST1_B8  (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDST1_B16 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use);
void fastInverseDST1_B16 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDST1_B32 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use);
void fastInverseDST1_B32 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDST1_B64 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use);
void fastInverseDST1_B64 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDST1_B128(const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use);
void fastInverseDST1_B128(const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDST1_B6  (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use);
void fastInverseDST1_B6  (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDST1_B10 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use);
void fastInverseDST1_B10 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDST1_B12 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use);
void fastInverseDST1_B12 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDST1_B20 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use);
void fastInverseDST1_B20 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDST1_B24 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use);
void fastInverseDST1_B24 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDST1_B40 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use);
void fastInverseDST1_B40 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDST1_B48 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use);
void fastInverseDST1_B48 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDST1_B80 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use);
void fastInverseDST1_B80 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDST1_B96 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use);
void fastInverseDST1_B96 (const TCoeff *src, TCoeff *dst, Int shift, Int line, Int iSkipLine, Int iSkipLine2, Int use, const TCoeff outputMinimum, const TCoeff outputMaximum);

#endif // __TRQUANT__