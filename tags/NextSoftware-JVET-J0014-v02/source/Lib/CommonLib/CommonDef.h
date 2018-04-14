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

/** \file     CommonDef.h
    \brief    Defines version information, constants and small in-line functions
*/

#ifndef __COMMONDEF__
#define __COMMONDEF__

#include <algorithm>
#include <iostream>
#include <iomanip>
#include <limits>


#if _MSC_VER > 1000
// disable "signed and unsigned mismatch"
#pragma warning( disable : 4018 )
// disable Bool coercion "performance warning"
#pragma warning( disable : 4800 )
#endif // _MSC_VER > 1000
#include "TypeDef.h"
#include "version.h"

#ifdef _MSC_VER
#if _MSC_VER <= 1500
inline Int64 abs (Int64 x) { return _abs64(x); };
#endif
#endif

//! \ingroup CommonLib
//! \{

// ====================================================================================================================
// Version information
// ====================================================================================================================

#define NV_VERSION        "1.4.1"                 ///< Current software version

// ====================================================================================================================
// Platform information
// ====================================================================================================================

#ifdef __GNUC__
#define NVM_COMPILEDBY  "[GCC %d.%d.%d]", __GNUC__, __GNUC_MINOR__, __GNUC_PATCHLEVEL__
#ifdef __IA64__
#define NVM_ONARCH    "[on 64-bit] "
#else
#define NVM_ONARCH    "[on 32-bit] "
#endif
#endif

#ifdef __INTEL_COMPILER
#define NVM_COMPILEDBY  "[ICC %d]", __INTEL_COMPILER
#elif  _MSC_VER
#define NVM_COMPILEDBY  "[VS %d]", _MSC_VER
#endif

#ifndef NVM_COMPILEDBY
#define NVM_COMPILEDBY "[Unk-CXX]"
#endif

#ifdef _WIN32
#define NVM_ONOS        "[Windows]"
#elif  __linux
#define NVM_ONOS        "[Linux]"
#elif  __CYGWIN__
#define NVM_ONOS        "[Cygwin]"
#elif __APPLE__
#define NVM_ONOS        "[Mac OS X]"
#else
#define NVM_ONOS "[Unk-OS]"
#endif

#define NVM_BITS          "[%d bit] ", (sizeof(Void*) == 8 ? 64 : 32) ///< used for checking 64-bit O/S

#ifndef NULL
#define NULL              0
#endif

// ====================================================================================================================
// Common constants
// ====================================================================================================================

static const UInt   MAX_UINT =                            0xFFFFFFFFU; ///< max. value of unsigned 32-bit integer
static const Int    MAX_INT =                              2147483647; ///< max. value of signed 32-bit integer
static const UChar  MAX_UCHAR =                                   255;
static const UChar  MAX_SCHAR =                                   127;
static const Double MAX_DOUBLE =                             1.7e+308; ///< max. value of Double-type value

// ====================================================================================================================
// Coding tool configuration
// ====================================================================================================================
// Most of these should not be changed - they resolve the meaning of otherwise magic numbers.

static const Int MAX_GOP =                                         64; ///< max. value of hierarchical GOP size
static const Int MAX_NUM_REF_PICS =                                16; ///< max. number of pictures used for reference
static const Int MAX_NUM_REF =                                     16; ///< max. number of entries in picture reference list
static const Int MAX_QP =                                          51;
static const Int NOT_VALID =                                       -1;

static const Int AMVP_MAX_NUM_CANDS =                               2; ///< AMVP: advanced motion vector prediction - max number of final candidates
static const Int AMVP_MAX_NUM_CANDS_MEM =                           3; ///< AMVP: advanced motion vector prediction - max number of candidates
static const Int AMVP_DECIMATION_FACTOR =                           4;
static const Int MRG_MAX_NUM_CANDS =                                7; ///< MERGE

static const Int MAX_TLAYER =                                       7; ///< Explicit temporal layer QP offset - max number of temporal layer

static const Int ADAPT_SR_SCALE =                                   1; ///< division factor for adaptive search range

static const Int MAX_NUM_PICS_IN_SOP =                           1024;

static const Int MAX_NESTING_NUM_OPS =                           1024;
static const Int MAX_NESTING_NUM_LAYER =                           64;

static const Int MAX_VPS_NUM_HRD_PARAMETERS =                       1;
static const Int MAX_VPS_OP_SETS_PLUS1 =                         1024;
static const Int MAX_VPS_NUH_RESERVED_ZERO_LAYER_ID_PLUS1 =         1;

static const Int MAXIMUM_INTRA_FILTERED_WIDTH =                    16;
static const Int MAXIMUM_INTRA_FILTERED_HEIGHT =                   16;

static const int MINIMUM_INTRA_BILATERAL_WIDTH =                   16;
static const int MINIMUM_INTRA_BILATERAL_HEIGHT =                  16;

static const int INTRA_BILATERAL_STRONG_WIDTH =                    64;
static const int INTRA_BILATERAL_STRONG_HEIGHT =                   64;
static const int MAX_MRL_OFFSET =                                   3;

static const Int MAX_CPB_CNT =                                     32; ///< Upper bound of (cpb_cnt_minus1 + 1)
static const Int MAX_NUM_LAYER_IDS =                               64;

static const Int COEF_REMAIN_BIN_REDUCTION =                        3; ///< indicates the level at which the VLC transitions from Golomb-Rice to TU+EG(k)

static const Int CU_DQP_TU_CMAX =                                   5; ///< max number bins for truncated unary
static const Int CU_DQP_EG_k =                                      0; ///< expgolomb order

static const Int SBH_THRESHOLD =                                    4; ///< value of the fixed SBH controlling threshold

static const Int CXFLAG_NUMBER =                                   16; ///< maximum number of largerThanX flag coded in one chunk
static const Int C1FLAG_NUMBER =                                    8; ///< maximum number of largerThan1 flag coded in one chunk: 16 in HM5
static const Int C2FLAG_NUMBER =                                    1; ///< maximum number of largerThan2 flag coded in one chunk: 16 in HM5

static const Int MAX_NUM_VPS =                                     16;
static const Int MAX_NUM_SPS =                                     16;
static const Int MAX_NUM_PPS =                                     64;

static const Int MLS_GRP_NUM =                                   1024; ///< Max number of coefficient groups, max(16, 256)

static const Int MLS_CG_SIZE =                                      4; ///< Coefficient group size of 4x4; = MLS_CG_LOG2_WIDTH + MLS_CG_LOG2_HEIGHT

static const Int ADJ_QUANT_SHIFT =                                  7;
static const Int ADJ_DEQUANT_SHIFT =            ( ADJ_QUANT_SHIFT + 1 );

static const Int RVM_VCEGAM10_M =                                   4;

static const Int NUM_LUMA_MODE =                                   67; ///< Planar + DC + 65 directional mode (4*16 + 1)
static const Int LM_FILTER_NUM =                                    4;
static const Int NUM_LMC_MODE =                   (2 + LM_FILTER_NUM); ///< LMC + MMLM + MFLM(4)
static const Int NUM_INTRA_MODE =      (NUM_LUMA_MODE + NUM_LMC_MODE);

static const Int NUM_DIR =           (((NUM_LUMA_MODE - 3) >> 2) + 1);
static const Int PLANAR_IDX =                                       0; ///< index for intra PLANAR mode
static const Int DC_IDX =                                           1; ///< index for intra DC     mode
static const Int HOR_IDX =                    (1 * (NUM_DIR - 1) + 2); ///< index for intra HORIZONTAL mode
static const Int DIA_IDX =                    (2 * (NUM_DIR - 1) + 2); ///< index for intra DIAGONAL   mode
static const Int VER_IDX =                    (3 * (NUM_DIR - 1) + 2); ///< index for intra VERTICAL   mode
static const Int VDIA_IDX =                   (4 * (NUM_DIR - 1) + 2); ///< index for intra VDIAGONAL  mode
static const Int NOMODE_IDX =                               MAX_UCHAR; ///< indicating uninitialized elements

static const UChar NUM_FTM_REG =                                    5; ///< total number of search regions
static const UChar FTM_RATE =                                       2; ///< Rate of FTM downsampling, search window etc.
static const UChar FTM_REG1 =                                       1; ///< FTM Reg1
static const UChar FTM_REG2 =                                       2; ///< FTM Reg2
static const UChar FTM_REG3 =                                       3; ///< FTM Reg3
static const UChar FTM_REG4 =                                       4; ///< FTM Reg4
static const UChar FTM_REG5 =                                       5; ///< FTM Reg5
static const UChar FTM_HORVER =                                     1; ///< FTM Reg type HOR and VER
static const UChar FTM_HOR =                                        2; ///< FTM Reg type HOR
static const UChar FTM_VER =                                        3; ///< FTM Reg type VER
static const UChar FTM_COUNT1 =                                     1; ///< FTM Count1
static const UChar FTM_COUNT2 =                                     2; ///< FTM Count2
static const UChar FTM_COUNT3 =                                     3; ///< FTM Count3
static const UChar FTM_COUNT4 =                                     4; ///< FTM Count4
static const Double FTM_LOW_BOUND =                              1.75; ///< Lower bound for speed up
static const Double FTM_HIGH_BOUND =                            10.00; ///< Upper bound for speed up
static const Double FTM_MULT_FACTOR =                             1.5; ///< FTM cost multiplying factor for speed up

static const Int NUM_CHROMA_MODE =                 (5 + NUM_LMC_MODE); ///< total number of chroma modes
static const Int NUM_DM_MODES =                                     5; ///< total number of chroma DM modes
static const Int LM_CHROMA_IDX =                        NUM_LUMA_MODE; ///< chroma mode index for derived from LM mode
static const Int MMLM_CHROMA_IDX =                  LM_CHROMA_IDX + 1;
static const Int LM_CHROMA_F1_IDX =                 LM_CHROMA_IDX + 2;
static const Int LM_CHROMA_F2_IDX =                 LM_CHROMA_IDX + 3;
static const Int LM_CHROMA_F3_IDX =                 LM_CHROMA_IDX + 4;
static const Int LM_CHROMA_F4_IDX =                 LM_CHROMA_IDX + 5;
static const Int DM_CHROMA_IDX =                       NUM_INTRA_MODE; ///< chroma mode index for derived from luma intra mode

static const UChar INTER_MODE_IDX =                               255; ///< index for inter modes
static const UInt  EMT_INTRA_MAX_CU =                              32; ///< Max Intra CU size applying EMT, supported values: 8, 16, 32, 64, 128
static const UInt  EMT_INTER_MAX_CU =                              32; ///< Max Inter CU size applying EMT, supported values: 8, 16, 32, 64, 128
static const UInt  EMT_INTRA_MAX_CU_WITH_QTBT =                    64; ///< Max Intra CU size applying EMT, supported values: 8, 16, 32, 64, 128
static const UInt  EMT_INTER_MAX_CU_WITH_QTBT =                    64; ///< Max Inter CU size applying EMT, supported values: 8, 16, 32, 64, 128

static const Int NUM_MOST_PROBABLE_MODES =                          3;
static const Int NUM_MOST_PROBABLE_MODES_67 =                       6;
static const Int MMLM_SAMPLE_NEIGHBOR_LINES =                       2;
static const Int LM_SYMBOL_NUM =                   (1 + NUM_LMC_MODE);

static const Int FAST_UDI_MAX_RDMODE_NUM = (NUM_LUMA_MODE + NUM_FTM_REG); ///< maximum number of RD comparison in fast-UDI estimation loop
static const Int FAST_UDI_MAX_RD_FOR_DIFF =                        32;

static const Int NSST_HYGT_PTS =                             (1 << 8);
static const Int NSST_SIG_NZ_LUMA =                                 1;
static const Int NSST_SIG_NZ_CHROMA =                               1;

static const Int MDCS_ANGLE_LIMIT =                                 9; ///< 0 = Horizontal/vertical only, 1 = Horizontal/vertical +/- 1, 2 = Horizontal/vertical +/- 2 etc...

static const Int MDCS_MAXIMUM_WIDTH =                               8; ///< (measured in pixels) TUs with width greater than this can only use diagonal scan
static const Int MDCS_MAXIMUM_HEIGHT =                              8; ///< (measured in pixels) TUs with height greater than this can only use diagonal scan


static const Int LOG2_MAX_NUM_COLUMNS_MINUS1 =                      7;
static const Int LOG2_MAX_NUM_ROWS_MINUS1 =                         7;

static const Int CABAC_INIT_PRESENT_FLAG =                          1;

static const Int LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS   = 4;
static const Int CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS = 8;
static const Int VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE         = 2;   ///< additional precision bit for MV storage

static const Int MAX_NUM_LONG_TERM_REF_PICS =                      33;
static const Int NUM_LONG_TERM_REF_PIC_SPS =                        0;


static const Int MAX_QP_OFFSET_LIST_SIZE =                          6; ///< Maximum size of QP offset list is 6 entries

// Cost mode support
static const Int LOSSLESS_AND_MIXED_LOSSLESS_RD_COST_TEST_QP =      0; ///< QP to use for lossless coding.
static const Int LOSSLESS_AND_MIXED_LOSSLESS_RD_COST_TEST_QP_PRIME =4; ///< QP' to use for mixed_lossy_lossless coding.

static const Int CR_FROM_CB_REG_COST_SHIFT                        = 9;

static const Int RExt__GOLOMB_RICE_ADAPTATION_STATISTICS_SETS =     4;

static const Int RExt__PREDICTION_WEIGHTING_ANALYSIS_DC_PRECISION = 0; ///< Additional fixed bit precision used during encoder-side weighting prediction analysis. Currently only used when high_precision_prediction_weighting_flag is set, for backwards compatibility reasons.

static const Int MAX_TIMECODE_SEI_SETS =                            3; ///< Maximum number of time sets

static const Int MAX_CU_DEPTH =                                     7; ///< log2(CTUSize)
static const Int MAX_CU_SIZE =                        1<<MAX_CU_DEPTH;
static const Int MIN_CU_LOG2 =                                      2;
static const Int MIN_PU_SIZE =                                      4;
static const Int MIN_TU_SIZE =                                      4;
static const Int MAX_TU_SIZE =                                    128;
static const Int MAX_LOG2_TU_SIZE_PLUS_ONE =                        8; ///< log2(MAX_TU_SIZE) + 1
static const Int MAX_NUM_PARTS_IN_CTU =                         ( ( MAX_CU_SIZE * MAX_CU_SIZE ) >> ( MIN_CU_LOG2 << 1 ) );
#if THRESHOLDING
static const Int MAX_TR_SIZE =                                    192;
#else
static const Int MAX_TR_SIZE =                            MAX_CU_SIZE;
#endif

static const Int JVET_C0024_ZERO_OUT_TH =                          32;
static const Double JVET_D0077_SPLIT_DECISION_COST_SCALE =       1.05;

static const Int MAX_NUM_PART_IDXS_IN_CTU_WIDTH = MAX_CU_SIZE/MIN_PU_SIZE; ///< maximum number of partition indices across the width of a CTU (or height of a CTU)
static const Int SCALING_LIST_REM_NUM =                             6;

static const Int QUANT_SHIFT =                                     14; ///< Q(4) = 2^14
static const Int IQUANT_SHIFT =                                     6;
static const Int SCALE_BITS =                                      15; ///< Precision for fractional bit estimates

static const Int SCALING_LIST_NUM = MAX_NUM_COMPONENT * NUMBER_OF_PREDICTION_MODES; ///< list number for quantization matrix

static const Int SCALING_LIST_START_VALUE =                         8; ///< start value for dpcm mode
static const Int MAX_MATRIX_COEF_NUM =                             64; ///< max coefficient number for quantization matrix
static const Int MAX_MATRIX_SIZE_NUM =                              8; ///< max size number for quantization matrix
static const Int SCALING_LIST_BITS =                                8; ///< bit depth of scaling list entries
static const Int LOG2_SCALING_LIST_NEUTRAL_VALUE =                  4; ///< log2 of the value that, when used in a scaling list, has no effect on quantisation
static const Int SCALING_LIST_DC =                                 16; ///< default DC value

static const Int CONTEXT_STATE_BITS =                               6;
static const Int LAST_SIGNIFICANT_GROUPS =                         14;
static const Int MAX_GR_ORDER_RESIDUAL =                           10;

static const Int AFFINE_MAX_NUM_V0 =                                3; ///< max number of motion candidates in top-left corner
static const Int AFFINE_MAX_NUM_V1 =                                2; ///< max number of motion candidates in top-right corner
static const Int AFFINE_MAX_NUM_V2 =                                2; ///< max number of motion candidates in left-bottom corner
static const Int AFFINE_MAX_NUM_COMB =                             12; ///< max number of combined motion candidates
static const Int AFFINE_MIN_BLOCK_SIZE =                            4; ///< Minimum affine MC block size

#if W0038_DB_OPT
static const Int MAX_ENCODER_DEBLOCKING_QUALITY_LAYERS =           8 ;
#endif

#if SHARP_LUMA_DELTA_QP
static const UInt LUMA_LEVEL_TO_DQP_LUT_MAXSIZE =                1024; ///< max LUT size for QP offset based on luma

#endif
static const Int NUM_EMT_CU_FLAG_CTX =                              6;      ///< number of context models for EMT CU-level flag

//QTBT high level parameters
//for I slice luma CTB configuration para.
static const Int    MAX_BT_DEPTH  =                                 4;      ///<  <=7
static const Int    MAX_BT_SIZE   =                                32;      ///<  [1<<MIN_QT_SIZE, 1<<CTU_LOG2]
static const Int    MIN_BT_SIZE   =                                 4;      ///<  can be set down to 1<<MIN_CU_LOG2

static const Int    MAX_TT_SIZE   =                                32;      ///<  [1<<MIN_QT_SIZE, 1<<CTU_LOG2]
static const Int    MAX_TT_SIZE_C =                                32;      ///<  [1<<MIN_QT_SIZE, 1<<CTU_LOG2]
static const Int    MIN_TT_SIZE   =                                 4;      ///<  can be set down to 1<<MIN_CU_LOG2
static const Int    MIN_TT_SIZE_C =                                 4;      ///<  can be set down to 1<<MIN_CU_LOG2
                                                                            //for P/B slice CTU config. para.
static const Int    MAX_BT_DEPTH_INTER =                            4;      ///< <=7
static const Int    MAX_BT_SIZE_INTER  =                          128;      ///< for initialization, [1<<MIN_BT_SIZE_INTER, 1<<CTU_LOG2]
static const Int    MIN_BT_SIZE_INTER  =                            4;      ///<

                                                                            //for I slice chroma CTB configuration para. (in luma samples)
static const Int    MAX_BT_DEPTH_C      =                           0;      ///< <=7
static const Int    MAX_BT_SIZE_C       =                          64;      ///< [1<<MIN_QT_SIZE_C, 1<<CTU_LOG2], in luma samples
static const Int    MIN_BT_SIZE_C       =                           4;      ///< can be set down to 4, in luma samples

static const Int    MAX_TT_SIZE_INTER  =                          128;      ///< for initialization, [1<<MIN_BT_SIZE_INTER, 1<<CTU_LOG2]
static const Int    MIN_TT_SIZE_INTER  =                            4;      ///<

static const SplitSeries SPLIT_BITS         =                       5;
static const SplitSeries SPLIT_DMULT        =                       5;
static const SplitSeries SPLIT_MASK         =                      31;      ///< = (1 << SPLIT_BITS) - 1

static const Int    SKIP_DEPTH =                                    3;
static const Int    SKIPHORNOVERQT_DEPTH_TH =                       2;
static const Int    PICTURE_DISTANCE_TH =                           1;
static const Int    FAST_SKIP_DEPTH =                               2;

static const Double PBINTRA_RATIO     =                             1.1;
static const Int    NUM_MRG_SATD_CAND =                             4;
static const Double MRG_FAST_RATIO    =                             1.25;

static const Double AMAXBT_TH32 =                                  15.0;
static const Double AMAXBT_TH64 =                                  30.0;

// need to know for static memory allocation
static const Int MAX_DELTA_QP   =                                   7;      ///< maximum supported delta QP value
static const Int MAX_TESTED_QPs =   ( 1 + 1 + ( MAX_DELTA_QP << 1 ) );      ///< dqp=0 +- max_delta_qp + lossless mode

static const Int COM16_C806_TRANS_PREC =                            2;

static const Int NUM_MERGE_IDX_EXT_CTX =                            5;
static const Int FRUC_MERGE_OFF =                                0x0 ;
static const Int FRUC_MERGE_BILATERALMV =                        0x01;
static const Int FRUC_MERGE_TEMPLATE =                           0x02;
static const Int FRUC_MERGE_TEMPLATE_SIZE =                        4 ;
static const Int FRUC_MERGE_REFINE_MVWEIGHT =                      4 ;
static const Int FRUC_MERGE_REFINE_MINBLKSIZE =                    4 ;
static const unsigned E0104_ALF_MAX_TEMPLAYERID =                  5;       // define to zero to switch of  code
static const unsigned C806_ALF_TEMPPRED_NUM =                      6;

static const Int NB_FRUC_CAND_ADDED =                              2 ; ///< for entire (AMVP and merge) CU, number of added spatial candidates in top, left, top-left, top-right, below-left <0-5>
static const Int NB_FRUC_CAND_ADDED_SUB =                          4 ; ///< for sub-blocks of merge CU, number of added spatial candidates in top, left, top-left, top-right, below-left <0-5>
static const UInt NB_FRUC_CAND_ATMVP =                             4 ; ///< for sub-blocks of merge CU, number of ATMVP candidates

static const Int DMVR_INTME_RANGE =                                 1;

static const Int NTAPS_LUMA               =                         8; ///< Number of taps for luma
static const Int NTAPS_CHROMA             =                         4; ///< Number of taps for chroma
static const Int NTAPS_LUMA_FRUC          =                         2;
static const auto HHI_MULTI_HYPOTHESEIS_MAX_CANDS =                 4;
static const auto HHI_MULTI_HYPOTHESEIS_NUM_WEIGHTS =               3;
static const auto HHI_MULTI_HYPOTHESEIS_WEIGHT_BITS =               4;
static const auto HHI_MULTI_HYPOTHESEIS_SEARCH_RANGE =             16;

static const Int MDBP_SUBPEL_OFFSET_LUMA =   ( NTAPS_LUMA   / 2 ) + 1;
static const Int MDBP_SUBPEL_OFFSET_CHROMA = ( NTAPS_CHROMA / 2 ) + 1;
static const Int MDBP_MIN_MODE =                                  -16;
static const Int MDBP_MAX_MODE =                                   16;
static const Int MDBP_MODE_OFF =                                    0;

static const Int MIN_SIZE_FOR_UPSAMPLING =                         32;

static const Int MRL_NUM_ADD_LINES =                                2; // number of additional mrl reference lines to test (1...3)
static const Int MRL_FAST_LOG2_MIN_SIZE =                           3; // fast mode extension: disable partitions with at least one side smaller than 2^MRL_FAST_LOG2_MIN_SIZE (2: off, 3...5)
#if THRESHOLDING
static const Int MAX_THRESHOLD_CAND =                               4;
#endif

// ====================================================================================================================
// Macro functions
// ====================================================================================================================

struct ClpRng
{
  int min;
  int max;
  int bd;
  int n;
};

struct ClpRngs
{
  ClpRng comp[MAX_NUM_COMPONENT]; ///< the bit depth as indicated in the SPS
  bool used;
  bool chroma;
};

template <typename T> inline T Clip3 (const T minVal, const T maxVal, const T a) { return std::min<T> (std::max<T> (minVal, a) , maxVal); }  ///< general min/max clip
template <typename T> inline T ClipBD( const T x, const Int bitDepth ) { return Clip3( T( 0 ), T( ( 1 << bitDepth ) - 1 ), x ); }
template <typename T> inline T ClipPel (const T a, const ClpRng& clpRng)         { return std::min<T> (std::max<T> (clpRng.min, a) , clpRng.max); }  ///< clip reconstruction

template <typename T> inline Void Check3( T minVal, T maxVal, T a)
{
  CHECK( ( a > maxVal ) || ( a < minVal ), "ERROR: Range check " << minVal << " >= " << a << " <= " << maxVal << " failed" );
}  ///< general min/max clip

extern MsgLevel g_verbosity;

#include <stdarg.h>
inline void msg( MsgLevel level, const char* fmt, ... )
{
  if( g_verbosity >= level )
  {
    va_list args;
    va_start( args, fmt );
    vfprintf( level == ERROR ? stderr : stdout, fmt, args );
    va_end( args );
  }
}

template<typename T> bool isPowerOf2( const T val ) { return ( val & ( val - 1 ) ) == 0; }

#define MEMORY_ALIGN_DEF_SIZE       32  // for use with avx2 (256 bit)

#define ALIGNED_MALLOC              1   ///< use 32-bit aligned malloc/free

#if ALIGNED_MALLOC
#if     ( _WIN32 && ( _MSC_VER > 1300 ) ) || defined (__MINGW64_VERSION_MAJOR)
#define xMalloc( type, len )        _aligned_malloc( sizeof(type)*(len), MEMORY_ALIGN_DEF_SIZE )
#define xFree( ptr )                _aligned_free  ( ptr )
#elif defined (__MINGW32__)
#define xMalloc( type, len )        __mingw_aligned_malloc( sizeof(type)*(len), MEMORY_ALIGN_DEF_SIZE )
#define xFree( ptr )                __mingw_aligned_free( ptr )
#else
namespace detail {
template<typename T>
T* aligned_malloc(size_t len, size_t alignement) {
  T* p = NULL;
  if( posix_memalign( (void**)&p, alignement, sizeof(T)*(len) ) )
  {
    THROW("posix_memalign failed");
  }
  return p;
}
}
#define xMalloc( type, len )        detail::aligned_malloc<type>( len, MEMORY_ALIGN_DEF_SIZE )
#define xFree( ptr )                free( ptr )
#endif

#else
#define xMalloc( type, len )        malloc   ( sizeof(type)*(len) )
#define xFree( ptr )                free     ( ptr )
#endif //#if ALIGNED_MALLOC

#if defined _MSC_VER
#define ALIGN_DATA(nBytes,v) __declspec(align(nBytes)) v
#else
//#elif defined linux
#define ALIGN_DATA(nBytes,v) v __attribute__ ((aligned (nBytes)))
//#else
//#error unknown platform
#endif

#if defined(__GNUC__) && !defined(__clang__)
#    define GCC_VERSION_AT_LEAST(x,y) (__GNUC__ > x || __GNUC__ == x && __GNUC_MINOR__ >= y)
#else
#    define GCC_VERSION_AT_LEAST(x,y) 0
#endif

#ifdef __clang__
#    define CLANG_VERSION_AT_LEAST(x,y) (__clang_major__ > x || __clang_major__ == x && __clang_minor__ >= y)
#else
#    define CLANG_VERSION_AT_LEAST(x,y) 0
#endif

#ifdef __GNUC__
#    define ALWAYS_INLINE __attribute__((always_inline)) inline
#elif defined _MSC_VER
#    define ALWAYS_INLINE __forceinline
#else
#    define ALWAYS_INLINE
#endif

#if HHI_SIMD_OPT

#if defined(__i386__) || defined(i386) || defined(__x86_64__) || defined(_M_X64) || defined (_WIN32) || defined (_MSC_VER)
#define TARGET_SIMD_X86
typedef enum{
  SCALAR = 0,
  SSE41,
  SSE42,
  AVX,
  AVX2,
  AVX512
} X86_VEXT;
#elif defined (__ARM_NEON__)
#define TARGET_SIMD_ARM 1
#else
#error no simd target
#endif

#ifdef TARGET_SIMD_X86
X86_VEXT read_x86_extension_flags(const std::string &extStrId = std::string());
const char* read_x86_extension(const std::string &extStrId);
#endif

#endif //HHI_SIMD_OPT

template <typename ValueType> inline ValueType leftShift       (const ValueType value, const Int shift) { return (shift >= 0) ? ( value                                  << shift) : ( value                                   >> -shift); }
template <typename ValueType> inline ValueType rightShift      (const ValueType value, const Int shift) { return (shift >= 0) ? ( value                                  >> shift) : ( value                                   << -shift); }
template <typename ValueType> inline ValueType leftShift_round (const ValueType value, const Int shift) { return (shift >= 0) ? ( value                                  << shift) : ((value + (ValueType(1) << (-shift - 1))) >> -shift); }
template <typename ValueType> inline ValueType rightShift_round(const ValueType value, const Int shift) { return (shift >= 0) ? ((value + (ValueType(1) << (shift - 1))) >> shift) : ( value                                   << -shift); }

//CASE-BREAK for breakpoints
#if defined ( _MSC_VER ) && defined ( _DEBUG )
#define _CASE(_x) if(_x)
#define _BREAK while(0);
#define _AREA_AT(_a,_x,_y,_w,_h)  (_a.x==_x && _a.y==_y && _a.width==_w && _a.height==_h)
#define _AREA_CONTAINS(_a,_x,_y)  (_a.contains( Position{ _x, _y} ))
#define _UNIT_AREA_AT(_a,_x,_y,_w,_h) (_a.Y().x==_x && _a.Y().y==_y && _a.Y().width==_w && _a.Y().height==_h)
#else
#define _CASE(...)
#define _BREAK
#define _AREA_AT(...)
#define _AREA_CONTAINS(_a,_x,_y)
#define _UNIT_AREA_AT(_a,_x,_y,_w,_h)
#endif

#if HHI_SPLIT_PARALLELISM || HHI_WPP_PARALLELISM
#include <omp.h>

#define PARL_PARAM(DEF) , DEF
#define PARL_PARAM0(DEF) DEF
#else
#define PARL_PARAM(DEF)
#define PARL_PARAM0(DEF)
#endif

//! \}

#endif // end of #ifndef  __COMMONDEF__
