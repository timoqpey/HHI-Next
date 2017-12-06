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

/** \file     EncSlice.h
    \brief    slice encoder class (header)
*/

#ifndef __ENCSLICE__
#define __ENCSLICE__

// Include files
#include "EncCu.h"
#include "WeightPredAnalysis.h"
#include "RateCtrl.h"

#include "CommonLib/CommonDef.h"
#include "CommonLib/Picture.h"

//! \ingroup EncoderLib
//! \{

class EncLib;
class EncGOP;

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// slice encoder class
class EncSlice
  : public WeightPredAnalysis
{
private:
  // encoder configuration
  EncCfg*                 m_pcCfg;                              ///< encoder configuration class

  // pictures
  PicList*                m_pcListPic;                          ///< list of pictures

  // processing units
  EncGOP*                 m_pcGOPEncoder;                       ///< GOP encoder
  EncCu*                  m_pcCuEncoder;                        ///< CU encoder

  // encoder search
  InterSearch*            m_pcInterSearch;                       ///< encoder search class

  // coding tools
  CABACWriter*            m_CABACWriter;
  TrQuant*                m_pcTrQuant;                          ///< transform & quantization

  // RD optimization
  RdCost*                 m_pcRdCost;                           ///< RD cost computation
  CABACWriter*            m_CABACEstimator;
  UInt64                  m_uiPicTotalBits;                     ///< total bits for the picture
  UInt64                  m_uiPicDist;                          ///< total distortion for the picture
  std::vector<Double>     m_vdRdPicLambda;                      ///< array of lambda candidates
  std::vector<Double>     m_vdRdPicQp;                          ///< array of picture QP candidates (double-type for lambda)
  std::vector<Int>        m_viRdPicQp;                          ///< array of picture QP candidates (Int-type)
  RateCtrl*               m_pcRateCtrl;                         ///< Rate control manager
  UInt                    m_uiSliceSegmentIdx;
  Ctx                     m_lastSliceSegmentEndContextState;    ///< context storage for state at the end of the previous slice-segment (used for dependent slices only).
  Ctx                     m_entropyCodingSyncContextState;      ///< context storate for state of contexts at the wavefront/WPP/entropy-coding-sync second CTU of tile-row
  CABACEncoder*           m_CABACEncoder;
  SliceType               m_encCABACTableIdx;
#if SHARP_LUMA_DELTA_QP
  Int                     m_gopID;
#endif
#if HHI_HLM_USE_QPA
  double*                 m_uEnerHpCtu;                         ///< CTU-wise L2 or squared L1 norm of high-passed luma input
  Pel*                    m_iOffsetCtu;                         ///< CTU-wise DC offset (later QP index offset) of luma input
#endif

#if SHARP_LUMA_DELTA_QP
  Double   calculateLambda( const Slice* slice, const Int GOPid, const Int depth, const Double refQP, const Double dQP, Int &iQP );
#endif
  Void     setUpLambda(Slice* slice, const Double dLambda, Int iQP);
  Void     calculateBoundingCtuTsAddrForSlice(UInt &startCtuTSAddrSlice, UInt &boundingCtuTSAddrSlice, Bool &haveReachedTileBoundary, Picture* pcPic, const Int sliceMode, const Int sliceArgument);

public:
  EncSlice();
  virtual ~EncSlice();

  Void    create              ( Int iWidth, Int iHeight, ChromaFormat chromaFormat, UInt iMaxCUWidth, UInt iMaxCUHeight, UChar uhTotalDepth );
  Void    destroy             ();
  Void    init                ( EncLib* pcEncLib, const SPS& sps );

  /// preparation of slice encoding (reference marking, QP and lambda)
  Void    initEncSlice        ( Picture*  pcPic, const Int pocLast, const Int pocCurr,
                                const Int iGOPid,   Slice*& rpcSlice, const Bool isField );
  Void    resetQP             ( Picture* pic, Int sliceQP, Double lambda );
#if SHARP_LUMA_DELTA_QP
  Void    updateLambda        ( Slice* slice, Double dQP);
#endif

  // compress and encode slice
  Void    precompressSlice    ( Picture* pcPic                                     );      ///< precompress slice for multi-loop slice-level QP opt.
  Void    compressSlice       ( Picture* pcPic, const Bool bCompressEntireSlice, const Bool bFastDeltaQP );      ///< analysis stage of slice
  Void    calCostSliceI       ( Picture* pcPic );
  Void    encodeSlice         ( Picture* pcPic, OutputBitstream* pcSubstreams, UInt &numBinsCoded );

  // misc. functions
  Void    setSearchRange      ( Slice* pcSlice  );                                  ///< set ME range adaptively

  EncCu*  getCUEncoder        ()                    { return m_pcCuEncoder; }                        ///< CU encoder
  Void    xDetermineStartAndBoundingCtuTsAddr  ( UInt& startCtuTsAddr, UInt& boundingCtuTsAddr, Picture* pcPic );
  UInt    getSliceSegmentIdx  ()                    { return m_uiSliceSegmentIdx;       }
  Void    setSliceSegmentIdx  (UInt i)              { m_uiSliceSegmentIdx = i;          }

  SliceType getEncCABACTableIdx() const             { return m_encCABACTableIdx;        }

private:
  Double  xGetQPValueAccordingToLambda ( Double lambda );
};

//! \}

#endif // __ENCSLICE__
