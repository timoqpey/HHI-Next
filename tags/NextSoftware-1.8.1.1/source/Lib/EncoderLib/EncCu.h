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

/** \file     EncCu.h
    \brief    Coding Unit (CU) encoder class (header)
*/

#ifndef __ENCCU__
#define __ENCCU__

// Include files
#include "CommonLib/CommonDef.h"
#include "CommonLib/IntraPrediction.h"
#include "CommonLib/InterPrediction.h"
#include "CommonLib/TrQuant.h"
#include "CommonLib/Unit.h"
#include "CommonLib/UnitPartitioner.h"

#include "CABACWriter.h"
#include "IntraSearch.h"
#include "InterSearch.h"
#include "RateCtrl.h"
#include "EncModeCtrl.h"
//! \ingroup EncoderLib
//! \{

class EncLib;
class HLSWriter;
class EncSlice;

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// CU encoder class
class EncCu
{
private:

  struct CtxPair
  {
    Ctx start;
    Ctx best;
  };

  std::vector<CtxPair>  m_CtxBuffer;
  CtxPair*              m_CurrCtx;
  CtxCache*             m_CtxCache;

  //  Data : encoder control
  int                   m_cuChromaQpOffsetIdxPlus1; // if 0, then cu_chroma_qp_offset_flag will be 0, otherwise cu_chroma_qp_offset_flag will be 1.

  XUCache               m_unitCache;

  CodingStructure    ***m_pTempCS;
  CodingStructure    ***m_pBestCS;
  CodingStructure    ***m_pTempCUWoOBMC; ///< Temporary CUs in each depth
  PelStorage          **m_pPredBufWoOBMC;

  //  Access channel
  EncCfg*               m_pcEncCfg;
  IntraSearch*          m_pcIntraSearch;
  InterSearch*          m_pcInterSearch;
  TrQuant*              m_pcTrQuant;
  RdCost*               m_pcRdCost;

  CABACWriter*          m_CABACEstimator;
  RateCtrl*             m_pcRateCtrl;

  CodingStructure    ***m_pImvTempCS;

  EncModeCtrl          *m_modeCtrl;

  PelStorage            m_acMergeBuffer[MRG_MAX_NUM_CANDS];

  MotionInfo            m_SubPuMiBuf   [( MAX_CU_SIZE * MAX_CU_SIZE ) >> ( MIN_CU_LOG2 << 1 )];
  MotionInfo            m_SubPuExtMiBuf[( MAX_CU_SIZE * MAX_CU_SIZE ) >> ( MIN_CU_LOG2 << 1 )];

  MotionInfo            m_SubPuFrucBuf [( MAX_CU_SIZE * MAX_CU_SIZE ) >> ( MIN_CU_LOG2 << 1 )];

public:
  /// copy parameters from encoder class
  void  init                ( EncLib* pcEncLib, const SPS& sps );

  /// create internal buffers
  void  create              ( EncCfg* encCfg );

  /// destroy internal buffers
  void  destroy             ();

  /// CTU analysis function
  void  compressCtu         ( CodingStructure& cs, const UnitArea& area, unsigned ctuRsAddr );
  /// CTU encoding function
  int   updateCtuDataISlice ( const CPelBuf buf );

  EncModeCtrl* getModeCtrl  () { return m_modeCtrl; }

  ~EncCu();

protected:

  void xCompressCU            ( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &pm );

  void xCheckBestMode         ( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &pm, const EncTestMode& encTestmode );

#if SHARP_LUMA_DELTA_QP
  void xCheckModeSplit        ( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &pm, const EncTestMode& encTestMode , Double& splitTotalCost);
#else
  void xCheckModeSplit        ( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &pm, const EncTestMode& encTestMode );
#endif

  void xCheckRDCostIntra      ( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &pm, const EncTestMode& encTestMode );
  void xCheckIntraPCM         ( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &pm, const EncTestMode& encTestMode );

  void xCheckDQP              ( CodingStructure& cs, bool bKeepCtx = false);
  void xFillPCMBuffer         ( CodingUnit &cu);

  void xCheckRDCostAffineMerge2Nx2N( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode );
  void xCheckRDCostInter      ( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &pm, const EncTestMode& encTestMode );
  bool xCheckRDCostInterIMV   ( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &pm, const EncTestMode& encTestMode );
  void xCheckRDCostInterWoOBMC( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &pm, const EncTestMode& encTestMode );

  void xEncodeDontSplit       ( CodingStructure &cs, Partitioner &partitioner);

  void xCheckRDCostMerge2Nx2N ( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &pm, const EncTestMode& encTestMode );

  void xCheckRDCostMerge2Nx2NFRUC( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode );
};

//! \}

#endif // __ENCMB__
