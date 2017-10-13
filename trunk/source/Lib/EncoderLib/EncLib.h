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

/** \file     EncLib.h
    \brief    encoder class (header)
*/

#ifndef __ENCTOP__
#define __ENCTOP__

// Include files
#include "CommonLib/TrQuant.h"
#include "CommonLib/LoopFilter.h"
#include "CommonLib/NAL.h"

#include "Utilities/VideoIOYuv.h"

#include "EncCfg.h"
#include "EncGOP.h"
#include "EncSlice.h"
#include "VLCWriter.h"
#include "CABACWriter.h"
#include "InterSearch.h"
#include "IntraSearch.h"
#include "EncSampleAdaptiveOffset.h"
#include "EncAdaptiveLoopFilter.h"
#include "RateCtrl.h"
//! \ingroup EncoderLib
//! \{

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// encoder class
class EncLib : public EncCfg
{
private:
  // picture
  Int                       m_iPOCLast;                     ///< time index (POC)
  Int                       m_iNumPicRcvd;                  ///< number of received pictures
  UInt                      m_uiNumAllPicCoded;             ///< number of coded pictures
  PicList                   m_cListPic;                     ///< dynamic list of pictures

  // encoder search
  InterSearch               m_cInterSearch;                 ///< encoder search class
  IntraSearch               m_cIntraSearch;                 ///< encoder search class
  // coding tool
  TrQuant                   m_cTrQuant;                     ///< transform & quantization class
  LoopFilter                m_cLoopFilter;                  ///< deblocking filter class
  EncSampleAdaptiveOffset   m_cEncSAO;                      ///< sample adaptive offset class
  EncAdaptiveLoopFilter     m_cEncALF;
  HLSWriter                 m_HLSWriter;                    ///< CAVLC encoder
  CABACEncoder              m_CABACEncoder;

  // processing unit
  EncGOP                    m_cGOPEncoder;                  ///< GOP encoder
  EncSlice                  m_cSliceEncoder;                ///< slice encoder
  EncCu                     m_cCuEncoder;                   ///< CU encoder
  // SPS
  ParameterSetMap<SPS>      m_spsMap;                       ///< SPS. This is the base value. This is copied to PicSym
  ParameterSetMap<PPS>      m_ppsMap;                       ///< PPS. This is the base value. This is copied to PicSym
  // RD cost computation
  RdCost                    m_cRdCost;                      ///< RD cost computation class
  CtxCache                  m_CtxCache;                    ///< buffer for temporarily stored context models

  // quality control
  RateCtrl                  m_cRateCtrl;                    ///< Rate control class

protected:
  Void  xGetNewPicBuffer  ( std::list<PelUnitBuf*>& rcListPicYuvRecOut, Picture*& rpcPic, Int ppsId ); ///< get picture buffer which will be processed. If ppsId<0, then the ppsMap will be queried for the first match.
  Void  xInitVPS          (VPS &vps, const SPS &sps); ///< initialize VPS from encoder options
  Void  xInitSPS          (SPS &sps);                 ///< initialize SPS from encoder options
  Void  xInitPPS          (PPS &pps, const SPS &sps); ///< initialize PPS from encoder options
  Void  xInitScalingLists (SPS &sps, PPS &pps);   ///< initialize scaling lists
  Void  xInitHrdParameters(SPS &sps);                 ///< initialize HRD parameters

  Void  xInitPPSforTiles  (PPS &pps);
  Void  xInitRPS          (SPS &sps, Bool isFieldCoding);           ///< initialize PPS from encoder options

public:
  EncLib();
  virtual ~EncLib();

  Void      create          ();
  Void      destroy         ();
  Void      init            (Bool isFieldCoding);
  Void      deletePicBuffer ();

  // -------------------------------------------------------------------------------------------------------------------
  // member access functions
  // -------------------------------------------------------------------------------------------------------------------

  PicList*                getListPic            ()            { return  &m_cListPic;             }
  InterSearch*            getInterSearch        ()            { return  &m_cInterSearch;         }
  IntraSearch*            getIntraSearch        ()            { return  &m_cIntraSearch;         }

  TrQuant*                getTrQuant            ()            { return  &m_cTrQuant;             }
  LoopFilter*             getLoopFilter         ()            { return  &m_cLoopFilter;          }
  EncSampleAdaptiveOffset* getSAO               ()            { return  &m_cEncSAO;              }
  EncAdaptiveLoopFilter*  getALF                ()            { return  &m_cEncALF;              }
  EncGOP*                 getGOPEncoder         ()            { return  &m_cGOPEncoder;          }
  EncSlice*               getSliceEncoder       ()            { return  &m_cSliceEncoder;        }
  EncCu*                  getCuEncoder          ()            { return  &m_cCuEncoder;           }
  HLSWriter*              getHLSWriter          ()            { return  &m_HLSWriter;            }
  CABACEncoder*           getCABACEncoder       ()            { return  &m_CABACEncoder;         }

  RdCost*                 getRdCost             ()            { return  &m_cRdCost;              }
  CtxCache*               getCtxCache           ()            { return  &m_CtxCache;             }
  RateCtrl*               getRateCtrl           ()            { return  &m_cRateCtrl;            }
  Void selectReferencePictureSet(Slice* slice, Int POCCurr, Int GOPid );
  Int getReferencePictureSetIdxForSOP(Int POCCurr, Int GOPid );

  Bool                   PPSNeedsWriting(Int ppsId);
  Bool                   SPSNeedsWriting(Int spsId);

  // -------------------------------------------------------------------------------------------------------------------
  // encoder function
  // -------------------------------------------------------------------------------------------------------------------

  /// encode several number of pictures until end-of-sequence
  Void encode( Bool bEos,
               PelStorage* pcPicYuvOrg,
               PelStorage* pcPicYuvTrueOrg, const InputColourSpaceConversion snrCSC, // used for SNR calculations. Picture in original colour space.
               std::list<PelUnitBuf*>& rcListPicYuvRecOut,
               std::list<AccessUnit>& accessUnitsOut, Int& iNumEncoded );

  /// encode several number of pictures until end-of-sequence
  Void encode( Bool bEos,
               PelStorage* pcPicYuvOrg,
               PelStorage* pcPicYuvTrueOrg, const InputColourSpaceConversion snrCSC, // used for SNR calculations. Picture in original colour space.
               std::list<PelUnitBuf*>& rcListPicYuvRecOut,
               std::list<AccessUnit>& accessUnitsOut, Int& iNumEncoded, Bool isTff);

  Void printSummary(Bool isField) { m_cGOPEncoder.printOutSummary (m_uiNumAllPicCoded, isField, m_printMSEBasedSequencePSNR, m_printSequenceMSE, m_spsMap.getFirstPS()->getBitDepths()); }

};

//! \}

#endif // __ENCTOP__

