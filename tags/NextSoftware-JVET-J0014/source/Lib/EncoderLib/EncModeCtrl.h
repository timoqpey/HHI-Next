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

/** \file     EncModeCtrl.h
    \brief    Encoder controller for trying out specific modes
*/

#ifndef __ENCMODECTRL__
#define __ENCMODECTRL__

// Include files
#include "EncCfg.h"

#include "CommonLib/CommonDef.h"
#include "CommonLib/CodingStructure.h"

#include <typeinfo>
#include <vector>

//////////////////////////////////////////////////////////////////////////
// Encoder modes to try out
//////////////////////////////////////////////////////////////////////////


enum EncTestModeType
{
  ETM_MERGE_SKIP,
  ETM_MERGE_FRUC,
  ETM_INTER_ME,
  ETM_AFFINE,
  ETM_INTRA,
  ETM_IPCM,
  ETM_SPLIT_PERP,
  ETM_SPLIT_PARL,
  ETM_SPLIT_QT,
  ETM_SPLIT_BT_H,
  ETM_SPLIT_BT_V,
  ETM_SPLIT_BT_H_14,
  ETM_SPLIT_BT_H_34,
  ETM_SPLIT_BT_V_14,
  ETM_SPLIT_BT_V_34,
  ETM_SPLIT_TT_H,
  ETM_SPLIT_TT_V,
  ETM_POST_DONT_SPLIT, // dummy mode to collect the data from the unsplit coding
  ETM_TRIGGER_IMV_LIST,
  ETM_INTER_MULTIHYP,
  ETM_MOTION_VECTOR_REESTIMATION,
  ETM_INVALID
};

enum EncTestModeOpts
{
  ETO_STANDARD    =  0,                   // empty      (standard option)
  ETO_FORCE_MERGE =  1<<0,                // bit   0    (indicates forced merge)
  ETO_IMV_SHIFT   =     1,                // bits  1-3  (imv parameter starts at bit 1)
  ETO_IMV         =  7<<ETO_IMV_SHIFT,    // bits  1-3  (imv parameter uses 3 bits)
  ETO_LIC         =  1<<4,                // bit   4    (local illumination compensation)
  ETO_DUMMY       =  1<<5,                // bit   5    (dummy)
  ETO_SM_SHIFT    =     6,                // bits  6-10 (split modifier parameter start at bit 6)
  ETO_SM          = 31<<ETO_SM_SHIFT,     // bits  6-10 (split modifier uses 4 bits - values range from 0 - 11)
  ETO_NSST_SHIFT  =    11,                // bits 11-12 (nsst index with val 0-3)
  ETO_NSST        = 7 << ETO_NSST_SHIFT,
  ETO_PDPC_SHIFT  = 14,                   // bit  14
  ETO_PDPC        = 1 << ETO_PDPC_SHIFT,
  ETO_1D_PARTITIONS_SHIFT
                  = 15,                   // bits 14-15
  ETO_1D_VER_PARTITION
                  = VER_1D_PARTITION << ETO_1D_PARTITIONS_SHIFT,
  ETO_1D_HOR_PARTITION
                  = HOR_1D_PARTITION << ETO_1D_PARTITIONS_SHIFT,
  ETO_1D_PARTITIONS
                  = /*ETO_1D_VER_PARTITION | ETO_1D_HOR_PARTITION*/3 << ETO_1D_PARTITIONS_SHIFT,
  ETO_MRL_SHIFT   = 17,
  ETO_MRL         = 3 << ETO_MRL_SHIFT,    // bits 16-17 (mrl index with val 0-3)
  ETO_INVALID     = 0xffffffff            // bits 0-31  (invalid option)
};

static void getAreaIdx(const Area& area, const PreCalcValues &pcv, unsigned &idx1, unsigned &idx2, unsigned &idx3, unsigned &idx4)
{
  idx1 = (area.x & pcv.maxCUWidthMask)  >> MIN_CU_LOG2;
  idx2 = (area.y & pcv.maxCUHeightMask) >> MIN_CU_LOG2;
  idx3 = gp_sizeIdxInfo->idxFrom( area.width  );
  idx4 = gp_sizeIdxInfo->idxFrom( area.height );
}

struct EncTestMode
{
  EncTestMode()
    : type( ETM_INVALID ), opts( ETO_INVALID  ), partSize( NUMBER_OF_PART_SIZES ), qp( -1  ), lossless( false ) {}
  EncTestMode( EncTestModeType _type )
    : type( _type       ), opts( ETO_STANDARD ), partSize( SIZE_2Nx2N           ), qp( -1  ), lossless( false ) {}
  EncTestMode( EncTestModeType _type, int _qp, bool _lossless )
    : type( _type       ), opts( ETO_STANDARD ), partSize( SIZE_2Nx2N           ), qp( _qp ), lossless( _lossless ) {}
  EncTestMode( EncTestModeType _type, PartSize _partSize, EncTestModeOpts _opts, int _qp, bool _lossless )
    : type( _type       ), opts( _opts        ), partSize( _partSize            ), qp( _qp ), lossless( _lossless ) {}

  EncTestModeType type;
  EncTestModeOpts opts;
  PartSize        partSize;
  int             qp;
  bool            lossless;
};


inline bool isModeSplit( const EncTestMode& encTestmode )
{
  switch( encTestmode.type )
  {
  case ETM_SPLIT_PERP   :
  case ETM_SPLIT_PARL   :
  case ETM_SPLIT_QT     :
  case ETM_SPLIT_BT_H   :
  case ETM_SPLIT_BT_V   :
  case ETM_SPLIT_TT_H   :
  case ETM_SPLIT_TT_V   :
  case ETM_SPLIT_BT_H_14:
  case ETM_SPLIT_BT_V_14:
  case ETM_SPLIT_BT_H_34:
  case ETM_SPLIT_BT_V_34:
    return true;
  default:
    return false;
  }
}

inline bool isModeNoSplit( const EncTestMode& encTestmode )
{
  return !isModeSplit( encTestmode ) && encTestmode.type != ETM_POST_DONT_SPLIT;
}

inline bool isModeInter( const EncTestMode& encTestmode ) // perhaps remove
{
  return (   encTestmode.type == ETM_INTER_ME
          || encTestmode.type == ETM_MERGE_SKIP
          || encTestmode.type == ETM_MERGE_FRUC
          || encTestmode.type == ETM_AFFINE
          || encTestmode.type == ETM_INTER_MULTIHYP
          || encTestmode.type == ETM_MOTION_VECTOR_REESTIMATION
         );
}

inline PartSplit getPartSplit( const EncTestMode& encTestmode )
{
  switch( encTestmode.type )
  {
  case ETM_SPLIT_PERP   : return CU_PERPENDICULAR_SPLIT;
  case ETM_SPLIT_PARL   : return CU_PARALLEL_SPLIT;
  case ETM_SPLIT_QT     : return CU_QUAD_SPLIT;
  case ETM_SPLIT_BT_H   : return CU_HORZ_SPLIT;
  case ETM_SPLIT_BT_V   : return CU_VERT_SPLIT;
  case ETM_SPLIT_TT_H   : return CU_TRIH_SPLIT;
  case ETM_SPLIT_TT_V   : return CU_TRIV_SPLIT;
  case ETM_SPLIT_BT_H_14: return CU_HORZ_SPLIT_14;
  case ETM_SPLIT_BT_V_14: return CU_VERT_SPLIT_14;
  case ETM_SPLIT_BT_H_34: return CU_HORZ_SPLIT_34;
  case ETM_SPLIT_BT_V_34: return CU_VERT_SPLIT_34;
  default:                return CU_DONT_SPLIT;
  }
}

inline EncTestMode getCSEncMode( const CodingStructure& cs )
{
  return EncTestMode( EncTestModeType( (unsigned)cs.features[ENC_FT_ENC_MODE_TYPE] ),
                      PartSize       ( (unsigned)cs.features[ENC_FT_ENC_MODE_PART] ),
                      EncTestModeOpts( (unsigned)cs.features[ENC_FT_ENC_MODE_OPTS] ) );
}



//////////////////////////////////////////////////////////////////////////
// EncModeCtrl controls if specific modes should be tested
//////////////////////////////////////////////////////////////////////////

struct ComprCUCtx
{
  ComprCUCtx() : testModes(), extraFeatures()
  {
  }

  ComprCUCtx( const CodingStructure& cs, const UInt _minDepth, const UInt _maxDepth, const UInt numExtraFeatures )
    : minDepth      ( _minDepth  )
    , maxDepth      ( _maxDepth  )
    , testModes     (            )
    , lastTestMode  (            )
    , earlySkip     ( false      )
    , bestCS        ( nullptr    )
    , bestCU        ( nullptr    )
    , bestTU        ( nullptr    )
    , extraFeatures (            )
    , extraFeaturesd(            )
    , bestInterCost ( MAX_DOUBLE )
    , bestEmtSize2Nx2N1stPass
                    ( MAX_DOUBLE )
    , skipSecondEMTPass
                    ( false   )
    , interHad      { MAX_UINT,  MAX_UINT,  MAX_UINT   }
#if HHI_SPLIT_PARALLELISM
    , isLevelSplitParallel
                    ( false )
#endif
  {
    getAreaIdx( cs.area.Y(), *cs.pcv, cuX, cuY, cuW, cuH );
    partIdx = ( ( cuX << 8 ) | cuY );

    extraFeatures.reserve( numExtraFeatures );
    extraFeatures.resize ( numExtraFeatures, 0 );

    extraFeaturesd.reserve( numExtraFeatures );
    extraFeaturesd.resize ( numExtraFeatures, 0.0 );
  }

  unsigned                          minDepth;
  unsigned                          maxDepth;
  unsigned                          cuX, cuY, cuW, cuH, partIdx;
  std::vector<EncTestMode>          testModes;
  EncTestMode                       lastTestMode;
  bool                              earlySkip;
  CodingStructure                  *bestCS;
  CodingUnit                       *bestCU;
  TransformUnit                    *bestTU;
  static_vector<Int64,  30>         extraFeatures;
  static_vector<double, 30>         extraFeaturesd;
  double                            bestInterCost;
  double                            bestEmtSize2Nx2N1stPass;
  bool                              skipSecondEMTPass;
  Distortion                        interHad[3];
#if HHI_SPLIT_PARALLELISM
  bool                              isLevelSplitParallel;
#endif

  template<typename T> T    get( int ft )       const { return typeid(T) == typeid(double) ? (T&)extraFeaturesd[ft] : T(extraFeatures[ft]); }
  template<typename T> void set( int ft, T val )      { extraFeatures [ft] = Int64( val ); }
  void                      set( int ft, double val ) { extraFeaturesd[ft] = val; }
};

//////////////////////////////////////////////////////////////////////////
// EncModeCtrl - abstract class specifying the general flow of mode control
//////////////////////////////////////////////////////////////////////////

class EncModeCtrl
{
protected:

  const EncCfg         *m_pcEncCfg;
  const class RateCtrl *m_pcRateCtrl;
        class RdCost   *m_pcRdCost;
  const Slice          *m_slice;
#if SHARP_LUMA_DELTA_QP
  int                   m_lumaLevelToDeltaQPLUT[LUMA_LEVEL_TO_DQP_LUT_MAXSIZE];
  int                   m_lumaQPOffset;
#endif
  bool                  m_fastDeltaQP;
  static_vector<ComprCUCtx, ( MAX_CU_DEPTH << 2 )> m_ComprCUCtxList;
#if HHI_SPLIT_PARALLELISM
  int                   m_runNextInParallel;
#endif

public:

  virtual ~EncModeCtrl              () {}

  virtual void initCTUEncoding      ( const Slice &slice )                                                                  = 0;
  virtual void initCULevel          ( Partitioner &partitioner, const CodingStructure& cs )                                 = 0;
  virtual void finishCULevel        ( Partitioner &partitioner )                                                            = 0;

protected:

  virtual bool tryMode              ( const EncTestMode& encTestmode, const CodingStructure &cs, Partitioner& partitioner ) = 0;

public:

  virtual bool useModeResult        ( const EncTestMode& encTestmode, CodingStructure*& tempCS,  Partitioner& partitioner ) = 0;
#if HHI_SPLIT_PARALLELISM
  virtual void copyState            ( const EncModeCtrl& other, const UnitArea& area );
  virtual int  getNumParallelJobs   ( const CodingStructure &cs, Partitioner& partitioner )                                 const { return 1;     }
  virtual bool isParallelSplit      ( const CodingStructure &cs, Partitioner& partitioner )                                 const { return false; }
  virtual bool parallelJobSelector  ( const EncTestMode& encTestmode, const CodingStructure &cs, Partitioner& partitioner ) const { return true;  }
          void setParallelSplit     ( bool val ) { m_runNextInParallel = val; }
#endif

  void         init                 ( EncCfg *pCfg, RateCtrl *pRateCtrl, RdCost *pRdCost );
  bool         tryModeMaster        ( const EncTestMode& encTestmode, const CodingStructure &cs, Partitioner& partitioner );
  bool         nextMode             ( const CodingStructure &cs, Partitioner &partitioner );
  EncTestMode  currTestMode         () const;
  EncTestMode  lastTestMode         () const;
  void         setEarlySkipDetected ();
  virtual void setBest              ( CodingStructure& cs );
  bool         hasOnlySplitModes    () const;
  bool         anyMode              () const;

  const ComprCUCtx& getComprCUCtx   () { CHECK( m_ComprCUCtxList.empty(), "Accessing empty list!"); return m_ComprCUCtxList.back(); }

#if SHARP_LUMA_DELTA_QP
  void                  initLumaDeltaQpLUT();
  int                   calculateLumaDQP  ( const CPelBuf& rcOrg );
#endif
  void setFastDeltaQp                 ( bool b )                {        m_fastDeltaQP = b;                               }
  bool getFastDeltaQp                 ()                  const { return m_fastDeltaQP;                                   }

  double getBestInterCost             ()                  const { return m_ComprCUCtxList.back().bestInterCost;           }
  Distortion getInterHad              ( int i )           const { return m_ComprCUCtxList.back().interHad[i];             }
  void enforceInterHad                ( int i, Distortion had ) {        m_ComprCUCtxList.back().interHad[i] = had;       }
  double getEmtSize2Nx2NFirstPassCost ()                  const { return m_ComprCUCtxList.back().bestEmtSize2Nx2N1stPass; }
  bool getSkipSecondEMTPass           ()                  const { return m_ComprCUCtxList.back().skipSecondEMTPass;       }
  void setSkipSecondEMTPass           ( bool b )                {        m_ComprCUCtxList.back().skipSecondEMTPass = b;   }
  CodingUnit* getBestCuSoFar          ()                  const { return m_ComprCUCtxList.back().bestCU; }

protected:
  void xExtractFeatures ( const EncTestMode encTestmode, CodingStructure& cs );
  void xGetMinMaxQP     ( int& iMinQP, int& iMaxQP, const CodingStructure& cs, const Partitioner &pm, const int baseQP, const SPS& sps, const PPS& pps, const bool splitMode );
  int  xComputeDQP      ( const CodingStructure &cs, const Partitioner &pm );
};


//////////////////////////////////////////////////////////////////////////
// some utility interfaces that expose some functionality that can be used without concerning about which particular controller is used
//////////////////////////////////////////////////////////////////////////

struct SaveLoadStruct
{
  unsigned        split;
  SaveLoadTag     tag;
  unsigned        interDir;
  bool            mergeFlag;
  bool            LICFlag;
  unsigned        imv;
  unsigned        nsstIdx;
  bool            pdpc;
  unsigned        partIdx;
  unsigned        emtTuIndex;
  unsigned        emtCuFlag;
  unsigned        frucMode;
  bool            affineFlag;
  bool            multiHyp;
  bool            intraFTM;
  unsigned        mode1d;
  unsigned        mrlIdx;
};

class SaveLoadEncInfoCtrl
{
protected:

  SaveLoadStruct& getSaveLoadStruct    ( const UnitArea& area );
  SaveLoadStruct& getSaveLoadStructQuad( const UnitArea& area );

  void create   ();
  void destroy  ();
  void init     ( const Slice &slice );
#if HHI_SPLIT_PARALLELISM
  void copyState( const SaveLoadEncInfoCtrl &other, const UnitArea& area );
#endif

private:

  Slice const     *m_slice_sls;
  SaveLoadStruct **m_saveLoadInfo;

public:

  virtual ~SaveLoadEncInfoCtrl() { }

  SaveLoadTag getSaveLoadTag    ( const UnitArea& area );
  unsigned getSaveLoadInterDir  ( const UnitArea& area );
  unsigned getSaveLoadNsstIdx   ( const UnitArea& area );
  bool     getSaveLoadPdpc      ( const UnitArea& area );
  bool     getSaveLoadEmtCuFlag ( const UnitArea& area );
  unsigned getSaveLoadEmtTuIndex( const UnitArea& area );
  unsigned getSaveLoadFrucMode  ( const UnitArea& area );
  unsigned getSaveLoadAffineFlag( const UnitArea& area );
  bool     getSaveLoadMultiHyp  ( const UnitArea& area );
  bool     getSaveLoadIntraFTM  ( const UnitArea& area );
  unsigned getSaveLoadMode1d    ( const UnitArea& area );
  unsigned getSaveLoadMrlIdx    ( const UnitArea& area );
};

static const int MAX_STORED_CU_INFO_REFS = 4;

struct CodedCUInfo
{
  bool isInter;
  bool isIntra;
  bool isSkip;

  bool validMv[NUM_REF_PIC_LIST_01][MAX_STORED_CU_INFO_REFS];
  Mv   saveMv [NUM_REF_PIC_LIST_01][MAX_STORED_CU_INFO_REFS];
#if HHI_SPLIT_PARALLELISM

  uint64_t
       temporalId;
#endif
};

class CacheBlkInfoCtrl
{
private:

  unsigned         m_numWidths, m_numHeights;
  Slice const     *m_slice_chblk;
  // x in CTU, y in CTU, width, height
  CodedCUInfo   ***m_codedCUInfo[MAX_CU_SIZE >> MIN_CU_LOG2][MAX_CU_SIZE >> MIN_CU_LOG2];

protected:

  void create   ();
  void destroy  ();
#if HHI_SPLIT_PARALLELISM
public:
#endif
  void init     ( const Slice &slice );
#if HHI_SPLIT_PARALLELISM
private:
  uint64_t
       m_currTemporalId;
public:
  void tick     () { m_currTemporalId++; CHECK( m_currTemporalId <= 0, "Problem with integer overflow!" ); }
  // mark the state of the blk as changed within the current temporal id
  void copyState( const CacheBlkInfoCtrl &other, const UnitArea& area );
protected:
  void touch    ( const UnitArea& area );
#endif

  CodedCUInfo& getBlkInfo( const UnitArea& area );

public:

  virtual ~CacheBlkInfoCtrl() {}

  bool isSkip ( const UnitArea& area );

  bool getMv  ( const UnitArea& area, const RefPicList refPicList, const int iRefIdx,       Mv& rMv ) const;
  void setMv  ( const UnitArea& area, const RefPicList refPicList, const int iRefIdx, const Mv& rMv );
};

//////////////////////////////////////////////////////////////////////////
// EncModeCtrlHEVC - allows and controls modes available in HEVC
//                 - all part sizes, RQT, only quad splits for CUs and RQT
//////////////////////////////////////////////////////////////////////////

class EncModeCtrlQTwithRQT : public EncModeCtrl
{
  enum ExtraFeatures
  {
    PARENT_PART_SIZE = 0,
    PRE_AMP_SKIP,
    PRE_AMP_MERGE,
    PRE_AMP_PART_SIZE,
    PRE_AMP_WIDTH,
    TRY_AMP_MRG_HORZ,
    TRY_AMP_MRG_VERT,
    EARLY_SKIP_INTRA,
    DISABLE_LIC,
    BEST_NO_IMV_COST,
    BEST_IMV_COST,
    LAST_NSST_IDX,
    SKIP_OTHER_NSST,
    NUM_EXTRA_FEATURES
  };

  struct ImvCtx
  {
    // two times part size space to allow for all part sizes to be combined with all LIC modes - extend if needed)
    static_vector<EncTestMode, 3 * NUMBER_OF_PART_SIZES> testModes;
    static_vector<double,      3 * NUMBER_OF_PART_SIZES> testCosts;

    int loop;
    int idx;
    int max;


    ImvCtx() : testModes(), testCosts(), loop( 0 ), idx( 0 ), max( 0 )
    {
    }
  };

  static_vector<ImvCtx, MAX_CU_DEPTH> m_ImvCtxList;

public:

  virtual void initCTUEncoding  ( const Slice &slice );
  virtual void initCULevel      ( Partitioner &partitioner, const CodingStructure& cs );
  virtual void finishCULevel    ( Partitioner &partitioner );

  virtual bool tryMode          ( const EncTestMode& encTestmode, const CodingStructure &cs, Partitioner& partitioner );
  virtual bool useModeResult    ( const EncTestMode& encTestmode, CodingStructure*& tempCS,  Partitioner& partitioner );

#if HHI_SPLIT_PARALLELISM
  virtual void copyState        ( const EncModeCtrl& other, const UnitArea& area );
#endif
};

//////////////////////////////////////////////////////////////////////////
// EncModeCtrlMTnoRQT - allows and controls modes introduced by QTBT (inkl. multi-type-tree)
//                    - only 2Nx2N, no RQT, additional binary/triary CU splits
//////////////////////////////////////////////////////////////////////////

class EncModeCtrlMTnoRQT : public EncModeCtrl, public SaveLoadEncInfoCtrl, public CacheBlkInfoCtrl
{
  enum ExtraFeatures
  {
    DID_HORZ_SPLIT = 0,
    DID_VERT_SPLIT,
    DID_QUAD_SPLIT,
    BEST_HORZ_SPLIT_COST,
    BEST_VERT_SPLIT_COST,
    BEST_TRIH_SPLIT_COST,
    BEST_TRIV_SPLIT_COST,
    DO_TRIH_SPLIT,
    DO_TRIV_SPLIT,
    BEST_NON_SPLIT_COST,
    HISTORY_NEED_TO_SAVE,
    HISTORY_DO_SAVE,
    SAVE_LOAD_TAG,
    BEST_NO_IMV_COST,
    BEST_IMV_COST,
    LAST_NSST_IDX,
    SKIP_OTHER_NSST,
    QT_BEFORE_BT,
    IS_BEST_NOSPLIT_SKIP,
    MAX_QT_SUB_DEPTH,
    NUM_EXTRA_FEATURES
  };

  unsigned m_skipThreshold;

public:

  EncModeCtrlMTnoRQT ();
  ~EncModeCtrlMTnoRQT();

  virtual void initCTUEncoding    ( const Slice &slice );
  virtual void initCULevel        ( Partitioner &partitioner, const CodingStructure& cs );
  virtual void finishCULevel      ( Partitioner &partitioner );

  virtual bool tryMode            ( const EncTestMode& encTestmode, const CodingStructure &cs, Partitioner& partitioner );
  virtual bool useModeResult      ( const EncTestMode& encTestmode, CodingStructure*& tempCS,  Partitioner& partitioner );

#if HHI_SPLIT_PARALLELISM
  virtual void copyState          ( const EncModeCtrl& other, const UnitArea& area );

  virtual int  getNumParallelJobs ( const CodingStructure &cs, Partitioner& partitioner ) const;
  virtual bool isParallelSplit    ( const CodingStructure &cs, Partitioner& partitioner ) const;
  virtual bool parallelJobSelector( const EncTestMode& encTestmode, const CodingStructure &cs, Partitioner& partitioner ) const;
#endif
};

//////////////////////////////////////////////////////////////////////////
// EncModeCtrlGBS - mode control for general binary splits
//////////////////////////////////////////////////////////////////////////

class EncModeCtrlGBS : public EncModeCtrl, public CacheBlkInfoCtrl
{
  enum ExtraFeatures
  {
    BEST_NO_IMV_COST = 0,
    BEST_IMV_COST,
    DID_QUAD_SPLIT,
    QUAD_COST,
    MAX_QT_SUBDEPTH,
    QT_BEFORE_BT,
    PERP_MIN_SIZE,
    PARL_MIN_SIZE,
    DID_PERP_12_SPLIT,
    DID_PARL_12_SPLIT,
    PERP_12_COST,
    PARL_12_COST,
    BEST_NO_SPLIT_COST,
    LAST_NSST_IDX,
    SKIP_OTHER_NSST,
    IS_BEST_NOSPLIT_SKIP,
    NUM_EXTRA_FEATURES
  };

public:

  EncModeCtrlGBS();
  ~EncModeCtrlGBS();

  virtual void initCTUEncoding    ( const Slice &slice );
  virtual void initCULevel        ( Partitioner &partitioner, const CodingStructure& cs );
  virtual void finishCULevel      ( Partitioner &partitioner );

  virtual bool tryMode            ( const EncTestMode& encTestmode, const CodingStructure &cs, Partitioner& partitioner );
  virtual bool useModeResult      ( const EncTestMode& encTestmode, CodingStructure*& tempCS,  Partitioner& partitioner );

  virtual void setBest            ( CodingStructure& cs );

#if HHI_SPLIT_PARALLELISM
  virtual void copyState          ( const EncModeCtrl& other, const UnitArea& area );

  virtual int  getNumParallelJobs ( const CodingStructure &cs, Partitioner& partitioner ) const;
  virtual bool isParallelSplit    ( const CodingStructure &cs, Partitioner& partitioner ) const;
  virtual bool parallelJobSelector( const EncTestMode& encTestmode, const CodingStructure &cs, Partitioner& partitioner ) const;

#endif
};

//! \}

#endif // __ENCMODECTRL__
