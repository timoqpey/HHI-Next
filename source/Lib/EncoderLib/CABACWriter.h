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

/** \file     CABACWriter.h
 *  \brief    Writer for low level syntax
 */

#ifndef __CABACWRITER__
#define __CABACWRITER__

#include "CommonLib/BitStream.h"
#include "CommonLib/ContextModelling.h"
#include "BinEncoder.h"


//! \ingroup EncoderLib
//! \{


class CABACDataStore;
class CABACWriter
{
public:
  CABACWriter( BinEncIf& binEncoder ) : m_BinEncoder( binEncoder ), m_Bitstream( 0 ) { m_TestCtx = m_BinEncoder.getCtx(); }
  virtual ~CABACWriter() {}

public:
  void        initCtxModels             ( const Slice&                  slice,
                                          const CABACDataStore*         cabacDataStore );
  SliceType   getCtxInitId              ( const Slice&                  slice );
  void        initBitstream             ( OutputBitstream*              bitstream )           { m_Bitstream = bitstream; m_BinEncoder.init( m_Bitstream ); }

  const Ctx&  getCtx                    ()                                            const   { return m_BinEncoder.getCtx();  }
  Ctx&        getCtx                    ()                                                    { return m_BinEncoder.getCtx();  }

  void        start                     ()                                                    { m_BinEncoder.start(); }
  void        resetBits                 ()                                                    { m_BinEncoder.resetBits(); }
  uint64_t    getEstFracBits            ()                                            const   { return m_BinEncoder.getEstFracBits(); }
  uint32_t    getNumBins                ()                                                    { return m_BinEncoder.getNumBins(); }
  bool        isEncoding                ()                                                    { return m_BinEncoder.isEncoding(); }

  void        enableBinStore            ( const Slice&                  slice,
                                          CABACDataStore&               cabacDataStore );
  void        estWinSizes               ( const Slice&                  slice,
                                          CABACDataStore&               cabacDataStore ) const;
public:
  // slice segment data (clause 7.3.8.1)
  void        end_of_slice              ();

  // coding tree unit (clause 7.3.8.2)
  void        coding_tree_unit          (       CodingStructure&        cs,       const UnitArea&   area,       int (&qps)[2],  unsigned ctuRsAddr,  bool skipSao = false );

  // sao (clause 7.3.8.3)
  void        sao                       ( const Slice&                  slice,    unsigned          ctuRsAddr );
  void        sao_block_pars            ( const SAOBlkParam&            saoPars,  const BitDepths&  bitDepths,  bool* sliceEnabled, bool leftMergeAvail, bool aboveMergeAvail, bool onlyEstMergeInfo );
  void        sao_offset_pars           ( const SAOOffset&              ctbPars,  ComponentID       compID,     bool sliceEnabled,  int bitDepth );
  void        alf                       ( const Slice&                  slice,    const ALFParam& alfParam );
  void        alf                       ( const ALFParam&               alfParam, SliceType sliceType, bool isGALF );

  // coding (quad)tree (clause 7.3.8.4)
  void        coding_tree               ( const CodingStructure&        cs,       Partitioner&      pm,         CUCtx& cuCtx );
  void        gen_bin_split_mode        ( const PartSplit               split,    const CodingStructure& cs,    Partitioner& pm );
  void        gen_bin_split_mod         ( const PartSplit               split,    const CodingStructure& cs,    Partitioner& pm );
  void        split_cu_flag             ( bool                          split,    const CodingStructure& cs,    Partitioner& pm );
  void        split_cu_mode_mt          ( const PartSplit               split,    const CodingStructure& cs,    Partitioner& pm );

  // coding unit (clause 7.3.8.5)
  void        coding_unit               ( const CodingUnit&             cu,       Partitioner&      pm,         CUCtx& cuCtx );
  void        cu_transquant_bypass_flag ( const CodingUnit&             cu );
  void        cu_skip_flag              ( const CodingUnit&             cu );
  void        pred_mode                 ( const CodingUnit&             cu );
  void        part_mode                 ( const CodingUnit&             cu );
  void        mrl_idx                   ( const CodingUnit&             cu );
  void        pdpc_flag                 ( const CodingUnit&             cu );
  void        pcm_data                  ( const CodingUnit&             cu );
  void        pcm_flag                  ( const CodingUnit&             cu );
  void        cu_pred_data              ( const CodingUnit&             cu );
  void        cu_lic_flag               ( const CodingUnit&             cu );
  void        obmc_flag                 ( const CodingUnit&             cu );
  void        cu_diffusion_filter_idx   ( const CodingUnit&             cu );
  void        intra_luma_pred_modes     ( const CodingUnit&             cu );
  void        intra_luma_pred_mode      ( const PredictionUnit&         pu );
  void        intra_chroma_pred_modes   ( const CodingUnit&             cu );
  void        intra_chroma_lmc_mode     ( const PredictionUnit&         pu );
  void        intra_chroma_pred_mode    ( const PredictionUnit&         pu );
  void        cu_residual               ( const CodingUnit&             cu,       Partitioner&      pm,         CUCtx& cuCtx );
  void        rqt_root_cbf              ( const CodingUnit&             cu );
  void        end_of_ctu                ( const CodingUnit&             cu,       CUCtx&            cuCtx );
  void        nn_flag                   ( const CodingUnit&             cu );
  void        intra_luma_pred_modes_NN  ( const CodingUnit&             cu );
  void        intra_luma_pred_mode_NN   ( const PredictionUnit&         pu );
  void        nn_subsampling_flag       ( const CodingUnit&             cu );

  // prediction unit (clause 7.3.8.6)
  void        prediction_unit           ( const PredictionUnit&         pu );
  void        merge_flag                ( const PredictionUnit&         pu );
  void        affine_flag               ( const CodingUnit&             cu );
  void        merge_idx                 ( const PredictionUnit&         pu );
  void        merge_ref_pic_list        ( const PredictionUnit&         pu );
  void        imv_mode                  ( const CodingUnit&             cu );
  void        inter_pred_idc            ( const PredictionUnit&         pu );
  void        ref_idx                   ( const PredictionUnit&         pu,       RefPicList        eRefList );
  void        mvp_flag                  ( const PredictionUnit&         pu,       RefPicList        eRefList );
  void        ref_idx_mh                ( const int                     numRef,   const int         refIdx );
  void        mh_pred_data              ( const PredictionUnit&         pu );

  void        fruc_mrg_mode             ( const PredictionUnit&         pu );

  // pcm samples (clause 7.3.8.7)
  void        pcm_samples               ( const TransformUnit&          tu );

  // transform tree (clause 7.3.8.8)
  void        transform_tree            ( const CodingStructure&        cs,       Partitioner&      pm,     CUCtx& cuCtx,   ChromaCbfs& chromaCbfs );
  void        split_transform_flag      ( bool                          split,    unsigned          depth );
  void        cbf_comp                  ( const CodingStructure&        cs,       bool              cbf,    const CompArea& area, unsigned depth, bool isUsingMode1dPartitions = false, bool previousCbf = false );

  // mvd coding (clause 7.3.8.9)
  void        mvd_coding                ( const Mv &rMvd, UChar imv );

  // transform unit (clause 7.3.8.10)
  void        transform_unit            ( const TransformUnit&          tu,       CUCtx&            cuCtx,  ChromaCbfs& chromaCbfs );
  void        cu_qp_delta               ( const CodingUnit&             cu,       int               predQP, const SChar qp );
  void        cu_chroma_qp_offset       ( const CodingUnit&             cu );
#if !HM_EMT_NSST_AS_IN_JEM
  void        cu_emt_noqrt_idx          ( const CodingUnit&             cu );
#endif

  // residual coding (clause 7.3.8.11)
  void        residual_coding           ( const TransformUnit&          tu,       ComponentID       compID );
  void        transform_skip_flag       ( const TransformUnit&          tu,       ComponentID       compID );
  void        residual_nsst_mode        ( const CodingUnit&             cu,       CUCtx&            cuCtx  );
  void        emt_tu_index              ( const TransformUnit&          tu );
  void        emt_cu_flag               ( const CodingUnit&             cu );
  void        mode_1d_partitions        ( const CodingUnit&             cu );
  void        explicit_rdpcm_mode       ( const TransformUnit&          tu,       ComponentID       compID );
  void        last_sig_coeff            ( CoeffCodingContext&           cctx );
  void        residual_coding_subblock  ( CoeffCodingContext&           cctx,     const TCoeff*     coeff  );
  void        residual_coding_subblock_tcq  ( CoeffCodingContext&           cctx,     const TCoeff*     coeff, int& state );

  // cross component prediction (clause 7.3.8.12)
  void        cross_comp_pred           ( const TransformUnit&          tu,       ComponentID       compID );
#if THRESHOLDING
  void        thresholding              ( const TransformUnit&          tu );
#endif

private:
  void        code_unary_fixed          ( unsigned symbol, unsigned ctxId, unsigned unary_max, unsigned fixed );
  void        unary_max_symbol          ( unsigned symbol, unsigned ctxId0, unsigned ctxIdN, unsigned maxSymbol );
  void        unary_max_eqprob          ( unsigned symbol,                                   unsigned maxSymbol );
  void        exp_golomb_eqprob         ( unsigned symbol, unsigned count );
  void        encode_sparse_dt          ( DecisionTree& dt, unsigned toCodeId );

  // alf
  void        alf_aux                   ( const ALFParam&               alfParam, bool isGALF );
  void        alf_filter                ( const ALFParam&               alfParam, bool isGALF, bool bChroma = false );
  void        alf_cu_ctrl               ( const ALFParam&               alfParam );
  void        alf_chroma                ( const ALFParam&               alfParam );
  Int         alf_lengthGolomb          ( int coeffVal, int k );
  void        codeAlfUvlc               ( UInt uiCode );
  void        codeAlfSvlc               ( Int iCode );
  void        alfGolombEncode           ( int coeff, int k );

  // statistic
  unsigned    get_num_written_bits()    { return m_BinEncoder.getNumWrittenBits(); }

  Void  xWriteTruncBinCode(UInt uiSymbol, UInt uiMaxSymbol);
#if  JVET_C0038_NO_PREV_FILTERS
  Void  xWriteEpExGolomb(UInt uiSymbol, UInt uiCount);
#endif

private:
  BinEncIf&         m_BinEncoder;
  OutputBitstream*  m_Bitstream;
  Ctx               m_TestCtx;
};



class CABACEncoder
{
public:
  CABACEncoder()
    : m_CABACWriterStd      ( m_BinEncoderStd )
    , m_CABACWriterJMP      ( m_BinEncoderJMP )
    , m_CABACWriterJAW      ( m_BinEncoderJAW )
    , m_CABACWriterJMPAW    ( m_BinEncoderJMPAW )
    , m_CABACWriterMP       ( m_BinEncoderMP  )
    , m_CABACWriterMPI      ( m_BinEncoderMPI )
    , m_CABACWriterMPCW     ( m_BinEncoderMPCW )
    , m_CABACEstimatorStd   ( m_BitEstimatorStd )
    , m_CABACEstimatorJMP   ( m_BitEstimatorJMP )
    , m_CABACEstimatorJAW   ( m_BitEstimatorJAW )
    , m_CABACEstimatorJMPAW ( m_BitEstimatorJMPAW )
    , m_CABACEstimatorMP    ( m_BitEstimatorMP  )
    , m_CABACEstimatorMPI   ( m_BitEstimatorMPI )
    , m_CABACEstimatorMPCW  ( m_BitEstimatorMPCW)
    , m_CABACWriter         { &m_CABACWriterStd,    &m_CABACWriterJMP,    &m_CABACWriterJAW,    &m_CABACWriterJMPAW,    &m_CABACWriterMP,    &m_CABACWriterMPI,    &m_CABACWriterMPCW     }
    , m_CABACEstimator      { &m_CABACEstimatorStd, &m_CABACEstimatorJMP, &m_CABACEstimatorJAW, &m_CABACEstimatorJMPAW, &m_CABACEstimatorMP, &m_CABACEstimatorMPI, &m_CABACEstimatorMPCW  }
  {}

  CABACWriter*                getCABACWriter          ( const SPS*   sps   )        { return m_CABACWriter   [sps->getSpsNext().getCABACEngineMode()]; }
  CABACWriter*                getCABACEstimator       ( const SPS*   sps   )        { return m_CABACEstimator[sps->getSpsNext().getCABACEngineMode()]; }
private:
  BinEncoder_Std      m_BinEncoderStd;
  BinEncoder_JMP      m_BinEncoderJMP;
  BinEncoder_JAW      m_BinEncoderJAW;
  BinEncoder_JMPAW    m_BinEncoderJMPAW;
  BinEncoder_MP       m_BinEncoderMP;
  BinEncoder_MPI      m_BinEncoderMPI;
  BinEncoder_MPCW     m_BinEncoderMPCW;
  BitEstimator_Std    m_BitEstimatorStd;
  BitEstimator_JMP    m_BitEstimatorJMP;
  BitEstimator_JAW    m_BitEstimatorJAW;
  BitEstimator_JMPAW  m_BitEstimatorJMPAW;
  BitEstimator_MP     m_BitEstimatorMP;
  BitEstimator_MPI    m_BitEstimatorMPI;
  BitEstimator_MPCW   m_BitEstimatorMPCW;
  CABACWriter         m_CABACWriterStd;
  CABACWriter         m_CABACWriterJMP;
  CABACWriter         m_CABACWriterJAW;
  CABACWriter         m_CABACWriterJMPAW;
  CABACWriter         m_CABACWriterMP;
  CABACWriter         m_CABACWriterMPI;
  CABACWriter         m_CABACWriterMPCW;
  CABACWriter         m_CABACEstimatorStd;
  CABACWriter         m_CABACEstimatorJMP;
  CABACWriter         m_CABACEstimatorJAW;
  CABACWriter         m_CABACEstimatorJMPAW;
  CABACWriter         m_CABACEstimatorMP;
  CABACWriter         m_CABACEstimatorMPI;
  CABACWriter         m_CABACEstimatorMPCW;
  CABACWriter*        m_CABACWriter   [BPM_NUM-1];
  CABACWriter*        m_CABACEstimator[BPM_NUM-1];
};

//! \}

#endif //__CABACWRITER__
