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

/** \file     CABACReader.h
 *  \brief    Reader for low level syntax
 */

#ifndef __CABACREADER__
#define __CABACREADER__

#include "BinDecoder.h"

#include "CommonLib/ContextModelling.h"
#include "CommonLib/MotionInfo.h"
#include "CommonLib/UnitPartitioner.h"


class CABACDataStore;
class CABACReader
{
public:
  CABACReader( BinDecoderBase& binDecoder ) : m_BinDecoder( binDecoder ), m_Bitstream( 0 ) {}
  virtual ~CABACReader() {}

public:
  void        initCtxModels             ( Slice&                        slice, CABACDataStore*               cabacDataStore );
  void        initBitstream             ( InputBitstream*               bitstream )           { m_Bitstream = bitstream; m_BinDecoder.init( m_Bitstream ); }
  const Ctx&  getCtx                    ()                                            const   { return m_BinDecoder.getCtx();  }
  Ctx&        getCtx                    ()                                                    { return m_BinDecoder.getCtx();  }

public:
  // slice segment data (clause 7.3.8.1)
  bool        terminating_bit           ();
  void        remaining_bytes           ( bool                          noTrailingBytesExpected );

  // coding tree unit (clause 7.3.8.2)
  bool        coding_tree_unit          ( CodingStructure&              cs,     const UnitArea& area,     int (&qps)[2],   unsigned  ctuRsAddr );

  // sao (clause 7.3.8.3)
  void        sao                       ( CodingStructure&              cs,     unsigned        ctuRsAddr );

  void        alf                       ( CodingStructure&              cs );

  // coding (quad)tree (clause 7.3.8.4)
  bool        coding_tree               ( CodingStructure&              cs,     Partitioner&    pm,       CUCtx& cuCtx );
  PartSplit   gen_bin_split_mode        ( CodingStructure&              cs,     Partitioner&    pm );
  SplitModifier gen_bin_split_mod       ( CodingStructure&              cs,     Partitioner&    pm, const PartSplit baseSplit );
  bool        split_cu_flag             ( CodingStructure&              cs,     Partitioner&    pm );
  PartSplit   split_cu_mode_mt          ( CodingStructure&              cs,     Partitioner&    pm );

  // coding unit (clause 7.3.8.5)
  bool        coding_unit               ( CodingUnit&                   cu,     Partitioner&    pm,       CUCtx& cuCtx );
  void        cu_transquant_bypass_flag ( CodingUnit&                   cu );
  void        cu_skip_flag              ( CodingUnit&                   cu );
  void        pred_mode                 ( CodingUnit&                   cu );
  void        part_mode                 ( CodingUnit&                   cu );
  void        mrl_idx                   ( CodingUnit&                   cu );
  void        pdpc_flag                 ( CodingUnit&                   cu );
  void        pcm_flag                  ( CodingUnit&                   cu );
  void        cu_pred_data              ( CodingUnit&                   cu );
  void        cu_lic_flag               ( CodingUnit&                   cu );
  void        obmc_flag                 ( CodingUnit&                   cu );
  void        cu_diffusion_filter_idx   ( CodingUnit&                   cu );
  void        intra_luma_pred_modes     ( CodingUnit&                   cu );
  void        intra_chroma_pred_modes   ( CodingUnit&                   cu );
  bool        intra_chroma_lmc_mode     ( PredictionUnit&               pu );
  void        intra_chroma_pred_mode    ( PredictionUnit&               pu );
  void        cu_residual               ( CodingUnit&                   cu,     Partitioner&    pm,       CUCtx& cuCtx );
  void        rqt_root_cbf              ( CodingUnit&                   cu );
  bool        end_of_ctu                ( CodingUnit&                   cu,     CUCtx&          cuCtx );
  void        nn_flag                   ( CodingUnit&                   cu );
  void        intra_luma_pred_modes_NN  ( CodingUnit&                   cu );
  void        intra_luma_pred_mode_NN   ( PredictionUnit&               pu );
  void        nn_subsampling_flag       ( CodingUnit&                   cu );

  // prediction unit (clause 7.3.8.6)
  void        prediction_unit           ( PredictionUnit&               pu,     MergeCtx&       mrgCtx );
  void        merge_flag                ( PredictionUnit&               pu );
  void        merge_data                ( PredictionUnit&               pu );
  void        merge_ref_pic_list        ( PredictionUnit&               pu );
  void        affine_flag               ( CodingUnit&                   cu );
  void        merge_idx                 ( PredictionUnit&               pu );
  void        imv_mode                  ( CodingUnit&                   cu,     MergeCtx&       mrgCtx );
  void        inter_pred_idc            ( PredictionUnit&               pu );
  void        ref_idx                   ( PredictionUnit&               pu,     RefPicList      eRefList );
  void        mvp_flag                  ( PredictionUnit&               pu,     RefPicList      eRefList );
  int         ref_idx_mh                ( const int                     numRef );
  void        mh_pred_data              ( PredictionUnit&               pu );

  void        fruc_mrg_mode             ( PredictionUnit&               pu );
  // pcm samples (clause 7.3.8.7)
  void        pcm_samples               ( TransformUnit&                tu );

  // transform tree (clause 7.3.8.8)
  void        transform_tree            ( CodingStructure&              cs,     Partitioner&    pm,       CUCtx& cuCtx,  ChromaCbfs& chromaCbfs );
  bool        split_transform_flag      ( unsigned                      depth );
  bool        cbf_comp                  ( CodingStructure&              cs,     const CompArea& area,     unsigned depth, bool useMode1dPartitions = false, bool previousCbf = false );

  // mvd coding (clause 7.3.8.9)
  void        mvd_coding                ( Mv &rMvd );

  // transform unit (clause 7.3.8.10)
  void        transform_unit            ( TransformUnit&                tu,     CUCtx&          cuCtx,  ChromaCbfs& chromaCbfs );
  void        cu_qp_delta               ( CodingUnit&                   cu,     int             predQP, SChar& qp );
  void        cu_chroma_qp_offset       ( CodingUnit&                   cu );
#if !HM_EMT_NSST_AS_IN_JEM
  void        cu_emt_noqrt_idx          ( CodingUnit&                   cu );
#endif

  // residual coding (clause 7.3.8.11)
  void        residual_coding           ( TransformUnit&                tu,     ComponentID     compID );
  void        transform_skip_flag       ( TransformUnit&                tu,     ComponentID     compID );
  void        residual_nsst_mode        ( CodingUnit&                   cu );
  void        emt_tu_index              ( TransformUnit&                tu );
  void        emt_cu_flag               ( CodingUnit&                   cu );
  void        mode_1d_partitions        ( CodingUnit&                   cu );
  void        explicit_rdpcm_mode       ( TransformUnit&                tu,     ComponentID     compID );
  int         last_sig_coeff            ( CoeffCodingContext&           cctx );
  void        residual_coding_subblock  ( CoeffCodingContext&           cctx,   TCoeff*         coeff  );
  void        residual_coding_subblock_tcq
                                        ( CoeffCodingContext&           cctx,   TCoeff*         coeff, int& state );

  // cross component prediction (clause 7.3.8.12)
  void        cross_comp_pred           ( TransformUnit&                tu,     ComponentID     compID );
#if THRESHOLDING
  void        thresholding              ( TransformUnit&                tu );
#endif

private:
  unsigned    code_unary_fixed          ( unsigned ctxId, unsigned unary_max, unsigned fixed );
  unsigned    unary_max_symbol          ( unsigned ctxId0, unsigned ctxIdN, unsigned maxSymbol );
  unsigned    unary_max_eqprob          (                                   unsigned maxSymbol );
  unsigned    exp_golomb_eqprob         ( unsigned count );
  unsigned    decode_sparse_dt          ( DecisionTree& dt );
  unsigned    get_num_bits_read         () { return m_BinDecoder.getNumBitsRead(); }

  // neeeds clean up
  void        alf_aux                   ( ALFParam&               alfParam, bool isGALF );
  void        alf_filter                ( ALFParam&               alfParam, bool isGALF, bool bChroma = false );
  void        alf_chroma                ( ALFParam& alfParam );
  void        alf_cu_ctrl               ( ALFParam& alfParam );
  UInt        parseAlfUvlc();
  Int         parseAlfSvlc();
  Int         alfGolombDecode(Int k);

  Void        xReadTruncBinCode   ( UInt& ruiSymbol, UInt uiMaxSymbol );
  UInt        xReadEpExGolomb     ( UInt uiCount );

private:
  BinDecoderBase& m_BinDecoder;
  InputBitstream* m_Bitstream;
  MotionInfo      m_SubPuMiBuf   [( MAX_CU_SIZE * MAX_CU_SIZE ) >> ( MIN_CU_LOG2 << 1 )];
  MotionInfo      m_SubPuExtMiBuf[( MAX_CU_SIZE * MAX_CU_SIZE ) >> ( MIN_CU_LOG2 << 1 )];
};


class CABACDecoder
{
public:
  CABACDecoder()
    : m_CABACReaderStd  ( m_BinDecoderStd )
    , m_CABACReaderJMP  ( m_BinDecoderJMP )
    , m_CABACReaderJAW  ( m_BinDecoderJAW )
    , m_CABACReaderJMPAW( m_BinDecoderJMPAW )
    , m_CABACReaderMP   ( m_BinDecoderMP )
    , m_CABACReaderMPI  ( m_BinDecoderMPI )
    , m_CABACReaderMPCW ( m_BinDecoderMPCW )
    , m_CABACReader     { &m_CABACReaderStd, &m_CABACReaderJMP, &m_CABACReaderJAW, &m_CABACReaderJMPAW, &m_CABACReaderMP, &m_CABACReaderMPI, &m_CABACReaderMPCW }
  {}

  CABACReader*                getCABACReader    ( int           id    )       { return m_CABACReader[id]; }

private:
  BinDecoder_Std          m_BinDecoderStd;
  BinDecoder_JMP          m_BinDecoderJMP;
  BinDecoder_JAW          m_BinDecoderJAW;
  BinDecoder_JMPAW        m_BinDecoderJMPAW;
  BinDecoder_MP           m_BinDecoderMP;
  BinDecoder_MPI          m_BinDecoderMPI;
  BinDecoder_MPCW         m_BinDecoderMPCW;
  CABACReader             m_CABACReaderStd;
  CABACReader             m_CABACReaderJMP;
  CABACReader             m_CABACReaderJAW;
  CABACReader             m_CABACReaderJMPAW;
  CABACReader             m_CABACReaderMP;
  CABACReader             m_CABACReaderMPI;
  CABACReader             m_CABACReaderMPCW;
  CABACReader*            m_CABACReader[BPM_NUM-1];
};

#endif
