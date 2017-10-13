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


class CABACDecoder;
class CABACReader
{
public:
  CABACReader( BinDecoderBase& binDecoder ) : m_BinDecoder( binDecoder ), m_Bitstream( 0 ) {}
  virtual ~CABACReader() {}

public:
  void        initCtxModels             ( Slice&                        slice,
                                          CABACDecoder*                 cabacDecoder );
  void        initBitstream             ( InputBitstream*               bitstream )           { m_Bitstream = bitstream; m_BinDecoder.init( m_Bitstream ); }
  const Ctx&  getCtx                    ()                                            const   { return m_BinDecoder.getCtx();  }
  Ctx&        getCtx                    ()                                                    { return m_BinDecoder.getCtx();  }

public:
  // slice segment data (clause 7.3.8.1)
  bool        terminating_bit           ();
  void        remaining_bytes           ( bool                          noTrailingBytesExpected );

  // coding tree unit (clause 7.3.8.2)
  bool        coding_tree_unit          ( CodingStructure&              cs,     const UnitArea& area,     int& qp,   unsigned  ctuRsAddr );

  // sao (clause 7.3.8.3)
  void        sao                       ( CodingStructure&              cs,     unsigned        ctuRsAddr );

  void        alf                       ( CodingStructure&              cs );

  // coding (quad)tree (clause 7.3.8.4)
  bool        coding_tree               ( CodingStructure&              cs,     Partitioner&    pm,       CUCtx& cuCtx );
  bool        split_cu_flag             ( CodingStructure&              cs,     Partitioner&    pm );
  PartSplit   split_cu_mode_mt          ( CodingStructure&              cs,     Partitioner&    pm );

  // coding unit (clause 7.3.8.5)
  bool        coding_unit               ( CodingUnit&                   cu,     Partitioner&    pm,       CUCtx& cuCtx );
  void        cu_transquant_bypass_flag ( CodingUnit&                   cu );
  void        cu_skip_flag              ( CodingUnit&                   cu );
  void        pred_mode                 ( CodingUnit&                   cu );
  void        part_mode                 ( CodingUnit&                   cu );
  void        pdpc_flag                 ( CodingUnit&                   cu );
  void        pcm_flag                  ( CodingUnit&                   cu );
  void        cu_pred_data              ( CodingUnit&                   cu );
  void        cu_lic_flag               ( CodingUnit&                   cu );
  void        obmc_flag                 ( CodingUnit&                   cu );
  void        intra_luma_pred_modes     ( CodingUnit&                   cu );
  void        intra_chroma_pred_modes   ( CodingUnit&                   cu );
  bool        intra_chroma_lmc_mode     ( PredictionUnit&               pu );
  void        intra_chroma_pred_mode    ( PredictionUnit&               pu );
  void        cu_residual               ( CodingUnit&                   cu,     Partitioner&    pm,       CUCtx& cuCtx );
  void        rqt_root_cbf              ( CodingUnit&                   cu );
  bool        end_of_ctu                ( CodingUnit&                   cu,     CUCtx&          cuCtx );

  // prediction unit (clause 7.3.8.6)
  void        prediction_unit           ( PredictionUnit&               pu,     MergeCtx&       mrgCtx );
  void        merge_flag                ( PredictionUnit&               pu );
  void        merge_data                ( PredictionUnit&               pu );
  void        affine_flag               ( CodingUnit&                   cu );
  void        merge_idx                 ( PredictionUnit&               pu );
  void        imv_mode                  ( CodingUnit&                   cu,     MergeCtx&       mrgCtx );
  void        inter_pred_idc            ( PredictionUnit&               pu );
  void        ref_idx                   ( PredictionUnit&               pu,     RefPicList      eRefList );
  void        mvp_flag                  ( PredictionUnit&               pu,     RefPicList      eRefList );

  void        fruc_mrg_mode             ( PredictionUnit&               pu );

  // pcm samples (clause 7.3.8.7)
  void        pcm_samples               ( TransformUnit&                tu );

  // transform tree (clause 7.3.8.8)
  void        transform_tree            ( CodingStructure&              cs,     Partitioner&    pm,       CUCtx& cuCtx,  ChromaCbfs& chromaCbfs );
  bool        split_transform_flag      ( unsigned                      depth );
  bool        cbf_comp                  ( const CompArea&               area,   unsigned        depth );

  // mvd coding (clause 7.3.8.9)
  void        mvd_coding                ( Mv &rMvd );

  // transform unit (clause 7.3.8.10)
  void        transform_unit            ( TransformUnit&                tu,     CUCtx&          cuCtx,  ChromaCbfs& chromaCbfs );
  void        transform_unit_qtbt       ( TransformUnit&                tu,     CUCtx&          cuCtx,  ChromaCbfs& chromaCbfs );
  void        cu_qp_delta               ( CodingUnit&                   cu,     int             predQP );
  void        cu_chroma_qp_offset       ( CodingUnit&                   cu );

  // residual coding (clause 7.3.8.11)
  void        residual_nsst_mode        ( CodingUnit&                   cu );
  void        residual_coding           ( TransformUnit&                tu,     ComponentID     compID );
  void        transform_skip_flag       ( TransformUnit&                tu,     ComponentID     compID );
  void        emt_tu_index              ( TransformUnit&                tu );
  void        emt_cu_flag               ( CodingUnit&                   cu );
  void        explicit_rdpcm_mode       ( TransformUnit&                tu,     ComponentID     compID );
  int         last_sig_coeff            ( CoeffCodingContext&           cctx );
  void        residual_coding_subblock  ( CoeffCodingContext&           cctx,   TCoeff*         coeff  );

  // cross component prediction (clause 7.3.8.12)
  void        cross_comp_pred           ( TransformUnit&                tu,     ComponentID     compID );

private:
  unsigned    unary_max_symbol          ( unsigned ctxId0, unsigned ctxIdN, unsigned maxSymbol );
  unsigned    unary_max_eqprob          (                                   unsigned maxSymbol );
  unsigned    exp_golomb_eqprob         ( unsigned count );

  unsigned    decode_sparse_dt          ( DecisionTree& dt );
  unsigned    get_num_bits_read         () { return m_BinDecoder.getNumBitsRead(); }

  // neeeds clean up
  void        alf_aux                   ( ALFParam&               alfParam, bool isGALF );
  void        alf_filter                ( ALFParam&               alfParam, bool isGALF, bool bChroma = false );
  void        alf_chroma                ( ALFParam& alfParam );
  void        alf_cu_ctrl               ( ALFParam& alfParam, unsigned maxTotalCuDepth );
  UInt        parseAlfUvlc();
  Int         parseAlfSvlc();
  Int         alfGolombDecode(Int k);

  Void        xReadTruncBinCode   (UInt& ruiSymbol, UInt uiMaxSymbol);
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
    , m_CABACReader     { &m_CABACReaderStd, &m_CABACReaderJMP, &m_CABACReaderJAW, &m_CABACReaderJMPAW }
  {}

  CABACReader*                getCABACReader    ( int           id    )       { return m_CABACReader[id]; }

  void                        checkInit         ( const SPS*    sps   )       { m_CtxWSizeStore.checkInit(sps); }
  const std::vector<uint8_t>* getWinSizes       ( const Slice*  slice ) const { return m_CtxWSizeStore.getWinSizes(slice); }
  std::vector<uint8_t>&       getWSizeReadBuffer()                            { return m_CtxWSizeStore.getReadBuffer(); }

  void  loadCtxStates     ( const Slice*  slice, Ctx& ctx   ) const
  {
    if( slice->getSPS()->getSpsNext().getUseCIPF() )
    {
      m_CtxStateStore.loadCtx(slice,ctx);
    }
  }
  void  storeCtxStates    ( const Slice*  slice, const Ctx& ctx )
  {
    if( slice->getSPS()->getSpsNext().getUseCIPF() )
    {
      m_CtxStateStore.storeCtx( slice, ctx );
    }
  }
  void  updateBufferState ( const Slice* slice )
  {
    if ( slice->getPendingRasInit() )
    {
      m_CtxStateStore.clearValid();
    }
    m_CtxWSizeStore.updateState( slice, false );
  }

private:
  BinDecoder_Std          m_BinDecoderStd;
  BinDecoder_JMP          m_BinDecoderJMP;
  BinDecoder_JAW          m_BinDecoderJAW;
  BinDecoder_JMPAW        m_BinDecoderJMPAW;
  CABACReader             m_CABACReaderStd;
  CABACReader             m_CABACReaderJMP;
  CABACReader             m_CABACReaderJAW;
  CABACReader             m_CABACReaderJMPAW;
  CABACReader*            m_CABACReader[BPM_NUM-1];
  CtxStateStore           m_CtxStateStore;
  CtxWSizeStore           m_CtxWSizeStore;
};

#endif
