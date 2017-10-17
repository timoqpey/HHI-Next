/*!
***********************************************************************
*  \file
*      dtrace_next.h
*  \brief
*      dtrace support for next software
*  \author
*      Valeri George <valeri.george@hhi-extern.fraunhofer.de>
***********************************************************************
*/
#ifndef _DTRACE_NEXT_H_
#define _DTRACE_NEXT_H_

#include "dtrace.h"

#include "TLibCommon/TComRom.h"
#include "TLibCommon/TComPic.h"
#include "TLibCommon/TComYuv.h"


#if TRACING

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 
// DTRACE SHORT MANUAL
//
// 1. General info
// 
// DTrace is a simple tracer that can be controlled at command line of the executable.
//
// Please have a look into command line parameter options to find the correspondent DTrace parameters.
// There are only two command-line parameters: tracing file and tracing rule.
// At initialization, DTrace-module parses the rule and set up the tracing context. 
// The tracing context is stored in a global variable. All trace-outputs should use this global context.
// 
// 2. Parameters
// 2.1 Tracing file (--TraceFile)
// 
// Just a filename for a text-file.
// E.g.: --TraceFile="tracefile_rec.txt"
// 
// 2.2 Tracing rule (--TraceRule)
// 
// Tracing rule describes when during a runtime a particular output should be activated.
// Tracing rule consists of tracing channel(s) and tracing condition(s).
// The construction of the rule is: "channel_1,channel_2,...:condition_1,condition_2,..."
// 
// Example for a tracing rule: --TraceRule="D_CABAC:poc==0"
// Here, tracing channel is D_CABAC and condition is poc==0, which means CABAC tracing output is activated at POC 0 only. 
// You can also use poc>=2 or set the range like poc>=0,poc<=3 for example.
// 
// 
// 2.2.1 Tracing channel
// 
// Channels are defined in dtrace_next.h. Users can add their own channels.
// Just put the channel definition into enum-list AND finally add it to the next_channels-table in the function tracing_init().
// 
// 2.2.2 Tracing condition
// 
// Currently only "poc" is supported, feel free to add your own conditions.
// NOTE: Conditions are added and updated during runtime through DTRACE_UPDATE(...). 
// It updates the DTrace internal state, so channels can be activated at the right moment. 
// If it's not updated properly (at right place in the code, e.g. too late), the trace output can become wrong. 
// For example, "poc"-condition should be updated at the start of the picture(AccesUnit).
// Please look into source code for how the "poc"-condition is used.
//      
// 3. Using of DTrace macros
// 
// The most used macro is DTRACE. It's like a printf-function with some additional parameters at the beginning.
// Format: 
// DTRACE( tracing context, tracing channel, "..." [,params] );
// Example:
// DTRACE( g_trace_ctx, D_CABAC, "EP=%d \n", bin );
// There are also macros for output of buffers, picture components or conditional-outputs available. Please have a look into dtrace_next.h.
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// DTrace channels
// Use already defined channels or add your own tracing channels

enum DTRACE_CHANNEL {

  D_COMMON,
  D_REC_CB_LUMA,          // reconstructed coding block luma pixel
  D_REC_CB_CHROMA,        // reconstructed coding block chroma pixel
  D_REC_CB_LUMA_LF,       // reconstructed coding block luma pixel after deblocking filter
  D_REC_CB_CHROMA_LF,     // reconstructed coding block chroma pixel after deblocking filter
  D_REC_CB_LUMA_SAO,      // reconstructed coding block luma pixel after SAO filter
  D_REC_CB_CHROMA_SAO,    // reconstructed coding block chroma pixel after SAO filter
  D_REC_CB_LUMA_ALF,      // reconstructed coding block luma pixel after ALF filter
  D_REC_CB_CHROMA_ALF,    // reconstructed coding block chroma pixel after ALF filter
  D_ME,                   // Motion Estimation
  D_CABAC,                // CABAC engine
  D_AFFI2,
  D_BEST_MODE,            // Cost for coding mode (encoder only)
  D_MODE_COST,            // Cost for coding mode (encoder only)
  D_QP_PRED,              // QP Prediction for DQP process
  D_DQP,                  // Delta QP read/write
  D_QP,                   // final CU QP at reading/writing stage
  D_FRAC_BITS,            // Fractional bits during estimation 
  D_EST_FRAC_BITS,
  D_LF,                   // Deblocking filter
  D_TU_EST,               // TU estimation
  D_SAVELOAD,             // save load
  D_TU_ABS_SUM,
  D_INTRA_COST,           //intra cost
  D_TMP,
};

#define _CNL_DEF(_s) {_s,(std::string(#_s))}

inline CDTrace* tracing_init( std::string& sTracingFile, std::string& sTracingRule )
{
  dtrace_channel next_channels[] =
  {
    _CNL_DEF( D_COMMON ),           
    _CNL_DEF( D_REC_CB_LUMA ),      
    _CNL_DEF( D_REC_CB_CHROMA ),    
    _CNL_DEF( D_REC_CB_LUMA_LF ),   
    _CNL_DEF( D_REC_CB_CHROMA_LF ), 
    _CNL_DEF( D_REC_CB_LUMA_SAO ),  
    _CNL_DEF( D_REC_CB_CHROMA_SAO ),
    _CNL_DEF( D_REC_CB_LUMA_ALF ),
    _CNL_DEF( D_REC_CB_CHROMA_ALF ),
    _CNL_DEF( D_ME ),               
    _CNL_DEF( D_CABAC ),            
    _CNL_DEF( D_AFFI2 ),
    _CNL_DEF( D_BEST_MODE ),
    _CNL_DEF( D_MODE_COST ),
    _CNL_DEF( D_QP_PRED ),          
    _CNL_DEF( D_DQP ),              
    _CNL_DEF( D_QP ),               
    _CNL_DEF( D_FRAC_BITS ),
    _CNL_DEF( D_EST_FRAC_BITS ),
    _CNL_DEF( D_LF ),
    _CNL_DEF( D_TU_EST ),              
    _CNL_DEF( D_SAVELOAD ),              
    _CNL_DEF( D_TU_ABS_SUM ),
    _CNL_DEF( D_INTRA_COST ),
    _CNL_DEF( D_TMP ),
  };

  dtrace_channels_t channels( next_channels, &next_channels[sizeof( next_channels ) / sizeof( next_channels[0] )] );
  CDTrace *pDtrace = new CDTrace( sTracingFile, sTracingRule, channels );

  if( pDtrace->getLastError() )
  {
    printf( "%s\n", pDtrace->getErrMessage().c_str() );
    //return NULL;
  }

  return pDtrace;
}

inline void tracing_uninit( CDTrace *pDtrace )
{
  if( pDtrace )
    delete pDtrace;
}


template< typename Tsrc >
void dtrace_block( CDTrace *trace_ctx, DTRACE_CHANNEL channel, Tsrc *buf, unsigned stride, unsigned block_w, unsigned block_h )
{
  unsigned i, j;
  for( j = 0; j < block_h; j++ )
  {
    for( i = 0; i < block_w; i++ )
    {
      trace_ctx->dtrace( channel, "%04x ", buf[j*stride + i] );
      //trace_ctx->dtrace( channel, "%4d ", buf[j*stride + i] );
    }
    trace_ctx->dtrace( channel, "\n" );
  }
  trace_ctx->dtrace( channel, "\n" );
}

template< typename Tsrc >
void dtrace_frame_blockwise( CDTrace *trace_ctx, DTRACE_CHANNEL channel, Tsrc *buf, unsigned stride, unsigned frm_w, unsigned frm_h, unsigned block_w, unsigned block_h )
{
  unsigned i, j, block;
  for( j = 0, block = 0; j < frm_h; j += block_h )
  {
    unsigned blockhf = std::min( block_h, frm_h - j);
    Tsrc *p_buf = buf + j*stride;
    for( i = 0; i < frm_w; i += block_w, block++ )
    {
      unsigned blockwf = std::min( block_w, frm_w - i);

      trace_ctx->dtrace( channel, "Frame BLOCK=%d (x,y) = (%d, %d)\n", block, i, j );
      dtrace_block( trace_ctx, channel, p_buf, stride, blockwf, blockhf );
      p_buf += block_w;
    }
  }
}

#define DTRACE(ctx,channel,...)              ctx->dtrace( channel, __VA_ARGS__ )
#define DTRACE_UPDATE(ctx,s)                 if((ctx)){(ctx)->update((s));}
#define DTRACE_REPEAT(ctx,channel,times,...) ctx->dtrace_repeat( channel, times,__VA_ARGS__ )
#define DTRACE_COND(ctx,cond,channel,...)    { if( cond ) ctx->dtrace( channel, __VA_ARGS__ ); }
#define DTRACE_BLOCK(...)                    dtrace_block(__VA_ARGS__)
#define DTRACE_FRAME_BLOCKWISE(...)          dtrace_frame_blockwise(__VA_ARGS__)
#define DTRACE_GET_COUNTER(ctx,channel)      ctx->getChannelCounter(channel)


//////////////////////////////////////////////////////////////////////////
// Specialized functions
//////////////////////////////////////////////////////////////////////////
#if NEXT_SOFTWARE
#include "CommonLib/Slice.h"
#include "CommonLib/Unit.h"

inline void dtraceBlockRec( CodingStructure& cs, const UnitArea& ua, PredMode predMode, UInt zIdx = 0 )
{
  Int x0 = ua.lumaPos().x;
  Int y0 = ua.lumaPos().y;
  UInt    uiStride     = cs.getRecoBuf( ua.Y()  ).stride; 
  Pel*    piReco       = cs.getRecoBuf( ua.Y()  ).buf;
  UInt    uiWidth      = ua.lumaSize().width;
  UInt    uiHeight     = ua.lumaSize().height;
  DTRACE      ( g_trace_ctx, D_REC_CB_LUMA,   "%d, x=%d, y=%d, size=%dx%d, predmode=%d \n", zIdx, x0, y0, uiWidth, uiHeight, predMode );
  DTRACE_BLOCK( g_trace_ctx, D_REC_CB_LUMA,   piReco, uiStride, uiWidth, uiHeight );
  if( cs.area.chromaFormat == CHROMA_400 )
  {
    return;
  }
  UInt    uiCStride    = cs.getRecoBuf( ua.Cb() ).stride;
  Pel*    piRecoU      = cs.getRecoBuf( ua.Cb() ).buf;
  Pel*    piRecoV      = cs.getRecoBuf( ua.Cr() ).buf;
  DTRACE      ( g_trace_ctx, D_REC_CB_CHROMA, "%d, x=%d, y=%d, size=%dx%d, predmode=%d \n", zIdx, x0, y0, uiWidth, uiHeight, predMode );
  DTRACE_BLOCK( g_trace_ctx, D_REC_CB_CHROMA, piRecoU, uiCStride, uiWidth >> 1, uiHeight >> 1 );
  DTRACE_BLOCK( g_trace_ctx, D_REC_CB_CHROMA, piRecoV, uiCStride, uiWidth >> 1, uiHeight >> 1 );
}

inline void dtraceUnitComp( DTRACE_CHANNEL channel, CPelUnitBuf& pelUnitBuf, const UnitArea& ua, ComponentID compId, PredMode predMode, UInt zIdx = 0 )
{
  if( !g_trace_ctx ) return;
  const Pel* piReco   = pelUnitBuf.bufs[compId].buf;
  UInt       uiStride = pelUnitBuf.bufs[compId].stride;
  UInt       uiWidth  = ua.blocks[compId].width;
  UInt       uiHeight = ua.blocks[compId].height;
  Int x0              = ua.lumaPos().x;
  Int y0              = ua.lumaPos().y;

  DTRACE( g_trace_ctx, channel, "%s: %d, x=%d, y=%d, size=%dx%d, predmode=%d \n", g_trace_ctx->getChannelName(channel), zIdx, x0, y0, uiWidth, uiHeight, predMode );
  DTRACE_BLOCK( g_trace_ctx, channel, piReco, uiStride, uiWidth, uiHeight );
}

inline void dtracePicComp( DTRACE_CHANNEL channel, CodingStructure& cs, const CPelUnitBuf& pelUnitBuf, ComponentID compId )
{
  if( !g_trace_ctx ) return;
  const Pel* piSrc    = pelUnitBuf.bufs[compId].buf;
  UInt       uiStride = pelUnitBuf.bufs[compId].stride;
  UInt       uiWidth  = pelUnitBuf.bufs[compId].width;
  UInt       uiHeight = pelUnitBuf.bufs[compId].height;

  DTRACE( g_trace_ctx, channel, "\n%s: poc = %d, size=%dx%d\n\n", g_trace_ctx->getChannelName(channel), cs.slice->getPOC(), uiWidth, uiHeight );
  DTRACE_FRAME_BLOCKWISE( g_trace_ctx, channel, piSrc, uiStride, uiWidth, uiHeight, cs.sps->getMaxCUWidth(), cs.sps->getMaxCUWidth()/*8, 8*/ );
}


#define DTRACE_BLOCK_REC(...) dtraceBlockRec( __VA_ARGS__ )
#define DTRACE_UNIT_COMP(...) dtraceUnitComp( __VA_ARGS__ )
#define DTRACE_PIC_COMP(...)  dtracePicComp( __VA_ARGS__ )
#else

inline void dtraceBestMode(TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, UInt uiDepth, UInt uiWidth, UInt uiHeight, bool bCheckDepth)
{
#if JVET_C0024_QTBT
  PartSize ePartSize = SIZE_2Nx2N;
#else
  PartSize ePartSize = rpcTempCU->getPartitionSize(0);
#endif
  Distortion tempDist = rpcTempCU->getTotalDistortion();
  UInt64     tempBits = rpcTempCU->getTotalBits();
  UInt64     tempCost = (UInt64)rpcTempCU->getTotalCost();
  if( rpcTempCU->getTotalCost() == MAX_DOUBLE )
  {
    tempCost = 0;
    tempBits = 0;
    tempDist = 0;
  }

  bool isIntra = rpcTempCU->getPredictionMode( 0 ) == MODE_INTRA;

  if(!bCheckDepth)
  {
    DTRACE( g_trace_ctx, D_BEST_MODE, "CheckModeCost: %6lld %3d @(%4d,%4d) [%2dx%2d] %d (%d,%d,%2d,%d,%d,%d) tempCS = %lld (%d,%d), bestCS = %lld (%d,%d): --> choose %s",
            DTRACE_GET_COUNTER( g_trace_ctx, D_BEST_MODE ),
            rpcTempCU->getPic()->getPOC(),
            rpcTempCU->getCUPelX(), rpcTempCU->getCUPelY(),
            uiWidth, uiHeight,
            uiDepth,
            rpcTempCU->getQP( 0 ),
            rpcTempCU->getPredictionMode( 0 ),
            ePartSize,
            rpcTempCU->getMergeFlag( 0 ),
            isIntra ? rpcTempCU->getIntraDir( CHANNEL_TYPE_LUMA, 0 ) : 0, isIntra ? rpcTempCU->getIntraDir( CHANNEL_TYPE_CHROMA, 0 ) : 0,
            tempCost, ( int ) tempBits, ( int ) tempDist,
            rpcBestCU->getTotalCost() == MAX_DOUBLE ? 0ll : ( UInt64 ) rpcBestCU->getTotalCost(), ( int ) rpcBestCU->getTotalBits(), ( int ) rpcBestCU->getTotalDistortion(),
            rpcTempCU->getTotalCost() < rpcBestCU->getTotalCost() ? "TEMP\n" : "BEST\n" );
  }
  else
  {
    DTRACE( g_trace_ctx, D_BEST_MODE, "CheckModeSplitCost: %6lld %3d @(%4d,%4d) [%2dx%2d] -------------------------- tempCS = %lld (%d,%d), bestCS = %lld (%d,%d): --> choose %s",
            DTRACE_GET_COUNTER( g_trace_ctx, D_BEST_MODE ),
            rpcTempCU->getPic()->getPOC(),
            rpcTempCU->getCUPelX(), rpcTempCU->getCUPelY(),
            uiWidth, uiHeight,
            tempCost, ( int ) tempBits, ( int ) tempDist,
            rpcBestCU->getTotalCost() == MAX_DOUBLE ? 0ll : ( UInt64 ) rpcBestCU->getTotalCost(), ( int ) rpcBestCU->getTotalBits(), ( int ) rpcBestCU->getTotalDistortion(),
            rpcTempCU->getTotalCost() < rpcBestCU->getTotalCost() ? "TEMP STRUCTURE\n" : "BEST STRUCTURE\n" );
  }
}

inline void dtraceModeCost( TComDataCU*& rpcTempCU )
{
  UInt uiDepth  = rpcTempCU->getDepth( 0 );
#if JVET_C0024_QTBT
  UInt uiWidth  = rpcTempCU->getWidth( 0 );
  UInt uiHeight = rpcTempCU->getHeight( 0 );
  PartSize ePartSize = SIZE_2Nx2N;
#else
  UInt uiWidth  = rpcTempCU->getSlice()->getSPS()->getMaxCUWidth() >> uiDepth;
  UInt uiHeight = rpcTempCU->getSlice()->getSPS()->getMaxCUWidth() >> uiDepth;
  PartSize ePartSize = rpcTempCU->getPartitionSize( 0 );
#endif
  Distortion tempDist = rpcTempCU->getTotalDistortion();
  UInt64     tempBits = rpcTempCU->getTotalBits();
  UInt64     tempCost = ( UInt64 ) rpcTempCU->getTotalCost();

  if( rpcTempCU->getTotalCost() == MAX_DOUBLE )
  {
    tempCost = 0;
    tempBits = 0;
    tempDist = 0;
  }

  bool isIntra = rpcTempCU->getPredictionMode( 0 ) == MODE_INTRA;

#if VCEG_AZ07_FRUC_MERGE
  int fruc = rpcTempCU->getFRUCMgrMode( 0 );
#else
  int fruc = 0;
#endif

#if COM16_C806_OBMC
  int obmc = rpcTempCU->getOBMCFlag( 0 ) && rpcTempCU->getSlice()->getSPS()->getOBMC();
#else
  int obmc = 0;
#endif

#if VCEG_AZ06_IC
  int ic = rpcTempCU->getICFlag( 0 );
#else
  int ic = 0;
#endif

#if VCEG_AZ07_IMV
  int imv = rpcTempCU->getiMVFlag( 0 );
#else
  int imv = 0;
#endif
  
#if COM16_C1016_AFFINE
  int affine = rpcTempCU->getAffineFlag( 0 );
#else
  int affine = 0;
#endif

  DTRACE( g_trace_ctx, D_MODE_COST, "ModeCost: %6lld %3d @(%4d,%4d) [%2dx%2d] %d (qp%d,pm%d,ptSize%d,skip%d,mrg%d,fruc%d,obmc%d,ic%d,imv%d,affn%d,%d,%d) tempCS = %lld (%d,%d)\n",
          DTRACE_GET_COUNTER( g_trace_ctx, D_MODE_COST ),
          rpcTempCU->getPic()->getPOC(),
          rpcTempCU->getCUPelX(), rpcTempCU->getCUPelY(),
          uiWidth, uiHeight,
          uiDepth,
          rpcTempCU->getQP( 0 ),
          rpcTempCU->getPredictionMode( 0 ),
          ePartSize,
          rpcTempCU->getSkipFlag( 0 ),
          rpcTempCU->getMergeFlag( 0 ),
          fruc, obmc, ic, imv, affine,
          isIntra ? rpcTempCU->getIntraDir( CHANNEL_TYPE_LUMA, 0 ) : 0, isIntra ? rpcTempCU->getIntraDir( CHANNEL_TYPE_CHROMA, 0 ) : 0,
          tempCost, ( int ) tempBits, ( int ) tempDist );
}

inline void dtrace_crc( CDTrace *trace_ctx, DTRACE_CHANNEL channel, TComDataCU* pcCU, TComYuv* pcYUVBuf )
{
  trace_ctx->dtrace( channel, "CRC: %6lld %3d @(%4d,%4d) [%2dx%2d] ,Checksum(%x %x %x)\n",
    trace_ctx->getChannelCounter(channel),
    pcCU->getPic()->getPOC(),
    pcCU->getCUPelX(), pcCU->getCUPelY(),
    pcCU->getWidth(0), pcCU->getHeight(0),
    pcYUVBuf->calcCheckSum( COMPONENT_Y, pcCU->getSlice()->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA)),
    pcYUVBuf->calcCheckSum( COMPONENT_Cb, pcCU->getSlice()->getSPS()->getBitDepth(CHANNEL_TYPE_CHROMA)),
    pcYUVBuf->calcCheckSum( COMPONENT_Cr, pcCU->getSlice()->getSPS()->getBitDepth(CHANNEL_TYPE_CHROMA)));
}

inline void dtrace_ccrc( CDTrace *trace_ctx, DTRACE_CHANNEL channel, TComDataCU* pcCU, TComYuv* pcYUVBuf, ComponentID compId )
{
  trace_ctx->dtrace( channel, "CRC: %6lld %3d @(%4d,%4d) [%2dx%2d] ,comp %d Checksum(%x)\n",
    trace_ctx->getChannelCounter(channel),
    pcCU->getPic()->getPOC(), 
    pcCU->getCUPelX(), pcCU->getCUPelY(),
    pcCU->getWidth(0), pcCU->getHeight(0), compId,
    pcYUVBuf->calcCheckSum( compId, pcCU->getSlice()->getSPS()->getBitDepth( toChannelType(compId))));
}

#define DTRACE_BEST_MODE(...) dtraceBestMode(__VA_ARGS__)
#define DTRACE_MODE_COST(...) dtraceModeCost(__VA_ARGS__)
#endif
#define DTRACE_CRC(...)                      dtrace_crc(__VA_ARGS__)
#define DTRACE_CCRC(...)                     dtrace_ccrc(__VA_ARGS__)


#else
#define DTRACE(ctx,channel,...)
#define DTRACE_UPDATE(ctx,s)
#define DTRACE_COND(cond,level,...)
#define DTRACE_REPEAT(ctx,channel,times,...)
#define DTRACE_SET(_dst,_src)  (_dst)=(_src)
#define DTRACE_BLOCK(...)
#define DTRACE_FRAME_BLOCKWISE(...)
#define DTRACE_GET_COUNTER(ctx,channel)

#define DTRACE_BLOCK_REC(...)
#define DTRACE_UNIT_COMP(...)
#define DTRACE_PIC_COMP(...)
#define DTRACE_BEST_MODE(...)
#define DTRACE_MODE_COST(...)
#define DTRACE_CRC(...)
#define DTRACE_CCRC(...)
#endif

#endif // _DTRACE_HEVC_H_
