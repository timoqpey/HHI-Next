/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2015, ITU/ISO/IEC
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

/** \file     TEncBinCoderCABAC.cpp
    \brief    binary entropy encoder of CABAC
*/

#include "TEncBinCoderCABACCounter.h"
#include "TLibCommon/TComRom.h"
#include "TLibCommon/Debug.h"

#include "TLibCommon/dtrace_next.h"

#if TRACING
static unsigned int cabac_symbol_counter_est = 0;
#define CNT_OFFSET   0
#define STOP_SYM_CNT    3433632
#define STOP_SYM_CNT_EP STOP_SYM_CNT
#endif

#if FAST_BIT_EST

//! \ingroup TLibEncoder
//! \{


TEncBinCABACCounter::TEncBinCABACCounter()
{
}

TEncBinCABACCounter::~TEncBinCABACCounter()
{
}

Void TEncBinCABACCounter::finish()
{
  m_pcTComBitIf->write(0, UInt(m_fracBits >> 15) );
  m_fracBits &= 32767;
}

UInt TEncBinCABACCounter::getNumWrittenBits()
{
#if TRACING
  DTRACE(g_trace_ctx, D_EST_FRAC_BITS, "%d: FBits=%d\n", DTRACE_GET_COUNTER( g_trace_ctx, D_EST_FRAC_BITS ), m_fracBits);
#endif
  return m_pcTComBitIf->getNumberOfWrittenBits() + UInt( m_fracBits >> 15 );
}


UInt64 TEncBinCABACCounter::getFracBits()
{
  return ((UInt64)(m_pcTComBitIf->getNumberOfWrittenBits()) << 15) +  m_fracBits;
}
/**
 * \brief Encode bin
 *
 * \param binValue   bin value
 * \param rcCtxModel context model
 */
Void TEncBinCABACCounter::encodeBin( UInt binValue, ContextModel &rcCtxModel )
{
#if DEBUG_ENCODER_SEARCH_BINS
  const UInt64 startingFracBits = m_fracBits;
#endif
#if TRACING
  DTRACE(g_trace_ctx, D_FRAC_BITS, "%d" "  " "B=%d\n", DTRACE_GET_COUNTER( g_trace_ctx, D_FRAC_BITS ), rcCtxModel.getEntropyBits(binValue));
  cabac_symbol_counter_est++;
#endif

  m_uiBinsCoded += m_binCountIncrement;
  m_fracBits += rcCtxModel.getEntropyBits( binValue );
#if VCEG_AZ07_BAC_ADAPT_WDOW  || VCEG_AZ05_MULTI_PARAM_CABAC 
  if( binValue == 0 )
  {
    rcCtxModel.updateLPS();
  }
  else
  {
    rcCtxModel.updateMPS();
  }
#else
  rcCtxModel.update( binValue );
#endif

#if DEBUG_ENCODER_SEARCH_BINS
  if ((g_debugCounter + debugEncoderSearchBinWindow) >= debugEncoderSearchBinTargetLine)
  {
    std::cout << g_debugCounter << ": coding bin value " << binValue << ", fracBits = [" << startingFracBits << "->" << m_fracBits << "]\n";
  }

  if (g_debugCounter >= debugEncoderSearchBinTargetLine)
  {
    Char breakPointThis;
    breakPointThis = 7;
  }
  if (g_debugCounter >= (debugEncoderSearchBinTargetLine + debugEncoderSearchBinWindow))
  {
    exit(0);
  }
  g_debugCounter++;
#endif
}

/**
 * \brief Encode equiprobable bin
 *
 * \param binValue bin value
 */
Void TEncBinCABACCounter::encodeBinEP( UInt /*binValue*/ )
{
  m_uiBinsCoded += m_binCountIncrement;
  m_fracBits += 32768;
#if TRACING
  DTRACE(g_trace_ctx, D_FRAC_BITS, "%d" "  " "EP=%d\n", DTRACE_GET_COUNTER( g_trace_ctx, D_FRAC_BITS ), 32768);
  cabac_symbol_counter_est++;
#endif
}

/**
 * \brief Encode equiprobable bins
 *
 * \param binValues bin values
 * \param numBins number of bins
 */
Void TEncBinCABACCounter::encodeBinsEP( UInt /*binValues*/, Int numBins )
{
  m_uiBinsCoded += numBins & -m_binCountIncrement;
  m_fracBits += 32768 * numBins;
#if TRACING
  DTRACE(g_trace_ctx, D_FRAC_BITS, "%d" "  " "EPs=%d\n", DTRACE_GET_COUNTER( g_trace_ctx, D_FRAC_BITS ), 32768 * numBins);
  cabac_symbol_counter_est++;
#endif
}

/**
 * \brief Encode terminating bin
 *
 * \param binValue bin value
 */
Void TEncBinCABACCounter::encodeBinTrm( UInt binValue )
{
  m_uiBinsCoded += m_binCountIncrement;
  m_fracBits += ContextModel::getEntropyBitsTrm( binValue );
}

Void TEncBinCABACCounter::align()
{
  m_fracBits = (m_fracBits + 32767) & (~32767);
}

//! \}
#endif

