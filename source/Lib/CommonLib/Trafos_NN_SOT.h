
#ifndef __TRAFOS_NN_SOT__
#define __TRAFOS_NN_SOT__

#include "Rom.h"

extern const UShort g_TrKeys4_32[4][4][35][5];
extern const UShort g_TrKeys64[35][5];
extern const UShort g_TrKeysDefault[35][5];
extern const UShort g_TrKeys4_12[35][5];
extern const UShort g_TrKeys12_4[35][5];


extern const TMatrixCoeff g_aiNNT4x4[140 * 256];
extern const TMatrixCoeff g_aiNNT4x8[140 * 1024];
extern const TMatrixCoeff g_aiNNT4x16[140 * 1024];
extern const TMatrixCoeff g_aiNNT4x32[44 * 1024];
extern const TMatrixCoeff g_aiNNT8x4[140 * 1024];
extern const TMatrixCoeff g_aiNNT8x8[140 * 4096];
extern const TMatrixCoeff g_aiNNT8x16[140 * 4096];
extern const TMatrixCoeff g_aiNNT8x32[44 * 4096];
extern const TMatrixCoeff g_aiNNT16x4[140 * 1024];
extern const TMatrixCoeff g_aiNNT16x8[140 * 4096];
extern const TMatrixCoeff g_aiNNT16x16[140 * 4096];
extern const TMatrixCoeff g_aiNNT16x32[44 * 4096];
extern const TMatrixCoeff g_aiNNT32x4[44 * 1024];
extern const TMatrixCoeff g_aiNNT32x8[44 * 4096];
extern const TMatrixCoeff g_aiNNT32x16[44 * 4096];
extern const TMatrixCoeff g_aiNNT32x32[44 * 4096];


extern const TMatrixCoeff g_aiSOTT4x4[1 * 1];
extern const TMatrixCoeff g_aiSOTT4x8[1 * 1];
extern const TMatrixCoeff g_aiSOTT4x16[93 * 1024];
extern const TMatrixCoeff g_aiSOTT4x32[82 * 1024];
extern const TMatrixCoeff g_aiSOTT8x4[1 * 1];
extern const TMatrixCoeff g_aiSOTT8x8[1 * 1];
extern const TMatrixCoeff g_aiSOTT8x16[99 * 4096];
extern const TMatrixCoeff g_aiSOTT8x32[91 * 4096];
extern const TMatrixCoeff g_aiSOTT16x4[100 * 1024];
extern const TMatrixCoeff g_aiSOTT16x8[101 * 4096];
extern const TMatrixCoeff g_aiSOTT16x16[1 * 1];
extern const TMatrixCoeff g_aiSOTT16x32[101 * 4096];
extern const TMatrixCoeff g_aiSOTT32x4[79 * 1024];
extern const TMatrixCoeff g_aiSOTT32x8[92 * 4096];
extern const TMatrixCoeff g_aiSOTT32x16[102 * 4096];
extern const TMatrixCoeff g_aiSOTT32x32[1 * 1];
extern const TMatrixCoeff g_aiSOTT4x12[1 * 1];
extern const TMatrixCoeff g_aiSOTT12x4[1 * 1];


#endif
