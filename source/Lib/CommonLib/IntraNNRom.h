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

/** \file     IntraNNRom.h
 *  \brief    IntraNNRom
 */

#ifndef INTRA_NN_ROM
#define INTRA_NN_ROM

#include "CommonDef.h"

#define  MAX_NUM_NN_MODES            35
#define  NUM_NN_MODES_THIRTY_TWO     11
#define  MAX_NUM_LAYERS_BLOCK         4
#define  MAX_NUM_LAYERS_MODE          2

#define NUM_NSST_TRAFOS_PER_NN_MODE                      4
#define MAX_ONE_D_DIMENSION_FOR_NN_TRAFO                 8


#define  ASSUMED_KEPT_PRECISION                          7
// values for kept-precision=7
#define  MAX_SIZE_FOR_ELU_ZEROTH_LAYER_BLOCK          2841
#define  MAX_SIZE_FOR_ELU_FIRST_LAYER_BLOCK           5325
#define  MAX_SIZE_FOR_ELU_SECOND_LAYER_BLOCK          7809
#define  MAX_SIZE_FOR_ELU_THIRD_LAYER_BLOCK              1
#define  PRECISION_ELU_TABLE                             (ASSUMED_KEPT_PRECISION+2) 



extern int g_assumed_standard_bitdepth;
extern int g_max_size_NNs;
extern int g_PaddedSizes[65];

extern int g_numIntraModes_NN [MAX_CU_DEPTH + 1][MAX_CU_DEPTH + 1]; // number of NN-IntraModes  for width and height
extern int g_numIntraMPM_NN   [MAX_CU_DEPTH + 1][MAX_CU_DEPTH + 1]; // number of NN-Intra-MPMS  for width and height
extern int g_numIntraEPBins_NN[MAX_CU_DEPTH + 1][MAX_CU_DEPTH + 1]; // number of NN-IntraEPBins for width and height

// Block
extern int g_Architecture_Block_NN [MAX_CU_DEPTH + 1][MAX_CU_DEPTH + 1][MAX_NUM_LAYERS_BLOCK +1];
extern int g_NumLayers_Block_NN    [MAX_CU_DEPTH + 1][MAX_CU_DEPTH + 1];
extern int g_Context_Sizes_Block_NN[MAX_CU_DEPTH + 1][MAX_CU_DEPTH + 1];


// Mode
extern int g_Architecture_Mode_NN [MAX_CU_DEPTH + 1][MAX_CU_DEPTH + 1][MAX_NUM_LAYERS_MODE + 1];
extern int g_NumLayers_Mode_NN    [MAX_CU_DEPTH + 1][MAX_CU_DEPTH + 1];
extern int g_Context_Sizes_Mode_NN[MAX_CU_DEPTH + 1][MAX_CU_DEPTH + 1];


// Shift values
extern int g_Right_Shifts_ResultNN_wo_Bias         [MAX_CU_DEPTH + 1][MAX_CU_DEPTH + 1][MAX_NUM_NN_MODES];
extern int g_Right_Shifts_ResultNN_Matrix_Bias     [MAX_CU_DEPTH + 1][MAX_CU_DEPTH + 1][MAX_NUM_NN_MODES];

extern int g_Internal_Right_Shifts_wo_Bias         [MAX_CU_DEPTH + 1][MAX_CU_DEPTH + 1][MAX_NUM_LAYERS_BLOCK];
extern int g_Internal_Right_Shifts_Matrix_Bias     [MAX_CU_DEPTH + 1][MAX_CU_DEPTH + 1][MAX_NUM_LAYERS_BLOCK];

extern int g_Internal_Right_Shifts_Mode_wo_Bias    [MAX_CU_DEPTH + 1][MAX_CU_DEPTH + 1][MAX_NUM_LAYERS_MODE];
extern int g_Internal_Right_Shifts_Mode_Matrix_Bias[MAX_CU_DEPTH + 1][MAX_CU_DEPTH + 1][MAX_NUM_LAYERS_MODE];

extern int g_RightShifts_For_Last_Layer            [MAX_CU_DEPTH + 1][MAX_CU_DEPTH + 1];

extern int g_KeptPrecisionsJoint                   [MAX_CU_DEPTH + 1][MAX_CU_DEPTH + 1];

// ELU data
extern int g_EluTableSizes           [MAX_NUM_LAYERS_BLOCK];
extern int g_EluTableBlockZerothLayer[MAX_SIZE_FOR_ELU_ZEROTH_LAYER_BLOCK];
extern int g_EluTableBlockFirstLayer [MAX_SIZE_FOR_ELU_FIRST_LAYER_BLOCK];
extern int g_EluTableBlockSecondLayer[MAX_SIZE_FOR_ELU_SECOND_LAYER_BLOCK];
extern int g_EluTableBlockThirdLayer [MAX_SIZE_FOR_ELU_THIRD_LAYER_BLOCK];



void xGetEluTable(int* &elu_table, const int layer_idx);

bool   subSamplingAllowed     ( const int width, const int height );
bool   isTransposed           ( const int width, const int height );
int    getNumModesNNPadded    ( const int width, const int height );

void   computIntegerInverseDCT( const int* weights, int* weightsTransformed, const int width, const int height );

UChar  workInTrafoDomain      ( const int log2width, const int log2height );
UChar* getMaskNonZeroRows     ( const int log2width, const int log2height, const int mode_idx );
int    getNumNonZeroRowsBlock ( const int log2width, const int log2height, const int mode_idx );


// Functions for accessing weights
void getWeightPointersMode( const int width, const int height, const int layer_pos, int* &weightsMatrix, int* &weightsBias );

void getWeightPointerModeWidth4 ( const int height, const int layer_pos, int* &weightsMatrix, int* &weightsBias );
void getWeightPointerModeWidth8 ( const int height, const int layer_pos, int* &weightsMatrix, int* &weightsBias );
void getWeightPointerModeWidth16( const int height, const int layer_pos, int* &weightsMatrix, int* &weightsBias );
void getWeightPointerModeWidth32( const int height, const int layer_pos, int* &weightsMatrix, int* &weightsBias );

void getWeightPointersBlockLastLayer( const int width, const int height, const int mode_idx, short* &weightsMatrix, int* &weightsBias );

void getWeightPointerBlockLastLayerWidth4 ( const int height, const int mode_idx, short* &weightsMatrix, int* &weightsBias );
void getWeightPointerBlockLastLayerWidth8 ( const int height, const int mode_idx, short* &weightsMatrix, int* &weightsBias );
void getWeightPointerBlockLastLayerWidth16( const int height, const int mode_idx, short* &weightsMatrix, int* &weightsBias );
void getWeightPointerBlockLastLayerWidth32( const int height, const int mode_idx, short* &weightsMatrix, int* &weightsBias );

void getWeightPointersBlock( const int width, const int height, const int layer_pos, int* &weightsMatrix, int* &weightsBias );
void getWeightPointerBlockWidth4 ( const int height, const int layer_pos, int*&weightsMatrix, int*&weightsBias );
void getWeightPointerBlockWidth8 ( const int height, const int layer_pos, int*&weightsMatrix, int*&weightsBias );
void getWeightPointerBlockWidth16( const int height, const int layer_pos, int*&weightsMatrix, int*&weightsBias );
void getWeightPointerBlockWidth32( const int height, const int layer_pos, int*&weightsMatrix, int*&weightsBias );

#endif
