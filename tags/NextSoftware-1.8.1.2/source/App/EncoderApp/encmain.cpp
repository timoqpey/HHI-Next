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

/** \file     encmain.cpp
    \brief    Encoder application main
*/

#include <time.h>
#include <iostream>
#include "EncApp.h"
#include "Utilities/program_options_lite.h"

#include "svnrevision.h"

//! \ingroup EncoderApp
//! \{

static const UInt settingNameWidth = 66;
static const UInt settingHelpWidth = 84;
static const UInt settingValueWidth = 3;
// --------------------------------------------------------------------------------------------------------------------- //

//macro value printing function

#define PRINT_CONSTANT(NAME, NAME_WIDTH, VALUE_WIDTH) std::cout << std::setw(NAME_WIDTH) << #NAME << " = " << std::setw(VALUE_WIDTH) << NAME << std::endl;

static void printMacroSettings()
{
  if( g_verbosity >= DETAILS )
  {
    std::cout << "Non-environment-variable-controlled macros set as follows: \n" << std::endl;

    //------------------------------------------------

    //setting macros

    PRINT_CONSTANT( RExt__DECODER_DEBUG_BIT_STATISTICS,                         settingNameWidth, settingValueWidth );
    PRINT_CONSTANT( RExt__HIGH_BIT_DEPTH_SUPPORT,                               settingNameWidth, settingValueWidth );
    PRINT_CONSTANT( RExt__HIGH_PRECISION_FORWARD_TRANSFORM,                     settingNameWidth, settingValueWidth );
    PRINT_CONSTANT( ME_ENABLE_ROUNDING_OF_MVS,                                  settingNameWidth, settingValueWidth );

    //------------------------------------------------

    std::cout << std::endl;
  }
}

// ====================================================================================================================
// Main function
// ====================================================================================================================

int main(int argc, char* argv[])
{
  // print information
  fprintf( stdout, "\n" );
#ifdef SVNREVISION
  fprintf( stdout, "NextSoftware: Encoder Version %s (%s@r%s) ", NEXT_SOFTWARE_VERSION, SVNRELATIVEURL, SVNREVISION /*NV_VERSION*/ );
#else
  fprintf( stdout, "NextSoftware: Encoder Version %s ", NEXT_SOFTWARE_VERSION /*NV_VERSION*/ );
#endif
  fprintf( stdout, NVM_ONOS );
  fprintf( stdout, NVM_COMPILEDBY );
  fprintf( stdout, NVM_BITS );
#if HHI_SIMD_OPT
  std::string SIMD;
  df::program_options_lite::Options opts;
  opts.addOptions() ( "SIMD", SIMD, string( "" ), "" );
  df::program_options_lite::SilentReporter err;
  df::program_options_lite::scanArgv( opts, argc, ( const TChar** ) argv, err );
  fprintf( stdout, "[SIMD=%s] ", read_x86_extension( SIMD ) );
#endif
#if ENABLE_TRACING
  fprintf( stdout, "[ENABLE_TRACING] " );
#endif
  fprintf( stdout, "\n" );

  EncApp* pcEncApp = new EncApp;
  // create application encoder class
  pcEncApp->create();

  // parse configuration
  try
  {
    if(!pcEncApp->parseCfg( argc, argv ))
    {
      pcEncApp->destroy();
      return 1;
    }
  }
  catch (df::program_options_lite::ParseFailure &e)
  {
    std::cerr << "Error parsing option \""<< e.arg <<"\" with argument \""<< e.val <<"\"." << std::endl;
    return 1;
  }

#if PRINT_MACRO_VALUES
  printMacroSettings();
#endif

  // starting time
  Double dResult;
  clock_t lBefore = clock();

  // call encoding function
#ifndef _DEBUG
  try
  {
#endif
    pcEncApp->encode();
#ifndef _DEBUG
  }
  catch( Exception &e )
  {
    std::cerr << e.what() << std::endl;
    return 1;
  }
  catch( ... )
  {
    std::cerr << "Unspecified error occurred" << std::endl;
    return 1;
  }
#endif
  // ending time
  dResult = (Double)(clock()-lBefore) / CLOCKS_PER_SEC;
  printf("\n Total Time: %12.3f sec.\n", dResult);
  // destroy application encoder class
  pcEncApp->destroy();

  delete pcEncApp;

  return 0;
}

//! \}
