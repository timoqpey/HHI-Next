# minimum required cmake version
cmake_minimum_required( VERSION 3.5 FATAL_ERROR )

# project name
project( NextSoftware )

# use ccache
find_program( CCACHE_FOUND ccache )
if( CCACHE_FOUND )
  message( STATUS "ccache found. using it." )
  set_property( GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache )
  set_property( GLOBAL PROPERTY RULE_LAUNCH_LINK ccache )
endif()

# set default CMAKE_BUILD_TYPE to Release if not set
if( NOT CMAKE_BUILD_TYPE )
  set( CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel." FORCE )
endif()

if( CMAKE_SYSTEM_NAME STREQUAL "Linux" )
  if( CMAKE_CXX_COMPILER_ID STREQUAL "GNU" )
    set( USE_ADDRESS_SANITIZER OFF CACHE BOOL "Compiles with -sanitize=address and links to libasan" )
  endif()
endif()

set( SKIP_SVN_REVISION OFF CACHE BOOL "Don't add SVN path and revision info to the encoder and decoder output" )
set( SET_ENABLE_TRACING OFF CACHE BOOL "Set ENABLE_TRACING as a compiler flag" )
set( ENABLE_TRACING OFF CACHE BOOL "If ENABLE_TRACING is on, it will be set to this value" )

# set c++11
set( CMAKE_CXX_STANDARD 11 )
set( CMAKE_CXX_STANDARD_REQUIRED ON )

# compile everything position independent (even static libraries)
set( CMAKE_POSITION_INDEPENDENT_CODE TRUE )

# set verbose compile options
#set( CMAKE_VERBOSE_MAKEFILE ON )

# use folders in IDEs for projects (e.g. lib sample app test)
set_property( GLOBAL PROPERTY USE_FOLDERS ON )

# Include a utility module providing functions, macros, and settings
include( ${CMAKE_SOURCE_DIR}/cmake/modules/NextSoftwareEnv.cmake )

# Enable multithreading
nse_multithreading()

# Enable warnings for some generators and toolsets.
nse_enable_warnings( gcc warnings-as-errors -Wno-sign-compare )
# nse_enable_warnings( gcc -Wno-unused-variable )
# nse_enable_warnings( gcc-4.8 warnings-as-errors -Wno-unused-variable )

if( XCODE )
  nse_enable_warnings( clang warnings-as-errors
                            -Wno-deprecated-declarations
                            -Wno-unknown-attributes
                            -Wno-deprecated-register
                            -Wno-pessimizing-move
                            -Wno-absolute-value
                            -Wno-unused-const-variable )
else()
  nse_enable_warnings( clang warnings-as-errors
                            -Wno-unknown-attributes
                            -Wno-deprecated-register
                            -Wno-pessimizing-move
                            -Wno-absolute-value
                            -Wno-unused-const-variable )
endif()
#nse_enable_warnings( clang warnings-as-errors )

# enable warnings
nse_enable_warnings( msvc warnings-as-errors "/wd4996" )

# enable sse4.1 build for all source files for gcc and clang
if( UNIX )
  add_compile_options( "-msse4.1" )
endif()

# enable parallel build for Visual Studio
if( MSVC )
  add_compile_options( "/MP" )
  add_compile_options( "/EHsc" )
endif()

# set address sanitizer compiler arguments
if( CMAKE_CXX_COMPILER_ID STREQUAL "GNU" )
  if( CMAKE_SYSTEM_NAME STREQUAL "Linux" )
    if( USE_ADDRESS_SANITIZER )
      # add compile options
      add_compile_options( "-fsanitize=address" )
    endif()
  endif()
  
  if( CMAKE_CXX_COMPILER_VERSION VERSION_LESS 5.0 )
    add_compile_options( "-fabi-version=6" )
  endif()
endif()

# modify .lldbinit for lldb custom data formatters
if( XCODE )
  set( LLDB_INSTALL_ROOT "$ENV{HOME}/.lldb.d" )
  set( LLDBINIT_FILE "$ENV{HOME}/.lldbinit" )  
  set( ENABLE_LLDBINIT_UPDATE ON )  
  # add custom target to install LLDB files.
  add_subdirectory( "lldb" )
endif()

# add needed subdirectories
add_subdirectory( "source/Lib/CommonLib" )
add_subdirectory( "source/Lib/DecoderAnalyserLib" )
add_subdirectory( "source/Lib/DecoderLib" )
add_subdirectory( "source/Lib/EncoderLib" )
add_subdirectory( "source/Lib/Utilities" )

add_subdirectory( "source/App/DecoderAnalyserApp" )
add_subdirectory( "source/App/DecoderApp" )
add_subdirectory( "source/App/EncoderApp" )

