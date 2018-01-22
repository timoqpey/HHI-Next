# BBuildEnv.cmake
#

# -
# functions and macros
# -

macro( nse_multithreading )
  if( CMAKE_SYSTEM_NAME STREQUAL "Linux" )
    add_compile_options( -pthread )
    set( THREADS_PREFER_PTHREAD_FLAG ON )
  endif()
  find_package( Threads REQUIRED )
endmacro( nse_multithreading )

macro( nse_enable_warnings )
  # translate CMAKE_CXX_COMPILER_ID to a string which compares easily with toolset -> gcc, msvc, clang.
  if( CMAKE_CXX_COMPILER_ID STREQUAL "GNU" )
    set( _bb_warning_compiler_id "gcc" )
  elseif( CMAKE_CXX_COMPILER_ID MATCHES "^(AppleClang|Clang)$" )
    set( _bb_warning_compiler_id "clang" )
  elseif( CMAKE_CXX_COMPILER_ID STREQUAL "MSVC" )
    set( _bb_warning_compiler_id "msvc" )
  else()
    return()
  endif()
 
  set( _bb_add_warning_options OFF )
 
  if( ${ARGC} GREATER 0 )
    set( _bb_tmp_list_var "${ARGN}" )
    # analyze the first argument it may be a toolset, warnings-as-errors or a warning option
    if( "${ARGV0}" MATCHES "^(gcc|clang|msvc)" )
      #message( STATUS "nse_enable_warnings(): found toolset argument ${ARGV0}" )
      if( "${ARGV0}" MATCHES "^([a-z]+)-([0-9.]+)" )
        # strip version suffix; e.g. gcc-4.8 -> gcc
        set( _bb_warning_toolset "${CMAKE_MATCH_1}" )
        set( _bb_warning_toolset_version "${CMAKE_MATCH_2}" )
      else()
        set( _bb_warning_toolset "${ARGV0}" )
        # fake a version matching the current compiler version which simplifies the logic below.
        set( _bb_warning_toolset_version "${bb_compiler_version_major_minor}" )
      endif()
      # drop the toolset from the list of arguments to be processed later on
      list( LENGTH _bb_tmp_list_var _bb_tmp_list_len )
      if( _bb_tmp_list_len EQUAL 1 )
        unset( _bb_tmp_list_var )
      else()
        list( REMOVE_AT _bb_tmp_list_var 0 )
      endif()
      unset( _bb_tmp_list_len )
    else()
      # first argument is not a supported toolset specification -> enable warnings for all supported compilers.
      set( _bb_warning_toolset "${_bb_warning_compiler_id}" )
      set( _bb_warning_toolset_version "${bb_compiler_version_major_minor}" )      
    endif()
  else()
    # no arguments specified -> enable warnings for all supported compilers.
    set( _bb_warning_toolset "${_bb_warning_compiler_id}" )
    set( _bb_warning_toolset_version "${bb_compiler_version_major_minor}" )
  endif()
  
  if( ( _bb_warning_toolset STREQUAL _bb_warning_compiler_id ) AND ( _bb_warning_toolset_version VERSION_EQUAL bb_compiler_version_major_minor ) )
    set( _bb_add_warning_options ON )
  endif()

  if( _bb_add_warning_options )
    if( CMAKE_CXX_COMPILER_ID MATCHES "^(AppleClang|Clang)$" )
      if( NOT XCODE )
        set( _bb_warning_options -Wall )
      endif()
    elseif( CMAKE_CXX_COMPILER_ID STREQUAL "GNU" )
      set( _bb_warning_options -Wall -fdiagnostics-show-option )
    elseif( CMAKE_CXX_COMPILER_ID STREQUAL "MSVC" )
      # Do we have to override the warning level?
    endif()
    
    if( DEFINED _bb_tmp_list_var )
      #message( STATUS "nse_enable_warnings(): processing additional warning options ${_bb_tmp_list_var}" )
      foreach( _bb_v IN LISTS _bb_tmp_list_var )
        # message( STATUS "processing ${_bb_v}" )
        if( ${_bb_v} STREQUAL "warnings-as-errors" )
          if( ( CMAKE_CXX_COMPILER_ID STREQUAL "GNU" ) OR ( CMAKE_CXX_COMPILER_ID MATCHES "^(AppleClang|Clang)$" ) )
            list( APPEND _bb_warning_options "-Werror" )
          elseif( CMAKE_CXX_COMPILER_ID STREQUAL "MSVC" )
            if( DEFINED _bb_warning_options )
              list( APPEND _bb_warning_options "/WX" )
            else()
              set( _bb_warning_options "/WX" )
            endif()
          endif()
        else()
          list( APPEND _bb_warning_options "${_bb_v}" )
        endif()
      endforeach()
      unset( _bb_tmp_list_var )
    endif()
    if( DEFINED _bb_warning_options )
      message( STATUS "nse_enable_warnings: ${toolset} -> updating warnings flags: ${_bb_warning_options}" )   
      add_compile_options( ${_bb_warning_options} )
      unset( _bb_warning_options )
    endif()    
  endif()
  
  unset( _bb_tmp_list_var )
  unset( _bb_warning_toolset )
  unset( _bb_warning_toolset_version )
  unset( _bb_add_warning_options )
  #message( STATUS "nse_enable_warnings: leaving" )
endmacro( nse_enable_warnings )


function( _nse_set_platform_dir platform_dir_ )
  if( MSVC )
    # x64, Win32
    if( CMAKE_VS_PLATFORM_NAME STREQUAL "x64" )
      set( _platform_dir "x86_64" )
    elseif( CMAKE_VS_PLATFORM_NAME STREQUAL "Win32" )
      set( _platform_dir "x86" )
    endif()
  else()
    if( CMAKE_SYSTEM_PROCESSOR MATCHES "^(i686|x86)$" )
      set( _platform_dir "x86" )
    elseif( CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|AMD64)$" )
      set( _platform_dir "x86_64" )
    else()
      set( _platform_dir "${CMAKE_SYSTEM_PROCESSOR}" )
    endif()
  endif()
  set( ${platform_dir_} "${_platform_dir}" PARENT_SCOPE )
endfunction( _nse_set_platform_dir )

function( _nse_set_toolset_subdir toolset_subdir_ compiler_version_ )
  # Assume cmake is able to figure out CMAKE_CXX_COMPILER_ID for all compilers including cross compilers.
  if( CMAKE_CXX_COMPILER_ID MATCHES "^(AppleClang|Clang)$" )
    set( _toolset_subdir "clang-${compiler_version_}" )
  elseif( CMAKE_CXX_COMPILER_ID STREQUAL "GNU" )
    set( _toolset_subdir "gcc-${compiler_version_}" )
  elseif( CMAKE_CXX_COMPILER_ID STREQUAL "MSVC" )
    set( _toolset_subdir "msvc-${compiler_version_}" )
  endif()
  
  if( MINGW )
    set( _toolset_subdir "gcc-mingw-${bb_compiler_version_major_minor}" )
  elseif( CMAKE_CROSSCOMPILING )
    message( FATAL_ERROR "no support for this type of cross compiler, please contact technical support." )
  endif()
  if( DEFINED _toolset_subdir )
    set( ${toolset_subdir_} "${_toolset_subdir}" PARENT_SCOPE )
  endif()
endfunction( _nse_set_toolset_subdir )

function( _nse_get_boost_build_toolset toolset_ )
  # Assume cmake is able to figure out CMAKE_CXX_COMPILER_ID for all compilers including cross compilers.
  string( REGEX REPLACE "([0-9]+)\\.([0-9]+)([0-9.]+)?" "\\1.\\2" _compiler_version_major_minor ${CMAKE_CXX_COMPILER_VERSION} )

  if( CMAKE_CXX_COMPILER_ID MATCHES "^(AppleClang|Clang)$" )
    set( _toolset "clang-${_compiler_version_major_minor}" )
  elseif( CMAKE_CXX_COMPILER_ID STREQUAL "GNU" )
    set( _toolset "gcc-${_compiler_version_major_minor}" )
  elseif( CMAKE_CXX_COMPILER_ID STREQUAL "MSVC" )
    if( MSVC14 )
      set( _toolset "msvc-14.0" )
    elseif( MSVC12 )
      set( _toolset "msvc-12.0" )
    elseif( MSVC11 )
      set( _toolset "msvc-11.0" )
    elseif( MSVC10 )
      set( _toolset "msvc-10.0" )
    else()
      message( FATAL_ERROR "_nse_get_boost_build_toolset() does not support the selected MSVC version, please contact technical support." )
    endif()
  endif()
  set( ${toolset_} ${_toolset} PARENT_SCOPE )
 endfunction( )

# -
# end of functions and macros
# -


# Add a cmake generator alias
# --
# Visual Studio 15 [arch]
# Visual Studio 14 2015 [arch]
# Visual Studio 12 2013 [arch]
# Visual Studio 11 2012 [arch]
# Visual Studio 10 2010 [arch]
# --
if( CMAKE_GENERATOR STREQUAL "Unix Makefiles" )
  set( bb_generator_alias "umake" )
elseif( CMAKE_GENERATOR STREQUAL "Xcode" )
  set( bb_generator_alias "xcode" )
elseif( CMAKE_GENERATOR MATCHES "Visual Studio ([0-9][0-9])" )
  set( bb_generator_alias "vs${CMAKE_MATCH_1}" )
elseif( CMAKE_GENERATOR STREQUAL "Ninja" )
  set( bb_generator_alias "ninja" )
else()
  message( WARNING "BBuildEnv.cmake: generator '${CMAKE_GENERATOR}' is not fully supported yet, please contact technical support for further information." ) 
  #return()
  string( TOLOWER "${CMAKE_GENERATOR}" _generator_uc )
  string( REPLACE " " "_" bb_generator_alias "${_generator_uc}" )
  unset( _generator_uc )
endif()
#message( STATUS "bb_generator_alias: ${bb_generator_alias}" ) 
string( REGEX REPLACE "([0-9]+)\\.([0-9]+)([0-9.]+)?" "\\1.\\2" bb_compiler_version_major_minor ${CMAKE_CXX_COMPILER_VERSION} )

_nse_set_toolset_subdir( bb_toolset_subdir ${bb_compiler_version_major_minor} )
_nse_set_platform_dir( bb_platform_dir )

# set standard output directories: gcc-5.4/x86_64
if( DEFINED bb_generator_alias )
  set( bb_default_output_dir "${bb_generator_alias}/${bb_toolset_subdir}/${bb_platform_dir}" )
else()
  set( bb_default_output_dir "${bb_toolset_subdir}/${bb_platform_dir}" )
endif()

set( CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG          "${CMAKE_SOURCE_DIR}/bin/${bb_default_output_dir}/debug" )
set( CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE        "${CMAKE_SOURCE_DIR}/bin/${bb_default_output_dir}/release" )
set( CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO "${CMAKE_SOURCE_DIR}/bin/${bb_default_output_dir}/relwithdebinfo" )
set( CMAKE_RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL     "${CMAKE_SOURCE_DIR}/bin/${bb_default_output_dir}/minsizerel" )

set( CMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG          "${CMAKE_SOURCE_DIR}/lib/${bb_default_output_dir}/debug" )
set( CMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELEASE        "${CMAKE_SOURCE_DIR}/lib/${bb_default_output_dir}/release" )
set( CMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELWITHDEBINFO "${CMAKE_SOURCE_DIR}/lib/${bb_default_output_dir}/relwithdebinfo" )
set( CMAKE_ARCHIVE_OUTPUT_DIRECTORY_MINSIZEREL     "${CMAKE_SOURCE_DIR}/lib/${bb_default_output_dir}/minsizerel" )

set( CMAKE_LIBRARY_OUTPUT_DIRECTORY_DEBUG          "${CMAKE_SOURCE_DIR}/lib/${bb_default_output_dir}/debug" )
set( CMAKE_LIBRARY_OUTPUT_DIRECTORY_RELEASE        "${CMAKE_SOURCE_DIR}/lib/${bb_default_output_dir}/release" )
set( CMAKE_LIBRARY_OUTPUT_DIRECTORY_RELWITHDEBINFO "${CMAKE_SOURCE_DIR}/lib/${bb_default_output_dir}/relwithdebinfo" )
set( CMAKE_LIBRARY_OUTPUT_DIRECTORY_MINSIZEREL     "${CMAKE_SOURCE_DIR}/lib/${bb_default_output_dir}/minsizerel" )

