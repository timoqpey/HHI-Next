# executable
set( EXE_NAME DecoderApp )

# get source files
file( GLOB SRC_FILES "*.cpp" )

# get include files
file( GLOB INC_FILES "*.h" )

# get additional libs for gcc on Ubuntu systems
if( CMAKE_SYSTEM_NAME STREQUAL "Linux" )
  if( CMAKE_CXX_COMPILER_ID STREQUAL "GNU" )
    if( USE_ADDRESS_SANITIZER )
      set( ADDITIONAL_LIBS asan )
    endif()
  endif()
endif()

# NATVIS files for Visual Studio
if( MSVC )
  file( GLOB NATVIS_FILES "../../VisualStudio/*.natvis" )
endif()

# add executable
add_executable( ${EXE_NAME} ${SRC_FILES} ${INC_FILES} ${NATVIS_FILES} ${CMAKE_CURRENT_BINARY_DIR}/svnheader.h )
# include the output directory, where the svnrevision.h file is generated
include_directories(${CMAKE_CURRENT_BINARY_DIR})
if( SET_ENABLE_TRACING )
  if( ENABLE_TRACING )
    target_compile_definitions( ${EXE_NAME} PUBLIC ENABLE_TRACING=1 )
  else()
    target_compile_definitions( ${EXE_NAME} PUBLIC ENABLE_TRACING=0 )
  endif()
endif()
target_link_libraries( ${EXE_NAME} CommonLib DecoderLib Utilities Threads::Threads ${ADDITIONAL_LIBS} )

# Add a SVN revision generator
# a custom target that is always built
add_custom_target( DecSvnHeader ALL DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/svnheader.h )
# creates svnrevision.h using cmake script
add_custom_command( OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/svnheader.h COMMAND ${CMAKE_COMMAND} -DSOURCE_DIR=${CMAKE_SOURCE_DIR} -DGENERATE_DUMMY=${SKIP_SVN_REVISION} -P ${CMAKE_SOURCE_DIR}/cmake/modules/GetSVN.cmake )
# svnrevision.h is a generated file
set_source_files_properties( ${CMAKE_CURRENT_BINARY_DIR}/svnrevision.h PROPERTIES GENERATED TRUE HEADER_FILE_ONLY TRUE )

# explicitly say that the executable depends on the EncSvnHeader
add_dependencies( ${EXE_NAME} DecSvnHeader )

# lldb custom data formatters
if( XCODE )
  add_dependencies( ${EXE_NAME} Install${PROJECT_NAME}LldbFiles )
endif()

if( CMAKE_SYSTEM_NAME STREQUAL "Linux" )
  add_custom_command( TARGET ${EXE_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy
                                                          $<$<CONFIG:Debug>:${CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG}/DecoderApp>
                                                          $<$<CONFIG:Release>:${CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE}/DecoderApp>
                                                          $<$<CONFIG:RelWithDebInfo>:${CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO}/DecoderApp>
                                                          $<$<CONFIG:MinSizeRel>:${CMAKE_RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL}/DecoderApp>
                                                          $<$<CONFIG:Debug>:${CMAKE_SOURCE_DIR}/bin/DecoderAppStaticd>
                                                          $<$<CONFIG:Release>:${CMAKE_SOURCE_DIR}/bin/DecoderAppStatic>
                                                          $<$<CONFIG:RelWithDebInfo>:${CMAKE_SOURCE_DIR}/bin/DecoderAppStaticp>
                                                          $<$<CONFIG:MinSizeRel>:${CMAKE_SOURCE_DIR}/bin/DecoderAppStaticm> )
endif()                    

# example: place header files in different folders
source_group( "Natvis Files" FILES ${NATVIS_FILES} )

# set the folder where to place the projects
set_target_properties( ${EXE_NAME}  PROPERTIES FOLDER app )
set_target_properties( DecSvnHeader PROPERTIES FOLDER svn )
