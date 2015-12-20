##EQ10Q TopLevel CMake
cmake_minimum_required(VERSION 2.8)
PROJECT(eq10q)

##ADD_DEFINITIONS(-Wall -O3 -fPIC -finline-functions -finline-functions-called-once  -msse -mfpmath=sse -std=c99)
set(CMAKE_C_FLAGS "-Wall -O3 -fPIC -finline-functions -finline-functions-called-once  -msse -mfpmath=sse -std=c99")
#set(CMAKE_C_FLAGS "-Wall -O0 -g -fPIC -finline-functions -finline-functions-called-once  -msse -mfpmath=sse -std=c99")

set(CMAKE_CXX_FLAGS "-Wall -fPIC -std=c++11")
#set(CMAKE_CXX_FLAGS "-Wall -O0 -g -fPIC -DPIC") ##Debuging GUI

##ADD_DEFINITIONS(-Wall -O0 -g -fPIC -DPIC) ###USED FOR DEBUG

#Set Install prefix
set (CMAKE_INSTALL_PREFIX  "/usr/local/lib/lv2") #Install prefix
#set (CMAKE_INSTALL_PREFIX  "$ENV{HOME}/.lv2") #Install prefix
set (LV2_BUNDLE "dominik-sauer-fbd.lv2") #LV2 Bundel dir
set(EXECUTABLE_OUTPUT_PATH "${PROJECT_SOURCE_DIR}/bin")
set(LIBRARY_OUTPUT_PATH "${PROJECT_SOURCE_DIR}/bin")

add_subdirectory(gui) 
add_subdirectory(dsp)

#===========================BUILD EQ10Q ALL DSP FILES=============================================

#Build fbdm DSP
set( Eq_Bands_Count 10)
set( Eq_Channels_Count 1)
set( Eq_Uri  "\"https://github.com/Dominik-Sauer/eq10q-fork/fbdm\"")
configure_file("${PROJECT_SOURCE_DIR}/eq.c" "${PROJECT_SOURCE_DIR}/fbdm.c")
add_library(fbdm SHARED fbdm.c)
target_link_libraries( fbdm  lib_filter lib_vumeter m fftw3)

#Build fbds DSP
set( Eq_Bands_Count 10)
set( Eq_Channels_Count 2)
set( Eq_Uri  "\"https://github.com/Dominik-Sauer/eq10q-fork/fbds\"")
configure_file("${PROJECT_SOURCE_DIR}/eq.c" "${PROJECT_SOURCE_DIR}/fbds.c")
add_library(fbds SHARED fbds.c)
target_link_libraries( fbds  lib_filter lib_vumeter m fftw3)

#remove the lib prefix to all generated .so files
set_target_properties(fbdm fbds PROPERTIES PREFIX "")

#CLEAN generated .c files on run 'make clean'
SET( generatedEqConfigFiles ${PROJECT_SOURCE_DIR}/fbdm.c;${PROJECT_SOURCE_DIR}/fbds.c )
SET_DIRECTORY_PROPERTIES(PROPERTIES  ADDITIONAL_MAKE_CLEAN_FILES "${generatedEqConfigFiles}")


##===========================THE LV2 INSTALLER================================================
#INSTALL( DIRECTORY "${PROJECT_SOURCE_DIR}/${LV2_BUNDLE}" DESTINATION "${LV2_BUNDLE}")
INSTALL( FILES "manifest.ttl" "fbdm.ttl" "fbds.ttl" DESTINATION "${LV2_BUNDLE}")
INSTALL( TARGETS fbdm fbds DESTINATION "${LV2_BUNDLE}")

INCLUDE(CPack)
# cpack -G TGZ
 
