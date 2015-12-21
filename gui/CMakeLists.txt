##GUI CMake

#PROJECT(gui)

#GCC options example TODO remove -g option in release version
# TODO  segur k no e de fer un remove_definitions()???
##ADD_DEFINITIONS(-Wall -fPIC -DPIC -g)

find_package(PkgConfig)
pkg_check_modules(GTKMM gtkmm-2.4)

add_subdirectory(widgets) 

link_directories(${GTKMM_LIBRARY_DIRS})

include_directories(${GTKMM_INCLUDE_DIRS})

#===========================BUILD EQ10Q ALL GUI FILES=============================================

#Build fbdm GUI
set( Eq_Bands_Count 10)
set( Eq_Uri  "\"https://github.com/Dominik-Sauer/eq10q-fork/fbdm/gui\"")
configure_file("${PROJECT_SOURCE_DIR}/gui/eq10q_ui.cpp" "${PROJECT_SOURCE_DIR}/gui/fbdm_ui.cpp")
add_library(fbdm_gui SHARED fbdm_ui.cpp)
target_link_libraries( fbdm_gui  eqwin ${GTKMM_LIBRARIES})

#remove the lib prefix to all generated .so files
set_target_properties(fbdm_gui PROPERTIES PREFIX "")

##CLEAN generated .cpp files on run 'make clean'
SET(generatedGuiConfigFiles
    ${PROJECT_SOURCE_DIR}/gui/fbdm_ui.cpp
)
SET_DIRECTORY_PROPERTIES(PROPERTIES  ADDITIONAL_MAKE_CLEAN_FILES "${generatedGuiConfigFiles}")

##=================================INSTALLER===================================================================================
INSTALL( TARGETS fbdm_gui DESTINATION "${LV2_BUNDLE}/gui")
INSTALL( DIRECTORY "${PROJECT_SOURCE_DIR}/gui/knobs" DESTINATION "${LV2_BUNDLE}/gui")
INSTALL( DIRECTORY "${PROJECT_SOURCE_DIR}/gui/combopix" DESTINATION "${LV2_BUNDLE}/gui")
INSTALL( DIRECTORY "${PROJECT_SOURCE_DIR}/gui/icons" DESTINATION "${LV2_BUNDLE}/gui")

