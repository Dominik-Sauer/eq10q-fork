##Widgets CMake

link_directories(${GTKMM_LIBRARY_DIRS})

include_directories(${GTKMM_INCLUDE_DIRS})

add_library(eqwin  STATIC
		mainwidget.cpp bandctl.cpp  eqparams.cpp  knob.cpp knob2.cpp 
		vuwidget.cpp eqwindow.cpp setwidgetcolors.cpp
		bodeplot.cpp button.cpp toggle_button.cpp abbutton.cpp sidechainbox.cpp)

target_link_libraries( eqwin ${GTKMM_LIBRARIES} m)
