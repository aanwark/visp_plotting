cmake_minimum_required(VERSION 2.6)

project (visual-servo-ibvs-plotter)

find_package(VISP REQUIRED visp_core visp_robot visp_vs visp_blob visp_vision visp_io visp_gui visp_ar)

include_directories("/usr/local/include/visp3")

set(SRC_LIST plot_wireframe.cpp vpCustomFeature.cpp)


get_filename_component(target plot_wireframe.cpp NAME_WE)
include_directories(${VISP_INCLUDE_DIRS})
add_executable(${target} ${SRC_LIST})
target_link_libraries(${target} ${VISP_LIBRARIES})
