project (visual-servo-ibvs-plotter)

cmake_minimum_required(VERSION 2.6)

find_package(VISP REQUIRED visp_core visp_robot visp_vs visp_blob visp_vision visp_io visp_gui visp_ar)

set (test_cpp test_plot.cpp)
set (wireframe plot_wireframe.cpp)

visp_add_target (${test_cpp})
visp_add_target (${wireframe})
