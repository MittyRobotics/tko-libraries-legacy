cmake_minimum_required(VERSION 3.14)
project(cpp_util)

set(CMAKE_CXX_STANDARD 14)
add_compile_options(-bigobj)

add_library(tko-libraries-cpp-util SHARED library.cpp drake/discrete_algebraic_riccati_equation.cc drake/discrete_algebraic_riccati_equation.h)