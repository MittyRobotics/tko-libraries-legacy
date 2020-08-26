cmake_minimum_required(VERSION 3.14)
project(cpp_util)

set(CMAKE_USE_RELATIVE_PATHS  false)

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(CMAKE_C_COMPILER frc/bin/arm-frc-linux-gnueabi-gcc)

set(CMAKE_CXX_STANDARD 14)
#add_compile_options(-bigobj)

add_library(tko-libraries-cpp-util SHARED library.cpp drake/math/discrete_algebraic_riccati_equation.cpp drake/math/discrete_algebraic_riccati_equation.h drake/common/drake_assert.h drake/common/drake_throw.h drake/common/drake_assert_and_throw.cpp)