# A project that uses an installation of the helm libraries
# (IVP_BUILD_HELM_LIBS_ONLY) through find_package(IVP), to check that they and
# the exported package work without MOOS. CI builds and runs it.
cmake_minimum_required(VERSION 3.11)
project(helm_libs_consumer CXX)

find_package(IVP CONFIG REQUIRED)

add_executable(helm_libs_consumer main.cpp)
target_compile_features(helm_libs_consumer PRIVATE cxx_std_17)
target_link_libraries(helm_libs_consumer PRIVATE
  IVP::helmivp IVP::geodaid IVP::ivpbuild IVP::ivpsolve IVP::ivpcore IVP::logic)
