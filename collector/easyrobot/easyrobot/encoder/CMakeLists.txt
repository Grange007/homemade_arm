cmake_minimum_required(VERSION 3.1.0)
project(UnitreeMotorSDK)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

include_directories(include)
link_directories(lib)

set(EXTRA_LIBS libUnitreeMotorSDK_M80106_Linux64.so)

add_subdirectory(pybind11)
pybind11_add_module(UnitreeMotorSDK src/wrapper.cpp)
target_link_libraries(UnitreeMotorSDK PRIVATE ${EXTRA_LIBS})
set_target_properties(UnitreeMotorSDK PROPERTIES LIBRARY_OUTPUT_DIRECTORY "../lib")