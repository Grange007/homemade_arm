cmake_minimum_required(VERSION 2.8.2)
project(UnitreeMotorSDK_M80106)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

include_directories(include)
link_directories(lib)

set(EXTRA_LIBS libUnitreeMotorSDK_M80106_Linux64.so)

add_executable(start example/start.cpp)
target_link_libraries(start ${EXTRA_LIBS})
add_executable(stop example/stop.cpp)
target_link_libraries(stop ${EXTRA_LIBS})
