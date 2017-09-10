cmake_minimum_required(VERSION 2.8)

project (lsm9ds1)
set(CMAKE_BUILD_TYPE Release)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)
add_library(lsm9ds1 SHARED src/SparkFunLSM9DS1.cpp)

add_executable(lsm_test src/lsmtest.cpp)
TARGET_LINK_LIBRARIES(lsm_test lsm9ds1)
