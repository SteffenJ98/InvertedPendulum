cmake_minimum_required(VERSION 3.0.0)
project(InvertedPendulum VERSION 0.1.0 LANGUAGES C CXX)

add_subdirectory(include)
add_subdirectory(src)

find_library(WIRINGPI_LIBRARIES NAMES wiringPi)
add_executable(InvertedPendulum src/main.cpp )

target_link_libraries(InvertedPendulum ${WIRINGPI_LIBRARIES})
target_link_libraries(InvertedPendulum controller_lib)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

target_link_libraries(${PROJECT_NAME} 
  i2c MPU6050
)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)