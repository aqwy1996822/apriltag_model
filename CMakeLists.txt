cmake_minimum_required(VERSION 3.21)
project(apriltag_model)

set(CMAKE_CXX_STANDARD 11)

find_package(Threads REQUIRED)


find_package(OpenCV 3.4 REQUIRED)
message(STATUS "OpenCV Version: ${OpenCV_VERSION}")
message(STATUS "OpenCV_LIBS: ${OpenCV_LIBS}")
include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(include)

file(GLOB SRC "src/apriltag/*" "src/apriltag/common/*")
add_executable(ariltag_model main.cpp ${SRC})
target_link_libraries(ariltag_model ${OpenCV_LIBS})
target_link_libraries(ariltag_model Threads::Threads m)