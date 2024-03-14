cmake_minimum_required(VERSION 2.8.4)
project(TauLidar)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(OpenCV REQUIRED)
include_directories( ${OPENCV_INCLUDE_DIRS} )

include_directories(/usr/local/lib)
find_library(WIRING_LIB wiringPi)
#find_library(raspicam REQUIRED)
set(SOURCE_FILES src/TauCam.cpp main.cpp)

add_executable(taucam ${SOURCE_FILES} src/TauCam.h)
target_link_libraries(taucam ${WIRING_LIB} ${OpenCV_LIBS})
