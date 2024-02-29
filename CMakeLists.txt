cmake_minimum_required(VERSION 2.8.4)
project(TauLidar)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")




set(SOURCE_FILES src/TauCam.cpp main.cpp)

add_executable(taucam ${SOURCE_FILES} src/TauCam.h)
