cmake_minimum_required(VERSION 3.15)
project(splines C)

set(CMAKE_C_STANDARD 11)
SET(CMAKE_FIND_LIBRARY_SUFFIXES ".a")
SET(BUILD_SHARED_LIBS OFF)

find_package(SDL2 REQUIRED)
include_directories(${PROJECT_NAME} ${SDL2_INCLUDE_DIRS})
include_directories(${PROJECT_NAME} "/usr/local/Cellar/sdl2/2.0.10/include/")

add_executable(splines main.c)
target_link_libraries(splines ${SDL2_LIBRARIES} )
