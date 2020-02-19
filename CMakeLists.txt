cmake_minimum_required(VERSION 3.15)
project(splines C)

set(CMAKE_C_STANDARD 11)
SET(CMAKE_FIND_LIBRARY_SUFFIXES ".a")
SET(BUILD_SHARED_LIBS OFF)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake")

find_package(SDL2 REQUIRED)
find_package(SDL2_TTF REQUIRED)
find_package(Freetype REQUIRED)

include_directories(${PROJECT_NAME} ${SDL2_INCLUDE_DIRS})
include_directories(${PROJECT_NAME} "/usr/local/Cellar/sdl2/2.0.10/include/")
include_directories(${PROJECT_NAME} "/usr/local/Cellar/sdl2_ttf/2.0.15/include/")

add_executable(splines main.c)
target_link_libraries(splines ${SDL2_LIBRARIES} ${SDL2_TTF_LIBRARY} ${FREETYPE_LIBRARIES} "/usr/local/opt/bzip2/lib/libbz2.a" "/usr/local/opt/zlib/lib/libz.a"  "/usr/local/opt/libpng/lib/libpng.a")
