cmake_minimum_required(VERSION 2.8)
project(PathTracer CXX)
set (CMAKE_CXX_STANDARD 11)
# set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${ADDITIONAL_CFLAGS} -g")
# set(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_BUILD_TYPE "Debug")

set( CMAKE_CXX_FLAGS "-O3 -g -Wno-unused-command-line-argument" )
if(APPLE)
set(CMAKE_CXX_COMPILER "/usr/local/opt/llvm/bin/clang++")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp -L /usr/local/opt/libomp/lib")
endif()
message("CMAKE_CXX_FLAGS is ${CMAKE_CXX_FLAGS}")

# message("CMAKE_CXX_FLAGS_DEBUG is ${CMAKE_CXX_FLAGS_DEBUG}")
# message("CMAKE_CXX_FLAGS_RELEASE is ${CMAKE_CXX_FLAGS_RELEASE}")

include_directories(./include)
find_package(PkgConfig REQUIRED)
pkg_search_module(GLFW REQUIRED glfw3)
if(GLFW_FOUND)
    link_directories(${GLFW_LIBRARY_DIRS})
    include_directories(${GLFW_INCLUDE_DIRS})
endif()

find_package(OpenGL REQUIRED)
if(OPENGL_FOUND)
    link_directories(${OPENGL_LIBRARY_DIRS})
    include_directories(${OPENGL_INCLUDE_DIRS})
endif()

pkg_search_module(GLEW REQUIRED glew)
if(GLEW_FOUND)
    link_directories(${GLEW_LIBRARY_DIRS})
    include_directories(${GLEW_INCLUDE_DIRS})
endif()
set(GL_LIBS ${GLEW_LIBRARIES} ${GLFW_LIBRARIES} ${OPENGL_LIBRARIES})

set(LIBS ${GL_LIBS})

# compile
include_directories(./include)
include_directories(./src)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/../lib) 
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}) 

add_subdirectory(src)
set(main_src ./src/main.cpp)
add_executable(${PROJECT_NAME} ${main_src} )

target_link_libraries(${PROJECT_NAME} main ${LIBS})