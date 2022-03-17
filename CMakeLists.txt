# 指定CMake最低版本要求
cmake_minimum_required(VERSION 3.21)

# 设置指定的C++编译器版本是必须的，如果不设置，或者为OFF，则指定版本不可用时，会使用上一版本
set(CMAKE_CXX_STANDARD_REQUIRED OFF)

message(STATUS ${CMAKE_CXX_STANDARD_REQUIRED})

# 设置项目名称以及版本号
project(Tutorial VERSION 1.0)

# 指定C++版本,定义CMAKE_CXX-STANDARD变量,cmake执行时会用这个变量，指定为C++14，而不是C++11
set(CMAKE_CXX_STANDARD 14)

# 编译类型debug，而非release
# set(CMAKE_BUILD_TYPE Release)
set(CMAKE_BUILD_TYPE Debug)

#set(CMAKE_CXX_FLAGS_DEBUG “${CMAKE_CXX_FLAGS_DEBUG} -O2”)
#set(CMAKE_CXX_FLAGS “${CMAKE_CXX_FLAGS} -O2”)

# 设置C++编译选项
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

message(STATUS "setting Eigen directory start")
message(STATUS ${CMAKE_SYSTEM_PROCESSOR})
message(STATUS ${CMAKE_SYSTEM})
message(STATUS ${CMAKE_SYSTEM_NAME})
message(STATUS ${CMAKE_SYSTEM_VERSION})
message(STATUS ${PROJECT_SOURCE_DIR})
message(STATUS ${PROJECT_BINARY_DIR})
message(STATUS ${CMAKE_INCLUDE_PATH})
message(STATUS ${CMAKE_LIBRARY_PATH})
message(STATUS ${CMAKE_SYSTEM})
message(STATUS ${CMAKE_CURRENT_SOURCE_DIR})
message(STATUS ${CMAKE_CURRENT_BINARY_DIR})
message(STATUS ${CMAKE_SYSTEM})
message(STATUS ${CMAKE_CURRENT_LIST_FILE})
message(STATUS ${CMAKE_CURRENT_LIST_LINE})

message(STATUS ${LIBRARY_OUTPUT_PATH})
message(STATUS ${EXECUTABLE_OUTPUT_PATH})

# 测试，没用到
# find_path(myEigen NAMES R2EADME.md PATHS /untitled/eigen-3.4.0 "HAHA")

# 设置Eigen安装路径
#include_directories("C:/Users/uuxmas/CLionProjects/untitled/eigen-3.4.0")
include_directories("C:/eigen-3.4.0")

# 遍历当前路径下的所有源文件，并将其添加到变量DIR_SRCS中
aux_source_directory(. DIR_SRCS)
message("${DIR_SRCS}")

message(STATUS ${myEigen})
message(STATUS "setting Eigen directory finished")

include_directories(BEFORE .)
get_property(dirs DIRECTORY ${CMAKE_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
message(STATUS ">> === include_dirs = ${dirs}")
# find_package(Eigen3 REQUIRED)
message(STATUS ">> === find_package = ${Eigen3}")
# 编译源码生成目标（生成执行程序）
# add_executable(demo10 main.cpp)
# 添加名为demo10的可执行文件，该文件会由变量DIR_SRCS的源文件构建生成
add_executable(demo12 ${DIR_SRCS})