cmake_minimum_required(VERSION 3.8)

project("mtuav-competition")

if(CMAKE_COMPILER_IS_GNUCXX)
set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -std=c++17\
    -fthreadsafe-statics -fvisibility-inlines-hidden")
endif(CMAKE_COMPILER_IS_GNUCXX)
include_directories(api/)

#添加算法头文件路径
include_directories(example/algorithm/include/)
#添加算法源文件路径
file(GLOB DIR_SRCS_MAIN "${PROJECT_SOURCE_DIR}/example/*.cpp")
file(GLOB DIR_SRCS_ALG "${PROJECT_SOURCE_DIR}/example/algorithm/src/*.cpp")
#打印源文件路径
message(STATUS ${DIR_SRCS_MAIN})
message(STATUS  ${DIR_SRCS_ALG})

#打印include路径
get_property( dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES )
foreach(dir ${dirs} )
    message( STATUS "include dir=${dir}" )
endforeach()

#LINK_DIRECTORIES(libs/)
# 需要链接libs目录下的so文件
file(GLOB DIR_SKD_LIBS "${PROJECT_SOURCE_DIR}/libs/*.so")
message(STATUS ${DIR_SKD_LIBS})
execute_process(COMMAND export LD_LIBRARY_PATH="${PROJECT_SOURCE_DIR}/libs/":$LD_LIBRARY_PATH)

add_executable(mtuav_sdk_example ${DIR_SRCS_MAIN} ${DIR_SRCS_ALG})
target_link_libraries(mtuav_sdk_example  ${DIR_SKD_LIBS} -lpthread -lglog)
# 库文件安装到指定的位置
install(DIRECTORY libs/ DESTINATION /usr/lib)
