cmake_minimum_required(VERSION 3.8)
project("mtuav-competition")

if(CMAKE_COMPILER_IS_GNUCXX)
set(CMAKE_CXX_FLAGS "-Ofast -fopenmp -g ${CMAKE_C_FLAGS} -std=c++17\
    -fthreadsafe-statics -fvisibility-inlines-hidden")
set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif(CMAKE_COMPILER_IS_GNUCXX)

# 依赖包
find_package(eigen-quadprog)
find_package(yaml-cpp REQUIRED)

# 添加API路径
include_directories(api/)

#添加算法头文件路径
include_directories(algorithm/include/)

#添加算法源文件路径
file(GLOB DIR_SRCS_MAIN "${PROJECT_SOURCE_DIR}/algorithm/*.cpp")
file(GLOB DIR_SRCS_ALG "${PROJECT_SOURCE_DIR}/algorithm/src/*.cpp")

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

# 美团主文件
add_executable(mtuav_sdk_example ${DIR_SRCS_MAIN} ${DIR_SRCS_ALG})
target_link_libraries(mtuav_sdk_example ${DIR_SKD_LIBS} -lpthread -lglog)


# ------------------------------------------------------------------------------------------
# 测试专用
# 添加test_map.cpp的路径
set(TEST_MAP_SRC tests/test_map.cpp)
add_executable(test_map ${TEST_MAP_SRC})
target_link_libraries(test_map ${DIR_SKD_LIBS} -lpthread -lglog)

# 添加am算法的单独测试路径

# set(CMAKE_CXX_FLAGS "-Ofast -fopenmp -g ${CMAKE_CXX_FLAGS}")
# set( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} )
# link_libraries(stdc++fs)
set(TEST_AM_SRC tests/test_am.cpp)
# add_library(lib_am_swarm ${PROJECT_SOURCE_DIR}/algorithm/src/optim_am_swarm.cpp ${PROJECT_SOURCE_DIR}/algorithm/src/sim_am_swarm.cpp)
add_executable(test_am ${TEST_AM_SRC})
target_link_libraries(test_am eigen-quadprog yaml-cpp -lpthread -lglog)
# ------------------------------------------------------------------------------------------

# 美团库文件安装到指定的位置
install(DIRECTORY libs/ DESTINATION /usr/lib)

