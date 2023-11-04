cmake_minimum_required(VERSION 3.8)
project("mtuav-competition")

# 设置 C++17 标准并启用其他标志
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fthreadsafe-statics -fvisibility-inlines-hidden")

# 确保在调试版本中生成了完整的调试信息
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g")
set(CMAKE_BUILD_TYPE Debug)

# 依赖包
find_package(OpenMP REQUIRED)
if(OpenMP_CXX_FOUND)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# 添加外部依赖
add_subdirectory("thirdparty/polylidar")

# 添加include路径
include_directories(/usr/local/include/ortools/)
include_directories(algorithm/include/)
include_directories(path_finding/include/)
include_directories(api/)
link_directories(libs/)


# 美团主文件
set(TEST_MAIN_SRC algorithm/sdk_test_main.cpp
    ${PROJECT_SOURCE_DIR}/algorithm/src/algorithm.cpp
    ${PROJECT_SOURCE_DIR}/algorithm/src/current_game_info.cpp
    # path_finding/src/tinyxml2.cpp
)
add_executable(mtuav_sdk_example ${TEST_MAIN_SRC})
target_link_libraries(mtuav_sdk_example 
            mtuav_sdk
            Polylidar
            -lpathfindwrapper 
            -lpthread 
            -lglog)


# ------------------------------------------------------------------------------------------
# 添加CGO的单独测试路径
set(TEST_GO_SRC tests/test_golang.cpp)
add_executable(test_go ${TEST_GO_SRC})
target_link_libraries(test_go -lpathfindwrapper -lpthread -lglog)

# 添加SIPP的单独测试路径
set(SIPP_SRC
    path_finding/src/main.cpp  
    path_finding/src/tinyxml2.cpp
    path_finding/src/xmlLogger.cpp 
    path_finding/src/mission.cpp 
    path_finding/src/map.cpp 
    path_finding/src/task.cpp
    path_finding/src/config.cpp 
    path_finding/src/dynamicobstacles.cpp
    path_finding/src/aa_sipp.cpp
    path_finding/src/constraints.cpp)
add_executable(path_finding ${SIPP_SRC})
target_link_libraries(path_finding -lpthread -lglog)

# 添加ortools的单独测试路径
set(TEST_ORTOOLS_SRC tests/test_ortools.cpp)
add_executable(test_ortools ${TEST_ORTOOLS_SRC})
target_link_libraries(test_ortools ortools -lpthread -lglog)