cmake_minimum_required(VERSION 3.8)
project("mtuav-competition")

# 设置 C++17 标准并启用其他标志
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fthreadsafe-statics -fvisibility-inlines-hidden")

# 如果想要默认进行 Debug 编译，可以加入以下行：
# set(CMAKE_BUILD_TYPE Debug)

# 依赖包
find_package(OpenMP)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
find_package(eigen-quadprog)
find_package(nlohmann_json)

# 添加include路径
include_directories(algorithm/include/)
include_directories(/usr/local/include/ortools/)
include_directories(api/)
link_directories(libs/)

# 美团主文件
set(TEST_MAIN_SRC algorithm/sdk_test_main.cpp
    ${PROJECT_SOURCE_DIR}/algorithm/src/algorithm.cpp
    ${PROJECT_SOURCE_DIR}/algorithm/src/current_game_info.cpp
)
add_executable(mtuav_sdk_example ${TEST_MAIN_SRC})
target_link_libraries(mtuav_sdk_example mtuav_sdk -lpthread -lglog)

# ------------------------------------------------------------------------------------------
# 测试专用
# 添加test_map.cpp的路径
# set(TEST_MAP_SRC tests/test_map.cpp)
# add_executable(test_map ${TEST_MAP_SRC})
# target_link_libraries(test_map ${DIR_SKD_LIBS} -lglog)

# 添加am算法的单独测试路径
set(TEST_AM_SRC tests/test_am.cpp
    ${PROJECT_SOURCE_DIR}/algorithm/src/optim_am_swarm.cpp
    ${PROJECT_SOURCE_DIR}/algorithm/src/sim_am_swarm.cpp
)
add_executable(test_am ${TEST_AM_SRC})
target_link_libraries(test_am eigen-quadprog nlohmann_json -lpthread -lglog)


# 添加ortools的单独测试路径
set(TEST_ORTOOLS_SRC tests/test_ortools.cpp)
add_executable(test_ortools ${TEST_ORTOOLS_SRC})
target_link_libraries(test_ortools ortools -lpthread -lglog)