#include <glog/logging.h>
#include "mtuav_sdk.h"

int main() {
    // 初始化glog
    google::InitGoogleLogging("test_map");
    google::SetLogDestination(google::GLOG_INFO, "/workspace/mtuav-competition/log/test_map.log");
    FLAGS_alsologtostderr = false;  // 只输出到文件，不输出到stderr

    // 加载地图
    auto map = mtuav::Map::CreateMapFromFile("/workspace/mtuav-competition/map/test_map.bin");

    if (map == nullptr) {
        LOG(ERROR) << "Failed to load the map.";
        return -1;
    } else {
        LOG(INFO) << "Successfully loaded the map.";
    }

    // 获取地图的范围
    float min_x, max_x, min_y, max_y, min_z, max_z;
    map->Range(&min_x, &max_x, &min_y, &max_y, &min_z, &max_z);

    LOG(INFO) << "Map Range: x[" << min_x << ", " << max_x << "], "
              << "y[" << min_y << ", " << max_y << "], "
              << "z[" << min_z << ", " << max_z << "]";

    // 查询并打印所有位置的体素信息
    for (float x = min_x; x <= max_x; x += 10) {
        for (float y = min_y; y <= max_y; y += 10) {
            for (float z = min_z; z <= max_z; z += 10) {
                const mtuav::Voxel* voxel = map->Query(x, y, z);
                if (voxel != nullptr) {
                    LOG(INFO) << "Voxel at (" << x << ", " << y << ", " << z << "): "
                              << "Distance: " << voxel->distance
                              << ", Semantic: " << static_cast<int>(voxel->semantic);
                }
            }
        }
    }

    google::ShutdownGoogleLogging();
    return 0;
}