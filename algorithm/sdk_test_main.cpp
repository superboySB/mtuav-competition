#include <glog/logging.h>
#include <signal.h>
#include <unistd.h>
#include <chrono>
#include <iostream>
#include <thread>
#include "algorithm.h"
#include "current_game_info.h"
#include "mtuav_sdk.h"
#include "planner.h"
#include <fstream>
#include <sstream>
#include "Polylidar/Polylidar.hpp"

using namespace mtuav::algorithm;
using namespace mtuav;

typedef struct Vec2 {
  double x;
  double y;
} Vec2;

// 初始化算法类静态成员变量
int64_t Algorithm::flightplan_num = 0;
bool task_stop = false;

void sigint_handler(int sig) {
    if (sig == SIGINT) {
        // ctrl+c退出时执行的代码
        std::cout << "ctrl+c pressed!" << std::endl;
        task_stop = true;
    }
}

// 计算向量叉积
double cross(const Vec2& O, const Vec2& A, const Vec2& B) {
  return (A.x - O.x) * (B.y - O.y) - (A.y - O.y) * (B.x - O.x);
}

// 初始化+预计算
void initialize_my_drone_info(std::unordered_map<std::string, MyDroneInfo>& my_drone_info, 
        std::shared_ptr<Map> map, float map_min_x, float map_max_x, float map_min_y, 
        float map_max_y, float map_min_z, float map_max_z, std::vector<std::string>& unused_drone_id) {
    
    // 方案1：当前方案 (垃圾玩意，偶尔还炸)
    for (int i = 4; i <= 25; ++i) {
        std::ostringstream os;
        os << "drone-" << std::setfill('0') << std::setw(3) << i;
        unused_drone_id.push_back(os.str());
    }
    my_drone_info["drone-001"].flying_height = 120;
    my_drone_info["drone-002"].flying_height = 110;
    my_drone_info["drone-003"].flying_height = 100;

    my_drone_info["drone-001"].init_start_from_station_index = 0;
    my_drone_info["drone-002"].init_start_from_station_index = 4;
    my_drone_info["drone-003"].init_start_from_station_index = 7;
    

    // 方案2：适中方案（空域均分，低空太慢）
    // for (int i = 7; i <= 25; ++i) {
    //     std::ostringstream os;
    //     os << "drone-" << std::setfill('0') << std::setw(3) << i;
    //     unused_drone_id.push_back(os.str());
    // }
    // my_drone_info["drone-001"].flying_height = 120;
    // my_drone_info["drone-002"].flying_height = 110;
    // my_drone_info["drone-003"].flying_height = 100;
    // my_drone_info["drone-004"].flying_height = 90;
    // my_drone_info["drone-005"].flying_height = 80;
    // my_drone_info["drone-006"].flying_height = 70;

    // my_drone_info["drone-001"].init_start_from_station_index = 0;
    // my_drone_info["drone-002"].init_start_from_station_index = 4;
    // my_drone_info["drone-003"].init_start_from_station_index = 7;
    // my_drone_info["drone-004"].init_start_from_station_index = 1;
    // my_drone_info["drone-005"].init_start_from_station_index = 2;
    // my_drone_info["drone-006"].init_start_from_station_index = 3;

    // Iterate through the map to get all the keys
    for (auto& pair : my_drone_info) {
        // 第一步：利用Query提取grid map
        int step = 1; // assuming each cell represents a 1m x 1m area
        float z = my_drone_info[pair.first].flying_height; // height at which the 2D map is generated

        std::vector<std::vector<int>> grid(static_cast<int>((map_max_y - map_min_y) / step), 
                    std::vector<int>(static_cast<int>((map_max_x - map_min_x) / step), 0));
        
        for (float x = map_min_x; x <= map_max_x; x += step) {
            for (float y = map_min_y; y <= map_max_y; y += step) {
                const mtuav::Voxel* voxel = map->Query(x, y, z);
                if (voxel) {
                    int ix = (x - map_min_x) / step;
                    int iy = (y - map_min_y) / step;
                    // Use the distance value to set the grid cell value
                    if (voxel->distance <= 4) {
                        grid[iy][ix] = 1;
                    } else {
                        grid[iy][ix] = 0;
                    }
                }
            }
        }
        my_drone_info[pair.first].static_grid = grid;
        
        // 第二步：从grid map里提取边界点
        int grid_width = grid[0].size();
        int grid_height = grid.size();
        std::vector<Vec2> points_data_raw;
        std::vector<double> points_data_polylidar;
        for (int i=0; i<grid_height; i++){
            for (int j=0; j<grid_width; j++){
                if (grid[i][j]){
                    double point_x = static_cast<double>(j);
                    points_data_polylidar.push_back(point_x);
                    double point_y = static_cast<double>(i);
                    points_data_polylidar.push_back(point_y);
                    points_data_raw.push_back({j,i});
                }
            }
        }

        std::ostringstream json_stream;
        json_stream << R"json({
            "canvas": {"w": )json" << grid_width << R"json(, "h": )json" << grid_height << R"json(},
            "polygons": [
                [
                {"x": 0, "y": 0},
                {"x": )json" << grid_width << R"json(, "y": 0},
                {"x": )json" << grid_width << R"json(, "y": )json" << grid_height << R"json(},
                {"x": 0, "y": )json" << grid_height << R"json(}
                ],
        )json";

        std::vector<std::size_t> shape = {points_data_polylidar.size() / 2, 2};
        Polylidar::Matrix<double> points(points_data_polylidar.data(), shape[0], shape[1]);
        Polylidar::Polylidar3D pl(0.0, 2.0, 1, 3);
        Polylidar::MeshHelper::HalfEdgeTriangulation mesh;
        Polylidar::Planes planes;
        Polylidar::Polygons polygons;
        std:tie(mesh, planes, polygons) = pl.ExtractPlanesAndPolygons(points);
        for (int i=0; i< polygons.size(); i++)
        {
            // 第2.1步：稠密边界点
            // 创建一个新的Vec3 vector来存储提取的点
            std::vector<Vec2> extracted_points;
            extracted_points.reserve(polygons[i].shell.size()); // 优化，避免多次重新分配内存
            // 从后往前遍历索引数组
            for (auto it = polygons[i].shell.rbegin(); it != polygons[i].shell.rend(); ++it) {
                // 根据索引提取点并添加到新的vector中
                extracted_points.push_back(points_data_raw[*it]);
            }

            // 第2.2步：稀疏边界点
            std::vector<Vec2> poly_boundary_points;
            const double eps = 1e-10; // 可以调整精度
            for (int j = 0; j < extracted_points.size(); ++j) {
                // 计算当前点、前一个点和后一个点
                const Vec2& curr = extracted_points[j];
                const Vec2& prev = extracted_points[j == 0 ? extracted_points.size() - 1 : j - 1];
                const Vec2& next = extracted_points[(j + 1) % extracted_points.size()];
                // 如果叉积不为零，意味着有一个拐点
                if (std::abs(cross(prev, curr, next)) > eps) {
                    poly_boundary_points.push_back(curr);
                }
            }

            // 第2.3步：输出这一块的部分json段
            json_stream << "[";
            for (size_t j = 0; j < poly_boundary_points.size(); ++j) {
                json_stream << "{\"x\": " << poly_boundary_points[j].x
                            << ", \"y\": " << poly_boundary_points[j].y << "}";
                if (j < poly_boundary_points.size() - 1) {
                    json_stream << ",";
                }
            }
            json_stream << "]";

            if (i < polygons.size() - 1) {
                json_stream << ",";
            }
        }
        json_stream << R"json(]})json";
        std::string floorPlan = json_stream.str();
        pair.second.map_json = floorPlan;

        // 用于调试json形式
        std::ostringstream filename_stream;
        filename_stream << "/workspace/mtuav-competition/log/map-" << pair.first << ".json";
        std::string filename = filename_stream.str();
        std::ofstream file_out(filename);
        file_out << json_stream.str();
        file_out.close();
    }
}

int main(int argc, const char* argv[]) {
    signal(SIGINT, sigint_handler);
    FLAGS_alsologtostderr = true;   //除了日志文件之外是否需要标准输出
    FLAGS_colorlogtostderr = true;  //标准输出带颜色
    FLAGS_logbufsecs = 0;           //设置可以缓冲日志的最大秒数，0指实时输出
    FLAGS_max_log_size = 100;       //日志文件大小(单位：MB)
    FLAGS_stop_logging_if_full_disk = true;  //磁盘满时是否记录到磁盘
    google::InitGoogleLogging("uav_champ_example");
    // 配置本地log路径
    google::SetLogDestination(google::GLOG_INFO,
                              "/workspace/mtuav-competition/log/uav_champ_example_");

    // 配置本地路径读取地图信息
    // 用于单机版镜像
    auto map = mtuav::Map::CreateMapFromFile(
        "/workspace/mtuav-competition/map/test_map.bin");
    // 用于在线比赛系统
    // auto map = mtuav::Map::CreateMapFromFile(
    //     "/workspace/mtuav-competition/map/competition_map.bin");
    
    // 声明一个planner指针
    std::shared_ptr<Planner> planner = std::make_shared<Planner>(map);
    // LOG 打印是否成功读取地图
    if (map == nullptr) {
        LOG(INFO) << "Read map failed. ";
        return -1;
    } else {
        LOG(INFO) << "Read map successfully.";
    }

    // 下面使用测试账号仅用于登录单机版镜像
    mtuav::Response r =
        planner->Login("801f0ff5-5359-4c3e-99d4-f05d7eb47423", "e57aab02cf1f7433d7bf385748376164");
    // 下面使用测试账号仅用于登录在线比赛系统
    // mtuav::Response r =
    //     planner->Login("b87560ef-1f81-4545-948e-2b445544eb83", "aa855fbc9c433422d69584581d4a69c4");

    if (r.success == false) {
        LOG(INFO) << "Login failed, msg: " << r.msg;
        return -1;
    } else {
        LOG(INFO) << "Login successfully";
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    int task_num = planner->GetTaskCount();
    LOG(INFO) << "Task num: " << task_num;
    // TODO 选手指定比赛任务索引
    // Caution!!!
    // 通过sdk获取到3个任务，第一个和第二个任务为测试任务(获取到的任务列表索引为0和1)，可以无限次执行；
    // 第三个任务为正式比赛任务(获取到的任务列表索引为2)，限制为最多执行5次，最终结果为5次之中最好的成绩。
    int task_idx = 0;  //////// 慎重哦！
    // 获取比赛任务指针
    auto task = planner->QueryTask(task_idx);
    if (task == nullptr) {
        LOG(INFO) << "QueryTask failed., task index: " << task_idx;
        return -1;
    } else {
        LOG(INFO) << "QueryTask successfully, task index: " << task_idx
                  << ", task id: " << task->task_id;
    }

    // 声明比赛动态信息获取类（用于获取无人机实时状态，订单实时状态）
    std::shared_ptr<DynamicGameInfo> dynamic_info = DynamicGameInfo::getDynamicGameInfoPtr();
    // 设置任务结束标识符为false
    dynamic_info->set_task_stop_flag(false);
    LOG(INFO) << "An instance of class DynamicGameInfo is created. task stop flag: "
              << std::boolalpha << dynamic_info->get_task_stop_flag();

     // 【计时】记录开始时间点
    auto start_time = std::chrono::high_resolution_clock::now();
    // TODO 选手需要按照自己的设计，声明算法类
    std::shared_ptr<myAlgorithm> alg = std::make_shared<myAlgorithm>();
    // 1023: 自己维护一个恶心心的状态机
    map->Range(&alg->map_min_x, &alg->map_max_x, 
                &alg->map_min_y, &alg->map_max_y, 
                &alg->map_min_z, &alg->map_max_z);
    initialize_my_drone_info(alg->my_drone_info, map, alg->map_min_x, alg->map_max_x, alg->map_min_y, 
                alg->map_max_y, alg->map_min_z, alg->map_max_z, alg->unused_drone_id);
    // 将地图指针传入算法实例
    alg->set_map_info(map);
    // 将任务指针传入算法实例
    alg->set_task_info(std::move(task));

    // 将planner指针传入算法实例
    alg->set_planner(planner);
    LOG(INFO) << "An instance of contestant's algorihtm class is created. ";

    // 启动对应的比赛任务
    auto r2 = planner->StartTask(task_idx);
    if (r2.success == false) {
        LOG(INFO) << "Start task failed, msg: " << r2.msg;
        return -1;
    } else {
        LOG(INFO) << "Start task successfully, task index: " << task_idx;
    }
    // 记录结束时间点
    auto stop_time = std::chrono::high_resolution_clock::now();
    // 计算所经历的时间
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop_time - start_time);
    LOG(INFO) << "Initial procress consumes " << duration.count() << " ms";

    bool init_flag = false;
    while (!dynamic_info->get_task_stop_flag()) {
        if (task_stop == true) {
            planner->StopTask();
            LOG(INFO) << " Stop task by ctrl+c ";
            break;
        }

        LOG(INFO) << "Soving the problem using the the algorithm designed by contestants. ";
        // 调用算法类求解前，先更获取最新的动态信息
        alg->update_dynamic_info();
        LOG(INFO) << "The latest dynamic info has been fetched. ";
        
        if (!init_flag){
            auto& battery_station_positions = alg->_task_info->battery_stations;
            for (std::size_t i = 0; i < battery_station_positions.size(); ++i) {
                const Vec3& station = battery_station_positions[i];
                LOG(INFO) << "battery_stations " << i << ": x = " << station.x 
                        << ", y = " << station.y << ", z = " << station.z;
            }

            auto& landing_positions = alg->_task_info->landing_positions;
            for (std::size_t i = 0; i < landing_positions.size(); ++i) {
                const Vec3& station = landing_positions[i];
                LOG(INFO) << "landing_stations " << i << ": x = " << station.x 
                        << ", y = " << station.y << ", z = " << station.z;
            }
            init_flag = true;
        }
        
        // [核心]
        // 调用算法求解函数，solve函数内内部输出飞行计划,返回值为下次调用算法求解间隔（毫秒）
        int64_t sleep_time_ms = alg->solve();
        // [核心]

        LOG(INFO) << "Algorithm calculation completed, the next call interval is " << sleep_time_ms << " ms.";
        // 选手可自行控制算法的调用间隔
        std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time_ms));
    }

    sleep(1);
    planner->StopTask();
    google::ShutdownGoogleLogging();
    return 0;
}