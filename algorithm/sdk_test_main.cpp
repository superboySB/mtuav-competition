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

using namespace mtuav::algorithm;
using namespace mtuav;

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

// 生成新的XML字符串（只是一个示例，你可能需要根据自己的需要进行修改）
std::string GenerateNewXML(int width, int height, const std::vector<std::vector<int>>& grid) {
    std::ostringstream xmlStream;
    xmlStream << "<grid>\n";
    xmlStream << "  <width>" << width << "</width>\n";
    xmlStream << "  <height>" << height << "</height>\n";
    xmlStream << "  <data>\n";
    for (const auto& row : grid) {
        for (const auto& cell : row) {
            xmlStream << cell << " ";
        }
        xmlStream << "\n";
    }
    xmlStream << "  </data>\n";
    xmlStream << "</grid>\n";
    return xmlStream.str();
}

void SaveXMLToFile(const std::string& xmlContent, std::string mode, std::string drone_id) {
    std::stringstream filePathStream;
    filePathStream << "/workspace/mtuav-competition/params/"<< mode<<"-" << drone_id << ".xml";
    std::string filePath = filePathStream.str();

    std::ofstream outFile(filePath);
    if (outFile.is_open()) {
        outFile << xmlContent;
        outFile.close();
        std::cout << "XML has been successfully saved to " << filePath << std::endl;
    } else {
        std::cout << "Unable to open file for writing: " << filePath << std::endl;
    }
}

void initialize_my_drone_info(std::unordered_map<std::string, MyDroneInfo>& my_drone_info, 
        std::shared_ptr<Map> map, float map_min_x, float map_max_x, float map_min_y, 
        float map_max_y, float map_min_z, float map_max_z) {
    my_drone_info["drone-001"].flying_height = 70;
    my_drone_info["drone-002"].flying_height = 80;
    my_drone_info["drone-003"].flying_height = 90;
    my_drone_info["drone-004"].flying_height = 100;
    my_drone_info["drone-005"].flying_height = 110;
    my_drone_info["drone-006"].flying_height = 120;

    // Iterate through the map to get all the keys
    for (const auto& pair : my_drone_info) {
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

         // 计算新的width和height
        int new_width = grid[0].size();
        int new_height = grid.size();

        // 使用新的width、height和grid生成新的XML
        
        std::string newXML = GenerateNewXML(new_width, new_height, grid);
        std::string mode = "map";
        SaveXMLToFile(newXML, mode, pair.first);
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

    // std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    int task_num = planner->GetTaskCount();
    LOG(INFO) << "Task num: " << task_num;
    // TODO 选手指定比赛任务索引
    int task_idx = 0;
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

    // TODO 选手需要按照自己的设计，声明算法类
    std::shared_ptr<myAlgorithm> alg = std::make_shared<myAlgorithm>();
    // 1023: 自己维护一个恶心心的状态机
    map->Range(&alg->map_min_x, &alg->map_max_x, 
                &alg->map_min_y, &alg->map_max_y, 
                &alg->map_min_z, &alg->map_max_z);
    initialize_my_drone_info(alg->my_drone_info, map, alg->map_min_x, alg->map_max_x, alg->map_min_y, 
                alg->map_max_y, alg->map_min_z, alg->map_max_z);
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
        // 调用算法求解函数，solve函数内内部输出飞行计划,返回值为下次调用算法求解间隔（毫秒）
        int64_t sleep_time_ms = alg->solve();
        LOG(INFO) << "Algorithm calculation completed, the next call interval is " << sleep_time_ms
                  << " ms.";
        // 选手可自行控制算法的调用间隔
        std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time_ms));
    }

    sleep(1);
    planner->StopTask();
    google::ShutdownGoogleLogging();
    return 0;
}