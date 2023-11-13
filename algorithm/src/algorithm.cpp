#include <glog/logging.h>
#include <cstdlib> // for system()
#include <algorithm>    // C++ STL 算法库
#include "algorithm.h"  // 选手自行设计的算法头文件
#include <cmath>
#include "math.h"
#include <fstream>
#include <sstream>
#include "Polylidar/Polylidar.hpp"


namespace mtuav::algorithm {
// 算法基类Algorithm函数实现

void Algorithm::update_dynamic_info() {
    auto dynamic_info = DynamicGameInfo::getDynamicGameInfoPtr();
    if (dynamic_info == nullptr) {
        return;
    } else {
        auto [drone_info, cargo_info] = dynamic_info->get_current_info();
        this->_drone_info = drone_info;
        this->_cargo_info = cargo_info;
    }
}

void Algorithm::update_drone_info(const drones_info& latest_drone_info) {
    this->_drone_info.clear();
    for (auto& drone : latest_drone_info) {
        this->_drone_info.push_back(drone);
    }
    return;
}

void Algorithm::update_cargo_info(const cargoes_info& latest_cargo_info) {
    this->_cargo_info.clear();
    for (auto& [id, cargo] : latest_cargo_info) {
        this->_cargo_info[id] = cargo;
    }
    return;
}

void Algorithm::set_task_info(std::unique_ptr<TaskInfo> input_task) {
    this->_task_info = std::move(input_task);
}

void Algorithm::set_map_info(std::shared_ptr<Map> input_map) { this->_map = input_map; }

void Algorithm::set_planner(std::shared_ptr<Planner> input_planner) {
    this->_planner = input_planner;
}

const double epsilon = 1e-4;

// 计算两个向量的点积
double dot(const Vec2& v1, const Vec2& v2) {
    return v1.x * v2.x + v1.y * v2.y;
}

// 计算向量的模长
double norm(const Vec2& v) {
    return std::sqrt(v.x * v.x + v.y * v.y);
}

bool operator==(const Vec3& a, const Vec3& b) {
    return std::fabs(a.x - b.x) < epsilon && std::fabs(a.y - b.y) < epsilon && std::fabs(a.z - b.z) <= 4.01;
}

bool operator!=(const Vec3& a, const Vec3& b) {
    return std::fabs(a.x - b.x) > epsilon || std::fabs(a.y - b.y) > epsilon || std::fabs(a.z - b.z) > epsilon;
}

bool black_2d_position_check(const Vec3& position, const Vec3& target_position, const double bx, const double by) {
    if (std::fabs(position.x - bx) < epsilon && std::fabs(position.y - by) < epsilon) return true;
    if (std::fabs(target_position.x - bx) < epsilon && std::fabs(target_position.y - by) < epsilon) return true;
    return false;
}

double roundUpToMultipleOf4(double num) {
    if (std::fmod(num, 4) < epsilon ){
        return num;
    }else{
        return std::ceil((num + 3.0) / 4.0) * 4.0;
    }
}

double distance_2D(Vec3 A, Vec3 B) {
    return std::sqrt(std::pow(B.x - A.x, 2) + std::pow(B.y - A.y, 2));
}

double distance_3D(Vec3 A, Vec3 B) {
    return std::sqrt(std::pow(B.x - A.x, 2) + std::pow(B.y - A.y, 2) + std::pow(B.z - A.z, 2));
}

// 计算两个向量的夹角（以度为单位）
double angleBetween(const Vec2& v1, const Vec2& v2) {
    double dot_product = dot(v1, v2);
    double norms_product = norm(v1) * norm(v2);
    double cos_angle = dot_product / norms_product;
    // 防止由于浮点数精度问题导致acos函数的输入超出[-1, 1]的范围
    cos_angle = std::max(-1.0, std::min(1.0, cos_angle));
    double angle_radians = std::acos(cos_angle);
    double angle_degrees = angle_radians * (180.0 / M_PI);
    return angle_degrees;
}

// 判断三个点是否几乎共线
bool areCollinear(const Vec2& p1, const Vec2& p2, const Vec2& p3, const double angle_eps) {
    Vec2 v1 = {p2.x - p1.x, p2.y - p1.y};
    Vec2 v2 = {p3.x - p1.x, p3.y - p1.y};
    double angle = angleBetween(v1, v2);
    return std::abs(angle) < angle_eps || std::abs(angle - 180.0) < angle_eps;
}

// 计算多边形质心（进一步清空一些没有用的多边形关键点）
Vec2 calculatePolygonCentroid(const std::vector<Vec2>& points) {
    if (points.size() < 3) {
        // 对于少于三个点的情况，无法定义多边形，返回错误或默认值
        return Vec2{0, 0};
    }
    double centroidX = 0.0, centroidY = 0.0;
    double signedArea = 0.0;
    double x0 = 0.0; // 当前顶点的X
    double y0 = 0.0; // 当前顶点的Y
    double x1 = 0.0; // 下一个顶点的X
    double y1 = 0.0; // 下一个顶点的Y
    double a = 0.0;  // 部分面积

    // 以第一个点为起点，循环计算每个三角形的质心和面积
    for (int i = 0; i < points.size(); ++i) {
        x0 = points[i].x;
        y0 = points[i].y;
        x1 = points[(i + 1) % points.size()].x;
        y1 = points[(i + 1) % points.size()].y;
        a = x0 * y1 - x1 * y0;
        signedArea += a;
        centroidX += (x0 + x1) * a;
        centroidY += (y0 + y1) * a;
    }

    signedArea *= 0.5;
    centroidX /= (6.0 * signedArea);
    centroidY /= (6.0 * signedArea);

    // 处理由于浮点数计算导致的负面积情况
    if (signedArea < 0) {
        centroidX = -centroidX;
        centroidY = -centroidY;
    }

    return Vec2{centroidX, centroidY};
}


// 初始化+预计算
void myAlgorithm::initialize_static_grid() {
    // 利用Query提取grid map
    for (auto& pair : my_drone_info) {
        int step = 1; // assuming each cell represents a 1m x 1m area
        float z = pair.second.flying_height; // height at which the 2D map is generated

        std::vector<std::vector<int>> grid(static_cast<int>((map_max_y - map_min_y) / step), 
                    std::vector<int>(static_cast<int>((map_max_x - map_min_x) / step), 0));
        
        for (float x = map_min_x; x <= map_max_x; x += step) {
            for (float y = map_min_y; y <= map_max_y; y += step) {
                const mtuav::Voxel* voxel = _map->Query(x, y, z);
                if (voxel) {
                    int ix = (x - map_min_x) / step;
                    int iy = (y - map_min_y) / step;
                    // Use the distance value to set the grid cell value
                    if (voxel->distance <= 5) {
                        grid[iy][ix] = 1;
                    } else {
                        grid[iy][ix] = 0;
                    }
                }
            }
        }
        pair.second.static_grid = grid;
        pair.second.dynamic_grid = grid;
    }
}

void myAlgorithm::add_takeoff_grid(std::string this_drone_id, double safer_distance, double current_height){
    MyDroneInfo mydrone = my_drone_info[this_drone_id];
    auto takeoff_position = mydrone.flightplan_takeoff_position;
    int step = 1; // assuming each cell represents a 1m x 1m area

    for (float x = takeoff_position.x - safer_distance; x <= takeoff_position.x + safer_distance; x += step) {
        for (float y = takeoff_position.y - safer_distance; y <= takeoff_position.y + safer_distance; y += step) {
            // 转换为动态网格索引
            int ix = (int)((x - map_min_x) / step);
            int iy = (int)((y - map_min_y) / step);

            // 确保索引在动态网格的有效范围内
            if (ix >= map_min_x && ix <= map_max_x && iy >= map_min_y && iy <= map_max_y) {
                for (auto& pair : my_drone_info) {
                    if (pair.first == this_drone_id) continue;
                    if (pair.second.flying_height + safer_distance <= current_height) continue;
                    float z = pair.second.flying_height; // height at which the 2D map is generated
                    // 计算当前点到起飞点的二维距离
                    Vec3 target = {x,y,z};
                    if ((distance_2D(takeoff_position, target) <= safer_distance)){
                        pair.second.dynamic_grid[iy][ix] = 1;
                    }
                }
                
            }
        }
    }
}

void myAlgorithm::add_landing_grid(std::string this_drone_id, double safer_distance, double current_height){
    MyDroneInfo mydrone = my_drone_info[this_drone_id];
    auto land_position = mydrone.flightplan_land_position;
    int step = 1; // assuming each cell represents a 1m x 1m area

    for (float x = land_position.x - safer_distance; x <= land_position.x + safer_distance; x += step) {
        for (float y = land_position.y - safer_distance; y <= land_position.y + safer_distance; y += step) {
            // 转换为动态网格索引
            int ix = (int)((x - map_min_x) / step);
            int iy = (int)((y - map_min_y) / step);

            // 确保索引在动态网格的有效范围内
            if (ix >= map_min_x && ix <= map_max_x && iy >= map_min_y && iy <= map_max_y) {
                for (auto& pair : my_drone_info) {
                    if (pair.first == this_drone_id) continue;
                    if (current_height + safer_distance <= pair.second.flying_height) continue;
                    float z = pair.second.flying_height; // height at which the 2D map is generated
                    // 计算当前点到起飞点的二维距离
                    Vec3 target = {x,y,z};
                    if ((distance_2D(land_position, target) <= safer_distance)){
                        pair.second.dynamic_grid[iy][ix] = 1;
                    }
                }
                
            }
        }
    }
}

// 1. Find the current segment the drone is on or closest to.
// Helper function to find the closest point on a line segment.
Vec3 get_closest_point_on_line_segment(const Vec3& p, const Vec3& start, const Vec3& end) {
    Vec3 line_vec = {end.x - start.x, end.y - start.y, end.z - start.z};
    Vec3 p_vec = {p.x - start.x, p.y - start.y, p.z - start.z};
    double dot = p_vec.x * line_vec.x + p_vec.y * line_vec.y + p_vec.z * line_vec.z;
    double length_sq = line_vec.x * line_vec.x + line_vec.y * line_vec.y + line_vec.z * line_vec.z;
    double param = dot / length_sq;

    if (param < 0 || (start.x == end.x && start.y == end.y)) {
        return start;
    } else if (param > 1) {
        return end;
    } else {
        return {start.x + param * line_vec.x, start.y + param * line_vec.y, start.z + param * line_vec.z};
    }
}

size_t find_current_segment(const Vec3& current_position, const std::vector<Vec3>& waypoints) {
    size_t closest_segment = 0;
    double min_dist = std::numeric_limits<double>::max();

    for (size_t i = 0; i < waypoints.size() - 1; ++i) {
        Vec3 start = waypoints[i];
        Vec3 end = waypoints[i + 1];
        Vec3 closest_point = get_closest_point_on_line_segment(current_position, start, end);
        double dist = distance_2D(current_position, closest_point);

        if (dist < min_dist) {
            min_dist = dist;
            closest_segment = i;
        }
    }

    return closest_segment;
}

// 2. Get all grid points on the path between two waypoints.
std::vector<Vec3> get_grid_points_on_path(const Vec3& start, const Vec3& end) {
    std::vector<Vec3> grid_points;

    // Simple Bresenham algorithm for 2D.
    int dx = abs(end.x - start.x), sx = start.x < end.x ? 1 : -1;
    int dy = -abs(end.y - start.y), sy = start.y < end.y ? 1 : -1;
    int err = dx + dy, e2;

    int x = start.x, y = start.y;
    while (true) {
        grid_points.push_back({static_cast<double>(x), static_cast<double>(y), start.z});
        if (x == end.x && y == end.y) break;
        e2 = 2 * err;
        if (e2 >= dy) { err += dy; x += sx; }
        if (e2 <= dx) { err += dx; y += sy; }
    }

    return grid_points;
}

void myAlgorithm::add_flying_grid(std::string this_drone_id, double safer_distance, double current_height) {
    MyDroneInfo mydrone = my_drone_info[this_drone_id];
    auto current_position = mydrone.drone_position;
    auto all_flying_waypoints = mydrone.all_flying_waypoints;
    int step = 1;

    // 找到无人机当前位置在航线上的最近点或段
    size_t current_segment = find_current_segment(current_position, all_flying_waypoints);

    // 从当前段开始，遍历所有航点
    for (size_t i = current_segment; i < all_flying_waypoints.size() - 1; ++i) {
        // 对于每个航点和下一个航点之间的线段
        auto start_point = all_flying_waypoints[i];
        auto end_point = all_flying_waypoints[i + 1];

        // 获取该线段上的所有网格点
        std::vector<Vec3> grid_points_on_path = get_grid_points_on_path(start_point, end_point);

        // 遍历这些网格点，标记禁飞区
        for (const Vec3& grid_point : grid_points_on_path) {
            int grid_min_x = std::max(static_cast<int>(map_min_x), static_cast<int>((grid_point.x - safer_distance) / step));
            int grid_max_x = std::min(static_cast<int>(map_max_x), static_cast<int>((grid_point.x + safer_distance) / step));
            int grid_min_y = std::max(static_cast<int>(map_min_y), static_cast<int>((grid_point.y - safer_distance) / step));
            int grid_max_y = std::min(static_cast<int>(map_max_y), static_cast<int>((grid_point.y + safer_distance) / step));

            for (float ix = grid_min_x; ix <= grid_max_x; ix+=step) {
                for (float iy = grid_min_y; iy <= grid_max_y; iy+=step) {
                    for (auto& pair : my_drone_info) {
                        if (pair.first == this_drone_id) continue;
                        if (pair.second.flying_height + safer_distance <= current_height) continue;
                        float z = pair.second.flying_height; // height at which the 2D map is generated
                        // 计算当前点到起飞点的二维距离
                        Vec3 target = {ix * step, iy * step, grid_point.z};
                        if (distance_2D(grid_point, target) <= safer_distance){
                            pair.second.dynamic_grid[iy][ix] = 1;
                        }
                    }
                }
            }
        }
    }
}


// 更新各层飞机临时动态体素地图，增加未来航线的临时体素，清空已飞部分的临时体素
void myAlgorithm::update_dynamic_grid(double safe_distance, double factor) {
    for (auto pair : my_drone_info) {
        auto current_position = pair.second.drone_position;
        auto takeoff_position = pair.second.flightplan_takeoff_position;
        auto land_position = pair.second.flightplan_land_position;
        if (takeoff_position.x < 0) continue;

        if ((std::fabs(current_position.x - takeoff_position.x) < epsilon) 
          && (std::fabs(current_position.y - takeoff_position.y) < epsilon)){
            // 当前位置处于起飞阶段
            add_takeoff_grid(pair.first,safe_distance*factor, current_position.z);
            add_flying_grid(pair.first,safe_distance*factor, pair.second.flying_height);
            add_landing_grid(pair.first,safe_distance*factor, pair.second.flying_height);
        } 
        else if ((std::fabs(current_position.x - land_position.x) < epsilon) 
          && (std::fabs(current_position.y - land_position.y) < epsilon)){
            // 当前位置处于降落阶段
            add_landing_grid(pair.first,safe_distance*factor, current_position.z);
        } else {
            // 当前位置处于平飞阶段
            add_flying_grid(pair.first,safe_distance*factor, current_position.z);
            add_landing_grid(pair.first,safe_distance*factor, pair.second.flying_height);
        }
    }
}

// 一方面用于进行路径规划，另一方面也可以用来判断可达性
std::vector<Vec3> generate_waypoints_by_a_star(Vec3 start, Vec3 end, MyDroneInfo& mydrone) {
    // 改为从dynamic grid map里提取边界点
    // std::vector<std::vector<int>> grid = mydrone.static_grid; 
    std::vector<std::vector<int>> grid = mydrone.dynamic_grid;

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
                points_data_raw.push_back({point_x,point_y});
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
        // 第1步：稠密边界点
        // 创建一个新的Vec3 vector来存储提取的点
        std::vector<Vec2> original_points;
        original_points.reserve(polygons[i].shell.size()); // 优化，避免多次重新分配内存
        // 从后往前遍历索引数组
        for (auto it = polygons[i].shell.rbegin(); it != polygons[i].shell.rend(); ++it) {
            // 根据索引提取点并添加到新的vector中
            original_points.push_back(points_data_raw[*it]);
        }

        // 第2步：稀疏边界点
        std::vector<Vec2> simplified_points;
        // simplified_points = original_points; // 不简化对比
        // const double min_edge_length = 4; // 调整相近的点
        const double angle_eps = 5; // 设定一个阈值，例如10度，可以根据需要调整
        // Vec2 center = calculatePolygonCentroid(original_points);
        for (int i = 0; i < original_points.size(); ++i) {
            const Vec2& prev = original_points[i == 0 ? original_points.size() - 1 : i - 1];
            const Vec2& curr = original_points[i];
            const Vec2& next = original_points[(i + 1) % original_points.size()];  
            // // 检查当前边是否足够长
            // if (norm(Vec2{curr.x - prev.x, curr.y - prev.y}) < min_edge_length) {
            //     // 如果当前边不够长，比较与中心点的距离，保留较远的点
            //     double dist_to_center_curr = norm(Vec2{center.x - curr.x, center.y - curr.y});
            //     double dist_to_center_prev = norm(Vec2{center.x - prev.x, center.y - prev.y});
            //     if (dist_to_center_curr < dist_to_center_prev) {
            //         continue; // 跳过当前点，因为它比前一个点更接近中心
            //     }
            // }  
            // 使用角度判断是否为关键拐点
            if (!areCollinear(prev, curr, next, angle_eps)) {
                simplified_points.push_back(curr);
            }
        }

        // 第3步：输出这一块的部分json段
        json_stream << "[";
        for (size_t j = 0; j < simplified_points.size(); ++j) {
            json_stream << "{\"x\": " << simplified_points[j].x
                        << ", \"y\": " << simplified_points[j].y << "}";
            if (j < simplified_points.size() - 1) {
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

    // 第4步：用于调试json形式（在go后台demo代码上可以独立验证）
    // std::ostringstream filename_stream;
    // filename_stream << "/workspace/mtuav-competition/log/map-" << drone.drone_id << ".json";
    // std::string filename = filename_stream.str();
    // std::ofstream file_out(filename);
    // file_out << floorPlan;
    // file_out.close();
    ////////////////

    std::vector<char> writable(floorPlan.begin(), floorPlan.end());
    writable.push_back('\0'); // 确保以空字符终止
    FindPath_return result = FindPath(writable.data(), start.x, start.y, end.x, end.y);

    Vec2* raw_result = result.r0; // Assuming r0 is the pointer to Vec3 array
    int size = result.r1;         // Assuming r1 is the size of the array

    // Convert the C array to a std::vector<Vec3>
    std::vector<Vec2> path(raw_result, raw_result + size);
    free(raw_result);
    std::vector<Vec3> waypoints;
    if (size < 2){
        waypoints.push_back({-1,-1,-1});  // TODO: 把不可达的因素考虑进来吧
    }
    else{
        path.erase(path.begin());
        path.pop_back();
        
        for (Vec2 point: path){
            waypoints.push_back({point.x, point.y, mydrone.flying_height});
        }
    }

    return waypoints;
}

double estimate_total_time(const std::vector<CargoInfo>& selected_cargoes, 
                        const Vec3& initial_drone_position, 
                        int initial_time, 
                        int takeoff_time, 
                        int landing_time, 
                        double exp_speed,
                        double factor) {
    double total_time = initial_time;
    Vec3 current_position = initial_drone_position;

    for (const auto& cargo : selected_cargoes) {
        // 计算到货物位置所需的时间
        double time_to_cargo = distance_2D(current_position,cargo.position) * factor / exp_speed 
                                    + takeoff_time + landing_time;

        // 计算到目标位置所需的时间
        double time_to_target = distance_2D(cargo.position,cargo.target_position) * factor / exp_speed 
                                    + takeoff_time + landing_time;

        // 更新总时间和当前位置
        total_time += time_to_cargo + time_to_target;
        current_position = cargo.target_position;

        // 检查时间限制
        if (total_time > cargo.latest_seconds_left) {
            return -1;  // 表示有一个订单不能在时间内完成
        }
    }

    return total_time;
}

// 剩余电量是否能到达某个点的估计
bool can_not_arrive(Vec3 drone_position, Vec3 station_position, float battery, double exp_speed, 
                          double landing_time, double factor) {
    double exp_time = distance_2D(drone_position,station_position) * factor / exp_speed + landing_time;
    double exp_energy_consume = exp_time/20*3; 
    return exp_energy_consume >= battery;
}

// 计算订单的优先级分数
double calculatePriorityScore(const CargoInfo& cargo, double current_time, Vec3 current_position, double exp_speed, double takeoff_time, double landing_time) {
    // 计算到货物的时间和距离
    double time_to_pickup = distance_2D(current_position, cargo.position) / exp_speed + takeoff_time + landing_time;
    double time_to_delivery = distance_2D(cargo.position, cargo.target_position) / exp_speed + takeoff_time + landing_time;
    double total_time_to_complete = time_to_pickup + time_to_delivery;

    // 计算是否能在限定时间内完成交付
    bool can_deliver_before_expected = (current_time + total_time_to_complete < cargo.expected_seconds_left);
    bool can_deliver_before_latest = (current_time + total_time_to_complete < cargo.latest_seconds_left);

    // 定义分数组件
    double score = 0.0;
    double penalty_factor = -1; // 如果不能按时完成，给予的惩罚系数
    double early_bonus = 20;
    double base_score = 100; // 每个订单的基础分数

    // 如果不能按时完成
    if (!can_deliver_before_latest) {
        score += penalty_factor * (current_time + total_time_to_complete - cargo.latest_seconds_left);
    } else {
        // 如果能在期望时间内完成
        if (can_deliver_before_expected) {
            score += early_bonus;
        }
        // 基础分数
        score += base_score;
    }
    return score;
}


// 使用该估算函数来确定是否接受新的订单。这样可以确保所有选中的订单都能在规定的时间内完成。
// 这里所有的时间单位暂时都用秒叭，速度用m/s
std::vector<CargoInfo> selectAndOrderCargoes( 
        const std::vector<CargoInfo>& available_cargoes,
        const Vec3& drone_position,
        double current_drone_weight,
        double max_weight,
        int max_orders,
        double exp_speed,
        double takeoff_time,
        double landing_time,
        double current_time,
        double factor) {

    std::vector<CargoInfo> selected_cargoes;
    double current_weight = current_drone_weight;
    Vec3 current_position = drone_position;
    auto remaining_cargoes = available_cargoes;  // 创建一个副本以便我们可以修改它

    while (!remaining_cargoes.empty() && selected_cargoes.size() < max_orders) {
        auto best_cargo_it = std::min_element(remaining_cargoes.begin(), remaining_cargoes.end(),
            [current_position, &current_weight, &current_time, max_weight, exp_speed, takeoff_time, 
            landing_time](const CargoInfo& a, const CargoInfo& b) {
                // 计算到各货物的时间和距离
                double time_to_a = distance_2D(current_position,a.position) / exp_speed + takeoff_time + landing_time
                             + distance_2D(a.position,a.target_position) / exp_speed + takeoff_time + landing_time;
                
                double time_to_b = distance_2D(current_position,b.position) / exp_speed + takeoff_time + landing_time
                             + distance_2D(b.position,b.target_position) / exp_speed + takeoff_time + landing_time;

                // 计算是否能在限定时间内完成交付
                bool can_deliver_a = (current_time + time_to_a <= a.latest_seconds_left);
                bool can_deliver_b = (current_time + time_to_b <= b.latest_seconds_left);

                // 计算是否超过重量限制
                bool not_over_weight_a = (current_weight + a.weight <= max_weight);
                bool not_over_weight_b = (current_weight + b.weight <= max_weight);

                // 综合考虑
                if (!can_deliver_a && can_deliver_b) return false;
                if (can_deliver_a && !can_deliver_b) return true;
                if (!not_over_weight_a && not_over_weight_b) return false;
                if (not_over_weight_a && !not_over_weight_b) return true;

                double score_a = calculatePriorityScore(a, current_time, current_position, exp_speed, takeoff_time, landing_time);
                double score_b = calculatePriorityScore(b, current_time, current_position, exp_speed, takeoff_time, landing_time);
                
                if (score_a != score_b){
                    return score_a > score_b;
                }

                if (a.latest_seconds_left != b.latest_seconds_left){
                    return a.latest_seconds_left > b.latest_seconds_left;
                }

                // 在所有其他条件都不能用来决定 "最佳" 订单时，使用 id 来比较
                return a.id < b.id;
            });

        // 尝试添加这个新订单，看看是否所有订单都能在规定时间内完成
        std::vector<CargoInfo> temp_selected_cargoes = selected_cargoes;
        temp_selected_cargoes.push_back(*best_cargo_it);
        
        int estimated_time = estimate_total_time(temp_selected_cargoes, current_position, current_time,
                                                 takeoff_time, landing_time, exp_speed, factor);

        // 如果所有订单都能在规定时间内完成，那么正式添加这个新订单
        if (estimated_time != -1) {
            selected_cargoes.push_back(*best_cargo_it);
            current_weight += best_cargo_it->weight;
            current_position = best_cargo_it->target_position;
            current_time = estimated_time;  // 更新当前时间
        }

        // 从剩余货物列表中删除
        remaining_cargoes.erase(best_cargo_it);
    }

    // 此时，selected_cargoes 包含的都是能在规定时间内完成的订单
    return selected_cargoes;
}

void removeAcceptedCargoes(std::vector<CargoInfo>& cargoes_to_delivery_and_no_accepted, 
                            const std::vector<int>& unfinished_order) {
    
    cargoes_to_delivery_and_no_accepted.erase(
        std::remove_if(
            cargoes_to_delivery_and_no_accepted.begin(), 
            cargoes_to_delivery_and_no_accepted.end(),
            [&unfinished_order](const CargoInfo& cargo) {
                return std::find(unfinished_order.begin(), unfinished_order.end(), cargo.id) != unfinished_order.end();
            }
        ),
        cargoes_to_delivery_and_no_accepted.end()
    );
}

void removeConflictCargoes(std::vector<CargoInfo>& cargoes_to_delivery_and_no_accepted, 
                           const std::vector<int>& unfinished_order,
                           std::map<int, mtuav::CargoInfo> cargo_info,
                           double safety_distance,
                           double flying_height,
                           const std::vector<int>& black_list,
                           std::vector<Vec3> all_battery_stations,
                           const std::vector<std::vector<int>> dynamic_grid,
                           double factor) {
    // 删除所有可能导致与其他无人机碰撞的货物
    cargoes_to_delivery_and_no_accepted.erase(
        std::remove_if(
            cargoes_to_delivery_and_no_accepted.begin(), 
            cargoes_to_delivery_and_no_accepted.end(),
            [&unfinished_order, safety_distance,cargo_info, flying_height,black_list,factor,
                    all_battery_stations, dynamic_grid](const CargoInfo& cargo) {
                // 会有地点很高的外卖
                if ((cargo.position.z >= flying_height) || (cargo.target_position.z >= flying_height))  {
                    return true;
                }
                
                // 死活拿不到的外卖（之前是状态机维护问题，现在暂时认为不存在降落点误差）
                auto it = std::find(black_list.begin(), black_list.end(), cargo.id);
                if (it != black_list.end()) return true;             
                
                // 大一统判断方式：把整个航线经过的体素都锁定一下，避免反复时序上悬停的判断
                if (dynamic_grid[cargo.position.y][cargo.position.x] || dynamic_grid[cargo.target_position.y][cargo.target_position.x]){
                    return true;
                }
                // 以下策略过于琐碎且保守，尝试直接换成dynamic_grid结合规划的结果来一次性判断
                // 1. 删除所有id出现在当前所有飞机unfinished_order中的元素
                // if (std::find(unfinished_order.begin(), unfinished_order.end(), cargo.id) != unfinished_order.end()){
                //     return true;
                // }
                // 2. 删除这个可能导致碰撞的货物
                // for (const auto& other_cargo_id : unfinished_order) {
                //     if (other_cargo_id == -1) continue;
                //     CargoInfo other_cargo = cargo_info.at(other_cargo_id);
                //     if ((distance_2D(cargo.position, other_cargo.position) < safety_distance*factor) ||
                //         (distance_2D(cargo.position, other_cargo.target_position) < safety_distance*factor) ||
                //         (distance_2D(cargo.target_position, other_cargo.position) < safety_distance*factor) ||
                //         (distance_2D(cargo.target_position, other_cargo.target_position) < safety_distance*factor)
                //         ) {  //加一个缓冲距离
                //         return true;  
                //     }
                // }
                // 3. 放弃离楼太近的单
                // if (static_grid[cargo.position.y][cargo.position.x] || static_grid[cargo.target_position.y][cargo.target_position.x]){
                //     return true;
                // }
                // 4. 所有的充电桩附近的货物都别拿了吧
                // for (const auto& battery_station : all_battery_stations) {
                //     if ((distance_2D(cargo.position, battery_station) < safety_distance*factor) || 
                //         (distance_2D(cargo.target_position, battery_station) < safety_distance*factor)){
                //         return true;
                //     }
                // }

                return false;  // 保留这个货物
            }
        ),
        cargoes_to_delivery_and_no_accepted.end()
    );
}

// TODO 需要参赛选手自行设计求解算法
// TODO 下面给出一个简化版示例，用于说明无人机飞行任务下发方式
int64_t myAlgorithm::solve() {
    // 【计时】记录开始时间点
    auto start_time = std::chrono::high_resolution_clock::now();
    // 根据算法计算情况，得出下一轮的算法调用间隔，单位ms
    // 依据需求计算所需的sleep time
    int64_t sleep_time_ms = 500;
    int64_t takeoff_pending_time_ms = 10000;
    double dangerous_battery = 50;
    double safe_distance = 10;
    double safety_factor = 1.2;
    double available_breaking_distance = 100;  // 充电站附近的有效停靠点
    _map->Range(&map_min_x, &map_max_x, &map_min_y, &map_max_y, &map_min_z, &map_max_z);
    // 处理无人机信息，找出当前未装载货物的无人机集合
    std::vector<DroneStatus> drones_to_pick;
    std::vector<DroneStatus> drones_need_recharge;
    std::vector<DroneStatus> drones_need_break;
    std::vector<DroneStatus> drones_to_delivery;
    std::vector<DroneStatus> drones_to_hover;

    // 初始化无人机状态机
    if (my_drone_info.empty()){
        LOG(INFO) << "Initialize our used drones!";

        // 本地测试用方案
        my_drone_info["drone-001"].flying_height = 120;
        my_drone_info["drone-002"].flying_height = 120;
        my_drone_info["drone-003"].flying_height = 120;
        my_drone_info["drone-004"].flying_height = 108;
        my_drone_info["drone-005"].flying_height = 108;
        my_drone_info["drone-006"].flying_height = 108;
        my_drone_info["drone-007"].flying_height = 96;
        my_drone_info["drone-008"].flying_height = 96;
        my_drone_info["drone-009"].flying_height = 96;
        my_drone_info["drone-010"].flying_height = 84;
        my_drone_info["drone-011"].flying_height = 84;
        my_drone_info["drone-012"].flying_height = 84;
        my_drone_info["drone-013"].flying_height = 72;
        my_drone_info["drone-014"].flying_height = 72;
        my_drone_info["drone-015"].flying_height = 72;
        
        // 在线测试用方案
        // my_drone_info["drone-001"].flying_height = 120;
        // my_drone_info["drone-002"].flying_height = 108;
        // my_drone_info["drone-003"].flying_height = 96;
        // my_drone_info["drone-004"].flying_height = 84;
        // my_drone_info["drone-005"].flying_height = 72;

        // 更新初始固定障碍物
        initialize_static_grid();

        auto& battery_station_positions = _task_info->battery_stations;
        for (std::size_t i = 0; i < battery_station_positions.size(); ++i) {
            const Vec3& station = battery_station_positions[i];
            LOG(INFO) << "battery_stations " << i << ": x = " << station.x 
                    << ", y = " << station.y << ", z = " << station.z;
        }

        auto& landing_positions = _task_info->landing_positions;
        for (std::size_t i = 0; i < landing_positions.size(); ++i) {
            const Vec3& station = landing_positions[i];
            LOG(INFO) << "landing_stations " << i << ": x = " << station.x 
                    << ", y = " << station.y << ", z = " << station.z;
        }

        return sleep_time_ms;
    }

    // 处理订单信息，找出可进行配送的订单集合
    std::vector<CargoInfo> cargoes_to_delivery;
    for (const auto& [id, cargo] : this->_cargo_info) {
        // 只有当cargo的状态为CARGO_WAITING时，才是当前可配送的订单
        if (cargo.status == CargoStatus::CARGO_WAITING) {
            cargoes_to_delivery.push_back(cargo);
        }
        // 帮助更新状态机中批量送外卖、已完成/再也不能送的外卖
        if (cargo.status == CargoStatus::CARGO_DELIVERED || cargo.status == CargoStatus::CARGO_FAILED) {
            for (auto& [drone_id, drone_info] : my_drone_info) {
                auto& unfinished_cargoes = drone_info.unfinished_cargo_ids;
                
                int count_to_remove = std::count_if(unfinished_cargoes.begin(), unfinished_cargoes.end(),
                                                    [&cargo](int id) { return id == cargo.id; });
                
                auto it = std::remove_if(unfinished_cargoes.begin(), unfinished_cargoes.end(),
                                        [&cargo](int id) { return id == cargo.id; });
                unfinished_cargoes.erase(it, unfinished_cargoes.end());
                
                for (int i = 0; i < count_to_remove; ++i) {
                    unfinished_cargoes.push_back(-1);
                }
                
                while (unfinished_cargoes.size() < 3) {
                    unfinished_cargoes.push_back(-1);
                }
            }
            LOG(INFO) << "delivered cargo id: " << cargo.id << ", final status: " << int(cargo.status) 
                      << ", bonus interval: " << cargo.expected_seconds_left << ", ddl interval: " << cargo.latest_seconds_left;
        }
        // TODO: 依据订单信息定制化特殊操作
        // 太危险的订单是不是可以不接
    }
    LOG(INFO) << "cargo info size: " << this->_cargo_info.size()
              << ", cargo to delivery size: " << cargoes_to_delivery.size();
    
    // 维护无人机状态机的内容
    for (auto& drone : this->_drone_info) {
        if(my_drone_info.find(drone.drone_id) == my_drone_info.end()){
            LOG(INFO) << "Now we do not use: " << drone.drone_id;
            continue;
        }
        else{
            LOG(INFO) << "[mydrone] status, id: " << drone.drone_id
                      << ", drone status: " << int(drone.status)
                      << ", drone position: "<< drone.position.x << " " << drone.position.y << " " << drone.position.z
                      << ", drone battery: " << drone.battery;
            if (drone.status == Status::CRASH){
                LOG(INFO) << "drone crash reason: " << drone.crash_type;
            }
            
            my_drone_info[drone.drone_id].drone_position = drone.position;

            if ((drone.delivering_cargo_ids == my_drone_info[drone.drone_id].current_cargo_ids)){
                my_drone_info[drone.drone_id].cargo_info_unchanged = true;
            }
            else{
                my_drone_info[drone.drone_id].cargo_info_unchanged = false;
            }

            my_drone_info[drone.drone_id].drone_status = drone.status;
            my_drone_info[drone.drone_id].drone_battery = drone.battery;
            my_drone_info[drone.drone_id].current_cargo_ids = drone.delivering_cargo_ids;
            LOG(INFO) << "cargo info:";
            for (auto c : my_drone_info[drone.drone_id].current_cargo_ids) LOG(INFO) << "delivering-c-id: " << c;
            for (auto c: my_drone_info[drone.drone_id].unfinished_cargo_ids) LOG(INFO) << "unfinished-c-id: " << c;
            LOG(INFO) << drone.drone_id << "\'s target charging station:" 
                                        << " " << my_drone_info[drone.drone_id].target_charging_station.x
                                        << " " << my_drone_info[drone.drone_id].target_charging_station.y 
                                        << " " << my_drone_info[drone.drone_id].target_charging_station.z;
            LOG(INFO) << drone.drone_id << "\'s target breaking station:" 
                                        << " " << my_drone_info[drone.drone_id].target_breaking_station.x
                                        << " " << my_drone_info[drone.drone_id].target_breaking_station.y 
                                        << " " << my_drone_info[drone.drone_id].target_breaking_station.z;  

            // 初始化临时动态障碍物，默认有静态障碍信息
            my_drone_info[drone.drone_id].dynamic_grid = my_drone_info[drone.drone_id].static_grid;    
        }
    }

    // 更新临时动态障碍物，添加动态障碍信息（包括其它飞机航线产生的临时体素、可能的动态障碍物）
    update_dynamic_grid(safe_distance, safety_factor);
    
    // 每次只生成一条飞行计划
    bool has_plan_a_drone = false;
    
    for (auto& drone : this->_drone_info) {
        if (has_plan_a_drone) continue;
        if(my_drone_info.find(drone.drone_id) == my_drone_info.end()){
            continue;
        }
        MyDroneInfo& mydrone = my_drone_info[drone.drone_id];
        
        //////////////////////////////////////////////////////////////////////////////////////
        // 无人机状态1:READY
        bool has_cargo = false;
        for (auto cargo_id: mydrone.current_cargo_ids){
            if (cargo_id!=-1){
                has_cargo = true;
                break;
            }
        }
        // 尽量保证不要带着包裹去充电站
        bool need_charge = false;
        if ((!has_cargo) && (mydrone.drone_battery < dangerous_battery*1.5)){
            need_charge = true;
        }
        if ((has_cargo) && (mydrone.drone_battery < dangerous_battery)){
            need_charge = false;
        }
        if ((need_charge) && (mydrone.target_charging_station.x == -1)){
            auto available_battery_stations = this->_task_info->battery_stations;
            std::sort(available_battery_stations.begin(), available_battery_stations.end(), [drone](Vec3 p1, Vec3 p2) {
                double p1_to_drone = distance_3D(drone.position, p1);
                double p2_to_drone = distance_3D(drone.position, p2);
                return p1_to_drone < p2_to_drone;
            });
            int chosen_station_index = 0;
            auto chosen_charging_station = available_battery_stations.at(chosen_station_index);
            
            // 考虑是否可达（包括占用）
            if (mydrone.dynamic_grid[chosen_charging_station.y][chosen_charging_station.x]){
                auto available_breaking_stations = this->_task_info->landing_positions;
                std::sort(available_breaking_stations.begin(), available_breaking_stations.end(), [chosen_charging_station, mydrone](Vec3 p1, Vec3 p2) {
                    double drone_to_p1_to_station = distance_3D(mydrone.drone_position, p1) + distance_3D(p1,chosen_charging_station);
                    double drone_to_p2_to_station = distance_3D(mydrone.drone_position, p2) + distance_3D(p2,chosen_charging_station);
                    
                    // 计算是否能够到达相应临时停靠站
                    bool can_not_reach_p1 = mydrone.dynamic_grid[p1.y][p1.x];
                    bool can_not_reach_p2 = mydrone.dynamic_grid[p2.y][p2.x];
                    if (!can_not_reach_p1 && can_not_reach_p2) return true;
                    if (can_not_reach_p1 && !can_not_reach_p2) return false;
                    return drone_to_p1_to_station < drone_to_p2_to_station;
                });
                mydrone.target_breaking_station = available_breaking_stations.at(chosen_station_index);    
            } else {
                mydrone.target_charging_station = chosen_charging_station;
            }
        }

        if ((mydrone.drone_status == Status::READY) && ((mydrone.target_charging_station.x != -1))
          && (mydrone.drone_position!=mydrone.target_charging_station)){
            mydrone.target_breaking_station.x = -1;
            mydrone.target_breaking_station.y = -1;
            mydrone.target_breaking_station.z = -1;
            drones_need_recharge.push_back(drone);
            has_plan_a_drone = true;
            continue;
        }

        if ((mydrone.drone_status == Status::READY) && ((mydrone.target_breaking_station.x != -1))
          && (mydrone.drone_position!=mydrone.target_breaking_station)){
            drones_need_break.push_back(drone);
            has_plan_a_drone = true;
            continue;
        }

        if ((!need_charge) && (mydrone.target_charging_station.x != -1)){
            mydrone.target_charging_station.x = -1;
            mydrone.target_charging_station.y = -1;
            mydrone.target_charging_station.z = -1;
            mydrone.wait_to_work = true;
        }

        if (mydrone.drone_status == Status::READY) {
            double current_weight = 0;
            auto tmp_delivering_cargo_ids= drone.delivering_cargo_ids;
            auto tmp_unfinished_cargo_ids= my_drone_info[drone.drone_id].unfinished_cargo_ids;

            std::sort(tmp_delivering_cargo_ids.begin(), tmp_delivering_cargo_ids.end(), std::greater<>()); // 从大到小排序
            std::sort(tmp_unfinished_cargo_ids.begin(), tmp_unfinished_cargo_ids.end(), std::greater<>()); // 从大到小排序

            if ((tmp_unfinished_cargo_ids[0]==-1) && (tmp_unfinished_cargo_ids[1]==-1) &&
                    (tmp_unfinished_cargo_ids[2]==-1)) { 
                // 我需要无人机的最大重量，最大订单数量，当前时间等信息，选择和排序可配送的订单
                DroneLimits dl = this->_task_info->drones.front().drone_limits;
                std::vector<CargoInfo> cargoes_to_delivery_and_no_accepted = cargoes_to_delivery;  // 拷贝一个临时变量

                // 删掉已经被其它飞机接单的cargo、其它飞机未来落点下的cargo（TODO：目前感觉过于保守琐碎，切换成用dynamic grid）
                // for (const auto& pair : my_drone_info) {
                //     removeConflictCargoes(cargoes_to_delivery_and_no_accepted, pair.second.unfinished_cargo_ids,
                //                         this->_cargo_info, safe_distance, pair.second.flying_height, pair.second.black_cargo_list,
                //                         this->_task_info->battery_stations, pair.second.static_grid, safety_factor);
                removeConflictCargoes(cargoes_to_delivery_and_no_accepted, mydrone.unfinished_cargo_ids,
                            this->_cargo_info, safe_distance, mydrone.flying_height, mydrone.black_cargo_list,
                            this->_task_info->battery_stations, mydrone.dynamic_grid, safety_factor);
                

                // 以可行解为优先的多步贪心（速度估计暂时采用保守的15m/s，因为直接飞直线大概能到19，但不知道避障的开销)
                // TODO: 实测发现多单，还是有点复杂的（主要是去往充电站的问题）
                double max_orders = 1;
                std::vector<CargoInfo> delivery_order = selectAndOrderCargoes(cargoes_to_delivery_and_no_accepted, 
                                        drone.position, current_weight, dl.max_weight, max_orders,
                                        18, 10, 10, 0, safety_factor);
                if (!delivery_order.empty()){
                    std::vector<int> unfinished_order;
                    CargoInfo earliest_order = delivery_order.front();
                    unfinished_order.push_back(earliest_order.id);
                    while (unfinished_order.size() < dl.max_cargo_slots) {
                        unfinished_order.push_back(-1); 
                    }
                    mydrone.unfinished_cargo_ids = unfinished_order;
                    mydrone.wait_to_work = true;
                    drones_to_pick.push_back(drone);
                    has_plan_a_drone = true;
                } else {
                    LOG(INFO) << drone.drone_id << " - can not find any proper cargos....";
                }
                // 以下是多单相关代码   
                // std::vector<int> unfinished_order;
                // int max_orders = dl.max_cargo_slots; // 这个暂时太复杂了
                // double total_weight = 0.0;  // 用于跟踪unfinished_order中的总重量
                // while (!delivery_order.empty() && unfinished_order.size() < dl.max_cargo_slots) {
                //     // 取出delivery_order中最早加入的元素（在vector的前面）
                //     CargoInfo earliest_order = delivery_order.front();
                    
                //     // 检查添加这个订单后总重量是否会超过限制
                //     if (total_weight + earliest_order.weight <= dl.max_weight) {
                //         // 添加到unfinished_order，并更新总重量
                //         unfinished_order.push_back(earliest_order.id);
                //         total_weight += earliest_order.weight;
                //     } else {
                //         // 如果添加这个订单会导致超过重量限制，就跳出循环
                //         break;
                //     }
                    
                //     // 从delivery_order中移除已经加入到unfinished_order的元素
                //     delivery_order.erase(delivery_order.begin());
                // }
   
            } 
            else {
                if ((tmp_delivering_cargo_ids[0] != tmp_unfinished_cargo_ids[0] ||
                tmp_delivering_cargo_ids[1] != tmp_unfinished_cargo_ids[1] ||
                tmp_delivering_cargo_ids[2] != tmp_unfinished_cargo_ids[2])){
                    if ((mydrone.cargo_info_unchanged) && (!mydrone.wait_to_work)) {
                        LOG(INFO) << drone.drone_id << " - wait for sync and then deliver....";
                    } else{
                        drones_to_pick.push_back(drone); // 没有取到所有货物，先取货物
                        has_plan_a_drone = true;
                    }
                }
                else{
                    if ((mydrone.cargo_info_unchanged) && (tmp_delivering_cargo_ids[0]==-1)) {
                        LOG(INFO) << drone.drone_id << " - wait for sync and then pick....";
                    } else{
                        drones_to_delivery.push_back(drone); // 应该已经取到所有货物了，开始配送
                        has_plan_a_drone = true; 
                    }
                } 
            }
            continue;
        }

        //////////////////////////////////////////////////////////////////////////////////////
        // 无人机状态2:平飞
        if (mydrone.drone_status == Status::FLYING) {
            mydrone.wait_to_work = false;
            continue;
        }

        // 无人机状态3:悬停（应对动态障碍物）
        // if ((mydrone.drone_status == Status::HOVERING) && (!unsafe_flag) 
        //       && (mydrone.target_charging_position.x !=-1)
        //       && (mydrone.target_charging_position.y !=-1)
        //       && (mydrone.target_charging_position.z !=-1)) {
        //     drones_need_recharge.push_back(drone);
        //     continue;
        // }
    } 
    // 以上为参赛选手需要依据无人机信息定制化特殊操作
    
    LOG(INFO) << "drone info size: " << this->_drone_info.size()
              << ", drones to pick cargo: size: " << drones_to_pick.size()
              << ", drones to delivery size: " << drones_to_delivery.size()
              << ", drones to hover size: " << drones_to_hover.size()
              << ", drones need recharge size: " << drones_need_recharge.size()
              << ", drones need take break size: " << drones_need_break.size();
    LOG(INFO) << "drones to pick cargo: ";
    for (auto d : drones_to_pick) {
        LOG(INFO) << d.drone_id;
    }

    LOG(INFO) << "drones to delivery cargo: ";
    for (auto d : drones_to_delivery) {
        LOG(INFO) << d.drone_id;
    }

    LOG(INFO) << "drones need recharge: ";
    for (auto d : drones_need_recharge) {
        LOG(INFO) << d.drone_id;
    }

    LOG(INFO) << "drones need take break: ";
    for (auto d : drones_need_break) {
        LOG(INFO) << d.drone_id;
    }

    // 获取当前毫秒时间戳（可能会有延迟问题）
    std::chrono::time_point<std::chrono::system_clock, std::chrono::milliseconds> tp =
        std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());
    auto current = std::chrono::duration_cast<std::chrono::milliseconds>(tp.time_since_epoch());
    int64_t current_time = current.count();
    std::vector<std::tuple<std::string, FlightPlan>> flight_plans_to_publish;  //下发飞行任务
    DroneLimits dl = this->_task_info->drones.front().drone_limits;
    
    //////////////////////////////////////////////////////////////////////////////////////////////////
    // 无人机与订单进行匹配，并生成飞行轨迹
    //////////////////////////////////////////////////////////////////////////////////////////////////
    // 策略1：为没有订单的无人机生成取订单航线
    for (auto the_drone : drones_to_pick) {
        MyDroneInfo& mydrone = my_drone_info[the_drone.drone_id];
        
        CargoInfo the_cargo;
        auto tmp_delivering_cargo_ids= the_drone.delivering_cargo_ids;
        auto tmp_unfinished_cargo_ids= mydrone.unfinished_cargo_ids;
        
        bool has_find=false;
        std::sort(tmp_delivering_cargo_ids.begin(), tmp_delivering_cargo_ids.end(), std::greater<>()); // 从大到小排序
        std::sort(tmp_unfinished_cargo_ids.begin(), tmp_unfinished_cargo_ids.end(), std::greater<>()); // 从大到小排序
        for (size_t j = 0; j < std::round(dl.max_cargo_slots); ++j) {
            if (tmp_delivering_cargo_ids[j] != tmp_unfinished_cargo_ids[j]) {
                int chosen_cargo_id = tmp_unfinished_cargo_ids[j];
                if (this->_cargo_info.find(chosen_cargo_id) != this->_cargo_info.end()) {
                    has_find = true;
                    LOG(INFO) << the_drone.drone_id << " find pick-up cargo: " << chosen_cargo_id;
                    the_cargo = this->_cargo_info.at(chosen_cargo_id);
                }
                break;
            }
        }
        if (!has_find){
            LOG(INFO) << the_drone.drone_id << " can not find any pick-up cargo";
        }

        auto landing_position = the_cargo.position;
        // landing_position.z = roundUpToMultipleOf4(the_cargo.position.z);

        LOG(INFO) << the_drone.drone_id << " to pick cargo, id: " << the_cargo.id << ", start: " << the_cargo.position.x
                  << " " << the_cargo.position.y << " " << the_cargo.position.z
                  << ", target: " << the_cargo.target_position.x << " "
                  << the_cargo.target_position.y << " " << the_cargo.target_position.z
                  << ", current uav position:" << the_drone.position.x << " " << the_drone.position.y << " " << the_drone.position.z
                  << ", current landing position:" << landing_position.x << " " << landing_position.y << " " << landing_position.z;
        FlightPlan pickup;
        // TODO 参赛选手需要自己实现一个轨迹生成函数或中转点生成函数（trajectory复杂/waypoint简单）
        auto [pickup_waypoints, pickup_flight_time] = this->trajectory_generation(
            the_drone.position, landing_position, the_drone, false);  //暂时都使用轨迹生成函数，不使用中转点生成函数
        pickup.target_cargo_ids.push_back(the_cargo.id);
        pickup.flight_purpose = FlightPurpose::FLIGHT_TAKE_CARGOS;  // 飞行计划目标
        pickup.flight_plan_type = FlightPlanType::PLAN_TRAJECTORIES;  // 飞行计划类型：轨迹
        pickup.flight_id = std::to_string(++Algorithm::flightplan_num);
        pickup.takeoff_timestamp = current_time + takeoff_pending_time_ms; 
        pickup.segments = pickup_waypoints;
        // 在下发飞行计划前，选手可以使用该函数自行先校验飞行计划的可行性
        auto reponse_pickup = this->_planner->ValidateFlightPlan(dl, pickup);
        LOG(INFO) << "ValidateFlightPlan, sussess: " << reponse_pickup.success << ", err msg: " << reponse_pickup.msg;
        if ((!reponse_pickup.success) || (reponse_pickup.msg == "empty segments")){
            LOG(INFO) << "Move to blacklist: " << the_cargo.id;
            mydrone.black_cargo_list.push_back(the_cargo.id);        
        }
        double total_flight_seconds = pickup_flight_time/1000;
        if ((total_flight_seconds + distance_2D(the_cargo.position,the_cargo.target_position)*1.0/15 + 20 < the_cargo.latest_seconds_left) 
          && (total_flight_seconds/20*3<the_drone.battery)){
            flight_plans_to_publish.push_back({the_drone.drone_id, pickup});
            LOG(INFO) << "Successfully generated flight plan, flight id: " << pickup.flight_id
                  << ", drone id: " << the_drone.drone_id
                  << ", flight purpose: " << int(pickup.flight_purpose)
                  << ", flight type: " << int(pickup.flight_plan_type)
                  << ", cargo id: " << the_cargo.id;
        } else {
            LOG(INFO) << "Refuse the generated flight plan, and then reset previous unfinished_cargo_ids ( "
                      << mydrone.unfinished_cargo_ids[0] << " "
                      << mydrone.unfinished_cargo_ids[1] << " "
                      << mydrone.unfinished_cargo_ids[2] << " )"
                      << ", drone id: " << the_drone.drone_id
                      << ", flight purpose: " << int(pickup.flight_purpose)
                      << ", flight type: " << int(pickup.flight_plan_type)
                      << ", now cargo id: " << the_cargo.id; 
            for(auto& item : mydrone.unfinished_cargo_ids) item = -1;
        }
        break; // 每次只生成一条取货飞行计划
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////
    // 策略2：为已经取货的飞机生成送货飞行计划
    for (auto the_drone : drones_to_delivery) {
        MyDroneInfo& mydrone = my_drone_info[the_drone.drone_id];

        int the_cargo_id = 0;
        // 找到货仓中第一个id不为-1的货物
        auto tmp_delivering_cargo_ids= the_drone.delivering_cargo_ids;
        bool has_find=false;
        for (auto cid : tmp_delivering_cargo_ids) {
            if (cid != -1) {
                has_find = true;
                LOG(INFO) << the_drone.drone_id << " find deli-very cargo, id: " << cid;
                the_cargo_id = cid;
                break;
            }
        }
        if (!has_find){
            LOG(INFO) << the_drone.drone_id << " can not find any deli-very cargo";
        }

        if (this->_cargo_info.find(the_cargo_id) != this->_cargo_info.end()) {
            auto the_cargo = this->_cargo_info.at(the_cargo_id);

            auto landing_position = the_cargo.target_position;
            // landing_position.z = roundUpToMultipleOf4(the_cargo.target_position.z);

            LOG(INFO) << the_drone.drone_id << " go to deliver cargo, id: " << the_cargo.id << ", start: " << the_cargo.position.x
                  << " " << the_cargo.position.y << " " << the_cargo.position.z
                  << ", target: " << the_cargo.target_position.x << " "
                  << the_cargo.target_position.y << " " << the_cargo.target_position.z
                  << ", current uav position:" << the_drone.position.x << " " << the_drone.position.y << " " << the_drone.position.z
                  << ", current landing position:" << landing_position.x << " " << landing_position.y << " " << landing_position.z;

            FlightPlan delivery;
            auto [delivery_traj, delivery_flight_time] = this->trajectory_generation(
                the_drone.position, landing_position, the_drone, false);
            if (delivery_flight_time == -1) {
                // 轨迹生成失败
                LOG(INFO) << "trajectory generation failed. ";
                break;
            }
            delivery.flight_purpose = FlightPurpose::FLIGHT_DELIVER_CARGOS;
            delivery.flight_plan_type = FlightPlanType::PLAN_TRAJECTORIES;
            delivery.flight_id = std::to_string(++Algorithm::flightplan_num);
            delivery.takeoff_timestamp = current_time + takeoff_pending_time_ms;
            delivery.segments = delivery_traj;
            delivery.target_cargo_ids.push_back(the_cargo.id);
            // 在下发飞行计划前，选手可以使用该函数自行先校验飞行计划的可行性
            auto reponse_delivery = this->_planner->ValidateFlightPlan(dl, delivery);
            LOG(INFO) << "ValidateFlightPlan, sussess: " << reponse_delivery.success <<", err msg: " << reponse_delivery.msg;
            if ((!reponse_delivery.success)||(reponse_delivery.msg == "empty segments")){
                LOG(INFO) << "Move to blacklist: " << the_cargo.id;
                my_drone_info[the_drone.drone_id].black_cargo_list.push_back(the_cargo.id);
            }

            double total_flight_seconds = delivery_flight_time/1000;
            if ( total_flight_seconds < the_cargo.latest_seconds_left){
                flight_plans_to_publish.push_back({the_drone.drone_id, delivery});
                LOG(INFO) << "Successfully generated flight plan, flight id: " << delivery.flight_id
                          << ", drone id: " << the_drone.drone_id
                          << ", flight purpose: " << int(delivery.flight_purpose)
                          << ", flight type: " << int(delivery.flight_plan_type)
                          << ", cargo id: " << the_cargo.id;
            } else {
                flight_plans_to_publish.push_back({the_drone.drone_id, delivery});
                LOG(INFO) << "Not suggest generated flight plan, flight id: " << delivery.flight_id
                          << ", drone id: " << the_drone.drone_id
                          << ", flight purpose: " << int(delivery.flight_purpose)
                          << ", flight type: " << int(delivery.flight_plan_type)
                          << ", cargo id: " << the_cargo.id;
            }
        }
        break;  // 每次只生成一条送货飞行计划
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////
    // 策略3：为电量小于指定数值的无人机生成换电航线
    for (auto the_drone : drones_need_recharge) {
        MyDroneInfo& mydrone = my_drone_info[the_drone.drone_id];
        
        Vec3 the_selected_station;
        the_selected_station = mydrone.target_charging_station;
        auto landing_position = the_selected_station;
        // landing_position.z = roundUpToMultipleOf4(the_selected_station.z);

        LOG(INFO) << the_drone.drone_id << " go to charge station, position: " << the_selected_station.x
                  << " " << the_selected_station.y << " " << the_selected_station.z
                  << ", current uav position:" << the_drone.position.x << " " << the_drone.position.y << " " << the_drone.position.z
                  << ", current landing position:" << landing_position.x << " " << landing_position.y << " " << landing_position.z;  

        FlightPlan recharge;
        // TODO 参赛选手需要自己实现一个轨迹生成函数或中转点生成函数
        auto [recharege_traj, recharge_flight_time] = this->trajectory_generation(
            the_drone.position, landing_position, the_drone, false);  //此处使用轨迹生成函数
        recharge.flight_purpose = FlightPurpose::FLIGHT_EXCHANGE_BATTERY;
        recharge.flight_plan_type = FlightPlanType::PLAN_TRAJECTORIES;
        recharge.flight_id = std::to_string(++Algorithm::flightplan_num);
        recharge.takeoff_timestamp = current_time + takeoff_pending_time_ms;
        recharge.segments = recharege_traj;

        flight_plans_to_publish.push_back({the_drone.drone_id, recharge});
        if (!recharege_traj.empty()) {
            LOG(INFO) << "first point z: " << recharege_traj.front().position.z;
        } else {
            LOG(WARNING) << "recharege_traj is empty!";
        }
        LOG(INFO) << "Successfully generated flight plan, flight id: " << recharge.flight_id
                << ", drone id: " << the_drone.drone_id
                << ", flight purpose: " << int(recharge.flight_purpose)
                << ", flight type: " << int(recharge.flight_plan_type) << ", cargo id: none";

        break;  // 每次只生成一条换电飞行计划
    }


    //////////////////////////////////////////////////////////////////////////////////////////////////
    // 策略4：将需要充电、但暂时充电站被占用的飞机，放在临时停靠站排队，用来排队换电
    for (auto the_drone : drones_need_break) {
        MyDroneInfo& mydrone = my_drone_info[the_drone.drone_id];
        
        Vec3 the_selected_station;
        the_selected_station = mydrone.target_breaking_station;
        auto landing_position = the_selected_station;
        // landing_position.z = roundUpToMultipleOf4(the_selected_station.z);

        LOG(INFO) << the_drone.drone_id << " go to breaking station, position: " << the_selected_station.x
                  << " " << the_selected_station.y << " " << the_selected_station.z
                  << ", current uav position:" << the_drone.position.x << " " << the_drone.position.y << " " << the_drone.position.z
                  << ", current landing position:" << landing_position.x << " " << landing_position.y << " " << landing_position.z;  

        FlightPlan breaking;
        // TODO 参赛选手需要自己实现一个轨迹生成函数或中转点生成函数
        auto [breaking_traj, breaking_flight_time] = this->trajectory_generation(
            the_drone.position, landing_position, the_drone, false);  //此处使用轨迹生成函数
        breaking.flight_purpose = FlightPurpose::FLIGHT_COMMON;
        breaking.flight_plan_type = FlightPlanType::PLAN_TRAJECTORIES;
        breaking.flight_id = std::to_string(++Algorithm::flightplan_num);
        breaking.takeoff_timestamp = current_time + takeoff_pending_time_ms;
        breaking.segments = breaking_traj;

        flight_plans_to_publish.push_back({the_drone.drone_id, breaking});
        if (!breaking_traj.empty()) {
            LOG(INFO) << "first point z: " << breaking_traj.front().position.z;
        } else {
            LOG(WARNING) << "breaking_traj is empty!";
        }
        LOG(INFO) << "Successfully generated flight plan, flight id: " << breaking.flight_id
                << ", drone id: " << the_drone.drone_id
                << ", flight purpose: " << int(breaking.flight_purpose)
                << ", flight type: " << int(breaking.flight_plan_type) << ", cargo id: none";

        break;  // 每次只生成一条临时停靠计划
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////
    // 下发所求出的飞行计划
    for (auto& [drone_id, flightplan] : flight_plans_to_publish) {
        auto publish_result = this->_planner->DronePlanFlight(drone_id, flightplan);
        LOG(INFO) << "Published flight plan, flight id: " << flightplan.flight_id
                  << ", successfully?: " << std::boolalpha << publish_result.success
                  << ", msg: " << publish_result.msg;
    }
    // TODO 找出需要悬停的无人机（目前服务于未来可能的动态避障来用）
    for (auto& drone : drones_to_hover) {
        this->_planner->DroneHover(drone.drone_id);
        LOG(INFO) << "Send drone hover command, drone id: " << drone.drone_id;
    }

    // 记录结束时间点
    auto stop_time = std::chrono::high_resolution_clock::now();
    // 计算所经历的时间
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop_time - start_time);
    LOG(INFO) << "One solve procress consumes " << duration.count() << " ms";

    return sleep_time_ms;
}

std::tuple<std::vector<Segment>, int64_t> myAlgorithm::trajectory_generation(Vec3 start, Vec3 end,
                                                                             DroneStatus drone,
                                                                             bool without_taking_off) {
    // TODO: 此时默认without_taking_off=false
    MyDroneInfo& mydrone = my_drone_info[drone.drone_id];
    double flying_height = mydrone.flying_height;
    int64_t flight_time;

    TrajectoryGeneration tg;  // 引用example中的轨迹生成算法
    // 定义四个轨迹点
    Segment p1, p2;
    Vec3 p1_pos, p2_pos;
    p1_pos.x = start.x;
    p1_pos.y = start.y;
    p1_pos.z = start.z;
    p1.position = p1_pos;

    p2_pos.x = start.x;
    p2_pos.y = start.y;
    p2_pos.z = flying_height;
    p2.position = p2_pos;

    p1.seg_type = 0;
    p2.seg_type = 0;

    // 自己用一个恶心心的状态机，使用A*来做无冲突规划，如果中间没有障碍的话，应该这个值是空的
    std::vector<Vec3> flying_waypoints;
    flying_waypoints = generate_waypoints_by_a_star(start, end, mydrone);

    Segment p3, p4;  // p3 终点上方高度120米，p4 终点
    Vec3 p3_pos;
    p3_pos.x = end.x;
    p3_pos.y = end.y;
    p3_pos.z = flying_height;
    p3.position = p3_pos;
    p3.seg_type = 1;
    p4.position = end;
    p4.seg_type = 2;

    // 获取无人机的性能指标
    // 此处假设所有无人机均为同型号（dzp：这是一个重要假设！！！）
    DroneLimits dl = this->_task_info->drones.front().drone_limits;

    // 生成p1->p2段轨迹点
    int64_t p1top2_flight_time = 0;
    std::vector<mtuav::Segment> p1top2_segs;
    if (!without_taking_off){
        bool success_1 = tg.generate_traj_from_waypoints({p1.position, p2.position}, dl, 0, p1top2_segs);
        LOG(INFO) << "p1top2 traj gen: " << std::boolalpha << success_1;
        if (success_1 == false) {
            return {std::vector<mtuav::Segment>{}, -1};
        }
        p1top2_flight_time = p1top2_segs.back().time_ms;  // p1->p2飞行时间
    }
    

    // 生成p2->p3段轨迹点（TODO: 这里需要改进成A*）
    std::vector<mtuav::Segment> p2top3_segs;
    std::vector<mtuav::Vec3> all_waypoints;
    all_waypoints.push_back(p2.position); // 添加起始点
    all_waypoints.insert(all_waypoints.end(), flying_waypoints.begin(), flying_waypoints.end()); // 在起始点和终点之间插入额外的航点
    all_waypoints.push_back(p3.position); // 添加终点
    // 打印 all_flying_waypoints 中的所有 x、y、z 值
    LOG(INFO) << "All waypoints:";
    for (const auto& point : all_waypoints) {
        LOG(INFO) << "x: " << point.x << ", y: " << point.y << ", z: " << point.z;
    }
    bool success_2 = tg.generate_traj_from_waypoints(all_waypoints, dl, 1, p2top3_segs);
    LOG(INFO) << "p2top3 traj gen: " << std::boolalpha << success_2;
    if (success_2 == false) {
        return {std::vector<mtuav::Segment>{}, -1};
    }
    int64_t p2top3_flight_time = p2top3_segs.back().time_ms;  // p2->p3飞行时间
    // TODO: 此处将起飞点、中间点（可能为空）、降落点加入状态机用于绘制dynamic_grid
    mydrone.flightplan_takeoff_position = start;
    mydrone.flightplan_land_position = end;
    mydrone.all_flying_waypoints = all_waypoints;

    
    // 生成p3->p4段轨迹点
    std::vector<mtuav::Segment> p3top4_segs;
    bool success_3 = tg.generate_traj_from_waypoints({p3.position, p4.position}, dl, 2, p3top4_segs);
    LOG(INFO) << "p3top4 traj gen: " << std::boolalpha << success_3;
    if (success_3 == false) {
        return {std::vector<mtuav::Segment>{}, -1};
    }
    int64_t p3top4_flight_time = p3top4_segs.back().time_ms;  // p3->p4飞行时间

    // 合并p1->p4多段轨迹
    // 处理p2->p3段轨 更新轨迹点时间
    int64_t p1top2_last_time = 0;
    if (!without_taking_off){
        p1top2_last_time = p1top2_segs.back().time_ms;
    }
    LOG(INFO) << "p1top2_last_time " << p1top2_last_time;
    auto first_23 = p2top3_segs.begin();
    LOG(INFO) << "p1top3_FIRST_time " << first_23->time_ms;
    p2top3_segs.erase(first_23);
    LOG(INFO) << "p1top3_second_time " << first_23->time_ms;
    for (int i = 0; i < p2top3_segs.size(); i++) {
        p2top3_segs[i].time_ms = p2top3_segs[i].time_ms + p1top2_last_time;
    }
    // 处理p3->p4段轨 更新轨迹点时间
    int64_t p2top3_last_time = p2top3_segs.back().time_ms;
    LOG(INFO) << "p2top3_last_time " << p2top3_last_time;
    auto first_34 = p3top4_segs.begin();
    LOG(INFO) << "p1top4_FIRST_time " << first_34->time_ms;
    p3top4_segs.erase(first_34);
    LOG(INFO) << "p1top4_FIRST_time " << first_34->time_ms;
    for (int i = 0; i < p3top4_segs.size(); i++) {
        p3top4_segs[i].time_ms = p3top4_segs[i].time_ms + p2top3_last_time;
    }

    // 更新轨迹点时间后，合并轨迹，这里需要加入我做的无碰撞轨迹
    std::vector<mtuav::Segment> traj_segs;
    if (!without_taking_off){
        traj_segs.insert(traj_segs.end(), p1top2_segs.begin(), p1top2_segs.end());
    }
    traj_segs.insert(traj_segs.end(), p2top3_segs.begin(), p2top3_segs.end());
    traj_segs.insert(traj_segs.end(), p3top4_segs.begin(), p3top4_segs.end());

    // LOG(INFO) << "combined segs detail: ";
    // for (auto s : traj_segs) {
    //     LOG(INFO) << "seg, p: " << s.position.x << " " << s.position.y << " " << s.position.z
    //               << ", time_ms: " << s.time_ms << ", a: " << s.a.x << " " << s.a.y << " " << s.a.z
    //               << ", v: " << s.v.x << " " << s.v.y << " " << s.v.z << ", type: " << s.seg_type;
    // }

    // 计算p1->p4时间
    int64_t p1top4_flight_time = p1top2_flight_time + p2top3_flight_time + p3top4_flight_time;

    return {traj_segs, p1top4_flight_time};
}

std::string myAlgorithm::segments_to_string(std::vector<Segment> segs) {
    std::string str = "";
    for (auto s : segs) {
        auto x = s.position.x;
        auto y = s.position.y;
        auto z = s.position.z;
        std::string cor =
            "(" + std::to_string(x) + "," + std::to_string(y) + "," + std::to_string(z) + ")->";
        str = str + cor;
    }
    return str;
}

}  // namespace mtuav::algorithm


