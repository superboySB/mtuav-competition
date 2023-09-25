#ifndef MTUAV_SDK_TYPES_H
#define MTUAV_SDK_TYPES_H

#include <cstdint>
#include <string>
#include <vector>

#define ST_TAKING_OFF 0
#define ST_FLYING 1
#define ST_LANDING 2

#ifdef __cplusplus
extern "C" {
#endif
typedef struct Vec3 {
  double x;
  double y;
  double z;
} Vec3;

typedef struct Segment {
  uint64_t time_ms;         // 在time_ms飞行到目标位置，此时间是相对于起飞时间
  int seg_type;             // 此片段是那种类型，起飞、飞行、降落；要求起飞和降落必须垂直
  Vec3 position;            // 目标位置
  Vec3 v;                   // 到达目标位置时的速度，轨迹专用
  Vec3 a;                   // 到达目标位置时的加速度，轨迹专用
} Segment;
#ifdef __cplusplus
}
#endif

namespace mtuav {
typedef ::Vec3 Vec3;
typedef ::Segment Segment;
// 无人机的性能限制
typedef struct DroneLimits {
  // 最大速度
  double max_fly_speed_v;
  double max_fly_speed_h;
  // 最大飞行加速度
  double max_fly_acc_v;
  double max_fly_acc_h;
  // 最大载重
  double max_weight;
  // 最大货物数
  double max_cargo_slots;
  double min_fly_height;
  double max_fly_height;
  // 最大飞行时间
  double max_flight_seconds;
} DroneLimits;

enum CargoStatus {
  CARGO_UNKNOWN = 0,
  CARGO_WAITING = 1,
  CARGO_DELIVERING = 2,
  CARGO_DELIVERED = 3,
  CARGO_FAILED = 4,
};

typedef struct CargoInfo {
  // 货物ID
  int id;
  // 货物状态
  CargoStatus status;
  // 位置
  Vec3 position;
  // 重量
  double weight;
  // 价值
  double award;
  // 需要运送的位置
  Vec3 target_position;
  // 距离用户期望送达还剩多少秒，<0表示已经超时；这个之前送达能获得增大奖励
  int expected_seconds_left;
  // 距离最晚送达时间还剩多少秒，<0表示已经超时；这个之后送达会有惩罚（如果时间不够，可以选择放弃此单）
  int latest_seconds_left;
} CargoInfo;

enum FlightPurpose {
  // 无特定目标
  FLIGHT_COMMON = 0,
  // 目标装载货物的航线，此航线飞机落地后，会自动装载地面货物当指定货舱
  FLIGHT_TAKE_CARGOS = 1,
  // 目标配送货物的航线，此航线飞机落地后，会自动卸载指定货舱的货物
  FLIGHT_DELIVER_CARGOS = 2,
  // 目标换电航线，若降落点为换电站，此航线飞机落地后，会自动更换电池
  FLIGHT_EXCHANGE_BATTERY = 3,
};

enum FlightPlanType {
  // 航点模式，飞机按照离散的航点进行飞行，每个航点都是在给定时刻要到达的点。起始位置不需要指定。
  // 每两个航点构成一个行段，飞机都会经历一次加速和减速的过程，切加减速都以加大加速度来完成
  // 其中起飞和降落为独立阶段, 第一个和最后一个航段为起飞和降落航段，必须垂直与地面向上或向下
  // 在飞行计划的执行过程中，也可以更改飞行计划，执行流程类似
  // 飞机执行前会检测飞行计划，如果明显超出飞行能力，会规划失败
  PLAN_WAY_POINTS = 0,
  // 轨迹模式，规划者需要计算稠密的轨迹点，指定每个轨迹点位置、时间、速度以及加速度
  // 每个轨迹点的时间间隔最大100ms，对应的Segment数组的time_ms需要正序增长，间隔不大于100ms
  // 轨迹模式下飞机飞行效率更高，更高更快完成任务，因此获得奖励也更高
  // 轨迹模式下，起飞和降落阶段也需要通过轨迹来规划，仅要求垂直起降
  PLAN_TRAJECTORIES = 1,
};

typedef struct FlightPlan {
  // 飞行计划类型：航点/轨迹
  FlightPlanType flight_plan_type;
  // 此飞行计划的目的：换电/取货/送货
  FlightPurpose flight_purpose;
  // 标识航线，一个id只能执行一次，用于现在同一条航线的多次规划
  std::string flight_id;
  // 航线起飞的时间（UTC+8时间），单位是毫秒
  uint64_t takeoff_timestamp;
  // 所有飞行的片段，无人机会根据flight_plan_type来执行
  std::vector<Segment> segments;
  // 指定装载/卸载的货物id，注意：装载失败，卸载到错误位置不会有通知
  std::vector<int> target_cargo_ids;
} FlightPlan;

// 无人机信息
typedef struct DroneInfo {
  std::string drone_id;
  // 初始位置
  Vec3 initial_pos;
  // 飞机能力限制
  DroneLimits drone_limits;
} DroneInfo;

typedef struct TaskInfo {
  int32_t task_id;
  // 所使用的地图
  std::string map_id;
  // 此任务包含的所有无人机
  std::vector<DroneInfo> drones;
  // 电量是否会消耗
  bool battery_consuming;
  // 初赛中无障碍物
  bool has_obstacles;
  // 换电站信息
  std::vector<Vec3> battery_stations;
  // 临时降落点信息
  std::vector<Vec3> landing_positions;
} TaskInfo;

enum Status {
  // 等待航线
  READY = 0,
  // 起飞中
  TAKING_OFF = 1,
  // 平飞中
  FLYING = 2,
  // 悬停中
  HOVERING = 3,
  // 降落中
  LANDING = 4,
  // 坠机
  CRASH = 5,
  // 收到航线，待飞
  READY_TO_FLY = 6,
};

enum ObstacleType {
  OBSTACLE_UNKNOWN = 0,
  OBSTACLE_DRONE = 1,
  OBSTACLE_BIRD = 2,
  OBSTACLE_KITE = 3,
};

// 初赛中不会出现动态障碍物
typedef struct ObstacleInfo {
  Vec3 position;
  Vec3 velocity;
  double radius;
  ObstacleType obstacle_type;
} ObstacleInfo;

enum DroneCrashType {
  DRONE_CRASH_NONE = 0,
  DRONE_CRASH_LOW_BATTERY = 1,
  DRONE_CRASH_COLLIDE_OBSTACLE = 2,
  DRONE_CRASH_COLLIDE_DRONE = 3,
};

typedef struct DroneStatus {
  std::string drone_id;
  uint64_t timestamp;
  Status status;
  Vec3 position;
  double height;                              // distance to ground
  std::vector<int> delivering_cargo_ids;
  // 剩余电量
  float battery;
  std::vector<ObstacleInfo> detected_obstacles;
  // 如果status == CRASH，这里记录原因
  DroneCrashType crash_type;
} DroneStatus;

typedef struct Response {
  bool success;
  std::string msg;  // error msg
} Response;
}
#endif //MTUAV_SDK_TYPES_H
