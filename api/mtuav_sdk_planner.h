#ifndef MTUAV_SDK_PLANNER_H
#define MTUAV_SDK_PLANNER_H

#include <map>
#include <memory>
#include <string>

#include "mtuav_sdk_export.h"
#include "mtuav_sdk_map.h"
#include "mtuav_sdk_types.h"

namespace mtuav {
class SDK_EXPORT PlannerAgent {
 public:
  // 传入比赛的IP:port
  explicit PlannerAgent(std::string server_addr, std::shared_ptr<Map> map,
                        std::string log_path = "");
  virtual ~PlannerAgent();

  // 允许设置当前地图
  void SetMap(std::shared_ptr<Map> map);

  // 登录比赛服务器
  Response Login(std::string username, std::string password);

  // 获取当前任务数n
  int GetTaskCount();

  // 查看任务信息，参数为任务索引[0, n)
  std::unique_ptr<TaskInfo> QueryTask(int task_index);

  // 开始指定任务，参数为任务索引[0, n)
  // 注意：一个人同一时间只能启动一个任务
  //      断线后，1分钟内，重新连接，可以继续任务
  // 服务端收到请求后，如果当前执行队列已满，会返回失败
  // 如果返回成功，服务端会开始创建相关资源。因此从返回成功，到收到第一个TaskStatus会有一段时间间隔
  Response StartTask(int task_index);

  // 主动关闭当前任务，当前任务作废
  void StopTask();

  // 验证航线是否合法
  // 本地验证，仅仅根据无人机能力，计算航线是否合法
  // 本地验证通过，不代表下发后一定能执行。执行还依赖于当前无人机的状态和位置等信息
  Response ValidateFlightPlan(const DroneLimits& drone_limits,
                              const FlightPlan& flight_plan);

  // 给指定飞机下发航线，根据Response的返回值查看是否下发成功
  // 飞机收到航线后，会根据当前的限制对航线做预先验证，验证失败则不会执行航线
  // 若指定飞机当前为坠毁状态，则相当于无效规划
  // 若当前飞机正在航线中，或者处于悬停状态，则会中止当前航线，并跳过新航线的起飞阶段
  // 若当前处于轨迹飞行阶段，重新规划前最好先下发悬停指令
  // 换电：航线终点必须停在换电站，否则无法换电
  // 装载货物：航线终点必须停止货物起始位置，否则失败
  // 卸载货物：航线终点必须停止在地面，若当前位置不是货物的目标位置，则货物运输失败；
  // 注意：规划系统是个异步系统，在规划的同时，飞机也在执行，需要考虑状态不同步的问题
  Response DronePlanFlight(const std::string& drone_id,
                           const FlightPlan& flight_plan);


  // 通知指定无人机立刻在当前位置悬停
  // 会取消执行当前航线
  // 若当前无航线，则没有变化
  Response DroneHover(const std::string& drone_id);

 protected:
  /// 下面时回调函数，用于通知各项事件
  /// 这些回调函数都会在后台线程调用，注意资源竞争

  // 当前连接断线时，此函数会被调用
  virtual void OnSdkError(std::string error_msg) = 0;

  // 任务状态通知，每一秒一次，或者无人机、货物信息更新时也会调用
  // 此函数不能执行耗时很长的逻辑，建议次函数仅仅用于接受并保存无人机、货物的状态
  // 使用另外的线程，执行主要逻辑（计算航线等）
  // status 包含当前所有无人机的状态
  // cargos 包含当前任务所有货物的状态
  virtual void OnTaskStatus(int task_index, std::vector<DroneStatus> status,
                            std::map<int, CargoInfo> cargos) = 0;

  // 任务完成或者异常时调用
  // 注意，通过调用StopTask主动关闭的任务，此函数不会被调用
  virtual void OnTaskDone(int task_index, bool done, float grade) = 0;

 private:
  class Internal;
  std::unique_ptr<Internal> internal_;

  std::shared_ptr<Map> map_;
  std::string user_;
  std::string key_;
  int running_task_;
};
}

#endif //MTUAV_SDK_PLANNER_H
