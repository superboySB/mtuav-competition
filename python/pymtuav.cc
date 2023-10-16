//
// Created by marco on 23-10-9.
//
#include <pybind11/pybind11.h>
#include <pybind11/stl_bind.h>

using namespace pybind11::literals;

#include <map>
#include <memory>
#include <vector>

#include "mtuav_sdk_map.h"
#define protected public
#include "mtuav_sdk_planner.h"
#undef protected
#include "mtuav_sdk_types.h"

namespace py = pybind11;

PYBIND11_MAKE_OPAQUE(std::vector<int>);
PYBIND11_MAKE_OPAQUE(std::vector<mtuav::Segment>);
PYBIND11_MAKE_OPAQUE(std::vector<mtuav::DroneInfo>);
PYBIND11_MAKE_OPAQUE(std::vector<mtuav::Vec3>);
PYBIND11_MAKE_OPAQUE(std::vector<mtuav::ObstacleInfo>);
PYBIND11_MAKE_OPAQUE(std::vector<mtuav::DroneStatus>);
PYBIND11_MAKE_OPAQUE(std::map<int, mtuav::CargoInfo>);

namespace pymtuav {
class Map {
 public:
  explicit Map(std::string map_path)
      : map_(mtuav::Map::CreateMapFromFile(std::move(map_path))),
        max_x_(0), max_y_(0), max_z_(0) {
  }

  const mtuav::Voxel* Query(float x, float y, float z) {
    return map_->Query(x, y, z);
  }

  bool LoadRange() {
    if (map_) {
      float min_x, min_y, min_z;
      map_->Range(&min_x, &max_x_, &min_y, &max_y_, &min_z, &max_z_);
      return true;
    }

    return false;
  }

  static std::shared_ptr<Map> CreateMap(std::string map_path) {
    std::shared_ptr<Map> r(new Map(std::move(map_path)));
    if (r->LoadRange()) {
      return r;
    }
    return {};
  }

  float max_x_;
  float max_y_;
  float max_z_;

  std::shared_ptr<mtuav::Map> map_;
};

class PlannerProxy;

class Planner : public mtuav::PlannerAgent {
 public:
  using mtuav::PlannerAgent::PlannerAgent;

  void OnSdkError(std::string error_msg) override;
  void OnTaskStatus(int task_index, std::vector<mtuav::DroneStatus> status,
                    std::map<int, mtuav::CargoInfo> cargos) override;
  void OnTaskDone(int task_index, bool done, float grade) override;
  PlannerProxy* proxy_;
};

class PlannerProxy {
 public:
  PlannerProxy(std::string server_addr, std::shared_ptr<Map> map)
      : planner_(new Planner(std::move(server_addr), map->map_, "")) {
    planner_->proxy_ = this;
  }
  virtual ~PlannerProxy() = default;

  mtuav::Response Login(std::string username, std::string password) {
    return planner_->Login(std::move(username), std::move(password));
  }

  int GetTaskCount() {
    return planner_->GetTaskCount();
  }

  std::unique_ptr<mtuav::TaskInfo> QueryTask(int task_index) {
    return planner_->QueryTask(task_index);
  }

  mtuav::Response StartTask(int task_index) {
    return planner_->StartTask(task_index);
  }

  void StopTask() {
    planner_->StopTask();
  }

  mtuav::Response ValidateFlightPlan(const mtuav::DroneLimits& drone_limits,
                                     const mtuav::FlightPlan& flight_plan) {
    return planner_->ValidateFlightPlan(drone_limits, flight_plan);
  }

  mtuav::Response DronePlanFlight(const std::string& drone_id,
                                  const mtuav::FlightPlan& flight_plan) {
    return planner_->DronePlanFlight(drone_id, flight_plan);
  }
  mtuav::Response DroneHover(const std::string& drone_id) {
    return planner_->DroneHover(drone_id);
  }

  virtual void OnSdkError(std::string error_msg) = 0;
  virtual void OnTaskStatus(int task_index, std::vector<mtuav::DroneStatus> status,
                            std::map<int, mtuav::CargoInfo> cargos) = 0;
  virtual void OnTaskDone(int task_index, bool done, float grade) = 0;
 private:
  std::unique_ptr<Planner> planner_;
};

void Planner::OnSdkError(std::string error_msg) {
  proxy_->OnSdkError(std::move(error_msg));
}
void Planner::OnTaskStatus(int task_index,
                           std::vector<mtuav::DroneStatus> status,
                           std::map<int, mtuav::CargoInfo> cargos) {
  proxy_->OnTaskStatus(task_index, std::move(status), std::move(cargos));
}
void Planner::OnTaskDone(int task_index, bool done, float grade) {
  proxy_->OnTaskDone(task_index, done, grade);
}

class PyPlanner : public PlannerProxy {
 public:
  using PlannerProxy::PlannerProxy;

  void OnSdkError(std::string error_msg) override {
    PYBIND11_OVERRIDE_PURE(
        void,
        PlannerProxy,
        OnSdkError);
  }

  void OnTaskStatus(int task_index, std::vector<mtuav::DroneStatus> status,
                    std::map<int, mtuav::CargoInfo> cargos) override {
    PYBIND11_OVERRIDE_PURE(
        void,
        PlannerProxy,
        OnTaskStatus,
        task_index,
        status,
        cargos);
  }

  void OnTaskDone(int task_index, bool done, float grade) override {
    PYBIND11_OVERRIDE_PURE(
        void,
        PlannerProxy,
        OnTaskDone,
        task_index,
        done,
        grade);
  }
};

enum SegmentType {
  SEG_TAKING_OFF = 0,
  SEG_FLYING = 1,
  SEG_LANDING = 2,
};
}

PYBIND11_MODULE(pymtuav, m) {
  m.doc() = "python binding for mtuav_sdk";
  m.attr("ST_TAKING_OFF") = 0;
  m.attr("ST_FLYING") = 1;
  m.attr("ST_LANDING") = 2;

  py::bind_vector<std::vector<int>>(m, "IntVector");
  py::bind_vector<std::vector<mtuav::Segment>>(m, "SegmentVector");
  py::bind_vector<std::vector<mtuav::DroneInfo>>(m, "DroneInfoVector");
  py::bind_vector<std::vector<mtuav::Vec3>>(m, "Vec3Vector");
  py::bind_vector<std::vector<mtuav::ObstacleInfo>>(m, "ObstacleInfoVector");
  py::bind_vector<std::vector<mtuav::DroneStatus>>(m, "DroneStatusVector");
  py::bind_map<std::map<int, mtuav::CargoInfo>>(m, "CargoInfoMap");

  py::enum_<pymtuav::SegmentType>(m, "SegmentType")
      .value("SEG_TAKING_OFF", pymtuav::SEG_TAKING_OFF)
      .value("SEG_FLYING", pymtuav::SEG_FLYING)
      .value("SEG_LANDING", pymtuav::SEG_LANDING);

  py::enum_<mtuav::CargoStatus>(m, "CargoStatus")
      .value("CARGO_UNKNOWN", mtuav::CARGO_UNKNOWN)
      .value("CARGO_WAITING", mtuav::CARGO_WAITING)
      .value("CARGO_DELIVERING", mtuav::CARGO_DELIVERING)
      .value("CARGO_DELIVERED", mtuav::CARGO_DELIVERED)
      .value("CARGO_FAILED", mtuav::CARGO_FAILED);

  py::enum_<mtuav::FlightPurpose>(m, "FlightPurpose")
      .value("FLIGHT_COMMON", mtuav::FLIGHT_COMMON)
      .value("FLIGHT_TAKE_CARGOS", mtuav::FLIGHT_TAKE_CARGOS)
      .value("FLIGHT_DELIVER_CARGOS", mtuav::FLIGHT_DELIVER_CARGOS)
      .value("FLIGHT_EXCHANGE_BATTERY", mtuav::FLIGHT_EXCHANGE_BATTERY);

  py::enum_<mtuav::FlightPlanType>(m, "FlightPlanType")
      .value("PLAN_WAY_POINTS", mtuav::PLAN_WAY_POINTS)
      .value("PLAN_TRAJECTORIES", mtuav::PLAN_TRAJECTORIES);

  py::enum_<mtuav::Status>(m, "Status")
      .value("READY", mtuav::READY)
      .value("TAKING_OFF", mtuav::TAKING_OFF)
      .value("FLYING", mtuav::FLYING)
      .value("HOVERING", mtuav::HOVERING)
      .value("LANDING", mtuav::LANDING)
      .value("CRASH", mtuav::CRASH)
      .value("READY_TO_FLY", mtuav::READY_TO_FLY);

  py::enum_<mtuav::ObstacleType>(m, "ObstacleType")
      .value("OBSTACLE_UNKNOWN", mtuav::OBSTACLE_UNKNOWN)
      .value("OBSTACLE_DRONE", mtuav::OBSTACLE_DRONE)
      .value("OBSTACLE_BIRD", mtuav::OBSTACLE_BIRD)
      .value("OBSTACLE_KITE", mtuav::OBSTACLE_KITE);

  py::enum_<mtuav::DroneCrashType>(m, "DroneCrashType")
      .value("DRONE_CRASH_NONE", mtuav::DRONE_CRASH_NONE)
      .value("DRONE_CRASH_LOW_BATTERY", mtuav::DRONE_CRASH_LOW_BATTERY)
      .value("DRONE_CRASH_COLLIDE_OBSTACLE",
             mtuav::DRONE_CRASH_COLLIDE_OBSTACLE)
      .value("DRONE_CRASH_COLLIDE_DRONE", mtuav::DRONE_CRASH_COLLIDE_DRONE)
      .value("DRONE_CRASH_LANDED_ON_ILLEGAL_POSITION",
             mtuav::DRONE_CRASH_LANDED_ON_ILLEGAL_POSITION);

  py::class_<mtuav::Voxel>(m, "Voxel")
      .def(py::init<>())
      .def_readonly("distance", &mtuav::Voxel::distance)
      .def_readonly("semantic", &mtuav::Voxel::semantic);

  py::class_<pymtuav::Map, std::shared_ptr<pymtuav::Map>>(m, "Map")
      .def(py::init(&pymtuav::Map::CreateMap))
      .def("query", &pymtuav::Map::Query)
      .def_readonly("max_x", &pymtuav::Map::max_x_)
      .def_readonly("max_y", &pymtuav::Map::max_y_)
      .def_readonly("max_z", &pymtuav::Map::max_z_);

  py::class_<mtuav::Vec3>(m, "Vec3")
      .def(py::init<>())
      .def_readwrite("x", &mtuav::Vec3::x)
      .def_readwrite("y", &mtuav::Vec3::y)
      .def_readwrite("z", &mtuav::Vec3::z)
      .def("__repr__",
           [](const mtuav::Vec3 &a) {
             return "x: " + std::to_string(a.x) + ", y: " + std::to_string(a.y)
                 + ", z: " + std::to_string(a.z);
           }
      );

  py::class_<mtuav::Segment>(m, "Segment")
      .def(py::init<>())
      .def_readwrite("time_ms", &mtuav::Segment::time_ms)
      .def_readwrite("seg_type", &mtuav::Segment::seg_type)
      .def_readwrite("position", &mtuav::Segment::position)
      .def_readwrite("v", &mtuav::Segment::v)
      .def_readwrite("a", &mtuav::Segment::a);

  py::class_<mtuav::DroneLimits>(m, "DroneLimits")
      .def(py::init<>())
      .def_readonly("max_fly_speed_v", &mtuav::DroneLimits::max_fly_speed_v)
      .def_readonly("max_fly_speed_h", &mtuav::DroneLimits::max_fly_speed_h)
      .def_readonly("max_fly_acc_v", &mtuav::DroneLimits::max_fly_acc_v)
      .def_readonly("max_fly_acc_h", &mtuav::DroneLimits::max_fly_acc_h)
      .def_readonly("max_weight", &mtuav::DroneLimits::max_weight)
      .def_readonly("max_cargo_slots", &mtuav::DroneLimits::max_cargo_slots)
      .def_readonly("min_fly_height", &mtuav::DroneLimits::min_fly_height)
      .def_readonly("max_fly_height", &mtuav::DroneLimits::max_fly_height)
      .def_readonly("max_flight_seconds",
                    &mtuav::DroneLimits::max_flight_seconds);

  py::class_<mtuav::CargoInfo>(m, "CargoInfo")
      .def(py::init<>())
      .def_readonly("id", &mtuav::CargoInfo::id)
      .def_readonly("status", &mtuav::CargoInfo::status)
      .def_readonly("position", &mtuav::CargoInfo::position)
      .def_readonly("weight", &mtuav::CargoInfo::weight)
      .def_readonly("award", &mtuav::CargoInfo::award)
      .def_readonly("target_position", &mtuav::CargoInfo::target_position)
      .def_readonly("expected_seconds_left",
                    &mtuav::CargoInfo::expected_seconds_left)
      .def_readonly("latest_seconds_left",
                    &mtuav::CargoInfo::latest_seconds_left);

  py::class_<mtuav::DroneInfo>(m, "DroneInfo")
      .def(py::init<>())
      .def_readonly("drone_id", &mtuav::DroneInfo::drone_id)
      .def_readonly("initial_pos", &mtuav::DroneInfo::initial_pos)
      .def_readonly("drone_limits", &mtuav::DroneInfo::drone_limits);

  py::class_<mtuav::TaskInfo>(m, "TaskInfo")
      .def(py::init<>())
      .def_readonly("task_id", &mtuav::TaskInfo::task_id)
      .def_readonly("map_id", &mtuav::TaskInfo::map_id)
      .def_readonly("drones", &mtuav::TaskInfo::drones)
      .def_readonly("battery_consuming", &mtuav::TaskInfo::battery_consuming)
      .def_readonly("has_obstacles", &mtuav::TaskInfo::has_obstacles)
      .def_readonly("battery_stations", &mtuav::TaskInfo::battery_stations)
      .def_readonly("landing_positions", &mtuav::TaskInfo::landing_positions);

  py::class_<mtuav::ObstacleInfo>(m, "ObstacleInfo")
      .def(py::init<>())
      .def_readonly("position", &mtuav::ObstacleInfo::position)
      .def_readonly("velocity", &mtuav::ObstacleInfo::velocity)
      .def_readonly("radius", &mtuav::ObstacleInfo::radius)
      .def_readonly("obstacle_type", &mtuav::ObstacleInfo::obstacle_type);

  py::class_<mtuav::Response>(m, "Response")
      .def(py::init<>())
      .def_readonly("success", &mtuav::Response::success)
      .def_readonly("msg", &mtuav::Response::msg);

  py::class_<mtuav::DroneStatus>(m, "DroneStatus")
      .def(py::init<>())
      .def_readonly("drone_id", &mtuav::DroneStatus::drone_id)
      .def_readonly("timestamp", &mtuav::DroneStatus::timestamp)
      .def_readonly("status", &mtuav::DroneStatus::status)
      .def_readonly("position", &mtuav::DroneStatus::position)
      .def_readonly("height", &mtuav::DroneStatus::height)
      .def_readonly("delivering_cargo_ids",
                    &mtuav::DroneStatus::delivering_cargo_ids)
      .def_readonly("battery", &mtuav::DroneStatus::battery)
      .def_readonly("detected_obstacles",
                    &mtuav::DroneStatus::detected_obstacles)
      .def_readonly("crash_type", &mtuav::DroneStatus::crash_type);

  py::class_<mtuav::FlightPlan>(m, "FlightPlan")
      .def(py::init<>())
      .def_readwrite("flight_plan_type", &mtuav::FlightPlan::flight_plan_type)
      .def_readwrite("flight_purpose", &mtuav::FlightPlan::flight_purpose)
      .def_readwrite("flight_id", &mtuav::FlightPlan::flight_id)
      .def_readwrite("takeoff_timestamp", &mtuav::FlightPlan::takeoff_timestamp)
      .def_readwrite("segments", &mtuav::FlightPlan::segments)
      .def_readwrite("target_cargo_ids", &mtuav::FlightPlan::target_cargo_ids);

  py::class_<pymtuav::PlannerProxy, pymtuav::PyPlanner>(m, "PlannerAgent")
      .def(py::init<const std::string&, std::shared_ptr<pymtuav::Map>>())
      .def("Login", &pymtuav::PlannerProxy::Login)
      .def("GetTaskCount", &pymtuav::PlannerProxy::GetTaskCount)
      .def("QueryTask", &pymtuav::PlannerProxy::QueryTask)
      .def("StartTask", &pymtuav::PlannerProxy::StartTask)
      .def("StopTask", &pymtuav::PlannerProxy::StopTask)
      .def("ValidateFlightPlan", &pymtuav::PlannerProxy::ValidateFlightPlan)
      .def("DronePlanFlight", &pymtuav::PlannerProxy::DronePlanFlight)
      .def("DroneHover", &pymtuav::PlannerProxy::DroneHover)
      .def("OnSdkError", &pymtuav::PlannerProxy::OnSdkError)
      .def("OnTaskStatus", &pymtuav::PlannerProxy::OnTaskStatus)
      .def("OnTaskDone", &pymtuav::PlannerProxy::OnTaskDone);
}