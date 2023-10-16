#!/usr/bin/env python
# -*- coding: utf-8 -*-

# 使用方法，目前仅支持python3.10
#  mkdir mtuav_python
#  copy *.so mtuav_python/  # 把拿到的3个so复制到目录下
#  cd mtuav_python
#  export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/abs_path_to[mtuav_python]/   # 导出链接地址，注意使用绝对路径
#  vim pyplanner_test.py  # 编辑python代码，实现自己的算法
#  启动单机版docker
#  python3 pyplanner_test.py # 执行

import pymtuav
import time
import threading

lk = threading.Lock()
current_status = None
current_cargos = None
task_info = None
task_done = False
# saving planning context
plannings_drones = dict()
plannings_cargos = dict()
plannings_flights = dict()


def current_timestamp_ms():
  return int(time.time() * 1000)


def is_collision(plan):
  global plannings_flights
  global current_status
  # TODO: checking collision to other plannings here
  return False


def generate_flight_plan(drone_status, cargo_info):
  #TODO: 这里实现自己的规划算法
  plan = pymtuav.FlightPlan()
  plan.flight_plan_type = pymtuav.FlightPlanType.PLAN_WAY_POINTS
  plan.flight_purpose = pymtuav.FlightPurpose.FLIGHT_TAKE_CARGOS
  ts = current_timestamp_ms()
  plan.flight_id = "%s-%d" % (drone_status.drone_id, ts)
  # taking off timestamp, 10秒后起飞
  plan.takeoff_timestamp = ts + 10 * 1000

  seg1 = pymtuav.Segment()
  seg1.time_ms = 0
  seg1.seg_type = pymtuav.SegmentType.ST_TAKING_OFF
  seg1.position = drone_status.position
  seg2 = pymtuav.Segment()
  seg2.time_ms = 25000
  seg2.seg_type = pymtuav.SegmentType.ST_TAKING_OFF
  seg2.position = drone_status.position
  seg2.position.z = 120
  seg3 = pymtuav.Segment()
  seg3.time_ms = 145000
  seg3.seg_type = pymtuav.SegmentType.ST_FLYING
  seg3.position = cargo_info.position
  seg3.position.z = 120
  seg4 = pymtuav.Segment()
  seg4.time_ms = 170000
  seg4.seg_type = pymtuav.SegmentType.ST_LANDING
  seg4.position = cargo_info.position
  plan.segments.append(seg1)
  plan.segments.append(seg2)
  plan.segments.append(seg3)
  plan.segments.append(seg4)
  plan.target_cargo_ids.append(cargo_info.id)
  return plan


class Planner(pymtuav.PlannerAgent):
  def __int__(self, server_ip, voxel_map):
    pymtuav.PlannerAgent.__init__(server_ip, voxel_map)
    print("build planner: %s" % server_ip)

  def OnSdkError(self, error_msg):
    # may disconnect from SERVER, try it later or yell for help
    print("OnSdkError: %s" % error_msg)

  def OnTaskStatus(self, task_id, status, cargos):
    global current_status
    global current_cargos
    global lk
    print("OnTaskStatus: %d, %d, %d" % (task_id, len(status), len(cargos)))
    with lk:
      current_status = status
      current_cargos = cargos
      # TODO: checking cargo status in plannings_cargos, remove when cargo status changed or timeout(20s?)
      # TODO: checking drone status in plannings_drones, remove when drone status changed or timeout(20s?)

  def OnTaskDone(self, task_id, done, grade):
    global task_done
    # called when task done or all drone crashed
    print("OnTaskDone: %d, %d, %d" % (task_id, done, grade))
    task_done = True

  def select_drone(self):
    global current_status
    global plannings_drones
    global lk
    while lk:
      if current_status is None or len(current_status) == 0:
        return None
      for s in current_status:
        if s.drone_id in plannings_drones:
          continue
        if s.status == pymtuav.Status.READY:
          print("select drone, id: %s" % s.drone_id)
          return s
    # checking drone status here
    return None

  def select_cargo(self):
    global current_cargos
    global plannings_cargos
    global lk
    with lk:
      # select cargos here
      if current_cargos is None or len(current_cargos) == 0:
        return None

      for it in current_cargos.items():
        c = it[1]
        if c.id in plannings_cargos:
          continue
        if c.status == pymtuav.CargoStatus.CARGO_WAITING:
          print("select cargo, id: %d" % c.id)
          return c

    return None

  def planning(self, drone, cargo):
    global plannings_flights
    plan = generate_flight_plan(drone, cargo)
    if is_collision(plan):
      return False

    plannings_flights[drone.drone_id] = plan

    r = self.DronePlanFlight(drone.drone_id, plan)

    if not r.success:
      del plannings_flights[drone.drone_id]

    return r.success

def run():
  global task_info
  global plannings_drones
  global plannings_cargos
  global lk
  voxel_map = pymtuav.Map("/home/marco/planning_map_cloud/voxel_map.bin")
  pl = Planner("127.0.0.1:50051", voxel_map)
  r = pl.Login("801f0ff5-5359-4c3e-99d4-f05d7eb47423", "e57aab02cf1f7433d7bf385748376164")
  task_info = pl.QueryTask(0)
  r = pl.StartTask(0)
  while True:
    time.sleep(0.1)
    cargo = pl.select_cargo()
    if cargo is None:
      continue

    drone = pl.select_drone()
    if drone is None:
      continue

    with lk:
      ts = current_timestamp_ms()
      plannings_drones[drone.drone_id] = ts
      plannings_cargos[cargo.id] = ts

    s = pl.planning(drone, cargo)

    if not s:
      with lk:
        del plannings_drones[drone.drone_id]
        del plannings_cargos[cargo.id]


if __name__ == "__main__":
  run()



