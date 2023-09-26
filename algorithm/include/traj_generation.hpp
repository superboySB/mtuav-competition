#ifndef TRAJ_GENERATION_HPP
#define TRAJ_GENERATION_HPP

#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include "mtuav_sdk_types.h"


struct WaypointAccInfo {
    double total_seconds = 0.0;

    double t1 = 0.0;
    double v1 = 0.0;
    double a1 = 0.0;
    double delta_s1 = 0.0;

    double t2 = 0.0;
    double v2 = 0.0;
    double a2 = 0.0;
    double delta_s2 = 0.0;

    double t3 = 0.0;
    double v3 = 0.0;
    double a3 = 0.0;
    double delta_s3 = 0.0;

    WaypointAccInfo() {}

    void show() {
        std::cout << std::fixed << std::setprecision(10) << "delta_s1 " << delta_s1 << " v1 " << v1
                  << " a1 " << a1 << " t1 " << t1 << "\ndelta_s2 " << delta_s2 << " v2 " << v2
                  << " a2 " << a2 << " t2 " << t2 << "\ndelta_s3 " << delta_s3 << " v3 " << v3
                  << " a3 " << a3 << " t3 " << t3 << "\n total time " << total_seconds << std::endl;
    }
};

struct WaypointDistanceInfo {
    double vel = 0.0;
    double acc = 0.0;
    double travel_dist = 0.0;
    void show() {
        std::cout << " sample: p " << travel_dist << " vel " << vel << " acc " << acc << std::endl;
    }
};

class TrajectoryGeneration {
   public:
    TrajectoryGeneration() = default;
    ~TrajectoryGeneration() = default;

   public:
    // 根据两个首尾waypoint和约束，生成无人机轨迹  
    // int status = (0:垂直起飞段， 1:自由飞行阶段 ， 2:垂直降落段)
    bool generate_traj_from_waypoints(const std::vector<mtuav::Vec3>& waypoints,
                                      const mtuav::DroneLimits& limits, int status,
                                      std::vector<mtuav::Segment>& segments) {
        int sample_buffer = 0;
        int sample_index = 0;
        if (waypoints.size() < 2) {
            std::cout << " waypoint size < 2 " << std::endl;
            return false;
        }
        std::cout << "Path: " << std::endl;
        for (const auto& waypoint : waypoints) {
            std::cout << std::fixed << std::setprecision(2) << waypoint.x << " " << waypoint.y
                      << " " << waypoint.z << std::endl;
        }

        mtuav::Vec3 previous, current;
        std::vector<mtuav::Segment> single_segments;
        for (size_t i = 1; i < waypoints.size(); ++i) {
            single_segments.clear();
            previous = waypoints[i - 1];
            current = waypoints[i];
            if (!generate_traj(previous, current, limits, single_segments, status, sample_buffer,
                               sample_index)) {
                std::cout << "generate fail between " << i << " and " << i + 1 << std::endl;
                return false;
            }

            segments.insert(segments.end(), single_segments.begin(), single_segments.end());
        }

        if (sample_buffer != 0) {
            mtuav::Segment segment_point;
            segment_point.position = waypoints.back();
            segment_point.v.x = 0.0;
            segment_point.v.y = 0.0;
            segment_point.v.z = 0.0;
            segment_point.a.x = 0.0;
            segment_point.a.y = 0.0;
            segment_point.a.z = 0.0;

            segment_point.seg_type = status;
            segment_point.time_ms = segments.back().time_ms + _sample_step - sample_buffer;
            segments.emplace_back(segment_point);
        }

        return true;
    }

    // 2. 采样
   private:
    bool generate_traj(const mtuav::Vec3& previous, const mtuav::Vec3& current,
                       const mtuav::DroneLimits& limits, std::vector<mtuav::Segment>& segments,
                       int status, int& sample_buffer, int& sample_index) {
        double delta_p_x = current.x - previous.x;
        double delta_p_y = current.y - previous.y;
        double p_horizontal = std::sqrt(delta_p_x * delta_p_x + delta_p_y * delta_p_y);
        double delta_p_z = current.z - previous.z;
        double p = std::sqrt(delta_p_z * delta_p_z + delta_p_x * delta_p_x + delta_p_y * delta_p_y);

        if (std::fabs(delta_p_x) < 1e-6 && std::fabs(delta_p_y) < 1e-6 &&
            std::fabs(delta_p_z) < 1e-6) {
            return false;
        }

        double ratio_z = std::fabs(delta_p_z) / p;
        double ratio_hor = p_horizontal / p;

        double max_v, max_a;
        if (std::fabs(delta_p_z) < 1e-6 || ratio_z < 1e-6) {
            max_v = limits.max_fly_speed_h;
            max_a = limits.max_fly_acc_h;
        } else if (std::fabs(p_horizontal) < 1e-6 || ratio_hor < 1e-6) {
            max_v = limits.max_fly_speed_v;
            max_a = limits.max_fly_acc_v;
        } else {
            max_v = std::min(limits.max_fly_speed_h / ratio_hor, limits.max_fly_speed_v / ratio_z);
            max_a = std::min(limits.max_fly_acc_h / ratio_hor, limits.max_fly_acc_v / ratio_z);
        }

        std::cout << "Input p " << p << " max_v " << max_v << " max_a " << max_a << std::endl;
        WaypointAccInfo hor_info = generate_traj_1d(p, max_v, max_a);

        segments.clear();
        mtuav::Segment single_segment;
        int t = sample_buffer;
        std::cout << std::fixed << std::setprecision(10) << "total seconds "
                  << hor_info.total_seconds << std::endl;
        for (t; t < std::floor(hor_info.total_seconds * 1e3); t += _sample_step) {
            WaypointDistanceInfo dis_info = sample_traj(hor_info, t * 1e-3);
            mtuav::Vec3 delta_p = get_3d_from_1d(previous, current, dis_info.travel_dist);
            single_segment.position.x = delta_p.x + previous.x;
            single_segment.position.y = delta_p.y + previous.y;
            single_segment.position.z = delta_p.z + previous.z;

            single_segment.v = get_3d_from_1d(previous, current, dis_info.vel);
            single_segment.a = get_3d_from_1d(previous, current, dis_info.acc);
            single_segment.time_ms = sample_index * _sample_step;
            single_segment.seg_type = status;
            segments.emplace_back(single_segment);
            sample_index++;
        }

        sample_buffer = t - std::floor(hor_info.total_seconds * 1e3);

        return true;
    }

    mtuav::Vec3 get_3d_from_1d(const mtuav::Vec3& previous, const mtuav::Vec3& current,
                               double delta) {
        double delta_p_x = current.x - previous.x;
        double delta_p_y = current.y - previous.y;
        double delta_p_z = current.z - previous.z;
        double p = std::sqrt(delta_p_z * delta_p_z + delta_p_x * delta_p_x + delta_p_y * delta_p_y);

        mtuav::Vec3 point;
        point.x = delta_p_x / p * delta;
        point.y = delta_p_y / p * delta;
        point.z = delta_p_z / p * delta;
        return point;
    }

    WaypointAccInfo generate_traj_1d(double p, double max_v, double max_a) {
        WaypointAccInfo hor_info;

        // 三段式
        if (max_v * max_v / max_a < p) {
            hor_info.t1 = max_v / max_a;
            hor_info.a1 = max_a;
            hor_info.v1 = 0;
            hor_info.delta_s1 = 0.5 * max_v * max_v / max_a;

            hor_info.t2 = (p - max_v * max_v / max_a) / max_v;
            hor_info.a2 = 0;
            hor_info.v2 = max_v;
            hor_info.delta_s2 = p - max_v * max_v / max_a;

            hor_info.t3 = hor_info.t1;
            hor_info.a3 = -max_a;
            hor_info.v3 = max_v;
            hor_info.delta_s3 = hor_info.delta_s1;
        }
        // 两段式
        else {
            hor_info.t1 = std::sqrt(p / max_a);
            hor_info.a1 = max_a;
            hor_info.v1 = 0;
            hor_info.delta_s1 = p * 0.5;

            hor_info.t2 = 0;
            hor_info.a2 = 0;
            hor_info.v2 = hor_info.v1 + hor_info.a1 * hor_info.t1;
            hor_info.delta_s2 = 0;

            hor_info.t3 = hor_info.t1;
            hor_info.a3 = -max_a;
            hor_info.v3 = hor_info.v2;
            hor_info.delta_s3 = hor_info.delta_s1;
        }

        hor_info.total_seconds = hor_info.t1 + hor_info.t2 + hor_info.t3;
        hor_info.show();
        return hor_info;
    }

    WaypointDistanceInfo sample_traj(const WaypointAccInfo& info, double time) {
        WaypointDistanceInfo dis_info;
        if (time <= info.t1) {
            dis_info.acc = info.a1;
            dis_info.vel = info.a1 * time;
            dis_info.travel_dist = 0.5 * info.a1 * time * time;
        } else if (time <= info.t1 + info.t2) {
            dis_info.acc = 0;
            dis_info.vel = info.v2;
            dis_info.travel_dist = info.delta_s1 + info.v2 * (time - info.t1);
        } else if (time <= info.total_seconds) {
            dis_info.acc = info.a3;
            dis_info.vel = info.v2 + info.a3 * (time - info.t1 - info.t2);
            dis_info.travel_dist =
                info.delta_s1 + info.delta_s2 + info.v3 * (time - info.t1 - info.t2) +
                0.5 * info.a3 * (time - info.t1 - info.t2) * (time - info.t1 - info.t2);
        } else {
            dis_info.acc = 0;
            dis_info.vel = 0;
            dis_info.travel_dist = info.delta_s1 + info.delta_s2 + info.delta_s3;
        }
        dis_info.show();
        return dis_info;
    }

   private:
    int _sample_step = 100;  // ms
};


#endif