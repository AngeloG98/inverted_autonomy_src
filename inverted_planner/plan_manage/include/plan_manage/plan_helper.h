#pragma once

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <bspline_opt/uniform_bspline.h>

using namespace bspline_opt;

namespace inverted_planner{

struct LocalTrajInfo{
    int traj_id_;
    double duration_;
    ros::Time start_time_;
    Eigen::Vector3d start_pos_;
    // Hybird A star path
    vector<Eigen::Vector3d> dynamic_path_;
    UniformBspline position_traj_, velocity_traj_, acceleration_traj_;
};

} // namespace inverted_planner
