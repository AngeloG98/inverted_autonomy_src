#pragma once

#include <ros/ros.h>
#include <ros/duration.h>
#include <Eigen/Eigen>
#include <flight_msgs/TrajectoryPoint.h>
#include <plan_manage/plan_manager.h>
#include <plan_vis/plan_visualization.h>
#include <plan_env/gridmap.h>
#include <dynamic_search/rotation_astar.h>

#include "std_msgs/Int32.h"
#include <cmath>

namespace inverted_planner
{
class InvertedPlanner{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    InvertedPlanner(const ros::NodeHandle &nh, const ros::NodeHandle &pnh);

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Publisher bspline_pub_;
    std::shared_ptr<inverted_planner::PlanManager> plan_manager_;
    std::shared_ptr<inverted_planner::PlanVisualization> plan_visualizer_;
    std::shared_ptr<inverted_planner::GridMap> global_gridmap_;

    std::shared_ptr<inverted_planner::RotationAstar> rot_astar_;

    ros::Timer plan_timer_;
    ros::Timer plan_vis_timer_;
    ros::Subscriber start_plan_sub_;
    ros::Publisher trajectory_point_pub_;
    ros::Publisher finish_plan_pub_;
    void planPubCallback(const ros::TimerEvent &time);
    void planVisCallback(const ros::TimerEvent &time);
    void startPlanCallback(const std_msgs::Int32 &msg);

    bool go_;
    int count;

    // circle
    double r_;
    double w_dot_;
    double t_;
};

} // namespace inverted_planner


