#pragma once

#include <vector>
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <algorithm>
#include <bspline_opt/uniform_bspline.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

using std::vector;

namespace inverted_planner
{
class PlanVisualization
{
private:
    ros::NodeHandle nh_;

    ros::Publisher control_point_pub_;
    ros::Publisher trajectory_point_pub_;

public:
    PlanVisualization() {}
    ~PlanVisualization() {}
    PlanVisualization(ros::NodeHandle &nh);

    void displayMarkerList(ros::Publisher &pub, const vector<Eigen::Vector3d> &list, double scale,
                            Eigen::Vector4d color, int id);

    void displayControlPointList(Eigen::MatrixXd ctrl_pts, int id);
    void displayTrajectoryPointList(Eigen::MatrixXd ctrl_pts, int id);
};
} // namespace inverted_planner
