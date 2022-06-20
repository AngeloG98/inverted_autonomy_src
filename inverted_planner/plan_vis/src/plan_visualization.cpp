#include <plan_vis/plan_visualization.h>

namespace inverted_planner
{
PlanVisualization::PlanVisualization(ros::NodeHandle &nh)
{
    nh_ = nh;
    control_point_pub_ = nh_.advertise<visualization_msgs::Marker>("/plan_vis/control_point", 2);
    trajectory_point_pub_ = nh_.advertise<visualization_msgs::Marker>("/plan_vis/trajectory_point", 2);
    astar_sample_point_pub_ = nh_.advertise<visualization_msgs::Marker>("/plan_vis/astar_sample_point", 2);
}

// sphere and line
void PlanVisualization::displayMarkerList(ros::Publisher &pub, const vector<Eigen::Vector3d> &list, double scale,
                                            Eigen::Vector4d color, int id)
{
    visualization_msgs::Marker sphere, line_strip;
    sphere.header.frame_id = line_strip.header.frame_id = "map";
    sphere.header.stamp = line_strip.header.stamp = ros::Time::now();
    sphere.type = visualization_msgs::Marker::SPHERE_LIST;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    sphere.action = line_strip.action = visualization_msgs::Marker::ADD;
    sphere.id = id;
    line_strip.id = id + 1000;

    sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
    sphere.color.r = line_strip.color.r = color(0);
    sphere.color.g = line_strip.color.g = color(1);
    sphere.color.b = line_strip.color.b = color(2);
    sphere.color.a = line_strip.color.a = color(3) > 1e-5 ? color(3) : 1.0;
    sphere.scale.x = scale;
    sphere.scale.y = scale;
    sphere.scale.z = scale;
    line_strip.scale.x = scale / 2;
    geometry_msgs::Point pt;
    for (int i = 0; i < int(list.size()); i++)
    {
        pt.x = list[i](0);
        pt.y = list[i](1);
        pt.z = list[i](2);
        sphere.points.push_back(pt);
        line_strip.points.push_back(pt);
    }
    
    pub.publish(sphere);
    pub.publish(line_strip);
}


void PlanVisualization::displayControlPointList(Eigen::MatrixXd ctrl_pts, int id)
{
    // if (control_point_pub_.getNumSubscribers() == 0)
    // {
    //     return;
    // }
    
    vector<Eigen::Vector3d> list;
    for (int i = 0; i < ctrl_pts.cols(); i++)
    {
        Eigen::Vector3d pt = ctrl_pts.col(i).transpose();
        list.push_back(pt);
    }
    Eigen::Vector4d color(1, 0, 0, 1);
    displayMarkerList(control_point_pub_, list, 0.15, color, id);
}

void PlanVisualization::displayTrajectoryPointList(Eigen::MatrixXd traj_pts, int id)
{
    // if (control_point_pub_.getNumSubscribers() == 0)
    // {
    //     return;
    // }
    
    vector<Eigen::Vector3d> list;
    for (int i = 0; i < traj_pts.cols(); i++)
    {
        Eigen::Vector3d pt = traj_pts.col(i).transpose();
        list.push_back(pt);
    }
    Eigen::Vector4d color(0, 1, 0, 1);
    displayMarkerList(trajectory_point_pub_, list, 0.05, color, id);
}

void PlanVisualization::displayAstarSamplePointList(vector<Eigen::Vector3d> astar_sample_pts, int id){
    Eigen::Vector4d color(1, 1, 0, 1);
    displayMarkerList(astar_sample_point_pub_, astar_sample_pts, 0.05, color, id);
}

} // namespace inverted_planner