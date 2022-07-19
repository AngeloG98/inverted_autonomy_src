#include <plan_vis/plan_visualization.h>

namespace inverted_planner
{
PlanVisualization::PlanVisualization(ros::NodeHandle &nh)
{
    nh_ = nh;
    control_point_pub_ = nh_.advertise<visualization_msgs::Marker>("/plan_vis/control_point", 2);
    trajectory_point_pub_ = nh_.advertise<visualization_msgs::Marker>("/plan_vis/trajectory_point", 2);
    astar_sample_point_pub_ = nh_.advertise<visualization_msgs::Marker>("/plan_vis/astar_sample_point", 2);
    astar_sample_arrow_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/plan_vis/astar_sample_arrow", 2);
    astar_exp_point_pub_ = nh_.advertise<visualization_msgs::Marker>("/plan_vis/astar_exp_point", 2);
}


void PlanVisualization::displayMarkerList(ros::Publisher &pub, const vector<Eigen::Vector3d> &list, double scale,
                                            Eigen::Vector4d color, int id, bool display_info)
{
    /* display_info : true for both sphere and line
                      false for sphere only */
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
    if (display_info){
        pub.publish(sphere);
        pub.publish(line_strip);
    } else {
        pub.publish(sphere);
    }
}

void PlanVisualization::displayArrowMarkerList(ros::Publisher &pub, const vector<Eigen::Vector3d> &pt_list,
                                               const vector<Eigen::Vector3d> &force_list, double scale,
                                               Eigen::Vector4d color, int id){
    visualization_msgs::MarkerArray arrow_list;
    visualization_msgs::Marker arrow;
    arrow.type = visualization_msgs::Marker::ARROW;
    arrow.header.frame_id = "map";
    arrow.header.stamp = ros::Time::now();
    arrow.action = visualization_msgs::Marker::ADD;
    arrow.scale.x = scale;
    arrow.scale.y = 2*scale;
    arrow.scale.z = 0;
    arrow.color.r = color(0);
    arrow.color.g = color(1);
    arrow.color.b = color(2);
    arrow.color.a = color(3) > 1e-5 ? color(3) : 1.0;
    arrow.pose.orientation.w = 1.0;
    for (int i = 0; i < pt_list.size(); i++)
    {
        arrow.points.clear();
        geometry_msgs::Point p_i, p_f;
        double norm_fz;
   
        norm_fz = force_list[i].norm();
        p_i.x = pt_list[i](0);
        p_i.y = pt_list[i](1);
        p_i.z = pt_list[i](2);
        p_f.x = p_i.x + force_list[i](0)/20.0;
        p_f.y = p_i.y + force_list[i](1)/20.0;
        p_f.z = p_i.z + force_list[i](2)/20.0;
        arrow.points.push_back(p_i);
        arrow.points.push_back(p_f);
        arrow_list.markers.push_back(arrow);
        arrow.id++;
    }
    pub.publish(arrow_list);
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
    Eigen::Vector4d color(0, 0, 1, 1);
    displayMarkerList(control_point_pub_, list, 0.15, color, id, true);
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
    Eigen::Vector4d color(1, 0, 0, 1);
    displayMarkerList(trajectory_point_pub_, list, 0.05, color, id, true);
}

void PlanVisualization::displayAstarSamplePointList(vector<Eigen::Vector3d> astar_sample_pts,
                                                    vector<Eigen::Vector3d> astar_sample_fzs, int id)
{
    Eigen::Vector4d color(1, 1, 0, 1);
    displayMarkerList(astar_sample_point_pub_, astar_sample_pts, 0.1, color, id, true);
    Eigen::Vector4d color_arr(0, 1, 0, 1);
    displayArrowMarkerList(astar_sample_arrow_pub_, astar_sample_pts, astar_sample_fzs, 0.02, color_arr, id + 500);
}

void PlanVisualization::displayAstarExpandedPointList(vector<Eigen::Vector3d> astar_exp_pts, int id){
    Eigen::Vector4d color(153.0 / 255.0, 50.0 / 255.0, 204.0 / 255.0, 1);
    displayMarkerList(astar_exp_point_pub_, astar_exp_pts, 0.05, color, id, false);
}

} // namespace inverted_planner