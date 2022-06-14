#include <plan_manage/inverted_planner.h>

namespace inverted_planner
{
InvertedPlanner::InvertedPlanner(const ros::NodeHandle &nh,
                        const ros::NodeHandle &pnh)
        :nh_(nh), pnh_(pnh){
    plan_manager_ = std::make_shared<inverted_planner::PlanManager>();
    plan_visualizer_ = std::make_shared<inverted_planner::PlanVisualization>(nh_);
    global_gridmap_ = std::make_shared<inverted_planner::GridMap>(nh_);

    rot_astar_ = std::make_shared<inverted_planner::RotationAstar>();
    rot_astar_->setGridMap(global_gridmap_);
    rot_astar_->init();

    start_plan_sub_ = nh_.subscribe("trigger/bspline_plan", 1, &InvertedPlanner::startPlanCallback, this);
    trajectory_point_pub_ = nh_.advertise<flight_msgs::TrajectoryPoint>("trajectory_points/bspline", 1);
    plan_timer_ = nh_.createTimer(ros::Duration(1.0 / 300), &InvertedPlanner::planPubCallback, this);
    // plan_vis_timer_ = nh_.createTimer(ros::Duration(1.0 / 5), &InvertedPlanner::planVisCallback, this);


    // cubic b-spline
    Eigen::MatrixXd control_points(3,14);
    double ts = 10.0;
    for (int i = 0; i < 3; i++)
    {
        control_points(0, i) = 0.0;
        control_points(1, i) = 0.0;
        control_points(2, i) = -20.0;
    }
    for (int i = 3; i < 6; i++)
    {
        control_points(0, i) = (i-2)*ts;
        control_points(1, i) = 0.0;
        control_points(2, i) = -20.0 + (i-2)*ts/2;
    }
    control_points(0, 6) = (5-2)*ts + 20;
    control_points(1, 6) = 30;
    control_points(2, 6) = -20.0;

    control_points(0, 7) = (5-2)*ts + 40;
    control_points(1, 7) = 30;
    control_points(2, 7) = -20.0;

    control_points(0, 8) = (5-2)*ts + 60;
    control_points(1, 8) = 5;
    control_points(2, 8) = -20.0;

    control_points(0, 9) = (5-2)*ts + 80;
    control_points(1, 9) = -10.0;
    control_points(2, 9) = -20.0- 10.0;

    control_points(0, 10) = (5-2)*ts + 80;
    control_points(1, 10) = -25.0;
    control_points(2, 10) = -20.0 - 10.0;

    for (int i = 11; i < (11+3); i++)
    {
        control_points(0, i) = (5-2)*ts + 50;
        control_points(1, i) = -25.0;
        control_points(2, i) = -20.0 - 10.0;
    }
    plan_manager_->cubicBsplineFromControlPoint(control_points, ts);
    
}
void InvertedPlanner::startPlanCallback(const std_msgs::Int32 &msg){
    if (msg.data == 1){
        go = true;
    }
}

void InvertedPlanner::planVisCallback(const ros::TimerEvent &time){

    double T = plan_manager_->local_traj_.position_traj_.getTimeSum();
    double sample_rate = 1;
    const int num = T * sample_rate;

    // visualize control points
    plan_visualizer_->displayControlPointList(plan_manager_->local_traj_.position_traj_.get_control_points(), 0);
    // visualize trajectory points
    Eigen::MatrixXd trajectory_points(3, num);
    for (int i = 0; i < T * sample_rate; i++){
        Eigen::VectorXd point_i = plan_manager_->local_traj_.position_traj_.evaluateDeBoorT(i/sample_rate);
        trajectory_points(0, i) = point_i(0);
        trajectory_points(1, i) = point_i(1);
        trajectory_points(2, i) = point_i(2);
    }
    plan_visualizer_->displayTrajectoryPointList(trajectory_points, 1);
}

void InvertedPlanner::planPubCallback(const ros::TimerEvent &time){
    if (false){ // go
        double sample_rate = 300;

        // to flatness control
        // cubic b-spline
        double T = plan_manager_->local_traj_.position_traj_.getTimeSum();
        const int num = T * sample_rate;
        double yaw_i;

        Eigen::VectorXd pos_i;
        Eigen::VectorXd vel_i;
        Eigen::VectorXd acc_i;
        pos_i = plan_manager_->local_traj_.position_traj_.evaluateDeBoorT(count/sample_rate);
        vel_i = plan_manager_->local_traj_.velocity_traj_.evaluateDeBoorT(count/sample_rate);
        acc_i = plan_manager_->local_traj_.acceleration_traj_.evaluateDeBoorT(count/sample_rate);
        yaw_i = 0.0;
        if (count < num) count += 1;

        // msg
        flight_msgs::TrajectoryPoint curr_traj_point;
        ros::Duration dt(count / sample_rate);
        curr_traj_point.time_from_start = dt;

        curr_traj_point.pose.position.x = pos_i(0);
        curr_traj_point.pose.position.y = pos_i(1);
        curr_traj_point.pose.position.z = pos_i(2);

        curr_traj_point.velocity.linear.x = vel_i(0);
        curr_traj_point.velocity.linear.y = vel_i(1);
        curr_traj_point.velocity.linear.z = vel_i(2);

        curr_traj_point.acceleration.linear.x = acc_i(0);
        curr_traj_point.acceleration.linear.y = acc_i(1);
        curr_traj_point.acceleration.linear.z = acc_i(2);

        curr_traj_point.heading = yaw_i;

        trajectory_point_pub_.publish(curr_traj_point);
    }
}

} // namespace inverted_planner

int main(int argc, char **argv)
{
    ros::init(argc, argv, "inverted_planner");
    ros::NodeHandle nh("");
    ros::NodeHandle pnh("~");

    inverted_planner::InvertedPlanner inverted_planner(nh, pnh);

    // single-thread
    ros::spin();

    return 0;
}