#include <cmath>
#include <ros/ros.h>
#include <Eigen/Eigen>
#include "udp_interface/udp_interface.h"
#include "flight_common/state_estimate.h"
#include "flight_common/control_command.h"
#include "flight_common/trajectory_point.h"
#include "flight_common/math_common.h"

#include "lwq_flatness/control_helper.h"
#include "lwq_flatness/model_helper.h"
#include "std_msgs/Int32.h"
#include <mavros_msgs/RCOut.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <geometry_msgs/Point.h>

//
// #include <lwq_flatness/TrajectoryPoint.h>

namespace lwq_flatness{
class LwqFlatness{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    LwqFlatness(const ros::NodeHandle &nh, const ros::NodeHandle &pnh);

    // LwqFlatness(): LwqFlatness(ros::NodeHandle(), ros::NodeHandle("~")) {}

    // virtual ~LwqFlatness();

private:
    flight_common::ControlCommand run(const flight_common::TrajectoryPoint &reference_state,
                                      const flight_common::StateEstimate &estimate_state, const int &use_Musk);
    void odometryCallback(const nav_msgs::Odometry &odom_msg);
    void imuDataCallback(const sensor_msgs::Imu &imu_msg);
    void mavrosStateCallback(const mavros_msgs::State &msg);

    void mainCtrlCallback(const ros::TimerEvent &time);
    void trajectoryRecvCtrlCallback(const flight_msgs::TrajectoryPoint &trajectory_point_msg);
    
    Eigen::Vector3d computeBodyrateFeedback(const Eigen::Quaterniond &att_des, const Eigen::Quaterniond &att_est) const;
    Eigen::Vector3d computeAccFeedback(const Eigen::Vector3d &vel_des, const Eigen::Vector3d &vel_est) const;
    Eigen::Vector3d computeVelFeedback(const Eigen::Vector3d &pos_des, const Eigen::Vector3d &pos_est) const;

    void computeAttitudeThrustQuadrotor(const Eigen::Vector3d &acc_des, const flight_common::TrajectoryPoint &reference_state, flight_common::ControlCommand *Cmd_des);
    void computeAttitudeThrustFormAeroForceEstimate(const Eigen::Vector3d &vel_des, const Eigen::Vector3d &acc_des,
                                                    const flight_common::TrajectoryPoint &reference_state,
                                                    const flight_common::StateEstimate &estimate_state,
                                                    flight_common::ControlCommand *Cmd_des, const int &use_Musk);
    void computeAttitudeThrust(const Eigen::Vector3d &vel_des, const Eigen::Vector3d &acc_des,
                               const double &yaw_des, flight_common::ControlCommand *Cmd_des, const int &use_Musk);
    Eigen::Vector3d computeTorque(const flight_common::ControlCommand &Cmd_des,
                                  const flight_common::StateEstimate &estimate_state) const;
    Eigen::Vector4d doControlAllocation(const Eigen::Vector3d &torque, const double &collective_thrust) const;
    Eigen::Vector4d generatePWMs(const Eigen::Vector4d &rotor_thrust) const;
    double generateSigma(const double &collective_thrust) const;

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Timer flatness_ctrl_timer_;

    ros::Subscriber trajectory_point_sub_;
    ros::Subscriber odometry_sub_;
    ros::Subscriber imudata_sub_;
    ros::Subscriber mavros_state_sub_;

    ros::Publisher trajectory_trigger_pub_;
    ros::Publisher pwm_pub_;
    ros::Publisher att_pub_;

    ros::Publisher position_des_pub_;
    ros::Publisher velocity_des_pub_;
    ros::Publisher velocity_est_pub_;
    ros::Publisher acceleration_des_pub_;
    ros::Publisher euler_des_pub_;
    ros::Publisher euler_est_pub_;
    
    ros::Time time_ctrl_init;

    std::shared_ptr<lwq_flatness::ControlHelper> ctrl_data_;
    std::shared_ptr<lwq_flatness::ModelHelper> model_data_;

    std::shared_ptr<udp_interface::UdpInterface> udp_control_;

    Eigen::Matrix3d R_FLU_2_FRD;
    bool in_position;
    Eigen::Vector3d take_off_position;
    Eigen::Vector3d take_off_velocity;
    Eigen::Vector3d take_off_curr_position;
    bool get_trajectory;

    flight_common::StateEstimate curr_state_;
    flight_common::TrajectoryPoint reference_;

    // only for display purpose
    Eigen::Vector3d pos_des_;
    Eigen::Vector3d vel_des_;
    Eigen::Vector3d acc_des_;
    
    mavros_msgs::State mavrosState_;
};
}