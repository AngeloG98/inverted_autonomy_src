#include "lwq_flatness/lwq_flatness.h"

namespace lwq_flatness{

LwqFlatness::LwqFlatness(const ros::NodeHandle &nh,
                        const ros::NodeHandle &pnh)
        :nh_(nh), pnh_(pnh){

    ctrl_data_ = std::make_shared<lwq_flatness::ControlHelper>();
    ctrl_data_->setCtrlParams();
    model_data_ = std::make_shared<lwq_flatness::ModelHelper>();
    model_data_->setModelParams();

    // udp_control_ =
    //     std::make_shared<udp_interface::UdpInterface>(nh_, pnh_, "192.168.1.87", 20010, "192.168.1.197", 20020);

    in_position = false;
    get_trajectory = false;
    trajectory_ready = false;
    udp_ready = false;
    take_off_position << -33.0, 0.0, 6.0;
    take_off_velocity << 0.0, 0.0, 2.0;
    take_off_curr_position << -33.0, 0.0, -0.0;
    time_ctrl_init = ros::Time::now();
    R_FLU_2_FRD << 1, 0, 0,
        0, -1, 0,
        0, 0, -1;

    trajectory_point_sub_ = nh_.subscribe("trajectory_points/bspline", 1, &LwqFlatness::trajectoryRecvCtrlCallback, this);
    odometry_sub_ = nh_.subscribe("/mavros/local_position/odom", 2, &LwqFlatness::odometryCallback, this);
    imudata_sub_ = nh_.subscribe("/mavros/imu/data", 2, &LwqFlatness::imuDataCallback, this);
    mavros_state_sub_ = nh_.subscribe("mavros/state", 1, &LwqFlatness::mavrosStateCallback,this);
    finish_plan_sub_ = nh_.subscribe("trigger/finish_plan", 10, &LwqFlatness::finishPlanCallback, this);
    
    trajectory_trigger_pub_ = nh_.advertise<std_msgs::Int32>("trigger/bspline_plan", 1);
    pwm_pub_ = nh_.advertise<mavros_msgs::RCOut>("/mavros/rc/out", 1);
    att_pub_ = nh_.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 1);
    position_des_pub_ = pnh_.advertise<geometry_msgs::Point>("/desires/position", 1);
    velocity_des_pub_ = pnh_.advertise<geometry_msgs::Point>("/desires/velocity", 1);
    velocity_est_pub_ = pnh_.advertise<geometry_msgs::Point>("/estimates/velocity", 1);
    acceleration_des_pub_ = pnh_.advertise<geometry_msgs::Point>("/desires/acceleration", 1);
    euler_des_pub_ = pnh_.advertise<geometry_msgs::Point>("/desires/euler", 1);
    euler_est_pub_ = pnh_.advertise<geometry_msgs::Point>("/estimates/euler", 1);

    flatness_ctrl_timer_ = nh_.createTimer(ros::Duration(1.0 / ctrl_data_->freq), &LwqFlatness::mainCtrlCallback, this);
    
    vel_des_ << 0.0, 0.0, 0.0;
    acc_des_ << 0.0, 0.0, 0.0;
    pos_des_ << 0.0, 0.0, 0.0;
    ROS_INFO("Flatness controller ready!");
}

void LwqFlatness::mavrosStateCallback(const mavros_msgs::State &msg){
    mavrosState_ = msg;
}

void LwqFlatness::odometryCallback(const nav_msgs::Odometry &odom_msg){
    curr_state_.StateEstimateFromOdom(odom_msg);
    geometry_msgs::Point vel_est_msg;
    const Eigen::Matrix3d R_tmp = flight_common::quaternionToRotationMatrix(curr_state_.orientation);
    const Eigen::Vector3d vel_des_FLU = curr_state_.R_FLU_TO_FRD.transpose() * R_tmp.transpose() * curr_state_.velocity;
    vel_est_msg.x = vel_des_FLU(0);
    vel_est_msg.y = vel_des_FLU(1);
    vel_est_msg.z = vel_des_FLU(2);
    
    // vel_est_msg.x = curr_state_.velocity(0);
    // vel_est_msg.y = curr_state_.velocity(1);
    // vel_est_msg.z = curr_state_.velocity(2);
    velocity_est_pub_.publish(vel_est_msg);
    ROS_INFO("velocity_est_pub_!");
}

void LwqFlatness::imuDataCallback(const sensor_msgs::Imu& imu_msg){
    curr_state_.StateEstimateFromImu(imu_msg);
    geometry_msgs::Point euler_est_msg;
    const Eigen::Vector3d euler_est = flight_common::quaternionToEulerAnglesZYX(curr_state_.orientation);
    euler_est_msg.x = euler_est(0);
    euler_est_msg.y = euler_est(1);
    euler_est_msg.z = euler_est(2);
    euler_est_pub_.publish(euler_est_msg);
    ROS_INFO("euler_est_pub_!");
}

void LwqFlatness::trajectoryRecvCtrlCallback(
        const flight_msgs::TrajectoryPoint &trajectory_point_msg){
    get_trajectory = true;
    flight_common::TrajectoryPoint ref(trajectory_point_msg);
    reference_ = ref;
    ROS_INFO("trajectoryRecvCtrlCallback!");
}

void LwqFlatness::finishPlanCallback(const std_msgs::Int32 &msg){
    if (msg.data == 1){
        trajectory_ready = true;
    }
}

void LwqFlatness::mainCtrlCallback(const ros::TimerEvent &time){
if (trajectory_ready && !udp_ready){
    udp_control_ =
        std::make_shared<udp_interface::UdpInterface>(nh_, pnh_, "192.168.1.87", 20010, "192.168.1.197", 20020);
    udp_ready = true;
}

if (udp_ready){
    // udp
    mavrosState_.mode = "OFFBOARD";
    const flight_common::StateEstimate state = udp_control_->getStatefromUdp();

    // movros
    // const flight_common::StateEstimate state = curr_state_;

    flight_common::ControlCommand Command;

    if (mavrosState_.mode != "OFFBOARD"){
        ROS_INFO_STREAM_ONCE("Waiting for offboard command...");
        // set arm
        Command.armed = true;
        // set mode
        Command.control_mode = flight_common::ControlCommand::ControlMode::PWM;
        // Command.control_mode = flight_common::ControlCommand::ControlMode::BODY_RATE;
    }
    if (!in_position && mavrosState_.mode == "OFFBOARD") // take off
    {
        flight_common::TrajectoryPoint reference;

        ROS_INFO_STREAM_ONCE("Taking off ...");
        if (abs(take_off_curr_position(2)) > abs(take_off_position(2))*0.9) {
            reference.position = take_off_position;
            reference.velocity << 0.0, 0.0, 0.0;
            reference.acceleration << 0.0, 0.0, 0.0;
            if ((state.position - R_FLU_2_FRD * take_off_position).norm() < 2.0 && state.velocity.norm() < 0.5) {
                in_position = true;
            }
        } else {
            take_off_curr_position += take_off_velocity / ctrl_data_->freq;
            reference.position = take_off_curr_position;
            reference.velocity << 0.0, 0.0, 0.0;
            reference.acceleration << 0.0, 0.0, 0.0;
        }
        Command = run(reference, state, 1);
    }
    else if (mavrosState_.mode == "OFFBOARD")
    {
        ROS_INFO_STREAM_ONCE("Waiting for trajectory ...");
        if (!get_trajectory)
        {
            std_msgs::Int32 trajectory_trigger;
            trajectory_trigger.data = 1;
            trajectory_trigger_pub_.publish(trajectory_trigger);
            flight_common::TrajectoryPoint reference;
            reference.position = take_off_position;
            Command = run(reference, state, 1);
        } else {
            ROS_INFO_STREAM_ONCE("Executing trajectory ...");
            const flight_common::TrajectoryPoint reference = reference_;
            std::cout << "reference_:  "<< reference_.position(0) << std::endl;
            Command = run(reference, state, 0);
        }
    }
    // udp
    udp_control_->setCmdtoUdp(Command, state, pos_des_, vel_des_, acc_des_);
    
    // mavros
    // const mavros_msgs::RCOut cmd_out = Command.toMavrosMessagePWM();
    // pwm_pub_.publish(cmd_out);
    const mavros_msgs::AttitudeTarget cmd_out = Command.toMavrosMessageAttRate();
    att_pub_.publish(cmd_out);
    // ROS_INFO("NOW!");
    // std::cout << "sending cmd: "
    //           << "  x: " << cmd_out.body_rate.x << " y: " << cmd_out.body_rate.y << " z: " << cmd_out.body_rate.z << " t: " << cmd_out.thrust << std::endl;

    // for tuning
    geometry_msgs::Point pos_msg;
    const Eigen::Vector3d pos_des_ENU = state.R_ENU_TO_NED.transpose() * pos_des_;
    pos_msg.x = pos_des_ENU(0);
    pos_msg.y = pos_des_ENU(1);
    pos_msg.z = pos_des_ENU(2);

    geometry_msgs::Point vel_des_msg;
    // const Eigen::Matrix3d R_tmp = flight_common::quaternionToRotationMatrix(state.orientation);
    // const Eigen::Vector3d vel_des_FLU = state.R_FLU_TO_FRD.transpose() * R_tmp.transpose() * vel_des_;
    vel_des_msg.x = vel_des_(0);
    vel_des_msg.y = vel_des_(1);
    vel_des_msg.z = vel_des_(2);

    // geometry_msgs::Point vel_est_msg;
    // vel_est_msg.x = state.velocity(0);
    // vel_est_msg.y = state.velocity(1);
    // vel_est_msg.z = state.velocity(2);

    geometry_msgs::Point acc_msg;
    acc_msg.x = acc_des_(0);
    acc_msg.y = acc_des_(1);
    acc_msg.z = acc_des_(2);

    geometry_msgs::Point euler_des_msg;
    const Eigen::Vector3d euler_des = flight_common::quaternionToEulerAnglesZYX(Command.orientation);
    euler_des_msg.x = euler_des(0);
    euler_des_msg.y = euler_des(1);
    euler_des_msg.z = euler_des(2);

    // geometry_msgs::Point euler_est_msg;
    // const Eigen::Vector3d euler_est = flight_common::quaternionToEulerAnglesZYX(state.orientation);
    // euler_est_msg.x = euler_est(0);
    // euler_est_msg.y = euler_est(1);
    // euler_est_msg.z = euler_est(2);

    position_des_pub_.publish(pos_msg);
    velocity_des_pub_.publish(vel_des_msg);
    // velocity_est_pub_.publish(vel_est_msg);
    acceleration_des_pub_.publish(acc_msg);
    euler_des_pub_.publish(euler_des_msg);
    // euler_est_pub_.publish(euler_est_msg);
}
}

flight_common::ControlCommand LwqFlatness::run(
        const flight_common::TrajectoryPoint &reference_state,
        const flight_common::StateEstimate &estimate_state, const int &use_Musk){
    // compute flatness reference command
    flight_common::ControlCommand Cmd_ref;
    // init desired command
    flight_common::ControlCommand Cmd_des;
    // set arm
    Cmd_des.armed = true;
    // set mode
    Cmd_des.control_mode = flight_common::ControlCommand::ControlMode::PWM;
    // Cmd_des.control_mode = flight_common::ControlCommand::ControlMode::BODY_RATE;

    // position control
    pos_des_ = R_FLU_2_FRD * reference_state.position;
    // pos_des_ = reference_state.position;
    const Eigen::Vector3d vel_fd = computeVelFeedback(R_FLU_2_FRD * reference_state.position, estimate_state.position);

    // velocity control
    Eigen::Vector3d vel_des;
    if (use_Musk)
    {
        vel_des = reference_state.velocity + vel_fd;
    }
    else
    {
        // vel_des = reference_state.velocity + vel_fd;
        // vel_des = reference_state.velocity;
        vel_des = R_FLU_2_FRD * reference_state.velocity + vel_fd;
        // vel_des = R_FLU_2_FRD * reference_state.velocity;
        // vel_des = vel_fd;
    }
    vel_des_ = vel_des;
    // flight_common::limit(vel_des(0), 2.0);
    // flight_common::limit(vel_des(1), 2.0);
    // flight_common::limit(vel_des(2), 2.0);
    const Eigen::Vector3d acc_fd = computeAccFeedback(vel_des, estimate_state.velocity);

    // transform desire acceleration and yaw(heading) to desire attitude and thrust (use flatness transform as default here)
    Eigen::Vector3d acc_des;
    if (use_Musk)
    {
        acc_des = reference_state.acceleration + acc_fd;
    }
    else
    {
        // acc_des = reference_state.acceleration + acc_fd;
        // acc_des = reference_state.acceleration * 0.8 + acc_fd;
        // acc_des = R_FLU_2_FRD * reference_state.acceleration;
        acc_des = R_FLU_2_FRD * reference_state.acceleration + acc_fd;
        // acc_des = reference_state.acceleration;
        // acc_des = acc_fd;
    }
    acc_des_ = acc_des;
    // std::cout << "acc_des:  "<< acc_des << std::endl;
    // computeAttitudeThrust(vel_des, acc_des, reference_state.yaw, &Cmd_des, use_Musk);
    // computeAttitudeThrustFormAeroForceEstimate(vel_des, acc_des, reference_state, estimate_state, &Cmd_des, use_Musk);
    computeAttitudeThrustQuadrotor(acc_des, reference_state, &Cmd_des);

    // attitude control
    Eigen::Vector3d bodyrate_fd = computeBodyrateFeedback(Cmd_des.orientation, estimate_state.orientation);

    // bodyrate control
    Cmd_des.bodyrate = Cmd_ref.bodyrate + bodyrate_fd;
    Cmd_des.torque = computeTorque(Cmd_des, estimate_state);

    // compute control allocation and generate PWMs
    Cmd_des.rotor_thrust = doControlAllocation(Cmd_des.torque, Cmd_des.collective_thrust);
    Cmd_des.PWM = generatePWMs(Cmd_des.rotor_thrust);

    return Cmd_des;
}

Eigen::Vector3d LwqFlatness::computeBodyrateFeedback(
        const Eigen::Quaterniond &att_des,
        const Eigen::Quaterniond &att_est) const {
            
    const Eigen::Quaterniond q_err = att_est.inverse() * att_des;
    Eigen::Vector3d bodyrate;

    if (q_err.w() >= 0){
        bodyrate.x() = 2.0 * ctrl_data_->attitude.kpx * q_err.x();
        bodyrate.y() = 2.0 * ctrl_data_->attitude.kpy * q_err.y();
        bodyrate.z() = 2.0 * ctrl_data_->attitude.kpz * q_err.z();
    } else {
        bodyrate.x() = -2.0 * ctrl_data_->attitude.kpx * q_err.x();
        bodyrate.y() = -2.0 * ctrl_data_->attitude.kpy * q_err.y();
        bodyrate.z() = -2.0 * ctrl_data_->attitude.kpz * q_err.z();
    }
    return bodyrate;
}

Eigen::Vector3d LwqFlatness::computeAccFeedback(
        const Eigen::Vector3d &vel_des, const Eigen::Vector3d &vel_est) const {
    const Eigen::Vector3d vel_err = vel_des - vel_est;
    Eigen::Vector3d acceleration;
    // P
    acceleration(0) = ctrl_data_->velocity.kpx * vel_err(0);
    acceleration(1) = ctrl_data_->velocity.kpy * vel_err(1);
    acceleration(2) = ctrl_data_->velocity.kpz * vel_err(2);

    return acceleration;
}

Eigen::Vector3d LwqFlatness::computeVelFeedback(
        const Eigen::Vector3d &pos_des, const Eigen::Vector3d &pos_est) const {
    const Eigen::Vector3d pos_err = pos_des - pos_est;
    Eigen::Vector3d velocity;
    // P
    velocity(0) = ctrl_data_->position.kpx * pos_err(0);
    velocity(1) = ctrl_data_->position.kpy * pos_err(1);
    velocity(2) = ctrl_data_->position.kpz * pos_err(2);

    return velocity;
}

void LwqFlatness::computeAttitudeThrustQuadrotor(const Eigen::Vector3d &acc_des, const flight_common::TrajectoryPoint &reference_state, flight_common::ControlCommand *Cmd_des){
    // this reference_state is 'FLU' coordinate from plan, convert it to 'FRD' coordinate first will be needed!!!
    
    Eigen::Vector3d Ze(0, 0, 1);
    Eigen::Vector3d fd = acc_des - model_data_->gravity * Ze;
    Eigen::Vector3d z_b = -fd / fd.norm();
    Eigen::Vector3d x_c(cos(reference_state.yaw), sin(reference_state.yaw), 0);
    Eigen::Vector3d y_b = z_b.cross(x_c) / (z_b.cross(x_c)).norm();
    Eigen::Vector3d x_b = y_b.cross(z_b);
    Eigen::Matrix3d R;
    R << x_b(0), y_b(0), z_b(0), x_b(1), y_b(1), z_b(1), x_b(2), y_b(2), z_b(2);
    Cmd_des->orientation = flight_common::RotationMatrixToQuaternion(R);

    Cmd_des->collective_thrust = z_b.dot(fd);
    Cmd_des->collective_thrust *= model_data_->lwq_mass;
    // limit thrust
    flight_common::limit_negative(Cmd_des->collective_thrust, model_data_->thrust_max);
    Cmd_des->norm_collective_thrust = - Cmd_des->collective_thrust / model_data_->thrust_max;
}

void LwqFlatness::computeAttitudeThrustFormAeroForceEstimate(
        const Eigen::Vector3d &vel_des, const Eigen::Vector3d &acc_des,
        const flight_common::TrajectoryPoint &reference_state,
        const flight_common::StateEstimate &estimate_state,
        flight_common::ControlCommand *Cmd_des, const int &use_Musk){
    Eigen::Vector3d Ze(0, 0, 1);
    const double Va = estimate_state.velocity.norm();
    const double Qa = model_data_->rho * model_data_->S * Va / (2 * model_data_->lwq_mass);
    const Eigen::Matrix3d R_est = flight_common::quaternionToRotationMatrix(estimate_state.orientation);
    Eigen::Matrix3d phi_matrix;
    phi_matrix << model_data_->C_dx, 0, model_data_->C_dxz,
                    0, model_data_->C_y0, 0,
                    model_data_->C_dxz, 0, model_data_->C_dz;
    Eigen::Vector3d acc_af = Qa * R_est * phi_matrix * R_est.transpose() * estimate_state.velocity;

    Eigen::Vector3d acc_des_af = acc_des - model_data_->gravity * Ze + acc_af; // ' + af '
    Eigen::Vector3d z_b = -acc_des_af / acc_des_af.norm();
    Eigen::Vector3d x_c(cos(reference_state.yaw), sin(reference_state.yaw), 0);
    Eigen::Vector3d y_b = z_b.cross(x_c) / (z_b.cross(x_c)).norm();
    Eigen::Vector3d x_b = y_b.cross(z_b);
    Eigen::Matrix3d R;
    R << x_b(0), y_b(0), z_b(0), x_b(1), y_b(1), z_b(1), x_b(2), y_b(2), z_b(2);
    Cmd_des->orientation = flight_common::RotationMatrixToQuaternion(R);

    Cmd_des->collective_thrust = z_b.dot(acc_des_af);
    Cmd_des->collective_thrust *= model_data_->lwq_mass;
    // limit thrust
    flight_common::limit_negative(Cmd_des->collective_thrust, model_data_->thrust_max);
    Cmd_des->norm_collective_thrust = - Cmd_des->collective_thrust / model_data_->thrust_max;
}

void LwqFlatness::computeAttitudeThrust(
        const Eigen::Vector3d &vel_des,
        const Eigen::Vector3d &acc_des,
        const double &yaw_des,
        flight_common::ControlCommand *Cmd_des,
        const int &use_Musk){
    // lift wing quadcopter differential flatness
    // 'acceleration(x_ddot) and velocity(x_dot)' -> 'thrust and orientation'
    Eigen::Vector3d Ze(0, 0, 1);
    double Va = vel_des.norm();

    if (Va < 0.3 || use_Musk) {// low speed
        ROS_INFO_STREAM_THROTTLE(5, "Using low speed transform!");
        double Qa = model_data_->rho * model_data_->S * Va / (2 * model_data_->lwq_mass);
        Eigen::Vector3d a_x = Qa * model_data_->C_d0 * vel_des - model_data_->gravity * Ze + acc_des;
        Eigen::Vector3d a_y = Qa * model_data_->C_y0 * vel_des - model_data_->gravity * Ze + acc_des;
        Eigen::Vector3d x_c(cos(yaw_des), sin(yaw_des), 0);
        Eigen::Vector3d y_c(-sin(yaw_des), cos(yaw_des), 0);
        
        // Z-Y-X order 
        Eigen::Vector3d x_b = a_x.cross(y_c) / (a_x.cross(y_c)).norm();
        Eigen::Vector3d y_b = x_b.cross(a_y) / (x_b.cross(a_y)).norm();
        Eigen::Vector3d z_b = x_b.cross(y_b);
        Eigen::Matrix3d R;
        R << x_b(0), y_b(0), z_b(0), x_b(1), y_b(1), z_b(1), x_b(2), y_b(2), z_b(2);
        Cmd_des->orientation = flight_common::RotationMatrixToQuaternion(R);

        // force
        Eigen::Vector3d a_z = Qa * model_data_->C_dz * vel_des - model_data_->gravity * Ze + acc_des;
        Cmd_des->collective_thrust = Qa * model_data_->C_dx * x_b.dot(vel_des) + z_b.dot(a_z);
        Cmd_des->collective_thrust *= model_data_->lwq_mass;
    } else { // high speed
        ROS_INFO_STREAM_THROTTLE(5, "Using high speed transform!");
        // compute in constrained velocity frame
        // x
        Eigen::Vector3d x_s = vel_des / Va;
        double a_sx = x_s.dot(acc_des - model_data_->gravity * Ze);
        // z
        Eigen::Vector3d z_s = acc_des - model_data_->gravity * Ze - a_sx * x_s;
        double a_sz = -z_s.norm();
        Eigen::Vector3d z_s_norm = z_s / a_sz;
        // y
        Eigen::Vector3d y_s = z_s_norm.cross(x_s);
        // velocity frame rotation matrix
        Eigen::Matrix3d R_s;
        R_s << x_s(0), y_s(0), z_s_norm(0), x_s(1), y_s(1), z_s_norm(1), x_s(2), y_s(2), z_s_norm(2);

        // compute in body frame with big phi theory
        double QS = model_data_->rho * model_data_->S * pow(Va,2) / 2;
        double lift = model_data_->C_L * QS;
        double drag = model_data_->C_d0 * QS;
        double masz = model_data_->lwq_mass * a_sz;
        double masx = model_data_->lwq_mass * a_sx;
        double ka = model_data_->kappa;
        
        double k_term =
            sqrt(pow((drag + masx) * cos(ka), 2) + pow((drag + lift + masx) * sin(ka), 2) + pow(masz, 2) - (lift * masz * sin(2 * ka)) );
        double f_z =
            -abs(pow((drag + masx), 2) + pow(masz, 2) + lift * (drag + masx)) / k_term;
        double alpha =
            2 * atan((lift * sin(ka) - masz * cos(ka) + (drag + masx) * sin(ka) - k_term) / ((drag + masx) * cos(ka) + masz * sin(ka)));
        Cmd_des->collective_thrust = f_z;
        Cmd_des->alpha_guess = alpha;
        
        // alpha matrix, intend to rotate from velocity frame to body frame
        Eigen::Matrix3d R_alpha;
        R_alpha << cos(alpha - ka), 0, sin(alpha - ka), 0, 1, 0, -sin(alpha - ka), 0, cos(alpha - ka);

        // compute body frame rotation matrix with 'zero sliding assumpution'
        // compute body frame rotation matrix from alpha (and beta ?)
        Eigen::Matrix3d R;
        R = R_s * R_alpha; // can also include C_y0 term here.
        Cmd_des->orientation = flight_common::RotationMatrixToQuaternion(R);
    }
    // limit thrust
    flight_common::limit_negative(Cmd_des->collective_thrust, model_data_->thrust_max);
    Cmd_des->norm_collective_thrust = - Cmd_des->collective_thrust / model_data_->thrust_max;
    // std::cout << "norm_collective_thrust:  "<< Cmd_des->norm_collective_thrust << std::endl;
}

Eigen::Vector3d LwqFlatness::computeTorque(
        const flight_common::ControlCommand &Cmd_des, 
        const flight_common::StateEstimate &estimate_state) const {
    const Eigen::Vector3d bodyrate_err = Cmd_des.bodyrate - estimate_state.bodyrate;
    Eigen::Vector3d torque;
    // P
    torque(0) = ctrl_data_->bodyrate.kpx * bodyrate_err(0);
    torque(1) = ctrl_data_->bodyrate.kpy * bodyrate_err(1);
    torque(2) = ctrl_data_->bodyrate.kpz * bodyrate_err(2);
    // I
    ctrl_data_->bodyrate.I_term_x += bodyrate_err(0);
    ctrl_data_->bodyrate.I_term_y += bodyrate_err(1);
    ctrl_data_->bodyrate.I_term_z += bodyrate_err(2);
    flight_common::limit(ctrl_data_->bodyrate.I_term_x, ctrl_data_->bodyrate.saturI);
    flight_common::limit(ctrl_data_->bodyrate.I_term_y, ctrl_data_->bodyrate.saturI);
    flight_common::limit(ctrl_data_->bodyrate.I_term_z, 0.0);
    torque(0) += ctrl_data_->bodyrate.kix * ctrl_data_->bodyrate.I_term_x / ctrl_data_->freq; // *interval
    torque(1) += ctrl_data_->bodyrate.kiy * ctrl_data_->bodyrate.I_term_y / ctrl_data_->freq;
    torque(2) += ctrl_data_->bodyrate.kiz * ctrl_data_->bodyrate.I_term_z / ctrl_data_->freq;
    // D
    torque(0) -= ctrl_data_->bodyrate.kdx * (bodyrate_err(0) - ctrl_data_->bodyrate.last_Err_x) * ctrl_data_->freq; // /interval
    torque(1) -= ctrl_data_->bodyrate.kdy * (bodyrate_err(1) - ctrl_data_->bodyrate.last_Err_y) * ctrl_data_->freq;
    torque(2) -= ctrl_data_->bodyrate.kdz * (bodyrate_err(2) - ctrl_data_->bodyrate.last_Err_z) * ctrl_data_->freq;
    ctrl_data_->bodyrate.last_Err_x = bodyrate_err(0);
    ctrl_data_->bodyrate.last_Err_y = bodyrate_err(1);
    ctrl_data_->bodyrate.last_Err_z = bodyrate_err(2);
    // angular_acceleration * moment of inertia
    torque(0) *= model_data_->lwq_Jxx;
    torque(1) *= model_data_->lwq_Jyy;
    torque(2) *= model_data_->lwq_Jzz;
    // add forward torque cased from
    double Va = estimate_state.velocity.norm();
    double QS = model_data_->rho * model_data_->S * pow(Va,2) / 2;
    torque(0) -= 1.0 * QS * model_data_->C_l * Cmd_des.alpha_guess;
    torque(1) -= 1.0 * QS * model_data_->c_avr * (model_data_->C_m * Cmd_des.alpha_guess + model_data_->C_m0);
    torque(2) -= 1.0 * QS * model_data_->C_n * Cmd_des.alpha_guess;
    // limit torque
    flight_common::limit(torque(0), model_data_->torque_max);
    flight_common::limit(torque(1), model_data_->torque_max);
    flight_common::limit(torque(2), model_data_->torque_max/2);
    return torque;
}

Eigen::Vector4d LwqFlatness::doControlAllocation(
        const Eigen::Vector3d &torque, const double &collective_thrust) const{
    Eigen::Vector4d rotor_thrust;
    const Eigen::Vector4d body_ft(collective_thrust, torque(0), torque(1), torque(2));
    
    // allocation matrix could be compute once, and store in params
    // but due to unfixed eigen bugs, compute it here...
    Eigen::Matrix4d allocation_matrix;
    allocation_matrix << -1, -1, -1, -1,
        -model_data_->rotor_dy, model_data_->rotor_dy, model_data_->rotor_dy, -model_data_->rotor_dy,
        model_data_->rotor_dx, -model_data_->rotor_dx, model_data_->rotor_dx, -model_data_->rotor_dx,
        model_data_->rotor_Kmt, model_data_->rotor_Kmt, -model_data_->rotor_Kmt, -model_data_->rotor_Kmt;
    Eigen::Matrix4d inversed_allocation_matrix = allocation_matrix.inverse();

    rotor_thrust = inversed_allocation_matrix * body_ft;
    for (int i = 0; i < 4; i++){
        flight_common::limit_min(rotor_thrust(i), 0.2);
        flight_common::limit_max(rotor_thrust(i), model_data_->single_T_max * 0.95);
    }
    return rotor_thrust;
}

Eigen::Vector4d LwqFlatness::generatePWMs(const Eigen::Vector4d &rotor_thrust) const{
    Eigen::Vector4d PWM;
    if (model_data_->motor_nolinear){
        for (int i = 0; i < 4; i++){
            double motorSpeed = sqrt(rotor_thrust(i) / model_data_->rotor_Ct);
            double c = model_data_->motor_k0 - motorSpeed;
            double b = model_data_->motor_k1;
            double a = model_data_->motor_k2;
            double sigma = (-b + sqrt(b*b - 4*a*c)) / (2 * a);
            flight_common::limit_positive(sigma, 1.0);
            PWM(i) = sigma * 1000 + 1000;
        }
    }
    return PWM;
}

double LwqFlatness::generateSigma(const double &collective_thrust) const{
    double motorSpeed = sqrt(collective_thrust / 4 / model_data_->rotor_Ct);
    double c = model_data_->motor_k0 - motorSpeed;
    double b = model_data_->motor_k1;
    double a = model_data_->motor_k2;
    double sigma = (-b + sqrt(b*b - 4*a*c)) / (2 * a);
    flight_common::limit_positive(sigma, 1.0);
    return sigma;
}
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lwq_flatness");
    ros::NodeHandle nh("");
    ros::NodeHandle pnh("~");

    lwq_flatness::LwqFlatness lwq_flatness(nh, pnh);

    // single-thread
    // ros::spin();
    // multi-thread
    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();

    return 0;
}