#include "udp_interface/udp_interface.h"

namespace udp_interface{

UdpInterface::UdpInterface(const ros::NodeHandle &nh,
                            const ros::NodeHandle &pnh,
                            const char* local_ip,
                            const int local_port,
                            const char* target_ip,
                            const int target_port)
        :nh_(nh), pnh_(pnh),
        local_ip_(local_ip), local_port_(local_port),
        target_ip_(target_ip), target_port_(target_port){
    // client settings
    client_fd = socket(AF_INET, SOCK_DGRAM, 0); // IPV4 UDP
    memset(&ser_addr, 0, sizeof(ser_addr));
    ser_addr.sin_family = AF_INET;
    ser_addr.sin_addr.s_addr = inet_addr(target_ip_);
    ser_addr.sin_port = htons(target_port_);
    // client settings
    server_fd = socket(AF_INET, SOCK_DGRAM, 0);
    memset(&cli_addr, 0, sizeof(cli_addr));
    cli_addr.sin_family = AF_INET;
    cli_addr.sin_addr.s_addr = inet_addr(local_ip_);
    cli_addr.sin_port = htons(local_port_);
    bind_flag = bind(server_fd, (struct sockaddr *)&cli_addr, sizeof(cli_addr));
    if (bind_flag < 0){
        ROS_ERROR("Socket bind failed!");
    }
    // data init
    cmd.reset();
    state.reset();
    // timestamp init
    time_udp_init = ros::Time::now();
    //
    udp_send_timer_ = nh_.createTimer(ros::Duration(1.0 / 500), &UdpInterface::sendCmdCallback, this);
    udp_recv_timer_ = nh_.createTimer(ros::Duration(1.0 / 1000), &UdpInterface::recvStateCallback, this);

    ROS_INFO("Udp for control ready!");
}

void UdpInterface::setCmdtoUdp(
        const flight_common::ControlCommand &udpCmd, const flight_common::StateEstimate &state_use,
        const Eigen::Vector3d pos_des, const Eigen::Vector3d vel_des, const Eigen::Vector3d acc_des){
    if (udpCmd.armed){
        // switch (udpCmd.control_mode) {}
        // send all
        for (int i = 0; i < 3; i++){
            cmd.pos_cmd[i] = pos_des(i);
            cmd.vel_cmd[i] = vel_des(i);
            cmd.acc_cmd[i] = acc_des(i);
            cmd.att_cmd[i] = flight_common::quaternionToEulerAnglesZYX(udpCmd.orientation)(i);
            cmd.bodyrate_cmd[i] = udpCmd.bodyrate(i);
        }
        for (int i = 0; i < 4; i++){
            cmd.ctrls[i] = udpCmd.PWM(i);
        }
        cmd.alpha_guess = udpCmd.alpha_guess;
    }
}

flight_common::StateEstimate
UdpInterface::getStatefromUdp() const{
    flight_common::StateEstimate udpState;
    udpState.coordinate_frame = flight_common::StateEstimate::CoordinateFrame::WORLD;
    udpState.position << state.pos_e[0], state.pos_e[1], state.pos_e[2];
    udpState.velocity << state.vel_e[0], state.vel_e[1], state.vel_e[2];
    udpState.bodyrate << state.w_b[0], state.w_b[1], state.w_b[2];
    udpState.orientation.w() = state.quaternion[0]; // w x y z
    udpState.orientation.x() = state.quaternion[1];
    udpState.orientation.y() = state.quaternion[2];
    udpState.orientation.z() = state.quaternion[3];

    return udpState;
}

void UdpInterface::sendCmdCallback(const ros::TimerEvent &time){
    cM.musk = (int)1;
    cmd.timestamp = (double)(ros::Time::now() - time_udp_init).toSec(); //
    memcpy(cM.payload, &cmd, sizeof(cmd));
    //
    sendto(client_fd, (const char *)&cM, sizeof(cM), 0, (struct sockaddr *)&ser_addr, sizeof(ser_addr));
    // ROS_INFO("Sending...");
}

void UdpInterface::recvStateCallback(const ros::TimerEvent &time){
    // ROS_INFO("Receiving...");
    int recvlen;
    char buffer[sizeof(sM)];
    socklen_t cli_len = sizeof(cli_addr);
    //
    recvlen = recvfrom(server_fd, buffer, sizeof(buffer), 0, (struct sockaddr *)&cli_addr, &cli_len);
    memcpy(&sM, &buffer, sizeof(sM)); // 
    memcpy(&state, &sM.payload, sizeof(state)); //
}
}