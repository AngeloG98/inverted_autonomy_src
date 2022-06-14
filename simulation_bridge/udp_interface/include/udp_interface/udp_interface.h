#pragma once

#include <stdio.h>
#include <thread>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <string.h>
#include <unistd.h>
#include <iostream>
#include <ros/ros.h>

#include "flight_common/state_estimate.h"
#include "flight_common/control_command.h"
#include "flight_common/math_common.h"

#pragma pack(push) 
#pragma pack(1)
namespace udp_interface{

class UdpInterface{
public:
    UdpInterface(const ros::NodeHandle &nh,
                const ros::NodeHandle &pnh,
                const char* local_ip,
                const int local_port,
                const char* target_ip,
                const int target_port);

    void setCmdtoUdp(const flight_common::ControlCommand &udpCmd,
                     const flight_common::StateEstimate &state_use,
                     const Eigen::Vector3d pos_des, const Eigen::Vector3d vel_des, const Eigen::Vector3d acc_des);
    flight_common::StateEstimate getStatefromUdp() const;

private:
    void sendCmdCallback(const ros::TimerEvent &time);
    void recvStateCallback(const ros::TimerEvent &time);

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Timer udp_send_timer_, udp_recv_timer_;

    // cmd struct
    struct Cmd // lenth 5*3*4 + 4*4 + 4 + 8 = 88
    {
        // 4 channel control signal, could be '3 bodyrate and 1 thrust' or '4 pwms' e.g.
        float pos_cmd[3];
        float vel_cmd[3];
        float acc_cmd[3];
        float att_cmd[3];
        float bodyrate_cmd[3];
        float ctrls[4];
        float alpha_guess;
        double timestamp;
        Cmd(){
            reset();
        }
        void reset(){
            for (int i = 0; i < 3; i++){
                pos_cmd[i] = 0.0;
                vel_cmd[i] = 0.0;
                acc_cmd[i] = 0.0;
                att_cmd[i] = 0.0;
                bodyrate_cmd[i] = 0.0;
            }
            for (int i = 0; i < 4; i++){
                ctrls[i] = 0.0;
            }
            alpha_guess = 0.0;
            timestamp = 0.0;
        }
    };
    Cmd cmd;

    // state struct package
    typedef struct _CmdMessage // lenth = 96
    {
        int musk;
        int len; // 88
        char payload[sizeof(struct Cmd)];
        _CmdMessage(){
            musk = 0;
            len = 88;
            memset(payload, 0, sizeof(payload));
        }
    } CmdMessage;
    CmdMessage cM;

    // state struct
    struct State // lenth (3+3+3+9+4+3)*4 + 8 = 108
    {
        float pos_e[3]; // position in world frame
        float vel_e[3];
        float euler[3];
        float R_eb[9];
        float quaternion[4];
        float w_b[3];
        double timestamp;
        State(){
            reset();
        }
        void reset(){
            for (int i = 0; i < 3; i++){
                pos_e[i] = 0.0;
                vel_e[i] = 0.0;
                euler[i] = 0.0;
                w_b[i] = 0.0;
            }
            for (int i = 0; i < 9; i++){
                R_eb[i] = 0.0;
                if (i == 1 || 5 || 9) {R_eb[i] = 1.0;}
            }
            for (int i = 0; i < 4; i++){
                quaternion[i] = 0.0;
                if (i == 1) {quaternion[i] = 1.0;}
            }
            timestamp = 0.0;
        }
    };
    State state;

    // state struct package
    typedef struct _StateMessage // lenth = 116
    {
        int musk;
        int len; // 108
        char payload[sizeof(struct State)];
        _StateMessage(){
            musk = 0;
            len = 108;
            memset(payload, 0, sizeof(payload));
        }
    } StateMessage;
    StateMessage sM;

    const char* local_ip_;
    const int local_port_;
    const char* target_ip_;
    const int target_port_;
    int client_fd;
    int server_fd, bind_flag;
    struct sockaddr_in ser_addr;
    struct sockaddr_in cli_addr;

    ros::Time time_udp_init;
};

}

// recover struct align here, very important !!! 
// otherwise custom alignment will influence other head file, if with eigen make eigen goes wrong.
#pragma pack(pop) 