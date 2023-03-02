/*****************************************************************
 Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
******************************************************************/

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>
#include <iostream>
#include <unistd.h>
#include <string.h>

using namespace UNITREE_LEGGED_SDK;

class Custom
{
public:
    Custom(uint8_t level): 
      safe(LeggedType::Go1), 
      udp(level, 8090, "192.168.123.161", 8082){
        udp.InitCmdData(cmd);
    }
    void UDPRecv();
    void UDPSend();
    void RobotControl();

    Safety safe;
    UDP udp;
    HighCmd cmd = {0};
    HighState state = {0};
    int motiontime = 0;
    float dt = 0.002;     // 0.001~0.01
};


void Custom::UDPRecv()
{
    udp.Recv();
}

void Custom::UDPSend()
{  
    udp.Send();
}

void Custom::RobotControl() 
{
    motiontime += 2;
    udp.GetRecv(state);
    printf("%d   %f\n", motiontime, state.imu.quaternion[2]);

    cmd.mode = 0;      // 0:idle, default stand      1:forced stand     2:walk continuously
    cmd.gaitType = 0;
    cmd.speedLevel = 0;
    cmd.footRaiseHeight = 0;
    cmd.bodyHeight = 0;
    cmd.euler[0]  = 0;
    cmd.euler[1] = 0;
    cmd.euler[2] = 0;
    cmd.velocity[0] = 0.0f;
    cmd.velocity[1] = 0.0f;
    cmd.yawSpeed = 0.0f;
    cmd.reserve = 0;

    if(motiontime > 0 && motiontime < 500){
        cmd.mode = 1;
        //cmd.euler[0] = -0.3;
    }
     if(motiontime > 500 && motiontime < 46500){
        cmd.mode = 1;
    }
    if(motiontime >46500 && motiontime < 47000){
        cmd.mode = 1;
    }
 if(motiontime > 47000 && motiontime < 48000){
        cmd.mode = 2;
        cmd.gaitType = 1;
        cmd.yawSpeed=-0.13;
        printf("turn1\n");
    }
 if(motiontime > 48000 && motiontime < 52000){
        cmd.mode = 1;
        printf("stop\n");
    }
if(motiontime > 52000 && motiontime < 53000){
        cmd.mode = 2;
        cmd.gaitType = 1;
        cmd.yawSpeed=-0.13;
        printf("turn1\n");
    }
 if(motiontime > 53000 && motiontime < 57000){
        cmd.mode = 1;
        printf("stop\n");
    }
if(motiontime > 57000 && motiontime < 58000){
        cmd.mode = 2;
        cmd.gaitType = 1;
        cmd.yawSpeed=-0.13;
        printf("turn1\n");
    }
 if(motiontime > 58000 && motiontime < 62000){
        cmd.mode = 1;
        printf("stop\n");
    }
if(motiontime > 62000 && motiontime < 63000){
        cmd.mode = 2;
        cmd.gaitType = 1;
        cmd.yawSpeed=-0.13;
        printf("turn1\n");
    }
 if(motiontime > 63000 && motiontime < 68500){
        cmd.mode = 1;
        printf("stop\n");
    }
   
 if(motiontime > 68500 && motiontime < 99500){
 	cmd.mode = 1;
        printf("stop\n");
    }
    if(motiontime>99500 &&motiontime<100500 ){
       cmd.mode = 2;
        cmd.gaitType = 1;
        cmd.yawSpeed=0.13;
        printf("turn2\n");
    }
if(motiontime > 100500 && motiontime < 108500){
 	cmd.mode = 1;
        printf("stop\n");
    }
 if(motiontime>108500 &&motiontime<109500 ){
       cmd.mode = 2;
        cmd.gaitType = 1;
        cmd.yawSpeed=0.13;
        printf("turn2\n");
    }
if(motiontime >109500 && motiontime < 117500){
 	cmd.mode = 1;
        printf("stop\n");
    }
 if(motiontime>117500 &&motiontime<118500 ){
       cmd.mode = 2;
        cmd.gaitType = 1;
        cmd.yawSpeed=0.13;
        printf("turn2\n");
    }
if(motiontime > 118500 && motiontime < 126500){
 	cmd.mode = 1;
        printf("stop\n");
    }
 if(motiontime>126500 &&motiontime<127500 ){
       cmd.mode = 2;
        cmd.gaitType = 1;
        cmd.yawSpeed=0.13;
        printf("turn2\n");
    }
if(motiontime >127500 ){
 	cmd.mode = 1;
        printf("stop\n");
    }
    udp.SetSend(cmd);
}

int main(void) 
{
    std::cout << "Communication level is set to HIGH-level." << std::endl
              << "WARNING: Make sure the robot is standing on the ground." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    Custom custom(HIGHLEVEL);
    // InitEnvironment();
    LoopFunc loop_control("control_loop", custom.dt,    boost::bind(&Custom::RobotControl, &custom));
    LoopFunc loop_udpSend("udp_send",     custom.dt, 3, boost::bind(&Custom::UDPSend,      &custom));
    LoopFunc loop_udpRecv("udp_recv",     custom.dt, 3, boost::bind(&Custom::UDPRecv,      &custom));

    loop_udpSend.start();
    loop_udpRecv.start();
    loop_control.start();

    while(1){
        sleep(10);
    };

    return 0; 
}
