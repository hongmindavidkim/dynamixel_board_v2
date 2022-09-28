/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */
 
#include "mbed.h"
#include "crc.h"
#include "string.h"
#include "dynamixel_XM430.h"
#include "math.h"
#include <cmath>
#include <cstdint>
#include "actuator_transformation.h"
#include "math_ops.h"  
#include "ForceSensor.h"

//MAIN BUS CAN 
#define CAN_TX_DXL9            0
#define CAN_TX_DXL1            1 // commands from controller to DXLs
#define CAN_TX_DXL2            2
#define CAN_TX_DXL3            3
#define CAN_TX_DXL4            4
#define CAN_TX_DXL5            5
#define CAN_TX_DXL6            6
#define CAN_TX_DXL7            7
#define CAN_TX_DXL8            8

#define CAN_FORCE_1            9
#define CAN_FORCE_2            10
#define CAN_TOF_1              11
#define CAN_TOF_2              12

#define CAN_RX_DXL1            13 // responses from DXLs to controller
#define CAN_RX_DXL2            14
#define CAN_RX_DXL3            15
#define CAN_RX_DXL4            16
#define CAN_RX_DXL5            17
#define CAN_RX_DXL6            18
#define CAN_RX_DXL7            19
#define CAN_RX_DXL8            20
#define CAN_RX_DXL9            21

/// Value Limits ///
#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -65.0f
#define V_MAX 65.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 10.0f
#define T_MIN -72.0f
#define T_MAX 72.0f

#define KP_SCALE 50.0f
#define KD_SCALE 50.0f
#define T_SCALE 50.0f

// new limits for force sensors, need to test these!
#define FT_MIN -20.0f
#define FT_MAX 20.0f 
#define FN_MIN -30.0f
#define FN_MAX 30.0f
#define ANG_MIN -135.0f
#define ANG_MAX 45.0f
#define RNG_MAX 255 // this probably won't be necessary

// Initialize serial port
RawSerial pc(PA_9, PA_10, 921600);
RawSerial uart(PC_10, PC_11);
DigitalInOut RTS(PA_6);

// TODO: add wrist roll back in here!
uint8_t dxl_ID[] =  {1, 2, 3, 4, 5, 6, 7, 8, 9};
uint8_t dxl_PCIDs[] = {4, 8};
uint8_t dxl_left_ids[] = {1, 2, 3};
uint8_t dxl_right_ids[] = {5, 6, 7};
uint8_t dxl_wrist_ids[] = {4, 8, 9};

uint8_t idLength = sizeof(dxl_ID) / sizeof(dxl_ID[0]);


PinName SYS_CAN_TX = PA_12;
PinName SYS_CAN_RX = PA_11;
PinName SENSOR_CAN_TX = PB_6;
PinName SENSOR_CAN_RX = PB_5;
 
// CAN setup
CAN cansys(SYS_CAN_RX, SYS_CAN_TX, 1000000);
CAN cansens(SENSOR_CAN_RX, SENSOR_CAN_TX, 1000000);
CANMessage rxMsg;
CANMessage sensorRxMsg;
CANMessage txDxl1, txDxl2, txDxl3, txDxl4, txDxl5, txDxl6, txDxl7, txDxl8, txDxl9, txForce1, txForce2, txTOF1, txTOF2;
Timer CANTimer;
Timer loopTimer;
int loopTime;

// Variables for  dynamixel bus
// uint32_t goalDes1[4];
// uint32_t goalDes2[4];
// uint32_t goalPos[8];
float currentPos[9];
float currentVel[9];
float currentCur[9];
float currentJointTau[9];
int32_t pressure_raw1[8];
int32_t pressure_raw2[8];
uint8_t tof1[8];
uint8_t tof2[8];
uint8_t palmTOF; 
extern NeuralNet sensorB3;
ForceSensor forcesensor1(0, &sensorB3);
extern NeuralNet sensorB4;
ForceSensor forcesensor2(1, &sensorB4);

float Kt = 2.0f * (3.7f / 2.7f);
float Ktinv = 1/Kt;
float current_limit = 2.70f; //2.30f; // in A, max is 3.2(?)

float Kt_WR = 2.0f * (8.9f / 5.5f);
float Ktinv_WR = 1.0f/Kt_WR;
float current_limit_WR = 4.40f; // in A

float pulse_to_rad = (2.0f*PI)/4096.0f; // = 0.001534
float rpm_to_rads = (0.229f*2.0f*PI)/60.0f; // = 0.0239
// from joint space to actuator space... phidot = Jact*thetadot

float JactL[4][4] = { {(13.88f/15.48f), 0.0f, 0.0f, -(16.38f/15.48f)}, 
                     {(9.68f/15.48f), (13.88f/15.48f), 0.0f, (11.48f/15.48f)}, 
                     {(5.48f/15.48f), (5.48f/15.48f), (13.88f/15.48f), -(8.08f/15.48f)}, 
                     {0.0f, 0.0f, 0.0f, (15.98f/15.48f)} };

float JactR[4][4] = { {(13.88f/15.48f), 0.0f, 0.0f, -(16.38f/15.48f)}, 
                     {-(9.68f/15.48f), -(13.88f/15.48f), 0.0f, -(11.48f/15.48f)}, 
                     {(5.48f/15.48f), (5.48f/15.48f), (13.88f/15.48f), -(8.08f/15.48f)}, 
                     {0.0f, 0.0f, 0.0f, -(15.98f/15.48f)} };
                     
// from actuator space to joint space... thetadot = Jjoint*phidot
float JjointL[4][4] = {{1.11527378f, 0.0f, 0.0f, 1.14319052f},
                     {-0.777799f, 1.11527378f, 0.0f, -1.59847876f},
                     {-0.13323932f, -0.44032423f, 1.11527378f, 0.74367173f},
                     {0.0f, 0.0f, 0.0f, 0.96871089f}};

float JjointR[4][4] = {{1.11527378f, 0.0f, 0.0f, -1.14319052f},
                     {-0.777799f, -1.11527378f, 0.0f, 1.59847876f},
                     {-0.13323932f, 0.44032423f, 1.11527378f, -0.74367173f},
                     {0.0f, 0.0f, 0.0f, -0.96871089f}};



int32_t dxl_position_left[3];
int32_t dxl_position_right[3];
int32_t dxl_position_wrist[3];


int32_t dxl_velocity_left[3];
int32_t dxl_velocity_right[3];
int32_t dxl_velocity_wrist[3];


int16_t dxl_current_left[3];
int16_t dxl_current_right[3];
int16_t dxl_current_wrist[3];

float desired_current[9];
uint16_t current_command[9];

// CAN command variables
float dxl_pos_des[9];
float dxl_vel_des[9];
float dxl_tff_des[9];
float dxl_kp[9];
float dxl_kd[9];

void updateBus(XM430_bus *dxl_bus_left, XM430_bus *dxl_bus_right, XM430_bus *dxl_bus_wrist) {
    
    double jointAngles1[4];
    double jointVelocities1[4];
    double jointAngles2[4];
    double jointVelocities2[4];
    float motorTorques[8];
    float jointTorques1[4];
    float jointTorques2[4];
    float velData[8];

    dxl_bus_left->GetMultPositions(dxl_position_left, dxl_left_ids, 3);
    dxl_bus_right->GetMultPositions(dxl_position_right, dxl_right_ids, 3);
    dxl_bus_wrist->GetMultPositions(dxl_position_wrist, dxl_wrist_ids, 3);

    dxl_bus_left->GetMultVelocities(dxl_velocity_left, dxl_left_ids, 3);
    dxl_bus_right->GetMultVelocities(dxl_velocity_right, dxl_right_ids, 3);
    dxl_bus_wrist->GetMultVelocities(dxl_velocity_wrist, dxl_wrist_ids, 3);

    dxl_bus_left->GetMultCurrents(dxl_current_left, dxl_left_ids, 3);
    dxl_bus_right->GetMultCurrents(dxl_current_right, dxl_right_ids, 3);
    dxl_bus_wrist->GetMultCurrents(dxl_current_wrist, dxl_wrist_ids, 3);

    int32_t dxl_position[9] = {dxl_position_left[0], dxl_position_left[1], dxl_position_left[2], dxl_position_wrist[0], dxl_position_right[0], dxl_position_right[1], dxl_position_right[2], dxl_position_wrist[1], dxl_position_wrist[2]};
    int32_t dxl_velocity[9] = {dxl_velocity_left[0], dxl_velocity_left[1], dxl_velocity_left[2], dxl_velocity_wrist[0], dxl_velocity_right[0], dxl_velocity_right[1], dxl_velocity_right[2], dxl_velocity_wrist[1], dxl_velocity_wrist[2]};
    int16_t dxl_current[9] = {dxl_current_left[0], dxl_current_left[1], dxl_current_left[2], dxl_current_wrist[0], dxl_current_right[0], dxl_current_right[1], dxl_current_right[2], dxl_current_wrist[1], dxl_current_wrist[2]};

    for(int i=0; i<8; i++){
        velData[i] = rpm_to_rads*(float)dxl_velocity[i];
        motorTorques[i] = Kt*0.001f*(float)dxl_current[i];
    }

    // pc.printf("Transforms...");
    InverseActuatorTransformationL(jointAngles1, dxl_position[0], dxl_position[1], dxl_position[2], dxl_position[3]);
    InverseActuatorVelocityTransformationL(jointVelocities1, velData[0], velData[1], velData[2], velData[3]);
    InverseActuatorTransformationR(jointAngles2, dxl_position[4], dxl_position[5], dxl_position[6], dxl_position[7]);
    InverseActuatorVelocityTransformationR(jointVelocities2, velData[4], velData[5], velData[6], velData[7]);

    for(int i=0; i<4; i++){
        // convert from actuator torques to joint torques: tauJ = Jact^T * tauM
        jointTorques1[i] = JactL[0][i]*motorTorques[0] + JactL[1][i]*motorTorques[1] + JactL[2][i]*motorTorques[2] + JactL[3][i]*motorTorques[3];
        jointTorques2[i] = JactR[0][i]*motorTorques[4] + JactR[1][i]*motorTorques[5] + JactR[2][i]*motorTorques[6] + JactR[3][i]*motorTorques[7];
    }

    // pc.printf("Current values...");
    for(int i=0; i<3; i++){
        currentPos[i] = (float)jointAngles1[i];
        currentVel[i] = (float)jointVelocities1[i];
        currentPos[3+i] = (float)jointAngles2[i];
        currentVel[3+i] = (float)jointVelocities2[i];
        currentCur[i] = 0.001f*(float)dxl_current[i];
        currentCur[3+i] = 0.001f*(float)dxl_current[4+i];
        currentJointTau[i] = (float)jointTorques1[i];
        currentJointTau[3+i] = (float)jointTorques2[i];
    }
    currentPos[6] = (float)jointAngles1[3];
    currentVel[6] = (float)jointVelocities1[3];
    currentPos[7] = (float)jointAngles2[3];
    currentVel[7] = (float)jointVelocities2[3];
    currentCur[6] = 0.001f*(float)dxl_current[3];
    currentCur[7] = 0.001f*(float)dxl_current[7];
    currentJointTau[6] = (float)jointTorques1[3];
    currentJointTau[7] = (float)jointTorques2[3];

    currentPos[8] = (float(dxl_position[8])-2048.0f)*((2*PI/4096.0f)); // (float(x)-2048) * ((2*PI)/4096.0f)
    currentVel[8] = rpm_to_rads*(float)dxl_velocity[8];
    currentCur[8] = 0.001f*(float)dxl_current[8];
    currentJointTau[8] = Kt_WR*0.001f*(float)dxl_current[8];

    // calculate desired joint torques here, torques are in Nm
    // pc.printf("Desired joint torques...");
    float pos_act[9] = {(float)jointAngles1[0], (float)jointAngles1[1], (float)jointAngles1[2], (float)jointAngles1[3], (float)jointAngles2[0], (float)jointAngles2[1], (float)jointAngles2[2], (float)jointAngles2[3], (float)currentPos[8]};
    float vel_act[9] = {(float)jointVelocities1[0], (float)jointVelocities1[1], (float)jointVelocities1[2], (float)jointVelocities1[3], (float)jointVelocities2[0], (float)jointVelocities2[1], (float)jointVelocities2[2], (float)jointVelocities2[3], (float)currentVel[8]};
    
    float pos_des[9] = {dxl_pos_des[0], dxl_pos_des[1], dxl_pos_des[2], dxl_pos_des[6], dxl_pos_des[3], dxl_pos_des[4], dxl_pos_des[5],  dxl_pos_des[7], dxl_pos_des[8]};
    float vel_des[9] = {dxl_vel_des[0], dxl_vel_des[1], dxl_vel_des[2],  dxl_vel_des[6], dxl_vel_des[3], dxl_vel_des[4], dxl_vel_des[5],  dxl_vel_des[7], dxl_vel_des[8]};
    
    float KP_des[9] = {dxl_kp[0],dxl_kp[1],dxl_kp[2], dxl_kp[6], dxl_kp[3], dxl_kp[4], dxl_kp[5], dxl_kp[7], dxl_kp[8]}; // 0.8f*1.76f
    float KD_des[9] = {dxl_kd[0],dxl_kd[1],dxl_kd[2], dxl_kd[6], dxl_kd[3], dxl_kd[4], dxl_kd[5],dxl_kd[7], dxl_kd[8]}; // 0.6f*0.026f
    float tau_ff_des[9] = {dxl_tff_des[0],dxl_tff_des[1],dxl_tff_des[2],dxl_tff_des[6],dxl_tff_des[3],dxl_tff_des[4],dxl_tff_des[5],dxl_tff_des[7], dxl_tff_des[8] };
    
    float desired_joint_torques[9];
    float desired_actuator_torques[9];

    uint32_t abad_pos[2] = {(uint32_t)round(rad2pulse((15.98f/15.48f)* dxl_pos_des[6])), (uint32_t)round(rad2pulse((15.98f/15.48f) * -dxl_pos_des[7]))};

    for(int i=0; i<9; i++){
        desired_joint_torques[i] = KP_des[i]*(pos_des[i]-pos_act[i]) + KD_des[i]*(vel_des[i]-vel_act[i]) + tau_ff_des[i];
        // desired_joint_torques[i] = -0.1f*pos_act[i] - 0.01f*vel_act[i];
    }
    // convert to desired actuator torques
    for(int i=0; i<4; i++){
        desired_actuator_torques[i] = JjointL[0][i]*desired_joint_torques[0] + JjointL[1][i]*desired_joint_torques[1] + JjointL[2][i]*desired_joint_torques[2] + JjointL[3][i]*desired_joint_torques[3];
        desired_actuator_torques[i+4] = JjointR[0][i]*desired_joint_torques[4] + JjointR[1][i]*desired_joint_torques[5] + JjointR[2][i]*desired_joint_torques[6] + JjointR[3][i]*desired_joint_torques[7];
    }
    desired_actuator_torques[8] = desired_joint_torques[8]; 

    // convert to desired currents
    // pc.printf("Current commands.\n\r");
    for(int i = 0; i<idLength; i++){
        desired_current[i] = fmaxf(fminf( Ktinv*desired_actuator_torques[i], current_limit ), -current_limit );
        // current_command[i] = (int16_t)(1000.0f*desired_current[i]); // commanded current is in mA!
        current_command[i] = (int16_t)(1000.0f*desired_current[i]); // commanded current is in mA!
    }
    // different parameters for wrist roll motor
    desired_current[8] = fmaxf(fminf( Ktinv_WR*desired_actuator_torques[8], current_limit_WR ), -current_limit_WR );
    current_command[8] = (int16_t)(1000.0f*desired_current[8]);

    current_command[3] = (int16_t)(dxl_tff_des[6]/0.00269); //Abad joints, current limiting at 1193 * 2.69mA = 3209.17 mA
    current_command[7] = (int16_t)(dxl_tff_des[7]/0.00269); //Abad joints, current limiting at 1193 * 2.69mA = 3209.17 mA
    
    // pc.printf("Current Limits Commands m4: %d \n\r",current_command[3]);
    // pc.printf("Current Limits Commands m8: %d \n\r",current_command[7]);
    
    // zero out commands for the second finger
    // current_command[4] = 0;
    // current_command[5] = 0;
    // current_command[6] = 0;
    // current_command[7] = 0;
    
    uint16_t leftbus_commands[3] = {current_command[0], current_command[1], current_command[2]};
    uint16_t rightbus_commands[3] = {current_command[4], current_command[5], current_command[6]};
    uint16_t wristbus_commands[3] = {current_command[3], current_command[7], current_command[8]};
    
    dxl_bus_left->SetMultGoalCurrents(dxl_left_ids, 3, leftbus_commands);
    dxl_bus_right->SetMultGoalCurrents(dxl_right_ids, 3, rightbus_commands);
    dxl_bus_wrist->SetMultGoalPositions(dxl_PCIDs, 2, abad_pos);
    dxl_bus_wrist->SetMultGoalCurrents(dxl_wrist_ids, 3, wristbus_commands);
    

    // pc.printf("Command for finger 1: %.3f, %.3f, %.3f, %.3f\n\r", desired_joint_torques[0], desired_joint_torques[1], desired_joint_torques[2], desired_joint_torques[3]);

    // servo_time = t.read_us();
}

/// CAN Reply Packet Structure ///
/// 16 bit position, between -4*pi and 4*pi
/// 12 bit velocity, between -30 and + 30 rad/s
/// 12 bit current, between -40 and 40;
/// CAN Packet is 5 8-bit words
/// Formatted as follows.  For each quantity, bit 0 is LSB
/// 0: [position[15-8]]
/// 1: [position[7-0]] 
/// 2: [velocity[11-4]]
/// 3: [velocity[3-0], current[11-8]]
/// 4: [current[7-0]]
void pack_reply(CANMessage *msg, int dxl_id, float p, float v, float t){
    int p_int = float_to_uint(p, P_MIN, P_MAX, 16);
    int v_int = float_to_uint(v, V_MIN, V_MAX, 12);
    int t_int = float_to_uint(t*T_SCALE, -T_MAX, T_MAX, 12);
    msg->data[0] = dxl_id;
    msg->data[1] = p_int>>8;
    msg->data[2] = p_int&0xFF;
    msg->data[3] = v_int>>4;
    msg->data[4] = ((v_int&0xF)<<4) + (t_int>>8);
    msg->data[5] = t_int&0xFF;
    }
    
/// CAN Command Packet Structure ///
/// 16 bit position command, between -4*pi and 4*pi
/// 12 bit velocity command, between -30 and + 30 rad/s
/// 12 bit kp, between 0 and 500 N-m/rad
/// 12 bit kd, between 0 and 100 N-m*s/rad
/// 12 bit feed forward torque, between -18 and 18 N-m
/// CAN Packet is 8 8-bit words
/// Formatted as follows.  For each quantity, bit 0 is LSB
/// 0: [position[15-8]]
/// 1: [position[7-0]] 
/// 2: [velocity[11-4]]
/// 3: [velocity[3-0], kp[11-8]]
/// 4: [kp[7-0]]
/// 5: [kd[11-4]]
/// 6: [kd[3-0], torque[11-8]]
/// 7: [torque[7-0]]
void unpack_cmd(CANMessage msg){
    int p_int = (msg.data[0]<<8)|msg.data[1];
    int v_int = (msg.data[2]<<4)|(msg.data[3]>>4);
    int kp_int = ((msg.data[3]&0xF)<<8)|msg.data[4];
    int kd_int = (msg.data[5]<<4)|(msg.data[6]>>4);
    int t_int = ((msg.data[6]&0xF)<<8)|msg.data[7];

    if (msg.id == 0){ //WRIST ROLL
        dxl_pos_des[8] = uint_to_float(p_int, P_MIN, P_MAX, 16);
        dxl_vel_des[8] = uint_to_float(v_int, V_MIN, V_MAX, 12);
        dxl_kp[8] = uint_to_float(kp_int, KP_MIN, KP_MAX, 12)/KP_SCALE;
        dxl_kd[8] = uint_to_float(kd_int, KD_MIN, KD_MAX, 12)/KD_SCALE;
        dxl_tff_des[8] = uint_to_float(t_int, T_MIN, T_MAX, 12)/T_SCALE;
    }
    else{
        dxl_pos_des[msg.id-CAN_TX_DXL1] = uint_to_float(p_int, P_MIN, P_MAX, 16);
        dxl_vel_des[msg.id-CAN_TX_DXL1] = uint_to_float(v_int, V_MIN, V_MAX, 12);
        dxl_kp[msg.id-CAN_TX_DXL1] = uint_to_float(kp_int, KP_MIN, KP_MAX, 12)/KP_SCALE;
        dxl_kd[msg.id-CAN_TX_DXL1] = uint_to_float(kd_int, KD_MIN, KD_MAX, 12)/KD_SCALE;
        dxl_tff_des[msg.id-CAN_TX_DXL1] = uint_to_float(t_int, T_MIN, T_MAX, 12)/T_SCALE;
    }


    }

void unpack_sensor(CANMessage msg){
    /// unpack ints from can buffer ///
    int32_t p_raw_1 = (msg.data[0]<<24)|msg.data[1]<<16|msg.data[2]<<8|msg.data[3];
    int32_t p_raw_2 = (msg.data[4]<<24)|msg.data[5]<<16|msg.data[6]<<8|msg.data[7];

    if (msg.id == 0){
        pressure_raw1[0] = p_raw_1;
        pressure_raw1[1] = p_raw_2;
    }
    else if (msg.id == 1){
        pressure_raw1[2] = p_raw_1;
        pressure_raw1[3] = p_raw_2;
    }
    else if (msg.id == 2){
        pressure_raw1[4] = p_raw_1;
        pressure_raw1[5] = p_raw_2;

    }
    else if (msg.id == 3){
        pressure_raw1[6] = p_raw_1;
        pressure_raw1[7] = p_raw_2;
    }
    
    else if (msg.id == 5){
        pressure_raw2[0] = p_raw_1;
        pressure_raw2[1] = p_raw_2;
    }
    else if (msg.id == 6){
        pressure_raw2[2] = p_raw_1;
        pressure_raw2[3] = p_raw_2;
    }
    else if (msg.id == 7){
        pressure_raw2[4] = p_raw_1;
        pressure_raw2[5] = p_raw_2;

    }
    else if (msg.id == 8){
        pressure_raw2[6] = p_raw_1;
        pressure_raw2[7] = p_raw_2;
    }

    else if (msg.id == 4){
        for(int i = 0;i<8;i++){
            tof1[i] = msg.data[i];
        }
    }
    else if (msg.id == 9){
        for(int i = 0;i<8;i++){
            tof2[i] = msg.data[i];
        }
    }
    else if (msg.id == 10){
        palmTOF = msg.data[0];
    }
}

void pack_force_reply(CANMessage * msg, ForceSensor * fs){
     
     /// limit data to be within bounds ///
     float fx_temp = fminf(fmaxf(FT_MIN, fs->output_data[0]), FT_MAX);
     float fy_temp = fminf(fmaxf(FT_MIN, fs->output_data[1]), FT_MAX);   
     float fz_temp = fminf(fmaxf(FN_MIN, fs->output_data[2]), FN_MAX);   
     float theta_temp = fminf(fmaxf(ANG_MIN, fs->output_data[3]), ANG_MAX);   
     float phi_temp = fminf(fmaxf(ANG_MIN, fs->output_data[4]), ANG_MAX);                       
     /// convert floats to unsigned ints ///
     uint16_t fx_int = float_to_uint(fx_temp, FT_MIN, FT_MAX, 12); 
     uint16_t fy_int = float_to_uint(fy_temp, FT_MIN, FT_MAX, 12);  
     uint16_t fz_int = float_to_uint(fz_temp, FN_MIN, FN_MAX, 12);  
     uint16_t theta_int = float_to_uint(theta_temp, ANG_MIN, ANG_MAX, 12);  
     uint16_t phi_int = float_to_uint(phi_temp, ANG_MIN, ANG_MAX, 12);             
     /// pack ints into the can buffer ///
     msg->data[0] = (fs->_channel<<4)|(fx_int>>8);                                       
     msg->data[1] = fx_int&0xFF;
     msg->data[2] = fy_int>>4;
     msg->data[3] = ((fy_int&0x0F)<<4)|(fz_int>>8);
     msg->data[4] = fz_int&0xFF;
     msg->data[5] = theta_int>>4;
     msg->data[6] = ((theta_int&0x0F)<<4)|(phi_int>>8);
     msg->data[7] = phi_int&0xFF;
     }


/// ToF Sensor CAN Reply Packet Structure ///
/// 5 x 8bit range measurements
/// CAN packet is 6 8-bit words
/// Formatted as follows.  For each quantity, bit 0 is LSB
/// 0: finger ID (left is 0, right is 1)
/// 1: [tof[7-0]] 
/// 2: [tof[7-0]] 
/// 3: [tof[7-0]] 
/// 4: [tof[7-0]] 
/// 5: [tof[7-0]]
/// left finger is sensors 1,2,3,4,5
/// right finger is sensors 6,7,8,9,~ 

void pack_tof_reply(CANMessage * msg, uint8_t finger){

    /// pack ints into the can buffer ///   
    if (finger==0){
    msg->data[0] = tof1[0];
    msg->data[1] = tof1[1];
    msg->data[2] = tof1[2];
    msg->data[3] = tof1[3];
    msg->data[4] = palmTOF;
    } else if (finger==1){
    msg->data[0] = tof2[0];
    msg->data[1] = tof2[1];
    msg->data[2] = tof2[2];
    msg->data[3] = tof2[3]; 
    }
}


void pollSysCAN(){
    if(cansys.read(rxMsg)){ // TODO: move this to a check_CAN function
        if((rxMsg.id>=0)&&(rxMsg.id<=8)){
            // pc.printf("Rcvd: %d\n\r", rxMsg.id);
            unpack_cmd(rxMsg); // unpack command
            // pc.printf("Received: %d\n\r", rxMsg.id);
        }
    }
}

void pollSensCAN(){
    if(cansens.read(sensorRxMsg)){ // TODO: move this to a check_CAN function
        unpack_sensor(sensorRxMsg); // unpack command
    }
}


void sendCAN(){
    pack_reply(&txDxl1, dxl_ID[0], currentPos[0], currentVel[0], currentJointTau[0]); // Kt*currentCur[0]); 
    cansys.write(txDxl1);
    wait_us(100);
    pack_reply(&txDxl2, dxl_ID[1], currentPos[1], currentVel[1], currentJointTau[1]); //Kt*currentCur[1]); 
    cansys.write(txDxl2);
    wait_us(100);
    pack_reply(&txDxl3, dxl_ID[2], currentPos[2], currentVel[2], currentJointTau[2]); //Kt*currentCur[2]); 
    cansys.write(txDxl3);
    wait_us(100);
    pack_reply(&txDxl4, dxl_ID[3], currentPos[3], currentVel[3], currentJointTau[3]); //Kt*currentCur[3]); 
    cansys.write(txDxl4);
    wait_us(100);
    pack_reply(&txDxl5, dxl_ID[4], currentPos[4], currentVel[4], currentJointTau[4]); //Kt*currentCur[4]); 
    cansys.write(txDxl5);
    wait_us(100);
    pack_reply(&txDxl6, dxl_ID[5], currentPos[5], currentVel[5], currentJointTau[5]); //Kt*currentCur[5]); 
    cansys.write(txDxl6);
    wait_us(100);
    pack_reply(&txDxl7, dxl_ID[6], currentPos[6], currentVel[6], currentJointTau[6]); //Kt*currentCur[5]); 
    cansys.write(txDxl7);
    wait_us(100);
    pack_reply(&txDxl8, dxl_ID[7], currentPos[7], currentVel[7], currentJointTau[7]); //Kt*currentCur[5]); 
    cansys.write(txDxl8);
    wait_us(100);
    pack_tof_reply(&txTOF1, 0);
    cansys.write(txTOF1);
    wait_us(100);
    pack_tof_reply(&txTOF2, 1);
    cansys.write(txTOF2);
    wait_us(100);
    pack_force_reply(&txForce1, &forcesensor1);
    cansys.write(txForce1);
    wait_us(100);
    pack_force_reply(&txForce2, &forcesensor2);
    cansys.write(txForce2);
    wait_us(100);
    pack_reply(&txDxl9, 0, currentPos[8], currentVel[8], currentJointTau[8]); //Kt*currentCur[5]); 
    cansys.write(txDxl9);
    wait_us(100);
}
// Main loop, receiving commands as fast as possible, then writing to a dynamixel
int main() {
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
    wait_us(2000000);
    // XM430_bus dxl_bus(4000000, PC_10, PC_11, PA_6); // baud, tx, rx, rts
    XM430_bus dxl_bus_left(2000000, PC_10, PC_11, PA_4); // baud, tx, rx, rts, left finger
    XM430_bus dxl_bus_right(2000000, PC_12, PC_2, PA_5); // baud, tx, rx, rts, right finger
    XM430_bus dxl_bus_wrist(2000000, PC_6, PC_7, PA_7); // baud, tx, rx, rts, wrist and Ab/Ads
    wait_us(1000000);

    // Set up CAN
    // cansys.filter(CAN_ID , 0xFFF, CANStandard, 0); 
    txDxl1.id = CAN_RX_DXL1;
    txDxl1.len = 8;
    txDxl2.id = CAN_RX_DXL2;
    txDxl2.len = 8;
    txDxl3.id = CAN_RX_DXL3;
    txDxl3.len = 8;
    txDxl4.id = CAN_RX_DXL4;
    txDxl4.len = 8;
    txDxl5.id = CAN_RX_DXL5;
    txDxl5.len = 8;
    txDxl6.id = CAN_RX_DXL6;
    txDxl6.len = 8;
    txDxl7.id = CAN_RX_DXL7;
    txDxl7.len = 8;
    txDxl8.id = CAN_RX_DXL8;
    txDxl8.len = 8;
    txDxl9.id = CAN_RX_DXL9;
    txDxl9.len = 8;    
    txForce1.id = CAN_FORCE_1;
    txForce1.len = 8;
    txForce2.id = CAN_FORCE_2;
    txForce2.len = 8;
    txTOF1.id = CAN_TOF_1;
    txTOF1.len = 8;
    txTOF2.id = CAN_TOF_2;
    txTOF2.len = 8;
    rxMsg.len = 8;

    // dynamixel
    pc.printf("Setting up Dynamixel bus\n\r");
    // Enable dynamixels and set control mode...individual version
    for (int i=0; i<3; i++) {
        dxl_bus_left.SetTorqueEn(dxl_left_ids[i],0x00);
        dxl_bus_left.SetRetDelTime(dxl_left_ids[i],0x05); // 4us delay time?
        dxl_bus_left.SetControlMode(dxl_left_ids[i], CURRENT_CONTROL);
        wait_us(100000);
        dxl_bus_left.TurnOnLED(dxl_left_ids[i], 0x01);
        pc.printf("Init Dynamixel ID: %d \n\r",dxl_left_ids[i]);
        //dxl_bus.TurnOnLED(dxl_ID[i], 0x00); // turn off LED
        dxl_bus_left.SetTorqueEn(dxl_left_ids[i],0x01); //to be able to move 
        wait_us(100000);
    }

    for (int i=0; i<3; i++) {
        dxl_bus_right.SetTorqueEn(dxl_right_ids[i],0x00);
        dxl_bus_right.SetRetDelTime(dxl_right_ids[i],0x05); // 4us delay time?
        dxl_bus_right.SetControlMode(dxl_right_ids[i], CURRENT_CONTROL);
        wait_us(100000);
        dxl_bus_right.TurnOnLED(dxl_right_ids[i], 0x01);
        pc.printf("Init Dynamixel ID: %d \n\r",dxl_right_ids[i]);
        //dxl_bus.TurnOnLED(dxl_ID[i], 0x00); // turn off LED
        dxl_bus_right.SetTorqueEn(dxl_right_ids[i],0x01); //to be able to move 
        wait_us(100000);
    }


    for (int i=0; i<3; i++) {
        if (i == 2) {
        dxl_bus_wrist.SetTorqueEn(dxl_wrist_ids[i],0x00);
        dxl_bus_wrist.SetRetDelTime(dxl_wrist_ids[i],0x05); // 4us delay time?
        dxl_bus_wrist.SetControlMode(dxl_wrist_ids[i], CURRENT_CONTROL);
        wait_us(100000);
        dxl_bus_wrist.TurnOnLED(dxl_wrist_ids[i], 0x01);
        pc.printf("Init Dynamixel ID: %d \n\r",dxl_wrist_ids[i]);
        //dxl_bus.TurnOnLED(dxl_ID[i], 0x00); // turn off LED
        dxl_bus_wrist.SetTorqueEn(dxl_wrist_ids[i],0x01); //to be able to move 
        wait_us(100000);
        }
        else { 
        dxl_bus_wrist.SetTorqueEn(dxl_wrist_ids[i],0x00);
        dxl_bus_wrist.SetRetDelTime(dxl_wrist_ids[i],0x05); // 4us delay time?
        dxl_bus_wrist.SetControlMode(dxl_wrist_ids[i], CURRENT_POS_CONTROL);
        dxl_bus_wrist.SetVelocityProfile(dxl_wrist_ids[i], 0);
        dxl_bus_wrist.SetAccelerationProfile(dxl_wrist_ids[i], 0);
        dxl_bus_wrist.SetPosPGain(dxl_wrist_ids[i], 800);
        dxl_bus_wrist.SetPosDGain(dxl_wrist_ids[i], 15000);
        wait_us(100000);
        dxl_bus_wrist.TurnOnLED(dxl_wrist_ids[i], 0x01);
        pc.printf("Init Dynamixel ID: %d \n\r",dxl_wrist_ids[i]);
        //dxl_bus.TurnOnLED(dxl_ID[i], 0x00); // turn off LED
        dxl_bus_wrist.SetTorqueEn(dxl_wrist_ids[i],0x01); //to be able to move 
        wait_us(100000);
        }
    }

    Timer t;
    t.start();
    while(t.read() < 1.0){
        pollSensCAN();
    }
        
    forcesensor1.Initialize();
    forcesensor1.Sample();
    forcesensor1.Calibrate();
    wait_us(10000);
    forcesensor2.Initialize();
    forcesensor2.Sample();
    forcesensor2.Calibrate();
    wait_us(10000);

    pc.printf("Starting\r\n");
    CANTimer.start();
    loopTimer.reset();

    while(true){
    loopTimer.start();
        
        //get all the messages from the bus 
        pollSensCAN();
        pollSysCAN();
        pollSensCAN();
        pollSysCAN();
        pollSensCAN();
        pollSysCAN();
        pollSensCAN();
        pollSysCAN();
        pollSensCAN();
        pollSysCAN();
        pollSensCAN();
        pollSysCAN();
        pollSensCAN();
        pollSysCAN();
        pollSensCAN();
        pollSysCAN();
        pollSensCAN();
        pollSysCAN();
        pollSensCAN();
        pollSensCAN();

        // pc.printf("DXL bus\n\r");
        updateBus(&dxl_bus_left, &dxl_bus_right, &dxl_bus_wrist);
        forcesensor1.Sample();
        forcesensor1.Evaluate();
        forcesensor2.Sample();
        forcesensor2.Evaluate();
        //sendCAN();

        if (CANTimer.read() > 0.003){
            sendCAN();
            CANTimer.reset();
        }

    loopTime = loopTimer.read_us();
    loopTimer.reset();
    } 
    
}