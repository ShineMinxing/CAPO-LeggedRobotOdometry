#ifndef __CONTROL_FRAME_LOWLEVELSTATE_H__
#define __CONTROL_FRAME_LOWLEVELSTATE_H__

#include <iostream>
#include <vector>
#include <cstdint>

#define MOTOR_NUM 16

struct MotorState
{
    unsigned int mode;
    float q;
    float dq;
    float ddq;
    float tauEst;
    float temp;
    unsigned int cnt;

    MotorState()
    {
        q = 0;
        dq = 0;
        ddq = 0;
        tauEst = 0;
    }
};

struct IMU
{
    float quaternion[4];
    float gyroscope[3];
    float accelerometer[3];
    float pitch, roll, yaw;
    int64_t timestamp;
    IMU()
    {
        for (int i = 0; i < 3; i++)
        {
            quaternion[i] = 0;
            gyroscope[i] = 0;
            accelerometer[i] = 0;
        }
        quaternion[3] = 0;
        pitch = 0.0f;
        roll = 0.0f;
        yaw = 0.0f;
    }
};

struct Proprioception
{
    float PositionXYZ[9]; // 0-2: X Position Velocity Acceleration, 3-5: Y Position Velocity Acceleration, 6-8: Z Position Velocity Acceleration
    float OrientationRPY[9]; // 0-2: Roll Orientation Velocity Acceleration, 3-5: Pitch Orientation Velocity Acceleration, 6-8: Yaw Orientation Velocity Acceleration
    float FootfallAverage[3]; // 0-2: Footfall Average X, Footfall Average Y, Footfall Average Yaw
    float FootLandedProbability[4]; // FL, FR, RL, RR Foot Landed Probability
    float DogWeight;
    int LegCollisionDetect; // 0: No collision, 1: front, 2: front left, 3: left, 4: back left, 5: back, 6: back right, 7: right, 8: front right
    float JointsBodyWFPosition[4][4][3]; // MOTOR_NUM joints, each with X, Y, Z position relative to body in world frame
    float JointsBodyWFEffort[4][4][3]; // MOTOR_NUM joints, each with X, Y, Z position  relative to body in world frame
    float MotorGravityCompensate[4][3]; // FL, FR, RL, RR; Hip, Thigh, Calf gravity compensation torque

    Proprioception()
        : PositionXYZ{}, OrientationRPY{}, FootfallAverage{}, FootLandedProbability{},
        DogWeight(0.0f), LegCollisionDetect(0), JointsBodyWFPosition{}, JointsBodyWFEffort{}
    {}
};


struct LowlevelState
{
    IMU imu;                          // imu
    MotorState motorState[MOTOR_NUM]; // 电机状态
    Proprioception proprioception;    // 本体感知
};

#endif /* __CONTROL_FRAME_LOWLEVELSTATE_H__ */
