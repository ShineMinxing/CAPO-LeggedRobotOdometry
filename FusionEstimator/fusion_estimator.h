#ifndef __FUSION_ESTIMATOR_H_
#define __FUSION_ESTIMATOR_H_

/*
Proprioceptive legged odometry overview
本体感知腿式里程计说明

Purpose:
作用：
This estimator computes robot odometry using IMU and joint motor data.
It estimates body position, velocity, acceleration, orientation, angular velocity,
angular acceleration, foot contact probability, support-center position,
loaded weight, joint positions, joint equivalent forces, collision direction,
and optional joint gravity compensation.

```
本估计器使用 IMU 和关节电机数据计算机器人本体里程计。
它估计机体位置、速度、加速度、姿态角、角速度、角加速度、
足端触地概率、支撑中心位置、负载重量、关节位置、关节等效力、
碰撞方向，以及可选的关节重力补偿力矩。
```

Main entry:
主要入口：
1. Initialization:
初始化：
#include "fusion_estimator.h"
auto Robot_Estimation = CreateRobot_Estimation();

```
2. Runtime estimation:
   运行时估计：
       #include "LowlevelState.h"
       LowlevelState st{};
       Odometer odom = Robot_Estimation.fusion_estimator(st);

3. Runtime configuration:
   运行时配置：
       double status[100] = {0};
       Robot_Estimation.fusion_estimator_status(status);
```

Configuration index meaning:
配置索引说明：

```
IndexInOrOut = 0
IndexStatusOK = 1

IndexIMUAccEnable = 2
IndexIMUQuaternionEnable = 3
IndexIMUGyroEnable = 4

IndexJointsXYZEnable = 5
IndexJointsXYZVelEnable = 6
IndexJointsRPYEnable = 7
IndexJointsRPYAccEnable = 8

IndexSlopeModeTimeThreshold = 10
IndexSlopeModeAngleThreshold = 11
IndexLegFootForceThreshold = 12
IndexLegMinStairHeight = 13
IndexStairHeightFogotten = 14

IndexLegOrientationInitialWeight = 15
IndexLegOrientationTimeWeight = 16

IndexSlopeEstimationEnable = 17
IndexCollisionDetectEnable = 18
IndexGravityCompensateEnable = 19
```

Runtime control:
运行控制：
status[IndexInOrOut] == 1:
    write estimator parameters
    写入估计器参数
status[IndexInOrOut] == 2:
    read estimator parameters
    读取估计器参数
status[IndexInOrOut] == 3:
    reset estimated X/Y position and realign yaw correction
    将估计的 X/Y 位置清零，并重新对齐 yaw 修正量

status[IndexInOrOut] == 98:
    使用 Go2 轮足参数
status[IndexInOrOut] == 99:
    使用 Go2 点足参数
status[IndexInOrOut] == 121:
    使用 SP 参数
status[IndexInOrOut] == 140:
    使用 MW_D 参数
status[IndexInOrOut] == 141:
    使用 MP_A 参数
status[IndexInOrOut] == 142:
    使用 MW_B 参数
status[IndexInOrOut] == 160:
    使用 LW 参数
```

Algorithm pipeline:
算法流程：
1. IMU acceleration updates body acceleration states.
IMU 加速度更新机体加速度状态。

```
2. IMU quaternion and gyroscope update orientation and angular velocity states.
   IMU 四元数和角速度更新姿态与角速度状态。

3. Joint angles, velocities, and torques are converted into world-frame
   joint positions, foot velocity, and joint equivalent forces.
   根据关节角度、角速度和力矩计算世界系关节位置、
   足端速度和关节等效力。

4. Foot contact probability and support state are estimated from foot force.
   根据足端受力估计触地概率和支撑状态。

5. Supporting-foot positions and velocities update body position,
   velocity, and acceleration states.
   使用支撑足位置和速度更新机体位置、速度和加速度状态。

6. Wheel-foot models compensate wheel rolling, shank motion,
   lateral tire motion, and ground slope.
   轮足模型补偿轮子滚动、小腿摆动、轮胎侧向运动和地面坡度。

7. Foot geometry optionally corrects yaw when all feet are on the ground.
   全部足端着地时，可使用足端几何关系修正 yaw。

8. Foot forces optionally estimate loaded weight, angular acceleration,
   collision direction, and joint gravity compensation.
   足端受力可用于估计负载重量、角加速度、碰撞方向和关节重力补偿。
```

*/

#include <memory>
#include <vector>
#include <cmath>
#include <cstdio>
#include <iostream>

#include "SensorBase.h"
#include "Sensor_Legs.h"
#include "Sensor_IMU.h"
// #include "../Controller/ControlFrame/LowlevelState.h"
#include "LowlevelState.h"

enum ConfigIndex {

    IndexInOrOut = 0,
    IndexStatusOK = 1,
    IndexIMUAccEnable = 2,  
    IndexIMUQuaternionEnable = 3,
    IndexIMUGyroEnable = 4,  
    IndexJointsXYZEnable = 5,
    IndexJointsXYZVelEnable = 6,
    IndexJointsRPYEnable = 7,
    IndexJointsRPYAccEnable = 8,

    IndexSlopeModeTimeThreshold = 10,
    IndexSlopeModeAngleThreshold = 11,
    IndexLegFootForceThreshold = 12,  
    IndexLegMinStairHeight = 13,
    IndexStairHeightFogotten = 14,

    IndexLegOrientationInitialWeight = 15, 
    IndexLegOrientationTimeWeight = 16,

    IndexSlopeEstimationEnable = 17,
    IndexCollisionDetectEnable = 18,
    IndexGravityCompensateEnable = 19,
};

class FusionEstimatorCore
{
public:
    FusionEstimatorCore()
    {
        for (int i = 0; i < 2; ++i) {
            auto *ptr = new EstimatorPortN;
            StateSpaceModel_Go2_Initialization(ptr);
            sensors.emplace_back(ptr);
        }

        imu_acc     = std::make_shared<DataFusion::SensorIMUAcc>    (sensors[0]);
        imu_gyro    = std::make_shared<DataFusion::SensorIMUMagGyro>(sensors[1]);
        legs_pos    = std::make_shared<DataFusion::SensorLegsPos>   (sensors[0]);
        legs_ori    = std::make_shared<DataFusion::SensorLegsOri>   (sensors[1]);
        legs_ori->SetLegsPosRef(legs_pos.get());

        legs_ori->yaw_correct = 0.0;
    }

    ~FusionEstimatorCore()
    {
        for (auto* p : sensors) 
        {
            StateSpaceModel_Go2_EstimatorPortTermination(p);
            delete p;
        }
        sensors.clear();
    }

    FusionEstimatorCore(const FusionEstimatorCore&) = delete;
    FusionEstimatorCore& operator=(const FusionEstimatorCore&) = delete;

    FusionEstimatorCore(FusionEstimatorCore&&) noexcept = default;
    FusionEstimatorCore& operator=(FusionEstimatorCore&&) noexcept = default;

    void fusion_estimator_status(double status[100])
    {
        if (status[IndexInOrOut] == 1) 
        {
            status[IndexInOrOut] = 0;
            status[IndexStatusOK] = status[IndexStatusOK] + 1;

            if(!(status[IndexIMUAccEnable]||status[IndexIMUQuaternionEnable]||status[IndexIMUGyroEnable]||status[IndexJointsXYZEnable]||status[IndexJointsRPYEnable]))
                status[IndexStatusOK] = -999;

            imu_acc->IMUAccEnable               = status[IndexIMUAccEnable];
            imu_gyro->IMUQuaternionEnable       = status[IndexIMUQuaternionEnable];
            imu_gyro->IMUGyroEnable             = status[IndexIMUGyroEnable];
            legs_pos->JointsXYZEnable           = status[IndexJointsXYZEnable];
            legs_pos->JointsXYZVelocityEnable   = status[IndexJointsXYZVelEnable];
            legs_ori->JointsRPYEnable           = status[IndexJointsRPYEnable];
            legs_ori->JointsRPYAccEnable        = status[IndexJointsRPYAccEnable];

            legs_pos->SlopeModeTimeThreshold    = status[IndexSlopeModeTimeThreshold];
            legs_pos->SlopeModeAngleThreshold   = status[IndexSlopeModeAngleThreshold];
            legs_pos->FootEffortThreshold       = status[IndexLegFootForceThreshold];
            legs_pos->Environement_Height_Scope = status[IndexLegMinStairHeight];
            legs_pos->Data_Fading_Time          = status[IndexStairHeightFogotten];
            
            legs_ori->legori_init_weight        = status[IndexLegOrientationInitialWeight];
            legs_ori->legori_time_weight        = status[IndexLegOrientationTimeWeight];
            
            legs_pos->SlopeModeEnable           = status[IndexSlopeEstimationEnable];
            legs_ori->CollisionDetectEnable     = status[IndexCollisionDetectEnable];
            legs_pos->GravityCompensateEnable   = status[IndexGravityCompensateEnable];
        }
        else if (status[IndexInOrOut] == 2){
            status[IndexInOrOut] = 0;
            status[IndexStatusOK] = status[IndexStatusOK] + 10;
            if (status[IndexStatusOK] > 999)
                status[IndexStatusOK] = 1;

            status[IndexIMUAccEnable]                = imu_acc->IMUAccEnable;
            status[IndexIMUQuaternionEnable]         = imu_gyro->IMUQuaternionEnable;
            status[IndexIMUGyroEnable]               = imu_gyro->IMUGyroEnable;
            status[IndexJointsXYZEnable]             = legs_pos->JointsXYZEnable;
            status[IndexJointsXYZVelEnable]          = legs_pos->JointsXYZVelocityEnable;
            status[IndexJointsRPYEnable]             = legs_ori->JointsRPYEnable;
            status[IndexJointsRPYAccEnable]          = legs_ori->JointsRPYAccEnable;

            status[IndexSlopeModeTimeThreshold]      = legs_pos->SlopeModeTimeThreshold ;
            status[IndexSlopeModeAngleThreshold]     = legs_pos->SlopeModeAngleThreshold;
            status[IndexLegFootForceThreshold]       = legs_pos->FootEffortThreshold;
            status[IndexLegMinStairHeight]           = legs_pos->Environement_Height_Scope;
            status[IndexStairHeightFogotten]         = legs_pos->Data_Fading_Time ;

            status[IndexLegOrientationInitialWeight] = legs_ori->legori_init_weight;
            status[IndexLegOrientationTimeWeight]    = legs_ori->legori_time_weight;
            
            status[IndexSlopeEstimationEnable]       = legs_pos->SlopeModeEnable;
            status[IndexCollisionDetectEnable]       = legs_ori->CollisionDetectEnable;
            status[IndexGravityCompensateEnable]     = legs_pos->GravityCompensateEnable;
        }
        else if (status[IndexInOrOut] == 3){
            status[IndexInOrOut] = 0;
            status[IndexStatusOK] = status[IndexStatusOK] + 20;
            if (status[IndexStatusOK] > 999)
                status[IndexStatusOK] = 1;
            sensors[0]->EstimatedState[0] = 0;
            sensors[0]->EstimatedState[3] = 0;
            legs_ori->yaw_correct = legs_ori->yaw_correct - sensors[1]->EstimatedState[6];
            legs_pos->FootfallPositionRecordIsInitiated[0] = false;
            legs_pos->FootfallPositionRecordIsInitiated[1] = false;
            legs_pos->FootfallPositionRecordIsInitiated[2] = false;
            legs_pos->FootfallPositionRecordIsInitiated[3] = false;
        }
        else if (status[IndexInOrOut] == 98){
            status[IndexInOrOut] = 0;
            status[IndexStatusOK] = status[IndexStatusOK] + 98;
            if (status[IndexStatusOK] > 999)
                status[IndexStatusOK] = 1;
            legs_pos->UseGo2W();
        }
        else if (status[IndexInOrOut] == 99){
            status[IndexInOrOut] = 0;
            status[IndexStatusOK] = status[IndexStatusOK] + 99;
            if (status[IndexStatusOK] > 999)
                status[IndexStatusOK] = 1;
            legs_pos->UseGo2P();
        }
        else if (status[IndexInOrOut] == 121){
            status[IndexInOrOut] = 0;
            status[IndexStatusOK] = status[IndexStatusOK] + 121;
            if (status[IndexStatusOK] > 999)
                status[IndexStatusOK] = 1;
            legs_pos->UseSP();
        }
        else if (status[IndexInOrOut] == 140){
            status[IndexInOrOut] = 0;
            status[IndexStatusOK] = status[IndexStatusOK] + 140;
            if (status[IndexStatusOK] > 999)
                status[IndexStatusOK] = 1;
            legs_pos->UseMW_D();
        }
        else if (status[IndexInOrOut] == 141){
            status[IndexInOrOut] = 0;
            status[IndexStatusOK] = status[IndexStatusOK] + 141;
            if (status[IndexStatusOK] > 999)
                status[IndexStatusOK] = 1;
            legs_pos->UseMP_A();
        }
        else if (status[IndexInOrOut] == 142){
            status[IndexInOrOut] = 0;
            status[IndexStatusOK] = status[IndexStatusOK] + 142;
            if (status[IndexStatusOK] > 999)
                status[IndexStatusOK] = 1;
            legs_pos->UseMW_B();
        }
        else if (status[IndexInOrOut] == 160){
            status[IndexInOrOut] = 0;
            status[IndexStatusOK] = status[IndexStatusOK] + 160;
            if (status[IndexStatusOK] > 999)
                status[IndexStatusOK] = 1;
            legs_pos->UseLW();
        }
    }

    Proprioception fusion_estimator(const LowlevelState& st)
    {
        Proprioception proprio;
        
        double q[4] = {
            static_cast<double>(st.imu.quaternion[0]),
            static_cast<double>(st.imu.quaternion[1]),
            static_cast<double>(st.imu.quaternion[2]),
            static_cast<double>(st.imu.quaternion[3])
        };

        FLAG IsQuaternionOK = 0;
        array_quaternion_check(q, &IsQuaternionOK);
        if (!IsQuaternionOK||!imu_gyro->IMUQuaternionEnable)
        {
            if(!legs_ori->JointsRPYEnable)
                return proprio;
            else
            {
                q[0] = 1;
                q[1] = 0;
                q[2] = 0;
                q[3] = 0;
            }
        }
        else
            array_quaternion_normalize(q, q);
        
        const double CurrentTimestamp = 1e-3 * static_cast<double>(st.imu.timestamp);

        if (!(CurrentTimestamp - StartTimeStamp - LastUsedTimestamp < 1) || !(CurrentTimestamp - StartTimeStamp - LastUsedTimestamp >0))
            StartTimeStamp = CurrentTimestamp - LastUsedTimestamp;

        double UsedTimestamp = CurrentTimestamp - StartTimeStamp;
        LastUsedTimestamp = UsedTimestamp;

        if (imu_acc->IMUAccEnable) {
            double msg_acc[9] = {0};
            msg_acc[3*0 + 2] = static_cast<double>(st.imu.accelerometer[0]);
            msg_acc[3*1 + 2] = static_cast<double>(st.imu.accelerometer[1]);
            msg_acc[3*2 + 2] = static_cast<double>(st.imu.accelerometer[2]);
            
            if(Signal_Available_Check(msg_acc,0))
                imu_acc->SensorDataHandle(msg_acc, UsedTimestamp);
        }
        
        double array_EulerZYX[3] = {0.0, 0.0, 0.0};
        array_quaternion_to_eulerZYX(q, array_EulerZYX);
        double roll = array_EulerZYX[0];
        double pitch = array_EulerZYX[1];
        double yaw = array_EulerZYX[2];

        double msg_rpy[9] = {0};

        msg_rpy[3*0] = roll;
        msg_rpy[3*1] = pitch;
        msg_rpy[3*2] = yaw + legs_ori->yaw_correct;

        if(imu_gyro->IMUGyroEnable){
            msg_rpy[3*0 + 1] = static_cast<double>(st.imu.gyroscope[0]);
            msg_rpy[3*1 + 1] = static_cast<double>(st.imu.gyroscope[1]);
            msg_rpy[3*2 + 1] = static_cast<double>(st.imu.gyroscope[2]);
        }

        if(Signal_Available_Check(msg_rpy,1))
            imu_gyro->SensorDataHandle(msg_rpy, UsedTimestamp);
        
        if (legs_pos->JointsXYZEnable||legs_pos->JointsXYZVelocityEnable){
            double joint[48];

            for (int i = 0; i < 16; ++i) {
                const auto& m = st.motorState[i];
                joint[0 + i]  = static_cast<double>(m.q);
                joint[16 + i] = static_cast<double>(m.dq);
                joint[32 + i] = static_cast<double>(m.tauEst);
            }

            if (Signal_Available_Check(joint,2)) {

                legs_pos->SensorDataHandle(joint, UsedTimestamp);

                if(legs_pos->CalculateWeightEnable)
                    legs_pos->LoadedWeightCheck(joint, UsedTimestamp);

                if(legs_ori->JointsRPYAccEnable) 
                    legs_ori->SensorDataHandle(joint, UsedTimestamp);

                if(legs_ori->JointsRPYEnable)
                    legs_ori->CorrectYawByFootfall(joint, UsedTimestamp);
                
                if(legs_ori->CollisionDetectEnable)
                    legs_ori->CollisionDetect(UsedTimestamp);
            }
        }
        
        for(int i = 0; i < 9; ++i)
        {
            proprio.PositionXYZ[i] = static_cast<float>(sensors[0]->EstimatedState[i]);
            proprio.OrientationRPY[i] = static_cast<float>(sensors[1]->EstimatedState[i]);
        }

        for(int i = 0; i < 3; ++i)
            proprio.FootfallAverage[i] = static_cast<float>(legs_pos->FootfallAveragePosition[i]);

        for(int LegNumber = 0; LegNumber < legs_pos->ContactChainNum; ++LegNumber)
        {
            proprio.FootLandedProbability[LegNumber] = static_cast<float>(legs_pos->FootfallProbability[LegNumber]);

            for(int n = 0; n <= DataFusion::SensorLegsPos::FootNodeIndex; ++n)
                for(int i = 0; i < 3; ++i)
                {
                    proprio.JointsBodyWFPosition[LegNumber][n][i] = static_cast<float>(legs_pos->JointsBodyWFPosition[LegNumber][n][i]);
                    proprio.JointsBodyWFEffort[LegNumber][n][i] = static_cast<float>(legs_pos->JointsBodyWFEffort[LegNumber][n][i]);
                }
                
            for(int motor = 0; motor < 3; ++motor)
                proprio.MotorGravityCompensate[LegNumber][motor] = static_cast<float>(legs_pos->MotorGravityCompensate[LegNumber][motor]);
        }

        proprio.DogWeight = static_cast<float>(legs_pos->TimelyWeight);
        proprio.LegCollisionDetect = legs_ori->CollisionDetectedLeg;

        return proprio;
    }

private:
    std::vector<EstimatorPortN*> sensors;

    std::shared_ptr<DataFusion::SensorIMUAcc>     imu_acc;
    std::shared_ptr<DataFusion::SensorIMUMagGyro> imu_gyro;
    std::shared_ptr<DataFusion::SensorLegsPos>    legs_pos;
    std::shared_ptr<DataFusion::SensorLegsOri>    legs_ori;

    double LastUsedTimestamp = 0, StartTimeStamp = 0;
    double LastSignal[3][48] = {0};
    int LastSignalMaxIndex[3] = {9,9,48};
    
    bool Signal_Available_Check(double Signal[], int type)
    {
        bool diff = false;

        for (int i = 0; i < LastSignalMaxIndex[type]; ++i) {
            if (!(Signal[i] < 9999.0 && Signal[i] > -9999.0))
                return false;
            if (Signal[i] != LastSignal[type][i])
                diff = true;
        }
        if(!diff)
            return false;
        else
            for (int i = 0; i < LastSignalMaxIndex[type]; ++i)
                LastSignal[type][i] = Signal[i];
        return true;
    }
};

inline FusionEstimatorCore CreateRobot_Estimation()
{
    return FusionEstimatorCore{};
}

#endif  /* __FUSION_ESTIMATOR_H_ */