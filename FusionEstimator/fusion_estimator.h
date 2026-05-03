#ifndef __FUSION_ESTIMATOR_H_
#define __FUSION_ESTIMATOR_H_

/*
Proprioceptive legged odometry overview
本体感知腿式里程计说明

Purpose:
作用：
    This estimator computes robot odometry using only IMU and joint motor related data.
    It provides body position, velocity, acceleration, attitude, angular velocity,
    angular acceleration, support-state related statistics, and optional support-center
    / slope related quantities.

    本估计器仅依赖 IMU 和关节电机相关数据完成机器人的本体里程计估计。
    它输出机体的位置、速度、加速度、姿态角、角速度、角加速度，
    以及与支撑状态相关的统计量，并可选输出支撑中心 / 坡度相关量。

Main entry:
主要入口：
    1. Initialization example:
       初始化示例：
           #include "fusion_estimator.h"
           auto Robot_Estimation = CreateRobot_Estimation();

    2. Runtime estimation example:
       运行时估计示例：
           #include "LowlevelState.h"
           LowlevelState st{};
           Odometer odom = Robot_Estimation.fusion_estimator(st);

    3. Runtime status/config interface:
       运行时状态/参数接口：
           double status[200] = {0};
           Robot_Estimation.fusion_estimator_status(status);

Configuration index meaning:
配置索引说明：

    IndexInOrOut = 0
    IndexStatusOK = 1

    IndexIMUAccEnable = 2
    IndexIMUQuaternionEnable = 3
    IndexIMUGyroEnable = 4

    IndexJointsXYZEnable = 5
    IndexJointsVelocityXYZEnable = 6
    IndexJointsRPYEnable = 7

    IndexSlopeModeTimeThreshold = 8
    IndexSlopeModeAngleThreshold = 9
    IndexLegFootForceThreshold = 10
    IndexLegMinStairHeight = 11
    IndexStairHeightFogotten = 12

    IndexLegOrientationInitialWeight = 13
    IndexLegOrientationTimeWeight = 14

    IndexSlopeEstimationEnable = 15

Runtime control:
运行控制：
    status[IndexInOrOut] == 1:
        write/modify estimator parameters
        写入/修改估计器参数

    status[IndexInOrOut] == 2:
        read current estimator parameters and internal state
        读取当前估计器参数和部分内部状态

    status[IndexInOrOut] == 3:
        reset estimated position to zero and re-align yaw correction
        将估计位置清零，并重新对齐 yaw 修正项

    status[IndexInOrOut] == 4:
        switch to MP kinematic preset
        切换到 MP 运动学参数

    status[IndexInOrOut] == 5:
        switch to LW kinematic preset
        切换到 LW 运动学参数

    status[IndexInOrOut] == 6:
        switch to MW kinematic preset
        切换到 MW 运动学参数

    status[IndexInOrOut] == 99:
        switch to Go2P kinematic preset
        切换到 Go2P 运动学参数

Algorithm pipeline:
算法流程：
    1. IMU quaternion / gyroscope update attitude and angular rate related states.
       使用 IMU 四元数 / 角速度更新姿态与角速度相关状态。
    2. Joint motor states are converted to foot positions and velocities in body frame.
       将关节电机状态转换为机体系下的足端位置和速度。
    3. Foot positions / velocities are transformed into world-related support constraints.
       将足端位置 / 速度转化为世界系支撑约束。
    4. Contact / support state is inferred from joint-force-related information.
       基于关节力相关信息推断触地 / 支撑状态。
    5. When support constraints are available, body position and velocity are corrected.
       当支撑约束有效时，对机体位置和速度进行校正。
    6. Optional leg-based yaw correction is applied when enabled.
       当启用时，使用腿部几何关系对 yaw 进行辅助校正。
    7. Optional slope estimation is applied when enough support geometry is available.
       当支撑几何充分时，可选进行坡度估计。
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
    IndexJointsVelocityXYZEnable = 6,
    IndexJointsRPYEnable = 7,

    IndexSlopeModeTimeThreshold = 8,
    IndexSlopeModeAngleThreshold = 9,
    IndexLegFootForceThreshold = 10,  
    IndexLegMinStairHeight = 11,
    IndexStairHeightFogotten = 12,

    IndexLegOrientationInitialWeight = 13, 
    IndexLegOrientationTimeWeight = 14,

    IndexSlopeEstimationEnable = 15,
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

        yaw_correct = 0.0;
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

    void fusion_estimator_status(double status[200])
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
            legs_pos->JointsXYZVelocityEnable   = status[IndexJointsVelocityXYZEnable];
            legs_ori->JointsRPYEnable           = status[IndexJointsRPYEnable];

            legs_pos->SlopeModeTimeThreshold    = status[IndexSlopeModeTimeThreshold];
            legs_pos->SlopeModeAngleThreshold   = status[IndexSlopeModeAngleThreshold];
            legs_pos->FootEffortThreshold       = status[IndexLegFootForceThreshold];
            legs_pos->Environement_Height_Scope = status[IndexLegMinStairHeight];
            legs_pos->Data_Fading_Time          = status[IndexStairHeightFogotten];
            
            legs_ori->legori_init_weight        = status[IndexLegOrientationInitialWeight];
            legs_ori->legori_time_weight        = status[IndexLegOrientationTimeWeight];
            
            legs_pos->SlopeModeEnable           = status[IndexSlopeEstimationEnable];
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
            status[IndexJointsVelocityXYZEnable]     = legs_pos->JointsXYZVelocityEnable;
            status[IndexJointsRPYEnable]             = legs_ori->JointsRPYEnable;

            status[IndexSlopeModeTimeThreshold]      = legs_pos->SlopeModeTimeThreshold ;
            status[IndexSlopeModeAngleThreshold]     = legs_pos->SlopeModeAngleThreshold;
            status[IndexLegFootForceThreshold]       = legs_pos->FootEffortThreshold;
            status[IndexLegMinStairHeight]           = legs_pos->Environement_Height_Scope;
            status[IndexStairHeightFogotten]         = legs_pos->Data_Fading_Time ;

            status[IndexLegOrientationInitialWeight] = legs_ori->legori_init_weight;
            status[IndexLegOrientationTimeWeight]    = legs_ori->legori_time_weight;
            
            status[IndexSlopeEstimationEnable]            = legs_pos->SlopeModeEnable;

            for(int i = 0; i < 9; i++){
                status[50 + i] = sensors[0]->EstimatedState[i];
            }
            for(int i = 0; i < 9; i++){
                status[60 + i] = sensors[1]->EstimatedState[i];
            }
            for(int i = 0; i < 100; i++){
                status[100 + i] = sensors[0]->Double_Par[i];
            }
        }
        else if (status[IndexInOrOut] == 3){
            status[IndexInOrOut] = 0;
            status[IndexStatusOK] = status[IndexStatusOK] + 20;
            if (status[IndexStatusOK] > 999)
                status[IndexStatusOK] = 1;
            sensors[0]->EstimatedState[0] = 0;
            sensors[0]->EstimatedState[3] = 0;
            sensors[0]->EstimatedState[6] = 0;
            yaw_correct = - sensors[1]->EstimatedState[6];
            legs_pos->FootfallPositionRecordIsInitiated[0] = false;
            legs_pos->FootfallPositionRecordIsInitiated[1] = false;
            legs_pos->FootfallPositionRecordIsInitiated[2] = false;
            legs_pos->FootfallPositionRecordIsInitiated[3] = false;
        }

        else if (status[IndexInOrOut] == 4){
            status[IndexInOrOut] = 0;
            status[IndexStatusOK] = status[IndexStatusOK] + 40;
            if (status[IndexStatusOK] > 999)
                status[IndexStatusOK] = 1;
            legs_pos->UseMP();

        }
        else if (status[IndexInOrOut] == 5){
            status[IndexInOrOut] = 0;
            status[IndexStatusOK] = status[IndexStatusOK] + 80;
            if (status[IndexStatusOK] > 999)
                status[IndexStatusOK] = 1;
            legs_pos->UseLW();
        }
        else if (status[IndexInOrOut] == 6){
            status[IndexInOrOut] = 0;
            status[IndexStatusOK] = status[IndexStatusOK] + 120;
            if (status[IndexStatusOK] > 999)
                status[IndexStatusOK] = 1;
            legs_pos->UseMW();
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
    }

    Odometer fusion_estimator(const LowlevelState& st)
    {
        Odometer odom;
        
        const double q[4] = {
            static_cast<double>(st.imu.quaternion[0]),
            static_cast<double>(st.imu.quaternion[1]),
            static_cast<double>(st.imu.quaternion[2]),
            static_cast<double>(st.imu.quaternion[3])
        };

        if(!DataFusion::quat_is_ok(q))
            return odom;

        const double CurrentTimestamp = 1e-3 * static_cast<double>(st.imu.timestamp);
        static double LastUsedTimestamp = 0, StartTimeStamp = 0;

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

        double roll, pitch, yaw;
        double msg_rpy[9] = {0};

        DataFusion::quat_to_eulerZYX(q, roll, pitch, yaw);

        msg_rpy[3*0] = roll;
        msg_rpy[3*1] = pitch;
        msg_rpy[3*2] = yaw + yaw_correct;

        msg_rpy[3*0 + 1] = static_cast<double>(st.imu.gyroscope[0]);
        msg_rpy[3*1 + 1] = static_cast<double>(st.imu.gyroscope[1]);
        msg_rpy[3*2 + 1] = static_cast<double>(st.imu.gyroscope[2]);

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

            if (Signal_Available_Check(joint,2)||legs_pos->CalculateWeightEnable) {
                legs_pos->SensorDataHandle(joint, UsedTimestamp);
                legs_pos->LoadedWeightCheck(joint, UsedTimestamp);
                
                if (legs_ori->JointsRPYEnable) {
                    const double last_yaw = sensors[1]->EstimatedState[6] - yaw_correct;

                    legs_ori->SensorDataHandle(joint, UsedTimestamp);

                    yaw_correct = legs_ori->legori_correct - last_yaw;

                    if (!imu_gyro->IMUQuaternionEnable) {
                        sensors[1]->EstimatedState[6] = legs_ori->legori_correct;
                    }
                }
            }
        }

        odom.XPos = static_cast<float>(sensors[0]->EstimatedState[0]);
        odom.YPos = static_cast<float>(sensors[0]->EstimatedState[3]);
        odom.ZPos = static_cast<float>(sensors[0]->EstimatedState[6]);

        odom.XVel = static_cast<float>(sensors[0]->EstimatedState[1]);
        odom.YVel = static_cast<float>(sensors[0]->EstimatedState[4]);
        odom.ZVel = static_cast<float>(sensors[0]->EstimatedState[7]);

        odom.XAcc = static_cast<float>(sensors[0]->EstimatedState[2]);
        odom.YAcc = static_cast<float>(sensors[0]->EstimatedState[5]);
        odom.ZAcc = static_cast<float>(sensors[0]->EstimatedState[8]);

        odom.RollRad  = static_cast<float>(sensors[1]->EstimatedState[0]);
        odom.PitchRad = static_cast<float>(sensors[1]->EstimatedState[3]);
        odom.YawRad   = static_cast<float>(sensors[1]->EstimatedState[6]);

        odom.RollVel  = static_cast<float>(sensors[1]->EstimatedState[1]);
        odom.PitchVel = static_cast<float>(sensors[1]->EstimatedState[4]);
        odom.YawVel   = static_cast<float>(sensors[1]->EstimatedState[7]);

        odom.RollAcc  = static_cast<float>(sensors[1]->EstimatedState[2]);
        odom.PitchAcc = static_cast<float>(sensors[1]->EstimatedState[5]);
        odom.YawAcc   = static_cast<float>(sensors[1]->EstimatedState[8]);

        odom.FootfallAverageX   = static_cast<float>(legs_pos->FootfallAveragePosition[0]);
        odom.FootfallAverageY   = static_cast<float>(legs_pos->FootfallAveragePosition[1]);
        odom.FootfallAverageYaw = static_cast<float>(legs_pos->FootfallAveragePosition[2]);

        odom.LoadedWeight = static_cast<float>(legs_pos->TimelyWeight);

        odom.FLFootLanded = static_cast<float>(legs_pos->FootfallProbability[0]);
        odom.FRFootLanded = static_cast<float>(legs_pos->FootfallProbability[1]);
        odom.RLFootLanded = static_cast<float>(legs_pos->FootfallProbability[2]);
        odom.RRFootLanded = static_cast<float>(legs_pos->FootfallProbability[3]);

        return odom;
    }

private:
    std::vector<EstimatorPortN*> sensors;

    std::shared_ptr<DataFusion::SensorIMUAcc>     imu_acc;
    std::shared_ptr<DataFusion::SensorIMUMagGyro> imu_gyro;
    std::shared_ptr<DataFusion::SensorLegsPos>    legs_pos;
    std::shared_ptr<DataFusion::SensorLegsOri>    legs_ori;

    double legori_init_weight, legori_time_weight;
    double yaw_correct;

    bool Signal_Available_Check(double Signal[], int type)
    {
        static double last[3][48] = {0};
        static int Number[3] = {9,9,48};
        bool diff = false;

        for (int i = 0; i < Number[type]; ++i) {
            if (!(Signal[i] < 9999.0 && Signal[i] > -9999.0))
                return false;
            if (Signal[i] != last[type][i])
                diff = true;
        }
        if(!diff)
            return false;
        else
            for (int i = 0; i < Number[type]; ++i)
                last[type][i] = Signal[i];
        return true;
    }
};

inline FusionEstimatorCore CreateRobot_Estimation()
{
    return FusionEstimatorCore{};
}

#endif  /* __FUSION_ESTIMATOR_H_ */