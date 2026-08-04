#include "Sensor_Legs.h"

namespace DataFusion
{
    void SensorLegsPos::SensorDataHandle(double* Message, double Time) 
    {
        int i, LegNumber;
        ObservationTime = Time;
        
        for(i = 0; i < StateSpaceModel->Nz; i++)
            Observation[i] = 0;

        for(LegNumber = 0; LegNumber< ContactChainNum; LegNumber++)
        {            
            Joint2HipFoot(Message,LegNumber);
        }
        
        if(FootIsOnGround[0]||FootIsOnGround[1]||FootIsOnGround[2]||FootIsOnGround[3])
        {
            for(i = 0; i < 9; i++)
            {
                StateSpaceModel->Matrix_H[i * StateSpaceModel->Nx + i] = 0;
            }

            FootFallPositionRecordFun(Message);

            
            if(!FootFallPositionAvailable)
                return;

            if(JointsXYZEnable){
                for(i = 0; i < 3; i++)
                {
                    StateSpaceModel->Matrix_H[(3 * i + 0) * StateSpaceModel->Nx + (3 * i + 0)] = 1;
                }
            }
            if(JointsXYZVelocityEnable){
                for(i = 0; i < 3; i++)
                {
                    StateSpaceModel->Matrix_H[(3 * i + 1) * StateSpaceModel->Nx + (3 * i + 1)] = 1;
                    StateSpaceModel->Matrix_H[(3 * i + 2) * StateSpaceModel->Nx + (3 * i + 2)] = 1;
                }
            }
            StateSpaceModel_Go2_EstimatorPort(Observation, ObservationTime, StateSpaceModel);

            double p_w[MAX_CONTACT_CHAIN][2] = {{0.0}};
            for (LegNumber = 0; LegNumber < ContactChainNum; ++LegNumber) {
                if (FootIsOnGround[LegNumber]) {
                    p_w[LegNumber][0] = FootfallPositionRecord[LegNumber][0];
                    p_w[LegNumber][1] = FootfallPositionRecord[LegNumber][1];
                } else {
                    p_w[LegNumber][0] = StateSpaceModel->EstimatedState[0] + JointsBodyWFPosition[LegNumber][FootNodeIndex][0];
                    p_w[LegNumber][1] = StateSpaceModel->EstimatedState[3] + JointsBodyWFPosition[LegNumber][FootNodeIndex][1];
                }
            }

            if (ContactChainNum == 4) {
                const double fx = 0.5 * (p_w[0][0] + p_w[1][0]);
                const double fy = 0.5 * (p_w[0][1] + p_w[1][1]);
                const double rx = 0.5 * (p_w[2][0] + p_w[3][0]);
                const double ry = 0.5 * (p_w[2][1] + p_w[3][1]);

                const double x_mean = 0.5 * (fx + rx);
                const double y_mean = 0.5 * (fy + ry);

                double yaw_ff = std::atan2(fy - ry, fx - rx);
                array_angle_unwrap(&yaw_ff, &FootAverageLastYaw, &FootAverageTurn, 1);

                FootfallAveragePosition[0] = x_mean;
                FootfallAveragePosition[1] = y_mean;
                FootfallAveragePosition[2] = yaw_ff;
            } else {
                double x_sum = 0.0, y_sum = 0.0;
                int cnt = 0;
                for (LegNumber = 0; LegNumber < ContactChainNum; ++LegNumber) {
                    x_sum += p_w[LegNumber][0];
                    y_sum += p_w[LegNumber][1];
                    cnt++;
                }

                FootfallAveragePosition[0] = x_sum / (double)cnt;
                FootfallAveragePosition[1] = y_sum / (double)cnt;
                FootfallAveragePosition[2] = 0.0;
            }
        }

        if(GravityCompensateEnable)
            GravityCompensate();
    }

    void SensorLegsPos::LoadedWeightCheck(double* Message, double Time) 
    {
        const bool stand_on_ground = (FootfallProbability[0] + FootfallProbability[1] + FootfallProbability[2] + FootfallProbability[3]) > 2.5 && (std::abs(StateSpaceModel->EstimatedState[1]) + std::abs(StateSpaceModel->EstimatedState[4]) +std::abs(StateSpaceModel->EstimatedState[7])) < 0.05;

        if (stand_on_ground) {
            if (!WeightStableStand || Time < WeightStableTimestamp) {
                WeightStableStand = true;
                WeightStableTimestamp = Time;
            }
        } else {
            WeightStableStand = false;
            WeightSumBuffer_i = 0;
            WeightSumBuffer_n = 0;
            WeightSum = 0.0;
            return;
        }

        if (Time - WeightStableTimestamp < WeightStableTime)
            return;
        
        double fz_sum = 0;
        for(int LegNumber = 0; LegNumber < ContactChainNum; ++LegNumber)
            fz_sum += JointsBodyWFEffort[LegNumber][FootNodeIndex][2];

        if (WeightSumBuffer_n < WeightSumNum) {
            WeightSumBuffer[WeightSumBuffer_i] = fz_sum;
            WeightSum += fz_sum;
            WeightSumBuffer_n++;
        } else {
            WeightSum -= WeightSumBuffer[WeightSumBuffer_i];
            WeightSumBuffer[WeightSumBuffer_i] = fz_sum;
            WeightSum += fz_sum;
        }

        WeightSumBuffer_i++;
        if (WeightSumBuffer_i >= WeightSumNum)
            WeightSumBuffer_i = 0;

        const double mean100 = (WeightSumBuffer_n > 0) ? (WeightSum / (double)WeightSumBuffer_n) : 0.0;

        TimelyWeight = - mean100 / 9.79;

        if (TimelyWeight < MinimumWeight)
            TimelyWeight = MinimumWeight;
    }

    void SensorLegsPos::Joint2HipFoot(double *Message, int LegNumber)
    {
        double joint_org[MAX_CHAIN_NODE][3];
        double joint_axis[MAX_CHAIN_NODE][3];
        double joint_dq[MAX_CHAIN_NODE];
        double joint_tau[MAX_CHAIN_NODE];
        int joint_num = 0;

        const double p_zero[3] = {0.0, 0.0, 0.0};

        for (int n = 0; n < LegChains_[LegNumber].node_num; ++n)
        {
            const double* p_parent =
                (LegChains_[LegNumber].node[n].parent < 0) ?
                p_zero :
                LegChains_[LegNumber].node_pos_wf[LegChains_[LegNumber].node[n].parent];

            double* q_parent =
                (LegChains_[LegNumber].node[n].parent < 0) ?
                Est_Quaternion :
                LegChains_[LegNumber].node_quat_wf[LegChains_[LegNumber].node[n].parent];

            array_quaternion_rotate_vector(
                q_parent,
                LegChains_[LegNumber].node[n].t,
                LegChains_[LegNumber].node_pos_wf[n]
            );

            LegChains_[LegNumber].node_pos_wf[n][0] += p_parent[0];
            LegChains_[LegNumber].node_pos_wf[n][1] += p_parent[1];
            LegChains_[LegNumber].node_pos_wf[n][2] += p_parent[2];

            double q_pre[4];
            array_quaternion_multiplication(q_parent, LegChains_[LegNumber].node[n].q_fix, q_pre);

            if (LegChains_[LegNumber].node[n].q_index >= 0)
            {
                double axis_local[3] = {
                    (LegChains_[LegNumber].node[n].axis == TF_AXIS_X) ? 1.0 : 0.0,
                    (LegChains_[LegNumber].node[n].axis == TF_AXIS_Y) ? 1.0 : 0.0,
                    (LegChains_[LegNumber].node[n].axis == TF_AXIS_Z) ? 1.0 : 0.0
                };

                joint_org[joint_num][0] = LegChains_[LegNumber].node_pos_wf[n][0];
                joint_org[joint_num][1] = LegChains_[LegNumber].node_pos_wf[n][1];
                joint_org[joint_num][2] = LegChains_[LegNumber].node_pos_wf[n][2];

                array_quaternion_rotate_vector(q_pre, axis_local, joint_axis[joint_num]);

                joint_dq[joint_num] =
                    (LegChains_[LegNumber].node[n].dq_index >= 0) ?
                    Message[LegChains_[LegNumber].node[n].dq_index] :
                    0.0;

                joint_tau[joint_num] =
                    (LegChains_[LegNumber].node[n].tau_index >= 0) ?
                    Message[LegChains_[LegNumber].node[n].tau_index] :
                    0.0;

                const double h = 0.5 * Message[LegChains_[LegNumber].node[n].q_index];
                const double s = std::sin(h);

                double q_joint[4] = {
                    std::cos(h),
                    axis_local[0] * s,
                    axis_local[1] * s,
                    axis_local[2] * s
                };

                array_quaternion_multiplication(q_pre, q_joint, LegChains_[LegNumber].node_quat_wf[n]);
                array_quaternion_normalize(LegChains_[LegNumber].node_quat_wf[n], LegChains_[LegNumber].node_quat_wf[n]);

                joint_num++;
            }
            else
            {
                LegChains_[LegNumber].node_quat_wf[n][0] = q_pre[0];
                LegChains_[LegNumber].node_quat_wf[n][1] = q_pre[1];
                LegChains_[LegNumber].node_quat_wf[n][2] = q_pre[2];
                LegChains_[LegNumber].node_quat_wf[n][3] = q_pre[3];
            }
        }

        for (int n = 0; n < LegChains_[LegNumber].node_num; ++n)
            for (int i = 0; i < 3; ++i)
                JointsBodyWFPosition[LegNumber][n][i] = LegChains_[LegNumber].node_pos_wf[n][i];
        
        FootBodyVel_WF[LegNumber][0] = 0.0;
        FootBodyVel_WF[LegNumber][1] = 0.0;
        FootBodyVel_WF[LegNumber][2] = 0.0;

        double Jtau[3];
        double JJT[3][3];
        double JJT_inv[3][3];

        for (int target = 1; target <= joint_num; ++target)
        {
            Jtau[0] = Jtau[1] = Jtau[2] = 0.0;

            for (int r = 0; r < 3; ++r)
                for (int c = 0; c < 3; ++c)
                    JJT[r][c] = 0.0;

            for (int j = 0; j < target; ++j)
            {
                const double rx = (target == joint_num ? JointsBodyWFPosition[LegNumber][FootNodeIndex][0] : joint_org[target][0]) - joint_org[j][0];
                const double ry = (target == joint_num ? JointsBodyWFPosition[LegNumber][FootNodeIndex][1] : joint_org[target][1]) - joint_org[j][1];
                const double rz = (target == joint_num ? JointsBodyWFPosition[LegNumber][FootNodeIndex][2] : joint_org[target][2]) - joint_org[j][2];

                const double J0 = joint_axis[j][1] * rz - joint_axis[j][2] * ry;
                const double J1 = joint_axis[j][2] * rx - joint_axis[j][0] * rz;
                const double J2 = joint_axis[j][0] * ry - joint_axis[j][1] * rx;

                if (target == joint_num)
                {
                    FootBodyVel_WF[LegNumber][0] += J0 * joint_dq[j];
                    FootBodyVel_WF[LegNumber][1] += J1 * joint_dq[j];
                    FootBodyVel_WF[LegNumber][2] += J2 * joint_dq[j];
                }

                Jtau[0] += J0 * joint_tau[j];
                Jtau[1] += J1 * joint_tau[j];
                Jtau[2] += J2 * joint_tau[j];

                JJT[0][0] += J0 * J0;
                JJT[0][1] += J0 * J1;
                JJT[0][2] += J0 * J2;
                JJT[1][1] += J1 * J1;
                JJT[1][2] += J1 * J2;
                JJT[2][2] += J2 * J2;
            }

            JJT[1][0] = JJT[0][1];
            JJT[2][0] = JJT[0][2];
            JJT[2][1] = JJT[1][2];

            if (target < joint_num)
            {
                JJT[0][0] += 1e-4;
                JJT[1][1] += 1e-4;
                JJT[2][2] += 1e-4;
            }

            for (int i = 0; i < 3; ++i)
                JointsBodyWFEffort[LegNumber][target][i] = 0.0;

            if (array_3x3_inverse(JJT, JJT_inv) == _ERROR_NO_ERROR)
                array_3x3_multiply_vector(JJT_inv, Jtau, JointsBodyWFEffort[LegNumber][target]);
        }

        FootBodyTorq_WF[LegNumber][0] = JointsBodyWFPosition[LegNumber][FootNodeIndex][1] * JointsBodyWFEffort[LegNumber][FootNodeIndex][2] - JointsBodyWFPosition[LegNumber][FootNodeIndex][2] * JointsBodyWFEffort[LegNumber][FootNodeIndex][1];
        FootBodyTorq_WF[LegNumber][1] = JointsBodyWFPosition[LegNumber][FootNodeIndex][2] * JointsBodyWFEffort[LegNumber][FootNodeIndex][0] - JointsBodyWFPosition[LegNumber][FootNodeIndex][0] * JointsBodyWFEffort[LegNumber][FootNodeIndex][2];
        FootBodyTorq_WF[LegNumber][2] = JointsBodyWFPosition[LegNumber][FootNodeIndex][0] * JointsBodyWFEffort[LegNumber][FootNodeIndex][1] - JointsBodyWFPosition[LegNumber][FootNodeIndex][1] * JointsBodyWFEffort[LegNumber][FootNodeIndex][0];

        if(JointsBodyWFEffort[LegNumber][FootNodeIndex][2] >= 0.3 * FootEffortThreshold)
            FootfallProbability[LegNumber] = 0.0;
        else if(JointsBodyWFEffort[LegNumber][FootNodeIndex][2] <= 1.3 * FootEffortThreshold)
            FootfallProbability[LegNumber] = 1.0;
        else
            FootfallProbability[LegNumber] = (JointsBodyWFEffort[LegNumber][FootNodeIndex][2] - 0.3 * FootEffortThreshold) / (FootEffortThreshold);

        if(JointsBodyWFEffort[LegNumber][FootNodeIndex][2] < FootEffortThreshold)
            FootIsOnGround[LegNumber] = true;
        else
            FootIsOnGround[LegNumber] = false;

        if(FootIsOnGround[LegNumber] && !FootWasOnGround[LegNumber])
        {
            FootLanding[LegNumber] = true;
            FootLastMotion[LegNumber] = true;
        }
        else
        {
            FootLanding[LegNumber] = false;
        }

        if(!FootIsOnGround[LegNumber] && FootWasOnGround[LegNumber])
            FootLastMotion[LegNumber] = false;

        // 辨别狗趴在地上的状态
        if(LegNumber == ContactChainNum - 1){
            int count = 0;
            for(count = 0; count < ContactChainNum; count++)
                if(FootIsOnGround[count] || JointsBodyWFPosition[count][FootNodeIndex][2] < -0.3)
                    break;
            if(count == ContactChainNum)
                FootIsOnGround[LegNumber] = true;
        }

        FootWasOnGround[LegNumber] = FootIsOnGround[LegNumber];
    }

    void SensorLegsPos::FootFallPositionRecordFun(double *Message){

        double p_sum[3] = {0}, v_sum[3] = {0};
        int    leg_cnt = 0;

        double body_rpy[3] = {0.0, 0.0, 0.0};
        array_quaternion_to_eulerZYX(Est_Quaternion, body_rpy);
        double body_roll = body_rpy[0];
        double body_pitch = body_rpy[1];
        double body_yaw = body_rpy[2];

        double move_dir_x = 1.0, move_dir_y = 0.0, move_dir_z = 0.0;
        EstimateGroundPitchAlongHeading(move_dir_x, move_dir_y, move_dir_z);

        double FootfallPositionRecordTemp[MAX_CONTACT_CHAIN][3] = {{0.0}};
        double FootfallVelocityRecordTemp[MAX_CONTACT_CHAIN][3] = {{0.0}};
        bool FootIsOnGroundTemp[MAX_CONTACT_CHAIN] = {false};
        int WheelMoveRefLeg = -1;
        double WheelMoveAbsMin = 0.0;

        for (int LegNumber = 0; LegNumber < ContactChainNum; ++LegNumber)
        {
            if (!FootIsOnGround[LegNumber])
                continue;
            if(!FootfallPositionRecordIsInitiated[LegNumber])
            {
                FootfallPositionRecordIsInitiated[LegNumber] = true;
                FootLanding[LegNumber]= false;
                FootfallPositionRecord[LegNumber][0] = StateSpaceModel->EstimatedState[0] + JointsBodyWFPosition[LegNumber][FootNodeIndex][0];
                FootfallPositionRecord[LegNumber][1] = StateSpaceModel->EstimatedState[3] + JointsBodyWFPosition[LegNumber][FootNodeIndex][1];
                FootfallPositionRecord[LegNumber][2] = 0;
                FootfallPositionRecord[LegNumber][3] = ObservationTime;
                ClusterFootfallHeight(LegNumber, move_dir_z);

                if (LegChains_[LegNumber].wheel_q_index >= 0)
                    WheelAnglePrev[LegNumber] = Message[LegChains_[LegNumber].wheel_q_index];
                else
                    WheelAnglePrev[LegNumber] = 0.0;
                ShankPitchPrev[LegNumber] = body_pitch;
                for (int k = 0; k < LegChains_[LegNumber].pitch_joint_num; ++k)
                    if (LegChains_[LegNumber].pitch_q_index[k] >= 0)
                        ShankPitchPrev[LegNumber] -= Message[LegChains_[LegNumber].pitch_q_index[k]];
                ShankRollPrev[LegNumber] = body_roll;
                for (int k = 0; k < LegChains_[LegNumber].roll_joint_num; ++k)
                    if (LegChains_[LegNumber].roll_q_index[k] >= 0)
                        ShankRollPrev[LegNumber] -= Message[LegChains_[LegNumber].roll_q_index[k]];
            }
            else if(FootLanding[LegNumber])
            {
                FootLanding[LegNumber]= false;
                FootfallPositionRecord[LegNumber][0] = StateSpaceModel->EstimatedState[0] + JointsBodyWFPosition[LegNumber][FootNodeIndex][0];
                FootfallPositionRecord[LegNumber][1] = StateSpaceModel->EstimatedState[3] + JointsBodyWFPosition[LegNumber][FootNodeIndex][1];
                FootfallPositionRecord[LegNumber][2] = StateSpaceModel->EstimatedState[6] + JointsBodyWFPosition[LegNumber][FootNodeIndex][2];
                FootfallPositionRecord[LegNumber][3] = ObservationTime;
                
                if (LegChains_[LegNumber].wheel_q_index >= 0)
                    WheelAnglePrev[LegNumber] = Message[LegChains_[LegNumber].wheel_q_index];
                else
                    WheelAnglePrev[LegNumber] = 0.0;
                ShankPitchPrev[LegNumber] = body_pitch;
                for (int k = 0; k < LegChains_[LegNumber].pitch_joint_num; ++k)
                    if (LegChains_[LegNumber].pitch_q_index[k] >= 0)
                        ShankPitchPrev[LegNumber] -= Message[LegChains_[LegNumber].pitch_q_index[k]];
                ShankRollPrev[LegNumber] = body_roll;
                for (int k = 0; k < LegChains_[LegNumber].roll_joint_num; ++k)
                    if (LegChains_[LegNumber].roll_q_index[k] >= 0)
                        ShankRollPrev[LegNumber] -= Message[LegChains_[LegNumber].roll_q_index[k]];

                ClusterFootfallHeight(LegNumber, move_dir_z);
            }

            // 轮子转动角度
            double WheelMove = 0.0;
            double WheelSidewayMove = 0.0;
            double WheelVel = 0.0;

            if (LegChains_[LegNumber].wheel_radius > 0.0 && LegChains_[LegNumber].wheel_q_index >= 0 && LegChains_[LegNumber].wheel_dq_index >= 0)
            {
                double WheelRotationAngle = Message[LegChains_[LegNumber].wheel_q_index] - WheelAnglePrev[LegNumber];
                while (WheelRotationAngle >  M_PI) WheelRotationAngle -= 2.0*M_PI;
                while (WheelRotationAngle < -M_PI) WheelRotationAngle += 2.0*M_PI;
                WheelAnglePrev[LegNumber] = Message[LegChains_[LegNumber].wheel_q_index];

                double ShankPitchAngle = body_pitch;
                double ShankRollAngle = body_roll;
                double WheelRotationVelocityEff = Message[LegChains_[LegNumber].wheel_dq_index];

                for (int k = 0; k < LegChains_[LegNumber].pitch_joint_num; ++k)
                {
                    if (LegChains_[LegNumber].pitch_q_index[k] >= 0)
                        ShankPitchAngle -= Message[LegChains_[LegNumber].pitch_q_index[k]];
                    if (LegChains_[LegNumber].pitch_dq_index[k] >= 0)
                        WheelRotationVelocityEff -= Message[LegChains_[LegNumber].pitch_dq_index[k]];
                }
                for (int k = 0; k < LegChains_[LegNumber].roll_joint_num; ++k)
                    if (LegChains_[LegNumber].roll_q_index[k] >= 0)
                        ShankRollAngle -= Message[LegChains_[LegNumber].roll_q_index[k]];

                double temp = ShankPitchAngle;
                ShankPitchAngle -= ShankPitchPrev[LegNumber];
                while (ShankPitchAngle >  M_PI) ShankPitchAngle -= 2.0*M_PI;
                while (ShankPitchAngle < -M_PI) ShankPitchAngle += 2.0*M_PI;
                ShankPitchPrev[LegNumber] = temp;

                temp = ShankRollAngle;
                ShankRollAngle -= ShankRollPrev[LegNumber];
                while (ShankRollAngle >  M_PI) ShankRollAngle -= 2.0 * M_PI;
                while (ShankRollAngle < -M_PI) ShankRollAngle += 2.0 * M_PI;
                ShankRollPrev[LegNumber] = temp;

                WheelMove = LegChains_[LegNumber].wheel_radius * (WheelRotationAngle - ShankPitchAngle);
                WheelSidewayMove = (LegChains_[LegNumber].wheel_radius + LegChains_[LegNumber].wheel_width) * 2.0 * std::sin(0.5 * ShankRollAngle);
                WheelVel = LegChains_[LegNumber].wheel_radius * WheelRotationVelocityEff;

                if (WheelMoveRefLeg < 0 || std::abs(WheelMove) < WheelMoveAbsMin)
                {
                    WheelMoveRefLeg = LegNumber;
                    WheelMoveAbsMin = std::abs(WheelMove);
                }
            }

            FootfallPositionRecordTemp[LegNumber][0] = FootfallPositionRecord[LegNumber][0] + WheelMove * move_dir_x - WheelSidewayMove * move_dir_y;
            FootfallPositionRecordTemp[LegNumber][1] = FootfallPositionRecord[LegNumber][1] + WheelMove * move_dir_y + WheelSidewayMove * move_dir_x;
            FootfallPositionRecordTemp[LegNumber][2] = FootfallPositionRecord[LegNumber][2] + WheelMove * move_dir_z;
            FootfallVelocityRecordTemp[LegNumber][0] = -FootBodyVel_WF[LegNumber][0] + WheelVel * move_dir_x;
            FootfallVelocityRecordTemp[LegNumber][1] = -FootBodyVel_WF[LegNumber][1] + WheelVel * move_dir_y;
            FootfallVelocityRecordTemp[LegNumber][2] = -FootBodyVel_WF[LegNumber][2] + WheelVel * move_dir_z;
            FootIsOnGroundTemp[LegNumber] = true;

        }

        if (WheelMoveRefLeg >= 0)
        {
            for (int LegNumber = 0; LegNumber < ContactChainNum; ++LegNumber)
            {
                if (!FootIsOnGroundTemp[LegNumber]) continue;

                double CalculatedPosition[3];
                CalculatedPosition[0] = FootfallPositionRecordTemp[WheelMoveRefLeg][0] + JointsBodyWFPosition[LegNumber][FootNodeIndex][0] - JointsBodyWFPosition[WheelMoveRefLeg][FootNodeIndex][0];
                CalculatedPosition[1] = FootfallPositionRecordTemp[WheelMoveRefLeg][1] + JointsBodyWFPosition[LegNumber][FootNodeIndex][1] - JointsBodyWFPosition[WheelMoveRefLeg][FootNodeIndex][1];
                CalculatedPosition[2] = FootfallPositionRecordTemp[WheelMoveRefLeg][2] + JointsBodyWFPosition[LegNumber][FootNodeIndex][2] - JointsBodyWFPosition[WheelMoveRefLeg][FootNodeIndex][2];

                double dx = CalculatedPosition[0] - FootfallPositionRecordTemp[LegNumber][0];
                double dy = CalculatedPosition[1] - FootfallPositionRecordTemp[LegNumber][1];
                double dz = CalculatedPosition[2] - FootfallPositionRecordTemp[LegNumber][2];

                if (dx * dx + dy * dy + dz * dz > WheelPositionMismatchThreshold * WheelPositionMismatchThreshold)
                {
                    FootfallPositionRecordTemp[LegNumber][0] = CalculatedPosition[0];
                    FootfallPositionRecordTemp[LegNumber][1] = CalculatedPosition[1];
                    FootfallPositionRecord[LegNumber][2]     = CalculatedPosition[2];
                    ClusterFootfallHeight(LegNumber, move_dir_z);
                    FootfallPositionRecordTemp[LegNumber][2] = FootfallPositionRecord[LegNumber][2];
                    FootIsOnGroundTemp[LegNumber] = false;
                }
            }
        }

        for (int LegNumber = 0; LegNumber < ContactChainNum; ++LegNumber)
        {
            if (!FootIsOnGround[LegNumber]) continue;
            FootfallPositionRecord[LegNumber][0] = FootfallPositionRecordTemp[LegNumber][0];
            FootfallPositionRecord[LegNumber][1] = FootfallPositionRecordTemp[LegNumber][1];
            FootfallPositionRecord[LegNumber][2] = FootfallPositionRecordTemp[LegNumber][2];

            if (!FootIsOnGroundTemp[LegNumber]) continue;
            p_sum[0] += FootfallPositionRecordTemp[LegNumber][0] - JointsBodyWFPosition[LegNumber][FootNodeIndex][0];
            p_sum[1] += FootfallPositionRecordTemp[LegNumber][1] - JointsBodyWFPosition[LegNumber][FootNodeIndex][1];
            p_sum[2] += FootfallPositionRecordTemp[LegNumber][2] - JointsBodyWFPosition[LegNumber][FootNodeIndex][2];
            v_sum[0] += FootfallVelocityRecordTemp[LegNumber][0];
            v_sum[1] += FootfallVelocityRecordTemp[LegNumber][1];
            v_sum[2] += FootfallVelocityRecordTemp[LegNumber][2];
            leg_cnt++;
        }

        if (leg_cnt == 0)
            FootFallPositionAvailable = false;
        else{
            FootFallPositionAvailable = true;
            Observation[0] = p_sum[0] / (double)leg_cnt;
            Observation[3] = p_sum[1] / (double)leg_cnt;
            Observation[6] = p_sum[2] / (double)leg_cnt;
            Observation[1] = v_sum[0] / (double)leg_cnt;
            Observation[4] = v_sum[1] / (double)leg_cnt;
            Observation[7] = v_sum[2] / (double)leg_cnt;
            for (int LegNumber = 0; LegNumber < ContactChainNum; ++LegNumber)
            {
                Observation[2] -= JointsBodyWFEffort[LegNumber][FootNodeIndex][0] / TimelyWeight;
                Observation[5] -= JointsBodyWFEffort[LegNumber][FootNodeIndex][1] / TimelyWeight;
                Observation[8] -= JointsBodyWFEffort[LegNumber][FootNodeIndex][2] / TimelyWeight;
            }
          Observation[8] -= 9.79;
        }
    }

    void SensorLegsPos::ClusterFootfallHeight(int LegNumber, double move_dir_z)
    {
        int DecreaseSearch = 0, CheckIndex = MapHeightLeg[LegNumber], TempIntA = 0, TempIntB = 0;
        double Zdifference = 0.0;

        if(MapHeightStore[MapHeightLeg[LegNumber]][0] - FootfallPositionRecord[LegNumber][2] > 0)
            DecreaseSearch = 1;
        else
            DecreaseSearch = 0;

        //从上次足点高度记录开始，向上或下搜索，直到边界或越过当前足高度
        while(MapHeightStore[CheckIndex][3+DecreaseSearch] != -1)
        {
            //删除不是边界的过期节点
            if(std::abs(ObservationTime-MapHeightStore[CheckIndex][2]) > Data_Fading_Time && MapHeightStore[CheckIndex][4-DecreaseSearch] != -1)
            {
                //如果过期节点是一个在用足点记录，就不删除
                bool HeightRecordNotUsing = true;
                for(int Leg = 0; Leg < MAX_CONTACT_CHAIN; Leg++)
                    if(MapHeightLeg[Leg] == CheckIndex)
                        HeightRecordNotUsing = false;
                if(HeightRecordNotUsing)
                {
                    TempIntA = (int)MapHeightStore[CheckIndex][3 + DecreaseSearch];
                    MapHeightStore[int(MapHeightStore[CheckIndex][3])][4] = MapHeightStore[CheckIndex][4];
                    MapHeightStore[int(MapHeightStore[CheckIndex][4])][3] = MapHeightStore[CheckIndex][3];

                    if(CheckIndex != MapHeightStoreCount)
                    {
                        MapHeightStore[CheckIndex][0] = MapHeightStore[MapHeightStoreCount][0];
                        MapHeightStore[CheckIndex][1] = MapHeightStore[MapHeightStoreCount][1];
                        MapHeightStore[CheckIndex][2] = MapHeightStore[MapHeightStoreCount][2];
                        MapHeightStore[CheckIndex][3] = MapHeightStore[MapHeightStoreCount][3];
                        MapHeightStore[CheckIndex][4] = MapHeightStore[MapHeightStoreCount][4];
                        MapHeightStore[CheckIndex][5] = MapHeightStore[MapHeightStoreCount][5];
                        
                        //如果搬移的末位节点是足的记录点，就足记录重新指向
                        for(int Leg = 0; Leg < MAX_CONTACT_CHAIN; Leg++)
                            if(MapHeightLeg[Leg] == MapHeightStoreCount)
                                MapHeightLeg[Leg] = CheckIndex;

                        if(MapHeightStore[CheckIndex][3] != -1)
                            MapHeightStore[(int)MapHeightStore[CheckIndex][3]][4] = CheckIndex;
                        else
                            MapHeightStoreMaxMinIndex[0] = CheckIndex;
                        if(MapHeightStore[CheckIndex][4] != -1)
                            MapHeightStore[(int)MapHeightStore[CheckIndex][4]][3] = CheckIndex;
                        else
                            MapHeightStoreMaxMinIndex[1] = CheckIndex;

                        if(TempIntA == MapHeightStoreCount)
                            TempIntA = CheckIndex;
                        CheckIndex = TempIntA;
                    }
                    MapHeightStoreCount--;
                    CheckIndex = TempIntA;
                    continue;
                }
            }

            if(DecreaseSearch && MapHeightStore[CheckIndex][0] < FootfallPositionRecord[LegNumber][2])
                break;
            else if(!DecreaseSearch && MapHeightStore[CheckIndex][0] >= FootfallPositionRecord[LegNumber][2])
                break;
            else
                CheckIndex = MapHeightStore[CheckIndex][3+DecreaseSearch];
        }

        double TempDoubleA = std::abs(MapHeightStore[CheckIndex][0] - FootfallPositionRecord[LegNumber][2]);
        double TempDoubleB = 99.99;

        if(MapHeightStore[CheckIndex][4-DecreaseSearch] != -1)
            TempDoubleB = std::abs(MapHeightStore[int(MapHeightStore[CheckIndex][4-DecreaseSearch])][0] - FootfallPositionRecord[LegNumber][2]);

        // 如果都符合要求，优先选择早记录的点
        if((TempDoubleA <= Environement_Height_Scope && TempDoubleB <= Environement_Height_Scope && move_dir_z == 0.0) || (TempDoubleA <= SlopeModeStepHeightThreshold && TempDoubleB <= SlopeModeStepHeightThreshold))
            MapHeightStore[CheckIndex][5] < MapHeightStore[int(MapHeightStore[CheckIndex][4-DecreaseSearch])][5] ? TempDoubleB += 99.99 : TempDoubleA += 99.99;

        int HitIndex = TempDoubleA <= TempDoubleB ? CheckIndex : int(MapHeightStore[CheckIndex][4-DecreaseSearch]);

        if((std::abs(MapHeightStore[HitIndex][0] - FootfallPositionRecord[LegNumber][2]) <= Environement_Height_Scope && move_dir_z == 0.0) || std::abs(MapHeightStore[HitIndex][0] - FootfallPositionRecord[LegNumber][2]) <= SlopeModeStepHeightThreshold)
        {
            MapHeightStore[HitIndex][1] *= exp(- (ObservationTime - MapHeightStore[HitIndex][2]) / (10 * Data_Fading_Time));
            MapHeightStore[HitIndex][1] += 1;
            MapHeightStore[HitIndex][2] = ObservationTime;

            if(std::abs(MapHeightStore[HitIndex][0] - FootfallPositionRecord[LegNumber][2]) <= Environement_Height_Scope / 4)
                Zdifference = 0;
            else
            {
                //根据新落足点迭代历史高度记录
                // MapHeightStore[HitIndex][0] = (MapHeightStore[HitIndex][0] * MapHeightStore[HitIndex][1] + FootfallPositionRecord[LegNumber][2]) / (MapHeightStore[HitIndex][1] + 1);
                Zdifference = FootfallPositionRecord[LegNumber][2] - MapHeightStore[HitIndex][0];
            }
            MapHeightLeg[LegNumber] = HitIndex;
        }
        else
        {
            Zdifference = 0;
            MapHeightStoreCount++;

            // 如果存储量超过了最大存储限度，就删除距离当前足点更远的最大/最小值记录
            if(MapHeightStoreCount >= MapHeightStoreMax)
            {
                // 判断离最大/最小值更远, 删除更远的最值
                TempIntB = std::abs(MapHeightStore[MapHeightStoreMaxMinIndex[0]][0]-FootfallPositionRecord[LegNumber][2]) > std::abs(MapHeightStore[MapHeightStoreMaxMinIndex[1]][0]-FootfallPositionRecord[LegNumber][2]) ? 1 : 0;

                // 存储次大次小的节点
                TempIntA = (int)MapHeightStore[MapHeightStoreMaxMinIndex[1-TempIntB]][3 + TempIntB];
                MapHeightStore[TempIntA][4 - TempIntB] = -1;

                // 把末位节点放到最大最小的节点的数据空间
                MapHeightStore[MapHeightStoreMaxMinIndex[1-TempIntB]][0] = MapHeightStore[MapHeightStoreMax-1][0];
                MapHeightStore[MapHeightStoreMaxMinIndex[1-TempIntB]][1] = MapHeightStore[MapHeightStoreMax-1][1];
                MapHeightStore[MapHeightStoreMaxMinIndex[1-TempIntB]][2] = MapHeightStore[MapHeightStoreMax-1][2];
                MapHeightStore[MapHeightStoreMaxMinIndex[1-TempIntB]][3] = MapHeightStore[MapHeightStoreMax-1][3];
                MapHeightStore[MapHeightStoreMaxMinIndex[1-TempIntB]][4] = MapHeightStore[MapHeightStoreMax-1][4];
                MapHeightStore[MapHeightStoreMaxMinIndex[1-TempIntB]][5] = MapHeightStore[MapHeightStoreMax-1][5];
                for(int Leg = 0; Leg < MAX_CONTACT_CHAIN; Leg++)
                    if(MapHeightLeg[Leg] == MapHeightStoreMax-1)
                        MapHeightLeg[Leg] = MapHeightStoreMaxMinIndex[1-TempIntB];
                if(MapHeightStoreMaxMinIndex[0] == MapHeightStoreMax-1)
                    MapHeightStoreMaxMinIndex[0] = MapHeightStoreMaxMinIndex[1-TempIntB];
                if(MapHeightStoreMaxMinIndex[1] == MapHeightStoreMax-1)
                    MapHeightStoreMaxMinIndex[1] = MapHeightStoreMaxMinIndex[1-TempIntB];
                if(TempIntA == MapHeightStoreMax-1)
                    TempIntA = MapHeightStoreMaxMinIndex[1-TempIntB];
                if(CheckIndex == MapHeightStoreMax - 1)
                    CheckIndex = MapHeightStoreMaxMinIndex[1-TempIntB];
                if(HitIndex == MapHeightStoreMax - 1)
                    HitIndex = MapHeightStoreMaxMinIndex[1-TempIntB];

                //修改指向末位节点的节点记录
                if(MapHeightStore[MapHeightStoreMax-1][3] != -1 && MapHeightStoreMaxMinIndex[1] != MapHeightStoreMax-1)
                    MapHeightStore[int(MapHeightStore[MapHeightStoreMax-1][3])][4] = MapHeightStoreMaxMinIndex[1-TempIntB];
                if(MapHeightStore[MapHeightStoreMax-1][4] != -1 && MapHeightStoreMaxMinIndex[0] != MapHeightStoreMax-1)
                    MapHeightStore[int(MapHeightStore[MapHeightStoreMax-1][4])][3] = MapHeightStoreMaxMinIndex[1-TempIntB];

                //更新最大最小节点记录
                MapHeightStoreMaxMinIndex[1-TempIntB] = TempIntA;
                MapHeightStoreCount--;
            }

            MapHeightStore[MapHeightStoreCount][0] = FootfallPositionRecord[LegNumber][2];
            MapHeightStore[MapHeightStoreCount][1] = 1;
            MapHeightStore[MapHeightStoreCount][2] = ObservationTime;
            MapHeightStore[MapHeightStoreCount][5] = ObservationTime;

            // 如果选中的记录是边界且新数据超出边界
            if(MapHeightStore[HitIndex][3+DecreaseSearch] == -1 && ((MapHeightStore[MapHeightLeg[LegNumber]][0] - FootfallPositionRecord[LegNumber][2])*(MapHeightStore[HitIndex][0] - FootfallPositionRecord[LegNumber][2]) > 0))
            {
                MapHeightStore[HitIndex][3+DecreaseSearch] = MapHeightStoreCount;
                MapHeightStore[MapHeightStoreCount][3+DecreaseSearch] = -1;
                MapHeightStore[MapHeightStoreCount][4-DecreaseSearch] = HitIndex;
                MapHeightStoreMaxMinIndex[DecreaseSearch] = MapHeightStoreCount;
            }
            // 如果选中的记录是边界且新数据存储在边界内，或，选中的记录不是边界：需要在Hit和另一个数之间插入值
            else
            {
                MapHeightStore[MapHeightStoreCount][3+DecreaseSearch] = MapHeightStore[int(MapHeightStore[CheckIndex][4-DecreaseSearch])][3+DecreaseSearch];
                MapHeightStore[int(MapHeightStore[CheckIndex][4-DecreaseSearch])][3+DecreaseSearch] = MapHeightStoreCount;
                MapHeightStore[MapHeightStoreCount][4-DecreaseSearch] = MapHeightStore[CheckIndex][4-DecreaseSearch];
                MapHeightStore[CheckIndex][4-DecreaseSearch] = MapHeightStoreCount;
            }

            MapHeightLeg[LegNumber] = MapHeightStoreCount;
        }
        FootfallPositionRecord[LegNumber][2] = FootfallPositionRecord[LegNumber][2] - Zdifference;
    }

    void SensorLegsPos::EstimateGroundPitchAlongHeading(double& move_dir_x, double& move_dir_y, double& move_dir_z)
    {
        // 默认输出：平地前进
        move_dir_x = 1.0;
        move_dir_y = 0.0;
        move_dir_z = 0.0;

        // =========================================================
        // 1) 用当前着地足相对身体的世界坐标系相对位置拟合平面 z = a*x + b*y + c
        // =========================================================
        double sxx = 0.0, sxy = 0.0, syy = 0.0;
        double sx  = 0.0, sy  = 0.0;
        double sxz = 0.0, syz = 0.0, sz  = 0.0;
        int n = 0;
        int count = 0;

        for (int LegNumber = 0; LegNumber < ContactChainNum; ++LegNumber)
        {
            if (JointsBodyWFEffort[LegNumber][FootNodeIndex][2] >= FootEffortThreshold * SlopeModeFootForceAccept)
                continue;
                
            if (!FootfallPositionRecordIsInitiated[LegNumber])
                continue;

            // 至少两个足已落地超0.5秒
            if (ObservationTime - FootfallPositionRecord[LegNumber][3] > SlopeModeTimeThreshold)
                count++;

            const double x = JointsBodyWFPosition[LegNumber][FootNodeIndex][0];
            const double y = JointsBodyWFPosition[LegNumber][FootNodeIndex][1];
            const double z = JointsBodyWFPosition[LegNumber][FootNodeIndex][2];

            sxx += x * x;
            sxy += x * y;
            syy += y * y;
            sx  += x;
            sy  += y;
            sxz += x * z;
            syz += y * z;
            sz  += z;
            ++n;
        }

        double A[3][3] = {
            { sxx, sxy, sx },
            { sxy, syy, sy },
            { sx , sy , static_cast<double>(n) }
        };
        double rhs[3] = { sxz, syz, sz };
        double Ainv[3][3];
        double abc[3] = {0.0, 0.0, 0.0};

        // =========================================================
        // 2) 计算机身前向在世界系水平面的单位方向 hx, hy
        // =========================================================
        double hx = 1.0 - 2.0 * (Est_Quaternion[2] * Est_Quaternion[2] + Est_Quaternion[3] * Est_Quaternion[3]);
        double hy = 2.0 * (Est_Quaternion[1] * Est_Quaternion[2] + Est_Quaternion[0] * Est_Quaternion[3]);
        double hn = std::sqrt(hx * hx + hy * hy);
        if (hn < 1e-9)
        {
            hx = 1.0;
            hy = 0.0;
            hn = 1.0;
        }
        hx /= hn;
        hy /= hn;

        if (!SlopeModeEnable || n < 3 || count < 2 || array_3x3_inverse(A, Ainv) != _ERROR_NO_ERROR)
        {
            move_dir_x = hx;
            move_dir_y = hy;
            move_dir_z = 0.0;
            return;
        }

        array_3x3_multiply_vector(Ainv, rhs, abc);

        const double a = abc[0];
        const double b = abc[1];
        // =========================================================
        // 3) 计算“朝向方向上的坡度”
        //    k = dz / d(水平距离)
        // =========================================================
        const double k = a * hx + b * hy;

        // =========================================================
        // 4) 坡度小：只修正 XY
        // =========================================================
        if (std::fabs(k) <= std::tan(SlopeModeAngleThreshold))
        {
            move_dir_x = hx;
            move_dir_y = hy;
            move_dir_z = 0.0;
            return;
        }

        // =========================================================
        // 5) 坡度大：把轮子滚动分解到 XYZ
        //    hxy_n: 水平分量比例
        //    hz_n : 垂向分量比例
        // =========================================================
        const double hxy_n = 1.0 / std::sqrt(1.0 + k * k);
        const double hz_n  = k * hxy_n;

        move_dir_x = hx * hxy_n;
        move_dir_y = hy * hxy_n;
        move_dir_z = hz_n;
    }

    void SensorLegsPos::GravityCompensate()
    {
        for (int LegNumber = 0; LegNumber < ContactChainNum; ++LegNumber)
            for (int n = LegChains_[LegNumber].node_num - 1; n >= 0; --n)
            {
                if (LegChains_[LegNumber].node[n].q_index < 0)
                    continue;

                double axis_local[3] = {(LegChains_[LegNumber].node[n].axis == TF_AXIS_X) ? 1.0 : 0.0, (LegChains_[LegNumber].node[n].axis == TF_AXIS_Y) ? 1.0 : 0.0, (LegChains_[LegNumber].node[n].axis == TF_AXIS_Z) ? 1.0 : 0.0};
                double axis_wf[3];
                array_quaternion_rotate_vector(LegChains_[LegNumber].node_quat_wf[n], axis_local, axis_wf);
                MotorGravityCompensate[LegNumber][n] = 0.0;

                for (int j = n; j < LegChains_[LegNumber].node_num; ++j)
                {
                    TFNode& body = LegChains_[LegNumber].node[j];
                    const double* body_pos = LegChains_[LegNumber].node_pos_wf[j];
                    double com_wf[3];

                    array_quaternion_rotate_vector(LegChains_[LegNumber].node_quat_wf[j], body.com, com_wf);
                    com_wf[0] += body_pos[0];
                    com_wf[1] += body_pos[1];
                    com_wf[2] += body_pos[2];

                    MotorGravityCompensate[LegNumber][n] += 9.81 * body.mass * (axis_wf[0] * (com_wf[1] - LegChains_[LegNumber].node_pos_wf[n][1]) - axis_wf[1] * (com_wf[0] - LegChains_[LegNumber].node_pos_wf[n][0]));
                }
            }
    }

    void SensorLegsOri::SensorDataHandle(double* Message, double Time) 
    {
        int i;
        int n_ground = 0;

        for (int LegNumber = 0; LegNumber < legs_pos_ref_->ContactChainNum; LegNumber++) {
            if (legs_pos_ref_->FootIsOnGround[LegNumber])
                n_ground++;
        }

        if (n_ground < 2) {
            return;
        }

        double tau_w[3] = {0.0, 0.0, 0.0};

        for (i = 0; i < legs_pos_ref_->ContactChainNum; ++i) {
            tau_w[0] -= legs_pos_ref_->FootBodyTorq_WF[i][0];
            tau_w[1] -= legs_pos_ref_->FootBodyTorq_WF[i][1];
            tau_w[2] -= legs_pos_ref_->FootBodyTorq_WF[i][2];
        }

        const double inertia = 0.4 * legs_pos_ref_->TimelyWeight * 0.5 * 0.5 + 0.01;

        for (i = 0; i < StateSpaceModel->Nz; ++i)
            Observation[i] = 0.0;

        for (i = 0; i < 9; ++i)
            StateSpaceModel->Matrix_H[i * StateSpaceModel->Nx + i] = 0.0;

        Observation[2] = tau_w[0] / inertia;
        Observation[5] = tau_w[1] / inertia;
        Observation[8] = tau_w[2] / inertia;

        StateSpaceModel->Matrix_H[2 * StateSpaceModel->Nx + 2] = 1.0;
        StateSpaceModel->Matrix_H[5 * StateSpaceModel->Nx + 5] = 1.0;
        StateSpaceModel->Matrix_H[8 * StateSpaceModel->Nx + 8] = 1.0;

        StateSpaceModel_Go2_EstimatorPort(Observation, Time, StateSpaceModel);
    }

    void SensorLegsOri::CorrectYawByFootfall(double* Message, double Time) 
    {
        int i;
        int n_ground = 0;

        for (int LegNumber = 0; LegNumber < legs_pos_ref_->ContactChainNum; LegNumber++) {
            if (legs_pos_ref_->FootIsOnGround[LegNumber])
                n_ground++;
        }

        if (n_ground < legs_pos_ref_->ContactChainNum || AllFootOnGroundTimestamp == 0.0) {
            AllFootOnGroundTimestamp = Time;
            legori_current_weight = legori_init_weight;
            return;
        }
        else{
            legori_current_weight = (Time-AllFootOnGroundTimestamp) * (1.0 - legori_init_weight) /legori_time_weight + legori_init_weight;
            if(legori_current_weight>1.0)
                legori_current_weight = 1.0;
        }

        double q_yaw_inv[4];
        double array_EulerZYX[3] = {0.0, 0.0, - StateSpaceModel->EstimatedState[6]};
        array_eulerZYX_to_quaternion(array_EulerZYX, q_yaw_inv);

        double sx = 0.0, sy = 0.0;
        for (i = 0; i < legs_pos_ref_->ContactChainNum; ++i) {

            for (int j = i + 1; j < legs_pos_ref_->ContactChainNum; ++j) {

                double v_wf[3] = {
                    legs_pos_ref_->JointsBodyWFPosition[j][legs_pos_ref_->FootNodeIndex][0] - legs_pos_ref_->JointsBodyWFPosition[i][legs_pos_ref_->FootNodeIndex][0],
                    legs_pos_ref_->JointsBodyWFPosition[j][legs_pos_ref_->FootNodeIndex][1] - legs_pos_ref_->JointsBodyWFPosition[i][legs_pos_ref_->FootNodeIndex][1],
                    legs_pos_ref_->JointsBodyWFPosition[j][legs_pos_ref_->FootNodeIndex][2] - legs_pos_ref_->JointsBodyWFPosition[i][legs_pos_ref_->FootNodeIndex][2]
                };

                double v_rp[3];
                array_quaternion_rotate_vector(q_yaw_inv, v_wf, v_rp);

                double vw_x = legs_pos_ref_->FootfallPositionRecord[j][0] - legs_pos_ref_->FootfallPositionRecord[i][0];
                double vw_y = legs_pos_ref_->FootfallPositionRecord[j][1] - legs_pos_ref_->FootfallPositionRecord[i][1];
                double vw_z = legs_pos_ref_->FootfallPositionRecord[j][2] - legs_pos_ref_->FootfallPositionRecord[i][2];

                const double ang_rp = std::atan2(v_rp[1], v_rp[0]);
                const double ang_w  = std::atan2(vw_y,     vw_x);

                double yaw_ij = ang_w - ang_rp;
                array_angle_wrap(&yaw_ij, &yaw_ij, 1);

                const double pair_weight = legs_pos_ref_->FootfallProbability[i] * legs_pos_ref_->FootfallProbability[j];

                if (pair_weight <= 0.0)
                    continue;

                sx += pair_weight * std::cos(yaw_ij);
                sy += pair_weight * std::sin(yaw_ij);
            }
        }

        if (sx != 0.0 || sy != 0.0) {
            const double yaw_est = std::atan2(sy, sx);
            double err = yaw_est - StateSpaceModel->EstimatedState[6];
            array_angle_wrap(&err, &err, 1);
            yaw_correct += legori_current_weight * err;
            UpdateEst_Quaternion();
        }
    }
    
    void SensorLegsOri::CollisionDetect(double Time)
    {
        CollisionDetectedLeg = 0;

        if (CollisionHistoryCount == 10)
        {
            const double velocity_xy = std::hypot(CollisionVelMean_X, CollisionVelMean_Y);
            const double velocity_norm = std::hypot(velocity_xy, 0.5 * CollisionVelMean_Yaw);

            double impact_x = 0.0;
            double impact_y = 0.0;

            for (int LegNumber = 0; LegNumber < legs_pos_ref_->ContactChainNum; ++LegNumber)
            {
                impact_x += legs_pos_ref_->JointsBodyWFEffort[LegNumber][legs_pos_ref_->FootNodeIndex][0];
                impact_y += legs_pos_ref_->JointsBodyWFEffort[LegNumber][legs_pos_ref_->FootNodeIndex][1];
            }

            impact_x = legs_pos_ref_->StateSpaceModel->EstimatedState[2] + impact_x / legs_pos_ref_->TimelyWeight;
            impact_y = legs_pos_ref_->StateSpaceModel->EstimatedState[5] + impact_y / legs_pos_ref_->TimelyWeight;

            const double impact_norm = std::hypot(impact_x, impact_y);
            const double tilt_factor = std::fmax(((std::fabs(StateSpaceModel->EstimatedState[0]) + std::fabs(StateSpaceModel->EstimatedState[3])) * 180.0 / M_PI - 10.0) / 10.0 + 1.0, 1.0);
            const double angle_error = std::atan2(std::fabs(CollisionVelMean_Y * impact_x - CollisionVelMean_X * impact_y), -CollisionVelMean_X * impact_x - CollisionVelMean_Y * impact_y);
            const double angle_allow = std::fmin(30.0 + 5.0 * std::fabs(StateSpaceModel->EstimatedState[8]), 75.0) * M_PI / 180.0;

            // 速度大于0.3m/s，且加速度大于机器狗倾斜度因子*3.0m/s/s的基准，且加速度方向与速度方向夹角小于允许角度(30~75度)，且距离上次碰撞时间大于3s，则判定为碰撞
            if (velocity_norm >= 0.3 && velocity_xy > 1e-9 && impact_norm > 3.0 * tilt_factor && angle_error <= angle_allow && Time - CollisionLastTimestamp >= 3.0)
            {
                const double cy = std::cos(StateSpaceModel->EstimatedState[6]);
                const double sy = std::sin(StateSpaceModel->EstimatedState[6]);
                const double contact_x = -(cy * impact_x + sy * impact_y) / impact_norm;
                const double contact_y = -(-sy * impact_x + cy * impact_y) / impact_norm;
                const double face_cos = std::cos(15.0 * M_PI / 180.0); // 15度以内，视为面碰撞而非腿碰撞

                if (contact_x >= face_cos)
                    CollisionDetectedLeg = 1;
                else if (contact_y >= face_cos)
                    CollisionDetectedLeg = 3;
                else if (contact_x <= -face_cos)
                    CollisionDetectedLeg = 5;
                else if (contact_y <= -face_cos)
                    CollisionDetectedLeg = 7;
                else if (contact_x >= 0.0 && contact_y >= 0.0)
                    CollisionDetectedLeg = 2;
                else if (contact_x < 0.0 && contact_y >= 0.0)
                    CollisionDetectedLeg = 4;
                else if (contact_x < 0.0 && contact_y < 0.0)
                    CollisionDetectedLeg = 6;
                else
                    CollisionDetectedLeg = 8;

                CollisionLastTimestamp = Time;
            }
        }
        else if(CollisionHistoryCount < 0 || CollisionHistoryCount > 10)
            CollisionHistoryCount = 0;
        else
            CollisionHistoryCount++;

        CollisionVelMean_X += (legs_pos_ref_->StateSpaceModel->EstimatedState[1] - CollisionVelHistery[CollisionHistoryIndex][0]) * 0.1;
        CollisionVelMean_Y += (legs_pos_ref_->StateSpaceModel->EstimatedState[4] - CollisionVelHistery[CollisionHistoryIndex][1]) * 0.1;
        CollisionVelMean_Yaw += (StateSpaceModel->EstimatedState[7] - CollisionVelHistery[CollisionHistoryIndex][2]) * 0.1;

        CollisionVelHistery[CollisionHistoryIndex][0] = legs_pos_ref_->StateSpaceModel->EstimatedState[1];
        CollisionVelHistery[CollisionHistoryIndex][1] = legs_pos_ref_->StateSpaceModel->EstimatedState[4];
        CollisionVelHistery[CollisionHistoryIndex][2] = StateSpaceModel->EstimatedState[7];

        CollisionHistoryIndex = (CollisionHistoryIndex + 1) % 10;

    }
}