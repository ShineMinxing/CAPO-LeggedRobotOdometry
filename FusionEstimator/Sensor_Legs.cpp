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
            for(i = 0; i < 3; i++){
                FootBodyEff_WF[LegNumber][i] = 0;
            }
            for(i = 0; i < 3; i++)
            {
                SensorPosition[i] = LegChains_[LegNumber].node[0].t[i];
            }
            
            Joint2HipFoot(Message,LegNumber);
            
            for(i = 0; i < 3; i++)
            {
                StateSpaceModel->Double_Par[0 + LegNumber * 12 + 0 * 3 + i] = LegChains_[LegNumber].node_pos_wf[0][i];
                StateSpaceModel->Double_Par[0 + LegNumber * 12 + 1 * 3 + i] = LegChains_[LegNumber].node_pos_wf[1][i];
                StateSpaceModel->Double_Par[0 + LegNumber * 12 + 2 * 3 + i] = LegChains_[LegNumber].node_pos_wf[2][i];
                StateSpaceModel->Double_Par[0 + LegNumber * 12 + 3 * 3 + i] = FootBodyPos_WF[LegNumber][i];
            }

            if(JointsXYZEnable){
                for(i = 0; i < 3; i++)
                {
                    FootBodyPos_WF[LegNumber][i] = Observation[3 * i];
                }
            }

            if(JointsXYZVelocityEnable){
                for(i = 0; i < 3; i++)
                {
                    FootBodyVel_WF[LegNumber][i] = Observation[3 * i + 1];
                }
            }
        }
        
        if(FootIsOnGround[0]||FootIsOnGround[1]||FootIsOnGround[2]||FootIsOnGround[3])
        {
            for(i = 0; i < 9; i++)
            {
                StateSpaceModel->Matrix_H[i * StateSpaceModel->Nx + i] = 0;
            }

            FootFallPositionRecord(Message);

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
                    p_w[LegNumber][0] = StateSpaceModel->EstimatedState[0] + FootBodyPos_WF[LegNumber][0];
                    p_w[LegNumber][1] = StateSpaceModel->EstimatedState[3] + FootBodyPos_WF[LegNumber][1];
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
                static double yaw_ff_last = 0.0;
                static double yaw_ff_turn = 0.0;
                array_angle_unwrap(&yaw_ff, &yaw_ff_last, &yaw_ff_turn, 1);

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
        static constexpr int WIN_T = 25;
        static constexpr double STABLE_TIME = 0.5;

        static double buf100[WIN_T] = {0.0};
        static int    buf100_i = 0;
        static int    buf100_n = 0;
        static double sum100   = 0.0;

        static bool stable_started = false;
        static double stable_start_time = 0.0;

        const bool stand_on_ground = (FootfallProbability[0] + FootfallProbability[1] + FootfallProbability[2] + FootfallProbability[3]) > 2.5 && (std::abs(StateSpaceModel->EstimatedState[1]) + std::abs(StateSpaceModel->EstimatedState[4]) +std::abs(StateSpaceModel->EstimatedState[7])) < 0.05;

        if (stand_on_ground) {
            if (!stable_started || Time < stable_start_time) {
                stable_started = true;
                stable_start_time = Time;
            }
        } else {
            stable_started = false;
            buf100_i = 0;
            buf100_n = 0;
            sum100 = 0.0;
            return;
        }

        if (Time - stable_start_time < STABLE_TIME)
            return;
        
        const double fz_sum =
            FootBodyEff_WF[0][2] + FootBodyEff_WF[1][2] +
            FootBodyEff_WF[2][2] + FootBodyEff_WF[3][2];

        if (buf100_n < WIN_T) {
            buf100[buf100_i] = fz_sum;
            sum100 += fz_sum;
            buf100_n++;
        } else {
            sum100 -= buf100[buf100_i];
            buf100[buf100_i] = fz_sum;
            sum100 += fz_sum;
        }

        buf100_i++;
        if (buf100_i >= WIN_T)
            buf100_i = 0;

        const double mean100 = (buf100_n > 0) ? (sum100 / (double)buf100_n) : 0.0;

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
            array_quaternion_normalize(q_pre, q_pre);

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

        array_quaternion_rotate_vector(
            LegChains_[LegNumber].node_quat_wf[LegChains_[LegNumber].ee.parent],
            LegChains_[LegNumber].ee.t,
            FootBodyPos_WF[LegNumber]
        );

        FootBodyPos_WF[LegNumber][0] += LegChains_[LegNumber].node_pos_wf[LegChains_[LegNumber].ee.parent][0];
        FootBodyPos_WF[LegNumber][1] += LegChains_[LegNumber].node_pos_wf[LegChains_[LegNumber].ee.parent][1];
        FootBodyPos_WF[LegNumber][2] += LegChains_[LegNumber].node_pos_wf[LegChains_[LegNumber].ee.parent][2];

        Observation[0] = FootBodyPos_WF[LegNumber][0];
        Observation[3] = FootBodyPos_WF[LegNumber][1];
        Observation[6] = FootBodyPos_WF[LegNumber][2];

        Observation[1] = 0.0;
        Observation[4] = 0.0;
        Observation[7] = 0.0;

        double Jtau[3];
        double JJT[3][3];
        double JJT_inv[3][3];

        for (int target = 1; target <= joint_num; ++target)
        {
            if (target > 2 && target < joint_num)
                continue;

            Jtau[0] = Jtau[1] = Jtau[2] = 0.0;

            for (int r = 0; r < 3; ++r)
                for (int c = 0; c < 3; ++c)
                    JJT[r][c] = 0.0;

            for (int j = 0; j < target; ++j)
            {
                const double rx = (target == joint_num ? FootBodyPos_WF[LegNumber][0] : joint_org[target][0]) - joint_org[j][0];
                const double ry = (target == joint_num ? FootBodyPos_WF[LegNumber][1] : joint_org[target][1]) - joint_org[j][1];
                const double rz = (target == joint_num ? FootBodyPos_WF[LegNumber][2] : joint_org[target][2]) - joint_org[j][2];

                const double J0 = joint_axis[j][1] * rz - joint_axis[j][2] * ry;
                const double J1 = joint_axis[j][2] * rx - joint_axis[j][0] * rz;
                const double J2 = joint_axis[j][0] * ry - joint_axis[j][1] * rx;

                if (target == joint_num)
                {
                    Observation[1] += J0 * joint_dq[j];
                    Observation[4] += J1 * joint_dq[j];
                    Observation[7] += J2 * joint_dq[j];
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
                StateSpaceModel->Double_Par[48 + LegNumber * 9 + (target == joint_num ? 6 : (target - 1) * 3) + i] = 0.0;

            if (array_3x3_inverse(JJT, JJT_inv) == _ERROR_NO_ERROR)
                array_3x3_multiply_vector(JJT_inv, Jtau, &StateSpaceModel->Double_Par[48 + LegNumber * 9 + (target == joint_num ? 6 : (target - 1) * 3)]);
        }

        Observation[1] = -Observation[1];
        Observation[4] = -Observation[4];
        Observation[7] = -Observation[7];

        for (int i = 0; i < 3; ++i)
            FootBodyEff_WF[LegNumber][i] = StateSpaceModel->Double_Par[48 + LegNumber * 9 + 6 + i];

        FootBodyTorq_WF[LegNumber][0] = FootBodyPos_WF[LegNumber][1] * FootBodyEff_WF[LegNumber][2] - FootBodyPos_WF[LegNumber][2] * FootBodyEff_WF[LegNumber][1];
        FootBodyTorq_WF[LegNumber][1] = FootBodyPos_WF[LegNumber][2] * FootBodyEff_WF[LegNumber][0] - FootBodyPos_WF[LegNumber][0] * FootBodyEff_WF[LegNumber][2];
        FootBodyTorq_WF[LegNumber][2] = FootBodyPos_WF[LegNumber][0] * FootBodyEff_WF[LegNumber][1] - FootBodyPos_WF[LegNumber][1] * FootBodyEff_WF[LegNumber][0];


        if(FootBodyEff_WF[LegNumber][2] >= 0.3 * FootEffortThreshold)
            FootfallProbability[LegNumber] = 0.0;
        else if(FootBodyEff_WF[LegNumber][2] <= 1.3 * FootEffortThreshold)
            FootfallProbability[LegNumber] = 1.0;
        else
            FootfallProbability[LegNumber] = (FootBodyEff_WF[LegNumber][2] - 0.3 * FootEffortThreshold) / (FootEffortThreshold);

        if(FootBodyEff_WF[LegNumber][2] < FootEffortThreshold)
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
            int count;
            for(count = 0; count < ContactChainNum; count++)
                if(FootIsOnGround[count])
                    break;
            if(count == ContactChainNum && FootBodyPos_WF[LegNumber][2] > -0.25)
                FootIsOnGround[LegNumber] = true;
        }

        FootWasOnGround[LegNumber] = FootIsOnGround[LegNumber];
    }

    void SensorLegsPos::FootFallPositionRecord(double *Message){

        double p_sum[3] = {0}, v_sum[3] = {0};
        int    leg_cnt = 0;
        static double ShankPitchPrev[MAX_CONTACT_CHAIN] = {0};
        static double ShankRollPrev[MAX_CONTACT_CHAIN] = {0};

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
                FootfallPositionRecord[LegNumber][0] = StateSpaceModel->EstimatedState[0] + FootBodyPos_WF[LegNumber][0];
                FootfallPositionRecord[LegNumber][1] = StateSpaceModel->EstimatedState[3] + FootBodyPos_WF[LegNumber][1];
                FootfallPositionRecord[LegNumber][2] = 0;
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

            }
            else if(FootLanding[LegNumber])
            {
                FootLanding[LegNumber]= false;
                FootfallPositionRecord[LegNumber][0] = StateSpaceModel->EstimatedState[0] + FootBodyPos_WF[LegNumber][0];
                FootfallPositionRecord[LegNumber][1] = StateSpaceModel->EstimatedState[3] + FootBodyPos_WF[LegNumber][1];
                FootfallPositionRecord[LegNumber][2] = StateSpaceModel->EstimatedState[6] + FootBodyPos_WF[LegNumber][2];
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
            FootfallVelocityRecordTemp[LegNumber][0] = FootBodyVel_WF[LegNumber][0] + WheelVel * move_dir_x;
            FootfallVelocityRecordTemp[LegNumber][1] = FootBodyVel_WF[LegNumber][1] + WheelVel * move_dir_y;
            FootfallVelocityRecordTemp[LegNumber][2] = FootBodyVel_WF[LegNumber][2] + WheelVel * move_dir_z;
            FootIsOnGroundTemp[LegNumber] = true;

        }

        if (WheelMoveRefLeg >= 0)
        {
            for (int LegNumber = 0; LegNumber < ContactChainNum; ++LegNumber)
            {
                if (!FootIsOnGroundTemp[LegNumber]) continue;

                double CalculatedPosition[3];
                CalculatedPosition[0] = FootfallPositionRecordTemp[WheelMoveRefLeg][0] + FootBodyPos_WF[LegNumber][0] - FootBodyPos_WF[WheelMoveRefLeg][0];
                CalculatedPosition[1] = FootfallPositionRecordTemp[WheelMoveRefLeg][1] + FootBodyPos_WF[LegNumber][1] - FootBodyPos_WF[WheelMoveRefLeg][1];
                CalculatedPosition[2] = FootfallPositionRecordTemp[WheelMoveRefLeg][2] + FootBodyPos_WF[LegNumber][2] - FootBodyPos_WF[WheelMoveRefLeg][2];

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
            p_sum[0] += FootfallPositionRecordTemp[LegNumber][0] - FootBodyPos_WF[LegNumber][0];
            p_sum[1] += FootfallPositionRecordTemp[LegNumber][1] - FootBodyPos_WF[LegNumber][1];
            p_sum[2] += FootfallPositionRecordTemp[LegNumber][2] - FootBodyPos_WF[LegNumber][2];
            v_sum[0] += FootfallVelocityRecordTemp[LegNumber][0];
            v_sum[1] += FootfallVelocityRecordTemp[LegNumber][1];
            v_sum[2] += FootfallVelocityRecordTemp[LegNumber][2];
            leg_cnt++;
        }

        Observation[0] = p_sum[0] / (double)leg_cnt;
        Observation[3] = p_sum[1] / (double)leg_cnt;
        Observation[6] = p_sum[2] / (double)leg_cnt;
        Observation[1] = v_sum[0] / (double)leg_cnt;
        Observation[4] = v_sum[1] / (double)leg_cnt;
        Observation[7] = v_sum[2] / (double)leg_cnt;
        for (int LegNumber = 0; LegNumber < ContactChainNum; ++LegNumber)
        {
            Observation[2] -= FootBodyEff_WF[LegNumber][0] / TimelyWeight;
            Observation[5] -= FootBodyEff_WF[LegNumber][1] / TimelyWeight;
            Observation[8] -= FootBodyEff_WF[LegNumber][2] / TimelyWeight;
        }
        Observation[8] -= 9.79;
    }

    void SensorLegsPos::ClusterFootfallHeight(int LegNumber, double move_dir_z)
    {
        static double MapHeightStore[3][1000] = {0};
        static int MapHeightStoreMax = 0;
        int i = 0;
        double Zdifference = 99;
    
        for(i = 0; i < (MapHeightStoreMax+1); i++)
        {
            if(MapHeightStore[2][i] != 0 && std::abs(ObservationTime-MapHeightStore[2][i]) > Data_Fading_Time)
            {
                MapHeightStore[0][i] = 0;
                MapHeightStore[1][i] = 0;
                MapHeightStore[2][i] = 0;
        }
        }

        for(i = 0; i < (MapHeightStoreMax+1); i++){
            if((std::abs(MapHeightStore[0][i] - FootfallPositionRecord[LegNumber][2]) <= Environement_Height_Scope && move_dir_z == 0.0 ) || std::abs(MapHeightStore[0][i] - FootfallPositionRecord[LegNumber][2]) <= SlopeModeStepHeightThreshold)
            {

                MapHeightStore[1][i] *= exp(- (ObservationTime - MapHeightStore[2][i]) / (10 * Data_Fading_Time));
                MapHeightStore[1][i] += 1;
                MapHeightStore[2][i] = ObservationTime;
                if(std::abs(MapHeightStore[0][i] - FootfallPositionRecord[LegNumber][2]) <= Environement_Height_Scope/10)
                    Zdifference = 0;
                else
                    Zdifference = FootfallPositionRecord[LegNumber][2] - MapHeightStore[0][i];

                break;
            }
        }
        if(Zdifference == 99){
            Zdifference = 0;
            for(i = 0; i < (MapHeightStoreMax+1); i++)
            {
                if(MapHeightStore[2][i] == 0)
                {
                    MapHeightStore[0][i] = FootfallPositionRecord[LegNumber][2];
                    MapHeightStore[1][i] = 1;
                    MapHeightStore[2][i] = ObservationTime;
                    break;
                }
            }
            if(i >= 999)
            {
                for(i = 0; i < (MapHeightStoreMax+1); i++)
                {
                    if(MapHeightStore[2][i] != 0 && std::abs(ObservationTime-MapHeightStore[2][i]) > 60)
                    {
                        MapHeightStore[0][i] = 0;
                        MapHeightStore[1][i] = 0;
                        MapHeightStore[2][i] = 0;
                    }
                }
                i = 0;
                MapHeightStore[0][i] = FootfallPositionRecord[LegNumber][2];
                MapHeightStore[1][i] = 1;
                MapHeightStore[2][i] = ObservationTime;
            }
            if(i == MapHeightStoreMax + 1)
            {
                MapHeightStoreMax = i;
                MapHeightStore[0][i] = FootfallPositionRecord[LegNumber][2];
                MapHeightStore[1][i] = 1;
                MapHeightStore[2][i] = ObservationTime;
            }
            
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
        // 1) 用当前着地足的 FootBodyPos_WF 拟合平面 z = a*x + b*y + c
        // =========================================================
        double sxx = 0.0, sxy = 0.0, syy = 0.0;
        double sx  = 0.0, sy  = 0.0;
        double sxz = 0.0, syz = 0.0, sz  = 0.0;
        int n = 0;
        int count = 0;

        for (int LegNumber = 0; LegNumber < ContactChainNum; ++LegNumber)
        {
            if (FootBodyEff_WF[LegNumber][2] >= FootEffortThreshold * SlopeModeFootForceAccept)
                continue;
                
            if (!FootfallPositionRecordIsInitiated[LegNumber])
                continue;

            // 至少两个足已落地超0.5秒
            if (ObservationTime - FootfallPositionRecord[LegNumber][3] > SlopeModeTimeThreshold)
                count++;

            const double x = FootBodyPos_WF[LegNumber][0];
            const double y = FootBodyPos_WF[LegNumber][1];
            const double z = FootBodyPos_WF[LegNumber][2];

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

        if (!SlopeModeEnable || n < 3 || array_3x3_inverse(A, Ainv) != _ERROR_NO_ERROR || count < 2)
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
                double axis_local[3] = {(LegChains_[LegNumber].node[n].axis == TF_AXIS_X) ? 1.0 : 0.0, (LegChains_[LegNumber].node[n].axis == TF_AXIS_Y) ? 1.0 : 0.0, (LegChains_[LegNumber].node[n].axis == TF_AXIS_Z) ? 1.0 : 0.0};
                double axis_wf[3];
                array_quaternion_rotate_vector(LegChains_[LegNumber].node_quat_wf[n], axis_local, axis_wf);
                MotorGravityCompensate[LegNumber][n] = 0.0;

                for (int j = n; j <= LegChains_[LegNumber].node_num; ++j)
                {
                    TFNode& body = (j == LegChains_[LegNumber].node_num) ? LegChains_[LegNumber].ee : LegChains_[LegNumber].node[j];
                    const double* body_pos = (j == LegChains_[LegNumber].node_num) ? FootBodyPos_WF[LegNumber] : LegChains_[LegNumber].node_pos_wf[j];
                    double body_quat[4];
                    double com_wf[3];

                    if (j == LegChains_[LegNumber].node_num)
                    {
                        array_quaternion_multiplication(LegChains_[LegNumber].node_quat_wf[body.parent], body.q_fix, body_quat);
                        array_quaternion_normalize(body_quat, body_quat);
                    }
                    else
                    {
                        body_quat[0] = LegChains_[LegNumber].node_quat_wf[j][0];
                        body_quat[1] = LegChains_[LegNumber].node_quat_wf[j][1];
                        body_quat[2] = LegChains_[LegNumber].node_quat_wf[j][2];
                        body_quat[3] = LegChains_[LegNumber].node_quat_wf[j][3];
                    }

                    array_quaternion_rotate_vector(body_quat, body.com, com_wf);
                    com_wf[0] += body_pos[0];
                    com_wf[1] += body_pos[1];
                    com_wf[2] += body_pos[2];

                    MotorGravityCompensate[LegNumber][n] += 9.81 * body.mass * (axis_wf[0] * (com_wf[1] - LegChains_[LegNumber].node_pos_wf[n][1]) - axis_wf[1] * (com_wf[0] - LegChains_[LegNumber].node_pos_wf[n][0]));
                }

                StateSpaceModel->Double_Par[84 + LegNumber * 3 + n] = MotorGravityCompensate[LegNumber][n];
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
        static double TimeRecord = Time;

        for (int LegNumber = 0; LegNumber < legs_pos_ref_->ContactChainNum; LegNumber++) {
            if (legs_pos_ref_->FootIsOnGround[LegNumber])
                n_ground++;
        }

        if (n_ground < legs_pos_ref_->ContactChainNum){
            TimeRecord = Time;
            legori_current_weight = legori_init_weight;
            return;
        }
        else{
            legori_current_weight = (Time-TimeRecord) * (1.0 - legori_init_weight) /legori_time_weight + legori_init_weight;
            if(legori_current_weight>1.0)
                legori_current_weight = 1.0;
        }

        double q_yaw_inv[4];
        double array_EulerZYX[3] = {0.0, 0.0, - StateSpaceModel->EstimatedState[6]};
        array_eulerZYX_to_quaternion(array_EulerZYX, q_yaw_inv);
        array_quaternion_normalize(q_yaw_inv, q_yaw_inv);

        double sx = 0.0, sy = 0.0;
        for (i = 0; i < legs_pos_ref_->ContactChainNum; ++i) {

            for (int j = i + 1; j < legs_pos_ref_->ContactChainNum; ++j) {

                double v_wf[3] = {
                    legs_pos_ref_->FootBodyPos_WF[j][0] - legs_pos_ref_->FootBodyPos_WF[i][0],
                    legs_pos_ref_->FootBodyPos_WF[j][1] - legs_pos_ref_->FootBodyPos_WF[i][1],
                    legs_pos_ref_->FootBodyPos_WF[j][2] - legs_pos_ref_->FootBodyPos_WF[i][2]
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

                int k = i+j;
                if(i==0)
                    k--;

                // printf("%d: ang_rp-%lf; ang_w-%lf; yaw_ij-%lf \n",k,ang_rp,ang_w,yaw_ij);
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
        static double velocity_history[10][3] = {{0.0}};
        static double vx_mean = 0.0;
        static double vy_mean = 0.0;
        static double yaw_velocity_mean = 0.0;
        static int history_count = 0;
        static int history_index = 0;
        static double last_collision_time = -1e100;

        CollisionDetectedLeg = 0;

        if (history_count == 10)
        {
            const double velocity_xy = std::hypot(vx_mean, vy_mean);
            const double velocity_norm = std::hypot(velocity_xy, 0.5 * yaw_velocity_mean);

            double impact_x = 0.0;
            double impact_y = 0.0;

            for (int leg = 0; leg < legs_pos_ref_->ContactChainNum; ++leg)
            {
                impact_x += legs_pos_ref_->FootBodyEff_WF[leg][0];
                impact_y += legs_pos_ref_->FootBodyEff_WF[leg][1];
            }

            impact_x = legs_pos_ref_->StateSpaceModel->EstimatedState[2] + impact_x / legs_pos_ref_->TimelyWeight;
            impact_y = legs_pos_ref_->StateSpaceModel->EstimatedState[5] + impact_y / legs_pos_ref_->TimelyWeight;

            const double impact_norm = std::hypot(impact_x, impact_y);
            const double tilt_factor = std::fmax(((std::fabs(StateSpaceModel->EstimatedState[0]) + std::fabs(StateSpaceModel->EstimatedState[3])) * 180.0 / M_PI - 10.0) / 10.0 + 1.0, 1.0);
            const double angle_error = std::atan2(std::fabs(vy_mean * impact_x - vx_mean * impact_y), -vx_mean * impact_x - vy_mean * impact_y);
            const double angle_allow = std::fmin(30.0 + 5.0 * std::fabs(StateSpaceModel->EstimatedState[8]), 75.0) * M_PI / 180.0;

            // 速度大于0.3m/s，且加速度大于机器狗倾斜度因子*3.0m/s/s的基准，且加速度方向与速度方向夹角小于允许角度(30~75度)，且距离上次碰撞时间大于3s，则判定为碰撞
            if (velocity_norm >= 0.3 && velocity_xy > 1e-9 && impact_norm > 3.0 * tilt_factor && angle_error <= angle_allow && Time - last_collision_time >= 3.0)
            {
                const double cy = std::cos(StateSpaceModel->EstimatedState[6]);
                const double sy = std::sin(StateSpaceModel->EstimatedState[6]);
                const double contact_x = -(cy * impact_x + sy * impact_y) / impact_norm;
                const double contact_y = -(-sy * impact_x + cy * impact_y) / impact_norm;
                const double face_cos = std::cos(15.0 * M_PI / 180.0); // 15度以内，视为面碰撞而非腿碰撞

                if (contact_x >= face_cos)
                    CollisionDetectedLeg = 2;
                else if (contact_y <= -face_cos)
                    CollisionDetectedLeg = 4;
                else if (contact_x <= -face_cos)
                    CollisionDetectedLeg = 6;
                else if (contact_y >= face_cos)
                    CollisionDetectedLeg = 8;
                else if (contact_x >= 0.0 && contact_y >= 0.0)
                    CollisionDetectedLeg = 1;
                else if (contact_x >= 0.0)
                    CollisionDetectedLeg = 3;
                else if (contact_y < 0.0)
                    CollisionDetectedLeg = 5;
                else
                    CollisionDetectedLeg = 7;

                last_collision_time = Time;
            }
        }
        else if(history_count < 0 || history_count > 10)
            history_count = 0;
        else
            history_count++;

        vx_mean += (legs_pos_ref_->StateSpaceModel->EstimatedState[1] - velocity_history[history_index][0]) * 0.1;
        vy_mean += (legs_pos_ref_->StateSpaceModel->EstimatedState[4] - velocity_history[history_index][1]) * 0.1;
        yaw_velocity_mean += (StateSpaceModel->EstimatedState[7] - velocity_history[history_index][2]) * 0.1;

        velocity_history[history_index][0] = legs_pos_ref_->StateSpaceModel->EstimatedState[1];
        velocity_history[history_index][1] = legs_pos_ref_->StateSpaceModel->EstimatedState[4];
        velocity_history[history_index][2] = StateSpaceModel->EstimatedState[7];

        history_index = (history_index + 1) % 10;

    }
}