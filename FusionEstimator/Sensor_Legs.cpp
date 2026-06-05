#include "Sensor_Legs.h"

namespace DataFusion
{
    void SensorLegsPos::SensorDataHandle(double* Message, double Time) 
    {
        if((!JointsXYZEnable)&&(!JointsXYZVelocityEnable)){
            return;
        }

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
                StateSpaceModel->Double_Par[12 + LegNumber * 12 + 0 * 3 + i] = LegChains_[LegNumber].node_pos_wf[0][i];
                StateSpaceModel->Double_Par[12 + LegNumber * 12 + 1 * 3 + i] = LegChains_[LegNumber].node_pos_wf[1][i];
                StateSpaceModel->Double_Par[12 + LegNumber * 12 + 2 * 3 + i] = LegChains_[LegNumber].node_pos_wf[2][i];
                StateSpaceModel->Double_Par[12 + LegNumber * 12 + 3 * 3 + i] = FootBodyPos_WF[LegNumber][i];
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
    }

    void SensorLegsPos::LoadedWeightCheck(double* Message, double Time) 
    {
        static constexpr int WIN_T = 50;
        static constexpr int STABLE_N = 100;

        static double buf100[WIN_T] = {0.0};
        static int    buf100_i = 0;
        static int    buf100_n = 0;
        static double sum100   = 0.0;

        static int stable_cnt = 0; // 连续四足着地计数（饱和到 STABLE_N，避免溢出）

        const bool all_on_ground = (FootfallProbability[0] + FootfallProbability[1] + FootfallProbability[2] + FootfallProbability[3]) > 2.5;

        if (all_on_ground) {
            if (stable_cnt < STABLE_N) stable_cnt++;
        } else {
            stable_cnt = 0;
            buf100_i = 0; buf100_n = 0; sum100 = 0.0; 
        }

        if (stable_cnt >= STABLE_N) {
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
            if (buf100_i >= WIN_T) buf100_i = 0;

            const double mean100 = (buf100_n > 0) ? (sum100 / (double)buf100_n) : 0.0;

            TimelyWeight = - mean100 * 0.1;

            if(TimelyWeight < MinimumWeight) TimelyWeight = MinimumWeight;
        }
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

        double Jtau[3] = {0.0, 0.0, 0.0};
        double JJT[3][3] = {{0.0}};
        double JJT_inv[3][3];

        for (int j = 0; j < joint_num; ++j)
        {
            const double rx = FootBodyPos_WF[LegNumber][0] - joint_org[j][0];
            const double ry = FootBodyPos_WF[LegNumber][1] - joint_org[j][1];
            const double rz = FootBodyPos_WF[LegNumber][2] - joint_org[j][2];

            const double J0 = joint_axis[j][1] * rz - joint_axis[j][2] * ry;
            const double J1 = joint_axis[j][2] * rx - joint_axis[j][0] * rz;
            const double J2 = joint_axis[j][0] * ry - joint_axis[j][1] * rx;

            Observation[1] += J0 * joint_dq[j];
            Observation[4] += J1 * joint_dq[j];
            Observation[7] += J2 * joint_dq[j];

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

        Observation[1] = -Observation[1];
        Observation[4] = -Observation[4];
        Observation[7] = -Observation[7];

        JJT[1][0] = JJT[0][1];
        JJT[2][0] = JJT[0][2];
        JJT[2][1] = JJT[1][2];

        FootBodyEff_WF[LegNumber][0] = 0.0;
        FootBodyEff_WF[LegNumber][1] = 0.0;
        FootBodyEff_WF[LegNumber][2] = 0.0;
        
        if (array_3x3_inverse(JJT, JJT_inv) == _ERROR_NO_ERROR)
            array_3x3_multiply_vector(JJT_inv, Jtau, FootBodyEff_WF[LegNumber]);

        const double tau_w[3] = {
            FootBodyPos_WF[LegNumber][1] * FootBodyEff_WF[LegNumber][2] - FootBodyPos_WF[LegNumber][2] * FootBodyEff_WF[LegNumber][1],
            FootBodyPos_WF[LegNumber][2] * FootBodyEff_WF[LegNumber][0] - FootBodyPos_WF[LegNumber][0] * FootBodyEff_WF[LegNumber][2],
            FootBodyPos_WF[LegNumber][0] * FootBodyEff_WF[LegNumber][1] - FootBodyPos_WF[LegNumber][1] * FootBodyEff_WF[LegNumber][0]
        };

        for(int i = 0; i < 3; i++)
            StateSpaceModel->Double_Par[LegNumber * 3 + i] = FootBodyEff_WF[LegNumber][i];

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

                static double MapHeightStore[3][1000] = {0};
                static int MapHeightStoreMax = 0;
                int i = 0;
                double Zdifference = 99;
                
                // std::cout << "[LAND] L" << LegNumber
                //     << " t=" << ObservationTime
                //     << " z_in=" << (StateSpaceModel->EstimatedState[6] + Observation[6])
                //     << " scope=" << Environement_Height_Scope
                //     << " fade=" << Data_Fading_Time
                //     << " max=" << MapHeightStoreMax
                //     << std::endl;
            
                for(i = 0; i < (MapHeightStoreMax+1); i++)
                {
                    if(MapHeightStore[2][i] != 0 && std::abs(ObservationTime-MapHeightStore[2][i]) > Data_Fading_Time)
                    {
                        MapHeightStore[0][i] = 0;
                        MapHeightStore[1][i] = 0;
                        MapHeightStore[2][i] = 0;
                        // std::cout <<"One old step cleared" << std::endl;
                }
                }

                for(i = 0; i < (MapHeightStoreMax+1); i++){
                    // if(std::abs(MapHeightStore[0][i] - FootfallPositionRecord[LegNumber][2]) <= Environement_Height_Scope)
                    if((std::abs(MapHeightStore[0][i] - FootfallPositionRecord[LegNumber][2]) <= Environement_Height_Scope && move_dir_z == 0.0 ) || std::abs(MapHeightStore[0][i] - FootfallPositionRecord[LegNumber][2]) <= SlopeModeStepHeightThreshold)
                    {
                        // std::cout << "[HIT] i=" << i
                        //         << " rec=" << MapHeightStore[0][i]
                        //         << " in="  << FootfallPositionRecord[LegNumber][2]
                        //         << " dz="  << std::abs(MapHeightStore[0][i] - FootfallPositionRecord[LegNumber][2])
                        //         << " conf->" << MapHeightStore[1][i]
                        //         << std::endl;

                        MapHeightStore[1][i] *= exp(- (ObservationTime - MapHeightStore[2][i]) / (10 * Data_Fading_Time));
                        MapHeightStore[1][i] += 1;
                        MapHeightStore[2][i] = ObservationTime;
                        if(std::abs(MapHeightStore[0][i] - FootfallPositionRecord[LegNumber][2]) <= Environement_Height_Scope/10)
                            Zdifference = 0;
                        else
                            Zdifference = FootfallPositionRecord[LegNumber][2] - MapHeightStore[0][i];

                        // std::cout << "[CORR] z_diff=" << Zdifference
                        //     << " z_out=" << (FootfallPositionRecord[LegNumber][2] - Zdifference)
                        //     << std::endl;
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
                            // std::cout << "[NEW] i=" << i
                            //     << " h=" << MapHeightStore[0][i]
                            //     << " t=" << MapHeightStore[2][i]
                            //     << " max=" << MapHeightStoreMax
                            //     << std::endl;
                            break;
                        }
                    }
                    if(i >= 999)
                    {
                        // std::cout << "[FULL] store full-ish, overwrite slot0"
                        //     << " t=" << ObservationTime
                        //     << " max=" << MapHeightStoreMax
                        //     << std::endl;

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
                WheelSidewayMove = LegChains_[LegNumber].wheel_radius * 2.0 * std::sin(0.5 * ShankRollAngle);
                WheelVel = LegChains_[LegNumber].wheel_radius * WheelRotationVelocityEff;
            }

            FootfallPositionRecord[LegNumber][0] += WheelMove * move_dir_x + WheelSidewayMove * (-move_dir_y);
            FootfallPositionRecord[LegNumber][1] += WheelMove * move_dir_y + WheelSidewayMove * ( move_dir_x);
            FootfallPositionRecord[LegNumber][2] += WheelMove * move_dir_z;

            p_sum[0] += FootfallPositionRecord[LegNumber][0] - FootBodyPos_WF[LegNumber][0];
            p_sum[1] += FootfallPositionRecord[LegNumber][1] - FootBodyPos_WF[LegNumber][1];
            p_sum[2] += FootfallPositionRecord[LegNumber][2] - FootBodyPos_WF[LegNumber][2];

            v_sum[0] += FootBodyVel_WF[LegNumber][0] + WheelVel * move_dir_x;
            v_sum[1] += FootBodyVel_WF[LegNumber][1] + WheelVel * move_dir_y;
            v_sum[2] += FootBodyVel_WF[LegNumber][2] + WheelVel * move_dir_z;
            
            leg_cnt++;
        }

        Observation[0] = p_sum[0] / (double)leg_cnt;
        Observation[3] = p_sum[1] / (double)leg_cnt;
        Observation[6] = p_sum[2] / (double)leg_cnt;
        Observation[1] = v_sum[0] / (double)leg_cnt;
        Observation[4] = v_sum[1] / (double)leg_cnt;
        Observation[7] = v_sum[2] / (double)leg_cnt;
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

    void SensorLegsOri::SensorDataHandle(double* Message, double Time) 
    {
        static double TimeRecord = Time;
        int LegNumber, i;
        int n_ground = 0;
        const double yaw_now = StateSpaceModel->EstimatedState[6];

        legori_correct = yaw_now;

        for (LegNumber = 0; LegNumber < legs_pos_ref_->ContactChainNum; LegNumber++) {
            if (legs_pos_ref_->FootIsOnGround[LegNumber])
                n_ground++;
        }

        if (n_ground < 2) {
            return;
        }
        if (n_ground < legs_pos_ref_->ContactChainNum){
            TimeRecord = Time;
            legori_current_weight = legori_init_weight;
        }
        else{
            legori_current_weight = (Time-TimeRecord) * (1.0 - legori_init_weight) /legori_time_weight + legori_init_weight;
            if(legori_current_weight>1.0)
                legori_current_weight = 1.0;
        }

        double q_yaw_inv[4];
        double array_EulerZYX[3] = {0.0, 0.0, -yaw_now};
        array_eulerZYX_to_quaternion(array_EulerZYX, q_yaw_inv);
        array_quaternion_normalize(q_yaw_inv, q_yaw_inv);

        double sx = 0.0, sy = 0.0;
        for (i = 0; i < legs_pos_ref_->ContactChainNum; ++i) {
            if(!legs_pos_ref_->FootIsOnGround[i])
                continue;

            for (int j = i + 1; j < legs_pos_ref_->ContactChainNum; ++j) {
                if(!legs_pos_ref_->FootIsOnGround[j])
                    continue;

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
            const double yaw_now = StateSpaceModel->EstimatedState[6];
            double err = yaw_est - yaw_now;
            array_angle_wrap(&err, &err, 1);
            legori_correct = yaw_now + legori_current_weight * err;
            array_angle_wrap(&legori_correct, &legori_correct, 1);
        }
    }
}