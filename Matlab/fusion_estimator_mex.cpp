#include "mex.h"
#include <cstdint>
#include <cstring>
#include <cmath>

#ifndef NOMINMAX
#  define NOMINMAX
#endif
#ifndef _CRT_SECURE_NO_WARNINGS
#  define _CRT_SECURE_NO_WARNINGS
#endif
#ifndef _USE_MATH_DEFINES
#  define _USE_MATH_DEFINES
#endif

#include "../FusionEstimator/fusion_estimator.h"

static FusionEstimatorCore* g_core = nullptr;

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
    char cmd[32] = {0};
    mxGetString(prhs[0], cmd, sizeof(cmd));

    if (std::strcmp(cmd, "reset") == 0) {
        delete g_core;
        g_core = nullptr;
        if (nlhs > 0) plhs[0] = mxCreateLogicalScalar(true);
        return;
    }

    if (!g_core) g_core = new FusionEstimatorCore();

    if (std::strcmp(cmd, "status") == 0) {
        double status[100];
        std::memcpy(status, mxGetPr(prhs[1]), sizeof(status));
        g_core->fusion_estimator_status(status);

        if (nlhs > 0) {
            plhs[0] = mxCreateDoubleMatrix(100, 1, mxREAL);
            std::memcpy(mxGetPr(plhs[0]), status, sizeof(status));
        }
        return;
    }

    if (std::strcmp(cmd, "step") == 0) {
        const long long ts_ms = static_cast<long long>(llround(mxGetScalar(prhs[1])));

        const double* qd = mxGetPr(prhs[2]);
        const double* dqd = mxGetPr(prhs[3]);
        const double* taud = mxGetPr(prhs[4]);
        const double* accd = mxGetPr(prhs[5]);
        const double* gyrod = mxGetPr(prhs[6]);
        const double* quatd = mxGetPr(prhs[7]);

        LowlevelState st{};
        st.imu.timestamp = ts_ms;

        st.imu.accelerometer[0] = static_cast<float>(accd[0]);
        st.imu.accelerometer[1] = static_cast<float>(accd[1]);
        st.imu.accelerometer[2] = static_cast<float>(accd[2]);

        st.imu.gyroscope[0] = static_cast<float>(gyrod[0]);
        st.imu.gyroscope[1] = static_cast<float>(gyrod[1]);
        st.imu.gyroscope[2] = static_cast<float>(gyrod[2]);

        st.imu.quaternion[0] = static_cast<float>(quatd[0]);
        st.imu.quaternion[1] = static_cast<float>(quatd[1]);
        st.imu.quaternion[2] = static_cast<float>(quatd[2]);
        st.imu.quaternion[3] = static_cast<float>(quatd[3]);

        for (int i = 0; i < 16; ++i) {
            st.motorState[i].q = static_cast<float>(qd[i]);
            st.motorState[i].dq = static_cast<float>(dqd[i]);
            st.motorState[i].tauEst = static_cast<float>(taud[i]);
        }

        const Proprioception proprio = g_core->fusion_estimator(st);

        const char* fn[] = {"PositionXYZ", "OrientationRPY", "FootfallAverage", "FootLandedProbability", "DogWeight", "LegCollisionDetect", "JointsBodyWFPosition", "JointsBodyWFEffort","MotorGravityCompensate"};
        plhs[0] = mxCreateStructMatrix(1, 1, 9, fn);

        mxArray* position_xyz = mxCreateNumericMatrix(1, 9, mxSINGLE_CLASS, mxREAL);
        std::memcpy(mxGetData(position_xyz), proprio.PositionXYZ, sizeof(proprio.PositionXYZ));
        mxSetField(plhs[0], 0, "PositionXYZ", position_xyz);

        mxArray* orientation_rpy = mxCreateNumericMatrix(1, 9, mxSINGLE_CLASS, mxREAL);
        std::memcpy(mxGetData(orientation_rpy), proprio.OrientationRPY, sizeof(proprio.OrientationRPY));
        mxSetField(plhs[0], 0, "OrientationRPY", orientation_rpy);

        mxArray* footfall_average = mxCreateNumericMatrix(1, 3, mxSINGLE_CLASS, mxREAL);
        std::memcpy(mxGetData(footfall_average), proprio.FootfallAverage, sizeof(proprio.FootfallAverage));
        mxSetField(plhs[0], 0, "FootfallAverage", footfall_average);

        mxArray* foot_landed_probability = mxCreateNumericMatrix(1, 4, mxSINGLE_CLASS, mxREAL);
        std::memcpy(mxGetData(foot_landed_probability), proprio.FootLandedProbability, sizeof(proprio.FootLandedProbability));
        mxSetField(plhs[0], 0, "FootLandedProbability", foot_landed_probability);

        mxSetField(plhs[0], 0, "DogWeight", mxCreateDoubleScalar(static_cast<double>(proprio.DogWeight)));
        mxSetField(plhs[0], 0, "LegCollisionDetect", mxCreateDoubleScalar(static_cast<double>(proprio.LegCollisionDetect)));

        const mwSize dims[3] = {3, 4, 4};

        mxArray* joints_position = mxCreateNumericArray(3, dims, mxSINGLE_CLASS, mxREAL);
        std::memcpy(mxGetData(joints_position), proprio.JointsBodyWFPosition, sizeof(proprio.JointsBodyWFPosition));
        mxSetField(plhs[0], 0, "JointsBodyWFPosition", joints_position);

        mxArray* joints_effort = mxCreateNumericArray(3, dims, mxSINGLE_CLASS, mxREAL);
        std::memcpy(mxGetData(joints_effort), proprio.JointsBodyWFEffort, sizeof(proprio.JointsBodyWFEffort));
        mxSetField(plhs[0], 0, "JointsBodyWFEffort", joints_effort);

        mwSize gravity_dims[2] = {3, 4};
        mxArray* gravity_array = mxCreateNumericArray(2, gravity_dims, mxSINGLE_CLASS, mxREAL);
        std::memcpy(mxGetData(gravity_array), proprio.MotorGravityCompensate, sizeof(proprio.MotorGravityCompensate));
        mxSetField(plhs[0], 0, "MotorGravityCompensate", gravity_array);

        return;
    }

    if (nlhs > 0) plhs[0] = mxCreateDoubleMatrix(0, 0, mxREAL);
}