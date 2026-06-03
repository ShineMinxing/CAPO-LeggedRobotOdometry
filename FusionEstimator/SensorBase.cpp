#include "SensorBase.h"

namespace DataFusion
{
    double Est_Quaternion[4] = {1,0,0,0};
    double Est_QuaternionInv[4] = {1,0,0,0};

    void Sensors::UpdateEst_Quaternion()
    {
        double array_EulerZYX[3] = {
            StateSpaceModel->EstimatedState[0],
            StateSpaceModel->EstimatedState[3],
            StateSpaceModel->EstimatedState[6]
        };

        array_eulerZYX_to_quaternion(array_EulerZYX, Est_Quaternion);
        array_quaternion_conjugate(Est_Quaternion, Est_QuaternionInv);
    }

    void Sensors::ObservationCorrect_Position()
    {
        Est_Vector3dTemp1[0] = Observation[0];
        Est_Vector3dTemp1[1] = Observation[3];
        Est_Vector3dTemp1[2] = Observation[6];

        array_quaternion_rotate_vector(SensorQuaternion, Est_Vector3dTemp1, Est_Vector3dTemp1);
        array_quaternion_rotate_vector(Est_Quaternion, Est_Vector3dTemp1, Est_Vector3dTemp1);

        Est_Vector3dTemp2[0] = SensorPosition[0];
        Est_Vector3dTemp2[1] = SensorPosition[1];
        Est_Vector3dTemp2[2] = SensorPosition[2];

        array_quaternion_rotate_vector(Est_Quaternion, Est_Vector3dTemp2, Est_Vector3dTemp2);

        Observation[0] = Est_Vector3dTemp1[0] + Est_Vector3dTemp2[0];
        Observation[3] = Est_Vector3dTemp1[1] + Est_Vector3dTemp2[1];
        Observation[6] = Est_Vector3dTemp1[2] + Est_Vector3dTemp2[2];
    }

    void Sensors::ObservationCorrect_Velocity()
    {
        Est_Vector3dTemp1[0] = Observation[1];
        Est_Vector3dTemp1[1] = Observation[4];
        Est_Vector3dTemp1[2] = Observation[7];

        array_quaternion_rotate_vector(SensorQuaternion, Est_Vector3dTemp1, Est_Vector3dTemp1);
        array_quaternion_rotate_vector(Est_Quaternion, Est_Vector3dTemp1, Est_Vector3dTemp1);

        Est_SensorWorldPosition[0] = SensorPosition[0];
        Est_SensorWorldPosition[1] = SensorPosition[1];
        Est_SensorWorldPosition[2] = SensorPosition[2];

        array_quaternion_rotate_vector(Est_Quaternion, Est_SensorWorldPosition, Est_SensorWorldPosition);

        Est_BodyAngleVel[0] = Observation[1];
        Est_BodyAngleVel[1] = Observation[4];
        Est_BodyAngleVel[2] = Observation[7];

        array_vector_cross(Est_BodyAngleVel, Est_SensorWorldPosition, Est_SensorWorldVelocity);

        Est_Vector3dTemp1[0] -= Est_SensorWorldVelocity[0];
        Est_Vector3dTemp1[1] -= Est_SensorWorldVelocity[1];
        Est_Vector3dTemp1[2] -= Est_SensorWorldVelocity[2];

        Observation[1] = -Est_Vector3dTemp1[0];
        Observation[4] = -Est_Vector3dTemp1[1];
        Observation[7] = -Est_Vector3dTemp1[2];
    }

    void Sensors::ObservationCorrect_Acceleration()
    {
        Est_Vector3dTemp1[0] = Observation[2];
        Est_Vector3dTemp1[1] = Observation[5];
        Est_Vector3dTemp1[2] = Observation[8];

        array_quaternion_rotate_vector(SensorQuaternion, Est_Vector3dTemp1, Est_Vector3dTemp1);
        array_quaternion_rotate_vector(Est_Quaternion, Est_Vector3dTemp1, Est_Vector3dTemp1);

        Est_SensorPosition[0] = SensorPosition[0];
        Est_SensorPosition[1] = SensorPosition[1];
        Est_SensorPosition[2] = SensorPosition[2];

        Est_Vector3dTemp2[0] = Observation[1];
        Est_Vector3dTemp2[1] = Observation[4];
        Est_Vector3dTemp2[2] = Observation[7];

        array_vector_cross(Est_Vector3dTemp2, Est_SensorPosition, Est_BodyAngleVel);
        array_vector_cross(Est_Vector3dTemp2, Est_BodyAngleVel, Est_BodyAngleVel);

        Est_Vector3dTemp1[0] -= Est_BodyAngleVel[0];
        Est_Vector3dTemp1[1] -= Est_BodyAngleVel[1];
        Est_Vector3dTemp1[2] -= Est_BodyAngleVel[2];

        Observation[2] = Est_Vector3dTemp1[0];
        Observation[5] = Est_Vector3dTemp1[1];
        Observation[8] = Est_Vector3dTemp1[2];
    }

    void Sensors::ObservationCorrect_Orientation()
    {
        double array_EulerZYX[3] = {
            Observation[0],
            Observation[3],
            Observation[6]
        };

        array_eulerZYX_to_quaternion(array_EulerZYX, Est_QuaternionTemp1);
        array_quaternion_multiplication(Est_QuaternionTemp1, SensorQuaternionInv, Est_QuaternionTemp2);
        array_quaternion_normalize(Est_QuaternionTemp2, Est_QuaternionTemp2);
        array_quaternion_to_eulerZYX(Est_QuaternionTemp2, Est_Vector3dTemp1);

        Observation[0] = Est_Vector3dTemp1[0];
        Observation[3] = Est_Vector3dTemp1[1];
        Observation[6] = Est_Vector3dTemp1[2];
    }

    void Sensors::ObservationCorrect_AngularVelocity()
    {
        Est_Vector3dTemp1[0] = Observation[1];
        Est_Vector3dTemp1[1] = Observation[4];
        Est_Vector3dTemp1[2] = Observation[7];

        array_quaternion_rotate_vector(SensorQuaternion, Est_Vector3dTemp1, Est_Vector3dTemp1);
        array_quaternion_rotate_vector(Est_Quaternion, Est_Vector3dTemp1, Est_Vector3dTemp1);

        Observation[1] = Est_Vector3dTemp1[0];
        Observation[4] = Est_Vector3dTemp1[1];
        Observation[7] = Est_Vector3dTemp1[2];
    }

    void Sensors::ObservationCorrect_AngularAcceleration()
    {
        Est_Vector3dTemp1[0] = Observation[2];
        Est_Vector3dTemp1[1] = Observation[5];
        Est_Vector3dTemp1[2] = Observation[8];

        array_quaternion_rotate_vector(SensorQuaternion, Est_Vector3dTemp1, Est_Vector3dTemp1);
        array_quaternion_rotate_vector(Est_Quaternion, Est_Vector3dTemp1, Est_Vector3dTemp1);

        Observation[2] = Est_Vector3dTemp1[0];
        Observation[5] = Est_Vector3dTemp1[1];
        Observation[8] = Est_Vector3dTemp1[2];
    }
}