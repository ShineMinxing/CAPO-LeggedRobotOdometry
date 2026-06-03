#include "Sensor_IMU.h"

namespace DataFusion
{
    void SensorIMUAcc::SensorDataHandle(double* Message, double Time) 
    {
      if(!IMUAccEnable)
        return;
      
      ObservationTime = Time;
      
      memcpy(Observation,  Message, (size_t)StateSpaceModel->Nz * sizeof(double));

      for(int i = 0; i < 3; i++)
      {
          StateSpaceModel->Matrix_H[(3 * i + 0) * StateSpaceModel->Nx + (3 * i + 0)] = 0;
          StateSpaceModel->Matrix_H[(3 * i + 1) * StateSpaceModel->Nx + (3 * i + 1)] = 0;
          StateSpaceModel->Matrix_H[(3 * i + 2) * StateSpaceModel->Nx + (3 * i + 2)] = 1;
      }
          
      ObservationCorrect_Acceleration();
      Observation[8] -= 9.43;
      
      StateSpaceModel_Go2_EstimatorPort(Observation, ObservationTime, StateSpaceModel);

    }

    void SensorIMUMagGyro::SensorDataHandle(double* Message, double Time)
    {      
      
      ObservationTime = Time;
      memcpy(Observation,  Message, (size_t)StateSpaceModel->Nz * sizeof(double));

      if((!IMUQuaternionEnable)&&(!IMUGyroEnable)){
        StateSpaceModel->EstimatedState[0] = Observation[0];
        StateSpaceModel->EstimatedState[3] = Observation[3];
        StateSpaceModel->EstimatedState[6] = Observation[6];
        UpdateEst_Quaternion();
      }

      for(int i = 0; i < 9; i++)
          StateSpaceModel->Matrix_H[i * StateSpaceModel->Nx + i] = 0;

      if(IMUQuaternionEnable){
        for(int i = 0; i < 3; i++)
            StateSpaceModel->Matrix_H[(3 * i + 0) * StateSpaceModel->Nx + (3 * i + 0)] = 1;
        ObservationCorrect_Orientation();
      }

      if(IMUGyroEnable){
        for(int i = 0; i < 3; i++)
            StateSpaceModel->Matrix_H[(3 * i + 1) * StateSpaceModel->Nx + (3 * i + 1)] = 1;
        ObservationCorrect_AngularVelocity();
      }
        
      static double ObservationAngleLast[3] = {0.0, 0.0, 0.0};
      static double ObservationAngleTurn[3] = {0.0, 0.0, 0.0};
      double ObservationAngle[3] = {
          Observation[0],
          Observation[3],
          Observation[6]
      };

      array_angle_unwrap(ObservationAngle, ObservationAngleLast, ObservationAngleTurn, 3);

      Observation[0] = ObservationAngle[0];
      Observation[3] = ObservationAngle[1];
      Observation[6] = ObservationAngle[2];

      StateSpaceModel_Go2_EstimatorPort(Observation, ObservationTime, StateSpaceModel);

      UpdateEst_Quaternion();
    }
}