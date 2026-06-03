/*
Author：Sun Minxing
School: Institute of Optics And Electronics, Chinese Academy of Science
Email： 401435318@qq.com
*/
#pragma once

#include <fstream>
#include <memory>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <cstring>
#include "Estimators/EstimatorPortN.h"
#include "Estimators/matrix.h"

namespace DataFusion
{
  extern double Est_Quaternion[4];
  extern double Est_QuaternionInv[4];

  class Sensors
  {
    public:

      Sensors(EstimatorPortN* StateSpaceModel_)
      {        
        StateSpaceModel = StateSpaceModel_;

        for(int i = 0; i < StateSpaceModel->Nz; i++)
          StateSpaceModel->Matrix_R[i*StateSpaceModel->Nz+i] = R_diag[i];
      }

      virtual ~Sensors() = default;  
      virtual void SensorDataHandle(double* Message, double Time) {}
      double SensorPosition[3] = {0,0,0};
      double SensorQuaternion[4]    = {1,0,0,0};
      double SensorQuaternionInv[4] = {1,0,0,0};


    protected:

      EstimatorPortN* StateSpaceModel;
      double Est_QuaternionTemp1[4], Est_QuaternionTemp2[4], Est_QuaternionTemp3[4];
      double Est_BodyAngleVel[3], Est_SensorWorldPosition[3], Est_SensorWorldVelocity[3], Est_SensorPosition[3], Est_Vector3dTemp1[3], Est_Vector3dTemp2[3];

      double Observation[9] = {0};
      double ObservationTime = 0;
      double R_diag[9] = {1,1,1,1,1,1,1,1,1};

      void UpdateEst_Quaternion();
      void ObservationCorrect_Position();
      void ObservationCorrect_Velocity();
      void ObservationCorrect_Acceleration();
      void ObservationCorrect_Orientation();
      void ObservationCorrect_AngularVelocity();
      void ObservationCorrect_AngularAcceleration();

  };
}