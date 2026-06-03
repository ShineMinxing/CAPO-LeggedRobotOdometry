#pragma once

#include "SensorBase.h"

namespace DataFusion
{
  class SensorIMUAcc : public Sensors
  {
    public:

    SensorIMUAcc(EstimatorPortN* StateSpaceModel_):Sensors(StateSpaceModel_)
    {
      double AngleCorrect[3] = {0};
      AngleCorrect[0] = 0 * M_PI / 180.0;
      AngleCorrect[1] = 1.7 * M_PI / 180.0;
      AngleCorrect[2] = 0 * M_PI / 180.0;
      array_eulerZYX_to_quaternion(AngleCorrect, SensorQuaternion);
      array_quaternion_normalize(SensorQuaternion, SensorQuaternion);
      array_quaternion_conjugate(SensorQuaternion, SensorQuaternionInv);

      SensorPosition[0] = 0;
      SensorPosition[1] = 0;
      SensorPosition[2] = 0;
    }

    bool IMUAccEnable = true;

    void SensorDataHandle(double* Message, double Time)  override;

    protected:

  };

  class SensorIMUMagGyro : public Sensors
  {
    public:

    SensorIMUMagGyro(EstimatorPortN* StateSpaceModel_):Sensors(StateSpaceModel_)
    {
      double AngleCorrect[3] = {0};
      AngleCorrect[0] = 0 * M_PI / 180.0;
      AngleCorrect[1] = 1.7 * M_PI / 180.0;
      AngleCorrect[2] = 0 * M_PI / 180.0;
      array_eulerZYX_to_quaternion(AngleCorrect, SensorQuaternion);
      array_quaternion_normalize(SensorQuaternion, SensorQuaternion);
      array_quaternion_conjugate(SensorQuaternion, SensorQuaternionInv);

      SensorPosition[0] = 0;
      SensorPosition[1] = 0;
      SensorPosition[2] = 0;
    }

    bool IMUQuaternionEnable = true;
    bool IMUGyroEnable = true;

    void SensorDataHandle(double* Message, double Time) override;

    protected:

  };
}