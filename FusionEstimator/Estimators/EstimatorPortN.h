/*
% ===========================================================================
% Filename: EstimatorPortN.h
% Description:
%
% This is an estimator port to call different estimators. It's compatible to .c estimators and .cpp estiamtors.
% 
% After put those estimators into Estimator\C_Estimators, Estimator\Cpp_Estimators, 
% and add them into compile scope, you can easily call them.
% 
% You can see "EXPORT extern EstimatorPortN StateSpaceModel_Demo_;" below. 
% Of course you can:
% 1. Define more StateSpaceModel_Go2_ StateSpaceModel3_... in this h file.
% 2. Copy more Estimator\StateSpaceModel_Demo.c and name them with StateSpaceModel_Go2.cpp...
% 3. Replace all "StateSpaceModel_Demo_" to "StateSpaceModel_Go2_" in "StateSpaceModel_Go2.cpp", and you can define more 
%     linear or nonlinear state space model.
% 4. If you want to use different estimamtor to realize a StateSpaceModelN, replace all 
%     "Estimator1001_" to "EstimatorXXXX_", which is defined in your EstimatorXXXX_XXXXX.c/EstimatorXXXX_XXXXX.cpp.
% 
% This EstimatorPortN.h can be used in 6 method:
% 1. Normally include in C project like C_Demo.c.
% 2. Normally include in Cpp project like Cpp_Demo.cpp.
% 3. Generate a dll file in advance (Estimator\EstimatorPortN.dll Estimator\EstimatorPortN.lib) and call by C project like C_DLL_Demo.c.
% 4. Generate a dll file in advance (~) and call by Cpp project like Cpp_DLL_Demo.cpp.
% 5. Generate a dll file in advance (~) and call by Matlab project like M_DLL_Demo.m.
% 6. Generate a dll file in advance (~) and call by Python project like Python_DLL_Demo.py.
%
% Source files in Estimator\C_Estimators, Estimator\Cpp_Estimators and Estimator\StateSpaceModel_Demo.c are used. 
% Make sure there is at least one estimator method in the subdirectory, like 
% Estimator1001_Kalman.c and Estimator1001_Kalman.h.
%
% Mainly use .vscode\tasks.json and ..vscode\build_dll.bat to compile
%
% Initial version: Minxing Sun
% Unit: UCAS, Institute of Optics And Electronics, Lab 1, Class of 2020
% Email: 401435318@qq.com
% Date: November 16, 2024
% 
% Updates:
% Unit:
% Email:
% Date:
% ===========================================================================
*/

#ifndef ESTIMATOR_PORTN_H
#define ESTIMATOR_PORTN_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#ifdef _WIN32
    #define EXPORT __declspec(dllexport)
#else
    #define EXPORT
#endif

#ifdef __cplusplus
extern "C" {
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


typedef struct EstimatorPortN {

    char *PortName;
    char *PortIntroduction;

    int Nx;
    int Nz;
    int PredictStep;
    double Intervel;
    double PredictTime;
    double CurrentTimestamp;
    double StateUpdateTimestamp;
    double ObservationTimestamp;

    // Matrix
    // xi+1 = F*xi + G*B + w
    // yi = H*xi + v
    double *EstimatedState;                    
    double *PredictedState;
    double *CurrentObservation;
    double *PredictedObservation;
    double *Matrix_F;
    double *Matrix_G;
    double *Matrix_B;
    double *Matrix_H;
    double *Matrix_P;  // InitialProbability
    double *Matrix_Q;  // ProcessNoiseCovariance
    double *Matrix_R;  // ObserveNoiseCovariance
    int    *Int_Par;
    double *Double_Par;

    int Nx99;
    int Nz99;
    double *EstimatedState99;
    double *CurrentObservation99;
    double *Matrix_H99;
    double *Matrix_P99;

    void (*StateTransitionEquation)(double *In_State, double *Out_State, struct EstimatorPortN *estimator);
    void (*ObservationEquation)(double *In_State, double *Out_Observation, struct EstimatorPortN *estimator);
    void (*PredictionEquation)(double *In_State, double *Out_PredictedState, struct EstimatorPortN *estimator);

} EstimatorPortN;

// If you have a new state space model, copy following 4 lines, and substitute StateSpaceModel_Demo_ to StateSpaceModelN_
// And then, copy StateSpaceModel_Demo.c, and substitute all StateSpaceModel_Demo_ in it to StateSpaceModelN_
EXPORT extern EstimatorPortN StateSpaceModel_Go2_;
EXPORT void StateSpaceModel_Go2_Initialization(struct EstimatorPortN *estimator);
EXPORT void StateSpaceModel_Go2_EstimatorPort(double *In_Observation, double In_Observation_Timestamp, struct EstimatorPortN *estimator);
EXPORT void StateSpaceModel_Go2_EstimatorPortTermination(struct EstimatorPortN *estimator);

#ifdef __cplusplus
}
#endif

#endif // ESTIMATOR_PORTN_H