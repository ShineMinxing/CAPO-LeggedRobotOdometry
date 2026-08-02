/*
% ===========================================================================
% Kalman Estimator ID 1001
% Filename: Estimator1001_Kalman.c  Estimator1001_Kalman.h
% Description:
% This function estimator completes linear estimation according to the structure provided in EstimatorPortN.h and the specific state-space model provided in StateSpaceModelN.c.
%
% Initial version: Sun Minxing
% Unit: UCAS, Institute of Optics And Electronics, 2020 Class
% Email: 401435318@qq.com
% Date: November 16, 2024
% 
% Update:
% Unit:
% Email:
% Date:
% ===========================================================================
*/
#ifndef _Estimator1001_H_
#define _Estimator1001_H_


#include "matrix.h"
#include "EstimatorPortN.h"

void Estimator1001_Init(EstimatorPortN *estimator);
void Estimator1001_Input(EstimatorPortN *estimator);
void Estimator1001_Estimation(EstimatorPortN *estimator);
void Estimator1001_Output(EstimatorPortN *estimator);
void Estimator1001_Termination();

extern MATRIX *E1001_F, *E1001_FT, *E1001_G, *E1001_H, *E1001_HT, *E1001_Q, *E1001_R, *E1001_Xe, *E1001_Xp, *E1001_Z, *E1001_Ze, *E1001_P, *E1001_P_pre, *E1001_K;
extern MATRIX *E1001_XX1, *E1001_XX2, *E1001_ZX1, *E1001_XZ1, *E1001_ZZ1, *E1001_ZZ2, *E1001_X11, *E1001_Z11, *E1001_Z12, *E1001_eyeX, *E1001_L, *E1001_U, *E1001_y;

extern INTEGER E1001_reference_count;

#endif
