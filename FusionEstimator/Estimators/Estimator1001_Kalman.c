#include "Estimator1001_Kalman.h"

MATRIX *E1001_F, *E1001_FT, *E1001_G, *E1001_H, *E1001_HT, *E1001_Q, *E1001_R, *E1001_Xe, *E1001_Xp, *E1001_Z, *E1001_Ze, *E1001_P, *E1001_P_pre, *E1001_K;
MATRIX *E1001_XX1, *E1001_XX2, *E1001_ZX1, *E1001_XZ1, *E1001_ZZ1, *E1001_ZZ2, *E1001_X11, *E1001_Z11, *E1001_Z12, *E1001_eyeX, *E1001_L, *E1001_U, *E1001_y;
INTEGER E1001_reference_count = 0;

void Estimator1001_Init(EstimatorPortN *estimator)
{
    if (E1001_reference_count > 0)
    {
        E1001_reference_count++;
        return;
    }

    init_stack(&S);

    // Model parameter initialization
    E1001_F  = creat_zero_matrix(estimator->Nx, estimator->Nx, &errorID, &S);
    E1001_FT = creat_zero_matrix(estimator->Nx, estimator->Nx, &errorID, &S);
    E1001_G  = creat_zero_matrix(estimator->Nx, estimator->Nx, &errorID, &S);
    E1001_H  = creat_zero_matrix(estimator->Nz, estimator->Nx, &errorID, &S);
    E1001_HT = creat_zero_matrix(estimator->Nx, estimator->Nz, &errorID, &S);
    E1001_Q  = creat_zero_matrix(estimator->Nx, estimator->Nx, &errorID, &S);
    E1001_R  = creat_zero_matrix(estimator->Nz, estimator->Nz, &errorID, &S);
    E1001_Xe = creat_zero_matrix(estimator->Nx, 1, &errorID, &S);
    E1001_Xp = creat_zero_matrix(estimator->Nx, 1, &errorID, &S);
    E1001_Z  = creat_zero_matrix(estimator->Nz, 1, &errorID, &S);
    E1001_Ze = creat_zero_matrix(estimator->Nz, 1, &errorID, &S);
    E1001_K  = creat_zero_matrix(estimator->Nx, estimator->Nz, &errorID, &S);
    E1001_P  = creat_zero_matrix(estimator->Nx, estimator->Nx, &errorID, &S);
    E1001_P_pre  = creat_zero_matrix(estimator->Nx, estimator->Nx, &errorID, &S);
    E1001_L = creat_zero_matrix(estimator->Nz, estimator->Nz, &errorID, &S);
    E1001_U = creat_zero_matrix(estimator->Nz, estimator->Nz, &errorID, &S);
    E1001_y = creat_zero_matrix(estimator->Nz, estimator->Nz, &errorID, &S);
    
    memcpy(E1001_F->p, estimator->Matrix_F, (size_t)estimator->Nx * (size_t)estimator->Nx * sizeof(double));
    matrix_transpose(E1001_F, E1001_FT);
    memcpy(E1001_G->p, estimator->Matrix_G, (size_t)estimator->Nx * (size_t)estimator->Nx * sizeof(double));
    memcpy(E1001_Q->p, estimator->Matrix_Q, (size_t)estimator->Nx * (size_t)estimator->Nx * sizeof(double));
    memcpy(E1001_R->p, estimator->Matrix_R, (size_t)estimator->Nz * (size_t)estimator->Nz * sizeof(double));

    // Intermediate variables
    E1001_XX1 = creat_zero_matrix(estimator->Nx, estimator->Nx, &errorID, &S);
    E1001_XX2 = creat_zero_matrix(estimator->Nx, estimator->Nx, &errorID, &S);
    E1001_ZX1 = creat_zero_matrix(estimator->Nz, estimator->Nx, &errorID, &S);
    E1001_XZ1 = creat_zero_matrix(estimator->Nx, estimator->Nz, &errorID, &S);
    E1001_ZZ1 = creat_zero_matrix(estimator->Nz, estimator->Nz, &errorID, &S);
    E1001_ZZ2 = creat_zero_matrix(estimator->Nz, estimator->Nz, &errorID, &S);
    E1001_X11 = creat_zero_matrix(estimator->Nx, 1, &errorID, &S);
    E1001_Z11 = creat_zero_matrix(estimator->Nz, 1, &errorID, &S);
    E1001_Z12 = creat_zero_matrix(estimator->Nz, 1, &errorID, &S);
    E1001_eyeX = creat_eye_matrix(estimator->Nx, &errorID, &S);

    E1001_reference_count = 1;

    printf("Linear Kalman estimator is initialized\n");
}

void Estimator1001_Input(EstimatorPortN *estimator)
{
    memcpy(E1001_Z->p,  estimator->CurrentObservation, (size_t)estimator->Nz * sizeof(double));
    memcpy(E1001_Xe->p, estimator->EstimatedState,     (size_t)estimator->Nx * sizeof(double));
    memcpy(E1001_P_pre->p, estimator->Matrix_P, (size_t)estimator->Nx * estimator->Nx * sizeof(double));
    memcpy(E1001_H->p, estimator->Matrix_H, (size_t)estimator->Nz * estimator->Nx * sizeof(double));
}

void Estimator1001_Estimation(EstimatorPortN *estimator)
{
    Estimator1001_Input(estimator);

    if(fabs(estimator->StateUpdateTimestamp - estimator->ObservationTimestamp) > 0.1)
      estimator->StateUpdateTimestamp = estimator->ObservationTimestamp;
    else if(estimator->StateUpdateTimestamp != estimator->ObservationTimestamp)
    {
      // State prediction E1001_Xe
      matrix_multiplication(E1001_F, E1001_Xe, E1001_X11);
      matrix_valuation(E1001_X11, E1001_Xe, 0, 0);

      // Covariance prediction E1001_P_pre
      matrix_multiplication(E1001_F, E1001_P_pre, E1001_XX1);
      matrix_multiplication(E1001_XX1, E1001_FT, E1001_XX2);
      matrix_add(E1001_XX2, E1001_Q, E1001_P_pre);
    }

    // Compute Kalman gain E1001_K
    matrix_transpose(E1001_H, E1001_HT);
    matrix_multiplication(E1001_H, E1001_P_pre, E1001_ZX1);
    matrix_multiplication(E1001_ZX1, E1001_HT, E1001_ZZ1);
    matrix_add(E1001_ZZ1, E1001_R, E1001_ZZ1);
    matrix_inverse_noalloc(E1001_ZZ1, E1001_ZZ2, E1001_L, E1001_U, E1001_y);
    matrix_multiplication(E1001_P_pre, E1001_HT, E1001_XZ1);
    matrix_multiplication(E1001_XZ1, E1001_ZZ2, E1001_K);

    // State update E1001_Xe
    matrix_multiplication(E1001_H, E1001_Xe, E1001_Z11);
    matrix_subtraction(E1001_Z, E1001_Z11, E1001_Z12);
    matrix_multiplication(E1001_K, E1001_Z12, E1001_X11);
    matrix_add(E1001_Xe, E1001_X11, E1001_Xe);

    // Covariance update E1001_P
    matrix_multiplication(E1001_K, E1001_ZX1, E1001_XX1);
    matrix_subtraction(E1001_P_pre, E1001_XX1, E1001_P); 
    
    Estimator1001_Output(estimator);
}

void Estimator1001_Output(EstimatorPortN *estimator)
{
    memcpy(estimator->EstimatedState, E1001_Xe->p, (size_t)estimator->Nx * sizeof(double));
    memcpy(estimator->Matrix_P, E1001_P->p, (size_t)estimator->Nx * estimator->Nx * sizeof(double));
    estimator->StateUpdateTimestamp = estimator->ObservationTimestamp;
}

void Estimator1001_Termination()
{
    if (E1001_reference_count <= 0)
    {
        return;
    }

    E1001_reference_count--;

    if (E1001_reference_count == 0)
    {
        free_stack(&S);
    }
}
