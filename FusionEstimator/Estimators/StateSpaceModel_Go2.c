// used in go2 robot

#include "EstimatorPortN.h"
#include "Estimator1001_Kalman.h"

EstimatorPortN StateSpaceModel_Go2_;

void StateSpaceModel_Go2_StateTransitionFunction(double *In_State, double *Out_State, struct EstimatorPortN *estimator)
{
    MATRIX F;
    F.rows = estimator->Nx;
    F.columns = estimator->Nx;
    F.p = estimator->Matrix_F;

    MATRIX In;
    In.rows = estimator->Nx;
    In.columns = 1;
    In.p = In_State;

    MATRIX Out;
    Out.rows = estimator->Nx;
    Out.columns = 1;
    Out.p = Out_State;

    matrix_multiplication(&F, &In, &Out);
}

void StateSpaceModel_Go2_ObservationFunction(double *In_State, double *Out_Observation, EstimatorPortN *estimator)
{
    MATRIX H;
    H.rows = estimator->Nz;
    H.columns = estimator->Nx;
    H.p = estimator->Matrix_H;

    MATRIX In;
    In.rows = estimator->Nx;
    In.columns = 1;
    In.p = In_State;

    MATRIX Out;
    Out.rows = estimator->Nz;
    Out.columns = 1;
    Out.p = Out_Observation;

    matrix_multiplication(&H, &In, &Out);
}

void StateSpaceModel_Go2_PredictionFunction(double *In_State, double *Out_PredictedState, EstimatorPortN *estimator)
{
    MATRIX F;
    F.rows = estimator->Nx;
    F.columns = estimator->Nx;
    F.p = estimator->Matrix_F;

    MATRIX In;
    In.rows = estimator->Nx;
    In.columns = 1;
    In.p = In_State;

    MATRIX Out;
    Out.rows = estimator->Nx;
    Out.columns = 1;
    Out.p = Out_PredictedState;

    matrix_multiplication(&F, &In, &Out);
}

EXPORT void StateSpaceModel_Go2_EstimatorPort(double *In_Observation, double In_Observation_Timestamp, struct EstimatorPortN *estimator)
{
    double StateUpdateTimestamp = estimator->StateUpdateTimestamp;

    estimator->ObservationTimestamp = In_Observation_Timestamp;
    memcpy(estimator->CurrentObservation99, In_Observation, estimator->Nz99 * sizeof(double));

    for (int i = 0; i < estimator->Nx99 / estimator->Nx; i++)
    {
        memcpy(estimator->EstimatedState, estimator->EstimatedState99 + i * estimator->Nx, estimator->Nx * sizeof(double));
        memcpy(estimator->CurrentObservation, estimator->CurrentObservation99 + i * estimator->Nz, estimator->Nz * sizeof(double));

        for (int j = 0; j < estimator->Nz; j++)
        {
            memcpy(estimator->Matrix_H + j * estimator->Nx, estimator->Matrix_H99 + (i * estimator->Nz + j) * estimator->Nx99 + i * estimator->Nx, estimator->Nx * sizeof(double));
        }

        for (int j = 0; j < estimator->Nx; j++)
        {
            memcpy(estimator->Matrix_P + j * estimator->Nx, estimator->Matrix_P99 + (i * estimator->Nx + j) * estimator->Nx99 + i * estimator->Nx, estimator->Nx * sizeof(double));
        }

        estimator->StateUpdateTimestamp = StateUpdateTimestamp;
        Estimator1001_Estimation(estimator);

        memcpy(estimator->EstimatedState99 + i * estimator->Nx, estimator->EstimatedState, estimator->Nx * sizeof(double));

        for (int j = 0; j < estimator->Nx; j++)
        {
            memcpy(estimator->Matrix_P99 + (i * estimator->Nx + j) * estimator->Nx99 + i * estimator->Nx, estimator->Matrix_P + j * estimator->Nx, estimator->Nx * sizeof(double));
        }
    }

    estimator->StateUpdateTimestamp = estimator->ObservationTimestamp;
}

EXPORT void StateSpaceModel_Go2_EstimatorPortTermination(struct EstimatorPortN *estimator) {
    Estimator1001_Termination();

    free(estimator->PortName);
    estimator->PortName = NULL;
    free(estimator->PortIntroduction);
    estimator->PortIntroduction = NULL;
    free(estimator->EstimatedState);
    estimator->EstimatedState = NULL;
    free(estimator->PredictedState);
    estimator->PredictedState = NULL;
    free(estimator->CurrentObservation);
    estimator->CurrentObservation = NULL;
    free(estimator->PredictedObservation);
    estimator->PredictedObservation = NULL;
    free(estimator->Matrix_F);
    estimator->Matrix_F = NULL;
    free(estimator->Matrix_G);
    estimator->Matrix_G = NULL;
    free(estimator->Matrix_B);
    estimator->Matrix_B = NULL;
    free(estimator->Matrix_H);
    estimator->Matrix_H = NULL;
    free(estimator->Matrix_P);
    estimator->Matrix_P = NULL;
    free(estimator->Matrix_Q);
    estimator->Matrix_Q = NULL;
    free(estimator->Matrix_R);
    estimator->Matrix_R = NULL;
    free(estimator->Int_Par);
    estimator->Int_Par = NULL;
    free(estimator->Double_Par);
    estimator->Double_Par = NULL;
    
    free(estimator->EstimatedState99);
    estimator->EstimatedState99 = NULL;
    free(estimator->CurrentObservation99);
    estimator->CurrentObservation99 = NULL;
    free(estimator->Matrix_H99);
    estimator->Matrix_H99 = NULL;
    free(estimator->Matrix_P99);
    estimator->Matrix_P99 = NULL;

    printf("EstimatorPort terminated.\n");
}


EXPORT void StateSpaceModel_Go2_Initialization(EstimatorPortN *estimator) 
{
    char *PortNameTemp = "Tested Estimator v0.00";
    char *PortIntroductionTemp = "For Reference";

    #define StateSpaceModel_Go2_NX 3
    #define StateSpaceModel_Go2_NZ 3
    #define StateSpaceModel_Go2_NX99 9
    #define StateSpaceModel_Go2_NZ99 9
    #define StateSpaceModel_Go2_PredictStep 0
    #define StateSpaceModel_Go2_Interval 0.005
    #define StateSpaceModel_Go2_PredictTime 0
    #define StateSpaceModel_Go2_ObservationTimestamp 0

    double X0[StateSpaceModel_Go2_NX] = {
        0,0,0
    };

    double F[StateSpaceModel_Go2_NX * StateSpaceModel_Go2_NX] = {
        1,StateSpaceModel_Go2_Interval,StateSpaceModel_Go2_Interval * StateSpaceModel_Go2_Interval / 2,
        0,1,StateSpaceModel_Go2_Interval,
        0,0,1
    };

    double G[StateSpaceModel_Go2_NX * StateSpaceModel_Go2_NX] = {
        1,StateSpaceModel_Go2_Interval,StateSpaceModel_Go2_Interval * StateSpaceModel_Go2_Interval / 2,
        0,1,StateSpaceModel_Go2_Interval,
        0,0,1
    };

    double B[StateSpaceModel_Go2_NX] = {
        0,0,0
    };

    double H[StateSpaceModel_Go2_NZ * StateSpaceModel_Go2_NX] = {
        0,0,0,
        0,0,0,
        0,0,0
    };

    double P[StateSpaceModel_Go2_NX * StateSpaceModel_Go2_NX] = {
        1,0,0,
        0,1,0,
        0,0,1
    };

    double Q[StateSpaceModel_Go2_NX * StateSpaceModel_Go2_NX] = {
        1,0,0,
        0,1,0,
        0,0,1
    };

    double R[StateSpaceModel_Go2_NZ * StateSpaceModel_Go2_NZ] = {
        1,0,0,
        0,1,0,
        0,0,1
    };
    int Int_Par[100] = {0};
    double Double_Par[100] = {0};

    estimator->PortName = (char *)malloc(100);
    estimator->PortIntroduction = (char *)malloc(1000);

    estimator->Nx = StateSpaceModel_Go2_NX;
    estimator->Nz = StateSpaceModel_Go2_NZ;
    estimator->PredictStep = StateSpaceModel_Go2_PredictStep;
    estimator->Intervel = StateSpaceModel_Go2_Interval;
    estimator->PredictTime = StateSpaceModel_Go2_PredictTime;
    estimator->CurrentTimestamp = StateSpaceModel_Go2_ObservationTimestamp;
    estimator->StateUpdateTimestamp = StateSpaceModel_Go2_ObservationTimestamp;
    estimator->ObservationTimestamp = StateSpaceModel_Go2_ObservationTimestamp;

    estimator->EstimatedState = (double *)malloc(estimator->Nx * sizeof(double));
    estimator->PredictedState = (double *)malloc(estimator->Nx * sizeof(double));
    estimator->CurrentObservation = (double *)malloc(estimator->Nz * sizeof(double));
    estimator->PredictedObservation = (double *)malloc(estimator->Nz * sizeof(double));
    estimator->Matrix_F = (double *)malloc(estimator->Nx * estimator->Nx * sizeof(double));
    estimator->Matrix_G = (double *)malloc(estimator->Nx * estimator->Nx * sizeof(double));
    estimator->Matrix_B = (double *)malloc(estimator->Nx * sizeof(double));
    estimator->Matrix_H = (double *)malloc(estimator->Nz * estimator->Nx * sizeof(double));
    estimator->Matrix_P = (double *)malloc(estimator->Nx * estimator->Nx * sizeof(double));
    estimator->Matrix_Q = (double *)malloc(estimator->Nx * estimator->Nx * sizeof(double));
    estimator->Matrix_R = (double *)malloc(estimator->Nz * estimator->Nz * sizeof(double));
    estimator->Int_Par  = (int *)malloc(100 * sizeof(int));
    estimator->Double_Par = (double *)malloc(100 * sizeof(double));
    if (estimator->PortName == NULL || estimator->PortIntroduction == NULL ||\
        estimator->EstimatedState == NULL || estimator->PredictedState == NULL ||\
        estimator->CurrentObservation == NULL || estimator->PredictedObservation == NULL ||\
        estimator->Matrix_F == NULL || estimator->Matrix_G == NULL ||\
        estimator->Matrix_B == NULL || estimator->Matrix_H == NULL ||\
        estimator->Matrix_P == NULL || estimator->Matrix_Q == NULL ||\
        estimator->Matrix_R == NULL || estimator->Int_Par  == NULL||\
        estimator->Double_Par == NULL) 
        {
            perror("Memory allocation failed");
            exit(EXIT_FAILURE);
        }
    
    strcpy(estimator->PortName, PortNameTemp);
    strcpy(estimator->PortIntroduction, PortIntroductionTemp);
    memcpy(estimator->EstimatedState, X0, estimator->Nx * sizeof(double));
    memcpy(estimator->PredictedState, X0, estimator->Nx * sizeof(double));
    memcpy(estimator->Matrix_F, F, estimator->Nx * estimator->Nx * sizeof(double));
    memcpy(estimator->Matrix_G, G, estimator->Nx * estimator->Nx * sizeof(double));
    memcpy(estimator->Matrix_B, B, estimator->Nx * sizeof(double));
    memcpy(estimator->Matrix_H, H, estimator->Nz * estimator->Nx * sizeof(double));
    memcpy(estimator->Matrix_P, P, estimator->Nx * estimator->Nx * sizeof(double));
    memcpy(estimator->Matrix_Q, Q, estimator->Nx * estimator->Nx * sizeof(double));
    memcpy(estimator->Matrix_R, R, estimator->Nz * estimator->Nz * sizeof(double));
    memcpy(estimator->Int_Par, Int_Par, 100 * sizeof(int));
    memcpy(estimator->Double_Par, Double_Par, 100 * sizeof(double));

    
    estimator->Nx99 = StateSpaceModel_Go2_NX99;
    estimator->Nz99 = StateSpaceModel_Go2_NZ99;
    estimator->EstimatedState99 = (double *)malloc(estimator->Nx99 * sizeof(double));
    estimator->CurrentObservation99 = (double *)malloc(estimator->Nz99 * sizeof(double));
    estimator->Matrix_H99 = (double *)malloc(estimator->Nz99 * estimator->Nx99 * sizeof(double));
    estimator->Matrix_P99 = (double *)malloc(estimator->Nx99 * estimator->Nx99 * sizeof(double));
    if (estimator->EstimatedState99 == NULL ||\
        estimator->CurrentObservation99 == NULL || estimator->Matrix_H99 == NULL ||\
        estimator->Matrix_P99 == NULL)
    {
        perror("Memory allocation failed");
        exit(EXIT_FAILURE);
    }
    memset(estimator->EstimatedState99, 0, estimator->Nx99 * sizeof(double));
    memset(estimator->CurrentObservation99, 0, estimator->Nz99 * sizeof(double));
    memset(estimator->Matrix_H99, 0, estimator->Nz99 * estimator->Nx99 * sizeof(double));
    memset(estimator->Matrix_P99, 0, estimator->Nx99 * estimator->Nx99 * sizeof(double));
    for (int i = 0; i < estimator->Nx99 / estimator->Nx; i++)
    {
        memcpy(estimator->EstimatedState99 + i * estimator->Nx, estimator->EstimatedState, estimator->Nx * sizeof(double));

        for (int j = 0; j < estimator->Nz; j++)
        {
            memcpy(estimator->Matrix_H99 + (i * estimator->Nz + j) * estimator->Nx99 + i * estimator->Nx, estimator->Matrix_H + j * estimator->Nx, estimator->Nx * sizeof(double));
        }

        for (int j = 0; j < estimator->Nx; j++)
        {
            memcpy(estimator->Matrix_P99 + (i * estimator->Nx + j) * estimator->Nx99 + i * estimator->Nx, estimator->Matrix_P + j * estimator->Nx, estimator->Nx * sizeof(double));
        }
    }

    // Initiate pointer
    estimator->StateTransitionEquation = StateSpaceModel_Go2_StateTransitionFunction;
    estimator->ObservationEquation = StateSpaceModel_Go2_ObservationFunction;
    estimator->PredictionEquation = StateSpaceModel_Go2_PredictionFunction;

    printf("%s is initialized\n", estimator->PortName);

    Estimator1001_Init(estimator);
}