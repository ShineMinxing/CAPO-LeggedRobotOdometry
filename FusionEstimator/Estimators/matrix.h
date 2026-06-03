/* Matrix Operations Library Header File */
/*
% Please call void matrix_test(); function to understand how to use this file
% Initial version: Minxing Sun
% Unit: UCAS, Institute of Optics And Electronics, Lab 1, Class of 2020
% Email: 401435318@qq.com
% Date: November 16, 2024
% 
*/

#ifndef __MATRIX_H__
#define __MATRIX_H__

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <time.h>
#include <string.h>

// Macro Define Section
#define _IN
#define _OUT
#define _IN_OUT
#define MAX(x,y) (x)>(y)?(x):(y)
#define MIN(x,y) (x)<(y)?(x):(y)

#define _CRT_SECURE_NO_WARNINGS

#ifndef PI
#define PI 3.14159265358979323846
#endif
#ifndef M_PI
#define M_PI PI
#endif
#define POSITIVE_INFINITY 999999999
#define NEGATIVE_INFINITY -999999999
#define _ERROR_NO_ERROR                                             0x00000000    // No error
#define _ERROR_FAILED_TO_ALLOCATE_HEAP_MEMORY                       0x00000001    // Failed to allocate heap memory
#define _ERROR_SVD_EXCEED_MAX_ITERATIONS                            0x00000002    // SVD exceeded max iterations
#define _ERROR_MATRIX_ROWS_OR_COLUMNS_NOT_EQUAL                     0x00000003    // Matrix rows or columns not equal
#define _ERROR_MATRIX_MULTIPLICATION                                0x00000004    // Matrix multiplication error (columns of first matrix not equal to rows of second matrix)
#define _ERROR_MATRIX_MUST_BE_SQUARE                                0x00000005    // Matrix must be square
#define _ERROR_MATRIX_NORM_TYPE_INVALID                             0x00000006    // Invalid matrix norm type
#define _ERROR_MATRIX_EQUATION_HAS_NO_SOLUTIONS                     0x00000007    // Matrix equation has no solutions
#define _ERROR_MATRIX_EQUATION_HAS_INFINITY_MANY_SOLUTIONS          0x00000008    // Matrix equation has infinitely many solutions
#define _ERROR_QR_DECOMPOSITION_FAILED                              0x00000009    // QR decomposition failed
#define _ERROR_CHOLESKY_DECOMPOSITION_FAILED                        0x0000000A    // Cholesky decomposition failed
#define _ERROR_IMPROVED_CHOLESKY_DECOMPOSITION_FAILED               0x0000000B    // Improved Cholesky decomposition failed
#define _ERROR_LU_DECOMPOSITION_FAILED                              0x0000000C    // LU decomposition failed
#define _ERROR_CREATE_MATRIX_FAILED                                 0x0000000D    // Failed to create matrix
#define _ERROR_MATRIX_TRANSPOSE_FAILED                              0x0000000E    // Matrix transpose failed
#define _ERROR_CREATE_VECTOR_FAILED                                 0x0000000F    // Failed to create vector
#define _ERROR_VECTOR_DIMENSION_NOT_EQUAL                           0x00000010    // Vector dimensions not equal
#define _ERROR_VECTOR_NORM_TYPE_INVALID                             0x00000011    // Invalid vector norm type
#define _ERROR_VECTOR_CROSS_FAILED                                  0x00000012    // Vector cross product failed
#define _ERROR_INPUT_PARAMETERS_ERROR                               0x00010000    // Input parameters error

typedef unsigned int ERROR_ID;
typedef int INDEX;
typedef short FLAG;
typedef int INTEGER;
typedef double REAL;
typedef char* STRING;
typedef void VOID;

typedef struct matrix
{
    INTEGER rows;
    INTEGER columns;
    REAL* p;
} MATRIX;

typedef struct matrix_node
{
    MATRIX* ptr;
    struct matrix_node* next;
} MATRIX_NODE;

typedef struct matrix_element_node
{
    REAL* ptr;
    struct matrix_element_node* next;
} MATRIX_ELEMENT_NODE;

typedef struct stacks
{
    MATRIX_NODE* matrixNode;
    MATRIX_ELEMENT_NODE* matrixElementNode;
} STACKS;

#ifdef __cplusplus
extern "C" {
#endif

#ifndef ESTIMATOR_GLOBALS_H
#define ESTIMATOR_GLOBALS_H

extern ERROR_ID errorID;
extern REAL     trace;
extern STACKS   S;

#endif

// Initialize stack
VOID init_stack(_IN_OUT STACKS* S);

// Free stack
VOID free_stack(_IN STACKS* S);

double** Matrix_Jac_Eig(double **array, int n, double *eig);
int Matrix_Free(double **tmp, int m, int n);
ERROR_ID EigenValueVecter(_IN MATRIX* A, _OUT MATRIX* B, _OUT MATRIX* C);

void print_matrix(MATRIX* a, STRING string);
void matrix_test();

// Create matrix, return matrix pointer
MATRIX* creat_matrix(_IN INTEGER rows, _IN INTEGER columns, _OUT ERROR_ID* errorID, _OUT STACKS* S);
// Create multiple matrices, return matrix pointer, count number
MATRIX* creat_multiple_matrix(_IN INTEGER rows, _IN INTEGER columns, _IN INTEGER count, _OUT ERROR_ID* errorID, _OUT STACKS* S);
// Create zero matrix, return matrix pointer
MATRIX* creat_zero_matrix(_IN INTEGER rows, _IN INTEGER columns, _OUT ERROR_ID* errorID, _OUT STACKS* S);
// Create identity matrix, return matrix pointer
MATRIX* creat_eye_matrix(_IN INTEGER n, _OUT ERROR_ID* errorID, _OUT STACKS* S);
// Create diagonal matrix, return matrix pointer
MATRIX* creat_diag_matrix(_IN REAL *X, _IN INDEX L, _OUT ERROR_ID* errorID, _OUT STACKS* S); 
// Transform matrix into uniform random matrix
ERROR_ID matrix_UniformRandom(_IN MATRIX* A);
// Transform matrix into normal random matrix
ERROR_ID matrix_NormalRandom(_IN MATRIX* A);
// Clear matrix to zero matrix
ERROR_ID matrix_Clear(_IN MATRIX* A);
// Matrix addition A+B=C
ERROR_ID matrix_add(_IN MATRIX* A, _IN MATRIX* B, _OUT MATRIX* C);
// Matrix subtraction A-B=C
ERROR_ID matrix_subtraction(_IN MATRIX* A, _IN MATRIX* B, _OUT MATRIX* C);
// Matrix scalar multiplication a*B=C
ERROR_ID matrix_numbermulti(_IN REAL A, _IN MATRIX* B, _OUT MATRIX* C);
// Matrix multiplication A*B=C
ERROR_ID matrix_multiplication(_IN MATRIX* A, _IN MATRIX* B, _OUT MATRIX* C);
// Vector multiplication to get matrix v1*v2T=M
ERROR_ID matrix_vector2matrix(_IN MATRIX* A, _IN MATRIX* B, _OUT MATRIX* C);
// Matrix horizontal concatenation C=[A,B]
ERROR_ID matrix_rowmatching(_IN MATRIX* A, _IN MATRIX* B, _OUT MATRIX* C);
// Matrix vertical concatenation C=[A;B]
ERROR_ID matrix_columnmatching(_IN MATRIX* A, _IN MATRIX* B, _OUT MATRIX* C);
// Matrix extraction C=A(Rs:Rs+C->row, Cs:Cs+C->column)
ERROR_ID matrix_extraction(_IN MATRIX* A, _OUT MATRIX* C, _IN INDEX Rs, _IN INDEX Cs);
// Matrix assignment C(Rs:Rs+C->row, Cs:Cs+C->column)=A
ERROR_ID matrix_valuation(_IN MATRIX* A,  _IN MATRIX* C, _IN INDEX Rs, _IN INDEX Cs);
// Matrix determinant
ERROR_ID matrix_det(_IN MATRIX* A, _OUT MATRIX* detA);
double matrix_detsubF1(double array[5][5], int n);
double matrix_detsubF2(double array[5][5], int i, int n);
// Matrix inversion
ERROR_ID matrix_inverse(_IN MATRIX* A, _OUT MATRIX* invA);
// Matrix transpose
ERROR_ID matrix_transpose(_IN MATRIX* A, _OUT MATRIX* transposeA);
// Matrix trace
ERROR_ID matrix_trace(_IN MATRIX* A, _OUT REAL* trace);
// Cholesky decomposition of positive definite matrix A, output lower triangular matrix
ERROR_ID Cholesky_decomposition(_IN MATRIX* A, _OUT MATRIX* L);
// LUP decomposition of n*n matrix A, PA=L*U, output lower triangular matrix L, upper triangular matrix U, permutation matrix P
ERROR_ID lup_decomposition(_IN MATRIX* A, _OUT MATRIX* L, _OUT MATRIX* U, _OUT MATRIX* P);
// Solve matrix equation AX=B using LUP decomposition, where A(n*n), B(n*m), X(n*m) (written in matrix B)
ERROR_ID solve_matrix_equation_by_lup_decomposition(_IN MATRIX* A, _IN_OUT MATRIX* B);
// Vector cross product array_C=array_A×array_B, array_A/array_B/array_C are 3D vectors
ERROR_ID array_vector_cross(_IN REAL array_A[3], _IN REAL array_B[3], _OUT REAL array_C[3]);
// Check whether quaternion array_Q is valid unit quaternion, array_Q=[w,x,y,z]
ERROR_ID array_quaternion_check(_IN REAL array_Q[4], _OUT FLAG* IsOK);
// Quaternion conjugation array_Qc=conj(array_Q), array_Q/array_Qc=[w,x,y,z]
ERROR_ID array_quaternion_conjugate(_IN REAL array_Q[4], _OUT REAL array_Qc[4]);
// Quaternion multiplication array_C=array_A⊗array_B, array_A/array_B/array_C=[w,x,y,z]
ERROR_ID array_quaternion_multiplication(_IN REAL array_A[4], _IN REAL array_B[4], _OUT REAL array_C[4]);
// Quaternion normalization array_Qn=array_Q/||array_Q||, array_Q/array_Qn=[w,x,y,z]
ERROR_ID array_quaternion_normalize(_IN REAL array_Q[4], _OUT REAL array_Qn[4]);
// Rotate 3D vector by unit quaternion array_Vout=array_Q*array_V*conj(array_Q), array_Q=[w,x,y,z]
ERROR_ID array_quaternion_rotate_vector(_IN REAL array_Q[4], _IN REAL array_V[3], _OUT REAL array_Vout[3]);
// Transform ZYX Euler angle into quaternion, array_EulerZYX=[roll,pitch,yaw], array_Q=[w,x,y,z]
ERROR_ID array_eulerZYX_to_quaternion(_IN REAL array_EulerZYX[3], _OUT REAL array_Q[4]);
// Transform quaternion into ZYX Euler angle, array_Q=[w,x,y,z], array_EulerZYX=[roll,pitch,yaw]
ERROR_ID array_quaternion_to_eulerZYX(_IN REAL array_Q[4], _OUT REAL array_EulerZYX[3]);
// Wrap angle array element from (-inf,+inf) into [-PI,PI], array_B=wrap(array_A)
ERROR_ID array_angle_wrap(_IN REAL array_A[], _OUT REAL array_B[], _IN INTEGER length);
// Unwrap angle array by previous wrapped angle and turn counter, array_A=array_A+array_Turn*2*PI
ERROR_ID array_angle_unwrap(_IN_OUT REAL array_A[], _IN_OUT REAL array_Last[], _IN_OUT REAL array_Turn[], _IN INTEGER length);
// 3x3 matrix inversion array_invA=inv(array_A), array_A/array_invA are 3x3 matrices
ERROR_ID array_3x3_inverse(_IN REAL array_A[3][3], _OUT REAL array_invA[3][3]);
// 3x3 matrix and 3D vector multiplication array_B=array_A*array_V
ERROR_ID array_3x3_multiply_vector(_IN REAL array_A[3][3], _IN REAL array_V[3], _OUT REAL array_B[3]);

#ifdef __cplusplus
}
#endif
#endif
