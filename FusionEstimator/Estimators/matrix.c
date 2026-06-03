/* 矩阵运算库源文件 */

#include "matrix.h"

ERROR_ID errorID = 0;
REAL     trace   = 0;
STACKS   S;

// 矩阵测试 
void matrix_test()
{
    REAL a[3 * 3] = {4,2,3,6,5,1,8,6,4};
    REAL b[3 * 3] = {3,5,2,5,3,7,4,6,4};
    REAL f[2 * 3] = {1,2,5,8,3,6};
    REAL r[4 * 4] = {5.1,1,2.1,1,1,3.1,1,2.1,2.1,1,2.1,1,1,2.1,1,4.1};
    MATRIX *A=NULL, *B=NULL, *C=NULL, *D=NULL, *E=NULL, *F = NULL, *G = NULL, *Z=NULL, *invA=NULL,\
	 *m=NULL, *UR=NULL, *NR=NULL, *R=NULL, *DR=NULL, *L=NULL, *LT=NULL, *R2=NULL, *R3=NULL,\
	 *R4=NULL, *Diag=NULL, *V1=NULL, *V2=NULL, *VV=NULL, *detA, *detR;
    ERROR_ID errorID = _ERROR_NO_ERROR;
    REAL trace;
    STACKS S;

    init_stack(&S);

    Z = creat_zero_matrix(3,2,&errorID,&S);
    print_matrix(Z,"Z");

    E = creat_eye_matrix(3,&errorID,&S);
    print_matrix(E,"E");
    
    UR = creat_zero_matrix(3,2,&errorID,&S);
    matrix_UniformRandom(UR);
    print_matrix(UR,"UR");
    
    NR = creat_zero_matrix(3,2,&errorID,&S);
    matrix_NormalRandom(NR);
    print_matrix(NR,"NR");

    A = creat_matrix(3,3,&errorID,&S);
    A->p = a;
    print_matrix(A,"A");

    B = creat_matrix(3,3,&errorID,&S);
    B->p = b;
    print_matrix(B,"B");

    F = creat_matrix(2,3,&errorID,&S);
    F->p = f;
    print_matrix(F,"F");

    C = creat_matrix(3,3,&errorID,&S);
    D = creat_matrix(3,2,&errorID,&S);
    G = creat_matrix(2,3,&errorID,&S);
    invA = creat_matrix(3,3,&errorID,&S);

    errorID = matrix_add(A,B,C);
    errorID = matrix_subtraction(A,B,C);
    errorID = matrix_multiplication(A,B,C);

    errorID = matrix_multiplication(F,A,G);
    print_matrix(G,"G");

    errorID = matrix_transpose(G,D);
    print_matrix(D,"D");

    errorID = matrix_trace(A,&trace);

    errorID = matrix_inverse(A,invA);
    print_matrix(invA,"invA");
    
    
    R  = creat_zero_matrix(4,4,&errorID,&S);
    L  = creat_zero_matrix(4,4,&errorID,&S);
    LT = creat_zero_matrix(4,4,&errorID,&S);
    R->p = r;
    print_matrix(R,"R");
    errorID = Cholesky_decomposition(R, L);
    print_matrix(L,"L");
    errorID = matrix_transpose(L,LT);
    
    R2 = creat_zero_matrix(4,8,&errorID,&S);
    errorID = matrix_rowmatching(R,R,R2);
    print_matrix(R2,"R2");
    
    R3 = creat_zero_matrix(8,4,&errorID,&S);
    errorID = matrix_columnmatching(R,R,R3);
    print_matrix(R3,"R3");

	errorID = matrix_extraction(R3,NR,1,1);
    print_matrix(NR,"NR");
    
    errorID = matrix_valuation(F,R3,3,1);
    print_matrix(R3,"R3");
    
    Diag = creat_diag_matrix(f, 6, &errorID,&S);
    print_matrix(Diag,"Diag");
    
    V1 = creat_zero_matrix(3,1,&errorID,&S);
    V2 = creat_zero_matrix(1,4,&errorID,&S);
    VV = creat_zero_matrix(3,4,&errorID,&S);
	errorID = matrix_NormalRandom(V1);
	errorID = matrix_NormalRandom(V2);
    errorID = matrix_vector2matrix(V1,V2,VV);
    print_matrix(V1,"V1");
    print_matrix(V2,"V2");
    print_matrix(VV,"VV");
    printf("%d \n",errorID); 
    
    
    detA = creat_zero_matrix(1,1,&errorID,&S);
    detR = creat_zero_matrix(1,1,&errorID,&S);
    print_matrix(A,"A");
	errorID = matrix_det(A,detA);
    print_matrix(detA,"detA");
    print_matrix(R,"R");
	errorID = matrix_det(R,detR);
    print_matrix(detR,"detR");

	matrix_Clear(detR);
	print_matrix(detR,"detR");

    free_stack(&S);
    system("pause");
}

//初始化堆栈 
VOID init_stack(_IN_OUT STACKS* S)
{
    srand(time(NULL));
    if (S == NULL)
    {
        return;
    }
    memset(S,0,sizeof(STACKS));
}

//释放堆栈 
VOID free_stack(_IN STACKS* S)
{
    MATRIX_NODE* matrixNode = NULL;
    MATRIX_ELEMENT_NODE* matrixElementNode = NULL;
    if (S == NULL)
    {
        return;
    }
    while (S->matrixNode != NULL)
    {
        matrixNode = S->matrixNode;
        S->matrixNode = matrixNode->next;
        free(matrixNode->ptr);
        matrixNode->ptr = NULL;
        free(matrixNode);
        matrixNode = NULL;
    }
    while (S->matrixElementNode != NULL)
    {
        matrixElementNode = S->matrixElementNode;
        S->matrixElementNode = matrixElementNode->next;
        free(matrixElementNode->ptr);
        matrixElementNode->ptr = NULL;
        free(matrixElementNode);
        matrixElementNode = NULL;
    }
    
}


// 打印输出矩阵
VOID print_matrix(MATRIX* a, STRING string)
{
    INDEX i,j;
    printf("matrix %s:",string);
    printf("\n");
    for (i = 0; i < a->rows; i++)
    {
        for (j = 0; j < a->columns; j++)
        {
            printf("%lf  ", a->p[i * a->columns + j]);
        }
        printf("\n");
    }
    printf("\n");
}

// 创建矩阵
MATRIX* creat_matrix(_IN INTEGER rows, _IN INTEGER columns, _OUT ERROR_ID* errorID, _OUT STACKS* S)
{
    MATRIX* matrix = NULL;
    MATRIX_NODE* matrixNode = NULL;
    MATRIX_ELEMENT_NODE* matrixElementNode = NULL;

    if (errorID == NULL)
    {
        return NULL;
    }

    *errorID = _ERROR_NO_ERROR;
    if (rows <= 0 || columns <= 0 || S == NULL)
    {
        *errorID = _ERROR_INPUT_PARAMETERS_ERROR;
        printf("矩阵运算错误代码：%u\n", *errorID);
        return NULL;
    }

    matrix = (MATRIX*)malloc(sizeof(MATRIX));
    matrixNode = (MATRIX_NODE*)malloc(sizeof(MATRIX_NODE));
    matrixElementNode = (MATRIX_ELEMENT_NODE*)malloc(sizeof(MATRIX_ELEMENT_NODE));
    if (matrix == NULL || matrixNode == NULL || matrixElementNode == NULL)
    {
        free(matrix);
        matrix = NULL;
        free(matrixNode);
        matrixNode = NULL;
        free(matrixElementNode);
        matrixElementNode = NULL;

        *errorID = _ERROR_FAILED_TO_ALLOCATE_HEAP_MEMORY;
        printf("矩阵运算错误代码：%u\n", *errorID);
        return NULL;
    }

    matrix->rows = rows;
    matrix->columns = columns;
    matrix->p = (REAL*)malloc(rows * columns * sizeof(REAL));  //确保matrix非空才能执行指针操作
    if (matrix->p == NULL)
    {
        free(matrix->p);
        matrix->p = NULL;
        free(matrix);
        matrix = NULL;
        free(matrixNode);
        matrixNode = NULL;
        free(matrixElementNode);
        matrixElementNode = NULL;

        *errorID = _ERROR_FAILED_TO_ALLOCATE_HEAP_MEMORY;
        return NULL;
    }
    matrixNode->ptr = matrix;
    matrixNode->next = S->matrixNode;
    S->matrixNode = matrixNode;

    matrixElementNode->ptr = matrix->p;
    matrixElementNode->next = S->matrixElementNode;
    S->matrixElementNode = matrixElementNode;

    return matrix;
}

// 创建多个矩阵
MATRIX* creat_multiple_matrix(_IN INTEGER rows, _IN INTEGER columns, _IN INTEGER count, _OUT ERROR_ID* errorID, _OUT STACKS* S)
{
    INDEX i;
    MATRIX* matrix = NULL, *p = NULL;
    MATRIX_NODE* matrixNode = NULL;

    if (errorID == 0)
    {
        return NULL;
    }

    *errorID = _ERROR_NO_ERROR;
    if (rows <= 0 || columns <= 0 || count <= 0 || S == NULL)
    {
        *errorID = _ERROR_INPUT_PARAMETERS_ERROR;
        printf("矩阵运算错误代码：%u\n", *errorID);
        return NULL;
    }

    matrix = (MATRIX*)malloc(count * sizeof(MATRIX));
    matrixNode = (MATRIX_NODE*)malloc(sizeof(MATRIX_NODE));
    if (matrix == NULL || matrixNode == NULL)
    {
        free(matrix);
        matrix = NULL;
        free(matrixNode);
        matrixNode = NULL;

        *errorID = _ERROR_FAILED_TO_ALLOCATE_HEAP_MEMORY;
        printf("矩阵运算错误代码：%u\n", *errorID);
        return NULL;
    }

    for (i = 0; i < count; i++)
    {
        p = creat_matrix(rows, columns, errorID, S);
        if (p == NULL)
        {
            free(matrix);
            matrix = NULL;
            free(matrixNode);
            matrixNode = NULL;

            *errorID = _ERROR_FAILED_TO_ALLOCATE_HEAP_MEMORY;
            printf("矩阵运算错误代码：%u\n", *errorID);
            return NULL;
        }
        matrix[i] = *p;
    }

    matrixNode->ptr = matrix;
    matrixNode->next = S->matrixNode;
    S->matrixNode = matrixNode;

    return matrix;
}

// 创建零矩阵
MATRIX* creat_zero_matrix(_IN INTEGER rows, _IN INTEGER columns, _OUT ERROR_ID* errorID, _OUT STACKS* S)
{
    MATRIX* matrix = NULL;
    if (errorID == NULL)
    {
        return NULL;
    }

    *errorID = _ERROR_NO_ERROR;
    if (rows <= 0 || columns <= 0 || S == NULL)
    {
        *errorID = _ERROR_INPUT_PARAMETERS_ERROR;
        printf("矩阵运算错误代码：%u\n", *errorID);
        return NULL;
    }

    matrix = creat_matrix(rows, columns, errorID, S);
    if (*errorID == _ERROR_NO_ERROR)
    {
        memset(matrix->p, 0, rows * columns * sizeof(REAL)); //赋值
    }

    return matrix;
}

// 创建单位矩阵
MATRIX* creat_eye_matrix(_IN INTEGER n, _OUT ERROR_ID* errorID, _OUT STACKS* S)
{
    INDEX i;
    MATRIX* matrix = NULL;
    if (errorID == NULL)
    {
        return NULL;
    }

    *errorID = _ERROR_NO_ERROR;
    if (n <= 0|| S == NULL)
    {
        *errorID = _ERROR_INPUT_PARAMETERS_ERROR;
        printf("矩阵运算错误代码：%u\n", *errorID);
        return NULL;
    }

    matrix = creat_matrix(n, n, errorID, S);
    if (*errorID == _ERROR_NO_ERROR)
    {
        memset(matrix->p, 0, n * n * sizeof(REAL));  //先创建零矩阵
        for (i = 0; i < n; i++)
        {
            matrix->p[i * n + i] = 1.0;  //后主对角线赋值1.0
        }
    }
    return matrix;
}

// 创建对角矩阵 返回矩阵指针
MATRIX* creat_diag_matrix(_IN REAL *X, _IN INDEX L, _OUT ERROR_ID* errorID, _OUT STACKS* S)
{
    INDEX i;
    MATRIX* matrix = NULL;
    if (errorID == NULL)
    {
        return NULL;
    }

    *errorID = _ERROR_NO_ERROR;
    if (S == NULL)
    {
        *errorID = _ERROR_INPUT_PARAMETERS_ERROR;
        printf("矩阵运算错误代码：%u\n", *errorID);
        return NULL;
    }

    matrix = creat_matrix(L, L, errorID, S);
    if (*errorID == _ERROR_NO_ERROR)
    {
        memset(matrix->p, 0, L * L * sizeof(REAL));  //先创建零矩阵
        for (i = 0; i < L; i++)
        {
            matrix->p[i * L + i] = X[i];  //后主对角线赋值1.0
        }
    }
    return matrix;
}

// 矩阵转变为0-1均匀随机矩阵
ERROR_ID matrix_UniformRandom(_IN MATRIX* A)
{
    INDEX i;
    ERROR_ID errorID = _ERROR_NO_ERROR;
    if (errorID == _ERROR_NO_ERROR)
    {
        for (i = 0; i < (A->rows*A->columns) ; i++)
        {
            A->p[i] = (REAL)rand()/RAND_MAX;  //后主对角线赋值1.0
        }
        
    }

    return errorID;
}

// 矩阵转变为单位正态随机矩阵
ERROR_ID matrix_NormalRandom(_IN MATRIX* A)
{
    INDEX i;
    ERROR_ID errorID = _ERROR_NO_ERROR;
    REAL n1,n2;
    if (errorID == _ERROR_NO_ERROR)
    {
        for (i = 0; i < (A->rows*A->columns) ; i++)
        {
            n1 = (REAL)rand()/RAND_MAX;  
            n2 = (REAL)rand()/RAND_MAX;  
            A->p[i] = sqrt(-2*log(n1))*cos(n2*2*PI);  //后主对角线赋值1.0
        }
        
    }
    
    return errorID;
} 

// 矩阵清为零矩阵
ERROR_ID matrix_Clear(_IN MATRIX* A)
{
	INDEX i;
	ERROR_ID errorID = _ERROR_NO_ERROR;
	if (errorID == _ERROR_NO_ERROR)
	{
		for (i = 0; i < (A->rows*A->columns) ; i++)
		{
			A->p[i] = 0.0;
		}
	}

	return errorID;
} 

// 矩阵加法 A+B=C
ERROR_ID matrix_add(_IN MATRIX* A, _IN MATRIX* B, _OUT MATRIX* C)
{
    INDEX i,j;
    INTEGER rows, columns;
    ERROR_ID errorID = _ERROR_NO_ERROR;

    if (A == NULL || B == NULL || C == NULL)
    {
        errorID = _ERROR_INPUT_PARAMETERS_ERROR;
        printf("矩阵运算错误代码：%u\n", errorID);
        return errorID;
    }

    if (A->rows != B->rows || A->rows != C->rows || B->rows != C->rows || A->columns != B->columns || A->columns != C->columns || B->columns != C->columns)
    {
        errorID = _ERROR_MATRIX_ROWS_OR_COLUMNS_NOT_EQUAL;
        printf("矩阵运算错误代码：%u\n", errorID);
        return errorID;
    }

    rows = A->rows;
    columns = A->columns;
    for (i = 0; i < rows; i++)
    {
        for (j = 0; j < columns; j++)
        {
            C->p[i * columns + j] = A->p[i * columns + j] + B->p[i * columns + j];
        }
    }
    return errorID;
}

// 矩阵减法 A-B=C
ERROR_ID matrix_subtraction(_IN MATRIX* A, _IN MATRIX* B, _OUT MATRIX* C)
{
    INDEX i,j;
    INTEGER rows, columns;
    ERROR_ID errorID = _ERROR_NO_ERROR;

    if (A == NULL || B == NULL || C == NULL)
    {
        errorID = _ERROR_INPUT_PARAMETERS_ERROR;
        printf("矩阵运算错误代码：%u\n", errorID);
        return errorID;
    }

    if (A->rows != B->rows || A->rows != C->rows || B->rows != C->rows || A->columns != B->columns || A->columns != C->columns || B->columns != C->columns)
    {
        errorID = _ERROR_MATRIX_ROWS_OR_COLUMNS_NOT_EQUAL;
        printf("矩阵运算错误代码：%u\n", errorID);
        return errorID;
    }

    rows = A->rows;
    columns = A->columns;
    for (i = 0; i < rows; i++)
    {
        for (j = 0; j < columns; j++)
        {
            C->p[i * columns + j] = A->p[i * columns + j] - B->p[i * columns + j];
        }
    }
    return errorID;
}

// 矩阵数乘 C=a*B
ERROR_ID matrix_numbermulti(_IN REAL A, _IN MATRIX* B, _OUT MATRIX* C)
{
    INDEX i,j;
    ERROR_ID errorID = _ERROR_NO_ERROR;

    if (B == NULL || C == NULL)
    {
        errorID = _ERROR_INPUT_PARAMETERS_ERROR;
        printf("矩阵运算错误代码：%u\n", errorID);
        return errorID;
    }

    if (B->rows != C->rows || B->columns != C->columns)
    {
        errorID = _ERROR_MATRIX_MULTIPLICATION;
        printf("矩阵运算错误代码：%u\n", errorID);
        return errorID;
    }

    for (i = 0; i < B->rows; i++)
    {
        for (j = 0; j < B->columns; j++)
        {
            C->p[i * B->columns + j] = A * B->p[i * B->columns + j];
        }
    }
    return errorID;
}

// 矩阵乘法 C=A*B
ERROR_ID matrix_multiplication(_IN MATRIX* A, _IN MATRIX* B, _OUT MATRIX* C)
{
    INDEX i,j,k;
    REAL sum;
    ERROR_ID errorID = _ERROR_NO_ERROR;

    if (A == NULL || B == NULL || C == NULL)
    {
        errorID = _ERROR_INPUT_PARAMETERS_ERROR;
        printf("矩阵运算错误代码：%u\n", errorID);
        return errorID;
    }

    if (A->columns != B->rows || A->rows != C->rows || B->columns != C->columns)
    {
        errorID = _ERROR_MATRIX_MULTIPLICATION;
        printf("矩阵运算错误代码：%u\n", errorID);
        return errorID;
    }

    for (i = 0; i < A->rows; i++)
    {
        for (j = 0; j < B->columns; j++)
        {
            sum = 0.0;
            for (k = 0; k < A->columns; k++)
            {
                sum += A->p[i * A->columns + k] * B->p[k * B->columns + j];
            }
            C->p[i * B->columns + j] = sum;
        }
    }
    return errorID;
}

// 向量相乘得矩阵 v1*v2T=M
ERROR_ID matrix_vector2matrix(_IN MATRIX* A, _IN MATRIX* B, _OUT MATRIX* C)
{
	INDEX i,j;
    ERROR_ID errorID = _ERROR_NO_ERROR;

    if (A == NULL || B == NULL || C == NULL)
    {
        errorID = _ERROR_INPUT_PARAMETERS_ERROR;
        printf("矩阵运算错误代码：%u\n", errorID);
        return errorID;
    }

    if (A->rows != C->rows || B->columns != C->columns || A->columns != 1 || B->rows != 1)
    {
        errorID = _ERROR_MATRIX_MULTIPLICATION;
        printf("矩阵运算错误代码：%u\n", errorID);
        return errorID;
    }

    for (i = 0; i < C->rows; i++)
    {
        for (j = 0; j < C->columns; j++)
        {
            C->p[i * C->columns + j] = A->p[i] * B->p[j];
        }
    }
    return errorID;
} 

// 矩阵横向拼接 C=[A,B] 
ERROR_ID matrix_rowmatching(_IN MATRIX* A, _IN MATRIX* B, _OUT MATRIX* C)
{
	INDEX i,j;
    ERROR_ID errorID = _ERROR_NO_ERROR;

    if (A == NULL || B == NULL || C == NULL)
    {
        errorID = _ERROR_INPUT_PARAMETERS_ERROR;
        printf("矩阵运算错误代码：%u\n", errorID);
        return errorID;
    }

    if (A->rows != B->rows || A->rows != C->rows || A->columns + B->columns != C->columns)
    {
        errorID = _ERROR_MATRIX_ROWS_OR_COLUMNS_NOT_EQUAL;
        printf("矩阵运算错误代码：%u\n", errorID);
        return errorID;
    }
    
    for (i = 0; i < A->rows; i++)
    {
        for (j = 0; j < A->columns; j++)
        {
            C->p[i * C->columns + j] = A->p[i * A->columns + j];
        }
        for (j = 0; j < B->columns; j++)
        {
            C->p[i * C->columns + j + A->columns] = B->p[i * B->columns + j];
        }
    }
    return errorID;
}

// 矩阵纵向拼接 C=[A;B] 
ERROR_ID matrix_columnmatching(_IN MATRIX* A, _IN MATRIX* B, _OUT MATRIX* C)
{
	INDEX i,j;
    ERROR_ID errorID = _ERROR_NO_ERROR;

    if (A == NULL || B == NULL || C == NULL)
    {
        errorID = _ERROR_INPUT_PARAMETERS_ERROR;
        printf("矩阵运算错误代码：%u\n", errorID);
        return errorID;
    }

    if (A->columns != B->columns || A->columns != C->columns || A->rows + B->rows != C->rows)
    {
        errorID = _ERROR_MATRIX_ROWS_OR_COLUMNS_NOT_EQUAL;
        printf("矩阵运算错误代码：%u\n", errorID);
        return errorID;
    }
    
    for (i = 0; i < A->rows; i++)
    {
        for (j = 0; j < A->columns; j++)
        {
            C->p[i * C->columns + j] = A->p[i * A->columns + j];
        }
    }
    for (i = 0; i < B->rows; i++)
    {
        for (j = 0; j < B->columns; j++)
        {
            C->p[(i + A->rows) * C->columns + j] = B->p[i * B->columns + j];
        }
    }
    return errorID;
}

// 矩阵提取 C=A（Rs:Rs+C->row,Cs:Cs+C->column）
ERROR_ID matrix_extraction(_IN MATRIX* A, _OUT MATRIX* C, _IN INDEX Rs, _IN INDEX Cs)
{
	INDEX i,j;
    ERROR_ID errorID = _ERROR_NO_ERROR;

    if (A == NULL || C == NULL)
    {
        errorID = _ERROR_INPUT_PARAMETERS_ERROR;
        printf("矩阵运算错误代码：%u\n", errorID);
        return errorID;
    }

    if (A->rows < C->rows + Rs || A->columns < C->columns + Cs)
    {
        errorID = _ERROR_MATRIX_ROWS_OR_COLUMNS_NOT_EQUAL;
        printf("矩阵运算错误代码：%u\n", errorID);
        return errorID;
    }
    
    for (i = 0; i < C->rows; i++)
    {
        for (j = 0; j < C->columns; j++)
        {
            C->p[i * C->columns + j] = A->p[(i + Rs)* A->columns + j + Cs];
        }
    }
    return errorID;
}
// 矩阵赋值 C（Rs:Rs+C->row,Cs:Cs+C->column）=A 
ERROR_ID matrix_valuation(_IN MATRIX* A,  _IN MATRIX* C, _IN INDEX Rs, _IN INDEX Cs)
{
	INDEX i,j;
    ERROR_ID errorID = _ERROR_NO_ERROR;

    if (A == NULL || C == NULL)
    {
        errorID = _ERROR_INPUT_PARAMETERS_ERROR;
        printf("矩阵运算错误代码：%u\n", errorID);
        return errorID;
    }

    if (A->rows + Rs > C->rows || A->columns + Cs > C->columns)
    {
        errorID = _ERROR_MATRIX_ROWS_OR_COLUMNS_NOT_EQUAL;
        printf("矩阵运算错误代码：%u\n", errorID);
        return errorID;
    }
    
    for (i = Rs; i < Rs + A->rows; i++)
    {
        for (j = Cs; j < Cs + A->columns; j++)
        {
            C->p[i * C->columns + j] = A->p[(i - Rs)* A->columns + j - Cs];
        }
    }
    return errorID;
}
// 矩阵求行列式
ERROR_ID matrix_det(_IN MATRIX* A, _OUT MATRIX* detA)
{
    int i, j;
    double array[5][5];
    ERROR_ID errorID = _ERROR_NO_ERROR;

    if (A == NULL || detA == NULL || A->p == NULL || detA->p == NULL)
    {
        errorID = _ERROR_INPUT_PARAMETERS_ERROR;
        printf("矩阵运算错误代码：%u\n", errorID);
        return errorID;
    }

    if (A->rows != A->columns || detA->rows != 1 || detA->columns != 1)
    {
        errorID = _ERROR_MATRIX_ROWS_OR_COLUMNS_NOT_EQUAL;
        printf("矩阵运算错误代码：%u\n", errorID);
        return errorID;
    }

    /* 最小改动：原实现只支持 5 阶及以下，直接加保护，避免栈越界 */
    if (A->rows > 5)
    {
        errorID = _ERROR_INPUT_PARAMETERS_ERROR;
        printf("矩阵运算错误代码：%u\n", errorID);
        return errorID;
    }

    for (i = 0; i < A->rows; i++)
    {
        for (j = 0; j < A->rows; j++)
        {
            array[i][j] = A->p[j + i * A->columns];
        }
    }

    detA->p[0] = matrix_detsubF1(array, A->rows);
    return errorID;
}
double matrix_detsubF1(double array[5][5], int n)
{
	int i;
	double M, sum = 0.0;
	if (n == 1)//一阶行列式直接得出结果
		return array[0][0];
	else if (n > 1)
	{
		for (i = 0; i < n; i++)//按第一行展开
		{
			M = matrix_detsubF2(array, i, n);
			sum += pow(-1, i + 2) * array[0][i] * M;
		}
	}
	return sum;
}
double matrix_detsubF2(double array[5][5], int i, int n)
{	
	int j,k;
	double SubArray[5][5];
	
	//构造余子式的过程
		for (j = 0; j < n - 1; j++)
		{
			for (k = 0; k < n - 1; k++)
			{
				if (k < i)
					SubArray[j][k] = array[j + 1][k];
				else if (k >= i)
					SubArray[j][k] = array[j + 1][k + 1];
			}
		}
		
	return matrix_detsubF1(SubArray, n - 1);//计算余子式的行列式
}
// 矩阵求逆
ERROR_ID matrix_inverse(_IN MATRIX* A, _OUT MATRIX* invA)
{
    INDEX i;
    INTEGER n;
    MATRIX* Atemp = NULL;
    ERROR_ID errorID = _ERROR_NO_ERROR;
    STACKS S;

    if (A == NULL || invA == NULL || A->p == NULL || invA->p == NULL)
    {
        errorID = _ERROR_INPUT_PARAMETERS_ERROR;
        printf("矩阵运算错误代码：%u\n", errorID);
        return errorID;
    }

    if (A->rows != A->columns)
    {
        errorID = _ERROR_MATRIX_MUST_BE_SQUARE;
        printf("矩阵运算错误代码：%u\n", errorID);
        return errorID;
    }

    n = A->rows;
    if (invA->rows != n || invA->columns != n)
    {
        errorID = _ERROR_MATRIX_ROWS_OR_COLUMNS_NOT_EQUAL;
        printf("矩阵运算错误代码：%u\n", errorID);
        return errorID;
    }

    init_stack(&S);

    Atemp = creat_matrix(n, n, &errorID, &S);
    if (errorID != _ERROR_NO_ERROR)
    {
        goto EXIT;
    }

    memcpy(Atemp->p, A->p, n * n * sizeof(REAL));
    memset(invA->p, 0, n * n * sizeof(REAL));
    for (i = 0; i < n; i++)
    {
        invA->p[i * n + i] = 1.0;
    }

    errorID = solve_matrix_equation_by_lup_decomposition(Atemp, invA);

EXIT:
    free_stack(&S);
    return errorID;
}
// 矩阵转置
ERROR_ID matrix_transpose(_IN MATRIX* A, _OUT MATRIX* transposeA)
{
    INDEX i,j;
    ERROR_ID errorID = _ERROR_NO_ERROR;

    if (A == NULL || transposeA == NULL)
    {
        errorID = _ERROR_INPUT_PARAMETERS_ERROR;
        printf("矩阵运算错误代码：%u\n", errorID);
        return errorID;
    }

    if (A->rows != transposeA->columns || A->columns != transposeA->rows)
    {
        errorID = _ERROR_MATRIX_TRANSPOSE_FAILED;
        printf("矩阵运算错误代码：%u\n", errorID);
        return errorID;
    }

    for (i = 0; i < A->rows; i++)
    {
        for (j = 0; j < A->columns; j++)
        {
            transposeA->p[j * A->rows + i] = A->p[i * A->columns + j];
        }
    }
    return errorID;
}

// 矩阵的迹
ERROR_ID matrix_trace(_IN MATRIX* A, _OUT REAL* trace)
{
    INDEX i;
    ERROR_ID errorID = _ERROR_NO_ERROR;

    if (A == NULL || trace == NULL)
    {
        errorID = _ERROR_INPUT_PARAMETERS_ERROR;
        printf("矩阵运算错误代码：%u\n", errorID);
        return errorID;
    }

    if (A->rows != A->columns)
    {
        errorID = _ERROR_MATRIX_MUST_BE_SQUARE;
        printf("矩阵运算错误代码：%u\n", errorID);
        return errorID;
    }

    *trace = 0.0;
    for (i = 0; i < A->rows; i++)
    {
        *trace += A->p[i * A->columns + i];
    }
    return errorID;
}

// 正定矩阵A的Cholesky 分解  LT*L=R
ERROR_ID Cholesky_decomposition(_IN MATRIX* A, _OUT MATRIX* L)
{
    INDEX i, j, k, n;
    double sum = 0;
    ERROR_ID errorID = _ERROR_NO_ERROR;

    if (A == NULL || L == NULL)
    {
        errorID = _ERROR_INPUT_PARAMETERS_ERROR;
        printf("矩阵运算错误代码：%u\n", errorID);
        return errorID;
    }

    if (A->rows != A->columns)
    {
        errorID = _ERROR_MATRIX_MUST_BE_SQUARE;
        printf("矩阵运算错误代码：%u\n", errorID);
    }
    n = A->rows;
    memcpy(L->p, A->p, n * n * sizeof(REAL));
    for(k = 0; k < n; k++)
    {
        sum = 0;
        for(i = 0; i < k; i++)
            sum += L->p[k * n + i] * L->p[k * n + i];
        sum = A->p[k * n + k] - sum;
        L->p[k * n + k] = sqrt(sum > 0 ? sum : 0);
        for(i = k + 1; i < n; i++)
        {
            sum = 0;
            for(j = 0; j < k; j++)
                sum += L->p[i * n + j] * L->p[k * n + j];
            L->p[i * n + k] = (A->p[i * n + k] - sum) / L->p[k * n + k];
        }
        for(j = 0; j < k; j++)
            L->p[j * n + k] = 0;
    }
    return errorID;
}

// n行n列矩阵A的LUP分解 PA=L*U n行n列下三角L矩阵 n行n列上三角U矩阵 n行n列置换矩阵P
ERROR_ID lup_decomposition(_IN MATRIX* A, _OUT MATRIX* L, _OUT MATRIX* U, _OUT MATRIX* P)
{
    INDEX i, j, k, index, s, t;
    INTEGER n;
    REAL maxvalue, temp;
    ERROR_ID errorID = _ERROR_NO_ERROR;

    if (A == NULL || L == NULL || U == NULL || P == NULL)
    {
        errorID = _ERROR_INPUT_PARAMETERS_ERROR;
        printf("矩阵运算错误代码：%u\n", errorID);
        return errorID;
    }

    if (A->rows != A->columns)
    {
        errorID = _ERROR_MATRIX_MUST_BE_SQUARE;
        printf("矩阵运算错误代码：%u\n", errorID);
        return errorID;
    }

    n = A->rows;
    memcpy(U->p, A->p, n * n * sizeof(REAL));
    memset(L->p, 0, n * n * sizeof(REAL));
    memset(P->p, 0, n * n * sizeof(REAL));
    for (i = 0; i < n; i++)
    {
        L->p[i * n + i] = 1.0;
        P->p[i * n + i] = 1.0;
    }
    for (j = 0; j < n - 1; j++)
    {
        // Select i(>=j) that maxmizes |U(i,j)|
        index = -1;
        maxvalue = 0.0;
        for (i = j; i < n; i++)
        {
            temp = fabs(U->p[i * n + j]);
            if (temp > maxvalue)
            {
                maxvalue = temp;
                index = i;
            }
        }
        if (index == -1)
        {
            continue;
        }

        for (k = j; k < n; k++)
        {
            s = j * n + k;
            t = index * n + k;
            temp = U->p[s];
            U->p[s] = U->p[t];
            U->p[t] = temp;
        }

        for (k = 0; k < j; k++)
        {
            s = j * n + k;
            t = index * n + k;
            temp = L->p[s];
            L->p[s] = L->p[t];
            L->p[t] = temp;
        }

        for (k = 0; k < n; k++)
        {
            s = j * n + k;
            t = index * n + k;
            temp = P->p[s];
            P->p[s] = P->p[t];
            P->p[t] = temp;
        }

        for (i = j + 1; i < n; i++)
        {
            s = i * n + j;
            L->p[s] = U->p[s] / U->p[j * n + j];
            for (k = j; k < n; k++)
            {
                U->p[i * n + k] -= L->p[s] * U->p[j * n + k];
            }
        }
    }
    return errorID;
}
// LUP分解解矩阵方程AX=B，其中A(n*n)，B(n*m)，X(n*m)待求矩阵（写到矩阵B）
ERROR_ID solve_matrix_equation_by_lup_decomposition(_IN MATRIX* A, _IN_OUT MATRIX* B)
{
    INDEX i, j, k, index, s, t;
    INTEGER n, m;
    REAL sum, maxvalue, temp;
    MATRIX* L = NULL, *U = NULL, *y = NULL;
    ERROR_ID errorID = _ERROR_NO_ERROR;
    STACKS S;

    if (A == NULL || B == NULL || A->p == NULL || B->p == NULL)
    {
        errorID = _ERROR_INPUT_PARAMETERS_ERROR;
        printf("矩阵运算错误代码：%u\n", errorID);
        return errorID;
    }

    if (A->rows != A->columns || B->rows != A->rows)
    {
        errorID = _ERROR_MATRIX_MUST_BE_SQUARE;
        printf("矩阵运算错误代码：%u\n", errorID);
        return errorID;
    }

    init_stack(&S);

    n = A->rows;
    m = B->columns;

    L = creat_matrix(n, n, &errorID, &S);
    if (errorID != _ERROR_NO_ERROR) goto EXIT;

    U = creat_matrix(n, n, &errorID, &S);
    if (errorID != _ERROR_NO_ERROR) goto EXIT;

    /* 关键修复：这里必须是 n*m，而不是 n*n */
    y = creat_matrix(n, m, &errorID, &S);
    if (errorID != _ERROR_NO_ERROR) goto EXIT;

    memcpy(U->p, A->p, n * n * sizeof(REAL));
    memset(L->p, 0, n * n * sizeof(REAL));
    for (i = 0; i < n; i++)
    {
        L->p[i * n + i] = 1.0;
    }

    for (j = 0; j < n - 1; j++)
    {
        index = -1;
        maxvalue = 0.0;
        for (i = j; i < n; i++)
        {
            temp = fabs(U->p[i * n + j]);
            if (temp > maxvalue)
            {
                maxvalue = temp;
                index = i;
            }
        }
        if (index == -1)
        {
            continue;
        }

        for (k = j; k < n; k++)
        {
            s = j * n + k;
            t = index * n + k;
            temp = U->p[s];
            U->p[s] = U->p[t];
            U->p[t] = temp;
        }

        for (k = 0; k < j; k++)
        {
            s = j * n + k;
            t = index * n + k;
            temp = L->p[s];
            L->p[s] = L->p[t];
            L->p[t] = temp;
        }

        for (k = 0; k < m; k++)
        {
            s = j * m + k;
            t = index * m + k;
            temp = B->p[s];
            B->p[s] = B->p[t];
            B->p[t] = temp;
        }

        for (i = j + 1; i < n; i++)
        {
            s = i * n + j;
            L->p[s] = U->p[s] / U->p[j * n + j];
            for (k = j; k < n; k++)
            {
                U->p[i * n + k] -= L->p[s] * U->p[j * n + k];
            }
        }
    }

    for (i = 0; i < n; i++)
    {
        if (fabs(U->p[i * n + i]) < 1.0e-20)
        {
            errorID = _ERROR_MATRIX_EQUATION_HAS_NO_SOLUTIONS;
            goto EXIT;
        }
    }

    for (j = 0; j < m; j++)
    {
        for (i = 0; i < n; i++)
        {
            sum = 0.0;
            for (k = 0; k < i; k++)
            {
                sum += L->p[i * n + k] * y->p[k * m + j];
            }
            y->p[i * m + j] = B->p[i * m + j] - sum;
        }
    }

    for (j = 0; j < m; j++)
    {
        for (i = n - 1; i >= 0; i--)
        {
            sum = 0.0;
            for (k = i + 1; k < n; k++)
            {
                sum += U->p[i * n + k] * B->p[k * m + j];
            }
            B->p[i * m + j] = (y->p[i * m + j] - sum) / U->p[i * n + i];
        }
    }

EXIT:
    free_stack(&S);
    return errorID;
}
ERROR_ID EigenValueVecter(_IN MATRIX* A, _OUT MATRIX* B, _OUT MATRIX* C)
{
    INDEX i, j, n;
    ERROR_ID errorID = _ERROR_NO_ERROR;
    double **array = NULL;
    double *eig = NULL;
    double **Result = NULL;

    if (A == NULL || B == NULL || C == NULL || A->p == NULL || B->p == NULL || C->p == NULL)
    {
        errorID = _ERROR_INPUT_PARAMETERS_ERROR;
        printf("矩阵运算错误代码：%u\n", errorID);
        return errorID;
    }

    if (A->rows != B->rows || B->rows != C->rows || C->columns != 1)
    {
        errorID = _ERROR_INPUT_PARAMETERS_ERROR;
        printf("矩阵运算错误代码：%u\n", errorID);
        return errorID;
    }

    if (A->rows != A->columns || B->rows != B->columns)
    {
        errorID = _ERROR_MATRIX_MUST_BE_SQUARE;
        printf("矩阵运算错误代码：%u\n", errorID);
        return errorID;
    }

    n = A->rows;

    array = (double **)calloc((size_t)n, sizeof(double *));
    eig = (double *)malloc((size_t)n * sizeof(double));
    if (array == NULL || eig == NULL)
    {
        Matrix_Free(array, n, n);
        free(eig);
        printf("error :申请数组内存空间失败\n");
        return _ERROR_FAILED_TO_ALLOCATE_HEAP_MEMORY;
    }

    for (i = 0; i < n; i++)
    {
        array[i] = (double *)malloc((size_t)n * sizeof(double));
        if (array[i] == NULL)
        {
            Matrix_Free(array, n, n);
            free(eig);
            printf("error :申请数组子内存空间失败\n");
            return _ERROR_FAILED_TO_ALLOCATE_HEAP_MEMORY;
        }
    }

    for (i = 0; i < n; i++)
    {
        for (j = 0; j < n; j++)
        {
            array[i][j] = A->p[i * n + j];
        }
    }

    Result = Matrix_Jac_Eig(array, n, eig);
    if (Result == NULL)
    {
        Matrix_Free(array, n, n);
        free(eig);
        return _ERROR_CREATE_MATRIX_FAILED;
    }

    printf("特征矩阵:\n");
    for (i = 0; i < n; i++)
    {
        for (j = 0; j < n; j++)
        {
            B->p[i * n + j] = Result[i][j];
            printf("%f ", Result[i][j]);
        }
        printf("\n");
    }

    printf("特征值向量:\n");
    for (i = 0; i < n; i++)
    {
        C->p[i] = eig[i];
        printf("%f \n", eig[i]);
    }

    Matrix_Free(Result, n, n);
    Matrix_Free(array, n, n);
    free(eig);
    eig = NULL;

    return errorID;
}
double** Matrix_Jac_Eig(double **array, int n, double *eig)
{
    int i, j, flag, k;
    double sum = 0.0;
    double **temp_mat = NULL;
    double **result_temp = NULL;
    double **result = NULL;
    double **rot = NULL;
    double **mat = NULL;
    int p = 0, q = 1;
    double thresh = 0.0000000001;
    double max = 0.0;
    double tan_angle, sin_angle, cos_angle;

    if (array == NULL || eig == NULL || n <= 0)
    {
        return NULL;
    }

    if (n == 1)
    {
        result = (double **)calloc(1, sizeof(double *));
        if (result == NULL)
        {
            return NULL;
        }

        result[0] = (double *)malloc(sizeof(double));
        if (result[0] == NULL)
        {
            free(result);
            return NULL;
        }

        result[0][0] = 1.0;
        eig[0] = array[0][0];
        return result;
    }

    temp_mat = (double **)calloc((size_t)n, sizeof(double *));
    result_temp = (double **)calloc((size_t)n, sizeof(double *));
    if (temp_mat == NULL || result_temp == NULL)
    {
        Matrix_Free(temp_mat, n, n);
        Matrix_Free(result_temp, n, n);
        return NULL;
    }

    for (i = 0; i < n; i++)
    {
        temp_mat[i] = (double *)malloc((size_t)n * sizeof(double));
        if (temp_mat[i] == NULL)
        {
            Matrix_Free(temp_mat, n, n);
            Matrix_Free(result_temp, n, n);
            return NULL;
        }
    }

    for (i = 0; i < n; i++)
    {
        for (j = 0; j < n; j++)
        {
            temp_mat[i][j] = array[i][j];
        }
    }

    flag = 0;
    for (i = 0; i < n; i++)
    {
        for (j = i; j < n; j++)
        {
            if (array[i][j] != array[j][i])
            {
                flag = 1;
                break;
            }
        }
        if (flag == 1)
            break;
    }

    if (flag == 1)
    {
        printf("error in Matrix_Eig: 输入并非是对称矩阵:\n");
        Matrix_Free(temp_mat, n, n);
        Matrix_Free(result_temp, n, n);
        return NULL;
    }

    result = (double **)calloc((size_t)n, sizeof(double *));
    rot    = (double **)calloc((size_t)n, sizeof(double *));
    mat    = (double **)calloc((size_t)n, sizeof(double *));
    if (result == NULL || rot == NULL || mat == NULL)
    {
        Matrix_Free(temp_mat, n, n);
        Matrix_Free(result_temp, n, n);
        Matrix_Free(result, n, n);
        Matrix_Free(rot, n, n);
        Matrix_Free(mat, n, n);
        return NULL;
    }

    for (i = 0; i < n; i++)
    {
        result[i] = (double *)malloc((size_t)n * sizeof(double));
        result_temp[i] = (double *)malloc((size_t)n * sizeof(double));
        rot[i] = (double *)malloc((size_t)n * sizeof(double));
        mat[i] = (double *)malloc((size_t)n * sizeof(double));

        if (result[i] == NULL || result_temp[i] == NULL || rot[i] == NULL || mat[i] == NULL)
        {
            Matrix_Free(temp_mat, n, n);
            Matrix_Free(result_temp, n, n);
            Matrix_Free(result, n, n);
            Matrix_Free(rot, n, n);
            Matrix_Free(mat, n, n);
            return NULL;
        }
    }

    for (i = 0; i < n; i++)
    {
        for (j = 0; j < n; j++)
        {
            if (i == j)
            {
                result[i][j] = 1.0;
                mat[i][j] = 1.0;
            }
            else
            {
                result[i][j] = 0.0;
                mat[i][j] = 0.0;
            }
        }
    }

    max = array[0][1];
    p = 0;
    q = 1;
    for (i = 0; i < n; i++)
    {
        for (j = 0; j < n; j++)
        {
            if (i == j)
            {
                continue;
            }
            else
            {
                if (fabs(array[i][j]) >= fabs(max))
                {
                    max = array[i][j];
                    p = i;
                    q = j;
                }
            }
        }
    }

    while (fabs(max) > thresh)
    {
        tan_angle = -2 * array[p][q] / (array[q][q] - array[p][p]);
        sin_angle = sin(0.5 * atan(tan_angle));
        cos_angle = cos(0.5 * atan(tan_angle));

        for (i = 0; i < n; i++)
        {
            for (j = 0; j < n; j++)
            {
                if (i == j)
                    mat[i][j] = 1.0;
                else
                    mat[i][j] = 0.0;
            }
        }

        mat[p][p] = cos_angle;
        mat[q][q] = cos_angle;
        mat[q][p] = sin_angle;
        mat[p][q] = -sin_angle;

        for (i = 0; i < n; i++)
        {
            for (j = 0; j < n; j++)
            {
                rot[i][j] = array[i][j];
            }
        }

        for (j = 0; j < n; j++)
        {
            rot[p][j] = cos_angle * array[p][j] + sin_angle * array[q][j];
            rot[q][j] = -sin_angle * array[p][j] + cos_angle * array[q][j];
            rot[j][p] = cos_angle * array[j][p] + sin_angle * array[j][q];
            rot[j][q] = -sin_angle * array[j][p] + cos_angle * array[j][q];
        }

        rot[p][p] = array[p][p] * cos_angle * cos_angle +
                    array[q][q] * sin_angle * sin_angle +
                    2 * array[p][q] * cos_angle * sin_angle;
        rot[q][q] = array[q][q] * cos_angle * cos_angle +
                    array[p][p] * sin_angle * sin_angle -
                    2 * array[p][q] * cos_angle * sin_angle;
        rot[p][q] = 0.5 * (array[q][q] - array[p][p]) * 2 * sin_angle * cos_angle +
                    array[p][q] * (2 * cos_angle * cos_angle - 1);
        rot[q][p] = 0.5 * (array[q][q] - array[p][p]) * 2 * sin_angle * cos_angle +
                    array[p][q] * (2 * cos_angle * cos_angle - 1);

        for (i = 0; i < n; i++)
        {
            for (j = 0; j < n; j++)
            {
                array[i][j] = rot[i][j];
            }
        }

        max = array[0][1];
        p = 0;
        q = 1;
        for (i = 0; i < n; i++)
        {
            for (j = 0; j < n; j++)
            {
                if (i == j)
                {
                    continue;
                }
                else
                {
                    if (fabs(array[i][j]) >= fabs(max))
                    {
                        max = array[i][j];
                        p = i;
                        q = j;
                    }
                }
            }
        }

        for (i = 0; i < n; i++)
        {
            eig[i] = array[i][i];
        }

        for (i = 0; i < n; i++)
        {
            for (j = 0; j < n; j++)
            {
                sum = 0.0;
                for (k = 0; k < n; k++)
                {
                    sum += result[i][k] * mat[k][j];
                }
                result_temp[i][j] = sum;
            }
        }

        for (i = 0; i < n; i++)
        {
            for (j = 0; j < n; j++)
            {
                result[i][j] = result_temp[i][j];
            }
        }
    }

    for (i = 0; i < n; i++)
    {
        eig[i] = array[i][i];
    }

    for (i = 0; i < n; i++)
    {
        for (j = 0; j < n; j++)
        {
            array[i][j] = temp_mat[i][j];
        }
    }

    Matrix_Free(result_temp, n, n);
    Matrix_Free(rot, n, n);
    Matrix_Free(mat, n, n);
    Matrix_Free(temp_mat, n, n);

    return result;
}
int Matrix_Free(double **tmp, int m, int n)
{
	int i;
	if (tmp == NULL)
	{
		return(1);
	}
	for (i = 0; i < m; i++)
	{
		if (tmp[i] != NULL)
		{
			free(tmp[i]);
			tmp[i] = NULL;
		}
	}
	if (tmp != NULL)
	{
		free(tmp);
		tmp = NULL;
	}
	return(0);
}
ERROR_ID array_vector_cross(_IN REAL array_A[3], _IN REAL array_B[3], _OUT REAL array_C[3])
{
    ERROR_ID errorID = _ERROR_NO_ERROR;

    REAL ax, ay, az;
    REAL bx, by, bz;
    REAL cx, cy, cz;

    if (array_A == NULL || array_B == NULL || array_C == NULL)
    {
        errorID = _ERROR_INPUT_PARAMETERS_ERROR;
        printf("矩阵运算错误代码：%u\n", errorID);
        return errorID;
    }

    ax = array_A[0];
    ay = array_A[1];
    az = array_A[2];

    bx = array_B[0];
    by = array_B[1];
    bz = array_B[2];

    cx = ay * bz - az * by;
    cy = az * bx - ax * bz;
    cz = ax * by - ay * bx;

    array_C[0] = cx;
    array_C[1] = cy;
    array_C[2] = cz;

    return errorID;
}

ERROR_ID array_quaternion_check(_IN REAL array_Q[4], _OUT FLAG* IsOK)
{
    ERROR_ID errorID = _ERROR_NO_ERROR;

    REAL n2;

    if (array_Q == NULL || IsOK == NULL)
    {
        errorID = _ERROR_INPUT_PARAMETERS_ERROR;
        printf("矩阵运算错误代码：%u\n", errorID);
        return errorID;
    }

    n2 = array_Q[0] * array_Q[0] +
         array_Q[1] * array_Q[1] +
         array_Q[2] * array_Q[2] +
         array_Q[3] * array_Q[3];

    *IsOK = (isfinite(n2) && fabs(n2 - 1.0) < 1.0e-2) ? 1 : 0;

    return errorID;
}

ERROR_ID array_quaternion_conjugate(_IN REAL array_Q[4], _OUT REAL array_Qc[4])
{
    ERROR_ID errorID = _ERROR_NO_ERROR;

    REAL q0, q1, q2, q3;

    if (array_Q == NULL || array_Qc == NULL)
    {
        errorID = _ERROR_INPUT_PARAMETERS_ERROR;
        printf("矩阵运算错误代码：%u\n", errorID);
        return errorID;
    }

    q0 = array_Q[0];
    q1 = array_Q[1];
    q2 = array_Q[2];
    q3 = array_Q[3];

    array_Qc[0] = q0;
    array_Qc[1] = -q1;
    array_Qc[2] = -q2;
    array_Qc[3] = -q3;

    return errorID;
}

ERROR_ID array_quaternion_multiplication(_IN REAL array_A[4], _IN REAL array_B[4], _OUT REAL array_C[4])
{
    ERROR_ID errorID = _ERROR_NO_ERROR;

    REAL aw, ax, ay, az;
    REAL bw, bx, by, bz;
    REAL cw, cx, cy, cz;

    if (array_A == NULL || array_B == NULL || array_C == NULL)
    {
        errorID = _ERROR_INPUT_PARAMETERS_ERROR;
        printf("矩阵运算错误代码：%u\n", errorID);
        return errorID;
    }

    aw = array_A[0];
    ax = array_A[1];
    ay = array_A[2];
    az = array_A[3];

    bw = array_B[0];
    bx = array_B[1];
    by = array_B[2];
    bz = array_B[3];

    cw = aw * bw - ax * bx - ay * by - az * bz;
    cx = aw * bx + ax * bw + ay * bz - az * by;
    cy = aw * by - ax * bz + ay * bw + az * bx;
    cz = aw * bz + ax * by - ay * bx + az * bw;

    array_C[0] = cw;
    array_C[1] = cx;
    array_C[2] = cy;
    array_C[3] = cz;

    return errorID;
}

ERROR_ID array_quaternion_normalize(_IN REAL array_Q[4], _OUT REAL array_Qn[4])
{
    ERROR_ID errorID = _ERROR_NO_ERROR;

    REAL q0, q1, q2, q3;
    REAL n2;
    REAL inv_n;

    if (array_Q == NULL || array_Qn == NULL)
    {
        errorID = _ERROR_INPUT_PARAMETERS_ERROR;
        printf("矩阵运算错误代码：%u\n", errorID);
        return errorID;
    }

    q0 = array_Q[0];
    q1 = array_Q[1];
    q2 = array_Q[2];
    q3 = array_Q[3];

    n2 = q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3;

    if (!isfinite(n2) || n2 <= 1.0e-24)
    {
        array_Qn[0] = 1.0;
        array_Qn[1] = 0.0;
        array_Qn[2] = 0.0;
        array_Qn[3] = 0.0;

        errorID = _ERROR_INPUT_PARAMETERS_ERROR;
        printf("矩阵运算错误代码：%u\n", errorID);
        return errorID;
    }

    inv_n = 1.0 / sqrt(n2);

    array_Qn[0] = q0 * inv_n;
    array_Qn[1] = q1 * inv_n;
    array_Qn[2] = q2 * inv_n;
    array_Qn[3] = q3 * inv_n;

    return errorID;
}

ERROR_ID array_quaternion_rotate_vector(_IN REAL array_Q[4], _IN REAL array_V[3], _OUT REAL array_Vout[3])
{
    ERROR_ID errorID = _ERROR_NO_ERROR;

    REAL w, x, y, z;
    REAL vx, vy, vz;
    REAL tx, ty, tz;
    REAL ox, oy, oz;

    if (array_Q == NULL || array_V == NULL || array_Vout == NULL)
    {
        errorID = _ERROR_INPUT_PARAMETERS_ERROR;
        printf("矩阵运算错误代码：%u\n", errorID);
        return errorID;
    }

    w = array_Q[0];
    x = array_Q[1];
    y = array_Q[2];
    z = array_Q[3];

    vx = array_V[0];
    vy = array_V[1];
    vz = array_V[2];

    tx = 2.0 * (y * vz - z * vy);
    ty = 2.0 * (z * vx - x * vz);
    tz = 2.0 * (x * vy - y * vx);

    ox = vx + w * tx + y * tz - z * ty;
    oy = vy + w * ty + z * tx - x * tz;
    oz = vz + w * tz + x * ty - y * tx;

    array_Vout[0] = ox;
    array_Vout[1] = oy;
    array_Vout[2] = oz;

    return errorID;
}

ERROR_ID array_eulerZYX_to_quaternion(_IN REAL array_EulerZYX[3], _OUT REAL array_Q[4])
{
    ERROR_ID errorID = _ERROR_NO_ERROR;

    REAL roll, pitch, yaw;
    REAL half_roll, half_pitch, half_yaw;
    REAL cr, sr, cp, sp, cy, sy;

    if (array_EulerZYX == NULL || array_Q == NULL)
    {
        errorID = _ERROR_INPUT_PARAMETERS_ERROR;
        printf("矩阵运算错误代码：%u\n", errorID);
        return errorID;
    }

    roll = array_EulerZYX[0];
    pitch = array_EulerZYX[1];
    yaw = array_EulerZYX[2];

    half_roll = 0.5 * roll;
    half_pitch = 0.5 * pitch;
    half_yaw = 0.5 * yaw;

    cr = cos(half_roll);
    sr = sin(half_roll);
    cp = cos(half_pitch);
    sp = sin(half_pitch);
    cy = cos(half_yaw);
    sy = sin(half_yaw);

    array_Q[0] = cr * cp * cy + sr * sp * sy;
    array_Q[1] = sr * cp * cy - cr * sp * sy;
    array_Q[2] = cr * sp * cy + sr * cp * sy;
    array_Q[3] = cr * cp * sy - sr * sp * cy;

    return errorID;
}

ERROR_ID array_quaternion_to_eulerZYX(_IN REAL array_Q[4], _OUT REAL array_EulerZYX[3])
{
    ERROR_ID errorID = _ERROR_NO_ERROR;

    REAL w, x, y, z;
    REAL sinr_cosp, cosr_cosp;
    REAL sinp;
    REAL siny_cosp, cosy_cosp;

    if (array_Q == NULL || array_EulerZYX == NULL)
    {
        errorID = _ERROR_INPUT_PARAMETERS_ERROR;
        printf("矩阵运算错误代码：%u\n", errorID);
        return errorID;
    }

    w = array_Q[0];
    x = array_Q[1];
    y = array_Q[2];
    z = array_Q[3];

    sinr_cosp = 2.0 * (w * x + y * z);
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    sinp = 2.0 * (w * y - z * x);
    siny_cosp = 2.0 * (w * z + x * y);
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z);

    array_EulerZYX[0] = atan2(sinr_cosp, cosr_cosp);

    if (sinp >= 1.0)
    {
        array_EulerZYX[1] = 0.5 * PI;
    }
    else if (sinp <= -1.0)
    {
        array_EulerZYX[1] = -0.5 * PI;
    }
    else
    {
        array_EulerZYX[1] = asin(sinp);
    }

    array_EulerZYX[2] = atan2(siny_cosp, cosy_cosp);

    return errorID;
}

ERROR_ID array_angle_wrap(_IN REAL array_A[], _OUT REAL array_B[], _IN INTEGER length)
{
    ERROR_ID errorID = _ERROR_NO_ERROR;

    INDEX i;
    REAL a, a_org;

    if (array_A == NULL || array_B == NULL || length <= 0)
    {
        errorID = _ERROR_INPUT_PARAMETERS_ERROR;
        printf("矩阵运算错误代码：%u\n", errorID);
        return errorID;
    }

    for (i = 0; i < length; i++)
    {
        a = array_A[i];

        if (a > PI || a < -PI)
        {
            a_org = a;

            a = fmod(a + PI, 2.0 * PI);

            if (a < 0.0)
            {
                a += 2.0 * PI;
            }

            a -= PI;

            if (a == -PI && a_org > 0.0)
            {
                a = PI;
            }
        }

        array_B[i] = a;
    }

    return errorID;
}

ERROR_ID array_angle_unwrap(_IN_OUT REAL array_A[], _IN_OUT REAL array_Last[], _IN_OUT REAL array_Turn[], _IN INTEGER length)
{
    ERROR_ID errorID = _ERROR_NO_ERROR;

    INDEX i;
    REAL a, last, turn, diff;

    if (array_A == NULL || array_Last == NULL || array_Turn == NULL || length <= 0)
    {
        errorID = _ERROR_INPUT_PARAMETERS_ERROR;
        printf("矩阵运算错误代码：%u\n", errorID);
        return errorID;
    }

    for (i = 0; i < length; i++)
    {
        a = array_A[i];
        last = array_Last[i];
        turn = array_Turn[i];

        diff = last - a;

        if (diff > PI)
        {
            turn += 1.0;
        }
        else if (diff < -PI)
        {
            turn -= 1.0;
        }

        array_Last[i] = a;
        array_Turn[i] = turn;
        array_A[i] = a + turn * 2.0 * PI;
    }

    return errorID;
}

ERROR_ID array_3x3_inverse(_IN REAL array_A[3][3], _OUT REAL array_invA[3][3])
{
    ERROR_ID errorID = _ERROR_NO_ERROR;

    REAL a00, a01, a02;
    REAL a10, a11, a12;
    REAL a20, a21, a22;

    REAL c00, c01, c02;
    REAL c10, c11, c12;
    REAL c20, c21, c22;

    REAL det;
    REAL invdet;

    REAL r00, r01, r02;
    REAL r10, r11, r12;
    REAL r20, r21, r22;

    if (array_A == NULL || array_invA == NULL)
    {
        errorID = _ERROR_INPUT_PARAMETERS_ERROR;
        printf("矩阵运算错误代码：%u\n", errorID);
        return errorID;
    }

    a00 = array_A[0][0];
    a01 = array_A[0][1];
    a02 = array_A[0][2];

    a10 = array_A[1][0];
    a11 = array_A[1][1];
    a12 = array_A[1][2];

    a20 = array_A[2][0];
    a21 = array_A[2][1];
    a22 = array_A[2][2];

    c00 = a11 * a22 - a12 * a21;
    c01 = -(a10 * a22 - a12 * a20);
    c02 = a10 * a21 - a11 * a20;

    c10 = -(a01 * a22 - a02 * a21);
    c11 = a00 * a22 - a02 * a20;
    c12 = -(a00 * a21 - a01 * a20);

    c20 = a01 * a12 - a02 * a11;
    c21 = -(a00 * a12 - a02 * a10);
    c22 = a00 * a11 - a01 * a10;

    det = a00 * c00 + a01 * c01 + a02 * c02;

    if (fabs(det) < 1.0e-12)
    {
        errorID = _ERROR_MATRIX_EQUATION_HAS_NO_SOLUTIONS;
        printf("矩阵运算错误代码：%u\n", errorID);
        return errorID;
    }

    invdet = 1.0 / det;

    r00 = c00 * invdet;
    r01 = c10 * invdet;
    r02 = c20 * invdet;

    r10 = c01 * invdet;
    r11 = c11 * invdet;
    r12 = c21 * invdet;

    r20 = c02 * invdet;
    r21 = c12 * invdet;
    r22 = c22 * invdet;

    array_invA[0][0] = r00;
    array_invA[0][1] = r01;
    array_invA[0][2] = r02;

    array_invA[1][0] = r10;
    array_invA[1][1] = r11;
    array_invA[1][2] = r12;

    array_invA[2][0] = r20;
    array_invA[2][1] = r21;
    array_invA[2][2] = r22;

    return errorID;
}

ERROR_ID array_3x3_multiply_vector(_IN REAL array_A[3][3], _IN REAL array_V[3], _OUT REAL array_B[3])
{
    ERROR_ID errorID = _ERROR_NO_ERROR;

    REAL v0, v1, v2;
    REAL b0, b1, b2;

    if (array_A == NULL || array_V == NULL || array_B == NULL)
    {
        errorID = _ERROR_INPUT_PARAMETERS_ERROR;
        printf("矩阵运算错误代码：%u\n", errorID);
        return errorID;
    }

    v0 = array_V[0];
    v1 = array_V[1];
    v2 = array_V[2];

    b0 = array_A[0][0] * v0 + array_A[0][1] * v1 + array_A[0][2] * v2;
    b1 = array_A[1][0] * v0 + array_A[1][1] * v1 + array_A[1][2] * v2;
    b2 = array_A[2][0] * v0 + array_A[2][1] * v1 + array_A[2][2] * v2;

    array_B[0] = b0;
    array_B[1] = b1;
    array_B[2] = b2;

    return errorID;
}