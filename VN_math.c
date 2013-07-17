/***************** (C) COPYRIGHT 2009 VectorNav Technologies *******************
* File Name          : VN100.c
* Author             : John Brashear
* Version            : V1.0.0
* Date               : 09/26/2009
* Description        : This file provides all of the firmware functions specific
*                    : to the VN100.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING
* CUSTOMERS WITH EXAMPLE CODE IN ORDER TO SAVE THEM TIME. AS A RESULT,
* VECTORNAV SHALL NOT BE HELD LIABLE FOR ANY DIRECT, INDIRECT OR 
* CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE 
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE 
* CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "VN_math.h"
#include <math.h>
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : VN_CrossP(float *A, float *B, float *C)
* Description    : Compute the cross product of vector A with vector B.
* Equation       : C = cross(A, B)                                        
* Input          : A -> 3x1 vector
*                  B -> 3x1 vector
*                  C -> 3x1 vector
* Output         : None
* Return         : None
*******************************************************************************/
void VN_CrossP(float *A, float *B, float *C){
  C[0] = A[1]*B[2]-A[2]*B[1];
  C[1] = A[2]*B[0]-A[0]*B[2];
  C[2] = A[0]*A[1]-A[1]*B[0];
}

/*******************************************************************************
* Function Name  : VN_VecAdd(float *A, float *B, unsigned long rows, float *C)
* Description    : Compute the addition of vector A with vector B.
* Equation       : C = A + B
* Input          : A -> vector with length given by rows
*                : B -> vector with length given by rows
*                : rows -> length of vector A, B, and C
* Output         : C -> result of vector addition
* Return         : None
*******************************************************************************/
void VN_VecAdd(float *A, float *B, unsigned long rows, float *C){
  unsigned long i;
  for(i=0;i<rows;i++) C[i] = A[i] + B[i];
}

/*******************************************************************************
* Function Name  : VN_VecSub(float *A, float *B, unsigned long rows, float *C)
* Description    : Compute the subtraction of vector A with vector B.
* Equation       : C = A - B                                        
* Input          : A -> vector with length given by rows
*                : B -> vector with length given by rows
*                : rows -> length of vector A, B, and C
* Output         : C -> result of vector subtraction
* Return         : None
*******************************************************************************/
void VN_VecSub(float *A, float *B, unsigned long rows, float *C){
  unsigned long i;
  for(i=0;i<rows;i++) C[i] = A[i] - B[i];
}

/*******************************************************************************
* Function Name  : VN_VecMultT(float *A, float *BT, unsigned long rows, float **C)
* Description    : Compute the multiplication of a vector with the transpose of
*                  another vector. The result will be a square matrix with the
*                  size of nxn where n=rows.
* Equation       : C = A * transpose(B)                                        
* Input          : A -> vector with length given by rows
*                : BT -> vector with length given by rows
*                : rows -> length of vector A and B
* Output         : C -> result of multiplication of A with the transpose of B
* Return         : None
*******************************************************************************/
void VN_VecMultT(float *A, float *BT, unsigned long rows, float **C){
  unsigned long i, j;
  for(i=0;i<rows; i++){
    for(j=0;j<rows; j++){
      C[i][j] = A[i]*BT[j];
    }
  }
}

/*******************************************************************************
* Function Name  : VN_Identity(float scalar, unsigned long Arows, unsigned long Acols, float *A)
* Description    : Create an zero matrix with the diagonal elements equal to
*                : the magnitude of scalar.
* Equation       : A = scalar * eye(Arows, Acols)                                        
* Input          : scalar -> desired magnitude of diagnonal terms
*                : Arows -> number of rows for the resulting matrix A
*                : Acols -> number of columns for the resulting matrix A 
* Output         : A -> resulting diagonal matrix
* Return         : None
*******************************************************************************/
void VN_Identity(float scalar, unsigned long Arows, unsigned long Acols, float **A){
  unsigned long i,j;
  
  for(i=0;i<Arows;i++){
    for(j=0;j<Acols;j++){
      if(i==j){
        A[i][j] = scalar;
      }else{
        A[i][j] = 0;
      }
    }
  }
}

/*******************************************************************************
* Function Name  : VN_MatAdd(float **A, float **B, unsigned long Arows, unsigned long Acols, float **C){
* Description    : Compute the addition of matrix A and B.
* Equation       : C = A + B                                        
* Input          : A -> Matrix A with size of Arows x Acols.
*                : B -> Matrix B with size of Arows x Acols.
*                : Arows -> Number of rows in matrix A and B.
*                : Acols -> Number of cols in matrix A and B. 
* Output         : C -> Result of matrix addition with size of Arows x Acols.
* Return         : None
*******************************************************************************/
void VN_MatAdd(float **A, float **B, unsigned long Arows, unsigned long Acols, float **C){
  unsigned long i,j;
  for(i=0;i<Arows;i++){
    for(j=0;j<Acols;j++){
      C[i][j] = A[i][j] + B[i][j];
    }
  }
}

/*******************************************************************************
* Function Name  : VN_MatSub(float **A, float **B, unsigned long Arows, unsigned long Acols, float **C){
* Description    : Compute the subtraction of matrix A and B.
* Equation       : C = A - B                                        
* Input          : A -> Matrix A with size of Arows x Acols.
*                : B -> Matrix B with size of Arows x Acols.
*                : Arows -> Number of rows in matrix A and B.
*                : Acols -> Number of cols in matrix A and B. 
* Output         : C -> Result of matrix subtraction with size of Arows x Acols.
* Return         : None
*******************************************************************************/
void VN_MatSub(float **A, float **B, unsigned long Arows, unsigned long Acols, float **C){
  unsigned long i,j;
  for(i=0;i<Arows;i++){
    for(j=0;j<Acols;j++) C[i][j] = A[i][j] - B[i][j];
  }
}

/*******************************************************************************
* Function Name  : VN_MatMult(float **A, float **B, unsigned long Arows, unsigned long Acols, unsigned long Bcols, float **C)
* Description    : Compute the multplication of matrix A and B.
* Equation       : C = A * B
* Input          : A -> Matrix A with size of Arows x Acols
*                : B -> Matrix B with size of Acols x Bcols
*                : Arows -> Number of rows in matrix A
*                : Acols -> Number of columns in matrix A
*                : Bcols -> Number of columns in matrix B
* Output         : C -> Result of the matrix multplication with size Arows x Bcols
* Return         : None
*******************************************************************************/
void VN_MatMult(float **A, float **B, unsigned long Arows, unsigned long Acols, unsigned long Bcols, float **C){
  unsigned long i,j,k;
  float temp;
  for(i=0;i<Arows;i++){
    for(j=0;j<Bcols;j++){
      temp = 0;
      for(k=Acols; k--; ){
        temp += A[i][k]*B[k][j];
      }
      C[i][j] = temp;
    }
  }
}

/*******************************************************************************
* Function Name  : VN_MatMultMT(float **A, float **BT, unsigned long Arows, unsigned long Acols, unsigned long Bcols, float **C)
* Description    : Compute the multplication of matrix A with the transpose of matrix B.
* Equation       : C = A * transpose(B)
* Input          : A -> Matrix A with size of Arows x Acols
*                : B -> Matrix B with size of Acols x Bcols
*                : Arows -> Number of rows in matrix A
*                : Acols -> Number of columns in matrix A
*                : Bcols -> Number of columns in matrix B
* Output         : C -> Result of the matrix multplication with size Arows x Bcols
* Return         : None
*******************************************************************************/
void VN_MatMultMT(float **A, float **BT, unsigned long Arows, unsigned long Acols, unsigned long Brows, float **C){
  unsigned long i,j,k;
  float temp;
  for(i=0;i<Arows;i++){
    for(j=0;j<Brows;j++){
      temp = 0;
      for(k=Acols; k--; ){
        temp += A[i][k]*BT[j][k];
      }
      C[i][j] = temp;
    }
  }
}

/*******************************************************************************
* Function Name  : VN_MatScalarMult(double **A, double scalar, unsigned long Arows, unsigned long Acols, double **C)
* Description    : Compute the multplication of a scalar times a matrix.
* Equation       : C = scalar * A                                        
* Input          : scalar -> The scalar term
*                : A -> The matrix with the size Arows x Acols
*                : Arows -> The number of rows in the matrix A
*                : Acols -> The number of columns in the matrix B
* Output         : C -> The result of the operation scalar * A
* Return         : None
*******************************************************************************/
void VN_MatScalarMult(double **A, double scalar, unsigned long Arows, unsigned long Acols, double **C){
  unsigned long i,j;
  for(i=0;i<Arows;i++){
    for(j=0;j<Acols;j++){
      C[i][j] = scalar*A[i][j];
    }
  }
}

/*******************************************************************************
* Function Name  : VN_MatVecMult(float **A, float *B, unsigned long Arows, unsigned long Acols, float *C)
* Description    : Compute the multiplication of matrix A with vector B.
* Equation       : C = A * B                                        
* Input          : A -> Matrix with size Arows x Acols
*                : B -> Column vector with size Acols x 1
*                : Arows -> Number of rows in matrix A
*                : Acols -> Number of columns in matrix A
* Output         : C -> Matrix result with size Arows x Acols
* Return         : None
*******************************************************************************/
void VN_MatVecMult(float **A, float *B, unsigned long Arows, unsigned long Acols, float *C){
  unsigned long i,k;
  for(i=0;i<Arows;i++){
    C[i] = 0;
    for(k=0;k<Acols;k++) C[i] += A[i][k]*B[k];
  }
}

/*******************************************************************************
* Function Name  : VN_MatTVecMult(float **A, float *B, unsigned long Arows, unsigned long Acols, float *C)
* Description    : Multiply the transpose of the matrix A by the vector B.
* Equation       : C = transpose(A) * B                                        
* Input          : A -> Matrix with size Arows x Acols
*                : B -> Column vector with size Arows x 1
*                : Arows -> Number of rows in matrix A
*                : Acols -> Number of columns in matrix A
* Output         : C -> Matrix result with size Acols x Arows
* Return         : None
*******************************************************************************/
void VN_MatTVecMult(float **A, float *B, unsigned long Arows, unsigned long Acols, float *C){
  unsigned long i,k;
  for(i=0;i<Arows;i++){
    C[i] = 0;
    for(k=0;k<Acols;k++) C[i] += A[k][i]*B[k];
  }
}

/*******************************************************************************
* Function Name  : VN_MatCopy(float **A, unsigned long nrows, unsigned long ncols, float **B)
* Description    : Copy the values from one matrix to another.
* Equation       : B = A                                        
* Input          : A -> Matrix with size nrows x ncols
*                : nrows -> number of rows in matrix A
*                : ncols -> number of columns in matrix A
* Output         : B -> Resulting matrix with size nrows x ncols
* Return         : None
*******************************************************************************/
void VN_MatCopy(float **A, unsigned long nrows, unsigned long ncols, float **B){
  unsigned long i,j;
  for(i=0;i<nrows;i++){
    for(j=0;j<ncols;j++) B[i][j] = A[i][j];
  }
}

/*******************************************************************************
* Function Name  : VN_MatInv(float **A, s32 n, float **B)
* Description    : Compute the matrix inverse of A.
* Equation       : B = inv(A)                                        
* Input          : A -> Matrix with size n x n
*                : n -> Number of rows and columns in matrix A
* Output         : B -> Matrix result with size n x n
* Return         : None
*******************************************************************************/
void VN_MatInv(float **A, signed long n, float **B)
{
  int indxc[VN_INV_MAX_SIZE], indxr[VN_INV_MAX_SIZE], ipiv[VN_INV_MAX_SIZE];
  int i,icol,irow,j,k,l,ll;
  float big,dum,pivinv,temp;
  
  irow = 0;
  icol = 0;

  VN_MatCopy(A,n,n,B);

  for (j=0;j<n;j++) ipiv[j]=0;
  for (i=0;i<n;i++){
      big=0.0;
      for (j=0;j<n;j++)
        if (ipiv[j] != 1)
          for (k=0;k<n;k++) {
            if (ipiv[k] == 0) {
              if (fabs(B[j][k]) >= big) {
                big=(float)fabsf(B[j][k]);
                irow=j;
                icol=k;
              }
            }
          }
        ++(ipiv[icol]);
        if (irow != icol) {
          for (l=0;l<n;l++) VN_SWAP(B[irow][l],B[icol][l])
        }
        indxr[i]=irow;
        indxc[i]=icol;
        pivinv=1.0f/B[icol][icol];
        B[icol][icol]=1.0;
        for (l=0;l<n;l++) B[icol][l] *= pivinv;
          for (ll=0;ll<n;ll++)
            if (ll != icol) {
              dum=B[ll][icol];
              B[ll][icol]=0.0;
              for (l=0;l<n;l++) B[ll][l] -= B[icol][l]*dum;
            }
        }
        for (l=n-1;l>=0;l--) {
          if (indxr[l] != indxc[l])
            for (k=0;k<n;k++)
              VN_SWAP(B[k][indxr[l]],B[k][indxc[l]]);
        }
}

/*******************************************************************************
* Function Name  : VN_SkewMatrix(float *V, float **A)
* Description    : Compute the matrix cross product of vector V. This operation
*                : converts a vector into a matrix that when multplied by
*                : another vector would give the result of a cross product
*                : operation between the two vectors.
* Equation       : A = skew(V)  where A*b = cross(V,b)  if b is a vector same
*                : size as V.                                        
* Input          : V -> Vector of size 3x1 to perform operation on.
* Output         : A -> Resulting matrix with size 3x3.
* Return         : None
*******************************************************************************/
void VN_SkewMatrix(float *V, float **A){
  A[0][0] = 0;
  A[0][1] = -V[2];
  A[0][2] = V[1];
  A[1][0] = V[2];
  A[1][1] = 0;
  A[1][2] = -V[0];
  A[2][0] = -V[1];
  A[2][1] = V[0];
  A[2][2] = 0;
}

/*******************************************************************************
* Function Name  : VN_Transpose(float **A, unsigned long Arows, unsigned long Acols, float **B)
* Description    : Calculate the transpose of matrix A.
* Equation       : B = transpose(A)                                        
* Input          : A -> Matrix with size Arows x Acols
*                : Arows -> Number of rows in matrix A
*                : Acols -> Number of columns in matrix A
* Output         : B -> Resulting matrix with size Acols x Arows
* Return         : None
*******************************************************************************/
void VN_Transpose(float **A, unsigned long Arows, unsigned long Acols, float **B){
  unsigned long i,j;
  for(i=0;i<Arows;i++){
    for(j=0;j<Acols;j++){
      B[j][i] = A[i][j];
    }
  }
}

/*******************************************************************************
* Function Name  : VN_Norm(float *A, unsigned long m)
* Description    : Compute the length of the given vector with size m x 1
* Equation       : norm(A)                                        
* Input          : A -> Vector to compute the length of with size m x 1
*                : m -> Number of terms in the vector A.
* Output         : None
* Return         : The length of the given vector with size m x 1
*******************************************************************************/
float VN_Norm(float *A, unsigned long m){
  float nrm = 0;
  unsigned long i;
  for(i=0; i < m; i++){
    nrm += A[i]*A[i];
  }
  return sqrtf(nrm);  
}

/*******************************************************************************
* Function Name  : VN_Normalize(float *V1, unsigned long m, float *V2)
* Description    : Compute the unit normal vector with the direction given by
*                : vector V1.
* Equation       : V2 = V1 ./ norm(V1)                                        
* Input          : V1 -> Vector with size m x 1
*                : m -> Number of terms in the vector V1.
* Output         : V2 -> Unit vector with the size of m x 1.
* Return         : None
*******************************************************************************/
void VN_Normalize(float *V1, unsigned long m, float *V2){
  float nrm = VN_Norm(V1, m);
  unsigned long i;
  for(i=0; i < m; i++){
    V2[i] = V1[i] / nrm;
  }
}

/*******************************************************************************
* Function Name  : VN_TriU2TriL(float **A, unsigned long rows)
* Description    : Copys the terms in the upper right triangular portion of
*                : matrix A into the lower left portion of A such that A becomes
*                : a symmetric matrix.
* Equation       : B = triu(A) + triu(A)' - diag(diag(A))                                         
* Input          : A -> Square matrix with size rows x rows
*                : rows -> Number of rows in square matrix B
* Output         : A -> Square symmetric matrix A
* Return         : None
*******************************************************************************/
void VN_TriU2TriL(float **A, unsigned long rows){
  unsigned long i,j;
  for(i=0;i<rows;i++){
    for(j=i+1;j<rows;j++){
      A[j][i] = A[i][j];
    }
  }
}

/*******************************************************************************
* Function Name  : VN_quat2DCM(float *q, float **A)
* Description    : Convert a quaternion into to a directional cosine matrix.
* Equation       : A = quat2dcm(q)                                        
* Input          : q -> Quaternion attitude
* Output         : A -> Directional cosine matrix (3x3)
* Return         : None
*******************************************************************************/
void VN_Quat2DCM(float *q, float **A){

  /* Temporary variables */
  float t[12];
  
  t[0] = q[0]*q[0];
  t[1] = q[1]*q[1];
  t[2] = q[2]*q[2];
  t[3] = q[3]*q[3];
  t[4] = q[0]*q[1]*2.0;
  t[5] = q[0]*q[2]*2.0;
  t[6] = q[0]*q[3]*2.0;
  t[7] = q[1]*q[2]*2.0;
  t[8] = q[1]*q[3]*2.0;
  t[9] = q[2]*q[3]*2.0;
  t[10]= t[0]-t[1];
  t[11]= t[3]-t[2];
  A[0][0] =  t[10]+t[11];
  A[1][1] = -t[10]+t[11];
  A[2][2] = -t[0]-t[1]+t[2]+t[3];
  A[0][1] = t[4]+t[9];
  A[1][0] = t[4]-t[9];
  A[1][2] =  t[6]+t[7];
  A[2][1] = -t[6]+t[7];
  A[0][2] = t[5]-t[8];
  A[2][0] = t[5]+t[8];
}

/*******************************************************************************
* Function Name  : VN_YPR2DCM(float *YPR, float **A)
* Description    : Convert the given yaw, pitch, and roll into a directional
*                : cosine matrix.
* Equation       : A = ANGLE2DCM(YPR[0], YPR[1], YPR[2], 'ZYX')                                         
* Input          : YPR -> Yaw, pitch, roll as a 3x1 vector
* Output         : A -> Directional cosine matrix
* Return         : None
*******************************************************************************/
void VN_YPR2DCM(float *YPR, float **A){

  /* Temporary variables */
  float t[12];
  
  t[0] = sinf(YPR[0]);
  t[1] = cosf(YPR[0]);
  t[2] = sinf(YPR[1]);
  t[3] = cosf(YPR[1]);
  t[4] = sinf(YPR[2]);
  t[5] = cosf(YPR[2]);
  t[6] = t[4]*t[2];
  t[7] = t[5]*t[2];
  A[0][0] = t[3]*t[1];
  A[0][1] = t[3]*t[0];
  A[0][2] = -t[2];
  A[1][0] = t[6]*t[1]-t[5]*t[0];
  A[1][1] = t[6]*t[0]+t[5]*t[1];
  A[1][2] = t[4]*t[3];
  A[2][0] = t[7]*t[1]+t[4]*t[0];
  A[2][1] = t[7]*t[0]-t[4]*t[1];
  A[2][2] = t[5]*t[3];
}

/*******************************************************************************
* Function Name  : VN_MatZeros(float **A, unsigned long Arows, unsigned long Acols)
* Description    : Sets all elements of matrix A equal to zero.
* Equation       : A = zeros(Arows, Acols)                                        
* Input          : A -> Matrix with size of Arows x Acols
*                : Arows -> Number of rows in matrix A
*                : Acols -> Number of columns in matrix A
* Output         : Matrix A with all elements set to zero
* Return         : None
*******************************************************************************/
void VN_MatZeros(float **A, unsigned long Arows, unsigned long Acols)
{
  unsigned long i,j;
  
  for(i=0;i<Arows;i++){
    for(j=0;j<Acols;j++){
      A[i][j] = 0.0;
    }
  }
}

/*******************************************************************************
* Function Name  : VN_Quat2Euler121(float *q, float *Euler121)
* Description    : Convert a quaternion attitude representation to an Euler
*                : angle 1,2,1 set.
* Equation       : Euler121 = quat2angle(q, 'XYX')                                         
* Input          : q -> Quaternion 4x1 vector
* Output         : Euler angles
* Return         : None
*******************************************************************************/
void VN_Quat2Euler121(float *q, float *Euler121){

float t1, t2;

t1 = atan2f(q[2],q[1]);
t2 = atan2f(q[0],q[3]);

Euler121[0] = t1+t2;
Euler121[1] = 2*acosf(sqrtf(q[3]*q[3]+q[0]*q[0]));
Euler121[2] = t2-t1;
}

/*******************************************************************************
* Function Name  : VN_Quat2Euler123(float *q, float *Euler123)
* Description    : Convert a quaternion attitude representation to an Euler
*                : angle 1,2,3 set.                                         
* Equation       : Euler123 = quat2angle(q, 'XYZ')                                         
* Input          : q -> Quaternion 4x1 vector
* Output         : Euler angles
* Return         : None
*******************************************************************************/
void VN_Quat2Euler123(float *q, float *Euler123){

float q0, q1, q2, q3;

q0 = q[3];
q1 = q[0];
q2 = q[1];
q3 = q[2];

Euler123[0] = atan2f(-2*(q2*q3-q0*q1),q0*q0-q1*q1-q2*q2+q3*q3);
Euler123[1] = asinf(2*(q1*q3 + q0*q2));
Euler123[2]= atan2f(-2*(q1*q2-q0*q3),q0*q0+q1*q1-q2*q2-q3*q3);
}

/*******************************************************************************
* Function Name  : VN_Quat2Euler131(float *q, float *Euler131)
* Description    : Convert a quaternion attitude representation to an Euler
*                : angle 1,3,1 set.                                         
* Equation       : Euler131 = quat2angle(q, 'XZX')                                         
* Input          : q -> Quaternion 4x1 vector
* Output         : Euler angles
* Return         : None
*******************************************************************************/
void VN_Quat2Euler131(float *q, float *Euler131){

float t1, t2;

t1 = atan2f(q[1],q[2]);
t2 = atan2f(q[0],q[3]);

Euler131[0] = t2-t1;
Euler131[1] = 2*acosf(sqrtf(q[3]*q[3]+q[0]*q[0]));
Euler131[2] = t2+t1;
}

/*******************************************************************************
* Function Name  : VN_Quat2Euler132(float *q, float *Euler132)
* Description    : Convert a quaternion attitude representation to an Euler
*                : angle 1,3,2 set.                                         
* Equation       : Euler132 = quat2angle(q, 'XZY')                                         
* Input          : q -> Quaternion 4x1 vector
* Output         : Euler angles
* Return         : None
*******************************************************************************/
void VN_Quat2Euler132(float *q, float *Euler132){

float q0, q1, q2, q3;

q0 = q[3];
q1 = q[0];
q2 = q[1];
q3 = q[2];

Euler132[0] = atan2f(2*(q2*q3+q0*q1),q0*q0-q1*q1+q2*q2-q3*q3);
Euler132[1] = asinf(-2*(q1*q2-q0*q3));
Euler132[2]= atan2f(2*(q1*q3 + q0*q2),q0*q0+q1*q1-q2*q2-q3*q3);
}

/*******************************************************************************
* Function Name  : VN_Quat2Euler212(float *q, float *Euler212)
* Description    : Convert a quaternion attitude representation to an Euler
*                : angle 2,1,2 set.                                         
* Equation       : Euler212 = quat2angle(q, 'YXY')                                         
* Input          : q -> Quaternion 4x1 vector
* Output         : Euler angles
* Return         : None
*******************************************************************************/
void VN_Quat2Euler212(float *q, float *Euler212){

float t1, t2;

t1 = atan2f(q[2],q[0]);
t2 = atan2f(q[1],q[3]);

Euler212[0] = t2-t1;
Euler212[1] = 2*acosf(sqrtf(q[3]*q[3]+q[1]*q[1]));
Euler212[2] = t2+t1;
}

/*******************************************************************************
* Function Name  : VN_Quat2Euler213(float *q, float *Euler213)
* Description    : Convert a quaternion attitude representation to an Euler
*                : angle 2,1,3 set.                                         
* Equation       : Euler213 = quat2angle(q, 'YXZ')                                         
* Input          : q -> Quaternion 4x1 vector
* Output         : Euler angles
* Return         : None
*******************************************************************************/
void VN_Quat2Euler213(float *q, float *Euler213){

float q0, q1, q2, q3;

q0 = q[3];
q1 = q[0];
q2 = q[1];
q3 = q[2];

Euler213[0] = atan2f(2*(q1*q3 + q0*q2),q0*q0-q1*q1-q2*q2+q3*q3);
Euler213[1] = asinf(-2*(q2*q3-q0*q1));
Euler213[2]= atan2f(2*(q1*q2+q0*q3),q0*q0-q1*q1+q2*q2-q3*q3);
}

/*******************************************************************************
* Function Name  : VN_Quat2Euler231(float *q, float *Euler231)
* Description    : Convert a quaternion attitude representation to an Euler
*                : angle 2,3,1 set.                                         
* Equation       : Euler231 = quat2angle(q, 'YZX')                                         
* Input          : q -> Quaternion 4x1 vector
* Output         : Euler angles
* Return         : None
*******************************************************************************/
void VN_Quat2Euler231(float *q, float *Euler231){

float q0, q1, q2, q3;

q0 = q[3];
q1 = q[0];
q2 = q[1];
q3 = q[2];

Euler231[0] = atan2f(-2*(q1*q3-q0*q2), q0*q0+q1*q1-q2*q2-q3*q3);
Euler231[1] = asinf(2*(q1*q2+q0*q3));
Euler231[2]= atan2f(-2*(q2*q3-q0*q1),q0*q0-q1*q1+q2*q2-q3*q3);
}

/*******************************************************************************
* Function Name  : VN_Quat2Euler232(float *q, float *Euler232)
* Description    : Convert a quaternion attitude representation to an Euler
*                : angle 2,3,2 set.                                         
* Equation       : Euler232 = quat2angle(q, 'YZY')                                         
* Input          : q -> Quaternion 4x1 vector
* Output         : Euler angles
* Return         : None
*******************************************************************************/
void VN_Quat2Euler232(float *q, float *Euler232){

float t1, t2;

t1 = atan2f(q[0],q[2]);
t2 = atan2f(q[1],q[3]);

Euler232[0] = t1+t2;
Euler232[1] = 2*acosf(sqrtf(q[3]*q[3]+q[1]*q[1]));
Euler232[2] = t2-t1;
}

/*******************************************************************************
* Function Name  : VN_Quat2Euler312(float *q, float *Euler312)
* Description    : Convert a quaternion attitude representation to an Euler
*                : angle 3,1,2 set.                                         
* Equation       : Euler312 = quat2angle(q, 'ZXY')                                         
* Input          : q -> Quaternion 4x1 vector
* Output         : Euler angles
* Return         : None
*******************************************************************************/
void VN_Quat2Euler312(float *q, float *Euler312){

float q0, q1, q2, q3;

q0 = q[3];
q1 = q[0];
q2 = q[1];
q3 = q[2];

Euler312[0] = atan2f(-2*(q1*q2-q0*q3),q0*q0-q1*q1+q2*q2-q3*q3);
Euler312[1] = asinf(2*(q2*q3+q0*q1));
Euler312[2]= atan2f(-2*(q1*q3-q0*q2),q0*q0-q1*q1-q2*q2+q3*q3);
}

/*******************************************************************************
* Function Name  : VN_Quat2Euler313(float *q, float *Euler313)
* Description    : Convert a quaternion attitude representation to an Euler
*                : angle 3,1,3 set.                                         
* Equation       : Euler313 = quat2angle(q, 'ZXZ')                                         
* Input          : q -> Quaternion 4x1 vector
* Output         : Euler angles
* Return         : None
*******************************************************************************/
void VN_Quat2Euler313(float *q, float *Euler313){

float t1, t2;

t1 = atan2f(q[1],q[0]);
t2 = atan2f(q[2],q[3]);

Euler313[0] = t1+t2;
Euler313[1] = 2*acosf(sqrtf(q[3]*q[3]+q[2]*q[2]));
Euler313[2] = t2-t1;
}

/*******************************************************************************
* Function Name  : VN_Quat2Euler321(float *q, float *Euler321)
* Description    : Convert a quaternion attitude representation to an Euler
*                : angle 3,2,1 set.                                         
* Equation       : Euler321 = quat2angle(q, 'ZYX')                                         
* Input          : q -> Quaternion 4x1 vector
* Output         : Euler angles
* Return         : None
*******************************************************************************/
void VN_Quat2Euler321(float *q, float *Euler321){

float q0, q1, q2, q3;

q0 = q[3];
q1 = q[0];
q2 = q[1];
q3 = q[2];

Euler321[0] = atan2f(2*(q1*q2+q0*q3),q0*q0+q1*q1-q2*q2-q3*q3);
Euler321[1] = asinf(-2*(q1*q3-q0*q2));
Euler321[2]= atan2f(2*(q2*q3+q0*q1),q0*q0-q1*q1-q2*q2+q3*q3);
}

/*******************************************************************************
* Function Name  : VN_Quat2Euler323(float *q, float *Euler323)
* Description    : Convert a quaternion attitude representation to an Euler
*                : angle 3,2,3 set.                                         
* Equation       : Euler323 = quat2angle(q, 'ZYZ')                                         
* Input          : q -> Quaternion 4x1 vector
* Output         : Euler angles
* Return         : None
*******************************************************************************/
void VN_Quat2Euler323(float *q, float *Euler323){

float t1, t2;

t1 = atan2f(q[0],q[1]);
t2 = atan2f(q[2],q[3]);

Euler323[0] = t2-t1;
Euler323[1] = 2*acosf(sqrtf(q[3]*q[3]+q[2]*q[2]));
Euler323[2] = t2+t1;
}

/*******************************************************************************
* Function Name  : VN_Quat2Gibbs(float *q, float *Gibb)
* Description    : Convert a quaternion attitude representation to the Gibbs
*                : angle representation.                                         
* Input          : q -> Quaternion 4x1 vector
* Output         : Gibb -> Gibbs vector
* Return         : None
*******************************************************************************/
void VN_Quat2Gibbs(float *q, float *Gibb){
Gibb[0] = q[0]/q[3];
Gibb[1] = q[1]/q[3];
Gibb[2] = q[2]/q[3];
}

/*******************************************************************************
* Function Name  : VN_Quat2MRP(float *q, float *MRP)
* Description    : Convert a quaternion attitude representation to an Modified
*                : Rodrigues Parameters representation.                                         
* Input          : q -> Quaternion 4x1 vector
* Output         : MRP -> Modified Rodrigues Parameters
* Return         : None
*******************************************************************************/
void VN_Quat2MRP(float *q, float *MRP){
MRP[0] = q[0]/(1+q[3]);
MRP[1] = q[1]/(1+q[3]);
MRP[2] = q[2]/(1+q[3]);
}

/*******************************************************************************
* Function Name  : VN_Quat2PRV(float *q, float *PRV)
* Description    : Convert a quaternion attitude representation to the principal
*                : rotatin vector.                                         
* Input          : q -> Quaternion 4x1 vector
* Output         : PRV -> Principal rotation vector
* Return         : None
*******************************************************************************/
void VN_Quat2PRV(float *q, float *PRV){
float p, sp;
p = 2*acosf(q[3]);
sp = sinf(p/2);
PRV[0] = q[0]/sp*p;
PRV[1] = q[1]/sp*p;
PRV[2] = q[2]/sp*p;
}

/*******************************************************************************
* Function Name  : VN_AddQuat(float *q1, float *q2, float *q3)
* Description    : VN_AddQuat provides the quaternion which corresponds to
*                  performing two successive rotations from q1 and q2.
* Input          : q1 -> First quaternion
*                : q2 -> Second quaternion
* Output         : q3 -> Combined sucessive rotation of q1 and q2
* Return         : None
*******************************************************************************/
void VN_AddQuat(float *q1, float *q2, float *q3){
q3[3] = q2[3]*q1[3]-q2[0]*q1[0]-q2[1]*q1[1]-q2[2]*q1[2];
q3[0] = q2[0]*q1[3]+q2[3]*q1[0]+q2[2]*q1[1]-q2[1]*q1[2];
q3[1] = q2[1]*q1[3]-q2[2]*q1[0]+q2[3]*q1[1]+q2[0]*q1[2];
q3[2] = q2[2]*q1[3]+q2[1]*q1[0]-q2[0]*q1[1]+q2[3]*q1[2];
}

/*******************************************************************************
* Function Name  : VN_SubQuat(float *q1, float *q2, float *q3)
* Description    : VN_SubQuat provides the quaternion which cooresponds to
*                  the relative rotation from q2 to q1.                                    
* Input          : q1 -> First quaternion
*                : q2 -> Second quaternion
* Output         : q3 -> Relative rotation from q2 to q1
* Return         : None
*******************************************************************************/
void VN_SubQuat(float *q1, float *q2, float *q3){
q3[3] = q2[3]*q1[3]+q2[0]*q1[0]+q2[1]*q1[1]+q2[2]*q1[2];
q3[0] = -q2[0]*q1[3]+q2[3]*q1[0]+q2[2]*q1[1]-q2[1]*q1[2];
q3[1] = -q2[1]*q1[3]-q2[2]*q1[0]+q2[3]*q1[1]+q2[0]*q1[2];
q3[2] = -q2[2]*q1[3]+q2[1]*q1[0]-q2[0]*q1[1]+q2[3]*q1[2];
}

/*******************************************************************************
* Function Name  : VN_QuatKinematicDiffEq(float *q, float *rates, float *q_dot)
* Description    : Computes the time rate of change of the quaternion paramters
*                  as a function of the angular rates. You can use this function
*                  if you need to determine how the quaternion parameters are
*                  instantaniously changing as a function of time.                                        
* Input          : q -> Current attitude quaternion
*                  rates -> angular rates [rad/s]
* Output         : q_dot -> derivative of q
* Return         : None
*******************************************************************************/
void VN_QuatKinematicDiffEq(float *q, float *rates, float *q_dot){
q_dot[0] = 0.5f * ( q[3]*rates[0]-q[2]*rates[1]+q[1]*rates[2]);
q_dot[1] = 0.5f * ( q[2]*rates[0]+q[3]*rates[1]-q[0]*rates[2]);
q_dot[2] = 0.5f * (-q[1]*rates[0]+q[0]*rates[1]+q[3]*rates[2]);
q_dot[3] = 0.5f * (-q[0]*rates[0]-q[1]*rates[1]-q[2]*rates[2]);
}

/*******************************************************************************
* Function Name  : VN_YPRKinematicDiffEq(float *YPR, float *rates, float *YPR_dot)
* Description    : Computes the time rate of change of the 321 Euler angles
*                  (yaw, pitch, roll) as a function of the angular rates. You
*                  can use this function if you need to determine how the Euler
*                  angles are instantaniously changing as a function of time.                                        
* Input          : YPR -> Yaw, Pitch, Roll angles [rad]
*                  rates -> angular rates [rad/s]
* Output         : YPR_dot -> rate of change of yaw, pitch, roll [rad/s]
* Return         : None
*******************************************************************************/
void VN_YPRKinematicDiffEq(float *YPR, float *rates, float *YPR_dot){
YPR_dot[0] =          (sin(YPR[2])/cos(YPR[1]))*rates[1] + (cos(YPR[2])/cos(YPR[1]))*rates[2];
YPR_dot[1] =                        cos(YPR[2])*rates[1] -               sin(YPR[2])*rates[2];
YPR_dot[2] = rates[0] + sin(YPR[2])*tan(YPR[1])*rates[1] +   cos(YPR[2])*tan(YPR[1])*rates[2];
}






