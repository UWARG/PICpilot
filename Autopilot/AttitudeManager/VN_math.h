/***************** (C) COPYRIGHT 2009 VectorNav Technologies *******************
* File Name          : VN_math.h
* Author             : John Brashear
* Version            : V1.0.0
* Date               : 09/26/2009
* Description        : This file contains all the function prototypes for the
*                    : math related functions of the firmware library.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING
* CUSTOMERS WITH EXAMPLE CODE IN ORDER TO SAVE THEM TIME. AS A RESULT,
* VECTORNAV SHALL NOT BE HELD LIABLE FOR ANY DIRECT, INDIRECT OR 
* CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE 
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE 
* CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __VN_MATH_H
#define __VN_MATH_H

/* Includes ------------------------------------------------------------------*/
#include "VN_type.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define VN_PI              3.14159265
#define VN_INV_MAX_SIZE    9

/* Exported macro ------------------------------------------------------------*/
#define VN_RAD2DEG(deg)    (deg*180.0f/VN_PI)
#define VN_DEG2RAD(rad)   (rad*VN_PI/180.0f)
#define VN_SWAP(a,b) {temp=(a);(a)=(b);(b)=temp;}                                   

/* Matrix macros -------------------------------------------------------------*/
/* The following macros are used to allow for the creation of statically
   allocated matricies.  The matrix representation provide here consists
   of three variables for each matrix. They are as follows:
   
   VariableName_data : The data for the given matrix
   VariableName_ptr  : A array of pointers to the first element in each row
   VariableName      : A pointer to the VariableName_ptr. In other words a
                       pointer to the array of pointers of the column vectors
                       of the array.
                       
   Example: Lets say you want to create a 3x3 matrix. You could do this using
            the following three lines of code:
            
   float  A_data[9] = {0};
   float  A_ptr[3] = {&A_data[0], &A_data[3], &A_data[6]};
   float  **A = A_ptr;
   
   Now if you want to use any of the elements in the array you can do so as
   you normally would if it was a standard two-dimensional array.
   
   // Example - set the diagonal elements to one
   A[0][0] = 1.0f;
   A[1][1] = 1.0f;
   A[2][2] = 1.0f;
   
   You might be asking why not just initialize the array as a standard two
   dimensional array such as:
   
   float A[3][3];
   
   The reason for this is well explained in page 20 of the book titled
   "Numerical Recipes in C - The Art of Scientific Computing" Second
   Edition. In a nutshell the standard two dimensional arrays of C are
   not efficient for numerical methods because of the way in which C 
   references the array indicies. Take for example the following line
   of code:
   
   A[i][j] = 1.0f;
   
   When this line is compiled to assembly it will result in something similar
   to the following line of code:
   
   *(A + 3*i + j) = 1.0f;
   
   This means "to the address of A add 3 times i plus j and return 
   the value thus addressed. This code results in two integer adds and one
   integer multiplication for every array index reference. An alternative 
   method that is more efficient is to have a pointer to an array of
   pointers given as **A.  This will result in code similar to: 
   
   *(*(A+i) + j) = 1.0f;
   
   This means "to the address of A add i, take the value thus addressed as
   a new address, add j to it, return the value addressed by this new
   address". In this version only two integer adds are required, which
   eliminates a integer multiplication.  This will result in more
   efficient embedded code. The penalty paid for this more efficient method
   is that it will require the storage of additional terms in the form of a
   array of pointers to the rows of the matrix. This is a slight
   inconvenience as it will require that the user to initialize three
   variables as opposed to one for each matrix.  The macros provided here 
   are designed to simplify this process. A matrix can be initialized using
   these macros as shown below:
   
   CreateMatrix(A, 3, 3, {1, 0, 0, 0, 1, 0, 0, 0, 1});
   
   This line will result in the folloing lines of code:
   
   static float A_data[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
   static float *A_ptr[3] = {&A_data[0], &A_data[3], &A_data[6]};
   static float       **A = A_ptr;
   
   The macros given below will only work for matricies with sizes less
   than 12 in each dimension.   These macros are only compatible with C99 and
   above versions of C. If you are using strict ANSI C then you will need to 
   write out the three lines shown above for each matrix you want to
   initialize.
   */
#if __STDC_VERSION__ >= 199901L  /* Determine if compiler is C99 compatible */
#define VN_MATRIXPtrList_1(VN_MATRIX, COLS)    {&VN_MATRIX##_data[0]}
#define VN_MATRIXPtrList_2(VN_MATRIX, COLS)    {&VN_MATRIX##_data[0], &VN_MATRIX##_data[1*(COLS)]}
#define VN_MATRIXPtrList_3(VN_MATRIX, COLS)    {&VN_MATRIX##_data[0], &VN_MATRIX##_data[1*(COLS)], &VN_MATRIX##_data[2*(COLS)]}
#define VN_MATRIXPtrList_4(VN_MATRIX, COLS)    {&VN_MATRIX##_data[0], &VN_MATRIX##_data[1*(COLS)], &VN_MATRIX##_data[2*(COLS)], &VN_MATRIX##_data[3*(COLS)]}
#define VN_MATRIXPtrList_5(VN_MATRIX, COLS)    {&VN_MATRIX##_data[0], &VN_MATRIX##_data[1*(COLS)], &VN_MATRIX##_data[2*(COLS)], &VN_MATRIX##_data[3*(COLS)], &VN_MATRIX##_data[4*(COLS)]}
#define VN_MATRIXPtrList_6(VN_MATRIX, COLS)    {&VN_MATRIX##_data[0], &VN_MATRIX##_data[1*(COLS)], &VN_MATRIX##_data[2*(COLS)], &VN_MATRIX##_data[3*(COLS)], &VN_MATRIX##_data[4*(COLS)], &VN_MATRIX##_data[5*(COLS)]}
#define VN_MATRIXPtrList_7(VN_MATRIX, COLS)    {&VN_MATRIX##_data[0], &VN_MATRIX##_data[1*(COLS)], &VN_MATRIX##_data[2*(COLS)], &VN_MATRIX##_data[3*(COLS)], &VN_MATRIX##_data[4*(COLS)], &VN_MATRIX##_data[5*(COLS)], &VN_MATRIX##_data[6*(COLS)]}
#define VN_MATRIXPtrList_8(VN_MATRIX, COLS)    {&VN_MATRIX##_data[0], &VN_MATRIX##_data[1*(COLS)], &VN_MATRIX##_data[2*(COLS)], &VN_MATRIX##_data[3*(COLS)], &VN_MATRIX##_data[4*(COLS)], &VN_MATRIX##_data[5*(COLS)], &VN_MATRIX##_data[6*(COLS)], &VN_MATRIX##_data[7*(COLS)]}
#define VN_MATRIXPtrList_9(VN_MATRIX, COLS)    {&VN_MATRIX##_data[0], &VN_MATRIX##_data[1*(COLS)], &VN_MATRIX##_data[2*(COLS)], &VN_MATRIX##_data[3*(COLS)], &VN_MATRIX##_data[4*(COLS)], &VN_MATRIX##_data[5*(COLS)], &VN_MATRIX##_data[6*(COLS)], &VN_MATRIX##_data[7*(COLS)], &VN_MATRIX##_data[8*(COLS)]}
#define VN_MATRIXPtrList_10(VN_MATRIX, COLS)  {&VN_MATRIX##_data[0], &VN_MATRIX##_data[1*(COLS)], &VN_MATRIX##_data[2*(COLS)], &VN_MATRIX##_data[3*(COLS)], &VN_MATRIX##_data[4*(COLS)], &VN_MATRIX##_data[5*(COLS)], &VN_MATRIX##_data[6*(COLS)], &VN_MATRIX##_data[7*(COLS)], &VN_MATRIX##_data[8*(COLS)], &VN_MATRIX##_data[9*(COLS)]}
#define VN_MATRIXPtrList_11(VN_MATRIX, COLS)  {&VN_MATRIX##_data[0], &VN_MATRIX##_data[1*(COLS)], &VN_MATRIX##_data[2*(COLS)], &VN_MATRIX##_data[3*(COLS)], &VN_MATRIX##_data[4*(COLS)], &VN_MATRIX##_data[5*(COLS)], &VN_MATRIX##_data[6*(COLS)], &VN_MATRIX##_data[7*(COLS)], &VN_MATRIX##_data[8*(COLS)], &VN_MATRIX##_data[9*(COLS)], &VN_MATRIX##_data[10(COLS)]}
#define VN_MATRIXPtrList_12(VN_MATRIX, COLS)  {&VN_MATRIX##_data[0], &VN_MATRIX##_data[1*(COLS)], &VN_MATRIX##_data[2*(COLS)], &VN_MATRIX##_data[3*(COLS)], &VN_MATRIX##_data[4*(COLS)], &VN_MATRIX##_data[5*(COLS)], &VN_MATRIX##_data[6*(COLS)], &VN_MATRIX##_data[7*(COLS)], &VN_MATRIX##_data[8*(COLS)], &VN_MATRIX##_data[9*(COLS)], &VN_MATRIX##_data[10(COLS)], &VN_MATRIX##_data[11(COLS)]}

#define VN_CreateMatrix(VN_MATRIX, ROWS, COLS, ...)    static float   VN_MATRIX##_data[(ROWS) * (COLS)] = __VA_ARGS__;                \
                                                      static float*  VN_MATRIX##_ptr[] = VN_MATRIXPtrList_##ROWS(VN_MATRIX, COLS);  \
                                                      static float** VN_MATRIX = VN_MATRIX##_ptr
#endif
/* Exported functions ------------------------------------------------------- */

void VN_CrossP(float *A, float *B, float *C);
void VN_VecAdd(float *A, float *B, unsigned long rows, float *C);
void VN_VecSub(float *A, float *B, unsigned long rows, float *C);
void VN_VecMultT(float *A, float *BT, unsigned long rows, float **C);
void VN_Identity(float scalar, unsigned long Arows, unsigned long Acols, float **A);
void VN_MatAdd(float **A, float **B, unsigned long Arows, unsigned long Acols, float **C);
void VN_MatSub(float **A, float **B, unsigned long Arows, unsigned long Acols, float **C);
void VN_MatMult(float **A, float **B, unsigned long Arows, unsigned long Acols, unsigned long Bcols, float **C);
void VN_MatMultMT(float **A, float **BT, unsigned long Arows, unsigned long Acols, unsigned long Brows, float **C);
void VN_MatScalarMult(double **A, double scalar, unsigned long Arows, unsigned long Acols, double **C);
void VN_MatVecMult(float **A, float *B, unsigned long Arows, unsigned long Acols, float *C);
void VN_MatTVecMult(float **A, float *B, unsigned long Arows, unsigned long Acols, float *C);
void VN_MatCopy(float **A, unsigned long nrows, unsigned long ncols, float **B);
void VN_MatInv(float **A, signed long n, float **B);
void VN_SkewMatrix(float *V, float **A);
void VN_Transpose(float **A, unsigned long Arows, unsigned long Acols, float **B);
float VN_Norm(float *A, unsigned long m);
void VN_Normalize(float *V1, unsigned long m, float *V2);
void VN_TriU2TriL(float **A, unsigned long rows);
void VN_Quat2DCM(float *q, float **A);
void VN_YPR2DCM(float *YPR, float **A);
void VN_MatZeros(float **A, unsigned long Arows, unsigned long Acols);
void VN_Quat2Euler121(float *q, float *Euler121);
void VN_Quat2Euler123(float *q, float *Euler123);
void VN_Quat2Euler131(float *q, float *Euler131);
void VN_Quat2Euler132(float *q, float *Euler132);
void VN_Quat2Euler212(float *q, float *Euler212);
void VN_Quat2Euler213(float *q, float *Euler213);
void VN_Quat2Euler231(float *q, float *Euler231);
void VN_Quat2Euler232(float *q, float *Euler232);
void VN_Quat2Euler312(float *q, float *Euler312);
void VN_Quat2Euler313(float *q, float *Euler313);
void VN_Quat2Euler321(float *q, float *Euler321);
void VN_Quat2Euler323(float *q, float *Euler323);
void VN_Quat2Gibbs(float *q, float *Gibb);
void VN_Quat2MRP(float *q, float *MRP);
void VN_Quat2PRV(float *q, float *PRV);
void VN_AddQuat(float *q1, float *q2, float *q3);
void VN_SubQuat(float *q1, float *q2, float *q3);
void VN_QuatKinematicDiffEq(float *q, float *rates, float *q_dot);
void VN_YPRKinematicDiffEq(float *YPR, float *rates, float *YPR_dot);


#endif /* __VN_MATH_H */

/******************* (C) COPYRIGHT 2009 VectorNav Technologies *****************
***********************************END OF FILE*********************************/


