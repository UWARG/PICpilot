/***************** (C) COPYRIGHT 2009 VectorNav Technologies *******************
* File Name          : VN_lib.h
* Author             : John Brashear
* Version            : V1.0.0
* Date               : 09/26/2009
* Description        : This file includes the device header files for the user
*                    : application.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING
* CUSTOMERS WITH EXAMPLE CODE IN ORDER TO SAVE THEM TIME. AS A RESULT,
* VECTORNAV SHALL NOT BE HELD LIABLE FOR ANY DIRECT, INDIRECT OR 
* CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE 
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE 
* CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __VN_LIB_H
#define __VN_LIB_H

/* Includes ------------------------------------------------------------------*/
#include "VN_type.h"
#include "VN_math.h"
#include "VN_user.h"

#ifdef _VN100
  #include "VN100.h"
#endif /*_VN100 */


/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Convert 4 bytes to a 32-bit word in the order given with b1 as the most significant byte*/
#define VN_BYTES2WORD(b1, b2, b3, b4) (((unsigned long)(b1) << 24) | ((unsigned long)(b2) << 16) | ((unsigned long)(b3) << 8) | (unsigned long)(b4))

/* Bit mask to get the 1st byte (most significant) out of a 32-bit word */
#define VN_BYTE1(word)    ((unsigned char)(((word) & 0xFF000000) >> 24))

/* Bit mask to get the 2nd byte out of a 32-bit word */
#define VN_BYTE2(word)    ((unsigned char)(((word) & 0x00FF0000) >> 16))

/* Bit mask to get the 3rd byte (most significant) out of a 32-bit word */
#define VN_BYTE3(word)    ((unsigned char)(((word) & 0x0000FF00) >> 8))

/* Bit mask to get the 1st byte (least significant) out of a 32-bit word */
#define VN_BYTE4(word)    ((unsigned char)((word) & 0x000000FF))

/* Bit mask to get the nth byte out of a 32-bit word where n=0 is most significant and n=3 is least significant */
#define VN_BYTE(word, n)   ((unsigned char)((word & (0x000000FF << (n*8))) >> (n*8)))

/* Exported functions ------------------------------------------------------- */





#endif /* __VN_LIB_H */

/******************* (C) COPYRIGHT 2009 VectorNav Technologies *****************
***********************************END OF FILE*********************************/
