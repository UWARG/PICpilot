/***************** (C) COPYRIGHT 2009 VectorNav Technologies *******************
* File Name          : VN_user.h
* Author             : John Brashear
* Version            : V1.0.0
* Date               : 09/26/2009
* Description        : This is the user configuration header file. It contains
*                    : setup parameters and all the function prototypes for the 
*                    : methods that are hardware specific. These methods need to
*                    : be modified by the user in the file VN_user.c to be
*                    : compatible with their specific hardware architecture.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING
* CUSTOMERS WITH EXAMPLE CODE IN ORDER TO SAVE THEM TIME. AS A RESULT,
* VECTORNAV SHALL NOT BE HELD LIABLE FOR ANY DIRECT, INDIRECT OR 
* CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE 
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE 
* CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __VN_USER_H
#define __VN_USER_H 

/* Includes ------------------------------------------------------------------*/
#include "VN_type.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Comment the lines below to disable the specific device inclusion */

/*********************************** VN-100 ***********************************/
#define _VN100


/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void VN_SPI_SetSS(unsigned char sensorID, VN_PinState state);
unsigned long VN_SPI_SendReceive(unsigned long data);
void VN_Delay(unsigned long delay_uS);

#endif /* __VN_USER_H */

/******************* (C) COPYRIGHT 2009 VectorNav Technologies *****************
***********************************END OF FILE*********************************/
