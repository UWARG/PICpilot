/***************** (C) COPYRIGHT 2009 VectorNav Technologies *******************
* File Name          : VN100.h
* Author             : John Brashear
* Version            : V1.0.0
* Date               : 09/26/2009
* Description        : This file contains all the function prototypes for
*                    : the firmware specific to the VN-100.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING
* CUSTOMERS WITH EXAMPLE CODE IN ORDER TO SAVE THEM TIME. AS A RESULT,
* VECTORNAV SHALL NOT BE HELD LIABLE FOR ANY DIRECT, INDIRECT OR 
* CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE 
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE 
* CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __VN100_H
#define __VN100_H

/* Includes ------------------------------------------------------------------*/
#include "VN_type.h"
#include "VN_lib.h"

/* Exported constants --------------------------------------------------------*/

/* VN-100 Registers */
#define     VN100_REG_MODEL     1
#define     VN100_REG_HWREV     2
#define     VN100_REG_SN        3
#define     VN100_REG_FWVER     4
#define     VN100_REG_SBAUD     5
#define     VN100_REG_ADOR      6
#define     VN100_REG_ADOF      7
#define     VN100_REG_YPR       8
#define     VN100_REG_QTN       9
#define     VN100_REG_QTM       10
#define     VN100_REG_QTA       11
#define     VN100_REG_QTR       12
#define     VN100_REG_QMA       13
#define     VN100_REG_QAR       14
#define     VN100_REG_QMR       15
#define     VN100_REG_DCM       16
#define     VN100_REG_MAG       17
#define     VN100_REG_ACC       18
#define     VN100_REG_GYR       19
#define     VN100_REG_MAR       20
#define     VN100_REG_REF       21
#define     VN100_REG_SIG       22
#define     VN100_REG_HSI       23
#define     VN100_REG_ATP       24
#define     VN100_REG_ACT       25
#define     VN100_REG_RFR       26
#define     VN100_REG_YMR       27
#define     VN100_REG_ACG       28
  
#define     VN100_REG_RAW         251
#define     VN100_REG_CMV       252
#define     VN100_REG_STV         253
#define     VN100_REG_COV       254
#define     VN100_REG_CAL         255

/* SPI Buffer size */
#define     VN100_SPI_BUFFER_SIZE    93

/* Exported types ------------------------------------------------------------*/
/* Command IDs */
typedef enum VN100_CmdID
{
  VN100_CmdID_ReadRegister             =  0x01,
  VN100_CmdID_WriteRegister            =  0x02,
  VN100_CmdID_WriteSettings            =  0x03,
  VN100_CmdID_RestoreFactorySettings   =  0x04,
  VN100_CmdID_Tare                     = 0x05,
  VN100_CmdID_Reset                    = 0x06,
  VN100_CmdID_FlashFirmware            =  0x07,
  VN100_CmdID_SetRefFrame              = 0x08,
  VN100_CmdID_HardwareInLoop           =  0x09,
  VN100_CmdID_GetFlashCNT              =  0x0A,
  VN100_CmdID_Calibrate                = 0x0B  
} VN100_CmdID;

/* System Error */
typedef enum VN100_Error
{
  VN100_Error_None                      = 0,
  VN100_Error_ErrorListOverflow          = 255,
  VN100_Error_HardFaultException        = 1,
  VN100_Error_InputBufferOverflow        =  2,
  VN100_Error_InvalidChecksum            = 3,
  VN100_Error_InvalidCommand            = 4,
  VN100_Error_NotEnoughParameters        = 5,
  VN100_Error_TooManyParameters          = 6,
  VN100_Error_InvalidParameter          = 7,
  VN100_Error_InvalidRegister            = 8,
  VN100_Error_UnauthorizedAccess        = 9,
  VN100_Error_WatchdogReset              = 10,
  VN100_Error_OutputBufferOverflow      = 11,
  VN100_Error_InsufficientBandwidth      = 12
} VN100_Error;

/* Asynchronous Data Output Register */
typedef enum VN100_ADORType
{
  VN100_ADOR_OFF  = 0,
  VN100_ADOR_YPR  = 1,
  VN100_ADOR_QTN  = 2,
  VN100_ADOR_QTM  = 3,
  VN100_ADOR_QTA  = 4,
  VN100_ADOR_QTR  = 5,
  VN100_ADOR_QMA  = 6,
  VN100_ADOR_QAR  = 7,
  VN100_ADOR_QMR  = 8,
  VN100_ADOR_DCM  = 9,
  VN100_ADOR_MAG  = 10,
  VN100_ADOR_ACC  = 11,
  VN100_ADOR_GYR  = 12,
  VN100_ADOR_MAR  = 13,
  VN100_ADOR_YMR  = 14,

  VN100_ADOR_RAB  = 251,
  VN100_ADOR_RAW  = 252,
  VN100_ADOR_CMV  = 253,
  VN100_ADOR_STV  = 254,
  VN100_ADOR_COV  = 255
} VN100_ADORType;

/* Asynchronous Data Ouput Rate Register */
typedef enum VN100_ADOFType {
  VN100_ADOF_1HZ     =  1,
  VN100_ADOF_2HZ     = 2,
  VN100_ADOF_4HZ     = 4,
  VN100_ADOF_5HZ     = 5,
  VN100_ADOF_10HZ    = 10,
  VN100_ADOF_20HZ    = 20,
  VN100_ADOF_25HZ    = 25,
  VN100_ADOF_40HZ    = 40,
  VN100_ADOF_50HZ    = 50,
  VN100_ADOF_100HZ   = 100,
  VN100_ADOF_200HZ   = 200
} VN100_ADOFType;

/* Serial Baud Rate Register */
typedef enum VN100_BaudType {
  VN100_Baud_9600     =  9600,
  VN100_Baud_19200    = 19200,
  VN100_Baud_38400    = 38400,
  VN100_Baud_57600    = 57600,
  VN100_Baud_115200   = 115200,
  VN100_Baud_128000   = 128000,
  VN100_Baud_230400   = 230400,
  VN100_Baud_460800   = 460800,
  VN100_Baud_921600   = 921600
} VN100_BaudType;

/* Accelerometer Gain Type */
typedef enum VN100_AccGainType {
  VN100_AccGain_2G = 0,
  VN100_AccGain_6G = 1
} VN100_AccGainType;

/* 32-bit Parameter Type */
typedef union {
  unsigned long      UInt;
  float             Float;
} VN100_Param;

/* SPI Response Packet */
typedef struct {
  unsigned char         ZeroByte;
  unsigned char         CmdID;
  unsigned char          RegID;
  unsigned char          ErrID;
  VN100_Param Data[VN100_SPI_BUFFER_SIZE];
} VN100_SPI_Packet;

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
VN100_SPI_Packet* VN100_SPI_ReadRegister(unsigned char sensorID, unsigned char regID, unsigned char regWidth);
VN100_SPI_Packet* VN100_SPI_WriteRegister(unsigned char sensorID, unsigned char regID, unsigned char regWidth, unsigned long* ptrWriteValues);
VN100_SPI_Packet* VN100_SPI_GetModel(unsigned char sensorID, char* model);
VN100_SPI_Packet* VN100_SPI_GetHWRev(unsigned char sensorID, unsigned long* revision);
VN100_SPI_Packet* VN100_SPI_GetSerial(unsigned char sensorID, unsigned long* serialNumber);
VN100_SPI_Packet* VN100_SPI_GetFWVer(unsigned char sensorID, unsigned long* firmwareVersion);
VN100_SPI_Packet* VN100_SPI_GetBaudRate(unsigned char sensorID, VN100_BaudType* baudRate);
VN100_SPI_Packet* VN100_SPI_SetBaudRate(unsigned char sensorID, VN100_BaudType baudRate);
VN100_SPI_Packet* VN100_SPI_GetADOR(unsigned char sensorID, VN100_ADORType* ADOR);
VN100_SPI_Packet* VN100_SPI_SetADOR(unsigned char sensorID, VN100_ADORType ADOR);
VN100_SPI_Packet* VN100_SPI_GetADOF(unsigned char sensorID, VN100_ADOFType* ADOF);
VN100_SPI_Packet* VN100_SPI_SetADOF(unsigned char sensorID, VN100_ADOFType ADOF);
VN100_SPI_Packet* VN100_SPI_GetYPR(unsigned char sensorID, float* yaw, float* pitch, float* roll);
VN100_SPI_Packet* VN100_SPI_GetQuat(unsigned char sensorID, float* q);
VN100_SPI_Packet* VN100_SPI_GetQuatMag(unsigned char sensorID, float* q, float* mag);
VN100_SPI_Packet* VN100_SPI_GetQuatAcc(unsigned char sensorID, float* q, float* Acc);
VN100_SPI_Packet* VN100_SPI_GetQuatRates(unsigned char sensorID, float* q, float* rates);
VN100_SPI_Packet* VN100_SPI_GetQuatMagAcc(unsigned char sensorID, float* q, float* mag, float* Acc);
VN100_SPI_Packet* VN100_SPI_GetQuatAccRates(unsigned char sensorID, float* q, float* Acc, float* rates);
VN100_SPI_Packet* VN100_SPI_GetQuatMagAccRates(unsigned char sensorID, float* q, float* mag, float* Acc, float* rates);
VN100_SPI_Packet* VN100_SPI_GetYPRMagAccRates(unsigned char sensorID, float* YPR, float* mag, float* Acc, float* rates);
VN100_SPI_Packet* VN100_SPI_GetDCM(unsigned char sensorID, float **DCM);
VN100_SPI_Packet* VN100_SPI_GetMag(unsigned char sensorID, float* mag);
VN100_SPI_Packet* VN100_SPI_GetAcc(unsigned char sensorID, float* Acc);
VN100_SPI_Packet* VN100_SPI_GetRates(unsigned char sensorID, float* rates);
VN100_SPI_Packet* VN100_SPI_GetMagAccRates(unsigned char sensorID, float* mag, float* Acc, float* rates);
VN100_SPI_Packet* VN100_SPI_GetMagAccRef(unsigned char sensorID, float* refMag, float* refAcc);
VN100_SPI_Packet* VN100_SPI_SetMagAccRef(unsigned char sensorID, float* refMag, float* refAcc);
VN100_SPI_Packet* VN100_SPI_GetFiltMeasVar(unsigned char sensorID, float* measVariance);
VN100_SPI_Packet* VN100_SPI_SetFiltMeasVar(unsigned char sensorID, float* measVar);
VN100_SPI_Packet* VN100_SPI_GetHardSoftIronComp(unsigned char sensorID, float* HSI);
VN100_SPI_Packet* VN100_SPI_SetHardSoftIronComp(unsigned char sensorID, float* HSI);
VN100_SPI_Packet* VN100_SPI_GetFiltActTuning(unsigned char sensorID, float* gainM, float* gainA, float* memM, float* memA);
VN100_SPI_Packet* VN100_SPI_SetFiltActTuning(unsigned char sensorID, float gainM, float gainA, float memM, float memA);
VN100_SPI_Packet* VN100_SPI_GetAccComp(unsigned char sensorID, float* AccComp);
VN100_SPI_Packet* VN100_SPI_SetAccComp(unsigned char sensorID, float* AccComp);
VN100_SPI_Packet* VN100_SPI_GetRefFrameRot(unsigned char sensorID, float* refFrameRot);
VN100_SPI_Packet* VN100_SPI_SetRefFrameRot(unsigned char sensorID, float* refFrameRot);
VN100_SPI_Packet* VN100_SPI_GetAccGain(unsigned char sensorID, VN100_AccGainType* gain);
VN100_SPI_Packet* VN100_SPI_SetAccGain(unsigned char sensorID, VN100_AccGainType gain);
VN100_SPI_Packet* VN100_SPI_RestoreFactorySettings(unsigned char sensorID);
VN100_SPI_Packet* VN100_SPI_Tare(unsigned char sensorID);
void VN100_SPI_Reset(unsigned char sensorID);
void VN100_SPI_GetAccInertial(unsigned char sensorID, float *AccI);
void VN100_SPI_GetMagInertial(unsigned char sensorID, float *AccI);


#endif /* __VN100_H */

/******************* (C) COPYRIGHT 2009 VectorNav Technologies *****************
***********************************END OF FILE*********************************/
