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
#include "VN100.h"
#include "VN_lib.h"
#include "main.h"
#include "../Common/Interfaces/SPI.h"

#ifdef _VN100
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Buffer used for SPI read and write responses */
/* Both the read and write register SPI routines below use this packet
   to store the returned SPI response. None of the write register commands
   implemented in this library check the data that is returned by the sensor
   to ensure that it is consistent with the data that was sent.  For normal
   cases this isn't necessary however if you wish to implement your own
   checking then this is the structure that you need to check after each
   register set command.  The structure has the following form:
   VN_SPI_LastReceivedPacket.CmdID -> This is the ID for the command that
                                   the response is for
   VN_SPI_LastReceivedPacket.RegID -> This is the ID for the register that
                                   the response is for
   VN_SPI_LastReceivedPacket.Data[] -> This is the data that was returned by
                                    the sensor as an array of unsigned 32-bit
                                    integers  */
VN100_SPI_Packet VN_SPI_LastReceivedPacket = {0, 0, 0, 0, {{0}}};

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/*******************************************************************************
* Function Name  : VN100_initSPI()
* Description    : Initialize the SPI interface.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void VN100_initSPI(){
    initSPI(IMU_SPI_PORT, 10000, SPI_MODE3, SPI_BYTE, SPI_MASTER);
}
/*******************************************************************************
* Function Name  : VN100_SPI_ReadRegister(unsigned char sensorID, unsigned char regID, unsigned char regWidth)
* Description    : Read the register with the ID regID on a VN-100 sensor
*                  using the SPI interface.
* Input          : sensorID -> The sensor to get the requested data from.
*                : regID -> The requested register ID number
*                : regWidth -> The width of the requested register in 32-bit words
* Output         : None
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_ReadRegister(unsigned char sensorID, unsigned char regID, unsigned char regWidth){


  /* Pull SS line low to start transaction*/
  VN_SPI_SetSS(sensorID, VN_PIN_LOW);
  /* Send request */
  VN_SPI_SendReceive(VN_BYTES2WORD(0, 0, regID, VN100_CmdID_ReadRegister));
  /* Pull SS line high to end SPI transaction */
  VN_SPI_SetSS(sensorID, VN_PIN_HIGH);
  /* Delay for 50us */
  //TODO: See if we need this. If we're running out of time, leverage this however we can
  VN_Delay(50);
  /* Pull SS line low to start SPI transaction */
  VN_SPI_SetSS(sensorID, VN_PIN_LOW);

  /* Get response over SPI */
  unsigned long i;
  for(i=0;i<regWidth;i++){
    *(((unsigned long*)&VN_SPI_LastReceivedPacket) + i) = VN_SPI_SendReceive(0);
  }
//
//  /* Pull SS line high to end SPI transaction */
  VN_SPI_SetSS(sensorID, VN_PIN_HIGH);
  VN_Delay(50);
//  /* Return Error code */
  return &VN_SPI_LastReceivedPacket;
}

/*******************************************************************************
* Function Name  : VN100_SPI_WriteRegister(unsigned char sensorID, unsigned char regID, unsigned char regWidth, unsigned long* ptrWriteValues)
* Description    : Write to the register with the ID regID on VN-100 sensor
*                  using the SPI interface.
* Input          : sensorID -> The sensor to write the requested data to.
*                : regID -> The register ID number
*                : regWidth -> The width of the register in 32-bit words
* Output         : ptrWriteValues -> The data to write to the requested register.
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_WriteRegister(unsigned char sensorID, unsigned char regID, unsigned char regWidth, unsigned long* ptrWriteValues){

  unsigned long i;
  /* Pull SS line low to start transaction*/
  VN_SPI_SetSS(sensorID, VN_PIN_LOW);
  /* Send write command */
  VN_SPI_SendReceive(VN_BYTES2WORD(0, 0, regID, VN100_CmdID_WriteRegister));
  for(i=0;i<regWidth;i++){
    VN_SPI_SendReceive(ptrWriteValues[i]);
  }

  /* Pull SS line high to end SPI transaction */
  VN_SPI_SetSS(sensorID, VN_PIN_HIGH);

  /* Delay for 50us */
  VN_Delay(100);
  
  /* Pull SS line low to start SPI transaction */
  VN_SPI_SetSS(sensorID, VN_PIN_LOW);

  /* Get response over SPI */
  for(i=0;i<4;i++){
    *(((unsigned long*)&VN_SPI_LastReceivedPacket) + i) = VN_SPI_SendReceive(0);
  }

  /* Pull SS line high to end SPI transaction */
  VN_SPI_SetSS(sensorID, VN_PIN_HIGH);


  /* Return pointer to SPI packet */
  return &VN_SPI_LastReceivedPacket;
}

/*******************************************************************************
* Function Name  : VN100_SPI_GetModel(unsigned char sensorID, char* model)
* Description    : Read the model number from the sensor.
* Input          : sensorID -> The sensor to get the model number from.
* Output         : model -> Pointer to a character array where the requested
*                           model number is placed. This needs to be a character
*                           array that is 12 characters in size.
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_GetModel(unsigned char sensorID, char* model){

  unsigned long i;

  /* Read register */
  VN100_SPI_ReadRegister(sensorID, VN100_REG_MODEL, 3);

  /* Get model number */
  for(i=0;i<3;i++){
    *((unsigned long*)model + i) = VN_SPI_LastReceivedPacket.Data[i].UInt;
  }

  /* Return pointer to SPI packet */
  return &VN_SPI_LastReceivedPacket;
}

/*******************************************************************************
* Function Name  : VN100_SPI_GetHWRev(unsigned char sensorID, unsigned long* revision)
* Description    : Get the hardware revision for the sensor.
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : revision -> The hardware revision requested.
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_GetHWRev(unsigned char sensorID, unsigned long* revision){

  /* Read register */
  VN100_SPI_ReadRegister(sensorID, VN100_REG_HWREV, 1);

  /* Get hardware revision */
  *revision = VN_SPI_LastReceivedPacket.Data[0].UInt;

  /* Return pointer to SPI packet */
  return &VN_SPI_LastReceivedPacket;
}

/*******************************************************************************
* Function Name  : VN100_SPI_GetSerial(unsigned char sensorID, unsigned long* serialNumber)
* Description    : Get the serial number from the requested sensor.
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : serialNumber -> The serial number returned by the sensor.
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_GetSerial(unsigned char sensorID, unsigned long* serialNumber){

  unsigned long i;

  /* Read register */
  VN100_SPI_ReadRegister(sensorID, VN100_REG_SN, 3);

  /* Get model number */
  for(i=0;i<3;i++){
    *(serialNumber + i) = VN_SPI_LastReceivedPacket.Data[i].UInt;
  }

  /* Return pointer to SPI packet */
  return &VN_SPI_LastReceivedPacket;
}

/*******************************************************************************
* Function Name  : VN100_SPI_GetFWVer(unsigned char sensorID, unsigned long* firmwareVersion)
* Description    : Get the firmware version from the requested sensor.
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : firmwareVersion -> The firmware version returned.
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_GetFWVer(unsigned char sensorID, unsigned long* firmwareVersion){

  /* Read register */
  VN100_SPI_ReadRegister(sensorID, VN100_REG_FWVER, 1);

  /* Get hardware revision */
  *firmwareVersion = VN_SPI_LastReceivedPacket.Data[0].UInt;

  /* Return pointer to SPI packet */
  return &VN_SPI_LastReceivedPacket;
}

/*******************************************************************************
* Function Name  : VN100_SPI_GetBaudRate(unsigned char sensorID, VN100_BaudType baudRate)
* Description    : Get the serial baud rate from the requested sensor.
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : baudRate -> The baud rate returned by the sensor.
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_GetBaudRate(unsigned char sensorID, VN100_BaudType* baudRate){

  /* Read register */
  VN100_SPI_ReadRegister(sensorID, VN100_REG_SBAUD, 1);

  /* Get hardware revision */
  *baudRate = (VN100_BaudType)VN_SPI_LastReceivedPacket.Data[0].UInt;

  /* Return pointer to SPI packet */
  return &VN_SPI_LastReceivedPacket;
}

/*******************************************************************************
* Function Name  : VN100_SPI_SetBaudRate(unsigned char sensorID, VN100_BaudType baudRate)
* Description    : Set the serial baud rate for the requested sensor.
* Input          : sensorID -> The sensor to set.
* Output         : baudRate -> The baud rate to set on the sensor.
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_SetBaudRate(unsigned char sensorID, VN100_BaudType baudRate){

  unsigned long regValue = (unsigned long)baudRate;

  /* Write register and return SPI packet*/
  return VN100_SPI_WriteRegister(sensorID, VN100_REG_SBAUD, 1, &regValue);
}

/*******************************************************************************
* Function Name  : VN100_SPI_GetADOR(unsigned char sensorID, VN100_ADORType ADOR)
* Description    : Get the ADOR register value from the requested sensor.
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : ADOR -> The value returned for the ADOR register.
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_GetADOR(unsigned char sensorID, VN100_ADORType* ADOR){

  /* Read register */
  VN100_SPI_ReadRegister(sensorID, VN100_REG_ADOR, 1);

  /* Get hardware revision */
  *ADOR = (VN100_ADORType)VN_SPI_LastReceivedPacket.Data[0].UInt;

  /* Return pointer to SPI packet */
  return &VN_SPI_LastReceivedPacket;
}

/*******************************************************************************
* Function Name  : VN100_SPI_SetADOR(unsigned char sensorID, VN100_ADORType ADOR)
* Description    : Set the ADOR register value from the requested sensor.
* Input          : sensorID -> The sensor to set.
* Output         : ADOR -> The value to set the ADOR register to.
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_SetADOR(unsigned char sensorID, VN100_ADORType ADOR){

  unsigned long regValue = (unsigned long)ADOR;

  /* Write register and return SPI packet*/
  return VN100_SPI_WriteRegister(sensorID, VN100_REG_ADOR, 1, &regValue);
}

/*******************************************************************************
* Function Name  : VN100_SPI_GetADOF(unsigned char sensorID, VN100_ADOFType ADOF)
* Description    : Get the async data output frequency.
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : ADOR -> The frequency returned for the ADOF register.
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_GetADOF(unsigned char sensorID, VN100_ADOFType* ADOF){

  /* Read register */
  VN100_SPI_ReadRegister(sensorID, VN100_REG_ADOF, 1);

  /* Get hardware revision */
  *ADOF = (VN100_ADOFType)VN_SPI_LastReceivedPacket.Data[0].UInt;

  /* Return pointer to SPI packet */
  return &VN_SPI_LastReceivedPacket;
}

/*******************************************************************************
* Function Name  : VN100_SPI_SetADOF(unsigned char sensorID, VN100_ADOFType ADOF)
* Description    : Set the async data output frequency.
* Input          : sensorID -> The sensor to set.
* Output         : ADOR -> The desired frequency of the async data output.
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_SetADOF(unsigned char sensorID, VN100_ADOFType ADOF){

  unsigned long regValue = (unsigned long)ADOF;

  /* Write register and return SPI packet*/
  return VN100_SPI_WriteRegister(sensorID, VN100_REG_ADOR, 1, &regValue);
}

/*******************************************************************************
* Function Name  : VN100_SPI_GetYPR(unsigned char sensorID, float yaw, float pitch, float roll)
* Description    : Get the measured yaw, pitch, roll orientation angles.
* Input          : sensorID -> The sensor to set.
* Output         : yaw -> The yaw angle measured in degrees.
*                  pitch -> The pitch angle measured in degrees.
*                  roll -> The roll angle measured in degrees.
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_GetYPR(unsigned char sensorID, float* yaw, float* pitch, float* roll){

  /* Read register */
  VN100_SPI_ReadRegister(sensorID, VN100_REG_YPR, 3);

  /* Get Yaw, Pitch, Roll */
  *yaw   = VN_SPI_LastReceivedPacket.Data[0].Float;
  *pitch = VN_SPI_LastReceivedPacket.Data[1].Float;
  *roll  = VN_SPI_LastReceivedPacket.Data[2].Float;

  /* Return pointer to SPI packet */
  return &VN_SPI_LastReceivedPacket;
}

/*******************************************************************************
* Function Name  : VN100_SPI_GetQuat(unsigned char sensorID, float* q)
* Description    : Get the measured attitude quaternion. The quaternion is a 4x1
*                  vector unit vector with the fourth term q[3] as the scalar
*                  term.
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : q -> The address of the location to write the returned
*                       measured quaternion (4x1).
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_GetQuat(unsigned char sensorID, float* q){

  unsigned long i;

  /* Read register */
  VN100_SPI_ReadRegister(sensorID, VN100_REG_QTN, 4);

  /* Get Quaternion */
  for(i=0;i<4;i++){
    q[i] = VN_SPI_LastReceivedPacket.Data[i].Float;
  }

  /* Return pointer to SPI packet */
  return &VN_SPI_LastReceivedPacket;
}

/*******************************************************************************
* Function Name  : VN100_SPI_GetQuatMag(unsigned char sensorID, float* q, float* mag)
* Description    : Get the measured attitude quaternion and magnetic vector. The
*                  quaternion is a 4x1 unit vector with the fourth term q[3] as
*                  the scalar term. The magnetic is a 3x1 vector.  The measured
*                  magnetic vector does not have any usable units.  The magnetic
*                  vector is calibrated at the factory to have a magnitude of
*                  one on the XY plane.
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : q -> The address of the location to write the returned
*                       measured quaternion (4x1).
*                  mag -> The magnetic measured vector (3x1).
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_GetQuatMag(unsigned char sensorID, float* q, float* mag){

  unsigned long i;

  /* Read register */
  VN100_SPI_ReadRegister(sensorID, VN100_REG_QTM, 7);

  /* Get Quaternion */
  for(i=0;i<4;i++){
    q[i] = VN_SPI_LastReceivedPacket.Data[i].Float;
  }

  /* Get Magnetic */
  for(i=0;i<3;i++){
    mag[i] = VN_SPI_LastReceivedPacket.Data[i+4].Float;
  }

  /* Return pointer to SPI packet */
  return &VN_SPI_LastReceivedPacket;
}

/*******************************************************************************
* Function Name  : VN100_SPI_GetQuatAcc(unsigned char sensorID, float* q, float* acc)
* Description    : Get the measured attitude quaternion and acceleration vector.
*                  The quaternion is a 4x1 unit vector with the fourth term q[3]
*                  as the scalar term.
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : q -> Measured quaternion (4x1).
*                  acc -> Measured acceleration (3x1) in m/s^2.
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_GetQuatAcc(unsigned char sensorID, float* q, float* Acc){

  unsigned long i;

  /* Read register */
  VN100_SPI_ReadRegister(sensorID, VN100_REG_QTA, 7);

  /* Get Quaternion */
  for(i=0;i<4;i++){
    q[i] = VN_SPI_LastReceivedPacket.Data[i].Float;
  }

  /* Get Acceleration */
  for(i=0;i<3;i++){
    Acc[i] = VN_SPI_LastReceivedPacket.Data[i+4].Float;
  }

  /* Return pointer to SPI packet */
  return &VN_SPI_LastReceivedPacket;
}

/*******************************************************************************
* Function Name  : VN100_SPI_GetQuatRates(unsigned char sensorID, float* q, float* rates)
* Description    : Get the measured attitude quaternion and angular rates.
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : q -> Measured quaternion (4x1).
*                  rates -> Measured angular rates (3x1) in rad/s.
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_GetQuatRates(unsigned char sensorID, float* q, float* rates){

  unsigned long i;

  /* Read register */
  VN100_SPI_ReadRegister(sensorID, VN100_REG_QTR, 7);

  /* Get Quaternion */
  for(i=0;i<4;i++){
    q[i] = VN_SPI_LastReceivedPacket.Data[i].Float;
  }

  /* Get Angular Rates */
  for(i=0;i<3;i++){
    rates[i] = VN_SPI_LastReceivedPacket.Data[i+4].Float;
  }

  /* Return pointer to SPI packet */
  return &VN_SPI_LastReceivedPacket;
}

/*******************************************************************************
* Function Name  : VN100_SPI_GetQuatMagAcc(unsigned char sensorID, float* q, float* mag, float* acc)
* Description    : Get the measured attitude quaternion, magnetic and acceleration.
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : q -> Measured quaternion (4x1).
*                  mag -> The magnetic measured vector (3x1).
*                  acc -> Measured acceleration (3x1) in m/s^2.
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_GetQuatMagAcc(unsigned char sensorID, float* q, float* mag, float* Acc){

  unsigned long i;

  /* Read register */
  VN100_SPI_ReadRegister(sensorID, VN100_REG_QMA, 10);

  /* Get Quaternion */
  for(i=0;i<4;i++){
    q[i] = VN_SPI_LastReceivedPacket.Data[i].Float;
  }

  /* Get Magnetic */
  for(i=0;i<3;i++){
    mag[i] = VN_SPI_LastReceivedPacket.Data[i+4].Float;
  }

  /* Get Acceleration */
  for(i=0;i<3;i++){
    Acc[i] = VN_SPI_LastReceivedPacket.Data[i+7].Float;
  }

  /* Return pointer to SPI packet */
  return &VN_SPI_LastReceivedPacket;
}

/*******************************************************************************
* Function Name  : VN100_SPI_GetQuatAccRates(unsigned char sensorID, float* q, float* acc, float* rates)
* Description    : Get the measured attitude quaternion, acceleration, and angular rates.
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : q -> Measured quaternion (4x1).
*                  acc -> Measured acceleration (3x1) in m/s^2.
*                  rates -> Measured angular rates (3x1) in rad/s.
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_GetQuatAccRates(unsigned char sensorID, float* q, float* acc, float* rates){

  unsigned long i;

  /* Read register */
  VN100_SPI_ReadRegister(sensorID, VN100_REG_QAR, 10);

  /* Get Quaternion */
  for(i=0;i<4;i++){
    q[i] = VN_SPI_LastReceivedPacket.Data[i].Float;
  }

  /* Get Acceleration */
  for(i=0;i<3;i++){
    acc[i] = VN_SPI_LastReceivedPacket.Data[i+4].Float;
  }

  /* Get Angular Rates */
  for(i=0;i<3;i++){
    rates[i] = VN_SPI_LastReceivedPacket.Data[i+7].Float;
  }

  /* Return pointer to SPI packet */
  return &VN_SPI_LastReceivedPacket;
}

/*******************************************************************************
* Function Name  : VN100_SPI_GetQuatMagAccRates(unsigned char sensorID, float* q, float* mag, float* acc, float* rates)
* Description    : Get the measured attitude quaternion, magnetic, acceleration, and angular rates.
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : q -> Measured quaternion (4x1).
*                  mag -> The magnetic measured vector (3x1).
*                  acc -> Measured acceleration (3x1) in m/s^2.
*                  rates -> Measured angular rates (3x1) in rad/s.
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_GetQuatMagAccRates(unsigned char sensorID, float* q, float* mag, float* acc, float* rates){

  unsigned long i;

  /* Read register */
  VN100_SPI_ReadRegister(sensorID, VN100_REG_QMR, 13);

  /* Get Quaternion */
  for(i=0;i<4;i++){
    q[i] = VN_SPI_LastReceivedPacket.Data[i].Float;
  }

  /* Get Magnetic */
  for(i=0;i<3;i++){
    mag[i] = VN_SPI_LastReceivedPacket.Data[i+4].Float;
  }

  /* Get Acceleration */
  for(i=0;i<3;i++){
    acc[i] = VN_SPI_LastReceivedPacket.Data[i+7].Float;
  }

  /* Get Angular Rates */
  for(i=0;i<3;i++){
    rates[i] = VN_SPI_LastReceivedPacket.Data[i+10].Float;
  }

  /* Return pointer to SPI packet */
  return &VN_SPI_LastReceivedPacket;
}

/*******************************************************************************
* Function Name  : VN100_SPI_GetYPRMagAccRates(unsigned char sensorID, float* YPR, float* mag, float* acc, float* rates)
* Description    : Get the yaw, pitch, roll, magnetic, acceleration, and angular rates.
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : YPR -> Euler angles (Yaw, Pitch, Roll) in deg.
*                  mag -> The magnetic measured vector (3x1).
*                  acc -> Measured acceleration (3x1) in m/s^2.
*                  rates -> Measured angular rates (3x1) in rad/s.
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_GetYPRMagAccRates(unsigned char sensorID, float* YPR, float* mag, float* acc, float* rates){

  unsigned long i;

  /* Read register */
  VN100_SPI_ReadRegister(sensorID, VN100_REG_YMR, 12);

  /* Get Euler angles */
  for(i=0;i<3;i++){
    YPR[i] = VN_SPI_LastReceivedPacket.Data[i].Float;
  }

  /* Get Magnetic */
  for(i=0;i<3;i++){
    mag[i] = VN_SPI_LastReceivedPacket.Data[i+3].Float;
  }

  /* Get Acceleration */
  for(i=0;i<3;i++){
    acc[i] = VN_SPI_LastReceivedPacket.Data[i+6].Float;
  }

  /* Get Angular Rates */
  for(i=0;i<3;i++){
    rates[i] = VN_SPI_LastReceivedPacket.Data[i+9].Float;
  }

  /* Return pointer to SPI packet */
  return &VN_SPI_LastReceivedPacket;
}

/*******************************************************************************
* Function Name  : VN100_SPI_GetDCM(unsigned char sensorID, float* DCM)
* Description    : Get the measured attitude as a directional cosine matrix.
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : DCM -> Directional Cosine Matrix (9x1). The order of the terms
*                         in the matrix is {first row, second row, third row}.
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_GetDCM(unsigned char sensorID, float **DCM){

  unsigned long i,j;

  /* Read register */
  VN100_SPI_ReadRegister(sensorID, VN100_REG_DCM, 9);

  /* Get Directional Cosine Matrix */
  for(i=0;i<3;i++){
    for(j=0;j<3;j++){
      DCM[i][j] = VN_SPI_LastReceivedPacket.Data[i*3+j].Float;
    }
  }

  /* Return pointer to SPI packet */
  return &VN_SPI_LastReceivedPacket;
}

/*******************************************************************************
* Function Name  : VN100_SPI_GetMag(unsigned char sensorID, float* mag)
* Description    : Get the measured magnetic field. The measured magnetic field
*                  does not have any usable units.  The magnetic vector is
*                  calibrated at the factory to have a magnitude of one on the
*                  XY plane.
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : mag -> The magnetic measured vector (3x1).
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_GetMag(unsigned char sensorID, float* mag){

  unsigned long i;

  /* Read register */
  VN100_SPI_ReadRegister(sensorID, VN100_REG_MAG, 3);

  /* Get Magnetic */
  for(i=0;i<3;i++){
    mag[i] = VN_SPI_LastReceivedPacket.Data[i].Float;
  }

  /* Return pointer to SPI packet */
  return &VN_SPI_LastReceivedPacket;
}

/*******************************************************************************
* Function Name  : VN100_SPI_GetAcc(unsigned char sensorID, float* Acc)
* Description    : Get the measured acceleration. The measured acceleration has
*                  the units of m/s^2 and its range is dependent upon the gain
*                  set by the VN100_SPI_SetAccGain() function.
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : Acc -> The measured acceleration (3x1) in m/s^2.
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_GetAcc(unsigned char sensorID, float* Acc){

  unsigned long i;

  /* Read register */
  VN100_SPI_ReadRegister(sensorID, VN100_REG_ACC, 3);

  /* Get Acceleration */
  for(i=0;i<3;i++){
    Acc[i] = VN_SPI_LastReceivedPacket.Data[i].Float;
  }

  /* Return pointer to SPI packet */
  return &VN_SPI_LastReceivedPacket;
}

/*******************************************************************************
* Function Name  : VN100_SPI_GetRates(unsigned char sensorID, float* rates)
* Description    : Get the measured angular rates. The measured angular rates
*                  have units of rad/s. This is the filtered angular rate and is
*                  compensated by the onboard Kalman filter to account for gyro
*                  bias drift.
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : rates -> The measured angular rates (3x1) in rad/s.
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_GetRates(unsigned char sensorID, float* rates){

  unsigned long i;


  /* Read register */
  VN100_SPI_ReadRegister(sensorID, VN100_REG_GYR, 3);
  /* Get Angular Rates */
  for(i=0;i<3;i++){
    rates[i] = VN_SPI_LastReceivedPacket.Data[i].Float;
  }
  /* Return pointer to SPI packet */
  return &VN_SPI_LastReceivedPacket;
}

/*******************************************************************************
* Function Name  : VN100_SPI_GetMagAccRates(unsigned char sensorID, float* mag, float* Acc, float* rates)
* Description    : Get the measured magnetic, acceleration, and angular rates.
*                  The measurements are taken in the body reference frame.
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : mag -> Measured magnetic field (3x1) [Non-dimensional].
*                  Acc -> Measured acceleration (3x1) [m/s^2].
*                  rates -> Measured angular rates (3x1) [rad/s].
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_GetMagAccRates(unsigned char sensorID, float* mag, float* Acc, float* rates){

  unsigned long i;

  /* Read register */
  VN100_SPI_ReadRegister(sensorID, VN100_REG_MAR, 9);

  /* Get Magnetic */
  for(i=0;i<3;i++){
    mag[i] = VN_SPI_LastReceivedPacket.Data[i].Float;
  }

  /* Get Acceleration */
  for(i=0;i<3;i++){
    Acc[i] = VN_SPI_LastReceivedPacket.Data[i+3].Float;
  }

  /* Get Angular Rates */
  for(i=0;i<3;i++){
    rates[i] = VN_SPI_LastReceivedPacket.Data[i+6].Float;
  }

  /* Return pointer to SPI packet */
  return &VN_SPI_LastReceivedPacket;
}

/*******************************************************************************
* Function Name  : VN100_SPI_GetMagAccRef(unsigned char sensorID, float* refMag, float* refAcc)
* Description    : Get the magnetic and acceleration reference vectors. The
*                  reference vectors are the vectors measured by the magnetomter
*                  and Accerometer respectively in the inertial reference
*                  frame.  The inertial reference frame is NED (North, East, Down).
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : refMag -> The reference vector for the magnetic field.
*                  refAcc -> The reference vector for the Accerometer (gravity).
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_GetMagAccRef(unsigned char sensorID, float* refMag, float* refAcc){

  unsigned long i;

  /* Read register */
  VN100_SPI_ReadRegister(sensorID, VN100_REG_REF, 6);

  /* Get magnetic reference */
  for(i=0;i<3;i++){
    refMag[i] = VN_SPI_LastReceivedPacket.Data[i].Float;
  }

  /* Get acceleration reference */
  for(i=0;i<3;i++){
    refAcc[i] = VN_SPI_LastReceivedPacket.Data[i+3].Float;
  }

  /* Return pointer to SPI packet */
  return &VN_SPI_LastReceivedPacket;
}

/*******************************************************************************
* Function Name  : VN100_SPI_SetMagAccReference(unsigned char sensorID, float* refMag, float* refAcc)
* Description    : Set the magnetic and acceleration reference vectors. The
*                  reference vectors are the vectors measured by the magnetometer
*                  and accelerometer respectively in the inertial reference
*                  frame.  The inertial reference frame is NED (North, East, Down).
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : refMag -> The reference vector for the magnetic field.
*                  refAcc -> The reference vector for the Accelerometer (gravity).
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_SetMagAccRef(unsigned char sensorID, float* refMag, float* refAcc){

  float ref[6];

  ref[0] = refMag[0];
  ref[1] = refMag[1];
  ref[2] = refMag[2];
  ref[3] = refAcc[0];
  ref[4] = refAcc[1];
  ref[5] = refAcc[2];

  /* Write register and return SPI packet*/
  return VN100_SPI_WriteRegister(sensorID, VN100_REG_REF, 6, (unsigned long*)ref);
}

/*******************************************************************************
* Function Name  : VN100_SPI_GetFiltMeasVar(unsigned char sensorID, float* measVar)
* Description    : Get the Kalman filter measurement variance parameters. This is
*                  discussed in the User Manual in Section 6.22. The measurement
*                  variance parameters controls how much weight the Kalman filter
*                  will place on each measurement.  See application note A001 for
*                  more details on how to set these values for your specific
*                  application.
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : measVar -> The variance on the measured inputs to the
*                             filter. This is a (10x1) vector.
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_GetFiltMeasVar(unsigned char sensorID, float* measVar){

  unsigned long i;

  /* Read register */
  VN100_SPI_ReadRegister(sensorID, VN100_REG_SIG, 10);

  /* Get filter measurement variance */
  for(i=0;i<10;i++){
    measVar[i] = VN_SPI_LastReceivedPacket.Data[i].Float;
  }

  /* Return pointer to SPI packet */
  return &VN_SPI_LastReceivedPacket;
}

/*******************************************************************************
* Function Name  : VN100_SPI_SetFiltMeasVar(unsigned char sensorID, float* measVar)
* Description    : Set the Kalman filter measurement variance parameters. This is
*                  discussed in the User Manual in Section 6.22. The measurement
*                  variance parameters controls how much weight the Kalman filter
*                  will place on each measurement.  See application note A001 for
*                  more details on how to set these values for your specific
*                  application.
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : measVar -> The variance on the measured inputs to the
*                                  filter. This is a (10x1) vector.
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_SetFiltMeasVar(unsigned char sensorID, float* measVar){

  /* Write register and return SPI packet*/
  return VN100_SPI_WriteRegister(sensorID, VN100_REG_SIG, 10, (unsigned long*)measVar);
}

/*******************************************************************************
* Function Name  : VN100_SPI_GetHardSoftIronComp(unsigned char sensorID, float* HSI)
* Description    : Get the magnetic hard/soft iron compensation parameters. These
*                  values allow the magnetometer to compensate for distortions in
*                  the local magnetic field due to ferromagnetic materials in the
*                  vacinity of the sensor. More information on the parameters can
*                  be found in the User Manual in Section 6.23.
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : HSI -> magnetic hard/soft iron paramteters (12x1).
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_GetHardSoftIronComp(unsigned char sensorID, float* HSI){

  unsigned long i;

  /* Read register */
  VN100_SPI_ReadRegister(sensorID, VN100_REG_HSI, 12);

  /* Get magnetic hard/soft iron compensation parameters */
  for(i=0;i<12;i++){
    HSI[i] = VN_SPI_LastReceivedPacket.Data[i].Float;
  }

  /* Return pointer to SPI packet */
  return &VN_SPI_LastReceivedPacket;
}

/*******************************************************************************
* Function Name  : VN100_SPI_SetHardSoftIronComp(unsigned char sensorID, float* HSI)
* Description    : Set the magnetic hard/soft iron compensation parameters. These
*                  values allow the magnetometer to compensate for distortions in
*                  the local magnetic field due to ferromagnetic materials in the
*                  vacinity of the sensor. More information on the parameters can
*                  be found in the User Manual in Section 6.23.
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : HSI -> magnetic hard/soft iron parameters (12x1).
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_SetHardSoftIronComp(unsigned char sensorID, float* HSI){

  /* Write register and return SPI packet*/
  return VN100_SPI_WriteRegister(sensorID, VN100_REG_HSI, 12, (unsigned long*)HSI);
}

/*******************************************************************************
* Function Name  : VN100_SPI_GetFiltActTuning(unsigned char sensorID, float gainM, float gainA, float memM, float memA)
* Description    : Get the filter active tuning parameters. The active tuning
*                  parameters control how the filter handles dynamic disturbances
*                  in both magnetic and acceleration.  These values are not needed
*                  for normal operation.  More on these parameters can be found in
*                  the User Manual in Section 6.24.
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : gainM -> Magnetic Disturbance Gain
*                  gainA -> Acceleration Disturbance Gain
*                  memM -> Magnetic Disturbance Memory
*                  memA -> Acceleration Disturbance Gain
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_GetFiltActTuning(unsigned char sensorID, float* gainM, float* gainA, float* memM, float* memA){

  /* Read register */
  VN100_SPI_ReadRegister(sensorID, VN100_REG_ATP, 6);

  /* Get magnetic gain */
  *gainM = VN_SPI_LastReceivedPacket.Data[0].Float;

  /* Get acceleration gain */
  *gainA = VN_SPI_LastReceivedPacket.Data[3].Float;

  /* Get magnetic memory */
  *memM = VN_SPI_LastReceivedPacket.Data[6].Float;

  /* Get acceleration memory */
  *memA = VN_SPI_LastReceivedPacket.Data[9].Float;

  /* Return pointer to SPI packet */
  return &VN_SPI_LastReceivedPacket;
}

/*******************************************************************************
* Function Name  : VN100_SPI_SetFiltActTuning(unsigned char sensorID, float gainM, float gainA, float memM, float memA)
* Description    : Set the filter active tuning parameters. The active tuning
*                  parameters control how the filter handles dynamic disturbances
*                  in both magnetic and acceleration.  These values are not needed
*                  for normal operation.  More on these parameters can be found in
*                  the User Manual in Section 6.24.
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : gainM -> Magnetic Disturbance Gain
*                  gainA -> Acceleration Disturbance Gain
*                  memM -> Magnetic Disturbance Memory
*                  memA -> Acceleration Disturbance Gain
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_SetFiltActTuning(unsigned char sensorID, float gainM, float gainA, float memM, float memA){

  float atp[4];

  atp[0] = gainM;
  atp[1] = gainA;
  atp[2] = memM;
  atp[3] = memA;

  /* Write register and return SPI packet*/
  return VN100_SPI_WriteRegister(sensorID, VN100_REG_ATP, 6, (unsigned long*)atp);
}

/*******************************************************************************
* Function Name  : VN100_SPI_GetAccComp(unsigned char sensorID, float* AccComp)
* Description    : Get the accelerometer compensation parameters. The purpose of
*                  these parameters are explained in Section 6.25 of the User
*                  Manual. These parameters are not required for normal operation.
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : AccComp -> Acceleration compensation register values.
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_GetAccComp(unsigned char sensorID, float* AccComp){

  unsigned long i;

  /* Read register */
  VN100_SPI_ReadRegister(sensorID, VN100_REG_ACT, 12);

  /* Get accelerometer compensation parameters */
  for(i=0;i<12;i++){
    AccComp[i] = VN_SPI_LastReceivedPacket.Data[i].Float;
  }

  /* Return pointer to SPI packet */
  return &VN_SPI_LastReceivedPacket;
}

/*******************************************************************************
* Function Name  : VN100_SPI_SetAccComp(unsigned char sensorID, float* AccComp)
* Description    : Set the accelerometer compensation parameters. The purpose of
*                  these parameters is explained in Section 6.25 of the User
*                  Manual. These parameters are not required for normal operation.
* Input          : sensorID -> The sensor to get the requested data from.
                   AccComp -> Acceleration compensation register values.
* Output:        : None
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_SetAccComp(unsigned char sensorID, float* AccComp){

  /* Write register and return SPI packet*/
  return VN100_SPI_WriteRegister(sensorID, VN100_REG_ACT, 12, (unsigned long*)AccComp);
}

/*******************************************************************************
* Function Name  : VN100_SPI_GetRefFrameRot(unsigned char sensorID, float* refFrameRot)
* Description    : Get the reference frame rotation matrix. This matrix allows
*                  the user to transform all measured vectors from the body
*                  reference frame of the VN-100, to any other rigidly attached
*                  coordinate frame. The effect of this transformation is that
*                  the computed attitude solution and measured measurement
*                  vectors will now be measured in the chosen coordinate system
*                  of the user and not the VN-100 coordinate system.  This is
*                  further explained in Section 6.26 of the User Manual.
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : refFrameRot -> Reference frame rotation matrix (9x1).
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_GetRefFrameRot(unsigned char sensorID, float* refFrameRot){

  unsigned long i;

  /* Read register */
  VN100_SPI_ReadRegister(sensorID, VN100_REG_RFR, 12);

  /* Get reference frame rotation parameters */
  for(i=0;i<12;i++){
    refFrameRot[i] = VN_SPI_LastReceivedPacket.Data[i].Float;
  }

  /* Return pointer to SPI packet */
  return &VN_SPI_LastReceivedPacket;
}

/*******************************************************************************
* Function Name  : VN100_SPI_SetRefFrameRot(unsigned char sensorID, float* refFrameRot)
* Description    : Set the reference frame rotation matrix. This matrix allows
*                  the user to transform all measured vectors from the body
*                  reference frame of the VN-100, to any other rigidly attached
*                  coordinate frame. The effect of this transformation is that
*                  the computed attitude solution and measured measurement
*                  vectors will now be measured in the chosen coordinate system
*                  of the user and not the VN-100 coordinate system.  This is
*                  further explained in Section 6.26 of the User Manual.
* Input          : sensorID -> The sensor to get the requested data from.
*                  refFrameRot -> Reference frame rotation matrix (9x1).
* Output         : None
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_SetRefFrameRot(unsigned char sensorID, float* refFrameRot){

  /* Write register and return SPI packet*/
  return VN100_SPI_WriteRegister(sensorID, VN100_REG_RFR, 12, (unsigned long*)refFrameRot);
}

/*******************************************************************************
* Function Name  : VN100_SPI_GetAccGain(unsigned char sensorID, VN100_AccGainType gain)
* Description    : Get the current accelerometer gain setting. The accelerometer
*                  on the VN-100 can be set to either a +/- 2g or +/- 6g gain
*                  setting.
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : gain -> The current accelerometer gain setting.
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_GetAccGain(unsigned char sensorID, VN100_AccGainType* gain){

  /* Read register */
  VN100_SPI_ReadRegister(sensorID, VN100_REG_ACG, 1);

  /* Get accelerometer gain */
  *gain = (VN100_AccGainType)VN_SPI_LastReceivedPacket.Data[0].UInt;

  /* Return pointer to SPI packet */
  return &VN_SPI_LastReceivedPacket;
}

/*******************************************************************************
* Function Name  : VN100_SPI_SetAccGain(unsigned char sensorID, VN100_AccGainType gain)
* Description    : Set the current accelerometer gain setting. The accelerometer
*                  on the VN-100 can be set to either a +/- 2g or +/- 6g gain
*                  setting.
* Input          : sensorID -> The sensor to get the requested data from.
*                : gain -> The current accelerometer gain setting.
* Output         : None
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_SetAccGain(unsigned char sensorID, VN100_AccGainType gain){

  unsigned long regValue = (unsigned long)gain;

  /* Write register and return SPI packet*/
  return VN100_SPI_WriteRegister(sensorID, VN100_REG_ACG, 1, &regValue);
}
/*******************************************************************************
* Function Name  : VN100_SPI_VelocityCompensationMeasurement(unsigned char sensorID, float* velocity)
* Description    : Sets the current velocity of the VN100 chip with respect to the chip's standard sensor axis.
* Input          : sensorID -> The sensor to get the requested data from.
*                : velocity -> A 3-component velocity vector
* Output         : None
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_VelocityCompensationMeasurement(unsigned char sensorID, float* velocity){

  /* Write register and return SPI packet*/
  return VN100_SPI_WriteRegister(sensorID, VN100_REG_VCM, 12, (unsigned long*) velocity);
}
/*******************************************************************************
* Function Name  : VN100_SPI_WriteSettings(unsigned char sensorID)
* Description    : Command the given sensor to save its settings.
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : None
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
void VN100_SPI_WriteSettings(unsigned char sensorID){

  /* Pull SS line low */
  VN_SPI_SetSS(sensorID, VN_PIN_LOW);

  /* Send command over SPI */
  VN_SPI_SendReceive(VN_BYTES2WORD(0, 0, 0, VN100_CmdID_WriteSettings));
  VN_SPI_SendReceive(0);

  /* Pull SS line high */
  VN_SPI_SetSS(sensorID, VN_PIN_HIGH);
}
/*******************************************************************************
* Function Name  : VN100_SPI_RestoreFactoryDefaultSettings(unsigned char sensorID)
* Description    : Restore the selected sensor to factory default state. The
*                  values for factory default state for each register can be
*                  found in Section 7 of the User Manual.
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : None
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_RestoreFactorySettings(unsigned char sensorID){

  /* Pull SS line low */
  VN_SPI_SetSS(sensorID, VN_PIN_LOW);

  /* Send command over SPI */
  VN_SPI_SendReceive(VN_BYTES2WORD(0, 0, 0, VN100_CmdID_RestoreFactorySettings));
  VN_SPI_SendReceive(0);

  /* Pull SS line high */
  VN_SPI_SetSS(sensorID, VN_PIN_HIGH);

  /* Delay for 50 uS */
  VN_Delay(50);

  /* Pull SS line low */
  VN_SPI_SetSS(sensorID, VN_PIN_LOW);

  /* Get response bytes */
  *((unsigned long*)&VN_SPI_LastReceivedPacket    ) = VN_SPI_SendReceive(0);
  *((unsigned long*)&VN_SPI_LastReceivedPacket + 1) = VN_SPI_SendReceive(0);

  /* Pull SS line high */
  VN_SPI_SetSS(sensorID, VN_PIN_HIGH);

  /* Return pointer to SPI packet */
  return &VN_SPI_LastReceivedPacket;
}

/*******************************************************************************
* Function Name  : VN100_SPI_Tare(unsigned char sensorID)
* Description    : Send a tare command to the selected VN-100. The tare command
*                  will zero out the current sensor orientation.  The attitude
*                  of the sensor will be measured form this point onwards with
*                  respect to the attitude present when the tare command was
*                  issued.  It is important with v4 of the firmware to keep
*                  the device still for at least 3 seconds after performing a
*                  tare command.  The tare command will also set the reference
*                  vectors in the inertial frame to the vectors currently
*                  measured in the body frame.
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : None
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_Tare(unsigned char sensorID){

  /* Pull SS line low */
  VN_SPI_SetSS(sensorID, VN_PIN_LOW);

  /* Send command over SPI */
  VN_SPI_SendReceive(VN_BYTES2WORD(0, 0, 0, VN100_CmdID_Tare));
  VN_SPI_SendReceive(0);

  /* Pull SS line high */
  VN_SPI_SetSS(sensorID, VN_PIN_HIGH);

  /* Delay for 50 uS */
  VN_Delay(50);

  /* Pull SS line low */
  VN_SPI_SetSS(sensorID, VN_PIN_LOW);

  /* Get response bytes */
  *((unsigned long*)&VN_SPI_LastReceivedPacket    ) = VN_SPI_SendReceive(0);
  *((unsigned long*)&VN_SPI_LastReceivedPacket + 1) = VN_SPI_SendReceive(0);

  /* Pull SS line high */
  VN_SPI_SetSS(sensorID, VN_PIN_HIGH);

  /* Return pointer to SPI packet */
  return &VN_SPI_LastReceivedPacket;
}

/*******************************************************************************
* Function Name  : VN100_SPI_Reset(unsigned char sensorID)
* Description    : Command the given sensor to perform a device hardware reset.
*                  This is equivalent to pulling the NRST pin low on the VN-100.
*                  Any changes to any of the registers on the VN-100 that were
*                  made since last issuing a Write Settings commands will be lost.
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : None
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
void VN100_SPI_Reset(unsigned char sensorID){

  /* Pull SS line low */
  VN_SPI_SetSS(sensorID, VN_PIN_LOW);

  /* Send command over SPI */
  VN_SPI_SendReceive(VN_BYTES2WORD(0, 0, 0, VN100_CmdID_Reset));
  VN_SPI_SendReceive(0);

  /* Pull SS line high */
  VN_SPI_SetSS(sensorID, VN_PIN_HIGH);
}

/*******************************************************************************
* Function Name  : VN100_SPI_GetAccInertial(unsigned char sensorID, float *AccI)
* Description    : Request the inertial acceleration from the VN-100. This
*                  function will internally request both the measured acceleration
*                  and attitude from the sensor, then compute the inertial
*                  acceleration.  If you are wanting to integrate your acceleration
*                  to find velocity or position, then this is the acceleration
*                  that you want to measure. It is measured in a fixed
*                  NED (North, East, Down) coordinate frame.
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : AccI -> The inertial acceleration measured by the device.
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
void VN100_SPI_GetAccInertial(unsigned char sensorID, float *AccI){

  /* Create a matrix for the attitude */
#if __STDC_VERSION__ >= 199901L
  VN_CreateMatrix(A, 3, 3, {0.0});
#else
  static float A_data[9] = {0.0};
  static float *A_ptr[3] = {&A_data[0], &A_data[3], &A_data[6]};
  static float **A = A_ptr;
#endif

  /* Attitude quaternion */
  float q[4];

  /* Body acceleration vector */
  float AccB[3];

  /* Get the attitude quaternion and acceleration from VN-100 */
  VN100_SPI_GetQuatAcc(sensorID, q, AccB);

  /* Convert the quaternion into a directional cosine matrix */
  VN_Quat2DCM(q, A);

  /* Multiply transpose of DCM by body acceleration to get inertial acceleration */
  VN_MatTVecMult(A, AccB, 3, 3, AccI);
}

/*******************************************************************************
* Function Name  : VN100_SPI_GetMagInertial(unsigned char sensorID, float *MagI)
* Description    : Request the inertial magnetic measurement from the VN-100. This
*                  function will internally request both the measured magnetic
*                  and attitude from the sensor, then compute the inertial
*                  magnetic measurement.  It is measured in a fixed
*                  NED (North, East, Down) coordinate frame.
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : MagI -> The inertial magnetic measurement measured by the
*                : device.
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
void VN100_SPI_GetMagInertial(unsigned char sensorID, float *MagI){

  /* Create a matrix for the attitude */
#if __STDC_VERSION__ >= 199901L
  VN_CreateMatrix(A, 3, 3, {0.0});
#else
  static float A_data[9] = {0.0};
  static float *A_ptr[3] = {&A_data[0], &A_data[3], &A_data[6]};
  static float **A = A_ptr;
#endif

  /* Attitude quaternion */
  float q[4];

  /* Body magnetic vector */
  float MagB[3];

  /* Get the attitude quaternion and magnetic from VN-100 */
  VN100_SPI_GetQuatMag(sensorID, q, MagB);

  /* Convert the quaternion into a directional cosine matrix */
  VN_Quat2DCM(q, A);

  /* Multiply transpose of DCM by body magnetic to get inertial magnetic */
  VN_MatTVecMult(A, MagB, 3, 3, MagI);
}

#endif /* _VN100 */

/******************* (C) COPYRIGHT 2009 VectorNav Technologies *****************
***********************************END OF FILE*********************************/
