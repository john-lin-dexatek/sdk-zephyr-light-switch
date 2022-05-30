/*****************************************************************************
* File Name: CY8CMBR3xxx_APIs.h
*
* Version 1.00
*
* Description:
*   This file contains the declarations of all the high-level APIs.
*
* Note:
*   N/A
*
* Owner:
*   SRVS
*
* Related Document:
*   MBR3 Design Guide
*   MBR3 Device Register TRM
*   MBR3 Device Datasheet
*
* Hardware Dependency:
*   N/A
*
* Code Tested With:
*   PSoC Creator 3.0 CP7
*   CY3280-MBR3 Evaluation Kit
*   CY8CKIT-042 Pioneer Kit
*
******************************************************************************
* Copyright (2014), Cypress Semiconductor Corporation.
******************************************************************************
* This software is owned by Cypress Semiconductor Corporation (Cypress) and is
* protected by and subject to worldwide patent protection (United States and
* foreign), United States copyright laws and international treaty provisions.
* Cypress hereby grants to licensee a personal, non-exclusive, non-transferable
* license to copy, use, modify, create derivative works of, and compile the
* Cypress Source Code and derivative works for the sole purpose of creating
* custom software in support of licensee product to be used only in conjunction
* with a Cypress integrated circuit as specified in the applicable agreement.
* Any reproduction, modification, translation, compilation, or representation of
* this software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: CYPRESS MAKES NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, WITH
* REGARD TO THIS MATERIAL, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
* Cypress reserves the right to make changes without further notice to the
* materials described herein. Cypress does not assume any liability arising out
* of the application or use of any product or circuit described herein. Cypress
* does not authorize its products for use as critical components in life-support
* systems where a malfunction or failure may reasonably be expected to result in
* significant injury to the user. The inclusion of Cypress' product in a life-
* support systems application implies that the manufacturer assumes all risk of
* such use and in doing so indemnifies Cypress against all charges. Use may be
* limited by and subject to the applicable Cypress software license agreement.
*****************************************************************************/

#if !defined(CY8CMBR3xxx_APIS_H)
#define CY8CMBR3xxx_APIS_H
    
/*******************************************************************************
* Included headers
*******************************************************************************/
#include<stdint.h>
#include<stdbool.h>


/*******************************************************************************
* User defined Macros
*******************************************************************************/
/* The APIs CY8CMBR3xxx_ReadDiffCounts() and CY8CMBR3xxx_ReadSensorDebugData() 
 * try to read from the device for a maximum number of times defined by this 
 * value, as long as the sync counters do not match. Please change as required.
 */
#define CY8CMBR3xxx_SYNC_COUNTER_MATCH_RETRY             (10)


/*******************************************************************************
* Data Type Definitions
*******************************************************************************/
typedef uint8_t   uint8;
typedef uint16_t  uint16;
typedef uint32_t   uint32;

#ifndef FALSE
	#define FALSE false
#endif

#ifndef TRUE
	#define TRUE true
#endif

/*
#if !defined(CY8CMBR3xxx_DATATYPE_GUARD)
#define CY8CMBR3xxx_DATATYPE_GUARD

#define TRUE                                             (1)
#define FALSE                                            (0)

typedef unsigned char   uint8;
typedef unsigned short  uint16;
typedef unsigned long   uint32;
typedef uint8 bool;
#endif
*/

/*******************************************************************************
* Structure Definitions
*******************************************************************************/
/* 
 * This structure will hold the debug data read from the device by the 
 * CY8CMBR3xxx_ReadSensorData() API.
 */
typedef struct 
{
    uint8 sensorCp;
    uint16 sensorDiffCounts;
    uint16 sensorBaseline;
    uint16 sensorRawCounts;
    uint16 sensorAverageCounts;
} CY8CMBR3xxx_SENSORDATA;

/* 
 * This structure will hold the sensor status read from the device
 * by the CY8CMBR3xxx_ReadSensorStatus() API.
 * The slider positions and slider liftoff positions are valid for 
 * CY8CMBR3106S device only.
 */
typedef struct 
{
    uint16 buttonStatus;
    uint16 latchedButtonStatus;
    uint8 proxStatus;
    uint8 latchedProxStatus;
    uint8 slider1Position;
    uint8 liftoffSlider1Position;
    uint8 slider2Position;
    uint8 liftoffSlider2Position;
} CY8CMBR3xxx_SENSORSTATUS;


/*******************************************************************************
* Function Prototypes
*******************************************************************************/
bool CY8CMBR3xxx_WriteData(uint8 slaveAddress, uint8 *writeBuffer, uint8 numberOfBytes);
bool CY8CMBR3xxx_ReadData(uint8 slaveAddress, uint8 registerAddress, uint8 *readBuffer, uint8 numberOfBytes);
bool CY8CMBR3xxx_WriteDualByte(uint8 slaveAddress, uint8 registerAddress, uint16 writeData);
bool CY8CMBR3xxx_ReadDualByte(uint8 slaveAddress, uint8 registerAddress, uint16 *readData);
bool CY8CMBR3xxx_SendCommand(uint8 slaveAddress, uint8 command);
uint8 CY8CMRB3xxx_CheckCommandStatus(uint8 slaveAddress, uint8 *errorCode);
bool CY8CMBR3xxx_Configure(uint8 slaveAddress, const unsigned char *configuration);
uint16 CY8CMBR3xxx_CalculateCrc(uint8 *configuration);
bool CY8CMBR3xxx_VerifyDeviceOnBus(uint8 slaveAddress);
bool CY8CMBR3xxx_SetDebugDataSensorId(uint8 slaveAddress, uint8 sensorId);
bool CY8CMBR3xxx_ReadSensorDebugData(uint8 slaveAddress, CY8CMBR3xxx_SENSORDATA *debugData);
bool CY8CMBR3xxx_ReadDiffCounts(uint8 slaveAddress, uint16 *differenceCounts);
bool CY8CMBR3xxx_ReadSensorStatus(uint8 slaveAddress, CY8CMBR3xxx_SENSORSTATUS *status);

#endif
/****************************End of File***************************************/

/* Additional Parser function by Dexatek */
#define CY8CMBR3xxx_COMMAND_STATUS_BUSY                          (0)
#define CY8CMBR3xxx_COMMAND_STATUS_SUCCESS                       (1)
#define CY8CMBR3xxx_COMMAND_STATUS_FAILED                        (2)
#define CY8CMBR3xxx_REG_ADDRESS_LENGTH                           (1)
#define CY8CMBR3xxx_CONFIG_DATA_LENGTH_WITH_CRC_AND_ADDR         (129)
#define CY8CMBR3xxx_MAX_DEVICE_REGISTERS                         (252)
#define CY8CMBR3xxx_COMMAND_NO_ERROR                             (0)
#define CY8CMBR3xxx_REG_ADDRESS_OFFSET                           (0)
#define CY8CMBR3xxx_WRITE_DATA_LSB_OFFSET                        (1)
#define CY8CMBR3xxx_WRITE_DATA_MSB_OFFSET                        (2)
#define CY8CMBR3xxx_BYTE_WRITE_LENGTH                            (2)
#define CY8CMBR3xxx_BYTE_READ_LENGTH                             (1)
#define CY8CMBR3xxx_DUAL_BYTE_WRITE_LENGTH                       (3)
#define CY8CMBR3xxx_DUAL_BYTE_READ_LENGTH                        (2)
#define CY8CMBR3xxx_READ_DATA_LSB_OFFSET                         (0)
#define CY8CMBR3xxx_READ_DATA_MSB_OFFSET                         (1)
#define CY8CMBR3xxx_CONFIG_DATA_LENGTH_WITH_CRC                  (128)
#define CY8CMBR3xxx_COMMAND_SEND_LENGTH                          (2)
#define CY8CMBR3xxx_COMMAND_OFFSET                               (1)
#define CY8CMBR3xxx_COMMAND_STATUS_LENGTH                        (4)
#define CY8CMBR3xxx_CMD_STAT_CTRL_CMD_OFFSET                     (0)
#define CY8CMBR3xxx_CMD_STAT_CTRL_CMD_STATUS_OFFSET              (2)
#define CY8CMBR3xxx_CMD_STAT_CTRL_CMD_ERR_OFFSET                 (3)
#define CY8CMBR3xxx_CONFIG_DATA_WRITE_OFFSET                     (1)
#define CY8CMBR3xxx_VERIFY_DEVICE_READ_BUFFER_LENGTH             (5)
#define CY8CMBR3xxx_VERIFY_DEVICE_FAMILYID_OFFSET                (0)
#define CY8CMBR3xxx_VERIFY_DEVICE_DEVID_LSB_OFFSET               (1)
#define CY8CMBR3xxx_VERIFY_DEVICE_DEVID_MSB_OFFSET               (2)
#define CY8CMBR3xxx_VERIFY_DEVICE_DEVREV_LSB_OFFSET              (3)
#define CY8CMBR3xxx_VERIFY_DEVICE_DEVREV_MSB_OFFSET              (4)
#define CY8CMBR3xxx_SENSORID_SEND_LENGTH                         (2)
#define CY8CMBR3xxx_SENSORID_OFFSET                              (1)
#define CY8CMBR3xxx_SENSOR_DATA_LENGTH                           (13)
#define CY8CMBR3xxx_SENSOR_DATA_SYNC1_OFFSET                     (0)
#define CY8CMBR3xxx_SENSOR_DATA_SENSOR_ID_OFFSET                 (1)
#define CY8CMBR3xxx_SENSOR_DATA_AVG_CNT_LSB_OFFSET               (9)
#define CY8CMBR3xxx_SENSOR_DATA_AVG_CNT_MSB_OFFSET               (10)
#define CY8CMBR3xxx_SENSOR_DATA_BASELINE_LSB_OFFSET              (5)
#define CY8CMBR3xxx_SENSOR_DATA_BASELINE_MSB_OFFSET              (6)
#define CY8CMBR3xxx_SENSOR_DATA_CP_OFFSET                        (2)
#define CY8CMBR3xxx_SENSOR_DATA_DIFFCOUNTS_LSB_OFFSET            (3)
#define CY8CMBR3xxx_SENSOR_DATA_DIFFCOUNTS_MSB_OFFSET            (4)
#define CY8CMBR3xxx_SENSOR_DATA_RAWCOUNTS_LSB_OFFSET             (7)
#define CY8CMBR3xxx_SENSOR_DATA_RAWCOUNTS_MSB_OFFSET             (8)
#define CY8CMBR3xxx_SENSOR_DATA_SYNC2_OFFSET                     (12)
#define CY8CMBR3xxx_DIFFCOUNT_DATA_LENGTH                        (35)
#define CY8CMBR3xxx_DIFFCOUNT_SYNC0_OFFSET                       (0)
#define CY8CMBR3xxx_DIFFCOUNT_SYNC1_OFFSET                       (34)
#define CY8CMBR3xxx_DIFFCOUNT_WIDTH                              (2)
#define CY8CMBR3xxx_DIFFCOUNT_LSB_OFFSET                         (1)
#define CY8CMBR3xxx_DIFFCOUNT_MSB_OFFSET                         (2)
#define CY8CMBR3xxx_SENSOR_STATUS_LENGTH                         (10) 
#define CY8CMBR3xxx_BUTTONSTATUS_LSB_OFFSET                      (0)
#define CY8CMBR3xxx_BUTTONSTATUS_MSB_OFFSET                      (1)
#define CY8CMBR3xxx_LATCHEDBUTTONSTATUS_LSB_OFFSET               (2)
#define CY8CMBR3xxx_LATCHEDBUTTONSTATUS_MSB_OFFSET               (3)
#define CY8CMBR3xxx_PROXSTATUS_OFFSET                            (4)
#define CY8CMBR3xxx_LATCHEDPROXSTATUS_OFFSET                     (5)
#define CY8CMBR3xxx_SLIDER1POSITION_OFFSET                       (6)
#define CY8CMBR3xxx_LIFTOFFSLIDER1POSITION_OFFSET                (7)
#define CY8CMBR3xxx_SLIDER2POSITION_OFFSET                       (8)
#define CY8CMBR3xxx_LIFTOFFSLIDER2POSITION_OFFSET                (9)
#define CY8CMBR3xxx_BUFFER_INIT_VALUE                            (0x00)
#define CY8CMBR3xxx_CTRL_CMD_NO_ERROR                            (0)
#define CY8CMBR3xxx_DEFAULT_SENSOR_ID                            (0)
#define CY8CMBR3xxx_DEFAULT_CONFIG_INDEX                         (0)
#define CY8CMBR3xxx_BYTE_SHIFT_BITS                              (8)
#define CY8CMBR3xxx_BYTE_MASK                                    (0xFF)
#define CY8CMBR3xxx_LATCHED_BUTTON_STAT_LENGTH                   (2)

void parse_sensor_status(uint8_t *ptr_buffer, CY8CMBR3xxx_SENSORSTATUS *ptr_status);