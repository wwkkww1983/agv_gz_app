/**
  ******************************************************************************
  * @file    stm32_SMBUS_stack.h
  * @author  MCD Application Team
  * @version V2.0.0
  * @date    8-August-2016
  * @brief   This file provides a set of functions needed to manage the SMBUS STACK.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BSP_SMBUS_STACK_H
#define __BSP_SMBUS_STACK_H

#ifdef __cplusplus
extern "C"
{
#endif

  /* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
	
  /** @addtogroup STM32_SMBUS_STACK     SMBus 2.0 stack implementation
    * @{
    */

  /** @defgroup STM32_SMBUS_STACK_Defines SMBus stack definitions
    * @{
    */

#define SMBUS_SMS_NONE           ((uint32_t)0x00000000)  /*!< Uninitialized stack                                     */
#define SMBUS_SMS_READY          ((uint32_t)0x00000001)  /*!< No operation ongoing                                    */
#define SMBUS_SMS_TRANSMIT       ((uint32_t)0x00000002)  /*!< State of writing data to the bus                        */
#define SMBUS_SMS_RECEIVE        ((uint32_t)0x00000004)  /*!< State of receiving data on the bus                      */
#define SMBUS_SMS_PROCESSING     ((uint32_t)0x00000008)  /*!< Processing block (variable length transmissions)        */
#define SMBUS_SMS_RESPONSE_READY ((uint32_t)0x00000010)  /*!< Slave has reply ready for transmission (or command processing finished)        */
#define SMBUS_SMS_IGNORED        ((uint32_t)0x00000020)  /*!< The current command is not intended for this slave, ignore it (for ARP mainly) */
#define SMBUS_SMS_ALERT_PENDING  ((uint32_t)0x00000040)  /*!< Alert signal detected, address not known yet            */
#define SMBUS_SMS_ALERT_ADDRESS  ((uint32_t)0x00000080)  /*!< Alert signal treated, address read                      */
#define SMBUS_SMS_QUICK_CMD_W    ((uint32_t)0x00001000)  /*!< A Quick command write was received (slave-specific flag)*/
#define SMBUS_SMS_QUICK_CMD_R    ((uint32_t)0x00002000)  /*!< A Quick command read was received (slave-specific flag) */
#define SMBUS_SMS_ARP_AV         ((uint32_t)0x10000000)  /*!< Address resolution protocol address valid flag          */
#define SMBUS_SMS_ARP_AR         ((uint32_t)0x20000000)  /*!< Address resolution protocol address resolved flag       */
#define SMBUS_SMS_ARP_AM         ((uint32_t)0x40000000)  /*!< Address resolution protocol address match (ARP command processing)             */
#define SMBUS_SMS_RCV_BYTE_OFF   ((uint32_t)0x01000000)  /*!< Any direct read from slave is treated as Quick Command Read                    */
#define SMBUS_SMS_RCV_BYTE_LMT   ((uint32_t)0x02000000)  /*!< All Receive byte slave responses are ORed with 0x80     */
#define SMBUS_SMS_PEC_ACTIVE     ((uint32_t)0x04000000)  /*!< PEC flag - data integrity check is active               */
#define SMBUS_SMS_ZONE_READ      ((uint32_t)0x00000100)  /*!< Flag set on a slave device to indicate the status read is not completed        */
#define SMBUS_SMS_ERROR          ((uint32_t)0x00000200)  /*!< Any other error ( link level error, unknown command... )*/
#define SMBUS_SMS_ERR_BERR       ((uint32_t)0x00010000)  /*!< Bus error - misplaced start or stop, try lower speed    */
#define SMBUS_SMS_ERR_ARLO       ((uint32_t)0x00020000)  /*!< Arbitration lost error                                  */
#define SMBUS_SMS_ERR_ACKF       ((uint32_t)0x00040000)  /*!< ACK failure error - probably wrong slave address??      */
#define SMBUS_SMS_ERR_OVR        ((uint32_t)0x00080000)  /*!< Overrun/underrun error - SCL stretching perhaps should be enabled              */
#define SMBUS_SMS_ERR_HTO        ((uint32_t)0x00100000)  /*!< Timeout error                                           */
#define SMBUS_SMS_ERR_BTO        ((uint32_t)0x00200000)  /*!< Bus timeout error                                       */
#define SMBUS_SMS_ERR_PECERR     ((uint32_t)0x00800000)  /*!< PEC error - data integrity lost                         */
#define SMBUS_SMS_ACTIVE_MASK    ( SMBUS_SMS_TRANSMIT | SMBUS_SMS_RECEIVE | SMBUS_SMS_PROCESSING | SMBUS_SMS_RESPONSE_READY | SMBUS_SMS_ALERT_PENDING )

#define STACK_NBYTE_SIZE         ((uint32_t)40) /*!< maximum of allowed data bytes in a block transfer   */
  /* Note: As per SMBus 3.0 the process call may be up to 255B long. For SMBus 2.0 the buffer may be 40B */
#define PEC_SIZE                 ((uint32_t)1)  /*!< PEC size in bytes                                   */

#define SMBUS_ERROR_CRITICAL            ( SMBUS_SMS_ERR_HTO | SMBUS_SMS_ERR_BTO | SMBUS_SMS_ERR_BERR )
  /*!< Error suggests complete breakdown */
#define SMBUS_COM_ERROR                 ( SMBUS_SMS_ERR_ARLO | SMBUS_SMS_ERR_ACKF )
  /*!<  Error affecting single command, but within bus specification */
#define SMBUS_SMS_BUSY                  ( SMBUS_SMS_TRANSMIT | SMBUS_SMS_RECEIVE | SMBUS_SMS_PROCESSING | SMBUS_SMS_ALERT_PENDING )
  /*!<  Bus transaction ongoing mask */

#define PMBUS_COMMANDS_TAB_SIZE         ((uint8_t)145)  /* v1.1*/


/** 
 * @brief  Status structures definition  
 */ 
typedef enum 
{
	SMBUS_OK       = 0x00U,
	SMBUS_ERROR    = 0x01U,
	SMBUS_BUSY     = 0x02U,
	SMBUS_TIMEOUT  = 0x03U
} StatusTypeDef;

/**
 * @brief Command code descriptor structure
 */
typedef struct
{
	uint8_t cmnd_code;                /*!< command code - typically the first byte the master transmits after address */
	uint8_t cmnd_query;               /*!< supported direction, can be WRITE, READ, READ_AND_WRITE or READ_OR_WRITE */
	uint8_t cmnd_master_Tx_size;      /*!< number of bytes transmitted by the master, excluding the address, valid for WRITE and READ_OR_WRITE */
	uint8_t cmnd_master_Rx_size;      /*!< number of bytes transmitted by the slave, valid for READ_OR_WRITE and READ directions only */
} st_command_t;

/**
 * @brief Stack state machine handling structure - the stack operation context
 */
typedef struct
{
	uint32_t      StateMachine;              /*!< Keeps track of current stack state      */
	//    SMBUS_HandleTypeDef  *Device;            /*!< HAL driver handle  */
	uint8_t       *ARP_UDID;                 /*!< A UDID used in ARP */
	st_command_t  *CurrentCommand;           /*!< NULL, unless a command is being processed       */
	st_command_t  *CMD_table;                /*!< Pointer to first record in the table of all supported commands      */
	uint32_t      CMD_tableSize;             /*!< The supported commands table size    */    
	uint16_t      Byte_count;                /*!< Indicates current buffer position    */
	uint16_t      SlaveAddress;              /*!< Master remembers to which slave it is talking to */
	uint8_t       OwnAddress;                /*!< Device address, ARA response / Host: Address of the device that sent alert / */
	uint8_t       OpMode;                    /*!< Operation mode (read/write)      */
	uint8_t       SRByte;                    /*!< A preset byte for use in Send byte/ Receive byte transactions   */
	uint8_t       Buffer[STACK_NBYTE_SIZE+2];  /*!< The I/O buffer on which the SMBUS stack operates
	                                                extra 2 bytes are command code and block length */
} SMBUS_StackHandleTypeDef;

/** @defgroup STM32_SMBUS_STACK_Defines SMBus stack definitions
 * @{
 */
 /* ----------- definition of SMBUS function return codes -------------- */
#define STACK_OK        SMBUS_OK          /*!< No problem detected */
#define STACK_ERROR     SMBUS_ERROR       /*!< Problem appeared */
#define STACK_BUSY      SMBUS_BUSY        /*!< Could not process request, previous transaction is not finished */
#define STACK_TIMEOUT   SMBUS_TIMEOUT     /*!< Timeout */

 /* ----------- definition of SMBUS commands types -------------- */
#define READ            ((uint8_t)0x20)    /*!< Read operation / fixed size read only command */
#define WRITE           ((uint8_t)0x40)    /*!< Write operation / fixed size write only command */
#define READ_OR_WRITE   ((uint8_t)0x60)    /*!< Command can be executed either as read or as write of fixed size */
#define BLOCK           ((uint8_t)0x10)    /*!< Flag of block transmission - variable length */
#define BLOCK_READ      ((uint8_t)0x30)    /*!< Read block operation / read only command */
#define BLOCK_WRITE     ((uint8_t)0x50)    /*!< Write block operation / write only command */
#define BLK_PRC_CALL    ((uint8_t)0x90)    /*!< Block process call - write, then read, variable sizes */
#define PROCESS_CALL    ((uint8_t)0x80)    /*!< Simple process call - write, then read, fixed sizes */
#define BLK_RD_OR_WR    ((uint8_t)0x70)    /*!< Command can be executed either as read or as write of variable size */
#define	CHK_ADDR_DEVICE ((uint8_t)0xff)    /* check addr whether exist! */

  /* ------------ notable addresses -------------------- */
#define SMBUS_ADDR_ARA           ((uint16_t)0x18) /*!< Address used in alert response */
#define SMBUS_ADDR_HOST          ((uint16_t)0x10) /*!< Host address used in ARP */
#define SMBUS_ADDR_SBCHARGER     ((uint16_t)0x12) /*!< Typical charger address  */
#define SMBUS_ADDR_SBSELECTOR    ((uint16_t)0x14) /*!< Typical selector address */
#define SMBUS_ADDR_SB            ((uint16_t)0x16)
#define SMBUS_ADDR_ZONE_READ     ((uint16_t)0x50)
#define SMBUS_ADDR_ZONE_WRITE    ((uint16_t)0x6E)
#define SMBUS_ADDR_LCD           ((uint16_t)0x58)
#define SMBUS_ADDR_CCFL          ((uint16_t)0x5A) /*!< Typical CCFL backlight address*/
#define SMBUS_ADDR_ACCESS_DEF    ((uint16_t)0x6E)
#define SMBUS_ADDR_PCMCIA        ((uint16_t)0x86) /*!< mask, not actual value */
#define SMBUS_ADDR_VGA           ((uint16_t)0x88)
#define SMBUS_ADDR_DEFAULT       ((uint16_t)0xC2) /*!< not valid device address, used for ARP */
#define SMBUS_ADDR_DEVICE        ((uint16_t)0x1A)


  
/** @defgroup STM32_SMBUS_STACK_Interfaces SMBus stack public functions
 * @{
 */
#define STACK_SMBUS_IsReady(stack)              ((stack)->StateMachine&SMBUS_SMS_READY)
  /*!< Returns 1 if stack has ready flag set, 0 otherwise */
#define STACK_SMBUS_IsBlockingError(stack)      (((stack)->StateMachine&SMBUS_ERROR_CRITICAL)?1:0)
  /*!< Returns 1 in case of serious or unrecoverable error, 0 otherwise */
#define STACK_SMBUS_IsCmdError(stack)           (((stack)->StateMachine&SMBUS_COM_ERROR)?1:0)
  /*!< Returns 1 in case of command error (still within spec), 0 otherwise */
#define STACK_SMBUS_IsBusy(stack)               (((stack)->StateMachine&SMBUS_SMS_BUSY)?1:0)
  /*!< Returns 1 in case of transaction ongoing, 0 otherwise */
#define STACK_SMBUS_IsAlert(stack)              (((stack)->StateMachine&SMBUS_SMS_ALERT_ADDRESS)?1:0)
  /*!< Returns 1 in case of Alert event been treated recently, retrieving address, 0 otherwise */


void Init_SMBus(void);
StatusTypeDef SMBUS_Master_Transmit(SMBUS_StackHandleTypeDef *pStackContext);


#ifdef __cplusplus
}
#endif

#endif /* __SMBUS_STACK_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
