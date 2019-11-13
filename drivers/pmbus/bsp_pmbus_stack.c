/**
  ******************************************************************************
  * @file    stm32_PMBUS_stack.c
  * @author  MCD Application Team
  * @version V2.0.0
  * @date    8-August-2016
  * @brief   This file provides a set of functions needed to manage the PMBus
  *          on top of the SMBus stack.
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

/* Includes ------------------------------------------------------------------*/
#include "bsp_pmbus_stack.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"	

/** @addtogroup STM32_PMBUS_STACK
  * @{
  */

/** @defgroup STM32_PMBUS_STACK_Constants
  * @{
  */

/* ----------- definition of PMBUS commands ---------------- */
st_command_t PMBUS_COMMANDS_TAB[] =
  {
    { PMBC_PAGE                        , READ_OR_WRITE, 2, 1 },       /* code 00 */
    { PMBC_OPERATION                   , READ_OR_WRITE, 2, 1 },       /* code 01 */
    { PMBC_ON_OFF_CONFIG               , READ_OR_WRITE, 2, 1 },       /* code 02 */
    { PMBC_CLEAR_FAULTS                , WRITE, 1, 0 },               /* code 03 */
    { PMBC_PHASE                       , READ_OR_WRITE, 2, 1 },       /* code 04 */
#ifdef  PMBUS12
    { PMBC_PAGE_PLUS_WRITE             , BLOCK_WRITE, 3, 0 },         /* code 05 */
    { PMBC_PAGE_PLUS_READ              , BLK_PRC_CALL, 3, 2 },        /* code 06 */
#ifdef  PMBUS13
    { PMBC_ZONE_CONFIG                 , READ_OR_WRITE, 3, 2 },       /* code 07 */
    { PMBC_ZONE_ACTIVE                 , READ_OR_WRITE, 3, 2 },       /* code 08 */
#endif
#endif
    { PMBC_WRITE_PROTECT               , READ_OR_WRITE,  2, 1 },      /* code 10 */
    { PMBC_STORE_DEFAULT_ALL           , WRITE, 1, 0 },               /* code 11 */
    { PMBC_RESTORE_DEFAULT_ALL         , WRITE, 1, 0 },               /* code 12 */
    { PMBC_STORE_DEFAULT_CODE          , WRITE, 2, 0 },               /* code 13 */
    { PMBC_RESTORE_DEFAULT_CODE        , WRITE, 2, 0 },               /* code 14 */
    { PMBC_STORE_USER_ALL              , WRITE, 1, 0 },               /* code 15 */
    { PMBC_RESTORE_USER_ALL            , WRITE, 1, 0 },               /* code 16 */
    { PMBC_STORE_USER_CODE             , WRITE, 2, 0 },               /* code 17 */
    { PMBC_RESTORE_USER_CODE           , WRITE, 2, 0 },               /* code 18 */
    { PMBC_CAPABILITY                  , READ, 1, 1 },                /* code 19 */
    { PMBC_QUERY                       , PROCESS_CALL, 2, 1 },        /* code 1A */
#ifdef  PMBUS12
    { PMBC_SMBALERT_MASK               , READ_OR_WRITE, 3, 2 },       /* code 1B */
#endif
    { PMBC_VOUT_MODE                   , READ_OR_WRITE, 2, 1 },       /* code 20 */
    { PMBC_VOUNT_COMMAND               , READ_OR_WRITE, 3, 2 },       /* code 21 */
    { PMBC_VOUT_TRIM                   , READ_OR_WRITE, 3, 2 },       /* code 22 */
    { PMBC_VOUT_CAL_OFFSET             , READ_OR_WRITE, 3, 2 },       /* code 23 */
    { PMBC_VOUT_MAX                    , READ_OR_WRITE, 3, 2 },       /* code 24 */
    { PMBC_VOUT_MARGIN_HIGH            , READ_OR_WRITE, 3, 2 },       /* code 25 */
    { PMBC_VOUT_MARGIN_LOW             , READ_OR_WRITE, 3, 2 },       /* code 26 */
    { PMBC_VOUT_TRANSITION_RATE        , READ_OR_WRITE, 3, 2 },       /* code 27 */
    { PMBC_VOUT_DROOP                  , READ_OR_WRITE, 3, 2 },       /* code 28 */
    { PMBC_VOUT_SCALE_LOOP             , READ_OR_WRITE, 3, 2 },       /* code 29 */
    { PMBC_VOUT_SCALE_MONITOR          , READ_OR_WRITE, 3, 2 },       /* code 2A */
#ifdef  PMBUS13
    { PMBC_VOUT_MIN                    , READ_OR_WRITE, 3, 2 },       /* code 2B */
#endif
    { PMBC_COEFICIENTS                 , BLK_PRC_CALL, 3, 5 },        /* code 30 */
    { PMBC_POUT_MAX                    , READ_OR_WRITE, 3, 2 },       /* code 31 */
    { PMBC_MAX_DUTY                    , READ_OR_WRITE, 3, 2 },       /* code 32 */
    { PMBC_FREQUENCY_SWITCH            , READ_OR_WRITE, 3, 2 },       /* code 33 */
#ifdef  PMBUS13
    { PMBC_POWER_MODE                  , READ_OR_WRITE, 2, 1 },       /* code 34 */
#endif
    { PMBC_VIN_ON                      , READ_OR_WRITE, 3, 2 },       /* code 35 */
    { PMBC_VIN_OFF                     , READ_OR_WRITE, 3, 2 },       /* code 36 */
    { PMBC_INTERLEAVE                  , READ_OR_WRITE, 3, 2 },       /* code 37 */
    { PMBC_IOUT_CAL_GAIN               , READ_OR_WRITE, 3, 2 },       /* code 38 */
    { PMBC_IOUT_CAL_OFFSET             , READ_OR_WRITE, 3, 2 },       /* code 39 */
    { PMBC_FAN_CONFIG_1_2              , READ_OR_WRITE, 2, 1 },       /* code 3A */
    { PMBC_FAN_COMMAND_1               , READ_OR_WRITE, 3, 2 },       /* code 3B */
    { PMBC_FAN_COMMAND_2               , READ_OR_WRITE, 3, 2 },       /* code 3C */
    { PMBC_FAN_CONFIG_3_4              , READ_OR_WRITE, 2, 1 },       /* code 3D */
    { PMBC_FAN_COMMAND_3               , READ_OR_WRITE, 3, 2 },       /* code 3E */
    { PMBC_FAN_COMMAND_4               , READ_OR_WRITE, 3, 2 },       /* code 3F */
    { PMBC_VOUT_OV_FAULT_LIMIT         , READ_OR_WRITE, 3, 2 },       /* code 40 */
    { PMBC_VOUT_OV_FAULT_RESPONSE      , READ_OR_WRITE, 2, 1 },       /* code 41 */
    { PMBC_VOUT_OV_WARN_LIMIT          , READ_OR_WRITE, 3, 2 },       /* code 42 */
    { PMBC_VOUT_UV_WARN_LIMIT          , READ_OR_WRITE, 3, 2 },       /* code 43 */
    { PMBC_VOUT_UV_FAULT_LIMIT         , READ_OR_WRITE, 3, 2 },       /* code 44 */
    { PMBC_VOUT_UV_FAULT_RESPONSE      , READ_OR_WRITE, 3, 2 },       /* code 45 */
    { PMBC_IOUT_OC_FAULT_LIMIT         , READ_OR_WRITE, 2, 1 },       /* code 46 */
    { PMBC_IOUT_OC_FAULT_RESPONSE      , READ_OR_WRITE, 3, 2 },       /* code 47 */
    { PMBC_IOUT_OC_LV_FAULT_LIMIT      , READ_OR_WRITE, 3, 2 },       /* code 48 */
    { PMBC_IOUT_OC_LV_FAULT_RESPONSE   , READ_OR_WRITE, 2, 1 },       /* code 49 */
    { PMBC_IOUT_OC_WARN_LIMIT          , READ_OR_WRITE, 3, 2 },       /* code 4A */
    { PMBC_IOUT_UC_FAULT_LIMIT         , READ_OR_WRITE, 3, 2 },       /* code 4B */
    { PMBC_IOUT_UC_FAULT_RESPONSE      , READ_OR_WRITE, 2, 1 },       /* code 4C */
    { PMBC_OT_FAULT_LIMIT              , READ_OR_WRITE, 3, 2 },       /* code 4F */
    { PMBC_OT_FAULT_RESPONSE           , READ_OR_WRITE, 2, 1 },       /* code 50 */
    { PMBC_OT_WARN_LIMIT               , READ_OR_WRITE, 3, 2 },       /* code 51 */
    { PMBC_UT_WARN_LIMIT               , READ_OR_WRITE, 3, 2 },       /* code 52 */
    { PMBC_UT_FAULT_LIMIT              , READ_OR_WRITE, 3, 2 },       /* code 53 */
    { PMBC_UT_FAULT_RESPONSE           , READ_OR_WRITE, 2, 1 },       /* code 54 */
    { PMBC_VIN_OV_FAULT_LIMIT          , READ_OR_WRITE, 3, 2 },       /* code 55 */
    { PMBC_VIN_OV_FAULT_RESPONSE       , READ_OR_WRITE, 2, 1 },       /* code 56 */
    { PMBC_VIN_OV_WARN_LIMIT           , READ_OR_WRITE, 3, 2 },       /* code 57 */
    { PMBC_VIN_UV_WARN_LIMIT           , READ_OR_WRITE, 3, 2 },       /* code 58 */
    { PMBC_VIN_UV_FAULT_LIMIT          , READ_OR_WRITE, 3, 2 },       /* code 59 */
    { PMBC_VIN_UV_FAULT_RESPONSE       , READ_OR_WRITE, 2, 1 },       /* code 5A */
    { PMBC_IIN_OC_FAULT_LIMIT          , READ_OR_WRITE, 3, 2 },       /* code 5B */
    { PMBC_IIN_OC_FAULT_RESPONSE       , READ_OR_WRITE, 2, 1 },       /* code 5C */
    { PMBC_IIN_OC_WARN_LIMIT           , READ_OR_WRITE, 3, 2 },       /* code 5D */
    { PMBC_POWER_GOOD_ON               , READ_OR_WRITE, 3, 2 },       /* code 5E */
    { PMBC_POWER_GOOD_OFF              , READ_OR_WRITE, 3, 2 },       /* code 5F */
    { PMBC_TON_DELAY                   , READ_OR_WRITE, 3, 2 },       /* code 60 */
    { PMBC_TON_RISE                    , READ_OR_WRITE, 3, 2 },       /* code 61 */
    { PMBC_TON_MAX_FAULT_LIMIT         , READ_OR_WRITE, 3, 2 },       /* code 62 */
    { PMBC_TON_MAX_FAULT_RESPONSE      , READ_OR_WRITE, 2, 1 },       /* code 63 */
    { PMBC_TOFF_DELAY                  , READ_OR_WRITE, 3, 2 },       /* code 64 */
    { PMBC_TOFF_FALL                   , READ_OR_WRITE, 3, 2 },       /* code 65 */
    { PMBC_TOFF_MAX_WARN_LIMIT         , READ_OR_WRITE, 3, 2 },       /* code 66 */
    { PMBC_POUT_OP_FAULT_LIMIT         , READ_OR_WRITE, 3, 2 },       /* code 68 */
    { PMBC_POUT_OP_FAULT_RESPONSE      , READ_OR_WRITE, 2, 1 },       /* code 69 */
    { PMBC_POUT_OP_WARN_LIMIT          , READ_OR_WRITE, 3, 2 },       /* code 6A */
    { PMBC_PIN_OP_WARN_LIMIT           , READ_OR_WRITE, 3, 2 },       /* code 6B */
#ifdef  PMBUS12
    { PMBC_STATUS_BYTE                 , READ_OR_WRITE, 2, 1 },       /* code 78 */
    { PMBC_STATUS_WORD                 , READ_OR_WRITE, 3, 2 },       /* code 79 */
#else
    { PMBC_STATUS_BYTE                 , READ, 1, 1 },                /* code 78 */
    { PMBC_STATUS_WORD                 , READ, 1, 2 },                /* code 79 */
#endif
    { PMBC_STATUS_VOUT                 , READ, 1, 1 },                /* code 7A */
    { PMBC_STATUS_IOUT                 , READ, 1, 1 },                /* code 7B */
    { PMBC_STATUS_INPUT                , READ, 1, 1 },                /* code 7C */
    { PMBC_STATUS_TEMPERATURE          , READ, 1, 1 },                /* code 7D */
    { PMBC_STATUS_CML                  , READ, 1, 1 },                /* code 7E */
    { PMBC_STATUS_OTHER                , READ, 1, 1 },                /* code 7F */
    { PMBC_STATUS_MFR_SPECIFIC         , READ, 1, 1 },                /* code 80 */
    { PMBC_STATUS_FANS_1_2             , READ, 1, 1 },                /* code 81 */
    { PMBC_STATUS_FANS_3_4             , READ, 1, 1 },                /* code 82 */
#ifdef  PMBUS13
    { PMBC_READ_KWH_IN                 , READ, 1, 4 },                /* code 83 */
    { PMBC_READ_KWH_OUT                , READ, 1, 4 },                /* code 84 */
    { PMBC_READ_KWH_CONFIG             , READ_OR_WRITE, 3, 2 },       /* code 85 */
#endif
#ifdef  PMBUS12
    { PMBC_READ_EIN                    , BLOCK_READ, 1, 6 },          /* code 87 */
    { PMBC_READ_EOUT                   , BLOCK_READ, 1, 6 },          /* code 87 */
#endif
    { PMBC_READ_VIN                    , READ, 1, 2 },                /* code 88 */
    { PMBC_READ_IIN                    , READ, 1, 2 },                /* code 89 */
    { PMBC_READ_VCAP                   , READ, 1, 2 },                /* code 8A */
    { PMBC_READ_VOUT                   , READ, 1, 2 },                /* code 8B */
    { PMBC_READ_IOUT                   , READ, 1, 2 },                /* code 8C */
    { PMBC_READ_TEMPERATURE_1          , READ, 1, 2 },                /* code 8D */
    { PMBC_READ_TEMPERATURE_2          , READ, 1, 2 },                /* code 8E */
    { PMBC_READ_TEMPERATURE_3          , READ, 1, 2 },                /* code 8F */
    { PMBC_READ_FAN_SPEED_1            , READ, 1, 2 },                /* code 90 */
    { PMBC_READ_FAN_SPEED_2            , READ, 1, 2 },                /* code 91 */
    { PMBC_READ_FAN_SPEED_3            , READ, 1, 2 },                /* code 92 */
    { PMBC_READ_FAN_SPEED_4            , READ, 1, 2 },                /* code 93 */
    { PMBC_READ_DUTY_CYCLE             , READ, 1, 2 },                /* code 94 */
    { PMBC_READ_FREQUENCY              , READ, 1, 2 },                /* code 95 */
    { PMBC_READ_POUT                   , READ, 1, 2 },                /* code 96 */
    { PMBC_READ_PIN                    , READ, 1, 2 },                /* code 97 */
    { PMBC_PMBUS_REVISION              , READ, 1, 1 },                /* code 98 */
    { PMBC_MFR_ID                      , BLK_RD_OR_WR, 1, 1 },        /* code 99 */
    { PMBC_MFR_MODEL                   , BLK_RD_OR_WR, 1, 1 },        /* code 9A */
    { PMBC_MFR_REVISION                , BLK_RD_OR_WR, 1, 1 },        /* code 9B */
    { PMBC_MFR_LOCATION                , BLK_RD_OR_WR, 1, 1 },        /* code 9C */
    { PMBC_MFR_DATE                    , BLK_RD_OR_WR, 1, 1 },        /* code 9D */
    { PMBC_MFR_SERIAL                  , BLK_RD_OR_WR, 1, 1 },        /* code 9E */
#ifdef  PMBUS12
    { PMBC_APP_PROFILE_SUPPORT         , BLOCK_READ, 1, 2 },          /* code 9F */
#endif
    { PMBC_MFR_VIN_MIN                 , READ, 1, 2 },                /* code A0 */
    { PMBC_MFR_VIN_MAX                 , READ, 1, 2 },                /* code A1 */
    { PMBC_MFR_IIN_MAX                 , READ, 1, 2 },                /* code A2 */
    { PMBC_MFR_PIN_MAX                 , READ, 1, 2 },                /* code A3 */
    { PMBC_MFR_VOUT_MIN                , READ, 1, 2 },                /* code A4 */
    { PMBC_MFR_VOUT_MAX                , READ, 1, 2 },                /* code A5 */
    { PMBC_MFR_IOUT_MAX                , READ, 1, 2 },                /* code A6 */
    { PMBC_MFR_POUT_MAX                , READ, 1, 2 },                /* code A7 */
    { PMBC_MFR_TAMBIENT_MAX            , READ, 1, 2 },                /* code A8 */
    { PMBC_MFR_TAMBIENT_MIN            , READ, 1, 2 },                /* code A9 */
    { PMBC_MFR_EFFICIENCY_LL           , BLK_RD_OR_WR, 15, 14 },      /* code AA */
    { PMBC_MFR_EFFICIENCY_HL           , BLK_RD_OR_WR, 15, 14 },      /* code AB */
#ifdef  PMBUS12
    { PMBC_MFR_PIN_ACCURACY            , READ, 1, 1 },                /* code AC */
    { PMBC_IC_DEVICE_ID                , BLOCK_READ, 1, 2},           /* code AD */
    { PMBC_IC_DEVICE_REV               , BLOCK_READ, 1, 2},           /* code AE */
#endif
    { PMBC_USER_DATA_00                , BLK_RD_OR_WR, 1, 1 },        /* code B0 */
    { PMBC_USER_DATA_01                , BLK_RD_OR_WR, 1, 1 },        /* code B1 */
    { PMBC_USER_DATA_02                , BLK_RD_OR_WR, 1, 1 },        /* code B2 */
    { PMBC_USER_DATA_03                , BLK_RD_OR_WR, 1, 1 },        /* code B3 */
    { PMBC_USER_DATA_04                , BLK_RD_OR_WR, 1, 1 },        /* code B4 */
    { PMBC_USER_DATA_05                , BLK_RD_OR_WR, 1, 1 },        /* code B5 */
    { PMBC_USER_DATA_06                , BLK_RD_OR_WR, 1, 1 },        /* code B6 */
    { PMBC_USER_DATA_07                , BLK_RD_OR_WR, 1, 1 },        /* code B7 */
    { PMBC_USER_DATA_08                , BLK_RD_OR_WR, 1, 1 },        /* code B8 */
    { PMBC_USER_DATA_09                , BLK_RD_OR_WR, 1, 1 },        /* code B9 */
    { PMBC_USER_DATA_10                , BLK_RD_OR_WR, 1, 1 },        /* code BA */
    { PMBC_USER_DATA_11                , BLK_RD_OR_WR, 1, 1 },        /* code BB */
    { PMBC_USER_DATA_12                , BLK_RD_OR_WR, 1, 1 },        /* code BC */
    { PMBC_USER_DATA_13                , BLK_RD_OR_WR, 1, 1 },        /* code BD */
    { PMBC_USER_DATA_14                , BLK_RD_OR_WR, 1, 1 },        /* code BE */
    { PMBC_USER_DATA_15                , BLK_RD_OR_WR, 1, 1 },        /* code BF */
#ifdef  PMBUS12
    { PMBC_MFR_MAX_TEMP_1              , READ_OR_WRITE, 3, 2 },       /* code C0 */
    { PMBC_MFR_MAX_TEMP_2              , READ_OR_WRITE, 3, 2 },       /* code C1 */
    { PMBC_MFR_MAX_TEMP_3              , READ_OR_WRITE, 3, 2 },       /* code C2 */
#endif
  };

st_command_t PMBUS_COMMANDS_TEST[] =
  {
    { 0, WRITE, 2, 0 },
    { 1, BLK_PRC_CALL, 5, 4 },
    { 2, BLOCK_WRITE, 3, 0 },
    { 3, READ, 1, 2 },
    { 4, BLOCK_READ, 1, 4 },
    { 5, PROCESS_CALL, 3, 2},
    { 6, READ_OR_WRITE, 1, 1 },
    { 7, BLK_RD_OR_WR, 1, 1 }
  };



/** @defgroup STM32_PMBUS_STACK_Functions
  * @{
  */

#define		SEM_WATI_TIME		1000
  
static SMBUS_StackHandleTypeDef context;
static SMBUS_StackHandleTypeDef *pcontext = &context;
static SemaphoreHandle_t  xSemaphore = NULL;

/**
  * @brief  PMBUS master group command transmit, direction is implicitly write.
  * @param  pStackContext : Pointer to a SMBUS_StackHandleTypeDef structure that contains
  *                the configuration information for the specified SMBUS.
  * @param  pCommand : description of the command to be transmitted, NULL for quick command
  * @retval SMBus stack return code
  */
StatusTypeDef STACK_PMBUS_HostCommandGroup(SMBUS_StackHandleTypeDef *pStackContext)
{
	StatusTypeDef     result = STACK_ERROR;

	xSemaphoreTake( xSemaphore, (uint16_t)SEM_WATI_TIME/portTICK_RATE_MS );  // 获取信号量
	
//***********************************enter critical*******************************/
//taskENTER_CRITICAL();	
	if ( pStackContext->StateMachine != SMBUS_SMS_READY )
	{
		//***********************************exit critical*******************************/
//        taskEXIT_CRITICAL();
		return STACK_BUSY;
	}

	pStackContext->StateMachine = SMBUS_SMS_TRANSMIT;

	result = SMBUS_Master_Transmit(pStackContext);

	pStackContext->StateMachine = SMBUS_SMS_READY;
	
//***********************************exit critical*******************************/
//taskEXIT_CRITICAL();
	xSemaphoreGive( xSemaphore );  //给出互斥信号量 xSemaphore
	
	return result;
}


void sInitPMBus(void)
{
	pcontext->StateMachine = SMBUS_SMS_READY;

	/* 创建互斥信号量 */
	if ( xSemaphore == NULL )
	{
		xSemaphore = xSemaphoreCreateMutex();
	}
	
	Init_SMBus();
}

SMBUS_StackHandleTypeDef * pGetContext(void)
{
	return pcontext;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

