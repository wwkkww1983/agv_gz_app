#ifndef __BSP_RCB1600_48_H
#define __BSP_RCB1600_48_H

#include "bsp_pmbus_stack.h"
#include "uart.h"
#include "string.h"
#include "FreeRTOS.h"
#include "task.h"


// 控制选择48v还是24v，还有一处开关在app.h中，两处同时控制才会生效
#define _RCB_1600_48
//#define  _RCB_1600_24




#define		DEV_A		0x00
#define		DEV_B		0x01
#define		DEV_C		0x02
#define		DEV_D		0x03
#define		DEV_E		0x04


#define		CURVE_1				0x01
#define		CURVE_2				0x02											  
#define		CURVE_3				0x04										  
#define		CURVE_DEFAULT		0x08										  

										  
/*
 * RCB1600，电池充电配置定义，来自RCB-1600手册@page-16
 */
// Low byte
//Bit 1-0  CUVS ：充電曲線選擇
//00＝載入客戶燒錄充電曲線(default)
//01＝載入預設充電曲線#1
//10＝載入預設充電曲線#2
//11＝載入預設充電曲線#3
#define		CUVS				0x0003
#define		CUVS_DEFAULT		0x00
#define		CUVS_1				0x01
#define		CUVS_2				0x02
#define		CUVS_3				0x03

//Bit 3-2  TCS：溫度補償設定
//00＝ disable
//01＝ -3 mV/℃/cell (default)
//10＝ -4 mV/℃/cell
//11＝ -5 mV/℃/cell
#define		TCS					0x000C
#define		TCS_DISABLE			0x00
#define		TCS_3				0x01
#define		TCS_4				0x02
#define		TCS_5				0x03

//Bit 6  STGS：2/3段充電設定
//0＝ 3段充電 (default, CURVE_VBST and CURVE_V FLOAT)
//1＝ 2段充電 (only CURVE_VBST)
#define		STGS				0x0040

//High byte
//Bit 0 CCTOE：定電流階段充電超時指示開關
//0＝ 關閉 (default)
//1＝ 開啟
#define		CCTOE				0x0100

//Bit 1 CVTOE：定電壓階段充電超時指示開關
//0＝ 關閉 (default)
//1＝ 開啟
#define		CVTOE				0x0200

//Bit 2 FTTOE：浮充階段充電超時指示開關
//0＝ 關閉 (default)
//1＝ 開啟
#define		FVTOE				0x0400


										  
#ifdef	_RCB_1600_48										  
#define		DEV_DEFAULT_OUTPUT		((int8_t)48)
#define		DEV_MAX_OUTPUT			((int8_t)60)
#define		DEV_MIN_OUTPUT			((int8_t)36)

#elif defined _RCB_1600_24
#define		DEV_DEFAULT_OUTPUT		((int8_t)24)
#define		DEV_MAX_OUTPUT			((int8_t)30)
#define		DEV_MIN_OUTPUT			((int8_t)18)
#endif
extern uint8_t Device_List[5];


void sInitRCB1600(void);

uint8_t ChkAddrDev(uint8_t _addr);

void SetVout(uint8_t _addr, float voltage);
void ReadVout(uint8_t _addr, float *voltage);
void SetIout(uint8_t _addr, float _iout);
void ReadIout(uint8_t _addr, float *_iout);
void ReadChgStatus(uint8_t _addr, uint16_t *p_status);
void ReadChgStatus_St(uint8_t _addr, uint16_t *p_status, StatusTypeDef *st);
void ReadChgConfig(uint8_t _addr, uint16_t *p_config);
int8_t ReadTemperature( uint8_t _addr );
uint16_t ReadFanSpeed_1( uint8_t _addr );
uint16_t ReadFanSpeed_2( uint8_t _addr );
uint16_t ReadVin( uint8_t _addr );

float fReadCURVE_ICHG(uint8_t _addr);
float fReadCURVE_ITAPER(uint8_t _addr);
float fReadCURVE_VBST(uint8_t _addr);
float fReadCURVE_VFLOAT(uint8_t _addr);

void WriteCURVE_ICHG(uint8_t _addr, float ichg);
void WriteCURVE_ITAPER(uint8_t _addr, float itaper);
void WriteCURVE_VBST(uint8_t _addr, float vbst);
void WriteCURVE_VFLOAT(uint8_t _addr, float vfloat);


void SelectChgrCurve(uint8_t addr, uint8_t curve);

void ReadIout_St(uint8_t _addr, float *_iout, StatusTypeDef *status);
void ReadVout_St(uint8_t _addr, float *_vout, StatusTypeDef *status );
void SetVout_St(uint8_t _addr, float _voltage, StatusTypeDef *status );

void ReadMFR_ID(uint8_t _addr, uint8_t *ptr);
void ReadMFR_Model(uint8_t _addr, uint8_t *ptr);
void ReadMFR_Revision(uint8_t addr, uint8_t *ptr);
void ReadMFR_Location(uint8_t _addr, uint8_t *ptr);
void ReadMFR_Date(uint8_t _addr, uint8_t *ptr);
void ReadMFR_Serial(uint8_t _addr, uint8_t *ptr);


void SetIoutOC_FaultLimit_St(uint8_t _addr, float iout_limit, StatusTypeDef *status );
void ReadIoutOC_FaultLimit_St(uint8_t _addr, float *iout_limit, StatusTypeDef *status );


#endif



