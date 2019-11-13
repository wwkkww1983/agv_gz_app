#include "bsp_rpb1600_48.h"
#include "bsp_smbus_stack.h"



#ifndef __LOG
#define    __LOG_DEBUG    0
#endif
#define LOG_DEBUG(fmt,arg...)         do{\
                                          if(__LOG_DEBUG)\
											  printf("\r\n<<-DEBUG_DRIVER->> "fmt"\n",##arg);\
                                          }while(0)



#define			WAIT_TIME		15

#define         N_READ_VOUT        -9
#define         N_READ_IOUT        -2
#define         N_ICHG             -2
#define         N_ITAPER           -2
#define         N_FAN_SPEED_1       5
#define         N_FAN_SPEED_2       5
#define         N_READ_VIN         -1
#define         N_IOUT_OC_FAULT_LIMIT    -2


//static uint8_t Device_List[5] = {0x82, 0x84, 0x86, 0x88, 0x8A};

static SMBUS_StackHandleTypeDef *pcontext = NULL;

st_command_t PMBUS_COMMANDS_RCB1600[] = 
{																				
	{ PMBC_OPERATION                   , READ_OR_WRITE, 1, 1 },       /* code 01, idx = 0 */ 	
	{ PMBC_ON_OFF_CONFIG               , READ_OR_WRITE, 1, 1 },       /* code 02, idx = 1 */
	{ PMBC_CAPABILITY                  , READ, 1, 1 },                /* code 19, idx = 2 */
    { PMBC_VOUT_MODE                   , READ_OR_WRITE, 1, 1 },       /* code 20, idx = 3 */
    { PMBC_VOUNT_COMMAND               , READ_OR_WRITE, 2, 2 },       /* code 21, idx = 4 */
    { PMBC_VOUT_TRIM                   , READ_OR_WRITE, 2, 2 },       /* code 22, idx = 5 */
    { PMBC_IOUT_OC_FAULT_LIMIT         , READ_OR_WRITE, 2, 2 },       /* code 46, idx = 6 */
    { PMBC_IOUT_OC_FAULT_RESPONSE      , READ_OR_WRITE, 1, 1 },       /* code 47, idx = 7 */
	{ PMBC_STATUS_WORD                 , READ, 2, 2 },                /* code 79, idx = 8 */
    { PMBC_STATUS_VOUT                 , READ, 1, 1 },                /* code 7A, idx = 9 */
    { PMBC_STATUS_IOUT                 , READ, 1, 1 },                /* code 7B, idx = 10 */
    { PMBC_STATUS_INPUT                , READ, 1, 1 },                /* code 7C, idx = 11 */
    { PMBC_STATUS_TEMPERATURE          , READ, 1, 1 },                /* code 7D, idx = 12 */
    { PMBC_STATUS_CML                  , READ, 1, 1 },                /* code 7E, idx = 13 */
    { PMBC_STATUS_MFR_SPECIFIC         , READ, 1, 1 },                /* code 80, idx = 14 */
    { PMBC_STATUS_FANS_1_2             , READ, 1, 1 },                /* code 81, idx = 15 */
    { PMBC_READ_VIN                    , READ, 2, 2 },                /* code 88, idx = 16 */
    { PMBC_READ_VOUT                   , READ, 2, 2 },                /* code 8B, idx = 17 */
    { PMBC_READ_IOUT                   , READ, 2, 2 },                /* code 8C, idx = 18 */
    { PMBC_READ_TEMPERATURE_1          , READ, 2, 2 },                /* code 8D, idx = 19 */
    { PMBC_READ_FAN_SPEED_1            , READ, 2, 2 },                /* code 90, idx = 20 */
    { PMBC_READ_FAN_SPEED_2            , READ, 2, 2 },                /* code 91, idx = 21 */
    { PMBC_PMBUS_REVISION              , READ, 1, 1 },                /* code 98, idx = 22 */
    { PMBC_MFR_ID                      , BLK_RD_OR_WR, 12, 12 },      /* code 99, idx = 23 */
    { PMBC_MFR_MODEL                   , BLK_RD_OR_WR, 12, 12 },      /* code 9A, idx = 24 */
    { PMBC_MFR_REVISION                , BLK_RD_OR_WR, 6, 6 },        /* code 9B, idx = 25 */
    { PMBC_MFR_LOCATION                , BLK_RD_OR_WR, 3, 3 },        /* code 9C, idx = 26 */
    { PMBC_MFR_DATE                    , BLK_RD_OR_WR, 6, 6 },        /* code 9D, idx = 27 */
    { PMBC_MFR_SERIAL                  , BLK_RD_OR_WR, 12, 12 },      /* code 9E, idx = 28 */
    { PMBC_CURVE_ICHG                  , BLK_RD_OR_WR, 2, 2 },        /* code B0, idx = 29 */
    { PMBC_CURVE_VBST                  , BLK_RD_OR_WR, 2, 2 },        /* code B1, idx = 30 */
    { PMBC_CURVE_VFLOAT                , BLK_RD_OR_WR, 2, 2 },        /* code B2, idx = 31 */
    { PMBC_CURVE_ITAPER                , BLK_RD_OR_WR, 2, 2 },        /* code B3, idx = 32 */
    { PMBC_CURVE_CONFIG                , BLK_RD_OR_WR, 2, 2 },        /* code B4, idx = 33 */
    { PMBC_CURVE_CC_TIMEOUT            , BLK_RD_OR_WR, 2, 2 },        /* code B5, idx = 34 */
    { PMBC_CURVE_CV_TIMEOUT            , BLK_RD_OR_WR, 2, 2 },        /* code B6, idx = 35 */
    { PMBC_CURVE_FLOAT_TIMEOUT         , BLK_RD_OR_WR, 2, 2 },        /* code B7, idx = 36 */
    { PMBC_CHG_STATUS                  , BLK_RD_OR_WR, 2, 2 },        /* code B8, idx = 37 */
};


#if 0
uint16_t real_to_linear( float X, int8_t N )
{
	uint16_t Y = 0;
	uint16_t idx = 0, square = 1, factor = 0;
	uint16_t pmbdata = 0;
	uint16_t tmp = 0;

	// 求Y的值
	if( ( N>>7 ) == 0 )
	{
		factor = N;
	    for( idx = 0; idx < factor; idx++ )
	    {
	        square *= 2;
	    }

		Y = X / square;
	}
	else
	{
		factor = ~( N - 1 );

	    for( idx = 0; idx < factor; idx++ )
	    {
	        square *= 2;
	    }

		Y = X * square;
	}

	// 组合数据
	tmp |= N;
	tmp <<= 11;
	pmbdata = tmp | ( Y & 0x07FF);
//	pmbdata = ( N << 11 ) | ( Y & 0x07FF);
	printf("\r\nset:linear:%04X\n", pmbdata);

	return pmbdata;
}

float linear_to_real( uint16_t linear, int8_t N )
{
	float ret = 0;
	uint8_t idx = 0, square = 1;
	uint16_t para = 0;

	printf("\r\nread: linear:%04X", linear);

	// 取出低11位
	ret = ( linear & 0x07FF );

	// 负数，补码存储，最高位是1
	if( ( N>>7 ) == 0 )
	{
	    for( idx = 0; idx < N; idx++ )
	    {
	        square *= 2;
	    }
		// 计算出实际值
		ret *= (float)square;
	}
	else
	{
		N = ~( N - 1 );

	    for( idx = 0; idx < N; idx++ )
	    {
	        square *= 2;
	    }

		// 计算出实际值
		ret /= (float)square;
	}

	printf("\r\nret:%f", ret);

	return ret;
}

#endif


#if 1

uint16_t real_to_linear( float X, int8_t N )
{
	uint16_t Y = 0;
	uint16_t idx = 0, square = 1;
	uint16_t pmbdata = 0;
	uint16_t tmp = 0;
	uint8_t factor = 0;

	// 求Y的值
    //正数
	if( ( N & 0x80 ) == 0 ) 
	{
		factor = N;
	    for( idx = 0; idx < factor; idx++ )
	    {
	        square *= 2;
	    }

		Y = X / square;
	}
    //负数
	else
	{
		factor = ~( N - 1 );

	    for( idx = 0; idx < factor; idx++ )
	    {
	        square *= 2;
	    }

		Y = X * square;
	}

	// 组合数据
	tmp |= N;
	tmp <<= 11;
	pmbdata = tmp | ( Y & 0x07FF);
//	pmbdata = ( N << 11 ) | ( Y & 0x07FF);
//	printf("\r\nset:linear:%04X\n", pmbdata);

	return pmbdata;
}

float linear_to_real( uint16_t linear, int8_t N )
{
	int16_t Y = 0;
	int8_t tmpN = 0;
	float X = 0;
	uint16_t idx = 0, square = 1;

//	printf("\r\nread: linear:%04X", linear);

	// 取出低11位
	Y = ( linear & 0x07FF ) & 0xFFFF;

	// 负数，补码存储，最高位是1
	if( ( N & 0x80 ) == 0 ) // +
	{
		tmpN = N;
	    for( idx = 0; idx < N; idx++ )
	    {
	        square *= 2;
	    }
		// 计算出实际值
		X = (float)Y * square;
	}
	else // -
	{
		tmpN = ~( N - 1 );

	    for( idx = 0; idx < tmpN; idx++ )
	    {
	        square *= 2;
	    }

		// 计算出实际值
		X = (float)Y / square;
	}


//	printf("\r\nY:%04X, X:%f, N:%d", Y, X, N);

	return X;
}

#endif


float ioc_linear_to_real( int16_t ioc )
{
	return ( ioc & 0x07FF ) / ( float )4;
}

int16_t ioc_real_to_linear( float ioc )
{
	return ((int16_t)(ioc*4 + 59392));
}


/* Private functions ---------------------------------------------------------*/
static int16_t vout_trim_to_pmbdata(float linear_data)
{
	return ((int16_t)((linear_data - DEV_DEFAULT_OUTPUT) * 512));
}

static float vout_to_linear_data(uint16_t pmbus_data)
{
	return (pmbus_data / (float)512);	
}
static uint16_t vbst_to_pmdata(float vbst)
{
	return (uint16_t)(vbst * 512);
}

static uint16_t vfloat_to_pmdata(float vfloat)
{
	return (uint16_t)(vfloat * 512);
}

static uint16_t iout_to_pmbdata(float linear_data)
{
	return ((uint16_t)(linear_data*8 + 59392));
}

static float iout_to_linear_data(uint16_t pmbus_data)
{
    return (((pmbus_data & 0x07FF)/(float)4));
}

static int8_t temp_to_linear_data( uint16_t pmbus_data )
{
    return ( ( ( pmbus_data - 59392 ) / (int8_t)8 ) );
}

static uint16_t fan_speed_to_linear_data( uint16_t pmbus_data )
{
//    return ( ( pmbus_data - 8192) * 32 );
    return ( ( pmbus_data & 0x07FF) * 32 );
}

static uint16_t vin_to_linear_data( uint16_t pmbus_data )
{
    return ( ( pmbus_data & 0x07FF )/2 );
}


/* Global functions ---------------------------------------------------------*/
void sInitRCB1600(void)
{
	pcontext = pGetContext();
}

// 返回0，表示设备没有响应，返回1，表示设备响应
uint8_t ChkAddrDev(uint8_t _addr)
{
	pcontext->SlaveAddress = _addr;
	pcontext->OpMode = CHK_ADDR_DEVICE;
	
	if ( STACK_PMBUS_HostCommandGroup(pcontext) != SMBUS_OK)
		return 0;
	else
		return 1;
}

void SetVout(uint8_t _addr, float _voltage)
{
	uint16_t voltage = vout_trim_to_pmbdata(_voltage);
		
	pcontext->SlaveAddress = _addr;
	pcontext->OpMode = WRITE;
	pcontext->CMD_table = &PMBUS_COMMANDS_RCB1600[5];
	pcontext->Byte_count = PMBUS_COMMANDS_RCB1600[5].cmnd_master_Tx_size;
	memcpy(pcontext->Buffer, &voltage, pcontext->Byte_count);	
	
	if ( STACK_PMBUS_HostCommandGroup(pcontext) != SMBUS_OK)
		LOG_DEBUG("\r\nSetVout failed\r\n");	
}

void SetVout_St(uint8_t _addr, float _voltage, StatusTypeDef *status )
{
	uint16_t voltage = vout_trim_to_pmbdata(_voltage);
		
	pcontext->SlaveAddress = _addr;
	pcontext->OpMode = WRITE;
	pcontext->CMD_table = &PMBUS_COMMANDS_RCB1600[5];
	pcontext->Byte_count = PMBUS_COMMANDS_RCB1600[5].cmnd_master_Tx_size;
	memcpy(pcontext->Buffer, &voltage, pcontext->Byte_count);	
	
	if ( ( *status = STACK_PMBUS_HostCommandGroup(pcontext) ) != SMBUS_OK)
	{
	    pcontext->StateMachine = SMBUS_SMS_READY;
		LOG_DEBUG("\r\nSetVout failed\r\n");	
    }
}

void ReadVout_St(uint8_t _addr, float *_vout, StatusTypeDef *status )
{
	uint16_t voltate = 0;

	pcontext->SlaveAddress = _addr;
	pcontext->OpMode = READ;
	pcontext->CMD_table = &PMBUS_COMMANDS_RCB1600[17];
	pcontext->Byte_count = PMBUS_COMMANDS_RCB1600[17].cmnd_master_Rx_size;

	if ( ( *status = STACK_PMBUS_HostCommandGroup(pcontext) ) != SMBUS_OK)
	{
	    pcontext->StateMachine = SMBUS_SMS_READY;
		LOG_DEBUG("ReadVout failed");
	}

	memcpy(&voltate, pcontext->Buffer, pcontext->Byte_count);

	*_vout = vout_to_linear_data(voltate);
}

void ReadVout(uint8_t _addr, float *_vout)
{
	uint16_t voltate = 0;

	pcontext->SlaveAddress = _addr;
	pcontext->OpMode = READ;
	pcontext->CMD_table = &PMBUS_COMMANDS_RCB1600[17];
	pcontext->Byte_count = PMBUS_COMMANDS_RCB1600[17].cmnd_master_Rx_size;

	if ( STACK_PMBUS_HostCommandGroup(pcontext) != SMBUS_OK)
		LOG_DEBUG("ReadVout failed");

	memcpy(&voltate, pcontext->Buffer, pcontext->Byte_count);

	*_vout = vout_to_linear_data(voltate);
}


void SetIout(uint8_t _addr, float _iout)
{
	uint16_t current = iout_to_pmbdata(_iout);
		
	pcontext->SlaveAddress = _addr;
	pcontext->OpMode = WRITE;
	pcontext->CMD_table = &PMBUS_COMMANDS_RCB1600[6];
	pcontext->Byte_count = PMBUS_COMMANDS_RCB1600[6].cmnd_master_Tx_size;
	memcpy(pcontext->Buffer, &current, pcontext->Byte_count);	
	
	if ( STACK_PMBUS_HostCommandGroup(pcontext) != SMBUS_OK)
		LOG_DEBUG("\r\nSetIout failed\r\n");
}

void ReadIout(uint8_t _addr, float *_iout)
{
	uint16_t current = 0;

	pcontext->SlaveAddress = _addr;
	pcontext->OpMode = READ;
	pcontext->CMD_table = &PMBUS_COMMANDS_RCB1600[18];
	pcontext->Byte_count = PMBUS_COMMANDS_RCB1600[18].cmnd_master_Rx_size;

	if ( STACK_PMBUS_HostCommandGroup(pcontext) != SMBUS_OK)
		LOG_DEBUG("\r\nReadVout failed\r\n");

	memcpy(&current, pcontext->Buffer, pcontext->Byte_count);

	*_iout = iout_to_linear_data(current);
}

void ReadIout_St(uint8_t _addr, float *_iout, StatusTypeDef *status)
{
	uint16_t current = 0;

	pcontext->SlaveAddress = _addr;
	pcontext->OpMode = READ;
	pcontext->CMD_table = &PMBUS_COMMANDS_RCB1600[18];
	pcontext->Byte_count = PMBUS_COMMANDS_RCB1600[18].cmnd_master_Rx_size;

	if ( ( *status = STACK_PMBUS_HostCommandGroup(pcontext) ) != SMBUS_OK)
	{
	    pcontext->StateMachine = SMBUS_SMS_READY;
		LOG_DEBUG("\r\nReadIout failed\r\n");
	}

	memcpy(&current, pcontext->Buffer, pcontext->Byte_count);

	*_iout = iout_to_linear_data(current);
}

float fReadCURVE_ICHG(uint8_t _addr)
{
	uint16_t cc = 0;

	pcontext->SlaveAddress = _addr;
	pcontext->OpMode = READ;
	pcontext->CMD_table = &PMBUS_COMMANDS_RCB1600[29];
	pcontext->Byte_count = PMBUS_COMMANDS_RCB1600[29].cmnd_master_Rx_size;

	if ( STACK_PMBUS_HostCommandGroup(pcontext) != SMBUS_OK)
		LOG_DEBUG("\r\nReadChg CURVE_ICHG failed\r\n");

	memcpy(&cc, pcontext->Buffer, pcontext->Byte_count);

//	return ( iout_to_linear_data(cc) );
//    return ( pmbdata_to_lineardata( cc, -2 ) );
    return linear_to_real( cc, N_ICHG );
}

float fReadCURVE_ITAPER(uint8_t _addr)
{
	uint16_t tc = 0;

	pcontext->SlaveAddress = _addr;
	pcontext->OpMode = READ;
	pcontext->CMD_table = &PMBUS_COMMANDS_RCB1600[32];
	pcontext->Byte_count = PMBUS_COMMANDS_RCB1600[32].cmnd_master_Rx_size;

	if ( STACK_PMBUS_HostCommandGroup(pcontext) != SMBUS_OK)
		LOG_DEBUG("\r\nReadChg CURVE_ITAPER failed\r\n");

	memcpy(&tc, pcontext->Buffer, pcontext->Byte_count);

//	return ( iout_to_linear_data(tc) );
    return ( linear_to_real( tc, N_ITAPER ) );

}

float fReadCURVE_VBST(uint8_t _addr)
{
	uint16_t vbst = 0;

	pcontext->SlaveAddress = _addr;
	pcontext->OpMode = READ;
	pcontext->CMD_table = &PMBUS_COMMANDS_RCB1600[30];
	pcontext->Byte_count = PMBUS_COMMANDS_RCB1600[30].cmnd_master_Rx_size;

	if ( STACK_PMBUS_HostCommandGroup(pcontext) != SMBUS_OK)
		LOG_DEBUG("\r\nReadChg CURVE_VBST failed\r\n");

	memcpy(&vbst, pcontext->Buffer, pcontext->Byte_count);

	return ( vout_to_linear_data(vbst) );
}

float fReadCURVE_VFLOAT(uint8_t _addr)
{
	uint16_t vf = 0;

	pcontext->SlaveAddress = _addr;
	pcontext->OpMode = READ;
	pcontext->CMD_table = &PMBUS_COMMANDS_RCB1600[31];
	pcontext->Byte_count = PMBUS_COMMANDS_RCB1600[31].cmnd_master_Rx_size;

	if ( STACK_PMBUS_HostCommandGroup(pcontext) != SMBUS_OK)
		LOG_DEBUG("\r\nReadChg CURVE_VFLOAT failed\r\n");

	memcpy(&vf, pcontext->Buffer, pcontext->Byte_count);

	return ( vout_to_linear_data(vf) );
}

void WriteCURVE_ICHG(uint8_t _addr, float ichg)
{
//	uint16_t current = iout_to_pmbdata(ichg);
	uint16_t current = real_to_linear( ichg, N_ICHG );
	
	pcontext->SlaveAddress = _addr;
	pcontext->OpMode = WRITE;
	pcontext->CMD_table = &PMBUS_COMMANDS_RCB1600[29];
	pcontext->Byte_count = PMBUS_COMMANDS_RCB1600[29].cmnd_master_Tx_size;
	memcpy(pcontext->Buffer, &current, pcontext->Byte_count);	
	
	if ( STACK_PMBUS_HostCommandGroup(pcontext) != SMBUS_OK)
		LOG_DEBUG("\r\nSet CURVE ICHG failed\r\n");
}

void WriteCURVE_ITAPER(uint8_t _addr, float itaper)
{
//	uint16_t current = iout_to_pmbdata(itaper);
	uint16_t current = real_to_linear( itaper, N_ITAPER );
//	printf("\r\nitaper:%f, after convert:%04X\n", itaper, current);
		
	pcontext->SlaveAddress = _addr;
	pcontext->OpMode = WRITE;
	pcontext->CMD_table = &PMBUS_COMMANDS_RCB1600[32];
	pcontext->Byte_count = PMBUS_COMMANDS_RCB1600[32].cmnd_master_Tx_size;
	memcpy(pcontext->Buffer, &current, pcontext->Byte_count);	
	
	if ( STACK_PMBUS_HostCommandGroup(pcontext) != SMBUS_OK)
		LOG_DEBUG("\r\nSet CURVE ITAPER failed\r\n");
}

void WriteCURVE_VBST(uint8_t _addr, float vbst)
{
	uint16_t voltage = vbst_to_pmdata(vbst);
		
	pcontext->SlaveAddress = _addr;
	pcontext->OpMode = WRITE;
	pcontext->CMD_table = &PMBUS_COMMANDS_RCB1600[30];
	pcontext->Byte_count = PMBUS_COMMANDS_RCB1600[30].cmnd_master_Tx_size;
	memcpy(pcontext->Buffer, &voltage, pcontext->Byte_count);	
	
	if ( STACK_PMBUS_HostCommandGroup(pcontext) != SMBUS_OK)
		LOG_DEBUG("\r\nSet CURVE VBST failed\r\n");	
}

void WriteCURVE_VFLOAT(uint8_t _addr, float vfloat)
{
	uint16_t voltage = vfloat_to_pmdata(vfloat);
		
	pcontext->SlaveAddress = _addr;
	pcontext->OpMode = WRITE;
	pcontext->CMD_table = &PMBUS_COMMANDS_RCB1600[31];
	pcontext->Byte_count = PMBUS_COMMANDS_RCB1600[31].cmnd_master_Tx_size;
	memcpy(pcontext->Buffer, &voltage, pcontext->Byte_count);	
	
	if ( STACK_PMBUS_HostCommandGroup(pcontext) != SMBUS_OK)
		LOG_DEBUG("\r\nSet CURVE VFLOAT failed\r\n");	
}

void ReadChgStatus(uint8_t _addr, uint16_t *p_status)
{
	uint16_t status = 0;

	pcontext->SlaveAddress = _addr;
	pcontext->OpMode = READ;
	pcontext->CMD_table = &PMBUS_COMMANDS_RCB1600[37];
	pcontext->Byte_count = PMBUS_COMMANDS_RCB1600[37].cmnd_master_Rx_size;

	if ( STACK_PMBUS_HostCommandGroup(pcontext) != SMBUS_OK)
		LOG_DEBUG("\r\nReadChgStatus failed\r\n");

	memcpy(&status, pcontext->Buffer, pcontext->Byte_count);

	*p_status = status;
}

void ReadChgStatus_St(uint8_t _addr, uint16_t *p_status, StatusTypeDef *st)
{
	uint16_t status = 0;

	pcontext->SlaveAddress = _addr;
	pcontext->OpMode = READ;
	pcontext->CMD_table = &PMBUS_COMMANDS_RCB1600[37];
	pcontext->Byte_count = PMBUS_COMMANDS_RCB1600[37].cmnd_master_Rx_size;

	if ( ( *st = STACK_PMBUS_HostCommandGroup(pcontext) ) != SMBUS_OK)
	{
	    pcontext->StateMachine = SMBUS_SMS_READY;
		LOG_DEBUG("\r\nReadChgStatus failed\r\n");
	}

	memcpy(&status, pcontext->Buffer, pcontext->Byte_count);

	*p_status = status;
}

void ReadChgConfig(uint8_t _addr, uint16_t *p_config)
{
	uint16_t config = 0;

	pcontext->SlaveAddress = _addr;
	pcontext->OpMode = READ;
	pcontext->CMD_table = &PMBUS_COMMANDS_RCB1600[33];
	pcontext->Byte_count = PMBUS_COMMANDS_RCB1600[33].cmnd_master_Rx_size;

	if ( STACK_PMBUS_HostCommandGroup(pcontext) != SMBUS_OK)
		LOG_DEBUG("\r\nReadChgConfig failed\r\n");

	memcpy(&config, pcontext->Buffer, pcontext->Byte_count);

	*p_config = config;	
}

int8_t ReadTemperature( uint8_t _addr )
{
	uint16_t temp = 0;

	pcontext->SlaveAddress = _addr;
	pcontext->OpMode = READ;
	pcontext->CMD_table = &PMBUS_COMMANDS_RCB1600[19];
	pcontext->Byte_count = PMBUS_COMMANDS_RCB1600[19].cmnd_master_Rx_size;

	if ( STACK_PMBUS_HostCommandGroup(pcontext) != SMBUS_OK)
		LOG_DEBUG("\r\nReadTemperature failed\r\n");

	memcpy(&temp, pcontext->Buffer, pcontext->Byte_count);
	LOG_DEBUG("\r\nin read, temp:%d", temp);

	return temp_to_linear_data( temp );
}

uint16_t ReadFanSpeed_1( uint8_t _addr )
{
	uint16_t speed = 0;

	pcontext->SlaveAddress = _addr;
	pcontext->OpMode = READ;
	pcontext->CMD_table = &PMBUS_COMMANDS_RCB1600[20];
	pcontext->Byte_count = PMBUS_COMMANDS_RCB1600[20].cmnd_master_Rx_size;

	if ( STACK_PMBUS_HostCommandGroup(pcontext) != SMBUS_OK)
		LOG_DEBUG("\r\nReadFanSpeed_1 failed\r\n");

	memcpy(&speed, pcontext->Buffer, pcontext->Byte_count);
	LOG_DEBUG("\r\nin read, fan 1 speed:%d", speed);

	return linear_to_real( speed, N_FAN_SPEED_1 );
}

uint16_t ReadFanSpeed_2( uint8_t _addr )
{
	uint16_t speed = 0;

	pcontext->SlaveAddress = _addr;
	pcontext->OpMode = READ;
	pcontext->CMD_table = &PMBUS_COMMANDS_RCB1600[21];
	pcontext->Byte_count = PMBUS_COMMANDS_RCB1600[21].cmnd_master_Rx_size;

	if ( STACK_PMBUS_HostCommandGroup(pcontext) != SMBUS_OK)
		LOG_DEBUG("\r\nReadFanSpeed_2 failed\r\n");

	memcpy(&speed, pcontext->Buffer, pcontext->Byte_count);
	LOG_DEBUG("\r\nin read, fan 2 speed:%d", speed);

	return linear_to_real( speed, N_FAN_SPEED_2 );
}

uint16_t ReadVin( uint8_t _addr )
{
	uint16_t vin = 0;

	pcontext->SlaveAddress = _addr;
	pcontext->OpMode = READ;
	pcontext->CMD_table = &PMBUS_COMMANDS_RCB1600[16];
	pcontext->Byte_count = PMBUS_COMMANDS_RCB1600[16].cmnd_master_Rx_size;

	if ( STACK_PMBUS_HostCommandGroup(pcontext) != SMBUS_OK)
		LOG_DEBUG("\r\nReadVin failed\r\n");

	memcpy(&vin, pcontext->Buffer, pcontext->Byte_count);
	LOG_DEBUG("\r\nin read, vin:%d", vin);

	return linear_to_real( vin, N_READ_VIN );
}

void WriteChgrConfig(uint8_t _addr, uint16_t cfg)
{		
	pcontext->SlaveAddress = _addr;
	pcontext->OpMode = WRITE;
	pcontext->CMD_table = &PMBUS_COMMANDS_RCB1600[33];
	pcontext->Byte_count = PMBUS_COMMANDS_RCB1600[33].cmnd_master_Tx_size;
	memcpy(pcontext->Buffer, &cfg, pcontext->Byte_count);	
	
	if ( STACK_PMBUS_HostCommandGroup(pcontext) != SMBUS_OK)
		LOG_DEBUG("\r\nWrite Charge Config failed\r\n");
}

void SelectChgrCurve(uint8_t addr, uint8_t curve)
{
	uint16_t tmp_cfg = 0;
	TickType_t xLastWakeTime;
	TickType_t xFrequency = WAIT_TIME; // 15ms
	
	// Initialise the xLastWakeTime variable with the current time.
	xLastWakeTime = xTaskGetTickCount ();
	
	// read charg config
	ReadChgConfig(addr, &tmp_cfg);
	
	switch (curve) {
	case CURVE_DEFAULT:
		// bit0 = 0, bit1 = 0, 00
		tmp_cfg &= ~(1<<0);
		tmp_cfg &= ~(1<<1);
		break;
	case CURVE_1:
		// 01
		tmp_cfg |= (1<<0);
		tmp_cfg &= ~(1<<1);
		break;
	case CURVE_2:
		// 10
		tmp_cfg &= ~(1<<0);
		tmp_cfg |= (1<<1);
		break;
	case CURVE_3:
		// 11
		tmp_cfg |= (1<<0);
		tmp_cfg |= (1<<1);
		break;
	default:
		break;
	}
	
	// delay 15ms
	vTaskDelayUntil( &xLastWakeTime, xFrequency );
	
	WriteChgrConfig(addr, tmp_cfg);
	
}

void ReadMFR_ID(uint8_t _addr, uint8_t *ptr)
{
	pcontext->SlaveAddress = _addr;
	pcontext->OpMode = READ;
	pcontext->CMD_table = &PMBUS_COMMANDS_RCB1600[23];
	pcontext->Byte_count = PMBUS_COMMANDS_RCB1600[23].cmnd_master_Rx_size;

	if ( STACK_PMBUS_HostCommandGroup(pcontext) != SMBUS_OK)
		LOG_DEBUG("\r\nRead MFR ID failed\r\n");

	memcpy(ptr, pcontext->Buffer, pcontext->Byte_count);
	LOG_DEBUG("\r\nmfr id:%s", ptr);
}

void ReadMFR_Model(uint8_t _addr, uint8_t *ptr)
{
	pcontext->SlaveAddress = _addr;
	pcontext->OpMode = READ;
	pcontext->CMD_table = &PMBUS_COMMANDS_RCB1600[24];
	pcontext->Byte_count = PMBUS_COMMANDS_RCB1600[24].cmnd_master_Rx_size;

	if ( STACK_PMBUS_HostCommandGroup(pcontext) != SMBUS_OK)
		LOG_DEBUG("\r\nRead MFR Model failed\r\n");

	memcpy(ptr, pcontext->Buffer, pcontext->Byte_count);
	LOG_DEBUG("\r\nmfr model:%s", ptr);
}

void ReadMFR_Revision(uint8_t _addr, uint8_t *ptr)
{
	pcontext->SlaveAddress = _addr;
	pcontext->OpMode = READ;
	pcontext->CMD_table = &PMBUS_COMMANDS_RCB1600[25];
	pcontext->Byte_count = PMBUS_COMMANDS_RCB1600[25].cmnd_master_Rx_size;

	if ( STACK_PMBUS_HostCommandGroup(pcontext) != SMBUS_OK)
		LOG_DEBUG("\r\nRead MFR Revision failed\r\n");

	memcpy(ptr, pcontext->Buffer, pcontext->Byte_count);
	LOG_DEBUG("\r\nmfr revision:%s", ptr);
}

void ReadMFR_Location(uint8_t _addr, uint8_t *ptr)
{
	pcontext->SlaveAddress = _addr;
	pcontext->OpMode = READ;
	pcontext->CMD_table = &PMBUS_COMMANDS_RCB1600[26];
	pcontext->Byte_count = PMBUS_COMMANDS_RCB1600[26].cmnd_master_Rx_size;

	if ( STACK_PMBUS_HostCommandGroup(pcontext) != SMBUS_OK)
		LOG_DEBUG("\r\nRead MFR Location failed\r\n");

	memcpy(ptr, pcontext->Buffer, pcontext->Byte_count);
	LOG_DEBUG("\r\nmfr location:%s", ptr);
}

void ReadMFR_Date(uint8_t _addr, uint8_t *ptr)
{
	pcontext->SlaveAddress = _addr;
	pcontext->OpMode = READ;
	pcontext->CMD_table = &PMBUS_COMMANDS_RCB1600[27];
	pcontext->Byte_count = PMBUS_COMMANDS_RCB1600[27].cmnd_master_Rx_size;

	if ( STACK_PMBUS_HostCommandGroup(pcontext) != SMBUS_OK)
		LOG_DEBUG("\r\nRead MFR Date failed\r\n");

	memcpy(ptr, pcontext->Buffer, pcontext->Byte_count);
	LOG_DEBUG("\r\nmfr date:%s", ptr);
}

void ReadMFR_Serial(uint8_t _addr, uint8_t *ptr)
{
	pcontext->SlaveAddress = _addr;
	pcontext->OpMode = READ;
	pcontext->CMD_table = &PMBUS_COMMANDS_RCB1600[28];
	pcontext->Byte_count = PMBUS_COMMANDS_RCB1600[28].cmnd_master_Rx_size;

	if ( STACK_PMBUS_HostCommandGroup(pcontext) != SMBUS_OK)
		LOG_DEBUG("\r\nRead MFR Serial failed\r\n");

	memcpy(ptr, pcontext->Buffer, pcontext->Byte_count);
	LOG_DEBUG("\r\nmfr serial:%s", ptr);
}

void SetIoutOC_FaultLimit_St(uint8_t _addr, float iout_limit, StatusTypeDef *status )
{
	uint16_t data = real_to_linear(iout_limit, -2);
//	uint16_t data = ioc_real_to_linear( iout_limit );
//	printf("\r\ndata:%04X", data);
		
	pcontext->SlaveAddress = _addr;
	pcontext->OpMode = WRITE;
	pcontext->CMD_table = &PMBUS_COMMANDS_RCB1600[6];
	pcontext->Byte_count = PMBUS_COMMANDS_RCB1600[6].cmnd_master_Tx_size;
	memcpy(pcontext->Buffer, &data, pcontext->Byte_count);	
	
	if ( ( *status = STACK_PMBUS_HostCommandGroup(pcontext) ) != SMBUS_OK)
	{
	    pcontext->StateMachine = SMBUS_SMS_READY;
		LOG_DEBUG("\r\nSet Iout Limit failed\r\n");	
    }
}

void ReadIoutOC_FaultLimit_St(uint8_t _addr, float *iout_limit, StatusTypeDef *status )
{
	uint16_t current = 0;

	pcontext->SlaveAddress = _addr;
	pcontext->OpMode = READ;
	pcontext->CMD_table = &PMBUS_COMMANDS_RCB1600[6];
	pcontext->Byte_count = PMBUS_COMMANDS_RCB1600[6].cmnd_master_Rx_size;

	if ( ( *status = STACK_PMBUS_HostCommandGroup(pcontext) ) != SMBUS_OK)
	{
	    pcontext->StateMachine = SMBUS_SMS_READY;
		LOG_DEBUG("\r\nRead Iout fault limit failed\r\n");
	}

	memcpy(&current, pcontext->Buffer, pcontext->Byte_count);

	*iout_limit = linear_to_real(current, -2);
//	*iout_limit = ioc_linear_to_real( current );
//	printf("\r\nread:%04X, iout_limit:%f", current, *iout_limit);
}




