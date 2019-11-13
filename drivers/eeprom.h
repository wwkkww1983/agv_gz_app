#ifndef _BSP_EEPROM_I2C_H
#define _BSP_EEPROM_I2C_H

/* 
   @brief   
######################################################################
            地址0存放是否是第一次初始化的标志
			8个bit正好对应8个模块，bit0-bit7 -> unit1 - unit8，
			bit[x]=0表示未初始化过，bit[x]=1表示初始化过 

            地址1-9预留
            
######################################################################
			充电参数存储分配为：每个模块分配60个字节，详细如下：
			地址10-69存放模块1的充电参数：
			    其中，地址10-26存放三段式曲线的充电参数：
                    地址10     -> 当前的曲线
			        地址11-14  -> ichg
			            15-18  -> itaper
			    	    19-22  -> vbst
			    	    23-26  -> vfloat
			    其中，地址27-46存放预充电的充电参数：
			    	地址27-30  -> pre_iout
			    	    31-34  -> pre_vout
			    	    35-38  -> ichg
			    	    39-42  -> vbst
			    	    43-46  -> itaper
				其中，地址47-48存放自动充电的阈值        // 20190716添加
			    其中，地址49存放是否执行过电压采样校准，1Byte，1表示执行过，0表示未执行过 // 20190827添加
				      地址50-51存放校准值，2Byte, (放大倍数为100，比如校准值为1.56，那么存储的值为156)
			    其中，地址52-69预留
					
			< 所有模块的参数存储结构类似 >
			
			地址 70-129    存放模块2的充电参数:
			地址 130-189   存放模块3的充电参数:
			地址 190-249   存放模块4的充电参数:
			地址 250-309   存放模块5的充电参数:
			地址 310-369   存放模块6的充电参数:
			地址 370-429   存放模块7的充电参数:
			地址 430-489   存放模块8的充电参数:

######################################################################
			地址 490-499 存放进入设置的密码;预留10个字节
			地址 500-509 存放进入维护的密码;预留10个字节

20191027添加
######################################################################
			// 生产序号长度：16Byte，预留30Byte
            地址 550-579 存放生产序号

			// 产品编码长度：11Byte，预留30Byte
            地址 580-609 存放产品编码


######################################################################
			事件存储基地址为:900
            eeporm中存储划分范围
            base - 900
                     | is_full(2Byte) | latest_idx(2Byte*power_uni_num) | store_base |
 
            latest_idx -> 表示可以写入的最新序号
 */

#include "stm32f10x.h"
#include <inttypes.h>



void eeprom_init(void);
//uint8_t ee_CheckOk(void);
//void ee_Erase(void);
//uint8_t ee_Test(void);
uint8_t ee_ReadBytes(uint8_t *_pReadBuf, uint16_t _usAddress, uint16_t _usSize);
uint8_t ee_WriteBytes(uint8_t *_pWriteBuf, uint16_t _usAddress, uint16_t _usSize);


#endif // bsp_eeprom_i2c.h


