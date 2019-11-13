/**
 * @file wk2168.h
 * @brief wk2168寄存器地址
 * @author June_Zu
 * @version 1.1
 * @date 2019-01
 */
#ifndef _SPI_WK2168_H
#define _SPI_WK2168_H

//#include "main.h"
#include "stm32f10x.h"
/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define SPI1_WK2168_CS_HIGH()   GPIO_SetBits(GPIOA, GPIO_Pin_4) 
#define SPI2_WK2168_CS_HIGH()   GPIO_SetBits(GPIOB, GPIO_Pin_12)
#define SPI1_WK2168_CS_LOW()    GPIO_ResetBits(GPIOA, GPIO_Pin_4)
#define SPI2_WK2168_CS_LOW()    GPIO_ResetBits(GPIOB, GPIO_Pin_12)

//如下为WK系列串口扩展芯片的寄存器地址定义，不同的芯片，可能寄存器少一些，所以具体的寄存器请看数据手册
//wkxxxx  Global rigister address defines
#define 	WK2XXX_GENA     0X00
#define 	WK2XXX_GRST     0X01
#define		WK2XXX_GMUT     0X02
#define 	WK2XXX_GIER     0X10
#define 	WK2XXX_GIFR     0X11
#define 	WK2XXX_GPDIR    0X21
#define 	WK2XXX_GPDAT    0X31
#define 	WK2XXX_GPORT    1//	/wkxxxx  Global rigister of PORT
//wkxxxx  slave uarts  rigister address defines

#define 	WK2XXX_SPAGE    0X03
//PAGE0
#define 	WK2XXX_SCR      0X04
#define 	WK2XXX_LCR      0X05
#define 	WK2XXX_FCR      0X06
#define 	WK2XXX_SIER     0X07
#define 	WK2XXX_SIFR     0X08
#define 	WK2XXX_TFCNT    0X09
#define 	WK2XXX_RFCNT    0X0A
#define 	WK2XXX_FSR      0X0B
#define 	WK2XXX_LSR      0X0C
#define 	WK2XXX_FDAT     0X0D
#define 	WK2XXX_FWCR     0X0E
#define 	WK2XXX_RS485    0X0F
//PAGE1
#define 	WK2XXX_BAUD1    0X04
#define 	WK2XXX_BAUD0    0X05
#define 	WK2XXX_PRES     0X06
#define 	WK2XXX_RFTL     0X07
#define 	WK2XXX_TFTL     0X08
#define 	WK2XXX_FWTH     0X09
#define 	WK2XXX_FWTL     0X0A
#define 	WK2XXX_XON1     0X0B
#define 	WK2XXX_XOFF1    0X0C
#define 	WK2XXX_SADR     0X0D
#define 	WK2XXX_SAEN     0X0E
#define 	WK2XXX_RRSDLY   0X0F

//WK串口扩展芯片的寄存器的位定义
//wkxxx register bit defines
// GENA
#define 	WK2XXX_UT4EN	0x08
#define 	WK2XXX_UT3EN	0x04
#define 	WK2XXX_UT2EN	0x02
#define 	WK2XXX_UT1EN	0x01
//GRST
#define 	WK2XXX_UT4SLEEP	0x80
#define 	WK2XXX_UT3SLEEP	0x40
#define 	WK2XXX_UT2SLEEP	0x20
#define 	WK2XXX_UT1SLEEP	0x10
#define 	WK2XXX_UT4RST	0x08
#define 	WK2XXX_UT3RST	0x04
#define 	WK2XXX_UT2RST	0x02
#define 	WK2XXX_UT1RST	0x01
//GIER
#define 	WK2XXX_UT4IE	0x08
#define 	WK2XXX_UT3IE	0x04
#define 	WK2XXX_UT2IE	0x02
#define 	WK2XXX_UT1IE	0x01
//GIFR
#define 	WK2XXX_UT4INT	0x08
#define 	WK2XXX_UT3INT	0x04
#define 	WK2XXX_UT2INT	0x02
#define 	WK2XXX_UT1INT	0x01
//SPAGE
#define 	WK2XXX_SPAGE0	0x00
#define 	WK2XXX_SPAGE1   0x01
#define 	WK2XXX_RTSDLY   0X0F
//SCR
#define 	WK2XXX_SLEEPEN	0x04
#define 	WK2XXX_TXEN     0x02
#define 	WK2XXX_RXEN     0x01
//LCR
#define 	WK2XXX_BREAK	0x20
#define 	WK2XXX_IREN     0x10
#define 	WK2XXX_PAEN     0x08
#define 	WK2XXX_PAM1     0x04
#define 	WK2XXX_PAM0     0x02
#define 	WK2XXX_STPL     0x01
//FCR
//SIER
#define 	WK2XXX_FERR_IEN      0x80
#define 	WK2XXX_CTS_IEN       0x40
#define 	WK2XXX_RTS_IEN       0x20
#define 	WK2XXX_XOFF_IEN      0x10
#define 	WK2XXX_TFEMPTY_IEN   0x08
#define 	WK2XXX_TFTRIG_IEN    0x04
#define 	WK2XXX_RXOVT_IEN     0x02
#define 	WK2XXX_RFTRIG_IEN    0x01
//SIFR
#define 	WK2XXX_FERR_INT      0x80
#define 	WK2XXX_CTS_INT       0x40
#define 	WK2XXX_RTS_INT       0x20
#define 	WK2XXX_XOFF_INT      0x10
#define 	WK2XXX_TFEMPTY_INT   0x08
#define 	WK2XXX_TFTRIG_INT    0x04
#define 	WK2XXX_RXOVT_INT     0x02
#define 	WK2XXX_RFTRIG_INT    0x01


//TFCNT
//RFCNT
//FSR
#define 	WK2XXX_RFOE     0x80
#define 	WK2XXX_RFBI     0x40
#define 	WK2XXX_RFFE     0x20
#define 	WK2XXX_RFPE     0x10
#define 	WK2XXX_RDAT     0x08
#define 	WK2XXX_TDAT     0x04
#define 	WK2XXX_TFULL    0x02
#define 	WK2XXX_TBUSY    0x01
//LSR
#define 	WK2XXX_OE       0x08
#define 	WK2XXX_BI       0x04
#define 	WK2XXX_FE       0x02
#define 	WK2XXX_PE       0x01
//FWCR
//RS485
// 常用波特率定义
enum WKBaud{
B600=1,
B1200,
B2400,	         
B4800,          
B9600,	          
B19200,	       
B38400,
B76800,	
B1800,
B3600,          
B7200,         
B14400,	        
B28800,	        
B57600,	        
B115200,	        
B230400	        
};

void WK2168_Sys_Init(void);
void SPI1_Init(SPI_TypeDef* SPIx);			 //³õÊ¼»¯SPI¿Ú
void SPI1_SetSpeed(SPI_TypeDef* SPIx,u8 SpeedSet); //ÉèÖÃSPIËÙ¶È   
u8 SPI_ReadWriteByte(SPI_TypeDef* SPIx,u8 TxData);//SPI×ÜÏß¶ÁÐ´Ò»¸ö×Ö½Ú
void WK2XXX_SPI_Init(void);
void WK_RS485_CS(SPI_TypeDef* SPIx,u8 port,u8 RTXFlag);

void WK_RstInit(void);
void Wk_Init(SPI_TypeDef* SPIx,unsigned char port);
void Wk_SetBaud(SPI_TypeDef* SPIx, u8 port,enum WKBaud baud);
void WkWriteGReg(SPI_TypeDef* SPIx, unsigned char greg,unsigned char dat);
u8 WkReadGReg(SPI_TypeDef* SPIx, unsigned char greg);
void WkWriteSReg(SPI_TypeDef* SPIx, u8 port,u8 sreg,u8 dat);
u8 WkReadSReg(SPI_TypeDef* SPIx, u8 port,u8 sreg);
void WkWriteSFifo(SPI_TypeDef* SPIx, u8 port,u8 *dat,int num);
void WkReadSFifo(SPI_TypeDef* SPIx, u8 port,u8 *rec,int num);
int wk_RxChars(SPI_TypeDef* SPIx, u8 port,u8 *recbuf);
int wk_TxChars(SPI_TypeDef* SPIx, u8 port,int len,u8 *sendbuf);
int wk_TxLen(SPI_TypeDef* SPIx, u8 port);
void WK_RS485(SPI_TypeDef* SPIx, u8 port);
void WK_RTSCTS(SPI_TypeDef* SPIx, u8 port);

#endif /* _SPI_WK2168_H */

