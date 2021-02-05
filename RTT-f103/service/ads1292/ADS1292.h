#ifndef __ADS1292_H
#define __ADS1292_H

#include <rtthread.h>
#include <rtdevice.h>
#include "stm32f1xx_hal.h"
#include <rthw.h>
#include <drv_common.h>
#include <board.h>
#include <rtdbg.h>
#include "stm32f1xx.h"
////////////////////////////////////////////////////////////////////////////////
#define ADS_DRDY GET_PIN(B,8)
#define ADS_RESET GET_PIN(A,0)
#define ADS_START GET_PIN(A,2)
#define ADS_CLKSEL GET_PIN(A,3)
#define ADS_CS GET_PIN(A,4)
/////////////////////////////////////////////////////////////////////////////////////////////

extern uint8_t ads1292_Cache[9];
extern uint8_t ads1292_recive_flag;

/////////////////////////////////////////////////////////////////////////////////////////////

//ID
#define ADS1292_DEVICE DEVICE_ID_ADS1292R
	//1		CONFIG1
#define DATA_RATE DATA_RATE_500SPS
	//2		CONFIG2
#define PDB_LOFF_COMP PDB_LOFF_COMP_ON
#define PDB_REFBUF PDB_REFBUF_ON
#define VREF VREF_242V
#define CLK_EN CLK_EN_OFF
#define INT_TEST INT_TEST_OFF
	//4	5		CHSET
#define CNNNLE1_POWER PD_ON
#define CNNNLE1_GAIN GAIN_2
#define CNNNLE1_MUX MUX_Normal_input
#define CNNNLE2_POWER PD_ON
#define CNNNLE2_GAIN GAIN_6
#define CNNNLE2_MUX MUX_Normal_input
	//6		RLD_SENS
#define PDB_RLD PDB_RLD_ON
#define RLD_LOFF_SENSE RLD_LOFF_SENSE_OFF
#define RLD2N RLD_CANNLE_ON
#define RLD2P RLD_CANNLE_ON
#define RLD1N RLD_CANNLE_OFF
#define RLD1P RLD_CANNLE_OFF
	//7		LOFF_SENS
#define FLIP2 FLIP2_OFF
#define FLIP1 FLIP1_OFF
#define LOFF2N RLD_CANNLE_ON
#define LOFF2P RLD_CANNLE_ON
#define LOFF1N RLD_CANNLE_OFF
#define LOFF1P RLD_CANNLE_OFF
	//9		RSP1
#define RESP_DEMOD_EN1 RESP_DEMOD_ON
#define RESP_MOD_EN RESP_MOD_ON
#define RESP_PH 0X0D
#define RESP_CTRL RESP_CTRL_CLOCK_INTERNAL
	//10		RSP2
#define CALIB CALIB_OFF
#define FREQ FREQ_32K
#define RLDREF_INT RLDREF_INT_INTERNALLY



#define WAKEUP 0X02
#define STANDBY 0X04
#define RESET 0X06
#define START 0X08
#define STOP 0X0A
#define OFFSETCAL 0X1A


#define RDATAC 0X10
#define SDATAC 0X11
#define RDATA 0X12


#define RREG 0X20
#define WREG 0X40


#define ID 0
#define CONFIG1 1
#define CONFIG2 2
#define LOFF 3
#define CH1SET 4
#define CH2SET 5
#define RLD_SENS 6
#define LOFF_SENS 7
#define LOFF_STAT 8
#define RESP1 9
#define RESP2 10
#define GPIO 11

/////////////////////////////////////////////////////////////////////////////////////////////

#define DEVICE_ID_ADS1292 0X53
#define DEVICE_ID_ADS1292R 0X73
//CONFIG1
#define DATA_RATE_125SPS 0X00
#define DATA_RATE_250SPS 0X01
#define DATA_RATE_500SPS 0X02
#define DATA_RATE_1kSPS 0X03
#define DATA_RATE_2kSPS 0X04
#define DATA_RATE_4kSPS 0X05
#define DATA_RATE_8kSPS 0X06
//CONFIG2
#define PDB_LOFF_COMP_OFF 0
#define PDB_LOFF_COMP_ON 1
#define PDB_REFBUF_OFF 0
#define PDB_REFBUF_ON 1
#define VREF_242V 0
#define VREF_4V 1
#define CLK_EN_OFF 0
#define CLK_EN_ON 1
#define INT_TEST_OFF 0
#define INT_TEST_ON 1
//CHSET
#define PD_ON 0
#define PD_OFF 1
#define GAIN_6 0
#define GAIN_1 1
#define GAIN_2 2
#define GAIN_3 3
#define GAIN_4 4
#define GAIN_8 5
#define GAIN_12 6
#define MUX_Normal_input 0
#define MUX_input_shorted 1
#define MUX_Test_signal 5
#define MUX_RLD_DRP 6
#define MUX_RLD_DRM 7
#define MUX_RLD_DRPM 8
#define MUX_RSP_IN3P 9
//RLD_SENS
#define PDB_RLD_OFF 0
#define PDB_RLD_ON 1
#define RLD_LOFF_SENSE_OFF 0
#define RLD_LOFF_SENSE_ON 1
#define RLD_CANNLE_OFF 0
#define RLD_CANNLE_ON 1
//LOFF_SENS
#define FLIP2_OFF 0
#define FLIP2_ON 1
#define FLIP1_OFF 0
#define FLIP1_ON 1
#define LOFF_CANNLE_OFF 0
#define LOFF_CANNLE_ON 1
//RSP1
#define RESP_DEMOD_OFF 0
#define RESP_DEMOD_ON 1
#define RESP_MOD_OFF 0
#define RESP_MOD_ON 1
#define RESP_CTRL_CLOCK_INTERNAL 0
#define RESP_CTRL_CLOCK_EXTERNAL 1
//RSP2
#define CALIB_OFF 0
#define CALIB_ON 1
#define FREQ_32K 0
#define FREQ_64K 1
#define RLDREF_INT_EXTERN 0
#define RLDREF_INT_INTERNALLY 1

typedef struct
{
	uint8_t Data_Rate;
} ADS1292_CONFIG1;
typedef struct
{
	uint8_t Pdb_Loff_Comp;
	uint8_t Pdb_Refbuf;
	uint8_t Vref;
	uint8_t Clk_EN;
	uint8_t Int_Test;
} ADS1292_CONFIG2;
typedef struct
{
	uint8_t PD;
	uint8_t GAIN;
	uint8_t MUX;
} ADS1292_CHSET;
typedef struct
{
	uint8_t Pdb_Rld;		 
	uint8_t Rld_Loff_Sense;
	uint8_t Rld2N;
	uint8_t Rld2P;
	uint8_t Rld1N;
	uint8_t Rld1P;
} ADS1292_RLD_SENS;
typedef struct
{
	uint8_t Flip2;
	uint8_t Flip1;
	uint8_t Loff2N;
	uint8_t Loff2P;
	uint8_t Loff1N;
	uint8_t Loff1P;
} ADS1292_LOFF_SENS;
typedef struct
{
	uint8_t RESP_DemodEN;
	uint8_t RESP_modEN;
	uint8_t RESP_ph;
	uint8_t RESP_Ctrl;
} ADS1292_RESP1;
typedef struct
{
	uint8_t Calib;
	uint8_t freq;
	uint8_t Rldref_Int;
} ADS1292_RESP2;

void ADS1292_Init(void);
void ADS1292_PowerOnInit(void);
uint8_t ADS1292_SPI(uint8_t com);

void ADS1292_Send_CMD(uint8_t data);
void ADS1292_WR_REGS(uint8_t reg, uint8_t len, uint8_t *data);
uint8_t ADS1292_Read_Data(uint8_t *data);

void ADS1292_SET_REGBUFF(void);
uint8_t ADS1292_WRITE_REGBUFF(void);

uint8_t ADS1292_Noise_Test(void);
uint8_t ADS1292_Single_Test(void);
uint8_t ADS1292_Single_Read(void);
uint8_t Set_ADS1292_Collect(uint8_t mode);

#endif
