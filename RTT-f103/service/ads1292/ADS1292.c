
#include "ADS1292.h"

#include "led.h"

#define DEBUG_ADS1292
#define ADS1292_SPI_DEVICE_NAME     "spi10"
struct rt_spi_device *spi_dev_ads1292;
uint8_t ads1292_recive_flag = 0;
uint8_t ads1292_Cache[9];

uint8_t ADS1292_REG[12];																		   //ads1292
ADS1292_CONFIG1 Ads1292_Config1 = {DATA_RATE};											   //CONFIG1
ADS1292_CONFIG2 Ads1292_Config2 = {PDB_LOFF_COMP, PDB_REFBUF, VREF, CLK_EN, INT_TEST};	   //CONFIG2
ADS1292_CHSET Ads1292_Ch1set = {CNNNLE1_POWER, CNNNLE1_GAIN, CNNNLE1_MUX};				   //CH1SET
ADS1292_CHSET Ads1292_Ch2set = {CNNNLE2_POWER, CNNNLE2_GAIN, CNNNLE2_MUX};				   //CH2SET
ADS1292_RLD_SENS Ads1292_Rld_Sens = {PDB_RLD, RLD_LOFF_SENSE, RLD2N, RLD2P, RLD1N, RLD1P}; //RLD_SENS
ADS1292_LOFF_SENS Ads1292_Loff_Sens = {FLIP2, FLIP1, LOFF2N, LOFF2P, LOFF1N, LOFF1P};	   //LOFF_SENS
ADS1292_RESP1 Ads1292_Resp1 = {RESP_DEMOD_EN1, RESP_MOD_EN, RESP_PH, RESP_CTRL};		   //RSP1
ADS1292_RESP2 Ads1292_Resp2 = {CALIB, FREQ, RLDREF_INT};								   //RSP2

static int rt_hw_spi_flash_init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    spi_dev_ads1292 = (struct rt_spi_device *)rt_device_find(ADS1292_SPI_DEVICE_NAME);
    rt_spi_bus_attach_device(spi_dev_ads1292, "ads129", "spi1", RT_NULL);

    return RT_EOK;
}
/* 导出到自动初始化 */
INIT_COMPONENT_EXPORT(rt_hw_spi_flash_init);

void ADS1292_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	__HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

	//DRDY
    GPIO_InitStruct.Pin = GPIO_PIN_8;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	HAL_NVIC_SetPriority(EXTI9_5_IRQn,2,0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
	//RESRT
    GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	//START
	GPIO_InitStruct.Pin = GPIO_PIN_2;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	//CLKSEL
	GPIO_InitStruct.Pin = GPIO_PIN_3;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	rt_pin_write(ADS_CS,PIN_HIGH);
	
	ADS1292_PowerOnInit();
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	rt_interrupt_enter();
	
    if(GPIO_Pin == GPIO_PIN_8)
	{
		ADS1292_Read_Data(ads1292_Cache);
		ads1292_recive_flag = 1;
	}
	
  	rt_interrupt_leave();
}

uint8_t ADS1292_Read_Data(uint8_t *data)
{
	uint8_t i;

	rt_pin_write(ADS_CS,PIN_LOW);
	//rt_thread_delay(10);
	for (i = 0; i < 9; i++)
	{
		*data = ADS1292_SPI(0X00);
		data++;
	}
	//rt_thread_delay(10);
	rt_pin_write(ADS_CS,PIN_HIGH);
	return 0;
}

void ADS1292_SET_REGBUFF(void)
{
	ADS1292_REG[ID] = ADS1292_DEVICE;

	ADS1292_REG[CONFIG1] = 0x00;
	ADS1292_REG[CONFIG1] |= Ads1292_Config1.Data_Rate;

	ADS1292_REG[CONFIG2] = 0x00;
	ADS1292_REG[CONFIG2] |= Ads1292_Config2.Pdb_Loff_Comp << 6;
	ADS1292_REG[CONFIG2] |= Ads1292_Config2.Pdb_Refbuf << 5;
	ADS1292_REG[CONFIG2] |= Ads1292_Config2.Vref << 4;
	ADS1292_REG[CONFIG2] |= Ads1292_Config2.Clk_EN << 3;
	ADS1292_REG[CONFIG2] |= Ads1292_Config2.Int_Test << 1;
	ADS1292_REG[CONFIG2] |= 0x81;

	ADS1292_REG[LOFF] = 0x10;

	ADS1292_REG[CH1SET] = 0x00;
	ADS1292_REG[CH1SET] |= Ads1292_Ch1set.PD << 7;
	ADS1292_REG[CH1SET] |= Ads1292_Ch1set.GAIN << 4;
	ADS1292_REG[CH1SET] |= Ads1292_Ch1set.MUX;

	ADS1292_REG[CH2SET] = 0x00;
	ADS1292_REG[CH2SET] |= Ads1292_Ch2set.PD << 7;
	ADS1292_REG[CH2SET] |= Ads1292_Ch2set.GAIN << 4;
	ADS1292_REG[CH2SET] |= Ads1292_Ch2set.MUX;

	ADS1292_REG[RLD_SENS] = 0X00;
	ADS1292_REG[RLD_SENS] |= Ads1292_Rld_Sens.Pdb_Rld << 5;		  
	ADS1292_REG[RLD_SENS] |= Ads1292_Rld_Sens.Rld_Loff_Sense << 4;
	ADS1292_REG[RLD_SENS] |= Ads1292_Rld_Sens.Rld2N << 3;
	ADS1292_REG[RLD_SENS] |= Ads1292_Rld_Sens.Rld2P << 2;
	ADS1292_REG[RLD_SENS] |= Ads1292_Rld_Sens.Rld1N << 1;
	ADS1292_REG[RLD_SENS] |= Ads1292_Rld_Sens.Rld1P;
	ADS1292_REG[RLD_SENS] |= 0xc0;

	ADS1292_REG[LOFF_SENS] = 0X00;
	ADS1292_REG[LOFF_SENS] |= Ads1292_Loff_Sens.Flip2 << 5;
	ADS1292_REG[LOFF_SENS] |= Ads1292_Loff_Sens.Flip1 << 4;
	ADS1292_REG[LOFF_SENS] |= Ads1292_Loff_Sens.Loff2N << 3;
	ADS1292_REG[LOFF_SENS] |= Ads1292_Loff_Sens.Loff2P << 2;
	ADS1292_REG[LOFF_SENS] |= Ads1292_Loff_Sens.Loff1N << 1;
	ADS1292_REG[LOFF_SENS] |= Ads1292_Loff_Sens.Loff1P;

	ADS1292_REG[LOFF_STAT] = 0x00; 

	ADS1292_REG[RESP1] = 0X00;
	ADS1292_REG[RESP1] |= Ads1292_Resp1.RESP_DemodEN << 7;
	ADS1292_REG[RESP1] |= Ads1292_Resp1.RESP_modEN << 6;
	ADS1292_REG[RESP1] |= Ads1292_Resp1.RESP_ph << 2;
	ADS1292_REG[RESP1] |= Ads1292_Resp1.RESP_Ctrl;
	ADS1292_REG[RESP1] |= 0x02;

	ADS1292_REG[RESP2] = 0x00;
	ADS1292_REG[RESP2] |= Ads1292_Resp2.Calib << 7;
	ADS1292_REG[RESP2] |= Ads1292_Resp2.freq << 2;
	ADS1292_REG[RESP2] |= Ads1292_Resp2.Rldref_Int << 1;
	ADS1292_REG[RESP2] |= 0X01;

	ADS1292_REG[GPIO] = 0x0C;
}


uint8_t ADS1292_SPI(uint8_t com)
{
	struct rt_spi_message msg;
	uint8_t cmd;
	uint8_t recv;
	cmd = com;
	msg.send_buf = &cmd;
	msg.recv_buf = &recv;
	msg.length = 1;
	msg.cs_take = 1;
	msg.cs_release = 0;
	msg.next = RT_NULL;

	rt_spi_transfer_message(spi_dev_ads1292,&msg);
	return recv;
}


void ADS1292_Send_CMD(uint8_t data)
{
	struct rt_spi_message msg;
	uint8_t cmd;

    rt_pin_write(ADS_CS,PIN_LOW);
	rt_thread_delay(100);

	cmd = data;
	msg.send_buf = &cmd;
	msg.recv_buf = RT_NULL;
	msg.length = 1;
	msg.cs_take = 1;
	msg.cs_release = 0;
	msg.next = RT_NULL;

	rt_spi_transfer_message(spi_dev_ads1292,&msg);

	rt_thread_delay(100);
	rt_pin_write(ADS_CS,PIN_HIGH);
}


void ADS1292_WR_REGS(uint8_t reg, uint8_t len, uint8_t *data)
{
	uint8_t i;
	rt_pin_write(ADS_CS,PIN_LOW);
	rt_thread_delay(100);
	ADS1292_SPI(reg);
	rt_thread_delay(100);
	ADS1292_SPI(len - 1);
	if (reg & 0x40)
	{
		for (i = 0; i < len; i++)
		{
			rt_thread_delay(100);
			ADS1292_SPI(*data);
			data++;
		}
	}
	else
	{
		for (i = 0; i < len; i++)
		{
			rt_thread_delay(100);
			*data = ADS1292_SPI(RT_NULL);
			data++;
		}
	}
	rt_thread_delay(100);
	rt_pin_write(ADS_CS,PIN_HIGH);
}

uint8_t ADS1292_WRITE_REGBUFF(void)
{
	uint8_t i, res = 0;
	uint8_t REG_Cache[12];
	ADS1292_SET_REGBUFF();
	ADS1292_WR_REGS(WREG | CONFIG1, 11, ADS1292_REG + 1);
	rt_thread_mdelay(10);
	ADS1292_WR_REGS(RREG | ID, 12, REG_Cache);
	rt_thread_mdelay(10);

#ifdef DEBUG_ADS1292
	rt_kprintf("WRITE REG:\r\n");
	for (i = 0; i < 12; i++)
		rt_kprintf("%d %x\r\n", i, ADS1292_REG[i]);
	rt_kprintf("READ REG:\r\n");
#endif

	for (i = 0; i < 12; i++)
	{
		if (ADS1292_REG[i] != REG_Cache[i])
		{
			if (i != 0 && i != 8 && i != 11)
				res = 1;
			else
				continue;
		}
#ifdef DEBUG_ADS1292
		rt_kprintf("%d %x\r\n", i, REG_Cache[i]);
#endif
	}

#ifdef DEBUG_ADS1292
	if (res == 0)
		rt_kprintf("REG write success\r\n");
	else
		rt_kprintf("REG write err\r\n");
#endif
	return res;
}

void ADS1292_PowerOnInit(void)
{
	uint8_t i;
	uint8_t REG_Cache[12];

	ADS1292_Send_CMD(SDATAC);
	rt_thread_mdelay(100);
	ADS1292_Send_CMD(RESET);
	rt_thread_mdelay(1000);
	ADS1292_Send_CMD(SDATAC);
	rt_thread_mdelay(100);

#ifdef DEBUG_ADS1292
	ADS1292_WR_REGS(RREG | ID, 12, REG_Cache);
	rt_kprintf("read default REG:\r\n");
	for (i = 0; i < 12; i++)
		rt_kprintf("%d %x\r\n", i, REG_Cache[i]);
#endif
	//ADS1292_Send_CMD(STANDBY);
}


uint8_t ADS1292_Single_Test(void)
{
	uint8_t res = 0;
	Ads1292_Config2.Int_Test = INT_TEST_ON;
	Ads1292_Ch1set.MUX = MUX_Test_signal;
	Ads1292_Ch2set.MUX = MUX_Test_signal;

	if (ADS1292_WRITE_REGBUFF())
		res = 1;
	rt_thread_mdelay(10);
	return res;
}


uint8_t ADS1292_Noise_Test(void)
{
	uint8_t res = 0;
	Ads1292_Config2.Int_Test = INT_TEST_OFF;
	Ads1292_Ch1set.MUX = MUX_input_shorted;
	Ads1292_Ch2set.MUX = MUX_input_shorted;

	if (ADS1292_WRITE_REGBUFF())
		res = 1;
	rt_thread_mdelay(10);
	return res;
}


uint8_t ADS1292_Single_Read(void)
{
	uint8_t res = 0;
	Ads1292_Config2.Int_Test = INT_TEST_OFF;
	Ads1292_Ch1set.MUX = MUX_Normal_input;
	Ads1292_Ch2set.MUX = MUX_Normal_input;

	if (ADS1292_WRITE_REGBUFF())
		res = 1;
	rt_thread_mdelay(10);
	return res;
}


uint8_t Set_ADS1292_Collect(uint8_t mode)
{
	uint8_t res=0;

	rt_thread_mdelay(10);
	switch (mode)
	{
	case 0:
		res = ADS1292_Single_Read();
		break;
	case 1:
		res = ADS1292_Single_Test();
		break;
	case 2:
		res = ADS1292_Noise_Test();
		break;
	}
	if (res)
		return 1;
	ADS1292_Send_CMD(RDATAC);
	rt_thread_mdelay(10);
	ADS1292_Send_CMD(START);
	rt_thread_mdelay(10);
	return 0;
}
