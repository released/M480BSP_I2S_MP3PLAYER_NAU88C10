/**************************************************************************//**
 * @file     NAU88C10.c
 * @version  V1.00
 * @brief    M480 I2S Driver Sample Code
 *
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

#include "i2c_master.h"

/*---------------------------------------------------------------------------------------------------------*/

#define NAU8810_REG_RESET			0x00
#define NAU8810_REG_POWER1			0x01
#define NAU8810_REG_POWER2			0x02
#define NAU8810_REG_POWER3			0x03
#define NAU8810_REG_IFACE			0x04
#define NAU8810_REG_COMP			0x05
#define NAU8810_REG_CLOCK			0x06
#define NAU8810_REG_SMPLR			0x07
#define NAU8810_REG_DAC				0x0A
#define NAU8810_REG_DACGAIN		0x0B
#define NAU8810_REG_ADC				0x0E
#define NAU8810_REG_ADCGAIN		0x0F
#define NAU8810_REG_EQ1				0x12
#define NAU8810_REG_EQ2				0x13
#define NAU8810_REG_EQ3				0x14
#define NAU8810_REG_EQ4				0x15
#define NAU8810_REG_EQ5				0x16
#define NAU8810_REG_DACLIM1		0x18
#define NAU8810_REG_DACLIM2		0x19
#define NAU8810_REG_NOTCH1			0x1B
#define NAU8810_REG_NOTCH2			0x1C
#define NAU8810_REG_NOTCH3			0x1D
#define NAU8810_REG_NOTCH4			0x1E
#define NAU8810_REG_ALC1			0x20
#define NAU8810_REG_ALC2			0x21
#define NAU8810_REG_ALC3			0x22
#define NAU8810_REG_NOISEGATE		0x23
#define NAU8810_REG_PLLN			0x24
#define NAU8810_REG_PLLK1			0x25
#define NAU8810_REG_PLLK2			0x26
#define NAU8810_REG_PLLK3			0x27
#define NAU8810_REG_ATTEN			0x28
#define NAU8810_REG_INPUT_SIGNAL	0x2C
#define NAU8810_REG_PGAGAIN		0x2D
#define NAU8810_REG_ADCBOOST		0x2F
#define NAU8810_REG_OUTPUT			0x31
#define NAU8810_REG_SPKMIX			0x32
#define NAU8810_REG_SPKGAIN			0x36
#define NAU8810_REG_MONOMIX		0x38
#define NAU8810_REG_POWER4			0x3A
#define NAU8810_REG_TSLOTCTL1		0x3B
#define NAU8810_REG_TSLOTCTL2		0x3C
#define NAU8810_REG_DEVICE_REVID	0x3E
#define NAU8810_REG_I2C_DEVICEID		0x3F
#define NAU8810_REG_ADDITIONID		0x40
#define NAU8810_REG_RESERVE			0x41
#define NAU8810_REG_OUTCTL			0x45
#define NAU8810_REG_ALC1ENHAN1		0x46
#define NAU8810_REG_ALC1ENHAN2		0x47
#define NAU8810_REG_MISCCTL			0x49
#define NAU8810_REG_OUTTIEOFF		0x4B
#define NAU8810_REG_AGCP2POUT		0x4C
#define NAU8810_REG_AGCPOUT		0x4D
#define NAU8810_REG_AMTCTL			0x4E
#define NAU8810_REG_OUTTIEOFFMAN	0x4F


/*---------------------------------------------------------------------------------------------------------*/

void RecoveryFromArbLost(void)
{
    I2C0->CTL0 &= ~I2C_CTL0_I2CEN_Msk;
    I2C0->CTL0 |= I2C_CTL0_I2CEN_Msk;
}

uint8_t I2C_WriteNAU88C10(uint8_t u8addr, uint16_t u16data)
{		
	I2C_T *i2c = MASTER_I2C;	

    uint8_t Device_Address  = 0;
	uint8_t buf[2] = 0;
	uint16_t vaule = 0;
    uint8_t reg_addr  = 0;
    uint8_t reg_data  = 0;
	
	Device_Address = DeviceAddr_NAU88C10;	

	#if 1	
	reg_addr = ((u8addr << 1) | (u16data >> 8));
	reg_data = (u16data & 0x00FF) ;

	buf[0] = reg_addr;
	buf[1] = reg_data;		
	#else
	vaule = (u8addr << 9) | u16data ;
	buf[0] = vaule >> 8;
	buf[1] = (uint8_t) (vaule & 0xFF);	
	#endif
	
//	printf("Address : 0x%2X , register : 0x%4X , data : 0x%4X , (0x%4X ,0x%4X , 0x%4X ) \r\n" , Device_Address , u8addr , u16data ,  vaule , buf[0]  , buf[1]);


	#if defined (ENABLE_I2C_IRQ)
//	I2Cx_WriteMultiToSlaveIRQ(Device_Address, buf[0], &buf[1] , 1);
	I2Cx_WriteMultiToSlaveIRQ(Device_Address, reg_addr , &reg_data , 1);
	
	#elif defined (ENABLE_I2C_POLLING_API)
//	I2C_WriteMultiBytes(i2c, Device_Address , buf, 2);
	I2C_WriteByteOneReg(i2c, Device_Address , reg_addr, reg_data );	

	#elif defined (ENABLE_I2C_POLLING_DISCRETE)

	#if 1
restart:
    I2C_START(i2c);
    I2C_WAIT_READY(i2c);

    I2C_SET_DATA(i2c, Device_Address << 1);
    I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI);
    I2C_WAIT_READY(i2c);
    if(I2C_GET_STATUS(i2c) == 0x38)
    {
        RecoveryFromArbLost();
        goto restart;
    }
    else if(I2C_GET_STATUS(i2c) != 0x18)
        goto stop;

    I2C_SET_DATA(i2c, (uint8_t)reg_addr);
    I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI);
    I2C_WAIT_READY(i2c);
    if(I2C_GET_STATUS(i2c) == 0x38)
    {
        RecoveryFromArbLost();
        goto restart;
    }
    else if(I2C_GET_STATUS(i2c) != 0x28)
        goto stop;

    I2C_SET_DATA(i2c, (uint8_t)reg_data);
    I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI);
    I2C_WAIT_READY(i2c);
    if(I2C_GET_STATUS(i2c) == 0x38)
    {
        RecoveryFromArbLost();
        goto restart;
    }
    else if(I2C_GET_STATUS(i2c) != 0x28)
        goto stop;

stop:
    I2C_STOP(i2c);
    while(i2c->CTL0 & I2C_CTL0_STO_Msk);
	#else
    I2C_START(i2c);
    I2C_WAIT_READY(i2c);

    I2C_SET_DATA(i2c, Device_Address << 1 );
    I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI);
    I2C_WAIT_READY(i2c);

    I2C_SET_DATA(i2c, (uint8_t)reg_addr);
    I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI);
    I2C_WAIT_READY(i2c);

    I2C_SET_DATA(i2c, (uint8_t)reg_data);
    I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI);
    I2C_WAIT_READY(i2c);

    I2C_STOP(i2c);
	#endif
	
	#endif


	return 1;

}

/* config play sampling rate */
void NAU88C10_ConfigSampleRate(uint32_t u32SampleRate)
{
    printf("[NAU88C10] Configure Sampling Rate to %d\n", u32SampleRate);

	#if 0

    if((u32SampleRate % 8) == 0)
    {
        I2C_WriteNAU88C10(0x0005, 0x3126); //12.288Mhz
        I2C_WriteNAU88C10(0x0006, 0x0008);
    }
    else
    {
        I2C_WriteNAU88C10(0x0005, 0x86C2); //11.2896Mhz
        I2C_WriteNAU88C10(0x0006, 0x0007);
    }

    switch (u32SampleRate)
    {
    case 16000:
        I2C_WriteNAU88C10(0x0003,  0x801B); //MCLK = SYSCLK_SRC/12
        I2C_WriteNAU88C10(0x0004,  0x0001);
        I2C_WriteNAU88C10(0x0005,  0x3126); //MCLK = 4.096MHz
        I2C_WriteNAU88C10(0x0006,  0x0008);
        I2C_WriteNAU88C10(0x001D,  0x301A); //301A:Master, BCLK_DIV=MCLK/8=512K, LRC_DIV=512K/32=16K
        I2C_WriteNAU88C10(0x002B,  0x0002);
        I2C_WriteNAU88C10(0x002C,  0x0082);
        break;

    case 44100:
        I2C_WriteNAU88C10(0x001D,  0x301A); //301A:Master, BCLK_DIV=11.2896M/8=1.4112M, LRC_DIV=1.4112M/32=44.1K
        I2C_WriteNAU88C10(0x002B,  0x0012);
        I2C_WriteNAU88C10(0x002C,  0x0082);
        break;

    case 48000:
        I2C_WriteNAU88C10(0x001D,  0x301A); //301A:Master, BCLK_DIV=12.288M/8=1.536M, LRC_DIV=1.536M/32=48K
        I2C_WriteNAU88C10(0x002B,  0x0012);
        I2C_WriteNAU88C10(0x002C,  0x0082);
        break;

    case 96000:
        I2C_WriteNAU88C10(0x0003,  0x80A2); //MCLK = SYSCLK_SRC/2
        I2C_WriteNAU88C10(0x0004,  0x1801);
        I2C_WriteNAU88C10(0x0005,  0x3126); //MCLK = 24.576MHz
        I2C_WriteNAU88C10(0x0006,  0xF008);
        I2C_WriteNAU88C10(0x001D,  0x301A); //3019:Master, BCLK_DIV=MCLK/8=3.072M, LRC_DIV=3.072M/32=96K
        I2C_WriteNAU88C10(0x002B,  0x0001);
        I2C_WriteNAU88C10(0x002C,  0x0080);
        break;
    }

	#endif
}

/*
13.1. SOFTWARE RESET 
This is device Reset register.  Performing a write instruction to this register with any data will reset all the bits in the 
register map to default. 

*/

void NAU88C10_Reset(void)
{
    I2C_WriteNAU88C10(0,  0x1);
    I2C_WriteNAU88C10(0,  0);   // Reset all registers
    CLK_SysTickDelay(10000);

    printf("NAU88C10 Software Reset.\n");
}


void NAU88C10_Setup(void)
{
	uint8_t rx[2]={0};

	I2C_WriteNAU88C10(0x00 , 0x000);
    CLK_SysTickDelay(10000);
	
	I2C_WriteNAU88C10(0x01 , 0x11F);
	I2C_WriteNAU88C10(0x02 , 0x015);
	I2C_WriteNAU88C10(0x03 , 0x065);
	I2C_WriteNAU88C10(0x04 , BIT4);		//I2C_WriteNAU88C10(0x04 , BIT4|BIT3);	
	I2C_WriteNAU88C10(0x05 , 0x000);
    CLK_SysTickDelay(10000);
	
	I2C_WriteNAU88C10(0x06 , 0x00C);
	I2C_WriteNAU88C10(0x07 , 0x000);	//BIT2|BIT1
	I2C_WriteNAU88C10(0x0A , 0x008);
	I2C_WriteNAU88C10(0x0B , 0x0FF);
	I2C_WriteNAU88C10(0x0E , 0x108);
	I2C_WriteNAU88C10(0x0F , 0x0FF);
	I2C_WriteNAU88C10(0x12 , 0x12C);
	I2C_WriteNAU88C10(0x13 , 0x02C);
	I2C_WriteNAU88C10(0x14 , 0x02C);
	I2C_WriteNAU88C10(0x15 , 0x02C);
	I2C_WriteNAU88C10(0x16 , 0x02C);
	I2C_WriteNAU88C10(0x18 , 0x032);
	I2C_WriteNAU88C10(0x19 , 0x000);
	I2C_WriteNAU88C10(0x1B , 0x000);
	I2C_WriteNAU88C10(0x1C , 0x000);
	I2C_WriteNAU88C10(0x1D , 0x000);
	I2C_WriteNAU88C10(0x1E , 0x000);
	I2C_WriteNAU88C10(0x20 , 0x038);
	I2C_WriteNAU88C10(0x21 , 0x00B);
	I2C_WriteNAU88C10(0x22 , 0x032);
	I2C_WriteNAU88C10(0x23 , 0x000);
	
	I2C_WriteNAU88C10(0x24 , 0x006);
	I2C_WriteNAU88C10(0x25 , 0x009);
	I2C_WriteNAU88C10(0x26 , 0x06E);
	I2C_WriteNAU88C10(0x27 , 0x12F);
	
	I2C_WriteNAU88C10(0x28 , 0x000);
	I2C_WriteNAU88C10(0x2C , 0x003);
	I2C_WriteNAU88C10(0x2D , 0x010);
	I2C_WriteNAU88C10(0x2F , 0x100);
	I2C_WriteNAU88C10(0x31 , 0x00E);
	I2C_WriteNAU88C10(0x32 , 0x001);
	I2C_WriteNAU88C10(0x36 , 0x039);
	I2C_WriteNAU88C10(0x38 , 0x000);
	I2C_WriteNAU88C10(0x3A , 0x000);
	I2C_WriteNAU88C10(0x3B , 0x000);
	I2C_WriteNAU88C10(0x3C , 0x020);
	I2C_WriteNAU88C10(0x3E , 0x0EE);
	I2C_WriteNAU88C10(0x3F , 0x01A);
	I2C_WriteNAU88C10(0x40 , 0x0CA);
	I2C_WriteNAU88C10(0x41 , 0x1FF);
	I2C_WriteNAU88C10(0x45 , 0x1FF);
	I2C_WriteNAU88C10(0x46 , 0x064);
	I2C_WriteNAU88C10(0x47 , 0x1FF);
	I2C_WriteNAU88C10(0x49 , 0x000);
	I2C_WriteNAU88C10(0x4B , 0x1FF);
	I2C_WriteNAU88C10(0x4C , 0x1FF);
	I2C_WriteNAU88C10(0x4D , 0x001);
	I2C_WriteNAU88C10(0x4E , 0x1FF);
	I2C_WriteNAU88C10(0x4F , 0x1FF);

    printf("NAU88C10 Configured done.\n");

}


