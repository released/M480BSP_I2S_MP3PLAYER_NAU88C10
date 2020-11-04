/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    MP3 player sample plays MP3 files stored on SD memory card
 *
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"
#include "config.h"
#include "diskio.h"
#include "ff.h"

#include "i2c_master.h"

#define USE_M487_EVM_FREE_I2S
#define USE_M487_EVM_FREE_I2C

//#define USE_CUSTOMIZE_BOARD

#define USE_NAU88C10_12M		(12000000ul)
#define USE_NAU88C10_12_288M	(12288000ul)
//#define USE_MCLK_12M			/*put in KEIL compiler*/
//#define USE_MCLK_12_288M		/*put in KEIL compiler*/

//uint32_t volatile u32BuffPos = 0;
FATFS FatFs[FF_VOLUMES];               /* File system object for logical drive */

#ifdef __ICCARM__
#pragma data_alignment=32
BYTE Buff[16] ;                   /* Working buffer */
DMA_DESC_T DMA_DESC[2];
#else
BYTE Buff[16] __attribute__((aligned(32)));       /* Working buffer */
DMA_DESC_T DMA_DESC[2] __attribute__((aligned(32)));
#endif

//uint8_t bAudioPlaying = 0;
extern signed int aPCMBuffer[2][PCM_BUFFER_SIZE];
extern volatile uint8_t aPCMBuffer_Full[2];
volatile uint8_t u8PCMBuffer_Playing=0;
/*---------------------------------------------------------*/
/* User Provided RTC Function for FatFs module             */
/*---------------------------------------------------------*/
/* This is a real time clock service to be called from     */
/* FatFs module. Any valid time must be returned even if   */
/* the system does not support an RTC.                     */
/* This function is not required in read-only cfg.         */

unsigned long get_fattime (void)
{
    unsigned long tmr;

    tmr=0x00000;

    return tmr;
}


void SDH0_IRQHandler(void)
{
    unsigned int volatile isr;
    unsigned int volatile ier;

    // FMI data abort interrupt
    if (SDH0->GINTSTS & SDH_GINTSTS_DTAIF_Msk)
    {
        /* ResetAllEngine() */
        SDH0->GCTL |= SDH_GCTL_GCTLRST_Msk;
    }

    //----- SD interrupt status
    isr = SDH0->INTSTS;
    if (isr & SDH_INTSTS_BLKDIF_Msk)
    {
        // block down
        SD0.DataReadyFlag = TRUE;
        SDH0->INTSTS = SDH_INTSTS_BLKDIF_Msk;
    }

    if (isr & SDH_INTSTS_CDIF_Msk)   // port 0 card detect
    {
        //----- SD interrupt status
        // it is work to delay 50 times for SD_CLK = 200KHz
        {
            int volatile i;         // delay 30 fail, 50 OK
            for (i=0; i<0x500; i++);  // delay to make sure got updated value from REG_SDISR.
            isr = SDH0->INTSTS;
        }

        if (isr & SDH_INTSTS_CDSTS_Msk)
        {
            printf("\n***** card remove !\n");
            SD0.IsCardInsert = FALSE;   // SDISR_CD_Card = 1 means card remove for GPIO mode
            memset(&SD0, 0, sizeof(SDH_INFO_T));
        }
        else
        {
            printf("***** card insert !\n");
            SDH_Open(SDH0, CardDetect_From_GPIO);
            SDH_Probe(SDH0);
        }

        SDH0->INTSTS = SDH_INTSTS_CDIF_Msk;
    }

    // CRC error interrupt
    if (isr & SDH_INTSTS_CRCIF_Msk)
    {
        if (!(isr & SDH_INTSTS_CRC16_Msk))
        {
            //printf("***** ISR sdioIntHandler(): CRC_16 error !\n");
            // handle CRC error
        }
        else if (!(isr & SDH_INTSTS_CRC7_Msk))
        {
            if (!SD0.R3Flag)
            {
                //printf("***** ISR sdioIntHandler(): CRC_7 error !\n");
                // handle CRC error
            }
        }
        SDH0->INTSTS = SDH_INTSTS_CRCIF_Msk;      // clear interrupt flag
    }

    if (isr & SDH_INTSTS_DITOIF_Msk)
    {
        printf("***** ISR: data in timeout !\n");
        SDH0->INTSTS |= SDH_INTSTS_DITOIF_Msk;
    }

    // Response in timeout interrupt
    if (isr & SDH_INTSTS_RTOIF_Msk)
    {
        printf("***** ISR: response in timeout !\n");
        SDH0->INTSTS |= SDH_INTSTS_RTOIF_Msk;
    }
}

/*
	USE_M487_EVM_FREE_I2S
	PH10 : I2S0_LRCK
	PH9 : I2S0_DO
	PH8 : I2S0_DI
	PE0 : I2S0_MCLK
	PE1 : I2S0_BCLK

	USE_CUSTOMIZE_BOARD
	PB1 : I2S0_LRCK
	PB2 : I2S0_DO
	PB3 : I2S0_DI
	PB4 : I2S0_MCLK
	PB5 : I2S0_BCLK
*/
void I2S_Init(void)
{
    /* Reset NAU88C10 codec */
    NAU88C10_Reset();

    /* Open I2S0 interface and set to slave mode, stereo channel, I2S format */
    I2S_Open(I2S0, I2S_MODE_MASTER, 44100, I2S_DATABIT_16, I2S_ENABLE_MONO, I2S_FORMAT_I2S);//I2S_DISABLE_MONO
//    NVIC_EnableIRQ(I2S0_IRQn);

    /* Set MCLK and enable MCLK */
    I2S_EnableMCLK(I2S0, USE_NAU88C10_12M);
//    I2S0->CTL0 |= I2S_CTL0_ORDER_Msk;

    /* Initialize NAU88C10 codec */
    CLK_SysTickDelay(20000);
    NAU88C10_Setup();
	
}

void SD_Init(void)
{
    TCHAR       sd_path[] = { '0', ':', 0 };    /* SD drive started from 0 */

    SDH_Open_Disk(SDH0, CardDetect_From_GPIO);
    f_chdrive(sd_path);          /* set default path */
}

/*

	USE_M487_EVM_FREE_I2C
	PC11 : I2C0_SDA
	PC12 : I2C0_SCL

	USE_CUSTOMIZE_BOARD
	PA4 : I2C0_SDA
	PA5 : I2C0_SCL

*/
void I2C0_Init(void)
{
    SYS_ResetModule(I2C0_RST);
    /* Open I2C0 and set clock to 100k */
    I2C_Open(I2C0, 100000);

    I2C_SetSlaveAddr(I2C0, 0, DeviceAddr_NAU88C10, I2C_GCMODE_DISABLE);   /* Slave Address : 0011 010b */

    /* Get I2C0 Bus Clock */
    printf("I2C clock %d Hz\n", I2C_GetBusClockFreq(I2C0));

	#if defined (ENABLE_I2C_IRQ)
    I2C_EnableInt(I2C0);
    NVIC_EnableIRQ(I2C0_IRQn);	
	#endif
}


void PDMA_IRQHandler(void)
{
    uint32_t u32Status = PDMA_GET_INT_STATUS(PDMA);

    if (u32Status & 0x2)   /* done */
    {
        if (PDMA_GET_TD_STS(PDMA) & I2S_PDMA_OPENED_CH)
        {
            if(aPCMBuffer_Full[u8PCMBuffer_Playing^1] != 1)
                printf("underflow!!\n");
			
            aPCMBuffer_Full[u8PCMBuffer_Playing] = 0;       //set empty flag
            u8PCMBuffer_Playing ^= 1;
			
//			printf("u8PCMBuffer_Playing : 0x%2X\r\n" , u8PCMBuffer_Playing);			
        }
        PDMA_CLR_TD_FLAG(PDMA,PDMA_TDSTS_TDIF2_Msk);
    }
    else if(u32Status & 0x400)     /* Timeout */
    {
        PDMA_CLR_TMOUT_FLAG(PDMA,PDMA_TDSTS_TDIF2_Msk);
        printf("PDMA Timeout!!\n");
    }
    else
    {
        while(1)
        {
        	printf("%s : 0x%x\n",__FUNCTION__ , u32Status);
        }
    }
}

// Configure PDMA to Scatter Gather mode */
void PDMA_Init(void)
{
    DMA_DESC[0].ctl = ((PCM_BUFFER_SIZE-1)<<PDMA_DSCT_CTL_TXCNT_Pos)|PDMA_WIDTH_32|PDMA_SAR_INC|PDMA_DAR_FIX|PDMA_REQ_SINGLE|PDMA_OP_SCATTER;
    DMA_DESC[0].src = (uint32_t)&aPCMBuffer[0][0];
    DMA_DESC[0].dest = (uint32_t)&I2S0->TXFIFO;
    DMA_DESC[0].offset = (uint32_t)&DMA_DESC[1] - (PDMA->SCATBA);

    DMA_DESC[1].ctl = ((PCM_BUFFER_SIZE-1)<<PDMA_DSCT_CTL_TXCNT_Pos)|PDMA_WIDTH_32|PDMA_SAR_INC|PDMA_DAR_FIX|PDMA_REQ_SINGLE|PDMA_OP_SCATTER;
    DMA_DESC[1].src = (uint32_t)&aPCMBuffer[1][0];
    DMA_DESC[1].dest = (uint32_t)&I2S0->TXFIFO;
    DMA_DESC[1].offset = (uint32_t)&DMA_DESC[0] - (PDMA->SCATBA);

    PDMA_Open(PDMA, BIT0 << PDMA_CH_I2S);
    PDMA_SetTransferMode(PDMA, PDMA_CH_I2S, PDMA_I2S0_TX, 1, (uint32_t)&DMA_DESC[0]);

    PDMA_EnableInt(PDMA, PDMA_CH_I2S, PDMA_INT_TRANS_DONE);
    NVIC_EnableIRQ(PDMA_IRQn);

}

void UART0_Init(void)
{
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);

	printf("\r\n\r\nCLK_GetCPUFreq : %8d\r\n",CLK_GetCPUFreq());
	printf("CLK_GetHCLKFreq : %8d\r\n",CLK_GetHCLKFreq());	
	printf("CLK_GetHXTFreq : %8d\r\n",CLK_GetHXTFreq());
	printf("CLK_GetLXTFreq : %8d\r\n",CLK_GetLXTFreq());	
	printf("CLK_GetPCLK0Freq : %8d\r\n",CLK_GetPCLK0Freq());
	printf("CLK_GetPCLK1Freq : %8d\r\n",CLK_GetPCLK1Freq());

}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);

    /* Enable External XTAL (4~24 MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Waiting for 12MHz clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(FREQ_192MHZ);

    /* Set both PCLK0 and PCLK1 as HCLK/2 */
    CLK->PCLKDIV = CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2;

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable PDMA module clock */
    CLK_EnableModuleClock(PDMA_MODULE);

    /* Select UART module clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HXT, CLK_CLKDIV0_UART0(1));

    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk);
    SYS->GPB_MFPH |= (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    /* Init I2C */
	#if defined (USE_M487_EVM_FREE_I2C)
    CLK_EnableModuleClock(I2C0_MODULE);

    SYS->GPC_MFPH &= ~(SYS_GPC_MFPH_PC11MFP_Msk | SYS_GPC_MFPH_PC12MFP_Msk);
    SYS->GPC_MFPH |= (SYS_GPC_MFPH_PC11MFP_I2C0_SDA | SYS_GPC_MFPH_PC12MFP_I2C0_SCL);

    PC->SMTEN |= GPIO_SMTEN_SMTEN12_Msk;	//I2C0_SCL

	#elif defined (USE_CUSTOMIZE_BOARD)
    CLK_EnableModuleClock(I2C0_MODULE);

    SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA4MFP_Msk | SYS_GPA_MFPL_PA5MFP_Msk);
    SYS->GPA_MFPL |= (SYS_GPA_MFPL_PA4MFP_I2C0_SDA | SYS_GPA_MFPL_PA5MFP_I2C0_SCL);

    PA->SMTEN |= GPIO_SMTEN_SMTEN5_Msk;	//I2C0_SCL
    #endif


    /* Init I2S */
	#if defined (USE_M487_EVM_FREE_I2S)
    SYS->GPE_MFPL = (SYS->GPE_MFPL & ~(SYS_GPE_MFPL_PE0MFP_Msk|SYS_GPE_MFPL_PE1MFP_Msk)) |
                    (SYS_GPE_MFPL_PE0MFP_I2S0_MCLK|SYS_GPE_MFPL_PE1MFP_I2S0_BCLK);
    SYS->GPH_MFPH = (SYS->GPH_MFPH & ~(SYS_GPH_MFPH_PH8MFP_Msk|SYS_GPH_MFPH_PH9MFP_Msk|SYS_GPH_MFPH_PH10MFP_Msk)) |
                    (SYS_GPH_MFPH_PH10MFP_I2S0_LRCK|SYS_GPH_MFPH_PH9MFP_I2S0_DO|SYS_GPH_MFPH_PH8MFP_I2S0_DI );

    PE->SMTEN |= GPIO_SMTEN_SMTEN1_Msk;	//I2S0_BCLK						

	#elif defined (USE_CUSTOMIZE_BOARD)
    SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB1MFP_Msk | SYS_GPB_MFPL_PB2MFP_Msk| SYS_GPB_MFPL_PB3MFP_Msk| SYS_GPB_MFPL_PB4MFP_Msk| SYS_GPB_MFPL_PB5MFP_Msk);
    SYS->GPB_MFPL |= (SYS_GPB_MFPL_PB1MFP_I2S0_LRCK | SYS_GPB_MFPL_PB2MFP_I2S0_DO| SYS_GPB_MFPL_PB3MFP_I2S0_DI| SYS_GPB_MFPL_PB4MFP_I2S0_MCLK| SYS_GPB_MFPL_PB5MFP_I2S0_BCLK);

    PB->SMTEN |= GPIO_SMTEN_SMTEN5_Msk;	//I2S0_BCLK		
	#endif

    /* Select IP clock source */
    CLK_SetModuleClock(I2S0_MODULE, CLK_CLKSEL3_I2S0SEL_HXT, 0);
    /* Enable I2S0 module clock */
    CLK_EnableModuleClock(I2S0_MODULE);
	
    /* Init SD */
    /* select multi-function pin */
    SYS->GPE_MFPL &= ~(SYS_GPE_MFPL_PE7MFP_Msk     | SYS_GPE_MFPL_PE6MFP_Msk     | SYS_GPE_MFPL_PE3MFP_Msk      | SYS_GPE_MFPL_PE2MFP_Msk);
    SYS->GPE_MFPL |=  (SYS_GPE_MFPL_PE7MFP_SD0_CMD | SYS_GPE_MFPL_PE6MFP_SD0_CLK | SYS_GPE_MFPL_PE3MFP_SD0_DAT1 | SYS_GPE_MFPL_PE2MFP_SD0_DAT0);

    SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB5MFP_Msk      | SYS_GPB_MFPL_PB4MFP_Msk);
    SYS->GPB_MFPL |=  (SYS_GPB_MFPL_PB5MFP_SD0_DAT3 | SYS_GPB_MFPL_PB4MFP_SD0_DAT2);

    SYS->GPD_MFPH &= ~(SYS_GPD_MFPH_PD13MFP_Msk);
    SYS->GPD_MFPH |=  (SYS_GPD_MFPH_PD13MFP_SD0_nCD);

    /* Select IP clock source */
    CLK_SetModuleClock(SDH0_MODULE, CLK_CLKSEL0_SDH0SEL_PLL, CLK_CLKDIV0_SDH0(10));
    /* Enable IP clock */
    CLK_EnableModuleClock(SDH0_MODULE);

    /* Lock protected registers */
    SYS_LockReg();	
	
}

int32_t main (void)
{

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    UART0_Init();	
    SD_Init();
    I2C0_Init();
    I2S_Init();
	
    MP3Player();

    while(1);
}

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
