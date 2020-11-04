# M480BSP_I2S_MP3PLAYER_NAU88C10
 M480BSP_I2S_MP3PLAYER_NAU88C10

update @ 2020/11/04

1. add define ENABLE_USE_INT_FLASH , ENABLE_USE_SD_CARD

2. use ENABLE_USE_INT_FLASH , to separate get MP3 raw data from int. flash and deocde

3. remove define USE_MCLK_12M , USE_MCLK_12_288M in project 

4. modify NAU88C10 I2C address to 0x1A , driver will shift 1 bit (<<1) internally

5. add mp3table with size 48340

update @ 2020/09/24

1. use M487 EVM (I2S master) , to initial NAU88C10 (I2S slave)

2. put one MP3 file name as test.mp3 , in SD card root folder 

3. Hardware config : 

use define to separate I2S and I2C pin define (USE_M487_EVM_FREE_I2C , USE_M487_EVM_FREE_I2S , USE_CUSTOMIZE_BOARD)

USE_M487_EVM_FREE_I2C

PC11 : I2C0_SDA

PC12 : I2C0_SCL

USE_M487_EVM_FREE_I2S

PH10 : I2S0_LRCK

PH9 : I2S0_DO

PH8 : I2S0_DI

PE0 : I2S0_MCLK

PE1 : I2S0_BCLK

4. under KEILC project , with 2 global define : USE_MCLK_12M , USE_MCLK_12_288M

5. below is inital log for reference

![image](https://github.com/released/M480BSP_I2S_MP3PLAYER_NAU88C10/blob/master/log.jpg)

6. below is I2S clk

BCLK

![image](https://github.com/released/M480BSP_I2S_MP3PLAYER_NAU88C10/blob/master/scope_BCLK.bmp)

FS

![image](https://github.com/released/M480BSP_I2S_MP3PLAYER_NAU88C10/blob/master/scope_FS.bmp)

MCLK

![image](https://github.com/released/M480BSP_I2S_MP3PLAYER_NAU88C10/blob/master/scope_MCLK.bmp)

