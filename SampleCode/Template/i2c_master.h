
#include <stdio.h>
#include "NuMicro.h"

#define DeviceAddr_NAU88C10						(0x34) /* NAU88C10 Device ID */

//#define ENABLE_I2C_POLLING_DISCRETE
#define ENABLE_I2C_POLLING_API
//#define ENABLE_I2C_IRQ

//#define DEBUG_LOG_MASTER_LV1
//#define DEBUG_LOG_SLAVE_LV1
//#define DEBUG_LOG_SLAVE_LV2

#define MASTER_I2C						  		(I2C0)
#define MASTER_I2C_IRQn						  	(I2C0_IRQn)
#define I2Cx_Master_IRQHandler					(I2C0_IRQHandler)

#define I2C_WR                  					(0x00)
#define I2C_RD                  					(0x01)

#define MASTER_START_TRANSMIT			  		(0x08)
#define MASTER_REPEAT_START               		(0x10)
#define MASTER_TRANSMIT_ADDRESS_ACK       	(0x18)
#define MASTER_TRANSMIT_ADDRESS_NACK      	(0x20)
#define MASTER_TRANSMIT_DATA_ACK          	(0x28)
#define MASTER_TRANSMIT_DATA_NACK         	(0x30)
#define MASTER_ARBITRATION_LOST           		(0x38)
#define MASTER_RECEIVE_ADDRESS_ACK        		(0x40)
#define MASTER_RECEIVE_ADDRESS_NACK       		(0x48)
#define MASTER_RECEIVE_DATA_ACK           		(0x50)
#define MASTER_RECEIVE_DATA_NACK          		(0x58)
#define BUS_ERROR                         		(0x00)


#define SLAVE_I2C						  		(I2C0)
#define SLAVE_I2C_IRQn						  	(I2C0_IRQn)
#define I2Cx_Slave_IRQHandler						(I2C0_IRQHandler)

#define SLAVE_TRANSMIT_REPEAT_START_OR_STOP	(0xA0)
#define SLAVE_TRANSMIT_ADDRESS_ACK 			(0xA8)
#define SLAVE_TRANSMIT_DATA_ACK				(0xB8)
#define SLAVE_TRANSMIT_DATA_NACK             	(0xC0)
#define SLAVE_TRANSMIT_LAST_DATA_ACK         	(0xC8)
#define SLAVE_RECEIVE_ADDRESS_ACK            	(0x60)
#define SLAVE_RECEIVE_ARBITRATION_LOST       	(0x68)
#define SLAVE_RECEIVE_DATA_ACK               	(0x80)
#define SLAVE_RECEIVE_DATA_NACK              	(0x88)
#define GC_MODE_ADDRESS_ACK                	(0x70)
#define GC_MODE_ARBITRATION_LOST             	(0x78)
#define GC_MODE_DATA_ACK                   	(0x90)
#define GC_MODE_DATA_NACK                  	(0x98)
#define ADDRESS_TRANSMIT_ARBITRATION_LOST    	(0xB0)

void I2Cx_WriteSingleToSlaveIRQ(uint8_t address,uint8_t reg, uint8_t *data);
void I2Cx_ReadSingleToSlaveIRQ(uint8_t address, uint8_t reg,uint8_t *data);

void I2Cx_WriteMultiToSlaveIRQ(uint8_t address,uint8_t reg,uint8_t *data,uint16_t len);
void I2Cx_ReadMultiFromSlaveIRQ(uint8_t address,uint8_t reg,uint8_t *data,uint16_t len);


