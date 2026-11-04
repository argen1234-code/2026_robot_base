#ifndef BSP_I2C_H
#define BSP_I2C_H
#include <stdint.h>
#include <stdbool.h>
enum {I2C_IDLE=0,I2C_BUSY=1,I2C_OK=2,I2C_FAIL=3};
bool BspI2c_TransmitBegin(uint8_t address,const uint8_t *data,uint16_t size);
uint32_t BspI2c_Poll(void);
void BspI2c_RecoverBegin(void);
uint32_t BspI2c_RecoverPoll(void);
uint32_t BspI2c_LastError(void);
#endif
