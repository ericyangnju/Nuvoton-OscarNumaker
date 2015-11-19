#ifndef I2C0INT_H
#define I2C0INT_H
#include <stdint.h>
#include "config.h"

typedef void (*I2C_FUNC)(uint32_t u32Status);

extern uint8_t g_u8DeviceAddr;
extern uint8_t g_au8TxData[3];     //   2
extern volatile uint8_t g_u8RxData;
extern volatile uint8_t g_u8RxData1;

extern volatile uint8_t g_u8DataLen;
#ifdef I2C_TIMEOUT_ENABLE
extern volatile uint16_t i2c_novalidack_timeout;
extern volatile uint16_t i2c_unreachable_timeout;
#endif
extern volatile uint8_t g_u8EndFlag;
extern volatile I2C_FUNC s_I2C0HandlerFn;

void I2C_MasterRx(uint32_t u32Status);
void I2C_MasterTx(uint32_t u32Status);
void I2C_MasterRx_var(uint32_t u32Status);
void I2C_MasterTx_var(uint32_t u32Status);

#endif

