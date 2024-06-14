#ifndef MMA8451Q_H
#define MMA8451Q_H

#include <stdint.h>

void GPIO_Init(void);
void I2C1_Init(void);
void I2C1_Write(uint8_t addr, uint8_t reg, uint8_t data);
void I2C1_Read(uint8_t addr, uint8_t reg, uint8_t *buffer, uint16_t length);

#endif // MMA8451Q_H