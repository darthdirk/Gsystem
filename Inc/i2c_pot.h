#ifndef INC_I2C_POT_H_
#define INC_I2C_POT_H_

#include "stm32l4xx_hal.h"

#define DEV ADDRESS 0x005E
#define CMD_WRITE   0x00
#define CMD_READ    0x03

class I2C_Pot {

public:
    I2C_Pot(I2C_HandleTypeDef *);
    virtual ~I2C_Pot();
    bool SetPotValue(uint16_t);
    // bool GetStatusRegister(uint16_t *);

private:
    I2C_HandleTypeDef *i2c_;
    uint16_t address_ = DEV_ADDRESS;

    // Read will need inter only way to generate a restart insted of stop after transmit
    // bool Read(uint32_t address, uint8_t *data, uint32_t size);
    bool Write(uint8_t *data, uint16_t size);
    // bool Increment(uint32_t address, uint8_t &value);
    // bool Decrement(uint32_t address, uint8_t value);
};

#endif /* INC_I2C_POT_H_ */