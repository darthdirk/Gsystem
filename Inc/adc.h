#ifndef INC_ADC_H_
#define INC_ADC_H_

#include "stm32l4xx_hal.h"
#include <stdint.h>

class ADC {

public:
    ADC(ADC_HandleTypeDef *handle);
    virtual ~ADC();

    bool Read(uint32_t &value);

private:
    ADC_HandleTypeDef *handle_;
};

#endif /* INC_ADC_H_*/