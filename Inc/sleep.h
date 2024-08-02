#ifndef INC_SLEEP_H_
#define INC_SLEEP_H_

#include "stm32l4xx_hal.h"
#include "timer.h"
#include "gpio.h"
#include "sensor.h"

class Sleep : TimeMonitor {
public:
    Sleep();
    virtual ~Sleep();

    void LightSleep(int32_t msec);
    void Wake();
    void DeepSleep();

    void TimerCallback(int8_t id);

private:
    int8_t timer_id_;
};

#endif /* INC_SLEEP_H_ */