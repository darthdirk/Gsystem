#ifndef INC_TIMER_H_
#define INC_TIMER_H_

#include "stm32l4xx_hal.h"
//#include <vector>

// TODO: Probably make these static const values instead?
#define TIMER_MAX_SEC   30000
#define TIMER_MAX_MSEC  52000
#define TIMER_MAX_USEC  65000
#define MAX_TIMERS      6

class TimerMonitor {
public:
    virtual void TimerCallback(int8_t id) = 0;
};

struct TimerData {
    int8_t id;
    TIM_HandleTypeDef* handle;
    TimerMonitor* monitor;
    bool active;
    bool once;
    uint16_t ticks;
    uint16_t max_ticks;
};

class Timer{

public:
    // TODO Timer(std::vector<TIM_HandleTypeDef*> handles);
    Timer(TIM_HandleTypeDef* handles[], uint8_t length);
    virtual ~Timer();

    bool StartUSecTimer(int8_t &id, uint16_t usec, TimerMonitor* monitor, bool once=false);
    bool StartMSecTimer(int8_t &id, uint16_t msec, TimerMonitor* monitor, bool once=false);
    bool StartSecTimer(int8_t &id, uint16_t sec, TimerMonitor* monitor, bool once=false);
    bool StopTimer(int8_t &id);
    //std::vector<TimerData*> GetTimers() { return this->timers_; }
    TimerData** GetTimers() { return this->timers_; }

private:
    //std::vector<TimerData*> timers_;
    TimerData* timers_[MAX_TIMERS];
};

#endif /* INC_TIMER_H_ */