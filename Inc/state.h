#ifndef INC_STATE_H_
#define INC_STATE_H_

#include "states.h"
#include "gpio.h"
#include "timer.h"
#include "sleep.h"
#include "sensor.h"
#include "usb.h"
#include "battery.h"
#include "logger.h"
#include "notification.h"

class State : public GPIOMonitor, public TimerMonitor, public USBMonitor {
public:
    State();
    virtual ~State();

    virtual States StateDo() = 0;

    virtual void StateEntry() = 0;
    
    virtual void StateExit() = 0;

    virtual void GPIOCallback(GPIOPinId pin) = 0;

    virtual void TimerCallback(int8_t id) = 0;

    virtual void USBReceiveCallback(uint8_t *buffer, uint32_t length) = 0;

    virtual void USBDisconnectedCallback() = 0;

    virtual void USBConnectedCallback() = 0;

protected:

};

#endif /* INC_STATE_H_ */