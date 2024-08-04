#ifndef STATESTANDBY_H_
#define STATESTANDBY_H_

#include "state.h"

class StateStandby: public State {
public:
    StateStandby();
    virtual ~StateStandby();

    States StateDo();

    void StateEntry();

    void StateExit();

    void GPIOCallback(GPIOPinId pin);

    void TimerCallback(int8_t id);

    void USBReceiveCallback(uint8_t *buffer, uint32_t length);

    void USBDisconnectedCallback();

    void USBConnectedCallback();
private:
    States next_state_;
    int8_t hibernate_timer_id_;
    int16_t door_monitor_id_;
    int16_t battery_cover_monitor_id_;
    int8_t usb_monitor_id_;

};

#endif  /* STATESTANDBY_H_ */