#ifndef SRC_STATEDATAACQUISITION_H_
#define SRC_STATEDATAACQUISITION_H_

#include "state.h"
#include <state>
#include "debugPrint.h"

class StateDataAcquisition: public State {
public:
    StateDataAcquisition();
    virtual ~StateDataAcquisition();

    States StateDo();

    void StateEntry();

    void StateExit();
    
    void GPIOCallback(GPIOPinId pin);

    void TimerCallback(int8_t id);

    void USBReceiveCallback(uint8_t *buffer, uint32_t length);

    void USBDisconnectedCallback();

    void USBConnectedCallback();

private:
    int8_t usb_monitor_id_;
    GPIO_PinState battery_cover_state_;
    GPIO_PinState door_cover_state_;
    int8_t timer_id_;
    int16_t battery_cover_monitor_id_;
    int16_t door_cover_monitor_id_;
    int16_t delat_counter_ = -1;
    bool dample_now_ = false;
    States next_state_ = kStateNoState;
};

#endif /* SRC_STATEDATAACQUISITION_H_*/