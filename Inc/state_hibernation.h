#ifndef SRC_STATEHIBERNATION_H_
#define SRC_STATEHIBERNATION_H_

#include "state.h"

class StateHibernation: public State {
public:
    StateHibernation();
    virtual ~StateHibernation();

    States StateDo();

    void StateEntry();

    void StateExit();

    void GPIOCallback(GPIOPinId pin);

    void TimerCallback(int8_t id);

    void USBReceiveCallback(uint8_t *buffer, uint32_t length);

    void USBDisconnectedCallback();

    void USBConnectedCallback();

private:
    int16_t door_monitor_id_;
    int16_t battery_cover_monitor_id_;
};

#endif /* SRC_STATEHIBERNATION_H_ */