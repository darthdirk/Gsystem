#ifndef SRC_STATEMACHINE_H_
#define SRC_STATEMACHINE_H_

#include "adc.h"
#include "battery.h"
#include <map>
#include "gpio.h"
#include "notification.h"
#include "state.h"
#include "states.h"
#include "state_hibernation.h"
#include "state_standby.h"
#include "state_service_mode.h"
#include "state_data_acquisition.h"

class StateMachine {
public:

    StateMachine();
    virtual ~StateMachine();

    void Run();

private:

    States current_state_;

    // TODO refactor to an array so I can limit mem useage
    //std::map<States, State*> states_;
    State* states_[kStateMaxStates];

    State* GetState(States state);

    State* GetState();

    bool Transition(States state);
};

#endif /* SRC_STATEMACHINE_H_ */