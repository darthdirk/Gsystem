#ifndef INC_STATES_H_
#define INC_STATES_H_


/**
 * States defines the execution states for the LCTS system.
 */
enum States {
    kStateNoState           = -1,   /// Flag for an unknown or no state
    kStateHibernation       = 0,    /// The hibernation state is an ultra-low power state to conserve battery life
    kStateStandby           = 1,    /// The standby state performs checks prior to data acquisition and displays notifications
    kStateServiceMode       = 2,    /// The service mode state allows communication with an external computer and configuration
    kStateDataAcquisition   = 3,    /// The data acquisition state polls the sensor for data routinely
    kStateMaxStates         = 4     /// Flag for the maximum number of states for logistical use in code
};



#endif /* INC_STATES_H_ */