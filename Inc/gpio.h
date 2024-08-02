#ifndef INC_GPIO_H_
#define INC_GPIO_H_

#include "stm32l4xx_hal.h"
#include "main.h"
//#include <map>

#define MAX_GPIO_MONITORS 5

enum GPIOPinId {
    kPinNoPin               = -1,
    // UI Outputs
    kPinUILEDGreen          = 0,
    kPinAudioAnnunciator    = 1,
    kPinTestLED             = 2,
    // Outputs perf
    kPinSensorLEDCharge     = 3,
    kPinSensorLEDOn         = 4,
    kPinSensorLEDADCSelect  = 5,
    kPinSensorLEDADCStart   = 6,
    kPinSensorLEDADCPower   = 7,
    kPinSimLoad             = 8,
    // Inputs
    kPinSensorLEDADCEOC     = 9,
    kPinBatteryCover        = 10,
    kPinDoorCover           = 11,
    kPinMaxPins             = 12
};

enum GPIODirection {
    kDirectionInput = 0,
    kDirectionOutput = 1
};

class GPIOMonitor {
public:
    virtual void GPIOCallback(GPIOPinID pin) = 0;
};

/* GPIO PIN DEF*/
struct GpioPinData {
    uint16_t pin;
    GPIO_TypeDef* port;
    GPIODirection direction;
    GPIOMonitor* monitors[MAX_GPIO_MONITORS];
};

// hardware Interrupts
class GPIO {
public:

    GPIO();
    virtual ~GPIO();

    bool Set(GPIOPinId pin);
    bool Reset(GPIOPinId pin);
    bool Read(GPIOPinId pin, GPIO_PinState &state);
    bool Toggle(GPIOPinId pin);
    bool Write(GPIOPinId pin, GPIO_PinState state);

    int16_t RegisterMonitor(GPIOMonitor* monitor, GPIOPinId pin);
    uint16_t UnregisterMonitor(int16_t monitor_id, GPIOPinId pin);

    GpioPinData* GetPinData(GPIOPinId pin);

    static bool sleeping_;

private:
    //TODO std::map<GPIOPINID, GPIOPinData> pins_;
    GpioPinData pins_[kPinMaxPins];

    bool PinIsValid(GPIOPinId pin);
};

#endif /* INC_GPIO_H_ */