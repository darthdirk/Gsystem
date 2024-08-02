#ifndef SRC_BATTERY_H_
#define SRC_BATTERY_H_

#include "adc.h"
#include "gpio.h"

//#include <vector>
//#include <algorithm>
#include <cstdlib>

#define MAX_BATTERY_MONITORS 5

enum BatteryState {
    kBatteryUnknown,
    kBatteryHigh,
    kBatteryPartial,
    kBatteryInsufficient
};

class BatteryMonitor {
public:
    virtual void BatteriesChangedCallback() = 0;
};

class Battery {
public:
    Battery();
    virtual ~Battery();

    bool ReadBattery(float &voltage);
    bool GetState(BatteryState &state);
    int8_t RegisterMonitor(BatteryMonitor* monitor);
    bool UnregisterMonitor(int8_t id);

private:
    BatteryState state_;
    float voltage_;
    BatteryMonitor* monitors_[MAX_BATTERY_MONITORS];
    //std::vector<BatteryMonitor*> monitors_;

    void NotifyBatteriesChanged();
};

#endif /* SRC_MATTERY_H_*/
