#include "battery.h"

#define BATTERY_COUNTS_PER_VOLT         590
#define BATTERY_PARTIAL_THRESHOLD       5.0
#define BATTERY_INSUFFICIENT_THRESHOLD  4.5
#define BATTERY_TOLERANCE               0.05
#define BATTERY_REPLACEMENT_THRESHOLD   1.0

extern ADC* adc;
extern GPIO* gpio;

Battery::Battery() {
    this->state_ = kBatteryUnknown;
    for (int ii = 0; ii < MAX_BATTERY_MONITORS; ii++) {
        this->monitors_[ii] = nullptr;
    }
}

Battery::~Battery() {

}

bool Battery::ReadBattery(float &voltage) {
    uint32_t value;
    gpio->Set(kPinSimLoad);
    bool result = adc->Read(value);
    gpio->Reset(kPinSimLoad);
    if(result) {
        voltage = static_cast<float>(value) / static_cast<float>(BATTERY_COUNTS_PER_VOLT);
        if(std::abs(voltage - this->voltage_) > BATTERY_REPLACEMENT_THRESHOLD) {
            this->NotifyBatteriesChanged();
        }
        this->voltage_ = voltage;
        result = true;
    }
    return result;
}

bool Battery::GetState(BatteryState &state) {
    bool result = this->ReadBattery(this->voltage_);
    float low_threshold = BATTERY_PARTIAL_THRESHOLD;
    float insufficient_threshold = BATTERY_INSUFFICIENT_THRESHOLD;
    if(result) {
        // Move thresholds based on current state so we don't flip back and forth in states
        switch(this->state_) {
        case kBatteryHigh:
            low_threshold += BATTERY_TOLERANCE;
            break;
        case kBatteryPartial:
            low_threshold += BATTERY_TOLERANCE;
            insufficient_threshold -= BATTERY_TOLERANCE;
            break;
        case kBatteryInsufficient:
            insufficient_threshold += BATTERY_TOLERANCE;
            break;
        default:
            break;
        }
        if(this->voltage_ < insufficient_threshold) {
            this->state_ = kBatteryInsufficient;
        } else if (this->voltage_ < low_threshold) {
            this->state_ = kBatteryPartial;
        } else {
            this->state_ = kBatteryHigh;
        }
        state = this->state_;
        result = true;
    }
    return result;
}

int8_t Battery::RegisterMonitor(BatteryMonitor* monitor) {
    int8_t id = -1;
    /*
    auto iterator = std::find(this->monitors_.begin(), this->monitors_.end(), monitor);
    if(iterator == this->monitors_.end()) {
        this->monitors_.push_back(monitor);
    }*/
    for(int ii = 0; ii < MAX_BATTERY_MONITORS; ii++) {
        if(this->monitors_[ii] == nullptr) {
            this->monitors_[ii] = monitor;
            id = ii;
        }
    }
    return id;
}

bool Battery::UnregisterMonitor(int8_t id) {
    bool result = false;
     /*
    auto iterator = std::find(this->monitors_.begin(), this->monitors_.end(), monitor);
    if (iterator != this->monitors_.end()) {
        this->monitors_.erase(iterator);
        result = true;
    }
    */
    if (id >= 0 && id < MAX_BATTERY_MONITORS) {
        this->monitors_[id] = nullptr;
        result = true;
    }
    return result;
}

void Battery::NotifyBatteriesChanged() {
    /*
    for (const auto &monitor : this->monitors_) {
        monitor->BatterysChangedCallback();
    }*/
    for (int ii = 0; ii < MAX_BATTERY_MONITORS; ii++) {
        if(this->monitors_[ii] != nullptr) {
            this->monitors_[ii]->BatteriesChangedCallback();
        }
    }
}