#ifndef SRC_STATESERVICEMODE_H_
#define SRC_STATESERVICEMODE_H_

#define MAX_COMMAND_LENGTH 160

#include <cstring>
#include <ctype.h>
#include <string>
#include <map>
#include "state.h"
#include "adc.h"
#include "battery.h"
#include "usbd_cdc_if.h"
#include "rtc.h"
#include "version.h"
#include "main.h"
#include "i2c_pot.h"

class StateServiceMode: public State {
public:
    StateServiceMode();
    virtual ~StateServiceMode();

    States StateDo();

    void StateEntry();

    void StateExit();

    void GPIOCallback(GPIOPinId pin);

    void TimerCallback(int8_t id);

    void USBReceiveCallback(uint8_t *buffer, uint32_t length);

    void USBDisconnectedCallback();

    void USBConnectedCallback();

    void Transmit(const char *buffer, uint32_t length);

private:
    int8_t usb_monitor_id_;
    char command_buffer_[MAX_COMMAND_LENGTH + 1];
    uint8_t command_length_;
    GPIO_PinState battery_cover_state_;
    GPIO_PinState door_cover_state_;
    int8_t timer_id_;
    int16_t battery_cover_monitor_id_;
    int16_t door_cover_monitor_id_;
    int16_t exit_delay_counter_ = -1;
    int16_t burst_delay_counter_ = -1;
    uint16_t num_events_to_transmit_ = 0;

    struct LogDataParams {
        LogTypeID log_type = LogTypeID::None;
        bool delete_read_contents;
    } log_data_params_;

    void CommandRead(char **args, uint8_t argv);

    void CommandSet(char **args, uint8_t argv);

    void CommandSleep(char **args, uint8_t argv);

    void CommandConfig(char **args, uint8_t argv);

    //void Transmint(const char *buffer, uint32_t length);

    void Transmit(uint8_t *buffer, uint32_t length);

    void ProcessCommand();

    void ServiceOutputEventLog();
    void ServiceOutputDataDownloadStarted();
    void ServiceOutputDataDownloadComplete();
    void ServiceOutputLog();
    void GetConfigNames();
    bool CommandSetTime(char *);
    bool CommandSetDate(char *);
    bool CommandSetPotVal(char *);
    void ShowAllSamples();

};

#endif  /* SRC_STATESERVICEMODE_H_ */
