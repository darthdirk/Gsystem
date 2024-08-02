#ifndef INC_LOGGER_H_
#define INC_LOGGER_H_

#include <stdio.h>
#include "gpio.h"
#include "rtc.h"
#include "spi.h"
#include "battery.h"
#include "version.h"
#include "string.h"

#define MULTIPLE_CONFIGURATION_DATA_SETS
//#define LOGGER_TEST_CODE

enum class EventID : uint32_t {
    NewBatteries = 1,
    DoorOpened,
    DoorClosed,
    BattCoverRemoved,
    BattCoverReplaced,
    ObjDetected,
    ObjRemoved,
    DAQInProgress,
    ServiceModeEntered,
    ServiceModeExit,
    RCTUpdated,
    DataDownloadStarted,
    DataDownloadCompleted,
    BatteryCheckPreformed
};

enum class LogTypeID : uint8_t {
    None,
    SWVerHistory,
    BattChgHistory,
    DataDwnLdHistory,
    EventLogData
};

#define DEFAULT_LEDON_TIME 5000
#define DEFAULT_LEDOFF_TIME 95000
#define DEFAULT_CHARGEDELAY_TIME 1000
#define DEFAULT_NUM_BURST 5
#define DEFAULT_INTERBURST_DELAY 30
#define MAX_NUM_SAMPLES_PER_BURST 20

#define EVENT_DATA_PAD_SIZE 5
#define MAX_32BIT           0xffffffff
#define DEVICE_ID_SIZE      128
#define DATA_LOG_DATA_SIZE  12
#define MAX_USER_ID_SIZE    128
#define EVENTS_PER_PAGE (NVRAM_BYTES_PER_PAGE / sizeof(EventData))
#ifdef MULTIPLE_CONFIGURATION_DATA_SETS
#define MAX_NUM_CONFIGURATION_DATA_SETS 10
#define CONFIGURATION_DATA_SCHEMA 0x12345678
#define MAX_NUM_CHARS_IN_NAME   32

#endif
typedef struct {
    uint32_t counter;
    uint32_t time;
    EventID  eventID;
    uint32_t sampleADC;
} EventData;

typedef struct {
    uint32_t date_time;
    uint8_t data[DATA_LOG_DATA_SIZE];
} DataLogData;

class Logger : public BatteryMonitor, public GPIOMonitor {
private:
    bool ServiceAddDateLogData(uint32_t page_address, uint8_t *, uint8_t);
    bool ServiceGetDateLogHistory(uint32_t page_address, DataLogData *date_log_history, uint16_t *number);
    bool ServiceGetEventLogIndices();
    bool ServiceWriteEventIndices();
    GPIO_PinState battery_cover_state_;
    GPIO_PinState door_cover_state_;

    uint8_t event_page_buffer_[NVRAM_BYTES_PER_PAGE];
    uint16_t event_page_buff_idx_address_;

#ifdef MULTIPLE_CONFIGURATION_DATA_SETS
#pragma pack (4)
    struct Config_Data_New {
        char     data_set_name_[MAX_NUM_CHARS_IN_NAME];
        uint32_t led_on_time_ = DEFAULT_LEDON_TIME;
        uint32_t led_off_time_ = DEFAULT_LEDOFF_TIME;
        uint32_t num_burst_ = DEFAULT_NUM_BURST;
        uint32_t charge_delay_ = DEFAULT_CHARGEDELAY_TIME;
        uint32_t interburst_delay_ = DEFAULT_INTERBURST_DELAY;
        uint32_t pot_val_ = 0;
        uint32_t spare_1_;
        uint32_t spare_2_;
    } config_data_new_;

#pragma pack (4)
    struct Config_Data_With_Schema {
        Config_Data_New  data[MAX_NUM_CONFIGURATION_DATA_SETS];
        uint32_t         schema;
        uint32_t         saved_config_slot;
    } config_data_with_schema_;
#endif


    struct Config_Data {
        uint32_t programmed = 0xa5a5a5a5;
        uint32_t led_on_time_ = 5000; //microsec
        uint32_t led_off_time_ = 1000;
        uint32_t num_burst_ = 5;
        uint32_t charge_delay_ = 1000;
        uint32_t interburst_delay_ = 30; // sec
        uint32_t pot_val = 0;
    } config_data_;

    struct EventIndices {
        // The addr of the beginning of the current head page of the event
        uint32_t event_head_page_;
        // The addr of the beginning of the current tail page of the event
        uint32_t event_tail_page_;
        uint32_t event_head_address_;
        uint32_t event_tail_address_;
        uint32_t counter_;
    } event_indices_;

    int8_t battery_monitor_id_;
    int16_t battery_cover_monitor_id_;
    int16_t door_cover_monitor_id_;
    struct EventIndices lcl_event_indices;

public:
    Logger();
    virtual ~Logger();
    
    void TimeDate(void);

    void ResetIndices();
    void BatteriesChangedCallback();
    void GPIOCallback(GPIOPinId pin);
    bool EventAddLogEntry(EventID event_id, uint32_t ADCdample);
    bool EventWriteLog();
    bool ServiceGetDeviceID(char *, uint8_t&);
    bool ServiceSetDeviceID(char *, uint8_t);
    bool ServiceSetUserID(char *, uint8_t);
    bool SerciceGetUserID(char *, uint8_t&);
    bool ServiceAddSoftwareInstallation();
    bool ServiceGetSoftwareVersionHistory(DateLogData *sw_ver_history. uint16_t *number);
    bool ServiceWriteBatteryChange();
    bool ServiceGetBatteryChangeHistory(DateLogData *batt_chg_history, uint16_t *number);
    bool ServiceWriteDataDownload(uint32_t start_counter, uint32_t end_counter);
    bool ServiceGetDataDownloadHistory(DateLogData *download_history, uint16_t *number);
    bool GetEventLogData(EventData *, uint32_t &number, bool&);

    void SetLedOnTime(uint32_t);
    void SetLedOffTime(uint32_t);
    void SetNumBurst(uint32_t);
    void SetChargeDelay(uint32_t);
    void SetInterburstDelay(uint32_t);
    void SetPotVal(uint32_t);
    uint32_t GetLedOnTime();
    uint32_t GetLedOffTime();
    uint32_t GetNumBurst();
    uint32_t GetChargeDelay();
    uint32_t GetInterburstDelay();
    uint32_t GetVersion();
    uint32_t GetPotVal();
    bool WriteConfig();
#ifdef MULTIPLE_CONFIGURATION_DATA_SETS
    bool WriteConfig(char * name);
    bool ReadConfig(uint32_t config_slot);
    bool ReadConfig(char * name);
    bool EraseAllConfig(char * name);
    bool ReaseConfig(char * name);
    bool GetConfigNames(char name[MAX_NUM_CONFIGURATION_DATA_SETS][MAX_NUM_CHARS_IN_NAME], uint16_t &counter);
    bool ValidateConfigData();
#endif  

    uint32_t GetCurrentCounter();
    uint32_t GetHeadPage();
    uint32_t GetTailPage();
};

#endif /* INC_LOGGER_H_ */
