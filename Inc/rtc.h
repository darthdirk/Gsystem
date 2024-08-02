#ifndef INC_REALTIMECLOCK_H_
#define INC_REALTIMECLOCK_H_

#include "stm32l4xx_hal.h"

#define DAYS_TO_SECS 86400


class RealTimeClock {

public:
    RealTimeClock(RTC_HandleTypeDef *handle);
    virtual ~RealTimeClock();

    void SetTime(RTC_TimeTypeDef time) {
        HAL_RTC_SetTime(this->handle_, &time, RTC_FORMAT_BIN);
    }
    void GetTime() {
        HAL_RTC_GetTime(this->handle_, &sTime_, RTC_FORMAT_BIN);
    }
    void SetDate(RTC_DateTypeDef date) {
        HAL_RTC_SetDate(this->handle_, &Date_, RTC_FORMAT_BIN);
    }
    void GetDate() {
        HAL_RTC_GetDate(this->handle_, &sDate_, RTC_FORMAT_BIN);
    }

    void Date(char* data);

    void Time(char* time);

    void PrintTimeDate(char *);

    uint32_t DataToEpoch();

private:
    RTC_HandleTypeDef *handle_;
    RTC_TimeTypeDef sTime_;
    RTC_DateTypeDef sDate_;

    uint32_t epoch_days_fast(uint8_t y, uint8_t m, uint8_t d);
    uint32_t hms_to_time(uint8_t h, uint8_t m, uint8_t s);
};

#endif /* INC_REALTIMECLOCK_H_*/