#ifndef INC_SENSOR_H_
#define INC_SENSOR_H_

#include "stm32l4xx_hal.h"
#include "gpio.h"
#include "timer.h"
#include "logger.h"
#include "i2c_pot.h"
#include <string>

class Sensor {

public:
    Sensor(SPI_HandleTypeDef *spi);
    virtual ~Sensor();

    bool Configure();
    bool Sleep();

    bool Calibrate(uint32_t charge_delay_usec, uint32_t on_delay_usec, uint32_t off_delay_usec);
    bool BurstRead(uint32_t &value, uint32_t charge_delay_usec, uint32_t on_delay_usec, uint32_t off_delay_usec);
    bool DetectObj(uint32_t charge_delay_usec, uint32_t on_delay_usec, uint32_t off_delay_usec);
    void SetContinuous(bool);
    void GetContinuous();
    uint16_t *GetSampleData();
    void ResizeSampleData();
    void SetAllSamples(bool);
    bool GetAllSamples();
    void SetAllSamplesSingle(bool);
    bool GetAllSamplesSingle();
    bool IsCalibrated();
    void Delay(uint32_t usec);

    bool SingleRead(uint32_t &value, uint32_t on_delay_usec, uint32_t off_delay_usec);

private:
    SPI_HandleTypeDef *spi_;
    bool adc_ready_;

    // bool SingleRead(uint32_t &value, uint32_t on_delay_usec, uint32_t off_delay_usec);
    bool continuous_ = false;
    bool all_samples_ = false;
    bool all_samples_single_ = false;

    uint16_t sample_data_[MAX_NUM_SAMPLES_PER_BURST];
    bool obj_detected_;
};

#endif  /* INC_SENSOR_H_ */