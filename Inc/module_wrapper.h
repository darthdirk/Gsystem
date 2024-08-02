
#ifndef __STATE_MACHINE_WRAPPER_H
#define __STATE_MACHINE_WRAPPER_H

#include "stdint.h"

#ifdef __cplusplus
extern "C" {
#endif

// C type definitions for C++ objects
typedef struct StateMachine* SMHandle;
typedef struct ADC* ADCHandle;
typedef struct Timer* TimerHandle;
typedef struct GPIO* GPIOHandle;
typedef struct Battery* BatteryHandle;
typedef struct Notification* NotificationHandle;
typedef struct Sleep* SleepHandle;
typedef struct Sensor* SensorHandle;
typedef struct USBInterface* USBHandle;
typedef struct SPI* SPIHandle;
typedef struct RealTimeClock* RTCHandle;
typedef struct Logger* LoggerHandle;
typedef struct I2C_Pot* I2CHandle;

extern volatile uint8_t usb_busy;

// interface functions for main.c
void initialize_static_objects();
void main_loop();

// Functions to create static objects
SMHandle state_machine_create();
ADCHandle adc_create();
TimerHandle timer_create();
GPIOHandle gpio_create();
BatteryHandle battery_create();
SleepHandle sleep_create();
SensorHandle sensor_create();
NotificationHandle notification_create();
USBHandle usb_create();
SPIHandle spi_create();
RTCHandle rtc_create();
LoggerHandle logger_create();
I2CHandle i2c_create();

void usb_notify_receive(uint8_t *buffer, uint32_t length);
void usb_notify_disconnected();
void usb_notify_connected();

#ifdef __cplusplus
}
#endif

#endif
