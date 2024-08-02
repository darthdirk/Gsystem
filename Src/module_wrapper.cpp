#include "battery.h"
#include "gpio.h"
#include "module_wrapper.h"
#include "state_machine.h"
#include "timer.h"
#include "notification.h"
#include "usb.h"
#include "spi.h"
#include "rtc.h"
#include "logger.h"

#ifdef __cplusplus
extern "C" {
#endif

//hw handles
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim15;
extern TIM_HandleTypeDef htim16;
extern SPI_HandleTypeDef hspi2;
extern RTC_HandleTypeDef hrtc;
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart2;


// Static C objs
SMHandle state_machine_handle;
ADCHandle adc_handle;
GPIOHandle gpio_handle;
TimerHandle timer_handle;
BatteryHandle battery_handle;
NotificationHandle notification_handle;
SleepHandle sleep_handle;
SensorHandle sensor_handle;
USBHandle usb_handle;
SPIHandle spi_handle;
RTCHandle rtc_handle;
LoggerHandle logger_handle;
I2CHandle i2c_handle;

// Static C++ obj pointers
ADC* adc;
GPIO* gpio;
Timer* timer;
Battery* battery;
Notification* notification;
Sleep* sleep;
Sensor* sensor;
USBInterface* usb;
SPI* spi;
RealTimeClock* rtc;
Logger* logger;
I2C_Pot* i2c;

void initialize_static_objects() {
    // ORDER MATTERS!!
    adc_handle = adc_create();
    gpio_handle = gpio_create();
    timer_handle = timer_create();
    sleep_handle = sleep_create();
    battery_handle = battery_create();
    usb_handle = usb_create();
    spi_handle = spi_create();
    rtc_handle = rtc_create();
    logger_handle = logger_create();
    i2c_handle = i2c_create();
    sensor_handle = sensor_create();
    notification_handle = notification_create();
    state_machine_handle = state_machine_create();
}

void main_loop() {

    while(1) {
        state_machine_handle->Run();
    }
}

GPIOHandle gpio_create(){
    gpio = new GPIO();
    return gpio;
}

TimerHandle timer_create(){
    /*TODO
    std::vector<TIM_HANDLETYPEDEF*> handles;
    handles.push_back(&htim2);
    handles.push_back(&htim3);
    handles.push_back(&htim15);
    handles.push_back(&htim16);
    */
   TIM_HandleTypeDef *handles[MAX_TIMERS];
   handles[0] = &htim2;
   handles[1] = &htim3;
   handles[2] = &htim15;
   handles[3] = &htim16;
   timer = new Timer(handles, MAX_TIMERS);
   return timer;
}

ADCHandle adc_create() {
    adc = new ADC(&hadc1);
    return adc;
}

BatteryHandle battery_create() {
    battery = new Battery();
    return battery;
}

NotificationHandle notification_create() {
    notification = new Notification();
    return notification;
}

SensorHandle sensor_create() {
    sensor = new Sensor(&hspi2);
    return sensor;
}

SMHandle state_machine_create() {
    return new StateMachine();
}

USBHandle usb_create() {
    usb = new USBInterface();
    return usb;
}

SPIHandle spi_create() {
    spi = new SPI(&spi2);
    return spi;
}

RTCHandle rtc_create() {
    rtc = new RealTimeClock(&hrtc);
    return rtc;
}

I2CHandle = i2c_create() {
    i2c = new I2C_Pot(&hi2c1);
    return i2c;
}

LoggerHandle = logger_create() {
    logger = new Logger();
    return logger;
}

void usb_notify_receive(uint8_t *buffer, uint32_t length) {
    usb->NotifyReceive(buffer, length);
}

void usb_notify_disconnected() {
    usb->NotifyDisconnected();
}

// Unit Tests

#ifdef __cplusplus
}
#endif