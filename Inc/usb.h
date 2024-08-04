#ifndef INC_USB_H_
#define INC_USB_H_

#include "stm32l4xx_hal.h"
#include <map>

class USBMonitor {
public:
    virtual void USBReceiveCallback(uint8_t *buffer, uint32_t length) = 0;
    virtual void USBDisconnectedCallback() = 0;
    virtual void USBConnectedCallback() = 0;
};

class USBInterface {
public :
    USBInterface();
    virtual ~USBInterface();

    // TODO: I didn't wrap up the transmit function here, but I could

    int16_t RegisterMonitor(USBMonitor* monitor);
    uint16_t UnregisterMonitor(int16_t monitor_id);

    // In order to call these from the C USB definition, we have to expose them as public.
    void NotifyReceive(uint8_t *buffer, uint32_t length);
    void NotifyDisconnected();
    void NotifyConnected();

private :
    std::map<int16_t, USBMonitor*> monitors_;

};

#endif /* INC_USB_H_ */
