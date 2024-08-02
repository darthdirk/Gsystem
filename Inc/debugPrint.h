#ifndef INC_DEBUGPRINT_H_
#define INC_DEBUGPRINT_H_

#include "usbd_cdc_if.h"

class DebugPrint {
public:
    void debugPrint(const char *buffer, uint32_t length);
    void debugTransmit(uint8_t *buffer, uint32_t length);
};

#endif /* INC_DEBUGPRINT_H_*/