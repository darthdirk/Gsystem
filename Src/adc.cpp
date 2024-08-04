#include "adc.h"

// Constructor
ADC::ADC(ADC_HandleTypeDef* handle) {
    this->handle_ = handle;
}

// Destructor
ADC::~ADC() {

}

bool ADC::Read(uint32_t &value) {
    // Require ADC start event for each channel
    this->handle_->Init.DiscontinuousConvMode = ENABLE;

    // CH1
    HAL_ADC_Start(this->handle_);
    // Wait for the EOC flag
    HAL_ADC_PollForConversion(this->handle_, HAL_MAX_DELAY);
    value = HAL_ADC_GetValue(this->handle_);

    // TODO: error handling
    return true;
}