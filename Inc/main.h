
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l4xx_hal.h"


void Error_Handler(void);

#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOH
#define DOOR_COVER_Pin GPIO_PIN_1
#define DOOR_COVER_GPIO_Port GPIOA
#define DOOR_COVER_EXTI_IRQn EXTI1_IRQn
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define SMPS_EN_Pin GPIO_PIN_4
#define SMPS_EN_GPIO_Port GPIOA
#define SMPS_V1_Pin GPIO_PIN_5
#define SMPS_V1_GPIO_Port GPIOA
#define SMPS_PG_Pin GPIO_PIN_6
#define SMPS_PG_GPIO_Port GPIOA
#define SMPS_SW_Pin GPIO_PIN_7
#define SMPS_SW_GPIO_Port GPIOA
#define BATTERY_COVER_Pin GPIO_PIN_2
#define BATTERY_COVER_GPIO_Port GPIOB
#define BATTERY_COVER_EXTI_IRQn EXTI2_IRQn
#define ADC_SELECT_Pin GPIO_PIN_12
#define ADC_SELECT_GPIO_Port GPIOB
#define GREEN_LED_Pin GPIO_PIN_13
#define GREEN_LED_GPIO_Port GPIOB
#define AUDIO_ANNUNCIATOR_Pin GPIO_PIN_8
#define AUDIO_ANNUNCIATOR_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define SENSOR_LED_ON_Pin GPIO_PIN_5
#define SENSOR_LED_ON_GPIO_Port GPIOB
#define ADC_EOC_Pin GPIO_PIN_6
#define ADC_EOC_GPIO_Port GPIOB
#define ADC_EOC_EXTI_IRQn EXTI9_5_IRQn
#define ADC_START_Pin GPIO_PIN_7
#define ADC_START_GPIO_Port GPIOB
#define SENSOR_LED_CHARGE_Pin GPIO_PIN_8
#define SENSOR_LED_CHARGE_GPIO_Port GPIOB
#define SENSOR_LED_ADC_POWER_Pin GPIO_PIN_9
#define SENSOR_LED_ADC_POWER_GPIO_Port GPIOB


#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
