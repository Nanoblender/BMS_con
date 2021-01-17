
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>

//number of ticks per turn, encoder 24CPR
#define ENCODER_PERIOD (24*4-1)
// GPIO_PUPD_PULLUP GPIO_PUPD_PULLDOWN GPIO_PUPD_NONE
#define ENCODER_INPUT_CFG  GPIO_PUPD_PULLUP

/*******************************
Left encoder configuration
*******************************/
#define ENCODER_RCC_TIM RCC_TIM3
#define ENCODER_TIM TIM3

#define ENCODER_CH1_PORT GPIOA
#define ENCODER_CH1_PORT_RCC RCC_GPIOA
#define ENCODER_CH1_AF GPIO_AF2
#define ENCODER_CH1_PIN GPIO6

#define ENCODER_CH2_PORT GPIOA
#define ENCODER_CH2_PORT_RCC RCC_GPIOA
#define ENCODER_CH2_AF GPIO_AF2
#define ENCODER_CH2_PIN GPIO7



void encoder_setup(void);
int encoder_get_counter(void);