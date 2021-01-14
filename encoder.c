#include "encoder.h"

void encoder_setup(void)
{
    rcc_periph_clock_enable(ENCODER_RCC_TIM);
    timer_set_period(ENCODER_TIM, ENCODER_PERIOD);
    timer_slave_set_mode(ENCODER_TIM, 0x3); // encoder
    timer_ic_set_input(ENCODER_TIM, TIM_IC1, TIM_IC_IN_TI1);
    timer_ic_set_input(ENCODER_TIM, TIM_IC2, TIM_IC_IN_TI2);
    timer_direction_down(ENCODER_TIM);
    timer_enable_counter(ENCODER_TIM);

    rcc_periph_clock_enable(ENCODER_CH1_PORT_RCC);
    gpio_mode_setup(ENCODER_CH1_PORT, GPIO_MODE_AF, ENCODER_INPUT_CFG, ENCODER_CH1_PIN);
	gpio_set_af(ENCODER_CH1_PORT, ENCODER_CH1_AF, ENCODER_CH1_PIN);
    
    rcc_periph_clock_enable(ENCODER_CH2_PORT_RCC);
	gpio_mode_setup(ENCODER_CH2_PORT, GPIO_MODE_AF, ENCODER_INPUT_CFG, ENCODER_CH2_PIN);
	gpio_set_af(ENCODER_CH2_PORT, ENCODER_CH2_AF, ENCODER_CH2_PIN);
}

int encoder_get_counter(void)
{
  if(ENCODER_INVERSION)return ENCODER_PERIOD-timer_get_counter(ENCODER_TIM);
  return timer_get_counter(ENCODER_TIM);
}


