/* Copyright (c) 2009 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
* @defgroup blinky_example_pca10001_main main.c
* @{
* @ingroup blinky_example_pca10001
*
* @brief Blinky Example Application main file.
*
* This file contains the source code for a sample application using GPIO to drive LEDs.
*
*/

#include <stdbool.h>
#include <stdint.h>
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "boards.h"

static void gpio_init(void)
{
    nrf_gpio_cfg_input(BUTTON_0, BUTTON_PULL);
    nrf_gpio_cfg_output(22);

    //nrf_gpio_pin_write(22, BUTTON_0); //처음에 led를 on으로 시작하는 경우

    // Enable interrupt:
    NVIC_EnableIRQ(GPIOTE_IRQn);
    NRF_GPIOTE->CONFIG[0] =  (GPIOTE_CONFIG_POLARITY_Toggle << GPIOTE_CONFIG_POLARITY_Pos)
                           | (0 << GPIOTE_CONFIG_PSEL_Pos)  
                           | (GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos);
    NRF_GPIOTE->INTENSET  = GPIOTE_INTENSET_IN0_Set << GPIOTE_INTENSET_IN0_Pos;
}


/** @brief Function for handling the GPIOTE interrupt which is triggered on pin 0 change.
 */
void GPIOTE_IRQHandler(void)
{
    // Event causing the interrupt must be cleared.
    if ((NRF_GPIOTE->EVENTS_IN[0] == 1) && 
        (NRF_GPIOTE->INTENSET & GPIOTE_INTENSET_IN0_Msk))
    {
        NRF_GPIOTE->EVENTS_IN[0] = 0;
    }
    //nrf_gpio_pin_toggle(8);
    nrf_gpio_pin_toggle(22);
    //nrf_gpio_pin_toggle(18);
}



/**
 * @brief Function for application main entry.
 */
int main(void)
{
  // Configure LED-pins as outputs
  nrf_gpio_cfg_output(LED_0);
  nrf_gpio_cfg_output(LED_1);
  nrf_gpio_cfg_output(22);

  gpio_init();
	
  // LED 0 and LED 1 blink alternately.
  while(true)
  {
		/*
    nrf_gpio_pin_clear(LED_0);
    nrf_gpio_pin_set(LED_1);
    nrf_gpio_pin_clear(22);
		
    nrf_delay_ms(500);
    
    nrf_gpio_pin_clear(LED_1);
    nrf_gpio_pin_set(LED_0);
    nrf_gpio_pin_set(22);
		
    nrf_delay_ms(500);
		*/
  }
}

/**
 *@}
 **/
