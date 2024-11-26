/*
 * trigger.c
 *
 *  Created on: Nov 25, 2024
 *      Author: GeorgeVigelette
 */

#include "main.h"
#include "trigger.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>


// setup default
Trigger_Config_t trigger_config = { 40, 1000, 100, 100 };

HAL_StatusTypeDef Trigger_SetConfig(const Trigger_Config_t *config) {
    if (config == NULL) {
        return HAL_ERROR; // Null pointer guard
    }

    // Add range checks for the configuration parameters
    if (config->frequencyHz == 0 || config->triggerPulseWidthUsec == 0) {
        return HAL_ERROR; // Invalid configuration values
    }

    // TIM2 Configuration: Trigger Frequency
    uint32_t tim2_period = 1000000 / config->frequencyHz; // Convert frequency to period in microseconds
    __HAL_TIM_SET_AUTORELOAD(&htim2, tim2_period - 1);    // Set TIM2 period
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, config->triggerPulseWidthUsec); // Set PWM pulse width for TIM2 Channel 1

    // TIM4 Configuration: Laser Pulse Delay
    uint32_t adjusted_pulse_delay = config->laserPulseDelayUsec/2;
    __HAL_TIM_SET_AUTORELOAD(&htim4, adjusted_pulse_delay); // Set TIM4 period to ensure timing range
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, adjusted_pulse_delay/2); // Set PWM for 50% duty cycle on Channel 1

    // TIM5 Configuration: Laser Pulse Width (with inverted signal logic)

    __HAL_TIM_SET_AUTORELOAD(&htim5, config->laserPulseWidthUsec + adjusted_pulse_delay); // Extend by 50 us
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, adjusted_pulse_delay); // Set inverted PWM pulse width on Channel 2

    // Update the global trigger configuration
    trigger_config = *config;
    return HAL_OK;
}


HAL_StatusTypeDef Trigger_Start() {
    if (HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1) != HAL_OK) {
        return HAL_ERROR; // Handle error
    }
    if (HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2) != HAL_OK) {
        return HAL_ERROR; // Handle error
    }
    if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) != HAL_OK) {
        return HAL_ERROR; // Handle error
    }

    return HAL_OK;
}

HAL_StatusTypeDef Trigger_Stop() {
    if (HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1) != HAL_OK) {
        return HAL_ERROR; // Handle error
    }
    if (HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_2) != HAL_OK) {
        return HAL_ERROR; // Handle error
    }
    if (HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1) != HAL_OK) {
        return HAL_ERROR; // Handle error
    }

    return HAL_OK;
}
