/*
 * rover_firmware.c
 *
 *  Created on: Jun 11, 2025
 *      Author: Miriam Vitolo
 */
#include "cmsis_os.h"
#include "encoder.h"
#include "rover_firmware.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

static rover_t rover = {
    .encoder1_config = {
        .cpr = ENCODER_CPR,
        .gear_ratio = ENCODER_GEAR_RATIO,
        .htim = &htim1,
        .n_channels = ENCODER_N_CHANNELS,
        .sampling_time = ENCODER_SAMPLING_TIME
    },
    .encoder2_config = {
        .cpr = ENCODER_CPR,
        .gear_ratio = ENCODER_GEAR_RATIO,
        .htim = &htim2,
        .n_channels = ENCODER_N_CHANNELS,
        .sampling_time = ENCODER_SAMPLING_TIME
    },
    .encoder3_config = {
        .cpr = ENCODER_CPR,
        .gear_ratio = ENCODER_GEAR_RATIO,
        .htim = &htim3,
        .n_channels = ENCODER_N_CHANNELS,
        .sampling_time = ENCODER_SAMPLING_TIME
    },
    .encoder4_config = {
        .cpr = ENCODER_CPR,
        .gear_ratio = ENCODER_GEAR_RATIO,
        .htim = &htim4,
        .n_channels = ENCODER_N_CHANNELS,
        .sampling_time = ENCODER_SAMPLING_TIME
    },
    .motor_timer = &htim8
};

static rover_t * const instance = &rover;

rover_t* const rover_get_instance() {
    return instance;
}

Rover_StatusTypeDef rover_init(void){
	Rover_StatusTypeDef status = ROVER_ERROR;
	if ((encoder_init(&rover.encoder1, &rover.encoder1_config)== ENCODER_OK) &&
			(encoder_init(&rover.encoder2, &rover.encoder2_config) == ENCODER_OK) &&
			(encoder_init(&rover.encoder3, &rover.encoder3_config) == ENCODER_OK) &&
			(encoder_init(&rover.encoder4, &rover.encoder4_config) == ENCODER_OK))
	{
		status = ROVER_OK;
	}
	return status;
}

HAL_StatusTypeDef Start_PWM_Channels(void){
	HAL_StatusTypeDef status = HAL_ERROR;

	if ((rover.motor_timer != NULL) &&(HAL_TIM_PWM_Start(rover.motor_timer, TIM_CHANNEL_1) == HAL_OK) &&
		(HAL_TIM_PWM_Start(rover.motor_timer, TIM_CHANNEL_2) == HAL_OK) &&
		(HAL_TIM_PWM_Start(rover.motor_timer, TIM_CHANNEL_3) == HAL_OK) &&
		(HAL_TIM_PWM_Start(rover.motor_timer, TIM_CHANNEL_4) == HAL_OK))
	{
    status = HAL_OK;
	}
	return status;
}

Motor_StatusTypeDef stop_all_motors(void){
	Motor_StatusTypeDef status = MOTOR_ERROR;

	if (rover.motor_timer != NULL)
	{
		drive_motor(rover.motor_timer, TIM_CHANNEL_1, 0);
		drive_motor(rover.motor_timer, TIM_CHANNEL_2, 0);
		drive_motor(rover.motor_timer, TIM_CHANNEL_3, 0);
		drive_motor(rover.motor_timer, TIM_CHANNEL_4, 0);

		status = MOTOR_OK;
	}
	return status;
}

Encoder_StatusTypeDef motor_step(void){
	Encoder_StatusTypeDef status = ENCODER_ERROR;

    if ((encoder_update_speed(&rover.encoder1) == ENCODER_OK) &&
        (encoder_update_speed(&rover.encoder2) == ENCODER_OK) &&
        (encoder_update_speed(&rover.encoder3) == ENCODER_OK) &&
        (encoder_update_speed(&rover.encoder4) == ENCODER_OK))
    {
        status = ENCODER_OK;
    }

    return status;
}

void drive_motor(TIM_HandleTypeDef* timer,HAL_TIM_ActiveChannel channel, double desiredValue){
	uint32_t compare_value;
	if(timer != NULL){
		compare_value = (uint32_t)(((CONVERSION_FACTOR*desiredValue + STOP_PWM_TENSION)*((float)timer->Init.Period))/MAX_PWM_TENSION);
		__HAL_TIM_SET_COMPARE(timer,channel,compare_value);
	}
}

