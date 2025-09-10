/**
 * @file encoder.c
 *
 *
 * Created on: Jun 4, 2024
 * Author: Miriam Vitolo
 */

#include "encoder.h"
#include <stddef.h>
#include <math.h>


#define ENCODER_MAX_TICKS_PER_SEC   8400U

Encoder_StatusTypeDef encoder_init(encoder_t * encoder,
                                   const encoder_config_t * config)
{
    if ((encoder == NULL) || (config == NULL) || (config->htim == NULL) ||
        (config->cpr == 0U) || (config->gear_ratio == 0U) || (config->n_channels == 0U))
    {
        return ENCODER_ERROR;
    }

    encoder->config            = config;
    encoder->actual_speed_rpm  = 0.0;
    encoder->rotation_status   = DEFAULT_DIRECTION;
    HAL_TIM_Encoder_Start(config->htim, TIM_CHANNEL_ALL);
    encoder->last_tick_count   = __HAL_TIM_GET_COUNTER(config->htim);
    return ENCODER_OK;
}

Encoder_StatusTypeDef encoder_update_speed(encoder_t * encoder)
{
    if ((encoder == NULL) || (encoder->config == NULL) || (encoder->config->htim == NULL) ||
        (encoder->config->sampling_time<= 0.0f))
    {
        return ENCODER_ERROR;
    }

    TIM_HandleTypeDef * htim = encoder->config->htim;
    uint32_t period         = htim->Init.Period + 1U;

    uint32_t counter1 = encoder->last_tick_count;
    uint32_t counter2 = __HAL_TIM_GET_COUNTER(htim);
    uint8_t backward = __HAL_TIM_IS_TIM_COUNTING_DOWN(htim);
    int32_t delta = 0;

    if (counter2 == counter1)
    {
        delta = 0;
        encoder->rotation_status = DEFAULT_DIRECTION;
    }
    else if (backward==1)
    {
        if (counter2 < counter1)
        {
            delta = counter1 - counter2;
        }
        else
        {
            delta = (period - counter2) + counter1;
        }
        encoder->rotation_status = BACKWARD_DIRECTION;
    }
    else
    {
        if (counter2 > counter1)
        {
            delta = counter2 - counter1;
        }
        else
        {
            delta = (period - counter1) + counter2;
        }
        encoder->rotation_status = FORWARD_DIRECTION;
    }

    const double ticks_per_rev = (double)(encoder->config->cpr *
                                          encoder->config->gear_ratio *
                                          encoder->config->n_channels);

    encoder->actual_speed_rpm = ((double)delta * 60.0) / (ticks_per_rev * encoder->config->sampling_time);
    if((fabs(encoder->actual_speed_rpm) > 180.0f) || (fabs(encoder->actual_speed_rpm) < 0.1f)){
    	encoder->actual_speed_rpm = 0.0f;
    }
    if(encoder->rotation_status == BACKWARD_DIRECTION){
    	encoder->actual_speed_rpm *= -1;
    }
    encoder->last_tick_count = counter2;

    return ENCODER_OK;
}

