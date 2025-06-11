/**
 * @file    encoder.h
 * @author  Miriam Vitolo
 * @brief   Incremental encoder driver for STM32F767 MCUs (HAL-based)
 * @version 1.0
 * @date    2025-06-04
 *
 * @copyright
 * Copyright (c) 2025
 * All rights reserved.
 *
 * @details
 * This header defines the public API and data structures required to manage an
 * incremental quadrature encoder attached to a motor shaft.  The interface
 * relies exclusively on the STM32 HAL TIM encoder mode.  All computations are
 * performed in the host MCU; no interrupts are installed by this driver, so
 * it can be used either with polling or inside a user-defined ISR / RTOS task.
 *
 * The design strictly follows MISRA C:2012 requirements:
 *  - No function-like macros
 *  - Fixed-width integer types
 *  - Header self-containment
 *  - All identifiers documented and namespaced
 */

#ifndef ENCODER_H
#define ENCODER_H

/*=========================================================================*/
/*  INCLUDES                                                               */
/*=========================================================================*/

#include <stdint.h>      /* MISRA compliant fixed-width types */
#include "tim.h"         /* HAL TIM_HandleTypeDef definition  */

/*=========================================================================*/
/*  PUBLIC MACROS                                                          */
/*=========================================================================*/

/**
 * @typedef Encoder_StatusTypeDef
 * @brief   Generic return type used by all public functions of this module.
 */
typedef uint8_t Encoder_StatusTypeDef;

/** @brief Operation completed successfully. */
#define ENCODER_OK      ((Encoder_StatusTypeDef)0U)

/** @brief Generic failure (invalid parameters, HAL error, timeout, …). */
#define ENCODER_ERROR   ((Encoder_StatusTypeDef)1U)

/** @brief Counts per revolution of the encoder. */
#define ENCODER_CPR         24

/** @brief Gear reduction ratio between motor shaft and output shaft. */
#define ENCODER_GEAR_RATIO  51

/** @brief Number of encoder channels used */
#define ENCODER_N_CHANNELS  2

/** @brief Sampling time in seconds for encoder readings. */
#define ENCODER_SAMPLING_TIME  0.005


/*=========================================================================*/
/*  CONFIGURATION STRUCTURE                                                */
/*=========================================================================*/

/**
 * @struct  Encoder_config_t
 * @brief   Immutable HW-level parameters of a single encoder instance.
 *
 * @var htim       Pointer to a HAL timer already configured in encoder mode.
 * @var gear_ratio Mechanical gear ratio between motor shaft and final output.
 * @var cpr        Encoder Counts-Per-Revolution (single channel edges).
 */
typedef struct
{
    TIM_HandleTypeDef * htim;       /*!< HAL timer handle (must not be NULL). */
    uint16_t            gear_ratio; /*!< Gear ratio (≥ 1).                    */
    uint16_t            cpr;        /*!< Counts per revolution (≥ 1).         */
    uint8_t 			n_channels; /*!< Number of encoder channels	*/
    double		   		sampling_time;
} encoder_config_t;

/*=========================================================================*/
/*  ENUMERATION TYPES                                                      */
/*=========================================================================*/

/**
 * @enum    EncoderRotationEnum_t
 * @brief   Represents the rotation direction of the encoder.
 */
typedef enum {
    DEFAULT_DIRECTION  = 0,
    BACKWARD_DIRECTION = 1,
    FORWARD_DIRECTION  = 2
} EncoderRotationEnum_t;

/*=========================================================================*/
/*  RUNTIME STRUCTURE                                                      */
/*=========================================================================*/

/**
 * @struct  Encoder_t
 * @brief   Opaque handle storing dynamic state of an encoder instance.
 *
 * @var config             Pointer to constant configuration data.
 * @var last_tick_count    Counter value sampled on the previous update.
 * @var actual_speed_rpm   Last computed speed, expressed in output-shaft RPM.
 */
typedef struct
{
    const encoder_config_t * config;           /*!< Immutable configuration.      */
    uint32_t                last_tick_count;  /*!< Last TIM counter snapshot.    */
    double                	actual_speed_rpm;    /*!< Speed in revolutions-per-min. */
    EncoderRotationEnum_t 	rotation_status;
} encoder_t;

/*=========================================================================*/
/*  PUBLIC FUNCTION PROTOTYPES                                             */
/*=========================================================================*/

/**
 * @brief Initialise an @ref Encoder_t object to its default runtime values.
 *
 * The caller must have:
 *  - Enabled the TIM peripheral clock.
 *  - Configured the timer in quadrature-encoder interface mode.
 *  - Started the timer counter (HAL_TIM_Encoder_Start or equivalent).
 *
 * @param[out] encoder     Pointer to the @ref Encoder_t instance to initialise.
 * @param[in]  config      Pointer to a valid @ref Encoder_config_t structure.
 *
 * @return ENCODER_OK on success, otherwise ENCODER_ERROR.
 */
Encoder_StatusTypeDef encoder_init(encoder_t * encoder,
                                   const encoder_config_t * config);

/**
 * @brief Update the speed estimate (RPM) of the selected encoder.
 *
 * This function computes the difference between the current TIM counter value
 * and the value stored in @p encoder->last_tick_count, compensates for 16-bit
 * rollover, converts ticks to mechanical revolutions using the encoder's CPR
 * and gear ratio, and computes a rotational speed in RPM.
 *
 * @param[in,out] encoder  Pointer to the encoder instance.
 *
 * @return ENCODER_OK on success, otherwise ENCODER_ERROR.
 */
Encoder_StatusTypeDef encoder_update_speed(encoder_t * encoder);

#endif /* ENCODER_H */
