/*
 * rover_firmware.h
 *
 *  Created on: Jun 11, 2025
 *      Author: Miriam Vitolo
 */

#ifndef INC_ROVER_FIRMWARE_H_
#define INC_ROVER_FIRMWARE_H_

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "tim.h"
#include "encoder.h"
#include "mpu60x0.h"
#include "math.h"
#include "canManager.h"
#include "can_sender.h"

/* ============================ */
/* Feature Configuration Flags */
/* ============================ */

/**
 * @brief Enable IMU data acquisition and related CAN transmission.
 *
 * Define this macro to enable the update and transmission of IMU data (MPU60X0).
 * If undefined, IMU readings and their related CAN operations will be skipped.
 */
//#define USE_MPU

/**
 * @brief Type definition for rover status codes.
 *
 * This type is used to represent the status returned by rover functions.
 * It is defined as an 8-bit unsigned integer.
 */
typedef uint8_t Rover_StatusTypeDef;

/**
 * @brief Type definition for motor status codes.
 *
 * This type is used to represent the status of motor operations.
 * It is defined as an 8-bit unsigned integer.
 */
typedef uint8_t Motor_StatusTypeDef;

/**
 * @brief Alias for FreeRTOS static queue definition.
 *
 * This typedef provides a more intuitive naming for the static queue structure.
 */
typedef StaticQueue_t osStaticMessageQDef_t;

/**
 * @brief Minimum PWM tension for desired output (in volts).
 */
#define MIN_PWM_TENSION_DESIRED						(1.875)

/**
 * @brief Maximum PWM tension for desired output (in volts).
 */
#define MAX_PWM_TENSION_DESIRED						(3.125)

/**
 * @brief Saturation range for input-to-voltage conversion.
 */
#define SATURATION_RANGE							(24.0)

/**
 * @brief Conversion factor from physical command to PWM voltage.
 */
#define CONVERSION_FACTOR							((MAX_PWM_TENSION_DESIRED-MIN_PWM_TENSION_DESIRED)/SATURATION_RANGE)

/**
 * @brief PWM tension corresponding to a stopped motor (in volts).
 */
#define STOP_PWM_TENSION    						(2.525)

/**
 * @brief Maximum allowed PWM tension.
 */
#define MAX_PWM_TENSION								(3.3)

/**
 * @brief Wheel radius in meters.
 */
#define WHEEL_RADIUS_M 								(0.00)


/**
 * @brief Conversion factor from RPM to radians per second.
 */
#define RPM_TO_RAD_PER_SEC    						(2.0 * M_PI / 60.0)

/**
 * @brief Period of CAN message transmission in milliseconds.
 */
#define CAN_TX_PERIOD_MS                          	(2U)

/**
 * @brief Size of the CAN message queue.
 */
#define CAN_QUEUE_SIZE  16


/**
 * @brief Operation completed successfully.
 */
#define ROVER_OK      ((Rover_StatusTypeDef)0U)

/**
 * @brief Generic error status for rover operations.
 */
#define ROVER_ERROR   ((Rover_StatusTypeDef)1U)

/**
 * @brief Operation completed successfully for motor functions.
 */
#define MOTOR_OK 	  ((Motor_StatusTypeDef)0U)

/**
 * @brief Generic error status for motor operations.
 */
#define MOTOR_ERROR   ((Motor_StatusTypeDef)1U)

/**
 * @brief Structure representing the full state and configuration of the rover.
 *
 * This structure aggregates all hardware components (encoders, IMU, CAN bus, motor timers),
 * velocity state, and FreeRTOS message queues related to the rover's firmware.
 */
typedef struct
{
    encoder_t encoder1;	/**< Encoder for front-left wheel */
    encoder_t encoder2;	/**< Encoder for rear-left wheel */
    encoder_t encoder3;	/**< Encoder for front-right wheel */
    encoder_t encoder4;	/**< Encoder for rear-right wheel */

    encoder_config_t encoder1_config;	/**< Configuration for encoder 1 */
    encoder_config_t encoder2_config;	/**< Configuration for encoder 2 */
    encoder_config_t encoder3_config;	/**< Configuration for encoder 3 */
    encoder_config_t encoder4_config;	/**< Configuration for encoder 4 */

    TIM_HandleTypeDef *motor_timer;	/**< Timer used for motor PWM control */

    MPU60X0_t mpu;	/**< IMU sensor structure */
    MPU60X0_config_t mpu_config;	/**< IMU sensor configuration */

    canManager_t can_manager;	/**< CAN manager structure */
    canManager_config_t can_config;	/**< CAN manager configuration */

    float vx;	/**< Linear velocity in X direction (m/s) */
    float vy;	/**< Linear velocity in Y direction (m/s) */

    osMessageQueueId_t canMsgQueueHandle;	/**< Message queue handle for CAN messages */
    uint8_t canMsgQueueBuffer[CAN_QUEUE_SIZE * sizeof(can_msg_t)];	/**< Buffer for CAN message queue */
    osStaticMessageQDef_t canMsgQueueControlBlock;	/**< Control block for the static message queue */
    can_sender_t can_sender;	/**< CAN sender utility */
    osMessageQueueAttr_t canMsgQueue_attributes;	/**< Attributes of the CAN message queue */


} rover_t;

/**
 * @brief Returns the singleton instance of the rover structure.
 *
 * @return rover_t* Constant pointer to the rover instance.
 */
rover_t* const rover_get_instance(void);

/**
 * @brief Initializes the rover components (encoders, IMU, CAN, etc.).
 *
 * @return ROVER_OK if initialization is successful, otherwise ROVER_ERROR.
 */
Rover_StatusTypeDef rover_init(void);

/**
 * @brief Starts all PWM channels used for motor control.
 *
 * @return HAL_OK if all PWM channels start successfully, otherwise HAL_ERROR.
 */
HAL_StatusTypeDef Start_PWM_Channels(void);


/**
 * @brief Stops all motors by setting the control value to zero.
 *
 * @return MOTOR_OK if the operation is successful, otherwise MOTOR_ERROR.
 */
Motor_StatusTypeDef stop_all_motors(void);

/**
 * @brief Updates all motor encoder speeds.
 *
 * This function reads the actual speed in RPM from each of the four motor encoders
 * and updates their internal values.
 *
 * @return MOTOR_OK if all encoders update successfully, otherwise MOTOR_ERROR.
 */
Encoder_StatusTypeDef motor_control_step(void);

/**
 * @brief Sets the PWM duty cycle for a motor based on a physical control input.
 *
 * This function computes the appropriate compare value for a timer PWM channel
 * to represent a desired physical input as a voltage signal.
 * The mapping is defined by the following linear conversion:
 *
 * @code
 *     compare_value = ((CONVERSION_FACTOR * desiredValue + STOP_PWM_TENSION) * timer->Init.Period) / MAX_PWM_TENSION;
 * @endcode
 *
 * Where:
 * - `desiredValue` is the command in physical units.
 * - `CONVERSION_FACTOR` maps the physical input to a voltage range.
 * - `STOP_PWM_TENSION` defines the voltage corresponding to a stop state.
 * - `MAX_PWM_TENSION` represents the maximum output voltage.
 * - `timer->Init.Period` is the PWM resolution of the hardware timer.
 *
 * The resulting compare value is set into the timer’s compare register to generate the PWM signal.
 *
 * @param timer Pointer to the TIM handle configured for PWM.
 * @param channel Timer PWM channel to be updated.
 * @param desiredValue Desired control value to apply to the motor.
 *
 */
void drive_motor(TIM_HandleTypeDef* timer,HAL_TIM_ActiveChannel channel, double desiredValue);

/**
 * @brief Computes the rover's linear velocity components (vx, vy) in global coordinates.
 *
 * This function:
 * - Reads RPM values from all four wheel encoders.
 * - Converts them to linear velocities using the wheel radius.
 * - Computes average translational velocity and angular velocity (omega).
 * - Integrates orientation (theta) over time.
 * - Projects the velocity onto the global frame using cosine and sine of theta.
 *
 * @param Ts Sampling period in seconds (must be non-negative).
 * @return ROVER_OK if computation is successful, otherwise ROVER_ERROR.
 *
 */
Rover_StatusTypeDef rover_get_linear_velocity_xy(double Ts);

/**
 * @brief Sends encoder-based velocity data over CAN bus.
 *
 * Updates motor encoder speeds, calculates (vx, vy), formats a CAN message,
 * and enqueues it for transmission.
 *
 * @return ROVER_OK if the message is successfully queued for transmission, otherwise ROVER_ERROR.
 */
Rover_StatusTypeDef rover_enc_can_tx_step(void);

/**
 * @brief Sends IMU sensor data over CAN bus.
 *
 * Reads current gyroscope and accelerometer values, formats them into three separate
 * CAN frames (gyro XY, accel XY, and Z data), and queues them for transmission.
 *
 * @return ROVER_OK if all messages are successfully queued, otherwise ROVER_ERROR.
 */
Rover_StatusTypeDef rover_imu_can_tx_step(void);


/**
 * @brief Sends a CAN message from the rover's transmission queue.
 *
 * Dequeues one message from the CAN sender's internal buffer and sends it over the CAN bus.
 *
 * @return ROVER_OK if a message was transmitted successfully, otherwise ROVER_ERROR.
 */
Rover_StatusTypeDef rover_can_tx_step(void);

#endif /* INC_ROVER_FIRMWARE_H_ */
