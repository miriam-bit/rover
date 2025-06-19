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


typedef uint8_t Rover_StatusTypeDef;
typedef uint8_t Motor_StatusTypeDef;
typedef StaticQueue_t osStaticMessageQDef_t;

#define MIN_PWM_TENSION_DESIRED						(1.875)
#define MAX_PWM_TENSION_DESIRED						(3.125)
#define SATURATION_RANGE							(24.0)
#define CONVERSION_FACTOR							((MAX_PWM_TENSION_DESIRED-MIN_PWM_TENSION_DESIRED)/SATURATION_RANGE)
#define STOP_PWM_TENSION    						(2.525)
#define MAX_PWM_TENSION								(3.3)
#define WHEEL_RADIUS_M 								(0.00)
#define RPM_TO_RAD_PER_SEC    						(2.0 * M_PI / 60.0)
#define CAN_TX_PERIOD_MS                          	(2U)

#define CAN_QUEUE_SIZE  10

#define ROVER_OK      ((Rover_StatusTypeDef)0U)
#define ROVER_ERROR   ((Rover_StatusTypeDef)1U)

#define MOTOR_OK 	  ((Motor_StatusTypeDef)0U)
#define MOTOR_ERROR   ((Motor_StatusTypeDef)1U)

typedef struct
{
    encoder_t encoder1;
    encoder_t encoder2;
    encoder_t encoder3;
    encoder_t encoder4;

    encoder_config_t encoder1_config;
    encoder_config_t encoder2_config;
    encoder_config_t encoder3_config;
    encoder_config_t encoder4_config;

    TIM_HandleTypeDef *motor_timer;

    MPU60X0_t mpu;
    MPU60X0_config_t mpu_config;

    canManager_t can_manager;
    canManager_config_t can_config;

    float vx;
    float vy;

    osMessageQueueId_t canMsgQueueHandle;
    uint8_t canMsgQueueBuffer[CAN_QUEUE_SIZE * sizeof(can_msg_t)];
    osStaticMessageQDef_t canMsgQueueControlBlock;
    can_sender_t can_sender;
    osMessageQueueAttr_t canMsgQueue_attributes;


} rover_t;

rover_t *rover_get_instance(void);
Rover_StatusTypeDef rover_init(void);
HAL_StatusTypeDef Start_PWM_Channels(void);
Motor_StatusTypeDef stop_all_motors(void);
Encoder_StatusTypeDef motor_control_step(void);
void drive_motor(TIM_HandleTypeDef* timer,HAL_TIM_ActiveChannel channel, double desiredValue);
Rover_StatusTypeDef rover_get_linear_velocity_xy(double Ts);
Rover_StatusTypeDef rover_enc_can_tx_step(void);
Rover_StatusTypeDef rover_imu_can_tx_step(void);
Rover_StatusTypeDef rover_can_tx_step(void);
#endif /* INC_ROVER_FIRMWARE_H_ */
