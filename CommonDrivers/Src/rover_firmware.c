/*
 * rover_firmware.c
 *
 *  Created on: Jun 11, 2025
 *      Author: Miriam Vitolo
 */
#include "cmsis_os.h"
#include "encoder.h"
#include "rover_firmware.h"
#include "test_comunication.h"
#include "can_sender.h"
#include "i2c.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

/**
 * @brief Internal singleton instance of the rover.
 *
 * This structure is initialized once and provides centralized access to all
 * hardware drivers and runtime state related to the rover system.
 *
 * - Includes configuration for 4 encoders, MPU sensor, CAN manager, and PWM timer.
 * - Used internally via rover_get_instance().
 */
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
    .motor_timer = &htim8,
	.mpu_config = {
	    .hi2c = &hi2c2
	},
	.mpu = {
	    .accel = {0},
	    .gyro = {0}
	},
	.can_config = {
		 .hcan = &hcan1,
		 .tx_header = {
		 .IDE = CAN_ID_STD,
		 .RTR = CAN_RTR_DATA,
		 .DLC = 8,
		 .TransmitGlobalTime = DISABLE
		  },
		  .rx_fifo = CAN_RX_FIFO0,
		  .rx_interrupt = CAN_IT_RX_FIFO0_MSG_PENDING
	},
	.can_manager = {
	.config = &rover.can_config
	},
	.canMsgQueueHandle = NULL,
	.canMsgQueueBuffer = {0},
	.canMsgQueueControlBlock = {{0}},
	.can_sender = {
		 .can_manager = &rover.can_manager,
		 .xQueue = NULL,
		 .can_msg_buff = {
		 .msg = {0},
		 .id = 0
		 }
	},
	.canMsgQueue_attributes = {
		.name = "canMsgQueue",
		.cb_mem = &rover.canMsgQueueControlBlock,
		.cb_size = sizeof(rover.canMsgQueueControlBlock),
		.mq_mem = &rover.canMsgQueueBuffer,
		.mq_size = sizeof(rover.canMsgQueueBuffer)
	}


};

/**
 * @brief Singleton instance of the rover structure.
 */
static rover_t * const instance = &rover;

rover_t* const rover_get_instance() {
    return instance;
}

Rover_StatusTypeDef rover_init(void){
	rover.canMsgQueueHandle =  osMessageQueueNew (CAN_QUEUE_SIZE, sizeof(can_msg_t), &rover.canMsgQueue_attributes);
	rover.can_sender.xQueue = rover.canMsgQueueHandle;
	Rover_StatusTypeDef status = ROVER_ERROR;
	if ((encoder_init(&rover.encoder1, &rover.encoder1_config)== ENCODER_OK) &&
			(encoder_init(&rover.encoder2, &rover.encoder2_config) == ENCODER_OK) &&
			(encoder_init(&rover.encoder3, &rover.encoder3_config) == ENCODER_OK) &&
			(encoder_init(&rover.encoder4, &rover.encoder4_config) == ENCODER_OK) &&
			(stop_all_motors() == MOTOR_OK )&&
			(Start_PWM_Channels() == HAL_OK ) &&
			//(MPU60X0_init(&rover.mpu, &rover.mpu_config, HAL_MAX_DELAY) == MPU60X0_OK) &&
			(canManager_Init(&rover.can_manager) == CAN_MANAGER_OK) &&
			(canManager_AddAllowedId(&rover.can_manager, TEST_ID) == CAN_MANAGER_OK) &&
			(canManager_AddAllowedId(&rover.can_manager, LIN_VEL_XY_FEEDBACK_MSG_ID) == CAN_MANAGER_OK) &&
			(canManager_AddAllowedId(&rover.can_manager, IMU_GYRO_XY_FEEDBACK_MSG_ID) == CAN_MANAGER_OK) &&
			(canManager_AddAllowedId(&rover.can_manager, IMU_ACCEL_XY_FEEDBACK_MSG_ID) == CAN_MANAGER_OK) &&
			(canManager_AddAllowedId(&rover.can_manager, IMU_Z_FEEDBACK_MSG_ID) == CAN_MANAGER_OK)) // &&
			//(can_sender_init(&rover.can_sender, &rover.can_manager, rover.canMsgQueueHandle)) == CAN_SENDER_OK)
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

Motor_StatusTypeDef motor_control_step(void){
	Motor_StatusTypeDef status = MOTOR_ERROR;

    if ((encoder_update_speed(&rover.encoder1) == ENCODER_OK) &&
        (encoder_update_speed(&rover.encoder2) == ENCODER_OK) &&
        (encoder_update_speed(&rover.encoder3) == ENCODER_OK) &&
        (encoder_update_speed(&rover.encoder4) == ENCODER_OK))
    {
        status = MOTOR_OK;
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


Rover_StatusTypeDef rover_get_linear_velocity_xy(double Ts){
	Rover_StatusTypeDef status = ROVER_ERROR;
	static double theta = 0.0;
	if(Ts >= 0.0){
		double rpm_fl = rover.encoder1.actual_speed_rpm;
		double rpm_rl = rover.encoder2.actual_speed_rpm;
		double rpm_fr = rover.encoder3.actual_speed_rpm;
		double rpm_rr = rover.encoder4.actual_speed_rpm;
		double vel_left  = ((rpm_fl + rpm_rl) / 2.0) * WHEEL_RADIUS_M * RPM_TO_RAD_PER_SEC;
		double vel_right =  ((rpm_fr + rpm_rr) / 2.0) * WHEEL_RADIUS_M * RPM_TO_RAD_PER_SEC;
		double v = (vel_right + vel_left) / 2.0;
		double omega = rover.mpu.gyro.z;
		theta += omega * Ts;
		rover.vx = (float)(v * cos(theta));
		rover.vy = (float)(v * sin(theta));

		status = ROVER_OK;
	}

	return status;
}

Rover_StatusTypeDef rover_enc_can_tx_step(void){
	Rover_StatusTypeDef status = ROVER_ERROR;
	LinVelFeedback_t vel_data;
	LinVelFeedback_init(&vel_data);
	uint8_t can_frame[LIN_VEL_FRAME_LENGTH_IN_BYTE];
	can_msg_t message;
	if (motor_control_step() == MOTOR_OK) {
		if (rover_get_linear_velocity_xy(ENCODER_SAMPLING_TIME) == ROVER_OK) {
			setLinVel(&vel_data, rover.vx, rover.vy);
			if (LinVelFeedback_createXYFrame(&vel_data, can_frame) == LIN_VEL_FEEDBACK_OK) {
				memcpy(message.msg, can_frame, LIN_VEL_FRAME_LENGTH_IN_BYTE);
				message.id = LIN_VEL_XY_FEEDBACK_MSG_ID;
				if(can_sender_enqueue_msg(&rover.can_sender, &message) == CAN_SENDER_OK){
					status = ROVER_OK;
				}
			}
		}
	}
	return status;
}

Rover_StatusTypeDef rover_imu_can_tx_step(void){
	Rover_StatusTypeDef status = ROVER_ERROR;
	can_msg_t gyro_msg, accel_msg, z_msg;
	uint8_t gyro_frame[IMU_FRAME_LENGTH_IN_BYTE];
	uint8_t accel_frame[IMU_FRAME_LENGTH_IN_BYTE];
	uint8_t z_frame[IMU_FRAME_LENGTH_IN_BYTE];


	if (
	    MPU60X0_get_gyro_value(&rover.mpu, &rover.mpu.gyro) == MPU60X0_OK &&
	    MPU60X0_get_accel_value(&rover.mpu, &rover.mpu.accel) == MPU60X0_OK &&
	    IMUFeedback_createGyroXYFrame(&rover.mpu.gyro, gyro_frame) == IMU_FEEDBACK_OK &&
	    IMUFeedback_createAccelXYFrame(&rover.mpu.accel, accel_frame) == IMU_FEEDBACK_OK &&
	    IMUFeedback_createZFrame(&rover.mpu.gyro, &rover.mpu.accel, z_frame) == IMU_FEEDBACK_OK
	) {
	    memcpy(gyro_msg.msg, gyro_frame, IMU_FRAME_LENGTH_IN_BYTE);
	    gyro_msg.id = IMU_GYRO_XY_FEEDBACK_MSG_ID;

	    memcpy(accel_msg.msg, accel_frame, IMU_FRAME_LENGTH_IN_BYTE);
	    accel_msg.id = IMU_ACCEL_XY_FEEDBACK_MSG_ID;

	    memcpy(z_msg.msg, z_frame, IMU_FRAME_LENGTH_IN_BYTE);
	    z_msg.id = IMU_Z_FEEDBACK_MSG_ID;

	    if (
	        can_sender_enqueue_msg(&rover.can_sender, &gyro_msg) == CAN_SENDER_OK &&
	        can_sender_enqueue_msg(&rover.can_sender, &accel_msg) == CAN_SENDER_OK &&
	        can_sender_enqueue_msg(&rover.can_sender, &z_msg) == CAN_SENDER_OK
	    ) {
	        status = ROVER_OK;
	    }
	}
	return status;

}

Rover_StatusTypeDef rover_can_tx_step(void){
	Rover_StatusTypeDef status = ROVER_ERROR;
	if(can_sender_dequeue_msg(&rover.can_sender) == CAN_SENDER_OK){
		status = ROVER_OK;
	}
	return status;
}
