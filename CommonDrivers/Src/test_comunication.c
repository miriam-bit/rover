/*
 * test_comunication.c
 *
 *  Created on: Jun 13, 2025
 *      Author: Miriam Vitolo
 */

#include "test_comunication.h"
#include "canParser.h"
#include "string.h"


LinVelFeedback_Status_TypeDef LinVelFeedback_init(LinVelFeedback_t *vel_data){
	LinVelFeedback_Status_TypeDef status = LIN_VEL_FEEDBACK_ERR;
	if(vel_data != NULL){
		vel_data->x = 0U;
		vel_data->y = 0U;

		status = LIN_VEL_FEEDBACK_OK;
	}

	return status;
}


LinVelFeedback_Status_TypeDef setLinVel(LinVelFeedback_t *vel_data, float x, float y){
	LinVelFeedback_Status_TypeDef status = LIN_VEL_FEEDBACK_ERR;
	if(vel_data != NULL){
		vel_data->x = x;
		vel_data->y = y;

		status = LIN_VEL_FEEDBACK_OK;
	}

	return status;
}


LinVelFeedback_Status_TypeDef LinVelFeedback_createXYFrame(LinVelFeedback_t *vel_data, uint8_t* can_data){
	LinVelFeedback_Status_TypeDef status = LIN_VEL_FEEDBACK_ERR;
	if(vel_data != NULL && can_data != NULL){
		memset(can_data, 0, LIN_VEL_FRAME_LENGTH_IN_BYTE);
		uint32_t x_bits, y_bits;
		memcpy(&x_bits, &vel_data->x, sizeof(float));
		memcpy(&y_bits, &vel_data->y, sizeof(float));
		if ((CanParser_encode_can_frame(can_data, x_bits, LIN_VEL_X_START_BIT, LIN_VEL_X_LENGTH , LIN_VEL_X_ENDIANNESS) != CAN_PARSER_STATUS_ERR)&&
			(CanParser_encode_can_frame(can_data, y_bits, LIN_VEL_Y_START_BIT, LIN_VEL_Y_LENGTH, LIN_VEL_Y_ENDIANNESS) != CAN_PARSER_STATUS_ERR)){
			status = LIN_VEL_FEEDBACK_OK;
		}

	}

	return status;
}


IMUFeedback_Status_TypeDef IMUFeedback_createGyroXYFrame(Cartesian3D* gyro_data, uint8_t* can_data){
	IMUFeedback_Status_TypeDef status = IMU_FEEDBACK_ERR;
		if(gyro_data != NULL && can_data != NULL){
			memset(can_data, 0, IMU_FRAME_LENGTH_IN_BYTE);
			uint32_t x_bits, y_bits;
			memcpy(&x_bits, &gyro_data->x, sizeof(float));
			memcpy(&y_bits, &gyro_data->y, sizeof(float));
			if ((CanParser_encode_can_frame(can_data, x_bits, GYRO_X_START_BIT, GYRO_X_LENGTH, GYRO_X_ENDIANNESS) != CAN_PARSER_STATUS_ERR)&&
				(CanParser_encode_can_frame(can_data, y_bits, GYRO_Y_START_BIT, GYRO_Y_LENGTH, GYRO_Y_ENDIANNESS) != CAN_PARSER_STATUS_ERR)){

				status = IMU_FEEDBACK_OK;
			}

		}

		return status;

}


IMUFeedback_Status_TypeDef IMUFeedback_createAccelXYFrame(Cartesian3D* accel_data, uint8_t* can_data){
	IMUFeedback_Status_TypeDef status = IMU_FEEDBACK_ERR;
		if(accel_data != NULL && can_data != NULL){
			memset(can_data, 0, IMU_FRAME_LENGTH_IN_BYTE);
			uint32_t x_bits, y_bits;
			memcpy(&x_bits, &accel_data->x, sizeof(float));
			memcpy(&y_bits, &accel_data->y, sizeof(float));
			if ((CanParser_encode_can_frame(can_data, x_bits, ACCEL_X_START_BIT, ACCEL_X_LENGTH, ACCEL_X_ENDIANNESS) != CAN_PARSER_STATUS_ERR)&&
				(CanParser_encode_can_frame(can_data, y_bits, ACCEL_Y_START_BIT, ACCEL_Y_LENGTH, ACCEL_Y_ENDIANNESS) != CAN_PARSER_STATUS_ERR)){

				status = IMU_FEEDBACK_OK;
			}
		}

		return status;

}


IMUFeedback_Status_TypeDef IMUFeedback_createZFrame(Cartesian3D* gyro_data, Cartesian3D* accel_data, uint8_t* can_data){
	IMUFeedback_Status_TypeDef status = IMU_FEEDBACK_ERR;
		if(gyro_data != NULL && accel_data != NULL && can_data != NULL){
			 memset(can_data, 0, IMU_FRAME_LENGTH_IN_BYTE);
			 uint32_t gz_bits, az_bits;
			 memcpy(&gz_bits, &gyro_data->z, sizeof(float));
			 memcpy(&az_bits, &accel_data->z, sizeof(float));
			 if ((CanParser_encode_can_frame(can_data, gz_bits, GYRO_Z_START_BIT, GYRO_Z_LENGTH, GYRO_Z_ENDIANNESS) != CAN_PARSER_STATUS_ERR)&&
			 	 (CanParser_encode_can_frame(can_data, az_bits, ACCEL_Z_START_BIT, ACCEL_Z_LENGTH, ACCEL_Z_ENDIANNESS) != CAN_PARSER_STATUS_ERR)){

			 	 status = IMU_FEEDBACK_OK;
			 }
		}

		return status;

}
