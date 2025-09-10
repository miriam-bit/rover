/**
 * @file test_comunication.h
 * @brief Module for CAN feedback message construction (IMU and linear velocity).
 *
 * This module provides structures, constants, and helper functions to encode
 * various feedback messages (IMU data, and linear velocity) into CAN frames.
 *
 * @author Miriam Vitolo
 * @date June 13, 2025
 */
#ifndef INC_TEST_COMUNICATION_H_
#define INC_TEST_COMUNICATION_H_

#include <stdint.h>
#include <stdlib.h>
#include "geometry.h"

/**
 * @brief Status type for IMU feedback operations.
 */
typedef uint8_t IMUFeedback_Status_TypeDef;


/**
 * @brief Status type for linear velocity feedback operations.
 */
typedef uint8_t LinVelFeedback_Status_TypeDef;

#define IMU_FEEDBACK_OK ((IMUFeedback_Status_TypeDef) 0U)	 /**< Operation succeeded */
#define IMU_FEEDBACK_ERR ((IMUFeedback_Status_TypeDef) 1U)	/**< Operation failed */

#define LIN_VEL_FEEDBACK_OK ((LinVelFeedback_Status_TypeDef) 0U)	 /**< Operation succeeded */
#define LIN_VEL_FEEDBACK_ERR ((LinVelFeedback_Status_TypeDef) 1U)	/**< Operation failed */

#define RPM_REFERENCE_MSG_ID 		 (0x000A)	/**< CAN ID for RPM reference */
#define IMU_GYRO_XY_FEEDBACK_MSG_ID  (0x000B)	/**< CAN ID for gyroscope XY feedback */
#define IMU_ACCEL_XY_FEEDBACK_MSG_ID (0X000C)	/**< CAN ID for accelerometer XY feedback */
#define IMU_Z_FEEDBACK_MSG_ID        (0X000D)	/**< CAN ID for Z-axis feedback (gyro and accel) */
#define LIN_VEL_XY_FEEDBACK_MSG_ID   (0X000E)	/**< CAN ID for linear velocity XY feedback */

#define LIN_VEL_X_START_BIT      (0U)
#define LIN_VEL_X_LENGTH         (32U)
#define LIN_VEL_X_ENDIANNESS     (1U)

#define LIN_VEL_Y_START_BIT      (32U)
#define LIN_VEL_Y_LENGTH         (32U)
#define LIN_VEL_Y_ENDIANNESS     (1U)

#define GYRO_X_START_BIT         (0U)
#define GYRO_X_LENGTH            (32U)
#define GYRO_X_ENDIANNESS        (1U)

#define GYRO_Y_START_BIT         (32U)
#define GYRO_Y_LENGTH            (32U)
#define GYRO_Y_ENDIANNESS        (1U)

#define ACCEL_X_START_BIT        (0U)
#define ACCEL_X_LENGTH           (32U)
#define ACCEL_X_ENDIANNESS       (1U)

#define ACCEL_Y_START_BIT        (32U)
#define ACCEL_Y_LENGTH           (32U)
#define ACCEL_Y_ENDIANNESS       (1U)

#define GYRO_Z_START_BIT         (0U)
#define GYRO_Z_LENGTH            (32U)
#define GYRO_Z_ENDIANNESS        (1U)

#define ACCEL_Z_START_BIT        (32U)
#define ACCEL_Z_LENGTH           (32U)
#define ACCEL_Z_ENDIANNESS       (1U)

#define IMU_FRAME_LENGTH_IN_BYTE     (8U)	/**< Size of IMU feedback frame in bytes */
#define LIN_VEL_FRAME_LENGTH_IN_BYTE (8U)	/**< Size of linear velocity frame in bytes */
#define RPM_REFERENCE_FRAME_LENGTH   (8U)	/**< Size of RPM reference frame in bytes */

/**
 * @brief Structure representing linear velocity feedback.
 */
typedef struct {
    float x;	/**< Linear velocity in X direction */
    float y;	/**< Linear velocity in Y direction */
} LinVelFeedback_t;


/**
 * @brief Initializes a LinVelFeedback structure.
 *
 * @param vel_data Pointer to the LinVelFeedback structure.
 * @return LIN_VEL_FEEDBACK_OK if successful, otherwise LIN_VEL_FEEDBACK_ERR.
 */
LinVelFeedback_Status_TypeDef LinVelFeedback_init(LinVelFeedback_t *vel_data);


/**
 * @brief Sets the x and y linear velocity in a LinVelFeedback structure.
 *
 * @param vel_data Pointer to the LinVelFeedback structure.
 * @param x Velocity in X direction.
 * @param y Velocity in Y direction.
 * @return LIN_VEL_FEEDBACK_OK if successful, otherwise LIN_VEL_FEEDBACK_ERR.
 */
LinVelFeedback_Status_TypeDef setLinVel(LinVelFeedback_t *vel_data, float x, float y);


/**
 * @brief Creates a CAN frame from a LinVelFeedback structure.
 *
 * @param vel_data Pointer to the LinVelFeedback structure.
 * @param can_data Pointer to the output CAN frame buffer.
 * @return LIN_VEL_FEEDBACK_OK if frame is successfully created, otherwise LIN_VEL_FEEDBACK_ERR.
 */
LinVelFeedback_Status_TypeDef LinVelFeedback_createXYFrame(LinVelFeedback_t *vel_data, uint8_t* can_data);


/**
 * @brief Creates a CAN frame from gyro X and Y data.
 *
 * @param gyro_data Pointer to the gyro Cartesian3D structure.
 * @param can_data Pointer to the output CAN frame buffer.
 * @return IMU_FEEDBACK_OK if frame is successfully created, otherwise IMU_FEEDBACK_ERR.
 */
IMUFeedback_Status_TypeDef IMUFeedback_createGyroXYFrame(Cartesian3D* gyro_data, uint8_t* can_data);


/**
 * @brief Creates a CAN frame from accelerometer X and Y data.
 *
 * @param accel_data Pointer to the accelerometer Cartesian3D structure.
 * @param can_data Pointer to the output CAN frame buffer.
 * @return IMU_FEEDBACK_OK if frame is successfully created, otherwise IMU_FEEDBACK_ERR.
 */
IMUFeedback_Status_TypeDef IMUFeedback_createAccelXYFrame(Cartesian3D* accel_data, uint8_t* can_data);

/**
 * @brief Creates a CAN frame from gyro Z and accelerometer Z data.
 *
 * @param gyro_data Pointer to the gyro Cartesian3D structure.
 * @param accel_data Pointer to the accelerometer Cartesian3D structure.
 * @param can_data Pointer to the output CAN frame buffer.
 * @return IMU_FEEDBACK_OK if frame is successfully created, otherwise IMU_FEEDBACK_ERR.
 */
IMUFeedback_Status_TypeDef IMUFeedback_createZFrame(Cartesian3D* gyro_data, Cartesian3D* accel_data, uint8_t* can_data);

#endif /* INC_TEST_COMUNICATION_H_ */
