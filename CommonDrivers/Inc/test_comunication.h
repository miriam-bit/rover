/*
 * test_comunication.h
 *
 *  Created on: Jun 13, 2025
 *      Author: Miriam Vitolo
 */

#ifndef INC_TEST_COMUNICATION_H_
#define INC_TEST_COMUNICATION_H_

#include <stdint.h>
#include <stdlib.h>
#include "geometry.h"

typedef uint8_t TestFeedback_Status_TypeDef;
typedef uint8_t IMUFeedback_Status_TypeDef;
typedef uint8_t LinVelFeedback_Status_TypeDef;

#define TEST_FEEDBACK_OK ((TestFeedback_Status_TypeDef) 0U)
#define TEST_FEEDBACK_ERR ((TestFeedback_Status_TypeDef) 1U)

#define IMU_FEEDBACK_OK ((IMUFeedback_Status_TypeDef) 0U)
#define IMU_FEEDBACK_ERR ((IMUFeedback_Status_TypeDef) 1U)

#define LIN_VEL_FEEDBACK_OK ((LinVelFeedback_Status_TypeDef) 0U)
#define LIN_VEL_FEEDBACK_ERR ((LinVelFeedback_Status_TypeDef) 1U)

#define TEST_ID						 (0X9)
#define RPM_REFERENCE_MSG_ID 		 (0x000A)
#define IMU_GYRO_XY_FEEDBACK_MSG_ID  (0x000B)
#define IMU_ACCEL_XY_FEEDBACK_MSG_ID (0X000C)
#define IMU_Z_FEEDBACK_MSG_ID        (0X000D)
#define LIN_VEL_XY_FEEDBACK_MSG_ID   (0X000E)

#define TEST_START_BIT           (0U)
#define TEST_LENGTH              (32U)
#define TEST_ENDIANNESS          (1U)

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


#define TEST_LENGHT_IN_BYTE 	     (8U)
#define IMU_FRAME_LENGTH_IN_BYTE     (8U)
#define LIN_VEL_FRAME_LENGTH_IN_BYTE (8U)
#define RPM_REFERENCE_FRAME_LENGTH   (4U)

typedef struct {
    uint32_t value;
} TestFeedback_t;

/*
typedef struct {
    float x;
    float y;
    float z;
} IMUFeedback_t;
*/

typedef struct {
    float x;
    float y;
} LinVelFeedback_t;

TestFeedback_Status_TypeDef TestFeedback_init(TestFeedback_t *test_data);
TestFeedback_Status_TypeDef setTestValue(TestFeedback_t *test_data, uint32_t value);
TestFeedback_Status_TypeDef TestFeedback_createFrame(TestFeedback_t *test_data, uint8_t *can_data);

LinVelFeedback_Status_TypeDef LinVelFeedback_init(LinVelFeedback_t *vel_data);
LinVelFeedback_Status_TypeDef setLinVel(LinVelFeedback_t *vel_data, float x, float y);
LinVelFeedback_Status_TypeDef LinVelFeedback_createXYFrame(LinVelFeedback_t *vel_data, uint8_t* can_data);


/*IMUFeedback_Status_TypeDef IMUFeedback_init(Cartesian3D *imu_data);
IMUFeedback_Status_TypeDef setIMU(Cartesian3D* imu_data, float x, float y, float z);
*/
IMUFeedback_Status_TypeDef IMUFeedback_createGyroXYFrame(Cartesian3D* gyro_data, uint8_t* can_data);
IMUFeedback_Status_TypeDef IMUFeedback_createAccelXYFrame(Cartesian3D* accel_data, uint8_t* can_data);
IMUFeedback_Status_TypeDef IMUFeedback_createZFrame(Cartesian3D* gyro_data, Cartesian3D* accel_data, uint8_t* can_data);

#endif /* INC_TEST_COMUNICATION_H_ */
