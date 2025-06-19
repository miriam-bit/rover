/*
 * can_sender.h
 *
 *  Created on: Jun 18, 2025
 *      Author: Miriam Vitolo
 */

#ifndef INC_CAN_SENDER_H_
#define INC_CAN_SENDER_H_

#include "common_drivers.h"
#include "cmsis_os.h"
#include "canManager.h"

typedef uint8_t CANSender_StatusTypeDef;

#define CAN_MESSAGE_DIM                         (8U)

#define CAN_SENDER_OK                           ((CANSender_StatusTypeDef) 0U)

#define CAN_SENDER_ERROR                        ((CANSender_StatusTypeDef) 1U)

typedef struct {
    uint8_t msg[CAN_MESSAGE_DIM];
    uint32_t id;
} can_msg_t;

typedef struct {
	canManager_t *can_manager;
    osMessageQId xQueue;
    can_msg_t interr_buff;
} can_sender_t;

CANSender_StatusTypeDef can_sender_init(can_sender_t *can_sender, canManager_t *can_manager, osMessageQId xQueue);
CANSender_StatusTypeDef can_sender_enqueue_msg(can_sender_t *can_sender, const can_msg_t *interr_msg);
CANSender_StatusTypeDef can_sender_dequeue_msg(can_sender_t *can_sender);


#endif /* INC_CAN_SENDER_H_ */
