/*
 * can_sender.c
 *
 *  Created on: Jun 18, 2025
 *      Author: Miriam Vitolo
 */

#include "can_sender.h"
#include "queue.h"

CANSender_StatusTypeDef can_sender_init(can_sender_t *can_sender, canManager_t *can_manager, osMessageQId xQueue){
	 CANSender_StatusTypeDef status = CAN_SENDER_ERROR;
	 if(can_sender != NULL && can_manager != NULL && xQueue != NULL){
		 can_sender->can_manager = can_manager;
		 can_sender->xQueue = xQueue;
		 status = CAN_SENDER_OK;
	 }

	 return status;
}

CANSender_StatusTypeDef can_sender_enqueue_msg(can_sender_t *can_sender, const can_msg_t *msg){
	 CANSender_StatusTypeDef status = CAN_SENDER_ERROR;
	 if(can_sender != NULL){
		 if (xQueueSend(can_sender->xQueue, msg, 0U) == pdPASS) {
			 status = CAN_SENDER_OK;
		  }
	 }

	 return status;
}

CANSender_StatusTypeDef can_sender_dequeue_msg(can_sender_t *can_sender) {
    CANSender_StatusTypeDef status = CAN_SENDER_ERROR;
    if (can_sender != NULL) {
        if (uxQueueMessagesWaiting(can_sender->xQueue) > 0U) {
            if (HAL_CAN_GetTxMailboxesFreeLevel(can_sender->can_manager->config->hcan) > 0U) {
                can_msg_t *can_msg_buff = &can_sender->can_msg_buff;
                if (xQueuePeek(can_sender->xQueue, can_msg_buff, 0U) == pdPASS) {
                    if ((canManager_SendMessage(can_sender->can_manager, can_msg_buff->id, can_msg_buff->msg) == CAN_MANAGER_OK) &&
                    	(xQueueReceive(can_sender->xQueue, can_msg_buff, 0U) == pdPASS)){
                            status = CAN_SENDER_OK;
                    }
                }
            }
        } else {
            /* Queue empty: nothing to send, but it's not an error */
            status = CAN_SENDER_OK;
        }
    }

    return status;
}

