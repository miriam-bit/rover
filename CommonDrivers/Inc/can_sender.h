/**
 * @file can_sender.h
 * @brief Header for CAN sender module.
 *
 * This module provides a lightweight abstraction for sending CAN messages
 * using a FreeRTOS message queue and a CAN manager interface.
 *
 * @author Miriam Vitolo
 * @date June 18, 2025
 */

#ifndef INC_CAN_SENDER_H_
#define INC_CAN_SENDER_H_

#include "common_drivers.h"
#include "cmsis_os.h"
#include "canManager.h"

/**
 * @brief CAN Sender Status Type Definition.
 *
 * This type defines the return status used by CAN sender functions.
 * It is defined as an 8-bit unsigned integer.
 */
typedef uint8_t CANSender_StatusTypeDef;

/**
 * @brief Size in bytes of a CAN message payload.
 */
#define CAN_MESSAGE_DIM                         (8U)

#define CAN_MESSAGE_DIM (8U)

/**
 * @brief Operation completed successfully.
 */
#define CAN_SENDER_OK                           ((CANSender_StatusTypeDef) 0U)

/**
 * @brief Operation failed or an error occurred.
 */
#define CAN_SENDER_ERROR                        ((CANSender_StatusTypeDef) 1U)

/**
 * @brief Structure representing a CAN message.
 *
 * This structure contains the message payload and its associated CAN ID.
 */
typedef struct {
    uint8_t msg[CAN_MESSAGE_DIM];
    uint32_t id;
} can_msg_t;

/**
 * @brief Structure representing a CAN sender instance.
 *
 * This structure links a CAN manager and a FreeRTOS message queue
 * for managing asynchronous CAN transmissions.
 */
typedef struct {
	canManager_t *can_manager;
    osMessageQId xQueue;
    can_msg_t can_msg_buff;
} can_sender_t;


/**
 * @brief Initializes a CAN sender instance.
 *
 * Associates a CAN sender with a CAN manager and a FreeRTOS message queue.
 *
 * @param can_sender Pointer to the CAN sender structure to initialize.
 * @param can_manager Pointer to the CAN manager to be used for sending messages.
 * @param xQueue Handle to the FreeRTOS queue used for message buffering.
 * @return CAN_SENDER_OK if initialization is successful, otherwise CAN_SENDER_ERROR.
 */
CANSender_StatusTypeDef can_sender_init(can_sender_t *can_sender, canManager_t *can_manager, osMessageQId xQueue);

/**
 * @brief Enqueues a CAN message for transmission.
 *
 * Adds a CAN message to the internal message queue.
 *
 * @param can_sender Pointer to the CAN sender instance.
 * @param msg Pointer to the CAN message to enqueue.
 * @return CAN_SENDER_OK if the message is successfully enqueued, otherwise CAN_SENDER_ERROR.
 */
CANSender_StatusTypeDef can_sender_enqueue_msg(can_sender_t *can_sender, const can_msg_t *msg);


/**
 * @brief Dequeues and transmits the next CAN message if available.
 *
 * This function checks the internal message queue for pending messages.
 * If one is available and the CAN peripheral has a free mailbox, the message
 * is transmitted and removed from the queue.
 *
 * @param can_sender Pointer to the CAN sender instance.
 * @return CAN_SENDER_OK if a message is sent or if the queue is empty, otherwise CAN_SENDER_ERROR.
 */
CANSender_StatusTypeDef can_sender_dequeue_msg(can_sender_t *can_sender);


#endif /* INC_CAN_SENDER_H_ */
