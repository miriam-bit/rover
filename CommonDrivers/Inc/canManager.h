/*
 * canManager.h
 *
 *  Created on: Jul 13, 2024
 *      Author: emilioamato
 */

#ifndef INC_CANMANAGER_H_
#define INC_CANMANAGER_H_

#include <stdint.h>
#include "can.h"

typedef uint8_t canManager_Status_TypeDef;

#define CAN_MANAGER_TX_DATA_SIZE (8U)

#define CAN_MANAGER_RX_DATA_SIZE (8U) // To discuss (at the moment we need only 1 byte)

#define CAN_MANAGER_MAX_ALLOWED_IDS (5U) // Define the maximum number of allowed IDs

#define CAN_MANAGER_MAX_NUMBER_OF_ATTEMPTS (50U) // We sent info every 40 ms, so after 2 second that is not possibile to send new can frame we can consider can not available

#define CAN_MANAGER_RECEIVED_NEW_MESSAGE (1U)

#define CAN_MANAGER_NO_NEW_MESSAGE (0U)

typedef struct {

	CAN_HandleTypeDef *hcan;
	CAN_TxHeaderTypeDef tx_header;
	CAN_RxHeaderTypeDef rx_header;
	uint32_t rx_fifo;
	uint32_t rx_interrupt;

} canManager_config_t;

typedef struct {

	canManager_config_t* const config;
	uint32_t txMailbox;
	uint8_t tx_data[CAN_MANAGER_TX_DATA_SIZE];
	uint8_t rx_data[CAN_MANAGER_RX_DATA_SIZE];
	uint8_t number_of_attempts;
	uint32_t allowed_tx_ids[CAN_MANAGER_MAX_ALLOWED_IDS];
	uint8_t allowed_ids_count;
	uint8_t message_received;

} canManager_t;


#define CAN_MANAGER_OK		((canManager_Status_TypeDef) 0U)

#define CAN_MANAGER_ERR		((canManager_Status_TypeDef) 1U)


canManager_Status_TypeDef canManager_Init(canManager_t *canManager);

canManager_Status_TypeDef canManager_SendMessage(canManager_t *canManager, const uint32_t id, const uint8_t *can_data);

canManager_Status_TypeDef canManager_AddAllowedId(canManager_t *canManager, const uint32_t id);


#endif /* INC_CANMANAGER_H_ */
