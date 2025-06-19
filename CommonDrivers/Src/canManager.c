/*
 * canManager.c
 *
 *  Created on: Jul 13, 2024
 *      Author: emilioamato
 */

#include "canManager.h"

canManager_Status_TypeDef canManager_Init(canManager_t *canManager){

	canManager_Status_TypeDef status = CAN_MANAGER_ERR;

	if(canManager != NULL && canManager->config != NULL && canManager->config->hcan != NULL){

			if((HAL_CAN_Start(canManager->config->hcan) == HAL_OK) &&
			   (HAL_CAN_ActivateNotification(canManager->config->hcan, canManager->config->rx_interrupt) == HAL_OK)){
				canManager->allowed_ids_count = 0U;
				canManager->number_of_attempts = 0U;
				canManager->message_received = CAN_MANAGER_NO_NEW_MESSAGE;
				status =  CAN_MANAGER_OK;
			}
	}

	return status;

}


canManager_Status_TypeDef canManager_AddAllowedId(canManager_t *canManager, const uint32_t id){

	canManager_Status_TypeDef status = CAN_MANAGER_ERR;

	if(canManager != NULL && canManager->allowed_ids_count < CAN_MANAGER_MAX_ALLOWED_IDS){

		canManager->allowed_tx_ids[canManager->allowed_ids_count++] = id;
		status = CAN_MANAGER_OK;
	}

	return status;
}


static canManager_Status_TypeDef is_id_allowed(canManager_t *canManager, uint32_t id) {

	canManager_Status_TypeDef status = CAN_MANAGER_ERR;

    for (int i = 0; i < canManager->allowed_ids_count; i++) {
        if (canManager->allowed_tx_ids[i] == id) {
            status = CAN_MANAGER_OK;
        }
    }
    return status;
}

canManager_Status_TypeDef canManager_SendMessage(canManager_t *canManager, const uint32_t id, const uint8_t *can_data){

	canManager_Status_TypeDef status = CAN_MANAGER_ERR;

	if((canManager != NULL) && (can_data != NULL)){

		if(is_id_allowed(canManager, id) == CAN_MANAGER_OK){

			canManager->config->tx_header.StdId = id;
			canManager_config_t *config = (canManager_config_t*) canManager->config;

			if(HAL_CAN_GetTxMailboxesFreeLevel(config->hcan) > 0U){
				canManager->number_of_attempts = 0U;
				uint32_t * txMailboxPtr = (uint32_t *) &canManager->txMailbox;

				if(HAL_CAN_AddTxMessage(canManager->config->hcan, &(canManager->config->tx_header), can_data, txMailboxPtr) == HAL_OK){
					status = CAN_MANAGER_OK;
				}
			}
			else{
				canManager->number_of_attempts++;

				if(canManager->number_of_attempts < CAN_MANAGER_MAX_NUMBER_OF_ATTEMPTS){
					status = CAN_MANAGER_OK;
				}
			}
		}

	}


	return status;

}

