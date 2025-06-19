/*
 * canParser.c
 *
 *  Created on: Jul 15, 2024
 *      Author: emilioamato
 */


#include "canParser.h"

CanParser_Status_TypeDef CanParser_encode_can_frame(uint8_t *canFrame, uint32_t value, uint8_t startBit, uint8_t length, uint8_t endianess){

	CanParser_Status_TypeDef status = CAN_PARSER_STATUS_ERR;

	if((canFrame != NULL) && (length > 0)){

		uint32_t bit;
		uint8_t byteIndex;
		uint8_t bitIndex;


		if(endianess == LITTLE_ENDIAN){

			for (int i = 0; i < length; i++) {
					bit = (value >> i) & CAN_PARSER_MASK_FOR_PARSING;
					byteIndex = (startBit + i) / CAN_PARSER_BYTE_LENGTH_IN_BITS;
					bitIndex = (startBit + i) % CAN_PARSER_BYTE_LENGTH_IN_BITS;
					canFrame[byteIndex] |= bit << bitIndex;
			}
		}
		else{

            for (int i = 0; i < length; i++) {
                bit = (value >> (length - 1 - i)) & CAN_PARSER_MASK_FOR_PARSING;
                byteIndex = (startBit + i) / CAN_PARSER_BYTE_LENGTH_IN_BITS;
                bitIndex = CAN_PARSER_BYTE_LENGTH_IN_BITS - 1 - ((startBit + i) % CAN_PARSER_BYTE_LENGTH_IN_BITS);
                canFrame[byteIndex] |= bit << bitIndex;
            }
		}

		status = CAN_PARSER_STATUS_OK;

	}

	return status;

}

CanParser_Status_TypeDef CanParser_decode_can_frame(const uint8_t *canFrame, void *result, uint8_t startBit, uint8_t length, uint8_t endianess, uint8_t resultSize){

	CanParser_Status_TypeDef status = CAN_PARSER_STATUS_ERR;

	if(canFrame != NULL && result != NULL  && length > 0){

		uint8_t byteIndex;
		uint8_t bitIndex ;
		uint32_t bit;
		uint32_t tempResult = 0;

		if(endianess == LITTLE_ENDIAN){

			for (int i = 0; i < length; i++) {
				byteIndex = (startBit + i) / CAN_PARSER_BYTE_LENGTH_IN_BITS;
				bitIndex = (startBit + i) % CAN_PARSER_BYTE_LENGTH_IN_BITS;
				bit = (canFrame[byteIndex] >> bitIndex) & CAN_PARSER_MASK_FOR_PARSING;
				tempResult |= bit << i;
			}
		}
		else{

			for (int i = 0; i < length; i++) {
				byteIndex = (startBit + i) / CAN_PARSER_BYTE_LENGTH_IN_BITS;
				bitIndex = CAN_PARSER_BYTE_LENGTH_IN_BITS - 1 - ((startBit + i) % CAN_PARSER_BYTE_LENGTH_IN_BITS);
				bit = (canFrame[byteIndex] >> bitIndex) & CAN_PARSER_MASK_FOR_PARSING;
				tempResult |= bit << i;
			}
		}

		switch(resultSize) {
			case sizeof(uint8_t):
				*((uint8_t *)result) = (uint8_t)tempResult;
				status = CAN_PARSER_STATUS_OK;
				break;
			case sizeof(uint16_t):
				*((uint16_t *)result) = (uint16_t)tempResult;
				status = CAN_PARSER_STATUS_OK;
				break;
			case sizeof(uint32_t):
				*((uint32_t *)result) = (uint32_t)tempResult;
				status = CAN_PARSER_STATUS_OK;
				break;
			default:
				status = CAN_PARSER_STATUS_ERR;
		}

	}


	return status;

}
