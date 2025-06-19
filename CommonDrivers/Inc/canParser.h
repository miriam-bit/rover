/*
 * canParser.h
 *
 *  Created on: Jul 15, 2024
 *      Author: emilioamato
 */

#ifndef INC_CANPARSER_H_
#define INC_CANPARSER_H_

#include <stdint.h>
#include <stdlib.h>


typedef uint8_t CanParser_Status_TypeDef;

#define CAN_PARSER_STATUS_OK ((CanParser_Status_TypeDef) 0U)
#define CAN_PARSER_STATUS_ERR ((CanParser_Status_TypeDef) 1U)

#define CAN_PARSER_MASK_FOR_PARSING (0x00000001)
#define CAN_PARSER_BYTE_LENGTH_IN_BITS (8U)

typedef enum {

	LITTLE_ENDIAN = 1,
	BIG_ENDIAN = 0

} Endianess;

CanParser_Status_TypeDef CanParser_encode_can_frame(uint8_t *canFrame, uint32_t value, uint8_t startBit, uint8_t length, uint8_t endianess);

CanParser_Status_TypeDef CanParser_decode_can_frame(const uint8_t *canFrame, void *result, uint8_t startBit, uint8_t length, uint8_t endianess, uint8_t resultSize);

#endif /* INC_CANPARSER_H_ */

