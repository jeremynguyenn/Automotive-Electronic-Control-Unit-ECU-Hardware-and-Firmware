/*
 * communication_serial_protocol.h
 *
 * Created on: Aug 27, 2025
 */

#ifndef INC_COMMUNICATION_SERIAL_PROTOCOL_H_
#define INC_COMMUNICATION_SERIAL_PROTOCOL_H_

#include "stm32h7xx.h"  // For direct register access on STM32H7xx
#include "main.h"
#include "math.h"
#include "string.h"
#include "stdio.h"
#include <stdlib.h>

// Enum for protocol status
typedef enum {
	// Starting state
	FREE = 0,
	// Waiting state
	WAITING = 1,
	// Receiving state
	RX = 2,
	// Processing state
	PROCESSING = 3,
} PROTOCOLStatus;

// Send tick counter
uint16_t send_tick = 0;

// Receive buffer for protocol
uint8_t PROTOCOL_RX_Buffer[500];
// Stream data for protocol receive
uint8_t PROTOCOL_RX_Stream_Data = 0;
// Index for stream
int PROTOCOL_Stream_Index = 0;

// Last receive tick
uint16_t last_rx_tick = 0;

// Current protocol status
PROTOCOLStatus protocol_status = FREE;

// Timeout constant for protocol
const uint32_t PROTOCOL_TIMEOUT = 1 * 1000;
// Interval for send attempts in ms
const uint32_t SEND_ATTEMPT_INTERVAL_MS = 500;
// Interval for reading serial in ms
const uint32_t READ_SERIAL_INTERVAL_MS = 20;
// Interval for sending AT in ms
const uint32_t SEND_AT_INTERVAL_MS = 50;

// Volatile timestamp for last read serial
volatile uint32_t last_read_serial_timestamp = 0;
// Volatile timestamp for last send AT
volatile uint32_t last_send_at_timestamp = 0;

// Callback for protocol receive
void PROTOCOL_RX_Callback(void);
// Callback for protocol transmit
void PROTOCOL_TX_Callback(void);
// Function to reset buffers
void resetBuffers(void);
// Function to send command and get answer
uint8_t sendCommand(char command[], char answer[], uint32_t timeout);
// Function to transmit directly
void directTransmit(char *cmd);

#endif /* INC_COMMUNICATION_SERIAL_PROTOCOL_H_ */