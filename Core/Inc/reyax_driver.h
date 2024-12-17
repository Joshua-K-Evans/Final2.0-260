/*
 * reyax_driver.c
 *
 *  Created on: Nov 14, 2024
 *      Author: lynnrwatson
 */

#ifndef INC_REYAX_DRIVER_C_
#define INC_REYAX_DRIVER_C_





// Incoming messages are going to be one of the following types:
// See the documentation at:  Documents/LoRa-AT-Command-RYLR40x_RYLR89x_EN-8.pdf
typedef enum {
    OK_MSG,         // +OK
    RCV_MSG,        // +RCV
    ERROR_MSG,      // +ERROR
    RESET_MSG,      // +RESET
    READY_MSG,      // +READY
    ADDRESS_MSG,    // +ADDRESS
    NETWORK_MSG,    // +NETWORK
    SEND_MSG,       // +SEND
    FACTORY_MSG,    // +FACTORY
    UNKNOWN_MSG     // Unknown or unsupported type
} messageType;

// Now a Queue in which each element is a structure that has the messageType and the messageData
#define osWaitForever 0xFFFFFFFFU

#define MAX_MESSAGE_LENGTH 100 // Maximum length of the message string

#define NETWORK_ID 13
#define ALPHA_ADDRESS 1
#define BETA_ADDRESS 2


// Define the structure for queue elements
typedef struct {
    messageType theType;                // The message type
    char theBuffer[MAX_MESSAGE_LENGTH];              //  pointer to the message buffer
} MessageElement;

// for the demo, there are two connected to the same STM32
typedef enum {
    alpha,
    beta
} reyax_connection;



//  Prototype declaration
messageType parseMessageType(const char *response);
void initialize_LoRa();
void LoRa_Init(UART_HandleTypeDef uartHandle);
void LoRa_SendCommand(reyax_connection port, const char *cmd);
void LoRa_ReceiveResponse(reyax_connection port , char *responseBuffer, uint16_t maxLength);
void LoRa_SetAddress(reyax_connection port,uint8_t address);
void LoRa_SetNetworkID(reyax_connection port,uint8_t networkID);
void LoRa_SendMessage(reyax_connection port,uint8_t targetAddress, const char *message);
void LoRa_SendBuffer(reyax_connection port,uint8_t targetAddress, const uint8_t *dataBuffer, size_t bufferSize);
void setBuffer(char *buffer, size_t bufferSize, const char *input);
MessageElement processReceivedData(const char *data);
void USART_ClearInterruptAndBuffer(UART_HandleTypeDef *huart);



#endif /* INC_REYAX_DRIVER_C_ */
