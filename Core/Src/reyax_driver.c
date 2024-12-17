
/* This is a primitive driver for the REYAX parts 


*/



#include "main.h" // Or the specific STM32 HAL header for your MCU
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include "reyax_driver.h"
#include "cmsis_os.h"
extern osSemaphoreId_t got_alpha_messageHandle;
extern osSemaphoreId_t got_beta_messageHandle;
extern osSemaphoreId_t toggleLEDSemHandle;
extern osSemaphoreId_t toggleLED2semHandle;
extern uint8_t alphabyteReceived;
extern uint8_t betabyteReceived;
extern UART_HandleTypeDef *alphaUART,*betaUART;

////  Parse the message type
messageType parseMessageType(const char *response) {
    if (strstr(response, "+OK") == response) {
        return OK_MSG;
    } else if (strstr(response, "+RCV") == response) {
        return RCV_MSG;
    } else if (strstr(response, "+ERROR") == response) {
        return ERROR_MSG;
    } else if (strstr(response, "+RESET") == response) {
        return RESET_MSG;
    } else {
        return UNKNOWN_MSG;
    }
}

void sliceString(const char *data, char *result, int start, int length) {
    // Copy the specified substring to result
    strncpy(result, data + start, length);
    result[length] = '\0';  // Null-terminate the result
}



void initialize_LoRa()
	{
	// LoRa_SendCommand(const char *cmd);
	}

	////////////////////////////////////////////
	// Function to initialize UART (call during setup)
void LoRa_Init(UART_HandleTypeDef uartHandle)
	{
	////////////////////////////////////////////
    // huart1 = *uartHandle;
	}

	////////////////////////////////////////////
	// Function to send AT command and data
// void LoRa_SendCommand(UART_HandleTypeDef *uartHandle, const char *cmd)
void LoRa_SendCommand(reyax_connection port, const char *cmd)
{


UART_HandleTypeDef *the_uart;
osSemaphoreId_t the_semaphore;
uint8_t *the_byte;

	switch (port) {
	        case alpha:
	            the_uart = alphaUART;
	            the_semaphore = got_alpha_messageHandle;
	            the_byte = &alphabyteReceived;
	            break;
	        case beta:
	            the_uart = betaUART;
	            the_semaphore = got_beta_messageHandle;
	            the_byte = NULL;
	            break;
	        default:
	            printf("Unknown connection\n");
	    }

	////////////////////////////////////////////
    char buffer[50];
   // uint8_t *byte_in_add;
    snprintf(buffer, sizeof(buffer), "%s\r\n", cmd);
    // Turn on the receive interrupt every time a message gets sent
    // so the OK can get back
    // Take the semaphore.  When there's a full message, it will be released
    USART_ClearInterruptAndBuffer(the_uart);
	// Take it when I'm waiting for a message
    if (the_semaphore)
	 osSemaphoreRelease(the_semaphore);
	// if (osSemaphoreAcquire(the_semaphore, osWaitForever) == osOK) { printf("Semaphore taken by task: %s\n", pcTaskGetName(NULL));}
    HAL_UART_Receive_IT(the_uart, the_byte, 1);
    HAL_UART_Transmit(the_uart, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);

    osDelay(200);
	printf("Waited 200ms \r\n");

    // Wait for the OK
	}

	////////////////////////////////////////////
	// Function to receive data
void LoRa_ReceiveResponse(reyax_connection port,  char *responseBuffer, uint16_t maxLength)
{
	////////////////////////////////////////////
    // HAL_UART_Receive_IT(&huart1, &alphabyteReceived, 1);
	}


	////////////////////////////////////////////
void LoRa_SetAddress(reyax_connection port,uint8_t address)
{
	////////////////////////////////////////////
    char command[20];
    snprintf(command, sizeof(command), "AT+ADDRESS=%d", address);
    LoRa_SendCommand(port,command);
	}

	////////////////////////////////////////////
void LoRa_SetNetworkID(reyax_connection port,uint8_t networkID)
{
	////////////////////////////////////////////
    char command[20];
    snprintf(command, sizeof(command), "AT+NETWORKID=%d", networkID);
    LoRa_SendCommand(port,command);
	}

	////////////////////////////////////////////
void LoRa_SendMessage(reyax_connection port,uint8_t targetAddress, const char *message)
	{
	////////////////////////////////////////////
    char command[50];
    snprintf(command, sizeof(command), "AT+SEND=%d,%d,%s", targetAddress, strlen(message), message);
    LoRa_SendCommand(port,command);
	}



void LoRa_SendBuffer(reyax_connection port,uint8_t targetAddress, const uint8_t *dataBuffer, size_t bufferSize)
	{
	// Pass in a buffer pointer and send it.
    char command[50 + bufferSize * 2]; // Allowing space for hex data encoding

    // Convert binary data to a hex string
    char hexData[bufferSize * 2 + 1]; // Each byte becomes two hex chars, plus null terminator
    for (size_t i = 0; i < bufferSize; i++) {
        snprintf(&hexData[i * 2], 3, "%02X", dataBuffer[i]);
    }

    // Construct the command
    snprintf(command, sizeof(command), "AT+SEND=%d,%zu,%s", targetAddress, bufferSize, hexData);

    // Send the command
    LoRa_SendCommand(port,command);
}


bool LoRa_ReceiveMessage(reyax_connection port,char *messageBuffer, uint16_t bufferLength)
	{
    char response[100];
    LoRa_ReceiveResponse(port,response, sizeof(response));

    // Check if response is a received message, e.g., "+RCV"
    if (strstr(response, "+RCV")) {
        strncpy(messageBuffer, response, bufferLength);
        return true;
    }
    return false;
}


void setBuffer(char *buffer, size_t bufferSize, const char *input) {
    size_t inputLength = strlen(input);

    // Ensure input fits in the buffer
    if (inputLength >= bufferSize) {
        inputLength = bufferSize - 1; // Truncate to fit
    }

    // Copy the string into the buffer
    memcpy(buffer, input, inputLength);

    // Add the null terminator
    buffer[inputLength] = '\0';
}

MessageElement processReceivedData(const char *data) {

		/* Look at the buffer, known to have been terminated by the \n\r, and get
		 * message type and actual message and return a queue element
		 */

        int senderAddress = 0;
        int messageLength = 0;
        char message[100] = {0};  // Adjust size as needed	// this is a static array of 100 bytes
        int rssi = 0;
        int snr = 0;
        messageType theMessageType;
		MessageElement    theElement;

    // Check for AT commands first
    if (strncmp(data, "+OK", 3) == 0) {
        printf("AT Command Response: OK\n");
        theMessageType = OK_MSG;
        strncpy(theElement.theBuffer,data+4,MAX_MESSAGE_LENGTH);

    } else if (strncmp(data, "+ERROR", 6) == 0) {
        printf("AT Command Response: ERROR\n");
        theMessageType = ERROR_MSG;
        strncpy(theElement.theBuffer,data+7,MAX_MESSAGE_LENGTH);

    } else if (strncmp(data, "+RESET", 6) == 0) {
        printf("AT Command Response: RESET\n");
        theMessageType = RESET_MSG;
        strncpy(theElement.theBuffer,data+7,MAX_MESSAGE_LENGTH);

    } else if (strncmp(data, "+RCV=", 5) == 0) {
        theMessageType = RCV_MSG;
        strncpy(theElement.theBuffer,data+10,MAX_MESSAGE_LENGTH);

        // Parse the LoRa message
        if (sscanf(data, "+RCV=%d,%d,%[^,],%d,%d", &senderAddress, &messageLength, message, &rssi, &snr) == 5) {
            printf("LoRa Message Received:\n\r");
            printf("  Sender Address: %d\n\r", senderAddress);
            printf("  Message Length: %d\n\r", messageLength);
            printf("  Message: %s\n\r", message);
            printf("  RSSI: %d\n\r", rssi);
            printf("  SNR: %d\n\r", snr);

            osSemaphoreRelease(got_beta_messageHandle);
            // Process the message as needed sdf
        } else {
            printf("Invalid LoRa message format: %s\n", data);
        }
    } else {
        // Handle unknown responses or commands
        printf("Unknown data received: %s\n", data);
    }
    theElement.theType    = theMessageType;
	osSemaphoreRelease(got_alpha_messageHandle);
    return theElement;
}



/**
 * @brief Clears USART interrupt flags and flushes the receive buffer.
 * @param huart: Pointer to a UART_HandleTypeDef structure that contains
 *               the configuration information for the specified UART module.
 */
void USART_ClearInterruptAndBuffer(UART_HandleTypeDef *huart) {
    if (huart == NULL || huart->Instance == NULL) {
        return; // Protect against null pointers
    }

    USART_TypeDef *USARTx = huart->Instance;

    // Clear Overrun Error (ORE) flag if set
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE)) {
        __HAL_UART_CLEAR_OREFLAG(huart);
    }

    // Clear Framing Error (FE) flag if set
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_FE)) {
        __HAL_UART_CLEAR_FEFLAG(huart);
    }

    // Clear Noise Error (NE) flag if set
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_NE)) {
        __HAL_UART_CLEAR_NEFLAG(huart);
    }

    // Clear Parity Error (PE) flag if set
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_PE)) {
        __HAL_UART_CLEAR_PEFLAG(huart);
    }

    // Clear Transmit Complete (TC) flag if set
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_TC)) {
        __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_TC);
    }

    // Clear Receive Buffer (flush data register)
    while (__HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE)) {
        volatile uint8_t temp = (uint8_t)(USARTx->RDR); // Read and discard
        (void)temp; // Suppress unused variable warning
    }
}


