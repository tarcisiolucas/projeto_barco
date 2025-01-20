/*
 * JDY-18.c
 *
 *  Created on: Jan 14, 2025
 *      Author: tarci
 */

#include "JDY-18.h"

char inputBuffer[1];

char *atInstructions[] = {
	"AT+NAME",
	"AT+ROLE",
	"AT+BAUD",
	"AT+IBUUID",
	"AT+POWR",
	"AT+INQ",
	"AT+CONN",
	"AT+PERM"
};

scan_t scannedDevices;

char inputBuffer[1];
char uartBuffer[MAX_SIZE_UART_BUFFER];

UART_HandleTypeDef *huart_ble;

void BLE_Init(UART_HandleTypeDef* huart){
	huart_ble = huart;
}

void BLE_SendInstruction (AtInstruction_t instruction, char* parameter, UART_HandleTypeDef* huart) {
	char *instructionPrefix = atInstructions[instruction];
	char *completeInstruction = (char*) malloc(30 * sizeof(char));
	sprintf(completeInstruction, "%s%s\r\n", instructionPrefix, parameter);
	HAL_UART_Transmit(huart, (uint8_t *) completeInstruction, strlen(completeInstruction), HAL_MAX_DELAY);
	free(completeInstruction);
}

void BLE_ScanDevices(UART_HandleTypeDef* huart){
//	char *allText = (char *) malloc(1000 * sizeof(char));
//	buffer = allText;
	uint32_t start_time = HAL_GetTick();
	BLE_SendInstruction(SCAN_SLAVES, "", huart);
	memset(uartBuffer, 0, sizeof(uartBuffer));
	while(!((strstr(uartBuffer, "STOP:SCAN") != NULL))) {
		if (HAL_GetTick() - start_time >= 500) {
			break;
		}
		HAL_UART_Receive(huart, (uint8_t *) inputBuffer, 1, 500);
//		strcat(allText, inputBuffer);
		if (strlen(uartBuffer) + 1 < MAX_SIZE_UART_BUFFER) {
		    strcat(uartBuffer, inputBuffer);
		}
	}
//	strcpy(lendo,allText);
	JDY18Driver_ParseScanResponse(uartBuffer, &scannedDevices);
	memset(uartBuffer, 0, sizeof(uartBuffer));
//	free(allText);
}

void JDY18Driver_LoadDeviceInfo(char* start, char* end, device_t* device)
{

    if (start == NULL || end == NULL || device == NULL) {
        return; // Protege contra ponteiros nulos
    }

    size_t size = end - start;
    if (size <= 7) { // Tamanho insuficiente para conter dados válidos
        return;
    }

    char temp[size + 1]; // Buffer temporário
    memcpy(temp, start, size);
    temp[size] = '\0'; // Garante que a string esteja terminada em nulo

    char* macAddressPointer = temp + 7; // MAC começa após "+DEV:"
    if (macAddressPointer >= temp + size || macAddressPointer + MAC_ADDRESS_SIZE > temp + size) {
        return; // Verifica se o MAC está dentro dos limites
    }

    char* rssiPointer = macAddressPointer + MAC_ADDRESS_SIZE + 1; // RSSI após MAC
    if (rssiPointer >= temp + size) {
        return; // Verifica se o RSSI está dentro dos limites
    }

    char* namePointer = strstr(rssiPointer, ",");
    if (namePointer == NULL) {
        return; // Nome inválido ou delimitador ausente
    }
    namePointer += 1; // Salta a vírgula

    // Calcula o tamanho do RSSI
    size_t rssiSize = namePointer - rssiPointer - 1;
    if (rssiSize >= sizeof(temp)) {
        return; // Tamanho do RSSI fora dos limites
    }

    // Buffers temporários
    char rssiStr[rssiSize + 1];
    memcpy(rssiStr, rssiPointer, rssiSize);
    rssiStr[rssiSize] = '\0';

    // Copia os dados para o dispositivo
//    memset(device->mac, 0, MAC_ADDRESS_SIZE); // Limpa o MAC antes de copiar
//    memcpy(device->mac, macAddressPointer, MAC_ADDRESS_SIZE);

    memset(device->name, 0, MAX_DEVICE_NAME_SIZE); // Limpa o nome antes de copiar
    strncpy(device->name, namePointer, MAX_DEVICE_NAME_SIZE - 1);
    device->name[MAX_DEVICE_NAME_SIZE - 1] = '\0'; // Garante terminação nula

    device->rssi = atoi(rssiStr); // Converte RSSI para inteiro
}

void JDY18Driver_ParseScanResponse(char* scanResponse, scan_t* scan)
{
//	char *start, *end;
//	scan->size = 0;
//
//	if((start = strstr(scanResponse, INIT_MODULE_RESPONSE_SCAN))) {
//		while((end = strstr(start + 1, INIT_MODULE_RESPONSE_SCAN))) {
//			JDY18Driver_LoadDeviceInfo(start, end, &scan->devices[scan->size]);
//			start = end;
//			scan->size++;
//		}
//		if((end = strstr(start + 1, END_RESPONSE_SCAN))) {
//			JDY18Driver_LoadDeviceInfo(start, end, &scan->devices[scan->size]);
//			scan->size++;
//		}
//	}

    char *start, *end;
    memset(scan, 0, sizeof(scan_t));
    scan->size = 0;

    if ((start = strstr(scanResponse, INIT_MODULE_RESPONSE_SCAN))) {
        while ((end = strstr(start + 1, INIT_MODULE_RESPONSE_SCAN))) {
            JDY18Driver_LoadDeviceInfo(start, end, &scan->devices[scan->size]);
            start = end;
            scan->size++;
        }
        // Verifica se o próximo caractere após o último marcador é nulo
        if (*(start + strlen(INIT_MODULE_RESPONSE_SCAN)) == '\0') {
            JDY18Driver_LoadDeviceInfo(start, start + strlen(start), &scan->devices[scan->size]);
            scan->size++;
        }
    }
}

void BLE_GetScannedDevices(scan_t* devices) {
	*devices = scannedDevices;
}

