/*
 * JDY-18.h
 *
 *  Created on: Jan 14, 2025
 *      Authors:
 *      - Tarcisio Lucas Cairo Carvalho
 *      - Filipe Campos Reis
 *      - Pedro de Araujo Cruz
 *
 *  Description:
 *  This header file defines the interfaces for controlling and interacting with the JDY-18 Bluetooth module.
 *  It includes functions for initializing the module, sending AT commands, scanning for devices, and
 *  parsing scan responses.
 */


#ifndef INC_JDY_18_H_
#define INC_JDY_18_H_

#include "stm32f4xx.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>

#define INIT_MODULE_RESPONSE_SCAN "+DEV:"
#define END_RESPONSE_SCAN "+STOP:SCAN"

#define MAX_SIZE_DATA 30
#define MAX_SIZE_UART_BUFFER 1000
#define MAX_DEVICE_LIST 10
#define MAC_ADDRESS_SIZE 12
#define MAX_DEVICE_NAME_SIZE 50

typedef enum {
	SET_BLE_NAME,
	SET_BLE_ROLE,
	SET_BAUD,
	SET_IBUUID,
	SET_POWR,
	SCAN_SLAVES,
	CONENCT,
	SET_PERMISSIONS
} AtInstruction_t;

typedef struct {
	char name[MAX_DEVICE_NAME_SIZE];
	char mac[MAC_ADDRESS_SIZE];
	int rssi;
} device_t;

typedef struct {
	device_t devices[MAX_DEVICE_LIST];
	size_t size;
} scan_t;

extern scan_t scannedDevices;
extern char uartBuffer[MAX_SIZE_UART_BUFFER];
extern int loop;
//extern char buffer;

/**
 * @brief Inicializa o módulo Bluetooth.
 *
 * Configura a UART que será utilizada para a comunicação com o módulo Bluetooth.
 *
 * @param huart Ponteiro para o manipulador da UART (UART_HandleTypeDef) configurada para comunicação com o módulo.
 */
void BLE_Init(UART_HandleTypeDef* huart);

/**
 * @brief Envia um comando AT para o módulo Bluetooth.
 *
 * Constrói e envia um comando AT com um parâmetro opcional para o módulo Bluetooth usando a UART especificada.
 *
 * @param instruction Enum do tipo AtInstruction_t que indica o comando AT a ser enviado.
 * @param parameter String opcional contendo o parâmetro do comando AT.
 * @param huart Ponteiro para o manipulador da UART utilizada para comunicação.
 */
void BLE_SendInstruction (AtInstruction_t instruction, char* parameter, UART_HandleTypeDef* huart);

/**
 * @brief Realiza uma varredura de dispositivos Bluetooth.
 *
 * Inicia a varredura de dispositivos Bluetooth próximos e armazena os resultados em um buffer interno.
 *
 * @param huart Ponteiro para o manipulador da UART configurada para comunicação com o módulo.
 */
void BLE_ScanDevices(UART_HandleTypeDef* huart);

/**
 * @brief Carrega as informações de um dispositivo detectado.
 *
 * Extrai e processa as informações de um dispositivo Bluetooth a partir de uma string delimitada.
 *
 * @param start Ponteiro para o início da string contendo as informações do dispositivo.
 * @param end Ponteiro para o final da string contendo as informações do dispositivo.
 * @param device Estrutura do tipo device_t onde os dados do dispositivo serão armazenados.
 */
void JDY18Driver_LoadDeviceInfo(char* start, char* end, device_t* device);

/**
 * @brief Analisa a resposta da varredura do módulo Bluetooth.
 *
 * Processa a string contendo a resposta do comando de varredura e extrai as informações dos dispositivos detectados.
 *
 * @param scanResponse String contendo a resposta da varredura.
 * @param scan Estrutura do tipo scan_t onde os dispositivos detectados serão armazenados.
 */
void JDY18Driver_ParseScanResponse(char* scanResponse, scan_t* scan);

/**
 * @brief Recupera os dispositivos detectados na última varredura.
 *
 * Copia os dispositivos detectados durante a última varredura para uma estrutura fornecida pelo usuário.
 *
 * @param devices Ponteiro para uma estrutura do tipo scan_t onde os dispositivos serão armazenados.
 */
void BLE_GetScannedDevices(scan_t* devices);

#endif /* INC_JDY_18_H_ */
