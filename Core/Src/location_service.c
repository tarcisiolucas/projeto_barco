/**
 * @file location_service.c
 * @author Pierre Victor
 * 		   Mariana Leite
 * @brief 
 * @version 0.1
 * @date 2023-11-02
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "location_service.h"
#include "JDY-18.h"
#include "data_filter_service.h"

#include <math.h>

buffer_t b1Buffer;
buffer_t b2Buffer;
buffer_t b3Buffer;

/*
(lat, long)
Beacon (B1)  (-19.866581, -43.964787) > (7802949.9627924375, 608384.2009718714)
Chegada (B2) (-19.866733, -43.964666) > (7802933.062793205, 608396.7671230149)
Saida (B3)   (-19.866425, -43.964556) > (7802967.079134472, 608408.4941720127)
*/
location_t slaveBeaconLocationB1 = { 7802949.9627924375, 608384.2009718714 };
location_t slaveBeaconLocationB2 = { 7802933.062793205, 608396.7671230149 };
location_t slaveBeaconLocationB3 = { 7802967.079134472, 608408.4941720127 };
location_t masterLocation = { 0, 0 };

float trilaterationCalcCPartial = 0;
float trilaterationCalcFPartial = 0;

float b1Distance = 0, b2Distance = 0, b3Distance = 0;
float b1OldDistance = -1, b2OldDistance = -1, b3OldDistance = -1;

void LocationService_Init(UART_HandleTypeDef *huart)
{
	DataFilterService_InitBuffer(&b1Buffer);
	DataFilterService_InitBuffer(&b2Buffer);
	DataFilterService_InitBuffer(&b3Buffer);

	trilaterationCalcCPartial = - pow(slaveBeaconLocationB1.longitude, 2) + pow(slaveBeaconLocationB2.longitude, 2) - pow(slaveBeaconLocationB1.latitude, 2) + pow(slaveBeaconLocationB2.latitude, 2);
	trilaterationCalcFPartial = - pow(slaveBeaconLocationB2.longitude, 2) + pow(slaveBeaconLocationB3.longitude, 2) - pow(slaveBeaconLocationB2.latitude, 2) + pow(slaveBeaconLocationB3.latitude, 2);
}

float LocationService_CalculateDistance(int rssi)
{
//	float value = pow(10, ((MEASURED_POWER - rssi) / 20));
	if (rssi == 0) {
		return -1.0f; // Retorna -1 como erro
	}

	// Calcula a distância usando a fórmula
	float exponent = (MEASURED_POWER - rssi) / 20.0f;
	float distance = pow(10.0f, exponent);

	return distance;
}

void LocationService_UpdateLocation(UART_HandleTypeDef *huart)
{
	scan_t scan;
	memset(&scan, 0, sizeof(scan_t));

	BLE_ScanDevices(huart);
	BLE_GetScannedDevices(&scan);

	for(size_t i = 0; i < scan.size; i++) {
		if (i >= MAX_DEVICE_LIST) {
			break; // Protege contra acesso fora dos limites
		}

		char* deviceName = scan.devices[i].name;
		int rssi = scan.devices[i].rssi;

		if(strstr(deviceName, SLAVE_BEACON_NAME_B1) != NULL) {
			b1OldDistance = b1Distance;
			b1Distance = rssi;
			b1Distance = DataFilterService_MovingAverage(&b1Buffer, LocationService_CalculateDistance(rssi));
		} else if(strstr(deviceName, SLAVE_BEACON_NAME_B2) != NULL) {
			b2OldDistance = b2Distance;
			b2Distance = rssi;
			b2Distance = DataFilterService_MovingAverage(&b2Buffer, LocationService_CalculateDistance(rssi));
		} else if(strstr(deviceName, SLAVE_BEACON_NAME_B3) != NULL) {
			b3OldDistance = b3Distance;
			b3Distance = rssi;
			b3Distance = DataFilterService_MovingAverage(&b3Buffer, LocationService_CalculateDistance(rssi));
		}
	}

	if (b1Distance != b1OldDistance && b2Distance != b2OldDistance && b3Distance != b3OldDistance) {
		float trilaterationCalcA = -2*slaveBeaconLocationB1.longitude + 2*slaveBeaconLocationB2.longitude;
		float trilaterationCalcB = -2*slaveBeaconLocationB1.latitude + 2*slaveBeaconLocationB2.latitude;
		float trilaterationCalcC = pow(b1Distance, 2) - pow(b2Distance, 2) + trilaterationCalcCPartial;
		float trilaterationCalcD = -2*slaveBeaconLocationB2.longitude + 2*slaveBeaconLocationB3.longitude;
		float trilaterationCalcE = -2*slaveBeaconLocationB2.latitude + 2*slaveBeaconLocationB3.latitude;
		float trilaterationCalcF = pow(b2Distance, 2) - pow(b3Distance, 2) + trilaterationCalcFPartial;

		masterLocation.longitude = (trilaterationCalcC*trilaterationCalcE - trilaterationCalcF*trilaterationCalcB)/(trilaterationCalcE*trilaterationCalcA - trilaterationCalcB*trilaterationCalcD);
		masterLocation.latitude= (trilaterationCalcC*trilaterationCalcD - trilaterationCalcA*trilaterationCalcF)/(trilaterationCalcB*trilaterationCalcD - trilaterationCalcA*trilaterationCalcE);
	}

}

location_t LocationService_GetLocation()
{
	return masterLocation;
}

float LocationService_GetArrivalAngle()
{

	float deltaX = masterLocation.longitude - slaveBeaconLocationB2.longitude;
	float deltaY = masterLocation.latitude  - slaveBeaconLocationB2.latitude;

	    // Calcula o ângulo em radianos usando atan2(Y, X)
	    float angleRadians = atan2(deltaY, deltaX);

	    // Converte para graus
	    float angleDegrees = angleRadians * (180.0f / (float)M_PI);

	    // Ajusta para ficar no intervalo [0, 360)
	    if (angleDegrees < 0.0f) {
	        angleDegrees += 360.0f;
	    }

	    return angleDegrees;
//	return atan((masterLocation.longitude - slaveBeaconLocationB2.longitude) / (masterLocation.latitude - slaveBeaconLocationB2.latitude)) * 180 / 3.14159265359 + 180;
}

uint8_t LocationService_IsInDestiny()
{
	return ((masterLocation.longitude - slaveBeaconLocationB2.longitude) < PRECISION_BLE_METERS) && ((masterLocation.latitude - slaveBeaconLocationB2.latitude) < PRECISION_BLE_METERS) ? 1 : 0;
}
