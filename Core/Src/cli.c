/*
 * cli.c
 *
 *  Created on: May 27, 2025
 *      Author: horvz
 */
#include "cli.h"
#include <stdio.h>          // printf, sscanf, stb.
#include <string.h>         // strncmp, strcmp, memset, stb.
#include <stdint.h>         // uint8_t, uint16_t, stb.
#include <stdbool.h>        // bool típus
#include "calibration.h"    // CalibrationParams_t, stb.
#include "stm32f4xx_hal.h"  // HAL, ADC funkciókhoz
#include "cmsis_os2.h"      // FreeRTOS mutex kezelés: osMutexAcquire, osMutexRelease
#include "main.h"
#include <stdarg.h> 		// va_list, stb.

extern volatile uint8_t g_debug_enabled;
extern osMutexId_t uartMutexHandle;
extern UART_HandleTypeDef huart2;

int __io_putchar(int ch)		//printf uartra
{
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

//Wrapper fgv. a thread-save printf-hez, mutex-el védve
void cli_printf(const char *fmt, ...) {
	if (osMutexAcquire(uartMutexHandle, osWaitForever) == osOK) {
		va_list args;
		va_start(args, fmt);
		vprintf(fmt, args); // a printf variációja, elfogad va_listet
		va_end(args);
		osMutexRelease(uartMutexHandle);
	}
}

// Parancssor értelmező függvény
bool CLI_ProcessCommand(const char *input) {
	if (input == NULL)
		return false;

	if (strcmp(input, "help") == 0) { /* help szöveg */
			cli_printf("Available commands:\r\n");
			cli_printf("  help          - Show (this) command list.\r\n");
			cli_printf("  led_status    - Current status of LED and CHARGE outputs.\r\n");
			cli_printf("  s_read        - Display of calibrated light/battery and voltage measurements.\r\n");
			cli_printf("  debug	        - Continuous debug messages ON.\r\n");
			cli_printf("  d             - Continuous debug messages OFF.\r\n");

			return true;

	} else if (strcmp(input, "debug") == 0) {
			g_debug_enabled = 1;
			cli_printf("Debug mode ON\r\n");
			return true;

	} else if (strcmp(input, "d") == 0) {
			g_debug_enabled = 0;
			cli_printf("Debug mode OFF\r\n");
			return true;

	} else if (strcmp(input, "led_status") == 0) { /* led kiírás */
		// Példa: LED-ek globális változóból kiolvasva, mutex védetten!
		uint8_t half = 0, full = 0, charge = 0;
		if (osMutexAcquire(ledMutexHandle, 50) == osOK) {
			half = HAL_GPIO_ReadPin(LED_HALF_GPIO_Port, LED_HALF_Pin);
			full = HAL_GPIO_ReadPin(LED_FULL_GPIO_Port, LED_FULL_Pin);
			charge = HAL_GPIO_ReadPin(CHARGE_GPIO_Port, CHARGE_Pin);
			osMutexRelease(ledMutexHandle);
			cli_printf("LED_HALF: %d, LED_FULL: %d, CHARGE: %d\r\n", half, full,
					charge);
		} else {
			cli_printf("LED mutex acquire failed./r/n");
		}
		return true;

	} else if (strcmp(input, "s_read") == 0) { /* szenzoradatok */
		float light = 0.0f;
		float voltage = 0.0f;
		uint16_t raw_light = 0;
		uint16_t raw_battery = 0;

		if (osMutexAcquire(dataMutexHandle, 100) == osOK) {
			light = g_sensorData.lightLevel;
			voltage = g_sensorData.batteryVoltage;
			raw_light = g_sensorData.raw_light;
			raw_battery = g_sensorData.raw_battery;
			osMutexRelease(dataMutexHandle);

			cli_printf("Sensor data:\r\n");
			cli_printf("  Light: 	%.2f V (raw: %u)\r\n", light, raw_light);
			cli_printf("  Battery: 	%.2f V (raw: %u)\r\n", voltage, raw_battery);
		} else {
			cli_printf("Failed to read sensor data (mutex timeout).\r\n");
		}
		return true;
	}else {
		cli_printf("Unknown command\r\n");
		return false;
	}return false;
}
