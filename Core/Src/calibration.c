/*
 * calibration.c
 *
 *  Created on: May 27, 2025
 *      Author: horvz
 */

#include "calibration.h"
#include "main.h"
// ADC értékek megadása, fény 0-3,3; akku 3,2-4,2
void App_Calibration_Init(void)
{
    g_flashCalib.light.raw_min = 0;
    g_flashCalib.light.raw_max = 3200;
    g_flashCalib.light.measured_min = 0.0f;
    g_flashCalib.light.measured_max = 3.3f;

    g_flashCalib.battery.raw_min = 0;
    g_flashCalib.battery.raw_max = 3200;
    g_flashCalib.battery.measured_min = 3.2f;
    g_flashCalib.battery.measured_max = 4.2f;

    g_flashCalib.crc32 = 0;  // opcionális
    g_flashCalib.dummy = 0;  // opcionális
}


bool CalibrateValue(const CalibrationParams_t *params, uint16_t raw, float *result) {
	if (params == 0 || result == 0) {		// NULL pointer védelem (NULL-safe)
		return false;
	}

	if (!IsCalibrationValid(params)) { 		// Érvényesség-ellenőrzés!
	        return false;
    }

	if (params->raw_max == params->raw_min) {
		return false; 						// elkerülni az osztást nullával (x1 - x0 == 0)
	}

	float alpha = (float) (raw - params->raw_min)
			/ (params->raw_max - params->raw_min);
	*result = params->measured_min + alpha * (params->measured_max - params->measured_min); 	// Lineáris interpoláció kiszámítása

	return true;				//ha sikerült minden akkor az eredmény a result-ban van
}

bool IsCalibrationValid(const CalibrationParams_t *params) {
	if (params == 0) {
		return false;
	}

	return (params->raw_max > params->raw_min)
			&& (params->measured_max > params->measured_min);
}

void ResetCalibration(CalibrationParams_t *params) {
	if (params == 0) {
		return;
	}

	params->raw_min = 0;
	params->measured_min = 0.0f;
	params->raw_max = 0;
	params->measured_max = 0.0f;
}

