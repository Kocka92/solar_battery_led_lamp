/*
 * calibration.h
 *
 *  Created on: May 27, 2025
 *      Author: horvz
 */

#ifndef INC_CALIBRATION_H_
#define INC_CALIBRATION_H_

#include <stdint.h>
#include <stdbool.h>

// Kalibrációs paraméterek egy ADC csatornához
typedef struct {
    uint16_t raw_min;       // ADC érték min
    float measured_min;     // mért fizikai érték min

    uint16_t raw_max;		// ADC érték max
    float measured_max;		// mért fizikai érték max
} CalibrationParams_t;

typedef struct __attribute__((aligned(8))) {
    CalibrationParams_t light;						// fényérzékelő
    CalibrationParams_t battery;					// akkumulátor töltöttsége
    uint32_t crc32;  								// opcionális, ellenőrzéshez, (jelenleg nincs használatban, csak a helyet tartjuk fenn)
    uint32_t dummy;
} FlashCalibrationBlock_t;

void App_Calibration_Init(void);

bool CalibrateValue(const CalibrationParams_t* params, uint16_t raw, float* result);	// Lineáris interpoláció (NULL-safe)
bool IsCalibrationValid(const CalibrationParams_t* params);								// Kalibráció érvényesség ellenőrzése
void ResetCalibration(CalibrationParams_t* params);										// Kalibrációs értékek nullázása


#endif /* INC_CALIBRATION_H_ */
