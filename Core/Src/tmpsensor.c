/*
 * tmpsensor.c
 *
 *  Created on: Feb 22, 2023
 *      Author: LOC
 */
#include "tmpsensor.h"


/**
  * @brief Calculate temperature (tested on STM32F401, other MCU may have different constants!)
  * @note If IntRef not use, set it [ex.: #define TMPSENSOR_USE_INTREF 0]
  * @param Temperature sensor's ADC 16-bit value, Internal Reference ADC 16-bit value (if use)
  * @retval Internal sensor temperature
  */
double TMPSENSOR_getTemperature(uint16_t adc_sensor, uint16_t adc_intref){

#if(TMPSENSOR_USE_INTREF)

	double intref_vol = (adc_intref*TMPSENSOR_ADCVREFINT)/TMPSENSOR_ADCMAX;

#else

	double intref_vol = TMPSENSOR_ADCREFVOL;
#endif

//	double sensor_vol = adc_sensor * intref_vol/TMPSENSOR_ADCMAX;
	double sensor_vol = intref_vol;

	double sensor_tmp = (sensor_vol - TMPSENSOR_V25) *1000.0/TMPSENSOR_AVGSLOPE + 25.0;

	return sensor_tmp;
}


