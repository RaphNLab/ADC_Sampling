/*
 * ADC_drv.h
 *
 *  Created on: Oct 8, 2024
 *      Author: silvere
 */

#ifndef DRIVERS_ADC_DRV_H_
#define DRIVERS_ADC_DRV_H_

#include "global.h"

#define TS_CAL1_REG 		((uint16_t*) ((uint32_t) 0x1FF800FA))
#define TS_CAL2_REG 		((uint16_t*) ((uint32_t) 0x1FF800FE))
#define CALIBRATION_SIZE 	1U

#define V_REF				((uint32_t) 3000)
#define VDDA_APPLI          ((uint32_t) 3300)
#define RANGE_12BITS 		((uint32_t) 4095)


#define COMPUTATION_DIGITAL_12BITS_TO_VOLTAGE(ADC_DATA) (((ADC_DATA) * VDDA_APPLI) / RANGE_12BITS)


/* Calculate temperature sensor value take n from the user manual formula */
#define COMPUTATION_TEMPERATURE_TEMP30_TEMP110(TS_ADC_DATA)           \
  (((( ((int32_t)((TS_ADC_DATA * VDDA_APPLI) / V_REF)                 \
        - (int32_t) *TS_CAL1_REG)                                     \
     ) * (int32_t)(110 - 30)                                          \
    ) / (int32_t)(*TS_CAL2_REG - *TS_CAL1_REG)                        \
   ) + 30                                                             \
  )


void adc_config(void);
void adc_enable_temp(void);
uint16_t get_adc_value(void);
int16_t get_temperature(void);



#endif /* DRIVERS_ADC_DRV_H_ */
