#include <stdint.h>
#include "main.h"
#include "mppt.h"

uint32_t sample_pv_current(void){
	uint32_t current_sum = 0;
	uint32_t pv_current_sampled = 0;
	for(int i=0; i<50; i++){
		current_sum += *PV_voltage;
	}
	pv_current_sampled = current_sum/50;
	return pv_current_sampled;
}

uint32_t sample_pv_voltage(void){
	uint32_t voltage_sum = 0;
	uint32_t pv_voltage_sampled = 0;
	for(int i=0; i<50; i++){
		voltage_sum += *PV_voltage;
	}
	pv_voltage_sampled = voltage_sum/50;
	return pv_voltage_sampled;
}




