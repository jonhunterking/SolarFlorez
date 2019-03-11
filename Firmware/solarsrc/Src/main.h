#include <stdint.h>

uint32_t ADC_BUF[5];

float pv_current_adc = 0;
float pv_current = 0;
float pv_voltage = 0;
float pv_current_sum = 0;
float pv_voltage_sum = 0;
float boost_out_voltage;
float pv_power_new = 0;
int countcurr = 0;
float avecurr;
int pwm_tmp = 50;
void updateDutyCycle(int duty, uint32_t Channel);
uint32_t sample_pv_current(void);
uint32_t sample_pv_voltage(void);
