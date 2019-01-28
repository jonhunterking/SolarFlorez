#include <stdint.h>

uint32_t ADC_BUF[5];
/*extern uint32_t *PV_current = &ADC_BUF[0];
extern uint32_t *PV_voltage = &ADC_BUF[1];
extern uint32_t *boost_out_voltage = &ADC_BUF[2];
extern uint32_t *buck_out_current = &ADC_BUF[3];
extern uint32_t *buck_out_voltage = &ADC_BUF[4];*/
void updateDutyCycle(uint32_t duty, uint32_t Channel);
