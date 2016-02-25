#include "motor.h"

void SetSpeed(int motorNumber, uint8_t speed){
	uint32_t *ccr1 = (uint32_t *)&TIM1->CCR1;
	uint32_t *ccr2 = (uint32_t *)&TIM1->CCR2;
	
	if(motorNumber == 1){
		ccr1 = (uint32_t *)&TIM8->CCR1;
		ccr2 = (uint32_t *)&TIM8->CCR2;
	}
	
	*ccr1 = 0;
	*ccr2 = 0;
	if((int8_t)speed < 0) *ccr1 = (((speed*-1)&0x7F)*2);
	else *ccr2 = ((speed&0x7F)*2);
}
