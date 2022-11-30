/*
 * function.h
 *
 *  Created on: Nov 30, 2022
 *      Author: quentinrolland
 */

#ifndef INC_FUNCTION_H_
#define INC_FUNCTION_H_

#include "main.h"

void Init(I2C_HandleTypeDef* p_hi2c1);
void Measure_T(I2C_HandleTypeDef*,double*);

#endif /* INC_FUNCTION_H_ */
