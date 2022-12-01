/*
 * function.h
 *
 *  Created on: Nov 30, 2022
 *      Author: quentinrolland
 */

#ifndef INC_FUNCTION_H_
#define INC_FUNCTION_H_

#include "main.h"


void Init(I2C_HandleTypeDef* hi2cx);
void Measure_T(I2C_HandleTypeDef* hi2cx, double* Temperature);
void Measure_A(I2C_HandleTypeDef* hi2cx, double* Acceleration);
void Measure_Vitesse_angulaire(I2C_HandleTypeDef* hi2cx, double* tableau_donnee_utiles);
void Measure_M(I2C_HandleTypeDef* p_hi2c1,double* mag);

#endif /* INC_FUNCTION_H_ */
