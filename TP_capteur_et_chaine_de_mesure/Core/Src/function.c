/*
 * function.c
 *
 *  Created on: Nov 30, 2022
 *      Author: quentinrolland
 */


#include "function.h"
#include "Const.h"
#include "math.h"


 /**
  * \fn Init(I2C_HandleTypeDef* p_hi2c1)
  * \brief Cette fonction met à 1 le bit 7 de PWR_MGMT_1 pour faire un reset de l'ensemble des registres du capteur, puis attend
	 * 100ms et choisit une horlorge
	 * \param p_hi2c1
	 */
void Init(I2C_HandleTypeDef* p_hi2c1)
{

	uint8_t buff[6];
	buff[0] = 0x80;
	HAL_I2C_Mem_Write ( p_hi2c1, MPU_ADD,  PWR_MGMT_1,  1, &buff[0], 1, 10);
	HAL_Delay(100);
	buff[0] = 0x1;
	HAL_I2C_Mem_Write ( p_hi2c1, MPU_ADD, PWR_MGMT_1,  1, &buff[0], 1, 10);


	buff[0] =0x00;  // changement de la sensibilité de l'accélérometre  00=2g 10=4g 01=8g 11=16g
		HAL_I2C_Mem_Write ( p_hi2c1, MPU_ADD,  ACCEL_CONFIG,  1, &buff[0], 1, 10);


	buff[0]=0x2;
		  if(HAL_I2C_Mem_Write(p_hi2c1,MPU_ADD,INT_PIN_CFG,1,&buff[0],1,10)!=HAL_OK){
		  	  Error_Handler();
		  }
	buff[0]=0x16;
		 	  if(HAL_I2C_Mem_Write(p_hi2c1,MAGNETO_ADD,AK8963_CNTL,1,&buff[0],1,10)!=HAL_OK){
		 	  	  Error_Handler();

		 	  }
	buff[0]=0x3;
	if(HAL_I2C_Mem_Write(p_hi2c1,MPU_ADD,CONFIG,1,buff,1,10)!=HAL_OK){
		Error_Handler();

	}
	buff[0]=0xFF;
	if(HAL_I2C_Mem_Write(p_hi2c1,MPU_ADD,SMPLRT_DIV,1,buff,1,10)!=HAL_OK){
		Error_Handler();

	}
}


/**
 * \fn Measure_T(I2C_HandleTypeDef*,double*)
 * \brief Cette fonction permet de lire la temperature du capteur puis de l'enregistrer dans une variable globale de type double
	 * \param p_hi2c1
	 * un pointeur vers une vaiable Temperature de type double dans laquelle on stocke la valeur obtenu
	 */

void Measure_T(I2C_HandleTypeDef* p_hi2c1, double* Temperature)
{
	uint8_t buff[6];
	HAL_I2C_Mem_Read ( p_hi2c1, MPU_ADD, TEMP_OUT_H, 1, &buff[0], 1, 10);
	HAL_I2C_Mem_Read ( p_hi2c1, MPU_ADD, TEMP_OUT_L, 1, &buff[1], 1, 10);

	uint16_t Temp = 0;
	Temp = ((buff[0]<< 8) + buff[1]);

	*Temperature = ((Temp - ROOM_TEMP_OFFSET)/TEMP_SENS) + 21;

}









