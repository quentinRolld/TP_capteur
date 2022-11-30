/*
 ******************************************************************************
 * \file function.c
 *
 * \@Note Created on: Nov 30, 2022
 *      Author: quentinrolland
 ******************************************************************************
 */


#include "function.h"
#include "Const.h"
#include "math.h"

/**
 	 * @brief Init I2C1
 	 * @Note Cette fonction met à 1 le bit 7 de PWR_MGMT_1 pour faire un reset de l'ensemble des registres du capteur, puis attend
	 * 100ms et choisit une horlorge
	 * @param p_hi2c1 Pointeur vers une structure I2C qui contient l'information de configuration pour un i2c particulier
	 * @retval None
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
 	 * @brief Mesure temperature
 	 * @Note Cette fonction permet de lire la temperature du capteur puis de l'enregistrer dans une variable globale de type double
	 * @param p_hi2c1 Pointeur vers une structure I2C qui contient l'information de configuration pour un i2c particulier.
	 * @param Temperature Pointeur vers une zone mémoire de type double contenant l’information de température
	 * @retval None
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


/**
 	 * @brief Mesure acceleration
 	 * @Note Cette fonction permet de lire l'acceleration du capteur sur les axes x,y et z puis de l'enregistrer dans une variable globale de type double
	 * @param p_hi2c1 Pointeur vers une structure I2C qui contient l'information de configuration pour un i2c particulier
	 * @param Acceleration Pointeur vers une zone mémoire de type double contenant l’information de température
	 * @retval None
	 */

void Measure_A(I2C_HandleTypeDef* p_hi2c1,double* Acceleration){
	uint8_t buff[6];
	HAL_I2C_Mem_Read ( p_hi2c1, MPU_ADD, ACCEL_XOUT_H, 1, &buff[0], 6, 10);


	uint16_t ax = 0;
		ax = ((buff[0]<< 8) + buff[1]);

	uint16_t ay = 0;
		ay = ((buff[2]<< 8) + buff[3]);

	uint16_t az = 0;
		az = ((buff[4]<< 8) + buff[5]);

	Acceleration[0] = ax*9.81*2/32767.0;
	Acceleration[1] = ay*9.81*2/32767.0;
	Acceleration[2] = az*9.81*2/32767.0;
}








