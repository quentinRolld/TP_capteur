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
#include <stdio.h>

/**
 	 * @brief Init I2C1
 	 * @Note Cette fonction met à 1 le bit 7 de PWR_MGMT_1 pour faire un reset de l'ensemble des registres du capteur, puis attend
	 * 100ms et choisit une horlorge. De plus, elle désactive l'interface master I2C du MPU-9250 pour permettre la lecture des données du magnétomètre
	 * et elle configure ce dernier pour qu'il réalise des mesures en continu.
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


	buff[0]=0x2; // Bypass pour activer le magnétomètre
		  if(HAL_I2C_Mem_Write(p_hi2c1,MPU_ADD,INT_PIN_CFG,1,&buff[0],1,10)!=HAL_OK){
		  	  Error_Handler();
		  }
	buff[0]=0x16; // configuration mesure en continu du magnétomètre
		 	  if(HAL_I2C_Mem_Write(p_hi2c1,MAGNETO_ADD,AK8963_CNTL,1,&buff[0],1,10)!=HAL_OK){
		 	  	  Error_Handler();

		 	  }
	/*
	buff[0]=0x3; // réglage de la bande passante
	if(HAL_I2C_Mem_Write(p_hi2c1,MPU_ADD,CONFIG,1,buff,1,10)!=HAL_OK){
		Error_Handler();

	}
	buff[0]=0xFF; // réglage de la fréquence d'échantillonnage
	if(HAL_I2C_Mem_Write(p_hi2c1,MPU_ADD,SMPLRT_DIV,1,buff,1,10)!=HAL_OK){
		Error_Handler();
	}
	*/
}


/**
 	 * @brief Mesure temperature
 	 * @Note Cette fonction permet de lire la temperature du capteur puis de l'enregistrer dans une variable globale de type double
	 * @param hi2cx Pointeur vers une structure I2C qui contient l'information de configuration pour un i2c particulier.
	 * @param Temperature Pointeur vers une zone mémoire de type double contenant l’information de température
	 * @retval None
	 */

void Measure_T(I2C_HandleTypeDef* hi2cx, double* Temperature)
{
	uint8_t buff[6];
	HAL_I2C_Mem_Read ( hi2cx, MPU_ADD, TEMP_OUT_H, 1, &buff[0], 1, 10);
	HAL_I2C_Mem_Read ( hi2cx, MPU_ADD, TEMP_OUT_L, 1, &buff[1], 1, 10);

	uint16_t Temp = 0;
	Temp = ((buff[0]<< 8) + buff[1]);

	*Temperature = ((Temp - ROOM_TEMP_OFFSET)/TEMP_SENS) + 21;

}


/**
 	 * @brief Mesure acceleration
 	 * @Note Cette fonction permet de lire l'acceleration du capteur sur les axes x,y et z puis de l'enregistrer dans une variable globale de type double
	 * @param hi2cx Pointeur vers une structure I2C qui contient l'information de configuration pour un i2c particulier
	 * @param Acceleration Pointeur vers une zone mémoire de type double contenant l’information d'accélération
	 * @retval None
	 */

void Measure_A(I2C_HandleTypeDef* hi2cx,double* Acceleration){
	uint8_t buff[6];
	HAL_I2C_Mem_Read ( hi2cx, MPU_ADD, ACCEL_XOUT_H, 1, &buff[0], 6, 10);


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

/**
 	 * @brief Mesure vitesse angulaire
 	 * @Note Cette fonction permet de lire la vitesse angulaire du capteur sur les axes x,y et z puis de l'enregistrer dans une variable globale de type double
	 * @param hi2cx Pointeur vers une structure I2C qui contient l'information de configuration pour un i2c particulier
	 * @param tableau_donnee_utiles Pointeur vers une zone mémoire de type double contenant l’information de vitesse angulaire
	 * @retval None
	 */

void Measure_Vitesse_angulaire(I2C_HandleTypeDef* hi2cx, double* tableau_donnee_utiles)
{
	uint8_t buffer[6];
	uint8_t plage[1];
	uint16_t gyro_x;
	uint16_t gyro_y;
	uint16_t gyro_z;
	int p = 0;

	if( HAL_I2C_Mem_Read( hi2cx, MPU_ADD, GYRO_XOUT_H, 1, &buffer[0], 6, 10) != HAL_OK){
		printf("probleme lecture donnees gyro \r\n");
		Error_Handler();
	}

	if( HAL_I2C_Mem_Read( hi2cx, MPU_ADD, GYRO_CONFIG, 1, &plage[0], 1, 10) != HAL_OK){
		printf("probleme lecture plage echelle gyro \r\n");
		Error_Handler();
	}

	plage[0] = ((plage[0])&(0b11000))>>3;

                if(plage[0]==00)
		{
                    p =250;
                }
                else if(plage[0]==10)
		{
                    p =500;
                }
                else if(plage[0]==01)
		{
                    p =1000;
                }
                else {
                    p =2000;
                }


	gyro_x = (uint16_t)((buffer[0]<<8) + buffer[1]);
	gyro_y = (uint16_t)((buffer[2]<<8) + buffer[3]);
	gyro_z = (uint16_t)((buffer[4]<<8) + buffer[5]);

	tableau_donnee_utiles[0] =  gyro_x * p / 32767.0;
	tableau_donnee_utiles[1] =  gyro_y * p / 32767.0;
	tableau_donnee_utiles[2] =  gyro_z * p / 32767.0;
}




/**
 	 * @brief Mesure champ magnétique
 	 * @Note Cette fonction permet de lire la valeur du champ magnétique selon un repère d'axes x,y et z puis de l'enregistrer dans une variable globale de type double
	 * @param hi2cx Pointeur vers une structure I2C qui contient l'information de configuration pour un i2c particulier
	 * @param tableau_donnee_utiles Pointeur vers une zone mémoire de type double contenant l’information de champ magnétique
	 * @retval None
	 */

void Measure_M(I2C_HandleTypeDef* p_hi2c1,double* mag){

		int16_t x;
		int16_t y;
		int16_t z;
		int16_t asax;
		int16_t asay;
		int16_t asaz;

		uint8_t buffer[6];
		uint8_t asa[3];
		uint8_t drdy[1];


		HAL_I2C_Mem_Read(p_hi2c1,MAGNETO_ADD,AK8963_ST1,1,drdy,1,20);
		drdy[0]=((drdy[0])&(00000001));
		if(drdy[0]==1){

			if((HAL_I2C_Mem_Read(p_hi2c1,MAGNETO_ADD,AK8963_XOUT_L,1,buffer,7,20)==HAL_OK)
					&(HAL_I2C_Mem_Read(p_hi2c1,MAGNETO_ADD,AK8963_ASAX,1,asa,3,20)==HAL_OK)){


						x = (uint16_t)((buffer[1]<<8)+ buffer[0]);
						y = (uint16_t)((buffer[3]<<8)+ buffer[2]);
						z = (uint16_t)((buffer[5]<<8)+ buffer[4]);
						asax=asa[0];
						asay=asa[1];
						asaz=asa[2];
						mag[0] = (x*((((asax-128)*0.5)/128)+1)*4912)/32760;
						mag[1] = (y*((((asay-128)*0.5)/128)+1)*4912)/32760;
						mag[2] = (z*((((asaz-128)*0.5)/128)+1)*4912)/32760;
			}
		}
}

