#ifndef IMU_H
#define IMU_H


#include <stdint.h>

#ifdef USE_FREERTOS
#include "FreeRTOS.h"
#include "task.h"
#endif



//  FAIRE LE TEST WHO AM I DE L IMU!!!!!!!!!!!!!!!!!!!!

typedef enum {

	euler_roll = 0,
	euler_pitch,
	euler_yaw,
	angle_count               //=3

} euler_angle_t;          //index des differant axes autour du quel tourne le drone


typedef enum {

	axis_x = 0,
	axis_y,
	axis_z,
	axis_count  //=3

}axis_t;                 //index des differant axes sur les quels se deplace le drone

typedef enum {

	calib_not_done = 0,
	calib_running,
	calib_done,
}calibration_t;



typedef struct {

	float accel_raw[axis_count];
	float gyro_raw[axis_count];
	int32_t temperature;

	float accel_offset[axis_count];
	float gyro_offset[axis_count];

	float accel_lisse[axis_count];
	float gyro_lisse[axis_count];

	float accel[axis_count];
	float gyro[axis_count];
	float angle[angle_count];

}imu_sample_t;


/*============== DECLARATIONS DES FONCTIONS ===============*/



void init_MPU(void);                            /*---initialisation mpu----*/



#ifdef USE_FREERTOS

void MPU_RtosInit( TaskHandle_t taskToNotify); //initialisation du mpu (horloge i2c, temperature, vitesse angulaire accel, filtre


#endif


/** Lance une lecture non-bloquante (DMA) de 14 octets depuis 0x3B.
 *  Retourne pdTRUE si le lancement a réussi, sinon pdFALSE (bus busy, mutex timeout, etc.)
 */
BaseType_t MPU_StartReadDMA(void);



/** À appeler dans la tâche IMU après notification (ou périodiquement).
 *  Parse le buffer reçu et met à jour l’échantillon courant.
 */
void MPU_ProcessLatest(void);


const imu_sample_t *MPU_GetSample(void);   /** Accès lecture à la dernière mesure stable (double-buffer). */



void MPU_CalibStart();                     /*calibration imu gestion des offset*/


uint8_t MPU_IsCalibrated(void);




#endif

////////////////////////////////////////////




