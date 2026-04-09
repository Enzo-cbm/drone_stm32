
#ifndef ATTITUDE_H
#define ATTITUDE_H

#include "imu.h"
#include "timebase.h"
#include <stdbool.h>


/*
 * de imu.h attitude.h peut utiliser les structures suivantes
 *
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

}axis_t;
 *
 */

typedef enum {
	quat_index_w = 0,
	quat_index_x,
	quat_index_y,
	quat_index_z,
	quat_count,
}quaternion_index_t;


typedef struct {
	float quat_w;
	float quat_x;
	float quat_y;
	float quat_z;
}quaternion_t;

typedef struct {
	quaternion_t quat;
	float euler_angle[angle_count];
	bool is_attitude_valid;


} attitude_t;


typedef struct {
	float dt;
	float norme_quat;
	float gyro_temp[axis_count];
} is_valid_attitude_t;





/////////////////////////////////////initialisation et reset ///////////////////////////////////////////
void attitude_init(attitude_t *att);  //initialise le module : angles, quaternions, filtre




//////////////////////////////////////optention et validation des donnes///////////////////////////////////////
 //reception des donnes imu, correction/conversion, mise a jour de l attitude
void attitude_update(const imu_sample_t *imu, attitude_t *att);


bool attitude_is_valid(void);












#endif



