#ifndef SET_POINT_H
#define SET_POINT_H

#include "radio.h"
#include "imu.h"





typedef struct {
	float cmd[DEGRE_OF_LIBERTY];

}consigne_t;



void consigne_init(consigne_t *consigne, const attitude_t *att);

void set_point_update_from_radio(consigne_t *consigne, const radio_raw_t *radio_pulses, const attitude_t *att);






#endif



