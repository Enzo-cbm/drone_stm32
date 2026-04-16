#ifndef SET_POINT_H
#define SET_POINT_H

#include "radio.h"
#include "imu.h"





typedef struct {
	float stick_norm[angle_count];
	float rate_sp[angle_count];
	float angle_sp[angle_count];
	float cmd_throttle_us;

}consigne_t;



void consigne_init(consigne_t *consigne);

void set_point_update_from_radio(consigne_t *consigne, const radio_raw_t *radio_pulses, const attitude_t *att);






#endif



