#ifndef SET_POINT_H
#define SET_POINT_H

#include "radio.h"
#include "imu.h"





typedef struct {
	float cmd[DEGRE_OF_LIBERTY];

}consigne_t;



void consigne_init(void);

consigne_t consigne_get(void);
void set_point_update_from_radio(void);






#endif



