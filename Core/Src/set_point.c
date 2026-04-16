#include "set_point.h"

#include <string.h>
#include <math.h>




static const struct {
	float low_lim;
	float down_mid_lim;
	float upper_mid_lim;
	float upper_lim;
	float max_degre_sec_roll_pitch;
	float max_degre_sec_yaw;
	float max_angle_deg;
	float throttle_lim;
} var_consigne = {
		.low_lim         = 1000.0f,
		.down_mid_lim    = 1492.0f,
		.upper_mid_lim   = 1508.0f,
		.upper_lim       = 2000.0f,
		.max_degre_sec_roll_pitch = 200.0f,   //95.0f
		.max_degre_sec_yaw = 130.0f,
		.max_angle_deg = 35.0f,
		.throttle_lim    = 1050.0f,

};









static void consigne_reset(consigne_t *consigne){

	memset(consigne, 0, sizeof(*consigne));

}


void consigne_init(consigne_t *consigne){
	consigne_reset(consigne);
}





//maping des pulse radio -> degre/sec : [1000, 2000] -> [min_degre_sec , max_degre_sec ] ou vers -> [min_degre_angle , max_degre_angle ]
static inline float map_f(float x, float in_min, float in_max, float out_min, float out_max){

	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min ;
}





//fonction pour normaliser les pulse [1000, 2000] -> [-1, 1]
static inline float radio_to_stick_norm_deadband(float pulses){

	if(pulses > var_consigne.upper_mid_lim) {

		return map_f(pulses, var_consigne.upper_mid_lim, var_consigne.upper_lim, 0.0f, 1.0f);
	}
	else if( pulses < var_consigne.down_mid_lim ) {
		return map_f(pulses, var_consigne.low_lim, var_consigne.down_mid_lim, -1.0f, 0.0f);

	}else { return 0.0f;}

}









//optenire les conversions des pulses roll et pitch et deg_sec ou en degre
static void set_point_from_radio_roll_pitch(consigne_t *consigne, const radio_raw_t *radio_pulses) {

	consigne->stick_norm[ROLL]  = radio_to_stick_norm_deadband(radio_pulses->pulse_us[ch1]);
	consigne->stick_norm[PITCH] = -radio_to_stick_norm_deadband(radio_pulses->pulse_us[ch2]);  //pitch : axe inverse

	//rate set_point
	consigne->rate_sp[ROLL]     = consigne->stick_norm[ROLL] * var_consigne.max_degre_sec_roll_pitch;
	consigne->rate_sp[PITCH]    = consigne->stick_norm[PITCH] * var_consigne.max_degre_sec_roll_pitch;

	//angle set_point
	consigne->angle_sp[ROLL]     = consigne->stick_norm[ROLL] * var_consigne.max_angle_deg;
	consigne->angle_sp[PITCH]    = consigne->stick_norm[PITCH] * var_consigne.max_angle_deg;

}


//conversion des pulses yaw pas de convesion en angle car pas d'angle absolu comme roll ou pitch
static void set_point_from_radio_yaw(consigne_t *consigne, const radio_raw_t *radio_pulses) {


	if(radio_pulses->pulse_us[ch3] > var_consigne.throttle_lim) {

		consigne->stick_norm[YAW] = radio_to_stick_norm_deadband(radio_pulses->pulse_us[ch4]);
	}else {
	    consigne->stick_norm[YAW] = 0.0f;
	}

	//rate set_point
	consigne->rate_sp[YAW]     = consigne->stick_norm[YAW] * var_consigne.max_degre_sec_yaw;

}




//sauvegarde des pulse throttle tel quel avec une limite a 1700 us pour laisser de la marge au PID
static void set_point_from_radio_throttle(consigne_t *consigne, const radio_raw_t *radio_pulses) {

	//consigne->stick_norm[THROTTLE] = radio_to_stick_norm(radio_pulses->pulse_us[ch3]);

	// limitation throttle pour garder de la marge aux corrections PID
	consigne->cmd_throttle_us = radio_pulses->pulse_us[ch3];
	if (consigne->cmd_throttle_us> 1700.0f) { consigne->cmd_throttle_us = 1700.0f;}


}





void set_point_update_from_radio(consigne_t *consigne, const radio_raw_t *radio_pulses){

	set_point_from_radio_roll_pitch(consigne, radio_pulses);
	set_point_from_radio_yaw(consigne, radio_pulses);
	set_point_from_radio_throttle(consigne, radio_pulses);
}


/*
static inline float radio_to_stick_norm(float pulses) {

	return map_f(pulses, var_consigne.low_lim, var_consigne.upper_lim, 0, 1);
}
*/

