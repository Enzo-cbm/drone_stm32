#include "set_point.h"
#include "attitude.h"

#include <string.h>
#include <math.h>



static consigne_t consigne;



static const struct {
	float low_lim;
	float down_mid_lim;
	float upper_mid_lim;
	float upper_lim;
	float max_degre_sec;
	float min_degre_sec;
	float throttle_lim;
	float yaw_deadband;
	float yaw_hold_gain;
	float coef_stab;
} var_consigne = {
		.low_lim       = 1000.0f,
		.down_mid_lim  = 1492.0f,
		.upper_mid_lim = 1508.0f,
		.upper_lim     = 2000.0f,
		.max_degre_sec = 95.0f,
		.min_degre_sec = -95.0f,
		.throttle_lim  = 1050.0f,
		.yaw_deadband  = 2.0f,
		.yaw_hold_gain = 2.0f,
		.coef_stab     = 2.0f,
};

static float yaw_target;


//maping des pulse radio -> degre/sec : [1000, 2000] -> [min_degre_sec , max_degre_sec ]
static inline float map_f(float x, float in_min, float in_max, float out_min, float out_max){

	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min ;
}




static void consigne_reset(void){

	memset(&consigne, 0, sizeof(consigne));
	yaw_target = attitude_get()->euler_angle[euler_yaw];

}


void consigne_init(void){
	consigne_reset();
}


static void set_point_from_radio_roll_pitch(const radio_raw_t *radio_pulses, const attitude_t *attitude_1){

	//ROLL
	if (radio_pulses->pulse_us[ch1] > var_consigne.upper_mid_lim) {
		consigne.cmd[ROLL] = map_f(radio_pulses->pulse_us[ch1], var_consigne.upper_mid_lim, var_consigne.upper_lim, 0, var_consigne.max_degre_sec);
	} else if (radio_pulses->pulse_us[ch1] < var_consigne.down_mid_lim) {
		consigne.cmd[ROLL] = map_f(radio_pulses->pulse_us[ch1], var_consigne.low_lim, var_consigne.down_mid_lim, var_consigne.min_degre_sec, 0);
	} else {
		consigne.cmd[ROLL] = 0.0f;
	}

	//PITCH inverse
	if (radio_pulses->pulse_us[ch2] > var_consigne.upper_mid_lim) {
			consigne.cmd[PITCH] = map_f(radio_pulses->pulse_us[ch2], var_consigne.upper_mid_lim, var_consigne.upper_lim, 0, var_consigne.min_degre_sec);
		} else if (radio_pulses->pulse_us[ch2] < var_consigne.down_mid_lim) {
			consigne.cmd[PITCH] = map_f(radio_pulses->pulse_us[ch2], var_consigne.low_lim, var_consigne.down_mid_lim, var_consigne.max_degre_sec, 0);
		} else {
			consigne.cmd[PITCH] = 0.0f;
		}

	consigne.cmd[ROLL]  -= attitude_1->euler_angle[euler_roll] * var_consigne.coef_stab ;
	consigne.cmd[PITCH] -= attitude_1->euler_angle[euler_pitch] * var_consigne.coef_stab ;    //attention au bornage, a modifier


}


//amelioration du hold avec le futur magnetometre
static void set_point_from_radio_yaw(const radio_raw_t *radio_pulses, const attitude_t *attitude_1){

	float cmd_yaw = 0.0f;

	//yaw pris en compte uniquement si throttle > seuil

	if (radio_pulses->pulse_us[ch3] > var_consigne.throttle_lim ) {
		if (radio_pulses->pulse_us[ch4] > var_consigne.upper_mid_lim) {
			cmd_yaw = map_f(radio_pulses->pulse_us[ch4], var_consigne.upper_mid_lim, var_consigne.upper_lim, 0, var_consigne.max_degre_sec);
		}else if (radio_pulses->pulse_us[ch4] < var_consigne.down_mid_lim) {
			cmd_yaw = map_f(radio_pulses->pulse_us[ch4], var_consigne.low_lim, var_consigne.down_mid_lim, var_consigne.min_degre_sec, 0);
		} else {
			cmd_yaw = 0.0f;
		}

		//en cas de stick centre on veut un hold
		if (fabsf(cmd_yaw) < var_consigne.yaw_deadband)
		{
			float yaw_error = yaw_target - attitude_1->euler_angle[euler_yaw];

				//normalisation
			if (yaw_error > 180.0f) {yaw_error -= 360.0f; }
			if (yaw_error < -180.0f) {yaw_error += 360.0f; }

			consigne.cmd[YAW] = yaw_error * var_consigne.yaw_hold_gain;

		} else {
			consigne.cmd[YAW] = cmd_yaw;
			yaw_target = attitude_1->euler_angle[euler_yaw];
		}

	}else {
		consigne.cmd[YAW] = 0.0f;
	}


}

static void set_point_from_radio_throttle(const radio_raw_t *radio_pulses) {

	consigne.cmd[THROTTLE] = radio_pulses->pulse_us[ch3];

	if (consigne.cmd[THROTTLE] > 1700.0f) { consigne.cmd[THROTTLE] = 1700.0f;}

}

void set_point_update_from_radio(void){

	radio_raw_t radio_pulses;
	radio_get_raw_snapshot(&radio_pulses);

	const attitude_t *attitude_1 = attitude_get();



	set_point_from_radio_roll_pitch(&radio_pulses, attitude_1);
	set_point_from_radio_yaw(&radio_pulses, attitude_1);
	set_point_from_radio_throttle(&radio_pulses);
}

consigne_t consigne_get(void) {
	return consigne;
}




