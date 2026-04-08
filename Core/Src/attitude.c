#include "timebase.h"
#include "attitude.h"
#include "imu.h"
#include<math.h>





static attitude_t attitude;
static is_valid_attitude_t validation;



/////////////////////////////////////////////////convertion////////////////////

static const float RAD_TO_DEG = 57.2957795f;
static const float DEG_TO_RAD = 0.01745329f;


///////////////////////////////////////////////coefficiants mahonny///////////////////////////


static const struct {

	float kp;
	float ki;
	float correction_accel;

} pid_gain_mahonny = {
	.kp = 2.0f,
	.ki = 0.005f,
	.correction_accel = 0.05f,
};

static struct {

	float integralFBx;
	float integralFBy;
	float integralFBz;
} err_integrale_mahonny = {
		.integralFBx = 0.0f,
		.integralFBy = 0.0f,
		.integralFBz = 0.0f,
};


static  uint32_t last_us_dt;
static  quaternion_t last_valid_quat;



void attitude_reset(void){

	attitude.quat.quat_w = 1.0f;
	attitude.quat.quat_x = 0.0f;
	attitude.quat.quat_y = 0.0f;
	attitude.quat.quat_z = 0.0f;

	last_valid_quat = attitude.quat;

	attitude.is_attitude_valid = true;

	attitude.euler_angle[euler_roll] = 0.0f;
	attitude.euler_angle[euler_pitch] = 0.0f;
	attitude.euler_angle[euler_yaw] = 0.0f;

	err_integrale_mahonny.integralFBx = 0.0f;
	err_integrale_mahonny.integralFBy = 0.0f;
	err_integrale_mahonny.integralFBz = 0.0f;

	last_us_dt = timebase_now_us();



}




void attitude_init(void){

	attitude_reset();



}






//////////////////////////////////////////gestion quaternion fonction privee appartenant au file .c///////////////////////////
static inline quaternion_t quat_multiply (const quaternion_t *a, const quaternion_t *b) {   //a et b ne sont pas modifier donc on les protege de modification et passage par pointeur pour eviter la copie
	quaternion_t res;

	res.quat_w = a->quat_w*b->quat_w - a->quat_x*b->quat_x - a->quat_y*b->quat_y - a->quat_z*b->quat_z;
	res.quat_x = a->quat_w*b->quat_x + a->quat_x*b->quat_w + a->quat_y*b->quat_z - a->quat_z*b->quat_y;
	res.quat_y = a->quat_w*b->quat_y - a->quat_x*b->quat_z + a->quat_y*b->quat_w + a->quat_z*b->quat_x;
	res.quat_z = a->quat_w*b->quat_z + a->quat_x*b->quat_y - a->quat_y*b->quat_x + a->quat_z*b->quat_w;

	return res;

}

// !!!!!!!!!!!!!! initialiser last_valid_quat dans attitude_init !!!!!!!!!!!!!!!!!!!!!!!!
//amelioration : utiliser fast invert sqrt
static inline quaternion_t quat_normalize(const quaternion_t *q, quaternion_t *last_valid_quat) {

	quaternion_t res;

	float norme = sqrtf(q->quat_w*q->quat_w + q->quat_x*q->quat_x + q->quat_y*q->quat_y + q->quat_z*q->quat_z);

	if(norme < 1e-6f)
	    {
			return *last_valid_quat;
	    }

	float inv_norme = 1.0f/norme;

	res.quat_w = q->quat_w*inv_norme;
	res.quat_x = q->quat_x*inv_norme;
	res.quat_y = q->quat_y*inv_norme;
	res.quat_z = q->quat_z*inv_norme;

	*last_valid_quat = res;

	return res;

}

static void get_euler_angles_from_quaternion(void){

	attitude.euler_angle[euler_roll] = atan2f(2.0f*(attitude.quat.quat_w * attitude.quat.quat_x + attitude.quat.quat_y * attitude.quat.quat_z), 1.0f - 2.0f*(attitude.quat.quat_x * attitude.quat.quat_x + attitude.quat.quat_y * attitude.quat.quat_y))*RAD_TO_DEG;
	float s = 2.0f * (attitude.quat.quat_w * attitude.quat.quat_y - attitude.quat.quat_z * attitude.quat.quat_x);

	if(s > 1.0f) {s = 1.0f; }
	if(s < -1.0f) { s = -1.0f; }

	attitude.euler_angle[euler_pitch] = asinf(s) * RAD_TO_DEG;

	attitude.euler_angle[euler_yaw] = atan2f(2.0f*(attitude.quat.quat_w * attitude.quat.quat_z + attitude.quat.quat_x * attitude.quat.quat_y), 1.0f - 2.0f * (attitude.quat.quat_y * attitude.quat.quat_y + attitude.quat.quat_z * attitude.quat.quat_z)) * RAD_TO_DEG;

}

/*amelioration :
 * correction plus forte si a ~1g sinon moins forte
 * borner ou faire fuire Ki.(S e.dt)
 * continuer au gyro si accel invalide
 * meilleur test pour l accel : if (accel_norm > min && accel_norm < max)
 * magneto pour le yaw
 * low pass en amont sur ll accel
 */

//dans attidue_init definir la premier last_us
static void mahony_update(){

	imu_sample_t imu_sample = *MPU_GetSample();

	float dt = timebase_dt_s(&last_us_dt);
	validation.dt = dt;

	//normalisation de l acceleration a-> = g-> + a.lineaire->
	float accel_norm = sqrtf(imu_sample.accel[axis_x]*imu_sample.accel[axis_x] + imu_sample.accel[axis_y]*imu_sample.accel[axis_y] + imu_sample.accel[axis_z]*imu_sample.accel[axis_z]);
	validation.norme_accel = accel_norm;

	if (accel_norm < 1e-6f) {  //utiliser une meilleur protection
	    return;
	}

	float accel_inv_norm = 1.0f/accel_norm;


	imu_sample.accel[axis_x] *= accel_inv_norm;
	imu_sample.accel[axis_y] *= accel_inv_norm;
	imu_sample.accel[axis_z] *= accel_inv_norm;


	// estimation de l orientation du vecteur g->
	float gx = 2.0f * (attitude.quat.quat_x * attitude.quat.quat_z - attitude.quat.quat_w * attitude.quat.quat_y);
	float gy = 2.0f * (attitude.quat.quat_w * attitude.quat.quat_x + attitude.quat.quat_y * attitude.quat.quat_z);
	float gz = attitude.quat.quat_w * attitude.quat.quat_w - attitude.quat.quat_x * attitude.quat.quat_x - attitude.quat.quat_y * attitude.quat.quat_y + attitude.quat.quat_z * attitude.quat.quat_z;


	//erreur = a.mesure X g
	float ex = (imu_sample.accel[axis_y] * gz - imu_sample.accel[axis_z] * gy);
	float ey = (imu_sample.accel[axis_z] * gx - imu_sample.accel[axis_x] * gz);
	float ez = (imu_sample.accel[axis_x] * gy - imu_sample.accel[axis_y] * gx);




	//PI du gyro  w.corr = w + Kp.e + Ki.(S e.dt)


	//integrale :(S e.dt)
	err_integrale_mahonny.integralFBx += pid_gain_mahonny.ki * dt * ex;
	err_integrale_mahonny.integralFBy += pid_gain_mahonny.ki * dt * ey;
	err_integrale_mahonny.integralFBz += pid_gain_mahonny.ki * dt * ez;   //faire un blocage ou un leak de l integrale pour eviter un windup


	//corection gyro
	imu_sample.gyro[axis_x] += pid_gain_mahonny.kp * ex + err_integrale_mahonny.integralFBx;
	imu_sample.gyro[axis_y] += pid_gain_mahonny.kp * ey + err_integrale_mahonny.integralFBy;
	imu_sample.gyro[axis_z] += pid_gain_mahonny.kp * ez + err_integrale_mahonny.integralFBz;

	//integration quaternion

	quaternion_t omega;
	omega.quat_w = 0.0f;
	omega.quat_x = imu_sample.gyro[axis_x] * DEG_TO_RAD;
	omega.quat_y = imu_sample.gyro[axis_y] * DEG_TO_RAD;             //verifier que le gyro est en degre/seconde
	omega.quat_z = imu_sample.gyro[axis_z] * DEG_TO_RAD;

	quaternion_t quat_dot = quat_multiply(&attitude.quat, &omega); //q˙ = q⊗ω/2
	quat_dot.quat_w *= 0.5f;
	quat_dot.quat_x *= 0.5f;
	quat_dot.quat_y *= 0.5f;
	quat_dot.quat_z *= 0.5f;

	attitude.quat.quat_w += quat_dot.quat_w * dt;     //q=q+q˙dt
	attitude.quat.quat_x += quat_dot.quat_x * dt;
	attitude.quat.quat_y += quat_dot.quat_y * dt;
	attitude.quat.quat_z += quat_dot.quat_z * dt;

	attitude.quat = quat_normalize(&attitude.quat, &last_valid_quat);
	validation.norme_quat = sqrtf(
		    attitude.quat.quat_w * attitude.quat.quat_w +
		    attitude.quat.quat_x * attitude.quat.quat_x +
		    attitude.quat.quat_y * attitude.quat.quat_y +
		    attitude.quat.quat_z * attitude.quat.quat_z
		);

	validation.gyro_temp[axis_x] = imu_sample.gyro[axis_x];
	validation.gyro_temp[axis_y] = imu_sample.gyro[axis_y];
	validation.gyro_temp[axis_z] = imu_sample.gyro[axis_z];


}

//verifier les test de validite pour les valeur errone
bool attitude_is_valid(void){


	if(validation.dt > 0.02f || validation.dt < 0.0005f)
	{
		return false;
	}

	else if (fabsf(validation.norme_accel - 1.0f) > 0.5f)
	{
		return false;
	}
	else if (fabsf(validation.norme_quat - 1.0f )> 0.09f)
	{
		return false;
	}
	else if(fabsf(validation.gyro_temp[axis_x]) > 500.0f || fabsf(validation.gyro_temp[axis_y]) > 500.0f || fabsf(validation.gyro_temp[axis_z]) > 500.0f )
	{
		return false;
	}
	else { return true;}

}







void attitude_update(void){

	mahony_update();

	attitude.is_attitude_valid = attitude_is_valid();

	if(attitude.is_attitude_valid)
	{
		get_euler_angles_from_quaternion();
	}
}





//////////////////////////////////////////getter////////////////////////////////
const attitude_t* attitude_get(void){
	return &attitude;
}









