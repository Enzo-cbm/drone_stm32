#include "timebase.h"
#include "attitude.h"
#include "imu.h"
#include <math.h>





static is_valid_attitude_t validation;

static bool init_angle_gyro;


/////////////////////////////////////////////////convertion////////////////////

static const float RAD_TO_DEG = 57.2957795f;
static const float DEG_TO_RAD = 0.01745329f;



///////////////////////////////////////////////coefficiants mahonny///////////////////////////


static const struct {

	float kp_fort;
	float kp_moyen;
	float kp_faible;
	float ki;
	//float correction_accel;

} pid_gain_mahonny = {
	.kp_fort = 2.0f,
	.kp_moyen = 0.8f,
	.kp_faible = 0.0f,
	.ki = 0.005f,
	//.correction_accel = 0.05f,
};

static struct {   //correction integrale

	float integralFBx;
	float integralFBy;
	float integralFBz;
	float leak;
	float limit;
} err_integrale_mahonny = {
		.integralFBx = 0.0f,
		.integralFBy = 0.0f,
		.integralFBz = 0.0f,
		.leak = 0.9995f,
		.limit = 0.2f,
};


static  uint32_t last_us_dt;
static  quaternion_t last_valid_quat;



static void attitude_reset(attitude_t *att){

	att->quat.quat_w = 1.0f;
	att->quat.quat_x = 0.0f;
	att->quat.quat_y = 0.0f;
	att->quat.quat_z = 0.0f;

	last_valid_quat = att->quat;

	att->is_attitude_valid = true;

	att->euler_angle[euler_roll] = 0.0f;
	att->euler_angle[euler_pitch] = 0.0f;
	att->euler_angle[euler_yaw] = 0.0f;

	err_integrale_mahonny.integralFBx = 0.0f;
	err_integrale_mahonny.integralFBy = 0.0f;
	err_integrale_mahonny.integralFBz = 0.0f;

	last_us_dt = timebase_now_us();

	init_angle_gyro = false;

	for (int i = axis_x; i < axis_count; ++i) {
	    att->accel_norm[i] = 0.0f;
	    att->gyro_cor[i]  = 0.0f;
	    att->accel[i] = 0.0f;
	    att->gyro[i] = 0.0f;
	}

	validation.dt = 0.0f;
	validation.norme_quat = 1.0f;

}




void attitude_init(attitude_t *att){

	attitude_reset(att);

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








static void get_euler_angles_from_quaternion(attitude_t *att){

	att->euler_angle[euler_roll] = atan2f(2.0f*(att->quat.quat_w * att->quat.quat_x + att->quat.quat_y * att->quat.quat_z), 1.0f - 2.0f*(att->quat.quat_x * att->quat.quat_x + att->quat.quat_y * att->quat.quat_y))*RAD_TO_DEG;
	float s = 2.0f * (att->quat.quat_w * att->quat.quat_y - att->quat.quat_z * att->quat.quat_x);

	if(s > 1.0f) {s = 1.0f; }
	if(s < -1.0f) { s = -1.0f; }

	att->euler_angle[euler_pitch] = asinf(s) * RAD_TO_DEG;

	att->euler_angle[euler_yaw] = atan2f(2.0f*(att->quat.quat_w * att->quat.quat_z + att->quat.quat_x * att->quat.quat_y), 1.0f - 2.0f * (att->quat.quat_y * att->quat.quat_y + att->quat.quat_z * att->quat.quat_z)) * RAD_TO_DEG;

}







static inline float clamp(float val, const float limit){
	if (val > limit) {return limit;}
	if (val < -limit) {return -limit;}
	return val;
}





/*amelioration :
 * correction plus forte si a ~1g sinon moins forte                        ok
 * continuer au gyro si accel invalide                                     ok
 * borner ou faire fuire Ki.(S e.dt)                                       ok
 * meilleur test pour l accel : if (accel_norm > min && accel_norm < max)  ok
 * magneto pour le yaw
 * low pass en amont sur ll accel                                          ok
 */

//dans attidue_init definir la premier last_us
static void get_angle_mahony_filter(attitude_t *att){


	float dt = timebase_dt_s(&last_us_dt);
	validation.dt = dt;

	//normalisation de l acceleration a-> = g-> + a.lineaire->
	float accel_norm = sqrtf(att->accel[axis_x]*att->accel[axis_x] + att->accel[axis_y]*att->accel[axis_y] + att->accel[axis_z]*att->accel[axis_z]);







	// estimation de l orientation du vecteur g->
	float gx = 2.0f * (att->quat.quat_x * att->quat.quat_z - att->quat.quat_w * att->quat.quat_y);
	float gy = 2.0f * (att->quat.quat_w * att->quat.quat_x + att->quat.quat_y * att->quat.quat_z);
	float gz = att->quat.quat_w * att->quat.quat_w - att->quat.quat_x * att->quat.quat_x - att->quat.quat_y * att->quat.quat_y + att->quat.quat_z * att->quat.quat_z;

	float ex = 0.0f;
	float ey = 0.0f;
	float ez = 0.0f;

	float kp = pid_gain_mahonny.kp_faible;





	if (accel_norm > 1e-6f)
	{
		float accel_error = fabsf(accel_norm - 1.0f);
		if ( accel_error < 0.1f ) {kp = pid_gain_mahonny.kp_fort;}        //acceleration fiable : |accel_norm - 1| < 0.1 => correctif fort
		else if ( accel_error < 0.3f ) {kp = pid_gain_mahonny.kp_moyen;}  //acceleration peu fiable : 0.1< |accel_norm - 1| < 0.3 => correctif moyen
		else {kp = pid_gain_mahonny.kp_faible;} //acceleration non fiable : 0.3< |accel_norm - 1| => correctif nul

		float accel_inv_norm = 1.0f/accel_norm;


		att->accel_norm[axis_x] = att->accel[axis_x] * accel_inv_norm;
		att->accel_norm[axis_y] = att->accel[axis_y] * accel_inv_norm;
		att->accel_norm[axis_z] = att->accel[axis_z] * accel_inv_norm;

		//erreur = a.mesure X g
		ex = (att->accel_norm[axis_y] * gz - att->accel_norm[axis_z] * gy);
		ey = (att->accel_norm[axis_z] * gx - att->accel_norm[axis_x] * gz);
		ez = (att->accel_norm[axis_x] * gy - att->accel_norm[axis_y] * gx);


	}

	//integrale :(S e.dt) on ajoute un leak de l'integrale et une borne large

	err_integrale_mahonny.integralFBx *= err_integrale_mahonny.leak;  //leak
	err_integrale_mahonny.integralFBy *= err_integrale_mahonny.leak;
	err_integrale_mahonny.integralFBz *= err_integrale_mahonny.leak;


	err_integrale_mahonny.integralFBx += pid_gain_mahonny.ki * dt * ex;  //integrale
	err_integrale_mahonny.integralFBy += pid_gain_mahonny.ki * dt * ey;
	err_integrale_mahonny.integralFBz += pid_gain_mahonny.ki * dt * ez;

	err_integrale_mahonny.integralFBx = clamp(err_integrale_mahonny.integralFBx, err_integrale_mahonny.limit); //borne
	err_integrale_mahonny.integralFBy = clamp(err_integrale_mahonny.integralFBy, err_integrale_mahonny.limit);
	err_integrale_mahonny.integralFBz = clamp(err_integrale_mahonny.integralFBz, err_integrale_mahonny.limit);


	//PI du gyro  w.corr = w + Kp.e + Ki.(S e.dt)

	att->gyro_cor[axis_x] = att->gyro[axis_x];
	att->gyro_cor[axis_y] = att->gyro[axis_y];
	att->gyro_cor[axis_z] = att->gyro[axis_z];

	//corection gyro
	att->gyro_cor[axis_x] += kp * ex + err_integrale_mahonny.integralFBx;
	att->gyro_cor[axis_y] += kp * ey + err_integrale_mahonny.integralFBy;
	att->gyro_cor[axis_z] += kp * ez + err_integrale_mahonny.integralFBz;


	//integration quaternion
	quaternion_t omega;
	omega.quat_w = 0.0f;
	omega.quat_x = att->gyro_cor[axis_x] * DEG_TO_RAD;
	omega.quat_y = att->gyro_cor[axis_y] * DEG_TO_RAD;             //verifier que le gyro est en degre/seconde
	omega.quat_z = att->gyro_cor[axis_z] * DEG_TO_RAD;

	quaternion_t quat_dot = quat_multiply(&att->quat, &omega); //q˙ = q⊗ω/2
	quat_dot.quat_w *= 0.5f;
	quat_dot.quat_x *= 0.5f;
	quat_dot.quat_y *= 0.5f;
	quat_dot.quat_z *= 0.5f;

	att->quat.quat_w += quat_dot.quat_w * dt;     //q=q+q˙dt
	att->quat.quat_x += quat_dot.quat_x * dt;
	att->quat.quat_y += quat_dot.quat_y * dt;
	att->quat.quat_z += quat_dot.quat_z * dt;

	att->quat = quat_normalize(&att->quat, &last_valid_quat);
	validation.norme_quat = sqrtf(
			att->quat.quat_w * att->quat.quat_w +
			att->quat.quat_x * att->quat.quat_x +
			att->quat.quat_y * att->quat.quat_y +
			att->quat.quat_z * att->quat.quat_z
		);



}











//a changer lorsqu il y aura le magneto


static void calcul_angle_fusion(attitude_t *att) {


	// Première mesure : initialiser directement avec l'accéléromètre
	if (!init_angle_gyro) {

		init_angle_gyro = true;

		// On met aussi les angles de sortie pour éviter le saut

		 float roll  = atan2f( att->accel[axis_y], att->accel[axis_z]) ;
		 float pitch = atan2f( -att->accel[axis_x], sqrtf(att->accel[axis_y]*att->accel[axis_y] + att->accel[axis_z]*att->accel[axis_z])) ;
		 float yaw  = 0.0f;  //a changer lorsqu il y aura le magneto


		att->euler_angle[euler_roll] = roll * RAD_TO_DEG ;
		att->euler_angle[euler_pitch]= pitch  * RAD_TO_DEG ;
		att->euler_angle[euler_yaw]  = yaw * RAD_TO_DEG ;


		float cr = cosf(roll * 0.5f);
		float sr = sinf(roll * 0.5f);
        float cp = cosf(pitch * 0.5f);
	    float sp = sinf(pitch * 0.5f);
	    float cy = cosf(yaw * 0.5f);
	    float sy = sinf(yaw * 0.5f);

	    att->quat.quat_w = cr * cp * cy + sr * sp * sy;
	    att->quat.quat_x = sr * cp * cy - cr * sp * sy;
	    att->quat.quat_y = cr * sp * cy + sr * cp * sy;
	    att->quat.quat_z = cr * cp * sy - sr * sp * cy;

	    att->quat = quat_normalize(&att->quat, &last_valid_quat);
	    last_valid_quat = att->quat;

	    validation.dt = 0.001f;
	    validation.norme_quat = 1.0f;
	    att->is_attitude_valid = true;

	    last_us_dt = timebase_now_us();

	    return;

	}

	get_angle_mahony_filter(att);


}


















//verifier les test de validite pour les valeur errone et mettre des constante
void attitude_is_valid(attitude_t *att){


	if(validation.dt > 0.01f || validation.dt < 0.0005f)
	{
		att->is_attitude_valid = false;
	}

	else if (fabsf(validation.norme_quat - 1.0f )> 0.01f)
	{
		att->is_attitude_valid = false;
	}
	else if(fabsf(att->gyro_cor[axis_x]) > 1000.0f || fabsf(att->gyro_cor[axis_y]) > 1000.0f || fabsf(att->gyro_cor[axis_z]) > 1000.0f )
	{
		att->is_attitude_valid = false;
	}else {
		att->is_attitude_valid = true;
	}

}







void attitude_update(const imu_sample_t *imu, attitude_t *att){

	for (int i = axis_x; i < axis_count; ++i) {
	    att->gyro[i] = imu->gyro[i];
	    att->accel[i] = imu->accel[i];
	}

	if(!imu->calib_end){
		att->is_attitude_valid = false;
		return;
	}

	calcul_angle_fusion(att);

	attitude_is_valid(att);


	if(att->is_attitude_valid)
	{
		get_euler_angles_from_quaternion(att);
	}

}














