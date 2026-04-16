#include "pid.h"
#include <math.h>
#include <stdint.h>











typedef struct {

	//PID w : vitesse angulaire
	float erreur_w[angle_count];
	float erreur_integrale_w[angle_count];
	float erreur_derive_w[angle_count];

	float correct_P_w[angle_count];
	float correct_I_w[angle_count];
	float correct_D_w[angle_count];


	uint32_t last_us_w;




	//PID h : angle + vitesse angulaire
	float erreur_teta[angle_rp_count];
	float erreur_integrale_teta[angle_rp_count];


	float correct_P_teta[angle_rp_count];
	float correct_I_teta[angle_rp_count];
	float correct_PI_teta[angle_rp_count];

	float rate_sp[angle_count];




	uint32_t last_us_teta;





} pid_t;











static pid_t pid;







static const struct {
	float kp_w[angle_count];
	float ki_w[angle_count];
	float kd_w[angle_count];

	float max_pid_w;
	float integrale_max_w;

	float integrale_decay_w;
	float integrale_decay_relax_w;
	float integrale_threshold_w;    //seuil
	float w_relax_threshold;
	float deadband_w;



	float kp_teta[angle_rp_count];
	float ki_teta[angle_rp_count];

	float max_pid_teta;
	float integrale_max_teta;

	float integrale_decay_teta;
	float integrale_decay_relax_teta;
	float integrale_threshold_teta;
	float teta_relax_threshold;
	float deadband_teta;


} var_pid = {

	.kp_w                       = {0.475f, 0.475f, 2.75f},
	.kd_w                       = {1.95f, 1.95f, 0.0f},
	.ki_w                       = {0.017f, 0.017f, 0.017f},

	.max_pid_w                  = 400.0f,
	.integrale_max_w            = 1000.0f,           //changer la valeur

	.integrale_decay_w          = 0.98f,             //decroissance generale de l integrale
	.integrale_decay_relax_w    = 0.95f,             //decroissance de l'integrale en cas de faible erreur
	.integrale_threshold_w      = 2.0f,              //seuil anti windup
	.w_relax_threshold          = 2.0f,              //seuil de stabilite en vol
	.deadband_w                 = 0.4f,              //zone morte pour empecher le pid de faire trop de micro correction


	.kp_teta                    = {0.0f, 0.0f},      //changer les valeurs
	.ki_teta                    = {0.0f, 0.0f},      //changer les valeurs

	.max_pid_teta               = 400.0f,            //changer la valeur
	.integrale_max_teta         = 1000.0f,           //changer la valeur

	.integrale_decay_relax_teta = 0.995f,            //decroissance du terme integre en cas de faible erreur
	.integrale_decay_teta       = 0.998f,            //decroissance generale du terme integre
	.integrale_threshold_teta   = 2.0f,              //seuil anti windup
	.teta_relax_threshold       = 2.0f,              //seuil de stabilite en vol
	.deadband_teta              = 0.1f,              //zone morte anti micro correction


};







static inline float clamp(float val, const float limit){
	if (val > limit) {return limit;}
	if (val < -limit) {return -limit;}
	return val;
}












//init_mpu a faire avant init_pid car gyro doit contenire une valeur coherante
//faire l init de cons_pid
void pid_init(){

		for(int i = ROLL ; i < angle_count ; ++i)
		{
			pid.erreur_w[i]             = 0.0f;
			pid.erreur_integrale_w[i]   = 0.0f;
			pid.erreur_derive_w[i]      = 0.0f;

			pid.correct_P_w[i]          = 0.0f;
			pid.correct_I_w[i]          = 0.0f;
			pid.correct_D_w[i]          = 0.0f;
			pid.rate_sp[i]             = 0.0f;

		}

		for (int i = ROLL ; i <= PITCH ; ++i) {


			pid.erreur_teta[i]           = 0.0f;
			pid.erreur_integrale_teta[i] = 0.0f;

			pid.correct_P_teta[i]        = 0.0f;
			pid.correct_I_teta[i]        = 0.0f;
			pid.correct_PI_teta[i]       = 0.0f;

		}

		pid.last_us_w = timebase_now_us();
		pid.last_us_teta = timebase_now_us();

}





//////////////////////////// RELAXATION DE TERME INTEGRE PID_W ET PI_TETA ///////////////////////////////////////////////////




static void decay_and_relaxation_w(const consigne_t *consigne){


	for (int i = ROLL ; i < angle_count ; ++i)
	{
		//l'erreur et l'integrale sont videes si l erreur est faible
		if ( fabsf(pid.erreur_w[i] ) < var_pid.deadband_w )
		{
			pid.erreur_w[i] = 0.0f;
			pid.erreur_integrale_w[i] *= var_pid.integrale_decay_relax_w;
		}

		//si le drone est a l'arret l'integrale est videe
		if (drone_state.flying_state != MARCHE || consigne->cmd_throttle_us < 1050 )
		{
			pid.erreur_integrale_w[i] = 0.0f;
		}

		//relaxation general anti_windup
		if ( fabsf(pid.erreur_w[i]) > var_pid.integrale_threshold_w)
		{
				pid.erreur_integrale_w[i] *= var_pid.integrale_decay_w;
		}
	}
	//relaxation lente si stable en vol
	if ( fabsf(pid.erreur_w[ROLL]) < var_pid.w_relax_threshold  && fabsf( pid.erreur_w[PITCH]) < var_pid.w_relax_threshold  )
	{
		pid.erreur_integrale_w[ROLL] *= var_pid.integrale_decay_relax_w;
		pid.erreur_integrale_w[PITCH] *= var_pid.integrale_decay_relax_w;
	}
}












static void decay_and_relaxation_teta(const consigne_t *consigne){


	for (int i = ROLL ; i <= PITCH ; ++i)
	{
		//l'erreur est videe et l'integrale decroit si l erreur est faible
		if ( fabsf(pid.erreur_teta[i] ) < var_pid.deadband_teta )
		{
			pid.erreur_teta[i] = 0.0f;
			pid.erreur_integrale_teta[i] *= var_pid.integrale_decay_relax_teta;
		}

		//si le drone est a l'arret l'integrale est videe
		if (drone_state.flying_state != MARCHE || consigne->cmd_throttle_us < 1050 )
		{
			pid.erreur_integrale_teta[i] = 0.0f;
		}

		//relaxation general anti_windup
		if ( fabsf(pid.erreur_teta[i]) > var_pid.integrale_threshold_teta)
		{
				pid.erreur_integrale_teta[i] *= var_pid.integrale_decay_teta;
		}
	}
	//relaxation lente si stable en vol
	if ( fabsf(pid.erreur_teta[ROLL]) < var_pid.teta_relax_threshold  && fabsf( pid.erreur_teta[PITCH]) < var_pid.teta_relax_threshold  )
	{
		pid.erreur_integrale_teta[ROLL] *= var_pid.integrale_decay_relax_teta;
		pid.erreur_integrale_teta[PITCH] *= var_pid.integrale_decay_relax_teta;
	}
}





















//////////////////////////////// CALCUL DU CORRECTIF ANGULAIRE ////////////////////////////////////////


//coeficient du rapport entre la commande rate_sp et correctif angulaire

static inline float alpha_light(const float stick_position){

	if (drone_state.flying_mode == pid_mode_stab) {return 0.0f;}
	else {
		float a = fabsf(stick_position);

		a = (a - 0.15f)/0.85f; //deadband de 0.15f

		if ( a < 0.0f ) { a = 0.0f; } //<0.15f mode stab

		if ( a > 1.0f ) { a = 1.0f; } //>1.0 mode horizon

		return a * a * (3.0f - 2.0f * a); //evolution douce de degre 3
	}
}





static void calcul_w_stab_horizon(const consigne_t *cons){

	for (int i = ROLL ; i <= PITCH ; ++i){


			float a = alpha_light(cons->stick_norm[i]);

			pid.rate_sp[i] = pid.correct_PI_teta[i] * (1 - a)  +  a * cons->rate_sp[i] ;

		}
}





static void correctif_pi_teta(const consigne_t *consigne, const attitude_t *att){

	for (int i = ROLL ; i <= PITCH ; ++i){

		pid.erreur_teta[i] = consigne->angle_sp[i] - att->euler_angle[i];

	}

	decay_and_relaxation_teta(consigne);


	float dt = timebase_dt_s(&pid.last_us_teta);

	if (dt < 1e-6f || dt > 0.05f){return;}

	for(int i = ROLL ; i <= PITCH ; ++i)
	{
		pid.correct_P_teta[i] = var_pid.kp_teta[i] * pid.erreur_teta[i];


		pid.erreur_integrale_teta[i] += pid.erreur_teta[i] * dt;
		pid.erreur_integrale_teta[i]  = clamp(pid.erreur_integrale_teta[i] , var_pid.integrale_max_teta);
		pid.correct_I_teta[i]         = var_pid.ki_teta[i] * pid.erreur_integrale_teta[i] ;


		pid.correct_PI_teta[i] = pid.correct_P_teta[i] + pid.correct_I_teta[i];
		pid.correct_PI_teta[i] = clamp(pid.correct_PI_teta[i] , var_pid.max_pid_teta);

	}
}







//mode rate

// w_out = PID_W( w_consigne - w_reel )



//mode horizon : nelange de mode rate et mode ange

/*
 * Si le stick est centre le pid_Θ prend le dessu pour revenir a l'horizontal.
 * Si le stick est tres poussee le pid_w prend le dessus pour passer en mode rate
 *
 */

//w_out = pid_w( pid_Θ( Θ_cmd - Θ_reel )*(1-a) + a*w_stick - w_reel ) = pid_w(w_cmd - w_reel)

//w_cmd = pid_Θ( Θ_cmd - Θ_reel )*(1-a) + a*w_stick

//a~0 au centre
//a~1 stick pousse


//mode stab : a = 0

//w_out = pid_w( pid_Θ( Θ_cmd - Θ_reel )  - w_reel )


void update_PID_w_teta(const consigne_t *cons, consigne_pid_t *cons_pid, const attitude_t *att) {

	if (drone_state.flying_mode == pid_mode_accro)
	{
		for (int i = ROLL ; i < angle_count ; ++i)
		{
			pid.erreur_w[i] = cons->rate_sp[i] - att->gyro[i];
		}
	}else {
		correctif_pi_teta(cons, att);
		calcul_w_stab_horizon(cons);


		pid.erreur_w[ROLL]  = pid.rate_sp[ROLL]  - att->gyro[ROLL];
		pid.erreur_w[PITCH] = pid.rate_sp[PITCH] - att->gyro[PITCH];
		pid.erreur_w[YAW]   = cons->rate_sp[YAW] - att->gyro[YAW];
	}


	float dt = timebase_dt_s(&pid.last_us_w);

	if (dt < 1e-6f || dt > 0.05f){return;}          //verifier ;e bornage de dt

	decay_and_relaxation_w(cons);

	for(int i = ROLL ; i < angle_count ; ++i)
	{
		pid.correct_P_w[i] = var_pid.kp_w[i] * pid.erreur_w[i];


		pid.erreur_integrale_w[i]  += pid.erreur_w[i] * dt;
		pid.erreur_integrale_w[i]   = clamp(pid.erreur_integrale_w[i] , var_pid.integrale_max_w);
		pid.correct_I_w[i]          = var_pid.ki_w[i] * pid.erreur_integrale_w[i] ;


		pid.erreur_derive_w[i]      = att->gyro[i];
		pid.correct_D_w[i]          = - var_pid.kd_w[i] * pid.erreur_derive_w[i];


		cons_pid->correct_PID_w[i]  = pid.correct_P_w[i] + pid.correct_I_w[i] + pid.correct_D_w[i];
		cons_pid->correct_PID_w[i]  = clamp(cons_pid->correct_PID_w[i] , var_pid.max_pid_w);

		}

}















