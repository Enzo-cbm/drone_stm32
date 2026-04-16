#ifndef PID_H
#define PID_H

#include "imu.h"
#include "set_point.h"
#include "state.h"
#include "timebase.h"

#include <stdbool.h>

#define angle_rp_count 2







typedef struct {

	float correct_PID_w[angle_count];
	float is_pid_valid;


}consigne_pid_t;
















/*
 * d/dt (erreur) = d/dt (counsigne - mesure) = d/dt (w_consigne) - d/dt (gyro)
 *
 * si la consigne s eloigne d un coup de la mesure on creer un gros spike correctif
 * entrainant un comportement nerveux voir des oscillation : derivative kick
 *
 *
 * comme correctif_D = kd*d/dt(erreur) = kd*(w_consigne - gyro)/dt est cence amortir le mouvement
 *
 * ici je prendrai correctif_d = - kp.gyro    on obtien ainsi un terme proportionel a une derive angulaire
 *
 * ce qui permet d amortire directement la vitesse angulaire
 *
 *
 * le giro donne w = d/dt teta c est donc deja une derivee pas besoin de diviser une seconde fois par dt
 *
 * de plus le gyro est plus rapide que  la derivee sur les angles car il n y
 *
 * de filtre mahony applique dessus
 *
 */


/*
 *
 * sur le PI_teta pas de terme derive : dt mise a part il reviendrait a doubler le terme derive
 *
 *
 *
 *
 */



void pid_init();

void update_PID_w_teta(const consigne_t *cons, consigne_pid_t *cons_pid, const attitude_t *att);


#endif




