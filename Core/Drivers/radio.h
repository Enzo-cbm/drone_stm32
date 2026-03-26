#ifndef RADIO_H
#define RADIO_H

/*========== CODE RECEPTION DES IMPULSION RADIOS =================*/

/*
 *
 * ici on s ocupe uniquement de la gestion de l isr pour recevoir les signaux.
 * l isr doit etre la plus courte possible possible pour ne pas perturber le vol
 *
 * fonction :
 * 	-initialisation radio driver et radio rtos
 * 	reception des donnees
 * 	-determiner si les commandes sont valide

 * 	-failsafe
 *
 * 	l'interpretation des signaux en commande se fait dans le module controle.h/c
 *
 *
 *
 */

#include <stdint.h>
#include <stdbool.h>

#ifdef USE_FREERTOS

#include "FreeRTOS.h"
#include "task.h"
#endif

/*Liste channel*/

typedef enum {

	ch1 = 0,    //roll
	ch2,        //pitch
	ch3,        //yaw
	ch4,        //throttle
	RC_CH_COUNT      //nb channel

} radio_channel_t;




/*config*/

typedef struct {

	uint8_t num_channels;
	uint16_t min_pulse_us;     //largeur minimum
	uint16_t max_pulse_us;     //largeur maximum
	uint32_t timeout_us;


} radio_config_t;



/*snapchot*/
typedef struct {
	uint32_t  pulse_us[RC_CH_COUNT];  //dernieres largeur mesuree
	uint32_t valid_mask;               //1 si pulse_us[i] est valide   0b1011 ch 124 valide 3 invalide
	uint32_t last_update_us;          // timestamp de la derniere publication
	uint32_t frame_counter;           //incrementer a chhaque publication coherante

}radio_raw_t;


/*status*/
typedef struct {

	bool link_ok;             //signal present & pas timeout
	uint32_t valid_mask;       //canaux valide
	uint32_t age_us;          //now - las_update_us
	uint32_t frame_counter;   //pour detecter nouvelle frame

} radio_status_t;




/*====================driver==============================*/

void radioDriver_init(const radio_config_t *config);    /*init driver + etat interne (config, reset bufer)*/

void radioDriver_reset(void);                        /*resetcomplet buffeer flag counter*/

void radio_update(void);                  /* update periodique timeout age*/



/*==========================rtos==============================*/

#ifdef USE_FREERTOS
void radioRtos_bind_task(TaskHandle_t taskToNotify); /* stockage de la task a reveiller */
#endif


/*==========================getter==============================*/



void radio_get_raw_snapshot(radio_raw_t *out);          /* Copie un snapshot RAW cohérent */

bool radio_is_sample_valid(void);                    /* Raccourci : true si link_ok ET tous les canaux requis valides (selon cfg->num_channels) */





bool radio_link_ok(void);                            /* Raccourci : true si link_ok */

void radio_get_status(radio_status_t *out);                /* Copie le status (validité + fraîcheur) */





#endif
