#include "radio.h"

/*
 * programme de capture des front montant et descandant.
 *
 * l objectif est de capturer un front, identifier a quel channel il appartient et si il est montant ou descandant puis de mesurer la largeur du pulse
 *
 *
 * */

#include "task.h"
#include "FreeRTOS.h"
#include "stm32f4xx_hal.h"
#include "timebase.h"







static bool is_rising[RC_CH_COUNT];        //identifier si le front est montant ou descandant

static uint32_t t_rise[RC_CH_COUNT];       //memorisation des instant rising front

 //const radio_config_t default_config = {RC_CH_COUNT,2000, 1000, 100000};        //configuration a mettre dans le main

static radio_config_t cfg_radio;

static volatile radio_raw_t pulses;                //pulse et le validite

static radio_status_t status;             //determintaion de la validite

static uint32_t expected_mask;

static volatile uint32_t agg_mask = 0U;




#ifdef USE_FREERTOS
static TaskHandle_t radio_task_to_notify;
#endif




/*========================================== DRIVER ====================================================*/

void radioDriver_reset(){                                               //reinitialisation des variables du driver

	for(int i = 0; i<RC_CH_COUNT; ++i)
	{
		pulses.pulse_us[i] = 0;
		is_rising[i] = true;
		t_rise[i] = 0;
	}

	agg_mask = 0U;
	pulses.last_update_us = 0;
	pulses.valid_mask     = 0U;
	pulses.frame_counter  = 0;

}


void radioDriver_init(const radio_config_t *cfg){                     //creation de la task

	cfg_radio = *cfg;

	if (cfg_radio.num_channels > RC_CH_COUNT) { cfg_radio.num_channels = RC_CH_COUNT; }

	#ifdef USE_FREERTOS
	radio_task_to_notify = NULL;
	#endif

	expected_mask = (1U << cfg_radio.num_channels) -1U;

	radioDriver_reset();

}



/*========================================================= RTOS ======================================*/

#ifdef USE_FREERTOS
void radioRtos_bind_task(TaskHandle_t taskToNotify){

	radio_task_to_notify = taskToNotify;



}
#endif



/*
 * pour mesurer la taille du front on utilise un timer de 16bit, il est regler pour que un bit corresponde a 1us
 * reglage dans cubeMX
 * toutes les 65535us : 0 -> 1 -> 2 -> ... -> 65534 -> 65535 -> 0 -> 1 -> ... chaque tick incremente de 1bit
 * il y a 65535 +1 valeurs possible
 *
 * il faut donc verifier que pulses.pulse_us = t - t_rise[ch] est superieur a 0 et sinon reconstitue le temps ecoulle
 */
static inline uint32_t diff_ticks(uint32_t t, uint32_t t0, uint32_t max_tick){

	return ( t >= t0 ) ? (t - t0) : ((max_tick + 1U - t0) + t);
}









/*ici on ecris le callback qui est directement appele par le timer. en effet a la differance du mpu
*les donnes ne sont pas demander de manere periodique mais arrivent aleatoirement
*
*le callback doit :
*   	- identifier de quel channel vient le front
*   	- si il est montant ou descandant
*   	- calculer la largeur du pulse
*/


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){


	if(htim->Instance != TIM1) {return;}                     //protection, on verifie que le timer concerne soit celui portant sur la radio

	uint32_t ch;                                          //stockage de la current channel 0,1,2,3 logique

	uint32_t tim_channel;                                //stockage de la current channel du driver    acces au registre materiel du timer



	switch(htim->Channel) {                       	 	 //identification du canal

		case HAL_TIM_ACTIVE_CHANNEL_1 : ch = ch1; tim_channel = TIM_CHANNEL_1; break;
		case HAL_TIM_ACTIVE_CHANNEL_2 : ch = ch2; tim_channel = TIM_CHANNEL_2; break;
		case HAL_TIM_ACTIVE_CHANNEL_3 : ch = ch3; tim_channel = TIM_CHANNEL_3; break;
		case HAL_TIM_ACTIVE_CHANNEL_4 : ch = ch4; tim_channel = TIM_CHANNEL_4; break;
		default: return;
	}

	uint32_t tick_max = __HAL_TIM_GET_AUTORELOAD(htim); //nombre de tick du timer ici 65535

	uint32_t t = HAL_TIM_ReadCapturedValue(htim, tim_channel);      //lecteur timeur a l instant du front
	/* !!!!!!!!! verifier que le timer est bien en us tim1 doit etre a 16Mhz et psc = 15
	 * enventuellement mettre le filtre a entre 2 et 4 pour eviter les parasite!!!!!!!!!!!*/


	if (is_rising[ch]) {										     	 //identification d un front montant

		is_rising[ch] = false;                                    	     //le prochain front sera un front descandant

		t_rise[ch] = t;                                                  //memorisation de l instant du front montant


		//on force le channel a capturer un front descandant par securite la prochaine capture sera forcement un falling
		__HAL_TIM_SET_CAPTUREPOLARITY(htim, tim_channel, TIM_INPUTCHANNELPOLARITY_FALLING);

		return;


	}
	else {                                                               //dans le cas ou ce n est pas un front mmontant

		is_rising[ch] = true;                                            //le prochain front seera un front montant

		const uint32_t width =  diff_ticks(t, t_rise[ch], tick_max); // largeur du pulse

		//on force la capture d un front montant
		__HAL_TIM_SET_CAPTUREPOLARITY(htim, tim_channel, TIM_INPUTCHANNELPOLARITY_RISING);


		if(width < cfg_radio.min_pulse_us - 200U) {
			//si la largeur est surprennament petite on ignore le front
			return;
		}

		if(width > cfg_radio.max_pulse_us + 500U) {

			//si le front descandant est loupe on resynchronyse

			agg_mask &= ~(1U << ch);
			return;

		}

		pulses.pulse_us[ch] = width;



		if(cfg_radio.min_pulse_us <= pulses.pulse_us[ch] && cfg_radio.max_pulse_us >= pulses.pulse_us[ch]) //validite du pulse permettant le notification
		{

			agg_mask |= (1U <<ch);

			if((agg_mask & expected_mask) == expected_mask){        //dangereux si un canal se debranche                      //publication/notification

				pulses.frame_counter++;
				pulses.last_update_us = timebase_now_us();
				pulses.valid_mask = agg_mask; //publie
				agg_mask = 0U; //reset agregation


				#ifdef USE_FREERTOS
				if(radio_task_to_notify){	                			 //verification que la tache est active
					BaseType_t hpw = pdFALSE;                                        //token de priorite

					vTaskNotifyGiveFromISR(radio_task_to_notify, &hpw);              //declanchement de l isr uniquement lors de fronts descandant

					portYIELD_FROM_ISR(hpw);                                         //basculement du main sur la tache si haute priorite

				}
				#endif
			}
		}
		else{
			agg_mask &= ~(1U <<ch);
		}
	}
}



/*==================================================== getter ============================================*/


static inline uint32_t irq_save_disable(void){

	uint32_t primask = __get_PRIMASK();     // lit le bit primask du cpu : 0 irq masquable autorise, 1 irq masquable desactive
	__disable_irq();                        //desactivation des irq : interruption request
	return primask;                         //sauvegarde de l etat precedent

}

static inline void irq_restore(uint32_t primask){

	if((primask & 1U) == 0U) { __enable_irq(); }       //si les irq etaient activent avant on les reactive sinon on ne fait rien

}


//aucune irq ne doit modifier les pulses durant la copie, il faut donc les couper avant la copie puis les reactiver.
void radio_get_raw_snapshot(radio_raw_t *out){

	if(!out) {return;}                            //verification que out n est pas NULL

	uint32_t primask = irq_save_disable();        //desactive les irq

	*out = pulses;                                //copie complete du snapchot

	irq_restore(primask);                         //reactivation des irq



}


void radio_update(void){

	const uint32_t now_us = timebase_now_us();

	radio_raw_t snap;

	radio_get_raw_snapshot(&snap);             //copie safe de la struct pulses dans snap

	uint32_t age = (snap.last_update_us == 0U) ? 0xFFFFFFFFU : (now_us - snap.last_update_us);

	status.age_us = age;
	status.frame_counter = snap.frame_counter;
	status.valid_mask = snap.valid_mask;

	status.link_ok = (snap.last_update_us != 0U) && (age <= cfg_radio.timeout_us);



}


bool radio_is_sample_valid(void){

	return ((status.valid_mask & expected_mask) == expected_mask) && status.link_ok;

}



bool radio_link_ok(void){

	return status.link_ok;
}



void radio_get_status(radio_status_t *out){

	if (!out) return;
	*out = status;
}








