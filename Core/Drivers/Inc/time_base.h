#ifndef TIMEBASE_H
#define TIMEBASE_H

#include <stdint.h>

/*
 *
 * utiliser un timer 32 bit qui permet 4 294 967 296 ticks soit 4 294 967 296 us = 4294.967296s = 71.58minutes qui est plus long que la dureer de vol
 *
 */



void timebase_init(void);

uint32_t timebase_now_us(void);


float timebase_dt_s(uint32_t *last_us);   // calcule dt depuis last_us et met à jour last_us

void timebase_delay_us(uint32_t us);    //foncyion qui permet d'attendre un certain delay, utile pour les capteurs


#endif
