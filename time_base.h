#ifndef TIMEBASE_H
#define TIMEBASE_H



void timebase_init(void);
uint32_t timebase_now_us(void);
float timebase_dt_s(uint32_t *last_us);   // calcule dt depuis last_us et met à jour last_us



#endif
