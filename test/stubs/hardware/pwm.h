#pragma once
#include <stdint.h>
typedef struct { float div; uint16_t wrap; } pwm_config;
extern uint16_t h_lvl[8][2];        /* [slice][canal] */
static inline pwm_config pwm_get_default_config(void){ pwm_config c={1.f,0xFFFF}; return c; }
static inline void pwm_config_set_clkdiv(pwm_config *c,float d){ c->div=d; }
static inline void pwm_config_set_wrap(pwm_config *c,uint16_t w){ c->wrap=w; }
static inline void pwm_init(unsigned s,pwm_config *c,int st){(void)s;(void)c;(void)st;}
static inline void pwm_set_chan_level(unsigned s,unsigned ch,uint16_t l){ h_lvl[s][ch]=l; }
static inline void pwm_set_enabled(unsigned s,int e){(void)s;(void)e;}
static inline unsigned pwm_gpio_to_slice_num(unsigned g){ return (g>>1)&7u; }
static inline unsigned pwm_gpio_to_channel(unsigned g){ return g&1u; }
