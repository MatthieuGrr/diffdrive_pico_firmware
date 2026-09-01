/* Bouchons hote : juste ce que protocol.c consomme du SDK. */
#pragma once
typedef unsigned int uint;
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#define PICO_ERROR_TIMEOUT (-1)
extern int  h_getchar(void);
extern void h_put(const char *s, int len, bool nl, bool cr);
extern uint64_t h_now_us;
static inline int getchar_timeout_us(uint32_t us) { (void)us; return h_getchar(); }
static inline int stdio_put_string(const char *s,int len,bool nl,bool cr){ h_put(s,len,nl,cr); return len; }
static inline uint64_t time_us_64(void) { return h_now_us; }
/* GPIO — bouchons */
#define GPIO_OUT 1
#define GPIO_FUNC_PWM 4
static inline void gpio_init(unsigned p){(void)p;}
static inline void gpio_put(unsigned p,int v){(void)p;(void)v;}
static inline void gpio_set_dir(unsigned p,int d){(void)p;(void)d;}
static inline void gpio_set_function(unsigned p,int f){(void)p;(void)f;}
static inline void gpio_pull_up(unsigned p){(void)p;}
