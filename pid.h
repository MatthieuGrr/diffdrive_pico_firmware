/*
 * diffdrive_pico_firmware — PID de vitesse, une instance par roue.
 *
 * Erreur en ticks/s, sortie en unites de rapport cyclique (+/- DUTY_MAX).
 * Aucune constante physique : le firmware ignore ce qu'un tick represente.
 *
 * Deux details qui comptent sur un asservissement de vitesse :
 *  - le terme derive est calcule sur la MESURE, pas sur l'erreur. Un
 *    echelon de consigne ne produit donc pas de pic de derivee.
 *  - l'anti-emballement combine integration conditionnelle (on cesse
 *    d'integrer dans le sens qui aggrave une saturation deja atteinte) et
 *    ecretage dur de la contribution integrale.
 */
#ifndef DIFFDRIVE_PICO_PID_H
#define DIFFDRIVE_PICO_PID_H

#include <stdbool.h>

typedef struct {
    float kp, ki, kd, kff;
    float out_min, out_max;

    float integ;        /* somme des err*dt, en ticks (avant multiplication) */
    float prev_meas;
    bool  primed;

    /* Derniere decomposition, exposee pour le mode de test t 4. */
    float last_err;
    float last_out;
    bool  saturated;
} pid_t;

void  pid_init(pid_t *p, float kp, float ki, float kd, float kff,
               float out_min, float out_max);
void  pid_set_gains(pid_t *p, float kp, float ki, float kd);
void  pid_reset(pid_t *p);
float pid_update(pid_t *p, float setpoint, float measurement, float dt_s);

#endif /* DIFFDRIVE_PICO_PID_H */
