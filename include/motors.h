/*
 * diffdrive_pico_firmware — pilotage des deux DRV8871.
 *
 * Chaque pont est commande par deux PWM (IN1/IN2) issues de la MEME slice,
 * donc parfaitement synchrones. Le mode de decroissance est choisi par la
 * seule constante MOTOR_DECAY_MODE de config.h.
 *
 * Aucune limitation de vitesse, d'acceleration ni de rapport cyclique n'est
 * appliquee ici : la consigne est executee telle quelle. Le seul ecretage
 * est celui de la plage physique +/- DUTY_MAX.
 */
#ifndef DIFFDRIVE_PICO_MOTORS_H
#define DIFFDRIVE_PICO_MOTORS_H

#include <stdint.h>

#define MOT_L 0
#define MOT_R 1
#define MOT_N 2

void motors_init(void);

/* duty dans [-DUTY_MAX, +DUTY_MAX]. Positif = marche avant apres prise en
 * compte de MOT_x_INVERT. Les valeurs hors plage sont ecretees. */
void motors_set(int side, int duty);

/* Coupe les deux ponts en roue libre, quel que soit MOTOR_IDLE_BRAKE.
 * C'est la sortie utilisee par le chien de garde. */
void motors_coast_all(void);

/* Dernier rapport cyclique effectivement applique, avant inversion. */
int motors_get(int side);

const char *motors_decay_name(void);

/* Nombre de pas du compteur PWM, pour information dans le log de demarrage. */
uint16_t motors_pwm_top(void);

#endif /* DIFFDRIVE_PICO_MOTORS_H */
