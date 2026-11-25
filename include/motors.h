/*
 * diffdrive_pico_firmware — pilotage du Cytron MDD10A (deux canaux).
 *
 * Commande signe-amplitude : par moteur, une sortie PWM porte l'amplitude
 * et un GPIO DIR porte le sens. PWM a zero = moteur freine (le MDD10A n'a
 * pas d'etat roue libre, voir config.h).
 *
 * Aucune limitation de vitesse, d'acceleration ni de rapport cyclique n'est
 * appliquee ici : la consigne est executee telle quelle. Le seul ecretage
 * est celui de la plage physique +/- DUTY_MAX.
 *
 * INHIBITION : motors_inhibit(true) freine immediatement les deux moteurs et
 * fait ignorer toute consigne jusqu'a motors_inhibit(false). Appelable depuis
 * une interruption (simples ecritures de registres PWM) : c'est ce
 * qu'utilise l'alerte de surintensite de l'INA226.
 */
#ifndef DIFFDRIVE_PICO_MOTORS_H
#define DIFFDRIVE_PICO_MOTORS_H

#include <stdbool.h>
#include <stdint.h>

#define MOT_L 0
#define MOT_R 1
#define MOT_N 2

void motors_init(void);

/* duty dans [-DUTY_MAX, +DUTY_MAX]. Positif = marche avant apres prise en
 * compte de MOT_x_INVERT. Les valeurs hors plage sont ecretees. Sans effet
 * (moteur maintenu freine) tant que les moteurs sont inhibes. */
void motors_set(int side, int duty);

/* PWM a zero sur les deux canaux : les deux moteurs sont freines.
 * C'est la sortie utilisee par le chien de garde et par 's'. */
void motors_stop_all(void);

/* Inhibition de securite, sure en interruption. */
void motors_inhibit(bool on);
bool motors_inhibited(void);

/* Dernier rapport cyclique effectivement applique, avant inversion. */
int motors_get(int side);

/* Valeur de wrap du compteur PWM, pour information dans le log. */
uint16_t motors_pwm_top(void);

#endif /* DIFFDRIVE_PICO_MOTORS_H */
