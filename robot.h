/*
 * diffdrive_pico_firmware — etat partage du firmware.
 *
 * Une seule structure, une seule instance, un seul fil d'execution : la
 * boucle principale. Les interruptions ne font qu'armer un drapeau, rien
 * d'autre. Pas de verrou necessaire.
 *
 * Index 0 = roue gauche, index 1 = roue droite, partout.
 */
#ifndef DIFFDRIVE_PICO_ROBOT_H
#define DIFFDRIVE_PICO_ROBOT_H

#include <stdbool.h>
#include <stdint.h>

#include "pid.h"

typedef enum {
    CTRL_STOP = 0,      /* moteurs coupes                                  */
    CTRL_VELOCITY,      /* asservissement de vitesse, commande 'v'         */
    CTRL_OPENLOOP       /* rapport cyclique impose, commande 'o'           */
} ctrl_mode_t;

typedef struct {
    ctrl_mode_t ctrl_mode;
    int         test_mode;      /* TEST_NORMAL, TEST_ENCODERS, ...         */

    int32_t     sp_ticks[2];    /* consigne de vitesse, ticks/s            */
    int         ol_duty[2];     /* consigne de boucle ouverte, +/- DUTY_MAX*/
    int         out_duty[2];    /* rapport cyclique reellement applique    */

    pid_t       pid[2];

    uint64_t    last_cmd_us;    /* horodatage de la derniere commande valide*/
    bool        link_lost;      /* chien de garde logiciel declenche       */

    uint32_t    rx_bad;         /* commandes rejetees                      */
    uint32_t    overruns;       /* periodes de controle manquees           */
    bool        imu_present;
} robot_t;

extern robot_t g_robot;

/* Applique la coupure de securite : PWM a zero, PID remis a zero, mode STOP.
 * Utilise par le chien de garde et par la commande 's'. */
void robot_failsafe(void);

#endif /* DIFFDRIVE_PICO_ROBOT_H */
