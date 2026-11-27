/*
 * diffdrive_pico_firmware — protections batterie, arret d'urgence et
 * drapeaux d'etat.
 *
 * Logique pure, sans acces materiel : compilee telle quelle sur l'hote par
 * test/test_safety.c. Le pilote INA226 fournit les mesures, main.c applique
 * la decision (inhibition des moteurs).
 *
 * Defauts VERROUILLES : une fois declenches, ils restent actifs, moteurs
 * freines, jusqu'a une commande 's' explicite de l'hote ET la disparition
 * de la cause. Une consigne 'v' ou 'o' ne les efface jamais.
 */
#ifndef DIFFDRIVE_PICO_SAFETY_H
#define DIFFDRIVE_PICO_SAFETY_H

#include <stdbool.h>
#include <stdint.h>

/* Champ <flags> de la trame S, en decimal. Contrat du protocole v2 :
 * ne jamais renumeroter un bit existant. */
#define FLAG_LINK_LOST          0x01u   /* chien de garde 300 ms declenche */
#define FLAG_OVERCURRENT        0x02u   /* surintensite, verrouille        */
#define FLAG_UNDERVOLTAGE       0x04u   /* sous-tension, verrouille        */
#define FLAG_NO_BATTERY         0x08u   /* tension < VBAT_PRESENT_MV       */
#define FLAG_IMU_ABSENT         0x10u   /* BNO055 absent, gyro/accel a 0   */
#define FLAG_INA_ABSENT         0x20u   /* INA226 absent, v/i a zero       */
#define FLAG_TEST_MODE          0x40u   /* mode de test actif (t 1..4)     */
#define FLAG_MOTORS_INHIBITED   0x80u   /* moteurs interdits par un defaut */
#define FLAG_ESTOP              0x100u  /* arret d'urgence, verrouille     */

typedef struct {
    bool    ina_present;
    int32_t vbat_mv;
    int32_t ibat_ma;
    bool    alert_fired;        /* interruption ALERT survenue          */
    bool    alert_pin_active;   /* broche ALERT encore basse            */
    bool    estop_active;       /* entree AU a l'etat actif (brut)      */
} safety_in_t;

typedef struct {
    uint32_t latched;           /* FLAG_OVERCURRENT | FLAG_UNDERVOLTAGE
                                   | FLAG_ESTOP                          */
    uint32_t oc_ms;             /* duree passee au-dessus du seuil soutenu */
    uint32_t uv_ms;             /* duree passee sous le seuil de tension   */
    uint32_t estop_on_ms;       /* duree continue AU actif                 */
    uint32_t estop_off_ms;      /* duree continue AU au repos              */
    bool     no_battery;
    bool     inhibit;           /* decision : moteurs interdits         */
} safety_t;

void safety_init(safety_t *s);

/* Une fois par periode de controle. Renvoie s->inhibit. */
bool safety_update(safety_t *s, const safety_in_t *in, uint32_t dt_ms);

/* Demande d'effacement (commande 's'). N'efface un defaut que si sa cause
 * a disparu. Renvoie les defauts encore verrouilles. */
uint32_t safety_try_clear(safety_t *s, const safety_in_t *in);

/* Drapeaux FLAG_OVERCURRENT, FLAG_UNDERVOLTAGE, FLAG_ESTOP, FLAG_NO_BATTERY,
 * FLAG_INA_ABSENT et FLAG_MOTORS_INHIBITED. Les autres bits sont poses
 * par main.c. */
uint32_t safety_flags(const safety_t *s, const safety_in_t *in);

#endif /* DIFFDRIVE_PICO_SAFETY_H */
