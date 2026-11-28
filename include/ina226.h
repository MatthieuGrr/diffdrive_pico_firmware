/*
 * diffdrive_pico_firmware — INA226, tension et courant batterie, sur I2C1.
 *
 * Le shunt (INA226_SHUNT_UOHM) est en serie sur le + batterie, avant le
 * MDD10A : le courant mesure est celui des deux moteurs reunis (plus le peu
 * que consomme ce qui est en aval du shunt).
 *
 * On lit directement les registres de tension de shunt et de bus, sans
 * passer par le registre de calibration : la conversion est faite ici, en
 * entiers, par les fonctions ina226_*_from_raw ci-dessous (testees sur
 * l'hote).
 *
 * L'INA226 surveille lui-meme le courant : sa sortie ALERT (drain ouvert,
 * active basse, verrouillee) tombe des qu'une mesure depasse OC_TRIP_MA.
 * L'interruption GPIO associee freine les moteurs sans attendre la boucle
 * de controle (voir motors_inhibit).
 */
#ifndef DIFFDRIVE_PICO_INA226_H
#define DIFFDRIVE_PICO_INA226_H

#include <stdbool.h>
#include <stdint.h>

#define INA226_REG_CONFIG       0x00
#define INA226_REG_SHUNT        0x01    /* LSB 2,5 uV, signe        */
#define INA226_REG_BUS          0x02    /* LSB 1,25 mV, non signe   */
#define INA226_REG_MASK_EN      0x06
#define INA226_REG_ALERT_LIM    0x07
#define INA226_REG_MANUF_ID     0xFE    /* 0x5449 « TI »            */
#define INA226_REG_DIE_ID       0xFF    /* 0x2260                   */

#define INA226_MANUF_ID_VALUE   0x5449u
#define INA226_DIE_ID_VALUE     0x2260u

/* Mask/Enable : alerte sur surtension de shunt (= surintensite), verrouillee
 * jusqu'a la relecture du registre, sortie active basse. */
#define INA226_MASK_SOL         0x8000u
#define INA226_MASK_LEN         0x0001u
#define INA226_MASK_AFF         0x0010u     /* drapeau : alerte survenue */

/*
 * Configuration : moyenne de 4 echantillons, conversion shunt 588 us, bus
 * 1,1 ms, mode continu shunt + bus. Une mesure complete toutes les
 * ~6,8 ms : plus de deux par periode de controle, et l'alerte materielle
 * reagit en moins de 7 ms a une surintensite franche.
 *   bit 14 (reserve, lu a 1) | AVG=001 | VBUSCT=100 | VSHCT=011 | MODE=111
 */
#define INA226_CONFIG_VALUE     0x431Fu

typedef struct {
    bool     present;
    int32_t  vbat_mv;       /* tension de bus, mV                      */
    int32_t  ibat_ma;       /* courant, mA, positif = decharge         */
    uint32_t read_errors;
} ina226_t;

/* ---- conversions pures (aucun acces materiel) ---- */

/* Tension de shunt brute (LSB 2,5 uV) -> mA, pour un shunt en micro-ohms. */
static inline int32_t ina226_ma_from_raw(int16_t raw, uint32_t shunt_uohm)
{
    /* I = raw * 2,5 uV / R  =>  mA = raw * 2500 / R_uohm, arrondi au plus
     * proche dans les deux sens. */
    int64_t num = (int64_t)raw * 2500;
    int64_t den = (int64_t)shunt_uohm;
    return (int32_t)(num >= 0 ? (num + den / 2) / den : (num - den / 2) / den);
}

/* Tension de bus brute (LSB 1,25 mV) -> mV. */
static inline int32_t ina226_mv_from_raw(uint16_t raw)
{
    return (int32_t)(((uint32_t)raw * 5u + 2u) / 4u);
}

/* Seuil de courant (mA) -> valeur du registre Alert Limit, comparee par
 * l'INA226 a la tension de shunt brute. Ecretee a la pleine echelle. */
static inline uint16_t ina226_alert_limit_from_ma(int32_t ma, uint32_t shunt_uohm)
{
    int64_t v = ((int64_t)ma * (int64_t)shunt_uohm + 1249) / 2500;
    if (v < 0) v = 0;
    if (v > 32767) v = 32767;
    return (uint16_t)v;
}

/* ---- pilote ---- */

/* Initialise I2C1 et la broche ALERT, detecte et configure le composant.
 * Ne bloque jamais indefiniment. Renvoie true si l'INA226 repond. */
bool ina226_init(void);

/* Lit tension et courant (deux transactions courtes, ~0,2 ms a 400 kHz). */
void ina226_read(void);

/* Nouvelle tentative de detection quand le composant est absent. */
void ina226_retry(void);

/* Vrai si l'interruption ALERT s'est declenchee depuis le dernier
 * ina226_clear_alert(). Les moteurs ont deja ete inhibes dans l'ISR. */
bool ina226_alert_fired(void);

/* Relit Mask/Enable (ce qui relache ALERT cote INA226) et efface le
 * drapeau logiciel. Renvoie true si la broche ALERT est revenue haute. */
bool ina226_clear_alert(void);

/* Nombre de fronts ALERT rejetes par le filtre anti-parasite de l'ISR. */
uint32_t ina226_alert_glitches(void);

/* Etat de la broche ALERT (true = alerte active, broche basse). */
bool ina226_alert_pin_active(void);

const ina226_t *ina226_state(void);

/* Acces bruts pour le mode de test t 2. */
bool ina226_read_reg(uint8_t reg, uint16_t *val);

#endif /* DIFFDRIVE_PICO_INA226_H */
