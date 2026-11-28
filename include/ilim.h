/*
 * diffdrive_pico_firmware — limitation du courant d'appel des moteurs.
 *
 * Ce n'est PAS une rampe temporelle. Le courant d'un moteur a courant
 * continu est proportionnel a (tension appliquee - force contre-
 * electromotrice), et la f.c.e.m. est proportionnelle a la vitesse. On
 * estime donc, a partir de la vitesse MESUREE, le rapport cyclique qui
 * equilibrerait la f.c.e.m. :
 *
 *     duty_fcem = vel / (ILIM_TICKS_PER_DUTY * vbat / ILIM_VREF_MV)
 *
 * et on borne la consigne a duty_fcem +/- ILIM_DUTY_DELTA. Consequences :
 *  - une consigne qui suit deja le moteur (rampe de l'hote, PID qui
 *    converge) n'est JAMAIS modifiee : elle reste dans la fenetre ;
 *  - seul un saut que le moteur ne peut pas suivre est retenu, et
 *    seulement le temps que la roue prenne de la vitesse ;
 *  - un duty nul (FREIN) passe toujours, immediatement : le courant de
 *    freinage du MDD10A circule dans le pont, pas dans la batterie, et
 *    l'arret doit rester instantane.
 * Les coupures de securite (chien de garde, defauts, 's') appellent
 * motors_stop_all() et ne passent pas par ici.
 *
 * Fonctions pures, testees sur l'hote (test/test_ilim.c).
 */
#ifndef DIFFDRIVE_PICO_ILIM_H
#define DIFFDRIVE_PICO_ILIM_H

#include <stdint.h>
#include "config.h"

typedef struct { float lo, hi; } ilim_win_t;

static inline ilim_win_t ilim_window(float vel_ticks_s, int32_t vbat_mv)
{
    ilim_win_t w = { -(float)DUTY_MAX, (float)DUTY_MAX };
    if (!ILIM_ENABLE) return w;
    float v = (vbat_mv >= VBAT_PRESENT_MV) ? (float)vbat_mv : (float)ILIM_VREF_MV;
    float fcem = vel_ticks_s * (float)ILIM_VREF_MV / ((float)ILIM_TICKS_PER_DUTY * v);
    float lo = fcem - (float)ILIM_DUTY_DELTA, hi = fcem + (float)ILIM_DUTY_DELTA;
    if (lo > w.lo) w.lo = lo;
    if (hi < w.hi) w.hi = hi;
    if (w.lo > w.hi) w.lo = w.hi;   /* survitesse improbable : fenetre reduite */
    return w;
}

/* Applique la fenetre a une consigne de rapport cyclique ; 0 passe toujours. */
static inline int ilim_apply(int duty, ilim_win_t w)
{
    if (duty == 0) return 0;
    if ((float)duty > w.hi) return (int)w.hi;
    if ((float)duty < w.lo) return (int)w.lo;
    return duty;
}

#endif /* DIFFDRIVE_PICO_ILIM_H */
