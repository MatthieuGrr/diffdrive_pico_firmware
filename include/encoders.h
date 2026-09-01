/*
 * diffdrive_pico_firmware — lecture des encodeurs en quadrature.
 *
 * Le decodage x4 est integralement fait par le PIO (voir quadrature.pio) :
 * aucune interruption GPIO, aucune charge CPU proportionnelle a la vitesse.
 * Ce module se contente de recuperer le compteur signe publie par chaque
 * machine a etat, d'appliquer l'inversion de sens et l'offset de remise a
 * zero, et d'en deduire une vitesse en ticks/s.
 *
 * Les compteurs exposes sont CUMULES et ne sont jamais remis a zero
 * spontanement : une trame perdue est rattrapee par la suivante.
 */
#ifndef DIFFDRIVE_PICO_ENCODERS_H
#define DIFFDRIVE_PICO_ENCODERS_H

#include <stdbool.h>
#include <stdint.h>

#define ENC_L 0
#define ENC_R 1
#define ENC_N 2

typedef struct {
    int32_t  count[ENC_N];      /* compteur cumule, apres inversion et offset */
    float    vel[ENC_N];        /* vitesse filtree, ticks/s                   */
    int32_t  vel_i[ENC_N];      /* meme vitesse arrondie, pour la trame       */
    uint32_t timeouts[ENC_N];   /* publications PIO manquees (diagnostic)     */
} encoders_t;

void encoders_init(void);

/* Rafraichit compteurs et vitesses. dt_s est la periode reelle ecoulee
 * depuis l'appel precedent, en secondes. Ne bloque jamais plus de
 * ENC_READ_TIMEOUT_US par roue. */
void encoders_update(float dt_s);

/* Remet les compteurs cumules a zero sans perturber le PIO : seul un offset
 * logiciel est deplace. Annule aussi l'historique de vitesse pour eviter un
 * pic de vitesse fantome au tour suivant. */
void encoders_reset(void);

const encoders_t *encoders_state(void);

/* Compteur PIO brut, sans inversion ni offset. Reserve au diagnostic. */
int32_t encoders_raw(int side);

#endif /* DIFFDRIVE_PICO_ENCODERS_H */
