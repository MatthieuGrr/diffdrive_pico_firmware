#include "encoders.h"

#include "pico/stdlib.h"
#include "hardware/pio.h"

#include "config.h"
#include "log.h"
#include "quadrature.pio.h"

static encoders_t s_st;

static const uint  s_sm[ENC_N]     = { ENC_L_SM, ENC_R_SM };
static const uint  s_pin_a[ENC_N]  = { PIN_ENC_L_A, PIN_ENC_R_A };
static const int   s_sign[ENC_N]   = { ENC_L_INVERT ? -1 : 1,
                                       ENC_R_INVERT ? -1 : 1 };

static int32_t s_raw[ENC_N];        /* derniere valeur brute connue      */
static int32_t s_offset[ENC_N];     /* origine des compteurs cumules     */
static int32_t s_prev_count[ENC_N]; /* pour le calcul de vitesse         */
static bool    s_vel_primed;        /* premier passage : pas de vitesse  */

/*
 * Recupere le compteur publie par la machine a etat.
 *
 * Le programme PIO publie en continu avec "push noblock" : quand la FIFO est
 * pleine, les nouvelles valeurs sont jetees et la FIFO ne contient donc que
 * des valeurs perimees, potentiellement vieilles de plusieurs millisecondes.
 * Il faut donc vider la FIFO PUIS attendre la publication suivante, qui est
 * garantie fraiche. En regime normal cette attente dure environ 1 us.
 */
static int32_t quad_read(int side)
{
    PIO pio = ENC_PIO;
    uint sm = s_sm[side];

    while (!pio_sm_is_rx_fifo_empty(pio, sm)) {
        s_raw[side] = (int32_t)pio_sm_get(pio, sm);
    }

    absolute_time_t deadline = make_timeout_time_us(ENC_READ_TIMEOUT_US);
    while (pio_sm_is_rx_fifo_empty(pio, sm)) {
        if (time_reached(deadline)) {
            /* La SM ne publie plus. On rend la derniere valeur connue
             * plutot que de bloquer la boucle de controle. */
            s_st.timeouts[side]++;
            return s_raw[side];
        }
    }
    s_raw[side] = (int32_t)pio_sm_get(pio, sm);
    return s_raw[side];
}

void encoders_init(void)
{
    for (int i = 0; i < ENC_N; i++) {
        s_st.count[i]    = 0;
        s_st.vel[i]      = 0.0f;
        s_st.vel_i[i]    = 0;
        s_st.timeouts[i] = 0;
        s_raw[i]         = 0;
        s_offset[i]      = 0;
        s_prev_count[i]  = 0;
    }
    s_vel_primed = false;

    pio_sm_claim(ENC_PIO, ENC_L_SM);
    pio_sm_claim(ENC_PIO, ENC_R_SM);

    /* .origin 0 dans le programme : l'offset vaut toujours 0, et les deux
     * machines a etat partagent la meme copie du code (28 des 32
     * instructions du PIO). */
    int offset = pio_add_program(ENC_PIO, &quadrature_program);
    if (offset < 0) {
        /* Ne peut arriver que si quelque chose d'autre occupe deja ce PIO :
         * sans encodeurs, l'asservissement de vitesse n'a aucun sens. */
        LOG("[enc] ECHEC du chargement du programme PIO — pas d'encodeurs !");
        return;
    }

    for (int i = 0; i < ENC_N; i++) {
        quadrature_program_init(ENC_PIO, s_sm[i], (uint)offset, s_pin_a[i],
                                ENC_PIO_CLKDIV, ENC_PULLUP ? true : false);
    }

    /* Laisse la SM effectuer quelques echantillons pour qu'elle memorise
     * l'etat reel des broches, puis prend cette valeur comme origine :
     * les compteurs demarrent a zero quelle que soit la position des roues. */
    sleep_us(100);
    encoders_reset();

    LOG("[enc] PIO%d offset=%d SM=%u/%u clkdiv=%.1f pullup=%d",
        (int)pio_get_index(ENC_PIO), offset, ENC_L_SM, ENC_R_SM,
        (double)ENC_PIO_CLKDIV, ENC_PULLUP);
}

void encoders_reset(void)
{
    for (int i = 0; i < ENC_N; i++) {
        s_offset[i]     = quad_read(i);
        s_st.count[i]   = 0;
        s_prev_count[i] = 0;
        s_st.vel[i]     = 0.0f;
        s_st.vel_i[i]   = 0;
    }
    /* Le prochain update ne doit pas interpreter le saut d'origine comme
     * un deplacement. */
    s_vel_primed = false;
}

void encoders_update(float dt_s)
{
    for (int i = 0; i < ENC_N; i++) {
        int32_t raw = quad_read(i);
        /* Difference calculee en 32 bits signes : le debordement du
         * compteur PIO est absorbe naturellement. */
        int32_t rel = (int32_t)((uint32_t)raw - (uint32_t)s_offset[i]);
        s_st.count[i] = s_sign[i] * rel;
    }

    if (!s_vel_primed || dt_s <= 0.0f) {
        for (int i = 0; i < ENC_N; i++) {
            s_prev_count[i] = s_st.count[i];
            s_st.vel[i]     = 0.0f;
            s_st.vel_i[i]   = 0;
        }
        s_vel_primed = true;
        return;
    }

    for (int i = 0; i < ENC_N; i++) {
        int32_t d = (int32_t)((uint32_t)s_st.count[i] -
                              (uint32_t)s_prev_count[i]);
        s_prev_count[i] = s_st.count[i];

        float raw_vel = (float)d / dt_s;
        s_st.vel[i] += VEL_LPF_ALPHA * (raw_vel - s_st.vel[i]);
        s_st.vel_i[i] = (int32_t)(s_st.vel[i] >= 0.0f ? s_st.vel[i] + 0.5f
                                                      : s_st.vel[i] - 0.5f);
    }
}

const encoders_t *encoders_state(void)
{
    return &s_st;
}

int32_t encoders_raw(int side)
{
    if (side < 0 || side >= ENC_N) return 0;
    return s_raw[side];
}
