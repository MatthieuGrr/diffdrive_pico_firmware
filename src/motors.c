#include "motors.h"

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

#include "config.h"
#include "log.h"

typedef struct {
    uint     gpio_in1, gpio_in2;
    uint     slice;
    uint     chan1, chan2;
    int      sign;          /* +1 ou -1 selon MOT_x_INVERT */
    int      duty;          /* derniere consigne, avant inversion */
} motor_t;

static motor_t s_mot[MOT_N];
static uint16_t s_top;      /* valeur de wrap : le compteur va de 0 a s_top */

/* Un niveau de comparaison a s_top + 1 ne redescend jamais : sortie toujours
 * haute. Un niveau a 0 donne une sortie toujours basse. */
#define LEVEL_ALWAYS_HIGH   ((uint16_t)(s_top + 1u))
#define LEVEL_ALWAYS_LOW    ((uint16_t)0u)

static void motor_write(const motor_t *m, uint16_t lvl1, uint16_t lvl2)
{
    pwm_set_chan_level(m->slice, m->chan1, lvl1);
    pwm_set_chan_level(m->slice, m->chan2, lvl2);
}

static void motor_apply(motor_t *m, int duty)
{
    if (duty >  DUTY_MAX) duty =  DUTY_MAX;
    if (duty < -DUTY_MAX) duty = -DUTY_MAX;
    m->duty = duty;

    int signed_duty = duty * m->sign;
    int mag = signed_duty >= 0 ? signed_duty : -signed_duty;

    /* Mise a l'echelle de la plage protocole vers la plage compteur. */
    uint32_t span = (uint32_t)s_top + 1u;
    uint16_t lvl = (uint16_t)(((uint32_t)mag * span + (DUTY_MAX / 2)) / DUTY_MAX);
    if (lvl > span) lvl = (uint16_t)span;

    if (mag == 0) {
#if MOTOR_IDLE_BRAKE
        motor_write(m, LEVEL_ALWAYS_HIGH, LEVEL_ALWAYS_HIGH);   /* freinage  */
#else
        motor_write(m, LEVEL_ALWAYS_LOW, LEVEL_ALWAYS_LOW);     /* roue libre*/
#endif
        return;
    }

#if MOTOR_DECAY_MODE == DECAY_SLOW
    /*
     * Decroissance lente : une entree tenue haute, l'autre modulee en
     * complement. Le pont alterne entre entrainement et court-circuit du
     * moteur, ce qui maintient le courant pendant le temps mort.
     *   avant   : IN1 = 1, IN2 bas pendant lvl pas sur span
     *   arriere : IN2 = 1, IN1 bas pendant lvl pas sur span
     */
    uint16_t comp = (uint16_t)(span - lvl);
    if (signed_duty > 0) {
        motor_write(m, LEVEL_ALWAYS_HIGH, comp);
    } else {
        motor_write(m, comp, LEVEL_ALWAYS_HIGH);
    }
#else
    /*
     * Decroissance rapide : une seule entree modulee, l'autre a zero. Le
     * pont alterne entre entrainement et roue libre.
     */
    if (signed_duty > 0) {
        motor_write(m, lvl, LEVEL_ALWAYS_LOW);
    } else {
        motor_write(m, LEVEL_ALWAYS_LOW, lvl);
    }
#endif
}

void motors_init(void)
{
    static const uint in1[MOT_N] = { PIN_MOT_L_IN1, PIN_MOT_R_IN1 };
    static const uint in2[MOT_N] = { PIN_MOT_L_IN2, PIN_MOT_R_IN2 };
    static const int  sgn[MOT_N] = { MOT_L_INVERT ? -1 : 1,
                                     MOT_R_INVERT ? -1 : 1 };

    uint32_t f_sys = clock_get_hz(clk_sys);
    uint32_t top32 = f_sys / PWM_FREQ_HZ;
    if (top32 < 2u)      top32 = 2u;
    /* Borne a 65535 et non 65536 : s_top doit laisser la place a
     * LEVEL_ALWAYS_HIGH = s_top + 1 dans un uint16_t. A 65536 ce niveau
     * deborderait a 0, soit une sortie toujours BASSE au lieu de toujours
     * haute — une inversion silencieuse du sens de marche. */
    if (top32 > 65535u)  top32 = 65535u;
    s_top = (uint16_t)(top32 - 1u);

    for (int i = 0; i < MOT_N; i++) {
        motor_t *m = &s_mot[i];
        m->gpio_in1 = in1[i];
        m->gpio_in2 = in2[i];
        m->slice    = pwm_gpio_to_slice_num(in1[i]);
        m->chan1    = pwm_gpio_to_channel(in1[i]);
        m->chan2    = pwm_gpio_to_channel(in2[i]);
        m->sign     = sgn[i];
        m->duty     = 0;

        /* Les broches sont mises au niveau bas AVANT d'etre confiees au PWM :
         * le DRV8871 ne doit jamais voir d'impulsion parasite au demarrage. */
        gpio_init(m->gpio_in1);
        gpio_init(m->gpio_in2);
        gpio_put(m->gpio_in1, 0);
        gpio_put(m->gpio_in2, 0);
        gpio_set_dir(m->gpio_in1, GPIO_OUT);
        gpio_set_dir(m->gpio_in2, GPIO_OUT);

        pwm_config cfg = pwm_get_default_config();
        pwm_config_set_clkdiv(&cfg, 1.0f);
        pwm_config_set_wrap(&cfg, s_top);
        pwm_init(m->slice, &cfg, false);
        pwm_set_chan_level(m->slice, m->chan1, LEVEL_ALWAYS_LOW);
        pwm_set_chan_level(m->slice, m->chan2, LEVEL_ALWAYS_LOW);

        gpio_set_function(m->gpio_in1, GPIO_FUNC_PWM);
        gpio_set_function(m->gpio_in2, GPIO_FUNC_PWM);
        pwm_set_enabled(m->slice, true);
    }

    motors_coast_all();

    LOG("[mot] PWM %lu Hz top=%u (%u pas) decay=%s idle=%s",
        (unsigned long)(f_sys / (uint32_t)(s_top + 1u)), s_top,
        (unsigned)(s_top + 1u), motors_decay_name(),
        MOTOR_IDLE_BRAKE ? "frein" : "roue libre");
}

void motors_set(int side, int duty)
{
    if (side < 0 || side >= MOT_N) return;
    motor_apply(&s_mot[side], duty);
}

void motors_coast_all(void)
{
    for (int i = 0; i < MOT_N; i++) {
        s_mot[i].duty = 0;
        motor_write(&s_mot[i], LEVEL_ALWAYS_LOW, LEVEL_ALWAYS_LOW);
    }
}

int motors_get(int side)
{
    if (side < 0 || side >= MOT_N) return 0;
    return s_mot[side].duty;
}

const char *motors_decay_name(void)
{
#if MOTOR_DECAY_MODE == DECAY_SLOW
    return "lente";
#else
    return "rapide";
#endif
}

uint16_t motors_pwm_top(void)
{
    return s_top;
}
