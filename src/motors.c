#include "motors.h"

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

#include "config.h"
#include "log.h"

typedef struct {
    uint     gpio_pwm, gpio_dir;
    uint     slice, chan;
    int      sign;          /* +1 ou -1 selon MOT_x_INVERT */
    int      duty;          /* derniere consigne appliquee, avant inversion */
} motor_t;

static motor_t s_mot[MOT_N];
static uint16_t s_top;              /* le compteur va de 0 a s_top */
static volatile bool s_inhibit;

/* Un niveau a s_top + 1 ne redescend jamais : sortie toujours haute.
 * Un niveau a 0 donne une sortie toujours basse. */
#define LEVEL_ALWAYS_LOW    ((uint16_t)0u)

static void motor_brake(motor_t *m)
{
    pwm_set_chan_level(m->slice, m->chan, LEVEL_ALWAYS_LOW);
    m->duty = 0;
}

static void motor_apply(motor_t *m, int duty)
{
    if (s_inhibit) { motor_brake(m); return; }

    if (duty >  DUTY_MAX) duty =  DUTY_MAX;
    if (duty < -DUTY_MAX) duty = -DUTY_MAX;

    int signed_duty = duty * m->sign;
    int mag = signed_duty >= 0 ? signed_duty : -signed_duty;

    /* Mise a l'echelle de la plage protocole vers la plage compteur :
     * mag = DUTY_MAX donne span, sortie haute en permanence. */
    uint32_t span = (uint32_t)s_top + 1u;
    uint32_t lvl  = ((uint32_t)mag * span + (DUTY_MAX / 2)) / DUTY_MAX;
    if (lvl > span) lvl = span;

    if (mag == 0) { motor_brake(m); return; }

    /* Sens d'abord, amplitude ensuite. Sur le MDD10A, DIR n'a d'effet que
     * pendant que PWM est haut : basculer DIR avec l'ancienne amplitude
     * inverse au pire le pont pour une fraction de periode PWM, sans
     * court-circuit (le pont est gere par le driver). */
    gpio_put(m->gpio_dir, signed_duty < 0 ? 1 : 0);
    pwm_set_chan_level(m->slice, m->chan, (uint16_t)lvl);
    m->duty = duty;
}

void motors_init(void)
{
    static const uint pwm[MOT_N] = { PIN_MOT_L_PWM, PIN_MOT_R_PWM };
    static const uint dir[MOT_N] = { PIN_MOT_L_DIR, PIN_MOT_R_DIR };
    static const int  sgn[MOT_N] = { MOT_L_INVERT ? -1 : 1,
                                     MOT_R_INVERT ? -1 : 1 };

    s_inhibit = false;

    uint32_t f_sys = clock_get_hz(clk_sys);
    uint32_t top32 = f_sys / PWM_FREQ_HZ;
    if (top32 < 2u)      top32 = 2u;
    /* Borne a 65535 : le niveau "toujours haut" vaut s_top + 1 et doit
     * tenir dans un uint16_t. */
    if (top32 > 65535u)  top32 = 65535u;
    s_top = (uint16_t)(top32 - 1u);

    for (int i = 0; i < MOT_N; i++) {
        motor_t *m = &s_mot[i];
        m->gpio_pwm = pwm[i];
        m->gpio_dir = dir[i];
        m->slice    = pwm_gpio_to_slice_num(pwm[i]);
        m->chan     = pwm_gpio_to_channel(pwm[i]);
        m->sign     = sgn[i];
        m->duty     = 0;

        /* Broches au niveau bas AVANT d'etre confiees au PWM : le MDD10A ne
         * doit jamais voir d'impulsion parasite au demarrage. */
        gpio_init(m->gpio_pwm);
        gpio_put(m->gpio_pwm, 0);
        gpio_set_dir(m->gpio_pwm, GPIO_OUT);
        gpio_init(m->gpio_dir);
        gpio_put(m->gpio_dir, 0);
        gpio_set_dir(m->gpio_dir, GPIO_OUT);

        pwm_config cfg = pwm_get_default_config();
        pwm_config_set_clkdiv(&cfg, 1.0f);
        pwm_config_set_wrap(&cfg, s_top);
        pwm_init(m->slice, &cfg, false);
        pwm_set_chan_level(m->slice, m->chan, LEVEL_ALWAYS_LOW);
        gpio_set_function(m->gpio_pwm, GPIO_FUNC_PWM);
        pwm_set_enabled(m->slice, true);
    }

    motors_stop_all();

    LOG("[mot] MDD10A PWM %lu Hz, %u pas, arret = freinage",
        (unsigned long)(f_sys / (uint32_t)(s_top + 1u)),
        (unsigned)(s_top + 1u));
}

void motors_set(int side, int duty)
{
    if (side < 0 || side >= MOT_N) return;
    motor_apply(&s_mot[side], duty);
}

void motors_stop_all(void)
{
    for (int i = 0; i < MOT_N; i++) motor_brake(&s_mot[i]);
}

void motors_inhibit(bool on)
{
    s_inhibit = on;
    if (on) motors_stop_all();
}

bool motors_inhibited(void)
{
    return s_inhibit;
}

int motors_get(int side)
{
    if (side < 0 || side >= MOT_N) return 0;
    return s_mot[side].duty;
}

uint16_t motors_pwm_top(void)
{
    return s_top;
}
