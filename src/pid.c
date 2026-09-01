#include "pid.h"

void pid_init(pid_t *p, float kp, float ki, float kd, float kff,
              float out_min, float out_max)
{
    p->kp = kp;
    p->ki = ki;
    p->kd = kd;
    p->kff = kff;
    p->out_min = out_min;
    p->out_max = out_max;
    pid_reset(p);
}

void pid_set_gains(pid_t *p, float kp, float ki, float kd)
{
    p->kp = kp;
    p->ki = ki;
    p->kd = kd;
    /* L'integrale accumulee est conservee (la remettre a zero provoquerait
     * un a-coup en plein roulage) mais elle sera re-ecretee au prochain
     * cycle avec le nouveau ki. */
}

void pid_reset(pid_t *p)
{
    p->integ     = 0.0f;
    p->prev_meas = 0.0f;
    p->primed    = false;
    p->last_err  = 0.0f;
    p->last_out  = 0.0f;
    p->saturated = false;
}

float pid_update(pid_t *p, float setpoint, float measurement, float dt_s)
{
    if (dt_s <= 0.0f) {
        return p->last_out;
    }

    float err = setpoint - measurement;

    float p_term = p->kp * err;
    float ff     = p->kff * setpoint;

    /* Derive sur la mesure : d(err)/dt = -d(meas)/dt a consigne constante,
     * sans le pic parasite lors d'un echelon de consigne. */
    float d_term = 0.0f;
    if (p->primed && p->kd != 0.0f) {
        d_term = -p->kd * (measurement - p->prev_meas) / dt_s;
    }
    p->prev_meas = measurement;
    p->primed    = true;

    /* Integrale provisoire : on ne la valide qu'apres avoir constate si elle
     * pousse la sortie plus loin dans la saturation. */
    float integ_try = p->integ + err * dt_s;
    float out = ff + p_term + p->ki * integ_try + d_term;

    bool sat = false;
    if (out > p->out_max) {
        out = p->out_max;
        sat = true;
        if (err > 0.0f) integ_try = p->integ;    /* on gele l'integrale */
    } else if (out < p->out_min) {
        out = p->out_min;
        sat = true;
        if (err < 0.0f) integ_try = p->integ;
    }
    p->integ = integ_try;

    /* Second filet : la contribution integrale seule ne peut jamais depasser
     * la plage de sortie, meme apres un changement de gain a chaud. */
    if (p->ki > 0.0f) {
        float imax = p->out_max / p->ki;
        float imin = p->out_min / p->ki;
        if (p->integ > imax) p->integ = imax;
        if (p->integ < imin) p->integ = imin;
    } else {
        p->integ = 0.0f;
    }

    p->last_err  = err;
    p->last_out  = out;
    p->saturated = sat;
    return out;
}
