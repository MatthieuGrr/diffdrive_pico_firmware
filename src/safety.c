#include "safety.h"

#include <string.h>

#include "config.h"

static int32_t iabs(int32_t v) { return v < 0 ? -v : v; }

void safety_init(safety_t *s)
{
    memset(s, 0, sizeof(*s));
}

bool safety_update(safety_t *s, const safety_in_t *in, uint32_t dt_ms)
{
    /* Arret d'urgence : independant de l'INA226. */
    if (ESTOP_ENABLE && in->estop_active) {
        s->estop_on_ms += dt_ms;
        s->estop_off_ms = 0;
        if (s->estop_on_ms >= ESTOP_TRIP_MS) s->latched |= FLAG_ESTOP;
    } else {
        s->estop_off_ms += dt_ms;
        s->estop_on_ms = 0;
    }

    if (in->ina_present) {
        int32_t i = iabs(in->ibat_ma);

        /* Surintensite instantanee : l'INA226 l'a vue (ALERT), ou la mesure
         * de la boucle la voit. */
        if (in->alert_fired || i >= OC_TRIP_MA) {
            s->latched |= FLAG_OVERCURRENT;
        }

        /* Surintensite soutenue : blocage de roue. */
        if (i > OC_SUSTAINED_MA) {
            s->oc_ms += dt_ms;
            if (s->oc_ms >= OC_SUSTAINED_MS) s->latched |= FLAG_OVERCURRENT;
        } else {
            s->oc_ms = 0;
        }

        /* Batterie debranchee : etat transitoire, pas un defaut. */
        s->no_battery = in->vbat_mv < VBAT_PRESENT_MV;
        if (!s->no_battery && in->vbat_mv < UV_TRIP_MV) {
            s->uv_ms += dt_ms;
            if (s->uv_ms >= UV_DEBOUNCE_MS) s->latched |= FLAG_UNDERVOLTAGE;
        } else {
            s->uv_ms = 0;
        }
    } else {
        /* Sans capteur : plus de mesure, mais l'alerte materielle reste
         * prise en compte si elle a eu lieu. */
        if (in->alert_fired) s->latched |= FLAG_OVERCURRENT;
        s->no_battery = false;
        s->oc_ms = 0;
        s->uv_ms = 0;
    }

    s->inhibit = (s->latched != 0u) || (INA_REQUIRED && !in->ina_present);
    return s->inhibit;
}

uint32_t safety_try_clear(safety_t *s, const safety_in_t *in)
{
    if (s->latched & FLAG_ESTOP) {
        /* Bouton relache ET stable depuis ESTOP_RELEASE_MS. */
        bool released = !(ESTOP_ENABLE && in->estop_active) &&
                        s->estop_off_ms >= ESTOP_RELEASE_MS;
        if (released) s->latched &= ~FLAG_ESTOP;
    }
    if (s->latched & FLAG_OVERCURRENT) {
        bool cause_gone = !in->alert_pin_active &&
                          (!in->ina_present || iabs(in->ibat_ma) < OC_SUSTAINED_MA);
        if (cause_gone) {
            s->latched &= ~FLAG_OVERCURRENT;
            s->oc_ms = 0;
        }
    }
    if (s->latched & FLAG_UNDERVOLTAGE) {
        /* Sans mesure on ne peut pas prouver que la batterie est revenue :
         * on fait confiance a l'hote. Avec mesure, il faut repasser
         * au-dessus du seuil haut (hysteresis). */
        if (!in->ina_present || in->vbat_mv >= UV_CLEAR_MV) {
            s->latched &= ~FLAG_UNDERVOLTAGE;
            s->uv_ms = 0;
        }
    }
    s->inhibit = (s->latched != 0u) || (INA_REQUIRED && !in->ina_present);
    return s->latched;
}

uint32_t safety_flags(const safety_t *s, const safety_in_t *in)
{
    uint32_t f = s->latched;
    if (s->no_battery)    f |= FLAG_NO_BATTERY;
    if (!in->ina_present) f |= FLAG_INA_ABSENT;
    if (s->inhibit)       f |= FLAG_MOTORS_INHIBITED;
    return f;
}
