#include "ina226.h"

#include <string.h>

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"

#include "config.h"
#include "motors.h"
#include "log.h"

static ina226_t s_st;
static volatile bool s_alert;
static volatile uint32_t s_glitches;   /* fronts ALERT rejetes (parasites) */

/* ------------------------------------------------------------------------ */
/* Interruption ALERT                                                        */
/* ------------------------------------------------------------------------ */
/* Seule chose faite sous interruption dans tout le firmware, en dehors du
 * timer de controle : freiner les moteurs et lever un drapeau. Aucun acces
 * I2C ici, la boucle principale s'en charge. */
static void alert_isr(uint gpio, uint32_t events)
{
    if (gpio != PIN_INA_ALERT) return;
    if (events & GPIO_IRQ_EDGE_FALL) {
        /* Filtre anti-parasite. L'INA226 est en mode verrouille (LEN) : une
         * vraie alerte maintient ALERT a l'etat bas jusqu'a la relecture de
         * Mask/Enable. La commutation du pont (arret depuis la pleine
         * vitesse) induit sur le fil des impulsions breves que le front
         * descendant capte alors que l'INA226 n'a rien vu (AFF a 0, broche
         * deja haute a l'entree de l'ISR, constate au banc le 28/11). On
         * exige donc que la broche soit encore basse, deux fois a 2 us
         * d'intervalle. Cout pour une vraie alerte : 2 us. */
        if (gpio_get(PIN_INA_ALERT)) { s_glitches++; return; }
        busy_wait_us_32(2);
        if (gpio_get(PIN_INA_ALERT)) { s_glitches++; return; }
        motors_inhibit(true);
        s_alert = true;
    }
}

/* ------------------------------------------------------------------------ */
/* Acces registres (16 bits, octet de poids fort en premier)                 */
/* ------------------------------------------------------------------------ */

bool ina226_read_reg(uint8_t reg, uint16_t *val)
{
    uint8_t b[2];
    if (i2c_write_timeout_us(INA_I2C, INA226_ADDR, &reg, 1, true,
                             I2C_TIMEOUT_US) != 1) return false;
    if (i2c_read_timeout_us(INA_I2C, INA226_ADDR, b, 2, false,
                            I2C_TIMEOUT_US) != 2) return false;
    *val = (uint16_t)((b[0] << 8) | b[1]);
    return true;
}

static bool write_reg(uint8_t reg, uint16_t val)
{
    uint8_t b[3] = { reg, (uint8_t)(val >> 8), (uint8_t)val };
    return i2c_write_timeout_us(INA_I2C, INA226_ADDR, b, 3, false,
                                I2C_TIMEOUT_US) == 3;
}

/* ------------------------------------------------------------------------ */

static bool configure(void)
{
    uint16_t id = 0, die = 0;
    if (!ina226_read_reg(INA226_REG_MANUF_ID, &id))  return false;
    if (!ina226_read_reg(INA226_REG_DIE_ID, &die))   return false;
    if (id != INA226_MANUF_ID_VALUE || die != INA226_DIE_ID_VALUE) {
        LOG("[ina] identifiants inattendus : MANUF 0x%04X DIE 0x%04X", id, die);
        return false;
    }

    /* Reset, puis configuration de la mesure. */
    if (!write_reg(INA226_REG_CONFIG, 0x8000u)) return false;
    sleep_ms(2);
    if (!write_reg(INA226_REG_CONFIG, INA226_CONFIG_VALUE)) return false;

    uint16_t lim = ina226_alert_limit_from_ma(OC_TRIP_MA, INA226_SHUNT_UOHM);
    if (!write_reg(INA226_REG_ALERT_LIM, lim)) return false;
    if (!write_reg(INA226_REG_MASK_EN, INA226_MASK_SOL | INA226_MASK_LEN))
        return false;

    uint16_t back = 0;
    if (!ina226_read_reg(INA226_REG_CONFIG, &back) ||
        (back & 0x0FFFu) != (INA226_CONFIG_VALUE & 0x0FFFu)) {
        LOG("[ina] CONFIG relu = 0x%04X, attendu 0x%04X",
            back, INA226_CONFIG_VALUE);
        return false;
    }
    LOG("[ina] INA226 OK sur 0x%02X, shunt %lu uOhm, alerte a %d mA (lim 0x%04X)",
        INA226_ADDR, (unsigned long)INA226_SHUNT_UOHM, OC_TRIP_MA, lim);
    return true;
}

bool ina226_init(void)
{
    memset(&s_st, 0, sizeof(s_st));
    s_alert = false;

    i2c_init(INA_I2C, INA_I2C_BAUD);
    gpio_set_function(PIN_INA_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_INA_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_INA_SDA);
    gpio_pull_up(PIN_INA_SCL);

    /* ALERT : drain ouvert cote INA226, tirage interne ici. Une broche non
     * cablee reste donc haute, sans fausse alerte. */
    gpio_init(PIN_INA_ALERT);
    gpio_set_dir(PIN_INA_ALERT, GPIO_IN);
    gpio_pull_up(PIN_INA_ALERT);

    s_st.present = configure();
    if (!s_st.present) {
        LOG("[ina] INA226 ABSENT sur I2C1 0x%02X : pas de mesure batterie, "
            "pas de protection surintensite / sous-tension", INA226_ADDR);
    } else {
        (void)ina226_clear_alert();
    }

    /* L'interruption n'est armee qu'apres la configuration : un etat
     * d'ALERT herite d'avant le reset ne doit pas couper les moteurs. */
    gpio_set_irq_enabled_with_callback(PIN_INA_ALERT, GPIO_IRQ_EDGE_FALL,
                                       true, alert_isr);
    return s_st.present;
}

void ina226_read(void)
{
    if (!s_st.present) {
        s_st.vbat_mv = 0;
        s_st.ibat_ma = 0;
        return;
    }
    uint16_t sh = 0, bus = 0;
    if (!ina226_read_reg(INA226_REG_SHUNT, &sh) ||
        !ina226_read_reg(INA226_REG_BUS, &bus)) {
        s_st.read_errors++;
        if ((s_st.read_errors % 25u) == 0u) {
            LOG("[ina] %lu lectures en echec, passage en absent",
                (unsigned long)s_st.read_errors);
            s_st.present = false;
            s_st.vbat_mv = 0;
            s_st.ibat_ma = 0;
        }
        return;
    }
    s_st.ibat_ma = ina226_ma_from_raw((int16_t)sh, INA226_SHUNT_UOHM);
    s_st.vbat_mv = ina226_mv_from_raw(bus);
}

void ina226_retry(void)
{
    if (s_st.present) return;
    uint16_t id = 0;
    if (!ina226_read_reg(INA226_REG_MANUF_ID, &id)) return;
    LOG("[ina] INA226 reapparu, reconfiguration");
    s_st.read_errors = 0;
    s_st.present = configure();
}

bool ina226_alert_fired(void)
{
    return s_alert;
}

bool ina226_alert_pin_active(void)
{
    return !gpio_get(PIN_INA_ALERT);
}

bool ina226_clear_alert(void)
{
    uint16_t m = 0;
    if (s_st.present) (void)ina226_read_reg(INA226_REG_MASK_EN, &m);
    s_alert = false;
    return !ina226_alert_pin_active();
}

uint32_t ina226_alert_glitches(void)
{
    return s_glitches;
}

const ina226_t *ina226_state(void)
{
    return &s_st;
}
