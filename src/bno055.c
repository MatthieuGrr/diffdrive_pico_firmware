#include "bno055.h"

#include <string.h>

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"

#include "config.h"
#include "log.h"

static bno055_t s_st;

bool bno055_read_reg(uint8_t addr, uint8_t reg, uint8_t *dst, size_t len)
{
    int w = i2c_write_timeout_us(IMU_I2C, addr, &reg, 1, true, I2C_TIMEOUT_US);
    if (w != 1) return false;
    int r = i2c_read_timeout_us(IMU_I2C, addr, dst, len, false, I2C_TIMEOUT_US);
    return r == (int)len;
}

static bool bno055_write_reg(uint8_t addr, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    int w = i2c_write_timeout_us(IMU_I2C, addr, buf, 2, false, I2C_TIMEOUT_US);
    return w == 2;
}

bool bno055_probe_addr(uint8_t addr)
{
    uint8_t id = 0;
    if (!bno055_read_reg(addr, BNO055_REG_CHIP_ID, &id, 1)) return false;
    return id == BNO055_CHIP_ID_VALUE;
}

static void i2c_bus_init(void)
{
    i2c_init(IMU_I2C, IMU_I2C_BAUD);
    gpio_set_function(PIN_I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_I2C_SCL, GPIO_FUNC_I2C);
    /* Les modules BNO055 du commerce embarquent deja des tirages, mais les
     * activer en interne ne coute rien et sauve un cablage minimal. */
    gpio_pull_up(PIN_I2C_SDA);
    gpio_pull_up(PIN_I2C_SCL);
}

/* Cherche le composant sur les deux adresses possibles. */
static bool find_chip(uint8_t *found)
{
    static const uint8_t cand[2] = { BNO055_ADDR, BNO055_ADDR_ALT };
    for (int i = 0; i < 2; i++) {
        if (bno055_probe_addr(cand[i])) {
            *found = cand[i];
            return true;
        }
    }
    return false;
}

/* do_reset : reset logiciel prealable. Vrai au demarrage (etat deterministe,
 * ~700 ms), faux lors d'une redetection a chaud pour ne pas figer la boucle
 * de controle pendant une seconde. */
static bool configure(uint8_t addr, bool do_reset)
{
    /* Passage en mode configuration : 19 ms au pire depuis un mode fusion. */
    if (!bno055_write_reg(addr, BNO055_REG_OPR_MODE, BNO055_OPMODE_CONFIG))
        return false;
    sleep_ms(25);

    if (do_reset) {
        /* Reset logiciel : etat deterministe quel que soit ce qui a precede.
         * Le composant ne repond plus pendant sa sequence de demarrage. */
        (void)bno055_write_reg(addr, BNO055_REG_SYS_TRIGGER, 0x20);
        sleep_ms(30);
        bool back = false;
        for (int i = 0; i < 80 && !back; i++) {     /* jusqu'a ~800 ms */
            sleep_ms(10);
            back = bno055_probe_addr(addr);
        }
        if (!back) {
            LOG("[imu] pas de reponse apres le reset logiciel");
            return false;
        }
        sleep_ms(30);
        if (!bno055_write_reg(addr, BNO055_REG_OPR_MODE, BNO055_OPMODE_CONFIG))
            return false;
        sleep_ms(25);
    }

    if (!bno055_write_reg(addr, BNO055_REG_PAGE_ID, 0x00))       return false;
    if (!bno055_write_reg(addr, BNO055_REG_PWR_MODE,
                          BNO055_PWRMODE_NORMAL))                return false;
    sleep_ms(10);
    /* Oscillateur interne (bit 7 a 0). Efface aussi le bit de reset. */
    if (!bno055_write_reg(addr, BNO055_REG_SYS_TRIGGER, 0x00))   return false;
    sleep_ms(10);
    if (!bno055_write_reg(addr, BNO055_REG_UNIT_SEL,
                          BNO055_UNIT_SEL))                      return false;

    /* Mode IMU : fusion accelerometre + gyroscope, magnetometre inactif. */
    if (!bno055_write_reg(addr, BNO055_REG_OPR_MODE, BNO055_OPMODE_IMU))
        return false;
    sleep_ms(20);   /* 7 ms specifies depuis le mode configuration */

    uint8_t mode = 0xFF;
    if (!bno055_read_reg(addr, BNO055_REG_OPR_MODE, &mode, 1))   return false;
    if ((mode & 0x0F) != BNO055_OPMODE_IMU) {
        LOG("[imu] OPR_MODE relu = 0x%02X, attendu 0x%02X",
            mode, BNO055_OPMODE_IMU);
        return false;
    }
    return true;
}

bool bno055_init(void)
{
    memset(&s_st, 0, sizeof(s_st));
    s_st.addr = BNO055_ADDR;

    i2c_bus_init();

    /* Le BNO055 met jusqu'a 650 ms a demarrer apres la mise sous tension.
     * Le Pico, lui, demarre en quelques millisecondes : il faut l'attendre,
     * mais sans y passer la journee si le composant est absent. */
    uint8_t addr = 0;
    bool found = false;
    for (int i = 0; i < 90 && !found; i++) {    /* jusqu'a ~900 ms */
        found = find_chip(&addr);
        if (!found) sleep_ms(10);
    }

    if (!found) {
        s_st.present = false;
        LOG("[imu] BNO055 ABSENT (pas de CHIP_ID 0x%02X sur 0x%02X ni 0x%02X)",
            BNO055_CHIP_ID_VALUE, BNO055_ADDR, BNO055_ADDR_ALT);
        LOG("[imu] le robot roule sans IMU, champs g*/a* a zero dans la trame");
        return false;
    }

    s_st.addr    = addr;
    s_st.chip_id = BNO055_CHIP_ID_VALUE;

    if (!configure(addr, BNO055_RESET_AT_BOOT ? true : false)) {
        s_st.present = false;
        LOG("[imu] BNO055 vu sur 0x%02X mais configuration echouee", addr);
        return false;
    }

    s_st.present = true;
    bno055_read_calib();
    LOG("[imu] BNO055 OK sur 0x%02X, mode IMU (fusion 6 axes, magneto off)",
        addr);
    LOG("[imu] unites brutes : accel 100 LSB/(m/s2), gyro 16 LSB/(deg/s)");
    return true;
}

static inline int16_t le16(const uint8_t *p)
{
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

void bno055_read(void)
{
    if (!s_st.present) {
        memset(s_st.gyro, 0, sizeof(s_st.gyro));
        memset(s_st.accel, 0, sizeof(s_st.accel));
        return;
    }

    /* 0x08..0x19 : accel (6), magneto (6), gyro (6) sont contigus. Une seule
     * transaction donne un echantillon coherent des deux capteurs utiles. */
    uint8_t buf[18];
    if (!bno055_read_reg(s_st.addr, BNO055_REG_ACC_DATA, buf, sizeof(buf))) {
        s_st.read_errors++;
        memset(s_st.gyro, 0, sizeof(s_st.gyro));
        memset(s_st.accel, 0, sizeof(s_st.accel));
        /* Plusieurs echecs consecutifs : le composant a disparu du bus. */
        if ((s_st.read_errors % 50u) == 0u) {
            LOG("[imu] %lu lectures en echec, passage en absent",
                (unsigned long)s_st.read_errors);
            s_st.present = false;
        }
        return;
    }

    s_st.accel[0] = le16(&buf[0]);
    s_st.accel[1] = le16(&buf[2]);
    s_st.accel[2] = le16(&buf[4]);
    s_st.gyro[0]  = le16(&buf[12]);
    s_st.gyro[1]  = le16(&buf[14]);
    s_st.gyro[2]  = le16(&buf[16]);
}

void bno055_read_calib(void)
{
    if (!s_st.present) { s_st.calib = 0; return; }
    uint8_t c = 0;
    if (bno055_read_reg(s_st.addr, BNO055_REG_CALIB_STAT, &c, 1)) {
        s_st.calib = c;
    }
}

void bno055_retry(void)
{
    if (s_st.present) return;

    uint8_t addr = 0;
    if (!find_chip(&addr)) return;

    LOG("[imu] BNO055 reapparu sur 0x%02X, reconfiguration", addr);
    s_st.addr        = addr;
    s_st.chip_id     = BNO055_CHIP_ID_VALUE;
    s_st.read_errors = 0;
    if (configure(addr, false)) {
        s_st.present = true;
        LOG("[imu] BNO055 de nouveau operationnelle");
    } else {
        LOG("[imu] reconfiguration echouee");
    }
}

const bno055_t *bno055_state(void)
{
    return &s_st;
}
