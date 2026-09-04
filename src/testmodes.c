#include "testmodes.h"

#include <string.h>

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/watchdog.h"

#include "config.h"
#include "robot.h"
#include "encoders.h"
#include "motors.h"
#include "bno055.h"
#include "pid.h"
#include "log.h"

/* Nombre de periodes de controle entre deux affichages "lents". */
#define SLOW_DIV    (CONTROL_HZ / TEST_SLOW_LOG_HZ)

static uint32_t s_ticks;        /* periodes ecoulees depuis l'entree        */
static float    s_t;            /* temps ecoule depuis l'entree, secondes   */
static int      s_phase;        /* etat interne du test courant             */
static float    s_phase_t;      /* temps ecoule dans la phase courante      */

/* ------------------------------------------------------------------------ */

static void banner(int mode)
{
    switch (mode) {
    case TEST_NORMAL:
        LOG("=== t 0 : mode normal, le Pi reprend la main ===");
        break;
    case TEST_ENCODERS:
        LOG("=== t 1 : encodeurs seuls, moteurs coupes ===");
        LOG("    tourner une roue d'un tour exact doit donner %d counts",
            TEST_COUNTS_PER_REV);
        LOG("    et le signe doit s'inverser dans l'autre sens");
        LOG("    colonnes : enc_G enc_D | brut_G brut_D | v_G v_D (ticks/s)");
        break;
    case TEST_I2C:
        LOG("=== t 2 : bus I2C0 et BNO055 ===");
        break;
    case TEST_MOTOR_RAMP:
        LOG("=== t 3 : rampe en boucle ouverte, moteurs ALIMENTES ===");
        LOG("    surelever le robot, les roues doivent tourner librement");
        LOG("    si duty et vitesse sont de signes opposes sur une roue,");
        LOG("    inverser ENC_x_INVERT (ou MOT_x_INVERT) dans config.h");
        LOG("    colonnes : t duty_G duty_D | v_G v_D (ticks/s)");
        break;
    case TEST_PID_STEP:
        LOG("=== t 4 : echelon de consigne, PID ferme, moteurs ALIMENTES ===");
        LOG("    reglage a chaud avec la commande k <kp> <ki> <kd>");
        LOG("    colonnes : t sp | vG outG | vD outD");
        break;
    default:
        break;
    }
}

void testmodes_enter(int mode)
{
    if (mode < 0 || mode > TEST_MODE_MAX) return;

    /* Toute transition de mode part moteurs coupes. */
    motors_coast_all();
    g_robot.out_duty[0] = 0;
    g_robot.out_duty[1] = 0;
    pid_reset(&g_robot.pid[0]);
    pid_reset(&g_robot.pid[1]);

    g_robot.test_mode = mode;
    s_ticks   = 0;
    s_t       = 0.0f;
    s_phase   = 0;
    s_phase_t = 0.0f;

    /* Toute transition de mode, dans les deux sens, repart d'une consigne
     * nulle : ni l'entree ni la sortie d'un mode de test ne doit relancer
     * les moteurs sur une vieille consigne. Le Pi doit reemettre un 'v' ou
     * un 'o' explicite pour reprendre la main. */
    g_robot.ctrl_mode   = CTRL_STOP;
    g_robot.sp_ticks[0] = 0;
    g_robot.sp_ticks[1] = 0;
    g_robot.ol_duty[0]  = 0;
    g_robot.ol_duty[1]  = 0;

    banner(mode);
}

/* ------------------------------------------------------------------------ */
/* t 1 — encodeurs seuls                                                     */
/* ------------------------------------------------------------------------ */

static bool test_encoders(void)
{
    motors_coast_all();
    g_robot.out_duty[0] = 0;
    g_robot.out_duty[1] = 0;

    if ((s_ticks % SLOW_DIV) == 0u) {
        const encoders_t *e = encoders_state();
        LOG("t1 enc %+8ld %+8ld | brut %+8ld %+8ld | v %+7ld %+7ld",
            (long)e->count[ENC_L], (long)e->count[ENC_R],
            (long)encoders_raw(ENC_L), (long)encoders_raw(ENC_R),
            (long)e->vel_i[ENC_L], (long)e->vel_i[ENC_R]);
    }
    return true;   /* le test possede les moteurs (coupes) */
}

/* ------------------------------------------------------------------------ */
/* t 2 — I2C et BNO055                                                       */
/* ------------------------------------------------------------------------ */

static void i2c_scan(void)
{
    LOG("t2 scan du bus I2C0 (SDA=GP%u SCL=GP%u, %lu Hz)",
        PIN_I2C_SDA, PIN_I2C_SCL, (unsigned long)IMU_I2C_BAUD);

    int found = 0;
    for (uint8_t a = 0x08; a <= 0x77; a++) {
        uint8_t dummy;
        /* Une lecture d'un octet : un ACK d'adresse suffit a reveler la
         * presence d'un composant. */
        int r = i2c_read_timeout_us(IMU_I2C, a, &dummy, 1, false,
                                    I2C_TIMEOUT_US);
#if HW_WATCHDOG_ENABLE
        /* Un bus bloque bas ferait durer le scan jusqu'a 112 * I2C_TIMEOUT_US.
         * On entretient le chien de garde materiel pour que le diagnostic
         * aille a son terme au lieu de provoquer un redemarrage. */
        watchdog_update();
#endif
        if (r >= 0) {
            LOG("t2   adresse 0x%02X : ACK", a);
            found++;
        }
    }
    if (found == 0) {
        LOG("t2   AUCUN composant. Verifier alimentation, SDA/SCL, masse "
            "commune et tirages.");
    }
}

static void bno055_report(void)
{
    uint8_t id = 0;
    bool got = bno055_read_reg(BNO055_ADDR, BNO055_REG_CHIP_ID, &id, 1);
    uint8_t addr = BNO055_ADDR;
    if (!got) {
        got  = bno055_read_reg(BNO055_ADDR_ALT, BNO055_REG_CHIP_ID, &id, 1);
        addr = BNO055_ADDR_ALT;
    }

    if (!got) {
        LOG("t2 CHIP_ID illisible : pas de BNO055 sur 0x%02X ni 0x%02X",
            BNO055_ADDR, BNO055_ADDR_ALT);
        return;
    }

    LOG("t2 adresse 0x%02X  CHIP_ID = 0x%02X (%s, attendu 0x%02X)",
        addr, id, id == BNO055_CHIP_ID_VALUE ? "OK" : "INATTENDU",
        BNO055_CHIP_ID_VALUE);

    uint8_t v[4];
    if (bno055_read_reg(addr, BNO055_REG_OPR_MODE, &v[0], 1)) {
        LOG("t2 OPR_MODE = 0x%02X (%s)", v[0],
            (v[0] & 0x0F) == BNO055_OPMODE_IMU ? "IMU" : "autre");
    }
    if (bno055_read_reg(addr, BNO055_REG_SYS_STATUS, &v[0], 1) &&
        bno055_read_reg(addr, BNO055_REG_SYS_ERR, &v[1], 1)) {
        LOG("t2 SYS_STATUS = 0x%02X  SYS_ERR = 0x%02X", v[0], v[1]);
    }
    if (bno055_read_reg(addr, BNO055_REG_ST_RESULT, &v[0], 1)) {
        LOG("t2 ST_RESULT = 0x%02X (bits acc/mag/gyr/mcu, 1 = reussi)", v[0]);
    }
}

static void calib_report(void)
{
    const bno055_t *m = bno055_state();
    uint8_t c = m->calib;
    LOG("t2 CALIB_STAT = 0x%02X : sys=%u gyr=%u acc=%u mag=%u (0..3)",
        c, (c >> 6) & 3u, (c >> 4) & 3u, (c >> 2) & 3u, c & 3u);
    LOG("t2   en mode IMU, mag reste a 0 : c'est normal, il est desactive.");
    LOG("t2   gyr monte a 3 en laissant le robot immobile quelques secondes.");
}

static bool test_i2c(void)
{
    motors_coast_all();
    g_robot.out_duty[0] = 0;
    g_robot.out_duty[1] = 0;

    if (s_ticks == 0u) {
        i2c_scan();
        bno055_report();
    }

    if ((s_ticks % SLOW_DIV) == 0u) {
        const bno055_t *m = bno055_state();
        if (!m->present) {
            LOG("t2 BNO055 absente : pas de dump");
        } else {
            bno055_read_calib();
            calib_report();
            LOG("t2 gyro %+6d %+6d %+6d (16 LSB/dps) | "
                "accel %+6d %+6d %+6d (100 LSB/(m/s2))",
                m->gyro[0], m->gyro[1], m->gyro[2],
                m->accel[0], m->accel[1], m->accel[2]);
        }
    }
    return true;
}

/* ------------------------------------------------------------------------ */
/* t 3 — rampe en boucle ouverte                                             */
/* ------------------------------------------------------------------------ */
/*
 * Profil, en boucle :
 *   phase 0 : 0 -> +max        sur TEST_RAMP_SECONDS
 *   phase 1 : palier a +max    sur TEST_RAMP_HOLD_SECONDS
 *   phase 2 : +max -> -max     sur 2 * TEST_RAMP_SECONDS
 *   phase 3 : palier a -max    sur TEST_RAMP_HOLD_SECONDS
 *   phase 4 : -max -> 0        sur TEST_RAMP_SECONDS
 *   phase 5 : palier a 0       sur TEST_RAMP_HOLD_SECONDS
 *
 * Les deux moteurs recoivent la MEME consigne, sens avant. Une roue dont la
 * vitesse mesuree est de signe oppose a son duty signale une inversion
 * d'encodeur ou de moteur.
 */
static bool test_motor_ramp(float dt_s)
{
    const float M = (float)TEST_RAMP_DUTY_MAX;
    s_phase_t += dt_s;

    float duty = 0.0f;
    switch (s_phase) {
    case 0:
        duty = M * (s_phase_t / TEST_RAMP_SECONDS);
        if (s_phase_t >= TEST_RAMP_SECONDS) { s_phase = 1; s_phase_t = 0.0f; }
        break;
    case 1:
        duty = M;
        if (s_phase_t >= TEST_RAMP_HOLD_SECONDS) { s_phase = 2; s_phase_t = 0.0f; }
        break;
    case 2:
        duty = M - 2.0f * M * (s_phase_t / (2.0f * TEST_RAMP_SECONDS));
        if (s_phase_t >= 2.0f * TEST_RAMP_SECONDS) { s_phase = 3; s_phase_t = 0.0f; }
        break;
    case 3:
        duty = -M;
        if (s_phase_t >= TEST_RAMP_HOLD_SECONDS) { s_phase = 4; s_phase_t = 0.0f; }
        break;
    case 4:
        duty = -M + M * (s_phase_t / TEST_RAMP_SECONDS);
        if (s_phase_t >= TEST_RAMP_SECONDS) { s_phase = 5; s_phase_t = 0.0f; }
        break;
    default:
        duty = 0.0f;
        if (s_phase_t >= TEST_RAMP_HOLD_SECONDS) { s_phase = 0; s_phase_t = 0.0f; }
        break;
    }

    if (duty >  M) duty =  M;
    if (duty < -M) duty = -M;

    int d = (int)(duty >= 0.0f ? duty + 0.5f : duty - 0.5f);
    motors_set(MOT_L, d);
    motors_set(MOT_R, d);
    g_robot.out_duty[0] = d;
    g_robot.out_duty[1] = d;

    if ((s_ticks % SLOW_DIV) == 0u) {
        const encoders_t *e = encoders_state();
        LOG("t3 %6.2fs duty %+5d %+5d | v %+7ld %+7ld",
            (double)s_t, g_robot.out_duty[0], g_robot.out_duty[1],
            (long)e->vel_i[ENC_L], (long)e->vel_i[ENC_R]);
    }
    return true;
}

/* ------------------------------------------------------------------------ */
/* t 4 — echelon de consigne, boucle fermee                                  */
/* ------------------------------------------------------------------------ */
/*
 * Consigne carree 0 / +TEST_STEP_TICKS, de demi-periode TEST_STEP_SECONDS,
 * identique sur les deux roues. Trace a 50 Hz, une ligne courte : a
 * 115200 bauds une ligne de ~50 caracteres occupe ~4 ms des 20 ms de la
 * periode. Monter LOG_UART_BAUD reduit d'autant cette gigue.
 */
static bool test_pid_step(float dt_s)
{
    s_phase_t += dt_s;
    if (s_phase_t >= TEST_STEP_SECONDS) {
        s_phase_t = 0.0f;
        s_phase   = !s_phase;
    }
    float sp = s_phase ? (float)TEST_STEP_TICKS : 0.0f;

    const encoders_t *e = encoders_state();
    for (int i = 0; i < 2; i++) {
        float out = pid_update(&g_robot.pid[i], sp, e->vel[i], dt_s);
        int d = (int)(out >= 0.0f ? out + 0.5f : out - 0.5f);
        g_robot.sp_ticks[i] = (int32_t)sp;
        g_robot.out_duty[i] = d;
        motors_set(i, d);
    }

    LOG("t4 %6.2f sp %+6d | G %+7ld %+5d | D %+7ld %+5d",
        (double)s_t, (int)sp,
        (long)e->vel_i[ENC_L], g_robot.out_duty[0],
        (long)e->vel_i[ENC_R], g_robot.out_duty[1]);
    return true;
}

/* ------------------------------------------------------------------------ */

bool testmodes_step(float dt_s)
{
    if (g_robot.test_mode == TEST_NORMAL) return false;

    s_t += dt_s;

    bool owned;
    switch (g_robot.test_mode) {
    case TEST_ENCODERS:   owned = test_encoders();        break;
    case TEST_I2C:        owned = test_i2c();             break;
    case TEST_MOTOR_RAMP: owned = test_motor_ramp(dt_s);  break;
    case TEST_PID_STEP:   owned = test_pid_step(dt_s);    break;
    default:              owned = false;                  break;
    }

    s_ticks++;
    return owned;
}
