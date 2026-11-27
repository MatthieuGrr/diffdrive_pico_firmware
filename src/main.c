/*
 * diffdrive_pico_firmware — etage bas niveau d'une base differentielle.
 *
 * Cible : Raspberry Pi Pico 2 (RP2350, Cortex-M33), pico-sdk 2.3.0.
 * Hote : n'importe quel ordinateur parlant le protocole serie documente
 * (Raspberry Pi 5 sous ROS 2, Jetson, PC...), via l'USB du Pico.
 *
 * ORGANISATION
 * ------------
 * Un seul fil d'execution : la boucle principale. Le timer materiel a 50 Hz
 * ne fait qu'armer un compteur ; tout le travail (lecture des encodeurs, PID,
 * I2C, emission de la trame) se deroule en contexte normal. Les transactions
 * I2C et les ecritures USB ne sont donc jamais faites sous interruption, et
 * aucun verrou n'est necessaire.
 *
 * La boucle principale tourne a plusieurs centaines de kHz : elle lit le
 * port serie en continu et surveille le chien de garde a chaque tour, pas
 * seulement aux 50 Hz.
 *
 * FRONTIERE D'ABSTRACTION
 * -----------------------
 * Ticks et ticks/s, rien d'autre. Ni rayon de roue, ni entraxe, ni counts
 * par tour : la cinematique et la calibration vivent cote Pi.
 *
 * REGLE STDIO
 * -----------
 * printf / getchar / stdio == protocole robot sur USB CDC, exclusivement.
 * Tous les messages destines a un humain passent par LOG() sur UART0.
 */

#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/watchdog.h"
#include "hardware/sync.h"

#include "config.h"
#include "robot.h"
#include "log.h"
#include "encoders.h"
#include "motors.h"
#include "bno055.h"
#include "ina226.h"
#include "safety.h"
#include "pid.h"
#include "protocol.h"
#include "testmodes.h"

robot_t g_robot;

/* Arme par le timer materiel, consomme par la boucle principale. */
static volatile uint32_t s_pending_ticks;
static repeating_timer_t s_timer;

static bool control_timer_cb(repeating_timer_t *t)
{
    (void)t;
    s_pending_ticks++;
    return true;    /* on continue */
}

/* ------------------------------------------------------------------------ */

void robot_failsafe(void)
{
    motors_stop_all();
    g_robot.out_duty[0] = 0;
    g_robot.out_duty[1] = 0;
    g_robot.sp_ticks[0] = 0;
    g_robot.sp_ticks[1] = 0;
    g_robot.ol_duty[0]  = 0;
    g_robot.ol_duty[1]  = 0;
    g_robot.ctrl_mode   = CTRL_STOP;
    pid_reset(&g_robot.pid[0]);
    pid_reset(&g_robot.pid[1]);
}

/* Entree arret d'urgence (contact auxiliaire NF, voir PIN_ESTOP). */
static void estop_init(void)
{
    gpio_init(PIN_ESTOP);
    gpio_set_dir(PIN_ESTOP, GPIO_IN);
    if (ESTOP_ACTIVE_HIGH) gpio_pull_up(PIN_ESTOP);
    else                   gpio_pull_down(PIN_ESTOP);
}

static bool estop_pin_active(void)
{
    return gpio_get(PIN_ESTOP) == (ESTOP_ACTIVE_HIGH ? 1 : 0);
}

static void safety_inputs(safety_in_t *in)
{
    const ina226_t *p = ina226_state();
    in->ina_present      = p->present;
    in->vbat_mv          = p->vbat_mv;
    in->ibat_ma          = p->ibat_ma;
    in->alert_fired      = ina226_alert_fired();
    in->alert_pin_active = ina226_alert_pin_active();
    in->estop_active     = estop_pin_active();
}

void robot_clear_faults(void)
{
    if (g_robot.safety.latched == 0u) return;

    /* Relit Mask/Enable : l'INA226 relache ALERT si le courant est revenu
     * sous le seuil. */
    (void)ina226_clear_alert();

    safety_in_t in;
    safety_inputs(&in);
    uint32_t before = g_robot.safety.latched;
    uint32_t left   = safety_try_clear(&g_robot.safety, &in);
    if (left != before) {
        LOG("[secu] defauts effaces par 's' : 0x%02lX -> 0x%02lX",
            (unsigned long)before, (unsigned long)left);
    }
    if (left != 0u) {
        LOG("[secu] defaut toujours present (cause active) : 0x%02lX, "
            "I=%ld mA V=%ld mV", (unsigned long)left,
            (long)in.ibat_ma, (long)in.vbat_mv);
    }
    /* La levee de l'inhibition se fait dans la boucle de controle, apres
     * une nouvelle evaluation complete. */
}

/* Mesure batterie, decision de securite, et application. Appele a chaque
 * periode de controle, AVANT le calcul des commandes moteur. */
static void safety_step(uint32_t dt_ms)
{
    const ina226_t *p = ina226_state();
    if (p->present) {
        ina226_read();
    } else {
        static uint32_t retry = 0;
        if (++retry >= INA_RETRY_TICKS) { retry = 0; ina226_retry(); }
    }

    safety_in_t in;
    safety_inputs(&in);
    uint32_t before = g_robot.safety.latched;
    bool inhibit = safety_update(&g_robot.safety, &in, dt_ms);
    uint32_t after = g_robot.safety.latched;

    if (after & ~before) {
        robot_failsafe();
        LOG("[secu] DEFAUT %s%s%s : moteurs freines et verrouilles "
            "(I=%ld mA V=%ld mV). Effacement : commande 's'.",
            (after & ~before & FLAG_ESTOP)        ? "ARRET D'URGENCE " : "",
            (after & ~before & FLAG_OVERCURRENT)  ? "SURINTENSITE " : "",
            (after & ~before & FLAG_UNDERVOLTAGE) ? "SOUS-TENSION" : "",
            (long)in.ibat_ma, (long)in.vbat_mv);
    }

    if (inhibit) {
        if (!motors_inhibited()) motors_inhibit(true);
    } else if (motors_inhibited()) {
        /* Ne pas lever l'inhibition si ALERT vient juste de tomber entre la
         * lecture du drapeau et maintenant : on tranche sous IRQ masquees. */
        uint32_t irq = save_and_disable_interrupts();
        if (!ina226_alert_fired()) motors_inhibit(false);
        restore_interrupts(irq);
    }

    g_robot.vbat_mv = in.vbat_mv;
    g_robot.ibat_ma = in.ibat_ma;
    g_robot.flags   = safety_flags(&g_robot.safety, &in)
                    | (g_robot.link_lost   ? FLAG_LINK_LOST  : 0u)
                    | (!g_robot.imu_present ? FLAG_IMU_ABSENT : 0u)
                    | (g_robot.test_mode != TEST_NORMAL ? FLAG_TEST_MODE : 0u);
}

/* ------------------------------------------------------------------------ */

static void banner(void)
{
    LOG(" ");
    LOG("========================================================");
    LOG(" %s v%s — RP2350 / pico-sdk 2.3.0", FW_NAME, FW_VERSION);
    LOG(" compile le %s a %s", __DATE__, __TIME__);
    LOG("--------------------------------------------------------");
    LOG(" horloge systeme    : %lu Hz",
        (unsigned long)clock_get_hz(clk_sys));
    LOG(" moteur G  PWM/DIR  : GP%u / GP%u (MDD10A canal 1)",
        PIN_MOT_L_PWM, PIN_MOT_L_DIR);
    LOG(" moteur D  PWM/DIR  : GP%u / GP%u (MDD10A canal 2)",
        PIN_MOT_R_PWM, PIN_MOT_R_DIR);
    LOG(" encodeur G  A/B    : GP%u / GP%u", PIN_ENC_L_A, PIN_ENC_L_B);
    LOG(" encodeur D  A/B    : GP%u / GP%u", PIN_ENC_R_A, PIN_ENC_R_B);
    LOG(" I2C0 BNO055 SDA/SCL: GP%u / GP%u", PIN_I2C_SDA, PIN_I2C_SCL);
    LOG(" I2C1 INA226 SDA/SCL: GP%u / GP%u, ALERT GP%u",
        PIN_INA_SDA, PIN_INA_SCL, PIN_INA_ALERT);
    LOG(" arret d'urgence    : GP%u, %s, actuellement %s", PIN_ESTOP,
        ESTOP_ENABLE ? "surveille" : "IGNORE (ESTOP_ENABLE 0)",
        estop_pin_active() ? "ENFONCE ou non cable" : "relache");
    LOG(" UART0 logs TX/RX   : GP%u / GP%u @ %lu bauds",
        PIN_LOG_TX, PIN_LOG_RX, (unsigned long)LOG_UART_BAUD);
    LOG("--------------------------------------------------------");
    LOG(" PWM                : %lu Hz, %u pas",
        (unsigned long)PWM_FREQ_HZ, (unsigned)(motors_pwm_top() + 1u));
    LOG(" a duty nul / arret : freinage (MDD10A, pas de roue libre)");
    LOG(" inversions moteur  : G=%d D=%d", MOT_L_INVERT, MOT_R_INVERT);
    LOG(" inversions encodeur: G=%d D=%d", ENC_L_INVERT, ENC_R_INVERT);
    LOG(" boucle de controle : %u Hz", CONTROL_HZ);
    LOG(" PID par defaut     : kp=%.4f ki=%.4f kd=%.4f kff=%.4f",
        (double)PID_KP_DEFAULT, (double)PID_KI_DEFAULT,
        (double)PID_KD_DEFAULT, (double)PID_KFF);
    LOG(" chien de garde     : %lu ms sans commande => PWM a zero",
        (unsigned long)(CMD_TIMEOUT_US / 1000u));
    LOG(" chien de garde HW  : %s",
        HW_WATCHDOG_ENABLE ? "actif" : "inactif");
    LOG(" BNO055             : %s",
        g_robot.imu_present ? "present, mode IMU (magneto desactive)"
                            : "ABSENT — trame a zero sur g*/a*");
    LOG(" INA226             : %s",
        ina226_state()->present ? "present" : "ABSENT — pas de protection batterie");
    LOG(" surintensite       : %d mA instantane (ALERT), %d mA pendant %lu ms",
        OC_TRIP_MA, OC_SUSTAINED_MA, (unsigned long)OC_SUSTAINED_MS);
    LOG(" sous-tension       : %d mV pendant %lu ms, reprise >= %d mV",
        UV_TRIP_MV, (unsigned long)UV_DEBOUNCE_MS, UV_CLEAR_MV);
    LOG("--------------------------------------------------------");
    LOG(" protocole v%d sur USB CDC — aucun debug n'y transite", PROTO_VERSION);
    LOG(" modes de test : t 0 normal | t 1 encodeurs | t 2 I2C");
    LOG("                 t 3 rampe moteurs | t 4 echelon PID");
    LOG("========================================================");
}

/* Battement de coeur : sans sonde SWD, c'est le seul signe de vie visible
 * quand rien n'est branche sur l'UART.
 *   clignotement lent  : pas de commande recue, chien de garde arme
 *   clignotement rapide: lien serie actif
 *   allumee fixe       : mode de test actif                                */
static void heartbeat(uint64_t now_us)
{
#ifdef PICO_DEFAULT_LED_PIN
    uint32_t period_ms;
    if (g_robot.test_mode != TEST_NORMAL) {
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
        return;
    }
    period_ms = g_robot.link_lost ? 1000u : 200u;
    uint32_t ms = (uint32_t)(now_us / 1000u);
    gpio_put(PICO_DEFAULT_LED_PIN, (ms % period_ms) < (period_ms / 2u));
#else
    (void)now_us;
#endif
}

/* ------------------------------------------------------------------------ */

static void control_step(uint64_t now_us, float dt_s)
{
    encoders_update(dt_s);

    /* Securite avant toute commande moteur : un defaut detecte ici freine
     * les moteurs pour cette periode deja. */
    safety_step((uint32_t)(dt_s * 1000.0f + 0.5f));

    /* IMU : une seule transaction en rafale, bornee par I2C_TIMEOUT_US. */
    if (g_robot.imu_present) {
        bno055_read();
        g_robot.imu_present = bno055_state()->present;
    } else {
        static uint32_t retry = 0;
        if (++retry >= IMU_RETRY_TICKS) {
            retry = 0;
            bno055_retry();
            g_robot.imu_present = bno055_state()->present;
        }
    }

    /* Un mode de test actif prend la main sur les moteurs. */
    bool test_owns_motors = testmodes_step(dt_s);

    if (motors_inhibited()) {
        /* Defaut actif : les consignes sont acceptees (elles rearment le
         * chien de garde) mais rien n'est applique, et le PID ne doit pas
         * accumuler d'erreur pendant ce temps. */
        g_robot.out_duty[0] = 0;
        g_robot.out_duty[1] = 0;
        pid_reset(&g_robot.pid[0]);
        pid_reset(&g_robot.pid[1]);
    } else if (!test_owns_motors) {
        const encoders_t *e = encoders_state();
        switch (g_robot.ctrl_mode) {
        case CTRL_VELOCITY:
            for (int i = 0; i < 2; i++) {
                float out = pid_update(&g_robot.pid[i],
                                       (float)g_robot.sp_ticks[i],
                                       e->vel[i], dt_s);
                int d = (int)(out >= 0.0f ? out + 0.5f : out - 0.5f);
                g_robot.out_duty[i] = d;
                motors_set(i, d);
            }
            break;

        case CTRL_OPENLOOP:
            for (int i = 0; i < 2; i++) {
                g_robot.out_duty[i] = g_robot.ol_duty[i];
                motors_set(i, g_robot.ol_duty[i]);
            }
            break;

        case CTRL_STOP:
        default:
            g_robot.out_duty[0] = 0;
            g_robot.out_duty[1] = 0;
            motors_stop_all();
            break;
        }
    }

    /* La trame d'etat est emise dans tous les modes : on peut observer un
     * mode de test depuis le Pi en meme temps que sur l'UART. */
    protocol_send_state((uint32_t)(now_us / 1000u));
}

/* Chien de garde logiciel. Verifie a chaque tour de la boucle principale,
 * donc bien plus finement que la periode de controle. Seule securite non
 * negociable du firmware.
 * Suspendu pendant les modes de test : c'est alors le test qui pilote les
 * moteurs, pas le Pi. */
static void check_link_watchdog(uint64_t now_us)
{
    if (g_robot.test_mode != TEST_NORMAL) return;
    if (now_us - g_robot.last_cmd_us <= (uint64_t)CMD_TIMEOUT_US) return;
    if (g_robot.link_lost) return;

    g_robot.link_lost = true;
    robot_failsafe();
    LOG("[wd] aucune commande valide depuis %lu ms : PWM a zero",
        (unsigned long)(CMD_TIMEOUT_US / 1000u));
}

/* ------------------------------------------------------------------------ */

int main(void)
{
    /* stdio = protocole robot sur USB CDC uniquement. La console UART est
     * desactivee cote CMake : rien d'autre ne peut polluer ce flux. */
    stdio_init_all();

    log_init();

#ifdef PICO_DEFAULT_LED_PIN
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_LED_PIN, 1);
#endif

    bool wd_reboot = watchdog_caused_reboot();

    /* Etat initial : moteurs coupes avant toute autre chose. */
    g_robot.ctrl_mode   = CTRL_STOP;
    g_robot.test_mode   = TEST_NORMAL;
    g_robot.link_lost   = false;
    g_robot.rx_bad      = 0;
    g_robot.overruns    = 0;
    g_robot.sp_ticks[0] = g_robot.sp_ticks[1] = 0;
    g_robot.ol_duty[0]  = g_robot.ol_duty[1]  = 0;
    g_robot.out_duty[0] = g_robot.out_duty[1] = 0;

    safety_init(&g_robot.safety);
    g_robot.vbat_mv = g_robot.ibat_ma = 0;
    g_robot.flags   = 0;

    motors_init();
    motors_stop_all();

    for (int i = 0; i < 2; i++) {
        pid_init(&g_robot.pid[i],
                 PID_KP_DEFAULT, PID_KI_DEFAULT, PID_KD_DEFAULT, PID_KFF,
                 -(float)DUTY_MAX, (float)DUTY_MAX);
    }

    encoders_init();
    protocol_init();

    g_robot.imu_present = bno055_init();
    (void)ina226_init();
    estop_init();

    banner();
    if (wd_reboot) {
        LOG("[wd] ATTENTION : redemarrage provoque par le chien de garde "
            "materiel — la boucle principale s'etait bloquee");
    }

    /* Grace initiale : le Pi a CMD_TIMEOUT_US pour se manifester avant que
     * le chien de garde ne se plaigne. */
    g_robot.last_cmd_us = time_us_64();

    uint64_t last_step_us = time_us_64();
    s_pending_ticks = 0;

    if (!add_repeating_timer_us(-(int64_t)CONTROL_PERIOD_US,
                                control_timer_cb, NULL, &s_timer)) {
        LOG("[sys] ECHEC de l'armement du timer de controle");
    }

#if HW_WATCHDOG_ENABLE
    /* Redemarre la puce si la boucle principale se fige completement.
     * Deuxieme argument : pause quand un debogueur est attache. */
    watchdog_enable(HW_WATCHDOG_MS, true);
#endif

    while (true) {
        /* Le port serie est lu en continu, jamais uniquement a 50 Hz : une
         * commande est prise en compte dans la microseconde qui suit. */
        protocol_poll();

        uint64_t now = time_us_64();
        check_link_watchdog(now);

        if (s_pending_ticks) {
            uint32_t pending;
            /* Lecture-modification atomique vis-a-vis de l'interruption. */
            uint32_t irq = save_and_disable_interrupts();
            pending = s_pending_ticks;
            s_pending_ticks = 0;
            restore_interrupts(irq);

            if (pending > 1u) {
                g_robot.overruns += pending - 1u;
                /* On ne rattrape pas les periodes perdues : on repart de
                 * l'instant courant avec le dt reel. */
            }

            float dt_s = (float)(now - last_step_us) * 1e-6f;
            last_step_us = now;
            control_step(now, dt_s);
        }

        heartbeat(now);

#if HW_WATCHDOG_ENABLE
        watchdog_update();
#endif
        tight_loop_contents();
    }
}
