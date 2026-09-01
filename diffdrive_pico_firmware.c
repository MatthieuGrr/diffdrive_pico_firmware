/*
 * diffdrive_pico_firmware — etage bas niveau d'une base differentielle ROS 2.
 *
 * Cible : Raspberry Pi Pico 2 (RP2350, Cortex-M33), pico-sdk 2.3.0.
 * Maitre : Raspberry Pi 5, via une interface hardware_interface ros2_control.
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
    motors_coast_all();
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
    LOG(" moteur G  IN1/IN2  : GP%u / GP%u", PIN_MOT_L_IN1, PIN_MOT_L_IN2);
    LOG(" moteur D  IN1/IN2  : GP%u / GP%u", PIN_MOT_R_IN1, PIN_MOT_R_IN2);
    LOG(" encodeur G  A/B    : GP%u / GP%u", PIN_ENC_L_A, PIN_ENC_L_B);
    LOG(" encodeur D  A/B    : GP%u / GP%u", PIN_ENC_R_A, PIN_ENC_R_B);
    LOG(" I2C0 SDA/SCL       : GP%u / GP%u", PIN_I2C_SDA, PIN_I2C_SCL);
    LOG(" UART0 logs TX/RX   : GP%u / GP%u @ %lu bauds",
        PIN_LOG_TX, PIN_LOG_RX, (unsigned long)LOG_UART_BAUD);
    LOG("--------------------------------------------------------");
    LOG(" PWM                : %lu Hz, %u pas",
        (unsigned long)PWM_FREQ_HZ, (unsigned)(motors_pwm_top() + 1u));
    LOG(" decroissance       : %s  (MOTOR_DECAY_MODE)",
        motors_decay_name());
    LOG(" a duty nul         : %s",
        MOTOR_IDLE_BRAKE ? "freinage" : "roue libre");
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
    LOG("--------------------------------------------------------");
    LOG(" protocole robot sur USB CDC — aucun debug n'y transite");
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

    if (!test_owns_motors) {
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
            motors_coast_all();
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

    motors_init();
    motors_coast_all();

    for (int i = 0; i < 2; i++) {
        pid_init(&g_robot.pid[i],
                 PID_KP_DEFAULT, PID_KI_DEFAULT, PID_KD_DEFAULT, PID_KFF,
                 -(float)DUTY_MAX, (float)DUTY_MAX);
    }

    encoders_init();
    protocol_init();

    g_robot.imu_present = bno055_init();

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
