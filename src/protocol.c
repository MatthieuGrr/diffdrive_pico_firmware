#include "protocol.h"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "pico/stdlib.h"

#include "config.h"
#include "robot.h"
#include "encoders.h"
#include "motors.h"
#include "bno055.h"
#include "testmodes.h"
#include "log.h"

static char   s_rx[RX_LINE_MAX];
static size_t s_rx_len;
static bool   s_rx_overflow;      /* ligne trop longue : on jette jusqu'au \n */

/* ------------------------------------------------------------------------ */
/* Analyse lexicale minimale                                                 */
/* ------------------------------------------------------------------------ */
/* strtol / strtof plutot que sscanf : plus leger, et surtout on controle
 * exactement ce qui est accepte et ce qui est rejete.                       */

static char *skip_ws(char *p)
{
    while (*p == ' ' || *p == '\t') p++;
    return p;
}

static bool next_i32(char **p, int32_t *out)
{
    char *s = skip_ws(*p);
    if (*s == '\0') return false;
    char *end = NULL;
    long v = strtol(s, &end, 10);
    if (end == s) return false;                 /* aucun chiffre consomme */
    if (*end != '\0' && *end != ' ' && *end != '\t') return false;
    *out = (int32_t)v;
    *p = end;
    return true;
}

static bool next_f32(char **p, float *out)
{
    char *s = skip_ws(*p);
    if (*s == '\0') return false;
    char *end = NULL;
    float v = strtof(s, &end);
    if (end == s) return false;
    if (*end != '\0' && *end != ' ' && *end != '\t') return false;
    *out = v;
    *p = end;
    return true;
}

/* Vrai si seuls des blancs suivent : refuse les arguments surnumeraires. */
static bool at_end(char *p)
{
    return *skip_ws(p) == '\0';
}

/* ------------------------------------------------------------------------ */
/* Execution d'une ligne                                                     */
/* ------------------------------------------------------------------------ */

static void cmd_reject(const char *why, const char *line)
{
    g_robot.rx_bad++;
    LOG("[rx] rejet (%s) : \"%s\"", why, line);
}

static void exec_line(char *line)
{
    char *p = skip_ws(line);
    if (*p == '\0') return;                 /* ligne vide : silencieusement */

    char cmd = *p++;
    bool ok = false;

    switch (cmd) {
    case 'v': {                             /* consigne de vitesse */
        int32_t l, r;
        if (next_i32(&p, &l) && next_i32(&p, &r) && at_end(p)) {
            g_robot.sp_ticks[0] = l;
            g_robot.sp_ticks[1] = r;
            g_robot.ctrl_mode   = CTRL_VELOCITY;
            ok = true;
        }
        break;
    }
    case 'o': {                             /* boucle ouverte */
        int32_t l, r;
        if (next_i32(&p, &l) && next_i32(&p, &r) && at_end(p)) {
            if (l >  DUTY_MAX) l =  DUTY_MAX;
            if (l < -DUTY_MAX) l = -DUTY_MAX;
            if (r >  DUTY_MAX) r =  DUTY_MAX;
            if (r < -DUTY_MAX) r = -DUTY_MAX;
            g_robot.ol_duty[0] = (int)l;
            g_robot.ol_duty[1] = (int)r;
            g_robot.ctrl_mode  = CTRL_OPENLOOP;
            pid_reset(&g_robot.pid[0]);
            pid_reset(&g_robot.pid[1]);
            ok = true;
        }
        break;
    }
    case 's':                               /* arret immediat */
        if (at_end(p)) {
            robot_failsafe();
            ok = true;
        }
        break;

    case 'r':                               /* remise a zero des compteurs */
        if (at_end(p)) {
            encoders_reset();
            ok = true;
        }
        break;

    case 'k': {                             /* gains PID a chaud */
        float kp, ki, kd;
        if (next_f32(&p, &kp) && next_f32(&p, &ki) && next_f32(&p, &kd)
            && at_end(p)) {
            pid_set_gains(&g_robot.pid[0], kp, ki, kd);
            pid_set_gains(&g_robot.pid[1], kp, ki, kd);
            LOG("[pid] gains kp=%.4f ki=%.4f kd=%.4f",
                (double)kp, (double)ki, (double)kd);
            ok = true;
        }
        break;
    }
    case 't': {                             /* mode de test */
        int32_t n;
        if (next_i32(&p, &n) && at_end(p) && n >= 0 && n <= TEST_MODE_MAX) {
            testmodes_enter((int)n);
            ok = true;
        }
        break;
    }
    default:
        break;
    }

    if (ok) {
        /* Toute commande valide rearme le chien de garde logiciel. */
        g_robot.last_cmd_us = time_us_64();
        if (g_robot.link_lost) {
            g_robot.link_lost = false;
            LOG("[wd] lien serie retabli");
        }
    } else {
        cmd_reject("syntaxe", line);
    }
}

/* ------------------------------------------------------------------------ */
/* Reception                                                                 */
/* ------------------------------------------------------------------------ */

void protocol_init(void)
{
    s_rx_len      = 0;
    s_rx_overflow = false;
}

void protocol_poll(void)
{
    for (int budget = RX_CHARS_PER_POLL; budget > 0; budget--) {
        int c = getchar_timeout_us(0);          /* strictement non bloquant */
        if (c == PICO_ERROR_TIMEOUT || c < 0) return;

        if (c == '\n' || c == '\r') {
            if (s_rx_overflow) {
                s_rx_overflow = false;
                s_rx_len      = 0;
                g_robot.rx_bad++;
                LOG("[rx] ligne trop longue, ignoree");
                continue;
            }
            if (s_rx_len == 0) continue;        /* \r\n ou ligne vide */
            s_rx[s_rx_len] = '\0';
            exec_line(s_rx);
            s_rx_len = 0;
            continue;
        }

        if (s_rx_overflow) continue;            /* on purge jusqu'au \n */

        if (s_rx_len + 1 >= sizeof(s_rx)) {
            s_rx_overflow = true;
            continue;
        }
        s_rx[s_rx_len++] = (char)c;
    }
    /* Budget epuise : le reste sera lu au tour suivant. La boucle de
     * controle n'est jamais retardee par un flot d'entree soutenu. */
}

/* ------------------------------------------------------------------------ */
/* Emission de la trame d'etat                                               */
/* ------------------------------------------------------------------------ */

void protocol_send_state(uint32_t t_ms)
{
    const encoders_t *e = encoders_state();
    const bno055_t   *m = bno055_state();

    char line[TX_LINE_MAX];
    int n = snprintf(line, sizeof(line),
                     "S %lu %ld %ld %ld %ld %d %d %d %d %d %d",
                     (unsigned long)t_ms,
                     (long)e->count[ENC_L], (long)e->count[ENC_R],
                     (long)e->vel_i[ENC_L], (long)e->vel_i[ENC_R],
                     m->gyro[0],  m->gyro[1],  m->gyro[2],
                     m->accel[0], m->accel[1], m->accel[2]);

    if (n <= 0) return;
    if (n > (int)sizeof(line) - 4) n = (int)sizeof(line) - 4;   /* paranoia */

    uint8_t chk = 0;
    for (int i = 0; i < n; i++) chk ^= (uint8_t)line[i];

    n += snprintf(line + n, sizeof(line) - (size_t)n, " %02X", chk);

    /* stdio_put_string ecrit directement dans le pilote : ni la mise en
     * tampon de la libc, ni l'analyse de format de printf, ni traduction
     * CR/LF (troisieme argument a false). Le \n final est ajoute par le
     * deuxieme argument. L'ecriture est bornee par
     * PICO_STDIO_USB_STDOUT_TIMEOUT_US : si le Pi ne lit pas, les octets
     * sont abandonnes et la boucle 50 Hz continue. */
    stdio_put_string(line, n, true, false);
}
