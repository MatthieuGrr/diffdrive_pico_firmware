/*
 * diffdrive_pico_firmware — logs de debug sur UART0.
 *
 * REGLE ABSOLUE : stdio (printf, putchar, getchar) est reserve au protocole
 * robot sur USB CDC. Aucun message de debug ne doit passer par stdio.
 * Tout ce qui est destine a l'humain sort d'ici, via les fonctions UART
 * brutes du SDK, qui court-circuitent completement stdio.
 *
 * LOG() formate avec snprintf dans un tampon de pile puis emet le tampon
 * avec uart_write_blocking. C'est bloquant par construction : a 115200 bauds
 * une ligne de 60 caracteres occupe ~5 ms. Sans consequence pour le mode
 * normal et les modes de test lents, a garder a l'esprit pour t 4 qui trace
 * a 50 Hz (voir LOG_UART_BAUD dans config.h).
 */
#ifndef DIFFDRIVE_PICO_LOG_H
#define DIFFDRIVE_PICO_LOG_H

#include <stdio.h>
#include "hardware/uart.h"
#include "config.h"

void log_init(void);

#if LOG_ENABLE

/* Ajoute CR+LF : le terminal de l'adaptateur USB-UART est content, et cela
 * n'a aucun rapport avec le framing du protocole USB, qui est ailleurs. */
#define LOG(fmt, ...)                                                        \
    do {                                                                     \
        char _lb[LOG_BUF_SIZE];                                              \
        int _n = snprintf(_lb, sizeof(_lb), fmt "\r\n", ##__VA_ARGS__);      \
        if (_n > 0) {                                                        \
            if (_n > (int)sizeof(_lb) - 1) _n = (int)sizeof(_lb) - 1;        \
            uart_write_blocking(LOG_UART, (const uint8_t *)_lb, (size_t)_n); \
        }                                                                    \
    } while (0)

#else
#define LOG(fmt, ...) do { } while (0)
#endif

#endif /* DIFFDRIVE_PICO_LOG_H */
