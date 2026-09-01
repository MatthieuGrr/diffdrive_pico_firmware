#include "log.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"

void log_init(void)
{
    uart_init(LOG_UART, LOG_UART_BAUD);
    gpio_set_function(PIN_LOG_TX, UART_FUNCSEL_NUM(LOG_UART, PIN_LOG_TX));
    gpio_set_function(PIN_LOG_RX, UART_FUNCSEL_NUM(LOG_UART, PIN_LOG_RX));
    uart_set_format(LOG_UART, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(LOG_UART, true);
    /* Pas de controle de flux : l'adaptateur USB-UART n'a que TX/RX/GND. */
    uart_set_hw_flow(LOG_UART, false, false);
}
