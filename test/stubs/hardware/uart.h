#pragma once
#include <stddef.h>
#include <stdint.h>
typedef int uart_inst_t;
#define uart0 ((uart_inst_t *)0)
#define uart1 ((uart_inst_t *)1)
extern void h_log(const char *s, size_t n);
static inline void uart_write_blocking(uart_inst_t *u, const uint8_t *src, size_t len)
{ (void)u; h_log((const char *)src, len); }
