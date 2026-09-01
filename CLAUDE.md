# diffdrive_pico_firmware — firmware Pico 2

## Cible
Raspberry Pi Pico 2 (RP2350), cœur ARM Cortex-M33, pico-sdk v2.3.0.
Étage bas niveau d'un robot ROS 2 ; le maître est un Raspberry Pi 5.

## Brochage
| Fonction | Broches |
|---|---|
| Moteur G (DRV8871) | IN1=GP2, IN2=GP3 |
| Moteur D (DRV8871) | IN1=GP4, IN2=GP5 |
| Encodeur G | A=GP6, B=GP7 |
| Encodeur D | A=GP8, B=GP9 |
| I2C0 (BNO055) | SDA=GP16, SCL=GP17 |
| UART0 (logs) | TX=GP0, RX=GP1 |

Encodeurs alimentés en 3.3V. Le GPIO du RP2350 n'est pas tolérant 5V.

## Règles non négociables
- stdio USB == protocole robot. AUCUN printf de debug.
- Les logs passent par uart_write_blocking sur uart0, jamais par printf.
- `hardware_pwm` doit être dans target_link_libraries : l'assistant de
  création de projet ne l'ajoute pas.
- PICO_STDIO_DEFAULT_CRLF=0 dans le CMakeLists.
- Le firmware ignore rayon de roue, entraxe et counts/tour. Il ne connaît que
  des ticks. La cinématique vit côté Pi.
- Watchdog 300 ms : perte du lien série => PWM à zéro.
- Compteurs d'encodeurs cumulés dans les trames, jamais des deltas.

## Contexte matériel
1540 counts par tour de roue (11 PPR x4 x 35:1) — valeur à vérifier
empiriquement.
Moteurs 24V, 170 RPM.

## Développement
Pas de sonde SWD. Flash en BOOTSEL, debug par LOG() sur UART0 avec un
adaptateur USB-UART.