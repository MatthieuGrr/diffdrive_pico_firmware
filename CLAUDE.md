# diffdrive_pico_firmware — firmware Pico 2

## Cible
Raspberry Pi Pico 2 (RP2350), cœur ARM Cortex-M33, pico-sdk v2.3.0.
Étage bas niveau d'une base différentielle. L'hôte est un Raspberry Pi 5
(ROS 2) ou une Jetson (logiciel tiers) : tout passe par le protocole série
documenté dans docs/PROTOCOLE.md.

## Matériel (v2)
- Moteurs 36-555 planétaires 24 V 160 tr/min, encodeurs SCX-555 (16 PPR
  moteur, drain ouvert, alimentés en 3,3 V).
- Driver Cytron MDD10A : PWM + DIR par canal, 20 kHz max, PWM=0 = FREIN
  (pas de roue libre).
- INA226 sur I2C1 (shunt 10 mΩ, ±8,19 A), ALERT sur GP20.
- BNO055 sur I2C0.
- Batterie d'outillage 20 V (5S Li-ion).

## Brochage
| Fonction | Broches |
|---|---|
| UART0 (logs) | TX=GP0, RX=GP1 |
| MDD10A | PWM1=GP2, DIR1=GP3 (gauche), PWM2=GP4, DIR2=GP5 (droite) |
| Encodeur G | A=GP6, B=GP7 |
| Encodeur D | A=GP8, B=GP9 |
| I2C0 (BNO055) | SDA=GP16, SCL=GP17 |
| I2C1 (INA226) | SDA=GP18, SCL=GP19, ALERT=GP20 |

Tout en 3.3V. Le GPIO du RP2350 n'est pas tolérant 5V. Schéma : docs/cablage.png.

## Règles non négociables
- stdio USB == protocole robot. AUCUN printf de debug.
- Les logs passent par uart_write_blocking sur uart0, jamais par printf.
- `hardware_pwm` doit être dans target_link_libraries : l'assistant de
  création de projet ne l'ajoute pas.
- PICO_STDIO_DEFAULT_CRLF=0 dans le CMakeLists.
- Le firmware ignore rayon de roue, entraxe et counts/tour. Il ne connaît que
  des ticks. La cinématique vit côté hôte.
- Watchdog 300 ms : perte du lien série => moteurs freinés.
- Défauts batterie (surintensité, sous-tension) verrouillés, effacés
  seulement par 's' quand la cause a disparu.
- Compteurs d'encodeurs cumulés dans les trames, jamais des deltas.
- Protocole : toute modification de la trame ou des commandes met à jour
  PROTO_VERSION, docs/PROTOCOLE.md et les bancs d'essai. Ne jamais
  renuméroter un bit de flags existant ; ajouter des champs en fin de trame.

## À mesurer (v2)
Counts par tour de roue, pleine échelle en ticks/s (→ PID_KFF), sens des
moteurs et des encodeurs, gains PID, courant réel (→ seuils OC).

## Développement
Pas de sonde SWD. Flash en BOOTSEL, debug par LOG() sur UART0 avec un
adaptateur USB-UART. Bancs d'essai hôte : ./test/run.sh.
