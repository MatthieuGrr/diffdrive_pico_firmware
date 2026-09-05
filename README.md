# diffdrive_pico_firmware

Firmware de l'étage bas niveau d'une base robotique différentielle sous ROS 2.

Le Raspberry Pi Pico 2 (RP2350) pilote deux moteurs, lit deux encodeurs en
quadrature et une centrale inertielle, et dialogue en ASCII sur USB CDC avec un
Raspberry Pi 5 qui porte la pile ROS 2 et une interface `ros2_control`.

## Frontière d'abstraction

**Le Pico ne connaît ni le rayon de roue, ni l'entraxe, ni le nombre de counts
par tour.** Il ne manipule que des *ticks* et des *ticks/s*. Toute la
cinématique et toute la calibration vivent côté Pi. Aucune constante physique
n'apparaît dans le firmware — c'est délibéré, et c'est ce qui permet de changer
de roues ou de réducteur sans reflasher.

| Le Pico fait | Le Pi fait |
|---|---|
| PWM, décodage quadrature, PID de vitesse en ticks/s | odométrie, cinématique différentielle |
| horodatage, compteurs cumulés, IMU brute | conversion SI, fusion, `diff_drive_controller` |
| coupure de sécurité sur perte du lien | limitation de vitesse et d'accélération |

Le firmware **n'applique aucune limitation** de vitesse, d'accélération ou de
rapport cyclique : il exécute la consigne telle quelle. La seule sécurité non
négociable est le chien de garde de 300 ms.

## Matériel

| | |
|---|---|
| Carte | Raspberry Pi Pico 2 (RP2350, Cortex-M33), pico-sdk 2.3.0 |
| Moteurs | 2 × GA25-370, 24 V, 170 tr/min en sortie de réducteur |
| Drivers | 2 × DRV8871, un par moteur, commandés par 2 PWM (IN1/IN2) |
| Encodeurs | quadrature à effet Hall, 11 PPR par canal, réduction 34:1 |
| IMU | BNO055 en I2C, mode IMU (fusion 6 axes, magnétomètre désactivé) |

Le magnétomètre est volontairement coupé : sur une base à moteurs à courant
continu il ne mesure guère que le champ des moteurs.

**1496 counts par tour de roue**, soit 11 PPR × 4 (quadrature) × 34:1. Valeur
**mesurée** sur 10 tours de roue (14 960 ticks), pas déduite d'une fiche
technique : le réducteur livré est en 34:1 et non 35:1. Utilisée uniquement
**côté Pi** — se tromper de 35:1 coûte 2,9 % sur les distances et 10° sur un
tour complet du robot.

## Brochage

| Fonction | Broches |
|---|---|
| Moteur G (DRV8871) | IN1 = GP2, IN2 = GP3 |
| Moteur D (DRV8871) | IN1 = GP4, IN2 = GP5 |
| Encodeur G | A = GP6, B = GP7 |
| Encodeur D | A = GP8, B = GP9 |
| I2C0 (BNO055) | SDA = GP16, SCL = GP17 |
| UART0 (logs de debug) | TX = GP0, RX = GP1 |

> **Le GPIO du RP2350 n'est pas tolérant 5 V.** Les encodeurs doivent être
> alimentés en 3.3 V.

Deux contraintes sont vérifiées à la compilation par des `_Static_assert` dans
[`config.h`](include/config.h) : le canal B d'un encodeur doit être `A + 1` (le
programme PIO lit deux broches consécutives), et IN1/IN2 d'un même moteur
doivent partager la même slice PWM.

## Règle stdio — critique

Le projet est généré avec « Console over USB » activé et « Console over UART »
**désactivé**. En conséquence :

- `printf` / `getchar` / stdio **sont** le protocole robot, sur USB CDC. Rien
  d'autre n'y transite.
- Aucun `printf` de debug. Les logs passent exclusivement par la macro `LOG()`
  ([`log.h`](include/log.h)), bâtie sur `snprintf` + `uart_write_blocking` sur uart0,
  qui court-circuite stdio.
- `PICO_STDIO_DEFAULT_CRLF=0` dans le CMakeLists, sinon le SDK traduit `\n` en
  `\r\n` et corrompt le tramage.
- `PICO_STDIO_USB_STDOUT_TIMEOUT_US=1000` : si le Pi ne lit pas son port, les
  octets sont abandonnés au bout d'une milliseconde. La boucle 50 Hz ne peut
  jamais se faire bloquer par l'hôte.

## Compilation et flash

Depuis VS Code, l'extension Raspberry Pi Pico suffit — `.vscode/` est fourni.

En ligne de commande :

```bash
export PICO_SDK_PATH=~/.pico-sdk/sdk/2.3.0
export PATH=~/.pico-sdk/toolchain/15_2_Rel1/bin:~/.pico-sdk/ninja/v1.13.2:~/.pico-sdk/cmake/v4.3.4/bin:$PATH

cmake -S . -B build -G Ninja -DPICO_BOARD=pico2
cmake --build build
```

Produit `build/diffdrive_pico_firmware.uf2` (~70 Ko de flash, 3,5 Ko de RAM).

**Flash** : maintenir BOOTSEL au branchement USB, puis copier le `.uf2` sur le
volume `RPI-RP2`. Pas de sonde SWD nécessaire — le debug se fait par `LOG()` sur
UART0, lu avec un adaptateur USB-UART.

## Protocole série (USB CDC, lignes ASCII terminées par `\n`)

### Pi → Pico

| Commande | Effet |
|---|---|
| `v <l> <r>` | consigne de vitesse en ticks/s, entiers signés |
| `o <l> <r>` | boucle ouverte, rapport cyclique −1000..1000 |
| `s` | arrêt immédiat |
| `r` | remise à zéro des compteurs |
| `k <kp> <ki> <kd>` | réglage des gains PID à chaud (les deux roues) |
| `t <n>` | entrée en mode de test *n* |

Le parseur est non bloquant et strict : un argument manquant, surnuméraire ou
non numérique fait rejeter la ligne entière sans rien modifier. Toute commande
valide réarme le chien de garde.

### Pico → Pi, émis spontanément à 50 Hz

```
S <t_ms> <enc_l> <enc_r> <vel_l> <vel_r> <gx> <gy> <gz> <ax> <ay> <az> <chk>
```

| Champ | Signification |
|---|---|
| `t_ms` | millisecondes depuis le démarrage |
| `enc_*` | compteurs **cumulés** int32 — jamais des deltas : une trame perdue est rattrapée par la suivante |
| `vel_*` | vitesse mesurée en ticks/s |
| `g*`, `a*` | gyroscope et accéléromètre **bruts** du BNO055 (int16), non convertis |
| `chk` | checksum XOR, 2 chiffres hexadécimaux majuscules |

Conversion SI, à faire côté Pi (`UNIT_SEL = 0x00`) :
**accéléromètre 100 LSB/(m/s²)**, **gyroscope 16 LSB/(deg/s)**.

Si le BNO055 est absent, les six champs valent 0 et le robot roule quand même.

### Checksum — définition exacte

`chk` est le OU exclusif de **tous les octets depuis le `S` inclus jusqu'au
dernier caractère précédant l'espace qui sépare `<az>` de `<chk>`**. L'espace
séparateur, le champ `chk` lui-même et le `\n` final en sont exclus.

```python
import functools, operator

def verifier(ligne: str) -> bool:
    charge, chk = ligne.rstrip("\n").rsplit(" ", 1)
    return functools.reduce(operator.xor, charge.encode(), 0) == int(chk, 16)

assert verifier("S 123456 1496 -1496 4362 -4362 12 -34 56 -78 910 -1011 68\n")
```

## Modes de test intégrés

Envoyés par la commande `t` sur USB, **sortie lisible sur UART0**. Ils
permettent d'isoler chaque étage sans reflasher.

| Mode | Rôle |
|---|---|
| `t 0` | mode normal (défaut) |
| `t 1` | encodeurs seuls, moteurs coupés, compteurs à 5 Hz |
| `t 2` | scan du bus I2C, CHIP_ID (0xA0 attendu), calibration, dump gyro/accel à 5 Hz |
| `t 3` | moteurs en boucle ouverte, rampe lente, duty **et** vitesse mesurée côte à côte |
| `t 4` | PID fermé, échelon de consigne, trace à 50 Hz pour régler les gains |

`t 3` est celui qui révèle immédiatement un encodeur qui compte à l'envers par
rapport au sens de son moteur : duty et vitesse doivent avoir le même signe.

Il faut deux liaisons — la commande entre par l'USB, la sortie ressort par
l'UART :

```bash
# Terminal A — lecture des logs (adaptateur USB-UART sur GP0 + GND)
screen /dev/ttyUSB0 115200          # quitter : Ctrl-A puis K

# Terminal B — envoi des commandes (USB du Pico)
stty -F /dev/ttyACM0 raw -echo
printf 't 1\n' > /dev/ttyACM0
```

Le `raw` compte : sans lui la couche tty traduit `\n` en `\r\n`.

Deux comportements à connaître :

- **`t 3` et `t 4` alimentent les moteurs** dès réception. Robot surélevé,
  roues dans le vide.
- **Le chien de garde de 300 ms est suspendu en mode de test** : c'est le test
  qui pilote les moteurs, pas le Pi. Il reprend dès `t 0`, qui laisse les
  moteurs à l'arrêt jusqu'à un `v` ou un `o` explicite.

## Configuration

Tout se règle dans [`config.h`](include/config.h). Les entrées qui comptent :

| Constante | Défaut | Rôle |
|---|---|---|
| `MOTOR_DECAY_MODE` | `DECAY_SLOW` | décroissance lente (temps mort en freinage, réponse linéarisée à bas régime) ou `DECAY_FAST` (roue libre). **Une seule ligne à changer**, surchargeable par `-D` depuis CMake |
| `MOT_L_INVERT` / `MOT_R_INVERT` | 0 / 1 | sens des moteurs |
| `ENC_L_INVERT` / `ENC_R_INVERT` | 0 / 1 | signe des compteurs |
| `PWM_FREQ_HZ` | 25000 | au-dessus de l'audible, 6000 pas à 150 MHz |
| `CONTROL_HZ` | 50 | boucle PID, sur timer matériel |
| `CMD_TIMEOUT_US` | 300000 | chien de garde logiciel |
| `PID_KP/KI/KD_DEFAULT` | 0.12 / 0.60 / 0.0 | **placeholders**, à régler avec `t 4` et `k` |
| `PID_KFF` | 0.0 | anticipation optionnelle ; bon point de départ : `DUTY_MAX ÷ (ticks/s à pleine vitesse)` |
| `LOG_UART_BAUD` | 115200 | monter à 460800 réduit la gigue de `t 4` |

## Bancs d'essai hôte

```bash
./test/run.sh
```

Tourne sur le PC, sans SDK ni carte branchée : `gcc` et `python3` suffisent.
79 cas couvrant ce qui ne dépend pas du silicium, sur les modules où une erreur
coûterait le plus cher à diagnostiquer au banc :

- **décodeur PIO** — simulé instruction par instruction sur le binaire
  assemblé : x4 dans les deux sens, un tour de roue à 1496 counts, transitions
  illégales ignorées, rebonds sans dérive, état de départ quelconque. Limite
  mesurée : un état doit tenir ≥ 12 cycles PIO, soit 1 µs au réglage livré —
  marge ×230 sur les ~1,1 kHz du robot ;
- **protocole** — les 6 commandes, 13 formes invalides rejetées, ligne trop
  longue purgée puis resynchronisation, checksum vérifié par un contrôleur
  indépendant écrit depuis la documentation ci-dessus et non depuis le code
  émetteur ;
- **moteurs** — les 2001 consignes × 2 roues × 2 modes de décroissance, en
  simulant le compteur PWM pas à pas et en classant l'état du pont ;
- **PID** — anti-emballement comparé à un intégrateur naïf.

`protocol.c`, `motors.c` et `pid.c` sont compilés **tels quels** pour l'hôte ;
seuls les accès matériel sont bouchonnés (`test/stubs/`).

## Réglage automatique du PID

```bash
python3 tools/tune_pid.py        # robot AU SOL, espace dégagé, ~4 min
```

Identifie le modèle de chaque roue puis règle les gains par la mesure :

1. **contrôle préalable** — refuse de continuer si le binaire chargé n'est pas
   celui du dépôt (vérifié par le signe de la réponse) ;
2. **identification en boucle ouverte** — échelons **alternés avant/arrière**,
   donc le robot revient à son point de départ ; excursion de l'ordre d'un
   demi-mètre. Le modèle est ajusté sur la réponse en **position** : les
   compteurs cumulés sont exacts, alors que dériver amplifierait la
   quantification de 50 ticks/s, et cela évite de déconvoluer le filtre du
   firmware ;
3. **auto-réglage itératif** — la formule IMC de manuel donne ici jusqu'à 45 %
   de dépassement, parce qu'elle suppose qu'il n'y a pas d'anticipation : avec
   `PID_KFF`, l'anticipation et l'intégrateur fournissent tous deux le régime
   établi. Le script part donc d'un réglage volontairement mou et durcit
   d'après le dépassement **mesuré**, jusqu'à convergence.

Le réglage doit se faire **au sol, au poids final** : sur cales le moteur
n'entraîne que l'inertie de la roue, et les gains obtenus sont inutilisables
en charge.

En sortie, `reglage_pid.html` : un fichier autonome, sans dépendance, qui trace
toutes les mesures — réponses indicielles avec le modèle ajusté superposé,
caractéristique statique et sa régression, zone morte, et une courbe par
itération de l'auto-réglage. Chaque graphique est doublé d'un tableau de
valeurs et réagit au survol. Il est produit même si le réglage est interrompu.

L'outil se valide sans robot, sur un firmware simulé à paramètres connus :

```bash
python3 test/dryrun_tune.py      # ~2 min
```

## Première mise sous tension

1. **À vide, moteurs non alimentés.** Flash en BOOTSEL, USB-UART sur GP0/GND à
   115200. Le bandeau doit lister brochage, mode de décroissance compilé et
   présence du BNO055. LED : clignotement lent = pas de commande reçue.
2. **`t 1`** — tourner une roue d'un tour exact à la main : 1496. Dans
   l'autre sens, le signe s'inverse. Sinon, corriger `ENC_x_INVERT`.
3. **`t 2`** — CHIP_ID = 0xA0. `mag = 0` est normal en mode IMU. Laisser
   immobile quelques secondes pour voir `gyr` monter à 3.
4. **Alimenter les moteurs, robot surélevé. `t 3`** — vérifier que duty et
   vitesse ont le **même signe** sur chaque roue. Corriger `ENC_x_INVERT` /
   `MOT_x_INVERT` puis reflasher.
5. **`t 3` de nouveau** — noter les ticks/s à duty +1000 : c'est la pleine
   échelle, et `DUTY_MAX ÷ cette valeur` donne un bon `PID_KFF`.
6. **`t 4`** — régler à chaud par `k <kp> <ki> <kd>` jusqu'à une réponse propre.
7. **`t 0`, puis depuis le Pi** — vérifier les checksums, envoyer `v 500 500`
   et **débrancher le câble** : les deux PWM doivent tomber à zéro sous 300 ms.

## Organisation du code

```
CMakeLists.txt  pico_sdk_import.cmake  README.md  CLAUDE.md
include/   en-têtes du firmware
src/       implémentation
pio/       programmes PIO
test/      bancs d'essai hôte
```

| Fichier | Rôle |
|---|---|
| [`src/main.c`](src/main.c) | boucle principale, chien de garde, ordonnancement 50 Hz |
| [`config.h`](include/config.h) | brochage, réglages, gains, assertions de compilation |
| [`quadrature.pio`](pio/quadrature.pio) | décodage quadrature x4, 28 instructions, 1 SM par encodeur |
| [`encoders.c`](src/encoders.c) | lecture des compteurs, vitesse, remise à zéro par offset |
| [`motors.c`](src/motors.c) | DRV8871, PWM 25 kHz, modes de décroissance |
| [`pid.c`](src/pid.c) | PID de vitesse, dérivée sur la mesure, anti-emballement |
| [`bno055.c`](src/bno055.c) | IMU, absence non fatale, redétection à chaud |
| [`protocol.c`](src/protocol.c) | parseur non bloquant, trame d'état, checksum |
| [`testmodes.c`](src/testmodes.c) | modes `t 1` à `t 4` |
| [`log.h`](include/log.h) | macro `LOG()` sur UART0 |

Un seul fil d'exécution : la boucle principale. Le timer matériel 50 Hz ne fait
qu'armer un compteur ; I2C et écritures USB ne sont jamais exécutés sous
interruption, donc aucun verrou n'est nécessaire.

## Licence

Pas encore définie. En l'absence de fichier `LICENSE`, le dépôt reste sous
copyright par défaut — personne n'est autorisé à le réutiliser.
