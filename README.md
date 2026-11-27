# diffdrive_pico_firmware

Firmware de l'étage bas niveau d'une base robotique différentielle.

Le Raspberry Pi Pico 2 (RP2350) pilote deux moteurs, lit deux encodeurs en
quadrature, une centrale inertielle et un capteur tension / courant de
batterie, et dialogue en ASCII sur USB CDC avec un ordinateur hôte : un
Raspberry Pi 5 sous ROS 2 (`ros2_control`), une Jetson, ou n'importe quel
programme qui parle le protocole.

**Écrire son propre pilote côté hôte : voir [`docs/PROTOCOLE.md`](docs/PROTOCOLE.md).**
Câblage : [`docs/cablage.png`](docs/cablage.png).

## Frontière d'abstraction

**Le Pico ne connaît ni le rayon de roue, ni l'entraxe, ni le nombre de counts
par tour.** Il ne manipule que des *ticks* et des *ticks/s*. Toute la
cinématique et toute la calibration vivent côté hôte. Aucune constante physique
n'apparaît dans le firmware — c'est délibéré, et c'est ce qui permet de changer
de roues ou de réducteur sans reflasher.

| Le Pico fait | L'hôte fait |
|---|---|
| PWM, décodage quadrature, PID de vitesse en ticks/s | odométrie, cinématique différentielle |
| horodatage, compteurs cumulés, IMU brute, tension / courant | conversion SI, fusion |
| coupure sur perte du lien, surintensité, sous-tension | limitation de vitesse et d'accélération |

Le firmware **n'applique aucune limitation** de vitesse, d'accélération ou de
rapport cyclique : il exécute la consigne telle quelle. Ses seules sécurités
sont celles que l'hôte ne peut pas garantir lui-même : le chien de garde de
300 ms, et les coupures sur surintensité et sous-tension de la batterie.

## Matériel

| | |
|---|---|
| Carte | Raspberry Pi Pico 2 (RP2350, Cortex-M33), pico-sdk 2.3.0 |
| Moteurs | 2 × motoréducteur planétaire 36-555, 24 V, 160 tr/min en sortie |
| Driver | Cytron MDD10A, deux canaux, 10 A continus, commande PWM + DIR |
| Encodeurs | SCX-555 magnétiques, 16 PPR sur l'arbre moteur, drain ouvert |
| IMU | BNO055 sur I2C0, mode IMU (fusion 6 axes, magnétomètre désactivé) |
| Batterie | INA226 sur I2C1, shunt 10 mΩ en série sur le + batterie (±8,19 A) |
| Alimentation | batterie d'outillage 20 V (5S Li-ion) |

Le magnétomètre est volontairement coupé : sur une base à moteurs à courant
continu il ne mesure guère que le champ des moteurs.

**Counts par tour de roue : à mesurer** (`t 1`, 10 tours de roue à la main).
Attendu : 16 PPR × 4 (quadrature) × rapport du réducteur. Pour mémoire, la v1
(GA25-370) donnait 1496, et le rapport réel s'était révélé différent de
l'annonce : ne pas se fier au chiffre du vendeur.

Le MDD10A **n'a pas de roue libre** : PWM à zéro = moteur court-circuité, donc
freiné. Duty nul, commande `s` et toutes les coupures de sécurité freinent les
roues.

## Brochage

Schéma complet : [`docs/cablage.png`](docs/cablage.png) (source
[`docs/cablage.svg`](docs/cablage.svg), régénéré par `docs/gen_cablage.py`).

| Fonction | Broches (n° physique) |
|---|---|
| UART0 (logs de debug) | TX = GP0 (1), RX = GP1 (2), GND (3) |
| MDD10A | PWM1 = GP2 (4), DIR1 = GP3 (5), PWM2 = GP4 (6), DIR2 = GP5 (7), GND (8) |
| Encodeur G | A = GP6 (9), B = GP7 (10) |
| Encodeur D | A = GP8 (11), B = GP9 (12), GND (13) |
| I2C0 (BNO055) | SDA = GP16 (21), SCL = GP17 (22), GND (23) |
| I2C1 (INA226) | SDA = GP18 (24), SCL = GP19 (25), ALERT = GP20 (26), GND (28) |
| Arrêt d'urgence | contact NF entre GP21 (27) et GND (28), tirage 4,7 kΩ vers 3V3 |
| Rails | 3V3(OUT) (36), GND (38) |

Canal 1 du MDD10A = moteur gauche, canal 2 = moteur droit.

Fils des moteurs 36-555 : **blanc** moteur +, **jaune** moteur −, **vert**
VCC encodeur, **bleu** GND encodeur, **noir** signal A, **rouge** signal B.
Attention, le rouge et le noir sont des *signaux*, pas l'alimentation.

> **Le GPIO du RP2350 n'est pas tolérant 5 V.** Encodeurs, BNO055 et INA226
> sont alimentés par le rail 3V3 : les tirages embarqués de leurs modules
> restent ainsi à 3,3 V. Ne pas relier VBUS ni VSYS : le Pico est alimenté
> par l'USB de l'hôte.

L'INA226 a son propre bus I2C : un BNO055 qui bloque le bus I2C0 ne peut pas
rendre muet le capteur de sécurité. Sa sortie ALERT freine les moteurs sur
interruption, sans attendre la boucle de contrôle.

L'arrêt d'urgence coupe la puissance des moteurs par lui-même, en
matériel. Le Pico ne fait que le **surveiller** par un contact auxiliaire
**normalement fermé** du bouton, entre GP21 et GND : bouton relâché, GP21
est à 0 ; bouton enfoncé ou fil coupé, GP21 remonte à 1 et le drapeau `ESTOP`
(256) se lève. Une rupture de câblage arrête donc le robot au lieu de passer
inaperçue. Prévoir une résistance de tirage de 4,7 kΩ vers 3V3 (le tirage
interne de ~50 kΩ est trop faible pour un fil qui longe les câbles moteur)
et, si possible, 100 nF entre GP21 et GND. Sans bouton câblé, le robot reste
en `ESTOP` : c'est voulu. Pour un banc sans bouton, relier GP21 à GND ou
mettre `ESTOP_ENABLE` à 0 dans `config.h`.

Une contrainte est vérifiée à la compilation par un `_Static_assert` dans
[`config.h`](include/config.h) : le canal B d'un encodeur doit être `A + 1` (le
programme PIO lit deux broches consécutives).

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
- `PICO_STDIO_USB_STDOUT_TIMEOUT_US=1000` : si l'hôte ne lit pas son port, les
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

Produit `build/diffdrive_pico_firmware.uf2` (~75 Ko de flash, 3,5 Ko de RAM).

Sans l'extension VS Code (Debian/Ubuntu) : `apt install gcc-arm-none-eabi
libnewlib-arm-none-eabi libstdc++-arm-none-eabi-newlib cmake ninja-build`, puis
`git clone -b 2.3.0 https://github.com/raspberrypi/pico-sdk` avec le
sous-module `lib/tinyusb`, et `PICO_SDK_PATH` pointé dessus.

**Flash** : maintenir BOOTSEL au branchement USB, puis copier le `.uf2` sur le
volume `RPI-RP2`. Pas de sonde SWD nécessaire — le debug se fait par `LOG()` sur
UART0, lu avec un adaptateur USB-UART.

## Protocole série (v2)

Spécification complète, destinée à qui écrit un pilote côté hôte :
**[`docs/PROTOCOLE.md`](docs/PROTOCOLE.md)**. Résumé :

### Hôte → Pico

| Commande | Effet |
|---|---|
| `v <l> <r>` | consigne de vitesse en ticks/s, entiers signés |
| `o <l> <r>` | boucle ouverte, rapport cyclique −1000..1000 |
| `s` | arrêt immédiat (freinage), **et acquittement des défauts verrouillés** |
| `r` | remise à zéro des compteurs |
| `k <kp> <ki> <kd>` | réglage des gains PID à chaud (les deux roues) |
| `t <n>` | entrée en mode de test *n* |
| `i` | identification : réponse `I <nom> <version> <proto> <chk>` |

Le parseur est non bloquant et strict : un argument manquant, surnuméraire ou
non numérique fait rejeter la ligne entière sans rien modifier. Toute commande
valide réarme le chien de garde.

### Pico → hôte, émis spontanément à 50 Hz

```
S <t_ms> <enc_l> <enc_r> <vel_l> <vel_r> <gx> <gy> <gz> <ax> <ay> <az> <vbat_mv> <ibat_ma> <flags> <chk>
```

Les onze premiers champs sont ceux de la v1, à la même place : un lecteur v1
qui lit les champs par position continue de fonctionner. `vbat_mv` et
`ibat_ma` viennent de l'INA226 (0 s'il est absent) ; `flags` est un champ de
bits décrit dans [`docs/PROTOCOLE.md`](docs/PROTOCOLE.md) (perte du lien,
surintensité, sous-tension, batterie débranchée, capteurs absents, mode de
test, moteurs interdits).

Checksum : OU exclusif de tous les octets depuis la lettre initiale jusqu'au
caractère qui précède l'espace séparant le dernier champ de `chk`, émis en
deux chiffres hexadécimaux majuscules.

```python
import functools, operator

def verifier(ligne: str) -> bool:
    charge, chk = ligne.rstrip("\n").rsplit(" ", 1)
    return functools.reduce(operator.xor, charge.encode(), 0) == int(chk, 16)
```

### Protections batterie et arrêt d'urgence

| Défaut | Déclenchement | Effacement |
|---|---|---|
| Surintensité instantanée | une mesure de l'INA226 ≥ `OC_TRIP_MA` (7,5 A) : ALERT, freinage sous interruption | `s`, une fois ALERT relâchée |
| Surintensité soutenue | ≥ `OC_SUSTAINED_MA` (5 A) pendant `OC_SUSTAINED_MS` (1 s) | `s`, courant revenu sous 5 A |
| Sous-tension | < `UV_TRIP_MV` (15 V) pendant `UV_DEBOUNCE_MS` (2 s) | `s`, tension ≥ `UV_CLEAR_MV` (16 V) |
| Arrêt d'urgence | GP21 à 1 pendant `ESTOP_TRIP_MS` (40 ms) | `s`, bouton relâché depuis `ESTOP_RELEASE_MS` (200 ms) |

Les défauts sont **verrouillés** : moteurs freinés, consignes `v`/`o` acceptées
mais sans effet, jusqu'à un `s` explicite *et* la disparition de la cause.
Sous 5 V, la batterie est considérée débranchée (robot au banc sur USB seul) :
drapeau `NO_BATTERY`, pas de défaut. Seuils de départ, à ajuster après mesure
du courant réel en roulage.

## Modes de test intégrés

Envoyés par la commande `t` sur USB, **sortie lisible sur UART0**. Ils
permettent d'isoler chaque étage sans reflasher.

| Mode | Rôle |
|---|---|
| `t 0` | mode normal (défaut) |
| `t 1` | encodeurs seuls, moteurs coupés, compteurs à 5 Hz |
| `t 2` | scan des deux bus I2C, BNO055 (CHIP_ID 0xA0) et INA226 (0x5449 / 0x2260), puis gyro/accel et tension/courant à 5 Hz |
| `t 3` | moteurs en boucle ouverte, rampe lente, duty, vitesse mesurée et courant côte à côte |
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
  qui pilote les moteurs, pas l'hôte. Il reprend dès `t 0`, qui laisse les
  moteurs à l'arrêt jusqu'à un `v` ou un `o` explicite.
- **Les protections batterie restent actives en mode de test.**

## Configuration

Tout se règle dans [`config.h`](include/config.h). Les entrées qui comptent :

| Constante | Défaut | Rôle |
|---|---|---|
| `MOT_L_INVERT` / `MOT_R_INVERT` | 0 / 1 | sens des moteurs, **à vérifier** avec `t 3` |
| `ENC_L_INVERT` / `ENC_R_INVERT` | 0 / 1 | signe des compteurs, **à vérifier** avec `t 1` et `t 3` |
| `PWM_FREQ_HZ` | 20000 | maximum du MDD10A, 7500 pas à 150 MHz |
| `CONTROL_HZ` | 50 | boucle PID, sur timer matériel |
| `CMD_TIMEOUT_US` | 300000 | chien de garde logiciel |
| `PID_KP/KI/KD_DEFAULT` | 0.087 / 0.40 / 0.0 | **provisoires** (gains v1 divisés par deux), à remesurer |
| `PID_KFF` | 0.0 | anticipation, désactivée jusqu'à la mesure de la pleine échelle : `DUTY_MAX ÷ (ticks/s à duty 1000)` |
| `OC_TRIP_MA` / `OC_SUSTAINED_MA` | 7500 / 5000 | surintensité instantanée (matérielle) / soutenue |
| `UV_TRIP_MV` / `UV_CLEAR_MV` | 15000 / 16000 | sous-tension et hystérésis, batterie 5S |
| `INA_REQUIRED` | 0 | 1 = moteurs interdits si l'INA226 ne répond pas |
| `LOG_UART_BAUD` | 115200 | monter à 460800 réduit la gigue de `t 4` |

## Bancs d'essai hôte

```bash
./test/run.sh
```

Tourne sur le PC, sans SDK ni carte branchée : `gcc` et `python3` suffisent.
Plus d'une centaine de cas couvrant ce qui ne dépend pas du silicium, sur les modules où une erreur
coûterait le plus cher à diagnostiquer au banc :

- **décodeur PIO** — simulé instruction par instruction sur le binaire
  assemblé : x4 dans les deux sens, un tour de roue à 1496 counts, transitions
  illégales ignorées, rebonds sans dérive, état de départ quelconque. Limite
  mesurée : un état doit tenir ≥ 12 cycles PIO, soit 1 µs au réglage livré —
  marge ×230 sur les ~1,1 kHz du robot ;
- **protocole** — les 7 commandes, 14 formes invalides rejetées, trame v2, ligne trop
  longue purgée puis resynchronisation, checksum vérifié par un contrôleur
  indépendant écrit depuis la documentation ci-dessus et non depuis le code
  émetteur ;
- **moteurs** — les 2001 consignes × 2 roues, en simulant le compteur PWM pas
  à pas et en appliquant la table de vérité du MDD10A, plus l'inhibition de
  sécurité ;
- **sécurité** — surintensité instantanée et soutenue, sous-tension et son
  hystérésis, batterie débranchée, capteur absent, conversions de l'INA226 ;
- **PID** — anti-emballement comparé à un intégrateur naïf.

`protocol.c`, `motors.c`, `pid.c` et `safety.c` sont compilés **tels quels** pour l'hôte ;
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

1. **Avant de brancher les moteurs**, vérifier au multimètre les couleurs de
   fils (vert / bleu = alimentation de l'encodeur) : une inversion détruit
   l'encodeur.
2. **À vide, moteurs non alimentés.** Flash en BOOTSEL, USB-UART sur GP0/GND à
   115200. Le bandeau doit lister brochage, BNO055 et INA226 présents.
   LED : clignotement lent = pas de commande reçue.
3. **`t 1`** — tourner une roue de 10 tours exacts à la main : le compteur
   divisé par 10 donne les counts par tour, à reporter dans
   `TEST_COUNTS_PER_REV` et côté hôte. Dans l'autre sens, le signe s'inverse.
4. **`t 2`** — CHIP_ID = 0xA0 et INA226 MANUF 0x5449 / DIE 0x2260. Tension
   batterie cohérente avec un voltmètre, courant proche de 0 à l'arrêt.
5. **Alimenter les moteurs, robot surélevé. `t 3`** — vérifier que duty et
   vitesse ont le **même signe** sur chaque roue et qu'un duty positif fait
   avancer. Corriger `ENC_x_INVERT` / `MOT_x_INVERT` puis reflasher. Noter les
   ticks/s à duty +1000 (pleine échelle → `PID_KFF`) et le courant.
6. **Régler le PID au sol** avec `tools/tune_pid.py` (après y avoir reporté
   `KFF` et un `MAX_TICKS_S` adapté à la nouvelle pleine échelle).
7. **`t 0`, puis depuis l'hôte** — vérifier les checksums, envoyer `v 500 500`
   et **débrancher le câble** : les roues doivent être freinées sous 300 ms.
8. **Surintensité** — robot au sol, bloquer une roue en `o 1000 1000` : coupure
   attendue en ~1 s, drapeau `OVERCURRENT`, puis `s` pour réarmer.

## Organisation du code

```
CMakeLists.txt  pico_sdk_import.cmake  README.md  CLAUDE.md
include/   en-têtes du firmware
src/       implémentation
pio/       programmes PIO
test/      bancs d'essai hôte
docs/      protocole pour l'hôte, schéma de câblage
```

| Fichier | Rôle |
|---|---|
| [`src/main.c`](src/main.c) | boucle principale, chien de garde, sécurité, ordonnancement 50 Hz |
| [`config.h`](include/config.h) | brochage, réglages, gains, assertions de compilation |
| [`quadrature.pio`](pio/quadrature.pio) | décodage quadrature x4, 28 instructions, 1 SM par encodeur |
| [`encoders.c`](src/encoders.c) | lecture des compteurs, vitesse, remise à zéro par offset |
| [`motors.c`](src/motors.c) | MDD10A, PWM 20 kHz + DIR, inhibition de sécurité |
| [`ina226.c`](src/ina226.c) | tension et courant batterie, alerte de surintensité sur interruption |
| [`safety.c`](src/safety.c) | logique pure des défauts verrouillés et des drapeaux |
| [`pid.c`](src/pid.c) | PID de vitesse, dérivée sur la mesure, anti-emballement |
| [`bno055.c`](src/bno055.c) | IMU, absence non fatale, redétection à chaud |
| [`protocol.c`](src/protocol.c) | parseur non bloquant, trame d'état, checksum |
| [`testmodes.c`](src/testmodes.c) | modes `t 1` à `t 4` |
| [`log.h`](include/log.h) | macro `LOG()` sur UART0 |

Un seul fil d'exécution : la boucle principale. Le timer matériel 50 Hz ne fait
qu'armer un compteur, et l'interruption ALERT de l'INA226 ne fait que freiner
les moteurs et lever un drapeau ; I2C et écritures USB ne sont jamais exécutés
sous interruption.

## Licence

Pas encore définie. En l'absence de fichier `LICENSE`, le dépôt reste sous
copyright par défaut — personne n'est autorisé à le réutiliser.
