# Base différentielle — protocole série v2

Ce document suffit pour écrire un pilote côté hôte (Jetson, PC, Raspberry Pi…)
dans n'importe quel langage. Le firmware n'a besoin d'aucun logiciel
particulier côté hôte : ni ROS ni bibliothèque, seulement un port série.

Un exemple de pilote minimal, commenté, est fourni :
[`exemple_pilote.py`](exemple_pilote.py) (Python 3 + pyserial).

## 1. Vue d'ensemble

Le microcontrôleur de la base (Raspberry Pi Pico 2) :

- asservit la vitesse de chaque roue (PID à 50 Hz) sur la consigne que vous
  envoyez ;
- publie 50 fois par seconde l'état complet : compteurs des encodeurs,
  vitesses mesurées, centrale inertielle, tension et courant de la batterie,
  drapeaux d'état ;
- coupe les moteurs tout seul si vous cessez d'envoyer des commandes, si le
  courant est excessif ou si la batterie est trop basse.

**Il ne connaît que des *ticks* d'encodeur et des *ticks/s*.** Les
conversions en mètres, mètres par seconde et radians, l'odométrie et la
cinématique sont à faire côté hôte, avec les constantes du §8.

## 2. Connexion

| | |
|---|---|
| Liaison | câble USB entre l'hôte et le port micro-USB du Pico |
| Type | USB CDC ACM (port série virtuel) : `/dev/ttyACM*` sous Linux |
| Vitesse | sans objet (USB) ; toute valeur de baudrate convient |
| Descripteur USB | VID `2e8a`, fabricant `MatthieuGrr`, produit `diffdrive_pico_firmware` |
| Alimentation | l'USB alimente l'électronique de la base ; les moteurs sont sur la batterie |

Règle udev pour un nom stable, par exemple `/etc/udev/rules.d/99-base.rules` :

```
SUBSYSTEM=="tty", ATTRS{idVendor}=="2e8a", ATTRS{product}=="diffdrive_pico_firmware", SYMLINK+="base_robot", MODE="0666"
```

Ouvrir le port **en mode brut** (pas d'écho, pas de traduction de fin de
ligne). Avec pyserial c'est le comportement par défaut ; en shell :
`stty -F /dev/ttyACM0 raw -echo`.

La base n'attend pas l'hôte : elle émet ses trames dès la mise sous tension,
que le port soit ouvert ou non. À l'ouverture, la première ligne reçue peut
être tronquée : ignorez toute ligne dont le checksum est faux.

## 3. Format des lignes

- ASCII, un message par ligne, terminé par `\n` (un `\r\n` est accepté en
  entrée ; le Pico n'émet jamais de `\r`).
- Champs séparés par un espace ; entiers en décimal.
- Longueur maximale d'une commande : 95 caractères.

## 4. Commandes (hôte → base)

| Commande | Arguments | Effet |
|---|---|---|
| `v <l> <r>` | entiers signés, ticks/s | consigne de vitesse, roue gauche et roue droite. Passe en asservissement |
| `o <l> <r>` | entiers −1000..1000 | boucle ouverte : rapport cyclique imposé (‰). Écrêté à ±1000 |
| `s` | — | arrêt immédiat (freinage) **et** acquittement des défauts (§7) |
| `r` | — | remise à zéro des compteurs d'encodeurs |
| `i` | — | identification : la base répond par une ligne `I` (§5.2) |
| `k <kp> <ki> <kd>` | flottants | gains du PID, réservé à la mise au point |
| `t <n>` | entier 0..4 | mode de test, réservé à la maintenance : **ne pas utiliser** |

Exemples : `v 1500 1500\n` (avancer), `v -800 800\n` (pivoter sur place),
`s\n`.

Règles :

- **Stricte** : une ligne mal formée (argument manquant, en trop ou non
  numérique, commande inconnue) est ignorée en entier, sans réponse.
- **Pas d'accusé de réception.** Vérifiez l'effet dans les trames `S`.
- **Aucune limitation** de vitesse ni d'accélération côté base : la consigne
  est appliquée telle quelle. Les rampes et limites sont à faire côté hôte.
- Les moteurs ont un sens « avant » commun : une consigne positive sur les
  deux roues fait avancer le robot, et les compteurs croissent en marche
  avant.

### Chien de garde (à lire absolument)

**Toute commande valide réarme un chien de garde de 300 ms.** Sans commande
valide pendant 300 ms, la base freine les deux moteurs et lève le drapeau
`LINK_LOST`. Il faut donc renvoyer la consigne en continu :

- envoyez `v` (ou `o`) à **20 Hz au moins** (toutes les 50 ms), même si la
  consigne ne change pas ;
- après une coupure, la base reste arrêtée : un nouveau `v` la relance, et
  `LINK_LOST` retombe.

C'est voulu : si votre programme plante, se fige ou perd le câble, le robot
s'arrête seul.

## 5. Messages (base → hôte)

### 5.1 Trame d'état `S`, 50 Hz

```
S <t_ms> <enc_l> <enc_r> <vel_l> <vel_r> <gx> <gy> <gz> <ax> <ay> <az> <vbat_mv> <ibat_ma> <flags> <chk>
```

| # | Champ | Type | Unité | Signification |
|---|---|---|---|---|
| 1 | `t_ms` | uint32 | ms | horodatage depuis le démarrage de la base (reboucle après ~49 jours) |
| 2 | `enc_l` | int32 | ticks | compteur **cumulé** de la roue gauche |
| 3 | `enc_r` | int32 | ticks | compteur cumulé de la roue droite |
| 4 | `vel_l` | int32 | ticks/s | vitesse mesurée, roue gauche (filtrée) |
| 5 | `vel_r` | int32 | ticks/s | vitesse mesurée, roue droite |
| 6–8 | `gx gy gz` | int16 | 1/16 °/s | gyroscope brut (BNO055) |
| 9–11 | `ax ay az` | int16 | 1/100 m/s² | accéléromètre brut (BNO055) |
| 12 | `vbat_mv` | int32 | mV | tension batterie |
| 13 | `ibat_ma` | int32 | mA | courant batterie, positif en décharge |
| 14 | `flags` | uint32 | — | champ de bits, §6 |
| 15 | `chk` | 2 hex | — | checksum, §5.3 |

Remarques :

- **Les compteurs sont cumulés, jamais des deltas** : calculez vous-même la
  différence entre deux trames. Une trame perdue est ainsi sans conséquence,
  la suivante rattrape tout. Ils ne reviennent à zéro que sur `r` ou au
  redémarrage de la base (détectable par `t_ms` qui recule).
- Pour l'odométrie, préférez les compteurs aux vitesses : `vel_*` est filtré
  et quantifié (pas de 50 ticks/s), utile pour la supervision seulement.
- Gyroscope et accéléromètre : diviser par 16 pour des °/s, par 100 pour des
  m/s². L'orientation des axes dépend du montage du BNO055 sur la base (§8).
  Si l'IMU est absente, ces six champs valent 0 et le drapeau `IMU_ABSENT`
  est levé.
- `vbat_mv` / `ibat_ma` valent 0 si le capteur est absent (`INA_ABSENT`). Le
  courant peut être négatif brièvement au freinage (récupération).

Exemple :

```
S 123456 15230 15012 3012 2987 -3 12 1 -21 4 981 19234 1520 0 5F
```

### 5.2 Identification `I`, en réponse à `i`

```
I <nom> <version_firmware> <version_protocole> <chk>
```

Exemple : `I diffdrive_pico_firmware 2.0.0 2 0C`. Ce document décrit la
**version de protocole 2**. Vérifiez-la au démarrage de votre pilote et
refusez de piloter une version inconnue.

Les lignes `I` arrivent intercalées entre les trames `S` : triez les lignes
par leur première lettre.

### 5.3 Checksum

`chk` est le **OU exclusif de tous les octets de la ligne**, depuis la lettre
initiale (`S` ou `I`) incluse jusqu'au dernier caractère qui précède l'espace
séparant le dernier champ de `chk`. Cet espace, `chk` et le `\n` sont exclus.
Il est écrit en deux chiffres hexadécimaux majuscules.

```python
import functools, operator

def checksum_ok(ligne: str) -> bool:
    charge, chk = ligne.rstrip("\n").rsplit(" ", 1)
    return functools.reduce(operator.xor, charge.encode("ascii"), 0) == int(chk, 16)
```

```c
/* ligne sans le \n final */
int checksum_ok(const char *ligne) {
    const char *sp = strrchr(ligne, ' ');
    if (!sp) return 0;
    unsigned char x = 0;
    for (const char *p = ligne; p < sp; p++) x ^= (unsigned char)*p;
    return x == (unsigned char)strtoul(sp + 1, NULL, 16);
}
```

Jetez toute ligne au checksum faux, au nombre de champs inattendu, ou qui ne
commence pas par une lettre connue. Pour la compatibilité future, un pilote
peut accepter des champs supplémentaires avant `chk` (une version ultérieure
n'en ajoutera qu'en fin de trame) — mais il doit vérifier la version de
protocole avec `i`.

## 6. Drapeaux (`flags`)

| Bit | Valeur | Nom | Signification | Verrouillé |
|---|---|---|---|---|
| 0 | 1 | `LINK_LOST` | pas de commande valide depuis 300 ms, moteurs freinés | non : retombe à la prochaine commande valide |
| 1 | 2 | `OVERCURRENT` | surintensité, moteurs freinés | **oui** |
| 2 | 4 | `UNDERVOLTAGE` | batterie trop basse, moteurs freinés | **oui** |
| 3 | 8 | `NO_BATTERY` | tension < 5 V : batterie débranchée ou éteinte, les moteurs ne peuvent pas tourner | non |
| 4 | 16 | `IMU_ABSENT` | BNO055 muet, champs `g*`/`a*` à 0 | non |
| 5 | 32 | `INA_ABSENT` | capteur batterie muet, `vbat`/`ibat` à 0, **plus de protection batterie** | non |
| 6 | 64 | `TEST_MODE` | mode de maintenance actif : envoyer `t 0` | non |
| 7 | 128 | `MOTORS_INHIBITED` | un défaut interdit les moteurs : consignes ignorées | — |

`flags == 0` : tout va bien. Les bits non listés sont réservés et valent 0.

## 7. Sécurités et défauts

| Défaut | Déclenchement |
|---|---|
| Perte du lien | aucune commande valide depuis 300 ms |
| Surintensité instantanée | courant ≥ 7,5 A : coupure matérielle en quelques millisecondes |
| Surintensité soutenue | courant > 5 A pendant 1 s (roue bloquée, robot coincé) |
| Sous-tension | tension < 15,0 V pendant 2 s |

Lorsqu'un défaut **verrouillé** survient (`OVERCURRENT`, `UNDERVOLTAGE`) :

1. les moteurs sont freinés immédiatement ;
2. le drapeau du défaut et `MOTORS_INHIBITED` sont levés ;
3. les commandes `v` et `o` sont acceptées (elles entretiennent le chien de
   garde) mais **n'ont aucun effet** ;
4. le défaut ne s'efface que par une commande **`s`**, et seulement si sa
   cause a disparu : courant revenu sous 5 A, ou tension remontée au-dessus
   de **16,0 V** (recharger ou changer la batterie). Sinon `s` est sans effet
   et le drapeau reste levé ;
5. après l'effacement, le robot reste à l'arrêt jusqu'au prochain `v`/`o`.

Conduite recommandée côté pilote : afficher le défaut à l'opérateur, ne
**pas** envoyer `s` automatiquement en boucle (une roue bloquée se remettrait
à forcer), et n'acquitter qu'après action humaine.

À l'arrêt (consigne nulle, `s`, chien de garde, défaut), les moteurs sont
**freinés**, pas en roue libre : le robot ne se pousse pas facilement à la
main tant que la batterie est branchée.

## 8. Constantes physiques de la base

À utiliser pour convertir ticks ↔ mètres. **Valeurs à compléter après
mesure** sur le robot livré.

| Constante | Valeur | Remarque |
|---|---|---|
| Counts par tour de roue (`N`) | *à mesurer* | 16 PPR × 4 × rapport du réducteur |
| Diamètre de roue (`D`) | *à mesurer* | |
| Entraxe (`L`) | *à mesurer* | distance entre les points de contact des deux roues |
| Vitesse maximale | *à mesurer* | en ticks/s, à duty 1000, sur batterie chargée |
| Orientation de l'IMU | *à préciser* | axes x/y/z du BNO055 par rapport au robot |

Conversions :

```
distance_roue [m]   = Δticks × π·D / N
vitesse_roue [m/s]  = ticks/s × π·D / N
consigne [ticks/s]  = v [m/s] × N / (π·D)

v      = (v_r + v_l) / 2          # vitesse linéaire du robot
ω      = (v_r − v_l) / L          # vitesse angulaire, rad/s, positive à gauche
v_l    = v − ω·L/2 ;  v_r = v + ω·L/2
```

## 9. Pilote minimal recommandé

1. Ouvrir le port, vider le tampon d'entrée.
2. Envoyer `i`, attendre une ligne `I` valide, vérifier la version de
   protocole (2).
3. Envoyer `s` puis `r` : arrêt, défauts acquittés, compteurs à zéro.
4. Boucle :
   - lire toutes les lignes disponibles, vérifier le checksum, garder la
     dernière trame `S` ;
   - surveiller `flags` (§6) et réagir aux défauts ;
   - envoyer la consigne `v <l> <r>` **toutes les 50 ms au plus**.
5. À la sortie (y compris sur erreur ou Ctrl-C) : envoyer `s`.

Voir [`exemple_pilote.py`](exemple_pilote.py), qui fait exactement cela.

## 10. Pièges fréquents

- **Le robot s'arrête tout seul après 300 ms** : la consigne n'est pas
  renvoyée en continu (§4). Attention aux boucles qui n'envoient qu'en cas de
  changement de consigne.
- **Rien ne bouge, `flags` contient 128** : un défaut est verrouillé, voir
  §7. Avec 8 : la batterie est débranchée ou éteinte.
- **Checksums faux en série** : port ouvert en mode « cooked » (traduction de
  fin de ligne, écho). Ouvrir en mode brut.
- **Les compteurs semblent sauter** : ils sont cumulés (§5.1) ; ne pas les
  additionner comme des deltas.
- **Saturation** : au-delà de la vitesse maximale (§8), l'asservissement ne
  peut plus suivre ; limiter la consigne côté hôte.
- Ne pas envoyer de commandes `k` ou `t` : elles modifient le réglage ou
  font passer la base en mode de maintenance (moteurs pilotés par le test,
  chien de garde suspendu). En cas d'envoi accidentel : `t 0`.
