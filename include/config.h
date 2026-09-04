/*
 * diffdrive_pico_firmware — configuration centrale du firmware
 *
 * Tout ce qui se regle sans toucher a la logique est ici : brochage,
 * frequences, inversions de sens, gains par defaut, temporisations.
 *
 * FRONTIERE D'ABSTRACTION
 * -----------------------
 * Ce firmware ne connait NI rayon de roue, NI entraxe, NI counts par tour.
 * Il ne manipule que des ticks et des ticks/s. Aucune constante physique ne
 * doit apparaitre dans ce fichier ni ailleurs dans le firmware : la
 * cinematique et la calibration vivent cote Raspberry Pi 5.
 */
#ifndef DIFFDRIVE_PICO_CONFIG_H
#define DIFFDRIVE_PICO_CONFIG_H

/* ------------------------------------------------------------------------ */
/* Identite                                                                  */
/* ------------------------------------------------------------------------ */
#define FW_NAME                 "diffdrive_pico_firmware"
#define FW_VERSION              "1.0.0"

/* ------------------------------------------------------------------------ */
/* Brochage — RP2350, le GPIO n'est PAS tolerant 5V.                         */
/* Les encodeurs doivent etre alimentes en 3.3V.                             */
/* ------------------------------------------------------------------------ */

/* Moteurs : DRV8871, deux entrees logiques par pont.                        */
/* IN1/IN2 d'un meme moteur doivent partager la meme slice PWM :             */
/* slice = (gpio >> 1) & 7, canal A = gpio pair, canal B = gpio impair.      */
#define PIN_MOT_L_IN1           2u      /* slice 1, canal A */
#define PIN_MOT_L_IN2           3u      /* slice 1, canal B */
#define PIN_MOT_R_IN1           4u      /* slice 2, canal A */
#define PIN_MOT_R_IN2           5u      /* slice 2, canal B */

/* Encodeurs en quadrature. Le canal B doit imperativement etre A + 1 :      */
/* le programme PIO lit deux broches consecutives a partir de A.             */
#define PIN_ENC_L_A             6u
#define PIN_ENC_L_B             7u      /* informatif : doit valoir A + 1 */
#define PIN_ENC_R_A             8u
#define PIN_ENC_R_B             9u      /* informatif : doit valoir A + 1 */

/* I2C0 — BNO055 */
#define IMU_I2C                 i2c0
#define PIN_I2C_SDA             16u
#define PIN_I2C_SCL             17u
#define IMU_I2C_BAUD            400000u

/* UART0 — logs de debug uniquement. JAMAIS le protocole robot.              */
#define LOG_UART                uart0
#define PIN_LOG_TX              0u
#define PIN_LOG_RX              1u
/* 115200 est le choix sur avec n'importe quel adaptateur USB-UART.
 * Si le votre suit (CP2102 / FT232 / CH340 le font tous), monter a 460800
 * divise par 4 le temps passe dans uart_write_blocking, ce qui reduit
 * nettement la gigue de la boucle 50 Hz en mode de test t 4. */
#define LOG_UART_BAUD           115200u

/* ------------------------------------------------------------------------ */
/* Moteurs                                                                   */
/* ------------------------------------------------------------------------ */

/* Frequence PWM. Au-dessus de l'audible, sous la limite du DRV8871 (100 kHz). */
#define PWM_FREQ_HZ             25000u

/* Plage de rapport cyclique du protocole et de la sortie PID.               */
#define DUTY_MAX                1000

/*
 * MODE DE DECROISSANCE DU DRV8871 — la constante a changer.
 *
 * Table de verite du DRV8871 :
 *   IN1=0 IN2=0  roue libre (coast)      — decroissance rapide
 *   IN1=1 IN2=0  marche avant
 *   IN1=0 IN2=1  marche arriere
 *   IN1=1 IN2=1  freinage (brake)        — decroissance lente
 *
 * DECAY_FAST : on module IN1 (ou IN2) et on maintient l'autre a 0. Pendant
 *   le temps mort le moteur est en roue libre. Zone morte importante a bas
 *   rapport cyclique, reponse fortement non lineaire.
 *
 * DECAY_SLOW : on maintient une entree a 1 et on module l'autre en
 *   complement. Pendant le temps mort le moteur est court-circuite : le
 *   courant est maintenu, la reponse couple/duty est bien plus lineaire a
 *   bas regime. C'est en general ce qu'il faut pour un asservissement de
 *   vitesse a faible rapport cyclique.
 *
 * Passer de l'un a l'autre = changer cette seule ligne.
 */
#define DECAY_FAST              0
#define DECAY_SLOW              1
#ifndef MOTOR_DECAY_MODE        /* surchargeable par -D depuis CMake */
#define MOTOR_DECAY_MODE        DECAY_SLOW
#endif

/*
 * Comportement a duty nul. Independant du mode de decroissance : a l'arret
 * on veut un etat previsible.
 *   0 = roue libre (IN1=0, IN2=0)  — le robot peut etre pousse a la main
 *   1 = freinage   (IN1=1, IN2=1)
 * La coupure de securite du chien de garde utilise TOUJOURS la roue libre,
 * quelle que soit cette valeur.
 */
#ifndef MOTOR_IDLE_BRAKE
#define MOTOR_IDLE_BRAKE        0
#endif

/*
 * Inversion du sens de rotation. DETERMINE EXPERIMENTALEMENT :
 * avec (0, 1), un rapport cyclique positif faisait RECULER le robot, et les
 * encodeurs comptaient negativement — donc les encodeurs avaient deja le bon
 * signe (arriere = negatif) et c'etait la commande moteur qui etait a
 * l'envers. D'ou (1, 0).
 */
#define MOT_L_INVERT            1
#define MOT_R_INVERT            0

/* ------------------------------------------------------------------------ */
/* Encodeurs                                                                 */
/* ------------------------------------------------------------------------ */

/* PIO dedie aux encodeurs. Le programme occupe 28 des 32 instructions :     */
/* ce PIO ne peut rien heberger d'autre. Les 2 SM partagent la meme copie.   */
#define ENC_PIO                 pio0
#define ENC_L_SM                0u
#define ENC_R_SM                1u

/* Diviseur d'horloge PIO. Le programme prend 9 a 13 cycles par echantillon. */
/* 12.5 sur un systeme a 150 MHz => ~1 Mech/s, soit ~200 echantillons par    */
/* front a pleine vitesse. Augmenter pour filtrer davantage le bruit.        */
#define ENC_PIO_CLKDIV          12.5f

/* Resistances de tirage internes sur A et B. A laisser a 1 si les sorties   */
/* de l'encodeur sont a collecteur ouvert, sans effet si elles sont push-pull.*/
#define ENC_PULLUP              1

/*
 * Inversion du signe des compteurs. VERIFIE EXPERIMENTALEMENT : ces valeurs
 * sont correctes, les deux encodeurs comptent bien positivement en marche
 * avant une fois MOT_x_INVERT corrige. Ne pas y toucher.
 */
#define ENC_L_INVERT            0
#define ENC_R_INVERT            1   /* encodeurs montes en miroir */

/* Attente maximale d'une publication fraiche du PIO. En fonctionnement      */
/* normal la SM publie toutes les ~1 us : ce delai n'est qu'un garde-fou.    */
#define ENC_READ_TIMEOUT_US     200u

/* ------------------------------------------------------------------------ */
/* Boucle de controle                                                        */
/* ------------------------------------------------------------------------ */

#define CONTROL_HZ              50u
#define CONTROL_PERIOD_US       (1000000u / CONTROL_HZ)

/*
 * Filtre passe-bas du premier ordre sur la vitesse mesuree.
 *   vel = alpha * vel_brute + (1 - alpha) * vel_precedente
 * 1.0 = aucun filtrage. A 50 Hz la mesure brute est quantifiee par le nombre
 * entier de ticks vus sur 20 ms, donc bruitee a bas regime. Ce n'est PAS une
 * limitation de vitesse : la consigne n'est jamais retouchee.
 */
#define VEL_LPF_ALPHA           0.5f

/* ------------------------------------------------------------------------ */
/* PID de vitesse — gains par defaut                                         */
/* ------------------------------------------------------------------------ */
/*
 * Erreur en ticks/s, sortie en unites de rapport cyclique (+/- DUTY_MAX).
 * Ces valeurs ne sont que des points de depart : le reglage se fait a chaud
 * avec la commande 'k' en observant le mode de test t 4.
 */
#define PID_KP_DEFAULT          0.12f
#define PID_KI_DEFAULT          0.60f
#define PID_KD_DEFAULT          0.0f

/*
 * Terme d'anticipation : duty = KFF * consigne + PID.
 * Hors protocole (la commande 'k' ne regle que kp/ki/kd), donc constante de
 * compilation. 0.0 = desactive, le PID travaille seul.
 *
 * MESURE au banc, apres rodage des moteurs :
 *     roue gauche 3807 ticks/s a duty 1000  ->  1000/3807 = 0.263
 *     roue droite 3643 ticks/s a duty 1000  ->  1000/3643 = 0.274
 * Les deux roues ne different plus que de 5 % (22 % avant rodage), une
 * constante unique est donc legitime. Valeur retenue legerement sous la
 * moyenne : mieux vaut que l'integrateur ajoute que retranche.
 */
#define PID_KFF                 0.26f

/* ------------------------------------------------------------------------ */
/* Securite                                                                  */
/* ------------------------------------------------------------------------ */

/*
 * Chien de garde logiciel : seule securite non negociable du firmware.
 * Sans commande valide recue depuis ce delai, les deux PWM sont mis a zero
 * immediatement. Verifie a chaque tour de la boucle principale, pas
 * seulement a 50 Hz.
 * N'agit qu'en mode normal (t 0) : dans les modes de test moteur, c'est le
 * test lui-meme qui pilote les moteurs.
 */
#define CMD_TIMEOUT_US          300000u

/*
 * Chien de garde materiel du RP2350 : redemarre la puce si la boucle
 * principale se bloque completement. Doit rester tres large devant la
 * periode de controle.
 */
#define HW_WATCHDOG_ENABLE      1
#define HW_WATCHDOG_MS          1000u

/* ------------------------------------------------------------------------ */
/* BNO055                                                                    */
/* ------------------------------------------------------------------------ */

/* 0x28 quand ADR est au niveau bas (cas par defaut des modules), 0x29 sinon.*/
#define BNO055_ADDR             0x28
#define BNO055_ADDR_ALT         0x29

/* Reset logiciel au demarrage : etat deterministe, mais ~700 ms de boot.    */
#define BNO055_RESET_AT_BOOT    1

/* Delai maximal d'une transaction I2C. Garantit que la boucle de controle   */
/* ne se bloque jamais sur un bus mort.                                      */
#define I2C_TIMEOUT_US          3000u

/* Tentative de redetection quand l'IMU est absente, en periodes de controle.*/
/* 250 * 20 ms = 5 s.                                                        */
#define IMU_RETRY_TICKS         250u

/*
 * Unites du BNO055 (registre UNIT_SEL). On garde les valeurs par defaut du
 * composant et on transmet les entiers 16 bits bruts : la conversion SI se
 * fait cote Pi. Facteurs a utiliser la-bas :
 *     accelerometre : 100 LSB par m/s^2
 *     gyroscope     :  16 LSB par degre/s
 */
#define BNO055_UNIT_SEL         0x00

/* ------------------------------------------------------------------------ */
/* Modes de test (commande 't', sortie sur UART0)                            */
/* ------------------------------------------------------------------------ */
#define TEST_NORMAL             0
#define TEST_ENCODERS           1
#define TEST_I2C                2
#define TEST_MOTOR_RAMP         3
#define TEST_PID_STEP           4
#define TEST_MODE_MAX           4

/*
 * Counts par tour de roue — PUREMENT INFORMATIF, aucun calcul ne l'utilise.
 * Ne sert qu'au message du mode t 1, pour vous donner la cible a comparer.
 * La frontiere d'abstraction tient : la conversion ticks -> distance vit
 * cote Pi, ce firmware ne fait qu'afficher ce nombre dans un log.
 *
 * Mesure sur 10 tours de roue a la main : 14960 ticks, soit
 *     1496 = 11 PPR x 4 (quadrature) x 34:1 (reduction)
 * Le reducteur est bien en 34:1 et non 35:1 comme annonce initialement.
 */
#define TEST_COUNTS_PER_REV     1496

/* Cadence d'affichage des modes lents (t 1, t 2, t 3), en Hz.               */
#define TEST_SLOW_LOG_HZ        5u

/* t 3 : duree d'une rampe complete 0 -> +max (ou +max -> -max en double).   */
#define TEST_RAMP_SECONDS       5.0f
/* t 3 : palier d'observation en haut et en bas de rampe, en secondes.       */
#define TEST_RAMP_HOLD_SECONDS  1.5f
/* t 3 : rapport cyclique maximal atteint par la rampe.                      */
#define TEST_RAMP_DUTY_MAX      DUTY_MAX

/* t 4 : amplitude de l'echelon de consigne, en ticks/s, et sa demi-periode. */
#define TEST_STEP_TICKS         1500
#define TEST_STEP_SECONDS       2.0f

/* ------------------------------------------------------------------------ */
/* Logs UART                                                                 */
/* ------------------------------------------------------------------------ */
#define LOG_ENABLE              1
#define LOG_BUF_SIZE            160

/* ------------------------------------------------------------------------ */
/* Protocole                                                                 */
/* ------------------------------------------------------------------------ */
#define RX_LINE_MAX             96      /* longueur max d'une commande */
#define TX_LINE_MAX             160     /* longueur max d'une trame S   */
/* Caracteres consommes au maximum par passage dans la boucle principale :   */
/* borne le temps passe a parser si le Pi deverse un flot continu.           */
#define RX_CHARS_PER_POLL       256

/* ------------------------------------------------------------------------ */
/* Verifications de coherence a la compilation                               */
/* ------------------------------------------------------------------------ */
_Static_assert(PIN_ENC_L_B == PIN_ENC_L_A + 1u,
               "le canal B de l'encodeur gauche doit etre A + 1 (contrainte PIO)");
_Static_assert(PIN_ENC_R_B == PIN_ENC_R_A + 1u,
               "le canal B de l'encodeur droit doit etre A + 1 (contrainte PIO)");
_Static_assert((PIN_MOT_L_IN1 >> 1) == (PIN_MOT_L_IN2 >> 1),
               "IN1/IN2 du moteur gauche doivent partager la meme slice PWM");
_Static_assert((PIN_MOT_R_IN1 >> 1) == (PIN_MOT_R_IN2 >> 1),
               "IN1/IN2 du moteur droit doivent partager la meme slice PWM");
_Static_assert((PIN_MOT_L_IN1 & 1u) == 0u && (PIN_MOT_L_IN2 & 1u) == 1u,
               "IN1 doit etre sur le canal A (gpio pair) et IN2 sur le canal B");
_Static_assert((PIN_MOT_R_IN1 & 1u) == 0u && (PIN_MOT_R_IN2 & 1u) == 1u,
               "IN1 doit etre sur le canal A (gpio pair) et IN2 sur le canal B");
_Static_assert(ENC_L_SM != ENC_R_SM, "une machine a etat distincte par encodeur");

#endif /* DIFFDRIVE_PICO_CONFIG_H */
