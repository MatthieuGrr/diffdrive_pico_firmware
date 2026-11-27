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
 * cinematique et la calibration vivent cote hote (Raspberry Pi 5 ou Jetson).
 */
#ifndef DIFFDRIVE_PICO_CONFIG_H
#define DIFFDRIVE_PICO_CONFIG_H

/* ------------------------------------------------------------------------ */
/* Identite                                                                  */
/* ------------------------------------------------------------------------ */
#define FW_NAME                 "diffdrive_pico_firmware"
#define FW_VERSION              "2.1.0"
/* Version du protocole serie. Incrementee a chaque changement incompatible
 * des commandes ou de la trame S ; renvoyee par la commande 'i'.           */
#define PROTO_VERSION           2

/* ------------------------------------------------------------------------ */
/* Brochage — RP2350, le GPIO n'est PAS tolerant 5V.                         */
/* Les encodeurs doivent etre alimentes en 3.3V.                             */
/* ------------------------------------------------------------------------ */

/* Moteurs : Cytron MDD10A, commande signe-amplitude (PWM + DIR) par canal.  */
/* Canal 1 du MDD10A = moteur gauche, canal 2 = moteur droit.               */
/* PWM sur une sortie de slice, DIR en simple GPIO.                          */
#define PIN_MOT_L_PWM           2u      /* MDD10A PWM1, slice 1 canal A */
#define PIN_MOT_L_DIR           3u      /* MDD10A DIR1                  */
#define PIN_MOT_R_PWM           4u      /* MDD10A PWM2, slice 2 canal A */
#define PIN_MOT_R_DIR           5u      /* MDD10A DIR2                  */

/* Encodeurs en quadrature. Le canal B doit imperativement etre A + 1 :      */
/* le programme PIO lit deux broches consecutives a partir de A.             */
#define PIN_ENC_L_A             6u
#define PIN_ENC_L_B             7u      /* informatif : doit valoir A + 1 */
#define PIN_ENC_R_A             8u
#define PIN_ENC_R_B             9u      /* informatif : doit valoir A + 1 */

/* I2C0 — BNO055, seul sur son bus */
#define IMU_I2C                 i2c0
#define PIN_I2C_SDA             16u
#define PIN_I2C_SCL             17u
/* 100 kHz et pas 400 : les pull-ups en place sont trop faibles pour tenir
 * les fronts montants a 400 kHz, l'IMU decrochait. */
#define IMU_I2C_BAUD            100000u

/* I2C1 — INA226 (tension / courant batterie), seul sur son bus : un BNO055
 * qui bloque le bus I2C0 ne peut pas rendre muet le capteur de securite.   */
#define INA_I2C                 i2c1
#define PIN_INA_SDA             18u
#define PIN_INA_SCL             19u
#define INA_I2C_BAUD            400000u
/* Sortie ALERT de l'INA226 : drain ouvert, active a l'etat bas, tirage
 * interne du Pico. Freine les moteurs sur interruption (surintensite).    */
#define PIN_INA_ALERT           20u

/* Surveillance de l'arret d'urgence (AU). Le bouton coupe lui-meme la
 * puissance moteur ; le Pico ne fait que LIRE un contact auxiliaire NF
 * (normalement ferme) du bouton, cable entre GP21 et GND (broche 28,
 * voisine). Tirage interne du Pico vers 3V3, sans resistance externe : les
 * fils sont courts et l'antirebond (ESTOP_TRIP_MS) filtre les parasites.
 *   bouton relache : contact ferme  -> GP21 a 0 -> normal
 *   bouton enfonce : contact ouvert -> GP21 a 1 -> FLAG_ESTOP
 *   fil coupe ou debranche          -> GP21 a 1 -> FLAG_ESTOP (securite
 *   positive : une panne de cablage arrete le robot, elle ne le cache pas) */
#define PIN_ESTOP               21u

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

/* Frequence PWM. Au-dessus de l'audible, et limite haute du MDD10A (20 kHz). */
#define PWM_FREQ_HZ             20000u

/* Plage de rapport cyclique du protocole et de la sortie PID.               */
#define DUTY_MAX                1000

/*
 * Table de verite du MDD10A (signe-amplitude), par canal :
 *   PWM=0  DIR=x   sorties A=0 B=0   moteur court-circuite a la masse (FREIN)
 *   PWM=1  DIR=0   A=1 B=0           marche dans un sens
 *   PWM=1  DIR=1   A=0 B=1           marche dans l'autre
 *
 * Consequences :
 *   - le temps mort du PWM est TOUJOURS un freinage (decroissance lente) :
 *     reponse couple/duty bien lineaire a bas regime ;
 *   - le MDD10A n'a PAS d'etat roue libre : duty nul, commande 's' et
 *     coupures de securite FREINENT le moteur. Robot a l'arret = roues
 *     retenues ; il ne se pousse pas a la main moteurs sous tension.
 */

/*
 * Inversion du sens de rotation. A VERIFIER au banc avec t 3 apres le
 * changement de moteurs : un duty positif doit faire AVANCER le robot.
 * Inverser les fils blanc/jaune d'un moteur revient au meme.
 */
#define MOT_L_INVERT            0
#define MOT_R_INVERT            1   /* moteurs montes en miroir */

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

/* Resistances de tirage internes sur A et B. Les encodeurs SCX-555 sont a  */
/* drain ouvert : a laisser a 1. Leurs tirages embarques sont relies a leur */
/* VCC, d'ou l'alimentation des encodeurs en 3,3 V et jamais en 5 V.        */
#define ENC_PULLUP              1

/*
 * Inversion du signe des compteurs. A VERIFIER avec t 1 puis t 3 apres le
 * changement de moteurs : les deux compteurs doivent croitre en marche
 * avant. Permuter les fils noir (A) et rouge (B) revient au meme.
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
 *
 * PROVISOIRES depuis le passage aux motoreducteurs 36-555 et au MDD10A.
 * Les gains v1 (kp 0.1749, ki 0.8074, kff 0.266) avaient ete mesures sur
 * les GA25-370 a 1496 counts/tour. Les 555 donnent environ deux fois plus
 * de ticks/s a vitesse egale : gain de la machine environ double, donc
 * gains v1 divises par deux comme point de depart prudent.
 * A REMESURER au sol avec tools/tune_pid.py.
 */
#define PID_KP_DEFAULT          0.087f
#define PID_KI_DEFAULT          0.40f
#define PID_KD_DEFAULT          0.0f

/*
 * Terme d'anticipation : duty = KFF * consigne + PID.
 * Hors protocole (la commande 'k' ne regle que kp/ki/kd), donc constante de
 * compilation. 0.0 = desactive, le PID travaille seul.
 * DESACTIVE tant que la pleine echelle des nouveaux moteurs n'est pas
 * mesuree : relever les ticks/s a duty 1000 avec t 3, puis
 * KFF = 1000 / ces ticks/s, legerement arrondi par defaut.
 */
#define PID_KFF                 0.0f

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
/* Surveillance de la batterie (INA226)                                      */
/* ------------------------------------------------------------------------ */
/*
 * Shunt du module INA226 : resistance « R010 » = 10 mOhm. Plage de mesure
 * +/- 81,92 mV / 10 mOhm = +/- 8,19 A, resolution 0,25 mA.
 */
#define INA226_ADDR             0x40    /* A0 = A1 = GND */
#define INA226_SHUNT_UOHM       10000u  /* 10 mOhm */

/*
 * SURINTENSITE — deux niveaux, tous deux VERROUILLES (voir commande 's').
 *
 * 1. Seuil instantane, materiel : l'INA226 compare lui-meme chaque mesure
 *    au seuil et tire ALERT a la masse ; le Pico freine les moteurs dans
 *    l'interruption, sans attendre la boucle. Doit rester sous la pleine
 *    echelle du shunt (8190 mA).
 * 2. Seuil soutenu, logiciel : courant au-dessus de OC_SUSTAINED_MA pendant
 *    OC_SUSTAINED_MS. Attrape un blocage de roue sans couper sur les pics
 *    de demarrage.
 *
 * Valeurs de depart, A AJUSTER apres mesure du courant reel en roulage.
 */
#define OC_TRIP_MA              7500
#define OC_SUSTAINED_MA         5000
#define OC_SUSTAINED_MS         1000u

/*
 * SOUS-TENSION — protege la batterie d'outillage 20 V (5S Li-ion, 3,0 V
 * par element en fin de decharge). Verrouillee apres UV_DEBOUNCE_MS sous le
 * seuil : les creux de tension a l'acceleration ne declenchent rien.
 * Ne se deverrouille (par 's') qu'au-dessus de UV_CLEAR_MV.
 */
#define UV_TRIP_MV              15000
#define UV_CLEAR_MV             16000
#define UV_DEBOUNCE_MS          2000u

/* Sous cette tension, on considere la batterie debranchee (robot sur USB
 * seul, au banc) : drapeau NO_BATTERY, non verrouille, pas de sous-tension. */
#define VBAT_PRESENT_MV         5000

/*
 * INA226 absent : 0 = le robot roule quand meme (drapeau INA_ABSENT dans la
 * trame, plus aucune protection de batterie) ; 1 = moteurs interdits.
 */
#define INA_REQUIRED            0

/*
 * ARRET D'URGENCE — voir PIN_ESTOP. Defaut VERROUILLE : il faut relacher le
 * bouton PUIS envoyer 's' pour pouvoir rouler a nouveau.
 * ESTOP_ENABLE 0 ignore l'entree (drapeau jamais pose), pour un banc sans
 * bouton. A 1 sans rien de cable, GP21 flotte a 1 : AU permanent, voulu.
 * Antirebond : AU retenu apres ESTOP_TRIP_MS a l'etat actif, bouton
 * considere relache apres ESTOP_RELEASE_MS a l'etat repos.
 */
#define ESTOP_ENABLE            1
#define ESTOP_ACTIVE_HIGH       1       /* 1 = contact NF vers GND (cf. ci-dessus) */
#define ESTOP_TRIP_MS           40u
#define ESTOP_RELEASE_MS        200u

/* ------------------------------------------------------------------------ */
/* BNO055                                                                    */
/* ------------------------------------------------------------------------ */

/* 0x28 quand ADR est au niveau bas (cas par defaut des modules), 0x29 sinon.*/
#define BNO055_ADDR             0x28
#define BNO055_ADDR_ALT         0x29

/* Reset logiciel au demarrage : etat deterministe, mais ~700 ms de boot.    */
#define BNO055_RESET_AT_BOOT    1

/* Delai maximal d'une transaction I2C. Garantit que la boucle de controle   */
/* ne se bloque jamais sur un bus mort. A 100 kHz la rafale de 18 octets     */
/* occupe deja ~1.7 ms : 5 ms laisse de la marge sans menacer les 20 ms de   */
/* la periode de controle.                                                   */
#define I2C_TIMEOUT_US          5000u

/* Tentative de redetection quand l'IMU est absente, en periodes de controle.*/
/* 250 * 20 ms = 5 s.                                                        */
#define IMU_RETRY_TICKS         250u
/* Idem pour l'INA226.                                                       */
#define INA_RETRY_TICKS         250u

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
 * cote hote, ce firmware ne fait qu'afficher ce nombre dans un log.
 *
 * Encodeur SCX-555 : 16 PPR sur l'arbre moteur, soit 64 counts par tour
 * moteur en x4, multiplies par le rapport du reducteur planetaire.
 * A MESURER sur 10 tours de roue a la main avec t 1 (0 = inconnu).
 */
#define TEST_COUNTS_PER_REV     0

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
#define TX_LINE_MAX             192     /* longueur max d'une trame S   */
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
_Static_assert(PIN_MOT_L_PWM != PIN_MOT_R_PWM && PIN_MOT_L_DIR != PIN_MOT_R_DIR,
               "broches moteur en double");
_Static_assert(OC_TRIP_MA < 8190, "seuil de surintensite au-dela de la pleine echelle du shunt");
_Static_assert(OC_SUSTAINED_MA < OC_TRIP_MA, "le seuil soutenu doit etre sous le seuil instantane");
_Static_assert(UV_CLEAR_MV > UV_TRIP_MV, "hysteresis de sous-tension inversee");
_Static_assert(PIN_ESTOP != PIN_INA_ALERT, "AU et ALERT sur la meme broche");
_Static_assert(ENC_L_SM != ENC_R_SM, "une machine a etat distincte par encodeur");

#endif /* DIFFDRIVE_PICO_CONFIG_H */
