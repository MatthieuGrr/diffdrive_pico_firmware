/*
 * diffdrive_pico_firmware — modes de test integres, sortie sur UART0.
 *
 * Objectif : isoler chaque etage sans reflasher, avec un simple adaptateur
 * USB-UART branche sur GP0/GP1.
 *
 *   t 0  mode normal (defaut)
 *   t 1  encodeurs seuls, moteurs coupes, compteurs a 5 Hz
 *   t 2  I2C : scan du bus, CHIP_ID, calibration, puis gyro/accel a 5 Hz
 *   t 3  moteurs en boucle ouverte, rampe lente, duty et vitesse cote a cote
 *   t 4  PID ferme, echelon de consigne, trace a 50 Hz
 *
 * IMPORTANT — chien de garde : dans les modes 3 et 4, c'est le test qui
 * pilote les moteurs, pas le Pi. Le chien de garde logiciel de 300 ms est
 * donc suspendu tant qu'un mode de test est actif ; il reprend des le retour
 * en t 0. Un mode de test ne s'active que sur commande explicite.
 *
 * Les trames S continuent d'etre emises sur USB dans tous les modes : on
 * peut observer le test depuis le Pi en meme temps que sur l'UART.
 */
#ifndef DIFFDRIVE_PICO_TESTMODES_H
#define DIFFDRIVE_PICO_TESTMODES_H

#include <stdbool.h>

/* Entree dans un mode (coupe les moteurs et reinitialise l'etat du test). */
void testmodes_enter(int mode);

/* Appele une fois par periode de controle quand un mode de test est actif.
 * Renvoie true si le test a pris la main sur les moteurs : la boucle
 * normale ne doit alors pas les ecraser. */
bool testmodes_step(float dt_s);

#endif /* DIFFDRIVE_PICO_TESTMODES_H */
