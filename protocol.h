/*
 * diffdrive_pico_firmware — protocole serie sur USB CDC.
 *
 * stdio EST le protocole. Aucun message de debug ne transite ici : les logs
 * sortent sur UART0 via LOG().
 *
 * Pi -> Pico, lignes ASCII terminees par \n :
 *   v <l> <r>          consigne de vitesse, ticks/s, entiers signes
 *   o <l> <r>          boucle ouverte, rapport cyclique -1000..1000
 *   s                  arret immediat
 *   r                  remise a zero des compteurs
 *   k <kp> <ki> <kd>   gains PID a chaud (flottants, appliques aux deux roues)
 *   t <n>              entree en mode de test n
 *
 * Pico -> Pi, spontanement a 50 Hz :
 *   S <t_ms> <enc_l> <enc_r> <vel_l> <vel_r> <gx> <gy> <gz> <ax> <ay> <az> <chk>
 *
 * CHECKSUM — definition exacte, a reproduire cote Pi :
 *   chk est le OU exclusif de TOUS les octets de la ligne depuis le 'S'
 *   inclus jusqu'au dernier caractere precedant l'espace qui separe <az> de
 *   <chk>. L'espace separateur, le champ chk lui-meme et le \n final sont
 *   exclus. Il est emis en deux chiffres hexadecimaux majuscules.
 *
 *   Python :
 *       payload, chk = line.rstrip('\n').rsplit(' ', 1)
 *       ok = functools.reduce(operator.xor, payload.encode(), 0) == int(chk, 16)
 */
#ifndef DIFFDRIVE_PICO_PROTOCOL_H
#define DIFFDRIVE_PICO_PROTOCOL_H

#include <stdint.h>

void protocol_init(void);

/* Consomme ce qui est disponible sur l'USB CDC et execute les commandes
 * completes. Ne bloque jamais : au plus RX_CHARS_PER_POLL caracteres par
 * appel. A appeler aussi souvent que possible. */
void protocol_poll(void);

/* Construit et emet une trame S. Ecriture bornee dans le temps : si le Pi
 * ne lit pas, les octets sont abandonnes plutot que d'attendre. */
void protocol_send_state(uint32_t t_ms);

#endif /* DIFFDRIVE_PICO_PROTOCOL_H */
