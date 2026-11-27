#!/usr/bin/env python3
"""Pilote minimal de la base differentielle, protocole serie v2.

Exemple de reference pour ecrire son propre pilote (voir PROTOCOLE.md).
Dependance : pyserial (pip install pyserial, ou apt install python3-serial).

    python3 exemple_pilote.py /dev/ttyACM0

Fait avancer le robot doucement pendant 3 s, puis l'arrete, en affichant
l'etat. Robot sureleve pour un premier essai.
"""
import functools
import operator
import sys
import time

import serial

PROTO_ATTENDU = 2
PERIODE_CONSIGNE = 0.05     # 20 Hz : bien sous le chien de garde de 300 ms

FLAGS = {
    0x01: "LINK_LOST", 0x02: "OVERCURRENT", 0x04: "UNDERVOLTAGE",
    0x08: "NO_BATTERY", 0x10: "IMU_ABSENT", 0x20: "INA_ABSENT",
    0x40: "TEST_MODE", 0x80: "MOTORS_INHIBITED", 0x100: "ESTOP",
}
CHAMPS_S = ("t_ms", "enc_l", "enc_r", "vel_l", "vel_r",
            "gx", "gy", "gz", "ax", "ay", "az",
            "vbat_mv", "ibat_ma", "flags")


def checksum_ok(ligne: str) -> bool:
    try:
        charge, chk = ligne.rsplit(" ", 1)
        return functools.reduce(operator.xor, charge.encode("ascii"), 0) == int(chk, 16)
    except ValueError:
        return False


def noms_flags(f: int) -> str:
    return "|".join(n for b, n in FLAGS.items() if f & b) or "OK"


class Base:
    def __init__(self, port):
        # port : chemin du peripherique, ou tout objet a la maniere de
        # serial.Serial (write / read / reset_input_buffer).
        self.ser = serial.Serial(port, 115200, timeout=0) if isinstance(port, str) else port
        self._tampon = b""
        self.etat = None            # derniere trame S valide, sous forme de dict
        self.ident = None           # derniere ligne I : (nom, version, proto)

    # ---- emission ----
    def envoyer(self, cmd: str):
        self.ser.write((cmd + "\n").encode("ascii"))

    def vitesse(self, gauche: int, droite: int):
        """Consigne en ticks/s. A rappeler au moins toutes les 300 ms."""
        self.envoyer(f"v {int(gauche)} {int(droite)}")

    def stop(self):
        """Freine, et acquitte les defauts dont la cause a disparu."""
        self.envoyer("s")

    # ---- reception ----
    def lire(self):
        """Consomme tout ce qui est arrive ; met a jour self.etat / self.ident."""
        self._tampon += self.ser.read(4096)
        *lignes, self._tampon = self._tampon.split(b"\n")
        for brut in lignes:
            ligne = brut.decode("ascii", "replace").strip()
            if not ligne or not checksum_ok(ligne):
                continue                    # ligne tronquee ou corrompue
            p = ligne.split()
            if p[0] == "S" and len(p) == 16:
                self.etat = dict(zip(CHAMPS_S, map(int, p[1:15])))
            elif p[0] == "I" and len(p) == 5:
                self.ident = (p[1], p[2], int(p[3]))

    def identifier(self, delai=1.0):
        self.ser.reset_input_buffer()
        self.ident = None
        self.envoyer("i")
        fin = time.time() + delai
        while time.time() < fin and self.ident is None:
            self.lire()
            time.sleep(0.01)
        if self.ident is None:
            raise RuntimeError("pas de reponse a 'i' : mauvais port, ou firmware v1 ?")
        if self.ident[2] != PROTO_ATTENDU:
            raise RuntimeError(f"protocole v{self.ident[2]}, ce pilote attend v{PROTO_ATTENDU}")
        return self.ident


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
    base = Base(port)
    print("identification :", base.identifier())
    base.stop()
    base.envoyer("r")

    consigne = 500                  # ticks/s, prudent
    debut = time.time()
    try:
        while time.time() - debut < 3.0:
            base.lire()
            base.vitesse(consigne, consigne)        # renvoyee a chaque tour
            e = base.etat
            if e:
                print(f"t={e['t_ms']:>8} ms  enc {e['enc_l']:>7} {e['enc_r']:>7}  "
                      f"v {e['vel_l']:>6} {e['vel_r']:>6} ticks/s  "
                      f"{e['vbat_mv']/1000:5.2f} V {e['ibat_ma']/1000:+5.2f} A  "
                      f"{noms_flags(e['flags'])}")
                if e["flags"] & 0x80:
                    print("defaut verrouille : moteurs interdits, intervention requise")
                    break
            time.sleep(PERIODE_CONSIGNE)
    finally:
        base.stop()                 # toujours, meme sur Ctrl-C ou exception


if __name__ == "__main__":
    main()
