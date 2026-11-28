#!/usr/bin/env bash
# Bancs d'essai hote du firmware diffdrive_pico_firmware.
#
# Ils ne remplacent pas la validation sur materiel : ils verifient la logique
# qui ne depend pas du RP2350, sur les modules ou une erreur couterait le plus
# cher a diagnostiquer au banc.
#
#   - le decodeur de quadrature PIO, simule instruction par instruction
#   - le parseur de commandes et le checksum de la trame d'etat
#   - la table de verite du MDD10A et l'inhibition de securite
#   - les protections batterie (surintensite, sous-tension) et l'INA226
#   - l'anti-emballement du PID
#
# src/protocol.c, src/motors.c, src/pid.c et src/safety.c sont compiles TELS QUELS pour
# l'hote ; seuls les acces materiel sont bouchonnes (test/stubs).
set -u
cd "$(dirname "$0")/.."
ST=test/stubs
CC="${CC:-gcc}"
CFLAGS="-std=c11 -Wall -Wextra -O1 -Iinclude -I$ST"
OUT=$(mktemp -d); trap 'rm -rf "$OUT"' EXIT
rc=0

echo "########## decodeur de quadrature PIO ##########"
python3 test/test_quadrature_pio.py || rc=1

echo; echo "########## protocole serie ##########"
$CC $CFLAGS -o "$OUT/p" test/test_protocol.c src/protocol.c src/pid.c && "$OUT/p" || rc=1

echo; echo "########## PID ##########"
$CC $CFLAGS -o "$OUT/i" test/test_pid.c src/pid.c -lm && "$OUT/i" || rc=1

echo; echo "########## limitation du courant d'appel ##########"
$CC $CFLAGS -o "$OUT/l" test/test_ilim.c src/pid.c -lm && "$OUT/l" || rc=1

echo; echo "########## moteurs — MDD10A ##########"
$CC $CFLAGS -o "$OUT/m" test/test_motors.c src/motors.c -lm && "$OUT/m" || rc=1

echo; echo "########## securite batterie et conversions INA226 ##########"
$CC $CFLAGS -o "$OUT/s" test/test_safety.c src/safety.c && "$OUT/s" || rc=1

echo
if [ $rc -eq 0 ]; then echo "=== TOUS LES BANCS PASSENT ==="
else echo "=== AU MOINS UN BANC A ECHOUE ==="; fi
exit $rc
