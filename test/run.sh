#!/usr/bin/env bash
# Bancs d'essai hote du firmware diffdrive_pico_firmware.
#
# Ils ne remplacent pas la validation sur materiel : ils verifient la logique
# qui ne depend pas du RP2350, sur les modules ou une erreur couterait le plus
# cher a diagnostiquer au banc.
#
#   - le decodeur de quadrature PIO, simule instruction par instruction
#   - le parseur de commandes et le checksum de la trame d'etat
#   - la table de verite du DRV8871 dans les deux modes de decroissance
#   - l'anti-emballement du PID
#
# Les modules protocol.c, motors.c et pid.c sont compiles TELS QUELS pour
# l'hote ; seuls les acces au materiel sont bouchonnes (test/stubs).
set -u
cd "$(dirname "$0")/.."
ST=test/stubs
CC="${CC:-gcc}"
CFLAGS="-std=c11 -Wall -Wextra -O1 -I. -I$ST"
OUT=$(mktemp -d); trap 'rm -rf "$OUT"' EXIT
rc=0

echo "########## decodeur de quadrature PIO ##########"
python3 test/test_quadrature_pio.py || rc=1

echo; echo "########## protocole serie ##########"
$CC $CFLAGS -o "$OUT/p" test/test_protocol.c protocol.c pid.c && "$OUT/p" || rc=1

echo; echo "########## PID ##########"
$CC $CFLAGS -o "$OUT/i" test/test_pid.c pid.c -lm && "$OUT/i" || rc=1

for mode in DECAY_FAST DECAY_SLOW; do
    echo; echo "########## moteurs — MOTOR_DECAY_MODE = $mode ##########"
    $CC $CFLAGS -DMOTOR_DECAY_MODE=$mode -o "$OUT/m" test/test_motors.c motors.c -lm \
        && "$OUT/m" || rc=1
done

echo
if [ $rc -eq 0 ]; then echo "=== TOUS LES BANCS PASSENT ==="
else echo "=== AU MOINS UN BANC A ECHOUE ==="; fi
exit $rc
