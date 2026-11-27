#!/usr/bin/env python3
"""Genere docs/cablage.svg : brochage du Pico 2 et cablage de la base v2."""
from pathlib import Path

W, H = 1500, 1270
PITCH = 40
TOP = 150
BX0, BX1 = 600, 860          # corps du Pico
LX, RX = 330, 1130           # bord interieur des composants gauche / droite

# Brochage physique du Pico 2, vu de dessus, USB en haut.
LEFT = ["GP0", "GP1", "GND", "GP2", "GP3", "GP4", "GP5", "GND", "GP6", "GP7",
        "GP8", "GP9", "GND", "GP10", "GP11", "GP12", "GP13", "GND", "GP14", "GP15"]
RIGHT = ["VBUS", "VSYS", "GND", "3V3_EN", "3V3(OUT)", "ADC_VREF", "GP28", "GND",
         "GP27", "GP26", "RUN", "GP22", "GND", "GP21", "GP20", "GP19", "GP18",
         "GND", "GP17", "GP16"]          # broches 40 -> 21

C = {"sig": "#1f5fbf", "i2c": "#7a3db8", "gnd": "#222", "pwr": "#c62828",
     "free": "#b0b0b0", "box": "#f4f6fa", "edge": "#56657a"}

out = []
def add(s): out.append(s)

def y_of(i): return TOP + i * PITCH

def text(x, y, s, size=14, anchor="start", weight="normal", fill="#222"):
    add(f'<text x="{x}" y="{y}" font-size="{size}" text-anchor="{anchor}" '
        f'font-weight="{weight}" fill="{fill}">{s}</text>')

def wire(x1, y1, x2, y2, color, label=None, lx=None, anchor="middle"):
    add(f'<path d="M{x1},{y1} H{x2}" stroke="{color}" stroke-width="2.5" fill="none"/>')
    if label:
        text(lx if lx is not None else (x1 + x2) / 2, y1 - 5, label, 12, anchor, fill=color)

def box(x, y, w, h, title, sub=None, tx=None):
    add(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="8" fill="{C["box"]}" '
        f'stroke="{C["edge"]}" stroke-width="1.5"/>')
    tx = x + 14 if tx is None else tx
    text(tx, y + 24, title, 15, weight="bold")
    if sub:
        for k, line in enumerate(sub.split("|")):
            text(tx, y + 44 + 17 * k, line, 12, fill="#555")

def tag(x, y, s, color, anchor="start"):
    w = 9 * len(s) + 14
    rx = x if anchor == "start" else x - w
    add(f'<rect x="{rx}" y="{y-11}" width="{w}" height="20" rx="10" fill="{color}"/>')
    text(rx + w / 2, y + 4, s, 12, "middle", "bold", "#fff")

def pinpad(x, y, name, used):
    col = "#2e7d32" if used else "#fff"
    add(f'<circle cx="{x}" cy="{y}" r="7" fill="{col}" stroke="#333" stroke-width="1.2"/>')

# ---------------------------------------------------------------- en-tete
add(f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" '
    f'viewBox="0 0 {W} {H}" font-family="DejaVu Sans, Arial, sans-serif">')
add(f'<rect width="{W}" height="{H}" fill="#fff"/>')
text(W / 2, 40, "Base différentielle v2 — câblage du Raspberry Pi Pico 2", 24, "middle", "bold")
text(W / 2, 66, "Vue de dessus, USB en haut. Tous les signaux sont en 3,3 V. "
     "Rails communs : 3V3 = broche 36, GND = broche 38.", 14, "middle", fill="#444")

usedL = {"GP0", "GP1", "GP2", "GP3", "GP4", "GP5", "GP6", "GP7", "GP8", "GP9"}
usedR = {"GP16", "GP17", "GP18", "GP19", "GP20", "GP21", "3V3(OUT)"}
gndL = {2, 7, 12}             # index des GND cables a gauche
gndR = {2, 17, 12}            # broche 38, 23, 28

# ---------------------------------------------------------------- Pico
add(f'<rect x="{BX0}" y="{TOP-50}" width="{BX1-BX0}" height="{20*PITCH+60}" rx="10" '
    f'fill="#1b6e3a" stroke="#0d3d1f" stroke-width="2"/>')
add(f'<rect x="{(BX0+BX1)/2-30}" y="{TOP-66}" width="60" height="26" rx="4" fill="#bbb" stroke="#666"/>')
text((BX0 + BX1) / 2, TOP - 48, "USB", 11, "middle", "bold")
text((BX0 + BX1) / 2, TOP + 9.5 * PITCH, "Pico 2", 22, "middle", "bold", "#fff")
text((BX0 + BX1) / 2, TOP + 9.5 * PITCH + 24, "RP2350", 14, "middle", fill="#d7f0de")
text((BX0 + BX1) / 2, TOP + 9.5 * PITCH + 70, "USB → Jetson", 13, "middle", fill="#fff")
text((BX0 + BX1) / 2, TOP + 9.5 * PITCH + 88, "(alimentation + protocole)", 11, "middle", fill="#d7f0de")

for i, n in enumerate(LEFT):
    y = y_of(i)
    used = n in usedL or i in gndL
    pinpad(BX0, y, n, used)
    text(BX0 + 14, y + 5, n, 12, fill="#fff" if used else "#9fc9ad")
    text(BX0 - 14, y + 5, str(i + 1), 11, "end", fill="#888")
for i, n in enumerate(RIGHT):
    y = y_of(i)
    used = n in usedR or i in gndR
    pinpad(BX1, y, n, used)
    text(BX1 - 14, y + 5, n, 12, "end", fill="#fff" if used else "#9fc9ad")
    text(BX1 + 14, y + 5, str(40 - i), 11, fill="#888")

# ---------------------------------------------------------------- gauche
def left_block(i0, i1, title, sub, pins, vcc=False):
    y0, y1 = y_of(i0) - 18, y_of(i1) + 18
    box(40, y0, LX - 40, y1 - y0, title, sub)
    for i, (label, kind, name) in pins:
        y = y_of(i)
        wire(LX, y, BX0 - 7, y, C[kind], name, (LX + BX0) / 2)
        text(LX - 10, y + 5, label, 13, "end")
    if vcc:
        tag(54, y1 - 22, "VCC vert ← 3V3", C["pwr"])

left_block(0, 2, "Adaptateur USB-UART", "logs de débogage|115200 8N1", [
    (0, ("RX", "sig", "GP0 · UART0 TX")),
    (1, ("TX", "sig", "GP1 · UART0 RX")),
    (2, ("GND", "gnd", "GND"))])
left_block(3, 7, "Cytron MDD10A", "connecteur logique|5 broches", [
    (3, ("PWM1", "sig", "GP2 · PWM moteur G")),
    (4, ("DIR1", "sig", "GP3 · DIR moteur G")),
    (5, ("PWM2", "sig", "GP4 · PWM moteur D")),
    (6, ("DIR2", "sig", "GP5 · DIR moteur D")),
    (7, ("GND", "gnd", "GND"))])
left_block(8, 12, "Encodeurs SCX-555", "16 PPR côté moteur|⚠ rouge/noir = signaux", [
    (8, ("G · A noir", "sig", "GP6 · enc G A")),
    (9, ("G · B rouge", "sig", "GP7 · enc G B")),
    (10, ("D · A noir", "sig", "GP8 · enc D A")),
    (11, ("D · B rouge", "sig", "GP9 · enc D B")),
    (12, ("GND bleu ×2", "gnd", "GND"))], vcc=True)

# ---------------------------------------------------------------- droite
def right_block(i0, i1, title, sub, pins, vcc=True, extra_h=0):
    y0, y1 = y_of(i0) - 18, y_of(i1) + 18
    box(RX, y0, W - 40 - RX, y1 - y0, title, sub, tx=RX + 110)
    for i, (label, kind, name) in pins:
        y = y_of(i)
        wire(BX1 + 7, y, RX, y, C[kind], name, (RX + BX1) / 2)
        text(RX + 10, y + 5, label, 13)
    if vcc:
        tag(RX + 110, y1 - 22, vcc if isinstance(vcc, str) else "VCC ← 3V3", C["pwr"])

# Rails d'alimentation
y36, y38, y40 = y_of(4), y_of(2), y_of(0)
box(RX, y_of(0) - 18, W - 40 - RX, 4 * PITCH + 36, "Rails", "barrette 3V3 / GND", tx=RX + 150)
wire(BX1 + 7, y40, RX, y40, C["free"], "VBUS : non câblé", (RX + BX1) / 2)
wire(BX1 + 7, y38, RX, y38, C["gnd"], "GND", (RX + BX1) / 2)
wire(BX1 + 7, y36, RX, y36, C["pwr"], "3V3(OUT), 300 mA max", (RX + BX1) / 2)
text(RX + 10, y38 + 5, "rail GND", 13)
text(RX + 10, y36 + 5, "rail 3V3", 13, fill=C["pwr"], weight="bold")

right_block(12, 13, "Arrêt d'urgence", "contact auxiliaire NF", [
    (12, ("borne 1", "gnd", "GND")),
    (13, ("borne 2", "sig", "GP21 · AU (1 = enfoncé)"))], vcc=False)
tag(W - 50, y_of(13) - 1, "tirage interne", C["sig"], "end")
right_block(14, 16, "INA226 (0x40)", "I2C1, 400 kHz|bus à lui seul|shunt 10 mΩ", [
    (14, ("ALERT", "sig", "GP20 · ALERT (IRQ)")),
    (15, ("SCL", "i2c", "GP19 · I2C1 SCL")),
    (16, ("SDA", "i2c", "GP18 · I2C1 SDA"))], vcc="VCC ← 3V3, GND ← rail")
right_block(17, 19, "BNO055 (0x28)", "I2C0, 100 kHz", [
    (17, ("GND", "gnd", "GND")),
    (18, ("SCL", "i2c", "GP17 · I2C0 SCL")),
    (19, ("SDA", "i2c", "GP16 · I2C0 SDA"))])

# ---------------------------------------------------------------- puissance
py = TOP + 20 * PITCH + 40
add(f'<line x1="40" y1="{py}" x2="{W-40}" y2="{py}" stroke="#ccc" stroke-dasharray="6 6"/>')
text(40, py + 30, "Puissance (fils de section adaptée, 10 A max par canal)", 17, weight="bold")
ry = py + 100
nodes = [(60, 180, "Batterie 20 V", "outillage, ~18 V"), (265, 120, "Fusible", "~10 A"),
         (410, 225, "INA226", "IN+ → shunt 10 mΩ → IN−"),
         (660, 165, "Arrêt d'urgence", "contact NF puissance"),
         (850, 120, "MDD10A", "B+ / B−")]
for x, w, t, s in nodes:
    add(f'<rect x="{x}" y="{ry-30}" width="{w}" height="60" rx="8" '
        f'fill="{C["box"]}" stroke="{C["edge"]}"/>')
    text(x + 12, ry - 6, t, 14, weight="bold")
    text(x + 12, ry + 14, s, 11, fill="#555")
for (x1, w1, *_), (x2, *_) in zip(nodes, nodes[1:]):
    add(f'<path d="M{x1 + w1},{ry} H{x2}" stroke="{C["pwr"]}" stroke-width="4" fill="none"/>')
add(f'<path d="M150,{ry+30} V{ry+70} H910 V{ry+30}" stroke="{C["gnd"]}" stroke-width="4" fill="none"/>')
text(530, ry + 64, "Batterie − → MDD10A B− (masse commune avec le GND du Pico)", 12, "middle")
text(742, ry - 42, "après l'INA226 : la batterie reste mesurée bouton enfoncé", 11, "middle", fill="#555")
text(742, ry + 92, "2ᵉ contact NF du même bouton → GP21 (ci-dessus)", 11, "middle", fill=C["sig"])
add(f'<path d="M970,{ry-12} H1040" stroke="#555" stroke-width="3" fill="none"/>')
add(f'<path d="M970,{ry+12} H1040" stroke="#555" stroke-width="3" fill="none"/>')
for k, (lab, mot) in enumerate([("M1A / M1B", "moteur G, fils blanc / jaune"), ("M2A / M2B", "moteur D, fils blanc / jaune")]):
    yy = ry - 34 + k * 48
    add(f'<rect x="1040" y="{yy}" width="380" height="40" rx="8" fill="{C["box"]}" stroke="{C["edge"]}"/>')
    text(1052, yy + 25, f"{lab} → {mot}", 13)

# ---------------------------------------------------------------- legende
ly = H - 60
for k, (lab, col) in enumerate([("signal 3,3 V", C["sig"]), ("I2C", C["i2c"]),
                                 ("GND", C["gnd"]), ("3V3 / puissance", C["pwr"]),
                                 ("non câblé", C["free"])]):
    x = 60 + k * 230
    add(f'<path d="M{x},{ly} H{x+40}" stroke="{col}" stroke-width="3" fill="none"/>')
    text(x + 50, ly + 5, lab, 13)
add(f'<circle cx="1230" cy="{ly}" r="7" fill="#2e7d32" stroke="#333"/>')
text(1244, ly + 5, "broche utilisée", 13)
text(W / 2, H - 22, "Libres : GP10–GP15, GP22, GP26–GP28 (ADC). "
     "Encodeurs alimentés en 3,3 V : leurs résistances de tirage embarquées restent sûres pour le RP2350. ALERT : drain ouvert, tirage interne. AU : tirage interne, fils courts.",
     12, "middle", fill="#555")
add("</svg>")

Path(__file__).with_name("cablage.svg").write_text("\n".join(out), encoding="utf-8")
print("ok")
