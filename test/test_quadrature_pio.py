M = 0xFFFFFFFF
TABLE = [21,16,20,21,20,21,21,16,16,21,21,20,21,20,16,21]

class SM:
    def __init__(self):
        self.pc=0; self.x=0; self.y=0; self.isr=0; self.fifo=[]
    def step(self, pins):
        pc=self.pc; nxt=(pc+1)&0x1F
        if pc < 16: self.pc=TABLE[pc]; return
        if pc in (16,18): self.y=(~self.y)&M; self.pc=nxt
        elif pc==17:
            nz=self.y!=0; self.y=(self.y-1)&M; self.pc=18 if nz else nxt
        elif pc==19: self.pc=21
        elif pc==20:
            nz=self.y!=0; self.y=(self.y-1)&M; self.pc=21 if nz else nxt
        elif pc==21: self.x=self.isr; self.pc=nxt
        elif pc==22: self.isr=self.y; self.pc=nxt
        elif pc==23:
            if len(self.fifo)<4: self.fifo.append(self.isr)
            self.isr=0; self.pc=nxt
        elif pc==24: self.isr=0; self.pc=nxt
        elif pc==25: self.isr=((self.isr<<2)|(self.x&3))&M; self.pc=nxt
        elif pc==26: self.isr=((self.isr<<2)|(pins&3))&M; self.pc=nxt
        elif pc==27: self.pc=self.isr&0x1F
        else: raise AssertionError(pc)

def signed(v): return v-(1<<32) if v & 0x80000000 else v

def read_count(sm, pins, budget=400):
    """quad_read() : vide la FIFO puis attend une publication fraiche."""
    while sm.fifo: sm.fifo.pop(0)
    for _ in range(budget):
        sm.step(pins)
        if sm.fifo: return signed(sm.fifo.pop(0))
    raise AssertionError("aucune publication fraiche")

def run(states, hold, settle=80):
    """Reproduit la sequence reelle : amorcage, prise de la reference
    (encoders_reset), puis mesure du delta — comme le firmware."""
    sm=SM()
    for _ in range(settle): sm.step(states[0])
    base = read_count(sm, states[0])          # <- encoders_reset()
    for s in states[1:]:
        for _ in range(hold): sm.step(s)
    return read_count(sm, states[-1]) - base

FWD=[0,1,3,2]; REV=[0,2,3,1]
def cyc(seq,n): return (seq*n)+[seq[0]]

ok=True
def check(label, got, want):
    global ok
    if got!=want: ok=False
    print(f"  {'OK ' if got==want else 'ECHEC'} {label:46s} obtenu={got:+6d} attendu={want:+6d}")

print("Decodage x4 (30 cycles PIO par etat) :")
for t in (1,4,10,50):
    check(f"avant, {t} cycles de quadrature",   run(cyc(FWD,t),30),  4*t)
    check(f"arriere, {t} cycles de quadrature", run(cyc(REV,t),30), -4*t)

print("\nUn tour de roue complet — 385 cycles de quadrature = 1540 counts :")
check("avant",   run(cyc(FWD,385),30),  1540)
check("arriere", run(cyc(REV,385),30), -1540)

print("\nAller-retour, retour exact au point de depart :")
check("+100 transitions puis -100", run(cyc(FWD,25)+REV*25+[REV[0]],30), 0)

print("\nEtat de depart quelconque : pas de comptage fantome (grace a la reference) :")
for s0 in (0,1,2,3):
    i = FWD.index(s0)
    seq = [s0] + [FWD[(i+1+k) % 4] for k in range(12)]   # 12 vraies transitions
    check(f"depart en etat {s0}, 12 transitions avant", run(seq,30), 12)

print("\nTransitions illegales (les deux voies changent) : jamais comptees :")
check("0,3,0,3,0", run([0,3,0,3,0],30), 0)
check("1,2,1,2,1", run([1,2,1,2,1],30), 0)
check("2,1,2,1,2", run([2,1,2,1,2],30), 0)

print("\nGigue / rebond sur une frontiere : aucune derive :")
check("0,1,0,1,0,1,0", run([0,1,0,1,0,1,0],30), 0)
check("3,2,3,2,3,2,3", run([3,2,3,2,3,2,3],30), 0)

print("\nCompteur negatif et passage par zero :")
check("40 transitions arriere depuis la reference", run(cyc(REV,10),30), -40)

print("\nDuree minimale de maintien d'un etat (limite d'echantillonnage) :")
worst=None
for hold in range(1,30):
    got = run(cyc(FWD,20),hold)
    if got==80 and worst is None: worst=hold
    elif got!=80: worst=None
print(f"  maintien minimal sans perte de front : {worst} cycles PIO")
check("13 cycles de maintien suffisent", run(cyc(FWD,20),13), 80)

print("\n=> " + ("TOUS LES CAS PASSENT" if ok else "AU MOINS UN ECHEC"))
