"""Faux Pico : rejoue le comportement du firmware (protocole, PID, moteurs
avec zone morte, encodeurs quantifies) pour eprouver tune_pid.py de bout en
bout sans robot. Les frames sont emises en temps reel a 50 Hz."""
import time, math, functools, operator, sys

class Roue:
    def __init__(self, K, tau, dead, inv=1):
        self.K, self.tau, self.dead, self.inv = K, tau, dead, inv
        self.v = 0.0; self.x = 0.0
    def pas(self, duty, dt):
        mag = max(abs(duty) - self.dead, 0.0)
        cible = self.K * mag * (1 if duty >= 0 else -1)
        e = math.exp(-dt/self.tau)
        self.x += cible*dt + (self.v - cible)*self.tau*(1-e)
        self.v = cible + (self.v - cible)*e
        return int(self.x)

class PID:   # transcription fidele de pid.c
    def __init__(s, kp, ki, kd, kff, lo, hi):
        s.kp, s.ki, s.kd, s.kff, s.lo, s.hi = kp, ki, kd, kff, lo, hi; s.reset()
    def reset(s): s.integ=0.0; s.prev=0.0; s.primed=False; s.last=0.0
    def update(s, sp, meas, dt):
        if dt <= 0: return s.last
        err = sp - meas
        d = -s.kd*(meas - s.prev)/dt if (s.primed and s.kd) else 0.0
        s.prev = meas; s.primed = True
        it = s.integ + err*dt
        out = s.kff*sp + s.kp*err + s.ki*it + d
        if out > s.hi:
            out = s.hi
            if err > 0: it = s.integ
        elif out < s.lo:
            out = s.lo
            if err < 0: it = s.integ
        s.integ = it
        if s.ki > 0:
            s.integ = max(min(s.integ, s.hi/s.ki), s.lo/s.ki)
        else: s.integ = 0.0
        s.last = out; return out

class FauxPico:
    def __init__(self, gauche, droite, kff=0.26, kp=0.12, ki=0.60):
        self.G, self.D = gauche, droite
        self.pid = [PID(kp,ki,0,kff,-1000,1000), PID(kp,ki,0,kff,-1000,1000)]
        self.mode = "stop"; self.sp = [0,0]; self.ol = [0,0]
        self.off = [0,0]; self.sortie = b""; self.entree = b""
        self.t0 = time.time(); self.prochain = 0.0
        self.enc = [0,0]; self.prev = [0,0]; self.vf = [0.0,0.0]
        self.dernier_cmd = self.t0
    # --- interface serie ---
    def write(self, b):
        self.entree += b
        while b"\n" in self.entree:
            l, self.entree = self.entree.split(b"\n", 1)
            self._cmd(l.decode("ascii","replace").strip())
    def flush(self): pass
    def reset_input_buffer(self): self.sortie = b""
    def close(self): pass
    def read(self, n=1):
        self._avance()
        out, self.sortie = self.sortie[:n], self.sortie[n:]
        if not out: time.sleep(0.002)
        return out
    # --- firmware ---
    def _cmd(self, l):
        p = l.split()
        if not p: return
        c = p[0]; ok = False
        try:
            if c=="v" and len(p)==3: self.sp=[int(p[1]),int(p[2])]; self.mode="vel"; ok=True
            elif c=="o" and len(p)==3:
                self.ol=[max(-1000,min(1000,int(p[1]))),max(-1000,min(1000,int(p[2])))]
                self.mode="ol"; [q.reset() for q in self.pid]; ok=True
            elif c=="s" and len(p)==1:
                self.mode="stop"; self.sp=[0,0]; self.ol=[0,0]
                [q.reset() for q in self.pid]; ok=True
            elif c=="r" and len(p)==1:
                self.off=[int(self.G.x), int(self.D.x)]; self.prev=[0,0]; ok=True
            elif c=="k" and len(p)==4:
                for q in self.pid: q.kp,q.ki,q.kd = float(p[1]),float(p[2]),float(p[3])
                ok=True
            elif c=="t" and len(p)==2: ok=True
        except ValueError: ok=False
        if ok: self.dernier_cmd = time.time()
    def _avance(self):
        now = time.time() - self.t0
        while now >= self.prochain:
            dt = 0.02
            if time.time() - self.dernier_cmd > 0.300 and self.mode != "stop":
                self.mode = "stop"; [q.reset() for q in self.pid]
            duty = [0,0]
            if self.mode == "ol": duty = list(self.ol)
            elif self.mode == "vel":
                duty = [int(round(self.pid[i].update(self.sp[i], self.vf[i], dt)))
                        for i in (0,1)]
                duty = [max(-1000,min(1000,d)) for d in duty]
            xg = self.G.pas(duty[0], dt); xd = self.D.pas(duty[1], dt)
            self.enc = [xg - self.off[0], xd - self.off[1]]
            for i in (0,1):
                brut = (self.enc[i] - self.prev[i]) / dt
                self.prev[i] = self.enc[i]
                self.vf[i] += 0.5*(brut - self.vf[i])
            t_ms = int(self.prochain*1000)
            ch = (f"S {t_ms} {self.enc[0]} {self.enc[1]} "
                  f"{int(round(self.vf[0]))} {int(round(self.vf[1]))} 0 0 0 0 0 0")
            ck = functools.reduce(operator.xor, ch.encode(), 0)
            self.sortie += f"{ch} {ck:02X}\n".encode()
            self.prochain += dt
