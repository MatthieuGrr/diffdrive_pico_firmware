#!/usr/bin/env python3
"""
Identification du modele de vitesse et calcul des gains PID — ROBOT AU SOL.

A executer avec le robot pose au sol, dans un espace degage, a son poids
final. Les gains obtenus ne valent que pour la charge et le revetement du
test : identifier sur cales donnerait des gains inutilisables en charge.

Deroulement :
  0. controle prealable : verifie que le firmware charge est bien le bon
  1. identification en boucle ouverte, echelons alternes avant/arriere
  2. ajustement d'un modele du premier ordre par roue
  3. calcul des gains par methode IMC, borne par les limites de la chaine
  4. verification en boucle fermee sur echelons de consigne

Les echelons sont systematiquement alternes : le robot revient a son point
de depart, l'excursion reste de l'ordre de deux tours de roue.

Securite : 's' est envoye sur toute sortie, y compris interruption. Si ce
script meurt, le chien de garde de 300 ms du firmware coupe les PWM.
"""
import serial, time, math, functools, operator, signal, sys, json

PORT        = "/dev/ttyACM0"
DUTIES      = [300, 500, 700, 900]   # niveaux d'identification
STEP_HOLD   = 0.9                    # duree d'un echelon, secondes
DEAD_MAX    = 320                    # borne du balayage de zone morte
VERIF_SP    = 1500                   # consigne de verification, ticks/s
VERIF_HOLD  = 1.2
ALPHA       = 0.5                    # VEL_LPF_ALPHA du firmware
T_CTRL      = 0.02                   # periode de controle
KFF         = 0.26                   # PID_KFF du firmware
MAX_TICKS_S = 5000                   # garde-fou de survitesse

TAU_FILTRE  = -T_CTRL / math.log(1 - ALPHA)   # 28.9 ms

# ---------------------------------------------------------------- liaison
s = None
_buf = b""

def connecter():
    global s
    s = serial.Serial(PORT, 115200, timeout=0.05)
    time.sleep(0.3); s.reset_input_buffer()

def env(c):
    s.write((c + "\n").encode()); s.flush()

def log(m):
    print(f"[{time.strftime('%H:%M:%S')}] {m}", flush=True)

def arret(*_):
    for _ in range(3):
        env("s"); time.sleep(0.1)
    log("MOTEURS COUPES")
    sys.exit(1)

signal.signal(signal.SIGINT, arret)
signal.signal(signal.SIGTERM, arret)

_stats = {"lignes": 0, "rejets": 0}

def frames():
    """Trames valides depuis le dernier appel : (t_ms, encG, encD, vitG, vitD)."""
    global _buf
    _buf += s.read(8192)
    *lig, _buf = _buf.split(b"\n")
    out = []
    for l in lig:
        l = l.decode("ascii", "replace").strip()
        if not l:
            continue
        _stats["lignes"] += 1
        if not l.startswith("S "):
            _stats["rejets"] += 1; continue
        try:
            ch, ck = l.rsplit(" ", 1)
            if functools.reduce(operator.xor, ch.encode(), 0) != int(ck, 16):
                _stats["rejets"] += 1; continue
            p = l.split()
            out.append((int(p[1]), int(p[2]), int(p[3]), int(p[4]), int(p[5])))
        except ValueError:
            _stats["rejets"] += 1
    return out

def tenir(cmd, duree):
    """Maintient une commande en re-armant le chien de garde. Renvoie les trames."""
    frames(); env(cmd); dernier = time.time(); fin = time.time() + duree; ech = []
    while time.time() < fin:
        if time.time() - dernier > 0.10:
            env(cmd); dernier = time.time()
        for f in frames():
            ech.append(f)
            if abs(f[3]) > MAX_TICKS_S or abs(f[4]) > MAX_TICKS_S:
                log(f"!! survitesse {f[3]}/{f[4]} — ARRET"); arret()
        time.sleep(0.004)
    return ech

def repos(d=0.6):
    env("s"); time.sleep(d); frames()

def attendre_arret(seuil=25, tmax=4.0):
    """Coupe et ATTEND l'arret reel des deux roues.

    Indispensable : le robot en charge continue sur son erre pendant plusieurs
    centaines de millisecondes. Sans cette attente, le mouvement residuel est
    compte comme un demarrage et la zone morte mesuree est trop basse ; et
    l'identification part d'une vitesse non nulle alors que le modele ajuste
    suppose v(0)=0."""
    env("s"); frames()
    fin = time.time() + tmax; calmes = 0
    while time.time() < fin:
        f = frames()
        if f:
            g, d = f[-1][3], f[-1][4]
            calmes = calmes + 1 if (abs(g) < seuil and abs(d) < seuil) else 0
            if calmes >= 5:          # 100 ms de calme consecutif
                return True
        time.sleep(0.004)
    return False

# ------------------------------------------------------- ajustement du modele
def ajuster(ech, duty):
    """
    Ajuste v(t) = K.u.(1 - exp(-(t-theta)/tau)) sur la REPONSE EN POSITION,
    dont l'expression integree est
        x(t) = K.u.((t-theta) - tau.(1 - exp(-(t-theta)/tau))) + c

    Deux raisons de travailler sur la position plutot que sur la vitesse :
      - les compteurs cumules sont exacts, alors que deriver amplifie la
        quantification de 50 ticks/s ;
      - cela evite completement de deconvoluer le filtre du firmware, puisqu'on
        n'utilise pas la vitesse qu'il publie.

    K ET c sont LINEAIRES dans le modele : a (tau, theta) fixe, on les resout
    tous deux analytiquement par moindres carres, et on ne balaie que deux
    parametres. L'offset libre c est essentiel : sans lui, ancrer la position
    sur le premier echantillon perdrait le deplacement deja accompli quand la
    capture demarre apres le debut de l'echelon, et sous-estimerait tau de
    20 a 30 %. theta est autorise a etre negatif pour la meme raison.
    """
    if len(ech) < 12:
        return None
    t0 = ech[0][0]
    idx = 1 if duty > 0 else 2          # colonne encodeur (placee par l'appelant)
    T = [(e[0] - t0) / 1000.0 for e in ech]
    X = [float(e[idx]) for e in ech]
    n = len(T)

    def essai(tau, theta):
        sb = sbb = sx = sbx = 0.0
        B = []
        for t, x in zip(T, X):
            dt = t - theta
            b = duty * (dt - tau * (1.0 - math.exp(-dt / tau))) if dt > 0 else 0.0
            B.append(b); sb += b; sbb += b * b; sx += x; sbx += b * x
        det = n * sbb - sb * sb
        if abs(det) < 1e-12:
            return None
        K = (n * sbx - sb * sx) / det
        c = (sbb * sx - sb * sbx) / det
        r = sum((x - K * b - c) ** 2 for b, x in zip(B, X))
        return r, K

    best = None
    for i in range(170):                      # tau : 10 ms .. 855 ms
        tau = 0.01 + 0.005 * i
        for j in range(21):                   # theta : -40 ms .. +60 ms
            th = -0.04 + 0.005 * j
            r = essai(tau, th)
            if r and (best is None or r[0] < best[0]):
                best = (r[0], tau, th, r[1])
    if best is None:
        return None
    _, tau, th, _ = best                      # affinage local
    for a in range(-5, 6):
        for b in range(-4, 5):
            tau2 = tau + 0.001 * a; th2 = th + 0.001 * b
            if tau2 <= 0: continue
            r = essai(tau2, th2)
            if r and r[0] < best[0]:
                best = (r[0], tau2, th2, r[1])
    _, tau, th, K = best
    return dict(tau=tau, theta=th, K=K, vss=K * duty)


def regression(pts):
    """v_ss = a.u + b  ->  gain incremental a, zone morte -b/a."""
    n = len(pts)
    su = sum(u for u, _ in pts); sv = sum(v for _, v in pts)
    suu = sum(u * u for u, _ in pts); suv = sum(u * v for u, v in pts)
    d = n * suu - su * su
    if abs(d) < 1e-9: return None, None
    a = (n * suv - su * sv) / d
    b = (sv - a * su) / n
    return a, (-b / a if a else None)

# =========================================================================
def main():
  resultats = {}
  connecter()
  try:
      repos(0.4); env("r"); time.sleep(0.4); frames()

      # ---- 0. controle prealable du firmware ------------------------------
      log("=== 0. CONTROLE DU FIRMWARE CHARGE ===")
      ech = tenir("o 500 500", 0.7); repos()
      if not ech:
          if _stats["lignes"] == 0:
              log("aucun octet recu — verifiez le cable et que rien d'autre")
              log("n'occupe le port (un screen ouvert, par exemple)")
          else:
              log(f"{_stats['lignes']} lignes recues mais {_stats['rejets']} "
                  f"rejetees : checksum ou format inattendu.")
              log("Le firmware charge n'est peut-etre pas celui de ce depot.")
          arret()
      vg = sum(e[3] for e in ech[-10:]) / 10; vd = sum(e[4] for e in ech[-10:]) / 10
      log(f"    duty +500 -> vit_G={vg:+.0f}  vit_D={vd:+.0f}")
      if vg < 0 or vd < 0:
          log("    ECHEC : un rapport cyclique POSITIF donne une vitesse NEGATIVE.")
          log("    C'est l'ancien binaire. Reflashez build/diffdrive_pico_firmware.uf2")
          log("    (correction MOT_x_INVERT + PID_KFF) avant de relancer.")
          arret()
      log("    OK — sens correct, le binaire est a jour.")

      # ---- 1a. zone morte en charge ---------------------------------------
      log("\n=== 1a. ZONE MORTE (au sol, en charge) ===")
      zm = {}
      for nom, i in (("G", 0), ("D", 1)):
          seuil = None
          for d in range(20, DEAD_MAX + 1, 20):
              cmd = f"o {d} 0" if i == 0 else f"o 0 {d}"
              attendre_arret()
              frames(); env(cmd); time.sleep(0.05)
              f = frames()
              while not f:
                  f = frames(); time.sleep(0.01)
              e0 = f[-1]; last = e0; der = time.time(); fin = time.time() + 0.6
              while time.time() < fin:
                  if time.time() - der > 0.10: env(cmd); der = time.time()
                  g = frames()
                  if g: last = g[-1]
                  time.sleep(0.004)
              repos(0.25)
              if abs(last[1 + i] - e0[1 + i]) > 3:   # marge sur le bruit
                  seuil = d; break
          zm[nom] = seuil
          log(f"    roue {nom} : premier mouvement a duty {seuil}")
      resultats["zone_morte"] = zm

      # ---- 1b. reponses indicielles ---------------------------------------
      log("\n=== 1b. REPONSES INDICIELLES (alternees, retour au point de depart) ===")
      modeles = {"G": [], "D": []}
      vss = {"G": [], "D": []}
      for duty in DUTIES:
          for sgn in (+1, -1):
              d = duty * sgn
              attendre_arret()
              ech = tenir(f"o {d} {d}", STEP_HOLD)
              repos(0.4)
              if len(ech) < 12:
                  log(f"    duty {d:+5} : trop peu de trames"); continue
              for nom, col in (("G", 1), ("D", 2)):
                  sous = [(e[0], e[col], e[col]) for e in ech]
                  m = ajuster(sous, d)
                  if not m: continue
                  modeles[nom].append(m)
                  # VALEURS ABSOLUES : avec les duty signes, la symetrie des
                  # points +/-u force l'ordonnee a l'origine a zero, ce qui
                  # efface la zone morte et fausse le gain incremental.
                  vss[nom].append((abs(d), abs(m["vss"])))
                  log(f"    duty {d:+5} roue {nom} : K={m['K']:.3f}  "
                      f"tau={1000*m['tau']:.0f} ms  retard={1000*m['theta']:.0f} ms  "
                      f"v_ss={m['vss']:+.0f} ticks/s")

      # ---- 2. synthese du modele ------------------------------------------
      log("\n=== 2. MODELE IDENTIFIE ===")
      modele = {}
      for nom in ("G", "D"):
          if not modeles[nom]:
              log(f"    roue {nom} : identification impossible"); arret()
          taus = sorted(m["tau"] for m in modeles[nom])
          tau = taus[len(taus) // 2]                       # mediane, robuste
          a, u0 = regression(vss[nom])
          modele[nom] = dict(tau=tau, K=a, u0=u0)
          log(f"    roue {nom} : gain {a:.3f} ticks/s par unite de duty | "
              f"tau {1000*tau:.0f} ms | zone morte extrapolee duty {u0:.0f}")
      resultats["modele"] = modele

      # ---- 3. point de depart analytique --------------------------------
      log("\n=== 3. POINT DE DEPART ===")
      log(f"    bornes de la chaine : filtre de mesure {1000*TAU_FILTRE:.0f} ms, "
          f"echantillonnage {1000*T_CTRL:.0f} ms")
      lam_min = max(3 * T_CTRL, 2 * TAU_FILTRE)
      # Depart volontairement mou. La formule IMC de manuel (kp = tau/(K.lambda),
      # ki = 1/(K.lambda)) donne jusqu'a 45 % de depassement ici, parce qu'elle
      # suppose qu'il n'y a PAS d'anticipation : avec kff, l'integrateur et
      # l'anticipation fournissent tous deux le regime etabli, et l'integrale
      # accumulee pendant la montee est en trop. Verifie en simulation contre
      # le vrai pid.c. On part donc bas et on monte par mesure.
      kps, kis = [], []
      for nom in ("G", "D"):
          tau, K = modele[nom]["tau"], modele[nom]["K"]
          lam = max(lam_min, tau)
          kps.append(0.50 * tau / (K * lam)); kis.append(0.15 / (K * lam))
          log(f"    roue {nom} : lambda {1000*lam:.0f} ms -> "
              f"kp {kps[-1]:.4f}  ki {kis[-1]:.3f}")
      kp, ki = min(kps), min(kis)      # le firmware n'a qu'un jeu : le plus doux
      log(f"    depart retenu : kp={kp:.4f}  ki={ki:.3f}")

      # ---- 4. auto-reglage sur le robot ----------------------------------
      log("\n=== 4. AUTO-REGLAGE SUR LE ROBOT (echelons alternes) ===")
      lam_ref = max(lam_min, max(modele[n]["tau"] for n in ("G", "D")))

      def essai(kp, ki):
          """Applique les gains, fait un aller-retour, renvoie le pire des deux
          roues en depassement et en temps d'etablissement."""
          env(f"k {kp:.5f} {ki:.5f} 0"); time.sleep(0.15)
          dep = etab = 0.0; det = []
          for sgn in (+1, -1):
              sp = VERIF_SP * sgn
              attendre_arret()
              ech = tenir(f"v {sp} {sp}", VERIF_HOLD); repos(0.4)
              if len(ech) < 20:
                  return None
              t0 = ech[0][0]
              for nom, col in (("G", 3), ("D", 4)):
                  v = [e[col] for e in ech]; t = [(e[0]-t0)/1000.0 for e in ech]
                  pic = max(v) if sgn > 0 else min(v)
                  d = 100*(abs(pic)-abs(sp))/abs(sp)
                  te = 0.0
                  for tt, vv in zip(t, v):
                      if abs(vv - sp) > 0.05*abs(sp): te = tt
                  fin_v = sum(v[-15:])/15
                  err = 100*(abs(fin_v)-abs(sp))/abs(sp)
                  dep = max(dep, d); etab = max(etab, te)
                  det.append((sgn, nom, d, te, err))
          return dep, etab, det

      DEP_CIBLE = 8.0
      meilleur = None
      for it in range(8):
          r = essai(kp, ki)
          if r is None:
              log("    trop peu de trames — liaison instable"); break
          dep, etab, det = r
          log(f"    iteration {it+1} : kp={kp:.4f} ki={ki:.3f} -> "
              f"depassement {dep:+5.1f} %  etabli en {etab:.2f} s")
          for sgn, nom, d, te, err in det:
              log(f"        consigne {VERIF_SP*sgn:+5} roue {nom} : "
                  f"dep {d:+5.1f} %  etabli {te:4.2f} s  err stat {err:+5.1f} %")
          if meilleur is None or (dep <= DEP_CIBLE + 4 and etab < meilleur[3]):
              meilleur = (kp, ki, dep, etab)
          if dep > DEP_CIBLE + 4:
              ki *= 0.55; kp *= 0.85
              log("        -> trop de depassement, on assouplit")
          elif etab > 2.5 * lam_ref and dep < DEP_CIBLE + 2:
              ki *= 1.5; kp *= 1.2
              log("        -> trop lent, on durcit")
          else:
              log("        -> criteres atteints")
              meilleur = (kp, ki, dep, etab); break

      if meilleur:
          kp, ki, dep, etab = meilleur
          env(f"k {kp:.5f} {ki:.5f} 0")
      resultats["gains"] = dict(kp=kp, ki=ki, kd=0.0)

      # ---- bilan -----------------------------------------------------------
      kff_reco = sum(1.0/modele[n]["K"] for n in ("G", "D")) / 2
      log("\n=== BILAN ===")
      log(f"    depassement {dep:.1f} %   etablissement {etab:.2f} s")
      log(f"    a mettre dans config.h :")
      log(f"        #define PID_KP_DEFAULT          {kp:.4f}f")
      log(f"        #define PID_KI_DEFAULT          {ki:.4f}f")
      log(f"        #define PID_KD_DEFAULT          0.0f")
      log(f"        #define PID_KFF                 {kff_reco:.3f}f"
          f"   (actuellement {KFF}, mesure au sol)")
      if abs(kff_reco - KFF) / KFF > 0.12:
          log(f"    NOTE : le kff mesure au sol ({kff_reco:.3f}) s'ecarte de "
              f"{100*abs(kff_reco-KFF)/KFF:.0f} % de celui flashe ({KFF}).")
          log(f"    Reflashez avec la nouvelle valeur puis relancez ce script :")
          log(f"    les gains seront meilleurs avec une anticipation juste.")
      resultats["kff_reco"] = kff_reco

  finally:
        for _ in range(3):
            env("s"); time.sleep(0.1)
        log("moteurs coupes.")
        try:
            with open("/tmp/tune_pid_resultats.json", "w") as f:
                json.dump(resultats, f, indent=2)
        except Exception:
            pass
        try: s.close()
        except Exception: pass


if __name__ == "__main__":
    main()
