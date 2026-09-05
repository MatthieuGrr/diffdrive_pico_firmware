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
import serial, time, math, functools, operator, signal, sys, json, os
import rapport

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

PORT        = "/dev/ttyACM0"
RAPPORT     = "reglage_pid.html"       # graphiques de toutes les mesures
DUTIES      = [300, 500, 700, 900]   # niveaux d'identification
STEP_HOLD   = 0.9                    # duree d'un echelon, secondes
DEAD_MAX    = 320                    # borne du balayage de zone morte
VERIF_SP    = 1500                   # consigne de verification, ticks/s
VERIF_HOLD  = 2.0      # assez long pour que l'etablissement soit mesurable
ALPHA       = 0.5                    # VEL_LPF_ALPHA du firmware
T_CTRL      = 0.02                   # periode de controle
KFF         = 0.26                   # PID_KFF du firmware
MAX_TICKS_S = 5000                   # garde-fou de survitesse

TAU_FILTRE  = -T_CTRL / math.log(1 - ALPHA)   # 28.9 ms

# ---------------------------------------------------------------- liaison
s = None
_buf = b""

def connecter():
    """Ouvre la liaison, avec un diagnostic lisible plutot qu'une trace.
    Un port ABSENT et un port LIBRE ne se distinguent pas avec fuser : il faut
    tester l'existence du fichier."""
    global s
    if not os.path.exists(PORT):
        log(f"{PORT} n'existe pas — le Pico n'est pas vu par la machine.")
        log("Verifiez le cable USB (c'est le suspect numero un quand le robot")
        log("vient de rouler), puis que 'lsusb' montre bien 2e8a:0009.")
        sys.exit(2)
    try:
        s = serial.Serial(PORT, 115200, timeout=0.05)
    except serial.SerialException as e:
        log(f"{PORT} existe mais refuse de s'ouvrir : {e}")
        log("Un autre programme le tient peut-etre (un screen reste ouvert ?).")
        sys.exit(2)
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
    idx = 1 if duty > 0 else 2          # colonne encodeur (placee par l'appelant)

    # DETECTION DU DEMARRAGE REEL. Une roue peut rester collee plusieurs
    # centaines de millisecondes avant de partir d'un coup : mesure sur ce
    # robot, 360 ms d'immobilite a duty -700. Chercher le depart dans une
    # fenetre de +/-60 ms autour de la commande fait alors interpreter la
    # montee tardive comme un gain enorme. On recale donc la fenetre sur le
    # premier mouvement franc, deux echantillons avant.
    x0 = ech[0][idx]
    dep = 0
    for k, e in enumerate(ech):
        if abs(e[idx] - x0) > 4:
            dep = max(0, k - 2); break
    else:
        return None                     # la roue n'a jamais bouge
    ech = ech[dep:]
    if len(ech) < 10:
        return None
    t0 = ech[0][0]
    T = [(e[0] - t0) / 1000.0 for e in ech]
    X = [float(e[idx]) for e in ech]
    n = len(T)
    retard_colle = dep * 0.02

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
    r, tau, th, K = best
    # Qualite : residu quadratique moyen rapporte a l'amplitude du deplacement.
    etendue = max(X) - min(X)
    qualite = (math.sqrt(r / n) / etendue) if etendue > 1 else 9.9
    # Un parametre colle a une borne de grille signale un ajustement qui n'a
    # pas trouve d'optimum interne : la valeur ne veut rien dire.
    borne = tau <= 0.012 or tau >= 0.850 or th <= -0.039 or th >= 0.059
    return dict(tau=tau, theta=th, K=K, vss=K * duty,
                qualite=qualite, borne=borne, colle=retard_colle)


def regression(pts, robuste=True):
    """v_ss = a.u + b  ->  gain incremental a, zone morte -b/a.

    Avec rejet des aberrants : une roue qui colle puis se decroche produit un
    point tres eloigne, et un seul suffit a fausser la pente."""
    if robuste and len(pts) >= 5:
        a0, _ = _moindres_carres(pts)
        if a0:
            res = sorted(abs(v - a0 * u) for u, v in pts)
            med = res[len(res) // 2]
            garde = [(u, v) for u, v in pts
                     if abs(v - a0 * u) <= max(3.0 * med, 0.12 * abs(v) + 50)]
            if len(garde) >= 4:
                pts = garde
    return _moindres_carres(pts)


def _moindres_carres(pts):
    n = len(pts)
    su = sum(u for u, _ in pts); sv = sum(v for _, v in pts)
    suu = sum(u * u for u, _ in pts); suv = sum(u * v for u, v in pts)
    d = n * suu - su * su
    if abs(d) < 1e-9: return None, None
    a = (n * suv - su * sv) / d
    b = (sv - a * su) / n
    return a, (-b / a if a else None)


# =========================================================================
def generer_rapport(res, chemin):
    """Assemble le rapport HTML. Tolerant a un run interrompu : chaque section
    n'est produite que si les donnees correspondantes existent."""
    import html as _h
    S, SD = rapport.SERIE, rapport.SERIE_D
    Ra, RaD = rapport.RAMPE, rapport.RAMPE_D
    sections = []

    # --- resume ----------------------------------------------------------
    g = res.get("gains"); mod = res.get("modele")
    if g or mod:
        dl = []
        if mod:
            for n in ("G", "D"):
                if n in mod:
                    dl.append((f"Roue {n} — gain",
                               f"{mod[n]['K']:.3f} ticks/s par unite de duty"))
                    dl.append((f"Roue {n} — constante de temps",
                               f"{1000*mod[n]['tau']:.0f} ms"))
        if res.get("zone_morte"):
            for n in ("G", "D"):
                if res["zone_morte"].get(n):
                    dl.append((f"Roue {n} — zone morte",
                               f"duty {res['zone_morte'][n]}"))
        if g:
            dl.append(("Gains retenus",
                       f"kp {g['kp']:.4f} · ki {g['ki']:.4f} · kd 0"))
        it = res.get("iterations")
        if it:
            dl.append(("Resultat final",
                       f"depassement {it[-1]['dep']:.1f} % · "
                       f"etabli en {it[-1]['etab']:.2f} s"))
        corps = "".join(f"<dt>{_h.escape(a)}</dt><dd>{_h.escape(b)}</dd>"
                        for a, b in dl)
        bloc = f'<div class="res"><dl>{corps}</dl></div>'
        if g:
            bloc += ('<pre>' + _h.escape(
                f"#define PID_KP_DEFAULT          {g['kp']:.4f}f\n"
                f"#define PID_KI_DEFAULT          {g['ki']:.4f}f\n"
                f"#define PID_KD_DEFAULT          0.0f\n"
                + (f"#define PID_KFF                 {res['kff_reco']:.3f}f"
                   if res.get("kff_reco") else "")) + '</pre>')
        sections.append(("", [bloc]))

    # --- reponses indicielles --------------------------------------------
    cs = res.get("courbes_step") or []
    if cs:
        blocs = []
        for nom in ("G", "D"):
            duties = sorted({c["duty"] for c in cs
                             if c["roue"] == nom and c["duty"] > 0})
            series = []
            for k, d in enumerate(duties[:4]):
                c = next(x for x in cs if x["roue"] == nom and x["duty"] == d)
                series.append(dict(nom=f"duty {d}", couleur=Ra[k],
                                   couleur_sombre=RaD[k], etiquette=True,
                                   pts=list(zip(c["t"], c["v"]))))
            for k, d in enumerate(duties[:4]):
                c = next(x for x in cs if x["roue"] == nom and x["duty"] == d)
                series.append(dict(nom=f"modele {d}", couleur=Ra[k],
                                   couleur_sombre=RaD[k], tirets=True,
                                   cache_legende=True,
                                   pts=list(zip(c["t"], c["vm"]))))
            if series:
                blocs.append(rapport.graphe_lignes(
                    f"Reponse indicielle en boucle ouverte — roue {nom}",
                    "trait plein : vitesse mesuree · tirets : modele ajuste, "
                    "passe par le meme filtre que le firmware",
                    series, "temps (s)", "vitesse (ticks/s)", ident=True))
        sections.append(("Identification du modele", blocs))

    # --- caracteristique statique ----------------------------------------
    if mod and res.get("statique"):
        series = []
        for k, nom in enumerate(("G", "D")):
            pts = res["statique"].get(nom) or []
            if not pts:
                continue
            K, u0 = mod[nom]["K"], mod[nom]["u0"]
            xmax = max(x for x, _ in pts)
            series.append(dict(nom=f"roue {nom}", couleur=S[k],
                               couleur_sombre=SD[k], pts=pts,
                               droite=[(u0, 0.0), (xmax, K * (xmax - u0))],
                               zm=u0))
        if series:
            sections.append(("Caracteristique statique", [rapport.graphe_nuage(
                "Vitesse etablie en fonction du rapport cyclique",
                "la droite est la regression ; son intersection avec l'axe "
                "donne la zone morte extrapolee (cercle creux)",
                series, "rapport cyclique", "vitesse etablie (ticks/s)",
                ident=True)]))

    # --- zone morte -------------------------------------------------------
    zc = res.get("courbes_zm") or {}
    if any(zc.values()):
        series = []
        for k, nom in enumerate(("G", "D")):
            if zc.get(nom):
                series.append(dict(nom=f"roue {nom}", couleur=S[k],
                                   couleur_sombre=SD[k], pts=zc[nom]))
        sections.append(("Zone morte", [rapport.graphe_nuage(
            "Deplacement obtenu a chaque palier de rapport cyclique",
            "en charge, au sol. Le balayage s'arrete au premier mouvement franc.",
            series, "rapport cyclique", "ticks parcourus", ident=True)]))

    # --- auto-reglage ------------------------------------------------------
    its = res.get("iterations") or []
    if its:
        blocs = []
        for nom in ("G", "D"):
            choisies = its if len(its) <= 4 else [its[0], its[len(its)//3],
                                                  its[2*len(its)//3], its[-1]]
            series = []
            for k, itr in enumerate(choisies):
                c = next((x for x in itr["courbes"] if x["roue"] == nom), None)
                if not c:
                    continue
                series.append(dict(nom=f"iter. {itr['n']}", couleur=Ra[k],
                                   couleur_sombre=RaD[k], etiquette=True,
                                   pts=list(zip(c["t"], c["v"]))))
            if series:
                blocs.append(rapport.graphe_lignes(
                    f"Reponse en boucle fermee — roue {nom}",
                    f"echelon de consigne a {its[0]['sp']} ticks/s ; "
                    "chaque courbe est une iteration de l'auto-reglage",
                    series, "temps (s)", "vitesse (ticks/s)",
                    reperes=[(its[0]["sp"], "consigne")], ident=True))
        if len(its) > 1:
            blocs.append(rapport.graphe_lignes(
                "Convergence — depassement",
                "critere vise : au plus 8 %",
                [dict(nom="depassement", couleur=S[0], couleur_sombre=SD[0],
                      pts=[(i["n"], i["dep"]) for i in its])],
                "iteration", "depassement (%)", h=210,
                reperes=[(8.0, "cible")]))
            blocs.append(rapport.graphe_lignes(
                "Convergence — temps d'etablissement",
                "duree pour entrer et rester dans la bande +/-5 %",
                [dict(nom="etablissement", couleur=S[1], couleur_sombre=SD[1],
                      pts=[(i["n"], i["etab"]) for i in its])],
                "iteration", "temps (s)", h=210))
        sections.append(("Auto-reglage sur le robot", blocs))

    if not sections:
        sections.append(("", ["<p class=vide>Aucune mesure exploitable.</p>"]))
    meta = (f"Mesure du {time.strftime('%d/%m/%Y a %H:%M')} — "
            f"robot au sol, {len(res.get('courbes_step') or [])} echelons "
            f"d'identification, {len(its)} iteration(s) d'auto-reglage.")
    with open(chemin, "w", encoding="utf-8") as f:
        f.write(rapport.page("Reglage du PID de vitesse", meta, sections))
    return chemin


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
      resultats["courbes_zm"] = {"G": [], "D": []}
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
              bouge = last[1 + i] - e0[1 + i]
              resultats["courbes_zm"][nom].append((d, bouge))
              if abs(bouge) > 3:   # marge sur le bruit
                  seuil = d; break
          zm[nom] = seuil
          log(f"    roue {nom} : premier mouvement a duty {seuil}")
      resultats["zone_morte"] = zm

      # ---- 1b. reponses indicielles ---------------------------------------
      log("\n=== 1b. REPONSES INDICIELLES (alternees, retour au point de depart) ===")
      modeles = {"G": [], "D": []}
      vss = {"G": [], "D": []}
      resultats["courbes_step"] = []
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
                  # courbe mesuree + modele ajuste, ramenes au meme filtre que
                  # celui du firmware pour une comparaison honnete
                  t0f = ech[0][0]
                  tt = [(e[0] - t0f) / 1000.0 for e in ech]
                  vv = [e[3 if nom == "G" else 4] for e in ech]
                  vm, f = [], 0.0
                  for t in tt:
                      dtm = t - m["theta"]
                      brut = (m["K"] * d * (1 - math.exp(-dtm / m["tau"]))
                              if dtm > 0 else 0.0)
                      f += ALPHA * (brut - f)
                      vm.append(round(f, 1))
                  resultats["courbes_step"].append(
                      dict(roue=nom, duty=d, t=[round(x, 3) for x in tt],
                           v=vv, vm=vm))
                  if m.get("borne") or m.get("qualite", 9) > 0.02:
                      log(f"    duty {d:+5} roue {nom} : ajustement rejete "
                          f"(qualite {m.get('qualite', 0):.3f}"
                          f"{', parametre en butee' if m.get('borne') else ''})")
                      continue
                  if m.get("colle", 0) > 0.15:
                      log(f"    duty {d:+5} roue {nom} : {1000*m['colle']:.0f} ms "
                          f"d'adherence au demarrage avant le depart")
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
          resultats.setdefault("statique", {})[nom] = sorted(vss[nom])
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
          dep = etab = ond = 0.0; det = []; courbes = []
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
                  # Ondulation en regime etabli. Un gain trop fort fait chasser
                  # la quantification de 50 ticks/s : la vitesse reste juste en
                  # moyenne mais la commande s'agite, et sur un vrai moteur cela
                  # s'entend et se paie en courant. Les trames ne portent pas le
                  # rapport cyclique, l'ondulation de vitesse en est le temoin.
                  # ECART INTERQUARTILE. L'ecart-type confond un decrochage
                  # isole (le robot accroche) avec une commande qui chasse la
                  # quantification ; l'ecart absolu median, lui, tombe a zero
                  # sur une oscillation carree, qui est bimodale. L'IQR traite
                  # correctement les deux : il ignore quelques points extremes
                  # et voit une oscillation soutenue.
                  q = sorted(v[-25:] if len(v) >= 25 else v)
                  iqr = q[(3*len(q))//4] - q[len(q)//4]
                  ond = max(ond, 100*abs(iqr)/abs(sp))
                  dep = max(dep, d); etab = max(etab, te)
                  det.append((sgn, nom, d, te, err))
                  if sgn > 0:
                      courbes.append(dict(roue=nom,
                                          t=[round(x, 3) for x in t], v=v))
          return dep, etab, det, courbes, ond

      DEP_CIBLE = 8.0
      OND_MAX   = 6.0        # ondulation toleree (ecart absolu median), %
      base_kp, base_ki = kp, ki

      # Recherche par ENCADREMENT sur un multiplicateur unique applique aux
      # deux gains. Comme kp/ki reste constant, seul le gain de boucle bouge :
      # le zero du correcteur ne se deplace pas. La version precedente montait
      # et descendait par facteurs fixes et oscillait sans converger — mesure
      # sur ce robot : 2,3 -> 7,1 -> 12,2 -> 6,5 -> 9,5 -> 14,7 -> 8,3 -> 11,9 %.
      # Un encadrement ne peut pas osciller : chaque essai retrecit l'intervalle.
      m = 1.0; bas = haut = None; meilleur = None
      for it in range(9):
          kp, ki = base_kp * m, base_ki * m
          r = essai(kp, ki)
          if r is None:
              log("    trop peu de trames — liaison instable"); break
          dep, etab, det, courbes, ond = r
          # Une ondulation tres elevee sur un robot au sol trahit presque
          # toujours un decrochage mecanique, pas une commande qui chasse :
          # on refait la mesure plutot que de laisser un accrochage orienter
          # la recherche. Mesure sur ce robot : 5 essais sur 8 perturbes,
          # avec des ondulations de 24 a 99 % la ou les essais propres
          # donnaient 1,5 et 1,9 %.
          for _ in range(2):
              if ond <= 3 * OND_MAX:
                  break
              log(f"        ondulation {ond:.0f} % — perturbation mecanique "
                  f"probable, on refait la mesure")
              r2 = essai(kp, ki)
              if r2 is None:
                  break
              if r2[4] < ond:
                  dep, etab, det, courbes, ond = r2
          etabli = etab < VERIF_HOLD - 0.15
          resultats.setdefault("iterations", []).append(
              dict(n=it + 1, kp=kp, ki=ki, dep=dep, etab=etab,
                   sp=VERIF_SP, ond=ond, courbes=courbes))
          log(f"    essai {it+1} : x{m:.2f}  kp={kp:.4f} ki={ki:.3f} -> "
              f"depassement {dep:+5.1f} %  ondulation {ond:4.1f} %  "
              + (f"etabli en {etab:.2f} s" if etabli else "PAS ETABLI"))
          for sgn, nom, d, te, err in det:
              log(f"        consigne {VERIF_SP*sgn:+5} roue {nom} : "
                  f"dep {d:+5.1f} %  etabli {te:4.2f} s  err stat {err:+5.1f} %")

          # Trois cas distincts, et surtout deux facons opposees d'echouer :
          # trop de depassement demande d'assouplir, ne pas atteindre la
          # consigne demande au contraire de durcir. Les confondre fait
          # descendre le gain alors que la boucle est deja trop molle.
          if dep > DEP_CIBLE:
              haut = m
              m = (bas + m) / 2 if bas is not None else m * 0.55
              log(f"        -> depassement excessif, on assouplit (x{m:.2f})")
          elif etabli and ond > OND_MAX:
              haut = m
              m = (bas + m) / 2 if bas is not None else m * 0.55
              log(f"        -> commande trop agitee, on assouplit (x{m:.2f})")
          elif not etabli:
              m = m * 1.7 if haut is None else (m + haut) / 2
              log(f"        -> consigne pas atteinte, on durcit (x{m:.2f})")
          else:
              bas = m
              # On garde le plus RAPIDE des reglages conformes, pas le plus
              # ferme : au-dela d'un certain gain la reponse se degrade sans
              # que le depassement franchisse encore le seuil.
              cand = (kp, ki, dep, etab, ond)
              if meilleur is None or etab < meilleur[3] - 1e-9:
                  meilleur = cand
              m = m * 1.7 if haut is None else (m + haut) / 2
              log(f"        -> conforme, on tente plus ferme (x{m:.2f})")

          if bas is not None and haut is not None and haut / bas < 1.15:
              log("        -> encadrement resserre, recherche terminee"); break

      if meilleur is None:
          log("    AUCUN reglage n'a tenu les criteres. Les gains restent au "
              "point de depart, a considerer comme non valides.")
          kp, ki = base_kp, base_ki
          dep = etab = float("nan")
      else:
          kp, ki, dep, etab, ond = meilleur
          env(f"k {kp:.5f} {ki:.5f} 0")
          log(f"\n    RETENU : kp={kp:.4f} ki={ki:.3f} -> depassement {dep:.1f} %, "
              f"etabli en {etab:.2f} s, ondulation {ond:.1f} %")
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
        # Le rapport est produit meme si le run a ete interrompu : une mesure
        # partielle vaut mieux que rien quand on est au milieu d'une piece.
        try:
            c = generer_rapport(resultats, RAPPORT)
            log(f"rapport graphique : {os.path.abspath(c)}")
        except Exception as e:
            log(f"rapport non genere : {e}")
        try:
            with open("/tmp/tune_pid_resultats.json", "w") as f:
                json.dump(resultats, f, indent=2)
        except Exception:
            pass
        try: s.close()
        except Exception: pass


if __name__ == "__main__":
    main()
