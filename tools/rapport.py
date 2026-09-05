"""
Rapport graphique de tune_pid.py — SVG pur, sans dependance.

Produit un fichier HTML autonome qui s'ouvre dans n'importe quel navigateur :
courbes de reponse indicielle avec le modele ajuste superpose, caracteristique
statique avec sa regression et la zone morte, convergence de l'auto-reglage.
Chaque graphique est double d'un tableau, et le survol donne les valeurs.
"""
import html, json, math

# --- palette (validee en clair et en sombre : bande de luminosite, plancher de
# --- chroma, separation daltonienne, contraste sur le fond) ------------------
SERIE   = ["#2a78d6", "#eb6834"]                     # roue G, roue D
SERIE_D = ["#3987e5", "#d95926"]
RAMPE   = ["#86b6ef", "#5598e7", "#2a78d6", "#1c5cab"]   # duty croissant
RAMPE_D = ["#184f95", "#256abf", "#3987e5", "#6da7ec"]

L, R, T, B = 64, 18, 16, 42      # marges du cadre de trace


def _ech(v, v0, v1, p0, p1):
    if v1 == v0:
        return (p0 + p1) / 2
    return p0 + (v - v0) * (p1 - p0) / (v1 - v0)


def _jolis_pas(lo, hi, n=5):
    """Graduations rondes couvrant [lo, hi]."""
    if hi <= lo:
        hi = lo + 1
    brut = (hi - lo) / n
    mag = 10 ** math.floor(math.log10(brut)) if brut > 0 else 1
    for m in (1, 2, 2.5, 5, 10):
        if brut / mag <= m:
            pas = m * mag; break
    else:
        pas = 10 * mag
    d = math.floor(lo / pas) * pas
    out = []
    while d <= hi + pas * 1e-9:
        out.append(round(d, 10)); d += pas
    return out


def _fmt(v):
    a = abs(v)
    if a >= 1000: return f"{v:,.0f}".replace(",", " ")
    if a >= 10:   return f"{v:.0f}"
    if a >= 1:    return f"{v:.1f}"
    if a == 0:    return "0"
    return f"{v:.2f}"


def cadre(w, h, xlo, xhi, ylo, yhi, xlab, ylab, xticks=None, yticks=None):
    """Grille, axes et etiquettes. Renvoie (svg, fonction x, fonction y)."""
    fx = lambda v: _ech(v, xlo, xhi, L, w - R)
    fy = lambda v: _ech(v, ylo, yhi, h - B, T)
    s = []
    for v in (yticks if yticks is not None else _jolis_pas(ylo, yhi)):
        if v < ylo - 1e-9 or v > yhi + 1e-9: continue
        y = fy(v)
        s.append(f'<line x1="{L}" y1="{y:.1f}" x2="{w-R}" y2="{y:.1f}" '
                 f'class="grille"/>')
        s.append(f'<text x="{L-8}" y="{y+4:.1f}" class="tick" '
                 f'text-anchor="end">{_fmt(v)}</text>')
    for v in (xticks if xticks is not None else _jolis_pas(xlo, xhi)):
        if v < xlo - 1e-9 or v > xhi + 1e-9: continue
        x = fx(v)
        s.append(f'<text x="{x:.1f}" y="{h-B+18}" class="tick" '
                 f'text-anchor="middle">{_fmt(v)}</text>')
    if ylo < 0 < yhi:
        s.append(f'<line x1="{L}" y1="{fy(0):.1f}" x2="{w-R}" y2="{fy(0):.1f}" '
                 f'class="axe"/>')
    s.append(f'<line x1="{L}" y1="{h-B}" x2="{w-R}" y2="{h-B}" class="axe"/>')
    s.append(f'<text x="{(L+w-R)/2:.0f}" y="{h-4}" class="axe-titre" '
             f'text-anchor="middle">{html.escape(xlab)}</text>')
    s.append(f'<text x="14" y="{(T+h-B)/2:.0f}" class="axe-titre" '
             f'text-anchor="middle" transform="rotate(-90 14 {(T+h-B)/2:.0f})">'
             f'{html.escape(ylab)}</text>')
    return "".join(s), fx, fy


def chemin(pts, fx, fy):
    if not pts: return ""
    return "M" + " L".join(f"{fx(x):.1f},{fy(y):.1f}" for x, y in pts)


def graphe_lignes(titre, sous, series, xlab, ylab, w=780, h=300,
                  reperes=(), ident=None):
    """series : liste de dicts {nom, pts:[(x,y)], couleur, couleur_sombre,
       tirets:bool, etiquette:bool}. reperes : [(y, texte)] lignes de reference."""
    xs = [p[0] for s in series for p in s["pts"]]
    ys = [p[1] for s in series for p in s["pts"]] + [r[0] for r in reperes]
    if not xs:
        return f'<figure class="carte"><figcaption>{html.escape(titre)}</figcaption>' \
               f'<p class="vide">aucune donnee</p></figure>'
    xlo, xhi = min(xs), max(xs)
    ylo, yhi = min(ys), max(ys)
    marge = (yhi - ylo) * 0.12 or 1
    ylo, yhi = ylo - marge, yhi + marge
    g, fx, fy = cadre(w, h, xlo, xhi, ylo, yhi, xlab, ylab)
    corps = [g]
    for y, txt in reperes:
        corps.append(f'<line x1="{L}" y1="{fy(y):.1f}" x2="{w-R}" y2="{fy(y):.1f}" '
                     f'class="repere"/>')
        corps.append(f'<text x="{w-R-4}" y="{fy(y)-6:.1f}" class="repere-txt" '
                     f'text-anchor="end">{html.escape(txt)}</text>')
    for i, s in enumerate(series):
        cls = "trait tirets" if s.get("tirets") else "trait"
        corps.append(f'<path d="{chemin(s["pts"], fx, fy)}" class="{cls}" '
                     f'style="stroke:var(--s{i})"/>')
        if s.get("etiquette") and s["pts"]:
            x, y = s["pts"][-1]
            corps.append(f'<text x="{fx(x)-4:.1f}" y="{fy(y)-7:.1f}" '
                         f'class="etiq" text-anchor="end" '
                         f'style="fill:var(--s{i})">{html.escape(s["nom"])}</text>')
    style = ";".join(f"--s{i}:{s['couleur']}" for i, s in enumerate(series))
    styleD = ";".join(f"--s{i}:{s.get('couleur_sombre', s['couleur'])}"
                      for i, s in enumerate(series))
    leg = "".join(
        f'<span class="cle"><i style="background:var(--s{i})"></i>'
        f'{html.escape(s["nom"])}</span>'
        for i, s in enumerate(series) if not s.get("cache_legende"))
    dj = json.dumps({"series": [{"nom": s["nom"], "pts": s["pts"]}
                                for s in series if not s.get("cache_legende")],
                     "xlab": xlab, "ylab": ylab,
                     "box": [L, T, w - R, h - B],
                     "dom": [xlo, xhi, ylo, yhi]})
    return (f'<figure class="carte" style="{style}" data-sombre="{styleD}">'
            f'<figcaption>{html.escape(titre)}'
            f'<small>{html.escape(sous)}</small></figcaption>'
            f'<div class="legende">{leg}</div>'
            f'<div class="zone">'
            f'<svg viewBox="0 0 {w} {h}" class="viz" '
            f'data-graphe=\'{html.escape(dj, quote=True)}\'>{"".join(corps)}'
            f'<line class="croix" x1="0" y1="{T}" x2="0" y2="{h-B}" '
            f'style="display:none"/></svg>'
            f'<div class="bulle" hidden></div></div>'
            + (tableau(series, xlab, ident) if ident else "") + '</figure>')


def graphe_nuage(titre, sous, series, xlab, ylab, w=780, h=320, ident=None):
    """series : {nom, pts, couleur, couleur_sombre, droite:[(x,y),(x,y)]}"""
    xs = [p[0] for s in series for p in s["pts"]]
    ys = [p[1] for s in series for p in s["pts"]]
    for s in series:
        for p in s.get("droite", []):
            xs.append(p[0]); ys.append(p[1])
    if not xs:
        return ""
    xlo, xhi = min(xs + [0]), max(xs)
    ylo, yhi = min(ys + [0]), max(ys)
    marge = (yhi - ylo) * 0.10 or 1
    yhi += marge
    g, fx, fy = cadre(w, h, xlo, xhi * 1.02, ylo, yhi, xlab, ylab)
    corps = [g]
    for i, s in enumerate(series):
        if s.get("droite"):
            corps.append(f'<path d="{chemin(s["droite"], fx, fy)}" '
                         f'class="trait tirets" style="stroke:var(--s{i})"/>')
        for x, y in s["pts"]:
            corps.append(f'<circle cx="{fx(x):.1f}" cy="{fy(y):.1f}" r="5" '
                         f'class="point" style="fill:var(--s{i})">'
                         f'<title>{html.escape(s["nom"])} — {xlab} {_fmt(x)}, '
                         f'{ylab} {_fmt(y)}</title></circle>')
        if s.get("zm") is not None:
            corps.append(f'<circle cx="{fx(s["zm"]):.1f}" cy="{fy(0):.1f}" r="4" '
                         f'class="zm" style="stroke:var(--s{i})"/>')
    style = ";".join(f"--s{i}:{s['couleur']}" for i, s in enumerate(series))
    styleD = ";".join(f"--s{i}:{s.get('couleur_sombre', s['couleur'])}"
                      for i, s in enumerate(series))
    leg = "".join(f'<span class="cle"><i style="background:var(--s{i})"></i>'
                  f'{html.escape(s["nom"])}</span>' for i, s in enumerate(series))
    return (f'<figure class="carte" style="{style}" data-sombre="{styleD}">'
            f'<figcaption>{html.escape(titre)}<small>{html.escape(sous)}</small>'
            f'</figcaption><div class="legende">{leg}</div>'
            f'<svg viewBox="0 0 {w} {h}" class="viz">{"".join(corps)}</svg>'
            + (tableau(series, xlab, ident) if ident else "") + '</figure>')


def tableau(series, xlab, ident):
    """Vue tabulaire : l'identite ne doit jamais reposer sur la seule couleur."""
    lignes = []
    for s in series:
        for x, y in s["pts"]:
            lignes.append((s["nom"], x, y))
    if len(lignes) > 400:
        lignes = lignes[::max(1, len(lignes)//400)]
    corps = "".join(f"<tr><td>{html.escape(n)}</td><td>{_fmt(x)}</td>"
                    f"<td>{_fmt(y)}</td></tr>" for n, x, y in lignes)
    return (f'<details class="tab"><summary>Tableau des valeurs</summary>'
            f'<table><thead><tr><th>serie</th><th>{html.escape(xlab)}</th>'
            f'<th>valeur</th></tr></thead><tbody>{corps}</tbody></table></details>')


CSS = """
:root{color-scheme:light;
 --plan:#f9f9f7; --fond:#fcfcfb; --encre:#0b0b0b; --encre2:#52514e;
 --muet:#898781; --grille:#e1e0d9; --axe:#c3c2b7; --bord:rgba(11,11,11,.10);}
@media (prefers-color-scheme:dark){:root:not([data-theme="light"]){
 color-scheme:dark;
 --plan:#0d0d0d; --fond:#1a1a19; --encre:#fff; --encre2:#c3c2b7;
 --muet:#898781; --grille:#2c2c2a; --axe:#383835; --bord:rgba(255,255,255,.10);}}
:root[data-theme="dark"]{color-scheme:dark;
 --plan:#0d0d0d; --fond:#1a1a19; --encre:#fff; --encre2:#c3c2b7;
 --muet:#898781; --grille:#2c2c2a; --axe:#383835; --bord:rgba(255,255,255,.10);}
*{box-sizing:border-box}
body{margin:0;padding:28px 20px 60px;background:var(--plan);color:var(--encre);
 font:14px/1.55 system-ui,-apple-system,"Segoe UI",sans-serif;}
.enveloppe{max-width:860px;margin:0 auto}
h1{font-size:21px;margin:0 0 4px;letter-spacing:-.01em}
.meta{color:var(--encre2);margin:0 0 26px;font-size:13px}
h2{font-size:15px;margin:34px 0 12px;color:var(--encre);
 border-top:1px solid var(--bord);padding-top:18px}
.carte{background:var(--fond);border:1px solid var(--bord);border-radius:10px;
 margin:0 0 18px;padding:14px 14px 6px}
figcaption{font-weight:600;font-size:14px;margin-bottom:2px}
figcaption small{display:block;font-weight:400;color:var(--encre2);font-size:12.5px;
 margin-top:2px}
.legende{display:flex;flex-wrap:wrap;gap:14px;margin:10px 0 2px;font-size:12.5px;
 color:var(--encre2)}
.cle{display:inline-flex;align-items:center;gap:6px}
.cle i{width:11px;height:11px;border-radius:3px;display:inline-block}
.zone{position:relative}
.viz{width:100%;height:auto;display:block;overflow:visible}
.grille{stroke:var(--grille);stroke-width:1}
.axe{stroke:var(--axe);stroke-width:1}
.repere{stroke:var(--muet);stroke-width:1.5;stroke-dasharray:2 4}
.repere-txt{fill:var(--muet);font-size:11px}
.tick{fill:var(--muet);font-size:11px;font-variant-numeric:tabular-nums}
.axe-titre{fill:var(--encre2);font-size:12px}
.trait{fill:none;stroke-width:2;stroke-linejoin:round;stroke-linecap:round}
.tirets{stroke-dasharray:5 4;stroke-width:1.6;opacity:.85}
.etiq{font-size:11.5px;font-weight:600}
.point{stroke:var(--fond);stroke-width:2}
.zm{fill:none;stroke-width:2}
.croix{stroke:var(--muet);stroke-width:1;pointer-events:none}
.bulle{position:absolute;pointer-events:none;background:var(--fond);
 border:1px solid var(--bord);border-radius:7px;padding:7px 9px;font-size:12px;
 box-shadow:0 4px 14px rgba(0,0,0,.13);white-space:nowrap;z-index:5}
.bulle b{font-weight:600}
.bulle span{color:var(--encre2)}
.tab{margin:8px 0 4px;font-size:12.5px}
.tab summary{cursor:pointer;color:var(--encre2);padding:4px 0}
.tab table{border-collapse:collapse;width:100%;margin-top:6px;
 font-variant-numeric:tabular-nums}
.tab th,.tab td{text-align:left;padding:3px 8px;border-bottom:1px solid var(--bord)}
.tab th{color:var(--muet);font-weight:500}
.tab tbody{display:block;max-height:260px;overflow-y:auto}
.tab thead,.tab tbody tr{display:table;width:100%;table-layout:fixed}
.res{background:var(--fond);border:1px solid var(--bord);border-radius:10px;
 padding:14px 16px;margin-bottom:18px}
.res dl{display:grid;grid-template-columns:auto 1fr;gap:4px 18px;margin:0}
.res dt{color:var(--encre2)}
.res dd{margin:0;font-variant-numeric:tabular-nums;font-weight:600}
pre{background:var(--fond);border:1px solid var(--bord);border-radius:8px;
 padding:12px 14px;overflow-x:auto;font-size:12.5px;margin:0}
.vide{color:var(--muet);padding:20px 0}
"""

JS = """
document.querySelectorAll('.carte[data-sombre]').forEach(function(c){
  var d=c.getAttribute('data-sombre');
  function maj(){var noir=matchMedia('(prefers-color-scheme:dark)').matches
      && document.documentElement.dataset.theme!=='light'
      || document.documentElement.dataset.theme==='dark';
    c.setAttribute('style', noir ? d : c.dataset.clair);}
  c.dataset.clair=c.getAttribute('style');
  matchMedia('(prefers-color-scheme:dark)').addEventListener('change',maj); maj();
});
document.querySelectorAll('svg[data-graphe]').forEach(function(svg){
  var g=JSON.parse(svg.getAttribute('data-graphe'));
  var zone=svg.parentNode, bulle=zone.querySelector('.bulle');
  var croix=svg.querySelector('.croix');
  var x0=g.box[0],y0=g.box[1],x1=g.box[2],y1=g.box[3];
  var dx0=g.dom[0],dx1=g.dom[1];
  function px2val(px){return dx0+(px-x0)*(dx1-dx0)/(x1-x0);}
  svg.addEventListener('mousemove',function(e){
    var r=svg.getBoundingClientRect(), vb=svg.viewBox.baseVal;
    var px=(e.clientX-r.left)*vb.width/r.width;
    if(px<x0||px>x1){bulle.hidden=true;croix.style.display='none';return;}
    var v=px2val(px), lignes=[];
    g.series.forEach(function(s){
      if(!s.pts.length)return;
      var best=s.pts[0],bd=Math.abs(s.pts[0][0]-v);
      s.pts.forEach(function(p){var d=Math.abs(p[0]-v); if(d<bd){bd=d;best=p;}});
      lignes.push('<span>'+s.nom+'</span> <b>'+Math.round(best[1]*100)/100+'</b>');
    });
    croix.setAttribute('x1',px);croix.setAttribute('x2',px);
    croix.style.display='';
    bulle.innerHTML='<span>'+g.xlab+'</span> <b>'+(Math.round(v*1000)/1000)+
      '</b><br>'+lignes.join('<br>');
    bulle.hidden=false;
    var lx=(e.clientX-r.left)+14, ly=(e.clientY-r.top)-10;
    if(lx+bulle.offsetWidth>r.width) lx=(e.clientX-r.left)-bulle.offsetWidth-14;
    bulle.style.left=lx+'px'; bulle.style.top=ly+'px';
  });
  svg.addEventListener('mouseleave',function(){
    bulle.hidden=true; croix.style.display='none';});
});
"""


def page(titre, meta, sections):
    """sections : liste de (titre_section, [morceaux html])."""
    corps = []
    for t, blocs in sections:
        if t:
            corps.append(f"<h2>{html.escape(t)}</h2>")
        corps.extend(blocs)
    return (f"<!doctype html><html lang=fr><head><meta charset=utf-8>"
            f"<meta name=viewport content='width=device-width,initial-scale=1'>"
            f"<title>{html.escape(titre)}</title><style>{CSS}</style></head>"
            f"<body><div class=enveloppe><h1>{html.escape(titre)}</h1>"
            f"<p class=meta>{meta}</p>{''.join(corps)}</div>"
            f"<script>{JS}</script></body></html>")
