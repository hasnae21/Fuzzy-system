import nbformat as nbf

nb = nbf.v4.new_notebook()
nb.metadata = {
    "kernelspec": {"display_name": "Python 3", "language": "python", "name": "python3"},
    "language_info": {"name": "python", "version": "3.10.0"},
    "colab": {"provenance": []},
}

cells = []

def md(src):
    cells.append(nbf.v4.new_markdown_cell(src))

def code(src):
    cells.append(nbf.v4.new_code_cell(src))

# ─────────────────────────────────────────────────────────────────────────────
# CELL 0 — Title
# ─────────────────────────────────────────────────────────────────────────────
md("""# 🏎 Line Follower Robot — Fuzzy Logic + Transformer vs PID
## Simulation complète Python / Google Colab

**Objectif :** Démonstration visuelle que le contrôleur **Fuzzy Logic + Transformer** est supérieur au **PID classique** pour un robot suiveur de ligne sur autodrome.

### Architecture du système :
| Composant | Description |
|-----------|-------------|
| **Fuzzy Logic (Mamdani)** | 21 règles, fonctions trapézoïdales/triangulaires, COG |
| **Transformer simplifié** | Mémoire 16 états, correction par tendance linéaire |
| **PID classique** | Kp/Ki/Kd fixes, vitesse constante (référence) |
| **Variables floues clés** | `vitesse_virage` + `vitesse_droite` (adaptation à la courbure) |

---
> **Pour exécuter :** `Runtime > Run all` ou `Ctrl+F9`
""")

# ─────────────────────────────────────────────────────────────────────────────
# CELL 1 — Install
# ─────────────────────────────────────────────────────────────────────────────
code("""# ══════════════════════════════════════════════════════
# CELLULE 1 — Installation des dépendances
# ══════════════════════════════════════════════════════
import subprocess, sys

def install(pkg):
    subprocess.check_call([sys.executable, '-m', 'pip', 'install', pkg, '-q'])

try:
    import matplotlib
    import numpy
    print("✅ matplotlib et numpy déjà installés")
except ImportError:
    print("Installation en cours...")
    install('matplotlib')
    install('numpy')
    print("✅ Installation terminée")

print("✅ Toutes les dépendances sont disponibles")
""")

# ─────────────────────────────────────────────────────────────────────────────
# CELL 2 — Imports
# ─────────────────────────────────────────────────────────────────────────────
code("""# ══════════════════════════════════════════════════════
# CELLULE 2 — Imports
# ══════════════════════════════════════════════════════
import math
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.animation as animation
from matplotlib.patches import FancyArrow, Circle, Rectangle, FancyBboxPatch
from matplotlib.gridspec import GridSpec
from matplotlib.colors import LinearSegmentedColormap
from dataclasses import dataclass, field
from typing import List, Dict, Tuple
import os
import warnings
warnings.filterwarnings('ignore')

# Créer le dossier img pour sauvegarder les graphiques
os.makedirs('img', exist_ok=True)

# Style sombre pour tous les graphiques
plt.rcParams.update({
    'figure.facecolor': '#0a0c11',
    'axes.facecolor':   '#12151c',
    'axes.edgecolor':   '#252a3a',
    'text.color':       '#dde2f0',
    'axes.labelcolor':  '#7a82a0',
    'xtick.color':      '#454d68',
    'ytick.color':      '#454d68',
    'grid.color':       '#1e2230',
    'grid.alpha':       0.6,
    'font.family':      'monospace',
})

print("✅ Imports et configuration terminés")
print(f"📁 Dossier img/ créé : {os.path.abspath('img')}")
""")

# ─────────────────────────────────────────────────────────────────────────────
# CELL 3 — Membership functions
# ─────────────────────────────────────────────────────────────────────────────
md("""---
## 📐 Partie 1 — Fonctions d'appartenance (Membership Functions)
Les fonctions d'appartenance définissent comment une valeur numérique est convertie en degré d'appartenance flou.  
Nous utilisons des **fonctions trapézoïdales** et **triangulaires**.
""")

code("""# ══════════════════════════════════════════════════════
# CELLULE 3 — Fonctions d'appartenance
# ══════════════════════════════════════════════════════

def trapeze(x: float, a: float, b: float, c: float, d: float) -> float:
    \"\"\"
    Fonction d'appartenance TRAPÉZOÏDALE.
    Retourne 1.0 sur le plateau [b,c], monte de a→b, descend de c→d.

         1.0  ┌──────────┐
              /            \\
         0.0─/──────────────\\─
             a   b        c   d
    \"\"\"
    if x <= a or x >= d: return 0.0
    if b <= x <= c:      return 1.0
    if x < b: return (x - a) / (b - a) if b != a else 1.0
    return (d - x) / (d - c) if d != c else 1.0

def triangle(x: float, a: float, b: float, c: float) -> float:
    \"\"\"
    Fonction d'appartenance TRIANGULAIRE.
    Sommet à b, retourne 0 hors de [a,c].

         1.0       /\\
                  /  \\
         0.0 ────/    \\────
                 a  b  c
    \"\"\"
    if x <= a or x >= c: return 0.0
    if x <= b: return (x - a) / (b - a) if b != a else 1.0
    return (c - x) / (c - b) if c != b else 1.0


# ──────────────────────────────────────────────────────
# ENSEMBLES FLOUS — ERREUR LATÉRALE ∈ [-3, +3]
# ──────────────────────────────────────────────────────
class FuzzySets:
    # Entrée 1 : Erreur latérale (7 ensembles)
    @staticmethod
    def error_NL(e): return trapeze(e, -3.0, -3.0, -2.0, -1.0)   # Négatif Large
    @staticmethod
    def error_NM(e): return triangle(e, -2.5, -1.5, -0.5)         # Négatif Moyen
    @staticmethod
    def error_NS(e): return triangle(e, -1.2, -0.5,  0.0)         # Négatif Small
    @staticmethod
    def error_ZE(e): return triangle(e, -0.6,  0.0,  0.6)         # Zéro
    @staticmethod
    def error_PS(e): return triangle(e,  0.0,  0.5,  1.2)         # Positif Small
    @staticmethod
    def error_PM(e): return triangle(e,  0.5,  1.5,  2.5)         # Positif Moyen
    @staticmethod
    def error_PL(e): return trapeze(e,  1.0,  2.0,  3.0,  3.0)   # Positif Large

    # Entrée 2 : Dérivée de l'erreur (5 ensembles)
    @staticmethod
    def rate_NL(r): return trapeze(r, -2.0, -2.0, -1.2, -0.5)
    @staticmethod
    def rate_NS(r): return triangle(r, -1.5, -0.6,  0.0)
    @staticmethod
    def rate_ZE(r): return triangle(r, -0.5,  0.0,  0.5)
    @staticmethod
    def rate_PS(r): return triangle(r,  0.0,  0.6,  1.5)
    @staticmethod
    def rate_PL(r): return trapeze(r,  0.5,  1.2,  2.0,  2.0)

    # Entrée 3 : Courbure (3 ensembles)
    @staticmethod
    def curv_STRAIGHT(c): return trapeze(c, 0.0, 0.0, 0.15, 0.35)
    @staticmethod
    def curv_MILD(c):     return triangle(c, 0.2, 0.45, 0.70)
    @staticmethod
    def curv_SHARP(c):    return trapeze(c, 0.55, 0.75, 1.0, 1.0)

    # Singletons de sortie
    STEER_CENTERS         = {'HL':-1.0,'ML':-0.65,'SL':-0.3,'ZE':0.0,'SR':0.3,'MR':0.65,'HR':1.0}
    CORNER_SPEED_CENTERS  = {'STOP':0.05,'SLOW':0.25,'MEDIUM_SLOW':0.45,'MEDIUM':0.65,'FAST':0.85}
    STRAIGHT_SPEED_CENTERS= {'SLOW':0.30,'MEDIUM':0.55,'FAST':0.75,'VERY_FAST':0.90,'TURBO':1.00}


fs = FuzzySets()
print("✅ Fonctions d'appartenance définies")
print(f"   Erreur   : 7 ensembles (NL, NM, NS, ZE, PS, PM, PL)")
print(f"   Dérivée  : 5 ensembles (NL, NS, ZE, PS, PL)")
print(f"   Courbure : 3 ensembles (STRAIGHT, MILD, SHARP)")
print(f"   Braquage : 7 singletons | Vit.virage : 5 | Vit.droite : 5")
""")

# ─────────────────────────────────────────────────────────────────────────────
# CELL 4 — Plot membership functions
# ─────────────────────────────────────────────────────────────────────────────
md("""### Visualisation des fonctions d'appartenance""")

code("""# ══════════════════════════════════════════════════════
# CELLULE 4 — Tracé des fonctions d'appartenance
# ══════════════════════════════════════════════════════

COLORS = {
    'NL':'#ff6b6b','NM':'#ffa07a','NS':'#ffd166',
    'ZE':'#00e5b0','PS':'#4cc9f0','PM':'#9b72cf','PL':'#c77dff',
    'NL2':'#ff6b6b','NS2':'#ffd166','ZE2':'#00e5b0','PS2':'#4cc9f0','PL2':'#9b72cf',
    'STRAIGHT':'#4cc9f0','MILD':'#ffd166','SHARP':'#ff6b6b',
}

fig, axes = plt.subplots(1, 3, figsize=(18, 5))
fig.patch.set_facecolor('#0a0c11')
fig.suptitle('Fonctions d\'appartenance — Variables floues d\'entrée',
             color='#dde2f0', fontsize=14, fontweight='bold', y=1.02)

# ── Erreur latérale ──────────────────────────────────
ax = axes[0]
ax.set_facecolor('#12151c')
xe = np.linspace(-3, 3, 300)
sets_e = {
    'NL': [fs.error_NL(v) for v in xe],
    'NM': [fs.error_NM(v) for v in xe],
    'NS': [fs.error_NS(v) for v in xe],
    'ZE': [fs.error_ZE(v) for v in xe],
    'PS': [fs.error_PS(v) for v in xe],
    'PM': [fs.error_PM(v) for v in xe],
    'PL': [fs.error_PL(v) for v in xe],
}
for name, vals in sets_e.items():
    ax.plot(xe, vals, color=COLORS[name], linewidth=2.5, label=name)
    # Fill under curve
    ax.fill_between(xe, vals, alpha=0.08, color=COLORS[name])
ax.set_title('Erreur latérale  ∈ [-3, +3]', color='#00e5b0', fontweight='bold')
ax.set_xlabel('Erreur (normalisée)', color='#7a82a0')
ax.set_ylabel('Degré d\'appartenance µ', color='#7a82a0')
ax.legend(loc='upper right', fontsize=8, framealpha=0.3)
ax.set_xlim(-3.2, 3.2); ax.set_ylim(-0.05, 1.1)
ax.grid(True, alpha=0.3)
ax.axvline(0, color='#ffffff', alpha=0.1, linewidth=0.5)
# Label positions
for name, center in [('NL',-2.5),('NM',-1.5),('NS',-0.5),
                     ('ZE',0),('PS',0.5),('PM',1.5),('PL',2.5)]:
    ax.text(center, 1.05, name, ha='center', fontsize=7,
            color=COLORS[name], fontweight='bold')

# ── Dérivée de l'erreur ──────────────────────────────
ax = axes[1]
ax.set_facecolor('#12151c')
xr = np.linspace(-2, 2, 300)
sets_r = {
    'NL': [fs.rate_NL(v) for v in xr],
    'NS': [fs.rate_NS(v) for v in xr],
    'ZE': [fs.rate_ZE(v) for v in xr],
    'PS': [fs.rate_PS(v) for v in xr],
    'PL': [fs.rate_PL(v) for v in xr],
}
ck = {'NL':'#ff6b6b','NS':'#ffd166','ZE':'#00e5b0','PS':'#4cc9f0','PL':'#9b72cf'}
for name, vals in sets_r.items():
    ax.plot(xr, vals, color=ck[name], linewidth=2.5, label=name)
    ax.fill_between(xr, vals, alpha=0.08, color=ck[name])
ax.set_title('Dérivée de l\'erreur  ∈ [-2, +2]', color='#ffd166', fontweight='bold')
ax.set_xlabel('Taux de variation', color='#7a82a0')
ax.legend(loc='upper right', fontsize=8, framealpha=0.3)
ax.set_xlim(-2.2, 2.2); ax.set_ylim(-0.05, 1.1)
ax.grid(True, alpha=0.3)

# ── Courbure ──────────────────────────────────────────
ax = axes[2]
ax.set_facecolor('#12151c')
xc = np.linspace(0, 1, 300)
sets_c = {
    'STRAIGHT': [fs.curv_STRAIGHT(v) for v in xc],
    'MILD':     [fs.curv_MILD(v) for v in xc],
    'SHARP':    [fs.curv_SHARP(v) for v in xc],
}
ck2 = {'STRAIGHT':'#4cc9f0','MILD':'#ffd166','SHARP':'#ff6b6b'}
for name, vals in sets_c.items():
    ax.plot(xc, vals, color=ck2[name], linewidth=2.5, label=name)
    ax.fill_between(xc, vals, alpha=0.12, color=ck2[name])
ax.set_title('Courbure de la piste  ∈ [0, 1]', color='#ff6b6b', fontweight='bold')
ax.set_xlabel('Courbure locale', color='#7a82a0')
ax.legend(loc='upper center', fontsize=8, framealpha=0.3)
ax.set_xlim(-0.05, 1.05); ax.set_ylim(-0.05, 1.1)
ax.grid(True, alpha=0.3)
# Zone labels
ax.text(0.1,  0.5, 'DROITE', ha='center', color='#4cc9f0', fontsize=8, alpha=0.7)
ax.text(0.45, 0.5, 'VIRAGE\\nDOUX', ha='center', color='#ffd166', fontsize=8, alpha=0.7)
ax.text(0.85, 0.5, 'VIRAGE\\nSERRÉ', ha='center', color='#ff6b6b', fontsize=8, alpha=0.7)

plt.tight_layout()
plt.savefig('img/01_membership_functions.png', dpi=150, bbox_inches='tight',
            facecolor='#0a0c11')
plt.show()
print("💾 Sauvegardé : img/01_membership_functions.png")
""")

# ─────────────────────────────────────────────────────────────────────────────
# CELL 5 — Fuzzy Rules
# ─────────────────────────────────────────────────────────────────────────────
md("""---
## 📋 Partie 2 — Base de Règles Mamdani (21 règles)
""")

code("""# ══════════════════════════════════════════════════════
# CELLULE 5 — Base de règles Mamdani
# ══════════════════════════════════════════════════════

FUZZY_RULES = [
    # GROUPE 1 — Correction proportionnelle à l'erreur (R1–R7)
    ("NL","ANY","ANY", "HR","STOP","SLOW"),
    ("NM","ANY","ANY", "MR","SLOW","MEDIUM"),
    ("NS","ANY","ANY", "SR","MEDIUM_SLOW","FAST"),
    ("ZE","ANY","ANY", "ZE","FAST","TURBO"),
    ("PS","ANY","ANY", "SL","MEDIUM_SLOW","FAST"),
    ("PM","ANY","ANY", "ML","SLOW","MEDIUM"),
    ("PL","ANY","ANY", "HL","STOP","SLOW"),
    # GROUPE 2 — Anticipation par dérivée (R8–R11)
    ("NS","PL","ANY",  "ZE","MEDIUM","VERY_FAST"),
    ("PS","NL","ANY",  "ZE","MEDIUM","VERY_FAST"),
    ("ZE","PL","ANY",  "SL","MEDIUM_SLOW","FAST"),
    ("ZE","NL","ANY",  "SR","MEDIUM_SLOW","FAST"),
    # GROUPE 3 — Virages serrés SHARP (R12–R16)
    ("ZE","ANY","SHARP","ZE","SLOW","MEDIUM_SLOW"),
    ("PM","ANY","SHARP","HL","STOP","SLOW"),
    ("NM","ANY","SHARP","HR","STOP","SLOW"),
    ("PS","ANY","SHARP","ML","SLOW","SLOW"),
    ("NS","ANY","SHARP","MR","SLOW","SLOW"),
    # GROUPE 4 — Lignes droites STRAIGHT (R17–R19)
    ("ZE","ZE","STRAIGHT","ZE","FAST","TURBO"),
    ("PS","ZE","STRAIGHT","SL","VERY_FAST","VERY_FAST"),
    ("NS","ZE","STRAIGHT","SR","VERY_FAST","VERY_FAST"),
    # GROUPE 5 — Sécurité / état critique (R20–R21)
    ("NL","NL","ANY",  "HR","STOP","SLOW"),
    ("PL","PL","ANY",  "HL","STOP","SLOW"),
]

GROUPS = {
    'G1 — Correction erreur (R1–R7)':   (0,  7,  '#00e5b0'),
    'G2 — Anticipation dérivée (R8–R11)':(7,  11, '#4cc9f0'),
    'G3 — Virages serrés (R12–R16)':     (11, 16, '#ffd166'),
    'G4 — Lignes droites (R17–R19)':     (16, 19, '#9b72cf'),
    'G5 — Sécurité (R20–R21)':           (19, 21, '#ff6b6b'),
}

# ── Affichage texte ──────────────────────────────────
print("╔══════════════════════════════════════════════════════════════════════════╗")
print("║  BASE DE RÈGLES FUZZY MAMDANI — 21 règles                               ║")
print("╚══════════════════════════════════════════════════════════════════════════╝")
print(f"{'N°':>3}  {'Erreur':>6}  {'Taux':>8}  {'Courb':>8}  →  {'Braquage':>10}  {'V.Virage':>12}  {'V.Droite':>12}")
print("─"*74)
group_names = list(GROUPS.keys())
gidx = 0
for i, rule in enumerate(FUZZY_RULES):
    e_set, r_set, c_set, steer, cs, ss = rule
    # Group header
    for gname, (gs, ge, gc) in GROUPS.items():
        if i == gs:
            print(f"\\n  ▶ {gname}")
            print("  " + "─"*70)
    print(f"  R{i+1:02d}  {e_set:>6}  {r_set:>8}  {c_set:>8}  →  {steer:>10}  {cs:>12}  {ss:>12}")

print(f"\\n✅ {len(FUZZY_RULES)} règles chargées")
""")

# ─────────────────────────────────────────────────────────────────────────────
# CELL 6 — Rules visualization
# ─────────────────────────────────────────────────────────────────────────────
code("""# ══════════════════════════════════════════════════════
# CELLULE 6 — Visualisation des règles (heatmap)
# ══════════════════════════════════════════════════════

fig, axes = plt.subplots(1, 2, figsize=(16, 6))
fig.patch.set_facecolor('#0a0c11')
fig.suptitle('Visualisation de la Base de Règles Mamdani',
             color='#dde2f0', fontsize=13, fontweight='bold')

# ── Heatmap erreur → braquage ──────────────────────
ax = axes[0]
ax.set_facecolor('#12151c')
e_vals = np.linspace(-3, 3, 200)
steer_vals = []
for e in e_vals:
    num, den = 0.0, 0.0
    for rule in FUZZY_RULES:
        e_s, r_s, c_s, steer_o, cs_o, ss_o = rule
        me = {'NL':fs.error_NL(e),'NM':fs.error_NM(e),'NS':fs.error_NS(e),
              'ZE':fs.error_ZE(e),'PS':fs.error_PS(e),'PM':fs.error_PM(e),
              'PL':fs.error_PL(e)}.get(e_s, 1.0) if e_s!='ANY' else 1.0
        alpha = me
        if alpha < 0.001: continue
        if steer_o not in fs.STEER_CENTERS: continue
        num += alpha * fs.STEER_CENTERS[steer_o]
        den += alpha
    steer_vals.append(num/den if den>0 else 0.0)

ax.plot(e_vals, steer_vals, color='#00e5b0', linewidth=2.5, label='Braquage Fuzzy')
ax.plot(e_vals, [-1.8*e/3 for e in e_vals], color='#ff4560', linewidth=1.5,
        linestyle='--', label='PID (proportionnel)')
ax.fill_between(e_vals, steer_vals, alpha=0.12, color='#00e5b0')
ax.axhline(0, color='#ffffff', alpha=0.2, linewidth=0.5)
ax.axvline(0, color='#ffffff', alpha=0.2, linewidth=0.5)
ax.set_title('Surface de contrôle : Erreur → Braquage', color='#00e5b0', fontweight='bold')
ax.set_xlabel('Erreur latérale'); ax.set_ylabel('Commande braquage')
ax.legend(fontsize=9, framealpha=0.3); ax.grid(True, alpha=0.3)
ax.set_xlim(-3.2, 3.2); ax.set_ylim(-1.1, 1.1)

# Zone annotations
for x, label, col in [(-2.5,'Tourner\\nfort droite','#ffd166'),
                        (0,'Tout\\ndroit','#00e5b0'),
                        (2.5,'Tourner\\nfort gauche','#ffd166')]:
    ax.annotate(label, xy=(x, steer_vals[int((x+3)/6*199)]),
                fontsize=7, color=col, ha='center',
                bbox=dict(boxstyle='round,pad=0.2', facecolor='#12151c', alpha=0.7))

# ── Vitesse selon courbure ──────────────────────────
ax = axes[1]
ax.set_facecolor('#12151c')
c_vals = np.linspace(0, 1, 200)
# Pour erreur=0 (robot centré) → montrer comment la vitesse varie avec la courbure
spd_fuzzy = []
spd_pid   = []
for c in c_vals:
    # Inférence simple pour erreur=0
    num_cs, den_cs = 0.0, 0.0
    num_ss, den_ss = 0.0, 0.0
    for rule in FUZZY_RULES:
        e_s, r_s, c_s, steer_o, cs_o, ss_o = rule
        me = fs.error_ZE(0.0) if e_s=='ZE' else (1.0 if e_s=='ANY' else 0.0)
        mc = {'STRAIGHT':fs.curv_STRAIGHT(c),'MILD':fs.curv_MILD(c),
              'SHARP':fs.curv_SHARP(c)}.get(c_s, 1.0) if c_s!='ANY' else 1.0
        alpha = min(me, mc)
        if alpha < 0.001: continue
        if cs_o in fs.CORNER_SPEED_CENTERS:
            num_cs += alpha * fs.CORNER_SPEED_CENTERS[cs_o]; den_cs += alpha
        if ss_o in fs.STRAIGHT_SPEED_CENTERS:
            num_ss += alpha * fs.STRAIGHT_SPEED_CENTERS[ss_o]; den_ss += alpha
    cs_out = num_cs/den_cs if den_cs>0 else 0.5
    ss_out = num_ss/den_ss if den_ss>0 else 0.5
    eff_spd = cs_out * c + ss_out * (1-c)
    spd_fuzzy.append(eff_spd)
    spd_pid.append(0.60)  # PID vitesse constante

ax.plot(c_vals, spd_fuzzy, color='#00e5b0', linewidth=2.5, label='Fuzzy (adaptative)')
ax.fill_between(c_vals, spd_fuzzy, alpha=0.12, color='#00e5b0')
ax.plot(c_vals, spd_pid, color='#ff4560', linewidth=2, linestyle='--', label='PID (constante 0.60)')
ax.fill_between(c_vals, spd_fuzzy, spd_pid,
                where=[f>p for f,p in zip(spd_fuzzy, spd_pid)],
                alpha=0.15, color='#4cc9f0', label='Avantage Fuzzy (droite)')
ax.fill_between(c_vals, spd_fuzzy, spd_pid,
                where=[f<p for f,p in zip(spd_fuzzy, spd_pid)],
                alpha=0.15, color='#9b72cf', label='Avantage Fuzzy (virage)')
ax.set_title('Vitesse effective vs Courbure  (erreur=0)', color='#ffd166', fontweight='bold')
ax.set_xlabel('Courbure (0=droite, 1=virage serré)'); ax.set_ylabel('Vitesse normalisée')
ax.legend(fontsize=8, framealpha=0.3); ax.grid(True, alpha=0.3)
ax.set_xlim(-0.02, 1.02); ax.set_ylim(0, 1.1)

plt.tight_layout()
plt.savefig('img/02_fuzzy_rules_surface.png', dpi=150, bbox_inches='tight',
            facecolor='#0a0c11')
plt.show()
print("💾 Sauvegardé : img/02_fuzzy_rules_surface.png")
""")

# ─────────────────────────────────────────────────────────────────────────────
# CELL 7 — Fuzzy Engine + PID + Transformer
# ─────────────────────────────────────────────────────────────────────────────
md("""---
## ⚙️ Partie 3 — Moteurs de contrôle (Fuzzy, PID, Transformer)
""")

code("""# ══════════════════════════════════════════════════════
# CELLULE 7 — Moteur d'inférence Fuzzy Mamdani
# ══════════════════════════════════════════════════════

class FuzzyEngine:
    \"\"\"
    Inférence Mamdani complète.
    Pipeline : Fuzzification → AND=min → Agrégation → Défuzzification COG
    \"\"\"
    def __init__(self):
        self.fs = FuzzySets()
        self.last_active_rules = []

    def fuzzify_error(self, e):
        return {'NL':self.fs.error_NL(e),'NM':self.fs.error_NM(e),
                'NS':self.fs.error_NS(e),'ZE':self.fs.error_ZE(e),
                'PS':self.fs.error_PS(e),'PM':self.fs.error_PM(e),
                'PL':self.fs.error_PL(e)}

    def fuzzify_rate(self, r):
        return {'NL':self.fs.rate_NL(r),'NS':self.fs.rate_NS(r),
                'ZE':self.fs.rate_ZE(r),'PS':self.fs.rate_PS(r),
                'PL':self.fs.rate_PL(r)}

    def fuzzify_curvature(self, c):
        return {'STRAIGHT':self.fs.curv_STRAIGHT(c),
                'MILD':self.fs.curv_MILD(c),'SHARP':self.fs.curv_SHARP(c)}

    def infer(self, error, error_rate, curvature):
        mu_e = self.fuzzify_error(error)
        mu_r = self.fuzzify_rate(error_rate)
        mu_c = self.fuzzify_curvature(curvature)
        steer_n=steer_d=cs_n=cs_d=ss_n=ss_d=0.0
        active = []
        for rule in FUZZY_RULES:
            e_s,r_s,c_s,steer_o,cs_o,ss_o = rule
            ae = mu_e.get(e_s,1.0) if e_s!='ANY' else 1.0
            ar = mu_r.get(r_s,1.0) if r_s!='ANY' else 1.0
            ac = mu_c.get(c_s,1.0) if c_s!='ANY' else 1.0
            alpha = min(ae,ar,ac)
            if alpha < 0.001: continue
            sv = self.fs.STEER_CENTERS.get(steer_o)
            cv = self.fs.CORNER_SPEED_CENTERS.get(cs_o)
            dv = self.fs.STRAIGHT_SPEED_CENTERS.get(ss_o)
            if sv is None or cv is None or dv is None: continue
            steer_n+=alpha*sv; steer_d+=alpha
            cs_n+=alpha*cv;    cs_d+=alpha
            ss_n+=alpha*dv;    ss_d+=alpha
            active.append({'rule':f"{e_s}/{r_s}/{c_s}",
                           'consequence':f"steer={steer_o}, cs={cs_o}, ss={ss_o}",
                           'activation':round(alpha,4)})
        steer = steer_n/steer_d if steer_d>0 else 0.0
        cs    = cs_n/cs_d       if cs_d>0    else 0.5
        ss    = ss_n/ss_d       if ss_d>0    else 0.5
        self.last_active_rules = sorted(active,key=lambda x:-x['activation'])[:8]
        return {'steer':max(-1.,min(1.,steer)),'corner_speed':max(0.,min(1.,cs)),
                'straight_speed':max(0.,min(1.,ss)),'active_rules':self.last_active_rules,
                'mu_error':mu_e,'mu_rate':mu_r,'mu_curv':mu_c}


# ══════════════════════════════════════════════════════
# Transformer simplifié (mémoire + tendance)
# ══════════════════════════════════════════════════════

class SimpleTransformer:
    \"\"\"Mémoire de 16 états + correction par régression linéaire (tendance)\"\"\"
    def __init__(self, memory_size=16):
        self.memory = []
        self.max_size = memory_size

    def update(self, state):
        self.memory.append(state)
        if len(self.memory) > self.max_size:
            self.memory.pop(0)

    def predict(self):
        if len(self.memory) < 5:
            return {'steer_correction':0.,'speed_correction':0.,'confidence':0.}
        errors = [s['error'] for s in self.memory[-10:]]
        x = np.arange(len(errors))
        coeffs = np.polyfit(x, errors, 1)
        trend = coeffs[0]
        if abs(trend) > 0.02:
            corr = float(np.clip(trend*2., -0.15, 0.15))
            conf = min(0.8, abs(trend)*5.)
        else:
            corr, conf = 0., 0.
        return {'steer_correction':corr,'speed_correction':corr*0.5,'confidence':conf}


# ══════════════════════════════════════════════════════
# Contrôleur PID classique
# ══════════════════════════════════════════════════════

class PIDController:
    \"\"\"PID standard — vitesse constante, pas d'adaptation à la courbure\"\"\"
    def __init__(self, kp=1.8, ki=0.025, kd=1.4, speed=0.60):
        self.kp=kp; self.ki=ki; self.kd=kd; self.speed=speed
        self._integral=0.; self._prev_error=0.; self.dt=1./60.

    def compute(self, error):
        self._integral = max(-10., min(10., self._integral + error*self.dt))
        deriv = (error - self._prev_error)/self.dt
        p,i,d = self.kp*error, self.ki*self._integral, self.kd*deriv
        steer = max(-1., min(1., p+i+d))
        self._prev_error = error
        return {'steer':steer,'speed':self.speed,'p_term':round(p,4),
                'i_term':round(i,4),'d_term':round(d,4)}

    def reset(self):
        self._integral=0.; self._prev_error=0.


print("✅ FuzzyEngine, SimpleTransformer, PIDController définis")

# Test rapide
eng = FuzzyEngine()
out = eng.infer(error=1.5, error_rate=0.3, curvature=0.6)
print(f"\\nTest inférence (err=1.5, rate=0.3, curv=0.6) :")
print(f"  → Braquage    : {out['steer']:.3f}")
print(f"  → V.Virage    : {out['corner_speed']:.3f}")
print(f"  → V.Droite    : {out['straight_speed']:.3f}")
print(f"  → Règles actives : {len(out['active_rules'])}")
for r in out['active_rules'][:3]:
    print(f"     {r['rule']:30s} α={r['activation']:.3f}  →  {r['consequence']}")
""")

# ─────────────────────────────────────────────────────────────────────────────
# CELL 8 — Autodrome + Simulation Engine
# ─────────────────────────────────────────────────────────────────────────────
md("""---
## 🏟️ Partie 4 — Autodrome oval + Moteur de simulation
""")

code("""# ══════════════════════════════════════════════════════
# CELLULE 8 — Autodrome oval réaliste
# ══════════════════════════════════════════════════════

class Autodrome:
    \"\"\"
    Circuit oval réaliste avec :
    - Grande ligne droite (départ/arrivée)
    - Grand virage à chaque extrémité
    - Légère chicane pour varier la courbure
    - Ligne centrale pour le suivi
    \"\"\"
    def __init__(self):
        self.waypoints = self._build_track()
        self.total_length = len(self.waypoints)

    def _build_track(self):
        pts = []
        N = 480  # Résolution haute pour un oval fluide
        for i in range(N):
            t = i / N
            angle = t * 2 * math.pi
            # Oval : rayon X plus grand que rayon Y
            rx = 0.38
            ry = 0.26
            # Légère déformation sinusoïdale pour créer des variations de courbure
            # (simule les zones d'accélération / freinage d'un vrai circuit)
            deform = 0.025 * math.sin(angle * 2 - 0.5)
            rx_local = rx + deform
            x = 0.5 + rx_local * math.cos(angle)
            y = 0.5 + ry * math.sin(angle)
            pts.append((x, y))
        return pts

    def get_point(self, t):
        idx = int((t % 1.0) * (self.total_length - 1))
        return self.waypoints[idx]

    def get_tangent(self, t):
        idx  = int((t % 1.0) * (self.total_length - 1))
        idx2 = (idx + 1) % self.total_length
        p1, p2 = self.waypoints[idx], self.waypoints[idx2]
        dx, dy = p2[0]-p1[0], p2[1]-p1[1]
        n = math.hypot(dx, dy) + 1e-9
        return dx/n, dy/n

    def get_curvature(self, t):
        dt = 0.004
        tx1, ty1 = self.get_tangent(t - dt)
        tx2, ty2 = self.get_tangent(t + dt)
        cross = tx1*ty2 - ty1*tx2
        angle = abs(math.asin(max(-1., min(1., cross))))
        return min(1., angle / (math.pi / 6))


# ══════════════════════════════════════════════════════
# Robot state + Simulation Engine
# ══════════════════════════════════════════════════════

@dataclass
class RobotState:
    x: float = 0.5; y: float = 0.5; angle: float = 0.0; t: float = 0.0
    error: float = 0.0; error_rate: float = 0.0
    speed: float = 0.5; steer: float = 0.0
    laps: int = 0; lost_count: int = 0
    total_error: float = 0.0; steps: int = 0; mode: str = 'fuzzy'


class SimulationEngine:
    DT = 1.0/60.0
    TRACK_WIDTH = 0.055

    def __init__(self):
        self.track = Autodrome()
        self.fuzzy = FuzzyEngine()
        self.transformer = SimpleTransformer()
        self.pid = PIDController()
        # Les deux robots démarrent au même endroit (t=0.0)
        self.state_fuzzy = RobotState(mode='fuzzy', t=0.0)
        self.state_pid   = RobotState(mode='pid',   t=0.0)
        self._init_robots()

    def _init_robots(self):
        for bot in [self.state_fuzzy, self.state_pid]:
            bot.x, bot.y = self.track.get_point(bot.t)
            tx, ty = self.track.get_tangent(bot.t)
            bot.angle = math.atan2(ty, tx)

    def _sense(self, bot):
        best_t, best_d = bot.t, float('inf')
        for delta in np.linspace(-0.04, 0.04, 20):
            tt = (bot.t + delta) % 1.0
            px, py = self.track.get_point(tt)
            d = (bot.x-px)**2 + (bot.y-py)**2
            if d < best_d:
                best_d, best_t = d, tt
        bot.t = best_t
        px, py = self.track.get_point(best_t)
        tx, ty = self.track.get_tangent(best_t)
        nx, ny = -ty, tx
        dx, dy = bot.x-px, bot.y-py
        raw_error = (dx*nx + dy*ny) / self.TRACK_WIDTH
        error = max(-3., min(3., raw_error * 3.0))
        curvature = self.track.get_curvature(best_t)
        return error, curvature

    def _advance(self, bot, steer, speed):
        step = self.DT * speed * 0.75
        bot.angle += steer * speed * 3.5 * self.DT
        bot.x += speed * math.cos(bot.angle) * step
        bot.y += speed * math.sin(bot.angle) * step
        bot.t = (bot.t + step * 0.85) % 1.0

    def step_fuzzy(self):
        bot = self.state_fuzzy
        error, curvature = self._sense(bot)
        rate = (error - bot.error)/self.DT if bot.steps>0 else 0.
        bot.error = error
        bot.error_rate = max(-2., min(2., rate))
        out = self.fuzzy.infer(error, bot.error_rate, curvature)
        steer = out['steer']
        speed = out['corner_speed']*curvature + out['straight_speed']*(1-curvature)
        speed = max(0.1, min(1., speed))
        self.transformer.update({'error':error,'rate':bot.error_rate,
                                  'curvature':curvature,'speed':speed})
        pred = self.transformer.predict()
        steer = max(-1., min(1., steer + pred['steer_correction']*pred['confidence']))
        speed = max(0.1, min(1., speed + pred['speed_correction']*pred['confidence']))
        self._advance(bot, steer, speed)
        bot.steer=steer; bot.speed=speed; bot.steps+=1
        bot.total_error += abs(error)
        if abs(error)>1.2: bot.lost_count+=1
        if bot.t<0.01 and bot.steps>100: bot.laps+=1
        return {'fz':bot, 'error':error, 'curvature':curvature,
                'active_rules':self.fuzzy.last_active_rules}

    def step_pid(self):
        bot = self.state_pid
        error, curvature = self._sense(bot)
        bot.error = error
        out = self.pid.compute(error)
        self._advance(bot, out['steer'], out['speed'])
        bot.steer=out['steer']; bot.speed=out['speed']; bot.steps+=1
        bot.total_error += abs(error)
        if abs(error)>1.2: bot.lost_count+=1
        if bot.t<0.01 and bot.steps>100: bot.laps+=1
        return {'pid':bot,'error':error,'curvature':curvature,'pid_output':out}

    def reset(self):
        self.state_fuzzy = RobotState(mode='fuzzy', t=0.0)
        self.state_pid   = RobotState(mode='pid',   t=0.0)
        self.transformer = SimpleTransformer()
        self.pid.reset()
        self._init_robots()


sim = SimulationEngine()
print("✅ Autodrome oval + SimulationEngine initialisés")
print(f"   Waypoints    : {len(sim.track.waypoints)}")
print(f"   Largeur piste: {sim.TRACK_WIDTH}")
print(f"   Départ Fuzzy : t=0.0  →  x={sim.state_fuzzy.x:.3f}, y={sim.state_fuzzy.y:.3f}")
print(f"   Départ PID   : t=0.0  →  x={sim.state_pid.x:.3f},   y={sim.state_pid.y:.3f}")
""")

# ─────────────────────────────────────────────────────────────────────────────
# CELL 9 — Run simulation
# ─────────────────────────────────────────────────────────────────────────────
md("""---
## 🚀 Partie 5 — Exécution de la simulation (N pas)
""")

code("""# ══════════════════════════════════════════════════════
# CELLULE 9 — Simulation complète (3 tours)
# ══════════════════════════════════════════════════════

sim.reset()

N_STEPS = 4000  # ~67 secondes à 60Hz — suffisant pour ~3 tours

# Historiques
hist = {
    'fz_x':[], 'fz_y':[], 'fz_err':[], 'fz_speed':[], 'fz_steer':[],
    'fz_curv':[], 'fz_laps':[],
    'pid_x':[], 'pid_y':[], 'pid_err':[], 'pid_speed':[], 'pid_steer':[],
    'pid_curv':[], 'pid_laps':[],
    'active_rules_count':[], 'steps':[],
}

print("▶ Simulation en cours...")
for step in range(N_STEPS):
    rf = sim.step_fuzzy()
    rp = sim.step_pid()
    fz  = rf['fz']
    pid = rp['pid']

    hist['fz_x'].append(fz.x);        hist['fz_y'].append(fz.y)
    hist['fz_err'].append(fz.error);   hist['fz_speed'].append(fz.speed)
    hist['fz_steer'].append(fz.steer); hist['fz_curv'].append(rf['curvature'])
    hist['fz_laps'].append(fz.laps)

    hist['pid_x'].append(pid.x);       hist['pid_y'].append(pid.y)
    hist['pid_err'].append(pid.error);  hist['pid_speed'].append(pid.speed)
    hist['pid_steer'].append(pid.steer);hist['pid_curv'].append(rp['curvature'])
    hist['pid_laps'].append(pid.laps)

    hist['active_rules_count'].append(len(rf['active_rules']))
    hist['steps'].append(step)

    if (step+1) % 1000 == 0:
        print(f"  Pas {step+1:4d}/{N_STEPS}  |  "
              f"Fuzzy: tours={fz.laps} err_moy={fz.total_error/fz.steps:.3f}  |  "
              f"PID:   tours={pid.laps} err_moy={pid.total_error/pid.steps:.3f}")

fz_final  = sim.state_fuzzy
pid_final = sim.state_pid
print(f"\\n{'═'*62}")
print(f"  RÉSULTATS FINAUX après {N_STEPS} pas ({N_STEPS/60:.0f}s)")
print(f"{'═'*62}")
print(f"  {'Métrique':<25} {'Fuzzy+Trans':>15}  {'PID':>12}")
print(f"  {'─'*52}")
print(f"  {'Tours complétés':<25} {fz_final.laps:>15}  {pid_final.laps:>12}")
fze = fz_final.total_error/max(1,fz_final.steps)
pide= pid_final.total_error/max(1,pid_final.steps)
print(f"  {'Erreur moyenne':<25} {fze:>15.4f}  {pide:>12.4f}")
print(f"  {'Sorties de piste':<25} {fz_final.lost_count:>15}  {pid_final.lost_count:>12}")
gain = (pide-fze)/pide*100 if pide>0 else 0
print(f"  {'─'*52}")
print(f"  → Fuzzy meilleur de {gain:.1f}% sur l'erreur moyenne" if gain>0
      else f"  → PID meilleur de {-gain:.1f}% sur l'erreur moyenne")
""")

# ─────────────────────────────────────────────────────────────────────────────
# CELL 10 — Autodrome visualization (static)
# ─────────────────────────────────────────────────────────────────────────────
md("""---
## 📊 Partie 6 — Visualisations statiques complètes
""")

code("""# ══════════════════════════════════════════════════════
# CELLULE 10 — Dessin de l'autodrome avec trajectoires
# ══════════════════════════════════════════════════════

def draw_car(ax, x, y, angle, color, size=0.018, label=''):
    \"\"\"Dessine une voiture de course stylisée\"\"\"
    # Corps principal
    cos_a, sin_a = math.cos(angle), math.sin(angle)
    # Coins du rectangle
    L, W = size*2.2, size*1.0
    corners = [(-L/2,-W/2),( L/2,-W/2),( L/2, W/2),(-L/2, W/2)]
    rotated = [(x + cx*cos_a - cy*sin_a, y + cx*sin_a + cy*cos_a) for cx,cy in corners]
    car_patch = plt.Polygon(rotated, closed=True, facecolor=color, edgecolor='white',
                             linewidth=0.8, alpha=0.95, zorder=10)
    ax.add_patch(car_patch)
    # Flèche direction
    ax.annotate('', xy=(x + L*0.7*cos_a, y + L*0.7*sin_a),
                xytext=(x, y),
                arrowprops=dict(arrowstyle='->', color='white', lw=1.2),
                zorder=11)
    # Label
    if label:
        ax.text(x + size*1.5*cos_a - size*1.5*sin_a,
                y + size*1.5*sin_a + size*1.5*cos_a,
                label, color=color, fontsize=7, fontweight='bold',
                ha='center', zorder=12)

fig, ax = plt.subplots(1, 1, figsize=(16, 8))
fig.patch.set_facecolor('#0a0c11')
ax.set_facecolor('#0d0f14')

# ── Asphalte (piste colorée par courbure) ─────────────
track_pts = sim.track.waypoints
n = len(track_pts)
TRACK_W = 0.052  # demi-largeur visuelle

from matplotlib.collections import LineCollection
from matplotlib.cm import ScalarMappable
from matplotlib.colors import Normalize

# Colormap courbure : bleu=droite, orange=virage
cmap_curv = LinearSegmentedColormap.from_list('track',
    ['#1a4a8a','#2a6a4a','#8a6a0a','#8a2a0a'])

segments_outer = []; segments_inner = []; curvatures_seg = []
for i in range(n):
    j = (i+1)%n
    x1,y1 = track_pts[i]; x2,y2 = track_pts[j]
    tx = x2-x1; ty = y2-y1
    L = math.hypot(tx,ty)+1e-9
    nx,ny = -ty/L, tx/L
    c = sim.track.get_curvature(i/n)
    curvatures_seg.append(c)
    segments_outer.append([(x1+nx*TRACK_W*1.5, y1+ny*TRACK_W*1.5),
                            (x2+nx*TRACK_W*1.5, y2+ny*TRACK_W*1.5)])
    segments_inner.append([(x1-nx*TRACK_W*1.5, y1-ny*TRACK_W*1.5),
                            (x2-nx*TRACK_W*1.5, y2-ny*TRACK_W*1.5)])

# Fond de la piste (asphalte)
for i in range(n):
    j=(i+1)%n
    x1,y1=track_pts[i]; x2,y2=track_pts[j]
    tx=x2-x1; ty=y2-y1; L=math.hypot(tx,ty)+1e-9
    nx,ny=-ty/L,tx/L
    c = curvatures_seg[i]
    gray = int(42 + c*18)
    color = f'#{gray:02x}{gray:02x}{gray+6:02x}'
    pts_patch = [
        (x1+nx*TRACK_W*1.5, y1+ny*TRACK_W*1.5),
        (x2+nx*TRACK_W*1.5, y2+ny*TRACK_W*1.5),
        (x2-nx*TRACK_W*1.5, y2-ny*TRACK_W*1.5),
        (x1-nx*TRACK_W*1.5, y1-ny*TRACK_W*1.5),
    ]
    ax.add_patch(plt.Polygon(pts_patch, closed=True, facecolor=color,
                              edgecolor='none', zorder=1))

# Heatmap courbure overlay
for i in range(0,n,2):
    j=(i+2)%n
    x1,y1=track_pts[i]; x2,y2=track_pts[j]
    c = curvatures_seg[i]
    if c > 0.25:
        tx=x2-x1; ty=y2-y1; L=math.hypot(tx,ty)+1e-9
        nx,ny=-ty/L,tx/L
        alpha_heat = (c-0.25)*0.35
        pts_h = [(x1+nx*TRACK_W*1.3,y1+ny*TRACK_W*1.3),
                 (x2+nx*TRACK_W*1.3,y2+ny*TRACK_W*1.3),
                 (x2-nx*TRACK_W*1.3,y2-ny*TRACK_W*1.3),
                 (x1-nx*TRACK_W*1.3,y1-ny*TRACK_W*1.3)]
        ax.add_patch(plt.Polygon(pts_h, closed=True,
                                  facecolor='#ff5020', edgecolor='none',
                                  alpha=alpha_heat, zorder=2))

# Bordures blanches
for i in range(0,n,1):
    j=(i+1)%n
    x1,y1=track_pts[i]; x2,y2=track_pts[j]
    tx=x2-x1; ty=y2-y1; L=math.hypot(tx,ty)+1e-9
    nx,ny=-ty/L,tx/L
    for side,lw in [(1.5,1.0),(-1.5,1.0)]:
        ax.plot([x1+nx*TRACK_W*side, x2+nx*TRACK_W*side],
                [y1+ny*TRACK_W*side, y2+ny*TRACK_W*side],
                color='white', alpha=0.15, linewidth=lw, zorder=3)

# Ligne centrale (tirets blancs)
xs = [p[0] for p in track_pts] + [track_pts[0][0]]
ys = [p[1] for p in track_pts] + [track_pts[0][1]]
ax.plot(xs, ys, color='white', alpha=0.22, linewidth=0.9,
        linestyle=(0,(6,10)), zorder=4)

# ── Trajectoires ──────────────────────────────────────
fz_x  = hist['fz_x'];  fz_y  = hist['fz_y']
pid_x = hist['pid_x']; pid_y = hist['pid_y']
# Gradient de couleur sur la trajectoire
n_trail = len(fz_x)
for k in range(0, n_trail-1, 3):
    alpha = 0.3 + 0.5*(k/n_trail)
    ax.plot([fz_x[k],fz_x[k+1]], [fz_y[k],fz_y[k+1]],
            color='#00e5b0', alpha=alpha, linewidth=1.5, zorder=5)
    ax.plot([pid_x[k],pid_x[k+1]], [pid_y[k],pid_y[k+1]],
            color='#ff4560', alpha=alpha, linewidth=1.5, zorder=5)

# ── Position finale des voitures ──────────────────────
fz_f  = sim.state_fuzzy
pid_f = sim.state_pid
draw_car(ax, fz_f.x,  fz_f.y,  fz_f.angle,  '#00e5b0', label='FUZZY')
draw_car(ax, pid_f.x, pid_f.y, pid_f.angle, '#ff4560', label='PID')

# ── Ligne de départ ───────────────────────────────────
sx, sy = sim.track.get_point(0.0)
tx, ty = sim.track.get_tangent(0.0)
nx, ny = -ty, tx
ax.plot([sx-nx*TRACK_W*1.6, sx+nx*TRACK_W*1.6],
        [sy-ny*TRACK_W*1.6, sy+ny*TRACK_W*1.6],
        color='white', linewidth=2.5, alpha=0.8, zorder=6)
ax.text(sx+0.02, sy-0.04, 'START', color='white', fontsize=9,
        fontweight='bold', alpha=0.7, zorder=7)

# Marqueurs de zones
for t_pos, label, col in [(0.25,'VIRAGE\\nDROIT','#ff6b6b'),
                           (0.5,'LIGNE\\nDROITE','#4cc9f0'),
                           (0.75,'VIRAGE\\nGAUCHE','#ff6b6b')]:
    px2, py2 = sim.track.get_point(t_pos)
    ax.text(px2, py2+0.07, label, ha='center', color=col,
            fontsize=7, fontweight='bold', alpha=0.6, zorder=7)

# ── Annotations performance ────────────────────────────
fze2  = fz_final.total_error/max(1,fz_final.steps)
pide2 = pid_final.total_error/max(1,pid_final.steps)
info = (f"FUZZY+TRANS   Tours:{fz_final.laps}  Err:{fze2:.3f}  Hors:{fz_final.lost_count}\\n"
        f"PID           Tours:{pid_final.laps}  Err:{pide2:.3f}  Hors:{pid_final.lost_count}")
ax.text(0.02, 0.97, info, transform=ax.transAxes,
        color='#dde2f0', fontsize=9, fontfamily='monospace',
        verticalalignment='top',
        bbox=dict(boxstyle='round,pad=0.5', facecolor='#12151c',
                  edgecolor='#252a3a', alpha=0.9), zorder=8)

# Légende
leg_items = [
    mpatches.Patch(facecolor='#00e5b0', label='Fuzzy + Transformer'),
    mpatches.Patch(facecolor='#ff4560', label='PID classique'),
    mpatches.Patch(facecolor='#ff5020', alpha=0.5, label='Zone de virage (chaleur)'),
]
ax.legend(handles=leg_items, loc='lower right', fontsize=9,
          framealpha=0.4, facecolor='#12151c', edgecolor='#252a3a')

ax.set_title('🏎 Autodrome Oval — Trajectoires Fuzzy+Transformer vs PID',
             color='#dde2f0', fontsize=13, fontweight='bold', pad=12)
ax.set_xlim(0.05, 0.95); ax.set_ylim(0.12, 0.88)
ax.set_aspect('equal'); ax.axis('off')

plt.tight_layout()
plt.savefig('img/03_autodrome_trajectoires.png', dpi=150, bbox_inches='tight',
            facecolor='#0a0c11')
plt.show()
print("💾 Sauvegardé : img/03_autodrome_trajectoires.png")
""")

# ─────────────────────────────────────────────────────────────────────────────
# CELL 11 — Error comparison chart
# ─────────────────────────────────────────────────────────────────────────────
code("""# ══════════════════════════════════════════════════════
# CELLULE 11 — Comparaison erreur latérale & vitesse
# ══════════════════════════════════════════════════════

fig, axes = plt.subplots(2, 2, figsize=(16, 9))
fig.patch.set_facecolor('#0a0c11')
fig.suptitle('Analyse comparative — Fuzzy+Transformer vs PID',
             color='#dde2f0', fontsize=13, fontweight='bold')

steps_arr = np.array(hist['steps'])
t_sec = steps_arr / 60.0  # temps en secondes

# ── Erreur latérale ──────────────────────────────────
ax = axes[0,0]; ax.set_facecolor('#12151c')
ax.plot(t_sec, hist['fz_err'],  color='#00e5b0', linewidth=1.2,
        label='Fuzzy+Trans', alpha=0.85)
ax.plot(t_sec, hist['pid_err'], color='#ff4560', linewidth=1.2,
        label='PID', alpha=0.85)
ax.fill_between(t_sec, hist['fz_err'],  alpha=0.06, color='#00e5b0')
ax.fill_between(t_sec, hist['pid_err'], alpha=0.06, color='#ff4560')
ax.axhline(0, color='white', alpha=0.15, linewidth=0.5)
ax.axhline( 1.2, color='#ffd166', alpha=0.3, linewidth=0.7, linestyle='--')
ax.axhline(-1.2, color='#ffd166', alpha=0.3, linewidth=0.7, linestyle='--')
ax.text(t_sec[-1]*0.02, 1.35, 'Seuil hors-piste', color='#ffd166', fontsize=7)
ax.set_title('Erreur latérale (historique complet)', color='#00e5b0')
ax.set_xlabel('Temps (s)'); ax.set_ylabel('Erreur latérale')
ax.legend(fontsize=9, framealpha=0.3); ax.grid(True, alpha=0.3)
ax.set_ylim(-3.5, 3.5)

# ── Vitesse ───────────────────────────────────────────
ax = axes[0,1]; ax.set_facecolor('#12151c')
ax.plot(t_sec, hist['fz_speed'],  color='#00e5b0', linewidth=1.2,
        label='Fuzzy+Trans (adaptative)')
ax.plot(t_sec, hist['pid_speed'], color='#ff4560', linewidth=1.2,
        label='PID (constante)', linestyle='--')
ax.fill_between(t_sec, hist['fz_speed'], hist['pid_speed'],
                where=[f>p for f,p in zip(hist['fz_speed'],hist['pid_speed'])],
                alpha=0.1, color='#4cc9f0', label='Avantage Fuzzy (+ rapide)')
ax.fill_between(t_sec, hist['fz_speed'], hist['pid_speed'],
                where=[f<p for f,p in zip(hist['fz_speed'],hist['pid_speed'])],
                alpha=0.1, color='#9b72cf', label='Avantage Fuzzy (ralentit)')
ax.set_title('Vitesse adaptative vs constante', color='#ffd166')
ax.set_xlabel('Temps (s)'); ax.set_ylabel('Vitesse normalisée')
ax.legend(fontsize=8, framealpha=0.3); ax.grid(True, alpha=0.3)
ax.set_ylim(0, 1.1)

# ── Erreur absolue cumulée ────────────────────────────
ax = axes[1,0]; ax.set_facecolor('#12151c')
fz_cum  = np.cumsum(np.abs(hist['fz_err']))
pid_cum = np.cumsum(np.abs(hist['pid_err']))
ax.plot(t_sec, fz_cum,  color='#00e5b0', linewidth=2, label='Fuzzy+Trans')
ax.plot(t_sec, pid_cum, color='#ff4560', linewidth=2, label='PID')
ax.fill_between(t_sec, fz_cum, pid_cum,
                where=pid_cum>fz_cum, alpha=0.12,
                color='#00e5b0', label='Avantage Fuzzy')
ax.set_title('Erreur absolue cumulée (IAE)', color='#4cc9f0')
ax.set_xlabel('Temps (s)'); ax.set_ylabel('Σ|erreur|')
ax.legend(fontsize=9, framealpha=0.3); ax.grid(True, alpha=0.3)
final_gain = (pid_cum[-1]-fz_cum[-1])/pid_cum[-1]*100
ax.text(0.98, 0.05, f'Réduction IAE: {final_gain:.1f}%',
        transform=ax.transAxes, ha='right', color='#00e5b0', fontsize=10,
        fontweight='bold',
        bbox=dict(boxstyle='round', facecolor='#0a0c11', alpha=0.8))

# ── Règles actives ────────────────────────────────────
ax = axes[1,1]; ax.set_facecolor('#12151c')
ax.plot(t_sec, hist['active_rules_count'], color='#9b72cf', linewidth=1.2,
        alpha=0.8)
ax.fill_between(t_sec, hist['active_rules_count'], alpha=0.1, color='#9b72cf')
# Curvature overlay
ax2 = ax.twinx()
ax2.set_facecolor('none')
ax2.plot(t_sec, hist['fz_curv'], color='#ffd166', linewidth=0.8,
         alpha=0.5, label='Courbure')
ax2.set_ylabel('Courbure', color='#ffd166')
ax2.tick_params(axis='y', colors='#ffd166')
ax.set_title('Règles Fuzzy actives vs Courbure', color='#9b72cf')
ax.set_xlabel('Temps (s)'); ax.set_ylabel('Nb règles actives', color='#9b72cf')
ax.grid(True, alpha=0.3); ax.set_ylim(0, 12)
legend_items = [mpatches.Patch(color='#9b72cf',label='Règles actives'),
                mpatches.Patch(color='#ffd166',label='Courbure')]
ax.legend(handles=legend_items, fontsize=8, framealpha=0.3)

plt.tight_layout()
plt.savefig('img/04_comparaison_erreur_vitesse.png', dpi=150, bbox_inches='tight',
            facecolor='#0a0c11')
plt.show()
print("💾 Sauvegardé : img/04_comparaison_erreur_vitesse.png")
""")

# ─────────────────────────────────────────────────────────────────────────────
# CELL 12 — Active rules heatmap
# ─────────────────────────────────────────────────────────────────────────────
code("""# ══════════════════════════════════════════════════════
# CELLULE 12 — Heatmap d'activation des 21 règles
# ══════════════════════════════════════════════════════

# Ré-exécution pour collecter les activations par règle
sim2 = SimulationEngine()
N2 = 1200  # ~20s

rule_activations = np.zeros((21, N2))
errors2 = []; curvs2 = []

for step in range(N2):
    error2, curv2 = sim2._sense(sim2.state_fuzzy)
    rate2 = (error2 - sim2.state_fuzzy.error)/sim2.DT if step>0 else 0.
    sim2.state_fuzzy.error = error2
    sim2.state_fuzzy.error_rate = max(-2.,min(2.,rate2))
    out2 = sim2.fuzzy.infer(error2, sim2.state_fuzzy.error_rate, curv2)
    sim2.step_pid()
    sim2._advance(sim2.state_fuzzy, out2['steer'],
                  out2['corner_speed']*curv2 + out2['straight_speed']*(1-curv2))
    sim2.state_fuzzy.steps += 1
    sim2.state_fuzzy.t = (sim2.state_fuzzy.t + 0.001) % 1.0

    # Récupérer activations
    mu_e = sim2.fuzzy.fuzzify_error(error2)
    mu_r = sim2.fuzzy.fuzzify_rate(sim2.state_fuzzy.error_rate)
    mu_c = sim2.fuzzy.fuzzify_curvature(curv2)
    for ri, rule in enumerate(FUZZY_RULES):
        e_s,r_s,c_s,_,_,_ = rule
        ae = mu_e.get(e_s,1.) if e_s!='ANY' else 1.
        ar = mu_r.get(r_s,1.) if r_s!='ANY' else 1.
        ac = mu_c.get(c_s,1.) if c_s!='ANY' else 1.
        rule_activations[ri, step] = min(ae,ar,ac)
    errors2.append(error2); curvs2.append(curv2)

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(16, 10),
                                 gridspec_kw={'height_ratios':[3,1]})
fig.patch.set_facecolor('#0a0c11')

# Heatmap
im = ax1.imshow(rule_activations, aspect='auto', cmap='hot',
                 extent=[0, N2/60, 0, 21], origin='lower', vmin=0, vmax=1)
ax1.set_facecolor('#12151c')

# Group separators
group_boundaries = [0, 7, 11, 16, 19, 21]
group_labels_y   = [3.5, 9, 13.5, 17.5, 20]
group_labels_txt = ['G1\nErreur\n(R1–R7)','G2\nDérivée\n(R8–R11)',
                    'G3\nVirage\n(R12–R16)','G4\nDroite\n(R17–R19)',
                    'G5\nSécurité\n(R20–R21)']
group_colors2 = ['#00e5b0','#4cc9f0','#ffd166','#9b72cf','#ff6b6b']
for gb in group_boundaries:
    ax1.axhline(gb-0.5, color='#252a3a', linewidth=1.5)
for gy, gt, gc in zip(group_labels_y, group_labels_txt, group_colors2):
    ax1.text(-0.8, gy, gt, ha='right', va='center', color=gc,
             fontsize=7, fontweight='bold')

# Y-axis labels
rule_labels = [f"R{i+1:02d}" for i in range(21)]
ax1.set_yticks(range(21))
ax1.set_yticklabels(rule_labels, fontsize=7)
ax1.set_title('Heatmap d\'activation des 21 règles Fuzzy au cours du temps',
              color='#dde2f0', fontweight='bold')
ax1.set_xlabel('Temps (s)'); ax1.set_ylabel('Règle')
plt.colorbar(im, ax=ax1, label='Degré d\'activation α', fraction=0.02)

# Erreur + courbure en dessous
t2 = np.arange(N2)/60.
ax2.set_facecolor('#12151c')
ax2.plot(t2, errors2, color='#00e5b0', linewidth=1., label='Erreur Fuzzy', alpha=0.8)
ax2_b = ax2.twinx()
ax2_b.set_facecolor('none')
ax2_b.plot(t2, curvs2, color='#ffd166', linewidth=0.8, alpha=0.5, label='Courbure')
ax2_b.set_ylabel('Courbure', color='#ffd166')
ax2_b.tick_params(axis='y', colors='#ffd166')
ax2.axhline(0, color='white', alpha=0.1, linewidth=0.5)
ax2.set_xlabel('Temps (s)'); ax2.set_ylabel('Erreur latérale', color='#00e5b0')
ax2.set_xlim(0, N2/60); ax2.grid(True, alpha=0.3)
ax2.legend(fontsize=8, framealpha=0.3, loc='upper right')

plt.tight_layout()
plt.savefig('img/05_heatmap_regles_activation.png', dpi=150, bbox_inches='tight',
            facecolor='#0a0c11')
plt.show()
print("💾 Sauvegardé : img/05_heatmap_regles_activation.png")
""")

# ─────────────────────────────────────────────────────────────────────────────
# CELL 13 — PID analysis
# ─────────────────────────────────────────────────────────────────────────────
code("""# ══════════════════════════════════════════════════════
# CELLULE 13 — Analyse détaillée du contrôleur PID
# ══════════════════════════════════════════════════════

sim3 = SimulationEngine()
N3 = 1800

p_terms=[]; i_terms=[]; d_terms=[]; pid_steers=[]; pid_errs=[]; pid_curvs=[]

for _ in range(N3):
    rp = sim3.step_pid()
    sim3.step_fuzzy()
    po = rp['pid_output']
    p_terms.append(po['p_term']); i_terms.append(po['i_term'])
    d_terms.append(po['d_term']); pid_steers.append(rp['pid'].steer)
    pid_errs.append(rp['error']); pid_curvs.append(rp['curvature'])

t3 = np.arange(N3)/60.

fig, axes = plt.subplots(2, 2, figsize=(16, 9))
fig.patch.set_facecolor('#0a0c11')
fig.suptitle('Analyse détaillée du contrôleur PID  (Kp=1.8, Ki=0.025, Kd=1.4)',
             color='#ff4560', fontsize=13, fontweight='bold')

# Termes P, I, D
ax = axes[0,0]; ax.set_facecolor('#12151c')
ax.plot(t3, p_terms, color='#ff4560', linewidth=1.2, label=f'P = Kp·e')
ax.plot(t3, i_terms, color='#ffd166', linewidth=1.2, label=f'I = Ki·∫e dt')
ax.plot(t3, d_terms, color='#4cc9f0', linewidth=1.2, label=f'D = Kd·de/dt')
ax.fill_between(t3, p_terms, alpha=0.08, color='#ff4560')
ax.set_title('Termes P, I, D', color='#ff4560', fontweight='bold')
ax.set_xlabel('Temps (s)'); ax.set_ylabel('Valeur terme')
ax.legend(fontsize=9, framealpha=0.3); ax.grid(True, alpha=0.3)

# Braquage vs erreur
ax = axes[0,1]; ax.set_facecolor('#12151c')
ax.scatter(pid_errs, pid_steers, c=pid_curvs, cmap='plasma',
           s=1.5, alpha=0.4)
e_range = np.linspace(-3,3,100)
ax.plot(e_range, [-1.8*e/3 for e in e_range], color='#ffd166',
        linewidth=2, linestyle='--', label='Proportionnel Kp·e/3')
ax.set_title('Nuage de points : Erreur → Braquage', color='#ffd166', fontweight='bold')
ax.set_xlabel('Erreur latérale'); ax.set_ylabel('Braquage PID')
ax.legend(fontsize=8, framealpha=0.3); ax.grid(True, alpha=0.3)
sm = plt.cm.ScalarMappable(cmap='plasma', norm=plt.Normalize(0,1))
plt.colorbar(sm, ax=ax, label='Courbure', fraction=0.03)

# Distribution de l'erreur
ax = axes[1,0]; ax.set_facecolor('#12151c')
bins = np.linspace(-3, 3, 40)
ax.hist(hist['fz_err'],  bins=bins, color='#00e5b0', alpha=0.6,
        label=f'Fuzzy (σ={np.std(hist["fz_err"]):.3f})', edgecolor='none')
ax.hist(hist['pid_err'], bins=bins, color='#ff4560', alpha=0.6,
        label=f'PID   (σ={np.std(hist["pid_err"]):.3f})', edgecolor='none')
ax.axvline(np.mean(hist['fz_err']),  color='#00e5b0', linewidth=2, linestyle='--')
ax.axvline(np.mean(hist['pid_err']), color='#ff4560', linewidth=2, linestyle='--')
ax.set_title('Distribution de l\'erreur latérale', color='#dde2f0', fontweight='bold')
ax.set_xlabel('Erreur latérale'); ax.set_ylabel('Fréquence')
ax.legend(fontsize=9, framealpha=0.3); ax.grid(True, alpha=0.3)

# Box plot comparatif
ax = axes[1,1]; ax.set_facecolor('#12151c')
data_box = [np.abs(hist['fz_err']), np.abs(hist['pid_err'])]
bp = ax.boxplot(data_box, labels=['Fuzzy+Trans', 'PID'],
                patch_artist=True, notch=True,
                boxprops=dict(linewidth=1.5),
                medianprops=dict(linewidth=2.5))
bp['boxes'][0].set_facecolor('#00e5b040'); bp['boxes'][0].set_edgecolor('#00e5b0')
bp['boxes'][1].set_facecolor('#ff456040'); bp['boxes'][1].set_edgecolor('#ff4560')
bp['medians'][0].set_color('#00e5b0'); bp['medians'][1].set_color('#ff4560')
for w in bp['whiskers']:  w.set_color('#7a82a0')
for c in bp['caps']:      c.set_color('#7a82a0')
ax.set_title('Box plot : |erreur| Fuzzy vs PID', color='#dde2f0', fontweight='bold')
ax.set_ylabel('|Erreur latérale|'); ax.grid(True, alpha=0.3, axis='y')
# Annotation médiane
for i, (label,color) in enumerate([('Fuzzy+Trans','#00e5b0'),('PID','#ff4560')]):
    med = np.median(np.abs(data_box[i]))
    ax.text(i+1, med+0.03, f'Médiane={med:.3f}', ha='center',
            color=color, fontsize=8, fontweight='bold')

plt.tight_layout()
plt.savefig('img/06_analyse_pid.png', dpi=150, bbox_inches='tight',
            facecolor='#0a0c11')
plt.show()
print("💾 Sauvegardé : img/06_analyse_pid.png")
""")

# ─────────────────────────────────────────────────────────────────────────────
# CELL 14 — Fuzzy surface 3D
# ─────────────────────────────────────────────────────────────────────────────
code("""# ══════════════════════════════════════════════════════
# CELLULE 14 — Surface de contrôle Fuzzy 3D
# ══════════════════════════════════════════════════════

from mpl_toolkits.mplot3d import Axes3D

fig = plt.figure(figsize=(18, 7))
fig.patch.set_facecolor('#0a0c11')
fig.suptitle('Surfaces de contrôle Fuzzy — Braquage et Vitesse',
             color='#dde2f0', fontsize=13, fontweight='bold')

e_range = np.linspace(-3, 3, 30)
c_range = np.linspace( 0, 1, 30)
E, C = np.meshgrid(e_range, c_range)
Z_steer = np.zeros_like(E)
Z_speed = np.zeros_like(E)

eng3d = FuzzyEngine()
for i in range(E.shape[0]):
    for j in range(E.shape[1]):
        out3d = eng3d.infer(E[i,j], 0.0, C[i,j])
        Z_steer[i,j] = out3d['steer']
        spd = out3d['corner_speed']*C[i,j] + out3d['straight_speed']*(1-C[i,j])
        Z_speed[i,j] = spd

# Colormap custom
cmap_steer = LinearSegmentedColormap.from_list('steer',
    ['#ff4560','#1a1a2e','#00e5b0'])
cmap_speed = LinearSegmentedColormap.from_list('speed',
    ['#ff4560','#ffd166','#00e5b0'])

# Braquage
ax1 = fig.add_subplot(121, projection='3d')
ax1.set_facecolor('#0d0f14')
surf1 = ax1.plot_surface(E, C, Z_steer, cmap=cmap_steer, alpha=0.85,
                          linewidth=0, antialiased=True)
ax1.contour(E, C, Z_steer, levels=8, cmap=cmap_steer, alpha=0.3, offset=-1.1)
fig.colorbar(surf1, ax=ax1, shrink=0.5, label='Braquage [-1,+1]')
ax1.set_xlabel('Erreur', color='#7a82a0')
ax1.set_ylabel('Courbure', color='#7a82a0')
ax1.set_zlabel('Braquage', color='#00e5b0')
ax1.set_title('Surface : Erreur×Courbure → Braquage', color='#00e5b0', pad=10)
ax1.tick_params(colors='#454d68', labelsize=7)
ax1.view_init(elev=28, azim=-55)
ax1.set_zlim(-1.1, 1.1)

# Vitesse
ax2 = fig.add_subplot(122, projection='3d')
ax2.set_facecolor('#0d0f14')
surf2 = ax2.plot_surface(E, C, Z_speed, cmap=cmap_speed, alpha=0.85,
                          linewidth=0, antialiased=True)
ax2.contour(E, C, Z_speed, levels=8, cmap=cmap_speed, alpha=0.3, offset=0.0)
fig.colorbar(surf2, ax=ax2, shrink=0.5, label='Vitesse [0,1]')
ax2.set_xlabel('Erreur', color='#7a82a0')
ax2.set_ylabel('Courbure', color='#7a82a0')
ax2.set_zlabel('Vitesse', color='#ffd166')
ax2.set_title('Surface : Erreur×Courbure → Vitesse effective', color='#ffd166', pad=10)
ax2.tick_params(colors='#454d68', labelsize=7)
ax2.view_init(elev=28, azim=-55)
ax2.set_zlim(0, 1.05)

plt.tight_layout()
plt.savefig('img/07_surface_fuzzy_3d.png', dpi=150, bbox_inches='tight',
            facecolor='#0a0c11')
plt.show()
print("💾 Sauvegardé : img/07_surface_fuzzy_3d.png")
""")

# ─────────────────────────────────────────────────────────────────────────────
# CELL 15 — Transformer analysis
# ─────────────────────────────────────────────────────────────────────────────
code("""# ══════════════════════════════════════════════════════
# CELLULE 15 — Analyse du Transformer (mémoire + tendance)
# ══════════════════════════════════════════════════════

sim4 = SimulationEngine()
N4 = 1200

trans_corr=[]; trans_conf=[]; fz_err4=[]; fz_steer_pre=[]; fz_steer_post=[]

for step in range(N4):
    bot = sim4.state_fuzzy
    err4, curv4 = sim4._sense(bot)
    rate4 = (err4 - bot.error)/sim4.DT if step>0 else 0.
    bot.error = err4; bot.error_rate = max(-2.,min(2.,rate4))

    out4 = sim4.fuzzy.infer(err4, bot.error_rate, curv4)
    steer_pre = out4['steer']
    speed4 = out4['corner_speed']*curv4 + out4['straight_speed']*(1-curv4)
    speed4 = max(0.1, min(1., speed4))

    sim4.transformer.update({'error':err4,'rate':bot.error_rate,
                              'curvature':curv4,'speed':speed4})
    pred4 = sim4.transformer.predict()
    steer_post = max(-1.,min(1., steer_pre + pred4['steer_correction']*pred4['confidence']))

    sim4._advance(bot, steer_post, speed4)
    bot.steer=steer_post; bot.speed=speed4; bot.steps+=1
    sim4.step_pid()

    trans_corr.append(pred4['steer_correction']*pred4['confidence'])
    trans_conf.append(pred4['confidence'])
    fz_err4.append(err4)
    fz_steer_pre.append(steer_pre)
    fz_steer_post.append(steer_post)

t4 = np.arange(N4)/60.

fig, axes = plt.subplots(2, 2, figsize=(16, 9))
fig.patch.set_facecolor('#0a0c11')
fig.suptitle('Analyse du Transformer — Mémoire & Correction par tendance',
             color='#9b72cf', fontsize=13, fontweight='bold')

# Correction Transformer
ax = axes[0,0]; ax.set_facecolor('#12151c')
ax.plot(t4, trans_corr, color='#9b72cf', linewidth=1.2, label='Correction Transformer')
ax.fill_between(t4, trans_corr, alpha=0.1, color='#9b72cf')
ax.axhline(0, color='white', alpha=0.2, linewidth=0.5)
ax.set_title('Correction du Transformer (pondérée par confiance)', color='#9b72cf')
ax.set_xlabel('Temps (s)'); ax.set_ylabel('Correction braquage')
ax.legend(fontsize=9, framealpha=0.3); ax.grid(True, alpha=0.3)

# Confiance
ax = axes[0,1]; ax.set_facecolor('#12151c')
ax.plot(t4, trans_conf, color='#ffd166', linewidth=1.2, label='Confiance')
ax.fill_between(t4, trans_conf, alpha=0.1, color='#ffd166')
ax.set_title('Niveau de confiance du Transformer [0, 1]', color='#ffd166')
ax.set_xlabel('Temps (s)'); ax.set_ylabel('Confiance')
ax.set_ylim(-0.05, 1.0); ax.legend(fontsize=9, framealpha=0.3); ax.grid(True, alpha=0.3)

# Braquage avant/après correction
ax = axes[1,0]; ax.set_facecolor('#12151c')
ax.plot(t4, fz_steer_pre,  color='#4cc9f0', linewidth=1.0,
        label='Avant Transformer (Fuzzy pur)', alpha=0.7)
ax.plot(t4, fz_steer_post, color='#00e5b0', linewidth=1.3,
        label='Après Transformer', alpha=0.9)
diff = [b-a for a,b in zip(fz_steer_pre, fz_steer_post)]
ax.fill_between(t4, fz_steer_pre, fz_steer_post,
                alpha=0.2, color='#9b72cf', label='Δ Correction')
ax.set_title('Braquage Fuzzy avant/après correction Transformer', color='#4cc9f0')
ax.set_xlabel('Temps (s)'); ax.set_ylabel('Braquage')
ax.legend(fontsize=8, framealpha=0.3); ax.grid(True, alpha=0.3)

# Histogramme des corrections
ax = axes[1,1]; ax.set_facecolor('#12151c')
diff_arr = np.array(diff)
non_zero = diff_arr[np.abs(diff_arr) > 0.001]
if len(non_zero) > 10:
    ax.hist(non_zero, bins=30, color='#9b72cf', alpha=0.7, edgecolor='none')
    ax.axvline(np.mean(non_zero), color='#ffd166', linewidth=2,
               label=f'Moyenne = {np.mean(non_zero):.4f}')
    ax.axvline(0, color='white', alpha=0.3, linewidth=1)
    ax.set_title(f'Distribution des corrections Transformer\\n({len(non_zero)} corrections non-nulles)',
                 color='#9b72cf')
else:
    ax.text(0.5, 0.5, 'Pas assez de\\ncorrections non-nulles\\n(tendance trop faible)',
            ha='center', va='center', transform=ax.transAxes,
            color='#7a82a0', fontsize=10)
    ax.set_title('Distribution des corrections Transformer', color='#9b72cf')
ax.set_xlabel('Correction braquage'); ax.set_ylabel('Fréquence')
ax.legend(fontsize=9, framealpha=0.3); ax.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig('img/08_analyse_transformer.png', dpi=150, bbox_inches='tight',
            facecolor='#0a0c11')
plt.show()
print("💾 Sauvegardé : img/08_analyse_transformer.png")
""")

# ─────────────────────────────────────────────────────────────────────────────
# CELL 16 — Final summary dashboard
# ─────────────────────────────────────────────────────────────────────────────
md("""---
## 📊 Partie 7 — Tableau de bord final (Comparaison complète)
""")

code("""# ══════════════════════════════════════════════════════
# CELLULE 16 — Dashboard récapitulatif
# ══════════════════════════════════════════════════════

fz_f2  = sim.state_fuzzy
pid_f2 = sim.state_pid
fze_m  = fz_f2.total_error/max(1,fz_f2.steps)
pide_m = pid_f2.total_error/max(1,pid_f2.steps)
fz_std  = np.std(hist['fz_err'])
pid_std = np.std(hist['pid_err'])
gain_pct = (pide_m - fze_m)/max(pide_m,1e-9)*100

fig = plt.figure(figsize=(18, 12))
fig.patch.set_facecolor('#0a0c11')
gs = GridSpec(3, 4, figure=fig, hspace=0.4, wspace=0.35)

# ── Titre ─────────────────────────────────────────────
ax_title = fig.add_subplot(gs[0, :])
ax_title.set_facecolor('#12151c')
ax_title.text(0.5, 0.7, '🏎  TABLEAU DE BORD — FUZZY+TRANSFORMER vs PID',
              ha='center', va='center', fontsize=16, fontweight='bold',
              color='#dde2f0', transform=ax_title.transAxes)
ax_title.text(0.5, 0.2,
    f'Simulation {N_STEPS} pas ({N_STEPS/60:.0f}s) · 60 Hz · '
    f'Fuzzy: 21 règles Mamdani · Transformer: mémoire 16 états',
    ha='center', va='center', fontsize=10, color='#7a82a0',
    transform=ax_title.transAxes)
ax_title.axis('off')

def metric_box(ax, title, fz_val, pid_val, unit='', lower_is_better=True):
    ax.set_facecolor('#12151c')
    ax.axis('off')
    fz_better = (fz_val < pid_val) == lower_is_better
    fz_col  = '#00e5b0' if fz_better else '#ff4560'
    pid_col = '#ff4560' if fz_better else '#00e5b0'
    ax.text(0.5, 0.88, title, ha='center', va='top', fontsize=9,
            color='#7a82a0', transform=ax.transAxes)
    ax.text(0.25, 0.5, f'{fz_val:.3f}{unit}',  ha='center', va='center',
            fontsize=14, fontweight='bold', color=fz_col, transform=ax.transAxes)
    ax.text(0.75, 0.5, f'{pid_val:.3f}{unit}', ha='center', va='center',
            fontsize=14, fontweight='bold', color=pid_col, transform=ax.transAxes)
    ax.text(0.25, 0.12, 'FUZZY+T', ha='center', fontsize=7,
            color=fz_col, transform=ax.transAxes)
    ax.text(0.75, 0.12, 'PID', ha='center', fontsize=7,
            color=pid_col, transform=ax.transAxes)
    if fz_better:
        ax.text(0.5, 0.5, '→', ha='center', va='center', fontsize=10,
                color='#00e5b0', transform=ax.transAxes)
    border_col = '#00e5b0' if fz_better else '#ff4560'
    for spine in ax.spines.values():
        spine.set_edgecolor(border_col); spine.set_linewidth(1.5); spine.set_visible(True)

# Metric boxes
metric_box(fig.add_subplot(gs[1,0]), 'Erreur moyenne',     fze_m, pide_m)
metric_box(fig.add_subplot(gs[1,1]), 'Écart-type erreur',  fz_std, pid_std)
metric_box(fig.add_subplot(gs[1,2]), 'Tours complétés',    fz_f2.laps, pid_f2.laps,
           lower_is_better=False)
metric_box(fig.add_subplot(gs[1,3]), 'Sorties de piste',   fz_f2.lost_count, pid_f2.lost_count)

# ── Mini trajectoire ──────────────────────────────────
ax_traj = fig.add_subplot(gs[2, 0:2])
ax_traj.set_facecolor('#0d0f14')
# Piste simplifiée
xs2 = [p[0] for p in sim.track.waypoints] + [sim.track.waypoints[0][0]]
ys2 = [p[1] for p in sim.track.waypoints] + [sim.track.waypoints[0][1]]
ax_traj.plot(xs2, ys2, color='#3a4060', linewidth=8, solid_capstyle='round')
ax_traj.plot(xs2, ys2, color='white', linewidth=0.5, alpha=0.3, linestyle='--')
N_show = min(len(hist['fz_x']), 4000)
ax_traj.plot(hist['fz_x'][:N_show],  hist['fz_y'][:N_show],
             color='#00e5b0', linewidth=1.2, alpha=0.75, label='Fuzzy+Trans')
ax_traj.plot(hist['pid_x'][:N_show], hist['pid_y'][:N_show],
             color='#ff4560', linewidth=1.2, alpha=0.75, label='PID')
ax_traj.set_title('Trajectoires sur l\'autodrome', color='#dde2f0', fontsize=10)
ax_traj.legend(fontsize=8, framealpha=0.3)
ax_traj.set_aspect('equal'); ax_traj.axis('off')

# ── Erreur IAE ────────────────────────────────────────
ax_iae = fig.add_subplot(gs[2, 2:4])
ax_iae.set_facecolor('#12151c')
fz_iae  = np.cumsum(np.abs(hist['fz_err']))
pid_iae = np.cumsum(np.abs(hist['pid_err']))
t_plot  = np.arange(len(fz_iae))/60.
ax_iae.plot(t_plot, fz_iae,  color='#00e5b0', linewidth=2, label='Fuzzy+Trans (IAE)')
ax_iae.plot(t_plot, pid_iae, color='#ff4560', linewidth=2, label='PID (IAE)')
ax_iae.fill_between(t_plot, fz_iae, pid_iae,
                    where=pid_iae>=fz_iae, alpha=0.1, color='#00e5b0')
ax_iae.set_title(f'IAE cumulé · Réduction Fuzzy : {gain_pct:.1f}%',
                 color='#dde2f0', fontsize=10)
ax_iae.set_xlabel('Temps (s)'); ax_iae.set_ylabel('Σ|erreur|')
ax_iae.legend(fontsize=9, framealpha=0.3); ax_iae.grid(True, alpha=0.3)
ax_iae.text(0.98, 0.08,
    f'Fuzzy IAE finale: {fz_iae[-1]:.1f}\\nPID   IAE finale: {pid_iae[-1]:.1f}\\n'
    f'Gain: {gain_pct:.1f}%',
    transform=ax_iae.transAxes, ha='right', va='bottom',
    fontsize=9, color='#00e5b0', fontweight='bold',
    bbox=dict(boxstyle='round', facecolor='#0a0c11', alpha=0.8))

plt.savefig('img/09_dashboard_final.png', dpi=150, bbox_inches='tight',
            facecolor='#0a0c11')
plt.show()
print("💾 Sauvegardé : img/09_dashboard_final.png")
""")

# ─────────────────────────────────────────────────────────────────────────────
# CELL 17 — Summary
# ─────────────────────────────────────────────────────────────────────────────
md("""---
## ✅ Partie 8 — Récapitulatif et fichiers sauvegardés
""")

code("""# ══════════════════════════════════════════════════════
# CELLULE 17 — Récapitulatif final
# ══════════════════════════════════════════════════════
import os

print("╔══════════════════════════════════════════════════════════════════════════╗")
print("║  RÉSUMÉ FINAL — Line Follower Fuzzy+Transformer vs PID                  ║")
print("╚══════════════════════════════════════════════════════════════════════════╝")
print()
fz_e  = sim.state_fuzzy.total_error/max(1,sim.state_fuzzy.steps)
pid_e = sim.state_pid.total_error/max(1,sim.state_pid.steps)
gain  = (pid_e - fz_e)/max(pid_e,1e-9)*100
print(f"  {'Métrique':<30} {'Fuzzy+Transformer':>18}  {'PID classique':>14}")
print("  " + "─"*64)
print(f"  {'Erreur latérale moyenne':<30} {fz_e:>18.4f}  {pid_e:>14.4f}")
print(f"  {'Écart-type erreur':<30} {np.std(hist['fz_err']):>18.4f}  {np.std(hist['pid_err']):>14.4f}")
print(f"  {'Tours complétés':<30} {sim.state_fuzzy.laps:>18}  {sim.state_pid.laps:>14}")
print(f"  {'Sorties de piste':<30} {sim.state_fuzzy.lost_count:>18}  {sim.state_pid.lost_count:>14}")
print(f"  {'IAE cumulée':<30} {np.sum(np.abs(hist['fz_err'])):>18.1f}  {np.sum(np.abs(hist['pid_err'])):>14.1f}")
print()
verdict = "✅ FUZZY+TRANSFORMER SUPÉRIEUR" if gain>5 else ("⚠️  PERFORMANCES COMPARABLES" if abs(gain)<5 else "⚠️  PID supérieur (ajuster règles)")
print(f"  → {verdict}  (gain sur erreur : {gain:.1f}%)")
print()
print("─"*70)
print("  📁 Images sauvegardées dans le dossier img/ :")
for f in sorted(os.listdir('img')):
    size = os.path.getsize(f'img/{f}') // 1024
    print(f"     {f:<45}  {size:>4} Ko")
print()
print("  📋 Architecture du système Fuzzy :")
print(f"     • Fonctions d'appartenance : 7+5+3 = 15 ensembles")
print(f"     • Base de règles           : {len(FUZZY_RULES)} règles Mamdani")
print(f"     • Défuzzification          : COG sur singletons")
print(f"     • Transformer              : mémoire {16} états, correction ±0.15")
print(f"     • Variables floues clés    : vitesse_virage + vitesse_droite")
""")

nb.cells = cells

with open('/home/claude/line_follower_notebook.ipynb', 'w', encoding='utf-8') as f:
    nbf.write(nb, f)

print(f"✅ Notebook créé : {len(cells)} cellules")
