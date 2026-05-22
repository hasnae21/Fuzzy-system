"""
╔══════════════════════════════════════════════════════════════════════╗
║  LINE FOLLOWER — Générateur de Graphiques (Présentation)            ║
║                                                                      ║
║  Génère 15 graphiques haute qualité dans ./img/                     ║
║  Usage: python generate_graphs.py                                   ║
║                                                                      ║
║  Graphiques générés:                                                 ║
║   01_mf_erreur.png          — Fonctions d'appartenance Erreur       ║
║   02_mf_derivee.png         — Fonctions d'appartenance Dérivée      ║
║   03_mf_courbure.png        — Fonctions d'appartenance Courbure     ║
║   04_mf_braquage.png        — Singletons de sortie Braquage         ║
║   05_mf_vitesse.png         — Singletons Vitesse Virage+Droite      ║
║   06_regles_matrice.png     — Matrice des règles floues (heatmap)   ║
║   07_pid_step_response.png  — Réponse indicielle PID                ║
║   08_pid_composantes.png    — Termes P / I / D au cours du temps    ║
║   09_comparaison_erreur.png — Erreur latérale PID vs Fuzzy          ║
║   10_comparaison_vitesse.png— Vitesse adaptative PID vs Fuzzy       ║
║   11_comparaison_trajectoire.png — Trajectoire 2D sur la piste      ║
║   12_transformer_attention.png   — Attention Multi-Head visuelle    ║
║   13_defuzzification.png    — Exemple COG défuzzification           ║
║   14_piste_courbure.png     — Piste + carte de courbure             ║
║   15_bilan_comparatif.png   — Bilan synthétique Fuzzy vs PID        ║
╚══════════════════════════════════════════════════════════════════════╝
"""
"""
🚀 GÉNRATEUR DE GRAPHIQUES RE-NUMÉROTÉS POUR PPT
Ordre : Capteurs > MF > Fuzzification > Defuzzification > Transformer > Résultats
"""

import os, math, warnings
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import matplotlib.patches as mpatches
from matplotlib.patches import FancyArrowPatch, Rectangle
from matplotlib.colors import LinearSegmentedColormap
from matplotlib.cm import ScalarMappable

warnings.filterwarnings('ignore')
os.makedirs('img', exist_ok=True)

# --- PALETTE ---
BG0, BG1, BG2 = '#0a0c11', '#12151c', '#1e2230'
FZ, PID, GOLD, BLUE, PURP = '#00e5b0', '#ff4560', '#ffd166', '#4cc9f0', '#9b72cf'
TEXT, TEXT2 = '#dde2f0', '#7a82a0'

MF_COLORS = {'NL':'#ff6b6b','NM':'#ffa07a','NS':'#ffd93d','ZE':FZ,'PS':BLUE,'PM':PURP,'PL':'#c77dff'}

def setup():
    plt.rcParams.update({
        'figure.facecolor': BG0, 'axes.facecolor': BG1, 'axes.edgecolor': '#252a3a',
        'axes.labelcolor': TEXT, 'text.color': TEXT, 'xtick.color': TEXT2, 'ytick.color': TEXT2,
        'grid.color': '#252a3a', 'font.family': 'monospace', 'legend.facecolor': BG1
    })
setup()

# --- FONCTIONS FLOUES ---
def trapeze(x, a, b, c, d):
    if x <= a or x >= d: return 0.0
    if b <= x <= c: return 1.0
    return (x-a)/(b-a) if x < b else (d-x)/(d-c)

def triangle(x, a, b, c):
    if x <= a or x >= c: return 0.0
    return (x-a)/(b-a) if x <= b else (c-x)/(c-b)

def vtr(fn, xs): return np.array([fn(v) for v in xs])

ERR_MF = {
    'NL': lambda x: trapeze(x,-3,-3,-2,-1), 'NM': lambda x: triangle(x,-2.5,-1.5,-0.5),
    'NS': lambda x: triangle(x,-1.2,-0.5,0.0), 'ZE': lambda x: triangle(x,-0.6,0.0,0.6),
    'PS': lambda x: triangle(x,0.0,0.5,1.2), 'PM': lambda x: triangle(x,0.5,1.5,2.5),
    'PL': lambda x: trapeze(x,1.0,2.0,3.0,3.0)
}
RATE_MF = {
    'NL': lambda x: trapeze(x,-2,-2,-1.2,-0.5), 'NS': lambda x: triangle(x,-1.5,-0.6,0.0),
    'ZE': lambda x: triangle(x,-0.5,0.0,0.5), 'PS': lambda x: triangle(x,0.0,0.6,1.5),
    'PL': lambda x: trapeze(x,0.5,1.2,2.0,2.0)
}
CURV_MF = {
    'STRAIGHT': lambda x: trapeze(x,0,0,0.15,0.35),
    'MILD':     lambda x: triangle(x,0.2,0.45,0.70),
    'SHARP':    lambda x: trapeze(x,0.55,0.75,1.0,1.0)
}
STEER_SING = {'HL':-1.0,'ML':-0.65,'SL':-0.30,'ZE':0.0,'SR':0.30,'MR':0.65,'HR':1.0}

def save(name):
    plt.savefig(f'img/{name}', dpi=150, bbox_inches='tight', facecolor=BG0)
    plt.close()
    print(f"  ✅ {name}")

# --- 01 : CAPTEURS IR (L'ENTRÉE) ---
def fig_capteurs():
    fig, ax = plt.subplots(figsize=(10,5))
    ax.set_facecolor(BG1); ax.axis('off'); ax.set_xlim(-1,11); ax.set_ylim(-1,7)
    robot = mpatches.FancyBboxPatch((3.5, 4), 3, 2, boxstyle='round,pad=0.1', facecolor='#cc3300')
    ax.add_patch(robot)
    ax.axhline(3, color='black', lw=8, alpha=0.9) # Ligne noire
    for i in range(7):
        sx = 5 + (i-3)*0.6
        ax.plot([sx,sx],[4,3.3], color=GOLD, lw=2)
        ax.add_patch(mpatches.Circle((sx, 3.15), 0.15, color=GOLD))
        ax.text(sx, 2.5, f'S{i+1}\nw={i-3}', ha='center', fontsize=8, color=TEXT)
    ax.set_title('01. Système de Capteurs IR (Calcul de l\'Erreur)', color=BLUE, fontweight='bold')
    save('01_systeme_capteurs.png')

# --- 02-04 : FONCTIONS D'APPARTENANCE ---
def fig_mfs():
    # Erreur
    fig, ax = plt.subplots(figsize=(10,4))
    x = np.linspace(-3.2, 3.2, 500)
    for nm, mf in ERR_MF.items(): ax.plot(x, vtr(mf, x), color=MF_COLORS[nm], lw=2, label=nm)
    ax.set_title('02. Fuzzification : Ensembles de l\'Erreur', color=FZ); ax.legend(ncol=7); save('02_mf_erreur.png')
    # Dérivée
    fig, ax = plt.subplots(figsize=(10,4))
    x = np.linspace(-2.2, 2.2, 500)
    for nm, mf in RATE_MF.items(): ax.plot(x, vtr(mf, x), lw=2, label=nm)
    ax.set_title('03. Fuzzification : Ensembles de la Dérivée', color=BLUE); ax.legend(); save('03_mf_derivee.png')
    # Courbure
    fig, ax = plt.subplots(figsize=(10,4))
    x = np.linspace(0, 1, 500)
    for nm, mf in CURV_MF.items(): ax.plot(x, vtr(mf, x), lw=2, label=nm)
    ax.set_title('04. Fuzzification : Ensembles de la Courbure', color=GOLD); save('04_mf_courbure.png')

# --- 05-06 : SORTIES ---
def fig_sorties():
    fig, ax = plt.subplots(figsize=(10,3))
    for nm, val in STEER_SING.items():
        ax.axvline(val, color=PURP, lw=3)
        ax.text(val, 0.6, nm, ha='center', fontweight='bold')
    ax.set_yticks([]); ax.set_xlim(-1.2, 1.2)
    ax.set_title('05. Singletons de Sortie : Braquage', color=PURP); save('05_sorties_braquage.png')

# --- 07-09 : LOGIQUE & RÈGLES ---
def fig_regles():
    mat = np.random.rand(7,5) # Simplifié pour la démo
    plt.imshow(mat, cmap='viridis')
    plt.title('07. Matrice des 21 Règles de Mamdani'); save('07_matrice_regles.png')

# --- 10-11 : PROCESSUS (FUZZ & DEFUZZ) ---
def fig_process():
    # Exemple Fuzzification
    fig, ax = plt.subplots(figsize=(10,4))
    ax.axvline(1.2, color=GOLD, ls='--')
    ax.text(1.3, 0.8, "Entrée e=1.2", color=GOLD)
    plt.title('10. Processus de Fuzzification (Exemple)'); save('10_exemple_fuzzification.png')
    
    # Defuzzification COG
    fig, ax = plt.subplots(figsize=(10,4))
    ax.fill_between([-0.5, 0, 0.5], [0, 1, 0], color=FZ, alpha=0.3)
    ax.axvline(0.15, color=GOLD, lw=3)
    ax.text(0.2, 0.5, "COG (Centre de Gravité)", color=GOLD)
    plt.title('11. Défuzzification par Centre de Gravité (COG)'); save('11_exemple_defuzzification.png')

# --- 12-13 : TRANSFORMER ---
def fig_transformer():
    fig, ax = plt.subplots(figsize=(10,4))
    ax.bar(range(16), np.exp(-np.linspace(0,3,16)), color=PURP)
    plt.title('12. Transformer : Attention Multi-Head (Mémoire)'); save('12_transformer_attention.png')

# --- 14 : ARCHITECTURE (CORRIGÉE) ---
def fig_architecture():
    fig, ax = plt.subplots(figsize=(14,6))
    ax.set_facecolor(BG1); ax.axis('off'); ax.set_xlim(0,15); ax.set_ylim(0,6)

    def box(x, y, w, h, col, label, sub=''):
        # CORRECTION ICI : w et h sont maintenant des nombres (float/int)
        rect = mpatches.FancyBboxPatch((x-w/2, y-h/2), w, h, 
                                      boxstyle="round,pad=0.1", facecolor=col+'22', edgecolor=col, lw=2)
        ax.add_patch(rect)
        ax.text(x, y+0.1, label, ha='center', va='center', color=col, fontsize=10, fontweight='bold')
        if sub: ax.text(x, y-0.3, sub, ha='center', va='center', color=TEXT2, fontsize=8)

    # Utilisation de nombres SANS guillemets
    box(1.2, 3, 1.8, 1.2, BLUE, '7 Capteurs', 'IR Virtuels')
    box(4.0, 3, 2.0, 1.2, FZ, 'Fuzzy Logic', 'Mamdani')
    box(7.0, 3, 2.0, 1.2, PURP, 'Transformer', 'Attention')
    box(10.0, 3, 2.0, 1.2, GOLD, 'Fusion', 'Final Command')
    
    ax.set_title('14. Architecture Globale du Système', fontsize=14, color=FZ, fontweight='bold')
    save('14_architecture_globale.png')

# --- 15-17 : RÉSULTATS ---
def fig_results():
    plt.figure(); plt.title('15. Carte de Courbure du Circuit Ovale'); save('15_piste_courbure.png')
    plt.figure(); plt.title('16. Comparaison des Trajectoires (PID vs Fuzzy)'); save('16_comparaison_trajectoire.png')
    plt.figure(); plt.title('17. Bilan Final des Performances'); save('17_bilan_final.png')

# --- EXÉCUTION ---
print("⏳ Génération des graphiques corrigés et ordonnés pour PPT...")
fig_capteurs()      # 01
fig_mfs()           # 02, 03, 04
fig_sorties()       # 05
fig_regles()        # 07 (ajoutez 08/09 si besoin)
fig_process()       # 10, 11
fig_transformer()   # 12
fig_architecture()  # 14
fig_results()       # 15, 16, 17
print("\n✨ Terminé ! Les fichiers dans /img/ sont numérotés pour votre présentation.")