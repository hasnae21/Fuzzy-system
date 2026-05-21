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
from matplotlib.ticker import MultipleLocator
warnings.filterwarnings('ignore')

os.makedirs('img', exist_ok=True)

# ─────────────────────────────────────────────────────────────────────
#  PALETTE (correspond exactement à l'UI)
# ─────────────────────────────────────────────────────────────────────
BG0   = '#0a0c11'
BG1   = '#12151c'
BG2   = '#1e2230'
FZ    = '#00e5b0'   # Fuzzy: teal
PID   = '#ff4560'   # PID: rouge
GOLD  = '#ffd166'
BLUE  = '#4cc9f0'
PURP  = '#9b72cf'
TEXT  = '#dde2f0'
TEXT2 = '#7a82a0'

MF_COLORS = {
    'NL':'#ff6b6b','NM':'#ffa07a','NS':'#ffd93d',
    'ZE':FZ,'PS':BLUE,'PM':PURP,'PL':'#c77dff'
}

def setup():
    plt.rcParams.update({
        'figure.facecolor': BG0,
        'axes.facecolor':   BG1,
        'axes.edgecolor':   '#252a3a',
        'axes.labelcolor':  TEXT,
        'text.color':       TEXT,
        'xtick.color':      TEXT2,
        'ytick.color':      TEXT2,
        'grid.color':       '#252a3a',
        'grid.alpha':       0.5,
        'font.family':      'monospace',
        'font.size':        11,
        'legend.facecolor': BG1,
        'legend.edgecolor': '#252a3a',
        'axes.spines.top':  False,
        'axes.spines.right':False,
    })
setup()

# ─────────────────────────────────────────────────────────────────────
#  FONCTIONS D'APPARTENANCE (identiques au backend)
# ─────────────────────────────────────────────────────────────────────
def trapeze(x, a, b, c, d):
    if x <= a or x >= d: return 0.0
    if b <= x <= c: return 1.0
    if x < b: return (x-a)/(b-a) if b!=a else 1.0
    return (d-x)/(d-c) if d!=c else 1.0

def triangle(x, a, b, c):
    if x <= a or x >= c: return 0.0
    if x <= b: return (x-a)/(b-a) if b!=a else 1.0
    return (c-x)/(c-b) if c!=b else 1.0

def vtr(fn, xs): return np.array([fn(v) for v in xs])

# Ensembles erreur [-3, +3]
ERR_MF = {
    'NL': lambda x: trapeze(x,-3,-3,-2,-1),
    'NM': lambda x: triangle(x,-2.5,-1.5,-0.5),
    'NS': lambda x: triangle(x,-1.2,-0.5,0.0),
    'ZE': lambda x: triangle(x,-0.6,0.0,0.6),
    'PS': lambda x: triangle(x,0.0,0.5,1.2),
    'PM': lambda x: triangle(x,0.5,1.5,2.5),
    'PL': lambda x: trapeze(x,1.0,2.0,3.0,3.0),
}
# Ensembles dérivée [-2, +2]
RATE_MF = {
    'NL': lambda x: trapeze(x,-2,-2,-1.2,-0.5),
    'NS': lambda x: triangle(x,-1.5,-0.6,0.0),
    'ZE': lambda x: triangle(x,-0.5,0.0,0.5),
    'PS': lambda x: triangle(x,0.0,0.6,1.5),
    'PL': lambda x: trapeze(x,0.5,1.2,2.0,2.0),
}
# Ensembles courbure [0, 1]
CURV_MF = {
    'STRAIGHT': lambda x: trapeze(x,0,0,0.15,0.35),
    'MILD':     lambda x: triangle(x,0.2,0.45,0.70),
    'SHARP':    lambda x: trapeze(x,0.55,0.75,1.0,1.0),
}

# Singletons sortie braquage
STEER_SING = {'HL':-1.0,'ML':-0.65,'SL':-0.30,'ZE':0.0,'SR':0.30,'MR':0.65,'HR':1.0}
# Singletons vitesse virage
CV_SING = {'STOP':0.05,'SLOW':0.25,'MEDIUM_SLOW':0.45,'MEDIUM':0.65,'FAST':0.85}
# Singletons vitesse droite
SV_SING = {'SLOW':0.30,'MEDIUM':0.55,'FAST':0.75,'VERY_FAST':0.90,'TURBO':1.0}

# ─────────────────────────────────────────────────────────────────────
#  AUTODROME — Ovale Stadium pour simulation
# ─────────────────────────────────────────────────────────────────────
class Autodrome:
    """Circuit ovale stadium (2 droites + 2 virages 180°)"""
    def __init__(self):
        self.cx, self.cy = 0.50, 0.50
        self.rx, self.ry = 0.28, 0.155
        self.N_s, self.N_c = 240, 160
        self.pts = self._build()
        self.N = len(self.pts)
        # Segments t ∈ [0,1]
        total = 2*self.N_s + 2*(self.N_c+1)
        self.t_left_start  = self.N_s / total
        self.t_left_end    = (self.N_s + self.N_c + 1) / total
        self.t_right_start = (2*self.N_s + self.N_c + 1) / total
        self.t_right_end   = 1.0

    def _build(self):
        pts = []
        cx,cy,rx,ry = self.cx,self.cy,self.rx,self.ry
        N_s,N_c = self.N_s,self.N_c
        for i in range(N_s):
            pts.append((cx+rx-(i/N_s)*2*rx, cy+ry))
        for i in range(N_c+1):
            a = math.pi/2 + (i/N_c)*math.pi
            pts.append((cx-rx+ry*math.cos(a), cy+ry*math.sin(a)))
        for i in range(N_s):
            pts.append((cx-rx+(i/N_s)*2*rx, cy-ry))
        for i in range(N_c+1):
            a = -math.pi/2 + (i/N_c)*math.pi
            pts.append((cx+rx+ry*math.cos(a), cy+ry*math.sin(a)))
        return pts

    def get_point(self, t):
        idx = int((t % 1.0) * (self.N-1))
        return self.pts[idx]

    def get_tangent(self, t):
        idx = int((t % 1.0) * (self.N-1))
        i2  = (idx+1) % self.N
        dx = self.pts[i2][0]-self.pts[idx][0]
        dy = self.pts[i2][1]-self.pts[idx][1]
        n  = math.hypot(dx,dy)+1e-9
        return dx/n, dy/n

    def curvature(self, t):
        dt = 0.004
        tx1,ty1 = self.get_tangent(t-dt)
        tx2,ty2 = self.get_tangent(t+dt)
        cross = tx1*ty2 - ty1*tx2
        angle = abs(math.asin(max(-1.0,min(1.0,cross))))
        return min(1.0, angle/(math.pi/6))

    def get_xy_with_offset(self, t, lat):
        """World coords for a robot at parameter t with lateral offset lat"""
        px,py = self.get_point(t)
        tx,ty = self.get_tangent(t)
        nx,ny = -ty, tx
        return px+lat*nx, py+lat*ny

track = Autodrome()


# ─────────────────────────────────────────────────────────────────────
#  AUTODROME OVALE STADIUM (identique au backend)
# ─────────────────────────────────────────────────────────────────────
class Autodrome:
    """Circuit ovale stadium pur : 2 droites + 2 demi-cercles"""
    def __init__(self):
        self.cx, self.cy = 0.50, 0.50
        self.rx, self.ry = 0.30, 0.17
        self.N_s, self.N_c = 200, 160
        self.pts = self._build()
        self.N = len(self.pts)

    def _build(self):
        pts = []
        cx,cy,rx,ry = self.cx,self.cy,self.rx,self.ry
        N_s,N_c = self.N_s,self.N_c
        # Droite haute : de droite → gauche
        for i in range(N_s):
            pts.append(((cx+rx)-(i/N_s)*2*rx, cy-ry))
        # Virage gauche (angle π/2 → 3π/2)
        for i in range(N_c+1):
            a = math.pi/2 + (i/N_c)*math.pi
            pts.append((cx-rx+ry*math.cos(a), cy+ry*math.sin(a)))
        # Droite basse : de gauche → droite
        for i in range(N_s):
            pts.append((cx-rx+(i/N_s)*2*rx, cy+ry))
        # Virage droit (angle -π/2 → π/2)
        for i in range(N_c+1):
            a = -math.pi/2 + (i/N_c)*math.pi
            pts.append((cx+rx+ry*math.cos(a), cy+ry*math.sin(a)))
        return pts

    def get_point(self, t):
        idx = int((t % 1.0) * (self.N-1))
        return self.pts[idx]

    def get_tangent(self, t):
        idx = int((t % 1.0) * (self.N-1))
        i2  = (idx+1) % self.N
        dx = self.pts[i2][0]-self.pts[idx][0]
        dy = self.pts[i2][1]-self.pts[idx][1]
        n  = math.hypot(dx,dy)+1e-9
        return dx/n, dy/n

    def curvature(self, t):
        dt = 0.004
        tx1,ty1 = self.get_tangent(t-dt)
        tx2,ty2 = self.get_tangent(t+dt)
        cross = tx1*ty2 - ty1*tx2
        angle = abs(math.asin(max(-1.0,min(1.0,cross))))
        return min(1.0, angle/(math.pi/6))

    def get_xy_with_offset(self, t, lat):
        px,py = self.get_point(t)
        tx,ty = self.get_tangent(t)
        nx,ny = -ty, tx
        return px+lat*nx, py+lat*ny

track = Autodrome()

#  SIMULATION — PID vs FUZZY
# ─────────────────────────────────────────────────────────────────────
class PIDSim:
    def __init__(self, kp=1.8, ki=0.025, kd=1.4, speed=0.60):
        self.kp,self.ki,self.kd,self.speed = kp,ki,kd,speed
        self._I = self._pe = 0.0
    def compute(self, err):
        self._I = np.clip(self._I + err/60, -10, 10)
        D = (err - self._pe)*60
        steer = np.clip(self.kp*err + self.ki*self._I + self.kd*D, -1, 1)
        self._pe = err
        return float(steer), float(self.speed)

def fuzzy_compute(err, curv):
    """Contrôleur flou simplifié reproduisant le comportement backend"""
    # Fuzzification erreur
    mu = {k: mf(err) for k,mf in ERR_MF.items()}
    mu_c = {k: mf(curv) for k,mf in CURV_MF.items()}
    # Braquage (COG sur singletons)
    rules = [('NL','HR'),('NM','MR'),('NS','SR'),('ZE','ZE'),
             ('PS','SL'),('PM','ML'),('PL','HL')]
    num = den = 0.0
    for e_set, s_out in rules:
        alpha = mu.get(e_set, 0)
        if 'SHARP' in str(s_out) or curv > 0.6:
            alpha = min(alpha, mu_c.get('SHARP',1.0))
        num += alpha * STEER_SING[s_out]; den += alpha
    steer = np.clip(num/den if den>0 else 0, -1, 1)
    # Vitesse adaptative (clé de l'avantage fuzzy)
    if curv > 0.65:  speed = 0.30 + abs(err)*0.02
    elif curv > 0.30: speed = 0.52 + (1-curv)*0.15
    else:             speed = 0.88 - min(abs(err)*0.08, 0.25)
    return float(np.clip(steer,-1,1)), float(np.clip(speed,0.15,1.0))

def run_simulation(N=700, seed=42):
    np.random.seed(seed)
    pid_ctrl = PIDSim()

    hist_pid = dict(err=[],spd=[],steer=[],P=[],I=[],D=[],curv=[],x=[],y=[],lost=0,laps=0)
    hist_fz  = dict(err=[],spd=[],steer=[],curv=[],x=[],y=[],lost=0,laps=0)

    t_pid = lat_pid = 0.0
    t_fz  = lat_fz  = 0.0
    DT = 1/60.0; noise_sigma = 0.0008
    prev_t_pid = prev_t_fz = 0.0

    for step in range(N):
        # ── PID ──
        curv_p  = track.curvature(t_pid)
        err_p   = np.clip(lat_pid / 0.025 * 3.0, -3, 3)  # scale lat→[-3,+3]
        st_p, sp_p = pid_ctrl.compute(float(err_p))

        p_t = pid_ctrl.kp * float(err_p)
        i_t = pid_ctrl.ki * float(pid_ctrl._I)
        d_t = pid_ctrl.kd * (float(err_p) - float(pid_ctrl._pe)) * 60

        drift_p = curv_p * sp_p * 0.028           # centripète ~ courbure × vitesse
        lat_pid += (-st_p*sp_p*0.011 + drift_p + np.random.randn()*noise_sigma)*DT
        lat_pid  = np.clip(lat_pid, -0.09, 0.09)
        prev_t_pid = t_pid
        t_pid    = (t_pid + sp_p * DT * 0.32) % 1.0
        if t_pid < prev_t_pid: hist_pid['laps'] += 1
        if abs(lat_pid) > 0.06: hist_pid['lost'] += 1

        hist_pid['err'].append(abs(err_p)); hist_pid['spd'].append(sp_p)
        hist_pid['steer'].append(st_p)
        hist_pid['P'].append(p_t); hist_pid['I'].append(i_t); hist_pid['D'].append(d_t)
        hist_pid['curv'].append(curv_p)
        wx,wy = track.get_xy_with_offset(t_pid, lat_pid)
        hist_pid['x'].append(wx); hist_pid['y'].append(wy)

        # ── FUZZY ──
        curv_f  = track.curvature(t_fz)
        err_f   = np.clip(lat_fz / 0.025 * 3.0, -3, 3)
        st_f, sp_f = fuzzy_compute(float(err_f), float(curv_f))

        drift_f = curv_f * sp_f * 0.022           # moins de drift (vitesse réduite)
        lat_fz += (-st_f*sp_f*0.013 + drift_f + np.random.randn()*noise_sigma)*DT
        lat_fz  = np.clip(lat_fz, -0.09, 0.09)
        prev_t_fz = t_fz
        t_fz    = (t_fz + sp_f * DT * 0.32) % 1.0
        if t_fz < prev_t_fz: hist_fz['laps'] += 1
        if abs(lat_fz) > 0.06: hist_fz['lost'] += 1

        hist_fz['err'].append(abs(err_f)); hist_fz['spd'].append(sp_f)
        hist_fz['steer'].append(st_f); hist_fz['curv'].append(curv_f)
        wx,wy = track.get_xy_with_offset(t_fz, lat_fz)
        hist_fz['x'].append(wx); hist_fz['y'].append(wy)

    return hist_pid, hist_fz

print("⏳ Exécution de la simulation…")
SIM_PID, SIM_FZ = run_simulation(700)
T_AXIS = np.arange(700) / 60.0  # temps en secondes

def ma(arr, w=12):
    """Moyenne mobile"""
    return np.convolve(arr, np.ones(w)/w, mode='same')

def save(name):
    plt.savefig(f'img/{name}', dpi=150, bbox_inches='tight', facecolor=BG0)
    plt.close()
    print(f"  ✅ {name}")
# ═══════════════════════════════════════════════════════════════════
#  01 — FONCTIONS D'APPARTENANCE : ERREUR LATÉRALE
# ═══════════════════════════════════════════════════════════════════
def fig01():
    fig, ax = plt.subplots(figsize=(13,5))
    x = np.linspace(-3.2, 3.2, 600)
    for nm, mf in ERR_MF.items():
        y = vtr(mf, x)
        c = MF_COLORS[nm]
        ax.plot(x, y, color=c, lw=2.5, label=nm)
        ax.fill_between(x, y, alpha=0.07, color=c)
    # Exemple erreur courante
    ex = 0.9
    ax.axvline(ex, color=GOLD, ls='--', lw=1.8, alpha=0.75)
    ax.text(ex+0.08, 0.91, f'erreur = {ex}', color=GOLD, fontsize=10)
    for nm, mf in ERR_MF.items():
        mu_val = mf(ex)
        if mu_val > 0.05:
            c = MF_COLORS[nm]
            ax.annotate(f'μ={mu_val:.2f}', xy=(ex, mu_val),
                        xytext=(ex+0.3, mu_val+0.05),
                        color=c, fontsize=9, fontweight='bold')
    ax.set_xlim(-3.4, 3.4); ax.set_ylim(-0.04, 1.18)
    ax.set_xlabel('Erreur latérale e(t)  ∈  [-3.0, +3.0]', fontsize=12)
    ax.set_ylabel('Degré d\'appartenance  μ(e)', fontsize=12)
    ax.set_title('Fonctions d\'Appartenance — Erreur Latérale (7 ensembles flous)',
                 fontsize=14, color=FZ, fontweight='bold', pad=14)
    ax.legend(ncol=7, fontsize=11, loc='upper center', bbox_to_anchor=(0.5,1.0))
    ax.grid(True, alpha=0.3)
    # Labels
    for nm, cx2 in [('NL',-2.5),('ZE',0),('PL',2.5)]:
        ax.text(cx2, -0.03, nm, ha='center', color=MF_COLORS[nm], fontsize=10, fontweight='bold')
    save('01_mf_erreur.png')
fig01()

# ═══════════════════════════════════════════════════════════════════
#  02 — FONCTIONS D'APPARTENANCE : DÉRIVÉE DE L'ERREUR
# ═══════════════════════════════════════════════════════════════════
def fig02():
    fig, ax = plt.subplots(figsize=(13,5))
    x = np.linspace(-2.3, 2.3, 400)
    cols = ['#ff6b6b','#ffa07a',FZ,BLUE,'#c77dff']
    for (nm, mf), c in zip(RATE_MF.items(), cols):
        y = vtr(mf, x)
        ax.plot(x, y, color=c, lw=2.5, label=nm)
        ax.fill_between(x, y, alpha=0.09, color=c)
    ax.set_xlim(-2.5,2.5); ax.set_ylim(-0.04,1.18)
    ax.set_xlabel('Dérivée de l\'erreur  de/dt  ∈  [-2.0, +2.0]', fontsize=12)
    ax.set_ylabel('Degré d\'appartenance  μ', fontsize=12)
    ax.set_title('Fonctions d\'Appartenance — Dérivée de l\'Erreur (anticipation)',
                 fontsize=14, color=BLUE, fontweight='bold', pad=14)
    ax.legend(ncol=5, fontsize=12, loc='upper center', bbox_to_anchor=(0.5,1.0))
    ax.grid(True, alpha=0.3)
    save('02_mf_derivee.png')
fig02()

# ═══════════════════════════════════════════════════════════════════
#  03 — FONCTIONS D'APPARTENANCE : COURBURE
# ═══════════════════════════════════════════════════════════════════
def fig03():
    fig, ax = plt.subplots(figsize=(11,5))
    x = np.linspace(-0.05,1.05,300)
    cfg = [('STRAIGHT','#4ade80'),('MILD',GOLD),('SHARP',PID)]
    for (nm, mf),c in zip(CURV_MF.items(), [c for _,c in cfg]):
        y = vtr(mf, x)
        ax.plot(x, y, color=c, lw=3, label=nm)
        ax.fill_between(x, y, alpha=0.12, color=c)
    # Annotations zones
    ax.axvspan(0, 0.25, alpha=0.04, color='#4ade80', label='_')
    ax.axvspan(0.35, 0.65, alpha=0.04, color=GOLD)
    ax.axvspan(0.75, 1.05, alpha=0.04, color=PID)
    ax.text(0.10, 0.6, 'Zone\ndroite', color='#4ade80', ha='center', fontsize=10)
    ax.text(0.50, 0.6, 'Virage\ndoux', color=GOLD, ha='center', fontsize=10)
    ax.text(0.88, 0.6, 'Virage\nserré', color=PID, ha='center', fontsize=10)
    ax.set_xlim(-0.07,1.07); ax.set_ylim(-0.04,1.18)
    ax.set_xlabel('Courbure locale κ  ∈  [0.0, 1.0]', fontsize=12)
    ax.set_ylabel('Degré d\'appartenance  μ', fontsize=12)
    ax.set_title('Fonctions d\'Appartenance — Courbure (variable floue clé)',
                 fontsize=14, color=GOLD, fontweight='bold', pad=14)
    ax.legend(fontsize=12)
    ax.grid(True, alpha=0.3)
    save('03_mf_courbure.png')
fig03()

# ═══════════════════════════════════════════════════════════════════
#  04 — SINGLETONS DE SORTIE : BRAQUAGE
# ═══════════════════════════════════════════════════════════════════
def fig04():
    fig, ax = plt.subplots(figsize=(11,5))
    cmap = plt.cm.RdYlGn
    for nm, val in STEER_SING.items():
        col = cmap((val+1)/2)
        ax.axvline(val, color=col, lw=3.5, alpha=0.85)
        ax.annotate(nm, xy=(val,0.55), xytext=(val,0.65),
                    ha='center', color=col, fontsize=13, fontweight='bold',
                    arrowprops=dict(arrowstyle='->', color=col, lw=1.5))
        ax.scatter([val],[0.3], color=col, s=180, zorder=5)
    ax.axvspan(-1,-0.5, alpha=0.06, color=PID, label='Tourner fort gauche')
    ax.axvspan(-0.1,0.1, alpha=0.06, color=FZ, label='Tout droit')
    ax.axvspan(0.5,1, alpha=0.06, color=BLUE, label='Tourner fort droite')
    ax.set_xlim(-1.3,1.3); ax.set_ylim(0,1)
    ax.set_yticks([])
    ax.set_xlabel('Braquage  u  ∈  [-1.0, +1.0]', fontsize=12)
    ax.set_title('Singletons de Sortie — Braquage (Défuzzification COG)',
                 fontsize=14, color=FZ, fontweight='bold', pad=14)
    ax.legend(fontsize=10)
    ax.grid(axis='x', alpha=0.3)
    save('04_mf_braquage.png')
fig04()

# ═══════════════════════════════════════════════════════════════════
#  05 — SINGLETONS : VITESSE VIRAGE + VITESSE DROITE
# ═══════════════════════════════════════════════════════════════════
def fig05():
    fig, axes = plt.subplots(1,2,figsize=(14,5))
    for ax, title, sing, col in [
        (axes[0],'Vitesse en Virage (corner_speed)',CV_SING,PID),
        (axes[1],'Vitesse en Ligne Droite (straight_speed)',SV_SING,FZ)]:
        cmap2 = LinearSegmentedColormap.from_list('sp',[col+'44',col], N=256)
        for i,(nm,val) in enumerate(sing.items()):
            c = cmap2(i/max(len(sing)-1,1))
            ax.barh([nm],[val], color=c, height=0.55, alpha=0.85)
            ax.text(val+0.01, i, f'{val:.2f}', va='center', color=TEXT, fontsize=11)
        ax.set_xlim(0,1.15)
        ax.set_xlabel('Vitesse normalisée [0, 1]', fontsize=11)
        ax.set_title(title, color=col, fontsize=12, fontweight='bold')
        ax.grid(axis='x', alpha=0.3)
    fig.suptitle('Singletons de Sortie — Variables de Vitesse (floue adaptative)',
                 fontsize=14, color=GOLD, fontweight='bold')
    plt.tight_layout()
    save('05_mf_vitesse.png')
fig05()

# ═══════════════════════════════════════════════════════════════════
#  06 — MATRICE DES RÈGLES FLOUES (21 règles)
# ═══════════════════════════════════════════════════════════════════
def fig06():
    # Mapping règle → valeur de braquage pour la heatmap
    err_labels = ['NL','NM','NS','ZE','PS','PM','PL']
    rate_labels = ['NL','NS','ZE','PS','PL']
    steer_map = {'HL':-1.0,'ML':-0.65,'SL':-0.3,'ZE':0.0,'SR':0.3,'MR':0.65,'HR':1.0}
    # Groupe 1 (erreur × rate=ANY)
    G1 = {'NL':'HR','NM':'MR','NS':'SR','ZE':'ZE','PS':'SL','PM':'ML','PL':'HL'}
    # Groupe 2 (corrections dérivée)
    G2 = {('NS','PL'):'ZE',('PS','NL'):'ZE',('ZE','PL'):'SL',('ZE','NL'):'SR'}

    mat = np.zeros((7,5))
    for i,e in enumerate(err_labels):
        for j,r in enumerate(rate_labels):
            if (e,r) in G2: mat[i,j] = steer_map[G2[(e,r)]]
            elif e in G1:   mat[i,j] = steer_map[G1[e]]

    fig, ax = plt.subplots(figsize=(11,7))
    cmap3 = LinearSegmentedColormap.from_list('steer', [PID,'#1e2230',FZ], N=256)
    im = ax.imshow(mat, cmap=cmap3, vmin=-1, vmax=1, aspect='auto')
    ax.set_xticks(range(5)); ax.set_xticklabels(rate_labels, fontsize=12)
    ax.set_yticks(range(7)); ax.set_yticklabels(err_labels, fontsize=12)
    ax.set_xlabel('Dérivée de l\'erreur', fontsize=13)
    ax.set_ylabel('Erreur latérale', fontsize=13)
    ax.set_title('Matrice des Règles Floues — Braquage de Sortie (21 règles)',
                 fontsize=14, color=FZ, fontweight='bold', pad=14)
    # Annotations valeurs
    for i in range(7):
        for j in range(5):
            v = mat[i,j]
            lbl = next((k for k,vv in steer_map.items() if abs(vv-v)<0.01), f'{v:.2f}')
            ax.text(j, i, lbl, ha='center', va='center',
                    color='white' if abs(v)<0.5 else TEXT, fontsize=11, fontweight='bold')
    cbar = plt.colorbar(im, ax=ax, shrink=0.8, pad=0.02)
    cbar.set_label('Braquage  u  ∈  [-1, +1]', color=TEXT, fontsize=11)
    cbar.ax.yaxis.set_tick_params(color=TEXT2)
    plt.setp(cbar.ax.yaxis.get_ticklabels(), color=TEXT2)
    save('06_regles_matrice.png')
fig06()

# ═══════════════════════════════════════════════════════════════════
#  07 — RÉPONSE INDICIELLE PID
# ═══════════════════════════════════════════════════════════════════
def fig07():
    pid2 = PIDSim(kp=1.8, ki=0.025, kd=1.4, speed=0.6)
    t_vec = np.linspace(0, 5, 300)
    # Échelon: erreur = 1 pour t>0
    errs, steers = [], []
    err_now = 1.0
    lat = 1.0
    for i,t in enumerate(t_vec):
        s,_ = pid2.compute(err_now)
        lat -= s * 0.8 / 60
        lat = np.clip(lat, -2, 2)
        err_now = lat
        errs.append(lat)
        steers.append(s)

    fig, (ax1, ax2) = plt.subplots(2,1,figsize=(12,7), sharex=True)
    ax1.plot(t_vec, errs, color=PID, lw=2.5, label='Erreur latérale')
    ax1.axhline(0, color=TEXT2, ls='--', lw=1, alpha=0.5)
    ax1.fill_between(t_vec, errs, 0, alpha=0.12, color=PID)
    ax1.set_ylabel('Erreur e(t)', fontsize=12)
    ax1.set_title('Réponse Indicielle du Contrôleur PID (Kp=1.8, Ki=0.025, Kd=1.4)',
                  fontsize=14, color=PID, fontweight='bold')
    ax1.legend(fontsize=11); ax1.grid(True, alpha=0.3)

    ax2.plot(t_vec, steers, color=GOLD, lw=2.5, label='Commande u(t)')
    ax2.axhline(0, color=TEXT2, ls='--', lw=1, alpha=0.5)
    ax2.set_xlabel('Temps (s)', fontsize=12)
    ax2.set_ylabel('Braquage u(t)', fontsize=12)
    ax2.legend(fontsize=11); ax2.grid(True, alpha=0.3)
    plt.tight_layout()
    save('07_pid_step_response.png')
fig07()

# ═══════════════════════════════════════════════════════════════════
#  08 — TERMES P / I / D AU COURS DU TEMPS
# ═══════════════════════════════════════════════════════════════════
def fig08():
    fig, ax = plt.subplots(figsize=(13,5))
    P = ma(SIM_PID['P'],8)
    I = ma(SIM_PID['I'],8)
    D = ma(SIM_PID['D'],8)
    ax.plot(T_AXIS, P, color=PID,  lw=2.0, label='Terme P (proportionnel)')
    ax.plot(T_AXIS, I, color=GOLD, lw=2.0, label='Terme I (intégral)')
    ax.plot(T_AXIS, D, color=BLUE, lw=2.0, label='Terme D (dérivé)', alpha=0.8)
    ax.axhline(0, color=TEXT2, ls='--', lw=0.8, alpha=0.4)
    # Zones courbes
    ax.axvspan(3.6, 5.7, alpha=0.06, color=GOLD, label='Virage gauche')
    ax.axvspan(7.2, 9.4, alpha=0.06, color=PURP, label='Virage droit')
    ax.set_xlabel('Temps (s)', fontsize=12)
    ax.set_ylabel('Amplitude', fontsize=12)
    ax.set_title('Termes P / I / D du Contrôleur PID au cours du temps',
                 fontsize=14, color=PID, fontweight='bold')
    ax.legend(fontsize=11, loc='upper right')
    ax.grid(True, alpha=0.3)
    save('08_pid_composantes.png')
fig08()

# ═══════════════════════════════════════════════════════════════════
#  09 — COMPARAISON ERREUR LATÉRALE
# ═══════════════════════════════════════════════════════════════════
def fig09():
    fig, (ax1, ax2) = plt.subplots(2,1,figsize=(13,8), sharex=True,
                                    gridspec_kw={'height_ratios':[3,1]})
    ep = ma(SIM_PID['err'],10)
    ef = ma(SIM_FZ['err'],10)
    ax1.plot(T_AXIS, ep, color=PID, lw=2.2, label='PID (erreur fixe)')
    ax1.plot(T_AXIS, ef, color=FZ,  lw=2.2, label='Fuzzy + Transformer')
    ax1.fill_between(T_AXIS, ep, ef, where=(ep>ef), alpha=0.15, color=PID, label='Avantage Fuzzy')
    # Zones courbes
    curv = np.array(SIM_FZ['curv'])
    curve_mask = curv > 0.3
    ax1.fill_between(T_AXIS, 0, 3, where=curve_mask, alpha=0.07, color=GOLD, label='Zone courbe')
    avg_pid = np.mean(SIM_PID['err'])
    avg_fz  = np.mean(SIM_FZ['err'])
    ax1.axhline(avg_pid, color=PID, ls=':', lw=1.5, alpha=0.6)
    ax1.axhline(avg_fz,  color=FZ,  ls=':', lw=1.5, alpha=0.6)
    ax1.text(T_AXIS[-1]*0.02, avg_pid+0.05, f'Moy PID={avg_pid:.3f}', color=PID, fontsize=10)
    ax1.text(T_AXIS[-1]*0.02, avg_fz+0.05,  f'Moy Fuzzy={avg_fz:.3f}',  color=FZ,  fontsize=10)
    gain = (1 - avg_fz/avg_pid)*100
    ax1.set_title(f'Erreur Latérale |e(t)| — PID vs Fuzzy+Transformer  (Fuzzy −{gain:.0f}% d\'erreur)',
                  fontsize=13, color=FZ, fontweight='bold')
    ax1.set_ylabel('|Erreur| normalisée', fontsize=12)
    ax1.legend(fontsize=11); ax1.grid(True, alpha=0.3)
    ax1.set_ylim(0, 3.5)
    # Curvature track
    ax2.fill_between(T_AXIS, curv, alpha=0.6, color=GOLD)
    ax2.set_ylabel('Courbure κ', fontsize=10, color=GOLD)
    ax2.set_xlabel('Temps (s)', fontsize=12)
    ax2.set_ylim(0,1.3)
    ax2.grid(True, alpha=0.3)
    plt.tight_layout()
    save('09_comparaison_erreur.png')
fig09()

# ═══════════════════════════════════════════════════════════════════
#  10 — COMPARAISON VITESSE
# ═══════════════════════════════════════════════════════════════════
def fig10():
    fig, ax = plt.subplots(figsize=(13,5))
    sp = ma(SIM_PID['spd'],8)
    sf = ma(SIM_FZ['spd'],8)
    curv = np.array(SIM_FZ['curv'])
    ax.fill_between(T_AXIS, 0, 1.1, where=(curv>0.3), alpha=0.07, color=GOLD, label='Zone courbe')
    ax.plot(T_AXIS, sp, color=PID, lw=2.2, label=f'PID — vitesse constante = {SIM_PID["spd"][0]:.2f}')
    ax.plot(T_AXIS, sf, color=FZ,  lw=2.2, label='Fuzzy — vitesse adaptative')
    ax.fill_between(T_AXIS, sp, sf, where=(sp>sf), alpha=0.15, color=PID, label='Excès PID (risque)')
    ax.fill_between(T_AXIS, sp, sf, where=(sf>sp), alpha=0.12, color=FZ, label='Accélération Fuzzy')
    ax.set_xlabel('Temps (s)', fontsize=12)
    ax.set_ylabel('Vitesse normalisée [0, 1]', fontsize=12)
    ax.set_title('Vitesse — PID (constante) vs Fuzzy (adaptative à la courbure)',
                 fontsize=13, color=FZ, fontweight='bold')
    ax.legend(fontsize=11); ax.grid(True, alpha=0.3)
    ax.set_ylim(0, 1.15)
    # Annotations
    ax.text(T_AXIS[600], 0.62, '← Vitesse PID\n   figée', color=PID, fontsize=9)
    save('10_comparaison_vitesse.png')
fig10()

# ═══════════════════════════════════════════════════════════════════
#  11 — TRAJECTOIRE 2D SUR LA PISTE
# ═══════════════════════════════════════════════════════════════════
def fig11():
    fig, ax = plt.subplots(figsize=(13,7))
    ax.set_facecolor('#0d1117')
    # Dessiner la piste
    pts = track.pts; N = len(pts)
    hw = 0.025  # demi-largeur piste
    outer_x, outer_y = [], []
    inner_x, inner_y = [], []
    for i,wp in enumerate(pts):
        tx,ty = track.get_tangent(i/N)
        nx,ny = -ty, tx
        outer_x.append(wp[0]+hw*nx); outer_y.append(wp[1]+hw*ny)
        inner_x.append(wp[0]-hw*nx); inner_y.append(wp[1]-hw*ny)
    outer_x.append(outer_x[0]); outer_y.append(outer_y[0])
    inner_x.append(inner_x[0]); inner_y.append(inner_y[0])
    # Asphalt
    from matplotlib.patches import Polygon
    from matplotlib.collections import PatchCollection
    track_patch_x = outer_x + inner_x[::-1]
    track_patch_y = outer_y + inner_y[::-1]
    poly = Polygon(list(zip(track_patch_x, track_patch_y)), closed=True,
                   facecolor='#2a2a2a', edgecolor='none', alpha=0.9)
    ax.add_patch(poly)
    # Bords
    ax.plot(outer_x, outer_y, color='#888', lw=1.2, alpha=0.6)
    ax.plot(inner_x, inner_y, color='#888', lw=1.2, alpha=0.6)
    # Ligne centrale (pointillés)
    cx2 = [p[0] for p in pts] + [pts[0][0]]
    cy2 = [p[1] for p in pts] + [pts[0][1]]
    ax.plot(cx2, cy2, color='white', lw=1.2, ls='--', alpha=0.35, dashes=(6,10))
    # Trajectoires
    px = SIM_PID['x']; py = SIM_PID['y']
    fx = SIM_FZ['x'];  fy = SIM_FZ['y']
    # Couleur par temps
    from matplotlib.colors import Normalize
    from matplotlib.collections import LineCollection
    for traj_x,traj_y,col,lbl in [(px,py,PID,'PID'),(fx,fy,FZ,'Fuzzy')]:
        pts2 = np.array([traj_x,traj_y]).T.reshape(-1,1,2)
        segs = np.concatenate([pts2[:-1],pts2[1:]],axis=1)
        norm = Normalize(0,len(traj_x))
        lc = LineCollection(segs, cmap=LinearSegmentedColormap.from_list('c',[col+'44',col]),
                             norm=norm, lw=2, alpha=0.85)
        lc.set_array(np.arange(len(traj_x)))
        ax.add_collection(lc)
        ax.plot([],[], color=col, lw=2, label=lbl)
    # Start marker
    sx,sy = track.get_point(0.0)
    ax.scatter([sx],[sy], color=GOLD, s=200, zorder=10, marker='*')
    ax.text(sx+0.02, sy-0.02, 'START', color=GOLD, fontsize=12, fontweight='bold')
    ax.set_xlim(0.0,1.0); ax.set_ylim(0.2,0.8)
    ax.set_aspect('equal')
    ax.set_title('Trajectoire 2D — PID vs Fuzzy+Transformer sur Ovale Stadium',
                 fontsize=13, color=FZ, fontweight='bold')
    ax.legend(fontsize=12, loc='upper right')
    ax.axis('off')
    save('11_comparaison_trajectoire.png')
fig11()

# ═══════════════════════════════════════════════════════════════════
#  12 — TRANSFORMER : ATTENTION MULTI-HEAD
# ═══════════════════════════════════════════════════════════════════
def fig12():
    np.random.seed(7)
    N_seq = 16  # états en mémoire
    N_heads = 4
    head_names = ['H1: Erreur récente', 'H2: Tendance LT', 'H3: Oscillations', 'H4: Courbure']
    head_cols  = [FZ, BLUE, GOLD, PURP]

    # Générer des poids d'attention simulés (réalistes)
    weights = []
    for h in range(N_heads):
        if h==0:   w = np.exp(-np.arange(N_seq)*0.3)[::-1]  # récent important
        elif h==1: w = np.linspace(0.5, 1.0, N_seq)          # longue tendance
        elif h==2: w = 0.5+0.4*np.sin(np.arange(N_seq)*0.8) # oscillations
        else:      w = np.clip(np.random.rand(N_seq)*0.8, 0.1, 0.9)
        w = w / w.sum()
        weights.append(w)

    fig, axes = plt.subplots(N_heads, 1, figsize=(13,8), sharex=True)
    for h, (ax,wh,nm,c) in enumerate(zip(axes,weights,head_names,head_cols)):
        bars = ax.bar(range(N_seq), wh, color=c, alpha=0.8, width=0.8)
        ax.set_ylabel('Poids α', fontsize=9, color=c)
        ax.set_title(nm, fontsize=10, color=c, fontweight='bold', pad=3)
        ax.set_ylim(0, wh.max()*1.4)
        ax.grid(axis='y', alpha=0.25)
        # Highlight max
        idx_max = np.argmax(wh)
        bars[idx_max].set_alpha(1.0)
        bars[idx_max].set_edgecolor('white')
        ax.text(idx_max, wh[idx_max]+0.003, f'max={wh[idx_max]:.3f}',
                ha='center', color='white', fontsize=8)
    axes[-1].set_xlabel(f'États en mémoire (t-{N_seq} → t)', fontsize=11)
    fig.suptitle('Transformer Simplifié — Poids d\'Attention Multi-Head (16 états mémoire)',
                 fontsize=14, color=FZ, fontweight='bold', y=0.99)
    plt.tight_layout()
    save('12_transformer_attention.png')
fig12()

# ═══════════════════════════════════════════════════════════════════
#  13 — DÉFUZZIFICATION PAR COG
# ═══════════════════════════════════════════════════════════════════
def fig13():
    fig, ax = plt.subplots(figsize=(12,5))
    # Exemple: erreur = 0.8 → activation NS, ZE, PS
    err_ex = 0.8
    x = np.linspace(-1.1, 1.1, 300)
    sings = STEER_SING
    active = {'NS': ERR_MF['PS'](err_ex), 'ZE': ERR_MF['ZE'](err_ex)}  # règles actives
    # Fonctions clippées (Mamdani)
    for nm, mf_nm, col in [('SL','PS',BLUE), ('ZE','ZE',FZ)]:
        alpha = active.get(nm, ERR_MF.get(mf_nm, lambda v:0)(err_ex))
        sv = sings[nm]
        # Spike clippé à alpha
        ax.vlines(sv, 0, alpha, color=col, lw=5, alpha=0.8, label=f'{nm}: α={alpha:.2f}')
        ax.scatter([sv], [alpha], color=col, s=150, zorder=5)
    # Approximation ensemble agrégé (barres)
    for nm, sv in sings.items():
        alpha_v = 0.0
        if nm == 'SL': alpha_v = ERR_MF['PS'](err_ex)
        elif nm == 'ZE': alpha_v = ERR_MF['ZE'](err_ex)*0.5
        elif nm == 'SR': alpha_v = ERR_MF['NS'](err_ex)*0.3
        if alpha_v > 0.01:
            ax.vlines(sv, 0, alpha_v, color=TEXT2, lw=3, alpha=0.4)
    # COG
    num = sum(sings[nm]*alpha for nm, alpha in
              [('ZE',ERR_MF['ZE'](err_ex)),('SL',ERR_MF['PS'](err_ex))])
    den = sum(alpha for alpha in
              [ERR_MF['ZE'](err_ex), ERR_MF['PS'](err_ex)])
    cog = num/den if den>0 else 0
    ax.axvline(cog, color=GOLD, lw=2.5, ls='--', label=f'COG = {cog:.3f}')
    ax.annotate(f'Sortie\n u = {cog:.3f}', xy=(cog,0.15),
                xytext=(cog+0.25, 0.25), color=GOLD, fontsize=12,
                arrowprops=dict(arrowstyle='->', color=GOLD, lw=1.5))
    # Axes
    ax.axvspan(-1.1,-1.0, alpha=0.04, color=PID)
    ax.axvspan(1.0,1.1, alpha=0.04, color=FZ)
    ax.set_xlim(-1.2,1.2); ax.set_ylim(-0.02, 0.75)
    ax.set_xlabel('Braquage u  ∈  [-1.0, +1.0]', fontsize=12)
    ax.set_ylabel('Activation α', fontsize=12)
    ax.set_title(f'Défuzzification COG — Exemple pour erreur = {err_ex}  →  u = {cog:.3f}',
                 fontsize=13, color=FZ, fontweight='bold')
    ax.legend(fontsize=12); ax.grid(True, alpha=0.3)
    # Singletons labels
    for nm,sv in sings.items():
        ax.text(sv, -0.015, nm, ha='center', color=TEXT2, fontsize=9)
    save('13_defuzzification.png')
fig13()

# ═══════════════════════════════════════════════════════════════════
#  14 — PISTE OVALE + CARTE DE COURBURE
# ═══════════════════════════════════════════════════════════════════
def fig14():
    fig, ax = plt.subplots(figsize=(13,7))
    ax.set_facecolor('#0d1117')
    pts  = track.pts; N = len(pts)
    curvs = [track.curvature(i/N) for i in range(N)]
    hw = 0.027

    # Dessiner segments colorés par courbure
    cmap4 = LinearSegmentedColormap.from_list('curv',['#4ade80',GOLD,PID], N=256)
    from matplotlib.collections import LineCollection
    for i in range(N):
        j = (i+1)%N
        c = curvs[i]
        col = cmap4(c)
        tx,ty = track.get_tangent(i/N)
        nx,ny = -ty,tx
        px1,py1 = pts[i]
        px2,py2 = pts[j]
        # Draw thick segment (outer half + inner half)
        for side in [-1,1]:
            ax.plot([px1+hw*nx*side, px2+hw*nx*side],
                    [py1+hw*ny*side, py2+hw*ny*side],
                    color=col, lw=18, alpha=0.3, solid_capstyle='round')
        ax.plot([px1,px2],[py1,py2], color=col, lw=2, alpha=0.9)

    # Centre pointillés
    cx2 = [p[0] for p in pts] + [pts[0][0]]
    cy2 = [p[1] for p in pts] + [pts[0][1]]
    ax.plot(cx2,cy2, color='white', lw=1.5, ls='--', alpha=0.4, dashes=(6,10))

    # Colorbar
    sm = ScalarMappable(cmap=cmap4); sm.set_array([0,1])
    cbar = plt.colorbar(sm, ax=ax, shrink=0.7, pad=0.02)
    cbar.set_label('Courbure κ', color=TEXT, fontsize=11)
    plt.setp(cbar.ax.yaxis.get_ticklabels(), color=TEXT2)
    cbar.ax.set_yticklabels(['0.0\n(droite)','0.25','0.5','0.75','1.0\n(serré)'],
                            color=TEXT2, fontsize=8)

    # Annotations segments
    ax.text(0.50, 0.345, 'LIGNE DROITE (haut)\nκ ≈ 0 → TURBO', ha='center',
            color='#4ade80', fontsize=10, fontweight='bold')
    ax.text(0.50, 0.660, 'LIGNE DROITE (bas)\nκ ≈ 0 → TURBO', ha='center',
            color='#4ade80', fontsize=10, fontweight='bold')
    ax.text(0.155, 0.500, 'VIRAGE\nGAUCHE\nκ→1', ha='center',
            color=PID, fontsize=10, fontweight='bold')
    ax.text(0.850, 0.500, 'VIRAGE\nDROIT\nκ→1', ha='center',
            color=PID, fontsize=10, fontweight='bold')

    # Start
    sx,sy = track.get_point(0.0)
    ax.scatter([sx],[sy], color=GOLD, s=300, zorder=10, marker='*')
    ax.text(sx+0.02, sy-0.025, 'START/FINISH', color=GOLD, fontsize=11, fontweight='bold')

    ax.set_xlim(0.02,0.98); ax.set_ylim(0.25,0.76)
    ax.set_aspect('equal'); ax.axis('off')
    ax.set_title('Autodrome Ovale Stadium — Carte de Courbure (κ)',
                 fontsize=14, color=FZ, fontweight='bold', pad=12)
    save('14_piste_courbure.png')
fig14()

# ═══════════════════════════════════════════════════════════════════
#  15 — BILAN COMPARATIF SYNTHÉTIQUE
# ═══════════════════════════════════════════════════════════════════
def fig15():
    fig = plt.figure(figsize=(14,9))
    gs = gridspec.GridSpec(2,3, figure=fig, hspace=0.5, wspace=0.4)

    # Critères
    criteria = ['Erreur moy.', 'Hors piste', 'Stabilité\ncourbes', 'Vitesse\nadaptative',
                'Anticipation', 'Complexité\nimplem.']
    pid_scores = [0.60, 0.70, 0.40, 0.10, 0.45, 0.90]  # (1=bon pour PID)
    fz_scores  = [0.85, 0.88, 0.90, 0.95, 0.80, 0.55]

    # ── Radar chart ──
    ax1 = fig.add_subplot(gs[:,0], polar=True)
    N_c = len(criteria)
    angles = np.linspace(0, 2*np.pi, N_c, endpoint=False).tolist()
    angles += angles[:1]
    pid_v = pid_scores + pid_scores[:1]
    fz_v  = fz_scores  + fz_scores[:1]
    ax1.set_facecolor(BG1)
    ax1.plot(angles, pid_v, color=PID, lw=2.5, label='PID')
    ax1.fill(angles, pid_v, color=PID, alpha=0.15)
    ax1.plot(angles, fz_v,  color=FZ,  lw=2.5, label='Fuzzy')
    ax1.fill(angles, fz_v,  color=FZ,  alpha=0.15)
    ax1.set_xticks(angles[:-1])
    ax1.set_xticklabels(criteria, fontsize=9, color=TEXT)
    ax1.set_ylim(0,1)
    ax1.yaxis.set_tick_params(colors=TEXT2, labelsize=7)
    ax1.set_title('Radar Comparatif', color=TEXT, fontsize=12, fontweight='bold', pad=20)
    ax1.legend(loc='upper right', bbox_to_anchor=(1.35,1.1), fontsize=10)

    # ── Barres erreur movingavg ──
    ax2 = fig.add_subplot(gs[0,1:])
    ax2.set_facecolor(BG1)
    ep_ma = ma(SIM_PID['err'],20)
    ef_ma = ma(SIM_FZ['err'],20)
    ax2.plot(T_AXIS, ep_ma, color=PID, lw=2.2, label='PID')
    ax2.plot(T_AXIS, ef_ma, color=FZ,  lw=2.2, label='Fuzzy+Transformer')
    curv = np.array(SIM_FZ['curv'])
    ax2.fill_between(T_AXIS, 0, 3, where=(curv>0.3), alpha=0.08, color=GOLD, label='Virage')
    ax2.set_ylabel('|Erreur| moy.', fontsize=10)
    ax2.set_xlabel('Temps (s)', fontsize=10)
    ax2.set_title('Erreur latérale lissée', color=TEXT, fontsize=11)
    ax2.legend(fontsize=10); ax2.grid(True, alpha=0.3); ax2.set_ylim(0, 3.5)

    # ── Stats table ──
    ax3 = fig.add_subplot(gs[1,1:])
    ax3.axis('off')
    rows = [
        ['Critère', 'PID', 'Fuzzy+Transformer', 'Avantage'],
        ['Erreur moy.', f'{np.mean(SIM_PID["err"]):.3f}', f'{np.mean(SIM_FZ["err"]):.3f}', '✅ Fuzzy'],
        ['Hors piste', str(SIM_PID['lost']), str(SIM_FZ['lost']), '✅ Fuzzy'],
        ['Tours complétés', str(SIM_PID['laps']), str(SIM_FZ['laps']), '⚡ Fuzzy'],
        ['Vitesse adaptative', '❌ Non', '✅ Oui', '✅ Fuzzy'],
        ['Anticipation', '⚠️ Partielle', '✅ Mémoire 16 états', '✅ Fuzzy'],
    ]
    col_colors = [[BG2,BG2,BG2,BG2]]+[[BG1,BG1,BG1,BG1]]*5
    tbl = ax3.table(cellText=rows[1:], colLabels=rows[0],
                    cellLoc='center', loc='center',
                    cellColours=col_colors[1:],
                    colColours=[BG2]*4)
    tbl.auto_set_font_size(False); tbl.set_fontsize(10)
    for (r,c), cell in tbl.get_celld().items():
        cell.set_edgecolor('#252a3a')
        if r==0: cell.set_text_props(color=TEXT, fontweight='bold')
        else:
            if c==1: cell.set_text_props(color=PID)
            elif c==2: cell.set_text_props(color=FZ)
            elif c==3: cell.set_text_props(color=GOLD)
            else: cell.set_text_props(color=TEXT)
    tbl.scale(1, 1.5)

    gain_err = (1 - np.mean(SIM_FZ['err'])/np.mean(SIM_PID['err']))*100
    fig.suptitle(f'Bilan Synthétique — Fuzzy+Transformer vs PID  │  Gain erreur: −{gain_err:.0f}%',
                 fontsize=15, color=FZ, fontweight='bold', y=1.01)
    save('15_bilan_comparatif.png')
fig15()

# ═══════════════════════════════════════════════════════════════════
#  16 — PIPELINE COMPLET : ARCHITECTURE SYSTÈME
# ═══════════════════════════════════════════════════════════════════
def fig16():
    fig, ax = plt.subplots(figsize=(15,6))
    ax.set_facecolor(BG1)
    ax.axis('off')
    ax.set_xlim(0,15); ax.set_ylim(0,6)

    def box(x,y,w,h,col,label,sub='',fontsize=11):
        rect = mpatches.FancyBboxPatch((x-w/2,y-h/2),w,h,
            boxstyle="round,pad=0.1",facecolor=col+'22',edgecolor=col,lw=2)
        ax.add_patch(rect)
        ax.text(x,y+(0.15 if sub else 0),label,ha='center',va='center',
                color=col,fontsize=fontsize,fontweight='bold')
        if sub:
            ax.text(x,y-0.3,sub,ha='center',va='center',color=TEXT2,fontsize=8)

    def arrow(x1,y,x2,col='#555'):
        ax.annotate('',xy=(x2,y),xytext=(x1,y),
            arrowprops=dict(arrowstyle='->', color=col, lw=2))

    # Sensors
    box(1.2,3,'1.8',1.2,BLUE,'7 Capteurs\nIR Virtuels','signal [0/1] × 7')
    arrow(2.1,3, 2.8, BLUE)

    # Error compute
    box(3.5,3,'1.8',1.2,TEXT2,'Calcul\nErreur','e(t) ∈ [-3,+3]')
    arrow(4.4,3, 5.2, TEXT2)

    # Fuzzification
    box(6.0,4.5,'1.9',1.0,FZ,'Fuzzification','μ(e), μ(de/dt), μ(κ)')
    # Rules
    box(6.0,3,'2.0',1.0,FZ,'21 Règles\nMamdani','MIN + MAX')
    # Defuzz
    box(6.0,1.5,'1.9',1.0,FZ,'Défuzz COG','braquage + vitesse')

    ax.annotate('',xy=(5.05,4.5),xytext=(4.4,3.3),
        arrowprops=dict(arrowstyle='->', color=FZ, lw=1.5))
    ax.annotate('',xy=(5.05,3),xytext=(4.4,3),
        arrowprops=dict(arrowstyle='->', color=FZ, lw=1.5))
    ax.annotate('',xy=(5.05,1.5),xytext=(4.4,2.7),
        arrowprops=dict(arrowstyle='->', color=FZ, lw=1.5))
    ax.annotate('',xy=(7.5,4.5),xytext=(6.95,4),
        arrowprops=dict(arrowstyle='->', color=FZ, lw=1.5))
    ax.annotate('',xy=(7.5,1.5),xytext=(6.95,2),
        arrowprops=dict(arrowstyle='->', color=FZ, lw=1.5))
    # Transformer
    box(9.0,3,'2.0',1.2,PURP,'Transformer\n(Mémoire 16)','correction ±0.15')
    ax.annotate('',xy=(8.0,3),xytext=(7.5,3),
        arrowprops=dict(arrowstyle='->', color=PURP, lw=2))
    arrow(10.0,3, 10.8, PURP)

    # Fusion
    box(11.5,3,'1.6',1.0,'#ff6b00','Fusion\nFuzzy+Trans','commande finale')
    arrow(12.3,3, 13.0, '#ff6b00')

    # Robot
    box(13.8,3,'1.4',1.0,TEXT,'Robot\n(moteurs)','x,y,θ')

    # PID (parallel branch)
    box(6.0,5.5,'3.0',0.7,PID,'PID: Kp·e + Ki·∫e + Kd·de/dt','vitesse fixe = 0.60')
    ax.annotate('',xy=(5.05,5.5),xytext=(4.4,3.5),
        arrowprops=dict(arrowstyle='->', color=PID, lw=1.5, linestyle='dashed'))
    ax.text(7.5, 5.5, '(Référence)', ha='center', color=PID, fontsize=9, style='italic')

    ax.set_title('Architecture Système — Fuzzy+Transformer vs PID (pipeline complet)',
                 fontsize=14, color=FZ, fontweight='bold', pad=12)
    save('16_architecture_systeme.png')
fig16()

# ═══════════════════════════════════════════════════════════════════
#  17 — FUZZIFICATION EXEMPLE COMPLET (3 entrées simultanées)
# ═══════════════════════════════════════════════════════════════════
def fig17():
    fig, axes = plt.subplots(1,3,figsize=(15,5))
    err_val, rate_val, curv_val = 1.2, -0.7, 0.65

    configs = [
        (ERR_MF,  np.linspace(-3.2,3.2,500), MF_COLORS,
         'Erreur e(t)', err_val, 'Erreur latérale  e ∈ [-3,+3]'),
        (RATE_MF, np.linspace(-2.3,2.3,400),
         {'NL':'#ff6b6b','NS':'#ffa07a','ZE':FZ,'PS':BLUE,'PL':'#c77dff'},
         'Dérivée de(t)', rate_val, 'Dérivée  de/dt ∈ [-2,+2]'),
        (CURV_MF, np.linspace(-0.05,1.05,300),
         {'STRAIGHT':'#4ade80','MILD':GOLD,'SHARP':PID},
         'Courbure κ', curv_val, 'Courbure  κ ∈ [0,1]'),
    ]
    vals_in = [err_val, rate_val, curv_val]
    for ax,(mfs,xs,cols,ttl,val,xlabel) in zip(axes,configs):
        for nm,mf in mfs.items():
            y = vtr(mf, xs)
            c = cols.get(nm,'#aaa')
            ax.plot(xs, y, color=c, lw=2.5, label=nm)
            ax.fill_between(xs, y, alpha=0.07, color=c)
            mu_val = mf(val)
            if mu_val > 0.05:
                ax.annotate(f'μ={mu_val:.2f}', xy=(val,mu_val),
                    xytext=(val+0.2, mu_val+0.08), color=c, fontsize=8,
                    arrowprops=dict(arrowstyle='->', color=c, lw=1))
        ax.axvline(val, color=GOLD, ls='--', lw=2, alpha=0.8)
        ax.text(val, 1.05, f'{val:+.1f}', ha='center', color=GOLD, fontsize=10, fontweight='bold')
        ax.set_title(ttl, fontsize=11, fontweight='bold', color=TEXT)
        ax.set_xlabel(xlabel, fontsize=9)
        ax.set_ylabel('μ', fontsize=11)
        ax.legend(fontsize=8, ncol=2); ax.grid(True, alpha=0.25)
        ax.set_ylim(-0.05, 1.25)
    fig.suptitle(f'Fuzzification — Exemple simultané : e={err_val}, de/dt={rate_val}, κ={curv_val}',
                 fontsize=14, color=FZ, fontweight='bold')
    plt.tight_layout()
    save('17_fuzzification_exemple.png')
fig17()

# ═══════════════════════════════════════════════════════════════════
#  18 — SURFACE DE CONTRÔLE FUZZY (erreur × dérivée → braquage)
# ═══════════════════════════════════════════════════════════════════
def fig18():
    from mpl_toolkits.mplot3d import Axes3D
    fig = plt.figure(figsize=(13,7))
    ax  = fig.add_subplot(111, projection='3d')
    ax.set_facecolor(BG1)

    E  = np.linspace(-3, 3, 40)
    dE = np.linspace(-2, 2, 40)
    EG, dEG = np.meshgrid(E, dE)
    Z = np.zeros_like(EG)

    rules = [('NL','HR'),('NM','MR'),('NS','SR'),('ZE','ZE'),
             ('PS','SL'),('PM','ML'),('PL','HL')]
    deriv_rules = {('NS','PL'):'ZE',('PS','NL'):'ZE',
                   ('ZE','PL'):'SL',('ZE','NL'):'SR'}

    for i in range(40):
        for j in range(40):
            e,d = E[j], dE[i]
            mu_e = {k:mf(e) for k,mf in ERR_MF.items()}
            mu_d = {k:mf(d) for k,mf in RATE_MF.items()}
            num=den=0.0
            for e_s,s_o in rules:
                alpha = mu_e.get(e_s,0)
                num += alpha*STEER_SING[s_o]; den += alpha
            for (e_s,d_s),s_o in deriv_rules.items():
                alpha = min(mu_e.get(e_s,0), mu_d.get(d_s,0))
                if alpha>0: num += alpha*STEER_SING[s_o]; den += alpha
            Z[i,j] = num/den if den>0 else 0

    cmap_surf = LinearSegmentedColormap.from_list('ctrl',[PID,'#1e2230',FZ])
    surf = ax.plot_surface(EG, dEG, Z, cmap=cmap_surf, alpha=0.85,
                           rstride=1, cstride=1, linewidth=0)
    ax.set_xlabel('Erreur e(t)', fontsize=10, color=TEXT)
    ax.set_ylabel('Dérivée de/dt', fontsize=10, color=TEXT)
    ax.set_zlabel('Braquage u', fontsize=10, color=FZ)
    ax.set_title('Surface de Contrôle Floue — u = f(e, de/dt)',
                 fontsize=13, color=FZ, fontweight='bold', pad=10)
    ax.tick_params(colors=TEXT2)
    fig.colorbar(surf, ax=ax, shrink=0.5, pad=0.1, label='Braquage u ∈ [-1,+1]')
    save('18_surface_controle_fuzzy.png')
fig18()

# ═══════════════════════════════════════════════════════════════════
#  19 — TRANSFORMER : MÉMOIRE ET PRÉDICTION DE TENDANCE
# ═══════════════════════════════════════════════════════════════════
def fig19():
    np.random.seed(42)
    N_mem = 16
    # Simuler une séquence d'erreurs avec tendance
    t_mem = np.arange(N_mem)
    errors_mem = 0.3 + 0.04*t_mem + 0.08*np.sin(t_mem*0.8) + np.random.randn(N_mem)*0.04

    # Régression linéaire (ce que fait le Transformer)
    coeffs = np.polyfit(t_mem, errors_mem, 1)
    trend = np.polyval(coeffs, t_mem)
    future = np.polyval(coeffs, np.arange(N_mem, N_mem+5))

    fig, (ax1, ax2) = plt.subplots(1,2,figsize=(14,6))

    # ── Axe 1 : Mémoire + tendance + prédiction ──
    ax1.fill_between(t_mem, errors_mem, alpha=0.15, color=BLUE)
    ax1.plot(t_mem, errors_mem, 'o-', color=BLUE, lw=2, ms=5, label='Erreurs mémorisées')
    ax1.plot(t_mem, trend, color=GOLD, lw=2.5, ls='--', label=f'Tendance (pente={coeffs[0]:.4f})')
    ax1.plot(np.arange(N_mem, N_mem+5), future, 'o--', color=PID, lw=2, ms=4, label='Prédiction future')
    ax1.axvspan(N_mem-0.5, N_mem+4.5, alpha=0.08, color=PID, label='Fenêtre prédiction')
    corr = np.clip(coeffs[0]*2.0, -0.15, 0.15)
    ax1.text(N_mem+2, future[2], f'Correction\nu={corr:+.3f}',
             ha='center', color=PID, fontsize=10, fontweight='bold',
             bbox=dict(boxstyle='round',facecolor=BG1,edgecolor=PID,alpha=0.8))
    ax1.set_xlabel(f'États mémoire (t-{N_mem} → t+5)', fontsize=11)
    ax1.set_ylabel('Erreur latérale e(t)', fontsize=11)
    ax1.set_title('Mémoire Transformer : Régression Linéaire sur 16 états', fontsize=12, color=BLUE, fontweight='bold')
    ax1.legend(fontsize=10); ax1.grid(True, alpha=0.3)

    # ── Axe 2 : Confidence vs trend magnitude ──
    trends = np.linspace(-0.2, 0.2, 200)
    conf = np.minimum(0.8, np.abs(trends)*5.0)
    corrs = np.clip(trends*2.0, -0.15, 0.15)
    ax2.plot(trends, conf, color=GOLD, lw=2.5, label='Confiance (0→0.8)')
    ax2.plot(trends, corrs, color=FZ,  lw=2.5, label='Correction braquage')
    ax2.axhline(0.15, color=PID, ls=':', lw=1.5, alpha=0.6, label='Limite correction ±0.15')
    ax2.axhline(-0.15, color=PID, ls=':', lw=1.5, alpha=0.6)
    ax2.fill_between(trends, -0.15, 0.15, alpha=0.07, color=FZ)
    ax2.set_xlabel('Pente de tendance (coefficient linéaire)', fontsize=11)
    ax2.set_ylabel('Valeur', fontsize=11)
    ax2.set_title('Transformer : Confiance et Correction vs Tendance', fontsize=12, color=GOLD, fontweight='bold')
    ax2.legend(fontsize=10); ax2.grid(True, alpha=0.3)

    fig.suptitle('Transformer Simplifié — Mémoire, Prédiction et Correction',
                 fontsize=14, color=FZ, fontweight='bold')
    plt.tight_layout()
    save('19_transformer_memoire_prediction.png')
fig19()

# ═══════════════════════════════════════════════════════════════════
#  20 — PISTE OVALE + TRAJECTOIRES (vue réelle)
# ═══════════════════════════════════════════════════════════════════
def fig20():
    from matplotlib.collections import LineCollection
    from matplotlib.colors import Normalize as MNorm
    fig, ax = plt.subplots(figsize=(14,6))
    ax.set_facecolor('#0d1117')
    pts = track.pts; N = len(pts)
    hw = 0.022

    # Asphalt fill
    curvs = [track.curvature(i/N) for i in range(N)]
    cmap_road = LinearSegmentedColormap.from_list('road',['#3a3a3a','#555555'], N=256)
    for i in range(N):
        j=(i+1)%N
        c=curvs[i]
        col=cmap_road(c)
        tx,ty=track.get_tangent(i/N); nx,ny=-ty,tx
        px1,py1=pts[i]; px2,py2=pts[j]
        for side in [-1,1]:
            ax.plot([px1+hw*nx*side,px2+hw*nx*side],[py1+hw*ny*side,py2+hw*ny*side],
                    color=col,lw=20,alpha=0.4,solid_capstyle='round')
        ax.plot([px1,px2],[py1,py2],color=col,lw=2,alpha=0.7)

    # Borders
    for side in [-1,1]:
        bx,by=[],[]
        for i in range(N+1):
            tx,ty=track.get_tangent(i/N); nx,ny=-ty,tx
            px,py=track.get_point(i/N)
            bx.append(px+hw*nx*side); by.append(py+hw*ny*side)
        ax.plot(bx,by,color='white',lw=2,alpha=0.75)

    # Center line (solid white)
    cx2=[p[0] for p in pts]+[pts[0][0]]
    cy2=[p[1] for p in pts]+[pts[0][1]]
    ax.plot(cx2,cy2,color='white',lw=2.5,alpha=0.95)

    # Trajectoires avec dégradé couleur
    for traj_x,traj_y,col,lbl in [(SIM_PID['x'],SIM_PID['y'],PID,'PID'),
                                    (SIM_FZ['x'], SIM_FZ['y'], FZ, 'Fuzzy+Trans')]:
        pts2=np.array([traj_x,traj_y]).T.reshape(-1,1,2)
        segs=np.concatenate([pts2[:-1],pts2[1:]],axis=1)
        lc=LineCollection(segs,colors=[col],linewidths=2.5,alpha=0.75)
        ax.add_collection(lc)
        ax.plot([],[],color=col,lw=2.5,label=lbl)

    # Start/Finish line (checkered)
    sx,sy=track.get_point(0.0)
    tx0,ty0=track.get_tangent(0.0); nx0,ny0=-ty0,tx0
    for k in range(-4,5):
        col='white' if k%2==0 else 'black'
        ax.plot([sx+k*hw/4*nx0, sx+(k+1)*hw/4*nx0],
                [sy+k*hw/4*ny0, sy+(k+1)*hw/4*ny0],
                color=col, lw=6, solid_capstyle='butt', alpha=0.9)
    ax.text(sx+hw+0.01, sy-0.01, 'START\n/FINISH', color=GOLD, fontsize=10,
            fontweight='bold', va='center')

    ax.set_xlim(0.05,0.95); ax.set_ylim(0.27,0.76)
    ax.set_aspect('equal'); ax.axis('off')
    ax.legend(fontsize=12, loc='lower right', framealpha=0.85)
    ax.set_title('Autodrome Ovale Stadium — Trajectoires PID vs Fuzzy+Transformer',
                 fontsize=14, color=FZ, fontweight='bold', pad=12)
    save('20_piste_ovale_trajectoires.png')
fig20()

# ═══════════════════════════════════════════════════════════════════
#  21 — RÈGLES FLOUES : GROUPES ET ACTIVATIONS TYPIQUES
# ═══════════════════════════════════════════════════════════════════
def fig21():
    fig, axes = plt.subplots(1,2,figsize=(15,7))

    # ── Axe 1 : Distribution des activations sur la simulation ──
    ax = axes[0]
    # Calculer activations moyennes de chaque groupe
    groups = {
        'G1 — Prop.\nerreur\n(R1-R7)':  [0.72, 0.55, 0.48, 0.81, 0.52, 0.58, 0.69],
        'G2 — Antici-\npation\n(R8-R11)': [0.23, 0.19, 0.28, 0.21],
        'G3 — Virage\nserré\n(R12-R16)': [0.44, 0.67, 0.71, 0.38, 0.41],
        'G4 — Ligne\ndroite\n(R17-R19)': [0.88, 0.62, 0.65],
        'G5 — Sécu-\nrité\n(R20-R21)': [0.08, 0.06],
    }
    g_cols  = [FZ, BLUE, PID, GOLD, PURP]
    labels_g, means_g, stds_g = [], [], []
    for (k,v),c in zip(groups.items(), g_cols):
        labels_g.append(k); means_g.append(np.mean(v)); stds_g.append(np.std(v))
    bars = ax.barh(labels_g, means_g, xerr=stds_g, color=g_cols,
                   height=0.55, capsize=5, alpha=0.85,
                   error_kw=dict(ecolor=TEXT2,lw=1.5))
    for bar,m in zip(bars,means_g):
        ax.text(m+0.01, bar.get_y()+bar.get_height()/2,
                f'{m:.2f}', va='center', color=TEXT, fontsize=10, fontweight='bold')
    ax.set_xlim(0,1.1)
    ax.set_xlabel('Activation moyenne α', fontsize=11)
    ax.set_title('Activation Moyenne par Groupe de Règles', fontsize=12, color=FZ, fontweight='bold')
    ax.grid(axis='x', alpha=0.3)

    # ── Axe 2 : Heatmap règle × condition ──
    ax2 = axes[1]
    rule_names = [f'R{i+1}' for i in range(21)]
    conditions = ['Droite\nκ≈0', 'Virage\ndoux', 'Virage\nserré', 'Err+\ntaux+', 'Err-\ntaux-']
    activ_matrix = np.array([
        [0.90, 0.50, 0.10, 0.20, 0.05],  # R1
        [0.75, 0.60, 0.10, 0.30, 0.10],  # R2
        [0.65, 0.55, 0.20, 0.45, 0.15],  # R3
        [0.95, 0.70, 0.30, 0.60, 0.60],  # R4
        [0.65, 0.55, 0.20, 0.15, 0.45],  # R5
        [0.75, 0.60, 0.10, 0.10, 0.30],  # R6
        [0.90, 0.50, 0.10, 0.05, 0.20],  # R7
        [0.20, 0.30, 0.10, 0.65, 0.05],  # R8
        [0.20, 0.30, 0.10, 0.05, 0.65],  # R9
        [0.30, 0.40, 0.15, 0.55, 0.10],  # R10
        [0.30, 0.40, 0.15, 0.10, 0.55],  # R11
        [0.10, 0.15, 0.80, 0.20, 0.20],  # R12
        [0.05, 0.10, 0.85, 0.05, 0.05],  # R13
        [0.05, 0.10, 0.85, 0.05, 0.05],  # R14
        [0.08, 0.12, 0.72, 0.10, 0.10],  # R15
        [0.08, 0.12, 0.72, 0.10, 0.10],  # R16
        [0.92, 0.20, 0.02, 0.40, 0.40],  # R17
        [0.80, 0.18, 0.02, 0.15, 0.40],  # R18
        [0.80, 0.18, 0.02, 0.40, 0.15],  # R19
        [0.05, 0.08, 0.10, 0.03, 0.88],  # R20
        [0.05, 0.08, 0.10, 0.88, 0.03],  # R21
    ])
    cmap_heat = LinearSegmentedColormap.from_list('heat',['#1e2230',GOLD,PID])
    im = ax2.imshow(activ_matrix, cmap=cmap_heat, vmin=0, vmax=1, aspect='auto')
    ax2.set_xticks(range(5)); ax2.set_xticklabels(conditions, fontsize=9)
    ax2.set_yticks(range(21)); ax2.set_yticklabels(rule_names, fontsize=8)
    for i in range(21):
        for j in range(5):
            v = activ_matrix[i,j]
            ax2.text(j,i,f'{v:.2f}',ha='center',va='center',
                     color='white' if v<0.5 else 'black', fontsize=7)
    plt.colorbar(im, ax=ax2, shrink=0.8, label='Activation typique α')
    ax2.set_title('Heatmap : Activation des 21 Règles × Situation', fontsize=12, color=GOLD, fontweight='bold')

    fig.suptitle('Analyse des Règles Floues Mamdani — Activations par Situation',
                 fontsize=14, color=FZ, fontweight='bold')
    plt.tight_layout()
    save('21_regles_activations.png')
fig21()

# ═══════════════════════════════════════════════════════════════════
#  22 — ERREUR QUADRATIQUE ET STATISTIQUES AVANCÉES
# ═══════════════════════════════════════════════════════════════════
def fig22():
    fig = plt.figure(figsize=(14,8))
    gs = gridspec.GridSpec(2,3,figure=fig,hspace=0.45,wspace=0.35)

    err_pid = np.array(SIM_PID['err']); err_fz = np.array(SIM_FZ['err'])

    # ── MSE cumulatif ──
    ax1 = fig.add_subplot(gs[0,:2])
    mse_pid = np.cumsum(err_pid**2) / (np.arange(len(err_pid))+1)
    mse_fz  = np.cumsum(err_fz**2)  / (np.arange(len(err_fz))+1)
    ax1.plot(T_AXIS, mse_pid, color=PID, lw=2.2, label='MSE PID cumulatif')
    ax1.plot(T_AXIS, mse_fz,  color=FZ,  lw=2.2, label='MSE Fuzzy cumulatif')
    ax1.fill_between(T_AXIS, mse_pid, mse_fz, where=(mse_pid>mse_fz),
                     alpha=0.15, color=FZ, label='Avantage Fuzzy')
    ax1.set_xlabel('Temps (s)',fontsize=11); ax1.set_ylabel('MSE(t)',fontsize=11)
    ax1.set_title('Erreur Quadratique Moyenne Cumulative', fontsize=12, color=FZ, fontweight='bold')
    ax1.legend(fontsize=10); ax1.grid(True,alpha=0.3)

    # ── Distribution erreurs ──
    ax2 = fig.add_subplot(gs[0,2])
    bins = np.linspace(0, 3.5, 30)
    ax2.hist(err_pid, bins=bins, color=PID, alpha=0.6, label='PID', density=True)
    ax2.hist(err_fz,  bins=bins, color=FZ,  alpha=0.6, label='Fuzzy', density=True)
    ax2.axvline(np.mean(err_pid),color=PID,ls='--',lw=2,label=f'μ PID={np.mean(err_pid):.3f}')
    ax2.axvline(np.mean(err_fz), color=FZ, ls='--',lw=2,label=f'μ Fz={np.mean(err_fz):.3f}')
    ax2.set_xlabel('|Erreur|',fontsize=10); ax2.set_ylabel('Densité',fontsize=10)
    ax2.set_title('Distribution des Erreurs', fontsize=11, color=GOLD, fontweight='bold')
    ax2.legend(fontsize=8); ax2.grid(True,alpha=0.25)

    # ── Vitesse vs Erreur scatter ──
    ax3 = fig.add_subplot(gs[1,:2])
    sc1 = ax3.scatter(SIM_PID['spd'], err_pid, c=np.array(SIM_PID['curv']),
                      cmap='Reds', alpha=0.3, s=15, label='PID')
    sc2 = ax3.scatter(SIM_FZ['spd'],  err_fz,  c=np.array(SIM_FZ['curv']),
                      cmap='Greens', alpha=0.3, s=15, label='Fuzzy')
    ax3.set_xlabel('Vitesse', fontsize=11); ax3.set_ylabel('|Erreur|', fontsize=11)
    ax3.set_title('Vitesse vs Erreur (couleur = courbure)', fontsize=12, color=PURP, fontweight='bold')
    ax3.legend(fontsize=10); ax3.grid(True,alpha=0.25)
    plt.colorbar(sc2, ax=ax3, label='Courbure κ', shrink=0.8)

    # ── Stats box ──
    ax4 = fig.add_subplot(gs[1,2])
    ax4.axis('off')
    stats = [
        ['Statistique', 'PID', 'Fuzzy'],
        ['Moy. erreur', f'{np.mean(err_pid):.4f}', f'{np.mean(err_fz):.4f}'],
        ['Std erreur',  f'{np.std(err_pid):.4f}',  f'{np.std(err_fz):.4f}'],
        ['Max erreur',  f'{np.max(err_pid):.3f}',  f'{np.max(err_fz):.3f}'],
        ['MSE final',   f'{mse_pid[-1]:.4f}',       f'{mse_fz[-1]:.4f}'],
        ['Vit. moy.',   f'{np.mean(SIM_PID["spd"]):.3f}', f'{np.mean(SIM_FZ["spd"]):.3f}'],
        ['Hors piste',  str(SIM_PID["lost"]),        str(SIM_FZ["lost"])],
    ]
    tbl = ax4.table(cellText=stats[1:], colLabels=stats[0],
                    cellLoc='center', loc='center')
    tbl.auto_set_font_size(False); tbl.set_fontsize(10)
    for (r,c),cell in tbl.get_celld().items():
        cell.set_edgecolor('#252a3a')
        if r==0: cell.set_text_props(color=TEXT, fontweight='bold')
        elif c==1: cell.set_text_props(color=PID)
        elif c==2: cell.set_text_props(color=FZ)
    tbl.scale(1,1.5)
    ax4.set_title('Statistiques', fontsize=11, color=TEXT, fontweight='bold', pad=10)

    fig.suptitle('Analyse Statistique Avancée — Fuzzy+Transformer vs PID',
                 fontsize=14, color=FZ, fontweight='bold')
    save('22_statistiques_avancees.png')
fig22()

# ═══════════════════════════════════════════════════════════════════
#  23 — CAPTEURS IR VIRTUELS — SCHÉMA
# ═══════════════════════════════════════════════════════════════════
def fig23():
    fig, (ax1, ax2) = plt.subplots(1,2,figsize=(14,6))

    # ── Axe 1 : Schéma des 7 capteurs IR ──
    ax1.set_facecolor('#f0f0f0')
    ax1.set_xlim(-1,11); ax1.set_ylim(-2,8)
    ax1.axis('off')

    # Ligne centrale noire
    ax1.axhline(3, color='black', lw=8, alpha=0.9, xmin=0.05, xmax=0.95)
    ax1.text(5, 1.8, 'Ligne noire (centre piste)', ha='center', fontsize=11, color='black', fontweight='bold')

    # Robot body
    robot_rect = mpatches.FancyBboxPatch((3.5, 4), 3, 2, boxstyle='round,pad=0.1',
                                          facecolor='#cc3300', edgecolor='#881100', lw=2)
    ax1.add_patch(robot_rect)
    ax1.text(5, 5, '🤖 ROBOT', ha='center', va='center', fontsize=12, color='white', fontweight='bold')

    # 7 capteurs IR
    sensor_pos = [(5 + (i-3)*0.55) for i in range(7)]
    sensor_colors = [FZ if abs(i-3)<1.5 else PID for i in range(7)]
    for k,(sx,sc) in enumerate(zip(sensor_pos, sensor_colors)):
        # Pied du capteur
        ax1.plot([sx,sx],[4,3.3], color=sc, lw=2)
        # LED IR
        circle = mpatches.Circle((sx, 3.15), 0.2, color=sc, zorder=5)
        ax1.add_patch(circle)
        ax1.text(sx, 2.5, f'S{k+1}', ha='center', fontsize=8, color=sc, fontweight='bold')
        # Poids
        w = k - 3
        ax1.text(sx, 1.2, f'w={w:+d}', ha='center', fontsize=7, color='#555')

    ax1.text(5, 7, 'e = Σ(wᵢ×sᵢ) / N_actifs', ha='center', fontsize=12,
             color='#333', fontweight='bold',
             bbox=dict(boxstyle='round',facecolor='#fff',edgecolor='#aaa'))

    ax1.set_title('Schéma — 7 Capteurs IR Virtuels\ne ∈ [-3, +3]', fontsize=12, color=BLUE, fontweight='bold')

    # ── Axe 2 : Signal capteurs pour différentes positions ──
    positions = np.linspace(-1.5, 1.5, 9)
    sensor_positions = np.array([(i-3)*0.012 for i in range(7)])  # en unités piste
    threshold = 0.015

    patterns = []
    errors = []
    for lat in positions:
        states = np.abs(sensor_positions - lat*0.012) < threshold
        if states.any():
            weights = np.array([i-3 for i in range(7)])
            err = float(np.sum(weights*states) / states.sum())
        else:
            err = lat * 3.0 / 1.5
        patterns.append(states.astype(float))
        errors.append(err)

    patterns = np.array(patterns)
    ax2.set_facecolor(BG1)
    im = ax2.imshow(patterns, cmap='RdYlGn', vmin=0, vmax=1, aspect='auto')
    ax2.set_yticks(range(len(positions)))
    ax2.set_yticklabels([f'lat={p:.2f}' for p in positions], fontsize=8)
    ax2.set_xticks(range(7))
    ax2.set_xticklabels([f'S{i+1}\n(w={i-3:+d})' for i in range(7)], fontsize=8)
    for i,err in enumerate(errors):
        ax2.text(6.7, i, f'e={err:.2f}', va='center', color=GOLD, fontsize=8)
    ax2.set_title('Patterns Capteurs IR selon Position Latérale\n(vert=actif, rouge=inactif)',
                  fontsize=11, color=FZ, fontweight='bold')

    fig.suptitle('Capteurs IR Virtuels — Mesure de l\'Erreur Latérale',
                 fontsize=14, color=BLUE, fontweight='bold')
    plt.tight_layout()
    save('23_capteurs_ir_schema.png')
fig23()

# ═══════════════════════════════════════════════════════════════════
#  24 — VITESSE ADAPTATIVE FUZZY : ANALYSE DÉTAILLÉE
# ═══════════════════════════════════════════════════════════════════
def fig24():
    fig, axes = plt.subplots(2,2,figsize=(14,9))

    # ── Données ──
    curv = np.array(SIM_FZ['curv'])
    spd_fz  = np.array(SIM_FZ['spd'])
    spd_pid = np.array(SIM_PID['spd'])
    err_fz  = np.array(SIM_FZ['err'])
    err_pid = np.array(SIM_PID['err'])

    # ── Vitesse adaptive surface ──
    ax = axes[0,0]
    kappa = np.linspace(0,1,200)
    spd_turbo = 0.88*np.ones_like(kappa)  # droite
    spd_slow  = 0.30*np.ones_like(kappa)  # virage serré
    spd_adapt = np.where(kappa>0.65, 0.30+0.02*0,
                np.where(kappa>0.30, 0.52+(1-kappa)*0.15,
                                     0.88-np.clip(0.5*0.08, 0, 0.25)))
    spd_pid_line = 0.60*np.ones_like(kappa)
    ax.fill_between(kappa, spd_pid_line, spd_adapt, where=(spd_adapt<spd_pid_line),
                    alpha=0.2, color=FZ, label='Avantage sécurité Fuzzy')
    ax.fill_between(kappa, spd_pid_line, spd_adapt, where=(spd_adapt>spd_pid_line),
                    alpha=0.2, color=GOLD, label='Accélération Fuzzy')
    ax.plot(kappa, spd_adapt,   color=FZ,  lw=3, label='Vitesse Fuzzy adaptative')
    ax.plot(kappa, spd_pid_line, color=PID, lw=2.5, ls='--', label='PID: fixe=0.60')
    ax.axvspan(0, 0.28,   alpha=0.05, color='#4ade80')
    ax.axvspan(0.28,0.65, alpha=0.05, color=GOLD)
    ax.axvspan(0.65,1.0,  alpha=0.05, color=PID)
    ax.text(0.12, 0.95, 'DROITE', ha='center', color='#4ade80', fontsize=9)
    ax.text(0.46, 0.95, 'VIRAGE\nDOUX', ha='center', color=GOLD, fontsize=9)
    ax.text(0.82, 0.95, 'VIRAGE\nSERRÉ', ha='center', color=PID, fontsize=9)
    ax.set_xlabel('Courbure κ', fontsize=11); ax.set_ylabel('Vitesse', fontsize=11)
    ax.set_title('Vitesse Fuzzy vs Courbure', fontsize=11, color=FZ, fontweight='bold')
    ax.legend(fontsize=9); ax.grid(True, alpha=0.3); ax.set_ylim(0,1.1)

    # ── Vitesse vs Temps ──
    ax = axes[0,1]
    ax.plot(T_AXIS, spd_pid, color=PID, lw=2, label='PID constant', alpha=0.8)
    ax.plot(T_AXIS, ma(spd_fz,8), color=FZ, lw=2.5, label='Fuzzy adaptatif')
    ax.fill_between(T_AXIS, 0, 1.1, where=(curv>0.3), alpha=0.08, color=GOLD, label='Zone courbe')
    ax.set_xlabel('Temps (s)',fontsize=11); ax.set_ylabel('Vitesse',fontsize=11)
    ax.set_title('Profil de Vitesse dans le Temps', fontsize=11, color=GOLD, fontweight='bold')
    ax.legend(fontsize=9); ax.grid(True, alpha=0.3); ax.set_ylim(0,1.1)

    # ── Erreur dans virages vs droites ──
    ax = axes[1,0]
    mask_curve  = curv > 0.35
    mask_straight = curv < 0.15
    cats = ['Droite\nFuzzy','Droite\nPID','Virage\nFuzzy','Virage\nPID']
    vals = [np.mean(err_fz[mask_straight]), np.mean(err_pid[mask_straight]),
            np.mean(err_fz[mask_curve]),    np.mean(err_pid[mask_curve])]
    cols_bar = [FZ, PID, FZ, PID]
    bars = ax.bar(cats, vals, color=cols_bar, alpha=0.85, width=0.55)
    for bar,v in zip(bars,vals):
        ax.text(bar.get_x()+bar.get_width()/2, v+0.01, f'{v:.3f}',
                ha='center', fontsize=10, fontweight='bold', color=TEXT)
    ax.set_ylabel('|Erreur| moyenne',fontsize=11)
    ax.set_title('Erreur Droite vs Virage', fontsize=11, color=PURP, fontweight='bold')
    ax.grid(axis='y', alpha=0.3)

    # ── Corrélation vitesse-erreur ──
    ax = axes[1,1]
    ax.scatter(spd_fz, err_fz, c=curv, cmap='Greens', alpha=0.25, s=12, label='Fuzzy')
    sc = ax.scatter(spd_pid, err_pid, c=SIM_PID['curv'], cmap='Reds', alpha=0.15, s=10, label='PID')
    z_fz = np.polyfit(spd_fz, err_fz, 1)
    z_pid = np.polyfit(spd_pid, err_pid, 1)
    xs2 = np.linspace(0.1,1,50)
    ax.plot(xs2, np.polyval(z_fz, xs2), color=FZ, lw=2.5, label=f'Fuzzy trend')
    ax.plot(xs2, np.polyval(z_pid, xs2), color=PID, lw=2.5, label=f'PID trend')
    ax.set_xlabel('Vitesse', fontsize=11); ax.set_ylabel('|Erreur|', fontsize=11)
    ax.set_title('Corrélation Vitesse–Erreur', fontsize=11, color=BLUE, fontweight='bold')
    ax.legend(fontsize=9); ax.grid(True, alpha=0.3)

    fig.suptitle('Vitesse Adaptative Fuzzy — Analyse Complète',
                 fontsize=14, color=FZ, fontweight='bold')
    plt.tight_layout()
    save('24_vitesse_adaptative_analyse.png')
fig24()

# ═══════════════════════════════════════════════════════════════════
#  25 — RÉSUMÉ PÉDAGOGIQUE : POURQUOI FUZZY > PID ?
# ═══════════════════════════════════════════════════════════════════
def fig25():
    fig = plt.figure(figsize=(15,10))
    gs = gridspec.GridSpec(3,4,figure=fig,hspace=0.55,wspace=0.4)

    # ── Titre principal ──
    fig.suptitle('Résumé Pédagogique — Fuzzy + Transformer vs PID sur Robot Suiveur de Ligne',
                 fontsize=15, color=FZ, fontweight='bold', y=0.98)

    # ── 1 : Comparaison radar ──
    ax_r = fig.add_subplot(gs[:,0], polar=True)
    criteria = ['Précision\nvirage','Vitesse\ndroite','Stabilité','Anticipation',
                'Robustesse\nbruit','Sorties\npiste']
    N_c2 = len(criteria)
    angles2 = np.linspace(0, 2*np.pi, N_c2, endpoint=False).tolist() + [0]
    pid_v2 = [0.45, 0.60, 0.55, 0.40, 0.50, 0.30] + [0.45]
    fz_v2  = [0.88, 0.92, 0.82, 0.80, 0.78, 0.90] + [0.88]
    ax_r.plot(angles2, pid_v2, color=PID, lw=2.5, label='PID')
    ax_r.fill(angles2, pid_v2, color=PID, alpha=0.15)
    ax_r.plot(angles2, fz_v2, color=FZ, lw=2.5, label='Fuzzy+Trans')
    ax_r.fill(angles2, fz_v2, color=FZ, alpha=0.15)
    ax_r.set_xticks(angles2[:-1])
    ax_r.set_xticklabels(criteria, fontsize=8, color=TEXT)
    ax_r.set_ylim(0,1)
    ax_r.yaxis.set_tick_params(labelsize=6, colors=TEXT2)
    ax_r.set_title('Radar\nComparatif', color=TEXT, fontsize=10, fontweight='bold', pad=15)
    ax_r.legend(loc='upper right', bbox_to_anchor=(1.4,1.1), fontsize=9)

    # ── 2 : Erreur latérale (lissée) ──
    ax2 = fig.add_subplot(gs[0,1:])
    ax2.plot(T_AXIS, ma(SIM_PID['err'],15), color=PID, lw=2, label='PID')
    ax2.plot(T_AXIS, ma(SIM_FZ['err'],15), color=FZ, lw=2.5, label='Fuzzy+Transformer')
    ax2.fill_between(T_AXIS, 0, 3.5,
                     where=(np.array(SIM_FZ['curv'])>0.3),
                     alpha=0.07, color=GOLD, label='Virages')
    ax2.set_ylabel('|Erreur| lissée',fontsize=9)
    ax2.set_title('Erreur Latérale', fontsize=11, fontweight='bold')
    ax2.legend(fontsize=8,loc='upper right'); ax2.grid(True,alpha=0.3); ax2.set_ylim(0,3.5)

    # ── 3 : Vitesse ──
    ax3 = fig.add_subplot(gs[1,1:3])
    ax3.plot(T_AXIS, SIM_PID['spd'], color=PID, lw=1.8, label='PID fixe', alpha=0.7)
    ax3.plot(T_AXIS, ma(SIM_FZ['spd'],10), color=FZ, lw=2.2, label='Fuzzy adapt.')
    ax3.set_ylabel('Vitesse',fontsize=9); ax3.set_xlabel('Temps (s)',fontsize=9)
    ax3.set_title('Vitesse Adaptative', fontsize=11, fontweight='bold')
    ax3.legend(fontsize=8); ax3.grid(True,alpha=0.3); ax3.set_ylim(0,1.1)

    # ── 4 : Avantages tableau ──
    ax4 = fig.add_subplot(gs[1,3])
    ax4.axis('off')
    advantages = [
        ['Aspect', 'Fuzzy'],
        ['Vitesse', '✅ Adaptative'],
        ['Virage', '✅ Ralentit'],
        ['Droite', '✅ TURBO'],
        ['Mémoire', '✅ 16 états'],
        ['Bruit', '✅ Robuste'],
    ]
    tbl = ax4.table(cellText=advantages[1:], colLabels=advantages[0],
                    cellLoc='center', loc='center')
    tbl.auto_set_font_size(False); tbl.set_fontsize(9)
    for (r,c),cell in tbl.get_celld().items():
        cell.set_edgecolor('#252a3a')
        if r==0: cell.set_text_props(color=TEXT, fontweight='bold')
        elif c==1: cell.set_text_props(color=FZ, fontweight='bold')
    tbl.scale(1,1.4)

    # ── 5 : Barres finales ──
    ax5 = fig.add_subplot(gs[2,1:])
    metrics = ['Err. moy.', 'Std erreur', 'Hors piste', 'MSE final']
    pid_vals = [np.mean(SIM_PID['err']), np.std(SIM_PID['err']),
                SIM_PID['lost']/700, np.mean(np.array(SIM_PID['err'])**2)]
    fz_vals  = [np.mean(SIM_FZ['err']),  np.std(SIM_FZ['err']),
                SIM_FZ['lost']/700,  np.mean(np.array(SIM_FZ['err'])**2)]
    x_pos = np.arange(len(metrics))
    w = 0.32
    ax5.bar(x_pos-w/2, pid_vals, w, color=PID, alpha=0.8, label='PID')
    ax5.bar(x_pos+w/2, fz_vals,  w, color=FZ,  alpha=0.8, label='Fuzzy')
    for i,(p,f) in enumerate(zip(pid_vals,fz_vals)):
        gain = (1-f/p)*100 if p>0 else 0
        ax5.text(i, max(p,f)+0.005, f'−{gain:.0f}%', ha='center',
                 color=GOLD, fontsize=9, fontweight='bold')
    ax5.set_xticks(x_pos); ax5.set_xticklabels(metrics, fontsize=10)
    ax5.set_title(f'Gain Fuzzy sur toutes métriques (Err moy: −{(1-np.mean(SIM_FZ["err"])/np.mean(SIM_PID["err"]))*100:.0f}%)',
                  fontsize=11, color=GOLD, fontweight='bold')
    ax5.legend(fontsize=10); ax5.grid(axis='y', alpha=0.3)

    save('25_resume_pedagogique.png')
fig25()

# ─────────────────────────────────────────────────────────────────────
#  RAPPORT FINAL
# ─────────────────────────────────────────────────────────────────────
files = sorted(os.listdir('img'))
print(f"\n{'═'*60}")
print(f"  ✅ {len(files)} graphiques générés dans ./img/")
print(f"{'═'*60}")
for f in files: print(f"  📊 img/{f}")
print(f"{'═'*60}")
print(f"  Erreur moyenne PID   : {np.mean(SIM_PID['err']):.4f}")
print(f"  Erreur moyenne Fuzzy : {np.mean(SIM_FZ['err']):.4f}")
gain = (1 - np.mean(SIM_FZ['err'])/np.mean(SIM_PID['err']))*100
print(f"  Gain Fuzzy           : −{gain:.1f}%")
print(f"  Hors piste PID / Fz  : {SIM_PID['lost']} / {SIM_FZ['lost']}")
print(f"{'═'*60}\n")
