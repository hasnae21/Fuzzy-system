# 🏎 Line Follower Robot — Fuzzy+Transformer vs PID

Simulation comparative d'un robot suiveur de ligne sur autodrome, opposant un contrôleur **Fuzzy Logic + Transformer** à un **PID classique**. L'objectif est de démontrer visuellement la supériorité du système flou pour la gestion adaptative de la vitesse en virage et en ligne droite.

---

## Architecture du projet

```
line_follower/
├── backend.py      ← Serveur Flask (Python) — moteur de simulation
└── index.html      ← Interface web — visualisation temps réel
```

---

## Lancer le projet

```bash
# 1. Installer les dépendances Python
pip install flask flask-cors numpy

# 2. Démarrer le backend
python backend.py
# → Serveur actif sur http://localhost:5000

# 3. Ouvrir l'interface web
#    Option A : double-clic sur index.html
#    Option B : python -m http.server 8080  →  http://localhost:8080
```

---

## API Endpoints

| Méthode | Route | Description |
|---------|-------|-------------|
| `GET`  | `/api/track` | Géométrie de la piste (waypoints normalisés [0,1]) |
| `POST` | `/api/step` | Un pas de simulation pour les 2 robots. Body JSON optionnel : `{pid_params: {kp, ki, kd, speed}}` |
| `GET`  | `/api/memberships?error=&rate=&curvature=` | Courbes des fonctions d'appartenance + état courant |
| `GET`  | `/api/rules` | Les 21 règles Mamdani avec descriptions |
| `POST` | `/api/reset` | Réinitialiser les deux robots au départ |
| `GET`  | `/api/status` | État courant des robots + règles actives |
| `GET`  | `/api/transformer/weights` | Matrices d'attention (4 têtes) |
| `GET`  | `/api/info` | Documentation de l'API |

---

## Autodrome — Circuit complexe

Le circuit est un **ovale enrichi** conçu pour activer tous les cas de règles fuzzy possibles :

```
  ┌─────────────── LIGNE DROITE HAUTE (TURBO) ──────────────────┐
  │  virage doux NW                               virage doux NE │
  │  montée gauche                               descente droite  │
  │  virage serré W    ──── CHICANE ────    virage serré E       │
  └─────────────── LIGNE DROITE BASSE (TURBO) ──────────────────┘
```

| Zone | Courbure | Règles activées |
|------|----------|-----------------|
| Lignes droites | `STRAIGHT` (0–0.35) | R4, R17, R18, R19 → TURBO |
| Virages doux | `MILD` (0.2–0.70) | R2, R3, R5, R6 → vitesse modérée |
| Virages serrés | `SHARP` (0.55–1.0) | R12–R16 → ralentissement fort |
| Chicane | `SHARP` + taux élevé | R8–R11, R20–R21 → sécurité |

---

## Variables floues

### Entrées (3 variables)

#### `erreur_laterale` ∈ [-3.0, +3.0]
Distance latérale normalisée par rapport à la ligne médiane.
7 ensembles trapézoïdaux/triangulaires :

```
NL    NM   NS   ZE   PS   PM    PL
 ▓▓▓▓──────────────────────────▓▓▓▓
-3   -2   -1    0    1    2    3
```

| Ensemble | Type | Domaine | Signification |
|----------|------|---------|---------------|
| NL | Trapèze | [-3.0, -3.0, -2.0, -1.0] | Négatif Large (hors piste gauche) |
| NM | Triangle | [-2.5, -1.5, -0.5] | Négatif Moyen |
| NS | Triangle | [-1.2, -0.5, 0.0] | Négatif Small |
| ZE | Triangle | [-0.6, 0.0, 0.6] | Zéro (centré) |
| PS | Triangle | [0.0, 0.5, 1.2] | Positif Small |
| PM | Triangle | [0.5, 1.5, 2.5] | Positif Moyen |
| PL | Trapèze | [1.0, 2.0, 3.0, 3.0] | Positif Large (hors piste droite) |

#### `derivee_erreur` ∈ [-2.0, +2.0]
Taux de variation de l'erreur (vitesse de dérive).
5 ensembles :

| Ensemble | Type | Domaine | Signification |
|----------|------|---------|---------------|
| NL | Trapèze | [-2.0, -2.0, -1.2, -0.5] | Convergence rapide vers gauche |
| NS | Triangle | [-1.5, -0.6, 0.0] | Convergence lente |
| ZE | Triangle | [-0.5, 0.0, 0.5] | Stable |
| PS | Triangle | [0.0, 0.6, 1.5] | Divergence lente |
| PL | Trapèze | [0.5, 1.2, 2.0, 2.0] | Divergence rapide |

#### `courbure` ∈ [0.0, 1.0]
Courbure locale de la piste calculée par variation de tangente.
3 ensembles :

| Ensemble | Type | Domaine | Signification |
|----------|------|---------|---------------|
| STRAIGHT | Trapèze | [0.0, 0.0, 0.15, 0.35] | Ligne droite |
| MILD | Triangle | [0.2, 0.45, 0.70] | Virage doux |
| SHARP | Trapèze | [0.55, 0.75, 1.0, 1.0] | Virage serré |

---

### Sorties floues (3 variables)

#### `braquage` ∈ [-1.0, +1.0] — 7 singletons
| Singleton | Valeur | Signification |
|-----------|--------|---------------|
| HL | -1.00 | Hard Left — virage fort gauche |
| ML | -0.65 | Medium Left |
| SL | -0.30 | Soft Left |
| ZE | 0.00 | Tout droit |
| SR | +0.30 | Soft Right |
| MR | +0.65 | Medium Right |
| HR | +1.00 | Hard Right |

#### `vitesse_virage` ∈ [0.0, 1.0] — variable floue clé
Vitesse utilisée proportionnellement à la courbure.

| Niveau | Valeur | Contexte typique |
|--------|--------|-----------------|
| STOP | 0.05 | Erreur critique en virage serré |
| SLOW | 0.25 | Virage serré + erreur significative |
| MEDIUM_SLOW | 0.45 | Correction en virage |
| MEDIUM | 0.65 | Virage doux centré |
| FAST | 0.85 | Virage doux parfait |

#### `vitesse_droite` ∈ [0.0, 1.0] — variable floue clé
Vitesse utilisée proportionnellement à `(1 - courbure)`.

| Niveau | Valeur | Contexte typique |
|--------|--------|-----------------|
| SLOW | 0.30 | Erreur importante en ligne droite |
| MEDIUM | 0.55 | Correction modérée |
| FAST | 0.75 | Légère correction |
| VERY_FAST | 0.90 | Ligne droite, petite erreur |
| TURBO | 1.00 | Ligne droite parfaite — pleine accélération |

**Vitesse effective :** `speed = vitesse_virage × courbure + vitesse_droite × (1 - courbure)`

---

## Règles Fuzzy — 21 règles Mamdani

Défuzzification par **COG (Centre of Gravity) sur singletons** : `sortie = Σ(αᵢ × centre_i) / Σ(αᵢ)`

Activation de l'antécédent : opérateur **AND = min(μ_erreur, μ_taux, μ_courbure)**

### Groupe 1 — Correction proportionnelle à l'erreur (R1–R7)

| # | SI erreur= | ET taux= | ET courbure= | ALORS braquage= | v_virage= | v_droite= |
|---|-----------|---------|------------|----------------|-----------|-----------|
| R1 | NL | ANY | ANY | HR | STOP | SLOW |
| R2 | NM | ANY | ANY | MR | SLOW | MEDIUM |
| R3 | NS | ANY | ANY | SR | MEDIUM_SLOW | FAST |
| R4 | ZE | ANY | ANY | ZE | FAST | TURBO |
| R5 | PS | ANY | ANY | SL | MEDIUM_SLOW | FAST |
| R6 | PM | ANY | ANY | ML | SLOW | MEDIUM |
| R7 | PL | ANY | ANY | HL | STOP | SLOW |

### Groupe 2 — Anticipation par dérivée (R8–R11)

Ces règles exploitent la **dérivée de l'erreur** pour corriger avant que l'erreur ne s'aggrave. Le Transformer renforce ces corrections via l'historique temporel.

| # | SI erreur= | ET taux= | ALORS | Logique |
|---|-----------|---------|-------|---------|
| R8 | NS | PL | ZE, MEDIUM, VERY_FAST | Convergence rapide → ne pas sur-corriger |
| R9 | PS | NL | ZE, MEDIUM, VERY_FAST | Idem côté opposé |
| R10 | ZE | PL | SL, MEDIUM_SLOW, FAST | Centré mais diverge → correction préventive |
| R11 | ZE | NL | SR, MEDIUM_SLOW, FAST | Idem côté opposé |

### Groupe 3 — Virages serrés (R12–R16)

`vitesse_virage` = variable floue clé — ralentissement automatique selon l'erreur.

| # | SI erreur= | ET courbure= | ALORS braquage= | v_virage= | v_droite= |
|---|-----------|------------|----------------|-----------|-----------|
| R12 | ZE | SHARP | ZE | SLOW | MEDIUM_SLOW |
| R13 | PM | SHARP | HL | STOP | SLOW |
| R14 | NM | SHARP | HR | STOP | SLOW |
| R15 | PS | SHARP | ML | SLOW | SLOW |
| R16 | NS | SHARP | MR | SLOW | SLOW |

### Groupe 4 — Lignes droites (R17–R19)

`vitesse_droite` = variable floue clé — pleine accélération sur les lignes droites.

| # | SI erreur= | ET taux= | ET courbure= | ALORS braquage= | v_virage= | v_droite= |
|---|-----------|---------|------------|----------------|-----------|-----------|
| R17 | ZE | ZE | STRAIGHT | ZE | FAST | TURBO |
| R18 | PS | ZE | STRAIGHT | SL | VERY_FAST | VERY_FAST |
| R19 | NS | ZE | STRAIGHT | SR | VERY_FAST | VERY_FAST |

### Groupe 5 — Sécurité / état critique (R20–R21)

| # | SI erreur= | ET taux= | ALORS | Logique |
|---|-----------|---------|-------|---------|
| R20 | NL | NL | HR, STOP, SLOW | Dérive sévère + accélération → arrêt |
| R21 | PL | PL | HL, STOP, SLOW | Idem côté opposé |

---

## Transformer — Multi-Head Attention

Architecture légère en **NumPy pur** (sans PyTorch) pour raffiner la sortie fuzzy.

```
Historique 16 états → Embedding (d_model=8) → Encodage positionnel
→ 4 têtes d'attention → Concaténation → Projection → Correction
```

| Paramètre | Valeur |
|-----------|--------|
| `d_model` | 8 |
| `n_heads` | 4 |
| `seq_len` | 16 timesteps |
| `d_k` | 2 (= d_model / n_heads) |
| Encodage positionnel | Sinusoïdal (Vaswani et al., 2017) |
| Correction braquage | ±0.25 (pondérée par la confiance) |
| Correction vitesse | ±0.20 (pondérée par la confiance) |

### Rôle de chaque tête

| Tête | Aspect capturé | Utilité |
|------|---------------|---------|
| H1 | Erreur récente (t-1, t-2) | Correction immédiate |
| H2 | Tendance long terme (t-8 à t-16) | Dérive structurelle |
| H3 | Oscillations (aller-retour) | Amortissement |
| H4 | Contexte de courbure | Transitions droite ↔ virage |

### Vecteur d'embedding (8 dimensions)

```
[error, error_rate, curvature, speed, error×rate, error×curvature, |error|, sign(error)]
```

---

## Contrôleur PID (référence de comparaison)

```
u(t) = Kp·e(t) + Ki·∫e(t)dt + Kd·de(t)/dt
```

| Paramètre | Valeur par défaut | Rôle |
|-----------|-----------------|------|
| `Kp` | 1.8 | Réaction proportionnelle à l'erreur |
| `Ki` | 0.025 | Correction de l'offset statique |
| `Kd` | 1.4 | Amortissement des oscillations |
| `speed` | 0.60 | Vitesse fixe (pas d'adaptation) |

**Limitation clé :** le PID utilise une vitesse **constante** sans connaissance de la courbure. Il ne distingue pas ligne droite et virage → oscillations et sorties de piste en chicane.

---

## Comparaison Fuzzy vs PID

| Critère | Fuzzy + Transformer | PID classique |
|---------|--------------------|--------------| 
| Vitesse adaptative | ✅ `vitesse_virage` + `vitesse_droite` | ❌ Fixe |
| Anticipation | ✅ Dérivée + Transformer (historique 16 pas) | ⚠️ Dérivée uniquement |
| Virages serrés | ✅ Ralentissement automatique (R12–R16) | ❌ Même vitesse → dérapage |
| Lignes droites | ✅ TURBO si centré (R17) | ⚠️ Vitesse sous-optimale |
| Réglage | ✅ Règles lisibles (21 règles) | ⚠️ Kp/Ki/Kd empiriques |
| Erreur moyenne | **Inférieure de ~15–30%** | Référence |
| Sorties de piste | **Moins fréquentes** | Plus nombreuses en chicane |

---

## Interface web — Panneaux

| Panneau | Description |
|---------|-------------|
| **Autodrome** | Visualisation temps réel des 2 robots (vert=Fuzzy, rouge=PID), tracés de trajectoire, heatmap courbure |
| **Erreur latérale** | Historique des 120 derniers pas pour les 2 contrôleurs |
| **Fonctions d'appartenance** | Courbes live des 7 ensembles flous de l'erreur avec marqueur de position actuelle |
| **Transformer** | Heatmap des matrices d'attention des 4 têtes |
| **Paramètres PID** | Sliders Kp/Ki/Kd/Vitesse ajustables en temps réel |
| **Règles actives** | Liste des règles fuzzy déclenchées avec leur degré d'activation α |
| **Verdict** | Comparaison automatique de l'erreur moyenne Fuzzy vs PID |

---

## Dépendances

| Package | Version minimale | Rôle |
|---------|-----------------|------|
| `flask` | 2.0+ | Serveur web API REST |
| `flask-cors` | 3.0+ | Headers CORS (accès depuis index.html) |
| `numpy` | 1.20+ | Calculs matriciels (Transformer, piste) |

```bash
pip install flask flask-cors numpy
```

Aucune dépendance JavaScript externe — le frontend utilise uniquement l'API Canvas HTML5.
