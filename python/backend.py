"""
╔══════════════════════════════════════════════════════════════════════════════╗
║   LINE FOLLOWER ROBOT — Backend Python (VERSION FINALE)                      ║
║                                                                               ║
║   Architecture :                                                              ║
║     • Contrôleur FUZZY Mamdani (21 règles, fonctions trapézoïdales)          ║
║     • Transformer simplifié (mémoire 16 états, prédiction par tendance)      ║
║     • Contrôleur PID classique (comparaison)                                 ║
║                                                                               ║
║   VARIABLES D'ENTRÉE (Fuzzification) :                                       ║
║     ┌─────────────────────────────────────────────────────────────────────┐  ║
║     │ 1. erreur_laterale ∈ [-3.0, +3.0]                                   │  ║
║     │    Distance normalisée entre le robot et la ligne médiane           │  ║
║     │    Ensembles : NL, NM, NS, ZE, PS, PM, PL (7 trapèzes/triangles)    │  ║
║     │                                                                      │  ║
║     │ 2. derivee_erreur ∈ [-2.0, +2.0]                                    │  ║
║     │    Taux de variation de l'erreur (vitesse de dérive)                │  ║
║     │    Ensembles : NL, NS, ZE, PS, PL (5 trapèzes/triangles)            │  ║
║     │                                                                      │  ║
║     │ 3. courbure ∈ [0.0, 1.0]                                            │  ║
║     │    Courbure locale de la piste (0=droite, 1=virage serré)           │  ║
║     │    Ensembles : STRAIGHT, MILD, SHARP (3 trapèzes/triangles)         │  ║
║     └─────────────────────────────────────────────────────────────────────┘  ║
║                                                                               ║
║   VARIABLES DE SORTIE (Défuzzification par COG sur singletons) :             ║
║     ┌─────────────────────────────────────────────────────────────────────┐  ║
║     │ 1. braquage ∈ [-1.0, +1.0]                                          │  ║
║     │    Commande de direction                                            │  ║
║     │    Singletons : HL=-1.00, ML=-0.65, SL=-0.30, ZE=0.00,              │  ║
║     │                  SR=+0.30, MR=+0.65, HR=+1.00                       │  ║
║     │                                                                      │  ║
║     │ 2. vitesse_virage ∈ [0.0, 1.0]  (variable floue clé)                │  ║
║     │    Vitesse utilisée en fonction de la courbure                       │  ║
║     │    Singletons : STOP=0.05, SLOW=0.25, MEDIUM_SLOW=0.45,             │  ║
║     │                  MEDIUM=0.65, FAST=0.85                             │  ║
║     │                                                                      │  ║
║     │ 3. vitesse_droite ∈ [0.0, 1.0]  (variable floue clé)                │  ║
║     │    Vitesse utilisée sur les lignes droites                          │  ║
║     │    Singletons : SLOW=0.30, MEDIUM=0.55, FAST=0.75,                  │  ║
║     │                  VERY_FAST=0.90, TURBO=1.00                         │  ║
║     └─────────────────────────────────────────────────────────────────────┘  ║
║                                                                               ║
║   VITESSE EFFECTIVE :                                                        ║
║     speed = vitesse_virage × courbure + vitesse_droite × (1 - courbure)      ║
║                                                                               ║
║   RÈGLES FLOUES MAMDANI (21 règles) :                                         ║
║     ┌─────────────────────────────────────────────────────────────────────┐  ║
║     │ GROUPE 1 — Correction proportionnelle à l'erreur (R1 à R7)          │  ║
║     │   R1: SI erreur=NL   ALORS HR, STOP, SLOW                           │  ║
║     │   R2: SI erreur=NM   ALORS MR, SLOW, MEDIUM                         │  ║
║     │   R3: SI erreur=NS   ALORS SR, MEDIUM_SLOW, FAST                    │  ║
║     │   R4: SI erreur=ZE   ALORS ZE, FAST, TURBO                          │  ║
║     │   R5: SI erreur=PS   ALORS SL, MEDIUM_SLOW, FAST                    │  ║
║     │   R6: SI erreur=PM   ALORS ML, SLOW, MEDIUM                         │  ║
║     │   R7: SI erreur=PL   ALORS HL, STOP, SLOW                           │  ║
║     │                                                                      │  ║
║     │ GROUPE 2 — Anticipation par dérivée (R8 à R11)                      │  ║
║     │   R8:  SI erreur=NS ET taux=PL  ALORS ZE, MEDIUM, VERY_FAST         │  ║
║     │   R9:  SI erreur=PS ET taux=NL  ALORS ZE, MEDIUM, VERY_FAST         │  ║
║     │   R10: SI erreur=ZE ET taux=PL  ALORS SL, MEDIUM_SLOW, FAST         │  ║
║     │   R11: SI erreur=ZE ET taux=NL  ALORS SR, MEDIUM_SLOW, FAST         │  ║
║     │                                                                      │  ║
║     │ GROUPE 3 — Virages serrés (courbure=SHARP) (R12 à R16)              │  ║
║     │   R12: SI erreur=ZE ET courbure=SHARP  ALORS ZE, SLOW, MEDIUM_SLOW  │  ║
║     │   R13: SI erreur=PM ET courbure=SHARP  ALORS HL, STOP, SLOW         │  ║
║     │   R14: SI erreur=NM ET courbure=SHARP  ALORS HR, STOP, SLOW         │  ║
║     │   R15: SI erreur=PS ET courbure=SHARP  ALORS ML, SLOW, SLOW         │  ║
║     │   R16: SI erreur=NS ET courbure=SHARP  ALORS MR, SLOW, SLOW         │  ║
║     │                                                                      │  ║
║     │ GROUPE 4 — Lignes droites (courbure=STRAIGHT) (R17 à R19)           │  ║
║     │   R17: SI erreur=ZE ET taux=ZE ET courbure=STRAIGHT  ALORS ZE, FAST, TURBO│
║     │   R18: SI erreur=PS ET taux=ZE ET courbure=STRAIGHT  ALORS SL, VERY_FAST, VERY_FAST│
║     │   R19: SI erreur=NS ET taux=ZE ET courbure=STRAIGHT  ALORS SR, VERY_FAST, VERY_FAST│
║     │                                                                      │  ║
║     │ GROUPE 5 — Sécurité / état critique (R20 à R21)                     │  ║
║     │   R20: SI erreur=NL ET taux=NL  ALORS HR, STOP, SLOW                │  ║
║     │   R21: SI erreur=PL ET taux=PL  ALORS HL, STOP, SLOW                │  ║
║     └─────────────────────────────────────────────────────────────────────┘  ║
║                                                                               ║
║   INFÉRENCE :                                                                ║
║     • Opérateur AND = MIN (minimum des degrés d'appartenance)                ║
║     • Agrégation = MAX (maximum des activations par singleton)               ║
║     • Défuzzification = COG (Centre of Gravity) sur singletons               ║
║                                                                               ║
║   TRANSFORMER SIMPLIFIÉ :                                                    ║
║     • Mémoire : 16 derniers états [error, rate, curvature, speed]            ║
║     • Prédiction : régression linéaire sur l'historique des erreurs          ║
║     • Correction braquage : ±0.15 max, pondérée par confiance                ║
║     • Correction vitesse : ±0.075 max, pondérée par confiance                ║
║                                                                               ║
║                                                                               ║
║   DIFFÉRENCES CLÉS FUZZY vs PID :                                            ║
║     ┌─────────────────────────────────────────────────────────────────────┐  ║
║     │ Critère              │ Fuzzy + Transformer │ PID classique          │  ║
║     │──────────────────────┼─────────────────────┼────────────────────────│  ║
║     │ Vitesse adaptative   │ ✅ Oui (courbure)   │ ❌ Non (constante)      │  ║
║     │ Anticipation         │ ✅ Dérivée + mémoire│ ⚠️ Dérivée uniquement   │  ║
║     │ Virages serrés       │ ✅ Ralentissement   │ ❌ Même vitesse         │  ║
║     │ Lignes droites       │ ✅ TURBO possible   │ ⚠️ Vitesse sous-optimale│  ║
║     │ Sorties de piste     │ ✅ Moins fréquentes │ ❌ Plus fréquentes      │  ║
║     └─────────────────────────────────────────────────────────────────────┘  ║
╚══════════════════════════════════════════════════════════════════════════════╝
"""

import math
import json
import time
import numpy as np
from flask import Flask, jsonify, request
from flask_cors import CORS
from dataclasses import dataclass, asdict
from typing import List, Dict, Tuple
import threading

app = Flask(__name__)
CORS(app)


# ═══════════════════════════════════════════════════════════════════════════════
# FONCTIONS D'APPARTENANCE
# ═══════════════════════════════════════════════════════════════════════════════

def trapeze(x: float, a: float, b: float, c: float, d: float) -> float:
    """Fonction d'appartenance trapézoïdale"""
    if x <= a or x >= d:
        return 0.0
    if b <= x <= c:
        return 1.0
    if x < b:
        return (x - a) / (b - a) if b != a else 1.0
    return (d - x) / (d - c) if d != c else 1.0


def triangle(x: float, a: float, b: float, c: float) -> float:
    """Fonction d'appartenance triangulaire"""
    if x <= a or x >= c:
        return 0.0
    if x <= b:
        return (x - a) / (b - a) if b != a else 1.0
    return (c - x) / (c - b) if c != b else 1.0


# ═══════════════════════════════════════════════════════════════════════════════
# ENSEMBLES FLOUS
# ═══════════════════════════════════════════════════════════════════════════════

class FuzzySets:
    """Définition des ensembles flous pour les entrées et sorties"""

    # ──────────────────────────────────────────────────────────────────────────
    # ENTRÉE 1 : ERREUR LATÉRALE ∈ [-3.0, +3.0]
    # ──────────────────────────────────────────────────────────────────────────
    @staticmethod
    def error_NL(e): return trapeze(e, -3.0, -3.0, -2.0, -1.0)  # Négatif Large
    @staticmethod
    def error_NM(e): return triangle(e, -2.5, -1.5, -0.5)       # Négatif Moyen
    @staticmethod
    def error_NS(e): return triangle(e, -1.2, -0.5, 0.0)        # Négatif Small
    @staticmethod
    def error_ZE(e): return triangle(e, -0.6, 0.0, 0.6)         # Zéro (centré)
    @staticmethod
    def error_PS(e): return triangle(e, 0.0, 0.5, 1.2)          # Positif Small
    @staticmethod
    def error_PM(e): return triangle(e, 0.5, 1.5, 2.5)          # Positif Moyen
    @staticmethod
    def error_PL(e): return trapeze(e, 1.0, 2.0, 3.0, 3.0)      # Positif Large

    # ──────────────────────────────────────────────────────────────────────────
    # ENTRÉE 2 : DÉRIVÉE DE L'ERREUR ∈ [-2.0, +2.0]
    # ──────────────────────────────────────────────────────────────────────────
    @staticmethod
    def rate_NL(r): return trapeze(r, -2.0, -2.0, -1.2, -0.5)   # Négatif Large
    @staticmethod
    def rate_NS(r): return triangle(r, -1.5, -0.6, 0.0)         # Négatif Small
    @staticmethod
    def rate_ZE(r): return triangle(r, -0.5, 0.0, 0.5)          # Zéro (stable)
    @staticmethod
    def rate_PS(r): return triangle(r, 0.0, 0.6, 1.5)           # Positif Small
    @staticmethod
    def rate_PL(r): return trapeze(r, 0.5, 1.2, 2.0, 2.0)       # Positif Large

    # ──────────────────────────────────────────────────────────────────────────
    # ENTRÉE 3 : COURBURE ∈ [0.0, 1.0]
    # ──────────────────────────────────────────────────────────────────────────
    @staticmethod
    def curv_STRAIGHT(c): return trapeze(c, 0.0, 0.0, 0.15, 0.35)  # Ligne droite
    @staticmethod
    def curv_MILD(c):     return triangle(c, 0.2, 0.45, 0.70)       # Virage doux
    @staticmethod
    def curv_SHARP(c):    return trapeze(c, 0.55, 0.75, 1.0, 1.0)   # Virage serré

    # ──────────────────────────────────────────────────────────────────────────
    # SORTIE 1 : BRAQUAGE ∈ [-1.0, +1.0] (singletons)
    # ──────────────────────────────────────────────────────────────────────────
    STEER_CENTERS = {
        'HL': -1.00,   # Hard Left (virage fort à gauche)
        'ML': -0.65,   # Medium Left
        'SL': -0.30,   # Soft Left
        'ZE': 0.00,    # Zero (tout droit)
        'SR': 0.30,    # Soft Right
        'MR': 0.65,    # Medium Right
        'HR': 1.00,    # Hard Right (virage fort à droite)
    }

    # ──────────────────────────────────────────────────────────────────────────
    # SORTIE 2 : VITESSE EN VIRAGE ∈ [0.0, 1.0] (singletons)
    # ──────────────────────────────────────────────────────────────────────────
    CORNER_SPEED_CENTERS = {
        'STOP': 0.05,          # Arrêt quasi total
        'SLOW': 0.25,          # Lent
        'MEDIUM_SLOW': 0.45,   # Moyennement lent
        'MEDIUM': 0.65,        # Moyen
        'FAST': 0.85,          # Rapide
    }

    # ──────────────────────────────────────────────────────────────────────────
    # SORTIE 3 : VITESSE EN LIGNE DROITE ∈ [0.0, 1.0] (singletons)
    # ──────────────────────────────────────────────────────────────────────────
    STRAIGHT_SPEED_CENTERS = {
        'SLOW': 0.30,          # Lent
        'MEDIUM': 0.55,        # Moyen
        'FAST': 0.75,          # Rapide
        'VERY_FAST': 0.90,     # Très rapide
        'TURBO': 1.00,         # Pleine vitesse
    }


# ═══════════════════════════════════════════════════════════════════════════════
# RÈGLES FLOUES MAMDANI (21 règles)
# Format: (erreur, taux, courbure, braquage, vitesse_virage, vitesse_droite)
# "ANY" signifie que la variable n'est pas conditionnée (activation=1.0)
# ═══════════════════════════════════════════════════════════════════════════════

FUZZY_RULES = [
    # ──────────────────────────────────────────────────────────────────────────
    # GROUPE 1 — Correction proportionnelle à l'erreur (R1 à R7)
    # Le robot corrige proportionnellement à son écart par rapport à la ligne
    # ──────────────────────────────────────────────────────────────────────────
    ("NL", "ANY", "ANY", "HR", "STOP", "SLOW"),      # R1: erreur forte gauche → tourner fort droite, très lent
    ("NM", "ANY", "ANY", "MR", "SLOW", "MEDIUM"),    # R2: erreur moyenne gauche → tourner moyenne droite
    ("NS", "ANY", "ANY", "SR", "MEDIUM_SLOW", "FAST"),# R3: erreur faible gauche → correction douce
    ("ZE", "ANY", "ANY", "ZE", "FAST", "TURBO"),     # R4: centré parfait → tout droit, vitesse max
    ("PS", "ANY", "ANY", "SL", "MEDIUM_SLOW", "FAST"),# R5: erreur faible droite → correction douce gauche
    ("PM", "ANY", "ANY", "ML", "SLOW", "MEDIUM"),    # R6: erreur moyenne droite → tourner moyenne gauche
    ("PL", "ANY", "ANY", "HL", "STOP", "SLOW"),      # R7: erreur forte droite → tourner fort gauche, très lent

    # ──────────────────────────────────────────────────────────────────────────
    # GROUPE 2 — Anticipation par dérivée (R8 à R11)
    # Utilise la tendance pour corriger AVANT que l'erreur ne s'aggrave
    # ──────────────────────────────────────────────────────────────────────────
    ("NS", "PL", "ANY", "ZE", "MEDIUM", "VERY_FAST"),   # R8: erreur gauche mais qui diverge → anticiper
    ("PS", "NL", "ANY", "ZE", "MEDIUM", "VERY_FAST"),   # R9: erreur droite mais qui diverge → anticiper
    ("ZE", "PL", "ANY", "SL", "MEDIUM_SLOW", "FAST"),   # R10: centré mais diverge droite → préventif gauche
    ("ZE", "NL", "ANY", "SR", "MEDIUM_SLOW", "FAST"),   # R11: centré mais diverge gauche → préventif droite

    # ──────────────────────────────────────────────────────────────────────────
    # GROUPE 3 — Virages serrés (courbure SHARP) (R12 à R16)
    # Ralentissement automatique et correction renforcée
    # ──────────────────────────────────────────────────────────────────────────
    ("ZE", "ANY", "SHARP", "ZE", "SLOW", "MEDIUM_SLOW"),   # R12: centré mais virage serré → ralentir
    ("PM", "ANY", "SHARP", "HL", "STOP", "SLOW"),          # R13: erreur droite + virage serré → urgence gauche
    ("NM", "ANY", "SHARP", "HR", "STOP", "SLOW"),          # R14: erreur gauche + virage serré → urgence droite
    ("PS", "ANY", "SHARP", "ML", "SLOW", "SLOW"),          # R15: petite erreur droite + virage → correction
    ("NS", "ANY", "SHARP", "MR", "SLOW", "SLOW"),          # R16: petite erreur gauche + virage → correction

    # ──────────────────────────────────────────────────────────────────────────
    # GROUPE 4 — Lignes droites (courbure STRAIGHT) (R17 à R19)
    # Accélération maximale sur les lignes droites
    # ──────────────────────────────────────────────────────────────────────────
    ("ZE", "ZE", "STRAIGHT", "ZE", "FAST", "TURBO"),       # R17: ligne droite parfaite → TURBO
    ("PS", "ZE", "STRAIGHT", "SL", "VERY_FAST", "VERY_FAST"), # R18: petite erreur droite → correction rapide
    ("NS", "ZE", "STRAIGHT", "SR", "VERY_FAST", "VERY_FAST"), # R19: petite erreur gauche → correction rapide

    # ──────────────────────────────────────────────────────────────────────────
    # GROUPE 5 — Sécurité / état critique (R20 à R21)
    # Détection des situations dangereuses (dérive + grande erreur)
    # ──────────────────────────────────────────────────────────────────────────
    ("NL", "NL", "ANY", "HR", "STOP", "SLOW"),          # R20: erreur forte + dérive négative → urgence
    ("PL", "PL", "ANY", "HL", "STOP", "SLOW"),          # R21: erreur forte + dérive positive → urgence
]


# ═══════════════════════════════════════════════════════════════════════════════
# MOTEUR D'INFÉRENCE FUZZY MAMDANI
# ═══════════════════════════════════════════════════════════════════════════════

class FuzzyEngine:
    """
    Moteur d'inférence floue Mamdani.

    Pipeline complet :
      1. Fuzzification    : calcul des degrés d'appartenance pour chaque entrée
      2. Application règles : α = min(mu_erreur, mu_taux, mu_courbure)
      3. Agrégation        : max des activations pour chaque singleton de sortie
      4. Défuzzification   : COG (Centre of Gravity) sur singletons

    Formule COG : sortie = Σ(αᵢ × centre_i) / Σ(αᵢ)
    """

    def __init__(self):
        self.fs = FuzzySets()
        self.last_active_rules: List[Dict] = []

    def fuzzify_error(self, e: float) -> Dict[str, float]:
        """Fuzzification de l'erreur latérale"""
        return {
            'NL': self.fs.error_NL(e), 'NM': self.fs.error_NM(e),
            'NS': self.fs.error_NS(e), 'ZE': self.fs.error_ZE(e),
            'PS': self.fs.error_PS(e), 'PM': self.fs.error_PM(e),
            'PL': self.fs.error_PL(e),
        }

    def fuzzify_rate(self, r: float) -> Dict[str, float]:
        """Fuzzification de la dérivée de l'erreur"""
        return {
            'NL': self.fs.rate_NL(r), 'NS': self.fs.rate_NS(r),
            'ZE': self.fs.rate_ZE(r), 'PS': self.fs.rate_PS(r),
            'PL': self.fs.rate_PL(r),
        }

    def fuzzify_curvature(self, c: float) -> Dict[str, float]:
        """Fuzzification de la courbure"""
        return {
            'STRAIGHT': self.fs.curv_STRAIGHT(c),
            'MILD': self.fs.curv_MILD(c),
            'SHARP': self.fs.curv_SHARP(c),
        }

    def infer(self, error: float, error_rate: float, curvature: float) -> Dict:
        """
        Inférence floue complète.

        Args:
            error: erreur latérale ∈ [-3, +3]
            error_rate: dérivée de l'erreur ∈ [-2, +2]
            curvature: courbure locale ∈ [0, 1]

        Returns:
            Dictionnaire contenant:
            - steer: commande de braquage ∈ [-1, +1]
            - corner_speed: vitesse en virage ∈ [0, 1]
            - straight_speed: vitesse en ligne droite ∈ [0, 1]
            - active_rules: liste des règles activées avec leur α
        """
        # 1. Fuzzification
        mu_e = self.fuzzify_error(error)
        mu_r = self.fuzzify_rate(error_rate)
        mu_c = self.fuzzify_curvature(curvature)

        # 2. Agrégation pour COG
        steer_num = steer_den = 0.0
        cs_num = cs_den = 0.0
        ss_num = ss_den = 0.0
        active_rules = []

        # 3. Parcours des règles
        for rule in FUZZY_RULES:
            e_set, r_set, c_set, steer_out, cs_out, ss_out = rule

            # Activation = MIN des degrés d'appartenance
            alpha_e = mu_e.get(e_set, 1.0) if e_set != 'ANY' else 1.0
            alpha_r = mu_r.get(r_set, 1.0) if r_set != 'ANY' else 1.0
            alpha_c = mu_c.get(c_set, 1.0) if c_set != 'ANY' else 1.0
            alpha = min(alpha_e, alpha_r, alpha_c)

            if alpha < 0.001:
                continue

            # Vérification des clés (sécurité)
            if steer_out not in self.fs.STEER_CENTERS:
                continue
            if cs_out not in self.fs.CORNER_SPEED_CENTERS:
                continue
            if ss_out not in self.fs.STRAIGHT_SPEED_CENTERS:
                continue

            # Contribution au COG
            sv = self.fs.STEER_CENTERS[steer_out]
            cv = self.fs.CORNER_SPEED_CENTERS[cs_out]
            dv = self.fs.STRAIGHT_SPEED_CENTERS[ss_out]

            steer_num += alpha * sv
            steer_den += alpha
            cs_num += alpha * cv
            cs_den += alpha
            ss_num += alpha * dv
            ss_den += alpha

            active_rules.append({
                'rule': f"R{FUZZY_RULES.index(rule)+1}: {e_set}/{r_set}/{c_set}",
                'consequence': f"steer={steer_out}, cs={cs_out}, ss={ss_out}",
                'activation': round(alpha, 4),
            })

        # 4. Défuzzification par COG
        steer = steer_num / steer_den if steer_den > 0 else 0.0
        cs = cs_num / cs_den if cs_den > 0 else 0.5
        ss = ss_num / ss_den if ss_den > 0 else 0.5

        # Stockage des règles actives pour l'affichage
        self.last_active_rules = sorted(active_rules, key=lambda x: -x['activation'])[:8]

        return {
            'steer': max(-1.0, min(1.0, steer)),
            'corner_speed': max(0.0, min(1.0, cs)),
            'straight_speed': max(0.0, min(1.0, ss)),
            'active_rules': self.last_active_rules,
            'mu_error': mu_e,
            'mu_rate': mu_r,
            'mu_curv': mu_c,
        }


# ═══════════════════════════════════════════════════════════════════════════════
# TRANSFORMER SIMPLIFIÉ (Mémoire et prédiction par tendance)
# ═══════════════════════════════════════════════════════════════════════════════

class SimpleTransformer:
    """
    Transformer simplifié pour la mémoire du robot.

    Architecture légère (sans PyTorch) :
    - Mémoire circulaire des 16 derniers états
    - Prédiction par régression linéaire sur l'historique des erreurs
    - Correction du braquage et de la vitesse basée sur la tendance

    Par rapport à un Transformer complet, cette version est :
    - Plus rapide (calcul en O(n) au lieu de O(n²))
    - Plus simple à déboguer
    - Suffisante pour démontrer l'apport de la mémoire
    """

    def __init__(self, memory_size: int = 16):
        self.memory: List[Dict] = []
        self.max_size = memory_size

    def update(self, state: Dict):
        """Ajoute un état à la mémoire"""
        self.memory.append(state)
        if len(self.memory) > self.max_size:
            self.memory.pop(0)

    def predict(self) -> Dict:
        """
        Prédit la correction à appliquer basée sur l'historique.

        Retourne:
            - steer_correction: correction du braquage ∈ [-0.15, +0.15]
            - speed_correction: correction de la vitesse ∈ [-0.075, +0.075]
            - confidence: confiance dans la prédiction ∈ [0, 1]
        """
        if len(self.memory) < 5:
            return {'steer_correction': 0.0, 'speed_correction': 0.0, 'confidence': 0.0}

        # Extraire les erreurs des 10 derniers états
        errors = [s['error'] for s in self.memory[-10:]]

        # Régression linéaire pour détecter la tendance
        x = np.arange(len(errors))
        coeffs = np.polyfit(x, errors, 1)
        trend = coeffs[0]  # Pente de la tendance

        # Si tendance significative, appliquer correction
        if abs(trend) > 0.02:
            # Correction proportionnelle à la tendance
            correction = np.clip(trend * 2.0, -0.15, 0.15)
            # Confiance basée sur l'intensité de la tendance
            confidence = min(0.8, abs(trend) * 5.0)
        else:
            correction = 0.0
            confidence = 0.0

        return {
            'steer_correction': correction,
            'speed_correction': correction * 0.5,  # Correction vitesse plus faible
            'confidence': confidence,
            'attention_weights': [[0.25, 0.25, 0.25, 0.25]] * 4  # Pour affichage
        }


# ═══════════════════════════════════════════════════════════════════════════════
# CONTRÔLEUR PID CLASSIQUE (Référence pour comparaison)
# ═══════════════════════════════════════════════════════════════════════════════

class PIDController:
    """
    Contrôleur PID standard.

    Formule : u(t) = Kp·e(t) + Ki·∫e(t)dt + Kd·de(t)/dt

    Limitation majeure : vitesse constante (pas d'adaptation à la courbure)
    Cela illustre pourquoi le Fuzzy est supérieur sur circuit varié.
    """

    def __init__(self):
        self.kp = 1.8      # Proportionnel
        self.ki = 0.025    # Intégral
        self.kd = 1.4      # Dérivé
        self.speed = 0.60  # Vitesse fixe (ne change pas)
        self._integral = 0.0
        self._prev_error = 0.0
        self.dt = 1.0 / 60.0

    def update_params(self, kp=None, ki=None, kd=None, speed=None):
        """Met à jour les paramètres PID en temps réel"""
        if kp is not None:
            self.kp = kp
        if ki is not None:
            self.ki = ki
        if kd is not None:
            self.kd = kd
        if speed is not None:
            self.speed = speed

    def compute(self, error: float) -> Dict:
        """
        Calcule la commande PID.

        Returns:
            steer: commande de braquage ∈ [-1, +1]
            speed: vitesse (constante)
            p_term, i_term, d_term: pour affichage
        """
        # Intégrale avec anti-windup
        self._integral += error * self.dt
        self._integral = max(-10.0, min(10.0, self._integral))

        # Dérivée
        derivative = (error - self._prev_error) / self.dt

        # Termes PID
        p_term = self.kp * error
        i_term = self.ki * self._integral
        d_term = self.kd * derivative

        # Sortie
        steer = p_term + i_term + d_term
        steer = max(-1.0, min(1.0, steer))

        self._prev_error = error

        return {
            'steer': steer,
            'speed': self.speed,
            'p_term': round(p_term, 4),
            'i_term': round(i_term, 4),
            'd_term': round(d_term, 4),
        }

    def reset(self):
        """Réinitialise l'état du PID"""
        self._integral = 0.0
        self._prev_error = 0.0


# ═══════════════════════════════════════════════════════════════════════════════
# AUTODROME (Circuit)
# ═══════════════════════════════════════════════════════════════════════════════

class Autodrome:
    """
    Circuit ovale avec variation continue de courbure.

    Caractéristiques :
    - 361 waypoints (coordonnées normalisées [0,1] × [0,1])
    - Courbure calculée par variation de tangente
    - Permet de tester toutes les règles floues (lignes droites, virages doux, serrés)
    """

    def __init__(self):
        self.waypoints = self._build_track()
        self.total_length = len(self.waypoints)

    def _build_track(self) -> List[Tuple[float, float]]:
        """Génère un circuit ovale avec variation de courbure"""
        pts = []
        for i in range(361):
            t = i / 360.0
            angle = t * 2 * math.pi
            # Rayon variable pour créer des virages plus ou moins serrés
            r = 0.35 + 0.08 * math.sin(angle * 3)
            x = 0.5 + r * math.cos(angle)
            y = 0.5 + r * math.sin(angle) * 0.7
            pts.append((x, y))
        return pts

    def get_point(self, t: float) -> Tuple[float, float]:
        """Retourne le point sur la piste pour un paramètre t ∈ [0, 1]"""
        idx = int((t % 1.0) * (self.total_length - 1))
        return self.waypoints[idx]

    def get_tangent(self, t: float) -> Tuple[float, float]:
        """Retourne le vecteur tangent normalisé à la piste"""
        idx = int((t % 1.0) * (self.total_length - 1))
        idx2 = (idx + 1) % self.total_length
        p1 = self.waypoints[idx]
        p2 = self.waypoints[idx2]
        dx, dy = p2[0] - p1[0], p2[1] - p1[1]
        n = math.hypot(dx, dy) + 1e-9
        return dx / n, dy / n

    def get_curvature(self, t: float) -> float:
        """
        Calcule la courbure locale ∈ [0, 1].

        0 = ligne droite, 1 = virage très serré.
        """
        dt = 0.005
        tx1, ty1 = self.get_tangent(t - dt)
        tx2, ty2 = self.get_tangent(t + dt)

        # Produit vectoriel pour l'angle entre les tangentes
        cross = tx1 * ty2 - ty1 * tx2
        angle = abs(math.asin(max(-1.0, min(1.0, cross))))

        # Normalisation (angle max = π/6 ≈ 30° pour 1.0)
        return min(1.0, angle / (math.pi / 6))


# ═══════════════════════════════════════════════════════════════════════════════
# SIMULATION ENGINE
# ═══════════════════════════════════════════════════════════════════════════════

@dataclass
class RobotState:
    """État complet d'un robot"""
    x: float = 0.5          # Position x (normalisée)
    y: float = 0.5          # Position y (normalisée)
    angle: float = 0.0      # Orientation (radians)
    t: float = 0.0          # Paramètre sur la piste [0,1]
    error: float = 0.0      # Erreur latérale courante
    error_rate: float = 0.0 # Dérivée de l'erreur
    speed: float = 0.5      # Vitesse actuelle [0,1]
    steer: float = 0.0      # Braquage actuel [-1,1]
    laps: int = 0           # Nombre de tours complétés
    lost_count: int = 0     # Nombre de sorties de piste
    total_error: float = 0.0# Somme des |error| pour calcul moyenne
    steps: int = 0          # Nombre de pas de simulation
    mode: str = 'fuzzy'     # 'fuzzy' ou 'pid'


# ═══════════════════════════════════════════════════════════════════════════════
# CAPTEURS IR VIRTUELS (7 capteurs)
# ═══════════════════════════════════════════════════════════════════════════════

class IRSensorArray:
    """Simulation de 7 capteurs IR virtuels disposés en ligne transversale"""
    
    NUM_SENSORS = 7
    SENSOR_SPACING = 0.012  # Espacement entre capteurs (en unités de piste)
    DETECTION_THRESHOLD = 0.015  # Distance max de détection
    
    def __init__(self, track):
        self.track = track
        # Position relative des capteurs: [-3, -2, -1, 0, 1, 2, 3] * SENSOR_SPACING
        self.positions = np.array([(i - (self.NUM_SENSORS - 1) / 2) * self.SENSOR_SPACING 
                                    for i in range(self.NUM_SENSORS)])
        self.weights = np.array([i - (self.NUM_SENSORS - 1) / 2 for i in range(self.NUM_SENSORS)])
    
    def sense(self, bot, track_width: float = 0.06) -> Tuple[List[bool], float, float]:
        """
        Mesure l'état des 7 capteurs IR.
        
        Retourne:
            - sensor_states: liste booléenne [7] (True si capteur détecte la ligne)
            - error: erreur latérale calculée à partir des capteurs [-3, +3]
            - raw_error: erreur brute non normalisée
        """
        # Point le plus proche sur la piste
        px, py = self.track.get_point(bot.t)
        tx, ty = self.track.get_tangent(bot.t)
        
        # Vecteurs perpendiculaires (normal à la piste)
        nx, ny = -ty, tx
        
        # Mesure de chaque capteur
        sensor_states = []
        weighted_sum = 0.0
        active_count = 0
        
        for i, pos in enumerate(self.positions):
            # Position du capteur
            sx = px + pos * nx
            sy = py + pos * ny
            
            # Distance du capteur à la piste
            dist = math.sqrt((bot.x - sx)**2 + (bot.y - sy)**2)
            
            # Détection: le capteur voit la ligne s'il est proche
            on_line = dist < self.DETECTION_THRESHOLD
            sensor_states.append(on_line)
            
            if on_line:
                # Contribution pondérée (poids proportionnel à la position)
                weighted_sum += self.weights[i]
                active_count += 1
        
        # Calcul de l'erreur
        if active_count > 0:
            raw_error = weighted_sum / active_count
        else:
            # Si aucun capteur ne détecte, estimé par position robot
            dx, dy = bot.x - px, bot.y - py
            raw_error = (dx * nx + dy * ny) / track_width
        
        # Normalisation de l'erreur [-3, +3]
        error = max(-3.0, min(3.0, raw_error))
        
        return sensor_states, error, raw_error


class SimulationEngine:
    """
    Moteur de simulation à 60 Hz.

    Gère simultanément les deux robots (Fuzzy+Transformer et PID)
    pour permettre la comparaison en temps réel.
    """

    DT = 1.0 / 60.0                      # Pas de temps (60 Hz)
    TRACK_WIDTH = 0.06                   # Demi-largeur de la piste

    def __init__(self):
        self.track = Autodrome()
        self.fuzzy = FuzzyEngine()
        self.transformer = SimpleTransformer()
        self.pid = PIDController()
        
        # Capteurs IR virtuels (7 capteurs)
        self.ir_sensors = IRSensorArray(self.track)

        self.state_fuzzy = RobotState(mode='fuzzy', t=0.0)
        self.state_pid = RobotState(mode='pid', t=0.1)

        self.history_fuzzy = []
        self._init_robots()

    def _init_robots(self):
        """Positionne les robots sur la piste"""
        for bot in [self.state_fuzzy, self.state_pid]:
            bot.x, bot.y = self.track.get_point(bot.t)
            tx, ty = self.track.get_tangent(bot.t)
            bot.angle = math.atan2(ty, tx)

    def _sense(self, bot: RobotState) -> Tuple[float, float]:
        """
        Mesure l'erreur latérale et la courbure via 7 capteurs IR virtuels.

        Retourne: (error, curvature)
        """
        # Trouver le point le plus proche sur la piste
        best_t, best_d = bot.t, float('inf')
        for delta in np.linspace(-0.05, 0.05, 20):
            tt = (bot.t + delta) % 1.0
            px, py = self.track.get_point(tt)
            d = (bot.x - px)**2 + (bot.y - py)**2
            if d < best_d:
                best_d, best_t = d, tt

        bot.t = best_t

        # Mesure via capteurs IR virtuels (7 capteurs)
        sensor_states, error, _ = self.ir_sensors.sense(bot, self.TRACK_WIDTH)

        # Courbure locale
        curvature = self.track.get_curvature(best_t)

        return error, curvature

    def _advance(self, bot: RobotState, steer: float, speed: float):
        """Met à jour la position du robot (modèle cinématique simple)"""
        step = self.DT * speed * 0.8
        bot.angle += steer * speed * 4.0 * self.DT
        bot.x += speed * math.cos(bot.angle) * step
        bot.y += speed * math.sin(bot.angle) * step
        bot.t = (bot.t + step * 0.9) % 1.0

    def step_fuzzy(self) -> Dict:
        """Un pas de simulation pour le robot Fuzzy + Transformer"""
        bot = self.state_fuzzy

        # Capteurs
        error, curvature = self._sense(bot)
        rate = (error - bot.error) / self.DT if bot.steps > 0 else 0.0
        bot.error = error
        bot.error_rate = max(-2.0, min(2.0, rate))

        # Inférence floue
        fuzzy_out = self.fuzzy.infer(error, bot.error_rate, curvature)
        steer_f = fuzzy_out['steer']

        # Vitesse adaptative (mélange entre vitesse_virage et vitesse_droite)
        speed = fuzzy_out['corner_speed'] * curvature + fuzzy_out['straight_speed'] * (1.0 - curvature)
        speed = max(0.1, min(1.0, speed))

        # Correction par Transformer (mémoire)
        self.transformer.update({'error': error, 'rate': bot.error_rate,
                                 'curvature': curvature, 'speed': speed})
        pred = self.transformer.predict()

        # Application des corrections
        steer_final = max(-1.0, min(1.0, steer_f + pred['steer_correction'] * pred['confidence']))
        speed_final = max(0.1, min(1.0, speed + pred['speed_correction'] * pred['confidence']))

        # Mise à jour
        self._advance(bot, steer_final, speed_final)
        bot.steer = steer_final
        bot.speed = speed_final
        bot.steps += 1
        bot.total_error += abs(error)

        if abs(error) > 1.2:
            bot.lost_count += 1
        if bot.t < 0.01 and bot.steps > 100:
            bot.laps += 1

        return {
            **asdict(bot),
            'fuzzy_output': fuzzy_out,
            'transformer': pred,
            'curvature': curvature,
        }

    def step_pid(self, params: Dict = None) -> Dict:
        """Un pas de simulation pour le robot PID"""
        bot = self.state_pid

        if params:
            self.pid.update_params(**params)

        error, curvature = self._sense(bot)
        bot.error = error

        pid_out = self.pid.compute(error)

        self._advance(bot, pid_out['steer'], pid_out['speed'])
        bot.steer = pid_out['steer']
        bot.speed = pid_out['speed']
        bot.steps += 1
        bot.total_error += abs(error)

        if abs(error) > 1.2:
            bot.lost_count += 1
        if bot.t < 0.01 and bot.steps > 100:
            bot.laps += 1

        return {**asdict(bot), 'pid_output': pid_out, 'curvature': curvature}

    def reset(self):
        """Réinitialise les deux robots"""
        self.state_fuzzy = RobotState(mode='fuzzy', t=0.0)
        self.state_pid = RobotState(mode='pid', t=0.1)
        self.history_fuzzy = []
        self.pid.reset()
        self._init_robots()

    def get_track_data(self) -> Dict:
        """Retourne les données de la piste pour l'affichage"""
        n = len(self.track.waypoints)
        points = [{'x': p[0], 'y': p[1]} for p in self.track.waypoints]
        curvatures = [self.track.get_curvature(i / n) for i in range(n)]
        return {'points': points, 'curvatures': curvatures, 'width': self.TRACK_WIDTH}


# ═══════════════════════════════════════════════════════════════════════════════
# API FLASK
# ═══════════════════════════════════════════════════════════════════════════════

sim = SimulationEngine()
sim_lock = threading.Lock()


@app.route('/api/track', methods=['GET'])
def api_track():
    """Retourne la géométrie de la piste"""
    return jsonify(sim.get_track_data())


@app.route('/api/step', methods=['POST'])
def api_step():
    """Un pas de simulation pour les deux robots"""
    data = request.get_json(silent=True) or {}
    with sim_lock:
        fuzzy_state = sim.step_fuzzy()
        pid_state = sim.step_pid(data.get('pid_params'))
    return jsonify({'fuzzy': fuzzy_state, 'pid': pid_state})


@app.route('/api/memberships', methods=['GET'])
def api_memberships():
    """Retourne les courbes des fonctions d'appartenance"""
    e = float(request.args.get('error', 0.0))
    xe = np.linspace(-3, 3, 100)
    fs = sim.fuzzy.fs

    return jsonify({
        'error': {
            'x': xe.tolist(),
            'NL': [fs.error_NL(v) for v in xe],
            'NM': [fs.error_NM(v) for v in xe],
            'NS': [fs.error_NS(v) for v in xe],
            'ZE': [fs.error_ZE(v) for v in xe],
            'PS': [fs.error_PS(v) for v in xe],
            'PM': [fs.error_PM(v) for v in xe],
            'PL': [fs.error_PL(v) for v in xe],
            'current': e,
        }
    })


@app.route('/api/reset', methods=['POST'])
def api_reset():
    """Réinitialise la simulation"""
    with sim_lock:
        sim.reset()
    return jsonify({'status': 'ok'})


@app.route('/api/status', methods=['GET'])
def api_status():
    """Retourne l'état courant des robots"""
    with sim_lock:
        return jsonify({
            'fuzzy': asdict(sim.state_fuzzy),
            'pid': asdict(sim.state_pid),
            'active_rules': sim.fuzzy.last_active_rules,
        })


# ═══════════════════════════════════════════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════════════════════════════════════════

if __name__ == '__main__':
    print("╔════════════════════════════════════════════════════════════════════╗")
    print("║   LINE FOLLOWER — Backend Fuzzy+Transformer vs PID                  ║")
    print("║   http://localhost:5000                                             ║")
    print("║                                                                     ║")
    print("║   Variables floues :                                                ║")
    print("║     Entrées:  erreur [-3,+3] | dérivée [-2,+2] | courbure [0,1]    ║")
    print("║     Sorties:  braquage [-1,+1] | v_virage [0,1] | v_droite [0,1]   ║")
    print("║   Règles: 21 règles Mamdani | Défuzzification: COG sur singletons  ║")
    print("║   Transformer: mémoire 16 états | prédiction par tendance          ║")
    print("╚════════════════════════════════════════════════════════════════════╝")
    app.run(debug=False, host='0.0.0.0', port=5000, threaded=True)