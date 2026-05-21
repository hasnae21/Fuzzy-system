/*
╔══════════════════════════════════════════════════════════════════════════════════╗
║   LINE FOLLOWER ROBOT — FUZZY LOGIC v5.0                                        ║
║   Arduino Mega 2560 + L298N (module rouge complet) + 10× TCRT5000              ║
║   Alimentation : 4× 18650 Li-ion 3.7V = 14.8V nominal / 16.8V pleine charge   ║
║                                                                                  ║
║   ⚠️  CÂBLAGE OBLIGATOIRE POUR 4 BATTERIES :                                   ║
║   ┌────────────────────────────────────────────────────────────────┐            ║
║   │  1. ENLEVER le jumper +5V du module L298N                      │            ║
║   │  2. Relier : Arduino 5V  →  borne +5V  du L298N               │            ║
║   │  3. Relier : 4 batteries →  borne +12V du L298N               │            ║
║   │  4. Relier : GND Arduino →  borne GND  du L298N               │            ║
║   │  Condensateurs module 35V max → 16.8V pleine charge ✅ SAFE   │            ║
║   └────────────────────────────────────────────────────────────────┘            ║
║                                                                                  ║
║   CALCUL TENSION :                                                               ║
║   4× 18650 = 14.8V nominal / 16.8V plein                                       ║
║   L298N chute ≈ 2V → moteurs voient ≈ 12.8V (nominal)                         ║
║   → PWM plus BAS qu'avec 3 batteries pour même vitesse physique                ║
║   → Gradient PWM plus fin → défuzzification COG plus précise ✅                ║
║                                                                                  ║
║   PISTE : Forme D — ligne droite + grand demi-cercle                            ║
║   CAPTEURS : 2× module 5 TCRT5000, montés à l'avant sous le châssis           ║
║                                                                                  ║
║   DISPOSITION PHYSIQUE DES CAPTEURS (vue de dessous) :                         ║
║    ←  extrême G          centre          extrême D  →                          ║
║    [S1]  [S2]  [S3]  [S4]  [S5] | [S6]  [S7]  [S8]  [S9]  [S10]             ║
║    A5    A0    A6    A1    A7   |  A2    A8    A3    A9    A4                  ║
║    i=0   i=1   i=2   i=3   i=4 |  i=5   i=6   i=7   i=8   i=9               ║
║    W=-9  W=-7  W=-5  W=-3  W=-1|  W=+1  W=+3  W=+5  W=+7  W=+9              ║
║                                                                                  ║
║   CÂBLAGE L298N → Arduino :                                                     ║
║     ENA=10(PWM), IN1=9, IN2=8   → Moteur GAUCHE                               ║
║     ENB=5(PWM),  IN3=7, IN4=6   → Moteur DROIT                                ║
║   ⚠️  Si moteur tourne à l'envers : inverser IN1↔IN2 ou IN3↔IN4              ║
╚══════════════════════════════════════════════════════════════════════════════════╝
*/

// ════════════════════════════════════════════════════════════════════════════════
//  CONFIGURATION RAPIDE — modifier ici si besoin
// ════════════════════════════════════════════════════════════════════════════════

// ── Vitesses PWM [0..255] ────────────────────────────────────────────────────
//
//  CALCUL POUR 4× 18650 + L298N complet :
//  ┌──────────────────────────────────────────────────────────────────────┐
//  │  4× 18650 Li-ion 3.7V                                                │
//  │    Pleine charge : 4× 4.2V = 16.8V                                  │
//  │    Nominale      : 4× 3.7V = 14.8V  ← référence de calcul          │
//  │    Déchargée     : 4× 3.0V = 12.0V                                  │
//  │                                                                       │
//  │  L298N complet : chute de tension interne ≈ 2V                      │
//  │    Tension réelle aux moteurs = Vbatt − 2V = 12.8V (nominal)       │
//  │                                                                       │
//  │  Vitesse moteur = (PWM / 255) × 12.8V                               │
//  │  ┌──────┬───────────────────────────────────────────────────────┐   │
//  │  │ PWM  │ Tension moteur  │ Effet                               │   │
//  │  ├──────┼─────────────────┼─────────────────────────────────────┤   │
//  │  │  45  │  2.3V           │ zone morte — moteur ne tourne pas ❌│   │
//  │  │  55  │  2.8V           │ démarre tout juste ✅               │   │
//  │  │  90  │  4.5V           │ virage stable ✅                    │   │
//  │  │ 130  │  6.5V           │ ligne droite ✅                     │   │
//  │  │ 170  │  8.5V           │ urgence/correction ✅               │   │
//  │  │ 220  │ 11.0V           │ trop rapide pour piste D ❌         │   │
//  │  └──────┴─────────────────┴─────────────────────────────────────┘   │
//  │                                                                       │
//  │  ✅ AVANTAGE 4 batteries pour fuzzy :                               │
//  │  Écart entre niveaux = 35-40 PWM (gradient fin)                     │
//  │  Le COG peut interpoler finement ENTRE les niveaux                  │
//  │  → comportement vraiment flou et graduel (pas ON/OFF)               │
//  └──────────────────────────────────────────────────────────────────────┘
//
//  RÉGLAGE SUR PISTE :
//  Si robot trop rapide en virage → baisser VIT_NORMAL  de 5 en 5
//  Si robot zigzague sur droite   → baisser VIT_RAPIDE  de 5 en 5
//  Si moteurs calent              → monter  VIT_LENT    de 5 en 5

#define VIT_STOP        0
#define VIT_LENT       55    // 2.8V moteur — démarre doucement
#define VIT_NORMAL     90    // 4.5V moteur — croisière virage modéré
#define VIT_RAPIDE    130    // 6.5V moteur — ligne droite bien centrée
#define VIT_MAX       170    // 8.5V moteur — correction urgence hors-ligne
#define VIT_MIN_PWM    50    // zone morte : en dessous le L298N ne fait rien

// ── Seuil de détection (0..1023) ────────────────────────────────────────────
// Ligne NOIRE sur fond BLANC (carreau blanc)
// Si capteurs inversés (blanc=ligne), changer onLine[i] = (val < THRESHOLD)
#define SEUIL_FIXE    500    // utilisé si calibration échoue

// ── Calibration ──────────────────────────────────────────────────────────────
#define CAL_DUREE_MS  4000   // 4 secondes pour calibrer (glisser sur ligne manuellement)
#define CAL_SPREAD_MIN  80   // si max-min < 80 → calibration ratée → seuil fixe

// ── Debug ─────────────────────────────────────────────────────────────────────
#define DEBUG_ON      true   // false = désactiver tous les Serial.print (plus rapide)
#define DEBUG_RULES   false  // true = affiche règles actives (très verbeux)
#define DEBUG_PERIODE  150   // millisecondes entre deux lignes de debug

// ════════════════════════════════════════════════════════════════════════════════
//  BROCHES
// ════════════════════════════════════════════════════════════════════════════════

// Moteurs
#define PIN_ENA  10
#define PIN_IN1   9
#define PIN_IN2   8
#define PIN_ENB   5
#define PIN_IN3   7
#define PIN_IN4   6

// Capteurs TCRT5000 — dans l'ordre gauche→droite (i=0 à i=9)
// Alternés entre les deux modules physiques comme câblé sur votre robot
const uint8_t SENSOR_PIN[10] = {
  A5,  // i=0 : S1 extrême gauche
  A0,  // i=1 : S2
  A6,  // i=2 : S3
  A1,  // i=3 : S4
  A7,  // i=4 : S5 centre gauche
  A2,  // i=5 : S6 centre droit
  A8,  // i=6 : S7
  A3,  // i=7 : S8
  A9,  // i=8 : S9
  A4   // i=9 : S10 extrême droite
};

// Poids de position pour chaque capteur (négatif=gauche, positif=droit)
const int WEIGHT[10] = { -9, -7, -5, -3, -1, +1, +3, +5, +7, +9 };

// ════════════════════════════════════════════════════════════════════════════════
//  VARIABLES GLOBALES
// ════════════════════════════════════════════════════════════════════════════════

int   sensorVal[10];      // lecture analogique brute
bool  onLine[10];         // vrai si ce capteur détecte la ligne
int   calMin[10];         // min mesuré à la calibration
int   calMax[10];         // max mesuré à la calibration
int   THRESHOLD;          // seuil de détection (calculé ou fixe)

float position  = 0.0;   // erreur latérale pondérée ∈ [-9, +9]
float courbure  = 0.0;   // courbure locale ∈ [0, 1]
int   sensorsON = 0;     // nombre de capteurs sur la ligne
bool  horsLigne = false;  // aucun capteur sur la ligne
int   lastDir   = 0;     // dernière direction connue: -1 gauche, 0 centre, +1 droite
int   horsLigneCount = 0; // compteur pour récupération progressive

int   vitMotG   = 0;
int   vitMotD   = 0;

// ════════════════════════════════════════════════════════════════════════════════
//  FONCTIONS D'APPARTENANCE FLOUES
// ════════════════════════════════════════════════════════════════════════════════

/*
 * TRAPÉZOÏDALE : retourne 1.0 sur le plateau [b,c], monte de a→b, descend de c→d
 *
 *   1.0  ┌──────────┐
 *        /            \
 *   0.0─/──────────────\─
 *       a   b        c   d
 */
inline float trapeze(float x, float a, float b, float c, float d) {
  if (x <= a || x >= d) return 0.0f;
  if (x >= b && x <= c) return 1.0f;
  if (x < b)  return (x - a) / (b - a);
  return (d - x) / (d - c);
}

/*
 * TRIANGULAIRE : sommet à b, retourne 0 hors de [a,c]
 *
 *   1.0      /\
 *           /  \
 *   0.0────/    \────
 *          a  b  c
 */
inline float triangle(float x, float a, float b, float c) {
  if (x <= a || x >= c) return 0.0f;
  if (x <= b) return (x - a) / (b - a);
  return (c - x) / (c - b);
}

inline float fmin2(float a, float b) { return a < b ? a : b; }

// ════════════════════════════════════════════════════════════════════════════════
//  ENSEMBLES FLOUS — POSITION ∈ [-9, +9]
// ════════════════════════════════════════════════════════════════════════════════
/*
 *  Visualisation des 5 ensembles sur [-9, +9] :
 *
 *  µ(p)  TG      G        C        D       TD
 *  1.0 ┌──┐
 *      │  │   /\       /\       /\    ┌──┐
 *      │  │  /  \     /  \     /  \   │  │
 *  0.0 └──┴─/────\───/────\───/────\──┴──┘
 *       -9 -6  -3  -1   0  +1  +3  +6  +9
 *
 *  TG = Très Gauche : robot très décalé à gauche → corriger fort vers droite
 *  G  = Gauche      : léger décalage gauche → corriger modéré
 *  C  = Centre      : bien centré → avancer tout droit
 *  D  = Droite      : léger décalage droite → corriger modéré
 *  TD = Très Droite : robot très décalé à droite → corriger fort vers gauche
 */

float pos_TG(float p) { return trapeze(p, -9.0f, -9.0f, -6.0f, -3.5f); }
float pos_G(float p)  { return triangle(p, -6.5f, -3.0f, -0.5f); }
float pos_C(float p)  { return triangle(p, -2.5f,  0.0f,  2.5f); }
float pos_D(float p)  { return triangle(p,  0.5f,  3.0f,  6.5f); }
float pos_TD(float p) { return trapeze(p,  3.5f,  6.0f,  9.0f,  9.0f); }

// ════════════════════════════════════════════════════════════════════════════════
//  ENSEMBLES FLOUS — COURBURE ∈ [0, 1]
// ════════════════════════════════════════════════════════════════════════════════
/*
 *  DROITE : courbure faible (ligne droite ou grande courbe)
 *  VIRAGE : courbure forte (virage prononcé)
 *
 *  Note : sur piste D, la grande courbe a une courbure MODÉRÉE (≈0.3-0.5)
 *  → Les deux ensembles se chevauchent pour une transition douce
 */

float curv_LIGNE(float c)  { return trapeze(c, 0.0f, 0.0f, 0.20f, 0.45f); }
float curv_VIRAGE(float c) { return trapeze(c, 0.30f, 0.55f, 1.0f, 1.0f); }

// ════════════════════════════════════════════════════════════════════════════════
//  LECTURE ET TRAITEMENT DES CAPTEURS
// ════════════════════════════════════════════════════════════════════════════════

void lireCapteurs() {
  sensorsON = 0;
  float sommeW = 0.0f;
  float sommeM = 0.0f;

  for (int i = 0; i < 10; i++) {
    sensorVal[i] = analogRead(SENSOR_PIN[i]);

    // Normalisation [calMin..calMax] → [0..1023]
    int val = map(sensorVal[i], calMin[i], calMax[i], 0, 1023);
    val = constrain(val, 0, 1023);

    // Détection : ligne noire sur fond blanc = valeur analogique HAUTE sur TCRT5000
    // (IR réfléchi moins sur noir → tension sortie plus haute)
    onLine[i] = (val > THRESHOLD);

    if (onLine[i]) {
      sensorsON++;
      // Appartenance progressive : force du signal au-delà du seuil
      float mu = (float)(val - THRESHOLD) / (float)(1023 - THRESHOLD);
      mu = constrain(mu, 0.0f, 1.0f);
      sommeW += WEIGHT[i] * mu;
      sommeM += mu;
    }
  }

  horsLigne = (sensorsON == 0);

  if (!horsLigne) {
    horsLigneCount = 0;
    // Position = moyenne pondérée par l'appartenance
    position = (sommeM > 0.001f) ? (sommeW / sommeM) : 0.0f;
    position = constrain(position, -9.0f, 9.0f);

    // Mise à jour direction mémorisée
    if      (position < -1.5f) lastDir = -1;
    else if (position >  1.5f) lastDir = +1;
    else                        lastDir  =  0;
  } else {
    horsLigneCount++;
  }
}

void calculerCourbure() {
  if (horsLigne) {
    // En mode hors-ligne, garder la dernière courbure connue
    return;
  }
  /*
   * Courbure estimée par la position absolue :
   * - Si le robot est centré (position ≈ 0) → droite
   * - Si le robot est dévié → virage en cours
   *
   * Cette méthode est plus robuste que de compter les capteurs extérieurs
   * sur cette configuration physique où les 10 capteurs sont proches.
   *
   * Formule : courbure = |position| / 9.0  (normalisée 0→1)
   * Lissage exponentiel pour éviter les sauts brusques
   */
  float nouvelleCourbure = fabsf(position) / 9.0f;
  courbure = 0.7f * courbure + 0.3f * nouvelleCourbure;  // filtre passe-bas
  courbure = constrain(courbure, 0.0f, 1.0f);
}

// ════════════════════════════════════════════════════════════════════════════════
//  COMMANDE MOTEURS
// ════════════════════════════════════════════════════════════════════════════════

void motorSetSpeed(int sG, int sD) {
  sG = constrain(sG, 0, 255);
  sD = constrain(sD, 0, 255);

  // Moteur GAUCHE
  if (sG == 0) {
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, LOW);
    analogWrite(PIN_ENA, 0);
  } else {
    digitalWrite(PIN_IN1, HIGH);
    digitalWrite(PIN_IN2, LOW);
    analogWrite(PIN_ENA, sG);
  }

  // Moteur DROIT
  if (sD == 0) {
    digitalWrite(PIN_IN3, LOW);
    digitalWrite(PIN_IN4, LOW);
    analogWrite(PIN_ENB, 0);
  } else {
    digitalWrite(PIN_IN3, HIGH);
    digitalWrite(PIN_IN4, LOW);
    analogWrite(PIN_ENB, sD);
  }
}

void motorStop() {
  analogWrite(PIN_ENA, 0);  analogWrite(PIN_ENB, 0);
  digitalWrite(PIN_IN1, LOW); digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_IN3, LOW); digitalWrite(PIN_IN4, LOW);
}

// Applique la zone morte : si la valeur demandée est > 0 mais < VIT_MIN_PWM,
// on force à VIT_MIN_PWM pour éviter que le moteur cale et chauffe.
int appliquerZoneMorte(int v) {
  if (v == 0) return 0;
  return max(v, VIT_MIN_PWM);
}

// ════════════════════════════════════════════════════════════════════════════════
//  INFÉRENCE FUZZY MAMDANI
// ════════════════════════════════════════════════════════════════════════════════
/*
 *  BASE DE RÈGLES (15 règles) :
 *  ════════════════════════════════════════════════════════════════════
 *  N°  │ Condition Position │ Condition Courbure │ motG      │ motD
 *  ────┼────────────────────┼────────────────────┼───────────┼──────────
 *  R01 │ TRÈS GAUCHE        │ (toute)            │ STOP      │ RAPIDE
 *  R02 │ GAUCHE             │ (toute)            │ LENT      │ NORMAL
 *  R03 │ CENTRE             │ (toute)            │ NORMAL    │ NORMAL
 *  R04 │ DROITE             │ (toute)            │ NORMAL    │ LENT
 *  R05 │ TRÈS DROITE        │ (toute)            │ RAPIDE    │ STOP
 *  R06 │ CENTRE             │ LIGNE DROITE       │ RAPIDE    │ RAPIDE  ← accélère
 *  R07 │ CENTRE             │ VIRAGE             │ NORMAL    │ NORMAL  ← prudent
 *  R08 │ GAUCHE             │ VIRAGE             │ STOP      │ RAPIDE  ← virage G
 *  R09 │ DROITE             │ VIRAGE             │ RAPIDE    │ STOP    ← virage D
 *  R10 │ TRÈS GAUCHE        │ VIRAGE             │ STOP      │ MAX     ← urgence
 *  R11 │ TRÈS DROITE        │ VIRAGE             │ MAX       │ STOP    ← urgence
 *  R12 │ GAUCHE             │ LIGNE DROITE       │ LENT      │ RAPIDE  ← rattrapage
 *  R13 │ DROITE             │ LIGNE DROITE       │ RAPIDE    │ LENT    ← rattrapage
 *  R14 │ TRÈS GAUCHE        │ LIGNE DROITE       │ STOP      │ RAPIDE  ← fort rattrap.
 *  R15 │ TRÈS DROITE        │ LIGNE DROITE       │ RAPIDE    │ STOP    ← fort rattrap.
 *  ════════════════════════════════════════════════════════════════════
 *
 *  DÉFUZZIFICATION : Centre Of Gravity (COG) sur singletons
 *    vitMotG = Σ(αᵢ × sg_i) / Σ(αᵢ)
 *    vitMotD = Σ(αᵢ × sd_i) / Σ(αᵢ)
 *
 *  VÉRIFICATION LIGNE DROITE (position≈0, courbure≈0) :
 *    R3: α=1.0, sg=NORMAL, sd=NORMAL → vitG=vitD ✅
 *    R6: α=1.0, sg=RAPIDE, sd=RAPIDE → vitG=vitD ✅
 *    → COG donne (NORMAL+RAPIDE)/2 ≈ vitesse identique des deux côtés ✅
 *
 *  VÉRIFICATION VIRAGE GAUCHE (position≈-4, courbure≈0.4) :
 *    R2: mu_G=0.8, sg=LENT, sd=NORMAL  → tend à tourner droite ✅
 *    R8: mu_G×mu_VI=0.5, sg=STOP, sd=RAPIDE → renforce ✅
 *    → motD > motG → tourne vers la gauche pour rattraper ✅
 */

void fuzzyInference() {

  // ── Cas hors-ligne : récupération ────────────────────────────────────────
  if (horsLigne) {
    if (lastDir < 0) {
      // Ligne était à gauche → pivoter vers gauche pour la retrouver
      vitMotG = VIT_STOP;
      vitMotD = VIT_LENT;
    } else if (lastDir > 0) {
      // Ligne était à droite → pivoter vers droite
      vitMotG = VIT_LENT;
      vitMotD = VIT_STOP;
    } else {
      // Ligne perdue en allant tout droit → avancer lentement
      vitMotG = VIT_LENT;
      vitMotD = VIT_LENT;
    }
    return;
  }

  // ── Fuzzification ─────────────────────────────────────────────────────────
  float mu_TG = pos_TG(position);
  float mu_G  = pos_G(position);
  float mu_C  = pos_C(position);
  float mu_D  = pos_D(position);
  float mu_TD = pos_TD(position);

  float mu_LI = curv_LIGNE(courbure);   // ligne droite
  float mu_VI = curv_VIRAGE(courbure);  // virage

  // ── Table des règles ──────────────────────────────────────────────────────
  // Chaque entrée : { activation alpha, vitesse_motG, vitesse_motD }
  // 'alpha' = AND = min(condition_position, condition_courbure)
  // Pour les règles sans condition courbure : alpha = condition_position seule
  
  const int NB_REGLES = 15;
  float alpha[NB_REGLES];
  int   sg[NB_REGLES];   // singleton sortie moteur Gauche
  int   sd[NB_REGLES];   // singleton sortie moteur Droit

  // R01 : TRÈS GAUCHE (ANY) → correction forte vers droite
  alpha[0]  = mu_TG;           sg[0]  = VIT_STOP;    sd[0]  = VIT_RAPIDE;

  // R02 : GAUCHE (ANY) → correction modérée vers droite
  alpha[1]  = mu_G;            sg[1]  = VIT_LENT;    sd[1]  = VIT_NORMAL;

  // R03 : CENTRE (ANY) → tout droit à vitesse normale
  alpha[2]  = mu_C;            sg[2]  = VIT_NORMAL;  sd[2]  = VIT_NORMAL;

  // R04 : DROITE (ANY) → correction modérée vers gauche
  alpha[3]  = mu_D;            sg[3]  = VIT_NORMAL;  sd[3]  = VIT_LENT;

  // R05 : TRÈS DROITE (ANY) → correction forte vers gauche
  alpha[4]  = mu_TD;           sg[4]  = VIT_RAPIDE;  sd[4]  = VIT_STOP;

  // R06 : CENTRE + LIGNE → accélérer (ligne droite bien centrée)
  alpha[5]  = fmin2(mu_C, mu_LI);  sg[5] = VIT_RAPIDE;  sd[5] = VIT_RAPIDE;

  // R07 : CENTRE + VIRAGE → ralentir (précaution en courbe)
  alpha[6]  = fmin2(mu_C, mu_VI);  sg[6] = VIT_NORMAL;  sd[6] = VIT_NORMAL;

  // R08 : GAUCHE + VIRAGE → virage gauche renforcé
  alpha[7]  = fmin2(mu_G, mu_VI);  sg[7] = VIT_STOP;    sd[7] = VIT_RAPIDE;

  // R09 : DROITE + VIRAGE → virage droite renforcé
  alpha[8]  = fmin2(mu_D, mu_VI);  sg[8] = VIT_RAPIDE;  sd[8] = VIT_STOP;

  // R10 : TRÈS GAUCHE + VIRAGE → urgence (correction maximale)
  alpha[9]  = fmin2(mu_TG, mu_VI); sg[9] = VIT_STOP;    sd[9] = VIT_MAX;

  // R11 : TRÈS DROITE + VIRAGE → urgence
  alpha[10] = fmin2(mu_TD, mu_VI); sg[10]= VIT_MAX;     sd[10]= VIT_STOP;

  // R12 : GAUCHE + LIGNE → rattrapage doux (décalé en ligne droite)
  alpha[11] = fmin2(mu_G, mu_LI);  sg[11]= VIT_LENT;    sd[11]= VIT_RAPIDE;

  // R13 : DROITE + LIGNE → rattrapage doux
  alpha[12] = fmin2(mu_D, mu_LI);  sg[12]= VIT_RAPIDE;  sd[12]= VIT_LENT;

  // R14 : TRÈS GAUCHE + LIGNE → rattrapage fort
  alpha[13] = fmin2(mu_TG, mu_LI); sg[13]= VIT_STOP;    sd[13]= VIT_RAPIDE;

  // R15 : TRÈS DROITE + LIGNE → rattrapage fort
  alpha[14] = fmin2(mu_TD, mu_LI); sg[14]= VIT_RAPIDE;  sd[14]= VIT_STOP;

  // ── Défuzzification COG sur singletons ────────────────────────────────────
  float numG = 0.0f, denG = 0.0f;
  float numD = 0.0f, denD = 0.0f;

  for (int i = 0; i < NB_REGLES; i++) {
    if (alpha[i] < 0.005f) continue;  // règle inactive, ignorer

    numG += alpha[i] * sg[i];
    denG += alpha[i];
    numD += alpha[i] * sd[i];
    denD += alpha[i];

    #if DEBUG_RULES
    if (alpha[i] > 0.05f) {
      Serial.print(F("R")); Serial.print(i + 1);
      Serial.print(F(" a=")); Serial.print(alpha[i], 2);
      Serial.print(F(" G=")); Serial.print(sg[i]);
      Serial.print(F(" D=")); Serial.print(sd[i]);
      Serial.print(F(" | "));
    }
    #endif
  }

  // Si aucune règle active (ne devrait pas arriver) → avancer normalement
  vitMotG = (denG > 0.01f) ? (int)(numG / denG) : VIT_NORMAL;
  vitMotD = (denD > 0.01f) ? (int)(numD / denD) : VIT_NORMAL;

  // Application de la zone morte moteur
  vitMotG = appliquerZoneMorte(constrain(vitMotG, 0, 255));
  vitMotD = appliquerZoneMorte(constrain(vitMotD, 0, 255));

  #if DEBUG_RULES
  Serial.println();
  #endif
}

// ════════════════════════════════════════════════════════════════════════════════
//  CALIBRATION STATIQUE
//  Le robot est POSÉ SUR LA LIGNE et tenu fixe pendant CAL_DUREE_MS.
//  On fait aussi une mesure hors-ligne (robot levé) puis on calcule le seuil.
// ════════════════════════════════════════════════════════════════════════════════

void calibrerCapteurs() {
  #if DEBUG_ON
  Serial.println(F("\n╔══════════════════════════════════════════════╗"));
  Serial.println(F("║            CALIBRATION CAPTEURS              ║"));
  Serial.println(F("╚══════════════════════════════════════════════╝"));
  Serial.println(F("Étape 1 : Posez le robot sur la LIGNE NOIRE"));
  Serial.println(F("          Appuyez sur le bouton RESET quand prêt"));
  Serial.println(F("          La calibration commence dans 2 secondes..."));
  #endif

  delay(2000);

  // Init
  for (int i = 0; i < 10; i++) {
    calMin[i] = 1023;
    calMax[i] = 0;
  }

  #if DEBUG_ON
  Serial.println(F("Calibration en cours... (glissez lentement sur la ligne)"));
  #endif

  // Phase 1 : lecture sur la ligne + hors-ligne par balayage manuel
  unsigned long debut = millis();
  int echantillons = 0;
  while (millis() - debut < CAL_DUREE_MS) {
    for (int i = 0; i < 10; i++) {
      int v = analogRead(SENSOR_PIN[i]);
      if (v < calMin[i]) calMin[i] = v;
      if (v > calMax[i]) calMax[i] = v;
    }
    echantillons++;
    delay(5);

    // Bip LED (si vous avez une LED sur pin 13)
    if (echantillons % 50 == 0) {
      digitalWrite(13, !digitalRead(13));
    }
  }

  // Calcul du seuil pour chaque capteur
  int somme = 0;
  int nbValides = 0;

  #if DEBUG_ON
  Serial.println(F("\nRésultats calibration :"));
  Serial.println(F(" i | Pin | Min  | Max  | Spread | Seuil  | État"));
  Serial.println(F("───┼─────┼──────┼──────┼────────┼────────┼──────"));
  #endif

  for (int i = 0; i < 10; i++) {
    int spread = calMax[i] - calMin[i];
    int seuilLocal = (calMin[i] + calMax[i]) / 2;

    #if DEBUG_ON
    Serial.print(F(" ")); Serial.print(i);
    Serial.print(F(" | A")); Serial.print(SENSOR_PIN[i]);
    Serial.print(F(" | ")); Serial.print(calMin[i]);
    Serial.print(F(" | ")); Serial.print(calMax[i]);
    Serial.print(F(" | ")); Serial.print(spread);
    Serial.print(F("    | ")); Serial.print(seuilLocal);
    if (spread >= CAL_SPREAD_MIN) {
      Serial.println(F(" | ✅ OK"));
    } else {
      Serial.println(F(" | ⚠️ faible spread"));
    }
    #endif

    if (spread >= CAL_SPREAD_MIN) {
      somme += seuilLocal;
      nbValides++;
    }
  }

  if (nbValides >= 5) {
    THRESHOLD = somme / nbValides;
    #if DEBUG_ON
    Serial.print(F("\nSeuil calculé : ")); Serial.println(THRESHOLD);
    #endif
  } else {
    THRESHOLD = SEUIL_FIXE;
    #if DEBUG_ON
    Serial.print(F("\n⚠️  Calibration insuffisante (<5 capteurs valides)"));
    Serial.print(F(" → Seuil fixe : ")); Serial.println(THRESHOLD);
    Serial.println(F("Conseil : refaire la calibration en glissant le robot"));
    Serial.println(F("          lentement de gauche à droite sur la ligne."));
    #endif
  }

  digitalWrite(13, LOW);
}

// ════════════════════════════════════════════════════════════════════════════════
//  SETUP
// ════════════════════════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  pinMode(13, OUTPUT);  // LED intégrée pour feedback

  // Moteurs
  pinMode(PIN_ENA, OUTPUT); pinMode(PIN_IN1, OUTPUT); pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_ENB, OUTPUT); pinMode(PIN_IN3, OUTPUT); pinMode(PIN_IN4, OUTPUT);
  motorStop();

  #if DEBUG_ON
  Serial.println(F("\n╔══════════════════════════════════════════════════════════════╗"));
  Serial.println(F("║   LINE FOLLOWER — FUZZY LOGIC v5.0                          ║"));
  Serial.println(F("║   Arduino Mega 2560 + L298N complet + 10×TCRT5000          ║"));
  Serial.println(F("║   4× 18650 Li-ion → 14.8V nominal → moteurs ≈12.8V        ║"));
  Serial.println(F("║   ⚠️  Jumper +5V ENLEVÉ — 5V Arduino → borne +5V L298N    ║"));
  Serial.println(F("╚══════════════════════════════════════════════════════════════╝"));
  Serial.println();
  Serial.println(F("Tension : 4×3.7V=14.8V - 2V(L298N) = 12.8V aux moteurs"));
  Serial.print(F("PWM → LENT:")); Serial.print(VIT_LENT);
  Serial.print(F("(")); Serial.print(VIT_LENT*12.8/255, 1); Serial.print(F("V)"));
  Serial.print(F("  NORMAL:")); Serial.print(VIT_NORMAL);
  Serial.print(F("(")); Serial.print(VIT_NORMAL*12.8/255, 1); Serial.print(F("V)"));
  Serial.print(F("  RAPIDE:")); Serial.print(VIT_RAPIDE);
  Serial.print(F("(")); Serial.print(VIT_RAPIDE*12.8/255, 1); Serial.print(F("V)"));
  Serial.print(F("  MAX:")); Serial.print(VIT_MAX);
  Serial.print(F("(")); Serial.print(VIT_MAX*12.8/255, 1); Serial.println(F("V)"));
  #endif

  calibrerCapteurs();

  // Countdown avant départ
  #if DEBUG_ON
  Serial.println(F("\nDémarrage dans :"));
  #endif
  for (int i = 3; i >= 1; i--) {
    #if DEBUG_ON
    Serial.print(i); Serial.println(F("..."));
    #endif
    digitalWrite(13, HIGH); delay(300);
    digitalWrite(13, LOW);  delay(700);
  }

  #if DEBUG_ON
  Serial.println(F("GO! 🚀"));
  Serial.println(F("pos    | courb | ON | vitG | vitD | [capteurs G4..G0|D0..D4]"));
  Serial.println(F("───────┼───────┼────┼──────┼──────┼──────────────────────────"));
  #endif
}

// ════════════════════════════════════════════════════════════════════════════════
//  LOOP PRINCIPAL
// ════════════════════════════════════════════════════════════════════════════════

unsigned long tDebug = 0;

void loop() {
  // 1. Lecture capteurs et calcul de position
  lireCapteurs();

  // 2. Estimation de la courbure locale
  calculerCourbure();

  // 3. Inférence fuzzy → vitMotG, vitMotD
  fuzzyInference();

  // 4. Commande moteurs
  motorSetSpeed(vitMotG, vitMotD);

  // 5. Debug série (non bloquant)
  #if DEBUG_ON
  if (millis() - tDebug >= DEBUG_PERIODE) {
    tDebug = millis();

    // Ligne de données compacte pour Serial Plotter et monitoring
    if (position >= 0) Serial.print(F(" "));
    Serial.print(position, 1);
    Serial.print(F("  | "));
    Serial.print(courbure, 2);
    Serial.print(F("  | "));
    Serial.print(sensorsON);
    Serial.print(F("  | "));
    Serial.print(vitMotG);
    if (vitMotG < 100) Serial.print(F(" "));
    Serial.print(F("  | "));
    Serial.print(vitMotD);
    if (vitMotD < 100) Serial.print(F(" "));
    Serial.print(F("  | ["));

    for (int i = 0; i < 10; i++) {
      Serial.print(onLine[i] ? F("#") : F("."));
      if (i == 4) Serial.print(F("|"));
    }
    Serial.print(F("] "));

    // Indicateur de situation
    if (horsLigne)           Serial.print(F(" HORS-LIGNE!"));
    else if (position > 5.0) Serial.print(F(" >>> TD"));
    else if (position > 2.0) Serial.print(F(" >>  D"));
    else if (position < -5.0)Serial.print(F(" TG <<<"));
    else if (position < -2.0)Serial.print(F(" G  <<"));
    else                     Serial.print(F(" CENTRE"));

    Serial.println();
  }
  #endif
}

/*
═══════════════════════════════════════════════════════════════════
  GUIDE DE RÉGLAGE TERRAIN — v5.0
  4× 18650 + L298N complet + Arduino Mega + 10× TCRT5000
═══════════════════════════════════════════════════════════════════

  CÂBLAGE 4 BATTERIES (OBLIGATOIRE) :
    ① ENLEVER le jumper +5V du module L298N
    ② Fil : Arduino pin 5V  →  borne +5V  du L298N
    ③ Fil : Batteries +     →  borne +12V du L298N
    ④ Fil : Batteries -     →  borne GND  du L298N
    ⑤ Fil : Arduino GND     →  borne GND  du L298N (masse commune)

  VÉRIFICATION AVANT DE DÉMARRER :
    - LED rouge du L298N allumée ? ✅ alimentation OK
    - Arduino alimenté par USB ou Vin ? ✅ logique OK
    - Jumper +5V bien enlevé ? ✅ régulateur interne bypassé

  TABLEAU DE RÉGLAGE PWM (Vmoteurs ≈ 12.8V) :
  ┌──────────────────────────────┬──────┬────────┬────────┬──────┐
  │ Comportement observé         │ LENT │ NORMAL │ RAPIDE │ MAX  │
  ├──────────────────────────────┼──────┼────────┼────────┼──────┤
  │ Valeurs v5.0 (défaut)        │  55  │   90   │  130   │  170 │
  │ Robot trop rapide en virage  │  55  │   75   │  110   │  150 │
  │ Moteurs calent (zone morte)  │  65  │   95   │  135   │  175 │
  │ Zigzague sur ligne droite    │  55  │   85   │  105   │  155 │
  │ Rate les virages             │  55  │   90   │  130   │  190 │
  └──────────────────────────────┴──────┴────────┴────────┴──────┘

  PROBLÈME : Capteurs ne détectent pas la ligne (carrelage blanc)
  SOLUTION : Pendant calibration, glisser le robot sur le ruban
             noir ET sur le carrelage. Répéter 2-3 fois en 4s.

  PROBLÈME : Logique inversée (détecte fond mais pas ligne)
  SOLUTION : Dans lireCapteurs() changer :
             onLine[i] = (val > THRESHOLD)  →  (val < THRESHOLD)

  PROBLÈME : Un moteur tourne à l'envers
  SOLUTION : Permuter IN1↔IN2 (gauche) ou IN3↔IN4 (droit)

  LECTURE SÉRIE — exemple normal :
    pos=0.0  courb=0.05  ON=2  G=90  D=90  [....##....] CENTRE
    pos=-3.2 courb=0.36  ON=2  G=55  D=130 [..##......] G  <<
    pos=+3.5 courb=0.39  ON=2  G=130 D=55  [......##..] >> D

═══════════════════════════════════════════════════════════════════
*/
