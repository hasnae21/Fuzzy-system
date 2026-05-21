/*
╔══════════════════════════════════════════════════════════════════════════════════╗
║   LINE FOLLOWER ROBOT — FUZZY LOGIC CONTROLLER (VERSION LENTE STABLE)           ║
║   Arduino Mega 2560 + L298N + 10× TCRT5000                                       ║
║                                                                                    ║
║   MODIFICATIONS :                                                                 ║
║     • Vitesses considérablement réduites pour la stabilité                       ║
║     • Vérification des règles floues pour ligne droite                           ║
║     • Mode debug pour analyser les décisions Fuzzy                               ║
╚══════════════════════════════════════════════════════════════════════════════════╝
*/

// ═══════════════════════════════════════════════════════════════════════════════
//  BROCHES — L298N
// ═══════════════════════════════════════════════════════════════════════════════

#define PIN_ENA    10   // PWM moteur gauche
#define PIN_IN1     9
#define PIN_IN2     8
#define PIN_ENB     5   // PWM moteur droit
#define PIN_IN3     7
#define PIN_IN4     6

// ═══════════════════════════════════════════════════════════════════════════════
//  BROCHES — CAPTEURS TCRT5000 (NOUVEL ORDRE)
// ═══════════════════════════════════════════════════════════════════════════════

#define IR_1  A5   // Capteur 1  (extrême gauche)
#define IR_2  A0   // Capteur 2
#define IR_3  A6   // Capteur 3
#define IR_4  A1   // Capteur 4
#define IR_5  A7   // Capteur 5  (centre gauche)
#define IR_6  A2   // Capteur 6  (centre droit)
#define IR_7  A8   // Capteur 7
#define IR_8  A3   // Capteur 8
#define IR_9  A9   // Capteur 9
#define IR_10 A4   // Capteur 10 (extrême droite)

const int SENSOR_PIN[10] = { IR_1, IR_2, IR_3, IR_4, IR_5, IR_6, IR_7, IR_8, IR_9, IR_10 };
const int WEIGHT[10] = { -9, -7, -5, -3, -1, +1, +3, +5, +7, +9 };

int sensorVal[10];
bool onLine[10];
int THRESHOLD = 500;

// ═══════════════════════════════════════════════════════════════════════════════
//  CALIBRATION CAPTEURS
// ═══════════════════════════════════════════════════════════════════════════════

int calMin[10], calMax[10];

// ═══════════════════════════════════════════════════════════════════════════════
//  ⚠️ PARAMÈTRES DE VITESSE - FORTEMENT RÉDUITS POUR STABILITÉ ⚠️
// ═══════════════════════════════════════════════════════════════════════════════

#define VITESSE_STOP       0
#define VITESSE_TRES_LENT 30    // Nouveau : très lent pour calibration
#define VITESSE_LENT      50    // Réduit: 80 → 50
#define VITESSE_NORMAL    80    // Réduit: 150 → 80
#define VITESSE_RAPIDE    120   // Réduit: 220 → 120
#define VITESSE_MAX       150   // Réduit: 255 → 150

// ═══════════════════════════════════════════════════════════════════════════════
//  MODE DEBUG (pour analyser les décisions Fuzzy)
// ═══════════════════════════════════════════════════════════════════════════════

#define DEBUG_FUZZY false   // Mettre à true pour voir les règles actives

// ═══════════════════════════════════════════════════════════════════════════════
//  FONCTIONS D'APPARTENANCE
// ═══════════════════════════════════════════════════════════════════════════════

float trapeze(float x, float a, float b, float c, float d) {
  if (x <= a || x >= d) return 0.0;
  if (x >= b && x <= c) return 1.0;
  if (x < b) return (b - a > 0) ? (x - a) / (b - a) : 1.0;
  return (d - c > 0) ? (d - x) / (d - c) : 1.0;
}

float triangle(float x, float a, float b, float c) {
  if (x <= a || x >= c) return 0.0;
  if (x <= b) return (b - a > 0) ? (x - a) / (b - a) : 1.0;
  return (c - b > 0) ? (c - x) / (c - b) : 1.0;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  UNIVERS DU DISCOURS — POSITION (erreur latérale) ∈ [-9, +9]
// ═══════════════════════════════════════════════════════════════════════════════

float pos_TRES_GAUCHE(float p) { return trapeze(p, -9.0, -9.0, -6.0, -3.5); }
float pos_GAUCHE(float p)      { return triangle(p, -6.0, -3.0, -0.5); }
float pos_CENTRE(float p)      { return triangle(p, -2.0,  0.0,  2.0); }
float pos_DROITE(float p)      { return triangle(p,  0.5,  3.0,  6.0); }
float pos_TRES_DROITE(float p) { return trapeze(p,  3.5,  6.0,  9.0,  9.0); }

// ═══════════════════════════════════════════════════════════════════════════════
//  UNIVERS DU DISCOURS — COURBURE ∈ [0, 1]
// ═══════════════════════════════════════════════════════════════════════════════

float curv_DROITE(float c)  { return trapeze(c, 0.0, 0.0, 0.25, 0.5); }
float curv_VIRAGE(float c)  { return trapeze(c, 0.3, 0.6, 1.0, 1.0); }

// ═══════════════════════════════════════════════════════════════════════════════
//  ÉTAT ROBOT
// ═══════════════════════════════════════════════════════════════════════════════

float position   = 0.0;
float courbure   = 0.0;
int   sensorsON  = 0;
bool  horsLigne  = false;
int   vitMotG    = 0;
int   vitMotD    = 0;
int   lastDir    = 0;

// ═══════════════════════════════════════════════════════════════════════════════
//  LECTURE CAPTEURS
// ═══════════════════════════════════════════════════════════════════════════════

void lireCapteurs() {
  sensorsON = 0;
  float sommeWeights = 0.0;
  float sommeMu      = 0.0;

  for (int i = 0; i < 10; i++) {
    sensorVal[i] = analogRead(SENSOR_PIN[i]);
    int val = map(sensorVal[i], calMin[i], calMax[i], 0, 1023);
    val = constrain(val, 0, 1023);
    onLine[i] = (val > THRESHOLD);
    if (onLine[i]) {
      sensorsON++;
      float mu = (float)(val - THRESHOLD) / (1023.0 - THRESHOLD);
      sommeWeights += WEIGHT[i] * mu;
      sommeMu      += mu;
    }
  }

  horsLigne = (sensorsON == 0);

  if (!horsLigne) {
    position = (sommeMu > 0.0) ? (sommeWeights / sommeMu) : 0.0;
    position = constrain(position, -9.0, 9.0);
    if      (position < -1.0) lastDir = -1;
    else if (position >  1.0) lastDir = +1;
    else                      lastDir  = 0;
  }
}

void calculerCourbure() {
  int ext = 0;
  if (onLine[0]) ext++;
  if (onLine[1]) ext++;
  if (onLine[8]) ext++;
  if (onLine[9]) ext++;

  if (sensorsON > 0) {
    courbure = (float)ext / (float)sensorsON;
    courbure = constrain(courbure * 2.0, 0.0, 1.0);
  } else {
    courbure = 0.5;
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  MOTEUR
// ═══════════════════════════════════════════════════════════════════════════════

void motorSetSpeed(int speedG, int speedD) {
  speedG = constrain(speedG, 0, 255);
  speedD = constrain(speedD, 0, 255);

  if (speedG == 0) {
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, LOW);
  } else {
    digitalWrite(PIN_IN1, HIGH);
    digitalWrite(PIN_IN2, LOW);
  }
  analogWrite(PIN_ENA, speedG);

  if (speedD == 0) {
    digitalWrite(PIN_IN3, LOW);
    digitalWrite(PIN_IN4, LOW);
  } else {
    digitalWrite(PIN_IN3, HIGH);
    digitalWrite(PIN_IN4, LOW);
  }
  analogWrite(PIN_ENB, speedD);
}

void motorStop() {
  analogWrite(PIN_ENA, 0);
  analogWrite(PIN_ENB, 0);
  digitalWrite(PIN_IN1, LOW); digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_IN3, LOW); digitalWrite(PIN_IN4, LOW);
}

// ═══════════════════════════════════════════════════════════════════════════════
//  INFÉRENCE FUZZY MAMDANI - AVEC RÈGLES VÉRIFIÉES
// ═══════════════════════════════════════════════════════════════════════════════
//
//  VÉRIFICATION DES RÈGLES POUR LIGNE DROITE :
//  ===========================================
//  
//  En ligne droite, on veut que le robot AVANCE TOUT DROIT.
//  Cela signifie : vitMotG ≈ vitMotD (même vitesse)
//  
//  Les règles qui s'appliquent en ligne droite (courbure ≈ 0) :
//  
//  R3 : position CENTRE → vitMotG = VITESSE_NORMAL, vitMotD = VITESSE_NORMAL ✅
//  R6 : CENTRE + ligne droite → vitMotG = VITESSE_RAPIDE, vitMotD = VITESSE_RAPIDE ✅
//  R12: GAUCHE + ligne droite → vitMotG = VITESSE_LENT, vitMotD = VITESSE_RAPIDE
//  R13: DROITE + ligne droite → vitMotG = VITESSE_RAPIDE, vitMotD = VITESSE_LENT
//  
//  → Si le robot est centré (position ≈ 0), les règles R3 et R6 dominent
//  → Les vitesses sont égales → le robot avance droit ✅
//
// ═══════════════════════════════════════════════════════════════════════════════

void fuzzyInference() {
  if (horsLigne) {
    // Règle de récupération : chercher la ligne
    if (lastDir < 0) {
      vitMotG = VITESSE_STOP;
      vitMotD = VITESSE_LENT;
    } else if (lastDir > 0) {
      vitMotG = VITESSE_LENT;
      vitMotD = VITESSE_STOP;
    } else {
      vitMotG = VITESSE_STOP;
      vitMotD = VITESSE_STOP;
    }
    return;
  }

  // Fuzzification
  float mu_TG = pos_TRES_GAUCHE(position);
  float mu_G  = pos_GAUCHE(position);
  float mu_C  = pos_CENTRE(position);
  float mu_D  = pos_DROITE(position);
  float mu_TD = pos_TRES_DROITE(position);
  float mu_DR = curv_DROITE(courbure);
  float mu_VI = curv_VIRAGE(courbure);

  // Structure d'une règle: (alpha, vitesse_gauche, vitesse_droite)
  struct Regle { float alpha; int sg; int sd; };
  
  Regle regles[15] = {
    // R1: TRÈS GAUCHE → tourner FORT à droite
    { mu_TG,               VITESSE_STOP,       VITESSE_RAPIDE },
    // R2: GAUCHE → tourner à droite modéré
    { mu_G,                VITESSE_LENT,       VITESSE_NORMAL },
    // R3: CENTRE → tout droit (important pour ligne droite !)
    { mu_C,                VITESSE_NORMAL,     VITESSE_NORMAL },
    // R4: DROITE → tourner à gauche modéré
    { mu_D,                VITESSE_NORMAL,     VITESSE_LENT   },
    // R5: TRÈS DROITE → tourner FORT à gauche
    { mu_TD,               VITESSE_RAPIDE,     VITESSE_STOP   },
    
    // R6: CENTRE + LIGNE DROITE → accélérer tout droit (important !)
    { min(mu_C, mu_DR),    VITESSE_RAPIDE,     VITESSE_RAPIDE },
    // R7: CENTRE + VIRAGE → ralentir tout droit
    { min(mu_C, mu_VI),    VITESSE_NORMAL,     VITESSE_NORMAL },
    
    // R8: GAUCHE + VIRAGE → tourner droite renforcé
    { min(mu_G, mu_VI),    VITESSE_STOP,       VITESSE_RAPIDE },
    // R9: DROITE + VIRAGE → tourner gauche renforcé
    { min(mu_D, mu_VI),    VITESSE_RAPIDE,     VITESSE_STOP   },
    
    // R10: TRÈS GAUCHE + VIRAGE → urgence droite
    { min(mu_TG, mu_VI),   VITESSE_STOP,       VITESSE_MAX    },
    // R11: TRÈS DROITE + VIRAGE → urgence gauche
    { min(mu_TD, mu_VI),   VITESSE_MAX,        VITESSE_STOP   },
    
    // R12: GAUCHE + LIGNE DROITE → correction douce droite
    { min(mu_G, mu_DR),    VITESSE_TRES_LENT,  VITESSE_RAPIDE },
    // R13: DROITE + LIGNE DROITE → correction douce gauche
    { min(mu_D, mu_DR),    VITESSE_RAPIDE,     VITESSE_TRES_LENT },
    
    // R14: TRÈS GAUCHE + LIGNE DROITE → rattrapage
    { min(mu_TG, mu_DR),   VITESSE_STOP,       VITESSE_RAPIDE },
    // R15: TRÈS DROITE + LIGNE DROITE → rattrapage
    { min(mu_TD, mu_DR),   VITESSE_RAPIDE,     VITESSE_STOP   },
  };

  // Défuzzification COG (Centre Of Gravity)
  float numG = 0.0, denG = 0.0;
  float numD = 0.0, denD = 0.0;

  for (int i = 0; i < 15; i++) {
    if (regles[i].alpha < 0.01) continue;  // Seuil plus bas pour plus de réactivité
    
    numG += regles[i].alpha * regles[i].sg;
    denG += regles[i].alpha;
    numD += regles[i].alpha * regles[i].sd;
    denD += regles[i].alpha;
    
    #if DEBUG_FUZZY
    if (regles[i].alpha > 0.1) {
      Serial.print("R"); Serial.print(i+1);
      Serial.print(":α="); Serial.print(regles[i].alpha, 2);
      Serial.print(" G="); Serial.print(regles[i].sg);
      Serial.print(" D="); Serial.print(regles[i].sd);
      Serial.print(" | ");
    }
    #endif
  }

  vitMotG = (denG > 0.01) ? (int)(numG / denG) : VITESSE_NORMAL;
  vitMotD = (denD > 0.01) ? (int)(numD / denD) : VITESSE_NORMAL;
  
  // Application d'une vitesse minimale pour éviter l'arrêt complet
  if (vitMotG < VITESSE_TRES_LENT && vitMotG > 0) vitMotG = VITESSE_TRES_LENT;
  if (vitMotD < VITESSE_TRES_LENT && vitMotD > 0) vitMotD = VITESSE_TRES_LENT;
  
  vitMotG = constrain(vitMotG, 0, 255);
  vitMotD = constrain(vitMotD, 0, 255);
}

// ═══════════════════════════════════════════════════════════════════════════════
//  CALIBRATION CAPTEURS
// ═══════════════════════════════════════════════════════════════════════════════

void calibrerCapteurs() {
  Serial.println(F("\n=== CALIBRATION CAPTEURS (3 secondes) ==="));
  Serial.println(F("Déplacer le robot lentement au-dessus de la ligne..."));

  for (int i = 0; i < 10; i++) {
    calMin[i] = 1023;
    calMax[i] = 0;
  }

  unsigned long debut = millis();
  while (millis() - debut < 3000) {
    motorSetSpeed(VITESSE_TRES_LENT, VITESSE_TRES_LENT);  // Lent pour calibration
    for (int i = 0; i < 10; i++) {
      int v = analogRead(SENSOR_PIN[i]);
      if (v < calMin[i]) calMin[i] = v;
      if (v > calMax[i]) calMax[i] = v;
    }
    delay(10);
  }
  motorStop();

  Serial.println(F("Calibration terminée :"));
  for (int i = 0; i < 10; i++) {
    Serial.print(F("  C")); Serial.print(i+1);
    Serial.print(F(" (A")); Serial.print(SENSOR_PIN[i]);
    Serial.print(F(") min=")); Serial.print(calMin[i]);
    Serial.print(F(" max=")); Serial.println(calMax[i]);
  }
  
  // Calculer seuil automatique
  int totalSeuil = 0;
  for (int i = 0; i < 10; i++) {
    totalSeuil += (calMin[i] + calMax[i]) / 2;
  }
  THRESHOLD = totalSeuil / 10;
  Serial.print(F("Seuil automatique: "));
  Serial.println(THRESHOLD);
}

// ═══════════════════════════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);

  pinMode(PIN_ENA, OUTPUT);
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_ENB, OUTPUT);
  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);
  motorStop();

  Serial.println(F("\n╔══════════════════════════════════════════════════════════╗"));
  Serial.println(F("║  LINE FOLLOWER — FUZZY LOGIC (VERSION LENTE STABLE)     ║"));
  Serial.println(F("║  Arduino Mega + L298N + 10×TCRT5000                      ║"));
  Serial.println(F("╚══════════════════════════════════════════════════════════╝"));
  
  calibrerCapteurs();

  Serial.println(F("\n🚀 Démarrage du suivi de ligne..."));
  Serial.println(F("📊 Position | Courbure | ON | G | D | Capteurs"));
  Serial.println(F("─────────────────────────────────────────────────────────"));
}

// ═══════════════════════════════════════════════════════════════════════════════
//  LOOP PRINCIPAL
// ═══════════════════════════════════════════════════════════════════════════════

unsigned long lastDebug = 0;

void loop() {
  lireCapteurs();
  calculerCourbure();
  fuzzyInference();
  motorSetSpeed(vitMotG, vitMotD);

  if (millis() - lastDebug >= 100) {
    lastDebug = millis();
    
    // Format compact pour debug
    Serial.print(position, 1);
    Serial.print("  |  ");
    Serial.print(courbure, 2);
    Serial.print("  |  ");
    Serial.print(sensorsON);
    Serial.print("  |  ");
    Serial.print(vitMotG);
    Serial.print("  |  ");
    Serial.print(vitMotD);
    Serial.print("  |  ");
    
    // Affichage capteurs
    for (int i = 0; i < 10; i++) {
      Serial.print(onLine[i] ? "#" : ".");
    }
    Serial.println();
    
    #if DEBUG_FUZZY
    Serial.println();
    #endif
  }
}