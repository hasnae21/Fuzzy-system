/*
 * ╔══════════════════════════════════════════════════════════════════════════╗
 * ║   LINE FOLLOWER - LECTURE 10 CAPTEURS TCRT5000                          ║
 * ║   Arduino Mega 2560                                                     ║
 * ║                                                                          ║
 * ║   Ordre des capteurs (de gauche à droite):                              ║
 * ║   1:A5, 2:A0, 3:A6, 4:A1, 5:A7, 6:A2, 7:A8, 8:A3, 9:A9, 10:A4         ║
 * ╚══════════════════════════════════════════════════════════════════════════╝
 */

// ═══════════════════════════════════════════════════════════════════════════
//  BROCHES CAPTEURS (NOUVEL ORDRE de GAUCHE à DROITE)
// ═══════════════════════════════════════════════════════════════════════════

// Ordre physique des capteurs (de gauche à droite)
#define IR_1 A5   // Capteur 1  (extrême gauche)
#define IR_2 A0   // Capteur 2
#define IR_3 A6   // Capteur 3
#define IR_4 A1   // Capteur 4
#define IR_5 A7   // Capteur 5  (centre gauche)
#define IR_6 A2   // Capteur 6  (centre droit)
#define IR_7 A8   // Capteur 7
#define IR_8 A3   // Capteur 8
#define IR_9 A9   // Capteur 9
#define IR_10 A4  // Capteur 10 (extrême droite)

// Tableau des broches pour itération facile
const int IR_PINS[10] = { IR_1, IR_2, IR_3, IR_4, IR_5, IR_6, IR_7, IR_8, IR_9, IR_10 };

// ═══════════════════════════════════════════════════════════════════════════
//  PARAMÈTRES
// ═══════════════════════════════════════════════════════════════════════════

// Seuil de détection (ajuster selon votre piste)
#define SEUIL_NOIR  400

// Période d'affichage (ms)
#define DISPLAY_MS  100

// Poids des capteurs (de -4.5 à +4.5 pour 10 capteurs)
const float WEIGHTS[10] = {
  -4.5, -3.5, -2.5, -1.5, -0.5,   // 5 capteurs gauche
   0.5,  1.5,  2.5,  3.5,  4.5    // 5 capteurs droite
};

// Noms des capteurs pour affichage
const char* SENSOR_NAMES[10] = {
  "C1", "C2", "C3", "C4", "C5",
  "C6", "C7", "C8", "C9", "C10"
};

// ═══════════════════════════════════════════════════════════════════════════
//  DÉCLARATION DES MODES D'AFFICHAGE (AVANT LES FONCTIONS QUI LES UTILISENT)
// ═══════════════════════════════════════════════════════════════════════════

enum DisplayMode { MODE_FULL, MODE_RAW, MODE_BIN, MODE_COMPACT };
DisplayMode currentMode = MODE_FULL;

// ═══════════════════════════════════════════════════════════════════════════
//  VARIABLES GLOBALES
// ═══════════════════════════════════════════════════════════════════════════

int rawValues[10];      // Valeurs brutes (0-1023)
int sensorState[10];    // 1 = ligne noire, 0 = blanc
float position = 0.0;   // Position calculée de la ligne (-4.5 à +4.5)
int activeCount = 0;    // Nombre de capteurs actifs

unsigned long lastDisplay = 0;

// ═══════════════════════════════════════════════════════════════════════════
//  PROTOTYPES DES FONCTIONS
// ═══════════════════════════════════════════════════════════════════════════

void readAllSensors();
float computeLinePosition();
void drawPositionBar();
void printHeader();
void displayData();
void displayCompact();
void calibrateSensors();
void handleSerialCommand();

// ═══════════════════════════════════════════════════════════════════════════
//  FONCTION DE LECTURE DES 10 CAPTEURS
// ═══════════════════════════════════════════════════════════════════════════

void readAllSensors() {
  // Lecture des valeurs brutes dans l'ordre
  for (int i = 0; i < 10; i++) {
    rawValues[i] = analogRead(IR_PINS[i]);
  }
  
  // Conversion en binaire selon seuil
  activeCount = 0;
  for (int i = 0; i < 10; i++) {
    sensorState[i] = (rawValues[i] < SEUIL_NOIR) ? 1 : 0;
    if (sensorState[i]) activeCount++;
  }
}

// ═══════════════════════════════════════════════════════════════════════════
//  CALCUL DE LA POSITION DE LA LIGNE
// ═══════════════════════════════════════════════════════════════════════════

float computeLinePosition() {
  if (activeCount == 0) {
    return position;  // Conserver dernière position si ligne perdue
  }
  
  float weightedSum = 0.0;
  for (int i = 0; i < 10; i++) {
    if (sensorState[i]) {
      weightedSum += WEIGHTS[i];
    }
  }
  
  float pos = weightedSum / activeCount;
  
  // Correction pour les cas extrêmes
  if (activeCount == 1 && sensorState[0]) pos = -4.8;
  if (activeCount == 1 && sensorState[9]) pos = 4.8;
  
  return pos;
}

// ═══════════════════════════════════════════════════════════════════════════
//  AFFICHAGE GRAPHIQUE DE LA POSITION
// ═══════════════════════════════════════════════════════════════════════════

void drawPositionBar() {
  int barPos = (int)((position + 5.0) / 9.0 * 20);
  barPos = constrain(barPos, 0, 20);
  
  Serial.print("│");
  for (int i = 0; i < 20; i++) {
    if (i == barPos) Serial.print("●");
    else if (i == 10) Serial.print("│");
    else Serial.print("─");
  }
  Serial.println("│");
}

// ═══════════════════════════════════════════════════════════════════════════
//  AFFICHAGE COMPLET
// ═══════════════════════════════════════════════════════════════════════════

void printHeader() {
  Serial.println();
  Serial.println("╔════════════════════════════════════════════════════════════════════════════════════════════════════════════════╗");
  Serial.println("║                    LINE FOLLOWER - 10 CAPTEURS TCRT5000 (ORDRE CUSTOM)                                       ║");
  Serial.println("╠════════════════════════════════════════════════════════════════════════════════════════════════════════════════╣");
  Serial.println("║                                                                                                                ║");
  Serial.println("║  ORDRE DES CAPTEURS (Gauche → Droite):                                                                         ║");
  Serial.println("║    C1:A5  C2:A0  C3:A6  C4:A1  C5:A7  |  C6:A2  C7:A8  C8:A3  C9:A9  C10:A4                                 ║");
  Serial.println("║                                                                                                                ║");
  Serial.println("║  FORMAT: [RAW] valeurs brutes (0-1023)                                                                         ║");
  Serial.println("║          [BIN] █=noir  ░=blanc                                                                                ║");
  Serial.println("║          [POS] position calculée (-4.5 à +4.5)                                                                ║");
  Serial.println("║                                                                                                                ║");
  Serial.println("╚════════════════════════════════════════════════════════════════════════════════════════════════════════════════╝");
  Serial.println();
}

void displayData() {
  // En-tête
  Serial.print("      ");
  for (int i = 0; i < 5; i++) {
    Serial.print(SENSOR_NAMES[i]);
    Serial.print("   ");
  }
  Serial.print(" │ ");
  for (int i = 5; i < 10; i++) {
    Serial.print(SENSOR_NAMES[i]);
    Serial.print("   ");
  }
  Serial.println();
  
  Serial.print("      ");
  for (int i = 0; i < 5; i++) {
    Serial.print("────");
    if (i < 4) Serial.print(" ");
  }
  Serial.print("─┼─");
  for (int i = 5; i < 10; i++) {
    Serial.print("────");
    if (i < 9) Serial.print(" ");
  }
  Serial.println();
  
  // Broches
  Serial.print("[PIN] ");
  for (int i = 0; i < 5; i++) {
    Serial.print(IR_PINS[i]);
    if (IR_PINS[i] < 10) Serial.print(" ");
    Serial.print("  ");
    if (i < 4) Serial.print(" ");
  }
  Serial.print(" │ ");
  for (int i = 5; i < 10; i++) {
    Serial.print(IR_PINS[i]);
    if (IR_PINS[i] < 10) Serial.print(" ");
    Serial.print("  ");
    if (i < 9) Serial.print(" ");
  }
  Serial.println();
  
  // Poids
  Serial.print("[WGT] ");
  for (int i = 0; i < 5; i++) {
    Serial.print(WEIGHTS[i], 1);
    if (WEIGHTS[i] >= 0) Serial.print(" ");
    if (abs(WEIGHTS[i]) < 10) Serial.print(" ");
    if (i < 4) Serial.print(" ");
  }
  Serial.print(" │ ");
  for (int i = 5; i < 10; i++) {
    Serial.print(WEIGHTS[i], 1);
    if (WEIGHTS[i] >= 0) Serial.print(" ");
    if (abs(WEIGHTS[i]) < 10) Serial.print(" ");
    if (i < 9) Serial.print(" ");
  }
  Serial.println();
  
  // Valeurs brutes
  Serial.print("[RAW] ");
  for (int i = 0; i < 5; i++) {
    Serial.print(rawValues[i]);
    if (rawValues[i] < 10) Serial.print("   ");
    else if (rawValues[i] < 100) Serial.print("  ");
    else if (rawValues[i] < 1000) Serial.print(" ");
    if (i < 4) Serial.print(" ");
  }
  Serial.print(" │ ");
  for (int i = 5; i < 10; i++) {
    Serial.print(rawValues[i]);
    if (rawValues[i] < 10) Serial.print("   ");
    else if (rawValues[i] < 100) Serial.print("  ");
    else if (rawValues[i] < 1000) Serial.print(" ");
    if (i < 9) Serial.print(" ");
  }
  Serial.println();
  
  // Binaire
  Serial.print("[BIN] ");
  for (int i = 0; i < 5; i++) {
    Serial.print(sensorState[i] ? "█" : "░");
    if (i < 4) Serial.print("   ");
  }
  Serial.print(" │ ");
  for (int i = 5; i < 10; i++) {
    Serial.print(sensorState[i] ? "█" : "░");
    if (i < 9) Serial.print("   ");
  }
  Serial.println();
  
  // Informations
  Serial.print("[INF] ");
  Serial.print("Actifs: ");
  Serial.print(activeCount);
  Serial.print("  |  Position: ");
  if (position >= 0) Serial.print("+");
  Serial.print(position, 3);
  
  Serial.print("  →  ");
  if (position < -3.5) Serial.print("<<< TRÈS À GAUCHE");
  else if (position < -2.0) Serial.print("<< À GAUCHE");
  else if (position < -0.8) Serial.print("< Gauche");
  else if (position < -0.2) Serial.print("← Légèrement gauche");
  else if (position < 0.2) Serial.print("✓ CENTRE");
  else if (position < 0.8) Serial.print("→ Légèrement droite");
  else if (position < 2.0) Serial.print(">> Droite");
  else if (position < 3.5) Serial.print(">>> À DROITE");
  else Serial.print(">>> TRÈS À DROITE");
  Serial.println();
  
  // Barre graphique
  drawPositionBar();
  
  // Séparateur
  Serial.println("────────────────────────────────────────────────────────────────────────────────────────────────────────────────");
  Serial.println();
}

// ═══════════════════════════════════════════════════════════════════════════
//  MODE COMPACT
// ═══════════════════════════════════════════════════════════════════════════

void displayCompact() {
  for (int i = 0; i < 10; i++) {
    Serial.print(sensorState[i] ? "1" : "0");
  }
  Serial.print(":");
  Serial.println(position, 2);
}

// ═══════════════════════════════════════════════════════════════════════════
//  CALIBRATION
// ═══════════════════════════════════════════════════════════════════════════

void calibrateSensors() {
  Serial.println();
  Serial.println("╔══════════════════════════════════════════════════════════════════════════╗");
  Serial.println("║                         MODE CALIBRATION                                 ║");
  Serial.println("╠══════════════════════════════════════════════════════════════════════════╣");
  Serial.println("║  1. Placez le robot sur une surface BLANCHE, tapez 'white'              ║");
  Serial.println("║  2. Placez le robot sur la ligne NOIRE, tapez 'black'                   ║");
  Serial.println("╚══════════════════════════════════════════════════════════════════════════╝");
  Serial.println();
  
  int whiteValues[10] = {0};
  int blackValues[10] = {0};
  bool whiteDone = false;
  bool blackDone = false;
  
  while (!whiteDone) {
    if (Serial.available()) {
      String cmd = Serial.readStringUntil('\n');
      cmd.trim();
      cmd.toLowerCase();
      if (cmd == "white") {
        Serial.println("📝 Mesure sur fond BLANC...");
        for (int i = 0; i < 10; i++) {
          whiteValues[i] = analogRead(IR_PINS[i]);
          Serial.print(SENSOR_NAMES[i]);
          Serial.print(": ");
          Serial.println(whiteValues[i]);
        }
        whiteDone = true;
      }
    }
    delay(50);
  }
  
  while (!blackDone) {
    if (Serial.available()) {
      String cmd = Serial.readStringUntil('\n');
      cmd.trim();
      cmd.toLowerCase();
      if (cmd == "black") {
        Serial.println("📝 Mesure sur ligne NOIRE...");
        for (int i = 0; i < 10; i++) {
          blackValues[i] = analogRead(IR_PINS[i]);
          Serial.print(SENSOR_NAMES[i]);
          Serial.print(": ");
          Serial.println(blackValues[i]);
        }
        blackDone = true;
      }
    }
    delay(50);
  }
  
  Serial.println("\n📊 SEUILS RECOMMANDÉS:");
  int totalSeuil = 0;
  for (int i = 0; i < 10; i++) {
    int seuil = (whiteValues[i] + blackValues[i]) / 2;
    totalSeuil += seuil;
    Serial.print(SENSOR_NAMES[i]);
    Serial.print(": ");
    Serial.print(seuil);
    if (i < 9) Serial.print("  |  ");
  }
  Serial.print("\n\n👉 Seuil moyen: ");
  Serial.println(totalSeuil / 10);
  Serial.println("👉 Utilisez 'seuil XXX' pour changer (ex: seuil 400)");
}

// ═══════════════════════════════════════════════════════════════════════════
//  GESTION DES COMMANDES SÉRIE
// ═══════════════════════════════════════════════════════════════════════════

void handleSerialCommand() {
  if (!Serial.available()) return;
  
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  cmd.toLowerCase();
  
  if (cmd == "help") {
    Serial.println("\n📋 COMMANDES:");
    Serial.println("  help      - Aide");
    Serial.println("  calibrate - Calibration");
    Serial.println("  seuil XX  - Changer seuil");
    Serial.println("  raw       - Mode brut");
    Serial.println("  bin       - Mode binaire");
    Serial.println("  pos       - Mode compact");
    Serial.println("  full      - Mode complet");
  }
  else if (cmd == "calibrate") {
    calibrateSensors();
  }
  else if (cmd == "raw") {
    currentMode = MODE_RAW;
    Serial.println("✅ Mode: valeurs brutes");
  }
  else if (cmd == "bin") {
    currentMode = MODE_BIN;
    Serial.println("✅ Mode: binaire");
  }
  else if (cmd == "pos") {
    currentMode = MODE_COMPACT;
    Serial.println("✅ Mode: position seule");
  }
  else if (cmd == "full") {
    currentMode = MODE_FULL;
    Serial.println("✅ Mode: affichage complet");
  }
  else if (cmd.startsWith("seuil")) {
    int newSeuil = cmd.substring(5).toInt();
    if (newSeuil > 0 && newSeuil < 1024) {
      Serial.print("✅ Seuil modifié à ");
      Serial.println(newSeuil);
    }
  }
}

// ═══════════════════════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  
  for (int i = 0; i < 10; i++) {
    pinMode(IR_PINS[i], INPUT);
  }
  
  delay(2000);
  printHeader();
  Serial.println("✅ Système prêt. Tapez 'help' pour les commandes.\n");
}

// ═══════════════════════════════════════════════════════════════════════════
//  LOOP PRINCIPAL
// ═══════════════════════════════════════════════════════════════════════════

void loop() {
  handleSerialCommand();
  readAllSensors();
  position = computeLinePosition();
  
  unsigned long now = millis();
  if (now - lastDisplay >= DISPLAY_MS) {
    lastDisplay = now;
    
    switch (currentMode) {
      case MODE_RAW:
        for (int i = 0; i < 10; i++) {
          Serial.print(rawValues[i]);
          if (i < 9) Serial.print(",");
        }
        Serial.println();
        break;
        
      case MODE_BIN:
        for (int i = 0; i < 10; i++) {
          Serial.print(sensorState[i]);
        }
        Serial.println();
        break;
        
      case MODE_COMPACT:
        displayCompact();
        break;
        
      default:
        displayData();
        break;
    }
  }
}