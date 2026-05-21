// LINE FOLLOWER PID - Version Simple et Robuste
// Branchements: Moteurs: (5,7,8) et (6,9,10) | Capteurs: A0-A4

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);

// === MOTEURS ===
#define ENA 5
#define IN1 7
#define IN2 8
#define ENB 6
#define IN3 9
#define IN4 10

// === CAPTEURS ===
const int pins[5] = {A0, A1, A2, A3, A4};
const float weights[5] = {-2, -1, 0, 1, 2};

// === PID ===
float Kp = 18.0, Ki = 0.01, Kd = 12.0;
int baseSpeed = 180;
float error = 0, lastError = 0, integral = 0;

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < 5; i++) pinMode(pins[i], INPUT);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  
  lcd.init(); lcd.backlight();
  lcd.setCursor(0,0); lcd.print("Line Follower");
  lcd.setCursor(0,1); lcd.print("PID Ready");
  delay(2000);
}

void loop() {
  // Lecture capteurs
  int sensors[5] = {0,0,0,0,0};
  int active = 0;
  for (int i = 0; i < 5; i++) {
    int val = analogRead(pins[i]);
    sensors[i] = (val < 500) ? 1 : 0;  // Seuil 500 (ajuster)
    active += sensors[i];
  }
  
  // Calcul position/erreur
  if (active > 0) {
    float sum = 0;
    for (int i = 0; i < 5; i++) sum += weights[i] * sensors[i];
    error = sum / active;
    error = constrain(error, -2, 2);
  }
  
  // PID
  integral += error * 0.02;
  integral = constrain(integral, -100, 100);
  float derivative = (error - lastError) / 0.02;
  float correction = Kp * error + Ki * integral + Kd * derivative;
  
  // Vitesses
  int left = constrain(baseSpeed + correction, 60, 255);
  int right = constrain(baseSpeed - correction, 60, 255);
  
  // Commander moteurs
  motorControl(IN1, IN2, ENA, left);
  motorControl(IN3, IN4, ENB, right);
  
  lastError = error;
  
  // Affichage
  lcd.setCursor(0,0);
  lcd.print("E:");
  lcd.print(error, 1);
  lcd.print(" L:");
  lcd.print(left);
  lcd.setCursor(0,1);
  lcd.print("R:");
  lcd.print(right);
  
  Serial.print(error);
  Serial.print(",");
  Serial.print(left);
  Serial.print(",");
  Serial.println(right);
  
  delay(20);
}

void motorControl(int inA, int inB, int pwm, int speed) {
  if (speed >= 0) {
    digitalWrite(inA, HIGH);
    digitalWrite(inB, LOW);
  } else {
    digitalWrite(inA, LOW);
    digitalWrite(inB, HIGH);
    speed = -speed;
  }
  analogWrite(pwm, constrain(speed, 0, 255));
}