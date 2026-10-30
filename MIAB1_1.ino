#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// LCD à l'adresse 0x20 (Tinkercad)
LiquidCrystal_I2C lcd(0x20, 16, 2);

// Broches
const int POT_CONSIGNE = A0;
const int POT_CAP      = A1;
const int EN1 = 5;   // PWM
const int IN1 = 4;
const int IN2 = 3;

// Zone morte pour éviter les oscillations
const int TOLERANCE = 5;

void setup() {
  pinMode(EN1, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  lcd.init();
  lcd.backlight();
  lcd.print("Pilotage auto");
  delay(1000);
  lcd.clear();
}

void loop() {
  // --- 1. Lecture de la consigne (0-359°) ---
  int valConsigne = analogRead(POT_CONSIGNE);
  int capConsigne = map(valConsigne, 0, 1023, 0, 359);

  // --- 2. Lecture du cap mesuré simulé (0-359°) ---
  int valCap = analogRead(POT_CAP);
  int capMesure = map(valCap, 0, 1023, 0, 359);

  // --- 3. Calcul de l'erreur angulaire ---
  int erreur = capConsigne - capMesure;
  if (erreur > 180)  erreur -= 360;
  if (erreur < -180) erreur += 360;

  // --- 4. Commande du moteur ---
  int vitesse = constrain(map(abs(erreur), 0, 180, 60, 255), 0, 255);

  if (abs(erreur) <= TOLERANCE) {
    // Cap atteint : arrêt
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(EN1, 0);
  } else if (erreur > 0) {
    // Tourner dans un sens
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(EN1, vitesse);
  } else {
    // Tourner dans l'autre sens
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(EN1, vitesse);
  }

  // --- 5. Affichage LCD ---
  lcd.setCursor(0, 0);
  lcd.print("Cons:");
  lcd.print(capConsigne);
  lcd.print((char)223); // symbole °
  lcd.print("   ");

  lcd.setCursor(0, 1);
  lcd.print("Cap :");
  lcd.print(capMesure);
  lcd.print((char)223);
  lcd.print("   ");

  delay(100);
}