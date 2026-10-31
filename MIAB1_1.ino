#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// LCD at address 0x20 (Tinkercad)
LiquidCrystal_I2C lcd(0x20, 16, 2);

// Pins
const int EN1 = 5;   // PWM enable
const int IN1 = 4;
const int IN2 = 3;

// Dead band to avoid oscillation around the setpoint
const int TOLERANCE = 5;

// Simulated compass: the heading changes according to the motor command.
// The physical heading potentiometer is not read; the heading is computed here.
const float DT          = 0.1;   // simulated time step (s)
const float MAX_SPEED   = 30.0;  // rotation speed at PWM 255 (deg/s)
const float INERTIA     = 0.3;   // fraction of the speed gap closed each step (0..1)

// Automatic test series: one setpoint per test
const int NB_TESTS = 10;
const int SETPOINTS[NB_TESTS] = {45, 90, 180, 270, 350, 10, 135, 300, 200, 0};
const float MAX_DURATION    = 20.0; // maximum duration of a test (s)
const float STABLE_DURATION = 2.0;  // time spent within tolerance to validate a test (s)
const int LOG_STEP          = 5;    // log one line every 5 steps (0.5 s)

float heading = 0;       // simulated heading (deg)
float angSpeed = 0;      // simulated rotation speed (deg/s)
int test = 0;
float t = 0;             // elapsed time within the test (s)
float tStable = 0;
float tResponse = -1;    // first time the error enters the dead band (s)
float overshoot = 0;     // largest error measured on the wrong side of the setpoint (deg)
int startSign = 0;
int startHeading = 0;
long step = 0;

// Shortest-path angular error, wrapped to the range -180..+180 deg
int angularError(int setpoint, float currentHeading) {
  int error = setpoint - (int)round(currentHeading);
  if (error > 180)  error -= 360;
  if (error < -180) error += 360;
  return error;
}

void driveMotor(int direction, int speed) {
  if (direction == 0) {
    // Heading reached: stop
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(EN1, 0);
  } else if (direction > 0) {
    // Turn one way
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(EN1, speed);
  } else {
    // Turn the other way
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(EN1, speed);
  }
}

// Reset the per-test counters before starting a new test
void startTest() {
  t = 0;
  tStable = 0;
  tResponse = -1;
  overshoot = 0;
  step = 0;
  startHeading = (int)round(heading) % 360;
  int error = angularError(SETPOINTS[test], heading);
  startSign = (error > 0) - (error < 0);
}

void setup() {
  pinMode(EN1, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  Serial.begin(9600);

  lcd.init();
  lcd.backlight();
  lcd.print("Auto pilot");
  delay(1000);
  lcd.clear();

  // CSV headers: D = sample line, R = per-test summary line
  Serial.println("# D,test,t_s,setpoint,heading,error,pwm");
  Serial.println("# R,test,setpoint,start_heading,final_heading,final_error,t_response_s,overshoot,duration_s");
  startTest();
}

void loop() {
  if (test >= NB_TESTS) {
    driveMotor(0, 0);
    return;
  }
  int setpoint = SETPOINTS[test];

  // --- 1. Angular error ---
  int error = angularError(setpoint, heading);

  // --- 2. Motor command: PWM grows with the error, zero inside the dead band ---
  int direction = 0;
  int speed = 0;
  if (abs(error) > TOLERANCE) {
    direction = error > 0 ? 1 : -1;
    speed = constrain(map(abs(error), 0, 180, 60, 255), 0, 255);
  }
  driveMotor(direction, speed);

  // --- 3. Simulated compass: the motor turns the heading (first-order inertia) ---
  float targetSpeed = direction * MAX_SPEED * speed / 255.0;
  angSpeed += (targetSpeed - angSpeed) * INERTIA;
  heading += angSpeed * DT;
  if (heading >= 360) heading -= 360;
  if (heading < 0)    heading += 360;
  t += DT;
  step++;

  // --- 4. Test metrics (response time and overshoot) ---
  error = angularError(setpoint, heading);
  if (error * startSign < 0) overshoot = max(overshoot, (float)abs(error));
  if (abs(error) <= TOLERANCE) {
    if (tResponse < 0) tResponse = t;
    tStable += DT;
  } else {
    tStable = 0;
  }

  // --- 5. Serial logging ---
  if (step % LOG_STEP == 0) {
    Serial.print("D,");
    Serial.print(test + 1);    Serial.print(",");
    Serial.print(t, 1);        Serial.print(",");
    Serial.print(setpoint);    Serial.print(",");
    Serial.print(heading, 1);  Serial.print(",");
    Serial.print(error);       Serial.print(",");
    Serial.println(direction * speed);
  }

  // The I2C screen slows Tinkercad down a lot: refresh only every 2 simulated seconds
  if (step % (4 * LOG_STEP) == 0) {
    lcd.setCursor(0, 0);
    lcd.print("T");
    lcd.print(test + 1);
    lcd.print(" Set:");
    lcd.print(setpoint);
    lcd.print((char)223); // degree symbol
    lcd.print("   ");

    lcd.setCursor(0, 1);
    lcd.print("Hdg :");
    lcd.print((int)round(heading) % 360);
    lcd.print((char)223);
    lcd.print("      ");
  }

  // --- 6. End of test: stable within tolerance, or maximum duration reached ---
  if (tStable >= STABLE_DURATION || t >= MAX_DURATION) {
    Serial.print("R,");
    Serial.print(test + 1);            Serial.print(",");
    Serial.print(setpoint);            Serial.print(",");
    Serial.print(startHeading);        Serial.print(",");
    Serial.print(heading, 1);          Serial.print(",");
    Serial.print(error);               Serial.print(",");
    Serial.print(tResponse, 1);        Serial.print(",");
    Serial.print(overshoot, 1);        Serial.print(",");
    Serial.println(t, 1);

    test++;
    if (test >= NB_TESTS) {
      Serial.println("# END");
      lcd.clear();
      lcd.print("Tests finished");
    } else {
      startTest();
    }
  }
}
