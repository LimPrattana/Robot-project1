/************* PARAMÈTRES MÉCANIQUES *************/
#define RATIO      9.6
#define CPR_MOTOR  11
#define CPR_TOTAL  (CPR_MOTOR * RATIO)

/************* FILTRE *************/
#define ALPHA 0.85

/************* PARAMÈTRES TEMPORELS *************/
#define SAMPLE_MS      20
#define DIRECTION_MS   2000
#define PWM_SPEED      150
#define NUM_MOTORS     3

/************* STRUCTURE MOTEUR *************/
struct Motor {
  // Broches
  uint8_t pinIN1, pinIN2, pinENA, pinENCA, pinENCB;

  // Encodeur (accès depuis l'ISR)
  volatile long posi = 0;
  long lastPosi = 0;

  // Vitesse
  float rpm = 0;
  float rpmFilt = 0;

  // Sens
  bool forward = true;
};

Motor motors[NUM_MOTORS] = {
  // IN1, IN2, ENA, ENCA, ENCB
  {2,  4,  15, 18, 19},
  {16, 17, 5,  34, 35},
  {25, 26, 27, 32, 33}
};

/************* VARIABLES TEMPS *************/
unsigned long t0 = 0;
unsigned long lastSample = 0;
unsigned long lastDirChange = 0;
float t = 0;
float deltaT = 0;

/**************** INTERRUPTION (générique via pointeur) ****************/
void IRAM_ATTR readEncoderISR(void* arg) {
  Motor* m = (Motor*) arg;
  int b = digitalRead(m->pinENCB);
  if (b > 0) {
    m->posi++;
  } else {
    m->posi--;
  }
}

/************* FONCTION SENS DE ROTATION *************/
void setDirection(Motor &m, bool fwd) {
  digitalWrite(m.pinIN1, fwd ? HIGH : LOW);
  digitalWrite(m.pinIN2, fwd ? LOW  : HIGH);
}

/******************** SETUP ********************/
void setup() {
  Serial.begin(9600);

  // En-tête CSV
  Serial.print("t");
  for (int i = 0; i < NUM_MOTORS; i++) {
    Serial.print(",rpm"); Serial.print(i+1);
    Serial.print(",rpmFilt"); Serial.print(i+1);
    Serial.print(",dir"); Serial.print(i+1);
  }
  Serial.println();

  for (int i = 0; i < NUM_MOTORS; i++) {
    pinMode(motors[i].pinIN1, OUTPUT);
    pinMode(motors[i].pinIN2, OUTPUT);
    pinMode(motors[i].pinENA, OUTPUT);
    pinMode(motors[i].pinENCA, INPUT);
    pinMode(motors[i].pinENCB, INPUT);

    attachInterruptArg(digitalPinToInterrupt(motors[i].pinENCA),
                        readEncoderISR, &motors[i], RISING);

    setDirection(motors[i], motors[i].forward);
    analogWrite(motors[i].pinENA, PWM_SPEED);
    // Si analogWrite ne compile pas sur votre core ESP32 :
    // remplacez par ledcAttach(pinENA, 5000, 8); ledcWrite(pinENA, PWM_SPEED);
  }

  t0 = micros();
  lastSample = millis();
  lastDirChange = millis();
}

/******************** LOOP ********************/
void loop() {
  unsigned long now = millis();

  /************* CHANGEMENT DE SENS (synchronisé, non bloquant) *************/
  if (now - lastDirChange >= DIRECTION_MS) {
    lastDirChange = now;

    for (int i = 0; i < NUM_MOTORS; i++) {
      analogWrite(motors[i].pinENA, 0);
    }
    delayMicroseconds(2000);

    for (int i = 0; i < NUM_MOTORS; i++) {
      motors[i].forward = !motors[i].forward;
      setDirection(motors[i], motors[i].forward);
      analogWrite(motors[i].pinENA, PWM_SPEED);
    }
  }

  /************* ÉCHANTILLONNAGE VITESSE *************/
  if (now - lastSample >= SAMPLE_MS) {
    deltaT = (now - lastSample) / 1000.0;
    lastSample = now;
    t = (micros() - t0) / 1.0e6;

    Serial.print(t, 3);

    for (int i = 0; i < NUM_MOTORS; i++) {
      long pos;
      noInterrupts();
      pos = motors[i].posi;
      interrupts();

      long deltaP = pos - motors[i].lastPosi;
      motors[i].lastPosi = pos;

      motors[i].rpm = (deltaP / deltaT) * (60.0 / CPR_TOTAL);
      motors[i].rpmFilt = ALPHA * motors[i].rpmFilt + (1.0 - ALPHA) * motors[i].rpm;

      Serial.print(",");
      Serial.print(motors[i].rpm, 1);
      Serial.print(",");
      Serial.print(motors[i].rpmFilt, 1);
      Serial.print(",");
      Serial.print(motors[i].forward ? "AVANT" : "ARRIERE");
    }
    Serial.println();
  }
}