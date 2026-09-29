/******************** BLYNK ********************/
#define BLYNK_TEMPLATE_ID "TMPL6T9iJN52n"
#define BLYNK_TEMPLATE_NAME "RoboticProject"
#define BLYNK_AUTH_TOKEN "Cui2ZUkIER67ssIMmyo0decq_rEn4B7t"

#define BLYNK_PRINT Serial

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>

char ssid[] = "@JumboPlusIoT";
char pass[] = "12345678";

/******************** VACUUM LED ********************/
int vacum = 12;
bool vacum_status = false;

/************* PARAMÈTRES MÉCANIQUES *************/
#define RATIO      9.6
#define CPR_MOTOR  11
#define CPR_TOTAL  (CPR_MOTOR * RATIO)

/************* FILTRE *************/
#define ALPHA 0.85

/************* PARAMÈTRES TEMPORELS *************/
#define SAMPLE_MS      20
#define NUM_MOTORS     3
#define MAX_PWM        255

/************* STRUCTURE MOTEUR *************/
struct Motor {
  uint8_t pinIN1, pinIN2, pinENA, pinENCA, pinENCB;
  volatile long posi = 0;
  long lastPosi = 0;
  float rpm = 0;
  float rpmFilt = 0;
  bool forward = true;
};

Motor motors[NUM_MOTORS] = {
  // IN1, IN2, ENA, ENCA, ENCB
  {4,  2,  15, 18, 19},  // motor[0] ซ้าย
  {16, 17, 5,  22, 23},  // motor[1] ขวา
  {25, 26, 27, 32, 33}   // motor[2] หลัง
};

/*
  ล้อ 1 = ซ้าย 150 องศา
  ล้อ 2 = ขวา 30 องศา
  ล้อ 3 = หลัง 270 องศา
*/
const float wheelAngle[NUM_MOTORS] = {
  2.61799,
  0.52360,
  4.71239
};

/************* ความเร็วเป้าหมาย *************/
float targetVx = 0.0;
float targetVy = 0.0;
float targetOmega = 0.0;
float speedStep = 0.6;

/************* VARIABLES TEMPS *************/
unsigned long t0 = 0;
unsigned long lastSample = 0;
float t = 0;
float deltaT = 0;

/************* SERIAL BUFFER *************/
String inputBuffer = "";

/************* สถานะ Blynk *************/
bool previousBlynkConnected = false;

/**************** ENCODER INTERRUPT ****************/
void IRAM_ATTR readEncoderISR(void *arg) {
  Motor *m = (Motor *)arg;
  int b = digitalRead(m->pinENCB);

  if (b > 0) {
    m->posi++;
  } else {
    m->posi--;
  }
}

/**************** MOTOR DIRECTION ****************/
void setDirection(Motor &m, bool fwd) {
  digitalWrite(m.pinIN1, fwd ? HIGH : LOW);
  digitalWrite(m.pinIN2, fwd ? LOW : HIGH);
}

/**************** หยุดหุ่นยนต์ ****************/
void stopRobot() {
  targetVx = 0;
  targetVy = 0;
  targetOmega = 0;
}

/**************** ควบคุมการเคลื่อนที่ ****************/
void handleMovement(int buttonState, float vx, float vy, float omega) {
  if (buttonState == 1) {
    targetVx = vx;
    targetVy = vy;
    targetOmega = omega;
  } else {
    stopRobot();
  }
}

/**************** คำนวณความเร็วล้อ ****************/
void setRobotVelocity(float vx, float vy, float omega) {
  float wheelSpeed[NUM_MOTORS];
  float maxAbs = 0;

  for (int i = 0; i < NUM_MOTORS; i++) {
    wheelSpeed[i] =
      -sin(wheelAngle[i]) * vx
      + cos(wheelAngle[i]) * vy
      + omega;

    if (fabs(wheelSpeed[i]) > maxAbs) {
      maxAbs = fabs(wheelSpeed[i]);
    }
  }

  // ป้องกันค่าความเร็วเกิน 1.0
  if (maxAbs > 1.0) {
    for (int i = 0; i < NUM_MOTORS; i++) {
      wheelSpeed[i] /= maxAbs;
    }
  }

  for (int i = 0; i < NUM_MOTORS; i++) {
    bool fwd = wheelSpeed[i] >= 0;

    int pwm = constrain(
      (int)(fabs(wheelSpeed[i]) * MAX_PWM),
      0,
      MAX_PWM
    );

    setDirection(motors[i], fwd);
    motors[i].forward = fwd;
    analogWrite(motors[i].pinENA, pwm);
  }
}

/**************** คำสั่งจาก BLYNK ****************/

/*
  V0 = เดินหน้า
  ตั้งปุ่มเป็น PUSH
*/
BLYNK_WRITE(V0) {
  handleMovement(
    param.asInt(),
    0,
    speedStep,
    0
  );
}

/*
  V1 = ถอยหลัง
  ตั้งปุ่มเป็น PUSH
*/
BLYNK_WRITE(V1) {
  handleMovement(
    param.asInt(),
    0,
    -speedStep,
    0
  );
}

/*
  V2 = ไปซ้าย
  ตั้งปุ่มเป็น PUSH
*/
BLYNK_WRITE(V2) {
  handleMovement(
    param.asInt(),
    -speedStep,
    0,
    0
  );
}

/*
  V3 = ไปขวา
  ตั้งปุ่มเป็น PUSH
*/
BLYNK_WRITE(V3) {
  handleMovement(
    param.asInt(),
    speedStep,
    0,
    0
  );
}

/*
  V4 = หมุนซ้าย
  ตั้งปุ่มเป็น PUSH
*/
BLYNK_WRITE(V4) {
  handleMovement(
    param.asInt(),
    0,
    0,
    -speedStep
  );
}

/*
  V5 = หมุนขวา
  ตั้งปุ่มเป็น PUSH
*/
BLYNK_WRITE(V5) {
  handleMovement(
    param.asInt(),
    0,
    0,
    speedStep
  );
}

/*
  V6 = เปิด/ปิดเครื่องดูดจำลองด้วย LED
  ตั้งปุ่มเป็น SWITCH
*/
BLYNK_WRITE(V6) {
  vacum_status = param.asInt();

  Serial.print("[VACUUM LED] ");
  Serial.println(vacum_status ? "ON" : "OFF");
}

/*
  V7 = Slider ความเร็ว
  ตั้งช่วง 10–100
*/
BLYNK_WRITE(V7) {
  int speedPercent = constrain(param.asInt(), 10, 100);
  speedStep = speedPercent / 100.0;

  Serial.print("[SPEED] ");
  Serial.print(speedPercent);
  Serial.println("%");
}
/*
  V8 = หยุดการเดินของหุ่นยนต์
  ตั้งปุ่มเป็น SWITCH
*/
BLYNK_WRITE(V8){
    handleMovement(
    param.asInt(),
    0,
    0,
    0
  );
}


/**************** เมื่อเชื่อมต่อ BLYNK ****************/
BLYNK_CONNECTED() {
  previousBlynkConnected = true;

  // เพื่อความปลอดภัย ให้เริ่มต้นด้วยการหยุด
  stopRobot();
  vacum_status = false;

  // รีเซ็ตปุ่มเคลื่อนที่ในแอป
  Blynk.virtualWrite(V0, 0);
  Blynk.virtualWrite(V1, 0);
  Blynk.virtualWrite(V2, 0);
  Blynk.virtualWrite(V3, 0);
  Blynk.virtualWrite(V4, 0);
  Blynk.virtualWrite(V5, 0);
  Blynk.virtualWrite(V6, 0);

  // โหลดค่าความเร็วล่าสุดจาก Slider
  Blynk.syncVirtual(V7);

  Serial.println("[BLYNK] Connected");
}

/**************** คำสั่งจาก SERIAL ****************/
void processCommand(String cmd) {
  cmd.trim();

  if (cmd.length() == 0) {
    return;
  }

  // รูปแบบ vx,vy,omega
  if (cmd.indexOf(',') >= 0) {
    float vals[3] = {0, 0, 0};
    int idx = 0;
    int start = 0;

    for (int i = 0; i <= cmd.length() && idx < 3; i++) {
      if (i == cmd.length() || cmd.charAt(i) == ',') {
        vals[idx++] = cmd.substring(start, i).toFloat();
        start = i + 1;
      }
    }

    targetVx = constrain(vals[0], -1.0, 1.0);
    targetVy = constrain(vals[1], -1.0, 1.0);
    targetOmega = constrain(vals[2], -1.0, 1.0);

    Serial.print("[CMD] vx=");
    Serial.print(targetVx);
    Serial.print(" vy=");
    Serial.print(targetVy);
    Serial.print(" omega=");
    Serial.println(targetOmega);

    return;
  }

  char c = cmd.charAt(0);

  switch (c) {
    case 'm':
      vacum_status = true;
      Blynk.virtualWrite(V6, 1);
      Serial.println("Working...");
      break;

    case 'n':
      vacum_status = false;
      Blynk.virtualWrite(V6, 0);
      Serial.println("OFF");
      break;

    case 'w':
      targetVy = speedStep;
      targetVx = 0;
      targetOmega = 0;
      break;

    case 's':
      targetVy = -speedStep;
      targetVx = 0;
      targetOmega = 0;
      break;

    case 'a':
      targetVx = -speedStep;
      targetVy = 0;
      targetOmega = 0;
      break;

    case 'd':
      targetVx = speedStep;
      targetVy = 0;
      targetOmega = 0;
      break;

    case 'q':
      targetOmega = -speedStep;
      targetVx = 0;
      targetVy = 0;
      break;

    case 'e':
      targetOmega = speedStep;
      targetVx = 0;
      targetVy = 0;
      break;

    case 'x':
      stopRobot();
      break;

    case '+':
      speedStep = constrain(speedStep + 0.1, 0.1, 1.0);
      Serial.print("[SPEED] ");
      Serial.println(speedStep);
      return;

    case '-':
      speedStep = constrain(speedStep - 0.1, 0.1, 1.0);
      Serial.print("[SPEED] ");
      Serial.println(speedStep);
      return;

    default:
      Serial.println(
        "[CMD ไม่รู้จัก] ใช้: w a s d q e x m n หรือ vx,vy,omega"
      );
      return;
  }

  Serial.print("[CMD] vx=");
  Serial.print(targetVx);
  Serial.print(" vy=");
  Serial.print(targetVy);
  Serial.print(" omega=");
  Serial.println(targetOmega);
}

/**************** อ่าน SERIAL ****************/
void readSerialCommand() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      if (inputBuffer.length() > 0) {
        processCommand(inputBuffer);
        inputBuffer = "";
      }
    } else {
      inputBuffer += c;
    }
  }
}

/******************** SETUP ********************/
void setup() {
  Serial.begin(9600);

  // LED จำลองเครื่องดูด
  pinMode(vacum, OUTPUT);
  digitalWrite(vacum, LOW);
  vacum_status = false;

  Serial.println();
  Serial.println("=== หุ่นยนต์ 3 ล้อ Omni + Blynk ===");
  Serial.println("กำลังตั้งค่ามอเตอร์...");

  for (int i = 0; i < NUM_MOTORS; i++) {
    pinMode(motors[i].pinIN1, OUTPUT);
    pinMode(motors[i].pinIN2, OUTPUT);
    pinMode(motors[i].pinENA, OUTPUT);
    pinMode(motors[i].pinENCA, INPUT);
    pinMode(motors[i].pinENCB, INPUT);

    attachInterruptArg(
      digitalPinToInterrupt(motors[i].pinENCA),
      readEncoderISR,
      &motors[i],
      RISING
    );
  }

  // เริ่มต้นให้มอเตอร์หยุด
  setRobotVelocity(0, 0, 0);

  t0 = micros();
  lastSample = millis();

  Serial.print("กำลังเชื่อมต่อ WiFi: ");
  Serial.println(ssid);

  Blynk.begin(
    BLYNK_AUTH_TOKEN,
    ssid,
    pass
  );

  Serial.println("ระบบพร้อมใช้งาน");
}

/******************** LOOP ********************/
void loop() {
  Blynk.run();

  /*
    ถ้าเคยเชื่อมต่อ Blynk แล้วการเชื่อมต่อหลุด
    ให้หยุดหุ่นยนต์และปิด LED
  */
  if (previousBlynkConnected && !Blynk.connected()) {
    stopRobot();
    vacum_status = false;
    previousBlynkConnected = false;

    Serial.println("[SAFETY] Blynk disconnected, robot stopped");
  }

  if (Blynk.connected()) {
    previousBlynkConnected = true;
  }

  unsigned long now = millis();

  /************* รับคำสั่งจาก Serial *************/
  readSerialCommand();

  /************* สั่งความเร็วมอเตอร์ *************/
  setRobotVelocity(
    targetVx,
    targetVy,
    targetOmega
  );

  /************* คำนวณความเร็ว Encoder *************/
  if (now - lastSample >= SAMPLE_MS) {
    deltaT = (now - lastSample) / 1000.0;
    lastSample = now;
    t = (micros() - t0) / 1.0e6;

    for (int i = 0; i < NUM_MOTORS; i++) {
      long pos;

      noInterrupts();
      pos = motors[i].posi;
      interrupts();

      long deltaP = pos - motors[i].lastPosi;
      motors[i].lastPosi = pos;

      motors[i].rpm =
        (deltaP / deltaT) *
        (60.0 / CPR_TOTAL);

      motors[i].rpmFilt =
        ALPHA * motors[i].rpmFilt +
        (1.0 - ALPHA) * motors[i].rpm;
    }
  }

  /************* LED จำลองเครื่องดูด *************/
  digitalWrite(
    vacum,
    vacum_status ? HIGH : LOW
  );
}