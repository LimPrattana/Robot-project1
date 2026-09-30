const int in1Pin = 14;  // H-Bridge input pins
const int in2Pin = 0;   // <-- เปลี่ยนเป็นขา output ที่ว่างจริงของคุณ (ห้ามใช้ 34/35/36/39)

void setup()
{
  Serial.begin(9600);
  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);
  digitalWrite(in1Pin, LOW);
  digitalWrite(in2Pin, LOW);
  Serial.println("+ - sets direction of motors, any other key stops motors");
}

void loop()
{
  if (Serial.available()) {
    char ch = Serial.read();

    if (ch == '\n' || ch == '\r') return;  // ข้ามตัวขึ้นบรรทัดใหม่

    if (ch == '+')
    {
      Serial.println("CW");
      digitalWrite(in1Pin, LOW);
      digitalWrite(in2Pin, HIGH);
    }
    else if (ch == '-')
    {
      Serial.println("CCW");
      digitalWrite(in1Pin, HIGH);
      digitalWrite(in2Pin, LOW);
    }
    else
    {
      Serial.println("Stop motors");
      digitalWrite(in1Pin, LOW);
      digitalWrite(in2Pin, LOW);
    }
  }
}