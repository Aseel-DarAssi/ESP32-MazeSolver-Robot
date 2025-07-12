#define IN1 16
#define IN2 17
#define IN3 18
#define IN4 19
#define ENA 32
#define ENB 33

void setup() {
  Serial.begin(115200);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);

  Serial.println("✅ Testing 90-degree right turn...");

  turnRight90();

  Serial.println("🛑 Finished turning. Check rotation.");
}

void loop() {
}

void driveMotors(int pwmL, int pwmR) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH); 
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);  
  analogWrite(ENA, pwmL);
  analogWrite(ENB, pwmR);
}

void stopMotors() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  analogWrite(ENA, 0); analogWrite(ENB, 0);
}

void turnRight90() {
  driveMotors(90, -90);  
  delay(600);            
  stopMotors();
}
