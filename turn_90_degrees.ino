// === Motor Pins ===
#define IN1 16  // Left motor direction
#define IN2 17
#define IN3 18  // Right motor direction
#define IN4 19
#define ENA 32  // Left motor speed (PWM)
#define ENB 33  // Right motor speed (PWM)

void setup() {
  // Initialize serial communication
  Serial.begin(115200);

  // Set motor pins as outputs
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);

  // Start message
  Serial.println("⌛ Waiting 2 seconds before starting rotation...");
  delay(2000);  

  Serial.println("✅ Starting 90-degree right turn (left wheel forward, right wheel backward)");
  turnRight90();  

  Serial.println("🛑 Turn completed. Check movement.");
}

void loop() {
  // Nothing in the main loop
}

// === Function to drive wheels with different speeds ===
void driveMotors(int pwmL, int pwmR) {
  // Left wheel forward
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  // Right wheel backward
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  // Send PWM signals for speed control
  analogWrite(ENA, pwmL);
  analogWrite(ENB, pwmR);
}

// === Function to stop the wheels ===
void stopMotors() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

// === Function for approximately 90-degree right turn (based on time) ===
void turnRight90() {
  driveMotors(150, 150);  // Left forward, right backward
  delay(236);             
  stopMotors();
}
