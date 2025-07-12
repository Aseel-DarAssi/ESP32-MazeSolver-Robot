#include <Wire.h>
#include <Adafruit_VL53L0X.h>

// === IR Sensor Pins ===
#define IR_LEFT 4
#define IR_RIGHT 5

// === Motor Driver Pins ===
#define IN1 16
#define IN2 17
#define IN3 18
#define IN4 19
#define ENA 32
#define ENB 33

// === Encoder Pins ===
#define ENCODER_L_A 34
#define ENCODER_L_B 35
#define ENCODER_R_A 14
#define ENCODER_R_B 27

// === Encoder Counters ===
volatile long encoderCountLeft = 0;
volatile long encoderCountRight = 0;
long lastCountLeft = 0, lastCountRight = 0;
int speedLeft = 0, speedRight = 0;
unsigned long lastSpeedCheck = 0;

// === Target Speeds ===
int targetSpeedLeft = 50;
int targetSpeedRight = 50;

// === Sensor Object ===
Adafruit_VL53L0X sensorFront = Adafruit_VL53L0X();

// === Target forward distance ===
bool movedForward = false;
int startDistance = 0;
int targetDistance = 300; // in millimeters = 30 cm

// === Encoder Interrupts ===
void IRAM_ATTR onEncoderLeftA() {
  if (digitalRead(ENCODER_L_A) == digitalRead(ENCODER_L_B))
    encoderCountLeft++;
  else
    encoderCountLeft--;
}

void IRAM_ATTR onEncoderRightA() {
  if (digitalRead(ENCODER_R_A) == digitalRead(ENCODER_R_B))
    encoderCountRight++;
  else
    encoderCountRight--;
}

// === Initialize Sensors ===
void setupSensors() {
  if (!sensorFront.begin()) {
    Serial.println("Failed to initialize front VL53L0X");
    while (1);
  }
}

// === Initialize Motors ===
void setupMotors() {
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
}

// === Initialize Encoders ===
void setupEncoders() {
  pinMode(ENCODER_L_A, INPUT);
  pinMode(ENCODER_L_B, INPUT);
  pinMode(ENCODER_R_A, INPUT);
  pinMode(ENCODER_R_B, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENCODER_L_A), onEncoderLeftA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_R_A), onEncoderRightA, CHANGE);
}

// === Update speed using encoder values ===
void updateSpeed() {
  if (millis() - lastSpeedCheck >= 200) {
    long pulsesLeft = encoderCountLeft - lastCountLeft;
    long pulsesRight = encoderCountRight - lastCountRight;

    speedLeft = pulsesLeft * 5;
    speedRight = pulsesRight * 5;

    lastCountLeft = encoderCountLeft;
    lastCountRight = encoderCountRight;
    lastSpeedCheck = millis();
  }
}

// === Movement Functions ===
void driveMotors(int pwmL, int pwmR) {
  // LEFT motor (reversed direction, going backward)
  digitalWrite(IN1, LOW);   // reversed
  digitalWrite(IN2, HIGH);  // reversed

  // RIGHT motor (kept normal, going forward)
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  analogWrite(ENA, pwmL);
  analogWrite(ENB, pwmR);
}

void moveForward() {
  driveMotors(targetSpeedLeft, targetSpeedRight);
}

void stopMotors() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

// === Setup ===
void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);  // ESP32 I2C pins

  setupSensors();
  setupMotors();
  setupEncoders();

  pinMode(IR_LEFT, INPUT);
  pinMode(IR_RIGHT, INPUT);

  Serial.println("System Ready");
}

// === Main Loop ===
void loop() {
  // Read distance from front sensor
  VL53L0X_RangingMeasurementData_t frontData;
  sensorFront.rangingTest(&frontData, false);
  int dFront = (frontData.RangeStatus != 4) ? frontData.RangeMilliMeter : 9999;

  updateSpeed();

  // First time, record starting distance
  if (!movedForward && dFront < 2000) {  // ensure space is available ahead
    startDistance = dFront;
    movedForward = true;
    moveForward();
    Serial.println("Started moving forward");
  }

  // If target distance is reached, stop
  if (movedForward && dFront <= (startDistance - targetDistance)) {
    stopMotors();
    Serial.println("Reached target distance of 30 cm. Stopped.");
    while (1);  // stop the program
  }

  // Print debug information
  Serial.print("dFront: "); Serial.print(dFront);
  Serial.print(" Speed L: "); Serial.print(speedLeft);
  Serial.print(" Speed R: "); Serial.println(speedRight);

  delay(100);
}
