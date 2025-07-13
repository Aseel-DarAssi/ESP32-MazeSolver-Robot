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

// === Maze Constants ===
#define MAZE_SIZE 8
int maze[MAZE_SIZE][MAZE_SIZE];
int robotX = 0, robotY = 0;
int direction = 0; // 0: North, 1: East, 2: South, 3: West
const int goalX = 3, goalY = 3;
int dx[4] = { 0, 1, 0, -1 };
int dy[4] = { -1, 0, 1, 0 };

// === Encoder Variables ===
volatile long encoderCountLeft = 0;
volatile long encoderCountRight = 0;
long lastCountLeft = 0, lastCountRight = 0;
int speedLeft = 0, speedRight = 0;
unsigned long lastSpeedCheck = 0;

// === Motor Speed ===
int targetSpeedLeft = 150;
int targetSpeedRight = 150;

// === Distance Sensor ===
Adafruit_VL53L0X sensorFront = Adafruit_VL53L0X();

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

// === Setup ===
void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  if (!sensorFront.begin()) {
    Serial.println("Failed to initialize VL53L0X");
    while (1);
  }

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);

  pinMode(ENCODER_L_A, INPUT);
  pinMode(ENCODER_L_B, INPUT);
  pinMode(ENCODER_R_A, INPUT);
  pinMode(ENCODER_R_B, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENCODER_L_A), onEncoderLeftA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_R_A), onEncoderRightA, CHANGE);

  pinMode(IR_LEFT, INPUT);
  pinMode(IR_RIGHT, INPUT);

  Serial.println("Robot ready with improved obstacle detection");
}

// === Speed Update ===
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

// === Movement ===
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

void turnLeft90() {
  driveMotors(-90, 90);
  delay(600);
  stopMotors();
}

void turnBack180() {
  driveMotors(90, -90);
  delay(1200);
  stopMotors();
}

int getFrontDistance() {
  VL53L0X_RangingMeasurementData_t frontData;
  sensorFront.rangingTest(&frontData, false);
  return (frontData.RangeStatus != 4) ? frontData.RangeMilliMeter : 9999;
}

void moveOneCellForward() {
  int startDistance = getFrontDistance();
  if (startDistance < 150) {
    Serial.println("Wall ahead, cannot move forward");
    stopMotors();
    return;
  }
  driveMotors(targetSpeedLeft, targetSpeedRight);

  while (true) {
    int current = getFrontDistance();
    if (current < 150) {
      Serial.println("Sudden wall ahead, stopping");
      stopMotors();
      break;
    }
    if (current <= (startDistance - 300)) {
      stopMotors();
      break;
    }
    delay(50);
  }
}

// === Maze Handling ===
void updateWalls() {
  int frontDist = getFrontDistance();
  bool wallF = frontDist < 150;
  bool wallR = digitalRead(IR_RIGHT) == LOW;
  bool wallL = digitalRead(IR_LEFT) == LOW;

  int x = robotX;
  int y = robotY;
  int dirs[3] = { direction, (direction + 1) % 4, (direction + 3) % 4 };
  bool walls[3] = { wallF, wallR, wallL };

  for (int i = 0; i < 3; i++) {
    int d = dirs[i];
    int nx = x + dx[d];
    int ny = y + dy[d];
    if (nx >= 0 && ny >= 0 && nx < MAZE_SIZE && ny < MAZE_SIZE) {
      if (walls[i]) {
        maze[nx][ny] = 1;
      } else if (maze[nx][ny] == 1) {
        maze[nx][ny] = 99;
      }
    }
  }
}

void floodFill(int gx, int gy) {
  for (int i = 0; i < MAZE_SIZE; i++) {
    for (int j = 0; j < MAZE_SIZE; j++) {
      if (maze[i][j] != 1)
        maze[i][j] = 99;
    }
  }
  maze[gx][gy] = 0;
  bool changed = true;
  while (changed) {
    changed = false;
    for (int x = 0; x < MAZE_SIZE; x++) {
      for (int y = 0; y < MAZE_SIZE; y++) {
        if (maze[x][y] < 99) {
          for (int d = 0; d < 4; d++) {
            int nx = x + dx[d];
            int ny = y + dy[d];
            if (nx >= 0 && ny >= 0 && nx < MAZE_SIZE && ny < MAZE_SIZE) {
              if (maze[nx][ny] > maze[x][y] + 1 && maze[nx][ny] != 1) {
                maze[nx][ny] = maze[x][y] + 1;
                changed = true;
              }
            }
          }
        }
      }
    }
  }
}

int chooseBestDirection() {
  int minVal = 100;
  int bestDir = -1;
  for (int d = 0; d < 4; d++) {
    int nx = robotX + dx[d];
    int ny = robotY + dy[d];
    if (nx >= 0 && ny >= 0 && nx < MAZE_SIZE && ny < MAZE_SIZE) {
      if (maze[nx][ny] < minVal && maze[nx][ny] != 1) {
        minVal = maze[nx][ny];
        bestDir = d;
      }
    }
  }
  return bestDir;
}

void rotateTo(int targetDir) {
  int diff = (targetDir - direction + 4) % 4;
  if (diff == 1) turnRight90();
  else if (diff == 2) turnBack180();
  else if (diff == 3) turnLeft90();
  direction = targetDir;
}

void loop() {
  updateSpeed();
  updateWalls();

  floodFill(goalX, goalY);

  int nextDir = chooseBestDirection();

  if (nextDir == -1) {
    Serial.println("No available path.");
    stopMotors();
    while (1);
  } else {
    rotateTo(nextDir);
    moveOneCellForward();

    robotX += dx[direction];
    robotY += dy[direction];

    Serial.print("Robot at (");
    Serial.print(robotX);
    Serial.print(", ");
    Serial.print(robotY);
    Serial.print("), direction: ");
    Serial.println(direction);
  }

  delay(200);
}
