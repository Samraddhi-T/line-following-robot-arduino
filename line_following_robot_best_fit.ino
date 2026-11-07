/*
  LINE FOLLOWING ROBOT + OBSTACLE DETECTION
  Best-fit sketch based on the project description and uploaded robot photo.

  Assumed hardware:
    Arduino Uno, L298N, 2 DC motors, 2 digital IR line sensors,
    HC-SR04 ultrasonic sensor, servo, 16x2 I2C LCD.

  ASSUMED PIN MAP (change these if your wiring differs):
    IR left  -> D2       IR right -> D3
    Servo    -> D4
    L298N ENA-> D5       L298N ENB-> D6
    L298N IN1-> D7       IN2 -> D8
    L298N IN3-> D9       IN4 -> D10
    HC-SR04 TRIG -> D12  ECHO -> D13
    I2C LCD SDA -> A4    SCL -> A5

  SENSOR ASSUMPTION:
    IR module gives LOW on the black line and HIGH on the light floor.
    If yours gives the opposite output, change LINE_LEVEL to HIGH.

  SAFETY:
    Test with wheels lifted first. Keep the robot on a clear, flat surface.
    Motor direction and sensor logic may need tuning for your exact wiring.
*/

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);  // Try 0x3F if your LCD stays blank.
Servo scanServo;

// IR sensors
const byte IR_LEFT = 2;
const byte IR_RIGHT = 3;
const byte LINE_LEVEL = LOW;

// Servo + ultrasonic sensor
const byte SERVO_PIN = 4;
const byte TRIG_PIN = 12;
const byte ECHO_PIN = 13;

const int OBSTACLE_LIMIT_CM = 20;
const byte SERVO_CENTER = 90;
const byte SERVO_LEFT = 150;
const byte SERVO_RIGHT = 30;

// L298N
const byte ENA = 5;
const byte IN1 = 7;
const byte IN2 = 8;
const byte ENB = 6;
const byte IN3 = 9;
const byte IN4 = 10;

// Tune these values for your motors and track.
const int FORWARD_SPEED = 145;
const int TURN_SPEED = 135;
const int REVERSE_SPEED = 125;

int lastDirection = 0; // -1 = left, 1 = right, 0 = none

int distanceCM() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(3);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 25000UL);
  if (duration == 0) return 400;
  return (int)(duration * 0.0343 / 2.0);
}

void leftMotor(int speedValue) {
  speedValue = constrain(speedValue, -255, 255);
  if (speedValue > 0) {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    analogWrite(ENA, speedValue);
  } else if (speedValue < 0) {
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
    analogWrite(ENA, -speedValue);
  } else {
    digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
  }
}

void rightMotor(int speedValue) {
  speedValue = constrain(speedValue, -255, 255);
  if (speedValue > 0) {
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
    analogWrite(ENB, speedValue);
  } else if (speedValue < 0) {
    digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
    analogWrite(ENB, -speedValue);
  } else {
    digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
    analogWrite(ENB, 0);
  }
}

void stopRobot() {
  leftMotor(0);
  rightMotor(0);
}

void forward() {
  leftMotor(FORWARD_SPEED);
  rightMotor(FORWARD_SPEED);
}

void turnLeft() {
  leftMotor(0);
  rightMotor(TURN_SPEED);
}

void turnRight() {
  leftMotor(TURN_SPEED);
  rightMotor(0);
}

void reverseBriefly() {
  leftMotor(-REVERSE_SPEED);
  rightMotor(-REVERSE_SPEED);
  delay(300);
  stopRobot();
}

void lcdStatus(const char *top, const char *bottom) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(top);
  lcd.setCursor(0, 1);
  lcd.print(bottom);
}

void lineFollow() {
  bool leftSeesLine = (digitalRead(IR_LEFT) == LINE_LEVEL);
  bool rightSeesLine = (digitalRead(IR_RIGHT) == LINE_LEVEL);

  // Both sensors on the line: proceed forward.
  if (leftSeesLine && rightSeesLine) {
    forward();
    lastDirection = 0;
  }
  // Only left sensor sees line: steer left.
  else if (leftSeesLine && !rightSeesLine) {
    turnLeft();
    lastDirection = -1;
  }
  // Only right sensor sees line: steer right.
  else if (!leftSeesLine && rightSeesLine) {
    turnRight();
    lastDirection = 1;
  }
  // Line temporarily lost: search in the last correction direction.
  else {
    if (lastDirection == -1) turnLeft();
    else if (lastDirection == 1) turnRight();
    else stopRobot();
  }
}

void avoidObstacle() {
  stopRobot();
  lcdStatus("Obstacle ahead", "Scanning sides");
  delay(250);

  scanServo.write(SERVO_LEFT);
  delay(450);
  int leftDistance = distanceCM();

  scanServo.write(SERVO_RIGHT);
  delay(450);
  int rightDistance = distanceCM();

  scanServo.write(SERVO_CENTER);
  delay(350);

  // If both sides appear blocked, reverse briefly before choosing
  // the side with the greater measured clearance.
  if (leftDistance < OBSTACLE_LIMIT_CM &&
      rightDistance < OBSTACLE_LIMIT_CM) {
    lcdStatus("Path blocked", "Repositioning");
    reverseBriefly();
  }

  if (leftDistance >= rightDistance) {
    lcdStatus("Avoiding", "Turning left");
    turnLeft();
    delay(500);
  } else {
    lcdStatus("Avoiding", "Turning right");
    turnRight();
    delay(500);
  }

  stopRobot();
  delay(150);
  lcdStatus("Line Following", "Resuming");
  delay(250);
}

void setup() {
  pinMode(IR_LEFT, INPUT);
  pinMode(IR_RIGHT, INPUT);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  scanServo.attach(SERVO_PIN);
  scanServo.write(SERVO_CENTER);

  lcd.init();
  lcd.backlight();

  stopRobot();
  lcdStatus("Line Follower", "Starting...");
  delay(1200);
}

void loop() {
  scanServo.write(SERVO_CENTER);
  int frontDistance = distanceCM();

  if (frontDistance > 0 && frontDistance <= OBSTACLE_LIMIT_CM) {
    avoidObstacle();
  } else {
    lineFollow();
  }

  delay(10);
}
