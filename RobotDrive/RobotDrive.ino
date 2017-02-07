#include <Servo.h>

const int motorA1 = 7; // IN A1 or IN1 (Motor A Direction)
const int motorA2 = 8; // IN A2 or IN2 (Motor A Direction)
const int motorB1 = 9; // IN B1 or IN3 (Motor B Direction)
const int motorB2 = 10; // IN B2 or IN4 (Motor B Direction)
const int ENA = 5; // ENA (PWM or Enable for Motor A)
const int ENB = 6; // ENB (PWM or Enable for Motor B)
const int servoPin = 11; //Servo that controls the ultrasonic sensor
const int sensor1 = A0; //Line-following sensors
const int sensor2 = A1;
const int sensor3 = A2;
int state1; //Detects whether or not line-following sensor detects a black line
int state2;
int state3;
boolean withinDistance; //Checks if the robot is within a certain distance from an obstacle
int rememberLastPosition; //Remembers whether the robot was to the left of the black line, to the right of the black line, or on the black line
boolean obstaclePassed = false;
const int rangeTriggerPin = A5;
const int rangeSensorPin = 4;
const unsigned long rangeTimeout = 2000;
unsigned long echoDelay = 0;
Servo servo;


void setup() {
  pinMode(motorA1, OUTPUT); //set pins on motors as outputs
  pinMode(motorA2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(rangeTriggerPin, OUTPUT);
  pinMode(rangeSensorPin, INPUT);
  pinMode(sensor1, INPUT); //set pins of line sensors as inputs
  pinMode(sensor2, INPUT);
  pinMode(sensor3, INPUT);
  servo.attach(servoPin); //intialize servos
  servo.write(75); //move servo on sensor to face forward
  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:
  state1 = digitalRead(sensor1); //read whether or not line sensors detect black or white (HIGH = black, LOW = white)
  state2 = digitalRead(sensor2);
  state3 = digitalRead(sensor3);
  withinDistance = obstacleDistance(10.0) > 2.0 && obstacleDistance(10.0) <= 10.0; //check if an obstacle is detected between 2 and 10 inches
  if (state2 == HIGH) { //middle sensor detects black
    if (state1 == HIGH && state3 == LOW) { //robot is positioned slightly off to the right
      rememberLastPosition = 3; //remember that robot was off to the right
      circle(0, 200, 10); //move slightly to the left
    }
    else {
      if (state3 == HIGH && state1 == LOW) { //robot is positioned slightly off to the left
        rememberLastPosition = 2; //remember that robot was off to the left
        circle(164, 0, 10); //move slightly to the right
      }
      else { //robot is centered perfectly
        rememberLastPosition = 1; //remember the robot was on track
        moveForward(164, 50);
      }
    }
  }
  else if (state2 == LOW) {
    if (state1 == HIGH) { //robot is positioned way off to the right
      rememberLastPosition = 3; //remember that robot was off to the right
      turnLeft(200, 50); //turn hard to the left
    }
    else {
      if (state3 == HIGH) { //robot is positioned way off to the left
        rememberLastPosition = 2; //remember that robot was off to the left
        turnRight(200, 50); //turn hard to the right
      }
      else { //robot is completely off track
        switch (rememberLastPosition) { //use value of rememberLastPosition to possibly bring the robot back on track
          case (3): //robot was off to the right
            turnLeft(200, 50);
          case (2): //robot was off to the left
            turnRight(200, 50);
          case (1): //robot was on track
            moveBackward(200, 50);
        }
      }
    }
  }
  if (withinDistance) { //obstacle is detected 8 inches away
    stop();
    while (withinDistance) { //robot will still stop as long as an obstacle is detected
        Serial.println("Obstacle Detected");
        delay(1500);
        withinDistance = obstacleDistance(10.0) > 2.0 && obstacleDistance(10.0) <= 10.0; //check if robot still detects an obstacle
      }   
  }
 }




void moveForward(int speed, int time) { //moves robot forward by powering both the left and right motors forward at equal speeds
  analogWrite(ENA, speed); //set speed of motor
  digitalWrite(motorA1, HIGH);
  digitalWrite(motorA2, LOW);
  analogWrite(ENB, speed);
  digitalWrite(motorB1, LOW);
  digitalWrite(motorB2, HIGH);
  delay(time); //amount of time the motors will move
}

void moveBackward(int speed, int time) { //moves robot backward by powering both the left and right motors backwards at equal speeds
  analogWrite(ENA, speed); //set speed of motor
  digitalWrite(motorA1, LOW);
  digitalWrite(motorA2, HIGH);
  analogWrite(ENB, speed);
  digitalWrite(motorB1, HIGH);
  digitalWrite(motorB2, LOW);
  delay(time); //amount of time the motors will move
}

void circle(int speed1, int speed2, int time) { //powers left and right motors with different speeds to move the robot in a circle
  analogWrite(ENA, speed1);
  digitalWrite(motorA1, HIGH);
  digitalWrite(motorA2, LOW);
  analogWrite(ENB, speed2);
  digitalWrite(motorB1, LOW);
  digitalWrite(motorB2, HIGH);
  delay(time); //amount of time the motors will move
}

void backwardsCircle(int speed1, int speed2, int time) { //powers left and right motors with different speeds backwards
  analogWrite(ENA, speed1);
  digitalWrite(motorA1, LOW);
  digitalWrite(motorA2, HIGH);
  analogWrite(ENB, speed2);
  digitalWrite(motorB1, HIGH);
  digitalWrite(motorB2, LOW);
  delay(time); //amount of time the motors will move
}

void turnRight(int speed, int time) { //pivots the robot left by moving the left motors forward and right motors backward
  analogWrite(ENA, speed); //set speed of motor
  digitalWrite(motorA1, HIGH);
  digitalWrite(motorA2, LOW);
  analogWrite(ENB, speed);
  digitalWrite(motorB1, HIGH);
  digitalWrite(motorB2, LOW);
  delay(time); //amount of time the motors will move
}

void turnLeft(int speed, int time) { //pivots the robot right by moving the left motors backward and right motors forward
  analogWrite(ENA, speed); //set speed of motor
  digitalWrite(motorA1, LOW);
  digitalWrite(motorA2, HIGH);
  analogWrite(ENB, speed);
  digitalWrite(motorB1, LOW);
  digitalWrite(motorB2, HIGH);
  delay(time); //amount of time the motors will move
}

void stop() { //shuts off power on both motors for an indefinite amount of time
  digitalWrite(ENA, LOW);
  digitalWrite(ENB, LOW);
}

void stop(int time) { //shuts off power on both motors to wait for a duration
  digitalWrite(ENA, LOW);
  digitalWrite(ENB, LOW);
  delay(time); //amount of time the motors will stop
}

float obstacleDistance(float withinInches) { //uses ultrasonic sensor to calculate the distance between the sensor and an object
  digitalWrite(rangeTriggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(rangeTriggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(rangeTriggerPin, LOW);
  unsigned long lengthLimit = long(rangeTimeout / 13.5 * withinInches); //creates limit of signal to match a certain distance
  echoDelay = pulseIn(rangeSensorPin, HIGH, lengthLimit); //measure time it takes to send signal and retrieve it
  return echoDelay / 148.148;
}

boolean obstacleDetected(float withinInches) { //uses ultrssonic sensor to detect whether or not an object is within a certain distance
  digitalWrite(rangeTriggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(rangeTriggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(rangeTriggerPin, LOW);
  echoDelay = pulseIn(rangeSensorPin, HIGH, rangeTimeout); //measure time it takes to send signal and retrieve it
  float length = echoDelay / 148.148;
  return length <= withinInches;
 }




