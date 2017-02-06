#include <Servo.h>

const int motorA1 = 7; // IN A1 or IN1 (Motor A Direction)
const int motorA2 = 8; // IN A2 or IN2 (Motor A Direction)
const int motorB1 = 9; // IN B1 or IN3 (Motor B Direction)
const int motorB2 = 10; // IN B2 or IN4 (Motor B Direction)
const int ENA = 5; // ENA (PWM or Enable for Motor A)
const int ENB = 6; // ENB (PWM or Enable for Motor B)
const int servoPin = 11; //Servo that controls the ultrasonic sensor
const int eggServoPin = 3;
const int tonePin = 2; //Controls speakers
const int sensor1 = A0; //Line-following sensors
const int sensor2 = A1;
const int sensor3 = A2;
int state1; //Detects whether or not line-following sensor detects a black line
int state2;
int state3;
boolean withinDistance; //Checks if the robot is within a certain distance from an obstacle
int rememberLastPosition; //Remembers whether the robot was to the left of the black line, to the right of the black line, or on the black line
float cupDistance; //Calculates the distance from the ultrasonic sensor to the cup
boolean ledOn = false;
boolean obstaclePassed = false;

const int rangeTriggerPin = A5;
const int rangeSensorPin = 4;
const unsigned long rangeTimeout = 2000;
unsigned long echoDelay = 0;
Servo servo;
Servo eggServo;

const char notes[] = "gabygabyxzCDxzCDabywabywzCDEzCDEbywFCDEqywFGDEqi        azbC"; // a space represents a rest, letters represent notes
int length = sizeof(notes); // the number of notes
int beats[] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 3, 3, 16,};
int tempo = 100;

void setup() {
  pinMode(motorA1, OUTPUT); //set pins on motors as outputs
  pinMode(motorA2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(tonePin, OUTPUT); //set pin of speaker as output
  pinMode(rangeTriggerPin, OUTPUT);
  pinMode(rangeSensorPin, INPUT);
  pinMode(sensor1, INPUT); //set pins of line sensors as inputs
  pinMode(sensor2, INPUT);
  pinMode(sensor3, INPUT);
  pinMode(13, OUTPUT); //set LED pin on board as output
  servo.attach(servoPin); //intialize servos
  eggServo.attach(eggServoPin);
  servo.write(75); //move servo on sensor to face forward
  eggServo.write(180); //move servo on egg to hold the egg
}

void loop() {
  // put your main code here, to run repeatedly:
  state1 = digitalRead(sensor1); //read whether or not line sensors detect black or white (HIGH = black, LOW = white)
  state2 = digitalRead(sensor2);
  state3 = digitalRead(sensor3);
  withinDistance = obstacleDistance(10.0) > 2.0 && obstacleDistance(10.0) <= 10.0; //check if an obstacle is detected between 2 and 10 inches
  if (obstaclePassed){ 
    noTone(tonePin); //shuts off tone that indicates an obstacle is detected
    obstaclePassed = false;
  }
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
    noTone(tonePin); //turn off tone of robot moving
    if (state1 == HIGH && state2 == HIGH && state3 == HIGH && obstacleDetected(8.0)) { //line sensors detect all black and an obstacle is detected within 8 inches away
      stop();
      noTone(tonePin);
      delay(1000);
      cupDistance = obstacleDistance(10.0); //check distance from sensor to cup
      while (cupDistance < 4.5 || cupDistance > 4.9) { //egg dropping mechanism is not aligned with the cup
        if (cupDistance < 4.5) { //robot is too close to the cup
          moveBackward(128, 10);
        }
        else if (cupDistance > 4.9) { //robot is too far away from the cup
          moveForward(128, 10);
        }
        cupDistance = obstacleDistance(10.0); //check distance from robot to cup again
      }
      stop();
      delay(1000);
      eggServo.write(90); //turn egg servo to 90 degree position to allow egg to follow
      playSong(); //play song to show that egg is ready
      while(true){ //stop the robot for an infinite amount of time
      stop();
      }
    }
    else {
      while (withinDistance) { //robot will still stop as long as an obstacle is detected
        tone(tonePin, map((echoDelay / 148.148), 0, 30, 60, 960)); //play tone to indicate obstacle is detected
        delayMicroseconds(10);
        delay(1500);
        withinDistance = obstacleDistance(10.0) > 2.0 && obstacleDistance(10.0) <= 10.0; //check if robot still detects an obstacle
      }
      obstaclePassed = true; //will allow the obstacle detection tone to turn off once the loop starts
    }
  }
  if (ledOn){
    digitalWrite(13, HIGH); //flash LED on and turn tone on
    tone(tonePin, 440);
  } 
  else{
    digitalWrite(13, LOW); //turn LED off and shut off tone
    noTone(tonePin);
  }
  ledOn = !ledOn; //allows LED to flash on and off


}

void moveForward(int speed, int time) { //moves robot forward by powering both the left and right motors forward at equal speeds
  analogWrite(ENA, speed); //set speed of motor
  digitalWrite(motorA1, HIGH);
  digitalWrite(motorA2, LOW);
  analogWrite(ENB, speed);
  digitalWrite(motorB1, HIGH);
  digitalWrite(motorB2, LOW);
  delay(time); //amount of time the motors will move
}

void moveBackward(int speed, int time) { //moves robot backward by powering both the left and right motors backwards at equal speeds
  analogWrite(ENA, speed); //set speed of motor
  digitalWrite(motorA1, LOW);
  digitalWrite(motorA2, HIGH);
  analogWrite(ENB, speed);
  digitalWrite(motorB1, LOW);
  digitalWrite(motorB2, HIGH);
  delay(time); //amount of time the motors will move
}

void circle(int speed1, int speed2, int time) { //powers left and right motors with different speeds to move the robot in a circle
  analogWrite(ENA, speed1);
  digitalWrite(motorA1, HIGH);
  digitalWrite(motorA2, LOW);
  analogWrite(ENB, speed2);
  digitalWrite(motorB1, HIGH);
  digitalWrite(motorB2, LOW);
  delay(time); //amount of time the motors will move
}

void backwardsCircle(int speed1, int speed2, int time) { //powers left and right motors with different speeds backwards
  analogWrite(ENA, speed1);
  digitalWrite(motorA1, LOW);
  digitalWrite(motorA2, HIGH);
  analogWrite(ENB, speed2);
  digitalWrite(motorB1, LOW);
  digitalWrite(motorB2, HIGH);
  delay(time); //amount of time the motors will move
}

void turnRight(int speed, int time) { //pivots the robot left by moving the left motors forward and right motors backward
  analogWrite(ENA, speed); //set speed of motor
  digitalWrite(motorA1, HIGH);
  digitalWrite(motorA2, LOW);
  analogWrite(ENB, speed);
  digitalWrite(motorB1, LOW);
  digitalWrite(motorB2, HIGH);
  delay(time); //amount of time the motors will move
}

void turnLeft(int speed, int time) { //pivots the robot right by moving the left motors backward and right motors forward
  analogWrite(ENA, speed); //set speed of motor
  digitalWrite(motorA1, LOW);
  digitalWrite(motorA2, HIGH);
  analogWrite(ENB, speed);
  digitalWrite(motorB1, HIGH);
  digitalWrite(motorB2, LOW);
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
  if (echoDelay == 0) { 
    noTone(2);
    return false;
  }
  else { 
    analogWrite(2, HIGH);

    return length <= withinInches;
  }
}



void playTone(int tone, int duration) {
  for (long i = 0; i < duration * 1000L; i += tone * 2) {
    digitalWrite(tonePin, HIGH);
    delayMicroseconds(tone);
    digitalWrite(tonePin, LOW);
    delayMicroseconds(tone);
  }
}

void playNote(char note, int duration) {
  char names[] = { 'c', 'd', 'e', 'f', 'g', 'x', 'a', 'z', 'b', 'C', 'y', 'D', 'w', 'E', 'F', 'q', 'G', 'i' };
  // c=C4, C = C5. These values have been tuned.
  int tones[] = { 1898, 1690, 1500, 1420, 1265, 1194, 1126, 1063, 1001, 947, 893, 843, 795, 749, 710, 668, 630, 594 }; //match notes with frequencies

  // play the tone corresponding to the note name
  for (int i = 0; i < 18; i++) {
    if (names[i] == note) {
      playTone(tones[i], duration);
    }
  }
}

void playSong() {
   for (int i = 0; i < length; i++) { //play through array of notes
    if (notes[i] == ' ') {
      delay(beats[i] * tempo); // rest
    } else {
      playNote(notes[i], beats[i] * tempo);
    }
   }
}
