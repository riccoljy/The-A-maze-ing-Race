#include <math.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <MeMCore.h>
#define LDR A1
#define ir A0
#define decoder2A A2
#define decoder2B A3


MeUltrasonicSensor ultrasonic(PORT_1);
MeBuzzer buzzer;
MeDCMotor leftMotor(M1);            // assigning leftMotor to port M1
MeDCMotor rightMotor(M2);           // assigning RightMotor to port M2
MeLineFollower lineFinder(PORT_2);  // assigning line block to port 2
uint8_t motorSpeed = 255;
uint8_t nintydelay = 500;
uint8_t double_turn_delay = 500;
float whiteArray[] = {615.00, 870.00, 762.00}, blackArray[] = {293.00, 520.00, 386.00}; 
float greyDiff[] = { whiteArray[0] - blackArray[0], whiteArray[1] - blackArray[1], whiteArray[2] - blackArray[2] };
int RGBWait = 100;
bool completed;
int colourcount;

void shineRed() {
  digitalWrite(decoder2A, HIGH);
  digitalWrite(decoder2B, LOW);
}

void shineGreen() {
  digitalWrite(decoder2A, HIGH);
  digitalWrite(decoder2B, HIGH);
}

void shineBlue() {
  digitalWrite(decoder2A, LOW);
  digitalWrite(decoder2B, HIGH);
}

void shineIR() {
  digitalWrite(decoder2A, LOW);
  digitalWrite(decoder2B, LOW);
}

int differentialIRRead(int sensorPin) {
  int ambientIR = 0;
  int reflectedIR = 0;
  shineGreen();
  ambientIR = analogRead(sensorPin);
  // Average ambient IR reading
  shineIR();
  reflectedIR = analogRead(sensorPin);
  int sensorReading = -(reflectedIR - ambientIR);
  return sensorReading;
}

//below here is for music
#define NOTE_B0  31
#define NOTE_C1  33
#define NOTE_CS1 35
#define NOTE_D1  37
#define NOTE_DS1 39
#define NOTE_E1  41
#define NOTE_F1  44
#define NOTE_FS1 46
#define NOTE_G1  49
#define NOTE_GS1 52
#define NOTE_A1  55
#define NOTE_AS1 58
#define NOTE_B1  62
#define NOTE_C2  65
#define NOTE_CS2 69
#define NOTE_D2  73
#define NOTE_DS2 78
#define NOTE_E2  82
#define NOTE_F2  87
#define NOTE_FS2 93
#define NOTE_G2  98
#define NOTE_GS2 104
#define NOTE_A2  110
#define NOTE_AS2 117
#define NOTE_B2  123
#define NOTE_C3  131
#define NOTE_CS3 139
#define NOTE_D3  147
#define NOTE_DS3 156
#define NOTE_E3  165
#define NOTE_F3  175
#define NOTE_FS3 185
#define NOTE_G3  196
#define NOTE_GS3 208
#define NOTE_A3  220
#define NOTE_AS3 233
#define NOTE_B3  247
#define NOTE_C4  262
#define NOTE_CS4 277
#define NOTE_D4  294
#define NOTE_DS4 311
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_FS4 370
#define NOTE_G4  392
#define NOTE_GS4 415
#define NOTE_A4  440
#define NOTE_AS4 466
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_CS5 554
#define NOTE_D5  587
#define NOTE_DS5 622
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_FS5 740
#define NOTE_G5  784
#define NOTE_GS5 831
#define NOTE_A5  880
#define NOTE_AS5 932
#define NOTE_B5  988
#define NOTE_C6  1047
#define NOTE_CS6 1109
#define NOTE_D6  1175
#define NOTE_DS6 1245
#define NOTE_E6  1319
#define NOTE_F6  1397
#define NOTE_FS6 1480
#define NOTE_G6  1568
#define NOTE_GS6 1661
#define NOTE_A6  1760
#define NOTE_AS6 1865
#define NOTE_B6  1976
#define NOTE_C7  2093
#define NOTE_CS7 2217
#define NOTE_D7  2349
#define NOTE_DS7 2489
#define NOTE_E7  2637
#define NOTE_F7  2794
#define NOTE_FS7 2960
#define NOTE_G7  3136
#define NOTE_GS7 3322
#define NOTE_A7  3520
#define NOTE_AS7 3729
#define NOTE_B7  3951
#define NOTE_C8  4186
#define NOTE_CS8 4435
#define NOTE_D8  4699
#define NOTE_DS8 4978
#define REST      0

int tempo = 300;
int melody[] = {  
  NOTE_C5,4, //1
  NOTE_F5,4, NOTE_F5,8, NOTE_G5,8, NOTE_F5,8, NOTE_E5,8,
  NOTE_D5,4, NOTE_D5,4, NOTE_D5,4,
  NOTE_G5,4, NOTE_G5,8, NOTE_A5,8, NOTE_G5,8, NOTE_F5,8,
  NOTE_E5,4, NOTE_C5,4, NOTE_C5,4,
  NOTE_A5,4, NOTE_A5,8, NOTE_AS5,8, NOTE_A5,8, NOTE_G5,8,
  NOTE_F5,4, NOTE_D5,4, NOTE_C5,8, NOTE_C5,8,
  NOTE_D5,4, NOTE_G5,4, NOTE_E5,4,

  NOTE_F5,2, NOTE_C5,4, //8 
  NOTE_F5,4, NOTE_F5,8, NOTE_G5,8, NOTE_F5,8, NOTE_E5,8,
  NOTE_D5,4, NOTE_D5,4, NOTE_D5,4,
  NOTE_G5,4, NOTE_G5,8, NOTE_A5,8, NOTE_G5,8, NOTE_F5,8,
  NOTE_E5,4, NOTE_C5,4, NOTE_C5,4,
  NOTE_A5,4, NOTE_A5,8, NOTE_AS5,8, NOTE_A5,8, NOTE_G5,8,
  NOTE_F5,4, NOTE_D5,4, NOTE_C5,8, NOTE_C5,8,
  NOTE_D5,4, NOTE_G5,4, NOTE_E5,4,
  NOTE_F5,2
};

int notes = sizeof(melody) / sizeof(melody[0]) / 2;
int wholenote = (60000 * 4) / tempo;
int divider = 0, noteDuration = 0;
void celebrate() {

  for (int thisNote = 0; thisNote < notes * 2; thisNote = thisNote + 2) {

    // calculates the duration of each note
    divider = melody[thisNote + 1];
    if (divider > 0) {
      noteDuration = (wholenote) / divider;
    } else if (divider < 0) {
      noteDuration = (wholenote) / abs(divider);
      noteDuration *= 1.5; 
    }

    buzzer.tone(melody[thisNote], noteDuration * 0.9);

    delay(noteDuration);
  }
}


double computeDistance(long r1, long g1, long b1, long r2, long g2, long b2) {
  return sqrt(sq(r2 - r1) + sq(g2 - g1) + sq(b2 - b1));
}


// returns 1=> red, returns 2=> green, returns 3=>orange, returns 4=>purple, returns 5=>blue, returns 6=>white
int detectColour() {
  // Perfect values for each colour: RGB
  int blue[] = {138, 169, 231};
  int green[] = {82, 122, 89};
  int red[] = {228, 99, 70};
  int purple[] = {163, 130, 178};
  int orange[] = {231, 155, 90};
  int white[] = {255, 255, 255};
  // Flashes Red colour
  int redReading;
  redReading = 0;
  shineRed();
  for (int i = 0; i < 5; i += 1) {
    redReading += (analogRead(LDR) - blackArray[0]) / (greyDiff[0]) * 255;
    delay(RGBWait);
  }
  redReading /= 5;

  // Flashes Green colour
  int greenReading;
  greenReading = 0;
  shineGreen();
  for (int i = 0; i < 5; i += 1) {
    greenReading += (analogRead(LDR) - blackArray[1]) / (greyDiff[1]) * 255;
    delay(RGBWait);
  }
  greenReading /= 5;

  // Flashes Blue colour
  int blueReading;
  blueReading = 0;
  shineBlue();
  for (int i = 0; i < 5; i += 1) {
    blueReading += (analogRead(LDR) - blackArray[2]) / (greyDiff[2]) * 255;
    delay(RGBWait);
  }
  blueReading /= 5;
  Serial.println(redReading);
  Serial.println(greenReading);
  Serial.println(blueReading);

  // Calculate distances to each reference color
  double distances[6];
  // Serial.println(redReading);
  // Serial.println(greenReading);
  // Serial.println(blueReading);
  distances[0] = computeDistance(redReading, greenReading, blueReading, red[0], red[1], red[2]);
  distances[1] = computeDistance(redReading, greenReading, blueReading, green[0], green[1], green[2]);
  distances[2] = computeDistance(redReading, greenReading, blueReading, orange[0], orange[1], orange[2]);
  distances[3] = computeDistance(redReading, greenReading, blueReading, purple[0], purple[1], purple[2]);
  distances[4] = computeDistance(redReading, greenReading, blueReading, blue[0], blue[1], blue[2]);
  distances[5] = computeDistance(redReading, greenReading, blueReading, white[0], white[1], white[2]);
  int minIndex = 0;
  for (int i = 1; i < 6; i++) {
    if (distances[i] < distances[minIndex])  minIndex = i;
  }
  return minIndex + 1;  // because the required result is 1-based
}


void moveForward() {
  leftMotor.run(-motorSpeed);
  rightMotor.run(motorSpeed);
}

void stopMotor() {
  leftMotor.run(0);
  rightMotor.run(0);
}

void turnRight() {
  long line_detect_time = millis();
  while (millis() - line_detect_time < 370) {
    leftMotor.run(-motorSpeed);
    rightMotor.run(-motorSpeed);
  }
  stopMotor();
}

void turnLeft() {
  long line_detect_time = millis();
  while (millis() - line_detect_time < 380) {
    leftMotor.run(motorSpeed);
    rightMotor.run(motorSpeed);
  }
  stopMotor();
}

void uTurn() {
  long line_detect_time = millis();
  while (millis() - line_detect_time < 700) {
    leftMotor.run(motorSpeed);
    rightMotor.run(motorSpeed);
  }
  stopMotor();
}

void doubleLeftTurn() {
  long line_detect_time = millis();
  while (millis() - line_detect_time < 380) {
    leftMotor.run(motorSpeed);
    rightMotor.run(motorSpeed);
  }
  stopMotor();
  line_detect_time = millis();
  while (millis() - line_detect_time < 1000) {
    moveForward();
  }
  stopMotor();
  line_detect_time = millis();
  while (millis() - line_detect_time < 425) {
    leftMotor.run(motorSpeed);
    rightMotor.run(motorSpeed);
  }
}

void doubleRightTurn() {
  long line_detect_time = millis();
  while (millis() - line_detect_time < 360) {
    leftMotor.run(-motorSpeed);
    rightMotor.run(-motorSpeed);
  }
  stopMotor();
  line_detect_time = millis();
  while (millis() - line_detect_time < 850) {
    moveForward();
  }
  stopMotor();
  line_detect_time = millis();
  while (millis() - line_detect_time < 380) {
    leftMotor.run(-motorSpeed);
    rightMotor.run(-motorSpeed);
  }
}

void nudgeLeft() {
    leftMotor.run(-motorSpeed+100);
    rightMotor.run(motorSpeed);
}

void nudgeRight() {
    leftMotor.run(-motorSpeed);
    rightMotor.run(motorSpeed-150);
}

bool lineDetected() {
  return (lineFinder.readSensors() == S1_IN_S2_IN);
}

void setup() {
  Serial.begin(9600); 
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);
  completed = false;
  colourcount = 0;
}

void loop() {
  if (completed) {stopMotor(); return;}
  if (!lineDetected()) {
    double dist = ultrasonic.distanceCm();
    if (dist < 9) nudgeLeft();
    else if (dist > 16) moveForward();
    else if (differentialIRRead(ir) > 105 ||  dist > 14.5) nudgeRight();
    else moveForward();
    return;
  }
  stopMotor();

  //if reach here, means line detected
  int colour = detectColour();
  colourcount++;
  if (colourcount == 1) colour = detectColour();
  Serial.println(colour);
  if (colour == 1 /*red*/) {
    turnLeft();
    return;
  }
  if (colour == 2 /*green*/) {
    turnRight();
    return;
  }
  if (colour == 3 /*orange*/) {
    uTurn();
    return;
  }
  if (colour == 4 /*purple*/) {
    doubleLeftTurn();
    return;
  }
  if (colour == 5 /*blue*/) {
    doubleRightTurn();
    return;
  }
  if (colour == 6 /*white*/) {
    stopMotor();
    // Serial.println("Completed!");
    celebrate();
    completed = true;
  }
}
