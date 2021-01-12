#define buzzer 36
#define LED1pin 25
#define LED2pin 38

int LED1status = 0;
int LED2status = 0;

char receivedChars[3];
boolean newData = false;

//motor driver LM298 control pin
int pin1R = 8;
int pin2R = 9;
int pin1L = 10;
int pin2L = 11;

//interrupt pin
int interruptL1 = 21;
int interruptL2 = 20;
int interruptR1 = 18;
int interruptR2 = 19;

// encoder parameters
long encoderValueR = 0;
long encoderValueL = 0;

int forwardStart = 0;
int rightStart = 0;
int leftStart = 0;
int targetValue = 0;

//////////////////////////////////////////////////////
//OLED Display
int sclk = 4; //brown--- connect this to the display module CLK pin (Serial Clock)
int mosi = 3; //orange--- connect this to the display module DIN pin (Serial Data)
int rst  = 7; //yellow--- connect this to the display module RES pin (Reset)
int dc   = 6; //green--- connect this to the display module D/C  pin (Data or Command)
int cs   = 5; //blue--- connect this to the display module CS  pin (Chip Select)

// Color definitions
#define  BLACK           0x0000
#define BLUE            0x0006
#define RED             0xF800
#define GREEN           0x07E0
#define CYAN            0x07FF
#define MAGENTA         0xF81F
#define YELLOW          0xFFE0  
#define WHITE           0xFFFF
#define BACKGROUND      0x0000

#include <Adafruit_SSD1331.h>
Adafruit_SSD1331 display = Adafruit_SSD1331(cs, dc, mosi, sclk, rst);

//MLX90614 Temperature Sensor
#include <Wire.h>
#include <Adafruit_MLX90614.h>

Adafruit_MLX90614 mlx = Adafruit_MLX90614();

///////////////////////////////////////////////////////
//Black Line Sensors
//see black tape is 1, LED off
//see white is 0, LED on
#define blackLinePinAnalogTR A1   //Top Right
#define blackLinePinTR 31         //Top Right
#define blackLinePinAnalogTL A2   //Top Left
#define blackLinePinTL 32         //Top Left
#define blackLinePinAnalogFR A0   //Far Right (near the wheel)
#define blackLinePinFR 30         //Far Right (near the wheel)
#define blackLinePinAnalogFL A3   //Far Left (near the wheel)
#define blackLinePinFL 33         //Far Left (near the wheel)

bool blackLineTR = 0; //top right
bool blackLineTL = 0; // top left
bool blackLineFR = 0; //far right
bool blackLineFL = 0; //far left
int  blackLineAnalogTR = 0; //top right
int  blackLineAnalogTL = 0; //top left
int  blackLineAnalogFR = 0; //far right
int  blackLineAnalogFL = 0; //far left

int rising_blackLineFR = 0;
int rising_blackLineFL = 0;
int prev_blackLineFR = 0;
int prev_blackLineFL = 0;

int x_axis = -1;
int y_axis = 0;
int x_destination = 0;
int y_destination = 0;
int directions = 0; // 0: East, 1: South, 2: West, 3: North
int facing_diff = 0;
int stopit = 1;

//////////////////////////////////////////////////////

//////////////////////////////////////////////////////
void setup()
{
  Serial.begin(115200);
  Serial2.begin(115200);

  //LEDs
  pinMode(LED1pin, OUTPUT);
  pinMode(LED2pin, OUTPUT);
  
  //Black Line sensor
  pinMode(blackLinePinTR, INPUT);
  pinMode(blackLinePinTL, INPUT);
  pinMode(blackLinePinFR, INPUT);
  pinMode(blackLinePinFL, INPUT);
  
  //Motor
  pinMode(pin1L, OUTPUT);
  pinMode(pin2L, OUTPUT);
  pinMode(pin1R, OUTPUT);
  pinMode(pin2R, OUTPUT);
  
  //Encoder
  pinMode(interruptL1, INPUT_PULLUP);
  pinMode(interruptL2, INPUT_PULLUP);
  pinMode(interruptR1, INPUT_PULLUP);
  pinMode(interruptR2, INPUT_PULLUP);
  
  //setup interrupt
  attachInterrupt(digitalPinToInterrupt(interruptL1), countL, FALLING);
  attachInterrupt(digitalPinToInterrupt(interruptR1), countR, FALLING);
  
  //Initial display
  display.begin();
  Display();

  mlx.begin();

  while (!Serial2) {
    ;
  }
}

//////////////////////////////////////////////////////

//////////////////////////////////////////////////////
void loop()
{
  if (stopit){
    IncomingData();
  }
  BlackLine();
  Directions();
  MotorControl();
  Reached();
  if (leftStart == 0 && rightStart == 0) {
    CheckCrossing();
  }
}
//////////////////////////////////////////////////////

//////////////////////////////////////////////////////
void IncomingData() {
  stopCar();
  static byte ndx = 0;
  char rc;
  
  while (Serial2.available() > 0 && newData == false) {
    rc = Serial2.read();
    if (rc != '\n') {
      receivedChars[ndx++] = rc;
    }
    else {
      receivedChars[ndx] = '\0'; // terminate the string
      ndx = 0;
      newData = true;
    }
  }
  
  if (newData == true) {
    Serial.print(receivedChars);
    x_destination = receivedChars[0] - '0';
    y_destination = receivedChars[1] - '0';
    stopit = 0;
    digitalWrite(LED1pin, LOW);
    digitalWrite(LED2pin, LOW);
    newData = false;
  }
}


void Reached(){
  if (x_destination == x_axis && y_destination == y_axis && stopit == 0){
    stopCar();
    Measurement();
    stopit = 1;
  }
}

void Measurement(){
  delay(500);
  //Take Temperature
  float temperature = mlx.readObjectTempC();

  //LEDs
  if (temperature > 39){
    digitalWrite(LED1pin, HIGH);
    tone(buzzer, 1000, 2000);
  }
  else{
    digitalWrite(LED2pin, HIGH);
  }

  //Update OLED
  if (temperature > 39){
    display.fillScreen(RED);
  }
  else{
    display.fillScreen(GREEN);
  }
  display.setTextColor(BLACK);  
  display.setTextSize(3);
  display.setCursor(5,20);
  display.print(temperature,1);
  display.setCursor(75,20);
  display.print("C");
}

void MotorControl(){
  if (facing_diff == 1) TurnLeft();
  else if (facing_diff == 2) Turn180();
  else if (facing_diff == 3) TurnRight();
  else GoForward();
}

void Directions(){
  if (x_destination > x_axis){
    facing_diff = directions;
  }
  else if (x_destination < x_axis){
    facing_diff = directions - 2;
  }
  else if (y_destination > y_axis){
    facing_diff = directions - 3;
  }
  else if (y_destination < y_axis){
    facing_diff = directions;
  }
  facing_diff = (facing_diff+4)%4;
}

void GoForward(){
  if (blackLineTR == 1 and blackLineTL == 0) {
    turnCarR();
    forwardStart = 0;
  }
  else if (blackLineTL == 1 and blackLineTR == 0) {
    turnCarL();
    forwardStart = 0;
  }
  else {
    if (forwardStart == 0) {
      stopCar();
      targetValue = 500;
      encoderValueL = 0;
      encoderValueR = 0;
      forwardStart = 1;
    }
    if (encoderValueL > targetValue or encoderValueR > targetValue) {
      stopCar();
    }
    else {
      if (encoderValueL > encoderValueR) {
        turnCarL();
      }
      else if (encoderValueR > encoderValueL) {
        turnCarR();
      }
      else {
        forwardCar();
      }
    }
  }
}

void TurnLeft(){
  if (leftStart == 0){
    stopCar();
    targetValue = 40;
    encoderValueL = 0;
    encoderValueR = 0;
    leftStart = 1;
  }
  if ((abs(encoderValueL) > targetValue or abs(encoderValueR) > targetValue) &&  blackLineTL == 1) {
    stopCar();
    forwardStart = 0;
    leftStart = 0;
    if (directions == 0){
      directions = 3;
    }
    else{
      directions--;
    }
  }
  else {
    if (abs(encoderValueL) < abs(encoderValueR)) {
      backwardLeftWheel();
      stopRightWheel();
    }
    else if (abs(encoderValueL) > abs(encoderValueR)) {
      forwardRightWheel();
      stopLeftWheel();
    }
    else {
      turnCarOnsiteL();
    }
  }
}

void TurnRight(){
  if (rightStart == 0){
    stopCar();
    targetValue = 40;
    encoderValueL = 0;
    encoderValueR = 0;
    rightStart = 1;
  }
  if ((abs(encoderValueL) > targetValue or abs(encoderValueR) > targetValue) &&  blackLineTR == 1) {
    stopCar();
    forwardStart = 0;
    rightStart = 0;
    directions = (directions+1)%4;
  }
  else {
    if (abs(encoderValueL) < abs(encoderValueR)) {
      forwardLeftWheel();
      stopRightWheel();
    }
    else if (abs(encoderValueL) > abs(encoderValueR)) {
      backwardRightWheel();
      stopLeftWheel();
    }
    else {
      turnCarOnsiteR();
    }
  }
}

void Turn180(){
  if (rightStart == 0){
    stopCar();
    targetValue = 50;
    encoderValueL = 0;
    encoderValueR = 0;
    rightStart = 1;
  }
  if ((abs(encoderValueL) > targetValue or abs(encoderValueR) > targetValue) &&  blackLineTR == 1) {
    stopCar();
    forwardStart = 0;
    rightStart = 0;
    directions = (directions+2)%4;
  }
  else {
    if (abs(encoderValueL) < abs(encoderValueR)) {
      forwardLeftWheel();
      stopRightWheel();
    }
    else if (abs(encoderValueL) > abs(encoderValueR)) {
      backwardRightWheel();
      stopLeftWheel();
    }
    else {
      turnCarOnsiteR();
    }
  }
}

void BlackLine() {
  blackLineTR = digitalRead(blackLinePinTR);
  blackLineTL = digitalRead(blackLinePinTL);
  blackLineFR = digitalRead(blackLinePinFR);
  blackLineFL = digitalRead(blackLinePinFL);
  blackLineAnalogTR = analogRead(blackLinePinAnalogTR);
  blackLineAnalogTL = analogRead(blackLinePinAnalogTL);
  blackLineAnalogFR = analogRead(blackLinePinAnalogFR);
  blackLineAnalogFL = analogRead(blackLinePinAnalogFL);
}

//Check if FR and FL crossed
void CheckCrossing(){
  if (blackLineFR == 1 && prev_blackLineFR == 0){
    rising_blackLineFR = 1;
  }
  if (blackLineFL == 1 && prev_blackLineFL == 0){
    rising_blackLineFL = 1;
  }
  if (rising_blackLineFR == 1 && rising_blackLineFL == 1){
    Display();
    rising_blackLineFR = 0;
    rising_blackLineFL = 0;
  }
  prev_blackLineFR = blackLineFR;
  prev_blackLineFL = blackLineFL;
}

//OLED Display
void Display(){
  stopCar();
  if (directions == 0) x_axis++;
  else if (directions == 1) y_axis--;
  else if (directions == 2) x_axis--;
  else if (directions == 3) y_axis++;
  display.fillScreen(BLACK);
  display.setTextColor(CYAN);  
  display.setTextSize(3);
  display.setCursor(0,20);
  display.print("(");
  display.setCursor(15,20);
  display.print(x_axis);
  display.setCursor(45,20);
  display.print(",");
  display.setCursor(60,20);
  display.print(y_axis);
  display.setCursor(75,20);
  display.print(")");
}

//////////////////////////////////////////////////////
//Interrupt subroutine
void countL() {
  if (digitalRead(interruptL2)) {
    encoderValueL--;
  }
  else {
    encoderValueL++;
  }
}
void countR() {
  if (digitalRead(interruptR2)) {
    encoderValueR++;
  }
  else {
    encoderValueR--;
  }
}
//////////////////////////////////////////////////////

//////////////////////////////////////////////////////
// Motor subrountines
void turnCarL() {
  digitalWrite(pin1R, 1);
  digitalWrite(pin2R, 0);
  digitalWrite(pin1L, 0);
  digitalWrite(pin2L, 0);
}
void turnCarR() {
  digitalWrite(pin1R, 0);
  digitalWrite(pin2R, 0);
  digitalWrite(pin1L, 1);
  digitalWrite(pin2L, 0);
}
void turnCarOnsiteL() {
  digitalWrite(pin1R, 1);
  digitalWrite(pin2R, 0);
  digitalWrite(pin1L, 0);
  digitalWrite(pin2L, 1);
}
void turnCarOnsiteR() {
  digitalWrite(pin1R, 0);
  digitalWrite(pin2R, 1);
  digitalWrite(pin1L, 1);
  digitalWrite(pin2L, 0);
}
void forwardCar() {
  digitalWrite(pin1R, 1);
  digitalWrite(pin2R, 0);
  digitalWrite(pin1L, 1);
  digitalWrite(pin2L, 0);
}
void backwardCar() {
  digitalWrite(pin1R, 0);
  digitalWrite(pin2R, 1);
  digitalWrite(pin1L, 0);
  digitalWrite(pin2L, 1);
}
void stopCar() {
  digitalWrite(pin1R, 0);
  digitalWrite(pin2R, 0);
  digitalWrite(pin1L, 0);
  digitalWrite(pin2L, 0);
}
void forwardRightWheel() {
  digitalWrite(pin1R, 1);
  digitalWrite(pin2R, 0);
}
void forwardLeftWheel() {
  digitalWrite(pin1L, 1);
  digitalWrite(pin2L, 0);
}
void backwardRightWheel() {
  digitalWrite(pin1R, 0);
  digitalWrite(pin2R, 1);
}
void backwardLeftWheel() {
  digitalWrite(pin1L, 0);
  digitalWrite(pin2L, 1);
}
void stopRightWheel() {
  digitalWrite(pin1R, 0);
  digitalWrite(pin2R, 0);
}
void stopLeftWheel() {
  digitalWrite(pin1L, 0);
  digitalWrite(pin2L, 0);
}
