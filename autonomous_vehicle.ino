/*Jonathan Arenson
UMD Engineering Competition
*/



#include <math.h>

#include <SoftwareSerial.h>
#include "enes100.h"
#include "OSV.h"
#include <Wire.h>
#include "Adafruit_TCS34725.h"

/* Example code for the Adafruit TCS34725 breakout library */

/* Connect SCL    to analog 5
   Connect SDA    to analog 4
   Connect VDD    to 3.3V DC
   Connect GROUND to common ground */
   
/* Initialise with default values (int time = 2.4ms, gain = 1x) */
// Adafruit_TCS34725 tcs = Adafruit_TCS34725();

/* Initialise with specific int time and gain values */
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_1X);




//Declare the software serial which on pins 8,9 which
//are connected to the RF module tx and rx.
SoftwareSerial sSerial(11,12);

//Declare an enes100::RfClient which is the interface for
//receiving markers and sending messages to the vision system.
enes100::RfClient<SoftwareSerial> rf(&sSerial);

//Declare an enes100::Marker.  This marker will store the id,
//x, y, theta, and time values of a received marker.
enes100::Marker marker;

/* Define pinout of Arduino to match physical connections */

const int pingPin = 5;
const int boomHeight = 290;

int pos = 0;

loopStates state = GETORIENTED;

void setup(void) { 
  sSerial.begin(9600);
  Serial.begin(9600);
  
    /* Define all 7 pins as outputs to the TB6612FNG driver */
  pinMode(PWMA,OUTPUT);
  pinMode(AIN1,OUTPUT);
  pinMode(AIN2,OUTPUT);
  pinMode(PWMB,OUTPUT);
  pinMode(BIN1,OUTPUT);
  pinMode(BIN2,OUTPUT);
  pinMode(STBY,OUTPUT);
  // Start up the library


  rf.resetServer();
  delay(500);
  rf.sendMessage("Average J's Connected");
  delay(500);
}


//
// This function sets all MC speed to 0 which will stop the OSV
// 
void stop(){
  digitalWrite (AIN1,HIGH);
  digitalWrite (AIN2,HIGH);
  analogWrite(PWMA,255);
  digitalWrite (BIN1,HIGH);
  digitalWrite (BIN2,HIGH);
  analogWrite(PWMB,255);
}

void start() {
  digitalWrite (STBY,HIGH);
}

//
// This function will have the OSV go straight at speed speed.
// The OSV will continue to go straight after the funtion returns.
//
void goStraight(int speed){  
  digitalWrite (AIN1,LOW);
  digitalWrite (AIN2,HIGH);
  analogWrite(PWMA,speed);
  digitalWrite (BIN1,LOW);
  digitalWrite (BIN2,HIGH);
  analogWrite(PWMB,speed);
}

//
// This function will have the OSV spin to right at speed speed
// The function first spins only the left wheels forward and then pauses for delayTime msecs
// The function then stops the left wheels and spins the right wheels in reverse for delyTime msecs
// After which all wheels are stopped.
//
// This function should be continuously called until the OSV reaches its desired heading
// 
void spinRight(int delayTime){
  int speed = SPINSPEED;
  stop();
  digitalWrite (AIN1,LOW);
  digitalWrite (AIN2,HIGH);
  analogWrite(PWMA,speed);
  digitalWrite (BIN1,HIGH);
  digitalWrite (BIN2,LOW);
  analogWrite(PWMB,speed); 
  delay(delayTime);
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
}

//
// This function will have the OSV spin to left at speed speed
// The function first spins only the left wheels in reverse and then pauses for delayTime msecs
// The function then stops the left wheels and spins the right wheels forward for delyTime msecs
// After which all wheels are stopped.
//
// This function should be continuously called until the OSV reaches its desired heading
//
void spinLeft(int delayTime){
  int speed = SPINSPEED;
  stop();
  digitalWrite (AIN1,HIGH);
  digitalWrite (AIN2,LOW);
  analogWrite(PWMA,speed);
  digitalWrite (BIN1,LOW);
  digitalWrite (BIN2,HIGH);
  analogWrite(PWMB,speed); 
  delay(delayTime);
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
}

//
// This function spins both left wheels forward and stops the right wheels to make the OSV go right.
// This OSV will continue to move in the direction after the call
//
void turnRight(){
  int speed = TURNSPEED;               //speed 0 to 255

  digitalWrite (AIN1,LOW);
  digitalWrite (AIN2,HIGH);
  analogWrite(PWMA,speed);
  digitalWrite (BIN1,HIGH);
  digitalWrite (BIN2,LOW);
  analogWrite(PWMB,speed); 
}

//
// This function spins both right wheels forward and stops the left wheels to make the OSV go left.
// This OSV will continue to move in the direction after the call
//
void turnLeft(){
  int speed = TURNSPEED;               //speed 0 to 255

  digitalWrite (AIN1,HIGH);
  digitalWrite (AIN2,LOW);
  analogWrite(PWMA,speed);
  digitalWrite (BIN1,LOW);
  digitalWrite (BIN2,HIGH);
  analogWrite(PWMB,speed); 
}

//
// This function spins both right wheels forward and spins the left wheels forward at half speed to make the OSV vere left.
// This OSV will continue to move in the direction after the call
//
void vereLeft(){
  int speed = VERESPEED;               //speed 0 to 255

  
  digitalWrite (AIN1,LOW);
  digitalWrite (AIN2,HIGH);
  analogWrite(PWMA,speed/VEREFACTOR);
  digitalWrite (BIN1,LOW);
  digitalWrite (BIN2,HIGH);
  analogWrite(PWMB,speed); 
}

//
// This function spins both left wheels forward and spins the rigth wheels forward at half speed to make the OSV vere right.
// This OSV will continue to move in the direction after the call
//
void vereRight(){
  int speed = VERESPEED;               //speed 0 to 255

  digitalWrite (AIN1,LOW);
  digitalWrite (AIN2,HIGH);
  analogWrite(PWMA,speed);
  digitalWrite (BIN1,LOW);
  digitalWrite (BIN2,HIGH);
  analogWrite(PWMB,speed/VEREFACTOR); 
}


void printGoStraight(int speed){
  Serial.print("Go Straight speed: ");Serial.println(speed);
  goStraight(speed);
}

void printTurnLeft(){
  Serial.println("Turn Left");
  turnLeft();
}

void printTurnRight(){
  Serial.println("Turn Right");
  turnRight();
}

void printSpinLeft(int delayTime){
  Serial.print("Spin Left Delay time: "); Serial.println(delayTime);
  spinLeft(delayTime);
}

void printSpinRight(int delayTime){
  Serial.print("Spin Right Delay time: "); Serial.println(delayTime);
  spinRight(delayTime);
}


void printVereRight(){
  Serial.println("Vere Right");
  vereRight();
}

void printVereLeft(){
  Serial.println("Vere Left");
  vereLeft();
}

void printStop(){
  Serial.println("Stop");
  stop();
}

int getQuad(float theta){
  int quad;
  if (marker.theta <= 0.0 && marker.theta >= -PI/2){
    quad = 1;
  }
  else if (marker.theta >= 0.0 && marker.theta <= PI/2) {
    quad = 2;
  }else if (marker.theta >= PI/2 && marker.theta <= PI){
    quad = 3;
  }
  else{
    quad = 4;
  }
  return quad;
}


bool inCenter(float y) {
  bool center = false;
  
  if (y >= LOWCENTER && y <= HIGHCENTER){
    center = true;
  }
  return center;
}

bool getOriented(enes100::Marker &marker){
  bool oriented = false;
  float diff;
  
  //
  // if we are in the middle
  //
  if (inCenter(marker.y)) {
    //
    // If we are out of the landing zone
    // Then set oriented to true
    // NOTE: we are assuming that we are facing the correct heading  
    if (marker.x > OUTOFLANDING) {
      oriented = true;
    }
    //
    // if we are facing the water
    //
    else if (marker.theta > 0.0 - FINEHEADINGERROR && marker.theta < 0.0 + FINEHEADINGERROR)
    {
      printGoStraight(STRAIGHTSPEED);
    }
    //
    // We need to turn right if theta is positive and left if it negative
    //
    else if (marker.theta > COARSEHEADINGERROR) {
      printSpinRight(SPINDELAY);
    }
    else if (marker.theta > MIDHEADINGERROR) {
      printTurnRight();
    }
    else if (marker.theta > FINEHEADINGERROR) {
      printVereRight();
    } 
    else if (marker.theta > -COARSEHEADINGERROR){
      printSpinLeft(SPINDELAY);
    }   
    else if (marker.theta > -MIDHEADINGERROR){
      printTurnLeft();
    }   
    else{
      printVereLeft();
    }   
  }
  //
  // We are on the lower y size of the zone....We need to get to middle
  // take heading PI/2
  // 
  else if (marker.y <= LOWCENTER) {
    int thetaQuad = getQuad(marker.theta);
    if (thetaQuad == 1){
      printSpinLeft(SPINDELAY);
    }
    else if (thetaQuad == 4){
      printSpinRight(SPINDELAY);
    }  
    else if (thetaQuad == 2){
      diff = PI/2 - marker.theta;
      if (diff > COARSEHEADINGERROR){
        printSpinLeft(SPINDELAY);
      }
      else if (diff > MIDHEADINGERROR){
        printTurnLeft();
      }
      else if (diff > FINEHEADINGERROR){
        printVereLeft();
      }
      else {
        printGoStraight(STRAIGHTSPEED);
      }
    }
    else {
      diff = marker.theta - PI/2;
      if (diff > COARSEHEADINGERROR){
        printSpinRight(SPINDELAY);   
      }
      else if (diff > MIDHEADINGERROR){
        printTurnRight();
      }
      else if (diff > FINEHEADINGERROR){
        printVereRight();
      }
      else {
        printGoStraight(STRAIGHTSPEED);
      }
    }
  }  
  //
  // We are on the upper y side of the zone... We need to get to the middle
  // take heading -PI/2 
  //
  else {
    int thetaQuad = getQuad(marker.theta);
    if (thetaQuad == 2){
      printSpinRight(SPINDELAY);
    }
    else if (thetaQuad == 3){
      printSpinLeft(SPINDELAY);
    }  
    else if (thetaQuad == 1){
      diff = marker.theta + PI/2;   //We know theta is negative.  So we want to find out how close we are to -PI/2...
      if (diff > COARSEHEADINGERROR){
        printSpinRight(SPINDELAY);
      }
      else if (diff > MIDHEADINGERROR){
        printTurnRight();
      }
      else if (diff > FINEHEADINGERROR){
        printVereRight();
      }
      else {
        printGoStraight(STRAIGHTSPEED);
      }
    }
    else {
      diff = -marker.theta - PI/2;
      if (diff > COARSEHEADINGERROR){
        printSpinLeft(SPINDELAY);   
      }
      else if (diff > MIDHEADINGERROR){
        printTurnLeft();
      }
      else if (diff > FINEHEADINGERROR){
        printVereLeft();
      }
      else {
        printGoStraight(STRAIGHTSPEED);
      }
    }
  }
  
  return oriented;
}



bool moveToWater3(enes100::Marker &marker){
  bool atWater = false;
  
  //
  // If we are at the water we are done
  // 
  if (measureHeight() > 60){
    printStop();
    atWater = true;
  }
  else {    
    //
    // Keep the car in the middle
    //
    if (marker.x > WATERLEFTTURN - .1 && marker.x < WATERLEFTTURN + .1){
      printGoStraight(STRAIGHTSPEED2);
    }
    else if (marker.x > WATERLEFTTURN){
      printTurnRight();
    }
    else {
      printTurnLeft();
    }
  }   
 return atWater;
}


bool moveToWater2(enes100::Marker &marker){
  bool atHeading = false;
  float diff;
  
  //
  // If we are at the heading we are done
  // 
  if (marker.theta > -PI/2 - MOVEERROR2 && marker.theta < -PI/2 + MOVEERROR2){
    printStop();
    atHeading = true;
  }
  else {    
    //
    //Move and make correction to heading
    //
    int thetaQuad = getQuad(marker.theta);
    if (thetaQuad == 1){
      printSpinRight(SPINDELAY);
    }
    else if (thetaQuad == 4){
       printSpinLeft(SPINDELAY);
    }
    else if (thetaQuad == 2){
      diff = -PI/2 - marker.theta;
      if (diff > MOVEERROR){
        printSpinRight(SPINDELAY);
      }
      else {
        printSpinRight(SPINDELAY/2); //reduce spin time so we don't over spin
      }
    }
    else {
      diff = marker.theta + PI/2;
      if (diff > MOVEERROR){
        printSpinLeft(SPINDELAY);   
      }
      else {
        printSpinLeft(SPINDELAY/2); //reduce spin time so we don't over spin
      }
    }  
  }   
  return atHeading;
}

bool moveToWater1(enes100::Marker &marker){
  bool atTurn = false;
  
  int straightSpeed = STRAIGHTSPEED;
  if (marker.x > WATERLEFTTURN - .25){
    straightSpeed = STRAIGHTSPEED;
  }
  
  if (marker.x > WATERLEFTTURN){
    atTurn = true;
    printStop();
  }
  else {    
    //
    // Keep the car in the middle
    //
    if (marker.y > 1.0 && marker.y < 1.2){
      printGoStraight(straightSpeed);
    }
    else if (marker.y > 0.9 && marker.y <1.3){
      if (marker.y < 1.1){
        printVereLeft();
      }
      else {
        printVereRight();
      }
    }
    //
    // This code assumes that we will never be too close to the trouble.  Since get oriented lines us up in the middle
    //
    else {
      if (marker.y < 1.1){
        printTurnLeft();
      }
      else {
        printTurnRight();
      }
    } 
  }   
  return atTurn;
}


int measureHeight() {
    Serial.println("measure Height");
    // establish variables for duration of the ping, 
    // and the distance result in inches and centimeters:
    long duration, mm;
  
    // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
    // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
    pinMode(pingPin, OUTPUT);
    digitalWrite(pingPin, LOW);
    delayMicroseconds(2);
    digitalWrite(pingPin, HIGH);
    delayMicroseconds(5);
    digitalWrite(pingPin, LOW);
  
    // The same pin is used to read the signal from the PING))): a HIGH
    // pulse whose duration is the time (in microseconds) from the sending
    // of the ping to the reception of its echo off of an object.
    pinMode(pingPin, INPUT);
    duration = pulseIn(pingPin, HIGH);
  
    // convert the time into a distance
  
    mm = microsecondsToMillimeters(duration);
    long boulderHeight = boomHeight - mm;
    
    
    delay(100);
    
    return boulderHeight;
  
}

void sendHeight(){
  int height = measureHeight();
  if (height > 60) {
    height+=70;
  }
  if (height > 250) {
   height+=30;
  } 
  String heightString = String(height);
  rf.sendMessage("Height: " + heightString + "mm");
  return;
}


long microsecondsToMillimeters(long microseconds){
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 2.9 / 2;
}

void findColor() {
  uint16_t r, g, b, c, colorTemp, lux;
  
  tcs.getRawData(&r, &g, &b, &c);
  //colorTemp = tcs.calculateColorTemperature(r, g, b);
  //lux = tcs.calculateLux(r, g, b);
  
  if(g > 600){
    rf.sendMessage("Color: Green");
  }
  else{
    rf.sendMessage("Color: Black");
  }
    
  
  //Serial.print("Color Temp: "); Serial.print(colorTemp, DEC); Serial.print(" K - ");
  //Serial.print("Lux: "); Serial.print(lux, DEC); Serial.print(" - ");
  //Serial.print("R: "); Serial.print(r, DEC); Serial.print(" ");
  //Serial.print("G: "); Serial.print(g, DEC); Serial.print(" ");
  //Serial.print("B: "); Serial.print(b, DEC); Serial.print(" ");
  //Serial.print("C: "); Serial.print(c, DEC); Serial.print(" ");
  //Serial.println(" ");
}

void loop(void) {
    start();
   if (state != COMPLETE) {
     Serial.print("State: ");Serial.println(state);
   }
   switch(state) {
       case GETORIENTED:
         if(rf.receiveMarker(&marker, MARKERID)){
            Serial.print("Received Mark x = "); Serial.print(marker.x);
            Serial.print(" y = "); Serial.print(marker.y);
            Serial.print(" theta = "); Serial.println(marker.theta);
            if (getOriented(marker)){
              rf.sendMessage("GetOriented Complete");
              state = MOVETOWATER1;
            }
         }
         else {
            Serial.println("Failed to read marker");
            printStop();
         }
       
       break;
       
       case MOVETOWATER1:
         if(rf.receiveMarker(&marker, MARKERID)){
            Serial.print("Received Mark x = "); Serial.print(marker.x);
            Serial.print(" y = "); Serial.print(marker.y);
            Serial.print(" theta = "); Serial.println(marker.theta);
            if (moveToWater1(marker)){
              rf.sendMessage("MoveToBoulder1 Complete");
              state = MOVETOWATER2;
            }
         }
         else {
            Serial.println("Failed to read marker");
            printStop();
         }
       break;
       
       case MOVETOWATER2:
         if(rf.receiveMarker(&marker, MARKERID)){
            Serial.print("Received Mark x = "); Serial.print(marker.x);
            Serial.print(" y = "); Serial.print(marker.y);
            Serial.print(" theta = "); Serial.println(marker.theta);
            if (moveToWater2(marker)){
              rf.sendMessage("MoveToBoulder2 Complete");
              state = MOVETOWATER3;
            }
         }
         else {
            Serial.println("Failed to read marker");
            printStop();
         }
       break;
       
       case MOVETOWATER3:
         if(rf.receiveMarker(&marker, MARKERID)){
            Serial.print("Received Mark x = "); Serial.print(marker.x);
            Serial.print(" y = "); Serial.print(marker.y);
            Serial.print(" theta = "); Serial.println(marker.theta);
            if (moveToWater3(marker)){
              delay(1000);
              rf.sendMessage("MoveToBoulder3 Complete");
              state = LOWERSENSOR;
            }
         }
         else {
            Serial.println("Failed to read marker");
            printStop();
         }
       break;
       
       case LOWERSENSOR:
          sendHeight();
          
          state = MEASURETEMP;
       break;
       
       case MEASURETEMP:
          findColor();
          state = COMPLETE;
       break;
   }
       
      
  delay(100);
 
}
 

