#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "point.h"
#define A 125.0
#define DIAMETER 25.36
#define PI 3.1415
#define tolerance 10

int done  = 0; 

//for now keep the period measurement constant
double rollingAvg = 3980;

volatile double pwm_value0 = 0;
double prev_pwm_value0;
volatile double ton0 = 0;
volatile double toff0 = 0;
volatile double prev_time0 = 0;
double rev0;
int flag0 = 0;
double position0;
double offset0;
double degree0;
volatile double pwm_value1 = 0;
double prev_pwm_value1 = 0; 
volatile double ton1 = 0;
volatile double toff1 = 0;
volatile double prev_time1 = 0;
double rev1 = 0;
int flag1 = 0;
double position1;
double offset1;
double degree1;

//pid constants
double P0 = 5;
double I0 = 0;
double D0 = 0;
double P1 = 5;
double I1 = 0;
double D1 = 0;

//pid values
double p0 = 0;
double i0 = 0;
double d0 = 0;
double p1 = 0;
double i1 = 0;
double d1 = 0;

double set0 = 0;
double set1 = 0;

//the value being sent to the motor (-255 to 255)
double motorOutput0 = 0;
double motorOutput1 = 0;

//start by assuming it is safe
int safe = 1;
//variables for the inputs
double inputx = 0;
double inputy = 0;

//only want to check the input every 10 iterations
int count = 0;

void setup() {
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(10, INPUT);
  pinMode(11, INPUT);  
  pinMode(12, INPUT);
  pinMode(13, INPUT);
  //digitalWrite(12, HIGH); 
  //digitalWrite(13, HIGH); 
    
  /* initialize serial                                       */
  Serial.begin(115200);
  //attach interrupt to digital pin 2 and 3
  attachInterrupt(0, rising0, RISING);
  attachInterrupt(1, rising1, RISING);

  //put the carriage in the corner, sometime the first cornering is not enough
  //corner();
  //delay(500);
  corner();
}

void loop() {  
  //Checks that the limit switches are not pushed 
  if(digitalRead(10) == HIGH || digitalRead(11) == HIGH /*|| digitalRead(12) == HIGH || digitalRead(13) == HIGH*/) {
    safe = 0;
  }

  //if the stage has not hit a wall
  if(safe == 1){
      //see if a new setpoint has been sent 
      double power = .1;
      double in = 0;  
      if(Serial.available() > 0 && count == 0){
        //can either send a new setpoint or ask for the current setpoint
        char commandChar = Serial.read();
        if(commandChar == 's' && done == 1){
          //set done to zero when a new position is input
          done = 0;
          
          point_t newPoint = readPoint();
          inputx = newPoint.x;
          inputy = newPoint.y; 
          
          double tempSet0 = XYToS0(inputx, inputy);
          double tempSet1 = XYToS1(inputx, inputy);

          //remove this when the linked queue of next points goes in
          set0 = tempSet0;
          set1 = tempSet1;
          
        }else if(commandChar == 'r'){
          Serial.print(SToX(degree0, degree1));
          Serial.print(",");
          Serial.println(SToY(degree0, degree1));
        }
       }
    
      //handle encoder rollover
      updatePWM0();
      updatePWM1();
      
      i0 = i0 + p0;
      d0 = p0 - (set0 - degree0);
      p0 = set0 - degree0;
    
      i1 = i1 + p1;
      d1 = p1 - (set1 - degree1);
      p1 = set1 - degree1;

      //output the control values to the motors
      runMotor(7, 8, 9, motorOutput0);
      runMotor(4, 5, 6, motorOutput1);

      //when the errors are small enough reports that the setpoint has be reached
      if (abs(p1) < 20 && abs(p0) < 20 && done == 0) {
        Serial.println("d"); 
        done  = 1; 
      }
      
      motorOutput0 = P0 * p0 + I0 * i0 + D0 * d0;
      motorOutput1 = P1 * p1 + I1 * i1 + D1 * d1;
           
  } else {
      Serial.println("hit wall");
      runMotor(7, 8, 9, 0);
      runMotor(4, 5, 6, 0);
  }

  //increase the iteration
  count = (count + 1) % 10;
  delay(10);
}

//pins 1 and 2 are the direction pins and pin3 is the pwm pin
void runMotor(int pin1, int pin2, int pin3, double value){
  if(value > 255){
     value = 255;
  }
  if(value < -255){
     value = -255;
  }

  //keep the motor from trying to run when the value is to low
  if(abs(value) < 100){
    value = 0;
  }
  
  if(value < 0){
    digitalWrite(pin1, HIGH);
    digitalWrite(pin2, LOW);
  }else if (value > 0){
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, HIGH);
  } else{
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, LOW);
  }
  analogWrite(pin3, abs(value));
}

void updatePWM0(){
  double curr_pwm_value0;
  //calculate pwm value 0 to 4095 from the times on and off
  curr_pwm_value0 = ((pwm_value0 * 4098) / (rollingAvg)) - 1;
            
  if (curr_pwm_value0 == 4096) {
    curr_pwm_value0 = 4095;
  }

  //first make sure that prev_pwm_value exists
  //rollover
  if(flag0 == 0){
    rev0 = 0;
    flag0 = 1;
    position0 = curr_pwm_value0;
    offset0 = position0;
  }else if(curr_pwm_value0 - prev_pwm_value0 < -2030) {
    rev0++;
  }else if(curr_pwm_value0 - prev_pwm_value0 > 2030){
    rev0--;
  } 

  prev_pwm_value0 = curr_pwm_value0;

  position0 = curr_pwm_value0 + rev0 * 4095 - offset0;
  degree0 = position0  * 360 / 4095;
}

void updatePWM1(){
  double curr_pwm_value1;
  //calculate pwm value 0 to 4095 from the times on and off
  curr_pwm_value1 = ((pwm_value1 * 4098) / (rollingAvg)) - 1;
            
  if (curr_pwm_value1 == 4096) {
    curr_pwm_value1 = 4095;
  }

  //first make sure that prev_pwm_value exists
  //rollover
  if(flag1 == 0){
    rev1 = 0;
    flag1 = 1;
    position1 = curr_pwm_value1;
    offset1 = position1;
  }else if(curr_pwm_value1 - prev_pwm_value1 < -2030) {
    rev1++;
  }else if(curr_pwm_value1 - prev_pwm_value1 > 2030){
    rev1--;
  } 

  prev_pwm_value1 = curr_pwm_value1;

  position1 = curr_pwm_value1 + rev1 * 4095 - offset1;
  degree1 = position1  * 360 / 4095;
}

void rising0() {
  attachInterrupt(0, falling0, FALLING);
  prev_time0 = micros();
}
 
void falling0() {
  attachInterrupt(0, rising0, RISING);
  pwm_value0 = micros()-prev_time0;
}
void rising1() {
  attachInterrupt(1, falling1, FALLING);
  prev_time1 = micros();
}
 
void falling1() {
  attachInterrupt(1, rising1, RISING);
  pwm_value1 = micros()-prev_time1;
}

void corner(){
    //put the carriage in the corner    
    runMotor(7, 8, 9, 175);
    runMotor(4, 5, 6, -175);
    while(digitalRead(10) != HIGH){
        delay(10);
    }  

    runMotor(7, 8, 9, -175);
    runMotor(4, 5, 6, -175);
    while(digitalRead(11) != HIGH){
        delay(10);
    }  
    
    motorOutput0 = 0;
    motorOutput1 = 0;

    runMotor(7, 8, 9, motorOutput0);
    runMotor(4, 5, 6, motorOutput1); 

    //get it slightly out of the corner
    runMotor(4, 5, 6, 255);
    delay(200);

    runMotor(7, 8, 9, motorOutput0);
    runMotor(4, 5, 6, motorOutput1); 
}

double XYToS0(double x, double y){
  return (x - y)/(DIAMETER * PI) * 360;
}

double XYToS1(double x, double y){
  return (x + y)/(DIAMETER * PI) * 360;
}

double SToX(double s0, double s1){
  return (s0 + s1) * (DIAMETER * PI) / 360 / 2;
}

double SToY(double s0 , double s1){
  return (s1 - s0) * (DIAMETER * PI) / 360 / 2;
}

point_t readPoint() {
  double power = .1;
  point_t ret; 
  double in;
  double inputx = 0;
  double inputy = 0; 
  char curr = Serial.read();
  
  while (curr != '.' && curr != ',') {
    inputx *= 10;
    inputx += curr - 48;
    curr = Serial.read();
  }
  
  while (curr != ',') {
    if(curr != '.'){
      in = (curr - 48);
      inputx += in * power;
      power = power / 10;
    }
    curr = Serial.read();
  }

  power = .1;
  curr = Serial.read();
  while (curr != '.' && curr != '\n') {
    inputy *= 10;
    in = curr - 48;
    inputy += in;
    curr = Serial.read();
  }
  
  while (curr != '\n') {
    if(curr != '.'){
      in = curr - 48;
      inputy += in * power;
      power = power / 10; 
    }
    curr = Serial.read();
  }
  ret.x = inputx;
  ret.y = inputy; 
  return ret;
}

double threshold(double val){
  if(val > 150){
     val = 150;
  }else if(val < -150){
     val = -150;
  }

  return val;
}


