#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define A 125.0
#define DIAMETER 25.36
#define PI 3.1415
#define threshold 255

typedef struct point {
  double x;
  double y;
  double vx;
  double vy;
}point_t;

//for now keep the period measurement constant
double rollingAvg = 3980;

//current speeds that we want to control to
double currSpeed0 = 0;
double currSpeed1 = 0; 

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
volatile double pwm_value0 = 0;
double prev_pwm_value0 = 0; 
volatile double ton0 = 0;
volatile double toff0 = 0;
volatile double prev_time0 = 0;
double rev0 = 0;
int flag0 = 0;
double position0; 
double oldCurrTime = 0; 
double offset0;
double degree0;
int index = 0; 
int flag = 0;
int voltage = 70; 
unsigned long count = 0; 
double lastSpeed01 = 0;
double lastSpeed02 = 0; 
double lastSpeed11 = 0;
double lastSpeed12 = 0; 

int testOutput = 0;

int motorOutput0 = 20;
int motorOutput1 = 20;

int counter = 0;
int k = 0;
double testValue = 0;

//pid constants
double P0 = .4;
double I0 = 0;
double D0 = 0;
double P1 = .4;
double I1 = 0;
double D1 = 0;

//pid values
double p0 = 0;
double i0 = 0;
double d0 = 0;
double p1 = 0;
double i1 = 0;
double d1 = 0;

//variable to keep track of when the move started
double startTime = 0;

//desired values calculated from serial input
point_t setpoint;
point_t startpoint;

//Start the speed all the way negative
int speed = 250;

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
  digitalWrite(12, HIGH); 
  digitalWrite(13, HIGH); 
    
  /* initialize serial                                       */
  Serial.begin(115200);
  //attach interrupt to digital pin 3
  attachInterrupt(1, rising1, RISING);
  attachInterrupt(0, rising0, RISING);
  Serial.print("START");

  corner();
  corner();
}

void loop() { 
  if (!(digitalRead(10) == HIGH || digitalRead(11) == HIGH || digitalRead(12) == HIGH || digitalRead(13) == HIGH)) {
    double currTime = micros();
    double oldDegree1 = degree1; 
    double oldDegree0 = degree0; 
    updatePWM1();
    updatePWM0();
    double speed1 = (degree1 - oldDegree1) * 1000000 / (currTime - oldCurrTime);
    double speed0 = (degree0 - oldDegree0) * 1000000 / (currTime - oldCurrTime);
    double filteredSpeed1 = (speed1 + lastSpeed11 + lastSpeed12) / 3; 
    lastSpeed12 = lastSpeed11;
    lastSpeed11 = filteredSpeed1;
    double filteredSpeed0 = (speed0 + lastSpeed01 + lastSpeed02) / 3; 
    lastSpeed02 = lastSpeed01;
    lastSpeed01 = filteredSpeed0; 
    count = (count + 1) % 10; 
    oldCurrTime = currTime; 
       
    if(testValue < 255){
      if(k == 0){
        //count and both speeds reported in a python list
        Serial.print("[");
        Serial.print(currTime);
        Serial.print(",");
        Serial.print(testValue);
        Serial.print(",");
        Serial.print(filteredSpeed0);
        Serial.print(",");
        Serial.print(filteredSpeed1);
        Serial.println("],");
      }
  
      k = (k + 1) % 10;

      //change the last values in these to get different directions
      runMotor(4, 5, 6, testValue);
      runMotor(7, 8, 9, 0);
      
      testValue += 1;
      
      delay(10);
   } 
  }else {
     runMotor(7,8,9,0);
     runMotor(4,5,6,0); 
  }
}

//pins 1 and 2 are the direction pins and pin3 is the pwm pin
void runMotor(int pin1, int pin2, int pin3, double value){
  if(value > 255){
     value = 255;
  }
  if(value < -255){
     value = -255;
  }
  if(abs(value) < 40){
    value = 0; 
  }
  if(value < 0){
    digitalWrite(pin1, HIGH);
    digitalWrite(pin2, LOW);
  }else {
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, HIGH);
  }
  analogWrite(pin3, abs(value));
}

void updatePWM0() {
  double curr_pwm_value0;
  //calculate pwm value 0 to 4095 from the times on and off
  curr_pwm_value0 = ((pwm_value0 * 4095) / (rollingAvg)) - 1;

  //Serial.println(curr_pwm_value1); 
  if (curr_pwm_value0 == 4096) {
    curr_pwm_value0 = 4095;
  }

  //first make sure that prev_pwm_value exists
  //rollover
  if(flag0 == 0 && pwm_value0 != 0){
    rev0 = 0;
    flag0 = 1;
    position0 = curr_pwm_value0;
    offset0 = position0;
  }else if(curr_pwm_value0 - prev_pwm_value0 < -2030) {
    offset0 = 0;
    rev0 = position0;
  }else if(curr_pwm_value0 - prev_pwm_value0 > 2030){
    offset0 = curr_pwm_value0;
    rev0 = position0;
  } 

  prev_pwm_value0 = curr_pwm_value0;
  //Serial.println(prev_pwm_value1); 
  position0 = curr_pwm_value0 + rev0 - offset0;
  degree0 = position0 * 360 / 4095;
}
void updatePWM1() {
  double curr_pwm_value1;
  //calculate pwm value 0 to 4095 from the times on and off
  curr_pwm_value1 = ((pwm_value1 * 4095) / (rollingAvg)) - 1;

  //Serial.println(curr_pwm_value1); 
  if (curr_pwm_value1 == 4096) {
    curr_pwm_value1 = 4095;
  }

  //first make sure that prev_pwm_value exists
  //rollover
  if(flag1 == 0 && pwm_value1 != 0){
    rev1 = 0;
    flag1 = 1;
    position1 = curr_pwm_value1;
    offset1 = position1;
  }else if(curr_pwm_value1 - prev_pwm_value1 < -2030) {
    offset1 = 0;
    rev1 = position1;
  }else if(curr_pwm_value1 - prev_pwm_value1 > 2030){
    offset1 = curr_pwm_value1;
    rev1 = position1;
  } 

  prev_pwm_value1 = curr_pwm_value1;
  position1 = curr_pwm_value1 + rev1 - offset1;
  degree1 = position1 * 360 / 4095;
}

void rising1() {
  attachInterrupt(1, falling1, FALLING);
  prev_time1 = micros();
}
 
void falling1() {
  attachInterrupt(1, rising1, RISING);
  pwm_value1 = micros()-prev_time1;
}

void rising0() {
  attachInterrupt(0, falling0, FALLING);
  prev_time0 = micros();
}
 
void falling0() {
  attachInterrupt(0, rising0, RISING);
  pwm_value0 = micros()-prev_time0;
}

void corner(){
    //put the carriage in the corner    
    runMotor(7, 8, 9, 70);
    runMotor(4, 5, 6, -70);
    while(digitalRead(12) != HIGH){
        delay(10);
    }  

    runMotor(7, 8, 9, -70);
    runMotor(4, 5, 6, -70);
    while(digitalRead(10) != HIGH){
        delay(10);
    }  
    
    motorOutput0 = 0;
    motorOutput1 = 0;

    runMotor(7, 8, 9, motorOutput0);
    runMotor(4, 5, 6, motorOutput1); 

    //get it slightly out of the corner
    runMotor(4, 5, 6, 70);
    delay(200);

    runMotor(7, 8, 9, motorOutput0);
    runMotor(4, 5, 6, motorOutput1); 
}


double solveForInput0(double desiredSpeed){
  return (5 * desiredSpeed + 167.233) / 28.2959;
}

double solveForInput1(double desiredSpeed){
  return (5 * desiredSpeed + 176.3784) / 34.0838;
}

double trapezoid(double time, double ramp) {
  double toReturn = time / ramp;
  if (toReturn > 1) {
    toReturn = 1;
  }
  return toReturn;
}

//clamps a number between zero and one
double clamp(double val){
  if(val > 1){
    return 1.0;
  }else if(val < 0){
    return 0.0;
  } else {
    return val;
  }
}

point_t readPoint() {
  double power = .1;
  point_t ret; 
  double in;
  double inputx = 0;
  double inputy = 0; 
  double inputvx = 0;
  double inputvy = 0;
  char sign;
  char curr = Serial.read();
            //first value
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

            //second number
            power = .1;
            curr = Serial.read();
            while (curr != '.' && curr != ',') {
              inputy *= 10;
              inputy += curr - 48;
              curr = Serial.read();
            }
            
            while (curr != ',') {
              if(curr != '.'){
                in = (curr - 48);
                inputy += in * power;
                power = power / 10;
              }
              curr = Serial.read();
            }

            //third number
            power = .1;
            sign = Serial.read();
            curr = Serial.read();
            while (curr != '.' && curr != ',') {
              inputvx *= 10;
              inputvx += curr - 48;
              curr = Serial.read();
            }
            
            while (curr != ',') {
              if(curr != '.'){
                in = (curr - 48);
                inputvx += in * power;
                power = power / 10;
              }
              curr = Serial.read();
            }

            if(sign == '-'){
              inputvx = -inputvx;
            }

            //fourth number
            power = .1;
            sign = Serial.read();
            curr = Serial.read();
            while (curr != '.' && curr != '\n') {
              inputvy *= 10;
              in = curr - 48;
              inputvy += in;
              curr = Serial.read();
            }
            
            while (curr != '\n') {
              if(curr != '.'){
                in = curr - 48;
                inputvy += in * power;
                power = power / 10; 
              }
              curr = Serial.read();
            }
            
            if(sign == '-'){
              inputvy = -inputvy;
            }
            
            ret.x = inputx;
            ret.y = inputy; 
            ret.vx = inputvx;
            ret.vy = inputvy;
            
            return ret;
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

