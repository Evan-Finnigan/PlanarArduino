#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define A 125.0
#define DIAMETER 25.36
#define PI 3.1415
#define e 2.71
#define threshold 70

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

//pid constants
double P0 = .5;
double I0 = 0;
double D0 = 0;
double P1 = .5;
double I1 = 0;
double D1 = 0;

//values for the logistic function
double currC = 0;
double currD = 0; 

//variable to keep track of when the move started
double startTime = 0;

//desired values calculated from serial input
point_t setpoint;
point_t startpoint;

//Start the speed all the way negative
int speed = 250;

double pausetime = 10;

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

  if(Serial.available() > 0 && count == 0){
    //can either send a new setpoint or ask for the current setpoint
    char commandChar = Serial.read();
    if(commandChar == 's'){
      point_t newPoint = readPoint();
      
      setpoint.x = XYToS0(newPoint.x, newPoint.y);
      setpoint.y = XYToS1(newPoint.x, newPoint.y);
      double totalTime = newPoint.vx;

      currC = calcC(totalTime);
      currD = calcD(totalTime); 

      Serial.print(newPoint.x);
      Serial.print(","); 
      Serial.print(newPoint.y);
      Serial.print(",");
      Serial.print(totalTime);
      
      startpoint.x = degree0;
      startpoint.y = degree1;
      
      double motorOutput0 = 0;
      double motorOutput1 = 0;
      startTime = millis(); 
      Serial.println("update");
    }else if(commandChar == 'r'){
      Serial.print(SToX(degree0, degree1));
      Serial.print(",");
      Serial.println(SToY(degree0, degree1));
    }
   }
  
  if (!(digitalRead(10) == HIGH || digitalRead(11) == HIGH || digitalRead(12) == HIGH || digitalRead(13) == HIGH)) {
    updatePWM1();
    updatePWM0();

    double timeSinceStart = millis() - startTime;

    //find how far along the device is 
    //double millis0 = inverseLogistic(currC, currD, degree0 / (setpoint.x - startpoint.x));
    //double millis1 = inverseLogistic(currC, currD, degree1 / (setpoint.y - startpoint.y));

    //find how fast the device should go for the next 10 ms
    double speed0 = (setpoint.x - startpoint.x) * (logistic(currC, currD, (timeSinceStart + pausetime)/1000) - logistic(currC, currD, timeSinceStart/1000)) / (pausetime / 1000);
    double speed1 = (setpoint.y - startpoint.y) * (logistic(currC, currD, (timeSinceStart + pausetime)/1000) - logistic(currC, currD, timeSinceStart/1000)) / (pausetime / 1000);
    //Serial.print(speed0);
    //Serial.print(",");
    //Serial.println(speed1);
    //calculate required motor outputs from desired speed, this is now open loop control
    motorOutput0 = solveForInput0(speed0, speed1); 
    motorOutput1 = solveForInput1(speed0, speed1); 

    runMotor(7, 8, 9, motorOutput0);
    runMotor(4, 5, 6, motorOutput1);
    delay(pausetime);
   } else {
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
    runMotor(7, 8, 9, threshold);
    runMotor(4, 5, 6, -threshold);
    while(digitalRead(12) != HIGH){
        delay(10);
    }  

    runMotor(7, 8, 9, -threshold);
    runMotor(4, 5, 6, -threshold);
    while(digitalRead(10) != HIGH){
        delay(10);
    }  
    
    motorOutput0 = 0;
    motorOutput1 = 0;

    runMotor(7, 8, 9, motorOutput0);
    runMotor(4, 5, 6, motorOutput1); 

    //get it slightly out of the corner
    runMotor(4, 5, 6, threshold);
    delay(500);

    runMotor(7, 8, 9, motorOutput0);
    runMotor(4, 5, 6, motorOutput1); 
}


double solveForInput0(double desiredSpeed0, double desiredSpeed1){
  return .606 * desiredSpeed0 + .006 * desiredSpeed1;
}

double solveForInput1(double desiredSpeed0, double desiredSpeed1){
  return .033 * desiredSpeed0 + .807 * desiredSpeed1; 
}

double calcC(double time) {
  return 99 / pow(.99 * 99 / .01, 1 / time); 
}
double calcD(double time) {
  return -log(.01 / (.99 * 99)) / time;
}

//calculate where the platform should be over time
double logistic(double c, double d, double time) {
  return 1 / (1 + c * pow(e, -d * (time - 1))); 
}

double inverseLogistic(double c, double d, double percentDone) {
  return - log(1 / c * (1 / percentDone - 1)) / d + 1;
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

