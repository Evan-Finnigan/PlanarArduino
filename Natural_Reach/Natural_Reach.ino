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
  Serial.print("hi"); 
  goToCenter();
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
    }else if(commandChar == 'c'){
      goToCenter();
    }
   }

 if (!(digitalRead(10) == HIGH || digitalRead(11) == HIGH || digitalRead(12) == HIGH || digitalRead(13) == HIGH)) {
    double timeSinceStart = millis() - startTime;
    updatePWM1();
    updatePWM0();


    //find how fast the device should go for the next 10 ms
    double speed0 = (setpoint.x - startpoint.x) * (logistic(currC, currD, (timeSinceStart + pausetime)) - logistic(currC, currD, timeSinceStart)) / (pausetime / 1000);
    double speed1 = (setpoint.y - startpoint.y) * (logistic(currC, currD, (timeSinceStart + pausetime)) - logistic(currC, currD, timeSinceStart)) / (pausetime / 1000);
    
    //calculate required motor outputs from desired speed, this is now open loop control
    motorOutput0 = speedToPWM0(speed0); 
    motorOutput1 = speedToPWM0(speed1); 

//    Serial.print(speed0);
//    Serial.print(",");
//    Serial.print(speed1);
//    Serial.print(","); 
//    Serial.print(motorOutput0);
//    Serial.print(",");
//    Serial.println(motorOutput1);

    runMotor(7, 8, 9, motorOutput0, 255);
    runMotor(4, 5, 6, motorOutput1, 255);
    delay(pausetime);
   } else {
     runMotor(7,8,9,0, 255);
     runMotor(4,5,6,0, 255); 
   }
  }

//pins 1 and 2 are the direction pins and pin3 is the pwm pin
void runMotor(int pin1, int pin2, int pin3, double value, double throttle){
  if(value > throttle){
     value = throttle;
  }
  if(value < -throttle){
     value = -throttle;
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
    runMotor(7, 8, 9, threshold, 55);
    runMotor(4, 5, 6, -threshold, 55);
    while(digitalRead(12) != HIGH){
        delay(10);
    }  

    runMotor(7, 8, 9, -threshold, 55);
    runMotor(4, 5, 6, -threshold, 55);
    while(digitalRead(10) != HIGH){
        delay(10);
    }  
    
    motorOutput0 = 0;
    motorOutput1 = 0;

    runMotor(7, 8, 9, motorOutput0, 55);
    runMotor(4, 5, 6, motorOutput1, 55); 

    //get it slightly out of the corner
    runMotor(4, 5, 6, threshold, 55);
    delay(500);

    runMotor(7, 8, 9, motorOutput0, 55);
    runMotor(4, 5, 6, motorOutput1, 55); 
}

double speedToPWM0(double speed) {
  double speedAbs = abs(speed); 
  double toReturn = - 1.0 / .0145 * log((speedAbs - 1223.0) / (331.9 - 1223.0)) + 50.0; 
  if (speed < 0) {
    toReturn = -toReturn;
  }
  return toReturn;
}

double speedToPWM1(double speed) {
  double speedAbs = abs(speed);
  double toReturn =  -1.0 / .0127 * log((speedAbs - 1238.0) / (276.9 - 1238.0)) + 40.0; 
  if (speed < 0) {
    toReturn = -toReturn; 
  }
  return toReturn; 
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
            curr = Serial.read();
            while (curr != '.' && curr != '\n') {
              inputvx *= 10;
              inputvx += curr - 48;
              curr = Serial.read();
            }
            
            while (curr != '\n') {
              if(curr != '.'){
                in = (curr - 48);
                inputvx += in * power;
                power = power / 10;
              }
              curr = Serial.read();
            }
            
            ret.x = inputx;
            ret.y = inputy; 
            ret.vx = inputvx;
            ret.vy = inputvy;

            Serial.print(ret.x);
            Serial.print(",");
            Serial.print(ret.y);
            Serial.print(",");
            Serial.print(ret.vx);
            Serial.print(",");
            
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

void goToCenter(){
  int safe = 1;

  //this is the defined center
  double set0 = XYToS0(115, 95);
  double set1 = XYToS1(115, 95);

  //PWM values 
  double P0 = 5;
  double P1 = 5;

  //error values 
  updatePWM0();
  updatePWM1();
  
  double p0 = set0 - degree0;
  double p1 = set1 - degree1;

  

  while(p0 > 3 || p1 > 3){
    //Checks that the limit switches are not pushed and that the curren limit is not exceeded (indicating an unsafe torque)
    if(digitalRead(10) == HIGH || digitalRead(11) == HIGH || digitalRead(12) == HIGH || digitalRead(13) == HIGH) {
      safe = 0;
      /*Serial.print(digitalRead(10));
      Serial.print(","); 
      Serial.print(digitalRead(11)); 
      Serial.print(",");
      Serial.print(digitalRead(12)); 
      Serial.print(",");
      Serial.println(digitalRead(13));*/ 
    }
    if(safe == 1){
      //Serial.println("hello"); 
      //rollover
      updatePWM0();
      updatePWM1();
      
      p0 = set0 - degree0;
    
      p1 = set1 - degree1;

      motorOutput0 = P0 * p0;
      motorOutput1 = P1 * p1;

      
      Serial.print(SToX(degree0, degree1));
      Serial.print(",");
      Serial.println(SToY(degree0, degree1)); 
      //output the control values to the motors
      runMotor(7, 8, 9, motorOutput0, 55);
      runMotor(4, 5, 6, motorOutput1, 55); 
    } else {
      runMotor(7, 8, 9, 0, 55);
      runMotor(4, 5, 6, 0, 55);
    }
  }
  Serial.println("done");
}
