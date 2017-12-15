#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define A 125.0
#define DIAMETER 25.36
#define PI 3.1415
#define threshold 70

//past speed values for filtering
double  outs0[] = {0,0,0};
double  ins0[] = {0,0,0};
double  outs1[] = {0,0,0};
double  ins1[] = {0,0,0};

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
double P0 = .15;
double I0 = 0;
double D0 = -.1;
double P1 = .15;
double I1 = 0;
double D1 = -.1;

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

  if(Serial.available() > 0 && count == 0){
    //can either send a new setpoint or ask for the current setpoint
    char commandChar = Serial.read();
    if(commandChar == 's'){
      point_t newPoint = readPoint();
      
      setpoint.x = XYToS0(newPoint.x, newPoint.y);
      setpoint.y = XYToS1(newPoint.x, newPoint.y);
      setpoint.vx = XYToS0(newPoint.vx, newPoint.vy);
      setpoint.vy = XYToS1(newPoint.vx, newPoint.vy);

//      Serial.print(newPoint.x);
//      Serial.print(","); 
//      Serial.print(newPoint.y);
//      Serial.print(",");
//      Serial.print(newPoint.vx);
//      Serial.print(",");
//      Serial.println(newPoint.vy);

      startpoint.x = degree0;
      startpoint.y = degree1;
      
      double motorOutput0 = 0;
      double motorOutput1 = 0;
      startTime = millis(); 
    }else if(commandChar == 'r'){
      Serial.print(SToX(degree0, degree1));
      Serial.print(",");
      Serial.println(SToY(degree0, degree1));
    }
   }
  
  if (!(digitalRead(10) == HIGH || digitalRead(11) == HIGH || digitalRead(12) == HIGH || digitalRead(13) == HIGH)) {
    double currTime = micros();
    double oldDegree1 = degree1; 
    double oldDegree0 = degree0; 
    updatePWM1();
    updatePWM0();
    double speed1 = (degree1 - oldDegree1) * 1000000 / (currTime - oldCurrTime);
    double speed0 = (degree0 - oldDegree0) * 1000000 / (currTime - oldCurrTime);
    double filteredSpeed1 = butterworth(speed1, ins1, outs1);
    double filteredSpeed0 = butterworth(speed0, ins0, outs0); 
    count = (count + 1) % 10; 
    oldCurrTime = currTime; 

    double timeSinceStart = millis() - startTime; 

    currSpeed0 = setpoint.vx * trapezoid(timeSinceStart, 300); 

    i0 = i0 + p0;
    d0 = p0 - (currSpeed0 - p0);
    p0 = currSpeed0 - filteredSpeed0;

    currSpeed1 = setpoint.vy * trapezoid(timeSinceStart, 300); 
  
    i1 = i1 + p1;
    d1 = p1 - (currSpeed1 - p1);
    p1 = currSpeed1 - filteredSpeed1;

    motorOutput0 = P0 * p0 + I0 * i0 + D0 * d0 + solveForInput(currSpeed0);
    motorOutput1 = P1 * p1 + I1 * i1 + D1 * d1 + solveForInput(currSpeed1); 

    //let position control take over at the very end of the move
    if (abs(degree0 - setpoint.x) < 80) {
      motorOutput0 = setpoint.x - degree0;
    }
    if (abs(degree1 - setpoint.y) < 80) {
      motorOutput1 = setpoint.y - degree1; 
    }

    runMotor(7, 8, 9, motorOutput0);
    runMotor(4, 5, 6, motorOutput1);

    Serial.print(filteredSpeed1);
     
       
    delay(10); 
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
  if(abs(value) < 40) {
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
double solveForInput(double speed){
  double speedAbs = abs(speed); 
  double toReturn = - 1.0 / .0145 * log((speedAbs - 1223.0) / (331.9 - 1223.0)) + 50.0; 
  if (speed < 0) {
    toReturn = -toReturn;
  }
  return toReturn;
}



//attempt at butterworth filtering
double butterworth(double newin, double * ins, double * outs) {
    double newout = .0134 * newin + .0267 * ins[1] + .0134 * ins[0] - (-1.6475 * outs[1] + .7009 * outs[0]);
    ins[0] = ins[1];
    ins[1] = newin;
    outs[0] = outs[1];
    outs[1] = newout;
    return newout; 
}


