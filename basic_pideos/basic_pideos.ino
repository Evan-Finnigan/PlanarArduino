#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define A 125.0
#define DIAMETER 25.36
#define tolerance 10
#define threshold 60

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
double curLim = 35;
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
double P0 = 2;
double I0 = 0;
double D0 = 0;
double P1 = 2;
double I1 = 0;
double D1 = 0;

//used for the catchup terms
double totalDistance0 = 0;
double totalDistance1 = 0;

//pid values
double p0 = 0;
double i0 = 0;
double d0 = 0;
double p1 = 0;
double i1 = 0;
double d1 = 0;


//get the carriage out of the corner, if it ever hits the walls it will stop
double set0 = 0;
double set1 = 0;

//the value being sent to the motor (-threshold to threshold)
double motorOutput0 = 0;
double motorOutput1 = 0;

//start by assuming it is safe
int safe = 1;

//variables for the inputs
double inputx = 0;
double inputy = 0;

//only want to check the input every 10 iterations
int count = 0;

typedef struct point {
  double x;
  double y;
}point_t;

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
  attachInterrupt(digitalPinToInterrupt(18), rising0, RISING);
  attachInterrupt(digitalPinToInterrupt(19), rising1, RISING);

  //put the carriage in the corner
  corner();
  

}

void loop() {   
  //Checks that the limit switches are not pushed and that the curren limit is not exceeded (indicating an unsafe torque)
  if(digitalRead(10) == HIGH || digitalRead(11) == HIGH || digitalRead(12) == HIGH || digitalRead(13) == HIGH
      /*||  analogRead(0) > curLim ||*/ /*|| analogRead(1) > curLim*/) {
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
          //see if a new setpoint has been sent 
          double power = .1;
          double in = 0;  
          if(Serial.available() > 0 && count == 0){
            //can either send a new setpoint or ask for the current setpoint
            char commandChar = Serial.read();
            if(commandChar == 's'){
              point_t newPoint = readPoint();
              inputx = newPoint.x;
              inputy = newPoint.y; 

              double oldSet0 = set0;
              double oldSet1 = set1;
              
              set0 = XYToS0(inputx, inputy);
              set1 = XYToS1(inputx, inputy);

              //set the total distances used in the catchup part
              totalDistance0 = set0 - oldSet0;
              totalDistance1 = set1 - oldSet1;
              
              double motorOutput0 = 0;
              double motorOutput1 = 0;
            }else if(commandChar == 'r'){
              Serial.print(SToX(degree0, degree1));
              Serial.print(",");
              Serial.println(SToY(degree0, degree1));
            }
           }
        
          //rollover
          updatePWM0();
          updatePWM1();
          
          i0 = i0 + p0;
          d0 = p0 - (set0 - degree0);
          p0 = set0 - degree0;
        
          i1 = i1 + p1;
          d1 = p1 - (set1 - degree1);
          p1 = set1 - degree1;

          motorOutput0 = P0 * p0 + I0 * i0 + D0 * d0;
          motorOutput1 = P1 * p1 + I1 * i1 + D1 * d1;
          
          //output the control values to the motors
          runMotor(7, 8, 9, motorOutput0);
          runMotor(4, 5, 6, motorOutput1);

          //test encoder
          //runMotor(7, 8, 9, 0);
          //runMotor(4, 5, 6, 0);

          if(count == 0){
//            Serial.print(SToX(degree0, degree1 ));
//            Serial.print(",");
//            Serial.println(SToY(degree0, degree1)); 
            //pin2
            Serial.print(degree0);
            Serial.print(",");
            //pin3
            Serial.println(degree1); 
          }  
  } else {
          runMotor(7, 8, 9, 0);
          runMotor(4, 5, 6, 0);
  }

  //increase the iteration
  count = (count + 1) % 10;
  delay(10);
}

//pins 1 and 2 are the direction pins and pin3 is the pwm pin
void runMotor(int pin1, int pin2, int pin3, double value){
  if(value > threshold){
     value = threshold;
  }
  if(value < -threshold){
     value = -threshold;
  }

  if(abs(value) < 40) {
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
  attachInterrupt(digitalPinToInterrupt(18), falling0, FALLING);
  prev_time0 = micros();
}
 
void falling0() {
  attachInterrupt(digitalPinToInterrupt(18), rising0, RISING);
  pwm_value0 = micros()-prev_time0;
}
void rising1() {
  attachInterrupt(digitalPinToInterrupt(19), falling1, FALLING);
  prev_time1 = micros();
}
 
void falling1() {
  attachInterrupt(digitalPinToInterrupt(19), rising1, RISING);
  pwm_value1 = micros()-prev_time1;
}

void corner(){
    
    //put the carriage in the corner
    runMotor(7, 8, 9, threshold);
    runMotor(4, 5, 6, -threshold);
    while(digitalRead(11) != HIGH){
        delay(10);
    }

    Serial.print("First pressed");

    runMotor(7, 8, 9, -threshold);
    runMotor(4, 5, 6, -threshold);
    while(digitalRead(13) != HIGH){
        delay(10);
    }  
    motorOutput0 = 0;
    motorOutput1 = 0;

    runMotor(7, 8, 9, motorOutput0);
    runMotor(4, 5, 6, motorOutput1); 

    //get it slightly out of the corner
    runMotor(4, 5, 6, threshold);
    delay(250);

    runMotor(7, 8, 9, motorOutput0);
    runMotor(4, 5, 6, motorOutput1);
    Serial.print("End Corner");  
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
