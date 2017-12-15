#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define threshold 70

//for now keep the period measurement constant
double rollingAvg = 3980;

volatile double pwm_value1 = 0;
double prev_pwm_value1 = 0; 
volatile double ton1 = 0;
volatile double toff1 = 0;
volatile double prev_time1 = 0;
double rev1 = 0;
int flag1 = 0;
double position1; 
double oldCurrTime = 0; 
double offset1;
double degree1;
unsigned long time = 0;
int index = 0; 
int flag = 0;
int voltage = 70; 
unsigned long count = 0; 
double  outs0[] = {0,0,0};
double  ins0[] = {0,0,0};
double  outs1[] = {0,0,0};
double  ins1[] = {0,0,0};

int testOutput = 0;

int motorOutput0 = 20;
int motorOutput1 = 20;

int counter = 0;

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
  attachInterrupt(0, rising1, RISING);
  Serial.print("START");

  corner();
}

void loop() { 
  double oldDegree1 = degree1; 
  updatePWM1();
  if (!(digitalRead(10) == HIGH || digitalRead(11) == HIGH || digitalRead(12) == HIGH || digitalRead(13) == HIGH && testOutput <= 255)) {
    double currTime = micros();
    runMotor(7,8,9,-threshold); 
    double speed = (degree1 - oldDegree1) * 1000000.0 / (currTime - oldCurrTime);
    double filteredSpeed = butterworth(speed, ins1, outs1);  
    count++; 
    oldCurrTime = currTime; 

//    if (counter %10 == 0) {
//      Serial.print(degree1); 
//      Serial.print(" "); 
//    }

    counter = (counter + 1) % 10;
    Serial.println(filteredSpeed);
    
    delay(10); 
   } else {
     runMotor(7,8,9,0); 
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
  if(value < 0){
    digitalWrite(pin1, HIGH);
    digitalWrite(pin2, LOW);
  }else {
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, HIGH);
  }
  analogWrite(pin3, abs(value));
}
void updatePWM1() {
  double curr_pwm_value1;
  //calculate pwm value 0 to 4095 from the times on and off
  curr_pwm_value1 = ((pwm_value1 * 4095) / (rollingAvg)) - 1;
            
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
    rev1 = position1;
    offset1 = 0;
  }else if(curr_pwm_value1 - prev_pwm_value1 > 2030){
    rev1 = position1;
    offset1 = curr_pwm_value1;
  } 

  prev_pwm_value1 = curr_pwm_value1;

  position1 = curr_pwm_value1 + rev1 - offset1;
  degree1 = position1  * 360 / 4095;
}

void rising1() {
  attachInterrupt(0, falling1, FALLING);
  prev_time1 = micros();
}
 
void falling1() {
  attachInterrupt(0, rising1, RISING);
  pwm_value1 = micros()-prev_time1;
}

void corner(){
    //put the carriage in the corner    
    runMotor(7, 8, 9, threshold);
    runMotor(4, 5, 6, -threshold);
    while(digitalRead(12) != HIGH){
        delay(10);
    }  

    runMotor(7, 8, 9, threshold);
    runMotor(4, 5, 6, threshold);
    while(digitalRead(11) != HIGH){
        delay(10);
    }  
    
    motorOutput0 = 0;
    motorOutput1 = 0;

    runMotor(7, 8, 9, motorOutput0);
    runMotor(4, 5, 6, motorOutput1); 

    //get it slightly out of the corner
    runMotor(7, 8, 9, -threshold);
    delay(500);

    runMotor(7, 8, 9, motorOutput0);
    runMotor(4, 5, 6, motorOutput1); 
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



