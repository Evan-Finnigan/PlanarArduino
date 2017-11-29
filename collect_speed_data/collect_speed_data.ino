//ratio for how many degrees a count corresponds to
#define COUNTS_TO_DEGREES 75926.3008706
 
//last time a rising or falling edge was seen
volatile long lastTime;
volatile long currentTime;

//elapsed time between the last two rising or falling events
//on the interrupt pins
volatile long elapsedTime;

//will be either -1 (reverse direction) or +1 (forward direction)
volatile double direction;

//array for median filtering
double previousValues[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
//counter to tell where to put the next value
int medianArrayLocation = 0;

int count = 0;

int k = 0;

double motorOutput0 = 0;
double motorOutput1 = 0;

void setup() {
  //pin setup for motor pins
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);

  //initialize serial connection
  Serial.begin(115200);

  //attach interrupt for encoder outputA
  attachInterrupt(digitalPinToInterrupt(2), ISRARising, RISING);

  //attach interrupt for encoder outputB
  attachInterrupt(digitalPinToInterrupt(3), ISRBRising, RISING);

  direction = 1;

  corner();
}

void loop() {
  //get the current motor velocity, median filtered
  double velocity = getSpeed();

  if(!(digitalRead(10) == HIGH || digitalRead(11) == HIGH || digitalRead(12) == HIGH || digitalRead(13) == HIGH)){
    
    if(count < 255){
      if(k == 0){
        Serial.print(medianFilter(velocity));
        Serial.println(",");
      }
  
      k = (k + 1) % 10;
    
      runMotor(4, 5, 6, count);
      count += 20;
      delay(10);
    }else{
      runMotor(4, 5, 6, 0);
    }
  }
}

//the following 4 functions handle counting ticks of the encoder
//which will be used to directly calculate the velocity
void ISRARising(){
  currentTime = micros();
  elapsedTime = currentTime - lastTime;
  lastTime = currentTime;

  //this logic can decide whether or not signal A is leading signal B
  if(digitalRead(3) == HIGH){
    direction = -1.0;
  }else{
    direction = 1.0;
  }
  
  attachInterrupt(digitalPinToInterrupt(2), ISRAFalling, FALLING);
}

void ISRBRising(){
  currentTime = micros();
  elapsedTime = currentTime - lastTime;
  lastTime = currentTime;
  attachInterrupt(digitalPinToInterrupt(3), ISRBFalling, FALLING);
}

void ISRAFalling(){
  currentTime = micros();
  elapsedTime = currentTime - lastTime;
  lastTime = currentTime;
  attachInterrupt(digitalPinToInterrupt(2), ISRARising, RISING);
}

void ISRBFalling(){
  currentTime = micros();
  elapsedTime = currentTime - lastTime;
  lastTime = currentTime;
  attachInterrupt(digitalPinToInterrupt(3), ISRBRising, RISING);
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

//TODO: try to find a way to make the speed zero when no new updates are seen
double getSpeed(){
  return direction / (double)elapsedTime * COUNTS_TO_DEGREES;
}

//TODO: find a way to filter over all data?
//median filter for use filtering speed data
//this uses the global variable array previousValues
double medianFilter(double val){
  //make sure the value is not nan
  if(isnan(val)){
    return 0;
  }
  //the newly input values are assigned to slots in the array
  //in a rotating fashion
  previousValues[medianArrayLocation] = val;

  //the median is just the median of the last 10 values  
  medianArrayLocation = (medianArrayLocation + 1) % 10;
  return (previousValues[0] + 
          previousValues[1] + 
          previousValues[2] + 
          previousValues[3] + 
          previousValues[4] +
          previousValues[5] + 
          previousValues[6] + 
          previousValues[7] + 
          previousValues[8] + 
          previousValues[9]) / 10.0;
}

void corner(){
    //put the carriage in the corner    
    runMotor(7, 8, 9, 100);
    runMotor(4, 5, 6, -100);
    while(digitalRead(12) != HIGH){
        delay(10);

        //safety modificiation, if any of the switches are pressed, 
        //don't allow either motor to move
        if(digitalRead(10) == HIGH || digitalRead(11) == HIGH || digitalRead(13) == HIGH){
          runMotor(7, 8, 9, 0);
          runMotor(4, 5, 6, 0);
        }
    }  

    runMotor(7, 8, 9, -100);
    runMotor(4, 5, 6, -100);
    while(digitalRead(10) != HIGH){
        delay(10);

         //safety modificiation, if any of the switches are pressed, 
        //don't allow either motor to move
        if(digitalRead(11) == HIGH || digitalRead(13) == HIGH){
          runMotor(7, 8, 9, 0);
          runMotor(4, 5, 6, 0);
        }
    }  
    
    motorOutput0 = 0;
    motorOutput1 = 0;

    runMotor(7, 8, 9, motorOutput0);
    runMotor(4, 5, 6, motorOutput1); 

    //get it to the middle
    runMotor(4, 5, 6, 100);
    delay(1000);

    runMotor(7, 8, 9, motorOutput0);
    runMotor(4, 5, 6, motorOutput1); 
}

