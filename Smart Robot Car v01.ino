// My Blauzahn Pirate Rover Robot Car
// v0.2
// works with the android app

// BASIC MOVEMENTS: Define PINS on L298N for Movement(Remove Jumpers on ENA & ENB to enable PWM)
#define EN1 5        // L298N ENA Enable/speed motor A Right 
#define EN2 6        // L298N ENB Enable/speed motor B Left
#define IN1 2        // L298N #1 in 1 motor A Right
#define IN2 3        // L298N #1 in 2 motor A Right
#define IN3 4        // L298N #1 in 3 motor B Left
#define IN4 7        // L298N #1 in 4 motor B Left

#define LED1 13

// OBSTACLE AVOIDANCE: Servo Library and Object & PIN for Servo
#include <Servo.h>  //servo library
Servo myservo;      // create servo object to control servo
#define SERVOPIN 9  //PMW Signal Servo Pin

// OBSTACLE AVOIDANCE: Define PINS on HC-SR04 Umtrasonic Sensor Module Pins (analog)
int Echo = A4;  
int Trig = A5;

// IR SENSORS: Define PINS
#define IR_FL !digitalRead(10)
#define IR_FR !digitalRead(11)
#define IR_BL !digitalRead(12)
#define IR_BR !digitalRead(8)

boolean Distance_FL;
boolean Distance_FR;
boolean Distance_BL;
boolean Distance_BR;

// BASIC MOVEMENTS: Define Variables
int command;            //Int to store app command state.
int speedCar = 200;     // PWM values 50 - 255, approx 980 Hz
int speed_Coeff = 2;

// OBSTACLE AVOIDANCE: Define Variables
int rightDistance = 0; 
int leftDistance = 0;
int middleDistance = 0;
int distanceLimit;

// ####################################
// ## ++ ## void setup Arduino ## ++ ##
// ####################################

// PINS SETUP
void setup() {
  pinMode(IN1, OUTPUT);   //L298N
  pinMode(IN2, OUTPUT);   //L298N
  pinMode(EN1, OUTPUT);   //L298N
  pinMode(IN3, OUTPUT);   //L298N
  pinMode(IN4, OUTPUT);   //L298N
  pinMode(EN2, OUTPUT);   //L298N
  pinMode(LED1, OUTPUT);    //LED
  myservo.attach(SERVOPIN);  // Obstacle Avoidance: Servo
  Serial.begin(9600);    //HC-06 Bluetooth & IR Sensors Read
  pinMode(IR_FL,INPUT);   //IR Sensors
  pinMode(IR_FR,INPUT);   //IR Sensors
  pinMode(IR_BL,INPUT);   //IR Sensors
  pinMode(IR_BR,INPUT);   //IR Sensors
} 



// ####################################
// ## ++ ## voids for Movement ## ++ ##
// ####################################

void goAhead(){   
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(EN1, speedCar);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(EN2, speedCar);   
}

void goBack(){ 
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(EN1, speedCar);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(EN2, speedCar);   
}

void goRight(){  
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(EN1, speedCar);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(EN2, speedCar);
}

void goLeft(){
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(EN1, speedCar);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(EN2, speedCar);       
}

void goAheadRight(){      
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(EN1, speedCar);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(EN2, speedCar/speed_Coeff);
}

void goAheadLeft(){      
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(EN1, speedCar/speed_Coeff);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(EN2, speedCar); 
}

void goBackRight(){ 
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(EN1, speedCar);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(EN2, speedCar/speed_Coeff);
}

void goBackLeft(){ 
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(EN1, speedCar/speed_Coeff);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(EN2, speedCar);
}

void stopRobot(){  
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(EN1, speedCar);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(EN2, speedCar);  
}

// ####################################
// ## ++ ## Obstacle Avoidance ## ++ ##
// ####################################

//Ultrasonic distance measurement Sub function
int Distance_test() {
  digitalWrite(Trig, LOW);   
  delayMicroseconds(2);
  digitalWrite(Trig, HIGH);  
  delayMicroseconds(20);
  digitalWrite(Trig, LOW);   
  float Fdistance = pulseIn(Echo, HIGH);  
  Fdistance = Fdistance / 58;       
  return (int)Fdistance;
} 


void goObstacleMode() { 
    myservo.write(90);  //set servo position according to scaled value
    delay(500); 
    middleDistance = Distance_test();
    distanceLimit = 40;
 
    if(middleDistance <= distanceLimit) {     
      stopRobot();
      delay(500);                         
      myservo.write(10);          
      delay(1000);      
      rightDistance = Distance_test();
      
      delay(500);
      myservo.write(90);              
      delay(1000);                                                  
      myservo.write(170);              
      delay(1000); 
      leftDistance = Distance_test();
      
      delay(500);
      myservo.write(90);              
      delay(1000);
      if(rightDistance > leftDistance) {
        goRight();
        delay(360);
      }
      else if(rightDistance < leftDistance) {
        goLeft();
        delay(360);
      }
      else if((rightDistance <= distanceLimit) || (leftDistance <= distanceLimit)) {
        goBack();
        delay(180);
      }
      else {
        goAhead();
      }
    }  
    else {
        goAhead();
    }                     
}

// ####################################
// ## ++ ## IR SENSOR          ## ++ ##
// ####################################

void testIR_SENSORS(){

  Distance_FL = IR_FL;
  Distance_FR = IR_FR;
  Distance_BL = IR_BL;
  Distance_BR = IR_BR;


  if((Distance_FL==HIGH) || (Distance_FR==HIGH) || (Distance_BL==HIGH) || (Distance_BR==HIGH)) {
    digitalWrite(LED1, HIGH);                            
  } 
    else { 
      digitalWrite(LED1, LOW);
    }
}




// ###########################
// ## ++ ## void LOOP ## ++ ##
// ###########################

void loop() {
  
  testIR_SENSORS();
  
  if (Serial.available() > 0) {
   command = Serial.read();
    stopRobot();      //Initialize with motors stopped.
    
    
    
    switch (command) {
      
      case 'S':stopRobot();break;
           
      case 'F':goAhead();break;
      case 'B':goBack();break;
      case 'L':goLeft();break;
      case 'R':goRight();break;
      case 'I':goAheadRight();break;
      case 'G':goAheadLeft();break;
      case 'J':goBackRight();break;
      case 'H':goBackLeft();break;

      case 'Z':goObstacleMode();break;


      case '0':speedCar = 180;break;
      case '1':speedCar = 190;break;
      case '2':speedCar = 200;break;
      case '3':speedCar = 205;break;
      case '4':speedCar = 210;break;
      case '5':speedCar = 220;break;
      case '6':speedCar = 230;break;
      case '7':speedCar = 240;break;
      case '8':speedCar = 245;break;
      case '9':speedCar = 250;break;
      case 'q':speedCar = 255;break;
    }
  }
}
