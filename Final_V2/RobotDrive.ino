///////////////////////////////////////////////////////////////////////////////////
//Robot_Dive.INO
//Written By: Ricardo Tapia Vargas
//
//Team: Monsters Inc.
//
///////////////////////////////////////////////////////////////////////////////////


#include "Romi_Motor_Power.h"
/* Defines pin configuration of robot */
#include "RSLK_Pins.h"
#include "SimpleRSLK.h"

Romi_Motor_Power left_motor;
Romi_Motor_Power right_motor;
int motorSpeed = 0;
int badDriver = 0;
bool  bPressed = false;
int BSend = 1;


void setup() {
  Serial.begin(115200);

  left_motor.begin(MOTOR_L_SLP_PIN,
           MOTOR_L_DIR_PIN,
           MOTOR_L_PWM_PIN);

  right_motor.begin(MOTOR_R_SLP_PIN,
            MOTOR_R_DIR_PIN,
            MOTOR_R_PWM_PIN);

  pinMode(BP_SW_PIN_0,INPUT_PULLUP);
  pinMode(BP_SW_PIN_1,INPUT_PULLUP);
  pinMode(BP_SW_PIN_2,INPUT_PULLUP);
  pinMode(BP_SW_PIN_3,INPUT_PULLUP);
  pinMode(BP_SW_PIN_4,INPUT_PULLUP);
  pinMode(BP_SW_PIN_5,INPUT_PULLUP);

  /* Left button on Launchpad */
  pinMode(LP_LEFT_BTN, INPUT_PULLUP);
  /* Red led in rgb led */
  pinMode(RED_LED,OUTPUT);
 // Serial.println("Wilbur reporting for duty!");
  //Serial.println("Use WASD to move and H to Halt!");

   left_motor.setSpeed(0);
   right_motor.setSpeed(0);

  
}

void loop() {
/* Enable both motors */
  left_motor.enableMotor();
  right_motor.enableMotor();

  /* Set both motors direction to forward */
  //left_motor.directionForward();
  //right_motor.directionForward();

  delay(50);

  /* Set motors speed to 10% of max value */
  //left_motor.setSpeed(20);
  //right_motor.setSpeed(20);

  if (Serial.available() > 0){
    char command = Serial.read();

    
    switch(command){
      case 'W':
          left_motor.setSpeed(40);
          right_motor.setSpeed(40);
          left_motor.directionForward();
          right_motor.directionForward();

      break;

      case 'S':
        left_motor.setSpeed(40);
        right_motor.setSpeed(40);
        left_motor.directionBackward();
        right_motor.directionBackward();


      break;

      case 'A':
        left_motor.setSpeed(40);
        right_motor.setSpeed(40);
        left_motor.directionForward();
        right_motor.directionBackward();

      break;

      case 'D':
          left_motor.setSpeed(40);
          right_motor.setSpeed(40);
          left_motor.directionBackward();
          right_motor.directionForward();


      break;

      case 'H':
        left_motor.setSpeed(0);
        right_motor.setSpeed(0);

      break;
      
      case 'C':
        left_motor.setSpeed(30);
        right_motor.setSpeed(30);
        left_motor.directionForward();
        right_motor.directionBackward();

      break;

      case 'R':
        left_motor.setSpeed(0);
        right_motor.setSpeed(0);

      break;

      case 'P':
      //dance mode
        left_motor.setSpeed(50);
        right_motor.setSpeed(50);
        left_motor.directionForward();
        right_motor.directionBackward();
        delay(2000);
        left_motor.directionBackward();
        right_motor.directionForward();
        delay(2000);
        

      break;

      default:
      left_motor.setSpeed(0);
      right_motor.setSpeed(0);
      break;
    }

    
  }


if (bPressed == false){
  if (digitalRead(BP_SW_PIN_0) == 0 || 
  digitalRead(BP_SW_PIN_1) == 0 || digitalRead(BP_SW_PIN_2) == 0 ||
  digitalRead(BP_SW_PIN_3) == 0 || digitalRead(BP_SW_PIN_4) == 0 ||
  digitalRead(BP_SW_PIN_5) == 0){
    
    bPressed = true;
    //Serial.write("p: ");
    //Serial.write(BSend);
    //Serial.write("\n");
    Serial.print("p: ");
    Serial.println(BSend);
  }
}
else{
  if(digitalRead(BP_SW_PIN_0) == 1 && 
  digitalRead(BP_SW_PIN_1) == 1 && digitalRead(BP_SW_PIN_2) == 1 &&
  digitalRead(BP_SW_PIN_3) == 1 && digitalRead(BP_SW_PIN_4) == 1 &&
  digitalRead(BP_SW_PIN_5) == 1){
    bPressed = false;
  }
}
  
   
 
}
