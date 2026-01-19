#include <IRremote.h>
#include <Servo.h>
#include <NewPing.h>

// State machines

enum RobotMode{control_mode, auto_mode};

enum ControlMode{control_idle, control_forward, control_backward, control_right, control_left};

enum AutoMode{auto_forward, auto_backward, auto_check_left, auto_check_right, auto_recenter_right, auto_recenter_left, auto_decide};

RobotMode mode = control_mode;

ControlMode ctrlState = control_idle;

AutoMode autoState = auto_forward;

// Pins
int left_up = 8;    //pin 8 of arduino to pin 7 of l293d 
int left_up2 = 9;   //pin 9 of arduino to pin 2 of l293d backward pin
int right_up = 11;   //pin 11 of arduino to pin 10 of l293d
int right_up2 = 10;  //pin 10 of arduino to pin 15 of l293d backward pin
//servo pin 2 ultrasonic sensor 12 13
//Mode button is B946FF00

int left_down = 6; 
int left_down2 = 5;
int right_down = 4;
int right_down2 = 3;

int RECV_PIN = 7;  //pin 7 of arduino to data pin of ir receiver

int led_control = A4;
int led_auto = A5;

int trig_pin = 12;
int echo_pin = A0;
int max_dist = 30;

// Global Variables
unsigned long code = 0;

unsigned long stateStartTime = 0;
unsigned long lastToggleTime = 0;

int frontDist = 0;
int leftDist = 0;
int rightDist = 0;

NewPing mySensor(trig_pin, echo_pin, max_dist);

void handleIR(){
  if(!IrReceiver.decode()) return;
  //Serial.println(IrReceiver.decodedIRData.decodedRawData, HEX);
  code = IrReceiver.decodedIRData.decodedRawData;
  IrReceiver.resume();  // Receive the next value

  if(code == 0xB946FF00 && millis() - lastToggleTime > 800){
    if(mode == control_mode){
      mode = auto_mode;
    } else {
      mode = control_mode;
    }

    ctrlState = control_idle;
    autoState = auto_forward;
    lastToggleTime = millis();
    code = 0;
    return;
  }

  if(mode != control_mode){
    code = 0;
    return;
  }

  //code = 0;
}

void stopMotors(){
  digitalWrite(left_up, LOW);
  digitalWrite(left_up2, LOW);
  digitalWrite(right_up, LOW);
  digitalWrite(right_up2, LOW);

  digitalWrite(left_down, LOW);
  digitalWrite(left_down2, LOW);
  digitalWrite(right_down, LOW);
  digitalWrite(right_down2, LOW);
}

void turnRight(){
  digitalWrite(left_up, LOW);
  digitalWrite(left_up2, HIGH);
  digitalWrite(right_up, LOW);
  digitalWrite(right_up2, LOW);

  digitalWrite(left_down, LOW);
  digitalWrite(left_down2, HIGH);
  digitalWrite(right_down, LOW);
  digitalWrite(right_down2, LOW);
}

void backRight(){
  digitalWrite(left_up, HIGH);
  digitalWrite(left_up2, LOW);
  digitalWrite(right_up, LOW);
  digitalWrite(right_up2, LOW);

  digitalWrite(left_down, HIGH);
  digitalWrite(left_down2, LOW);
  digitalWrite(right_down, LOW);
  digitalWrite(right_down2, LOW);
}

void turnLeft(){
  digitalWrite(left_up, LOW);
  digitalWrite(left_up2, LOW);
  digitalWrite(right_up, HIGH);
  digitalWrite(right_up2, LOW);

  digitalWrite(left_down, LOW);
  digitalWrite(left_down2, LOW);
  digitalWrite(right_down, HIGH);
  digitalWrite(right_down2, LOW);
}

void backLeft(){
  digitalWrite(left_up, LOW);
  digitalWrite(left_up2, LOW);
  digitalWrite(right_up, LOW);
  digitalWrite(right_up2, HIGH);

  digitalWrite(left_down, LOW);
  digitalWrite(left_down2, LOW);
  digitalWrite(right_down, LOW);
  digitalWrite(right_down2, HIGH);
}

void moveFront(){
  digitalWrite(left_up, LOW);
  digitalWrite(left_up2, HIGH);
  digitalWrite(right_up, HIGH);
  digitalWrite(right_up2, LOW);

  digitalWrite(left_down, LOW);
  digitalWrite(left_down2, HIGH);
  digitalWrite(right_down, HIGH);
  digitalWrite(right_down2, LOW);
}

void moveBack(){
  digitalWrite(left_up, HIGH);
  digitalWrite(left_up2, LOW);
  digitalWrite(right_up, LOW);
  digitalWrite(right_up2, HIGH);

  digitalWrite(left_down, HIGH);
  digitalWrite(left_down2, LOW);
  digitalWrite(right_down, LOW);
  digitalWrite(right_down2, HIGH);
}

void ControlModeFSM(){
  Serial.println("In the control mode");

  switch(code){
    case 0xE718FF00:
      ctrlState = control_forward;
      break;
    case 0xAD52FF00:
      ctrlState = control_backward;
      break;
    case 0xF708FF00:
      ctrlState = control_left;
      break;
    case 0xA55AFF00:
      ctrlState = control_right;
      break;
    case 0xE31CFF00:
      ctrlState = control_idle;
      break;
  }

  switch(ctrlState){
    case control_idle:
      Serial.println("control_idle");
      stopMotors();
      break;
    case control_forward:
      Serial.println("control_forward");
      moveFront();
      break;
    case control_backward:
      Serial.println("control_backward");
      moveBack();
      break;
    case control_right:
      Serial.println("control_right");
      turnRight();
      break;
    case control_left:
      Serial.println("control_left");
      turnLeft();
      break;
  }

  code = 0;
  // RED LED
  digitalWrite(led_auto, LOW);
  digitalWrite(led_control, HIGH);

}

void AutoModeFSM(){
  Serial.println("In the auto mode");
  switch(autoState){
    case auto_forward:
      moveFront();
      frontDist = mySensor.ping_cm();
      Serial.print("Front: ");
      Serial.println(frontDist);

      if (frontDist > 0 && frontDist < max_dist) {
        stopMotors();
        autoState = auto_backward;
        stateStartTime = millis();
      }
      break;
    case auto_backward:
      moveBack();
      if(millis() - stateStartTime > 1000){
        stopMotors();
        autoState = auto_check_right;
        stateStartTime = millis();
      } 
      break;
    case auto_check_left:
      turnLeft();
      if(millis() - stateStartTime > 3500){
        stopMotors();
        leftDist = mySensor.ping_cm();
        Serial.print("Left distance: ");
        Serial.println(leftDist);
        autoState = auto_recenter_left;
        stateStartTime = millis();
      } 
      break;
    case auto_recenter_left:
      backLeft();
      if(millis() - stateStartTime > 3500){
        stopMotors();
        autoState = auto_decide;
        stateStartTime = millis();
      } 
      break;
    case auto_check_right:
      turnRight();
      if(millis() - stateStartTime > 3500){
        stopMotors();
        rightDist = mySensor.ping_cm();
        Serial.print("Right distance: ");
        Serial.println(rightDist);
        autoState = auto_recenter_right;
        stateStartTime = millis();
      } 
      break;
    case auto_recenter_right:
      backRight();
      if(millis() - stateStartTime > 3500){
        stopMotors();
        autoState = auto_check_left;
        stateStartTime = millis();
      } 
      break;
    case auto_decide:
      if (leftDist > rightDist && leftDist > 20) {
        turnLeft();
        if (millis() - stateStartTime > 3500) {
          autoState = auto_forward;
          stateStartTime = millis();
        }
      } else if (rightDist > leftDist && rightDist > 20) {
        turnRight();
        if (millis() - stateStartTime > 3500) {
          autoState = auto_forward;
          stateStartTime = millis();
        }
      } else {
        moveBack();
        if (millis() - stateStartTime > 2000) {
          stopMotors();
          autoState = auto_forward;
          stateStartTime = millis();
        }
      }
      break;
  }
  digitalWrite(led_auto, HIGH);
  digitalWrite(led_control, LOW);
}

void setup() {
  pinMode(left_up, OUTPUT);
  pinMode(left_up2, OUTPUT);
  pinMode(right_up, OUTPUT);
  pinMode(right_up2, OUTPUT);

  pinMode(left_down, OUTPUT);
  pinMode(left_down2, OUTPUT);
  pinMode(right_down, OUTPUT);
  pinMode(right_down2, OUTPUT);

  pinMode(led_control, OUTPUT);
  pinMode(led_auto, OUTPUT);

  Serial.begin(9600);
  IrReceiver.begin(RECV_PIN, ENABLE_LED_FEEDBACK);  // Start the receiver
}

void loop() {
  handleIR();

  switch (mode) {
    case control_mode:
      ControlModeFSM();
      break;

    case auto_mode:
      AutoModeFSM();
      break;
  }
  
}