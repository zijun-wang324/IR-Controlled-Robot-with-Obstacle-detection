#include <IRremote.h>
#include <Servo.h>
#include <NewPing.h>

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

boolean isMode1 = false;

int RECV_PIN = 7;  //pin 7 of arduino to data pin of ir receiver

//int servo_pin = 2;
//Servo myServo;

int led_control = A4;
int led_auto = A5;

int trig_pin = 12;
int echo_pin = A0;
int max_dist = 30;
unsigned long code = 0;
unsigned long previousPing = 0;
const unsigned long pingInterval = 100; // ms between pings

NewPing mySensor(trig_pin, echo_pin, max_dist);

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

  //myServo.attach(servo_pin);
  //myServo.write(90);


  Serial.begin(9600);
  IrReceiver.begin(RECV_PIN, ENABLE_LED_FEEDBACK);  // Start the receiver
}

bool betterDelay(unsigned long ms){
  unsigned long previousMillis = millis();
  if(millis() - previousMillis < ms){
    if (IrReceiver.decode()) {
      unsigned long code = IrReceiver.decodedIRData.decodedRawData;
      IrReceiver.resume();

      // mode switch button
      if (code == 0xB946FF00) {
        isMode1 = !isMode1;
        return false;  // exit early
      }
    }
  }
  return true; // finished normally
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

void loop() {
  if (IrReceiver.decode()) {
    //Serial.println(IrReceiver.decodedIRData.decodedRawData, HEX);
    code = IrReceiver.decodedIRData.decodedRawData;
    IrReceiver.resume();  // Receive the next value
  
    if(code == 0xB946FF00){
      isMode1 = !isMode1;
      code = 0;
    }

    if(isMode1){
      code = 0;
    }

  }
  
  if(!isMode1){
  Serial.println("In the control mode");
  
  //bot moves front
    if (code == 0xE718FF00) {
      moveFront();
      Serial.println("Front");
    }
    //bot moves back
    if (code == 0xAD52FF00) {
      moveBack();
      Serial.println("Back");
    }
    //bot moves left
    if (code == 0xF708FF00) {
      turnLeft();
      Serial.println("Left");
    }
    //bot moves right
    if (code == 0xA55AFF00) {
      turnRight();
      Serial.println("Right");
    }
    //bot stops
    if (code == 0xE31CFF00) {
      stopMotors();
      
      Serial.println("Stop");
    }

    // RED LED
    digitalWrite(led_auto, LOW);
    digitalWrite(led_control, HIGH);
    
  } else {
    Serial.println("In the auto mode");
    unsigned long currentMillis = millis();
    if (currentMillis - previousPing >= pingInterval) {
      previousPing = currentMillis;

      stopMotors(); // VERY IMPORTANT

      int frontDist = mySensor.ping_cm();
      Serial.print("Front distance: ");
      Serial.println(frontDist);
      
      if (frontDist > 0 && frontDist < max_dist) {
        // Obstacle detected → stop and back up
        stopMotors();
        moveBack();
        delay(1000);
        stopMotors();

        // Check right
        turnRight();
        delay(3500);
        stopMotors();
        int rightDist = mySensor.ping_cm();
        Serial.print("Right distance: ");
        Serial.println(rightDist);

        // Check left (from original orientation)
        backRight();
        delay(3500);
        turnLeft(); // full left from original
        delay(3500);
        stopMotors();
        int leftDist = mySensor.ping_cm();
        Serial.print("Left distance: ");
        Serial.println(leftDist);

        // Return to original orientation
        backLeft();
        delay(3500); // fine-tune to center
        stopMotors();

        // Decide which way to turn
        if (leftDist > rightDist && leftDist > 20) {
          turnLeft();
          delay(3500);
        } else if (rightDist > leftDist && rightDist > 20) {
          turnRight();
          delay(3500);
        } else {
          // Both sides blocked → back up a bit more
          moveBack();
          delay(2000);
        }
        stopMotors();
      } else {
        // Clear path → move forward
        moveFront();
      }
    }
    // BLUE LED
    digitalWrite(led_auto, HIGH);
    digitalWrite(led_control, LOW);

  }
  
}