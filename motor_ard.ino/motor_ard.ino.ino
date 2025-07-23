// Arduino Code for the Autonomous Surface Vehicle
// Forward, Left, Right and Stop functions are defined.
// Serial communication exists with the computer.
// 18 August 2023
// by @VatozZ


//#include <MPU6050.h>
#include <Wire.h>
#include <Servo.h>
Servo right_motor;
Servo left_motor;
Servo led_pin ;


void setup(){
  // Set the baud rate  
  Serial.begin(9600);
  right_motor.attach(3); //right thruster -> 3. pin
  left_motor.attach(5);  // left thruster -> 5. pin
  led_pin.attach(4) ;//pin 
  left_motor.writeMicroseconds(800); 
  right_motor.writeMicroseconds(800);
  led_pin.writeMicroseconds(1000); }

void loop(){
 
  if(Serial.available() > 0) {

    String data = Serial.readStringUntil('\n');
    
	// Forward Thrust
	if(data=="w"){
		left_motor.writeMicroseconds(1050);
		right_motor.writeMicroseconds(1030);
		led_pin.writeMicroseconds(1000);
    }
	
	// Left Thrust
    if(data=="a"){
		left_motor.writeMicroseconds(800);
		right_motor.writeMicroseconds(1200);
		led_pin.writeMicroseconds(1500);
    }
	
	// Right Thrust
    if(data=="d"){
		left_motor.writeMicroseconds(1300);
		right_motor.writeMicroseconds(800);
		led_pin.writeMicroseconds(2000);
	}
	
	// Stop 
	if(data=="s"){
		left_motor.writeMicroseconds(800);
		right_motor.writeMicroseconds(800);
		led_pin.writeMicroseconds(1200);
    }
  }

}
