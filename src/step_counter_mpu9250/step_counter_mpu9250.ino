/*********************
 * This appears to be working with arduino 1.8.1 IDE (latest?)
 ***********************
 **
 **********************************************************
 **Made By: Michael Kennedy
 **********************************************************
 **
 ** This is the main file for the Arduino Pro Mini to control the OpenLog and MPU-9250 modules
 ** Could optimize the libraries to remove functions not used in the program if space is an issue.
 ** 
 ** This program relies on an outside source to set the UI_SET.TXT bit to the proper mode (step 1 or gyro 0)
 ** Also needs outside source to clear SCNT.TXT b/c each power cycle will Append to it regardless if it's clear
 **
 ** Escape character is three of $$$ (ASCII 36)
 **
 HAVE TO GO INTO ARDUINO LIBRARY FILES TO CHANGE SERIALPORT/SERIALPORT.H/ BUFFERED_TX AND ENABLE_RX_ERROR_CHECKING TO 0
 */


#include <Event.h>
#include <Timer.h>

#include "dmpKey.h"
#include "dmpmap.h"
#include "freeram.h"
#include "FreeStack.h"
#include "MinimumSerial.h"

#include "mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "inv_mpu.h"
#include "I2Cdev.h"
#define DEFAULT_MPU_HZ  (200) //actually the max as well for DMP
 
Timer t;
unsigned long stepCount = 0;
unsigned long stepTime = 0;
unsigned long lastStepCount = 0;
int ret;
int error123;

int UI_INT; //variable to hold step count/ gyrometer setting
int temp;

int ledPin = 13;

void setup() {
	  
	  pinMode(ledPin, OUTPUT);
    Fastwire::setup(400,0);
    ret = mympu_open(200); //enable mpu
    Serial.begin(57600);   //connect to OpenLog
    delay(1000);           //Let the
	
	
	Serial.println("reset");
	delay(1000);
	while(1) {
    if(Serial.available())
      if(Serial.read() == '>') break; //if this breaks than we have liftoff.. or command mode
	}								  //need to change this if I want to read config file first
	
	Serial.println("read UI_SET.TXT 0 2");
	
	//read in value from UI_SET text file, set in the user interface
	while(1) {
		if(Serial.available()){
			if(Serial.read() == '2')
        break;
		}
	}
	while(1){
    if(Serial.available()){
      temp = Serial.read();
      //Serial.println(temp);
      if ((temp == '0') || (temp == '1')){
        break;
      }
    }
	}

	if(temp == '0')
		UI_INT = 0;
	else if (temp == '1')
		UI_INT = 1;
  else
    UI_INT = 42;
	
	
	Serial.println("append SCNT.TXT");
	while(1) {
		if(Serial.available()){
			if(Serial.read() == '<') break;
	  }
	}
	
  //If Step_count mode
  //load the DMP module
  //turn it on and initizlize step count/stepTime  
	if(UI_INT == 1){
		dmp_load_motion_driver_firmware();
		dmp_set_fifo_rate(DEFAULT_MPU_HZ);
		mpu_set_dmp_state(1);
		
		dmp_set_pedometer_walk_time(stepTime);
		dmp_set_pedometer_step_count(stepCount);
	}

    //int tickEvent = t.every(5000, ticker); //only will check status of steps every 10 seconds
	
  delay(10000); //test to see if gyrometer stabilizes after 10 seconds
    

}

unsigned int c = 0; //cumulative number of successful MPU/DMP reads
unsigned int np = 0; //cumulative number of MPU/DMP reads that brought no packet back
unsigned int err_c = 0; //cumulative number of MPU/DMP reads that brought corrupted packet
unsigned int err_o = 0; //cumulative number of MPU/DMP reads that had overflow bit set
int newSteps = 0;
int newTime = 0;
int oldSteps = 0;
int set = 0;
int state = 0;


void loop() {

  //read DMP memory for step time and count
  //update if it has changed
  //might need to slow this down for less power?
  //DMP hold the value anyway  
	if(UI_INT == 1){ //if we want to count steps
    //t.update();
    dmp_get_pedometer_walk_time(&stepTime);
    dmp_get_pedometer_step_count(&stepCount);
    
     newSteps = stepCount;
     if(newSteps != oldSteps){
      oldSteps = newSteps;
      Serial.println(newSteps);
      //Serial.print("Walked " + String(stepCount) + " steps");
      //Serial.println(" (" + String((stepTime) / 1000.0) + " s)");
    }

    digitalWrite(ledPin, (state) ? HIGH : LOW); //toggle LED to view function
    state = !state;
	}
	
	else if(!UI_INT){
  
		ret = mympu_update(); //update GRYO registers

		switch (ret) {
		case 0: c++; break;
		case 1: np++; return;  //no packet error happens when running 8Mhz clock to a 200 hz update rate...
		case 2: err_o++; return;
		case 3: err_c++; return; 
		default: 
			Serial.print("READ ERROR!  ");
			Serial.println(ret);
			return;
		}	

    
  /*
		if (!(c%25)) {
			Serial.print(np); Serial.print("  "); Serial.print(err_c); Serial.print(" "); Serial.print(err_o);
			Serial.print(" Y: "); Serial.print(mympu.ypr[0]);
			Serial.print(" P: "); Serial.print(mympu.ypr[1]);
			Serial.print(" R: "); Serial.print(mympu.ypr[2]);
			Serial.print("\tgy: "); Serial.print(mympu.gyro[0]);
			Serial.print(" gp: "); Serial.print(mympu.gyro[1]);
			Serial.print(" gr: "); Serial.println(mympu.gyro[2]);
		}*/

    if (!(c%25)) {
      Serial.print(np);// Serial.print("  "); Serial.print(err_c); Serial.print(" "); Serial.print(err_o);
      Serial.print("  "); 
      Serial.print(mympu.ypr[0]); //yaw
      Serial.print("  "); 
      Serial.print(mympu.ypr[1]); //pitch
      Serial.print("  "); 
      Serial.println(mympu.ypr[2]); //roll
      //Serial.print("\tgy: "); Serial.print(mympu.gyro[0]);
      //Serial.print(" gp: "); Serial.print(mympu.gyro[1]);
      //Serial.print(" gr: "); Serial.println(mympu.gyro[2]);
    }
	}
}
/*
void ticker(){


}*/

