/*********************
 * This appears to be working with arduino 1.8.1 IDE (latest?)
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
#define DEFAULT_MPU_HZ  (20)
 
Timer t;
unsigned long stepCount = 0;
unsigned long stepTime = 0;
unsigned long lastStepCount = 0;
int ret;
int error123;
int ledPin = 13;

void setup() {
    Fastwire::setup(400,0);
    Serial.begin(57600);
    delay(1000); //let OPENLOG init.
    ret = mympu_open(200);

    dmp_load_motion_driver_firmware();
    //dmp_enable_feature(DMP_FEATURE_PEDOMETER); already set in mympu_open function
    dmp_set_fifo_rate(DEFAULT_MPU_HZ);
    mpu_set_dmp_state(1);
    
    dmp_set_pedometer_walk_time(stepTime);
    dmp_set_pedometer_step_count(stepCount);

    //int tickEvent = t.every(5000, ticker);
    
    Serial.print("MPU init: "); Serial.println(ret);
    Serial.print("Free mem: "); Serial.println(freeRam());
}

unsigned int c = 0; //cumulative number of successful MPU/DMP reads
unsigned int np = 0; //cumulative number of MPU/DMP reads that brought no packet back
unsigned int err_c = 0; //cumulative number of MPU/DMP reads that brought corrupted packet
unsigned int err_o = 0; //cumulative number of MPU/DMP reads that had overflow bit set
int newSteps = 0;
int newTime = 0;
int oldSteps = 0;
int set = 0;
//stage

void loop() {
    
    dmp_get_pedometer_walk_time(&stepTime);
    dmp_get_pedometer_step_count(&stepCount);
    //t.update();
    //ret = mympu_update(); this was to update the gyro values, causes error when using DMP
    if(!set){
      Serial.println("Pedometer will begin after 5 seconds of steps taken");
    }

   newSteps = stepCount;
   if(newSteps != oldSteps){

      oldSteps = newSteps;
      Serial.print("Walked " + String(stepCount) + " steps");
      Serial.println(" (" + String((stepTime) / 1000.0) + " s)");
    }

    set = 1;

/* SHOULD NEVER RETURN ERROR IF NOT READING ACCELEROMTER
    switch (ret) {
	case 0: c++; break;
	case 1: np++; return;
	case 2: err_o++; return;
	case 3: err_c++; return; 
	default: 
		Serial.print("READ ERROR!  ");
		Serial.println(ret);
		return;
    }
    set = 1;

    */

    /*if (!(c%25)) {
	    Serial.print(np); Serial.print("  "); Serial.print(err_c); Serial.print(" "); Serial.print(err_o);
	    Serial.print(" Y: "); Serial.print(mympu.ypr[0]);
	    Serial.print(" P: "); Serial.print(mympu.ypr[1]);
	    Serial.print(" R: "); Serial.print(mympu.ypr[2]);
	    Serial.print("\tgy: "); Serial.print(mympu.gyro[0]);
	    Serial.print(" gp: "); Serial.print(mympu.gyro[1]);
	    Serial.print(" gr: "); Serial.println(mympu.gyro[2]);

    }*/
}
/*
void ticker(){
  Serial.print("2 second tick: millis()=");
  Serial.println(millis());
}*/

