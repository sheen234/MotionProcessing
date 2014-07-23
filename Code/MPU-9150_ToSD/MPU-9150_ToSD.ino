// I2C device class (I2Cdev) demonstration Arduino sketch for MPU9150
// 1/4/2013 original by Jeff Rowberg <jeff@rowberg.net> at https://github.com/jrowberg/i2cdevlib
//          modified by Aaron Weiss <aaron@sparkfun.com>
//          modified by Cheyenne Yari <cyari@ucsd.edu>
//
// Changelog:
//     2011-10-07 - initial release
//     2013-1-4 - added raw magnetometer output
//     2014-4-17 - changed for use with Teensy 2.0
//     2014-5-22 - removed data out via serial
//     2014-5-22 - added use of SdFat to read raw data to a microSD card

/* ============================================
/* ============================================
I2Cdev device library code is placed under the MIT license

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

// Because we will be sending data to the microSD card, instead of the PC, it
// will be necessary to include the SD, SDFat, or SDFat32 library.
#include "SdFat.h"
#include "SdFatUtil.h"

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#include <Wire.h>

#define FILE_SIZE_MB 5
#define FILE_SIZE (1000000UL*FILE_SIZE_MB)
#define BUF_SIZE 100
#define FINAL_LOOP 3000

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 sensor;

// SD chip select pin
const uint8_t CS = 0;

// buffer for file input
// not using currently BUT I NEED TO
uint16_t buf[BUF_SIZE]

// file system
SdFat sd;

// test file
SdFile file;

// store error strings in flash to save RAM
#define error(s) sd.errorHalt_P(PSTR(s))

int16_t accelData[3];
int16_t gyroData[3];
int16_t magData[3];

#define LED_PIN 13

bool blinkState = false;

void setup() {
    // temporary loop counter NEED TO CHANGE
    int loopCount = 0; 
    
    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();

    // wait for Leonardo
    while(!Serial){}  
    
    // initialize serial communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    // it's really up to you depending on your project)
    Serial.begin(38400);
 
    // Make sure the default chip select pin is set to output.
    pinMode(CS, OUTPUT);
    
    // initialize the SD card at SPI_FULL_SPEED for the best performance.
    // try SPI_HALF_SPEED if bus error occurs.
    if(!sd.begin(CS, SPI_FULL_SPEED)) sd.initErrorHalt();
    
    Serial.print("\nType is FAT");
    Serial.println(int(sd.vol()->fatType());
    
    //open or create file - truncate existing file.
    if(!file.open("BENCH.DAT", 0_CREAT | 0_TRUNC | 0_RDWR)) {
      error("open failed");
    } 
    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();
    
    Serial.println(pstr("\nUse a freshly formated SD for the best performance.");
    
    // verify sensor connection
    Serial.println("Testing device connections...");
    Serial.println(sensor.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    // configure Arduino LED for
    pinMode(LED_PIN, OUTPUT);
    
    // catch any reset/power up problems
    delay(400);
}

void loop() {
  if(loopCount < FINAL_LOOP) {
    
    Serial.print("\nFree RAM: ");
    Serial.println(FreeRam());
    
    // read raw accel/gyro measurements from device
    sensor.getMotion9(&accelData[0], &accelData[1], &accelData[2], 
                      &gyroData[0], &gyroData[1], &gyroData[2], 
                      &magData[0], &magData[1], &magData[2]);

    // these methods (and a few others) are also available:
    //    accelgyro.getAcceleration(&ax, &ay, &az);
    //    accelgyro.getRotation(&gx, &gy, &gz);

    // write tab-separated accel/gyro x/y/z values to SD card
    char *outToSD = accelData[0] + "\t" + accelData[1] + "\t" + accelData[2] + "\t" + 
                     gyroData[0] + "\t" + gyroData[1] + "\t" + gyroData[2] + "\t" +
                     magData[0] + "\t" + magData[1] + "\t" + magData[2] + "\n";
                     
    file.write(outToSD);

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
    
    // increment loop
    loopCount++;
  }
  else if(loopCount == FINAL_LOOP)
  {
    file.close();
  }
  else
  {
    Serial.println("Sensor reading complete!!!");
  }
}
