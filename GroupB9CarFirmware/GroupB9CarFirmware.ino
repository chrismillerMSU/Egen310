/*********************************************************************
 
 Christopher Miller Egen 310

 This is the firware for our EGEN 310 car. It handles string packets sent
 in by a bluetooth LE connection and activates two motors acourdingly. 

 This code was influence by code by James Devi (The BLEsetup method).
*********************************************************************/
//including nessisary header files
#include <Arduino.h>
#include <Adafruit_BLE.h>
#include <Adafruit_BluefruitLE_SPI.h>
#include "BluefruitConfig.h"
#include <Adafruit_MotorShield.h>


// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

// And connect 2 DC motors to port M3 & M4 
Adafruit_DCMotor *L_MOTOR = AFMS.getMotor(3);
Adafruit_DCMotor *R_MOTOR = AFMS.getMotor(4);

//Set the name the will appear in the bluetooth scan on the app
String BROADCAST_NAME = "Group B9";
String BROADCAST_CMD = String("AT+GAPDEVNAME=" + BROADCAST_NAME);

//Start BLE device
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

// Error handling for crashes
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

// function prototypes over in packetparser.cpp
uint8_t readPacket(Adafruit_BLE *ble, uint16_t timeout);
float parsefloat(uint8_t *buffer);
void printHex(const uint8_t * data, const uint32_t numBytes);

// the packet buffer
extern uint8_t packetbuffer[];
char buf[60];

// Setting our relative motor speed (1-255)
#define ForwardSpeed                255
#define ReverseSpeed                255
#define TurningSpeed                255

//Starts all nessisay components when 32u4 is powered on
void setup(void) {
  Serial.begin(9600);
  

  AFMS.begin();  // create with the default frequency 1.6KHz

  // turn on motors
  L_MOTOR->setSpeed(0);
  L_MOTOR->run(RELEASE);

  R_MOTOR->setSpeed(0);
  R_MOTOR->run(RELEASE);
    
  Serial.begin(115200);
  

  //Start BLE
  BLEsetup();
}

//Runs constantly while 32u4 is on
void loop(void)
{
  // read new packet data
  uint8_t len = readPacket(&ble, BLE_READPACKET_TIMEOUT);

  readController();
}


bool isMoving = false;
unsigned long lastPress = 0;

bool readController(){
  uint8_t maxspeed;

 // if B means from Buttons
  if (packetbuffer[1] == 'B') {

    uint8_t buttnum = packetbuffer[2] - '0';
    boolean pressed = packetbuffer[3] - '0';
    //Checks is button is depressed (packetbuffer[3] == 1)
    if (pressed) {
      //If button 1 etc.
      //Left Forward
      if(buttnum == 1){
        isMoving = true; //Let us know we are moving
        L_MOTOR->run(FORWARD); //Set to predifined speeds from above
        R_MOTOR->run(FORWARD);
        maxspeed = ForwardSpeed;
        //Speed up motor
        for (int speed=0; speed < maxspeed; speed+=5) {
          R_MOTOR->setSpeed(speed);
          delay(5); // 250ms total to speed up
        }
        //Speed up left motor to half speed
        for (int speed=0; speed < maxspeed/2; speed+=5) {
          L_MOTOR->setSpeed(speed);
          delay(5); // 250ms total to speed up
        }
      }
      //Right forward
      if(buttnum == 2){
        isMoving = true;
        L_MOTOR->run(FORWARD);
        R_MOTOR->run(FORWARD);
        maxspeed = ForwardSpeed;
        //Speed up motor
        for (int speed=0; speed < maxspeed; speed+=5) {
          L_MOTOR->setSpeed(speed);
          delay(5); // 250ms total to speed up
        }
        //Speed up right motor to half speed
        for (int speed=0; speed < maxspeed/2; speed+=5) {
          R_MOTOR->setSpeed(speed);
          delay(5); // 250ms total to speed up
        }
        
      }
      //"Forward"
      if(buttnum == 5){
        isMoving = true;
        L_MOTOR->run(FORWARD);
        R_MOTOR->run(FORWARD);
        maxspeed = ForwardSpeed;
        ble.println("Forward");
      }
      //"Reverse"
      if(buttnum == 6){
        isMoving = true;
        L_MOTOR->run(BACKWARD);
        R_MOTOR->run(BACKWARD);
        maxspeed = ReverseSpeed;
        ble.println("Backward");        
      }
      //"Zero point left"
      if(buttnum == 7){
        isMoving = true;
        L_MOTOR->run(BACKWARD);
        R_MOTOR->run(FORWARD);
        maxspeed = TurningSpeed;
        ble.println("Left");
      }
      //"Zero point right"
      if(buttnum == 8){
        isMoving = true;
        L_MOTOR->run(FORWARD);
        R_MOTOR->run(BACKWARD);
        maxspeed = TurningSpeed;
        ble.println("Right");        
      }

      lastPress = millis();
      //If motors not yet sped up
      if(buttnum == 8 || buttnum == 7 || buttnum == 6 || buttnum == 5){
      // speed up the motors
        for (int speed=0; speed < maxspeed; speed+=5) {
          L_MOTOR->setSpeed(speed);
          R_MOTOR->setSpeed(speed);
          delay(5); // 250ms total to speed up
        }
      }
  } else {
      isMoving = false;
      // slow down the motors
      for (int speed = maxspeed; speed >= 0; speed-=5) {
        L_MOTOR->setSpeed(speed);
        R_MOTOR->setSpeed(speed);
        delay(5); // 50ms total to slow down
      }
      L_MOTOR->run(RELEASE);
      R_MOTOR->run(RELEASE);
    }
}
}

void BLEsetup(){
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  /* Perform a factory reset to make sure everything is in a known state */
  Serial.println(F("Performing a factory reset: "));
  if (! ble.factoryReset() ){
       error(F("Couldn't factory reset"));
  }

  //Convert the name change command to a char array
  BROADCAST_CMD.toCharArray(buf, 60);

  //Change the broadcast device name here!
  if(ble.sendCommandCheckOK(buf)){
    Serial.println("name changed");
  }
  delay(250);

  //reset to take effect
  if(ble.sendCommandCheckOK("ATZ")){
    Serial.println("resetting");
  }
  delay(250);

  //Confirm name change
  ble.sendCommandCheckOK("AT+GAPDEVNAME");

  /* Disable command echo from Bluefruit */
  ble.echo(false);


  /* Print Bluefruit information */
  ble.info();

 
  ble.verbose(false);  // debug info is a little annoying after this point!

  /* Wait for connection */
  while (! ble.isConnected()) {
      delay(500);
  }

 

  // Set Bluefruit to DATA mode
  ble.setMode(BLUEFRUIT_MODE_DATA);

}
