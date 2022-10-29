/*
 Name:		Arduino_MBAT_Template.ino
 Created:	2022/10/28 16:48:11
 Author:	huangmin
*/

//#define ARDUINO
//#define __AVR__

#include <Arduino.h>
#include <ArduinoRS485.h>
#include <ArduinoModbus.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>
#include <SoftwareSerial.h>
#include "ModbusSerial.h"

SoftwareSerial mySerial(13, 12); // RX, TX
ModbusSerial master;

void setup() 
{
    Serial.begin(9600);
    while (!Serial);

    //mySerial.begin(9600);
    master.config(&mySerial, 9600);

    delay(2000);
    //Serial.println("Modbus RTU Client Toggle");
}

uint8_t* on = new uint8_t[8]{ 0x01, 0x05, 0x00, 0x00, 0xff, 0x00, 0x8c, 0x3a };
uint8_t* off = new uint8_t[8]{ 0x01, 0x05, 0x00, 0x00, 0x00, 0x00, 0xcd, 0xca };



void loop() 
{
    
    //master.writeSingleCoil(1, 0x01, true);
    master.writeMultipleCoils(1, 0x00, 2, B011);
    delay(1000);

    
    //master.writeSingleCoil(1, 0x01, false);
    master.writeMultipleCoils(1, 0x00, 2, 0x00);
    delay(1000);

}