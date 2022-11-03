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

uint8_t* on = new uint8_t[8]{ 0x01, 0x05, 0x00, 0x00, 0xff, 0x00, 0x8c, 0x3a };
uint8_t* off = new uint8_t[8]{ 0x01, 0x05, 0x00, 0x00, 0x00, 0x00, 0xcd, 0xca };

static bool value[6] = { true, true, true, false, true, true };

void setup() 
{
    Serial.begin(9600);
    while (!Serial);

    //mySerial.begin(9600);
    master.config(&mySerial, 38400);

    delay(1000);
    Serial.println(sizeof(unsigned long));
    Serial.println(sizeof(value));
    Serial.println(sizeof(bool));
    Serial.println("Modbus RTU Client Toggle");
}



unsigned long ms;
void loop() 
{
    ms = millis();
    uint8_t result = master.readCoils(3, 0x00, 6);

    Serial.println(master.getLastError());    

#if false
    //master.writeSingleCoil(1, 0x01, true);
    //master.writeMultipleCoils(1, 0x00, 2, B011);
    value[0] = value[3] = true;
    master.writeMultipleCoils(3, 0x00, value, 6);
    delay(1000);

    result = master.readCoils(3, 0x00, 6);
    Serial.println(result, BIN);
    delay(1000);

    //master.writeSingleCoil(1, 0x01, false);
    //master.writeMultipleCoils(1, 0x00, 2, 0x00);
    value[0] = value[3] = false;
    master.writeMultipleCoils(3, 0x00, value, 6);
    delay(1000);

    result = master.readCoils(3, 0x00, 6);
    Serial.println(result, BIN);
    delay(1000);
#endif
    
    Serial.println(millis() - ms);
}