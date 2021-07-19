/*
 Name:		Arduino_Modbus_FreeRTOS.ino
 Created:	2021/7/15 15:42:07
 Author:	huangmin
*/
#include <ArduinoModbus.h>
#include <Arduino_FreeRTOS.h>
#include <semphr.h>

#include "Config.h"
#include <Arduino.h>

/// <summary>
/// 离散输入信号 与 线圈输出信号 的绑定模式
/// </summary>
enum BindingMode :uint8_t
{
	Default = 0x00,		//无关联关系
	Non_Lock = 0x01,	//非锁定，状态同步
	Lock = 0x02,		//锁定
	Only = 0x03,		//锁定，输出唯一状态
};


//void TaskStatusOutput(void* pvParameters);	//输出状态指示灯
//void TaskDigitalWrite(void* pvParameters);	//离散输出，线圈状态
//void TaskDigitalRead(void* pvParameters);	//离散输入
//void TaskAnalogRead(void* pvParameters);	//输入寄存器
//void TaskRegisterRead(void* pvParameters);	



// the setup function runs once when you press reset or power the board
void setup() 
{
	Serial.begin(9600);
	while (!Serial)
	{
		;
	}

	//ModbusRTUServer Config
	if (!ModbusRTUServer.begin(1, 9600))
	{
		while (1);
	}

	//xTaskCreate(TaskRegisterRead, "RegisterRead", 32, NULL, 1, NULL);
	xTaskCreate(TaskStatusOutput, "StatusOutput", 64, NULL, 2, NULL);

	//xTaskCreate(TaskDigitalRead, "DigitalRead", 32, NULL, 2, NULL);
	//xTaskCreate(TaskDigitalWrite, "DigitalWrite", 32, NULL, 2, NULL);
	//xTaskCreate(TaskAnalogRead, "AnalogRead", 32, NULL, 2 , NULL);


	ModbusRTUServer.configureHoldingRegisters(0x00, 1);

	ModbusRTUServer.holdingRegisterWrite(0x00, 0);
}

// the loop function runs over and over again until power down or reset
void loop() 
{	
}

void TaskStatusOutput(void* pvParameters)
{
	pinMode(LED_BUILTIN, OUTPUT);

	for (;;)
	{
		digitalWrite(LED_BUILTIN, HIGH); 
		vTaskDelay(100 / portTICK_PERIOD_MS); 

		digitalWrite(LED_BUILTIN, LOW);
		vTaskDelay(100 / portTICK_PERIOD_MS);
	}
}

void TaskRegisterRead(void* pvParameters)
{
	for (;;)
	{
		vTaskDelay(1);
	}
}

void TaskDigitalWrite(void* pvParameters)
{
	uint8_t numCoils = sizeof(coilPins) / sizeof(uint8_t);
	for (uint8_t i = 0; i < numCoils; i++)
	{
		pinMode(coilPins[i], OUTPUT);
	}

	for (;;)
	{
		for (uint8_t i = 0; i < numCoils; i++)
		{
			uint8_t coilValue = ModbusRTUServer.coilRead(i);
			digitalWrite(coilPins[i], coilValue);
		}
	}
}


void TaskDigitalRead(void* pvParameters __attribute__((unused)))
{
	uint8_t numCoils = sizeof(coilPins) / sizeof(uint8_t);
	uint8_t numDiscreteInputs = sizeof(discreteInputPins) / sizeof(uint8_t);
	for (uint8_t i = 0; i < numDiscreteInputs; i++)
	{
		pinMode(discreteInputPins[i], DISCRETE_INPUT_MODE);
	}

	for (;;)
	{
		int8_t only_index = -1;
		uint8_t minNum = min(numCoils, numDiscreteInputs);

		for (uint8_t i = 0; i < numDiscreteInputs; i++)
		{
			bool newValue = digitalRead(discreteInputPins[i]) == (DISCRETE_INPUT_MODE == INPUT_PULLUP ? 0x00 : 0x01);
			bool oldValue = ModbusRTUServer.discreteInputRead(i) == 0x01;
			if (newValue != oldValue)	ModbusRTUServer.discreteInputWrite(i, newValue);

		}

		vTaskDelay(1);
	}
}

#if CONFIG_USE_INPUT_REGISTERS
void TaskAnalogRead(void* pvParameters __attribute__((unused)))  // This is a Task.
{
	uint8_t numInputRegisters = sizeof(inputRegisterPins) / sizeof(uint8_t);
	for (uint8_t i = 0; i < numInputRegisters; i++)
	{
		pinMode(inputRegisterPins[i], INPUT);
	}

	for (;;)
	{
		for (uint8_t i = 0; i < numInputRegisters; i++)
		{
			uint16_t inputValue = analogRead(inputRegisterPins[i]);
			ModbusRTUServer.inputRegisterWrite(i, inputValue);
		}

		vTaskDelay(1);
	}
}
#endif