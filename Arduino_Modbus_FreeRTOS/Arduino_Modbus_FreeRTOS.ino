/*
 Name:		Arduino_Modbus_FreeRTOS.ino
 Created:	2021/7/15 15:42:07
 Author:	huangmin
*/
#include <ArduinoModbus.h>
#include <Arduino_FreeRTOS.h>
#include <semphr.h>

SemaphoreHandle_t xModbusSemaphore;

void TaskStatusOutput(void* pvParameters);	// ‰≥ˆ◊¥Ã¨÷∏ æµ∆
void TaskDigitalWrite(void* pvParameters);	//¿Î…¢ ‰≥ˆ£¨œﬂ»¶◊¥Ã¨
void TaskDigitalRead(void* pvParameters);	//¿Î…¢ ‰»Î
void TaskAnalogRead(void* pvParameters);	// ‰»Îºƒ¥Ê∆˜
void TaskRegisterRead(void* pvParameters);	



// the setup function runs once when you press reset or power the board
void setup() 
{
	Serial.begin(9600);
	while (!Serial)
	{
		;
	}

	if (xModbusSemaphore == NULL)  // Check to confirm that the Serial Semaphore has not already been created.
	{
		xModbusSemaphore = xSemaphoreCreateMutex();  // Create a mutex semaphore we will use to manage the Serial Port
		if ((xModbusSemaphore) != NULL)
			xSemaphoreGive((xModbusSemaphore));  // Make the Serial Port available for use, by "Giving" the Semaphore.
	}

	// Now set up two Tasks to run independently.
	xTaskCreate(
		TaskDigitalRead
		, "DigitalRead"  // A name just for humans
		, 128  // This stack size can be checked & adjusted by reading the Stack Highwater
		, NULL //Parameters for the task
		, 2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
		, NULL); //Task Handle

	xTaskCreate(
		TaskAnalogRead
		, "AnalogRead" // A name just for humans
		, 128  // Stack size
		, NULL //Parameters for the task
		, 1  // Priority
		, NULL); //Task Handle

	//ModbusRTUServer Config
	if (!ModbusRTUServer.begin(1, 9600))
	{
		while (1);
	}

	ModbusRTUServer.configureHoldingRegisters(0x00, 1);

	ModbusRTUServer.holdingRegisterWrite(0x00, 0);
}

// the loop function runs over and over again until power down or reset
void loop() 
{
	ModbusRTUServer.holdingRegisterWrite(0x00, 0);
}


void TaskDigitalRead(void* pvParameters __attribute__((unused)))  // This is a Task.
{
	/*
	  DigitalReadSerial
	  Reads a digital input on pin 2, prints the result to the serial monitor
	  This example code is in the public domain.
	*/

	// digital pin 2 has a pushbutton attached to it. Give it a name:
	uint8_t pushButton = 2;

	// make the pushbutton's pin an input:
	pinMode(pushButton, INPUT);

	for (;;) // A Task shall never return or exit.
	{
		// read the input pin:
		int buttonState = digitalRead(pushButton);

		// See if we can obtain or "Take" the Serial Semaphore.
		// If the semaphore is not available, wait 5 ticks of the Scheduler to see if it becomes free.
		if (xSemaphoreTake(xModbusSemaphore, (TickType_t)5) == pdTRUE)
		{
			// We were able to obtain or "Take" the semaphore and can now access the shared resource.
			// We want to have the Serial Port for us alone, as it takes some time to print,
			// so we don't want it getting stolen during the middle of a conversion.
			// print out the state of the button:
			Serial.println(buttonState);

			xSemaphoreGive(xModbusSemaphore); // Now free or "Give" the Serial Port for others.
		}

		vTaskDelay(1);  // one tick delay (15ms) in between reads for stability
	}
}

void TaskAnalogRead(void* pvParameters __attribute__((unused)))  // This is a Task.
{

	for (;;)
	{
		// read the input on analog pin 0:
		int sensorValue = analogRead(A0);

		// See if we can obtain or "Take" the Serial Semaphore.
		// If the semaphore is not available, wait 5 ticks of the Scheduler to see if it becomes free.
		if (xSemaphoreTake(xModbusSemaphore, (TickType_t)5) == pdTRUE)
		{
			// We were able to obtain or "Take" the semaphore and can now access the shared resource.
			// We want to have the Serial Port for us alone, as it takes some time to print,
			// so we don't want it getting stolen during the middle of a conversion.
			// print out the value you read:
			Serial.println(sensorValue);

			xSemaphoreGive(xModbusSemaphore); // Now free or "Give" the Serial Port for others.
		}

		vTaskDelay(1);  // one tick delay (15ms) in between reads for stability
	}
}