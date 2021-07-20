/*
	Name:		Arduino_Modbus_Template.ino
	Created:	2021/7/13 9:26:09
	Modbus:		nDO, nDI, nAI(0~5V)
	功能：
		1.支持线圈状态、离散输入、输入寄存器配置
		2.支持离散输入信号与线圈输出信号 的绑定
*/

#include <Arduino.h>
#include <ArduinoModbus.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>

#include "Config.h"
#include "Def.h"

//================================= Configuration Check =====================================
#if DEFAULT_SLAVE_ID <= 0x00 || DEFAULT_SLAVE_ID > 0xFE
#error "DEFAULT_SLAVE_ID 值超出范围 0x01~0xFE"
#endif

#if DEFAULT_BAUD_ID < 0 || DEFAULT_BAUD_ID > 6
#error "DEFAULT_BAUD_ID 值超出范围 0x00~0x06"
#endif

#if CONFIG_USE_DISCRETE_INPUTS
#if DISCRETE_INPUT_MODE != INPUT && DISCRETE_INPUT_MODE != INPUT_PULLUP
#error "DISCRETE_INPUT_MODE 值错误 INPUT or INPUT_PULLUP"
#endif
#endif
//================================= Configuration Check =====================================


uint8_t numCoils = 0;				//线圈状态数量
uint8_t numDiscreteInputs = 0;		//离散输入数量
uint8_t numInputRegisters = 0;		//输入寄存器数量
uint8_t numHoldingRegisters = 0;	//保持寄存器数量，输入/输出模式

uint16_t* workModes;				//每一路 输入/输出 对应的工作模式数组
int32_t*  workModeArgs;				//每一路 输入/输出 对应的工作模式所使用的参数

extern volatile unsigned long timer0_millis;				// = millis()
DeviceConfig deviceConfig = DeviceConfig_default;			//设备参数配置

uint32_t led_next_timer = 0;
uint32_t input_next_timer = 0;

uint8_t configSize = sizeof(DeviceConfig);
uint16_t configHRR[CONFIG_REGISTER_COUNT] = {};
DeviceConfig* configPtr = (DeviceConfig*)configHRR;


/// <summary>
/// reset config write to EEPROM，恢复到默认的参数 
/// (ConfigRegister) | [(input/output)]
/// </summary>
void resetConfig()
{
	uint8_t configSize = sizeof(DeviceConfig);
	DeviceConfig defaultConfig = DeviceConfig_default;
	defaultConfig.runCount = deviceConfig.runCount;		//运行次数不可恢复

	eeprom_write_block(&defaultConfig, (uint16_t*)0x00, configSize);
	for (uint8_t i = 0; i < numHoldingRegisters; i++)
		eeprom_write_word((uint16_t*)(configSize + (i * 2)), 0x0000);

	delay(100);
	resetArduino();
}

/// <summary>
/// read init config
/// </summary>
void readInitConfig()
{
	uint8_t configSize = sizeof(DeviceConfig);
	eeprom_read_block(&deviceConfig, (uint16_t*)0x00, configSize);

	deviceConfig.runCount++;
	if (deviceConfig.reset != 0x0000)				deviceConfig.reset = 0x0000;
	if (deviceConfig.version != ARDUINO_VERSION)	deviceConfig.version = ARDUINO_VERSION;
	if (deviceConfig.baudId > sizeof(BAUD_RATE) / sizeof(uint32_t))		deviceConfig.baudId = DEFAULT_BAUD_ID;
	if (deviceConfig.slaveId == 0x00 || deviceConfig.slaveId == 0xFF)	deviceConfig.slaveId = DEFAULT_SLAVE_ID;

	for (uint8_t i = 0; i < numHoldingRegisters; i ++)
	{
		workModes[i] = eeprom_read_word((uint16_t*)(configSize + (i * 2)));
		if ((workModes[i] & 0xFF) > BindingMode::Only)
		{
			workModes[i] = 0x0000;
			eeprom_write_word((uint16_t*)(configSize + (i * 2)), 0x0000);
		}
	}

	eeprom_write_block(&deviceConfig, (uint16_t*)0x00, configSize);
	delay(100);
}

/// <summary>
/// read register and update config
/// </summary>
void readUpdateConfig()
{
	bool updateWrite = false;
	for (uint8_t i = 0; i < CONFIG_REGISTER_COUNT; i++)
		configHRR[i] = ModbusRTUServer.holdingRegisterRead(i);

	//禁止修改的数据，没有使用输入寄存器
	if (configPtr->version != deviceConfig.version)
		ModbusRTUServer.holdingRegisterWrite(0x00, ARDUINO_VERSION);
	if (configPtr->runCount != deviceConfig.runCount)
		ModbusRTUServer.holdingRegisterWrite(0x04, deviceConfig.runCount);

	//Reset
	if (configPtr->reset == 0x01)
	{
		resetArduino();
	}
	else if (configPtr->reset == 0xFF)
	{
		resetConfig();
	}

	if (configPtr->slaveId != deviceConfig.slaveId)
	{
		if (configPtr->slaveId != 0x00 && configPtr->slaveId != 0xFF)
		{
			updateWrite = true;
			deviceConfig.slaveId = configPtr->slaveId;
		}
		else
		{
			ModbusRTUServer.holdingRegisterWrite(0x01, deviceConfig.slaveId);
		}
	}
	
	if (configPtr->baudId != deviceConfig.baudId)
	{
		if (configPtr->baudId < sizeof(BAUD_RATE) / sizeof(uint32_t))
		{
			updateWrite = true;
			deviceConfig.baudId = configPtr->baudId;
		}
		else
		{
			ModbusRTUServer.holdingRegisterWrite(0x02, deviceConfig.baudId);
		}
	}

	if(updateWrite)
		eeprom_write_block(&deviceConfig, (uint16_t*)0x00, configSize);

	//WorkMode
	if (numHoldingRegisters == 0) return;
	for (uint8_t i = 0; i < numHoldingRegisters; i++)
	{
		//args(uint8_t) | mode(uint8_t)
		uint16_t mode = ModbusRTUServer.holdingRegisterRead(CONFIG_REGISTER_COUNT + i);
		if (mode != workModes[i])
		{
			if ((mode & 0xFF) <= BindingMode::Only)
			{
				workModes[i] = mode;
				eeprom_write_word((uint16_t*)(configSize + (i * 2)), workModes[i]);
			}
			else
			{
				ModbusRTUServer.holdingRegisterWrite(CONFIG_REGISTER_COUNT + i, workModes[i]);
			}
		}
	}	
}

/// <summary>
/// the setup function runs once when you press reset or power the board
/// </summary>
void setup() 
{
	delay(100);
	//clearEEPROM();	

#if CONFIG_USE_COILS	//配置线圈，数字输出信号
	numCoils = sizeof(coilPins) / sizeof(uint8_t);
	for (uint8_t i = 0; i < numCoils; i++)
	{
		pinMode(coilPins[i], OUTPUT);
	}
#endif

#if CONFIG_USE_DISCRETE_INPUTS	//配置离散输入，数字输入信号
	numDiscreteInputs = sizeof(discreteInputPins) / sizeof(uint8_t);
	for (uint8_t i = 0; i < numDiscreteInputs; i++)
	{
		pinMode(discreteInputPins[i], DISCRETE_INPUT_MODE);
	}
#endif

#if CONFIG_USE_INPUT_REGISTERS	//配置输入寄存器，模拟输入信号(0~5V)
	numInputRegisters = sizeof(inputRegisterPins) / sizeof(uint8_t);
	for (uint8_t i = 0; i < numInputRegisters; i++)
	{
		pinMode(inputRegisterPins[i], INPUT);
	}
#endif

#if CONFIG_USE_COILS && CONFIG_USE_DISCRETE_INPUTS	//关联输入输出
	numHoldingRegisters = min(numCoils, numDiscreteInputs);
	if (numHoldingRegisters > 0)
	{
		workModes = (uint16_t*)malloc(numHoldingRegisters * sizeof(uint16_t));
		workModeArgs = (int32_t*)malloc(numHoldingRegisters * sizeof(int32_t));

		memset(workModes, 0x00, numHoldingRegisters * sizeof(uint16_t));
		memset(workModeArgs, 0x00, numHoldingRegisters * sizeof(int32_t));
	}
#endif

#if CONFIG_USE_LED_OUTPUT
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, LOW);
#endif
	
	readInitConfig();

	Serial.begin(BAUD_RATE[deviceConfig.baudId]);
	while (!Serial)
	{
		;
	}

	delay(100);		//ModbusRTUServer Config
	if (!ModbusRTUServer.begin(deviceConfig.slaveId, BAUD_RATE[deviceConfig.baudId]))
	{
		while (1)
		{
			;
		}
	}

	if (numCoils > 0)
		ModbusRTUServer.configureCoils(0x00, numCoils);
	if (numDiscreteInputs > 0)
		ModbusRTUServer.configureDiscreteInputs(0x00, numDiscreteInputs);
	if (numInputRegisters > 0)
		ModbusRTUServer.configureInputRegisters(0x00, numInputRegisters);

	uint8_t numConfigRegisters = CONFIG_REGISTER_COUNT + numHoldingRegisters;
	if (numConfigRegisters > 0)
	{
		ModbusRTUServer.configureHoldingRegisters(0x00, numConfigRegisters);

		uint16_t* config = (uint16_t*)&deviceConfig;
		for (uint8_t i = 0; i < CONFIG_REGISTER_COUNT; i++)
			ModbusRTUServer.holdingRegisterWrite(i, config[i]);

		for (uint8_t i = 0; i < numHoldingRegisters; i++)
			ModbusRTUServer.holdingRegisterWrite(CONFIG_REGISTER_COUNT + i, workModes[i]);
	}

	led_next_timer = timer0_millis + LED_TRUN_INTERVAL_MS;
	input_next_timer = timer0_millis + DINPUT_SHAKE_INTERVAL_MS;

#if CONFIG_USE_WDT
	wdt_enable(WDTO_4S);
#endif
	delay(100);
}

/// <summary>
/// the loop function runs over and over again until power down or reset
/// </summary>
void loop()
{
#if CONFIG_USE_WDT
	wdt_reset();
#endif

	ModbusRTUServer.poll();
	readUpdateConfig();


#if CONFIG_USE_LED_OUTPUT
	if (timer0_millis > led_next_timer)
	{
		digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
		led_next_timer = timer0_millis + LED_TRUN_INTERVAL_MS;
	}
#endif	


#if CONFIG_USE_COILS
	for (uint8_t i = 0; i < numCoils; i++)
	{
		uint8_t coilValue = ModbusRTUServer.coilRead(i);
		digitalWrite(coilPins[i], coilValue);
	}
#endif


#if CONFIG_USE_INPUT_REGISTERS
	for (uint8_t i = 0; i < numInputRegisters; i++)
	{
		uint16_t inputValue = analogRead(inputRegisterPins[i]);
		ModbusRTUServer.inputRegisterWrite(i, inputValue);
	}
#endif


#if CONFIG_USE_DISCRETE_INPUTS
	for (uint8_t i = 0; i < numHoldingRegisters; i++)
	{
		uint8_t args = workModes[i] >> 8;
		BindingMode workMode = (BindingMode)(workModes[i] & 0xFF);

		if (workMode == BindingMode::Inching)
		{
			if (workModeArgs[i] <= 0)
			{
				workModeArgs[i] = 0;
				continue;
			}

			if (workModeArgs[i] <= timer0_millis)
			{
				workModeArgs[i] = 0;
				ModbusRTUServer.coilWrite(i, 0x00);
			}
		}
		else if (workMode == BindingMode::Only)
		{
			workModeArgs[i] = -1;
		}
	}

	if (timer0_millis < input_next_timer) return;
	for (uint8_t i = 0; i < numDiscreteInputs; i++)
	{
		bool newValue = digitalRead(discreteInputPins[i]) == (DISCRETE_INPUT_MODE == INPUT_PULLUP ? 0x00 : 0x01);
		bool oldValue = ModbusRTUServer.discreteInputRead(i) == 0x01;

		bool inputRelease = newValue != oldValue && !newValue;		//输入释放状态
		if(newValue != oldValue)	ModbusRTUServer.discreteInputWrite(i, newValue);

#if CONFIG_USE_COILS
		if (i >= numHoldingRegisters) continue;

		uint8_t args = workModes[i] >> 8;
		BindingMode workMode = (BindingMode)(workModes[i] & 0xFF);

		if (workMode == BindingMode::Non_Lock)
		{
			ModbusRTUServer.coilWrite(i, newValue);
		}
		else if (workMode == BindingMode::Lock)
		{
			if (inputRelease)	//输入释放
			{
				ModbusRTUServer.coilWrite(i, !(ModbusRTUServer.coilRead(i) == 0x01));
			}
		}
		else if (workMode == BindingMode::Inching)	//点动模式
		{
			if (inputRelease)	//输入释放
			{
				if (args <= 1) args = 1;
				ModbusRTUServer.coilWrite(i, 0x01);
				workModeArgs[i] = timer0_millis + args * INCHING_INTERVAL_MS;
				//ModbusRTUServer.coilWrite(i, !(ModbusRTUServer.coilRead(i) == 0x01));
			}
		}
		else if (workMode == BindingMode::Only)
		{
			if (!bitRead(args, i)) continue;			
			if (inputRelease && workModeArgs[i] == -1)	//输入释放
			{
				workModeArgs[i] = i;
				for (uint8_t j = 0; j < numCoils; j++)
				{
					ModbusRTUServer.coilWrite(j, j == workModeArgs[i] ? 0x01 : 0x00);
				}
			}
		}
#endif
	}	

	input_next_timer = timer0_millis + DINPUT_SHAKE_INTERVAL_MS;
#endif
	
}
