/*
	Name:		Arduino_Modbus_Template.ino
	Created:	2021/7/13 9:26:09
	Modbus:		nDO, nDI, nAI(0~5V)
	���ܣ�
		1.֧����Ȧ״̬����ɢ���롢����Ĵ�������
		2.֧����ɢ�����ź�����Ȧ����ź� �İ�
*/

#include <Arduino.h>
#include <ArduinoModbus.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>

#include "Config.h"
#include "Def.h"

//================================= Configuration Check =====================================
#if DEFAULT_SLAVE_ID <= 0x00 || DEFAULT_SLAVE_ID > 0xFE
#error "DEFAULT_SLAVE_ID ֵ������Χ 0x01~0xFE"
#endif

#if DEFAULT_BAUD_ID < 0 || DEFAULT_BAUD_ID > 6
#error "DEFAULT_BAUD_ID ֵ������Χ 0x00~0x06"
#endif

#if DEFAULT_WORK_MODE < 0 || DEFAULT_WORK_MODE > 0x03
#error "DEFAULT_WORK_MODE ֵ������Χ 0x00~0x03"
#endif

#define INPUT_INTERVAL_MAX  5000
#if DEFAULT_INPUT_INTERVAL < 0 || DEFAULT_INPUT_INTERVAL > INPUT_INTERVAL_MAX
#error "DEFAULT_INPUT_INTERVAL ֵ������Χ 0~5000"
#endif

#if DEFAULT_LED_INTERVAL < 0 || DEFAULT_LED_INTERVAL > 0xFFFF
#error "DEFAULT_LED_INTERVAL ֵ������Χ 0x0000~0xFFFF"
#endif

#if CONFIG_USE_DISCRETE_INPUTS
#if DISCRETE_INPUT_MODE != INPUT && DISCRETE_INPUT_MODE != INPUT_PULLUP
#error "DISCRETE_INPUT_MODE ֵ���� INPUT or INPUT_PULLUP"
#endif
#endif
//================================= Configuration Check =====================================



// = millis()
extern volatile unsigned long timer0_millis;

uint8_t numCoils = 0;				//��Ȧ״̬����
uint8_t numDiscreteInputs = 0;		//��ɢ��������
uint8_t numInputRegisters = 0;		//����Ĵ�������
uint8_t numHoldingRegisters = 0;	//���ּĴ�������������/���ģʽ

//constexpr uint16_t configRegisters[] = { ARDUINO_VERSION , DEFAULT_SLAVE_ID ,DEFAULT_BAUD_ID , 0x0000, 0x0001, 0x0000, 0x0000, 0x0000};


ConfigRegister configRegister = ConfigRegister_default;

uint16_t* workModes;			//����/�������ģʽ����
int32_t* workModeArgs;

//uint16_t run_count = 0;											//�豸���д���
//BindingMode WorkMode = BindingMode(DEFAULT_WORK_MODE);			//��ɢ��������Ȧ����İ�ģʽ

//uint8_t slave_id = DEFAULT_SLAVE_ID;							//�豸��ַ
//uint8_t baud_id = DEFAULT_BAUD_ID;								//����������

uint32_t led_next_timer = 0;
//uint16_t led_interval_ms = LED_TRUN_INTERVAL_MS;				//led ָʾ��˸���ʱ�� ms

uint32_t input_next_timer = 0;
//uint16_t input_interval_ms = DINPUT_SHAKE_INTERVAL_MS;			//��ɢ����ȥ�����ʱ�� ms



/// <summary>
/// reset config���ָ���Ĭ�ϵĲ���
/// </summary>
void resetConfig()
{
	uint8_t blockSize = sizeof(ConfigRegister);
	ConfigRegister defaultConfig = ConfigRegister_default;

	eeprom_write_block(&defaultConfig, (uint16_t*)0x00, blockSize);
	for (uint8_t i = 0; i < numHoldingRegisters; i += 2)
		eeprom_write_word((uint16_t*)(blockSize + i), 0x0000);

	delay(100);
	resetArduino();
}

/// <summary>
/// read init config
///  run count(2Byte) | slave id(1Byte) | baud id(1Byte) | work mode(1Byte) | reserve(1Byte) | loop interval(2Byte) | led interval(2Byte) |
/// </summary>
void readInitConfig()
{
	uint8_t blockSize = sizeof(ConfigRegister);
	eeprom_read_block(&configRegister, (uint16_t*)0x00, blockSize);

	if (configRegister.runCount == 0x0000 || configRegister.runCount == 0xFFFF)
		resetConfig();
	else
		configRegister.runCount++;

	if (configRegister.reset != 0x0000) configRegister.reset = 0x0000;
	if (configRegister.version != ARDUINO_VERSION)	configRegister.version = ARDUINO_VERSION;
	if (configRegister.baudId > sizeof(BAUD_RATE) / sizeof(uint32_t)) configRegister.baudId = DEFAULT_BAUD_ID;
	if (configRegister.slaveId == 0x00 || configRegister.slaveId == 0xFF) configRegister.slaveId = DEFAULT_SLAVE_ID;

	if (numHoldingRegisters > 0)
	{
		for (uint8_t i = 0; i < numHoldingRegisters; i++)
		{
			workModes[i] = eeprom_read_word((uint16_t*)(blockSize + i));
		}
	}

	delay(100);
	eeprom_write_block(&configRegister, (uint16_t*)0x00, sizeof(ConfigRegister));
}

/// <summary>
/// read register and update config
/// </summary>
void readUpdateConfig()
{
	bool updateWrite = false;
	uint8_t blockSize = sizeof(ConfigRegister);

	uint16_t holding[CONFIG_REGISTER_COUNT] = {};
	ConfigRegister* config = (ConfigRegister*)holding;

	for (uint8_t i = 0; i < CONFIG_REGISTER_COUNT; i++)
		holding[i] = ModbusRTUServer.holdingRegisterRead(i);

	if (config->version != configRegister.version)
		ModbusRTUServer.holdingRegisterWrite(0x00, ARDUINO_VERSION);
	if (config->runCount != configRegister.runCount)
		ModbusRTUServer.holdingRegisterWrite(0x04, configRegister.runCount);

	//Reset
	if (config->reset == 0x01)
	{
		resetArduino();
	}
	else if (config->reset == 0xFF)
	{
		resetConfig();
	}

	if (config->slaveId != configRegister.slaveId)
	{
		if (config->slaveId != 0x00 || config->slaveId != 0xFF)
		{
			updateWrite = true;
			configRegister.slaveId = config->slaveId;
		}
		else
		{
			ModbusRTUServer.holdingRegisterWrite(0x01, configRegister.slaveId);
		}
	}
	
	if (config->baudId != configRegister.baudId)
	{
		if (config->baudId < sizeof(BAUD_RATE) / sizeof(uint32_t))
		{
			updateWrite = true;
			configRegister.baudId = config->baudId;
		}
		else
		{
			ModbusRTUServer.holdingRegisterWrite(0x02, configRegister.baudId);
		}
	}

	if(updateWrite)
		eeprom_write_block(&configRegister, (uint16_t*)0x00, sizeof(ConfigRegister));

	if (numHoldingRegisters == 0) return;
	for (uint8_t i = 0; i < numHoldingRegisters; i++)
	{
		//arg(uint8_t) | mode(uint8_t)
		uint16_t mode = ModbusRTUServer.holdingRegisterRead(CONFIG_REGISTER_COUNT + i);
		if (mode != workModes[i])
		{
			if ((mode & 0xFF) <= BindingMode::Only)
			{
				workModes[i] = mode;
				eeprom_write_word((uint16_t*)(blockSize + i), workModes[i]);
			}
			else
			{
				ModbusRTUServer.holdingRegisterWrite(CONFIG_REGISTER_COUNT + i, workModes[i]);
			}
		}
	}	
}

// the setup function runs once when you press reset or power the board
void setup() 
{
	delay(100);
	//clearEEPROM();
	
#if CONFIG_USE_WDT
	wdt_enable(WDTO_4S);
#endif

#if CONFIG_USE_COILS	//������Ȧ����������ź�
	numCoils = sizeof(coilPins) / sizeof(uint8_t);
	for (uint8_t i = 0; i < numCoils; i++)
	{
		pinMode(coilPins[i], OUTPUT);
	}
#endif

#if CONFIG_USE_DISCRETE_INPUTS	//������ɢ���룬���������ź�
	numDiscreteInputs = sizeof(discreteInputPins) / sizeof(uint8_t);
	for (uint8_t i = 0; i < numDiscreteInputs; i++)
	{
		pinMode(discreteInputPins[i], DISCRETE_INPUT_MODE);
	}
#endif

#if CONFIG_USE_INPUT_REGISTERS	//��������Ĵ�����ģ�������ź�(0~5V)
	numInputRegisters = sizeof(inputRegisterPins) / sizeof(uint8_t);
	for (uint8_t i = 0; i < numInputRegisters; i++)
	{
		pinMode(inputRegisterPins[i], INPUT);
	}
#endif

#if CONFIG_USE_COILS && CONFIG_USE_DISCRETE_INPUTS
	numHoldingRegisters = min(numCoils, numDiscreteInputs);
	if (numHoldingRegisters > 0)
	{
		workModes = (uint16_t*)malloc(numHoldingRegisters * sizeof(uint16_t));
		workModeArgs = (int32_t*)malloc(numHoldingRegisters * sizeof(int32_t));
	}
#else
	numHoldingRegisters = 0;
#endif

#if CONFIG_USE_LED_OUTPUT
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, LOW);
#endif
	
	readInitConfig();

	Serial.begin(BAUD_RATE[configRegister.baudId]);
	while (!Serial);

	Serial.println(configRegister.version);
	Serial.println(configRegister.slaveId);
	Serial.println(configRegister.baudId);
	//Serial.println(configRegister.inchingMs);

	while (1)
	{
		;
	}



	//ModbusRTUServer Config
	if (!ModbusRTUServer.begin(configRegister.slaveId, BAUD_RATE[configRegister.baudId]))
	{
		while (1);
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

		uint16_t* config = (uint16_t*)&configRegister;
		for(uint8_t i = 0; i < CONFIG_REGISTER_COUNT;i ++)
			ModbusRTUServer.holdingRegisterWrite(i, config[i]);

		for (uint8_t i = 0; i < numHoldingRegisters; i++)
			ModbusRTUServer.holdingRegisterWrite(CONFIG_REGISTER_COUNT + i, workModes[i]);
	}

	led_next_timer = timer0_millis + LED_TRUN_INTERVAL_MS;
	input_next_timer = timer0_millis + DINPUT_SHAKE_INTERVAL_MS;
}

// the loop function runs over and over again until power down or reset
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
			//workModeArgs[i] = -1;
		}
	}

	if (timer0_millis < input_next_timer) return;
	for (uint8_t i = 0; i < numDiscreteInputs; i++)
	{
		bool newValue = digitalRead(discreteInputPins[i]) == (DISCRETE_INPUT_MODE == INPUT_PULLUP ? 0x00 : 0x01);
		bool oldValue = ModbusRTUServer.discreteInputRead(i) == 0x01;

		bool inputRelease = newValue != oldValue && !newValue;		//�����ͷ�
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
			if (inputRelease)	//�����ͷ�
			{
				ModbusRTUServer.coilWrite(i, !(ModbusRTUServer.coilRead(i) == 0x01));
			}
		}
		else if (workMode == BindingMode::Inching)	//�㶯ģʽ
		{
			if (inputRelease)	//�����ͷ�
			{
				if (args <= 1) args = 1;
				workModeArgs[i] = timer0_millis + args * INCHING_INTERVAL_MS;
				ModbusRTUServer.coilWrite(i, !(ModbusRTUServer.coilRead(i) == 0x01));
			}
		}
		else if (workMode == BindingMode::Only)
		{
			if (inputRelease && workModeArgs[i] == -1)	//�����ͷ�
			{
				//if((args >> i) & 0x01)				
				workModeArgs[i] = i;
				for (uint8_t j = 0; j < numCoils; j++)
					ModbusRTUServer.coilWrite(j, j == workModeArgs[i] ? 0x01 : 0x00);
			}
		}
#endif
	}	

	input_next_timer = timer0_millis + DINPUT_SHAKE_INTERVAL_MS;
#endif

	
}
