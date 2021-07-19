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

//================================= Configuration Check =====================================
#if DEFAULT_SLAVE_ID <= 0x00 || DEFAULT_SLAVE_ID > 0xFE
#error "DEFAULT_SLAVE_ID 值超出范围 0x01~0xFE"
#endif

#if DEFAULT_BAUD_ID < 0 || DEFAULT_BAUD_ID > 6
#error "DEFAULT_BAUD_ID 值超出范围 0x00~0x06"
#endif

#if DEFAULT_WORK_MODE < 0 || DEFAULT_WORK_MODE > 0x03
#error "DEFAULT_WORK_MODE 值超出范围 0x00~0x03"
#endif

#define INPUT_INTERVAL_MAX  5000
#if DEFAULT_INPUT_INTERVAL < 0 || DEFAULT_INPUT_INTERVAL > INPUT_INTERVAL_MAX
#error "DEFAULT_INPUT_INTERVAL 值超出范围 0~5000"
#endif

#if DEFAULT_LED_INTERVAL < 0 || DEFAULT_LED_INTERVAL > 0xFFFF
#error "DEFAULT_LED_INTERVAL 值超出范围 0x0000~0xFFFF"
#endif

#if CONFIG_USE_DISCRETE_INPUTS
#if DISCRETE_INPUT_MODE != INPUT && DISCRETE_INPUT_MODE != INPUT_PULLUP
#error "DISCRETE_INPUT_MODE 值错误 INPUT or INPUT_PULLUP"
#endif
#endif
//================================= Configuration Check =====================================

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

// = millis()
extern volatile unsigned long timer0_millis;
//设备受支持的波特率
constexpr uint32_t BAUD_RATE[] = { 9600, 19200, 38400, 115200, 230400, 460800, 921600 };

uint8_t numCoils = 0;				//线圈状态数量
uint8_t numDiscreteInputs = 0;		//离散输入数量
uint8_t numInputRegisters = 0;		//输入寄存器数量
constexpr uint8_t numHoldingRegisters = 8;	//保持寄存器数量

uint16_t run_count = 0;											//设备运行次数
BindingMode WorkMode = BindingMode(DEFAULT_WORK_MODE);			//离散输入与线圈输出的绑定模式

uint8_t slave_id = DEFAULT_SLAVE_ID;							//设备地址
uint8_t baud_id = DEFAULT_BAUD_ID;								//波特率索引

uint32_t led_next_time = 0;
uint16_t led_interval_ms = DEFAULT_LED_INTERVAL;				//led 指示闪烁间隔时间 ms

uint32_t input_next_time = 0;
uint16_t input_interval_ms = DEFAULT_INPUT_INTERVAL;			//离散输入去抖间隔时间 ms

/// <summary>
/// reset Arduino
/// </summary>
void (*resetArduino)() = 0x0000;
//void resetArduino() { __asm__ __volatile__("jmp 0"); }

/// <summary>
/// clear EEPROM
/// </summary>
void clearEEPROM()
{
	for (int i = 0; i < E2END + 1; i++)
	{
		eeprom_write_byte((uint8_t*)i, 0x00);
	}
}

/// <summary>
/// reset config，恢复到默认的参数
/// </summary>
void resetConfig()
{
	eeprom_write_word((uint16_t*)0x00, 0x01);
	eeprom_write_byte((uint8_t*)0x02, DEFAULT_SLAVE_ID);
	eeprom_write_byte((uint8_t*)0x03, DEFAULT_BAUD_ID);
	eeprom_write_byte((uint8_t*)0x04, DEFAULT_WORK_MODE);
	eeprom_write_byte((uint8_t*)0x05, 0x00);
	eeprom_write_word((uint16_t*)0x06, DEFAULT_INPUT_INTERVAL);
	eeprom_write_word((uint16_t*)0x08, DEFAULT_LED_INTERVAL);

	delay(100);
	resetArduino();
}

/// <summary>
/// read init config
///  run count(2Byte) | slave id(1Byte) | baud id(1Byte) | work mode(1Byte) | reserve(1Byte) | loop interval(2Byte) | led interval(2Byte) |
/// </summary>
void readInitConfig()
{
	//0x00~0x01
	run_count = eeprom_read_word((uint16_t*)0x00);
	if (run_count == 0x0000 || run_count == 0xFFFF)	//如果运行次数 达到 65535 次，恢复出厂参数
	{
		resetConfig();
	}
	else
	{
		run_count = run_count + 1;
		eeprom_write_word(0x00, run_count);
	}

	//0x02 设备地址，或设备ID
	slave_id = eeprom_read_byte((uint8_t*)0x02);
	if (slave_id == 0x00 || slave_id == 0xFF)
	{
		slave_id = DEFAULT_SLAVE_ID;
		eeprom_write_byte((uint8_t*)0x02, slave_id);
	}

	//0x03	波特率ID
	baud_id = eeprom_read_byte((uint8_t*)0x03);
	if (baud_id > sizeof(BAUD_RATE) / sizeof(uint32_t))
	{
		baud_id = DEFAULT_BAUD_ID;
		eeprom_write_byte((uint8_t*)0x03, baud_id);
	}

	//0x04	工作模式，离散输入与离散输出的绑定模式
	WorkMode = (BindingMode)eeprom_read_byte((uint8_t*)0x04);
	if (WorkMode > BindingMode::Only)
	{
		WorkMode = BindingMode::Default;
		eeprom_write_byte((uint8_t*)0x04, (uint8_t)WorkMode);
	}

	//0x05 reserve 在此保留一个字节
	
	//0x06~0x07 离散输入去抖时间
	input_interval_ms = eeprom_read_word((uint16_t*)0x06);
	if (input_interval_ms > INPUT_INTERVAL_MAX)
	{
		input_interval_ms = DEFAULT_INPUT_INTERVAL;
		eeprom_write_word((uint16_t*)0x06, input_interval_ms);
	}
	
	//0x08~0x09	led输出间隔时间
	led_interval_ms = eeprom_read_word((uint16_t*)0x08);

	delay(200);
}

/// <summary>
/// read register and update config
/// </summary>
void updateConfig()
{
	uint8_t slaveId = ModbusRTUServer.holdingRegisterRead(0x00);	//从保持寄存器中读取的设备地址
	uint8_t baudId = ModbusRTUServer.holdingRegisterRead(0x01);		//从保持寄存器中读取的波特率ID
	uint8_t reset = ModbusRTUServer.holdingRegisterRead(0x02);		//从保持寄存器中读取的 Reset 状态
	uint8_t workMode = ModbusRTUServer.holdingRegisterRead(0x03);	//从保持寄存器中读取的工作模式
	uint16_t inputValue = ModbusRTUServer.holdingRegisterRead(0x04);//从保持寄存器中读取的离散输入去抖时间
	uint16_t ledValue = ModbusRTUServer.holdingRegisterRead(0x05);	//从保持寄存器中读取的led闪烁间隔时间
	uint16_t runCount = ModbusRTUServer.holdingRegisterRead(0x06);	//从保持寄存器中读取的运行次数，写无效

	if (slaveId != slave_id)
	{
		if (slaveId != 0x00 || slaveId != 0xFE || slaveId != 0xFF)
		{
			slave_id = slaveId;
			eeprom_write_byte((uint8_t*)0x02, slave_id);
		}
		else
		{
			ModbusRTUServer.holdingRegisterWrite(0x00, slave_id);
		}
	}

	if (baudId != baud_id)
	{
		if (baudId < sizeof(BAUD_RATE) / sizeof(uint32_t))
		{
			baud_id = baudId;
			eeprom_write_byte((uint8_t*)0x03, baud_id);
		}
		else
		{
			ModbusRTUServer.holdingRegisterWrite(0x01, baud_id);
		}
	}

	if (workMode != WorkMode)
	{
		if (workMode <= BindingMode::Only)
		{
			WorkMode = (BindingMode)workMode;
			eeprom_write_byte((uint8_t*)0x04, (uint8_t)workMode);

			for(uint8_t i = 0; i < numCoils; i ++)
				ModbusRTUServer.coilWrite(i, 0x00);
		}
		else
		{
			ModbusRTUServer.holdingRegisterWrite(0x03, WorkMode);
		}
	}

	if (inputValue != input_interval_ms)
	{
		if (inputValue <= INPUT_INTERVAL_MAX)
		{
			input_interval_ms = inputValue;
			eeprom_write_word((uint16_t*)0x06, input_interval_ms);
		}
		else
		{
			ModbusRTUServer.holdingRegisterWrite(0x04, input_interval_ms);
		}
	}

	if (ledValue != led_interval_ms)
	{
		led_interval_ms = ledValue;
		led_next_time = timer0_millis + led_interval_ms; 
		eeprom_write_word((uint16_t*)0x08, led_interval_ms);

#if CONFIG_USE_LED_OUTPUT
		if (led_interval_ms == 0x0000 || led_interval_ms == 0xFFFF)		//常亮或常闭状态
		{
			digitalWrite(ledPin, led_interval_ms == 0xFFFF);
		}
#endif
	}

	//写无效
	if (runCount != run_count)
	{
		ModbusRTUServer.holdingRegisterWrite(0x06, run_count);
	}

	//Reset
	if (reset == 0x01)
	{
		resetArduino();
		//asm volatile("jmp 0");
	}
	else if (reset == 0xFF)
	{
		resetConfig();
	}
}

// the setup function runs once when you press reset or power the board
void setup() 
{
	delay(100);
	//clearEEPROM();

#if USE_WDT
	wdt_enable(WDTO_8S);
#endif

#if CONFIG_USE_LED_OUTPUT
	pinMode(ledPin, OUTPUT);
	digitalWrite(ledPin, LOW);
#endif
	
	readInitConfig();

	Serial.begin(BAUD_RATE[baud_id]);
	while (!Serial);

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

	//ModbusRTUServer Config
	if (!ModbusRTUServer.begin(slave_id, BAUD_RATE[baud_id]))
	{
		while (1);
	}

	if (numCoils > 0)
		ModbusRTUServer.configureCoils(0x00, numCoils);
	if (numDiscreteInputs > 0)
		ModbusRTUServer.configureDiscreteInputs(0x00, numDiscreteInputs);
	if (numInputRegisters > 0)
		ModbusRTUServer.configureInputRegisters(0x00, numInputRegisters);

	if (numHoldingRegisters > 0)
	{
		ModbusRTUServer.configureHoldingRegisters(0x00, numHoldingRegisters);

		ModbusRTUServer.holdingRegisterWrite(0x00, slave_id);
		ModbusRTUServer.holdingRegisterWrite(0x01, baud_id);
		ModbusRTUServer.holdingRegisterWrite(0x02, 0x0000);		//reset
		ModbusRTUServer.holdingRegisterWrite(0x03, WorkMode);
		ModbusRTUServer.holdingRegisterWrite(0x04, input_interval_ms);
		ModbusRTUServer.holdingRegisterWrite(0x05, led_interval_ms);
		ModbusRTUServer.holdingRegisterWrite(0x06, run_count);
		ModbusRTUServer.holdingRegisterWrite(0x07, 0x0000);		//reserve
	}

#if CONFIG_USE_LED_OUTPUT
	led_next_time = timer0_millis + led_interval_ms;
	if (led_interval_ms == 0x0000 || led_interval_ms == 0xFFFF)
	{
		digitalWrite(ledPin, led_interval_ms == 0xFFFF);
	}
#endif

	input_next_time = timer0_millis + input_interval_ms;
}

// the loop function runs over and over again until power down or reset
void loop()
{
#if USE_WDT
	wdt_reset();
#endif


	ModbusRTUServer.poll();
	updateConfig();


#if CONFIG_USE_LED_OUTPUT
	if (led_interval_ms != 0x0000 && led_interval_ms != 0xFFFF && timer0_millis > led_next_time)
	{
		digitalWrite(ledPin, !digitalRead(ledPin));
		led_next_time = timer0_millis + led_interval_ms;
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

	if (timer0_millis < input_next_time) return;

	int8_t only_index = -1;
	uint8_t minNum = min(numCoils, numDiscreteInputs);
	for (uint8_t i = 0; i < numDiscreteInputs; i++)
	{
		bool newValue = digitalRead(discreteInputPins[i]) == (DISCRETE_INPUT_MODE == INPUT_PULLUP ? 0x00 : 0x01);
		bool oldValue = ModbusRTUServer.discreteInputRead(i) == 0x01;
		if(newValue != oldValue)	ModbusRTUServer.discreteInputWrite(i, newValue);

#if CONFIG_USE_COILS
		if (WorkMode == BindingMode::Non_Lock)
		{
			if(i < minNum) ModbusRTUServer.coilWrite(i, newValue);
		}
		else if (WorkMode == BindingMode::Lock)
		{
			if (newValue != oldValue && !newValue)	//输入释放
			{
				if (i < minNum) ModbusRTUServer.coilWrite(i, !(ModbusRTUServer.coilRead(i) == 0x01));
			}
		}
		else if (WorkMode == BindingMode::Only)
		{
			if (newValue != oldValue && !newValue && only_index == -1)	//输入释放
			{
				only_index = i;
				for (uint8_t j = 0; j < numCoils; j++)
					ModbusRTUServer.coilWrite(j, j == only_index ? 0x01 : 0x00);
			}
		}
#endif
	}	

	input_next_time = timer0_millis + input_interval_ms;
#endif

	
}
