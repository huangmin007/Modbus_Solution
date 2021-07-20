#pragma once

#define		CONFIG_REGISTER_COUNT		8

struct ConfigRegister_s
{
	uint16_t version;	//固件版本号
	uint16_t slaveId;	//设备地址
	uint16_t baudId;	//波特率ID
	uint16_t reset;		//重启设备
	uint16_t runCount;	//运行次数
	uint16_t reserve0;	//保留 0
	uint16_t reserve1;	//保留 1
	uint16_t reserve2;	//保留 2
} ConfigRegister_default = { ARDUINO_VERSION , DEFAULT_SLAVE_ID ,DEFAULT_BAUD_ID , 0x0000, 0x0001, INCHING_INTERVAL_MS, 0x0000, 0x0000 };

typedef struct ConfigRegister_s ConfigRegister;


/// <summary>
/// 离散输入信号 与 线圈输出信号 的绑定模式
/// </summary>
enum BindingMode :uint8_t
{
	Default = 0x00,		//无任何联运模式
	Non_Lock = 0x01,	//[本机] 输入/输出 非锁 联动模式
	Lock = 0x02,		//[本机] 输入/输出 自锁 联动模式
	Inching = 0x03,		//[本机] 输入/输出 点动 联动模式
	Only = 0x04,		//[本机] 互锁 联动模式，整机按一组 未实现
};


//设备受支持的波特率
constexpr uint32_t BAUD_RATE[] = { 9600, 19200, 38400, 115200, 230400, 460800, 921600 };


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