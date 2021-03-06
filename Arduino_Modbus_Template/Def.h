#pragma once

#define		ARDUINO_VERSION					0x0101		//固件版本号，高字节表示大版本号，低字节小版本号

#define		LED_TRUN_INTERVAL_MS			500			//LED 翻转间隔时间 ms
#define		DINPUT_SHAKE_INTERVAL_MS		8			//数字信号输入防抖间隔时间 ms
#define		INCHING_INTERVAL_MS				250			//点动模式的单位间隔时间 ms

#define		CONFIG_REGISTER_COUNT			8			//参数配置数据数量

#pragma pack (1)
struct DeviceConfig_s
{
	uint16_t version;	//固件版本号
	uint16_t slaveId;	//设备地址，低8位有效
	uint16_t baudId;	//波特率ID，低8位有效
	uint16_t reset;		//重启设备
	uint16_t runCount;	//运行次数
	uint16_t reserve0;	//保留 0
	uint16_t reserve1;	//保留 1
	uint16_t reserve2;	//保留 2
} DeviceConfig_default = { ARDUINO_VERSION , DEFAULT_SLAVE_ID ,DEFAULT_BAUD_ID , 0x0000, 0x0000, INCHING_INTERVAL_MS, 0x0000, 0x0000 };
#pragma pack ()

/// <summary>
/// 设备配置项
/// </summary>
typedef struct DeviceConfig_s DeviceConfig;


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
constexpr unsigned long BAUD_RATE[] = { 9600, 19200, 38400, 115200, 230400, 460800, 921600 };


/// <summary>
/// Reset Arduino
/// </summary>
void (*resetArduino)() = 0x0000;
//void resetArduino() { __asm__ __volatile__("jmp 0"); }


/// <summary>
/// Clear EEPROM
/// </summary>
void clearEEPROM()
{
	for (int i = 0; i < E2END + 1; i++)
	{
		eeprom_write_byte((uint8_t*)i, 0x00);
	}
	delay(200);
}