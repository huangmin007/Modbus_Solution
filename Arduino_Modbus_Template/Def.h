#pragma once

#define		ARDUINO_VERSION					0x0101		//固件版本号，高字节表示大版本号，低字节小版本号
#define		LED_TRUN_INTERVAL_MS			500			//LED 翻转间隔时间 ms
#define		DINPUT_SHAKE_INTERVAL_MS		8			//数字输入防抖间隔时间 ms
#define		INCHING_INTERVAL_MS				100			//点动模式的单位间隔时间 ms


/// <summary>
/// 离散输入信号 与 线圈输出信号 的绑定模式
/// </summary>
enum BindingMode :uint8_t
{
	Default = 0x00,		//无任何联运模式
	Non_Lock = 0x01,	//[本机] 输入/输出 非锁 联动模式

	Lock = 0x02,		//[本机] 输入/输出 自锁 联动模式

	Only = 0x03,		//[本机] 互锁 联动模式，整机按一组
	Inching = 0x04,		//[本机] 输入/输出 点动 联动模式
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