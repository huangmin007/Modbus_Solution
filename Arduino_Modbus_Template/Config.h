#pragma once

/*
	配置设备 ModbusRTU 设备输入输出
	Modbus:	nDO, nDI, nAI(0~5V)
*/

/*	编译前配置 */
//============================ Configuration Begin ===========================

#define		DEFAULT_SLAVE_ID				0x01		//配置设备的默认地址，		//0x01~0xFE
#define		DEFAULT_BAUD_ID					0x00		//配置设备默认的波特率索引	//{ 9600, 19200, 38400, 115200, 230400, 460800, 921600 }
#define		DEFAULT_WORK_MODE				0x00		//配置设备默认的工作模式	//{Default=0x00, Non_Lock=0x01, Lock=0x02, Only=0x03}
#define		DEFAULT_INPUT_INTERVAL			8			//配置离散输入默认去抖间隔时间 ms	//0~5000
#define		DEFAULT_LED_INTERVAL			500			//配置LED默认值闪烁间隔时间 ms，	//0x0000表示常闭，0xFFFF表示常开

#define		CONFIG_USE_WDT					true		//配置是否使用看门狗
#define		CONFIG_USE_COILS				true		//配置是否使用线圈输出（数字输出）
#define		CONFIG_USE_DISCRETE_INPUTS		true		//配置是否使用离散输入（数字输入）
#define		CONFIG_USE_INPUT_REGISTERS		true		//配置是否使用输入寄存器（模拟输入0~5V）
#define		CONFIG_USE_LED_OUTPUT			true		//配置是否使用 LED 引脚做为指示输出，如果使用后，则 LED 引脚不可做为输入/输出


#if CONFIG_USE_COILS
constexpr uint8_t coilPins[] = { 8, 9, 10, 11, 12 };			//配置线圈输出引脚
#endif

#if CONFIG_USE_DISCRETE_INPUTS
#define		DISCRETE_INPUT_MODE				INPUT_PULLUP		//配置引脚输入模式：INPUT, INPUT_PULLUP
constexpr uint8_t discreteInputPins[] = { 3, 4, 5, 6, 7 };		//配置离散输入引脚
#endif

#if CONFIG_USE_INPUT_REGISTERS
constexpr uint8_t inputRegisterPins[] = { A0, A1, A2, A3, A4, A5 };	//配置输入引脚模拟
#endif

#if CONFIG_USE_LED_OUTPUT
//constexpr uint8_t ledPin = 13;								//定义 LED 指示输出引脚
#endif

//================================= Configuration End =====================================

