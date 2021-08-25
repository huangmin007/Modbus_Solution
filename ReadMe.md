- ### Ardrino ModbusRTU Template
  
- #### 未解决问题
> Arduino数字2号引脚，做为离散输入，无法读取引脚状态问题，暂时没找到原因，其它引脚正常

- #### 说明
> - Arduion_Modbus_FreeRTOS 未完成，将来即使完成，只支持 Mage2560 大容量单片机
> - Arduion_Modbus_Template 可做为小容量单片机，自定义 ModbusRTU 产品使用，所开放的引脚由 Config.h 配置
> - 功能支持 nDO, nDI, nAI(0~5V)

- #### 使用
> 1. 下载 [ArduinoModbus](https://github.com/arduino-libraries/ArduinoModbus) 库
> 2. 跟据项目需求，配置 Config.h 头文件
``` C
#define		DEFAULT_SLAVE_ID				0x01		//配置设备的默认地址		//0x01~0xFE
#define		DEFAULT_BAUD_ID					0x00		//配置设备默认的波特率索引	//{ 9600, 19200, 38400, 115200, 230400, 460800, 921600 }

#define		CONFIG_USE_WDT					false		//配置是否使用看门狗
#define		CONFIG_USE_COILS				false		//配置是否使用线圈输出（数字输出信号）
#define		CONFIG_USE_DISCRETE_INPUTS		true		//配置是否使用离散输入（数字输入信号）
#define		CONFIG_USE_INPUT_REGISTERS		false		//配置是否使用输入寄存器（模拟输入0~5V信号）
#define		CONFIG_USE_LED_OUTPUT			true		//配置是否使用 LED 引脚做为指示输出，如果使用后，则 LED 引脚不可做为输入/输出


#if CONFIG_USE_COILS
constexpr uint8_t coilPins[] = { 7, 8, 9 };			//配置线圈输出引脚
#endif

#if CONFIG_USE_DISCRETE_INPUTS
#define		DISCRETE_INPUT_MODE				INPUT_PULLUP			//配置引脚输入模式：INPUT, INPUT_PULLUP
constexpr uint8_t discreteInputPins[] = { 3, 4, 5 };				//配置离散输入引脚，注意不可使用2号引脚做为离散输入
#endif

#if CONFIG_USE_INPUT_REGISTERS
constexpr uint8_t inputRegisterPins[] = { A0, A1, A2, A3, A4, A5 };	//配置输入引脚模拟
#endif
```
> 3. 烧录 Arduion 单片机

- #### 注意
> 1. 注意 Arduion 是5V单片机，所有引脚信号没有隔离电路