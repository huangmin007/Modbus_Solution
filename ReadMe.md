- ### Ardrino ModbusRTU Template
  
- #### δ�������
> Arduino����2�����ţ���Ϊ��ɢ���룬�޷���ȡ����״̬���⣬��ʱû�ҵ�ԭ��������������

- #### ˵��
> - Arduion_Modbus_FreeRTOS δ��ɣ�������ʹ��ɣ�Ҳֻ��֧�� Mage2560 ��������Ƭ��������� Arduino_Modebus_Template ���ܸ��ḻ
> - Arduion_Modbus_Template ����ΪС������Ƭ�����Զ��� ModbusRTU ��Ʒʹ�ã������ŵ������� Config.h ����
> - ����֧�� nDO, nDI, nAI(0~5V)

- #### ʹ��
> 1. ��װ Arduion IDE, ���� [ArduinoModbus](https://github.com/arduino-libraries/ArduinoModbus) �⣬���Ƶ� Arduion ��װĿ¼/libraries/ 
> 2. ������Ŀ�������� Config.h ͷ�ļ�
``` C
#define		DEFAULT_SLAVE_ID				0x01	        //�����豸��Ĭ�ϵ�ַ	//0x01~0xFE
#define		DEFAULT_BAUD_ID					0x00		//�����豸Ĭ�ϵĲ���������	//{ 9600, 19200, 38400, 115200, 230400, 460800, 921600 }

#define		CONFIG_USE_WDT					false		//�����Ƿ�ʹ�ÿ��Ź�
#define		CONFIG_USE_COILS				true		//�����Ƿ�ʹ����Ȧ�������������źţ�
#define		CONFIG_USE_DISCRETE_INPUTS		true		//�����Ƿ�ʹ����ɢ���루���������źţ�
#define		CONFIG_USE_INPUT_REGISTERS		false		//�����Ƿ�ʹ������Ĵ�����ģ������0~5V�źţ�
#define		CONFIG_USE_LED_OUTPUT			true		//�����Ƿ�ʹ�� LED ������Ϊָʾ��������ʹ�ú��� LED ���Ų�����Ϊ����/���


#if CONFIG_USE_COILS
constexpr uint8_t coilPins[] = { 3, 4, 5, 6, 7 };			//������Ȧ�������
#endif

#if CONFIG_USE_DISCRETE_INPUTS
#define		DISCRETE_INPUT_MODE			INPUT_PULLUP				//������������ģʽ��INPUT, INPUT_PULLUP
constexpr uint8_t discreteInputPins[] = { 8, 9, 10, 11, 12};		//������ɢ�������ţ�ע�ⲻ��ʹ�� 2 ��������Ϊ��ɢ����
#endif

#if CONFIG_USE_INPUT_REGISTERS
constexpr uint8_t inputRegisterPins[] = { A0, A1, A2, A3, A4, A5 };	//������������ģ��
```
> 3. ��¼ Arduion ��Ƭ��

- #### ע��
> 1. ע�� Arduion ��5V��Ƭ�������������ź�û�и����·