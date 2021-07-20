#pragma once

#define		ARDUINO_VERSION				0x0101		//�̼��汾�ţ����ֽڱ�ʾ��汾�ţ����ֽ�С�汾��
#define		CONFIG_REGISTER_COUNT		8			//����������������

#pragma pack (1)
struct DeviceConfig_s
{
	uint16_t version;	//�̼��汾��
	uint16_t slaveId;	//�豸��ַ
	uint16_t baudId;	//������ID
	uint16_t reset;		//�����豸
	uint16_t runCount;	//���д���
	uint16_t reserve0;	//���� 0
	uint16_t reserve1;	//���� 1
	uint16_t reserve2;	//���� 2
} DeviceConfig_default = { ARDUINO_VERSION , DEFAULT_SLAVE_ID ,DEFAULT_BAUD_ID , 0x0000, 0x0000, INCHING_INTERVAL_MS, 0x0000, 0x0000 };
#pragma pack ()

/// <summary>
/// �豸������
/// </summary>
typedef struct DeviceConfig_s DeviceConfig;


/// <summary>
/// ��ɢ�����ź� �� ��Ȧ����ź� �İ�ģʽ
/// </summary>
enum BindingMode :uint8_t
{
	Default = 0x00,		//���κ�����ģʽ
	Non_Lock = 0x01,	//[����] ����/��� ���� ����ģʽ
	Lock = 0x02,		//[����] ����/��� ���� ����ģʽ
	Inching = 0x03,		//[����] ����/��� �㶯 ����ģʽ
	Only = 0x04,		//[����] ���� ����ģʽ��������һ�� δʵ��
};


//�豸��֧�ֵĲ�����
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
	for (int i = 0; i < 512/*E2END + 1*/; i++)
	{
		eeprom_write_byte((uint8_t*)i, 0x00);
	}
	delay(200);
}