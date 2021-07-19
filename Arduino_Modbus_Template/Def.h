#pragma once

#define		ARDUINO_VERSION					0x0101		//�̼��汾�ţ����ֽڱ�ʾ��汾�ţ����ֽ�С�汾��
#define		LED_TRUN_INTERVAL_MS			500			//LED ��ת���ʱ�� ms
#define		DINPUT_SHAKE_INTERVAL_MS		8			//��������������ʱ�� ms
#define		INCHING_INTERVAL_MS				100			//�㶯ģʽ�ĵ�λ���ʱ�� ms


/// <summary>
/// ��ɢ�����ź� �� ��Ȧ����ź� �İ�ģʽ
/// </summary>
enum BindingMode :uint8_t
{
	Default = 0x00,		//���κ�����ģʽ
	Non_Lock = 0x01,	//[����] ����/��� ���� ����ģʽ

	Lock = 0x02,		//[����] ����/��� ���� ����ģʽ

	Only = 0x03,		//[����] ���� ����ģʽ��������һ��
	Inching = 0x04,		//[����] ����/��� �㶯 ����ģʽ
};


//�豸��֧�ֵĲ�����
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