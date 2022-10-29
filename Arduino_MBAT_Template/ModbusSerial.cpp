#include "ModbusSerial.h"

ModbusSerial::ModbusSerial()
{

}
ModbusSerial::~ModbusSerial()
{

}

bool ModbusSerial::send(uint8_t* frame, uint8_t length)
{
	(*_port).write(frame, length);
	(*_port).flush();
}

bool ModbusSerial::receive(uint8_t* frame)
{

}

bool ModbusSerial::config(SoftwareSerial* port, long baud)
{
	_port = port;
	(*_port).begin(baud);
	delay(1000);

	_t15 = baud > 19200 ? 750 : 15000000 / baud;	// 1T * 1.5 = T1.5
	_t35 = baud > 19200 ? 1750 : 35000000 / baud;	// 1T * 3.5 = T3.5
}

/// <summary>
/// 0x05 д������Ȧ(��Ȧ��ʾ�����������IO����), λ����, ����
/// </summary>
/// <param name="slave_addr"></param>
/// <param name="coil_start_addr"></param>
/// <param name="value"></param>
/// <returns></returns>
int8_t ModbusSerial::writeSingleCoil(uint8_t slave_addr, uint16_t coil_start_addr, bool value)
{
	//send
	_sendBuffer[0] = slave_addr;		//��ַ(�豸ID)
	_sendBuffer[1] = (uint8_t)0x05;		//������

	//��д�����Ȧ��ַ
	_sendBuffer[2] = (uint8_t)coil_start_addr >> 8;
	_sendBuffer[3] = (uint8_t)coil_start_addr & 0xFF;

	//��Ȧ��ַ��ֵ
	_sendBuffer[4] = value ? 0xFF : 0x00;
	_sendBuffer[5] = (uint8_t)0x00;

	//ѭ������У��
	_crc = crc16(_sendBuffer, 6);
	_sendBuffer[6] = _crc >> 8;
	_sendBuffer[7] = _crc & 0xFF;

	(*_port).write(_sendBuffer, 8);
	(*_port).flush();
	delayMicroseconds(_t35);

	//receive
	_len = 0;
	_tms = millis();
	while (_len == 0)
	{
		if (millis() - _tms > _timeout)
		{
			return -1;
		}

		_len = (*_port).available();
		delayMicroseconds(_t15);
	}
	if (_len == 0) return 0;

	(*_port).readBytes(_receiveBuffer, _len);
	_crc = _receiveBuffer[_len - 2] << 8 | _receiveBuffer[_len - 1];

#if false
	for (uint8_t i = 0; i < _len; i++)
	{
		Serial.print(_receiveBuffer[i], HEX);
		Serial.print(",");
	}
	Serial.println();
#endif

	
	return _crc == crc16(_receiveBuffer, _len - 2);
}

/// <summary>
/// 0x0F д�����Ȧ, λ����, ���
/// </summary>
/// <param name="slave_addr"></param>
/// <param name="coil_start_addr"></param>
/// <param name="value">�����д 8 ��������Ȧ</param>
/// <returns></returns>
int8_t ModbusSerial::writeMultipleCoils(uint8_t slave_addr, uint16_t coil_start_addr, uint16_t coil_mb, uint8_t value)
{
	//send
	_sendBuffer[0] = slave_addr;		//��ַ(�豸ID)
	_sendBuffer[1] = (uint8_t)0x0F;		//������

	//��д�����ʼ��Ȧ��ַ
	_sendBuffer[2] = (uint8_t)coil_start_addr >> 8;
	_sendBuffer[3] = (uint8_t)coil_start_addr & 0xFF;

	//��д�����Ȧ������
	_sendBuffer[4] = coil_mb >> 8;
	_sendBuffer[5] = coil_mb & 0xFF;

	//����д�����ݵ��ֽ���
	_sendBuffer[6] = 0x01;

	//�Ĵ�����ֵ
	_sendBuffer[7] = value;// >> 8;
	//_sendBuffer[8] = value & 0xFF;

	//ѭ������У��
	_crc = crc16(_sendBuffer, 8);
	_sendBuffer[8] = _crc >> 8;
	_sendBuffer[9] = _crc & 0xFF;

	(*_port).write(_sendBuffer, 10);
	(*_port).flush();
	delayMicroseconds(_t35);

	//receive
	_len = 0;
	_tms = millis();
	while (_len == 0)
	{
		if (millis() - _tms > _timeout)
		{
			return -1;
		}

		_len = (*_port).available();
		delayMicroseconds(_t15);
	}
	if (_len == 0) return 0;

	(*_port).readBytes(_receiveBuffer, _len);

#if true
	for (uint8_t i = 0; i < _len; i++)
	{
		Serial.print(_receiveBuffer[i], HEX);
		Serial.print(",");
	}
	Serial.println();
#endif

	_crc = _receiveBuffer[_len - 2] << 8 | _receiveBuffer[_len - 1];
	return _crc == crc16(_receiveBuffer, _len - 2);
}


/// <summary>
/// 0x01 ����Ȧ״̬, λ����, ��������
/// </summary>
/// <param name="slave_addr"></param>
/// <param name="coil_start_addr"></param>
/// <param name="mb"></param>
/// <returns></returns>
uint16_t ModbusSerial::readCoils(uint8_t slave_addr, uint16_t coil_start_addr, uint16_t coil_mb)
{

}
