#include "ModbusSerial.h"

ModbusSerial::ModbusSerial()
{

}
ModbusSerial::~ModbusSerial()
{

}

void ModbusSerial::config(SoftwareSerial* port, uint32_t baud)
{
	port->begin(baud);
	_port = port;
	//(*_port).begin(baud);
	delay(1000);

	_t15 = baud > 19200 ? 750 : 15000000 / baud;	// 1T * 1.5 = T1.5
	_t35 = baud > 19200 ? 1750 : 35000000 / baud;	// 1T * 3.5 = T3.5
}
void ModbusSerial::config(HardwareSerial* port, uint32_t baud)
{
	port->begin(baud);
	_port = port;
	//(*_port).begin(baud);
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
bool ModbusSerial::writeSingleCoil(uint8_t slave_addr, uint16_t coil_start_addr, bool value)
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
	while (_len != 8)
	{
		if (millis() - _tms > _timeout)
		{
			_mb_status = MB_READ_TIMEOUT;
			return false;
		}
		if ((*_port).available() <= 0) continue;
		_receiveBuffer[_len++] = (uint8_t)(*_port).read();
	}
	if (memcmp(_receiveBuffer, _sendBuffer, 6) != 0)
	{
		_mb_status = MB_FRAME_ERROR;
		return false;
	}

	_crc = _receiveBuffer[_len - 2] << 8 | _receiveBuffer[_len - 1];
	_mb_status = _crc == crc16(_receiveBuffer, _len - 2) ? MB_NO_ERROR : MB_CRC_ERROR;
	return _mb_status == MB_NO_ERROR;
}

/// <summary>
/// 0x0F д�����Ȧ, λ����, ���
/// </summary>
/// <param name="slave_addr"></param>
/// <param name="coil_start_addr"></param>
/// <param name="value"></param>
/// <param name="length"></param>
/// <returns></returns>
bool ModbusSerial::writeMultipleCoils(uint8_t slave_addr, uint16_t coil_start_addr, const bool value[], uint8_t length)
{
	uint16_t bool_value = 0x00;
	for (uint8_t i = 0; i < length; i++)
		bitWrite(bool_value, i, value[i]);

	return writeMultipleCoils(slave_addr, coil_start_addr, length, bool_value);
}
/// <summary>
/// 0x0F д�����Ȧ, λ����, ���
/// </summary>
/// <param name="slave_addr"></param>
/// <param name="coil_start_addr"></param>
/// <param name="value">�����д 8 ��������Ȧ</param>
/// <returns></returns>
bool ModbusSerial::writeMultipleCoils(uint8_t slave_addr, uint16_t coil_start_addr, uint16_t coil_mb, uint16_t value)
{
	//send
	_sendBuffer[0] = slave_addr;		//��ַ(�豸ID)
	_sendBuffer[1] = (uint8_t)0x0F;		//������

	//��д�����ʼ��Ȧ��ַ
	_sendBuffer[2] = (uint8_t)coil_start_addr >> 8;
	_sendBuffer[3] = (uint8_t)coil_start_addr & 0xFF;

	if (coil_mb > 16)
	{
		_mb_status = MB_OUT_OF_RANGE;
		return false;
	}

	//��д�����Ȧ������
	_sendBuffer[4] = coil_mb >> 8;
	_sendBuffer[5] = coil_mb & 0xFF;

	//����д�����ݵ��ֽ���
	_sendBuffer[6] = ceil(coil_mb / 8.0);
	if (_sendBuffer[6] > 2) return -2;

	//�Ĵ�����ֵ
	if (_sendBuffer[6] == 1)
	{
		_sendBuffer[7] = value & 0xFF;

		//ѭ������У��
		_crc = crc16(_sendBuffer, 8);
		_sendBuffer[8] = _crc >> 8;
		_sendBuffer[9] = _crc & 0xFF;
	}
	else if (_sendBuffer[6] == 2)
	{
		_sendBuffer[7] = value >> 8;
		_sendBuffer[8] = value & 0xFF;

		//ѭ������У��
		_crc = crc16(_sendBuffer, 9);
		_sendBuffer[9] = _crc >> 8;
		_sendBuffer[10] = _crc & 0xFF;
	}

	(*_port).write(_sendBuffer, _sendBuffer[6] == 1 ? 10 : 11);
	(*_port).flush();
	delayMicroseconds(_t35);

	//receive
	_len = 0;
	_tms = millis();
	while (_len != 8)
	{
		if (millis() - _tms > _timeout)
		{
			_mb_status = MB_READ_TIMEOUT;
			return false;
		}
		if ((*_port).available() <= 0) continue;
		_receiveBuffer[_len++] = (uint8_t)(*_port).read();
	}
	
	if (memcmp(_receiveBuffer, _sendBuffer, 6) != 0)
	{
		_mb_status = MB_FRAME_ERROR;
		return false;
	}

#if false
	for (uint8_t i = 0; i < _len; i++)
	{
		Serial.print(_receiveBuffer[i], HEX);
		Serial.print(",");
	}
	Serial.println();
#endif

	_crc = _receiveBuffer[_len - 2] << 8 | _receiveBuffer[_len - 1];
	_mb_status = _crc == crc16(_receiveBuffer, _len - 2) ? MB_NO_ERROR : MB_CRC_ERROR;
	return _mb_status == MB_NO_ERROR;
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
	//send
	_sendBuffer[0] = slave_addr;		//��ַ(�豸ID)
	_sendBuffer[1] = (uint8_t)0x01;		//������

	//��Ȧ����ʼ��ַ
	_sendBuffer[2] = (uint8_t)coil_start_addr >> 8;
	_sendBuffer[3] = (uint8_t)coil_start_addr & 0xFF;

	if (coil_mb > 16)
	{
		_mb_status = MB_OUT_OF_RANGE;
		return 0;
	}

	//��ѯ��Ȧ������
	_sendBuffer[4] = (uint8_t)coil_mb >> 8;
	_sendBuffer[5] = (uint8_t)coil_mb & 0xFF;

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
	uint8_t rec_len = 5 + ceil(coil_mb / 8.0);
	while (_len != rec_len)
	{
		if (millis() - _tms > _timeout)
		{
			_mb_status = MB_READ_TIMEOUT;
			return 0;
		}
		if ((*_port).available() <= 0) continue;
		_receiveBuffer[_len++] = (uint8_t)(*_port).read();
	}

	if (memcmp(_receiveBuffer, _sendBuffer, 2) != 0)
	{
		_mb_status = MB_FRAME_ERROR;
		return 0;
	}
	_crc = _receiveBuffer[_len - 2] << 8 | _receiveBuffer[_len - 1];
	if (_crc != crc16(_receiveBuffer, _len - 2))
	{
		_mb_status = MB_CRC_ERROR;
		return 0;
	}

	_mb_status = MB_NO_ERROR;
	if (_receiveBuffer[2] == 1) return _receiveBuffer[3];
	else if (_receiveBuffer[2] == 2) return (uint16_t)(_receiveBuffer[3] << 8 | _receiveBuffer[4]);
	else return 0;
}
