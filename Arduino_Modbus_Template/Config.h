#pragma once

/*
	�����豸 ModbusRTU �豸�������
	Modbus:	nDO, nDI, nAI(0~5V)
*/



#define		LED_TRUN_INTERVAL_MS			500			//LED ��ת���ʱ�� ms
#define		DINPUT_SHAKE_INTERVAL_MS		8			//�����ź�����������ʱ�� ms
#define		INCHING_INTERVAL_MS				250			//�㶯ģʽ�ĵ�λ���ʱ�� ms


/*	����ǰ���� */
//============================ Configuration Begin ===========================

#define		DEFAULT_SLAVE_ID				0x01		//�����豸��Ĭ�ϵ�ַ		//0x01~0xFE
#define		DEFAULT_BAUD_ID					0x00		//�����豸Ĭ�ϵĲ���������	//{ 9600, 19200, 38400, 115200, 230400, 460800, 921600 }

#define		CONFIG_USE_WDT					false		//�����Ƿ�ʹ�ÿ��Ź�
#define		CONFIG_USE_COILS				true		//�����Ƿ�ʹ����Ȧ�������������źţ�
#define		CONFIG_USE_DISCRETE_INPUTS		true		//�����Ƿ�ʹ����ɢ���루���������źţ�
#define		CONFIG_USE_INPUT_REGISTERS		true		//�����Ƿ�ʹ������Ĵ�����ģ������0~5V�źţ�
#define		CONFIG_USE_LED_OUTPUT			true		//�����Ƿ�ʹ�� LED ������Ϊָʾ��������ʹ�ú��� LED ���Ų�����Ϊ����/���


#if CONFIG_USE_COILS
constexpr uint8_t coilPins[] = { 8, 9, 10, 11, 12 };			//������Ȧ�������
#endif

#if CONFIG_USE_DISCRETE_INPUTS
#define		DISCRETE_INPUT_MODE				INPUT_PULLUP		//������������ģʽ��INPUT, INPUT_PULLUP
constexpr uint8_t discreteInputPins[] = { 3, 4, 5, 6, 7 };		//������ɢ��������
#endif

#if CONFIG_USE_INPUT_REGISTERS
constexpr uint8_t inputRegisterPins[] = { A0, A1, A2, A3, A4, A5 };	//������������ģ��
#endif

//================================= Configuration End =====================================

