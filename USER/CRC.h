#ifndef _CRC_H
#define _CRC_H 	

#define BYTE unsigned char
#define WORD unsigned short int

#define READ_COIL 0x01   //����Ȧ
#define READ_INPUT 0x02 //����ɢ������
#define READ_HOLDING 0x03 //�����ּĴ���
#define READ_INPUT_REG 0x04 //������Ĵ���
#define WRITE_COIL 0x05		//д������Ȧ
#define WRITE_REG  0x06		//д�����Ĵ���

WORD CRC16_1(BYTE* pchMsg, WORD wDataLen);
void Modbus_Tx(BYTE Modbus_Code,WORD Modbus_Addr,WORD Modbus_Data);
int Modbus_Rx_CRC(BYTE* buf,WORD Len);
#endif

