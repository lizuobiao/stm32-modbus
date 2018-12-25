#ifndef _CRC_H
#define _CRC_H 	

#define BYTE unsigned char
#define WORD unsigned short int

#define READ_COIL 0x01   //读线圈
#define READ_INPUT 0x02 //读离散量输入
#define READ_HOLDING 0x03 //读保持寄存器
#define READ_INPUT_REG 0x04 //读输入寄存器
#define WRITE_COIL 0x05		//写单个线圈
#define WRITE_REG  0x06		//写单个寄存器

WORD CRC16_1(BYTE* pchMsg, WORD wDataLen);
void Modbus_Tx(BYTE Modbus_Code,WORD Modbus_Addr,WORD Modbus_Data);
int Modbus_Rx_CRC(BYTE* buf,WORD Len);
#endif

