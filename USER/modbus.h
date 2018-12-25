#ifndef __MODBUS_H__
#define __MODBUS_H__

#include "sys.h"

#define MB_BOARDCAST_ADDR      0
#define MB_MAX_ADDRESS         247

typedef enum
{
    MB_EX_NONE = 0x00,
    MB_EX_ILLEGAL_FUNCTION = 0x01,
    MB_EX_ILLEGAL_DATA_ADDRESS = 0x02,
    MB_EX_ILLEGAL_DATA_VALUE = 0x03,
    MB_EX_SLAVE_DEVICE_FAILURE = 0x04,
    MB_EX_ACKNOWLEDGE = 0x05,
    MB_EX_SLAVE_BUSY = 0x06,
    MB_EX_MEMORY_PARITY_ERROR = 0x08,
    MB_EX_GATEWAY_PATH_FAILED = 0x0A,
    MB_EX_GATEWAY_TGT_FAILED = 0x0B
}eMBException;


/* modbus������
 *---------------------------------------------------------------------
 * ��Ȧ״̬        �ɶ���д     ���������,��:��ŷ������MOSFET���
 * ��ɢ����״̬    �ɶ�         ����������,��:���뿪�أ��ӽ�����
 * ����Ĵ���      �ɶ�         ģ��������,��:ģ��������
 * ���ּĴ���      �ɶ���д     ģ�������,��:ģ��������趨ֵ������ֵ
 *---------------------------------------------------------------------
 */
#define FUNC_READ_COIL_STATUS	 	1    /*--bit ����Ȧ״̬ 	  */
#define FUNC_READ_INPUT_DISCRETE 	2    /*--bit ����ɢ������ 	  */
#define FUNC_READ_HOLDING_REGISTER	3    /* 16bit   �����ּĴ���  */
#define FUNC_READ_INPUT_REGISTER	4    /* 16bit   ������Ĵ���  */

#define FUNC_WRITE_SINGLE_COIL	 	5    /*--bit д������Ȧ 	  */
#define FUNC_WRITE_SINGLE_REGISTER	6    /* 16bit   д�����Ĵ���  */
#define FUNC_WRITE_MULT_COIL		15   /*--bit д�����Ȧ 	  */
#define FUNC_WRITE_MULT_REGISTER	16   /* 16bit   д����Ĵ���  */

#define BigLittleSwap16(A) ((((int16_t)(A)&0xff00) >> 8) | (((int16_t)(A)&0x00ff) << 8))

struct mb_head_t
{
	u8	slave_addr;//�ӻ���ַ
	u8  func_code;
};

/* ������01��02��03��04�����ݸ�ʽ  */
struct func_read_data_t
{
	u16  start_addr;
	u16	 read_cnt;
};

/* ������05��06�����ݸ�ʽ  */
struct func_write_single_data_t
{
	u16 start_addr;
	u16 write_val;
};

/* ������15��16�����ݸ�ʽ  */
struct func_write_muti_data_t
{
	u16 start_addr;
	u16 write_cnt;
	u8  byte_cnt;
	u8  *write_data;
};

#define MB_SIZE_MAX 256

#define TRUE  1
#define FALSE 0

/* modbus working set */
struct mb_ws_t
{
	u8 f_recv_complete;//�Ƿ�������
	u8 f_tx_done;
	
	u32  len_in;//�������ݵĳ���
	u32  len_out;//�������ݵĳ���

	u32  tx_len;//�������ݵĳ���

	u8   pkt_buf[MB_SIZE_MAX]; /* ��˫����ֻ��Ҫһ�������� */
};

/********************�ⲿ���� **************************/
extern struct mb_ws_t  mb_ws;
void  md_pkt_deal(void);
#endif

