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


/* modbus功能码
 *---------------------------------------------------------------------
 * 线圈状态        可读可写     开关量输出,如:电磁阀输出，MOSFET输出
 * 离散输入状态    可读         开关量输入,如:拨码开关，接近开关
 * 输入寄存器      可读         模拟量输入,如:模拟量输入
 * 保持寄存器      可读可写     模拟量输出,如:模拟量输出设定值，参数值
 *---------------------------------------------------------------------
 */
#define FUNC_READ_COIL_STATUS	 	1    /*--bit 读线圈状态 	  */
#define FUNC_READ_INPUT_DISCRETE 	2    /*--bit 读离散量输入 	  */
#define FUNC_READ_HOLDING_REGISTER	3    /* 16bit   读保持寄存器  */
#define FUNC_READ_INPUT_REGISTER	4    /* 16bit   读输入寄存器  */

#define FUNC_WRITE_SINGLE_COIL	 	5    /*--bit 写单个线圈 	  */
#define FUNC_WRITE_SINGLE_REGISTER	6    /* 16bit   写单个寄存器  */
#define FUNC_WRITE_MULT_COIL		15   /*--bit 写多个线圈 	  */
#define FUNC_WRITE_MULT_REGISTER	16   /* 16bit   写多个寄存器  */

#define BigLittleSwap16(A) ((((int16_t)(A)&0xff00) >> 8) | (((int16_t)(A)&0x00ff) << 8))

struct mb_head_t
{
	u8	slave_addr;//从机地址
	u8  func_code;
};

/* 功能码01，02，03，04的数据格式  */
struct func_read_data_t
{
	u16  start_addr;
	u16	 read_cnt;
};

/* 功能码05，06的数据格式  */
struct func_write_single_data_t
{
	u16 start_addr;
	u16 write_val;
};

/* 功能码15，16的数据格式  */
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
	u8 f_recv_complete;//是否接收完成
	u8 f_tx_done;
	
	u32  len_in;//接收数据的长度
	u32  len_out;//发送数据的长度

	u32  tx_len;//发送数据的长度

	u8   pkt_buf[MB_SIZE_MAX]; /* 半双工，只需要一个缓冲区 */
};

/********************外部变量 **************************/
extern struct mb_ws_t  mb_ws;
void  md_pkt_deal(void);
#endif

