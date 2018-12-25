#include  "modbus.h"
#include  "usart.h"	 
#include <string.h>

static const u8 aucCRCHi[] = {
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40
};

static const u8 aucCRCLo[] = {
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7,
    0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E,
    0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9,
    0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC,
    0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
    0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32,
    0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D,
    0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 
    0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF,
    0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1,
    0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4,
    0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 
    0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA,
    0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
    0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0,
    0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97,
    0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E,
    0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89,
    0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83,
    0x41, 0x81, 0x80, 0x40
};

//16位crc校验码的生成
u16 mb_crc16( u8  * pucFrame, int usLen )
{
    u8            ucCRCHi = 0xFF;
    u8            ucCRCLo = 0xFF;
    int           iIndex;

    while( usLen-- )
    {
        iIndex = ucCRCLo ^ *( pucFrame++ );
        ucCRCLo = ( u8  )( ucCRCHi ^ aucCRCHi[iIndex] );
        ucCRCHi = aucCRCLo[iIndex];
    }
    return ( u16  )( ucCRCHi << 8 | ucCRCLo );
}

#define HEAD_LEN  	sizeof(struct mb_head_t)
#define MB_CRC_LEN	2

struct mb_ws_t  mb_ws;//串口数据结构体

u32	mb_address = 0x02; /* 板modbus地址 */

//设置modbus地址
void  mb_address_set(u32 address)
{
	mb_address = address;
}
//获取modbus地址
u32  mb_address_get(void)
{
	return mb_address;
}

/*    requset(请求)     |    response(回复)
 * -------------------------------------------
 *  slave_address  		|  slave_address(从机地址)
 *  func_code      		|  func_code(功能码)
 *  start_address(hi)   |  byte_count (数据的多少)
 *  start_address(lo)   |  data(coil)
 *  read_cnt(hi)        |   ...
 *  read_cnt(lo)        |  crc16(lo)  
 *  crc16(lo)           |  crc16(hi)
 *  crc16(hi)           |
 */
#define MB_READ_COIL_SIZE  	8

eMBException mb_read_coil_resp(struct mb_ws_t *mb_ws_ptr)
{
	u16 			 start_address;
	u16              read_cnt;
	eMBException     status = MB_EX_NONE;
	u8				 byte_cnt;
	struct func_read_data_t *func_read_data;

	/* 数据长度不符合命令格式，返回错误 */
	if (mb_ws.len_in != MB_READ_COIL_SIZE)
	{
		status = MB_EX_ILLEGAL_DATA_VALUE;
	}
	else
	{
		func_read_data = (struct func_read_data_t *)
		                    (&mb_ws_ptr->pkt_buf[HEAD_LEN]);//读取功能码和开始地址

		start_address = BigLittleSwap16(func_read_data->start_addr);//转化为16位的地址和功能码
		read_cnt      = BigLittleSwap16(func_read_data->read_cnt);
		
		byte_cnt = (read_cnt +7)/8;
		/* 响应返回数据字节个数 */
		mb_ws_ptr->pkt_buf[2] = byte_cnt;
		/* 发送数据区清0 */
		memset(mb_ws_ptr->pkt_buf+3, 0 , byte_cnt);
		
		mb_ws_ptr->tx_len = 3+byte_cnt;
	}

	return status;
}

/*    requset           |    response
 * -------------------------------------------
 *  slave_address  		|  slave_address
 *  func_code      		|  func_code
 *  start_address(hi)   |  byte_count 
 *  start_address(lo)   |  data holding regiseter{hi)
 *  read_cnt(hi)        |  data holding regiseter{lo)
 *  read_cnt(lo)        |  ...
 *  crc16(lo)           |  crc16(lo) 
 *  crc16(hi)           |  crc16(hi)
 */
#define MB_READ_HOLDING_REG_SIZE  	8

eMBException mb_read_holding_register_resp(struct mb_ws_t *mb_ws_ptr)
{
	//int              i;
	u16 			 start_address;
	u16              read_cnt;
	eMBException     status = MB_EX_NONE;
	u8				 byte_cnt;
	struct func_read_data_t *func_read_data;

	/* 数据长度不符合命令格式，返回错误 */
	if (mb_ws.len_in != MB_READ_HOLDING_REG_SIZE)
	{
		status = MB_EX_ILLEGAL_DATA_VALUE;
	}
	else
	{
		func_read_data = (struct func_read_data_t *)
		                     (&mb_ws_ptr->pkt_buf[HEAD_LEN]);

		start_address = BigLittleSwap16(func_read_data->start_addr);
		read_cnt      = BigLittleSwap16(func_read_data->read_cnt);
		
		byte_cnt = read_cnt*2;
		/* 响应返回数据字节个数 */
		mb_ws_ptr->pkt_buf[2] = byte_cnt;
		/* 发送数据区清0 */
		memset(mb_ws_ptr->pkt_buf+3, 0 , byte_cnt);

//		status =  mb_rw_hoiding_regiseter_cb(mb_ws_ptr->pkt_buf+3,
//		              						 start_address,
//		              						 read_cnt,
//		              						 MB_REG_READ);
		
		mb_ws_ptr->tx_len = 3+byte_cnt;
	}

	return status;
}

/*    requset           |    response
 * -------------------------------------------
 *  slave_address  		|  slave_address
 *  func_code      		|  func_code
 *  start_address(hi)   |  byte_count 
 *  start_address(lo)   |  data(discrete input)
 *  read_cnt(hi)        |   ...
 *  read_cnt(lo)        |  crc16(lo)  
 *  crc16(lo)           |  crc16(hi)
 *  crc16(hi)           |
 */
#define MB_READ_INPUT_DISCRETE_SIZE  	8

eMBException mb_read_input_discrete_resp(struct mb_ws_t *mb_ws_ptr)
{
	//int              i;
	u16 			 start_address;
	u16              read_cnt;
	eMBException     status = MB_EX_NONE;
	u8				 byte_cnt;
	struct func_read_data_t *func_read_data;

	/* 数据长度不符合命令格式，返回错误 */
	if (mb_ws.len_in != MB_READ_INPUT_DISCRETE_SIZE)
	{
		status = MB_EX_ILLEGAL_DATA_VALUE;
	}
	else
	{
		func_read_data = (struct func_read_data_t *)
		                    (&mb_ws_ptr->pkt_buf[HEAD_LEN]);

		start_address = BigLittleSwap16(func_read_data->start_addr);
		read_cnt      = BigLittleSwap16(func_read_data->read_cnt);
		
		byte_cnt = (read_cnt +7)/8;
		/* 响应返回数据字节个数 */
		mb_ws_ptr->pkt_buf[2] = byte_cnt;
		/* 发送数据区清0 */
		memset(mb_ws_ptr->pkt_buf+3, 0 , byte_cnt);

//		status =  mb_input_discrete_cb(mb_ws_ptr->pkt_buf+3,
//		              				   start_address,
//		              				   read_cnt);
		
		mb_ws_ptr->tx_len = 3+byte_cnt;
	}

	return status;
}

/*    requset           |    response
 * -------------------------------------------
 *  slave_address  		|  slave_address
 *  func_code      		|  func_code
 *  start_address(hi)   |  start_address(hi) 
 *  start_address(lo)   |  start_address(lo)
 *  force data(hi)      |  force data(hi)
 *  force data(lo)      |  force data(lo)
 *  crc16(lo)           |  crc16(lo) 
 *  crc16(hi)           |  crc16(hi)
 */
#define MB_WRITE_SINGLE_COIL_SIZE  	8

eMBException mb_write_single_coil_resp(struct mb_ws_t *mb_ws_ptr)
{
	//int              i;
	u16 			 start_address;
  u16              write_val;
	eMBException     status = MB_EX_NONE;
	u8				 byte_cnt;
	struct func_write_single_data_t  *func_write_single_data;

	/* 数据长度不符合命令格式，返回错误 */
	if (mb_ws.len_in != MB_WRITE_SINGLE_COIL_SIZE)
	{
		status = MB_EX_ILLEGAL_DATA_VALUE;
	}
	else
	{
		func_write_single_data = (struct func_write_single_data_t *)\
		                            (&mb_ws_ptr->pkt_buf[HEAD_LEN]);

		start_address = BigLittleSwap16(func_write_single_data->start_addr);
		write_val     = BigLittleSwap16(func_write_single_data->write_val);
		
//		status =  mb_rw_coil_cb(mb_ws_ptr->pkt_buf+4,
//		              			start_address,
//		              			1,
//		              			MB_REG_WRITE);
		
		mb_ws_ptr->tx_len = mb_ws_ptr->len_in-2;
	}

	return status;
}

/*    requset           |    response
 * -------------------------------------------
 *  slave_address  		|  slave_address
 *  func_code      		|  func_code
 *  start_address(hi)   |  start_address(hi) 
 *  start_address(lo)   |  start_address(lo)
 *  reg  num(hi)        |  reg num(hi)
 *  reg  num(lo)        |  reg num(lo)
 *  byte count          |  crc16(lo) 
 *  data(hi)            |  crc16(hi)
 *  data(lo)            |
 *  ...                 |
 *  crc16(lo)           |
 *  crc16(hi)           |
 */
#define MB_WRITE_MUTI_REG_SIZE_MIN  	11

eMBException mb_write_muti_register_resp(struct mb_ws_t *mb_ws_ptr)
{
	//int              i;
	u16 			 start_address;
    u16              write_cnt;
    u8               byte_cnt;
	eMBException     status = MB_EX_NONE;
	struct func_write_muti_data_t *func_write_muti_data;

	/* 数据长度不符合命令格式，返回错误 */
	if (mb_ws.len_in < MB_WRITE_MUTI_REG_SIZE_MIN)
	{
		status = MB_EX_ILLEGAL_DATA_VALUE;
	}
	else
	{
		func_write_muti_data = (struct func_write_muti_data_t *)\
		                          (&mb_ws_ptr->pkt_buf[HEAD_LEN]);

		start_address = BigLittleSwap16(func_write_muti_data->start_addr);
		write_cnt     = BigLittleSwap16(func_write_muti_data->write_cnt);
		byte_cnt      = func_write_muti_data->byte_cnt;

		if (write_cnt == byte_cnt/2)
		{
//			status =  mb_rw_hoiding_regiseter_cb(mb_ws_ptr->pkt_buf+7,
//			              						start_address,
//			              						write_cnt,
//			              						MB_REG_WRITE);
			
			mb_ws_ptr->tx_len = 6;
		}
		else
		{
			status = MB_EX_ILLEGAL_DATA_VALUE;
		}
	}

	return status;
}


/*    requset           |    response
 * -------------------------------------------
 *  slave_address  		|  slave_address
 *  func_code      		|  func_code
 *  start_address(hi)   |  start_address(hi) 
 *  start_address(lo)   |  start_address(lo)
 *  data(hi)            |  data(hi)
 *  data(lo)            |  data(lo)
 *  crc16(lo)           |  crc16(lo) 
 *  crc16(hi)           |  crc16(hi)
 */
#define MB_WRITE_SINGLE_REGISTER_SIZE  	8

eMBException mb_write_single_register_resp(struct mb_ws_t *mb_ws_ptr)
{
	//int              i;
	u16 			 start_address;
    u16              write_val;
	eMBException     status = MB_EX_NONE;

	struct func_write_single_data_t  *func_write_single_data;

	/* 数据长度不符合命令格式，返回错误 */
	if (mb_ws.len_in != MB_WRITE_SINGLE_REGISTER_SIZE)
	{
		status = MB_EX_ILLEGAL_DATA_VALUE;
	}
	else
	{
		func_write_single_data = (struct func_write_single_data_t *)\
		                            (&mb_ws_ptr->pkt_buf[HEAD_LEN]);

		start_address = BigLittleSwap16(func_write_single_data->start_addr);
		write_val     = BigLittleSwap16(func_write_single_data->write_val);
		
//		status =  mb_rw_hoiding_regiseter_cb(mb_ws_ptr->pkt_buf+4,
//		              			start_address,
//		              			1,
//		              			MB_REG_WRITE);
		
		mb_ws_ptr->tx_len = mb_ws_ptr->len_in-2;
	}

	return status;
}

/*    requset           |    response
 * -------------------------------------------
 *  slave_address  		|  slave_address
 *  func_code      		|  func_code
 *  start_address(hi)   |  start_address(hi) 
 *  start_address(lo)   |  start_address(lo)
 *  coil num(hi)        |  coil num(hi)
 *  coil num(lo)        |  coil num(lo)
 *  byte count          |  crc16(lo) 
 *  data(N)             |  crc16(hi)
 *  ...                 |
 *  crc16(lo)           |
 *  crc16(hi)           |
 */
#define MB_WRITE_MUTI_COIL_SIZE_MIN  	10

eMBException mb_write_muti_coil_resp(struct mb_ws_t *mb_ws_ptr)
{
	//int              i;
	u16 			 start_address;
    u16              write_cnt;
    u8               byte_cnt;
	eMBException     status = MB_EX_NONE;
	struct func_write_muti_data_t *func_write_muti_data;

	/* 数据长度不符合命令格式，返回错误 */
	if (mb_ws.len_in < MB_WRITE_MUTI_COIL_SIZE_MIN)
	{
		status = MB_EX_ILLEGAL_DATA_VALUE;
	}
	else
	{
		func_write_muti_data = (struct func_write_muti_data_t *)\
		                          (&mb_ws_ptr->pkt_buf[HEAD_LEN]);

		start_address = BigLittleSwap16(func_write_muti_data->start_addr);
		write_cnt     = BigLittleSwap16(func_write_muti_data->write_cnt);
		byte_cnt      = func_write_muti_data->byte_cnt;

		if ((write_cnt+7)/8 == byte_cnt)
		{
//			status =  mb_rw_coil_cb(mb_ws_ptr->pkt_buf+7,
//			              			start_address,
//			              			write_cnt,
//			              			MB_REG_WRITE);
			
			mb_ws_ptr->tx_len = 6;
		}
		else
		{
			status = MB_EX_ILLEGAL_DATA_VALUE;
		}
	}

	return status;
}

/*    requset           |    response
 * -------------------------------------------
 *  slave_address  		|  slave_address
 *  func_code      		|  func_code
 *  start_address(hi)   |  byte_count 
 *  start_address(lo)   |  data input regiseter{hi)
 *  read_cnt(hi)        |  data input regiseter{lo)
 *  read_cnt(lo)        |  ...
 *  crc16(lo)           |  crc16(lo) 
 *  crc16(hi)           |  crc16(hi)
 */
#define MB_READ_INPUT_DISCRETE_SIZE  	8

eMBException mb_read_input_register_resp(struct mb_ws_t *mb_ws_ptr)
{
	//int              i;
	u16 			 start_address;
	u16              read_cnt;
	eMBException     status = MB_EX_NONE;
	u8				 byte_cnt;
	struct func_read_data_t *func_read_data;

	/* 数据长度不符合命令格式，返回错误 */
	if (mb_ws.len_in != MB_READ_INPUT_DISCRETE_SIZE)
	{
		status = MB_EX_ILLEGAL_DATA_VALUE;
	}
	else
	{
		func_read_data = (struct func_read_data_t *)
		                      (&mb_ws_ptr->pkt_buf[HEAD_LEN]);

		start_address = BigLittleSwap16(func_read_data->start_addr);
		read_cnt      = BigLittleSwap16(func_read_data->read_cnt);
		
		byte_cnt = read_cnt*2;
		/* 响应返回数据字节个数 */
		mb_ws_ptr->pkt_buf[2] = byte_cnt;
		/* 发送数据区清0 */
		memset(mb_ws_ptr->pkt_buf+3, 0 , byte_cnt);

//		status =  mb_input_register_cb(mb_ws_ptr->pkt_buf+3,
//		              						 start_address,
//		              						 read_cnt);
		
		mb_ws_ptr->tx_len = 3+byte_cnt;
	}

	return status;
}

/*
 * Modbus包处理函数
 */
u16 crc = 0;
void  md_pkt_deal(void)
{
		u16             crc16_val;
		eMBException 	  exception;
		struct mb_head_t *mb_head_ptr;
		
		if (mb_ws.f_recv_complete != TRUE)
			return;
		mb_ws.f_recv_complete = FALSE;
		

		mb_head_ptr = (struct mb_head_t *)mb_ws.pkt_buf;

		if (mb_address == 33)
		{
				return;
		}

		/* 地址不匹配返回 */
		if (mb_head_ptr->slave_addr != mb_address &&
				mb_head_ptr->slave_addr != MB_BOARDCAST_ADDR)
		{
				return;
		}

		/* crc check, 如果CRC检验不成功，响应一个异常包 */
		crc = mb_crc16(mb_ws.pkt_buf, mb_ws.len_in);
		if ( mb_crc16(mb_ws.pkt_buf, mb_ws.len_in) != 0)
		{
				exception = MB_EX_MEMORY_PARITY_ERROR;
		}
		/* 包校验通过 */
		else
		{
				switch (mb_head_ptr->func_code)
				{
					case FUNC_READ_COIL_STATUS: 		/*--bit  1 读线圈状态 	  */
						exception = mb_read_coil_resp(&mb_ws);
						break;
					case FUNC_READ_INPUT_DISCRETE: 		/*--bit  2 读离散量输入  */
						exception = mb_read_input_discrete_resp(&mb_ws);
						break;
					case FUNC_READ_HOLDING_REGISTER:  	/* 16bit 3 读保持寄存器  */
						exception = mb_read_holding_register_resp(&mb_ws);
						break;
					case FUNC_READ_INPUT_REGISTER: 	 	/* 16bit 4 读输入寄存器  */
						exception = mb_read_input_register_resp(&mb_ws);
						break;
					case FUNC_WRITE_SINGLE_COIL: 		/*--bit  5 写单个线圈 	  */
						exception = mb_write_single_coil_resp(&mb_ws);
						break;
					case FUNC_WRITE_SINGLE_REGISTER: 	/* 16bit 6 写单个寄存器  */
						exception = mb_write_single_register_resp(&mb_ws);
						break;
					case FUNC_WRITE_MULT_COIL:  		/*--bit  15 写多个线圈 	  */
						exception = mb_write_muti_coil_resp(&mb_ws);
						break;
					case FUNC_WRITE_MULT_REGISTER:  	/* 16bit 16 写多个寄存器  */
						exception = mb_write_muti_register_resp(&mb_ws);
						break;
					default:
						exception = MB_EX_ILLEGAL_FUNCTION;
						break;
				}
			}

		/* 包解析出现异常 */
		if (exception != MB_EX_NONE)
		{
				mb_ws.pkt_buf[1] |=  0x80;
				mb_ws.pkt_buf[2]  =  exception;
				mb_ws.tx_len= 3;
		}

		/* add crc16 check to packet buffer */
		crc16_val = mb_crc16(mb_ws.pkt_buf, mb_ws.tx_len);	
		
		mb_ws.pkt_buf[mb_ws.tx_len] = (u8)crc16_val;
		mb_ws.pkt_buf[mb_ws.tx_len+1] = (u8)(crc16_val>>8);
		
		mb_ws.tx_len += MB_CRC_LEN;

		USART1_Send_Data(mb_ws.pkt_buf,mb_ws.tx_len);
		
}

