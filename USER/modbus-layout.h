#ifndef __MODBUS_LAYOUT_H__
#define __MODBUS_LAYOUT_H__

#include "sys.h"

#include "modbus.h"

#define  MB_COIL_BASE				10000
#define  MB_INPUT_REGISTER_BASE  	20000
#define  MB_INPUT_DISCRETE_BASE     0
#define  MB_HOLDING_REGISTER_BASE   30000

#define  MB_COIL_SIZE_MAX           100 //bit
#define  MB_INPUT_REGISTER_MAX      200 // 16bit
#define  MB_INPUT_DISCRETE_MAX      200 //bit
#define  MB_HOLDING_REGISTER_MAX    200 // 16bit

typedef enum
{
	MB_REG_READ  =0,     /*!< Read register values */
	MB_REG_WRITE =1    /*!< Update register values. */
}MB_RW_MODE;

struct mb_range_t
{
	u16 *data;
	u16  base;
	u16  size;
};


void set_temp_adc(u16 temp,u16 adc_value);
extern eMBException mb_input_register_cb(u8 * buf, u16 start_address, u16 reg_num);
#endif

