#include "modbus-layout.h"


int  mb_range_hit(u32 addr, u32 count, struct mb_range_t *range_ptr, u32 array_count)
{
	int i;

	if(count <= 0) 
		return -1;

	for (i=0; i<array_count; i++)
	{
		if ((addr >= range_ptr[i].base) && 
		    (addr+count) <= (range_ptr[i].base + range_ptr[i].size))
		{
			return i;
		}
	}

	return -1;
}

#define  IR_ANALOG_INPUT_BASE     100
#define  MAX_AI 4
u16 ai_data[MAX_AI] = {0x1111,0x2222,0x3333,0x4444};
struct mb_range_t  ir_array[] = 
{
  	{(u16 *)&ai_data[1], IR_ANALOG_INPUT_BASE,  MAX_AI-2},
};
#define IR_ARRAY_SIZE  (sizeof(ir_array)/sizeof(struct mb_range_t))

eMBException mb_input_register_cb(u8 * buf, u16 start_address, u16 reg_num)
{
	int i;
	int addr_offset;
	int array_index;
	u16 tmp;

	array_index = mb_range_hit(start_address, reg_num, ir_array, IR_ARRAY_SIZE);

	if (array_index == -1)
		return MB_EX_ILLEGAL_DATA_ADDRESS;

	for (i=0; i<reg_num; i++)
	{
		addr_offset = (start_address-ir_array[array_index].base+i);

		tmp = (ir_array[array_index].data[addr_offset]);
		*(buf++) = (u8)(tmp>>8);  //data high
		*(buf++) = (u8)tmp;       //data low
	}

	return MB_EX_NONE;
}
