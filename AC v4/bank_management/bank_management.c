#include <stdio.h>
#include "bank_management.h"

static non_volatile_mem_write_callback_t mem_write_callback_ptr;
static non_volatile_mem_read_callback_t mem_read_callback_ptr;

static uint8_t memory_write_with_check(uint8_t bank_index, uint8_t bank_byte_index, uint8_t bank_data);
static uint8_t bank_write_internal(uint8_t bank_index, uint8_t *bank_data);



static uint8_t memory_write_with_check(uint8_t bank_index, uint8_t bank_byte_index, uint8_t bank_data)
{
	uint8_t write_attempts = 3;
	uint8_t write_complete_flag = 0;
	
	while(write_attempts > 0 && write_complete_flag == 0)
	{
		mem_write_callback_ptr(bank_index, bank_byte_index, bank_data);
		
		if(bank_data == mem_read_callback_ptr(bank_index, bank_byte_index))
		{
			write_complete_flag = 1;
		}
		
		write_attempts--;
	}
	
	if (write_complete_flag == 0) return INVALID_WRITE_TO_NON_VOLATILE_MEMORY;

	return OK;
}

static uint8_t bank_write_internal(uint8_t bank_index, uint8_t *bank_data)
{
	uint8_t bank_read_byte = 0;
	uint8_t bank_sum = 0;
	uint8_t byte_index;
	
	if(bank_index > BANK_LAST_INDEX) return INVALID_ARG;

	for(byte_index = 0; byte_index < (BANK_LENGTH - 2); byte_index++)
	{
		bank_read_byte = mem_read_callback_ptr(bank_index, byte_index);
		
		if(bank_read_byte != *(bank_data + byte_index))
		{
			if(memory_write_with_check(bank_index, byte_index, *(bank_data + byte_index)) != 0) return INVALID_WRITE_TO_NON_VOLATILE_MEMORY;
		}
		bank_sum += *(bank_data + byte_index);
	}
	
	if(memory_write_with_check(bank_index, CHECKSUM_BYTE_INDEX,  bank_sum) != 0) return INVALID_WRITE_TO_NON_VOLATILE_MEMORY;
	if(memory_write_with_check(bank_index, CHECKSUM_NEG_BYTE_INDEX, ~bank_sum) != 0) return INVALID_WRITE_TO_NON_VOLATILE_MEMORY;

	return OK;
}

void bank_management_register_callbacks(non_volatile_mem_write_callback_t write_cb, non_volatile_mem_read_callback_t read_cb)
{
	mem_write_callback_ptr = write_cb;
	mem_read_callback_ptr = read_cb;
}

uint8_t bank_write(uint8_t bank_index, uint8_t *bank_data)
{
	uint8_t status;
	
	if(bank_index <= BANK_READ_ONLY_LAST_INDEX) return BANK_READ_ONLY;

	status = bank_write_internal(bank_index, bank_data);

	return status;
}

uint8_t bank_read(uint8_t bank_index, uint8_t *bank_data)
{
	uint8_t bank_sum = 0;
	uint8_t byte_index;
	
	if(bank_index > BANK_LAST_INDEX) return INVALID_ARG;
	
	for(byte_index = 0; byte_index < (BANK_LENGTH - 2); byte_index++)
	{
		*(bank_data + byte_index) = mem_read_callback_ptr(bank_index, byte_index);
		
		bank_sum += *(bank_data + byte_index);
	}
	
	if(mem_read_callback_ptr(bank_index, CHECKSUM_BYTE_INDEX) != bank_sum) return INVALID_CHECKSUM;
	bank_sum = ~bank_sum;
	if(mem_read_callback_ptr(bank_index, CHECKSUM_NEG_BYTE_INDEX) != bank_sum) return INVALID_CHECKSUM;
	
	return OK;
}

uint8_t bank_erase(uint8_t bank_index)
{
	uint8_t bank_temp[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	uint8_t status;
					
	status = bank_write_internal(bank_index, bank_temp);	
	
	return status;
}

uint8_t bank_write_byte(uint8_t bank_index, uint8_t byte_index, uint8_t write_data_byte)
{
	uint8_t bank_temp[8];
	uint8_t status;
	
	if(byte_index >= BANK_DATA_LENGTH) return INVALID_ARG;
	
	status = bank_read(bank_index, bank_temp);
	
	if(status > 0) return status;

	bank_temp[byte_index] = write_data_byte;
	
	status = bank_write(bank_index, bank_temp);
	
	if(status > 0) return status;	
	
	return OK;
}

uint8_t bank_read_byte(uint8_t bank_index, uint8_t byte_index, uint8_t *read_data_byte)
{
	uint8_t bank_temp[8];
	uint8_t status;
	
	if(byte_index >= BANK_DATA_LENGTH) return INVALID_ARG;	
	
	status = bank_read(bank_index, bank_temp);
	
	if(status > 0) return status;

	*read_data_byte = bank_temp[byte_index];	
	
	return OK;
}

uint8_t get_banks_status(void)
{
	uint8_t bank[8];
	uint8_t banks_checksum_status = 0x00;
	uint8_t status;
	uint8_t bank_number;
	
	for(bank_number = 0; bank_number < BANK_LAST_INDEX; bank_number++)
	{
		status = bank_read(bank_number, bank);
		
		if(status > 0)
		{
			banks_checksum_status |= (1<<bank_number);
		}
	}
	
	return banks_checksum_status;
}

uint8_t bank_write_tester_id(uint8_t tester_id)
{
	uint8_t bank_data[8];
	uint8_t status;
	
	if (tester_id == 0) return INVALID_ARG;
	
	status = bank_read(0, bank_data);
	
	if (status > 0) return status;
	
	bank_data[0] = tester_id;
	
	status = bank_write_internal(0, bank_data);
	
	return status;
}

uint8_t bank_read_tester_id(uint8_t *tester_id)
{
	uint8_t bank_data[8];
	uint8_t status = bank_read(0, bank_data);

	*tester_id = bank_data[0];
	
	if (status > 0) return status;
	
	if (bank_data[0] == 0) return EMPTY_DATA;

	return OK;
}

uint8_t bank_write_bitrate(uint8_t bitrate)
{
	uint8_t bank_data[8];
	uint8_t status;
	
	if (bitrate == 0 || bitrate > 14) return INVALID_ARG;
	
	status = bank_read(0, bank_data);
	
	if (status > 0) return status;
	
	bank_data[1] = bitrate;
	
	status = bank_write_internal(0, bank_data);
	
	return status;
}

uint8_t bank_read_bitrate(uint8_t *bitrate)
{
	uint8_t bank_data[8];
	uint8_t status = bank_read(0, bank_data);

	*bitrate = bank_data[1];
	
	if (status > 0) return status;
	
	if (bank_data[1] == 0) return EMPTY_DATA;
	
	return OK;
}

uint8_t bank_write_calibration_date(uint8_t day, uint8_t month, uint8_t year)
{
	uint8_t bank_data[8];
	uint8_t status;

	if(	year < 1 || year > 99 || month < 1 || month > 12 || day < 1 || day > 31 ) return INVALID_ARG;
	
	status = bank_read(0, bank_data);
	
	if (status > 0) return status;
	
	bank_data[5] = day;
	bank_data[6] = month;
	bank_data[7] = year;
	
	status = bank_write_internal(0, bank_data);
	
	return status;
}

uint8_t bank_read_calibration_date(uint8_t *day, uint8_t *month, uint8_t *year)
{
	uint8_t bank_data[8];
	uint8_t status = bank_read(0, bank_data);
	
	*day = bank_data[5];
	*month = bank_data[6];
	*year = bank_data[7];
	
	if (status > 0) return status;
	
	if (bank_data[5] == 0 || bank_data[6] == 0 || bank_data[7] == 0) return EMPTY_DATA;
	
	return OK;
}

uint8_t bank_write_check_date(uint8_t day, uint8_t month, uint8_t year)
{
	uint8_t bank_data[8];
	uint8_t status;

	if(	year < 1 || year > 99 || month < 1 || month > 12 || day < 1 || day > 31 ) return INVALID_ARG;
	
	status = bank_read(0, bank_data);
	
	if (status > 0) return status;
	
	bank_data[2] = day;
	bank_data[3] = month;
	bank_data[4] = year;
	
	status = bank_write_internal(0, bank_data);
	
	return status;
}

uint8_t bank_read_check_date(uint8_t *day, uint8_t *month, uint8_t *year)
{
	uint8_t bank_data[8];
	uint8_t status = bank_read(0, bank_data);
	
	*day = bank_data[2];
	*month = bank_data[3];
	*year = bank_data[4];
	
	if (status > 0) return status;
	
	if (bank_data[2] == 0 || bank_data[3] == 0 || bank_data[4] == 0) return EMPTY_DATA;
	
	return OK;
}

uint8_t bank_write_calibration_flag(uint8_t calibration_flag)
{
	uint8_t bank_data[8];
	uint8_t status;

	if(calibration_flag > 2) return INVALID_ARG;
	
	status = bank_read(1, bank_data);
	
	if (status > 0) return status;
	
	bank_data[0] = calibration_flag;
	
	status = bank_write_internal(1, bank_data);
	
	return status;
}

uint8_t bank_read_calibration_flag(uint8_t *calibration_flag)
{
	uint8_t bank_data[8];
	uint8_t status = bank_read(1, bank_data);
	
	*calibration_flag = bank_data[0];
	
	if (status > 0) return status;
	
	if (bank_data[0] == 0) return EMPTY_DATA;
	
	return OK;
}