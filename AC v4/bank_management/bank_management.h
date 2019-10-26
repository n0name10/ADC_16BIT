#ifndef BANK_MANAGEMENT_H
#define BANK_MANAGEMENT_H

#define BANK_MANAGEMENT_LIB_VER		1

#define BANK_READ_ONLY_LAST_INDEX	1
#define BANK_LAST_INDEX				3
#define BANK_LENGTH					10
#define BANK_DATA_LENGTH			BANK_LENGTH - 2

#define CHECKSUM_BYTE_INDEX			8
#define CHECKSUM_NEG_BYTE_INDEX		9

typedef enum{
	OK = 0,

	INVALID_ARG = 1,		// ODCZYT (niepoprawny index banku [dla BANK > BANK_LAST_INDEX]) 
							// ZAPIS (niepoprawna wartoœæ zapisywanego argumentu (nr. testera, prêdkoœc trans. itp) [BANK 0 i 1])
	EMPTY_DATA = 1,			// ODCZYT (odczytana wartoœæ wynosi 0 [dla BANK 0 i 1])

	INVALID_CHECKSUM = 2,	// ODCZYT & ZAPIS (niepoprawna suma kontrolna banku [BANK 0,1,2,3])
	
	INVALID_WRITE_TO_NON_VOLATILE_MEMORY = 3, // ZAPIS (nieudana operacja zapisu) [BANK 0,1,2,3])
	
	BANK_READ_ONLY = 4		// ZAPIS (informacja o banku typu "read-only) [BANK 0, 1]
}bank_operation_codes;

typedef enum{
	CALIBRATION_IN_PROGRESS = 1,
	CALIBRATION_COMPLETE = 2
}calibration_flag_codes;

typedef void	(*non_volatile_mem_write_callback_t)(uint8_t bank_index, uint8_t bank_byte_index, uint8_t bank_data);
typedef uint8_t (*non_volatile_mem_read_callback_t)(uint8_t bank_index, uint8_t bank_byte_index);

void bank_management_register_callbacks(non_volatile_mem_write_callback_t write_cb, non_volatile_mem_read_callback_t read_cb);

uint8_t bank_write(uint8_t bank_index, uint8_t *bank_data);
uint8_t bank_read(uint8_t bank_index, uint8_t *bank_data);
uint8_t bank_erase(uint8_t bank_index);

uint8_t bank_write_byte(uint8_t bank_index, uint8_t byte_index, uint8_t write_data_byte);
uint8_t bank_read_byte(uint8_t bank_index, uint8_t byte_index, uint8_t *read_data_byte);

uint8_t get_banks_status(void);

uint8_t bank_write_tester_id(uint8_t tester_id);
uint8_t bank_read_tester_id(uint8_t *tester_id);

uint8_t bank_write_bitrate(uint8_t bitrate);
uint8_t bank_read_bitrate(uint8_t *bitrate);

uint8_t bank_write_calibration_date(uint8_t day, uint8_t month, uint8_t year);
uint8_t bank_read_calibration_date(uint8_t *day, uint8_t *month, uint8_t *year);

uint8_t bank_write_check_date(uint8_t day, uint8_t month, uint8_t year);
uint8_t bank_read_check_date(uint8_t *day, uint8_t *month, uint8_t *year);

uint8_t bank_write_calibration_flag(uint8_t calibration_flag);
uint8_t bank_read_calibration_flag(uint8_t *calibration_flag);

#endif /* BANK_MANAGEMENT_H */