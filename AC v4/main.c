#define F_CPU 16000000UL

#include <avr/io.h>
#include <stdio.h>
#include <math.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <string.h>
#include "uart.h"
#include "bank_management/bank_management.h"

#define BANK_MANAGEMENT_LIB_VER_IN_USE 1

#if (BANK_MANAGEMENT_LIB_VER != BANK_MANAGEMENT_LIB_VER_IN_USE)
#error Nieaktualna wersja biblioteki: BANK_MANAGEMENT
#endif

#define PREAMBLE_VALUE	0x55
#define PREAMBLE		0
#define LENGTH_L		1
#define LENGTH_H		2
#define SLOT_ADDRESS	3
#define MODULE_TYPE		4
#define CMD				5
#define BOOT_MODE		6
#define DATA_START		7

#define MODULE_TYPE_VAL	3
#define MODULE_VER_H	1
#define MODULE_VER_L	3

#define SOFT_VER_HS		"4"
#define SOFT_VER_LS		"1"
#define SOFT_VER_H		4
#define SOFT_VER_L		1
//#define REV_VER			$WCREV$
//__attribute__( (section(".FW_ver")) ) uint8_t soft_version[2] = {SOFT_VER_L, SOFT_VER_H};
//__attribute__( (section(".rev_ver")) ) uint16_t rev_version = REV_VER;

#define BOOT_START_ADDRESS	0xF000		// byte-address
#define BOOT_ENTER_JUMP		asm volatile ("jmp 0xF000")

#define RX_MAX_DATA_LENGTH	120
#define TX_MAX_DATA_LENGTH	120

#define REG_UDR				UDR
#define REG_UCSRA			UCSRA
#define REG_UCSRB			UCSRB
#define REG_UCSRC			UCSRC
#define REG_UBRRL			UBRRL

#define BANK_EEPROM_START_ADDRESS	0x00

#define REG_PORT PORTC
#define REG_DDR DDRC

#define SPI_DDR DDRB
#define SPI_PORT PORTB
#define CS_PORT PORTB
#define CS_DDR DDRB

#define MOSI (1<<PB5)
#define MISO (1<<PB6)
#define SCK (1<<PB7)


#define CS_ADC0 (1<<PB4)
#define CS_ADC1 (1<<PB3)
#define DATA_REG (1<<PC3)
#define RESET_REG (1<<PC2)
#define SHIFT_CLK (1<<PC1)
#define LATCH_CLK (1<<PC0)

#define CLK_CONF (1<<0)
#define REF_SEL0 (1<<1)
#define REF_SEL1 (1<<2)
#define SCAN0 (1<<3)
#define SCAN1 (1<<4)
#define CH_SEL0 (1<<5)
#define CH_SEL1 (1<<6)
#define CH_SEL2 (1<<7)

#define CHANNEL_0 0
#define CHANNEL_1 CH_SEL0
#define CHANNEL_2 CH_SEL1
#define CHANNEL_3 CH_SEL0 | CH_SEL1
#define CHANNEL_4 CH_SEL2
#define CHANNEL_5 CH_SEL0 | CH_SEL2
#define CHANNEL_6 CH_SEL1 | CH_SEL2
#define CHANNEL_7 CH_SEL0 | CH_SEL1 | CH_SEL2

#define LED_DDR				DDRC
#define LED_PORT			PORTC
#define LED11_PIN			(1<<PC6)
#define LED12_PIN			(1<<PC5)
#define LED2_PIN			(1<<PC4)
#define LED_ERROR_ON        LED_PORT |= LED12_PIN; LED_PORT &= ~LED11_PIN;
#define LED_OK_ON	        LED_PORT |= LED11_PIN; LED_PORT &= ~LED12_PIN;
#define LED_OKERR_OFF       LED_PORT &= ~LED11_PIN; LED_PORT &= ~LED12_PIN;
#define LED_RX_ON           LED_PORT &= ~LED2_PIN;
#define LED_RX_OFF          LED_PORT |= LED2_PIN;

#define UART_PORT			PORTD
#define UART_PIN			PIND
#define UART_DDR			DDRD
#define UART_ERR_PIN		(1<<PD2)
#define UART_RTS_PIN		(1<<PD3)
#define UART_TX_PIN			(1<<PD1)
#define UART_RX_PIN			(1<<PD0)
#define MAX232_ERR_SET      UART_PORT &= ~UART_ERR_PIN;
#define MAX232_ERR_RST	    UART_PORT |= UART_ERR_PIN;
#define RTS_PIN				(UART_PIN & UART_RTS_PIN)

#define RELAY_GND_PORT		PORTA
#define RELAY_GND_DDR		DDRA
#define RELAY_GND_PIN		(1<<PA0)

#define INTERNAL_REF 0
#define EXTERNAL_REF REF_SEL0 | REF_SEL1

#define ADDRES_PORT		PORTD
#define ADDRES_DDR			DDRD
#define ADDRESS_PIN			PIND
#define ADDRES_0_PIN		(1<<7)
#define ADDRES_1_PIN		(1<<6)
#define ADDRES_2_PIN		(1<<5)
#define ADDRES_3_PIN		(1<<4)

//Enumy
typedef enum {SLAVE1,SLAVE2}SLAVE_Type;
typedef enum {INTERNAL,EXTERNAL}REF_Type;
typedef enum {RANGE4V,RANGE8V,RANGE16V,RANGE20V}RANGE_Type;
typedef enum {PARAM_CHECKSUM_OK, PARAM_CHECKSUM_ERR,PARAM_CHECKSUM_INV_ERR}PARAM_Type;
typedef enum {CALIB_OK, CALIB_VALUE_ERR, CALIB_CHECKSUM_ERR} CALIB_Type;
typedef enum {WITH_CORRECTION, WITHOUT_CORRECTION} COR_Type;

//Parametry przetwornika i zakresów
#define REFERENCE_TYPE EXTERNAL
#define DELAY_FOR_STABLE_US 200

//Adresy pamiêci EEPROM
#define EE_OFFSET_CALIBRATION 30
#define EE_OFFSET_SLAVE1 0
#define EE_OFFSET_SLAVE2 198
#define SIZE_TABLE_CALIBRATION 8*22 // 8 - liczba kana³ów, 20 - liczba wartoœci kalibracji

#define BANK_EEPROM_START_ADDRESS	0x00

#define CALIBRATION_TABLE_START_ADDRESS	0x32
#define CALIBRATION_TABLE_END_ADDRESS	CALIBRATION_TABLE_START_ADDRESS + (CALIBRATION_CHANNELS_SIZE * CALIBRATION_POINTS_SIZE)
#define CALIBRATION_CHANNELS_SIZE		16
#define CALIBRATION_POINTS_SIZE			22
#define CALIBRATION_TABLE_SIZE			(CALIBRATION_CHANNELS_SIZE * CALIBRATION_POINTS_SIZE)
#define CALIBRATION_VALUE_MIN			0
#define CALIBRATION_VALUE_MAX			255

#define	UNSUPORTED_BITRATE		4
#define INVALID_CAL_VALUE		4
#define	BANK_OPERATION_FAILED	5
#define	INVALID_CAL_TABLE		6

#define DEBUG 1
#define ONLY_20V_RANGE 1

///////////////////////////////////////////////////////////////
//       DEKLARACJE FUNKCJI
///////////////////////////////////////////////////////////////

//// BASIC ////
void RANGE_Set(uint8_t CHANNEL, RANGE_Type RANGE);
uint16_t Get_ADC_AutoRange(uint8_t CHANNEL, REF_Type REF_CFG, RANGE_Type * RANGE);
uint16_t Get_ADC_Val(uint8_t CHANNEL, REF_Type REF_CFG);
uint16_t Measure_Voltage(uint8_t Channel, COR_Type Correction, float * Res);

void Calibration_Table_Load_Default_Values(void);
uint8_t Calibration_Table_Load_Eeprom_Values(void);
uint8_t Calibration_Table_Save_Value(uint8_t channel, uint8_t point, uint8_t cal_value);
uint8_t Calibration_Table_End_Calibration(uint8_t received_checksum, uint8_t cal_date_year, uint8_t cal_date_month, uint8_t cal_date_day);

void REG_Init(void);
void REG_Load(uint32_t var);

void SPI_MasterInit(void);
void SPI_MasterTransmit(uint8_t cData);

uint8_t SPI_MasterRecieve(void);
void init_uc_io(void);
void init_uc_peripherals(void);
void init_uc_main_uart(void);

uint8_t get_reset_status(void);
uint8_t get_ubrrl_param(uint8_t bitrate);

void uart_send_string(const char *string);

void wait_25us(uint16_t count);
void wait_1ms(uint16_t count);

//// EEPROM ////
void eeprom_write(uint16_t address, uint8_t data);
uint8_t eeprom_write_with_check(uint16_t address, uint8_t data);
uint8_t eeprom_read(uint16_t address);

//// BANK MANAGEMENT ////
uint8_t non_volatile_memory_read(uint8_t bank_index, uint8_t bank_byte_index);
void non_volatile_memory_write(uint8_t bank_index, uint8_t bank_byte_index, uint8_t bank_data);

///////////////////////////////////////////////////////////////
//       ZMIENNE GLOBALNE
///////////////////////////////////////////////////////////////
volatile uint8_t slot_address1=0, slot_address2=0;
volatile uint8_t rx_data_frame[RX_MAX_DATA_LENGTH];
volatile uint16_t rx_data_counter;
volatile uint16_t rx_frame_length;
volatile uint8_t rx_frame_sum;
volatile uint8_t rx_frame_receive_flag;
volatile uint8_t flaga_tim0 = 0, flaga_tim1 = 0;
volatile uint16_t time_buf0 = 0, time_buf1 = 0;
uint16_t mask_dev = 0;
uint16_t avarage_ADC_table[CALIBRATION_CHANNELS_SIZE];
uint16_t min_ADC_table[CALIBRATION_CHANNELS_SIZE], max_ADC_table[CALIBRATION_CHANNELS_SIZE];
volatile uint8_t flag_time_probe = 1;


uint8_t tx_data_frame[TX_MAX_DATA_LENGTH];

const char module_description[] PROGMEM = "Plyta Analog-Cyfra v1.3 s"SOFT_VER_HS"."SOFT_VER_LS;

//// EEPROM ////
volatile uint8_t eeprom_status;
uint8_t calibration_table[CALIBRATION_CHANNELS_SIZE][CALIBRATION_POINTS_SIZE];
///////////////////////////////////////////////////////////////
//       START PROGRAMU
///////////////////////////////////////////////////////////////
int main(void)
{
	__asm__ volatile ("WDR");
	uint8_t bootloader_enter_flag = 0;
	uint8_t rst_state = 1;
	uint8_t ubrrl_param;
	
	uint8_t bank_temp[BANK_DATA_LENGTH];
	bank_management_register_callbacks(non_volatile_memory_write, non_volatile_memory_read);
	wait_1ms(500);
	
	init_uc_io();
	init_uc_peripherals();
	init_uc_main_uart();

	__asm__ volatile ("WDR");
	//WDTCR = ((1<<WDTOE) | (1<<WDE));				//WDTCR=0x18;
	//WDTCR = ((1<<WDE) | (1<<WDP2) | (1<<WDP1));		//WDTCR=0x0E;

	//ustawienie wektora przerwañ na adres firmware'u
	MCUCR |= (1 << IVCE);
	MCUCR = 0x00;
	
	uint8_t banks_checksum_status = get_banks_status();
	
	if((banks_checksum_status & (1<<0)) == 0)
	{
		uint8_t bitrate_id;
		bank_read_bitrate(&bitrate_id);
		ubrrl_param = get_ubrrl_param(bitrate_id);
	}
	else
	{
		ubrrl_param = 3;
	}
	
	uint8_t calibration_table_status = Calibration_Table_Load_Eeprom_Values();
	if(calibration_table_status != OK)
	{
		Calibration_Table_Load_Default_Values();
		banks_checksum_status |= (1<<7);
	}

	//Ustalanie pierwszego adresu p³yty AVR ADC0-7
	if(PIND & ADDRES_3_PIN) slot_address1 |= (1<<5);
	if(PIND & ADDRES_2_PIN) slot_address1 |= (1<<4);
	if(PIND & ADDRES_1_PIN) slot_address1 |= (1<<3);
	if(PIND & ADDRES_0_PIN) slot_address1 |= (1<<2);
	
	//Ustalanie drugiego adresu p³yty AVR ADC8-15
	slot_address2 = slot_address1 | 0x01;

	//ustawienia
	tx_data_frame[PREAMBLE] = PREAMBLE_VALUE;
	tx_data_frame[MODULE_TYPE] = MODULE_TYPE_VAL;
	tx_data_frame[BOOT_MODE] = 0x00;
	REG_Load(0xFFFFFFFF);
	
	sei();
	/////////////////////////////////////////////////////////////////
	// G£ÓWNA PÊTLA PROGRAMU
	/////////////////////////////////////////////////////////////////
	while(1)
	{
		uint16_t temp = 0;
		uint64_t temp_sum = 0;
		uint8_t lala = 0;
		
		if(mask_dev){
			flag_time_probe = 0;
			for(uint8_t i = 0; i<16 ; i++){
				if(mask_dev & (1<<i)){
					temp_sum = 0;
					for(uint8_t j = 0;j<16;j++){
						temp = Get_ADC_Val(i,REFERENCE_TYPE);
						temp_sum = temp_sum + temp;
						if(temp > max_ADC_table[i]) max_ADC_table[i] = temp;
						if(temp < min_ADC_table[i]) min_ADC_table[i] = temp;
					}
					 avarage_ADC_table[i] = temp_sum/20;
				}
			}
		}
		if(banks_checksum_status != 0){
			LED_ERROR_ON;
		}
		if(rx_frame_receive_flag == 1)
		{
			rx_frame_receive_flag = 0;
			tx_data_frame[LENGTH_H] = 0;
			tx_data_frame[CMD] = rx_data_frame[CMD];
			
			uint8_t tx_frame_send_flag = 1;
			
			uint8_t temp_var_a;
			uint8_t temp_var_b;
			uint8_t temp_var_c;
			uint16_t ia = 0;
			float Resolution = 0;

			switch(rx_data_frame[CMD])
			{
				case 0:		//Identyfikacja p³yty
				tx_data_frame[LENGTH_L] = 13;
				tx_data_frame[DATA_START+0] = MODULE_TYPE_VAL;
				tx_data_frame[DATA_START+1] = MODULE_VER_H;
				tx_data_frame[DATA_START+2] = MODULE_VER_L;
				tx_data_frame[DATA_START+3] = SOFT_VER_H;
				tx_data_frame[DATA_START+4] = SOFT_VER_L;
				break;
				
				case 1:		//Reset p³yty
				tx_data_frame[LENGTH_L] = 8;
				init_uc_io();
				init_uc_peripherals();
				
				if(banks_checksum_status != 0)
				{
					LED_ERROR_ON;
				}
				break;
				
				case 2:		//Kasuj b³êdy
				MAX232_ERR_RST
				banks_checksum_status = get_banks_status();
				if(banks_checksum_status == 0)
				{
					LED_OKERR_OFF;
				}
				
				tx_data_frame[LENGTH_L] = 10;
				tx_data_frame[DATA_START+0] = get_reset_status();
				tx_data_frame[DATA_START+1] = banks_checksum_status;
				break;
				
				case 3:		//Odczyt EEPROM
				temp_var_a = rx_data_frame[DATA_START+0];
				//ia|=ramka_rx[8]<<8;
				temp_var_b = rx_data_frame[DATA_START+2];
				//ib|=ramka_rx[10]<<8;
				tx_data_frame[LENGTH_L] = 12 + temp_var_b;
				temp_var_c = DATA_START + 4;
				for(; temp_var_b > 0; temp_var_b--)
				tx_data_frame[temp_var_c++] = eeprom_read(temp_var_a++);
				
				tx_data_frame[DATA_START+0] = rx_data_frame[DATA_START+0];
				tx_data_frame[DATA_START+1] = rx_data_frame[DATA_START+1];
				tx_data_frame[DATA_START+2] = rx_data_frame[DATA_START+2];
				tx_data_frame[DATA_START+3] = rx_data_frame[DATA_START+3];
				
				//Je¿eli wys³ano "zapytanie o nr. seryjny" to pobierz go z banku, pozosta³e pola wype³nij 0xFF
				if( rx_data_frame[DATA_START+0] == 0x00 && rx_data_frame[DATA_START+1] == 0x00 &&
				rx_data_frame[DATA_START+2] == 0x12 && rx_data_frame[DATA_START+3] == 0x00 )
				{
					for(uint8_t i = 4; i < 21; i++)
					{
						tx_data_frame[DATA_START+i] = 0xFF;
					}
					
					uint8_t bank_status = bank_read_tester_id(&tx_data_frame[DATA_START+21]);
					
					if(bank_status > 0)
					{
						tx_data_frame[DATA_START+21] = 0xFF;
					}
				}
				break;
				
				case 4:		//Zapis EEPROM
				temp_var_a = rx_data_frame[DATA_START+0];
				//ia|=ramka_rx[8]<<8;
				temp_var_b = rx_data_frame[DATA_START+2];
				//ib|=ramka_rx[10]<<8;
				temp_var_c = DATA_START + 4;
				for(; temp_var_b > 0; temp_var_b--)
				eeprom_write(temp_var_a++, rx_data_frame[temp_var_c++]);
				
				tx_data_frame[LENGTH_L] = 12;
				tx_data_frame[DATA_START+0] = rx_data_frame[DATA_START+0];
				tx_data_frame[DATA_START+1] = rx_data_frame[DATA_START+1];
				tx_data_frame[DATA_START+2] = rx_data_frame[DATA_START+2];
				tx_data_frame[DATA_START+3] = rx_data_frame[DATA_START+3];
				break;
				
				case 7:		//Wysy³a opis p³yty
				uart_send_string(module_description);
				break;
				
				case 8:		//Ustawia przerwanie
				if(rx_data_frame[DATA_START+0] == 0) MAX232_ERR_SET;
				if(rx_data_frame[DATA_START+0] != 0) MAX232_ERR_RST;
				
				tx_data_frame[LENGTH_L] = 9;
				tx_data_frame[DATA_START+0] = rx_data_frame[DATA_START+0];
				break;
				
				case 18:	//Pomiar z korekcj¹ w trybie 10BIT
				temp_var_a = rx_data_frame[DATA_START+0];
				if(temp_var_a > 7) temp_var_a = 7;
				
				ia = Measure_Voltage(temp_var_a, WITH_CORRECTION, &Resolution);
				ia = ia >> 6;
				
				tx_data_frame[LENGTH_L] = 14;
				tx_data_frame[DATA_START+0] = temp_var_a;
				tx_data_frame[DATA_START+1] = 0;
				tx_data_frame[DATA_START+2] = ia;
				tx_data_frame[DATA_START+3] = ia>>8;
				tx_data_frame[DATA_START+4] = ia;
				tx_data_frame[DATA_START+5] = ia>>8;
				
				#if DEBUG
				tx_frame_send_flag = 0;
				#endif
				break;
				
				//Pomiar w trybie rozszerzonym 16BIT bez korekcji
				case 19:
				temp_var_a = rx_data_frame[DATA_START+0];
				if(temp_var_a > 7) temp_var_a = 7;

				ia = Measure_Voltage(temp_var_a,WITHOUT_CORRECTION,&Resolution);
				
				temp_var_b = ia;
				temp_var_c = ia>>8;

				tx_data_frame[LENGTH_L] = 16;
				tx_data_frame[DATA_START+0] = temp_var_a;
				tx_data_frame[DATA_START+1] = 0;
				memcpy(tx_data_frame+DATA_START+2,&Resolution,4);
				tx_data_frame[DATA_START+6] = temp_var_b;
				tx_data_frame[DATA_START+7] = temp_var_c;
				
				#if DEBUG
				tx_frame_send_flag = 0;
				#endif
				break;
				
				//Pomiar w trybie rozszerzonym 16BIT z korekcj¹
				case 20:
				temp_var_a = rx_data_frame[DATA_START+0];
				if(temp_var_a > 7) temp_var_a = 7;
				
				ia = Measure_Voltage(temp_var_a,WITH_CORRECTION,&Resolution);
				
				temp_var_b = ia;
				temp_var_c = ia>>8;

				tx_data_frame[LENGTH_L] = 16;
				tx_data_frame[DATA_START+0] = temp_var_a;
				tx_data_frame[DATA_START+1] = 0;
				memcpy(tx_data_frame+DATA_START+2,&Resolution,4);
				tx_data_frame[DATA_START+6] = temp_var_b;
				tx_data_frame[DATA_START+7] = temp_var_c;
				
				#if DEBUG
				tx_frame_send_flag = 0;
				#endif
				break;
				
				//Rozkaz pomiaru ADC w trybie ci¹g³ym z rejestracj¹ MIN/MAX
				case 21:
				mask_dev = rx_data_frame[DATA_START+1];
				mask_dev = mask_dev << 8;
				mask_dev |= rx_data_frame[DATA_START+0];
				
				for(uint8_t i = 0; i<CALIBRATION_CHANNELS_SIZE;i++){
					if(mask_dev & (1<<i)){
						avarage_ADC_table[i] = 0x0000;
						min_ADC_table[i] = 0xFFFF;
						max_ADC_table[i] = 0x0000;
						}else{
						min_ADC_table[i] = 0x0000;
						max_ADC_table[i] = 0x0000;
					}

				}
				
				tx_data_frame[LENGTH_L] = 10;
				tx_data_frame[DATA_START+0] = rx_data_frame[DATA_START+0];
				tx_data_frame[DATA_START+1] = rx_data_frame[DATA_START+1];
				break;
				
				//Odczyt MIN/ŒREDNIEJ/MAX
				case 22:
				Resolution = 0.0003125;
				tx_data_frame[LENGTH_L] = CALIBRATION_CHANNELS_SIZE*6 + 12;
				memcpy(tx_data_frame+DATA_START,&Resolution,4);
				for(uint8_t i = 0;i<CALIBRATION_CHANNELS_SIZE*6;i=i+6){
					tx_data_frame[DATA_START+4+i] = min_ADC_table[i/6];
					tx_data_frame[DATA_START+4+i+1] = min_ADC_table[i/6]>>8;
					tx_data_frame[DATA_START+4+i+2] = avarage_ADC_table[i/6];
					tx_data_frame[DATA_START+4+i+3] = avarage_ADC_table[i/6]>>8;
					tx_data_frame[DATA_START+4+i+4] = max_ADC_table[i/6];
					tx_data_frame[DATA_START+4+i+5] = max_ADC_table[i/6]>>8;
				}
				break;
		
				//Za³¹cz przekaŸnik wspólnej masy
				case 25:
				tx_data_frame[LENGTH_L] = 8;
				RELAY_GND_PORT |= RELAY_GND_PIN;
				break;
				
				//Roz³¹cz przekaŸnik wspólnej masy
				case 26:
				tx_data_frame[LENGTH_L] = 8;
				RELAY_GND_PORT &= ~RELAY_GND_PIN;
				break;
				
				//WejdŸ do bootloadera
				case 132:
				tx_data_frame[LENGTH_L] = 9;
				tx_data_frame[DATA_START+0] = 0;
				bootloader_enter_flag = 1;
				break;
				
				/////////////////// KALIBRACJA /////////////////////////////////////////////////////////////////////////////////////////////////
				
				case 201:	//PanelLadujTabliceKalibracji
				banks_checksum_status &= ~(1<<7);
				
				uint8_t calibration_table_status = Calibration_Table_Load_Eeprom_Values();
				if(calibration_table_status != OK)
				{
					Calibration_Table_Load_Default_Values();
					banks_checksum_status |= (1<<7);
				}
				
				tx_data_frame[LENGTH_L] = 9;
				tx_data_frame[DATA_START+0] = calibration_table_status;
				break;
				
				case 203:	//PanelRozpocznijProcesKalibracji
				tx_data_frame[LENGTH_L] = 9;
				tx_data_frame[DATA_START+0] = bank_write_calibration_flag(CALIBRATION_IN_PROGRESS);
				break;
				
				case 204:	//PanelZakonczProcesKalibracji
				tx_data_frame[LENGTH_L] = 13;
				tx_data_frame[DATA_START+0] = rx_data_frame[DATA_START+0];
				tx_data_frame[DATA_START+1] = rx_data_frame[DATA_START+1];
				tx_data_frame[DATA_START+2] = rx_data_frame[DATA_START+2];
				tx_data_frame[DATA_START+3] = rx_data_frame[DATA_START+3];
				tx_data_frame[DATA_START+4] = Calibration_Table_End_Calibration(rx_data_frame[DATA_START+0], rx_data_frame[DATA_START+1],
				rx_data_frame[DATA_START+2], rx_data_frame[DATA_START+3]);
				break;
				
				case 205:	//PanelZapiszPunktKalibracyjny //
				tx_data_frame[LENGTH_L] = 12;
				tx_data_frame[DATA_START+0] = rx_data_frame[DATA_START+0];
				tx_data_frame[DATA_START+1] = rx_data_frame[DATA_START+1];
				tx_data_frame[DATA_START+2] = rx_data_frame[DATA_START+2];
				tx_data_frame[DATA_START+3] = Calibration_Table_Save_Value(	rx_data_frame[DATA_START+0],
				rx_data_frame[DATA_START+1],
				rx_data_frame[DATA_START+2] );
				break;
				
				/////////////////// BANKI /////////////////////////////////////////////////////////////////////////////////////////////////
				
				case 210:	//PanelWyczyœæBank
				tx_data_frame[LENGTH_L] = 10;
				tx_data_frame[DATA_START+0] = rx_data_frame[DATA_START+0];
				tx_data_frame[DATA_START+1] = bank_erase(rx_data_frame[DATA_START+0]);
				break;
				
				case 211:	//PanelCzytajBank
				tx_data_frame[LENGTH_L] = 18;
				tx_data_frame[DATA_START+0] = rx_data_frame[DATA_START+0];

				tx_data_frame[DATA_START+9] = bank_read(tx_data_frame[DATA_START+0], bank_temp);

				for(uint8_t i = 0; i < BANK_DATA_LENGTH; i++)
				{
					tx_data_frame[DATA_START+1+i] = bank_temp[i];
				}
				break;
				
				case 212:	//PanelZapiszBank
				tx_data_frame[LENGTH_L] = 18;
				tx_data_frame[DATA_START+0] = rx_data_frame[DATA_START+0];
				
				for(uint8_t i = 0; i < BANK_DATA_LENGTH; i++)
				{
					tx_data_frame[DATA_START+1+i] = rx_data_frame[DATA_START+1+i];
					bank_temp[i] = rx_data_frame[DATA_START+1+i];
				}
				
				tx_data_frame[DATA_START+9] = bank_write(tx_data_frame[DATA_START+0], bank_temp);
				break;

				case 213:	//PanelCzytajBajtBanku
				tx_data_frame[LENGTH_L] = 12;
				tx_data_frame[DATA_START+0] = rx_data_frame[DATA_START+0];
				tx_data_frame[DATA_START+1] = rx_data_frame[DATA_START+1];
				tx_data_frame[DATA_START+2] = rx_data_frame[DATA_START+2];
				tx_data_frame[DATA_START+3] = bank_read_byte(	tx_data_frame[DATA_START+0],
				tx_data_frame[DATA_START+1],
				&tx_data_frame[DATA_START+2]);
				break;
				
				case 214:	//PanelZapiszBajtBanku
				tx_data_frame[LENGTH_L] = 12;
				tx_data_frame[DATA_START+0] = rx_data_frame[DATA_START+0];
				tx_data_frame[DATA_START+1] = rx_data_frame[DATA_START+1];
				tx_data_frame[DATA_START+2] = rx_data_frame[DATA_START+2];
				tx_data_frame[DATA_START+3] = bank_write_byte(	tx_data_frame[DATA_START+0],
				tx_data_frame[DATA_START+1],
				tx_data_frame[DATA_START+2]);
				break;
				
				case 215:	//PanelCzytajNumerTester
				tx_data_frame[LENGTH_L] = 10;
				tx_data_frame[DATA_START+1] = bank_read_tester_id(&tx_data_frame[DATA_START+0]);

				break;
				
				case 216:	//PanelZapiszNumerTester
				tx_data_frame[LENGTH_L] = 10;
				tx_data_frame[DATA_START+0] = rx_data_frame[DATA_START+0];
				tx_data_frame[DATA_START+1] = bank_write_tester_id(tx_data_frame[DATA_START+0]);
				break;
				
				case 217:	//PanelCzytajPrêdkoœæTransmisji
				tx_data_frame[LENGTH_L] = 10;
				tx_data_frame[DATA_START+1] = bank_read_bitrate(&tx_data_frame[DATA_START+0]);
				break;

				case 218:	//PanelZapiszPrêdkoœæTransmisji
				tx_data_frame[LENGTH_L] = 10;
				tx_data_frame[DATA_START+0] = rx_data_frame[DATA_START+0];
				tx_data_frame[DATA_START+1] = bank_write_bitrate(tx_data_frame[DATA_START+0]);
				break;

				case 219:	//PanelOdczytajDateSprawdzenia
				tx_data_frame[LENGTH_L] = 12;
				tx_data_frame[DATA_START+3] = bank_read_check_date(	&tx_data_frame[DATA_START+0],
				&tx_data_frame[DATA_START+1],
				&tx_data_frame[DATA_START+2]);
				break;
				
				case 220:	//PanelZapiszDateSprawdzenia
				tx_data_frame[LENGTH_L] = 12;
				tx_data_frame[DATA_START+0] = rx_data_frame[DATA_START+0];
				tx_data_frame[DATA_START+1] = rx_data_frame[DATA_START+1];
				tx_data_frame[DATA_START+2] = rx_data_frame[DATA_START+2];
				tx_data_frame[DATA_START+3] = bank_write_check_date(rx_data_frame[DATA_START+0],
				rx_data_frame[DATA_START+1],
				rx_data_frame[DATA_START+2]);
				break;

				case 252:	//PanelZapiszPrêdkoœæTransmisji
				tx_data_frame[LENGTH_L] = 10;
				tx_data_frame[DATA_START+0] = rx_data_frame[DATA_START+0];
				tx_data_frame[DATA_START+1] = bank_write_bitrate(tx_data_frame[DATA_START+0]);
				
				if(tx_data_frame[DATA_START+1] == 0)
				{
					ubrrl_param = get_ubrrl_param(tx_data_frame[DATA_START+0]);
				}
				else
				{
					tx_data_frame[DATA_START+0] = 0xFF;
				}
				break;
				
				case 253:
				rx_frame_receive_flag = 0;
				rx_data_frame[CMD] = 0;
				while(1);
				tx_frame_send_flag = 0;
				break;
				
				case 254:		//Kasuj przerwania
				MAX232_ERR_RST;
				tx_frame_send_flag = 0;
				break;
				
				case 255:		//Reset p³yty
				init_uc_io();
				init_uc_peripherals();
				
				wait_1ms(100);
				
				if( (PORTA != 0x00) || (PORTB != 0x00) || (PORTC != 0x00) || (PORTD != 0x00) ) LED_ERROR_ON;
				
				tx_frame_send_flag = 0;
				break;
				
				default:
				tx_frame_send_flag = 0;
				break;
			}
			
			if(tx_frame_send_flag==1)
			{
				uint16_t i = 0xFFFF;
				uint16_t tx_frame_length = ((tx_data_frame[LENGTH_H]<<8) | tx_data_frame[LENGTH_L]) - 1;
				uint8_t tx_frame_sum = 0;
				while(++i < tx_frame_length)
				{
					while((REG_UCSRA & 0x20) == 0x00);
					REG_UDR = tx_data_frame[i];
					tx_frame_sum += tx_data_frame[i];
				}
				while((REG_UCSRA & 0x20) == 0x00);
				REG_UDR = tx_frame_sum;
				while((REG_UCSRA & 0x20) == 0x00);
			}
			LED_RX_OFF;
			LED_OKERR_OFF;
			
			if(bootloader_enter_flag == 1)
			{
				cli();
				BOOT_ENTER_JUMP;
			}
		}
		
		if(RTS_PIN != rst_state)
		{
			LED_RX_ON;
			rst_state = RTS_PIN;
			if(rst_state == 0)
			{
				REG_UBRRL = ubrrl_param;
			}
			else
			{
				REG_UBRRL = 25;   //38400 bit/s
			}
			wait_1ms(100);
			LED_RX_OFF;
		}
		
		__asm__ volatile ("WDR");
	}
}

////////////////////////////////////////////////////////////////
// PRZERWANIA
////////////////////////////////////////////////////////////////

//Przerwanie, procedura odbioru ramki
ISR(USART_RXC_vect)
{
	//---------------------------------------------------------------------------//----------
	// Preambu³a 0x55 | D³ugoœæ L | D³ugoœæ H | Adres |  Typ  | Rozkaz | 0x00 | Dane | Suma |
	//---------------------------------------------------------------------------//----------
	uint8_t rx_data = REG_UDR;
	rx_data_frame[rx_data_counter++] = rx_data;
	rx_frame_sum += rx_data;
	LED_RX_ON;
	
	if((rx_data_counter == 1) && (rx_data != PREAMBLE_VALUE))
	{
		rx_frame_sum = 0;
		rx_data_counter = 0;
	}
	if(rx_data_counter == 2){
		time_buf0 =0;
		flaga_tim0 = 1;
		rx_frame_length = rx_data;
	}

	if(rx_data_counter == 3)
	{
		rx_frame_length |= rx_data<<8;
		if(rx_frame_length > RX_MAX_DATA_LENGTH) rx_frame_length = RX_MAX_DATA_LENGTH;
	}
	
	if(rx_data_counter == rx_frame_length)
	{
		time_buf0 =0;
		flaga_tim0 = 0;
		
		rx_frame_sum -= rx_data;
		if(	((rx_data_frame[SLOT_ADDRESS] == slot_address1) || (rx_data_frame[SLOT_ADDRESS] == slot_address2) || (rx_data_frame[SLOT_ADDRESS] == 0xFF)) &&
		((rx_data_frame[MODULE_TYPE] == MODULE_TYPE_VAL) || (rx_data_frame[MODULE_TYPE] == 0x00))&& (rx_data_frame[rx_data_counter-1] == rx_frame_sum)){
			rx_frame_receive_flag = 1;
			LED_OK_ON;
		}
		else
		LED_RX_OFF;
		
		rx_frame_sum = 0;
		rx_data_counter = 0;
	}
}

// Przerwanie co 1 ms
ISR(TIMER0_COMP_vect)
{
	if(flaga_tim0){
		time_buf0++;
		if(time_buf0 >= 10){
			time_buf0 = 0;
			flaga_tim0 = 0;
			rx_frame_length = 0;
			rx_frame_sum = 0;
			rx_data_counter = 0;
			LED_RX_OFF;
		}
	}
}

////////////////////////////////////////////////////////////////
// FUNKCJE
////////////////////////////////////////////////////////////////

//Pomiar wartoœci napiêcia z korekcja oraz uœrednianiem
uint16_t Measure_Voltage(uint8_t Channel, COR_Type Correction, float * Res){
	float Factor_A = 0, Offset_B = 0, Resolution = 0;
	uint16_t Offset_Channel = 0, Voltage_Point = 0, Data = 0;
	uint8_t Temp_Dev = 0, Temp_Dev_Next = 0, Calib_Value = 0;
	RANGE_Type Range = RANGE20V;
	
	if(rx_data_frame[SLOT_ADDRESS] == slot_address1){
		Data = Get_ADC_AutoRange(Channel, REFERENCE_TYPE, &Range);
		Offset_Channel = 0;
	}
	else if(rx_data_frame[SLOT_ADDRESS] == slot_address2){
		Data = Get_ADC_AutoRange(Channel, REFERENCE_TYPE, &Range);
		Offset_Channel = 8;
	}
	else Data = 0;
	
	switch(Range){
		case RANGE4V:
		Resolution = 0.0000625;
		break;
		case RANGE8V:
		Resolution = 0.000125;
		break;
		case RANGE16V:
		Resolution = 0.000250;
		break;
		case RANGE20V:
		Resolution = 0.0003125;
		break;
		default:
		Resolution = 0;
		break;
	}
	calibration_table[0][14] = 185;
	calibration_table[0][15] = 190;
	if(Correction == WITH_CORRECTION){
		Voltage_Point = Data * Resolution;
		
		Temp_Dev = calibration_table[Channel+Offset_Channel][Voltage_Point];
		Temp_Dev_Next = calibration_table[Channel+Offset_Channel][Voltage_Point+1];
		
		Factor_A = (float)(Temp_Dev_Next - Temp_Dev) * Resolution;
		Offset_B = (float)Temp_Dev - (float)Voltage_Point/Resolution*Factor_A;
		Calib_Value = round(Factor_A*Data + Offset_B);
		if(((uint32_t)Data + Calib_Value) < 127) Data = 0;
		else if((uint32_t)Data + Calib_Value > 65535) Data = 65535;
		else Data = Data + Calib_Value - 127;
	}
	
	#if DEBUG
	uint64_t Voltage = 0;
	uint16_t Voltage_D = 0, Voltage_J = 0;
	uint8_t Text_Buf_Debug[50];
	
	Voltage = ((float)Data * 10000) * Resolution;
	Voltage_D = Voltage/10000;
	Voltage_J = Voltage%10000;
	sprintf((char*)Text_Buf_Debug,"ADC%u: %u.%4u RANGE:%u, CALIB: %d\n\r", Channel, Voltage_D, Voltage_J,Range,Calib_Value);
	UART_StringTransmit(Text_Buf_Debug);
	#endif
	
	*Res = Resolution;
	return Data;
}


// Pobierz wartoœæ ADC na danym kanale przy automatycznym dobieraniu zakresu
uint16_t Get_ADC_AutoRange(uint8_t CHANNEL, REF_Type REF_CFG, RANGE_Type * RANGE){
	uint64_t DATA_SUM=0;

	RANGE_Set(CHANNEL,RANGE20V);
	*RANGE = RANGE20V;
	
	#if !ONLY_20V_RANGE
	uint16_t DATA = 0;
	DATA = Get_ADC_Val(CHANNEL,REF_CFG);

	if(DATA < (65535 * 0.2)) {
		RANGE_Set(CHANNEL,RANGE4V);
		*RANGE = RANGE4V;
	}
	else if(DATA < (65535 * 0.4) ){
		RANGE_Set(CHANNEL,RANGE8V);
		*RANGE = RANGE8V;
	}
	else if(DATA < (65535 * 0.8)){
		RANGE_Set(CHANNEL,RANGE16V);
		*RANGE = RANGE16V;
	}
	#endif
	
	for(uint16_t i = 0;i<20;i++)
	{
		DATA_SUM += Get_ADC_Val(CHANNEL,REF_CFG);
		_delay_us(100);
	}
	
	return DATA_SUM / 20;
}

//Ustaw zakres danego kana³u
void RANGE_Set(uint8_t CHANNEL, RANGE_Type RANGE){
	//Dostosywanie numerów kana³ów do starej p³yty
	uint32_t RANGE_temp = 0xFFFFFFFF;
	if(CHANNEL >= 0 && CHANNEL <= 7){
		CHANNEL = 14 - 2*CHANNEL;
		RANGE_temp &= ~((uint32_t)3<<(30-CHANNEL*2));
		RANGE_temp |= (uint32_t)RANGE<<(30-CHANNEL*2);
		REG_Load(RANGE_temp);
	}
	else if(CHANNEL >= 8 && CHANNEL <= 15){
		CHANNEL = (15-CHANNEL)*2+1;
		RANGE_temp &= ~((uint32_t)3<<(30-CHANNEL*2));
		RANGE_temp |= (uint32_t)RANGE<<(30-CHANNEL*2);
		REG_Load(RANGE_temp);
	}
	_delay_us(DELAY_FOR_STABLE_US);
}

//Pobierz wartoœæ ADC z danego kana³u REF_CFG = EXTERNAL/INTERNAL, RES_CFG = BIT10/BIT16
uint16_t Get_ADC_Val(uint8_t CHANNEL, REF_Type REF_CFG){
	uint8_t DATA_H = 0, DATA_L = 0;
	uint16_t DATA = 0;
	uint8_t CONF_REG = 0;
	
	if(REF_CFG == INTERNAL) CONF_REG |= INTERNAL_REF;
	else if(REF_CFG == EXTERNAL) CONF_REG |= EXTERNAL_REF;
	
	//Dostosywanie numerów kana³ów do starej p³yty
	switch(CHANNEL){
		case 0:
		CHANNEL = 12;
		break;
		case 1:
		CHANNEL = 10;
		break;
		case 2:
		CHANNEL = 14;
		break;
		case 3:
		CHANNEL = 9;
		break;
		case 4:
		CHANNEL = 4;
		break;
		case 5:
		CHANNEL = 2;
		break;
		case 6:
		CHANNEL = 6;
		break;
		case 7:
		CHANNEL = 1;
		break;
		case 8:
		CHANNEL = 13;
		break;
		case 9:
		CHANNEL = 11;
		break;
		case 10:
		CHANNEL = 15;
		break;
		case 11:
		CHANNEL = 8;
		break;
		case 12:
		CHANNEL = 5;
		break;
		case 13:
		CHANNEL = 3;
		break;
		case 14:
		CHANNEL = 7;
		break;
		case 15:
		CHANNEL = 0;
		break;
	}

	cli();
	if(CHANNEL < 8){
		CONF_REG |=  (CHANNEL << 5) ;
		
		CS_PORT &= ~CS_ADC0;
		SPI_MasterTransmit(CONF_REG);
		DATA_H = SPI_MasterRecieve();
		DATA_L = SPI_MasterRecieve();
		CS_PORT |= CS_ADC0;

		}else if(CHANNEL <16){
		CONF_REG |= ((CHANNEL-8) << 5);
		CS_PORT &= ~CS_ADC1;
		SPI_MasterTransmit(CONF_REG);
		DATA_H = SPI_MasterRecieve();
		DATA_L = SPI_MasterRecieve();
		CS_PORT |= CS_ADC1;
	}
	sei();
	
	DATA = (uint16_t)(DATA_H<<8);
	DATA |= DATA_L;
	
	return DATA;
}

void REG_Init(void) {
	REG_DDR |= DATA_REG | RESET_REG | SHIFT_CLK | LATCH_CLK;
	REG_PORT |= RESET_REG;
	REG_PORT &= ~RESET_REG;
	REG_PORT |= RESET_REG;
}

void REG_Load(uint32_t var) {
	cli();
	for(uint8_t i = 0;i<=32;i++){
		REG_PORT |= SHIFT_CLK;
		_delay_us(1);
		if (var & (1 << 0)) {
			REG_PORT |= DATA_REG;
			}else{
			REG_PORT &= ~DATA_REG;
		}
		_delay_us(1);
		REG_PORT &= ~SHIFT_CLK;
		var = var >> 1;
		_delay_us(1);
	}
	REG_PORT |= LATCH_CLK;
	_delay_us(1);
	REG_PORT &= ~LATCH_CLK;
	sei();
}

void SPI_MasterInit(void){
	CS_DDR |= CS_ADC0 | CS_ADC1;
	CS_PORT |= CS_ADC0 | CS_ADC1;

	SPI_DDR |= MOSI | SCK;
	SPCR = (1<<SPE) | (1<<MSTR) | (1<<SPR0) | (1<<SPR1);
}

void SPI_MasterTransmit(uint8_t cData){
	SPDR = cData;
	flaga_tim1 = 1;
	while(!(SPSR&(1<<SPIF))&& flaga_tim1);
}

uint8_t SPI_MasterRecieve(void){
	SPDR = 0x00;
	flaga_tim1 = 1;
	while(!(SPSR&(1<<SPIF))  && flaga_tim1);
	return SPDR;
}


void init_uc_io(void)
{
	RELAY_GND_DDR |= RELAY_GND_PIN;
	RELAY_GND_PORT |= RELAY_GND_PIN;
	
	LED_DDR |= LED11_PIN | LED12_PIN | LED2_PIN;
	LED_RX_OFF;
	
	UART_DDR |= UART_ERR_PIN | UART_TX_PIN;
	UART_PORT|= UART_TX_PIN | UART_RTS_PIN;
	
	ADDRES_DDR &= ~0xF0;
	ADDRES_PORT |= 0xF0;
}

void init_uc_peripherals(void)
{
	//Inicjacja rejestrów
	REG_Init();
	
	//Inicjacja SPI
	SPI_MasterInit();
	
	//TC0 - tryb CTC
	TIMSK |= (1<<OCIE0); // Przerwanie od COMP
	TCCR0 |= (1<<CS01) | (1<<CS00) | (1<<WGM01); // Prescaler /64
	OCR0 = 250; // Przerwanie co 1 ms
	
}

void init_uc_main_uart(void)
{
	REG_UCSRB |= ((1<<RXCIE) | (1<<RXEN) | (1<<TXEN));
	REG_UCSRC |= ((1<<URSEL) | (1<<UCSZ1) | (1<<UCSZ0));
	REG_UBRRL = 25;   //38400 bit/s
};

uint8_t get_reset_status(void){
	uint8_t reset_status = 0x00;

	if(MCUCSR & (1<<PORF)) reset_status |= (1<<0);
	if(MCUCSR & (1<<BORF)) reset_status |= (1<<1);
	if(MCUCSR & (1<<WDRF)) reset_status |= (1<<2);
	if(MCUCSR & (1<<EXTRF)) reset_status |= (1<<3);
	
	MCUCSR = 0x00;
	
	return reset_status;
}

void uart_send_string(const char *string)
{
	uint8_t i = DATA_START;
	uint8_t znak = pgm_read_byte(string++);
	
	while(znak != 0x00)
	{
		tx_data_frame[i++] = znak;
		znak = pgm_read_byte(string++);
	}
	
	tx_data_frame[LENGTH_L] = i + 1;
}

uint8_t get_ubrrl_param(uint8_t bitrate)
{
	uint8_t ubrrl_param;
	switch (bitrate)
	{
		case 1:		ubrrl_param = 207;		break;		//4800
		case 2:		ubrrl_param = 103;		break;		//9600
		case 3: 	ubrrl_param = 68;		break;		//14400
		case 4:		ubrrl_param = 51;		break;		//19200
		case 5: 	ubrrl_param = 34;		break;		//28800
		case 6:		ubrrl_param = 25;		break;		//38400
		case 7: 	ubrrl_param = 16;		break;		//57600
		case 8: 	ubrrl_param = 12;		break;		//76800
		case 9: 	ubrrl_param = 8;		break;		//115200
		case 10: 	ubrrl_param = 7;		break;		//128000
		case 11: 	ubrrl_param = 3;		break;		//230400
		case 12: 	ubrrl_param = 3;		break;		//250000
		case 13: 	ubrrl_param = 1;		break;		//500000
		case 14: 	ubrrl_param = 0;		break; 		//1000000
		default:	ubrrl_param = 25;		break; 		//38400
	}
	
	return ubrrl_param;
}

void eeprom_write(uint16_t address, uint8_t data)
{
	while(EECR & 0x02);
	EEAR = address;
	EEDR = data;
	EECR |= 0x04;
	EECR |= 0x02;
}

uint8_t eeprom_write_with_check(uint16_t address, uint8_t data)
{
	uint8_t write_attempts = 3;
	uint8_t write_complete_flag = 0;
	
	while(write_attempts > 0 && write_complete_flag == 0)
	{
		eeprom_write(address, data);
		
		if(data == eeprom_read(address)) write_complete_flag = 1;

		write_attempts--;
	}
	
	if (write_complete_flag == 0) return 1;

	return 0;
}

uint8_t eeprom_read(uint16_t address)
{
	while(EECR & 0x02);
	EEAR = address;
	EECR |= 0x01;
	return EEDR;
}

void wait_25us(uint16_t count)
{
	__asm__ volatile (
	"1: sbiw %0,1" "\n\t"
	"brne 1b"
	: "=w" (count)
	: "0" (count)
	);
}

void wait_1ms(uint16_t count)
{
	for(;count!=0;count--) wait_25us(4000);
}

uint8_t non_volatile_memory_read(uint8_t bank_index, uint8_t bank_byte_index)
{
	uint16_t eeprom_address = BANK_EEPROM_START_ADDRESS + (bank_index * BANK_LENGTH) + bank_byte_index;
	
	uint8_t bank_data = eeprom_read(eeprom_address);

	return bank_data;
}

void non_volatile_memory_write(uint8_t bank_index, uint8_t bank_byte_index, uint8_t bank_data)
{
	uint16_t eeprom_address = BANK_EEPROM_START_ADDRESS + (bank_index * BANK_LENGTH) + bank_byte_index;
	
	eeprom_write(eeprom_address, bank_data);
}

uint8_t Calibration_Table_Save_Value(uint8_t channel, uint8_t point, uint8_t cal_value)
{
	if( (channel > (CALIBRATION_CHANNELS_SIZE - 1)) || (point > (CALIBRATION_POINTS_SIZE - 1)) ||
	(cal_value < CALIBRATION_VALUE_MIN) || (cal_value > CALIBRATION_VALUE_MAX) ) return INVALID_ARG;

	uint16_t eeprom_address = CALIBRATION_TABLE_START_ADDRESS + (channel * CALIBRATION_POINTS_SIZE) +  point;
	
	if(eeprom_write_with_check(eeprom_address, cal_value) != 0) return INVALID_WRITE_TO_NON_VOLATILE_MEMORY;
	
	return OK;
}

uint8_t Calibration_Table_End_Calibration(uint8_t received_checksum, uint8_t cal_date_year, uint8_t cal_date_month, uint8_t cal_date_day)
{
	uint8_t my_checksum = 0;
	uint8_t read_cal_value;
	uint8_t status;
	
	for (uint16_t address = CALIBRATION_TABLE_START_ADDRESS; address < CALIBRATION_TABLE_END_ADDRESS; address++)
	{
		read_cal_value = eeprom_read(address);
		
		if(read_cal_value < CALIBRATION_VALUE_MIN || read_cal_value > CALIBRATION_VALUE_MAX)
		{
			status = bank_write_calibration_flag(CALIBRATION_COMPLETE);
			if(status != OK) return BANK_OPERATION_FAILED;
			return INVALID_CAL_VALUE;
		}
		my_checksum += read_cal_value;
	}
	
	if(my_checksum != received_checksum) return INVALID_CHECKSUM;
	
	if(eeprom_write_with_check(CALIBRATION_TABLE_END_ADDRESS, my_checksum) != OK) return INVALID_WRITE_TO_NON_VOLATILE_MEMORY;
	
	if(eeprom_write_with_check(CALIBRATION_TABLE_END_ADDRESS + 1, ~my_checksum) != OK) return INVALID_WRITE_TO_NON_VOLATILE_MEMORY;
	
	status = bank_write_calibration_date(cal_date_day, cal_date_month, cal_date_year);
	if(status == INVALID_ARG) return INVALID_ARG;
	else if(status != OK) return BANK_OPERATION_FAILED;
	
	status = bank_write_calibration_flag(CALIBRATION_IN_PROGRESS);
	if(status != OK) return BANK_OPERATION_FAILED;
	
	return OK;
}

uint8_t Calibration_Table_Load_Eeprom_Values(void)
{
	uint16_t calibration_table_eeprom_address = CALIBRATION_TABLE_START_ADDRESS;
	uint8_t calibration_flag;
	uint8_t calibration_value;
	uint8_t calc_checksum = 0;
	uint8_t status;
	
	//sprawdzam czy flaga kalibracji posiada wartoœæ 1 (TABLE_DATA_CORRECT)
	//jezeli wartosc wynosi 0 to zmieniam wartosc na 2 (TABLE_DATA_INCORRECT)
	status = bank_read_calibration_flag(&calibration_flag);
	if(status != OK && status != EMPTY_DATA) return BANK_OPERATION_FAILED;
	
	if(status == EMPTY_DATA)
	{
		status = bank_write_calibration_flag(CALIBRATION_COMPLETE);
		if(status != OK) return BANK_OPERATION_FAILED;
		return INVALID_CAL_TABLE;
	}

	if(calibration_flag == CALIBRATION_COMPLETE) return INVALID_CAL_TABLE;
	
	//jezeli flaga kalibracji posiada wartosc 1 (TABLE_DATA_CORRECT)
	//to kontrolnie sprawdzam czy sumy kontrolne s¹ prawid³owe
	if(calibration_flag == CALIBRATION_IN_PROGRESS)
	{
		for(uint8_t channel = 0; channel < CALIBRATION_CHANNELS_SIZE; channel++)
		{
			for(uint8_t cal_point = 0; cal_point < CALIBRATION_POINTS_SIZE; cal_point++)
			{
				calibration_value = eeprom_read(calibration_table_eeprom_address++);
				calc_checksum = calc_checksum + calibration_value;
				calibration_table[channel][cal_point] = calibration_value;
			}
		}
		//je¿eli ktoras z sum kontrolnych jest nieprawidlowa
		//to zmieniam flage kalibracji na TABLE_DATA_INCORRECT
		if(eeprom_read(CALIBRATION_TABLE_END_ADDRESS) != calc_checksum)
		{
			status = bank_write_calibration_flag(CALIBRATION_COMPLETE);
			if(status != OK) return BANK_OPERATION_FAILED;
			return INVALID_CHECKSUM;
		}
		calc_checksum = ~calc_checksum;
		if(eeprom_read(CALIBRATION_TABLE_END_ADDRESS + 1) != calc_checksum)
		{
			status = bank_write_calibration_flag(CALIBRATION_COMPLETE);
			if(status != OK) return BANK_OPERATION_FAILED;
			return INVALID_CHECKSUM;
		}
	}
	
	return OK;
}

void Calibration_Table_Load_Default_Values(void)
{
	for(uint8_t channel = 0; channel < CALIBRATION_CHANNELS_SIZE; channel++)
	{
		for(uint8_t cal_point = 0; cal_point < CALIBRATION_POINTS_SIZE; cal_point++)
		{
			calibration_table[channel][cal_point] = 127;
		}
	}
}