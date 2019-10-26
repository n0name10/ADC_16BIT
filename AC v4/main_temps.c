#define F_CPU 16000000UL

#include <avr/io.h>
#include <stdio.h>
#include <math.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <string.h>
#include "uart.h"

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
#define SOFT_VER_LS		"0"
#define SOFT_VER_H		4
#define SOFT_VER_L		0
#define REV_VER			85

__attribute__( (section(".FW_ver")) ) uint8_t soft_version[2] = {SOFT_VER_L, SOFT_VER_H};
__attribute__( (section(".rev_ver")) ) uint16_t rev_version = REV_VER;

#define RX_MAX_DATA_LENGTH	100
#define TX_MAX_DATA_LENGTH	100

#define REG_PORT PORTC
#define REG_DDR DDRC

#define SPI_DDR DDRB
#define SPI_PORT PORTB

#define CS_PORT PORTB
#define CS_DDR DDRB

#define MOSI (1<<PB5)
#define MISO (1<<PB6)
#define SCK (1<<PB7)

#define LED_OFF LED_PORT |= LED2_PIN;
#define LED_ON LED_PORT &= ~LED2_PIN;

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
#define LED_ERR_SET         LED_PORT |= LED12_PIN; LED_PORT &= ~LED11_PIN;
#define LED_OK_SET			LED_PORT |= LED11_PIN; LED_PORT &= ~LED12_PIN;
#define LED1_RESET          LED_PORT &= ~LED12_PIN; LED_PORT &= ~LED11_PIN;

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
typedef enum {BIT10=10,BIT16=16}RES_Type;
typedef enum {RANGE4V,RANGE8V,RANGE16V,RANGE20V}RANGE_Type;
typedef enum {PARAM_CHECKSUM_OK, PARAM_CHECKSUM_ERR,PARAM_CHECKSUM_INV_ERR}PARAM_Type;
typedef enum {CALIB_OK, CALIB_VALUE_ERR, CALIB_CHECKSUM_ERR} CALIB_Type;

//Parametry przetwornika i zakresów
#define REFERENCE_TYPE EXTERNAL
#define DELAY_FOR_STABLE_US 250

//Adresy pamiêci EEPROM
#define EE_OFFSET_CALIBRATION 30
#define EE_OFFSET_SLAVE1 0
#define EE_OFFSET_SLAVE2 198
#define SIZE_TABLE_CALIBRATION 8*20 // 8 - liczba kana³ów, 20 - liczba wartoœci kalibracji

#define EE_OFFSET_PARAM_CHECKSUM 28
#define EE_OFFSET_PARAM_CHECKSUM_INV 29
#define EE_OFFSET_PARAM 17
#define PARAM_TABLE_SIZE 11

#define EE_OFFSET_UBRR 19

#define REG_UDR				UDR
#define REG_UCSRA			UCSRA
#define REG_UCSRB			UCSRB
#define REG_UCSRC			UCSRC
#define REG_UBRRL			UBRRL

#define BOOT_ENTER_JUMP		asm volatile ("jmp 0x3800") //skocz do sekcji boot
#define BOOT_START_ADDRESS	0x3800		// byte-address

#define DEBUG 1

///////////////////////////////////////////////////////////////
//       DEKLARACJE FUNKCJI
///////////////////////////////////////////////////////////////

//// BASIC ////

void RANGE_Set(uint8_t CHANNEL, RANGE_Type RANGE);
uint16_t GET_ADC_AutoRange(uint8_t CHANNEL, REF_Type REF_CFG, RANGE_Type * RANGE)	;
uint16_t GET_ADC_Val(uint8_t CHANNEL, REF_Type REF_CFG, RES_Type RES_CFG);
uint16_t Measure_Voltage_With_Correction(uint8_t Channel, float * Res);

void REG_Init(void);
void REG_Load(uint32_t var);

void SPI_MasterInit(void);
void SPI_MasterTransmit(uint8_t cData);
uint8_t SPI_MasterRecieve(void);

void init_uc_io(void);
void init_uc_peripherals(void);
void init_uc_main_uart(void);

uint8_t get_reset_status(void);

void uart_send_string(const char *string);

//// EEPROM ////
void eeprom_write(uint16_t address, uint8_t data);
uint8_t eeprom_read(uint16_t address);

void eeprom_write_checksum(void);
uint8_t eeprom_verify_checksum(void);
void eeprom_read_calibration_all(void);
uint8_t eeprom_read_calibration_table(SLAVE_Type slave);

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

uint8_t tx_data_frame[TX_MAX_DATA_LENGTH];

const char module_description[] PROGMEM = "Plyta Analog-Cyfra v1.3 s"SOFT_VER_HS"."SOFT_VER_LS;

//// EEPROM ////
volatile uint8_t eeprom_status;
uint8_t calibration_table[320];


///////////////////////////////////////////////////////////////
//       START PROGRAMU
///////////////////////////////////////////////////////////////
int main(void)
{
	uint8_t bootloader_enter_flag = 0;
	uint8_t rst_state = 0;
	uint8_t ubrrl_param;
	
	
	init_uc_io();
	init_uc_peripherals();
	init_uc_main_uart();
	
	_delay_ms(500);

	//__asm__ volatile ("WDR");
	//WDTCR = ((1<<WDTOE) | (1<<WDE));				//WDTCR=0x18;
	//WDTCR = ((1<<WDE) | (1<<WDP2) | (1<<WDP1));		//WDTCR=0x0E;

	//ustawienie wektora przerwañ na adres firmware'u
	GICR |= (1 << IVCE);
	GICR = 0x00;
	
	eeprom_read_calibration_all();
	
	uint8_t checksum_status = eeprom_verify_checksum();
	eeprom_status &= 0x0F;
	if(checksum_status == PARAM_CHECKSUM_OK)
	{
		ubrrl_param = eeprom_read(EE_OFFSET_UBRR);
	}
	else
	{
		eeprom_status |= checksum_status<<4;
		ubrrl_param = 3;
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
	
	// Deklaracja zmiennych pomocniczych
	uint8_t tx_frame_send_flag = 1;
	volatile uint8_t temp_var_a=0,temp_var_b=0,temp_var_c=0;
	volatile uint16_t ia=0,temp_var_16=0;
	RANGE_Type RANGE = RANGE20V;
	float Resolution = 0;
	
	sei();
	/////////////////////////////////////////////////////////////////
	// G£ÓWNA PÊTLA PROGRAMU
	/////////////////////////////////////////////////////////////////

	while(1){


		if(eeprom_status){
			LED_ERR_SET;
		}
		if(rx_frame_receive_flag == 1)
		{
			tx_frame_send_flag = 1;
			rx_frame_receive_flag = 0;
			tx_data_frame[LENGTH_H] = 0;
			tx_data_frame[CMD] = rx_data_frame[CMD];
			tx_data_frame[SLOT_ADDRESS] = rx_data_frame[SLOT_ADDRESS];
			
			switch(rx_data_frame[CMD])
			{
				case 0:		//Identyfikuj p³ytê
				tx_data_frame[LENGTH_L] = 13;
				tx_data_frame[DATA_START+0] = MODULE_TYPE_VAL;
				tx_data_frame[DATA_START+1] = MODULE_VER_H;
				tx_data_frame[DATA_START+2] = MODULE_VER_L;
				tx_data_frame[DATA_START+3] = SOFT_VER_H;
				tx_data_frame[DATA_START+4] = SOFT_VER_L;
				break;
				
				case 1:		//Reset p³yty testera
				tx_data_frame[LENGTH_L] = 8;
				init_uc_io();
				init_uc_peripherals();
				break;
				
				case 2:		//Czytaj status - bledy
				MAX232_ERR_RST;
				
				tx_data_frame[LENGTH_L] = 10;
				tx_data_frame[DATA_START+0] = get_reset_status();
				tx_data_frame[DATA_START+1] = eeprom_status;
				break;
				
				case 3:		//Odczyt EEPROM
				if(rx_data_frame[SLOT_ADDRESS] == slot_address1)
				temp_var_16 = rx_data_frame[DATA_START+0] + EE_OFFSET_SLAVE1;
				else if(rx_data_frame[SLOT_ADDRESS] == slot_address2){
					if(rx_data_frame[DATA_START+0]<30)
					temp_var_16 = rx_data_frame[DATA_START+0] + EE_OFFSET_SLAVE1;
					else temp_var_16 = rx_data_frame[DATA_START+0] + EE_OFFSET_SLAVE2-30;
				}
				//ia|=ramka_rx[8]<<8;
				temp_var_b = rx_data_frame[DATA_START+2];
				//ib|=ramka_rx[10]<<8;
				tx_data_frame[LENGTH_L] = 12 + temp_var_b;
				temp_var_c = 11;
				for(; temp_var_b > 0; temp_var_b--){
					__asm__ volatile ("WDR");
					tx_data_frame[temp_var_c++] = eeprom_read(temp_var_16++);
				}

				
				tx_data_frame[DATA_START+0] = rx_data_frame[DATA_START+0];
				tx_data_frame[DATA_START+1] = rx_data_frame[DATA_START+1];
				tx_data_frame[DATA_START+2] = rx_data_frame[DATA_START+2];
				tx_data_frame[DATA_START+3] = rx_data_frame[DATA_START+3];
				break;
				
				case 4:		//Zapis EEPROM
				if(rx_data_frame[SLOT_ADDRESS] == slot_address1)
				temp_var_16 = rx_data_frame[DATA_START+0] + EE_OFFSET_SLAVE1;
				else if(rx_data_frame[SLOT_ADDRESS] == slot_address2){
					if(rx_data_frame[DATA_START+0]<30)
					temp_var_16 = rx_data_frame[DATA_START+0] + EE_OFFSET_SLAVE1;
					else temp_var_16 = rx_data_frame[DATA_START+0] + EE_OFFSET_SLAVE2-30;
				}
				//ia|=ramka_rx[8]<<8;
				temp_var_b = rx_data_frame[DATA_START+2];
				//ib|=ramka_rx[10]<<8;
				temp_var_c = 11;
				for(; temp_var_b > 0; temp_var_b--){
					__asm__ volatile ("WDR");
					eeprom_write(temp_var_16++, rx_data_frame[temp_var_c++]);
				}
				
				tx_data_frame[LENGTH_L] = 12;
				tx_data_frame[DATA_START+0] = rx_data_frame[DATA_START+0];
				tx_data_frame[DATA_START+1] = rx_data_frame[DATA_START+1];
				tx_data_frame[DATA_START+2] = rx_data_frame[DATA_START+2];
				tx_data_frame[DATA_START+3] = rx_data_frame[DATA_START+3];
				break;
				
				case 7:		//Funkcja wysylajaca opis panela
				uart_send_string(module_description);
				break;
				
				case 8:		//Podaj stan na MAX232 (10 nozka)
				if(rx_data_frame[DATA_START+0] == 0)
				MAX232_ERR_SET;
				if(rx_data_frame[DATA_START+0] != 0)
				MAX232_ERR_RST;
				
				tx_data_frame[LENGTH_L] = 9;
				tx_data_frame[DATA_START+0] = rx_data_frame[DATA_START+0];
				break;
				
				case 16:	//Wyslij wszystkie ADC w trybie podstawowym 0-20V 10BIT
				temp_var_c = 7;
				for(temp_var_a = 0; temp_var_a < 8; temp_var_a++){
					if(rx_data_frame[SLOT_ADDRESS] == slot_address1){
						RANGE_Set(temp_var_a,RANGE20V);
						ia = GET_ADC_Val(temp_var_a, REFERENCE_TYPE, BIT10);
					}
					else if(rx_data_frame[SLOT_ADDRESS] == slot_address2){
						RANGE_Set(temp_var_a,RANGE20V);
						ia = GET_ADC_Val(temp_var_a+8, REFERENCE_TYPE, BIT10);
					}else ia = 0;

					tx_data_frame[temp_var_c++] = ia;
					tx_data_frame[temp_var_c++] = ia >> 8;
				}
				tx_data_frame[LENGTH_L] = 24;
				break;
				
				case 18:	//Pomiar z korekcj¹ w trybie 10BIT
				temp_var_a = rx_data_frame[DATA_START+0];
				if(temp_var_a > 7) temp_var_a = 7;
				
				ia = Measure_Voltage_With_Correction(temp_var_a,&Resolution);
				ia = (float)ia * Resolution/0.020;
				
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
				
				case 19:	//Pomiar bez korekcji w ustalonym zakresie
				temp_var_a = rx_data_frame[DATA_START+0];
				if(temp_var_a > 7) temp_var_a = 7;
				
				if(rx_data_frame[DATA_START+1] == 4) RANGE = RANGE4V;
				else if(rx_data_frame[DATA_START+1] == 8) RANGE = RANGE8V;
				else if(rx_data_frame[DATA_START+1] == 16) RANGE = RANGE16V;
				else if(rx_data_frame[DATA_START+1] == 20) RANGE = RANGE20V;
				
				RANGE_Set(temp_var_a,RANGE);
				
				if(rx_data_frame[SLOT_ADDRESS] == slot_address1){
					ia = GET_ADC_Val(temp_var_a, REFERENCE_TYPE, BIT10);
				}
				else if(rx_data_frame[SLOT_ADDRESS] == slot_address2){;
					ia = GET_ADC_Val(temp_var_a+8, REFERENCE_TYPE, BIT10);
				}else ia = 0;
				
				RANGE_Set(temp_var_a,RANGE20V);
				
				temp_var_b = ia;
				temp_var_c = ia>>8;

				tx_data_frame[LENGTH_L] = 14;
				tx_data_frame[DATA_START+0] = temp_var_a;
				tx_data_frame[DATA_START+1] = rx_data_frame[SLOT_ADDRESS];
				tx_data_frame[DATA_START+2] = temp_var_b;
				tx_data_frame[DATA_START+3] = temp_var_c;
				tx_data_frame[DATA_START+4] = temp_var_b;
				tx_data_frame[DATA_START+5] = temp_var_c;
				break;
				
				//Pomiar w trybie rozszerzonym 10BIT/16BIT AutoRange 0 - 4V, 0 - 8V, 0 - 16V, 0 - 20V
				case 20:
				temp_var_a = rx_data_frame[DATA_START+0];
				if(temp_var_a > 7) temp_var_a = 7;
				
				ia = Measure_Voltage_With_Correction(temp_var_a,&Resolution);
				
				temp_var_b = ia;
				temp_var_c = ia>>8;
				memcpy(tx_data_frame+DATA_START+2,&Resolution,4);
				tx_data_frame[LENGTH_L] = 16;
				tx_data_frame[DATA_START+0] = temp_var_a;
				tx_data_frame[DATA_START+1] = 0;
				tx_data_frame[DATA_START+6] = temp_var_b;
				tx_data_frame[DATA_START+7] = temp_var_c;
				#if DEBUG
				tx_frame_send_flag = 0;
				#endif
				break;
				
				//Za³¹cz przekaŸnik wspólnej masy
				case 21:
				tx_data_frame[LENGTH_L] = 8;
				RELAY_GND_PORT |= RELAY_GND_PIN;
				break;
				
				//Roz³¹cz przekaŸnik wspólnej masy
				case 22:
				tx_data_frame[LENGTH_L] = 8;
				RELAY_GND_PORT &= ~RELAY_GND_PIN;
				break;
				
				case 132:	//Wejdz do bootloader'a
				tx_data_frame[LENGTH_L] = 9;
				tx_data_frame[DATA_START+0] = 0;
				bootloader_enter_flag = 1;
				break;
				
				case 200:	//Zeruj tablicê kalibracji
				temp_var_16 = EE_OFFSET_SLAVE1 + EE_OFFSET_CALIBRATION;
				temp_var_b = 0;
				
				for(temp_var_c = 0; temp_var_c < SIZE_TABLE_CALIBRATION; temp_var_c++)
				{
					eeprom_write(temp_var_16++, 127);
					temp_var_b = temp_var_b + 127;
					__asm__ volatile ("WDR");
				}
				eeprom_write(temp_var_16, temp_var_b);

				temp_var_16 = EE_OFFSET_SLAVE2;
				temp_var_b = 0;
				
				for(temp_var_c = 0; temp_var_c < SIZE_TABLE_CALIBRATION; temp_var_c++)
				{
					eeprom_write(temp_var_16++, 127);
					temp_var_b = temp_var_b + 127;
					__asm__ volatile ("WDR");
				}
				eeprom_write(temp_var_16, temp_var_b);
				
				tx_data_frame[LENGTH_L] = 8;
				break;
				
				case 201:	//£aduj tablicê kalibracji
				eeprom_read_calibration_all();
				tx_data_frame[LENGTH_L] = 8;
				break;
				
				case 202:	//Przelicz i zapisz sumê nastaw
				eeprom_write_checksum();
				
				temp_var_a = eeprom_verify_checksum();
				eeprom_status &= 0x0F;
				eeprom_status |= temp_var_a<<4;
				
				tx_data_frame[LENGTH_L] = 8;
				break;
				
				case 252:
				temp_var_a = rx_data_frame[DATA_START+0];
				temp_var_b = rx_data_frame[DATA_START+1];
				
				switch (temp_var_a)
				{
					//case 0: 	ubrrl_param = 416;	break;	//2400
					case 1:		ubrrl_param = 207;	break;	//4800
					case 2:		ubrrl_param = 103;	break;	//9600
					case 3: 	ubrrl_param = 68;	break;	//14400
					case 4:		ubrrl_param = 51;	break;	//19200
					case 5: 	ubrrl_param = 34;	break;	//28800
					case 6:		ubrrl_param = 25;	break;	//38400
					case 7: 	ubrrl_param = 16;	break;	//57600
					case 8: 	ubrrl_param = 12;	break;	//76800
					case 9: 	ubrrl_param = 8;	break;	//115200
					case 10: 	ubrrl_param = 7;	break;	//128000
					case 11: 	ubrrl_param = 3;	break;	//230400
					case 12: 	ubrrl_param = 3;	break;	//250000
					case 13: 	ubrrl_param = 1;	break;	//500000
					case 14: 	ubrrl_param = 0;	break;	//1000000
				}
				
				if(eeprom_status == 0)
				{
					eeprom_write(EE_OFFSET_UBRR, ubrrl_param);
					eeprom_write_checksum();
					tx_data_frame[DATA_START+0] = ubrrl_param;
				}
				else
				tx_data_frame[DATA_START+0] = 0xFF;
				
				tx_frame_send_flag = 0;
				tx_data_frame[LENGTH_L] = 8;
				if (temp_var_b == 1) tx_frame_send_flag = 1;
				break;
				
				case 254:
				MAX232_ERR_RST;
				tx_frame_send_flag = 0;
				break;
				
				case 255:
				init_uc_io();
				init_uc_peripherals();
				tx_frame_send_flag = 0;
				break;
				
				default:
				tx_frame_send_flag = 0;
				break;
			}
			
			//Wyœlij odpowiedŸ je¿eli ustawiona flaga tx_frame_send_flag
			if(tx_frame_send_flag == 1)
			{
				uint16_t i = 0xFFFF;
				uint16_t tx_data_length = ((tx_data_frame[LENGTH_H]<<8) | tx_data_frame[LENGTH_L]) - 1;
				uint8_t tx_sum = 0;
				while(++i < tx_data_length)
				{
					while( (REG_UCSRA & (1<<UDRE) ) == 0x00);
					REG_UDR = tx_data_frame[i];
					tx_sum += tx_data_frame[i];
				}
				while( (REG_UCSRA & (1<<UDRE) ) == 0x00);
				REG_UDR = tx_sum;
				while( (REG_UCSRA & (1<<UDRE) ) == 0x00);
			}
			LED_OFF;
			LED1_RESET;
		}

		if(RTS_PIN != rst_state)
		{
			LED_ON;
			rst_state = RTS_PIN;
			if(rst_state == 0)
			{
				REG_UBRRL = ubrrl_param;   //256000 bit/s
			}
			else
			{
				REG_UBRRL = 25;				//38400 bit/s
			}
			_delay_ms(100);
			LED_OFF;
		}
		
		//Resetuj licznik WATCHDOG
		__asm__ volatile ("WDR");

		if(bootloader_enter_flag == 1)
		{
			cli();
			BOOT_ENTER_JUMP;
		}
	}//while(1)
}//main

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
	LED_ON;
	
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
			LED_OK_SET;
			rx_frame_receive_flag = 1;
		}
		else
		LED_OFF;
		
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
			LED_OFF;
		}
	}
}

////////////////////////////////////////////////////////////////
// FUNKCJE
////////////////////////////////////////////////////////////////

//Pomiar wartoœci napiêcia z korekcja oraz uœrednianiem
uint16_t Measure_Voltage_With_Correction(uint8_t Channel, float * Res){
	float Factor_A = 0, Offset_B = 0, Resolution = 0;
	uint16_t Index = 0, Offset_Index = 0, Voltage_Point = 0, Data = 0;
	uint8_t Temp_Dev = 0, Temp_Dev_Next = 0, Calib_Value = 0;
	RANGE_Type Range = RANGE20V;
	
	if(rx_data_frame[SLOT_ADDRESS] == slot_address1){
		Data = GET_ADC_Val(Channel, REFERENCE_TYPE, BIT16);
		Offset_Index = 0;
	}
	else if(rx_data_frame[SLOT_ADDRESS] == slot_address2){
		Data = GET_ADC_Val(Channel, REFERENCE_TYPE, BIT16);
		Offset_Index = SIZE_TABLE_CALIBRATION;
	}
	else Data = 0;
	
	switch(Range){
		case RANGE4V:
		Resolution = 0.0000625;
		Voltage_Point = Data * Resolution;
		break;
		case RANGE8V:
		Resolution = 0.000125;
		Voltage_Point = Data * Resolution + 1;
		break;
		case RANGE16V:
		Resolution = 0.000250;
		Voltage_Point = Data * Resolution + 2;
		break;
		case RANGE20V:
		Resolution = 0.0003125;
		Voltage_Point = Data * Resolution + 3;
		break;
		default:
		Resolution = 0;
		break;
	}
	Index = (uint16_t)Channel * 24 + Voltage_Point + Offset_Index;
	
	Temp_Dev = calibration_table[Index];
	Temp_Dev_Next = calibration_table[Index];
	
	Factor_A = (float)(Temp_Dev_Next - Temp_Dev) * Resolution;
	Offset_B = (float)Temp_Dev - (float)Voltage_Point/Resolution*Factor_A;
	Calib_Value = round(Factor_A*Data + Offset_B);
	Data = Data + Calib_Value;
	Data = Data - 127;
	
	#if DEBUG
	uint64_t Voltage = 0;
	uint16_t Volatege_D = 0, Voltage_J = 0;
	uint8_t Text_Buf_Debug[50];
	
	Voltage = ((uint64_t)Data * 100000) * Resolution;
	Volatege_D = Voltage/100000;
	Voltage_J = Voltage%100000;
	sprintf((char*)Text_Buf_Debug,"ADC%u: %u.%4u RANGE:%u, CALIB: %d\n\r", Channel, Volatege_D, Voltage_J,Range,Calib_Value);
	UART_StringTransmit(Text_Buf_Debug);
	#endif
	
	*Res = Resolution;
	return Data;
}

// Pobierz wartoœæ ADC na danym kanale przy automatycznym dobieraniu zakresu
uint16_t GET_ADC_AutoRange(uint8_t CHANNEL, REF_Type REF_CFG, RANGE_Type * RANGE){
	uint64_t DATA_SUM=0;
	uint16_t DATA = 0;
	
	RANGE_Set(CHANNEL,RANGE20V);
	DATA = GET_ADC_Val(CHANNEL,REF_CFG,BIT16);
	*RANGE = RANGE20V;

/*
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
*/	
	for(uint16_t i = 0;i<100;i++)
	{
		DATA_SUM += GET_ADC_Val(CHANNEL,REF_CFG,BIT16);
		_delay_us(10);
	}
	
	return DATA_SUM / 100;
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
uint16_t GET_ADC_Val(uint8_t CHANNEL, REF_Type REF_CFG, RES_Type RES_CFG){
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
	
	if(RES_CFG == BIT10) return DATA>>6;
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
	LED_OFF;
	
	UART_DDR |= UART_ERR_PIN | UART_TX_PIN;
	UART_PORT|= UART_TX_PIN;
	
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
	uint8_t ascii_char = pgm_read_byte(string++);
	
	while(ascii_char != 0x00)
	{
		tx_data_frame[i++] = ascii_char;
		ascii_char = pgm_read_byte(string++);
	}
	tx_data_frame[LENGTH_L] = i + 1;
}

void eeprom_write(uint16_t address, uint8_t data)
{
	while(EECR & (1<<EEWE));
	EEAR = address;
	EEDR = data;
	EECR |= (1<<EEMWE);
	EECR |= (1<<EEWE);
}

uint8_t eeprom_read(uint16_t address)
{
	while(EECR & (1<<EEWE));
	EEAR = address;
	EECR |= (1<<EERE);
	return EEDR;
}

void eeprom_write_checksum(void)
{
	uint8_t suma = 0;
	for(uint8_t i = 0;i<PARAM_TABLE_SIZE;i++)
	suma += eeprom_read(EE_OFFSET_PARAM+i);
	
	eeprom_write(EE_OFFSET_PARAM_CHECKSUM, suma);
	eeprom_write(EE_OFFSET_PARAM_CHECKSUM_INV, ~suma);
}

uint8_t eeprom_verify_checksum(void)
{
	uint8_t suma = 0;
	for(uint8_t i = 0;i<PARAM_TABLE_SIZE;i++)
	suma += eeprom_read(EE_OFFSET_PARAM+i);
	
	if(suma != eeprom_read(EE_OFFSET_PARAM_CHECKSUM)) return PARAM_CHECKSUM_ERR;
	suma = ~suma;
	if(suma != eeprom_read(EE_OFFSET_PARAM_CHECKSUM_INV)) return PARAM_CHECKSUM_INV_ERR;
	
	return PARAM_CHECKSUM_OK;
}

uint8_t eeprom_read_calibration_table(SLAVE_Type slave){
	uint16_t calibration_eeprom_address = 0;
	uint16_t calibration_table_address = 0;
	uint8_t calibration_data_sum = 0;
	uint8_t calibration_read_value=0;

	if(slave == SLAVE1){
		calibration_table_address = 0;
		calibration_eeprom_address = EE_OFFSET_CALIBRATION + EE_OFFSET_SLAVE1;
	}
	else if(slave == SLAVE2){
		calibration_table_address = SIZE_TABLE_CALIBRATION;
		calibration_eeprom_address = EE_OFFSET_SLAVE2;
	}else return CALIB_CHECKSUM_ERR;

	for(uint8_t i = 0; i < SIZE_TABLE_CALIBRATION; i++)
	{
		calibration_read_value = eeprom_read(calibration_eeprom_address++);
		calibration_data_sum = calibration_data_sum + calibration_read_value;
		calibration_table[calibration_table_address++] = calibration_read_value;
		if((calibration_read_value < 102) || (calibration_read_value > 152))
		return CALIB_VALUE_ERR;
	}
	if(calibration_data_sum != eeprom_read(calibration_eeprom_address))
	return CALIB_CHECKSUM_ERR;
	
	return CALIB_OK;
}

void eeprom_read_calibration_all(void){
	CALIB_Type cab_err = 0;
	cab_err |= eeprom_read_calibration_table(SLAVE1);
	cab_err |= eeprom_read_calibration_table(SLAVE2);
	if(cab_err != CALIB_OK)
	{
		for(uint16_t i = 0; i < 2*SIZE_TABLE_CALIBRATION; i++)
		{
			calibration_table[i] = 127;
		}
	}
	
	eeprom_status &= 0xF0;
	eeprom_status |= cab_err;
}

