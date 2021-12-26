/*
 * miernik mocy na AD8317
 *
 * (c) 2020 Jacek Lipkowski <sq5bpf@lipkowski.org>
 * 
 *
 * Licensed GPLv3
 *
 * This file uses parts of the stdiodemo code from the avr libc library. 
 * Please observe the licensing terms (and while you're at it you can 
 * buy me a beer too :)/
 * This is the original license text: 
 * ----------------------------------------------------------------------------
 * "THE BEER-WARE LICENSE" (Revision 42):
 * <joerg@FreeBSD.ORG> wrote this file.  As long as you retain this notice you
 * can do whatever you want with this stuff. If we meet some day, and you think
 * this stuff is worth it, you can buy me a beer in return.        Joerg Wunsch
 * ----------------------------------------------------------------------------
 *
 * 
 */

#include "defines.h"

#include <ctype.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>

#include <avr/interrupt.h>
#include <avr/eeprom.h>


#include <util/delay.h>

#include "lcd.h"
#include "uart.h"



#define BANNER "AD8317 Meter"

#define VERSION "1.0"
#define COPYRIGHT1 "(c) 2020"
#define COPYRIGHT2 "Jacek Lipkowski"
#define EMAIL "SQ5BPF"

/* serial */
volatile uint8_t serial_bufptr=0;
volatile uint8_t serial_iscmd=0;
volatile uint8_t serial_invalidate=0;
volatile unsigned char serial_bufr[256];

#define CLOCK_DIV 64
#define HZ 500 //czestotliwosc w Hz
#define TIMER_DEL  (uint16_t) (0x10000UL-((F_CPU/CLOCK_DIV)/HZ))    

/* stany aplikacji */
#define STATE_INIT 0
#define STATE_ATT 1
#define STATE_SELECT 2
#define STATE_SAVE 3
#define STATE_CAL 4
#define STATE_LPF 5

volatile uint8_t app_state=STATE_INIT;


/* buttons */
#define BUTTON_NONE 0
#define BUTTON_ENT (1<<0)
#define BUTTON_ESC (1<<1)
volatile uint8_t button=BUTTON_NONE;
volatile uint8_t button_pressed=BUTTON_NONE;


//liczniki  0-lokalne czekanie 1-wyjscie z state_info
#define COUNTER_MAX 3
volatile uint16_t counter[COUNTER_MAX];

#define COUNTER_BANNER 1000 //2s
#define COUNTER_DISPLAY 100 //200ms
#define COUNTER_ENCODER 100 //200ms
#define COUNTER_ADC 5 //10ms


volatile float avf=0.05;

volatile uint16_t  cur_adc;
volatile float cur_adc_avg;


/* calibration */
#define MAX_CAL_ENTRIES 10
#define FLAG_VALID (1<<0)
struct __attribute__ ((packed)) calentry {
	float adc;
	float dbm;
	uint8_t flags;
};

volatile uint8_t current_cal=0;
volatile uint8_t selected_cal=0;
volatile uint8_t current_calentry=0;

float calibration_dbm;
volatile float att=0;

struct calentry calibration[MAX_CAL_ENTRIES];

#define MAX_CALIBRATIONS 10
uint16_t EEMEM eeprom_magic_flag;

struct calentry EEMEM saved_calibration[MAX_CALIBRATIONS][MAX_CAL_ENTRIES];

/* sort calibration table */
void sortcal(void)
{
	struct calentry tmpcal;
	uint8_t i;
	uint8_t run=1;

	while (run) {
		run=0;
		for(i=0;i<(MAX_CAL_ENTRIES-1); i++) {
			if (calibration[i].adc>calibration[i+1].adc) {
				memcpy(&tmpcal,&calibration[i],sizeof(struct calentry));
				memcpy(&calibration[i],&calibration[i+1],sizeof(struct calentry));
				memcpy(&calibration[i+1],&tmpcal,sizeof(struct calentry));
				run=1;
			}

		}
	}
}

/* clear calibration table */
void clearcal(void)
{
	memset(&calibration,0,sizeof(calibration));
}

/*   */
void tmpcal(void)
{
	/* 
	 * dbm     adc
	 -3      328.8
	 -10     361.8
	 -40     747.8
	 -50     870.0
	 */

	calibration[0].adc=328.8;
	calibration[0].dbm=-3;
	calibration[0].flags=FLAG_VALID;

	calibration[1].adc=361.8;
	calibration[1].dbm=-10;
	calibration[1].flags=FLAG_VALID;

	calibration[2].adc=747.8;
	calibration[2].dbm=-40;
	calibration[2].flags=FLAG_VALID;

	calibration[3].adc=870.0;
	calibration[3].dbm=-50;
	calibration[3].flags=FLAG_VALID;

}

#define EEPROM_MAGIC 0x55AA
#define INVALID_CALIBRATION 100

void write_eeprom_cal(uint8_t cal) {
	eeprom_write_block(&calibration,&saved_calibration[cal],sizeof(calibration));

}

void read_eeprom_cal(uint8_t cal) {
	eeprom_read_block(&calibration,&saved_calibration[cal],sizeof(calibration));
	sortcal();
}

void clear_eeprom_cal(uint8_t j) {
	struct calentry tmpcalibration[MAX_CAL_ENTRIES];
	memset(&tmpcalibration,0,sizeof(tmpcalibration));
	eeprom_write_block(&tmpcalibration,&saved_calibration[j],sizeof(tmpcalibration));
}


void init_cfg(void) {
	if (eeprom_read_word(&eeprom_magic_flag)==EEPROM_MAGIC) {
		current_cal=0;
		read_eeprom_cal(current_cal);
		return;
	}
	tmpcal();
	write_eeprom_cal(0);
	eeprom_write_word(&eeprom_magic_flag,EEPROM_MAGIC);
	current_cal=INVALID_CALIBRATION;
}

float convcal(float adc)
{
	int8_t i;
	uint8_t minc=255;
	uint8_t maxc=255;

	for (i=0;i<MAX_CAL_ENTRIES; i++) {
		if ((calibration[i].flags&FLAG_VALID)&&(calibration[i].adc==adc)) return(calibration[i].dbm);
	}
	for (i=0;i<MAX_CAL_ENTRIES; i++) {
		if ((calibration[i].flags&FLAG_VALID)&&(calibration[i].adc<adc)) minc=i;
	}
	for (i=(MAX_CAL_ENTRIES-1); i>=0; i--) {
		if ((calibration[i].flags&FLAG_VALID)&&(calibration[i].adc>adc)) maxc=i;
	}
	if ((minc!=255)&&(maxc!=255)) {
		return(((float)(adc-calibration[minc].adc)/(float)(calibration[maxc].adc-calibration[minc].adc))*(calibration[maxc].dbm-calibration[minc].dbm)+calibration[minc].dbm);
	}

	if (minc!=255) {
		for (i=minc-1;i>=0; i--) {
			if  (calibration[i].flags&FLAG_VALID)
			{
				return(((float)(adc-calibration[minc].adc)/(float)(calibration[i].adc-calibration[minc].adc))*(calibration[i].dbm-calibration[minc].dbm)+calibration[minc].dbm);
			}
		}
	}

	if (maxc!=255) {
		for (i=maxc+1; i<MAX_CAL_ENTRIES; i++) {
			if  (calibration[i].flags&FLAG_VALID)
			{
				return(((float)(adc-calibration[maxc].adc)/(float)(calibration[i].adc-calibration[maxc].adc))*(calibration[i].dbm-calibration[maxc].dbm)+calibration[maxc].dbm);
			}
		}
	}


	return(666);
}


float convcal2(float adc) 
{
	int8_t i;
	uint8_t minc=255;
	uint8_t maxc=255;


	for (i=0;i<MAX_CAL_ENTRIES; i++) {
		if ((calibration[i].flags&FLAG_VALID)&&(calibration[i].adc==adc)) return(calibration[i].dbm);
	}
	for (i=0;i<MAX_CAL_ENTRIES; i++) {
		if ((calibration[i].flags&FLAG_VALID)&&(calibration[i].adc<adc)) minc=i;
	}
	for (i=(MAX_CAL_ENTRIES-1); i>=0; i--) {
		if ((calibration[i].flags&FLAG_VALID)&&(calibration[i].adc>adc)) maxc=i;
	}
	if ((minc!=255)&&(maxc!=255)) {
		return(((float)(adc-calibration[minc].adc)/(float)(calibration[maxc].adc-calibration[minc].adc))*(calibration[maxc].dbm-calibration[minc].dbm)+calibration[minc].dbm);
	}

	if (minc!=255) {
		for (i=(minc+1);i<MAX_CAL_ENTRIES; i++) {
			if  (calibration[i].flags&FLAG_VALID)
			{
				return(((float)(adc-calibration[minc].adc)/(float)(calibration[i].adc-calibration[minc].adc))*(calibration[i].dbm-calibration[minc].dbm)+calibration[minc].dbm);
			}
		}
	}

	if (maxc!=255) {
		for (i=MAX_CAL_ENTRIES-1; i>maxc; i--) {
			if  (calibration[i].flags&FLAG_VALID)
			{
				return(((float)(adc-calibration[maxc].adc)/(float)(calibration[i].adc-calibration[maxc].adc))*(calibration[i].dbm-calibration[maxc].dbm)+calibration[maxc].dbm);
			}
		}
	}


	return(666);
}

	static void
ioinit(void)
{
	uart_init();
	lcd_init();
}


volatile int8_t encoder_val;
volatile uint8_t encoder_speed;
volatile uint8_t encoder_grey;
volatile uint8_t encoder_prevgrey;

uint8_t read_gray_code_from_encoder(void )
{
	uint8_t val=0;

	if(!bit_is_clear(PIND, PD4))
		val |= (1<<1);

	if(!bit_is_clear(PIND, PD5))
		val |= (1<<0);

	return val;
}


ISR (TIMER1_OVF_vect)
{
	unsigned char i;
	uint8_t val_tmp; 

	if (bit_is_clear(PIND, PD7)) { 
		button|=BUTTON_ENT; 
	} else {
		if (button&BUTTON_ENT) button_pressed|=BUTTON_ENT;
	}
	if (bit_is_clear(PIND, PD6)) {
		button|=BUTTON_ESC;
	} else {
		if (button&BUTTON_ESC) button_pressed|=BUTTON_ESC;
	}

	TCNT1 = TIMER_DEL;
	for (i = 0; i < COUNTER_MAX; i++)
	{	if (counter[i])
		counter[i]--;
	}

	val_tmp = read_gray_code_from_encoder(); 
	//if (val_tmp==encoder_prevgrey) 
	if (1)
	{
		if(encoder_grey != val_tmp) 
		{ 
			if( /*(val==2 && val_tmp==3) ||*/ 
					(encoder_grey==3 && val_tmp==1) || 
					/*(val==1 && val_tmp==0) ||*/ 
					(encoder_grey==0 && val_tmp==2) 
			  ) 
			{ 
				encoder_val++;
				if (encoder_speed<20) 	encoder_speed++;

				//uart_putchar('+',stdout);
			} 
			else if( /*(val==3 && val_tmp==2) ||*/ 
					(encoder_grey==2 && val_tmp==0) || 
					/*(val==0 && val_tmp==1) ||*/ 
					(encoder_grey==1 && val_tmp==3) 
			       ) 
			{ 
				encoder_val--;
				if (encoder_speed<20) 	encoder_speed++;
				//uart_putchar('-',stdout);
			} 

			encoder_grey = val_tmp; 
		} 

	} else {
		encoder_prevgrey = val_tmp;
	}
	/*
	   float step=0.1;
	   if (encoder_speed>15) step=1;  

	 * if (app_state&STATE_ATT) {
	 if (encoder_val>2) { att=att+step; encoder_val=0; }
	 if (encoder_val<-2) { att=att-step; encoder_val=0; }
	 }
	 */
}


	void
timer_init (void)
{
	TCCR1A = 0;
	TCCR1B = _BV (CS11) | _BV (CS10);
	TCNT1 = TIMER_DEL;

	TIMSK1 = _BV (TOIE1);



}



FILE uart_str = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);
FILE lcd_str = FDEV_SETUP_STREAM(lcd_putchar, NULL, _FDEV_SETUP_WRITE);

	static void
delay_1s(void)
{
	uint8_t i;

	for (i = 0; i < 100; i++)
		_delay_ms(10);
}

uint16_t read_adc2(uint8_t chan) {
	uint8_t low,high;

	ADMUX = (1<<REFS0) | (1<<REFS1) | (chan & 0x0f);  //select input and ref
	ADCSRA |= (1<<ADSC);                 //start the conversion
	while (ADCSRA & (1<<ADSC));          //wait for end of conversion
	low=ADCL;  // reading ADCL locks both of these
	high=ADCH; // until reading ADCH; seems to ensure the value comes from the same conversion.
	return (high<<8) | low;
}

uint16_t read_adc() {
	uint8_t low,high;
	ADCSRA |= _BV(ADSC); 
	while (bit_is_set(ADCSRA, ADSC))
		; // ADSC is cleared when the conversion finishes   
	_delay_us(1);
	low=ADCL;  // reading ADCL locks both of these
	high=ADCH; // until reading ADCH; seems to ensure the value comes from the same conversion.
	return (high<<8) | low;
}

void init_adc(void)
{
	ADMUX = (1<<REFS0)|(1<<REFS1);     //select AVCC as reference
	ADCSRA = (1<<ADEN) | 7 ;  //enable and prescale = 128 (16MHz/128 = 125kHz)
	//DIDR0 = 0x3f;
	// DIDR0 = 0b11000000;
}




void update_display(void) {
	//	char pref[]="pnum k";
	//	char *cur_prefix=(char *)&pref;
	float dbmr;
	float dbm;
	float dbm2;
	uint8_t i;

	struct {
		char name;
		int8_t pwr;
	}
	siexps[11] = {
		{ 'a',-18},
		{ 'f',-15},
		{ 'p',-12},
		{ 'n',-9},
		{ 'u',-6},
		{ 'm',-3},
		{ ' ',0},
		{ 'k',3},
		{ 'M',6},
		{ 'G',9},
		{ 'T',12}

	};


	dbmr= convcal(cur_adc_avg);

	dbm= dbmr-att;


	lcd_setpos(LCD_LINE1);
	fprintf_P (stderr, PSTR ("%5.1f %+5.1f = "), dbmr,-att);
	lcd_setpos(LCD_LINE2);
	fprintf_P (stderr, PSTR ("%5.1fdBm "), dbm);

	for (i=0;i<11;i++)
	{
		dbm2=dbm-10*siexps[i].pwr-30;
		if ((dbm2>=0)&&(dbm2<=30))
		{
			fprintf_P (stderr, PSTR ("%5.1f%cW"), powf(10,(dbm2/10.0)),siexps[i].name);
			break;

		}

	}
	lcd_setpos(LCD_LINE3);
	fprintf_P (stderr, PSTR ("adc:%-4.0f e:%-3.0i"), 1024-cur_adc_avg,encoder_speed);

	lcd_setpos(LCD_LINE4);

	switch  (app_state) {
		case  STATE_INIT: fprintf_P (stderr, PSTR ("INIT ")); break;
		case STATE_ATT:
				  fprintf_P (stderr, PSTR ("ATT  "));
				  break;
		case STATE_LPF:
				  fprintf_P (stderr, PSTR ("LPF:%4.4f "),avf);
				  break;
		case  STATE_SELECT:
				  fprintf_P (stderr, PSTR ("SEL  CAL:%2i "),selected_cal);
				  break;
		case STATE_SAVE:
				  fprintf_P (stderr, PSTR ("SAVE CAL:%2i "),selected_cal);
				  break;
		case STATE_CAL:
				  fprintf_P (stderr, PSTR ("CAL:%2i %-6.1fdBm"),current_calentry,calibration_dbm);
				  break;
		default:
				  fprintf_P (stderr, PSTR ("HGW??? "));
				  break;


	}



}

void handle_adc() {
	cur_adc=read_adc2(7);
	cur_adc_avg=cur_adc_avg*(1.0-avf)+(float)cur_adc*avf;
}

void handle_encoder(void)
{
	uint8_t ok=0;
	uint8_t up;
	float step=0.1;

	if (encoder_speed>15) step=1;  
	if (encoder_val>2) { ok=1; up=1; encoder_val=0; }
	if (encoder_val<-2) { ok=1; up=0; encoder_val=0; }

	switch (app_state)
	{
		case STATE_ATT:
			if (ok) {
				if (up) { att=att+step; } else { att=att-step; }
			}
			break;

		case STATE_CAL:
			if (ok) {
				if (up) { calibration_dbm=calibration_dbm+step; } else { calibration_dbm=calibration_dbm-step; }
			}
			break;

		case STATE_LPF:
			if (ok) {
				if (up) { 
					if (avf<0.5) { avf=avf*2; } 
				} else { 
					if (avf>0.0005) { avf=avf/2; } 
				}
			}
			break;
		case STATE_SELECT:
		case STATE_SAVE:
			if (ok)
			{
				if (up) { 
					selected_cal=(selected_cal+1)%MAX_CAL_ENTRIES;

				} else { 
					if (selected_cal) { selected_cal--; } else { selected_cal=MAX_CAL_ENTRIES-1; }
				}	
			}	
			break;

	}
}

void  handle_buttons(void) {
	if (button_pressed&BUTTON_ESC)
	{
		button_pressed=button_pressed&~BUTTON_ESC;
		button=BUTTON_NONE;
		encoder_speed=0;
		encoder_val=0;
		switch  (app_state) {
			case  STATE_INIT: app_state=STATE_ATT;  break;
			case STATE_ATT: app_state=STATE_SELECT; break;
			case  STATE_SELECT: app_state=STATE_SAVE; selected_cal=current_cal; break;
			case STATE_SAVE: 
					    app_state=STATE_CAL; 
					    selected_cal=current_cal; 
					    current_calentry=0; 
					    calibration_dbm=0; 
					    clearcal();
					    break;
			case STATE_CAL: 
					    if (!current_calentry) read_eeprom_cal(current_cal); 
					    app_state=STATE_LPF; 
					    break;
			case STATE_LPF: app_state=STATE_ATT; break;
			default:
					break;
		}
		lcd_clearline(LCD_LINE4);

	}
	if (button_pressed&BUTTON_ENT)
	{
		button_pressed=button_pressed&~BUTTON_ENT;
		button=BUTTON_NONE;
		encoder_speed=0;
		encoder_val=0;
		switch  (app_state) {
			case STATE_SELECT: 
				read_eeprom_cal(selected_cal);	
				current_cal=selected_cal;
				app_state=STATE_ATT;
				lcd_clearline(LCD_LINE4);
				break;

			case STATE_SAVE: 
				write_eeprom_cal(selected_cal);       
				current_cal=selected_cal;
				app_state=STATE_ATT;
				lcd_clearline(LCD_LINE4);
				break;
			case STATE_CAL: 
				calibration[current_calentry].dbm=calibration_dbm;
				calibration[current_calentry].adc=cur_adc_avg;
				calibration[current_calentry].flags=FLAG_VALID;
				if (current_calentry<MAX_CAL_ENTRIES) current_calentry++;

				break;

			default: break;
		}

	}
}

void serial_prompt(void) {
	printf_P(PSTR("\n> "));
}

void serial_banner (void) {
	printf_P(PSTR(BANNER " " VERSION "\n" COPYRIGHT1 " " COPYRIGHT2 " " EMAIL "\n"));
}

void serial_dumpcal(uint8_t j) {
	uint8_t i;
	struct calentry tmpcalibration[MAX_CAL_ENTRIES];
	eeprom_read_block(&tmpcalibration,&saved_calibration[j],sizeof(tmpcalibration));
	printf_P(PSTR("===  CALIBRATION %i  ===\n"),j) ;

	for (i=0;i<MAX_CAL_ENTRIES;i++) {
		printf_P(PSTR("%i:\tadc:%6.1f\tdbm:%6.1f\tflags:%2.2x\r\n"),i,tmpcalibration[i].adc,tmpcalibration[i].dbm,tmpcalibration[i].flags);
	}
}

void serial_currentcal() {
	uint8_t i;
	printf_P(PSTR("===  CURRENT CALIBRATION  ===\n")) ;

	for (i=0;i<MAX_CAL_ENTRIES;i++) {
		printf_P(PSTR("%i:\tadc:%6.1f\tdbm:%6.1f\tflags:%2.2x\r\n"),i,calibration[i].adc,calibration[i].dbm,calibration[i].flags);
	}
}
void serial_readadc() {
	printf_P(PSTR("adc: %6.1f  current_adc: %i\n"),cur_adc_avg,cur_adc) ;
}
void serial_showlpf() {
	printf_P(PSTR("lpf: %f\n"),avf) ;
}


void handle_serial(void) {
	float tadc;
	float tdbm;
	uint8_t tflags;
	uint8_t i,j;
	struct calentry tmpcalibration[MAX_CAL_ENTRIES];

	uart_putchar('\n',stdout);
	serial_bufr[serial_bufptr]=0;
	if (strncmp_P(&serial_bufr,PSTR("help"),4)==0) {
		printf_P(PSTR("help: - show help\nver -  show version\ndumpcal - dump all calibration settings\nshowcal N - show calibration N (0-%i)\n"),MAX_CALIBRATIONS-1);
		printf_P(PSTR("currentcal - show current calibration\nsavecal N - save current calibration as calibration entry  N (0-%i)\n"),MAX_CALIBRATIONS-1);
		printf_P(PSTR("clearcur - clear current calibration\nclercal N - clear calibration entry  N (0-%i)\n"),MAX_CALIBRATIONS-1);
		printf_P(PSTR("readcal N - use calibration entry  N (0-%i)\neditcal I ADC DBM FLAGS - edit current calibration entry I"),MAX_CALIBRATIONS-1,MAX_CALIBRATIONS-1);
		printf_P(PSTR("readadc - read adc\nshowlpf - read lowpass filter parameters\nsetlpf N - set lowpass filter parameter N\n"));

	}
	if (strncmp_P(&serial_bufr,PSTR("version"),7)==0) {
		serial_banner();
	}
	if (strncmp_P(&serial_bufr,PSTR("dumpcal"),7)==0) {
		for (j=0;j<MAX_CALIBRATIONS;j++) { 
			serial_dumpcal(j);
		}
	}
	if (strncmp_P(&serial_bufr,PSTR("showcal "),8)==0) {
		i=sscanf_P(&serial_bufr,PSTR("showcal %i"),&j);
		if  ((i>0)&&(j>=0)&&(j<MAX_CALIBRATIONS)) { serial_dumpcal(j);
		} else {
			printf_P(PSTR("invalid: [%s]\n"),serial_bufr);
		}
	}
	if (strncmp_P(&serial_bufr,PSTR("currentcal"),10)==0) {
		serial_currentcal();
	}
	if (strncmp_P(&serial_bufr,PSTR("savecal "),8)==0) {
		i=sscanf_P(&serial_bufr,PSTR("savecal %i"),&j);
		if ((i>0)&&(j>=0)&&(j<MAX_CALIBRATIONS)) { write_eeprom_cal(j); 
		} else {
			printf_P(PSTR("invalid: [%s]\n"),serial_bufr);
		}
	}
	if (strncmp_P(&serial_bufr,PSTR("clearcur"),8)==0) {
		clearcal();
	}
	if (strncmp_P(&serial_bufr,PSTR("clearcal "),9)==0) {
		i=sscanf_P(&serial_bufr,PSTR("clearcal %i"),&j);
		if ((i>0)&&(j>=0)&&(j<MAX_CALIBRATIONS)) { 
			clear_eeprom_cal(j); 
		} else {
			printf_P(PSTR("invalid: [%s]\n"),serial_bufr);
		}
	}
	if (strncmp_P(&serial_bufr,PSTR("readcal "),8)==0) {
		i=sscanf_P(&serial_bufr,PSTR("readcal %i"),&j);
		if ((i>0)&&(j>=0)&&(j<MAX_CALIBRATIONS)) { 
			current_cal=j;
			read_eeprom_cal(current_cal);
		} else {
			printf_P(PSTR("invalid: [%s]\n"),serial_bufr);
		}
	}
	if (strncmp_P(&serial_bufr,PSTR("editcal "),8)==0) {
		i=sscanf_P(&serial_bufr,PSTR("editcal %i %f %f %2.2x"),&j,&tadc,&tdbm,&tflags);
		if ((i>0)&&(j>=0)&&(j<MAX_CALIBRATIONS)) {
			calibration[j].adc=tadc;
			calibration[j].dbm=tdbm;
			calibration[j].flags=tflags;
			sortcal();
		} else {
			printf_P(PSTR("invalid: [%s]\n"),serial_bufr);
		}
	}
	if (strncmp_P(&serial_bufr,PSTR("readadc"),7)==0) {
		serial_readadc();
	}

	if (strncmp_P(&serial_bufr,PSTR("showlpf"),7)==0) {
		serial_showlpf();
	}
	if (strncmp_P(&serial_bufr,PSTR("setlpf "),7)==0) {
		i=sscanf_P(&serial_bufr,PSTR("setlpf %f"),&tadc);
		if ((tadc>0)&&(tadc<=1)) {
			printf_P(PSTR("setting LPF to %f\n"),tadc);

			avf=tadc;
		} else {
			printf_P(PSTR("invalid: [%s]\n"),serial_bufr);
		}

	}

	uart_putchar('\n',stdout);
	serial_bufptr=0;
	serial_iscmd=0;
	serial_prompt();

}

void lcd_banner(void) 
{
	lcd_setpos(LCD_LINE1);
	fprintf_P (stderr, PSTR (BANNER " " VERSION));
	lcd_setpos(LCD_LINE2);
	fprintf_P (stderr, PSTR (COPYRIGHT1));
	lcd_setpos(LCD_LINE3);
	fprintf_P (stderr, PSTR (COPYRIGHT2));
	lcd_setpos(LCD_LINE4);
	fprintf_P (stderr, PSTR (EMAIL));
	counter[0]=COUNTER_BANNER;
	while(counter[0]);
	lcd_clear();
}

int main(void)
{
	uint8_t i;
	char buf[20], s[20];


	encoder_speed=0;
	init_adc();
	PORTD= _BV(PD4) | _BV(PD5) | _BV(PD6) | _BV(PD7);
	timer_init();
	clearcal();
	//	tmpcal();
	//	sortcal();
	init_cfg();
	uart_init();
	sei();

	stdout = stdin = &uart_str;
	stderr = &lcd_str;

	serial_banner();
	lcd_init();

lcd_banner();

	counter[0]=COUNTER_DISPLAY;
	counter[1]=COUNTER_ADC;
	counter[2]=COUNTER_ENCODER;

	ADMUX=0b00000111;

	app_state=STATE_ATT;

	serial_prompt();

	for (;;)
	{
		if (!counter[0]) {
			counter[0]=COUNTER_DISPLAY;
			update_display();	

		}

		if (!counter[1]) {
			counter[1]=COUNTER_ADC;
			handle_adc();

		}

		if (!counter[2]) {
			counter[2]=COUNTER_ENCODER;

			if (encoder_speed>5) {  encoder_speed=encoder_speed-5; }
		}
		if (serial_invalidate) {
			printf_P(PSTR("\n^C\n"));
			serial_prompt();
			serial_invalidate=0;
		}

		if (serial_iscmd) handle_serial();

		handle_encoder();
		handle_buttons();

	}

	return 0;
}
