#include "stm8s.h"
#include "milis.h"
#include "stm8_hd44780.h"
#include "stdio.h"
#include "delay.h"

void delay_ms(uint16_t ms)
{
    for (int16_t i = 0; i < ms; i++) {
        _delay_us(250);
        _delay_us(250);
        _delay_us(250);
        _delay_us(243);
    }
}

void init_enc(void);
void process_enc(void);
void init_timer(void);
void display_time(void);
void counting(void);
void remove_cursor(void);
void print_cursor(void);
void print_button(void);
void add_time(void);
void remove_time(void);

void init_spi(void);
void test(uint8_t* data, uint16_t delka);
void led_posun(uint8_t* data, uint16_t delka);

// test pattern for (8 RGB LED ring)
uint8_t colors[48]={
0x10,0x00,0x00, // B
0x00,0x10,0x00, // R
0x00,0x00,0x10, // G
0x00,0x00,0x00, // black
0x10,0x20,0x00, //
0x10,0x20,0x00, //
0x10,0x20,0x00, //
0x10,0x20,0x00, //
0x10,0x00,0x00, // B
0x00,0x10,0x00, // R
0x00,0x00,0x10, // G
0x00,0x00,0x00, // black
0x10,0x20,0x00, //
0x10,0x20,0x00, //
0x10,0x20,0x00, //
0x10,0x20,0x00, //
};
uint8_t* colors2;
uint8_t posun_posledne=0;
#define rychlost 500;
//colors2 = colors + 3;
#define L_PATTERN 0b01110000 // 3x125ns (8MHZ SPI)
#define H_PATTERN 0b01111100 // 5x125ns (8MHZ SPI), first and last bit must be zero (to remain MOSI in Low between frames/bits)


#define readA	GPIO_ReadInputPin(GPIOC,GPIO_PIN_4)//definování pinů enkodéru
#define readB GPIO_ReadInputPin(GPIOD,GPIO_PIN_3)
#define readC	GPIO_ReadInputPin(GPIOE,GPIO_PIN_5)

#define button1_position	0	//pozice tlačítka pause/unpause
#define button2_position	1	//reset
#define button3_position	2	//pozice tlačítka up/down
#define display_position	4	//pozice nejvyšší cifry displeje

char text[24];
int16_t x=1;				//pozice kurzoru
uint16_t minule=7;	//zaznamenání pozice enkodéru
int16_t cifra1=0;	//minuty a vteřiny (0-3599)
int16_t cifra2=0;	 	//hodiny (0-99)
uint16_t pause=1;				//řídí zastavování minutek 0=zapnuté 1=zastavené 2=změna času
uint16_t updown=1;			//počítání nahoru/dolů
uint16_t pause_minule=0; //pomocná proměnná pro tlačítko
uint16_t update_display=1;	//prvotní zápis na displej
uint16_t save_cifra1=0;
uint16_t save_cifra2=0;
uint16_t save_updown=0;

const uint8_t symbol1[8]={	//symbol
0b00000,
0b11000,
0b11110,
0b11111,
0b11110,
0b11000,
0b00000,
0b00000
};
const uint8_t symbol2[8]={	//symbol
0b00000,
0b01010,
0b01010,
0b01010,
0b01010,
0b01010,
0b00000,
0b00000
};
const uint8_t symbol3[8]={	//symbol
0b00100,
0b01110,
0b10101,
0b00100,
0b00100,
0b00100,
0b00100,
0b00000,
};
const uint8_t symbol4[8]={	//symbol
0b00100,
0b00100,
0b00100,
0b00100,
0b10101,
0b01110,
0b00100,
0b00000,
};
const uint8_t symbol5[8]={	//symbol
0b11111,
0b00000,
0b00000,
0b00000,
0b00000,
0b00000,
0b00000,
0b00000,
};
const uint8_t symbol6[8]={	//symbol
0b00000,
0b01110,
0b00001,
0b01101,
0b10001,
0b01110,
0b00000,
0b00000,
};

void main(void){
CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1); // 16MHz z interního RC oscilátoru
init_milis(); // milis kvuli delay_ms()
init_enc();		// inicializace vstupu enkodéru
lcd_init();		// inicializace displeje
init_spi();
lcd_store_symbol(0,symbol1); //uložit symbol
lcd_store_symbol(1,symbol2); //uložit symbol
lcd_store_symbol(2,symbol3); //uložit symbol
lcd_store_symbol(3,symbol4); //uložit symbol
lcd_store_symbol(4,symbol5); //uložit symbol
lcd_store_symbol(5,symbol6); //uložit symbol
save_cifra1=cifra1;
save_cifra2=cifra2;
save_updown=updown;
init_timer();	// spustí tim3 s poerušením každé 2ms
enableInterrupts(); // není nutné, protože tuto funkci voláme v init_milis()

lcd_clear();	//reset displeje
print_cursor();	//nastavit kurzor

lcd_gotoxy(display_position+5,0); //neměnné znaky
lcd_puts(":");
lcd_gotoxy(display_position+2,0);
lcd_puts(":");
lcd_gotoxy(button2_position,0);
lcd_putchar(5);
print_button();		//funkční proměnné tlačítka

  while (1){
		if(update_display==1){
			display_time();	//přepsat cifry
			update_display=0;
		}
		test(colors,sizeof(colors)); 	// transfer image into RGB LED string
  		delay_ms(5);  
	}
}
 INTERRUPT_HANDLER(TIM3_UPD_OVF_BRK_IRQHandler, 15)	//interrupt pro vstupy na ekodéru
 {
	 TIM3_ClearITPendingBit(TIM3_IT_UPDATE);
	 process_enc();
 }
 INTERRUPT_HANDLER(TIM2_UPD_OVF_BRK_IRQHandler, 15)	//interrupt pro počítání času
 {
	 TIM2_ClearITPendingBit(TIM2_IT_UPDATE);
	 counting();
 }

// takes array of LED_number * 3 bytes (RGB per LED)
void test(uint8_t* data, uint16_t length){
 uint8_t mask;
 disableInterrupts(); // can be omitted if interrupts do not take more then about ~25us
 while(length){   // for all bytes from input array
  length--;
  mask=0b10000000; // for all bits in byte
  while(mask){
   while(!(SPI->SR & SPI_SR_TXE)); // wait for empty SPI buffer
   if(mask & data[length]){ // send pulse with coresponding length ("L" od "H")
    SPI->DR = H_PATTERN;
   }else{
    SPI->DR = L_PATTERN;
   }
   mask = mask >> 1;
  }
 }
 enableInterrupts();
while(SPI->SR & SPI_SR_BSY); // wait until end of transfer - there should come "reset" (>50us in Low)
}
void led_posun(uint8_t* data, uint16_t length){
 uint8_t mask;
 disableInterrupts(); // can be omitted if interrupts do not take more then about ~25us
 while(length){   // for all bytes from input array
  length--;
  mask=0b10000000; // for all bits in byte
  while(mask){
   while(!(SPI->SR & SPI_SR_TXE)); // wait for empty SPI buffer
   if(mask & data[length]){ // send pulse with coresponding length ("L" od "H")
    SPI->DR = H_PATTERN;
   }else{
    SPI->DR = L_PATTERN;
   }
   mask = mask >> 1;
  }
 }
 enableInterrupts();
while(SPI->SR & SPI_SR_BSY); // wait until end of transfer - there should come "reset" (>50us in Low)
}


void init_spi(void){
// Software slave managment (disable CS/SS input), BiDirectional-Mode release MISO pin to general purpose
SPI->CR2 |= SPI_CR2_SSM | SPI_CR2_SSI | SPI_CR2_BDM | SPI_CR2_BDOE; 
SPI->CR1 |= SPI_CR1_SPE | SPI_CR1_MSTR; // Enable SPI as master at maximum speed (F_MCU/2, there 16/2=8MHz)
}


void init_timer(void){	//zapnout interrupt
TIM3_TimeBaseInit(TIM3_PRESCALER_16,1999);
TIM3_ITConfig(TIM3_IT_UPDATE, ENABLE);
TIM3_Cmd(ENABLE);
TIM2_TimeBaseInit(TIM2_PRESCALER_1024,15625);	//1x za vteřinu
TIM2_ITConfig(TIM2_IT_UPDATE, ENABLE);
TIM2_Cmd(ENABLE);
}

void init_enc(void){	//povolit piny pro enkodér
GPIO_Init(GPIOE,GPIO_PIN_5,GPIO_MODE_IN_PU_NO_IT); //enkodér
GPIO_Init(GPIOC,GPIO_PIN_6,GPIO_MODE_IN_PU_NO_IT); //enkodér
GPIO_Init(GPIOC,GPIO_PIN_7,GPIO_MODE_IN_PU_NO_IT); //tlačítko enkodéru
}

void process_enc(void){	//enkodér
	if(readA == RESET && readB == RESET){	//detekce rozepnutého stavu
		minule=0;
	}
	if(readA != RESET && readB != RESET){	//detekce sepnutého stavu
		minule=1;
	}
	if((minule==1 && readA!=RESET && readB == RESET) ||(minule==0 && readA==RESET && readB != RESET)){	//logika pro otáčení clockwise
		if(pause!=2){	
			remove_cursor();	//odstranění starého kurzoru
			minule=7;	//zapsat posunutí, ochrana proti opakování funkce
			x++;	//přičíst
			if(x>15){
				x=15;
			}
			print_cursor();	//napsat nový kurzor
		}
		if(pause==2){
			add_time();
			minule=7;
		}
	}
	if((minule==1 && readA==RESET && readB != RESET)||(minule==0 && readA!=RESET && readB == RESET)){	//logika pro otáčení counterclockwise
		if(pause!=2){	
			remove_cursor();	//odstranění starého kurzoru
			minule=7;	//zapsat posunutí, ochrana proti opakování funkce
			x--;	//odečíst
			if(x<0){
				x=0;
			}
				print_cursor();	//napsat nový kurzor
		}
		if(pause==2){
			remove_time();
			minule=7;
		}
	}
	if(readC==RESET && pause_minule==0){	//detekce zmáčknutí enkodéru
		if(x==button1_position){	//funkce časuj/pauza
			pause++;
			if(pause>1){
				pause=0;
			}
			if(pause==0){
				save_cifra1=cifra1;
				save_cifra2=cifra2;
				save_updown=updown;
			}
			if(updown==0){
				TIM2_SetCounter(15625);
			}
			if(updown==1){
				TIM2_SetCounter(0);
			}
			print_button();	//přepsat proměnné tlačítka
		}
		if(x==button2_position){
			cifra1=save_cifra1;
			cifra2=save_cifra2;
			updown=save_updown;
			pause=1;
			update_display=1;
			print_button();
		}
		if(x==button3_position){	//funkce počítej nahoru/dolů
			updown++;
			if(updown>1){
				updown=0;
			}
			print_button();	//přepsat proměnné tlačítka
		}
		else if((pause!=0)&&(x>=display_position)&&(x<=display_position+7)&&(x!=display_position+2)&&(x!=display_position+5)){	//měnit jednotlivé cifry
			pause++;
			if(pause>2){
				pause=1;
			}
			print_cursor();
		}
		pause_minule=1;	//pomocná proměnná aby se neopakovala funkce
	}
	if(readC!=RESET && pause_minule==1){	//reset pomocné proměnné
		pause_minule=0;
	}
}
void display_time(void){	//přepsat cifry
	disableInterrupts();
	
	lcd_gotoxy(display_position+0,0);
	sprintf(text,"%i",((cifra2/10)%10)); // poiprav text na displej
	lcd_puts(text); // vypiš poipravený text
	
	lcd_gotoxy(display_position+1,0);
	sprintf(text,"%i",((cifra2%10))); // poiprav text na displej
	lcd_puts(text); // vypiš poipravený text
	
	lcd_gotoxy(display_position+3,0);
	sprintf(text,"%i",((cifra1/600)%6)); // poiprav text na displej
	lcd_puts(text); // vypiš poipravený text
	
	lcd_gotoxy(display_position+4,0);
	sprintf(text,"%i",((cifra1/60)%10)); // poiprav text na displej
	lcd_puts(text); // vypiš poipravený text
	
	lcd_gotoxy(display_position+6,0);
	sprintf(text,"%i",((cifra1/10)%6)); // poiprav text na displej
	lcd_puts(text); // vypiš poipravený text
	
	lcd_gotoxy(display_position+7,0);
	sprintf(text,"%i",cifra1%10); // poiprav text na displej
	lcd_puts(text); // vypiš poipravený text
	
	enableInterrupts();
}
void counting(){	//čítání cifer
	if(pause==0){		//neměnit když pause
		if(updown==0){	//odečítání
			cifra1--;
			if(cifra1<0){	//musíme rozdělit do 2 proměnných protože maximálně jde použít 16 bitů a to nedá 99 hodin :)))
				cifra2--;		
				cifra1=3599;
				if(cifra2<0){
					cifra2=0;
					cifra1=0;
					pause=1;
					print_button();
				}
			}
		}
		if(updown==1){	//přičítání
			cifra1++;
			if(cifra1>3599){
				cifra2++;
				cifra1=0;
				if(cifra2>99){
					cifra2=99;
					cifra1=3599;
					pause=1;
					print_button();
				}
			}
		}
		update_display=1;
	}
}
void remove_cursor(void){	//odstranění starého kurzoru pro posun
	lcd_gotoxy(x,1);
	lcd_puts(" ");
}
void print_cursor(void){	//vytvořit nový kurzor pro posun
	if((pause==0)||(pause==1)){
		lcd_gotoxy(x,1);
		lcd_puts("^");
	}
	else if(pause==2){
		lcd_gotoxy(x,1);
		//lcd_puts("T");
		lcd_putchar(4);
	}
}
void print_button(void){	//přepsat funkční tlačítka, která se mehou měnit
	if(pause==0){	//pause/unpsause tlačítko
		lcd_gotoxy(button1_position,0);	
		//lcd_puts("X");
		lcd_putchar(1);
	}
	else if(pause==1){
		lcd_gotoxy(button1_position,0);
		lcd_putchar(0);
	}
	if(updown==0){	//tlačítko počítat nahoru/dolů
		lcd_gotoxy(button3_position,0);
		//lcd_puts("v");
		lcd_putchar(3);
	}
	if(updown==1){
		lcd_gotoxy(button3_position,0);
		//lcd_puts("^");
		lcd_putchar(2);
	}
}
void add_time(void){
	if(x==display_position+7){
		cifra1=cifra1+1;
		if(cifra1%10==0){
			cifra1=cifra1-10;
		}
	}
	if(x==display_position+6){
		cifra1=cifra1+10;
		if((cifra1/10)%6==0){
			cifra1=cifra1-60;
		}
	}
	if(x==display_position+4){
		cifra1=cifra1+60;
		if((cifra1/60)%10==0){
			cifra1=cifra1-600;
		}
	}
	if(x==display_position+3){
		cifra1=cifra1+600;
		if((cifra1/600)%6==0){
			cifra1=cifra1-3600;
		}
	}
	if(x==display_position+1){
		cifra2=cifra2+1;
		if(cifra2%10==0){
			cifra2=cifra2-10;
		}
	}
	if(x==display_position+0){
		cifra2=cifra2+10;
		if((cifra2/10)%10==0){
			cifra2=cifra2-100;
		}
	}
	update_display=1;
}
void remove_time(void){
	if(x==display_position+7){
		cifra1=cifra1-1;
		if(cifra1<0){
			cifra1=cifra1+10;
		}
		else if(cifra1%10==9){
			cifra1=cifra1+10;
		}
	}
	
	if(x==display_position+6){
		cifra1=cifra1-10;
		if(cifra1<0){
			cifra1=cifra1+60;
		}
		else if((cifra1/10)%6==5){
			cifra1=cifra1+60;
		}
	}
	if(x==display_position+4){
		cifra1=cifra1-60;
		if(cifra1<0){
			cifra1=cifra1+600;
		}
		else if((cifra1/60)%10==9){
			cifra1=cifra1+600;
		}
	}
	if(x==display_position+3){
		cifra1=cifra1-600;
		if(cifra1<0){
			cifra1=cifra1+3600;
		}
		else if((cifra1/600)%6==5){
			cifra1=cifra1+3600;
		}
	}
	if(x==display_position+1){
		cifra2=cifra2-1;
		if(cifra2<0){
			cifra2=cifra2+10;
		}
		else if(cifra2%10==9){
			cifra2=cifra2+10;
		}
	}
	if(x==display_position+0){
		cifra2=cifra2-10;
		if(cifra2<0){
			cifra2=cifra2+100;
		}
		else if((cifra2/10)%10==9){
			cifra2=cifra2+100;
		}
	}
	update_display=1;
}
#ifdef USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param file: pointer to the source file name
  * @param line: assert_param error line source number
  * @retval : None
  */
void assert_failed(u8* file, u32 line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/