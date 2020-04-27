//
//  TWI_Slave.c
//  TWI_Slave
//
//  Created by Sysadmin on 19.04.2020.
//  Copyright __MyCompanyName__ 2020. All rights reserved.
//



#include <avr/io.h>
#include <avr/delay.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <avr/eeprom.h>
#include <inttypes.h>
#include <avr/wdt.h>

//#include "twislave.c"
#include "lcd.c"

#include "adc.c"
#include "defines.h"
#include "irmp.c"

//***********************************

//***********************************

extern IRMP_DATA   irmp_data;

#define toggleA PORTC ^= (1<<PC4)
#define toggleB PORTC ^= (1<<PC5)

#define US (1000000 / F_INTERRUPTS)

uint8_t EEMEM WDT_ErrCount0;	// Akkumulierte WDT Restart Events
uint8_t EEMEM WDT_ErrCount1;	// WDT Restart Events nach wdt-reset

void eep_write_wochentag(uint8_t *ablauf[24], uint8_t *tag);


static volatile uint8_t SlaveStatus=0x00; //status


void delay_ms(unsigned int ms);
uint16_t EEMEM Brennerlaufzeit;	// Akkumulierte Laufzeit

void delay_ms(unsigned int ms);

uint8_t buerostatus=0x00;

volatile uint16_t Servotakt=20;					//	Abstand der Impulspakete
volatile uint16_t Servopause=0x00;				//	Zaehler fuer Pause
volatile uint16_t Servoimpuls=0x00;				//	Zaehler fuer Impuls
volatile uint8_t Servoimpulsdauer=20;			//	Dauer des Servoimpulses Definitiv
volatile uint8_t ServoimpulsdauerPuffer=22;		//	Puffer fuer Servoimpulsdauer
volatile uint8_t ServoimpulsdauerSpeicher=0;	//	Speicher  fuer Servoimpulsdauer
volatile uint8_t Potwert=45;
volatile uint8_t TWI_Pause=1;
volatile uint8_t ServoimpulsOK=0;				//	Zaehler fuer richtige Impulsdauer
uint8_t ServoimpulsNullpunkt=23;
uint8_t ServoimpulsSchrittweite=10;
uint8_t Servoposition[]={23,33,42,50,60};
volatile uint16_t ADCImpuls=0;

volatile uint8_t twicount=0;
volatile uint8_t twi=0;
uint8_t EEMEM WDT_ErrCount0;	// Akkumulierte WDT Restart Events
uint8_t EEMEM WDT_ErrCount1;	// WDT Restart Events nach wdt-reset

#pragma mark IRMP
// IRMP
//void timer1_init (void);
volatile uint16_t audio_remote_command = 0;

volatile uint8_t timer1count=0;

extern volatile uint8_t protokoll;
extern volatile uint8_t adresse;
extern volatile uint8_t code;
extern volatile uint16_t irmpcontrolD;

volatile uint8_t inputstatus = 0; // Eingaenge am ADC abfragen
volatile uint8_t lastinputstatus = 0; // letzter Status
volatile uint8_t kanalstatus = 0; // aktiver Kanal


volatile uint8_t outputstatus = 0; // status fuer Ausgang

volatile uint8_t loopstatus = 0; // Status fuer loop

volatile uint8_t exorcounter = 0; // Status fuer loop

volatile uint8_t aktuellerkanal = 0;



volatile uint8_t aktiverkanal = 0;
volatile uint8_t neuerkanal = 0;


volatile uint8_t remotechange = 0; // kanalwahl mit remote

uint16_t inputlevel[4] = {0};

// defines loopstatus
#define SEKUNDE         0
#define AMP_ON          1

uint16_t kanaldelay[4] = {0};
uint16_t kanaldelayA = KANALDELAY;
uint16_t kanaldelayB = KANALDELAY;
uint16_t kanaldelayC= KANALDELAY;
uint16_t kanaldelayD = KANALDELAY;

uint8_t kanalarray[4] = {0xFF}; // Liste fuer  Kanaele mit input

uint16_t outputdelay = OUTPUTDELAY;

uint16_t timer1counter = 0;
uint16_t sekundencounter = 0;
/* evaluate an IR remote signal */
void audio_remote(uint8_t command)
{
   /*
   if (   irmp_data.protocol != IRMP_NEC_PROTOCOL
       || irmp_data.address  != MY_REMOTE) {
      return;
   }
    */
   //uint16_t cmd = irmp_data.command;
   //audio_remote_command = cmd;
#pragma mark REMOTE
   remotechange = 0;
   switch (command)
   {
       /*
        #define APPLE_REW    0x08
        #define APPLE_FWD    0x07
        #define APPLE_PLUS   0x0B
        #define APPLE_MINUS  0x0D
        #define APPLE_PLAY   0x04
        #define APPLE_MENU   0x02
        */
      case APPLE_REW: /*  */
      {
         lcd_gotoxy(10,0);
         lcd_puts("rew  ");
         lcd_puthex(inputstatus);
         lcd_putc(' ');
         lcd_puthex(aktuellerkanal);
         if (aktuellerkanal && (inputstatus & (1<<(aktuellerkanal-1))))
         {
            aktuellerkanal++;
            remotechange |= (1<<aktuellerkanal);
         }
         break;
      }
         
      case APPLE_FWD:
      {
         lcd_gotoxy(10,0);
         lcd_puts("fwd  ");
         lcd_puthex(inputstatus);
         break;
      }
         
      case APPLE_PLUS:
      {
         lcd_gotoxy(10,0);
         lcd_puts("plus ");

         break;
      }
         
      case APPLE_MINUS:
      {
         lcd_gotoxy(10,0);
         lcd_puts("minus");

         break;
      }
         
      case APPLE_PLAY:
      {
         lcd_gotoxy(10,0);
         lcd_puts("play ");
         PORTD &= ~((1 << 3) | (1 << 4)| (1 << 5)| (1 << 6)); 
         break;
      }
         
      case APPLE_MENU:
      {
         lcd_gotoxy(10,0);
         lcd_puts("menu: ");
         loopstatus ^= (1<<AMP_ON);
         if (loopstatus & (1<<AMP_ON))
         {
            
            lcd_putc('1');
            outputdelay = OUTPUTDELAY;
         }
         else
         {
            lcd_putc('0');
            outputdelay = 0;
         //PORTB &= ~(1<<PB0);
         }
         break;
      }
        
         
      default:
      {
         break;
      }
   }
}



static void
timer1_init (void)
{
#if defined (__AVR_ATtiny45__) || defined (__AVR_ATtiny85__)                // ATtiny45 / ATtiny85:
   
#if F_CPU >= 16000000L
   OCR1C   =  (F_CPU / F_INTERRUPTS / 8) - 1;                              // compare value: 1/15000 of CPU frequency, presc = 8
   TCCR1   = (1 << CTC1) | (1 << CS12);                                    // switch CTC Mode on, set prescaler to 8
#else
   OCR1C   =  (F_CPU / F_INTERRUPTS / 4) - 1;                              // compare value: 1/15000 of CPU frequency, presc = 4
   TCCR1   = (1 << CTC1) | (1 << CS11) | (1 << CS10);                      // switch CTC Mode on, set prescaler to 4
#endif
   
#else                                                                       // ATmegaXX:
   OCR1A   =  (F_CPU / F_INTERRUPTS) - 1;                                  // compare value: 1/15000 of CPU frequency
   TCCR1B  = (1 << WGM12) | (1 << CS10);                                   // switch CTC Mode on, set prescaler to 1
#endif
   
#ifdef TIMSK1
   TIMSK1  = 1 << OCIE1A;                                                  // OCIE1A: Interrupt by timer compare
#else
   TIMSK   = 1 << OCIE1A;                                                  // OCIE1A: Interrupt by timer compare
#endif
}

ISR(TIMER1_COMPA_vect)                                                             // Timer1 output compare A interrupt service routine, called every 1/15000 sec
{
   //PORTC ^= (1<<PC5); 
   (void) irmp_ISR();
   //audio_remote();                                                        // call irmp ISR
   // call other timer interrupt routines...
   //inputstatus = (PINC & 0x0F);
   
   timer1counter++;
   if (timer1counter >= F_INTERRUPTS)
   {
      timer1counter = 0;
      sekundencounter++;
      //toggleB;
      loopstatus |= (1<<SEKUNDE); // sekundentasks in loop aktivieren
      //inputstatus = (PINC & 0x0F); // Status des Eingangsports aufnehmen
   }
 
}

void slaveinit(void)
{
	LOOPLEDDDR |= (1<<LOOPLED);		//Pin z von PORT D als Ausgang fuer loop-LED
//	DDRB &= ~(1<<PB1);	//Bit 1 von PORT B als Eingang fŸr IR
//	PORTB |= (1<<PB1);	//Pull-up

   DDRC |= (1<<PC4);   //Ausgang fuer control A
   PORTC |= (1<<PC4);   //Pull-up

   DDRC |= (1<<PC5);   //Ausgang fuer control B
   PORTC |= (1<<PC5);   //Pull-up

   DDRB |= (1<<PB0);   //Ausgang fuer Verstaerker
   PORTB &= ~(1<<PB0);   //LO

   DDRC &= ~(1<<AUDIO_A);  // Eingang von Audioquellen
 //  PORTC &= ~(1<<AUDIO_A); // LO
   DDRC &= ~(1<<AUDIO_B);
 //  PORTC &= ~(1<<AUDIO_B); // LO
   DDRC &= ~(1<<AUDIO_C);
 //  PORTC &= ~(1<<AUDIO_C); // LO
   DDRC &= ~(1<<AUDIO_D);
 //  PORTC &= ~(1<<AUDIO_D); // LO
   
   DDRD |= (1<<REL_A);   //Ausgang fuer Relais
   PORTD &= ~(1<<REL_A);   //LO
   DDRD |= (1<<REL_B);   //Ausgang fuer Relais
   PORTD &= ~(1<<REL_B);   //LO
   DDRD |= (1<<REL_C);   //Ausgang fuer Relais
   PORTD &= ~(1<<REL_C);   //LO
   DDRD |= (1<<REL_D);   //Ausgang fuer Relais
   PORTD &= ~(1<<REL_D);   //LO

	//LCD
	LCD_DDR |= (1<<LCD_RSDS_PIN);	//Pin 4 von PORT B als Ausgang fuer LCD
 	LCD_DDR |= (1<<LCD_ENABLE_PIN);	//Pin 5 von PORT B als Ausgang fuer LCD
	LCD_DDR |= (1<<LCD_CLOCK_PIN);	//Pin 6 von PORT B als Ausgang fuer LCD

}



void delay_ms(unsigned int ms)/* delay for a minimum of <ms> */
{
	// we use a calibrated macro. This is more
	// accurate and not so much compiler dependent
	// as self made code.
	while(ms){
		_delay_ms(0.96);
		ms--;
	}
}



void main (void) 
{
   /*
    in Start-loop in while
    init_twi_slave (SLAVE_ADRESSE);
    sei();
    */
   // IRMP
   IRMP_DATA   irmp_data;
   
   irmp_init();                                                            // initialize IRMP
   timer1_init();                                                          // initialize timer1
   initADC();
   
   wdt_disable();
   MCUSR &= ~(1<<WDRF);
   wdt_reset();
   WDTCR |= (1<<WDCE) | (1<<WDE);
   WDTCR = 0x00;
   
   slaveinit();
   //PORT2 |=(1<<PC4);
   //PORTC |=(1<<PC5);
   
   //uint16_t ADC_Wert= readKanal(0);
   
   /* initialize the LCD */
   lcd_initialize(LCD_FUNCTION_8x2, LCD_CMD_ENTRY_INC, LCD_CMD_ON);
   
   lcd_puts("Guten Tag\0");
   delay_ms(1000);
   lcd_cls();
   lcd_puts("AVR_Audio");
   
   
   uint8_t Tastenwert=0;
   uint8_t TastaturCount=0;
   uint8_t Servowert=0;
   uint8_t Servorichtung=1;
   
   uint16_t TastenStatus=0;
   uint16_t Tastencount=0;
   uint16_t Tastenprellen=0x01F;
   uint8_t Schalterposition=0;
   //timer0();
   
   //initADC(TASTATURPIN);
   //wdt_enable(WDTO_2S);
   
   uint16_t loopcount0=0;
   uint16_t startdelay0=0x01FF;
   //uint16_t startdelay1=0;
   
   uint16_t twi_LO_count0=0;
   uint16_t twi_LO_count1=0;
   
   
   //uint8_t twierrcount=0;
   LOOPLEDPORT |=(1<<LOOPLED);
   
   delay_ms(800);
   //eeprom_write_byte(&WDT_ErrCount0,0);
   uint8_t eepromWDT_Count0=eeprom_read_byte(&WDT_ErrCount0);
   //	uint8_t eepromWDT_Count1=eeprom_read_byte(&WDT_ErrCount1);
   uint16_t twi_HI_count0=0;
   
   if (eepromWDT_Count0==0xFF)
   {
      eepromWDT_Count0=0;
      
   }
   lcd_clr_line(0);
   sei ();                                                                 // enable interrupts
   
    while (1)
   {
      //Blinkanzeige
      loopcount0++;
      if (loopcount0==0xFFFF)
      {
         
         loopcount0=0;
         LOOPLEDPORT ^=(1<<LOOPLED);
         //delay_ms(10);
         TastaturCount++;
         //lcd_gotoxy(13,1);
         //lcd_putint(TastaturCount);
         lcd_gotoxy(19,0);
         lcd_putint1(sekundencounter);
         
         /*
         lcd_gotoxy(0,3);
         lcd_puthex(protokoll);
         protokoll=0;
         lcd_putc(' ');
         lcd_puthex(adresse);
         adresse=0;
         lcd_putc(' ');
         lcd_puthex(code);
         code=0;
         lcd_putc(' ');
         lcd_putint12(irmpcontrolD);
         irmpcontrolD=0;
         lcd_putc(' ');
         */
         
         /*
         lcd_puthex(kanaldelay[0]);
         lcd_putc(' ');
         lcd_puthex(kanaldelay[1]);
         lcd_putc(' ');
         lcd_puthex(kanaldelay[2]);
         lcd_putc(' ');
         lcd_puthex(kanaldelay[3]);
*/

         /*
         if ((inputstatus & 0x0F) > 0)
         {
            outputdelay = OUTPUTDELAY;
            //PORTB |= (1<<PB0);
         }
         else
         {
            if (outputdelay)
            {
               outputdelay--;
            }
            
            //PORTB &= ~(1<<PB0);
         }
         
         if (outputdelay)
         {
            PORTB |= (1<<PB0); // ON
         }
         else
         {
            PORTB &= ~(1<<PB0); // OFF
         }
         */
      }
#pragma mark SEKUNDE
      if (loopstatus & (1<<SEKUNDE)) // sekundentask abarbeiten
      {
         //cli();
         //lcd_clr_line(0);
         loopstatus &= ~(1<<SEKUNDE);
         
         // Inputlevel messen
         lcd_gotoxy(0,3);
         inputstatus=0;
         for (int kanal=0;kanal < 4;kanal++)
         {
            inputlevel[kanal] = readKanal(kanal);
            lcd_putint12(inputlevel[kanal]);
            lcd_putc(' ');
            if (inputlevel[kanal] > INPUTLEVEL) // kanal aktiv
            {
               inputstatus |= (1<<kanal);
            }
            //lcd_putint12(inputlevel[kanal]);
            //lcd_putc(' ');
         }
         
        // lcd_puthex(inputstatus);
         
         //inputstatus = (PINC & 0x0F); // Status des Eingangsports aufnehmen
       
         lcd_gotoxy(0,2);
         lcd_puthex(inputstatus);
         lcd_putc(' ');
         lcd_putint(outputdelay);
         lcd_putc(' ');
         lcd_puthex(kanaldelay[0]);
         lcd_putc(' ');
         lcd_puthex(kanaldelay[1]);
         lcd_putc(' ');
         lcd_puthex(kanaldelay[2]);
         lcd_putc(' ');
         lcd_puthex(kanaldelay[3]);

         
         
         /*
         aktiverkanal=0xFF;
         // aktiven Kanal suchen
         for (int kanal=0;kanal < 4;kanal++)
         {
            if (inputstatus & (1<<kanal))
            {
              // if (kanalarray[kanal] == 0) // neuer Kanal
               {
                  aktiverkanal = kanal;
               }
               kanalarray[kanal] = 1; 
            }
            else
            {
               kanalarray[kanal] = 0;
            }
         }
          
         lcd_gotoxy(0,1);
         lcd_putc('A');
         lcd_puthex(aktiverkanal);
         */
         
         lcd_gotoxy(0,0);
         lcd_puts("OD");
         uint8_t d = PIND;
         lcd_puthex((PIND & 0x78)>>3); // bit 3-6
         lcd_putc(' ');
         lcd_puts("IC");
         lcd_puthex(inputstatus);
         
         //Aenderung abfragen
         uint8_t change = inputstatus ^ lastinputstatus; 
         
         /*
         lcd_gotoxy(8,2);
         lcd_puts("ch");
         lcd_puthex(change);
         uint8_t ex = change & inputstatus;
         lcd_putc(' ');
         lcd_puts("ex");
         lcd_puthex(ex);
         */ 
         if (change) // Aenderung: neuer Kanal
         {
            exorcounter++;
            lcd_gotoxy(16,3);
            lcd_puthex(exorcounter);
            
            lcd_gotoxy(0,1);
            lcd_putc('A');
            lcd_putint1(aktuellerkanal);
            
            uint8_t kanalnew = change & inputstatus; 
            // > 0: Kanal von change ist neu;   0: Kanal von change ist weg
            
            
            if (aktuellerkanal < 0xFF) // aktueller kanal vorhanden, ausschalten
            {
               lcd_gotoxy(8,1);
               lcd_puts("weg");
               lcd_putint(aktuellerkanal);
               uint8_t relais = aktuellerkanal+3; // position auf PORTD
               PORTD &= ~(1 << relais);
               //inputstatus &= ~(1<<aktuellerkanal);
               aktuellerkanal = 0xFF;
               lcd_putc(' ');
               lcd_putint(aktuellerkanal);
               
            }
             

            if (inputstatus == 0) // Kein Eingang aktiv
            {
               PORTD &= ~((1 << 3) | (1 << 4)| (1 << 5)| (1 << 6)); // alle relais off
               neuerkanal = 0xFF;
               inputstatus = 0;
               //break;
            }
            else
            {
               
               // neuen aktuellen Kanal suchen
               neuerkanal = 0xFF;
               
               if (kanalnew) // neuer Kanal dazugekommen
               {
                  for (int kanal=0;kanal < 4;kanal++)
                  {
                     if ((change) & (1<<kanal))
                     {
                        neuerkanal = kanal;
                     }
                  }
               }
               else  // Kanal ausgeschaltet
               {
                  // ersten aktiven Kanal suchen
                  for (int kanal=0;kanal < 4;kanal++)
                  {
                     if ((inputstatus) & (1<<kanal))
                     {
                        neuerkanal = kanal;
                     }
                  }
                  
               }
               
               
               
               
               if (neuerkanal < 0xFF)
               {
                  aktuellerkanal = neuerkanal;
                  uint8_t relais = neuerkanal+3; // position auf PORTD
                  outputdelay = OUTPUTDELAY;
                  PORTD |= (1<< (relais));
                  kanalstatus |= (1<<neuerkanal);
               }
               
               lcd_gotoxy(3,1);
               lcd_putc('N');
               lcd_putint1(neuerkanal);
               
               
               
               /*
                lcd_gotoxy(4,1);
                lcd_putc('L');
                lcd_puthex(lastinputstatus);
                
                lcd_putc(' ');
                lcd_putc('I');
                lcd_puthex(inputstatus);
                lcd_putc(' ');
                
                lcd_putc('^');
                lcd_puthex(inputstatus ^ lastinputstatus);
                kanalstatus = inputstatus ^ lastinputstatus; //Neuer aktiver Kanal
                lcd_putc(' ');
                
                lcd_putc('K');
                lcd_puthex(kanalstatus);
                */
               
               // Input
               for (int kanal=0;kanal < 4;kanal++)
               {
                  uint8_t relais = kanal+3; // position auf PORTD
                  if (kanalstatus & (1<<kanal)) // noch aktiv
                  {
                     kanaldelay[kanal] = KANALDELAY;
                     //inputstatus |= (1<<kanal);
                     //       PORTD |= (1<<relais);
                     
                  }
                  else 
                  {
                     // PORTD &= ~(1<<relais);
                     //         inputstatus &= ~(1<<kanal);
                     //         kanaldelay[kanal] = 0;
                     
                     if (kanaldelay[kanal]) // noch nicht auf null
                     {
                        kanaldelay[kanal] --;
                     }
                     if (kanaldelay[kanal] == 0)
                     {
                        PORTD &= ~(1<<relais);
                     }
                     
                  }
                  
               } // Input
            }
            lastinputstatus = inputstatus;
            lcd_gotoxy(10,0);
            //lcd_putc('*');
            lcd_putc('I');
            lcd_puthex(inputstatus);
            
            lcd_putc(' ');
            lcd_putc('K');
            lcd_puthex(kanalstatus);
            
  //          lcd_putc(' ');
  //          lcd_putc('D');
  //          uint8_t d = PIND;
  //          lcd_puthex((PIND & 0x78)>>3); // bit 3-6


         } // if change
         else
         {
            //lcd_gotoxy(15,2);
            //lcd_puts("--");
         }
         
         
         
         // Output steuern
         if (((inputstatus & 0x0F) > 0) && (loopstatus & (1<<AMP_ON)))// mindestens ein Eingang aktiv
         {
            loopstatus |= (1<<AMP_ON);
            outputdelay = OUTPUTDELAY; // Maximaldauer
            //PORTB |= (1<<PB0);
         }
         else // kein Signal, demnaechst ausschalten
         {
            if (outputdelay)
            {
               outputdelay--;
            }
            
            //PORTB &= ~(1<<PB0);
         }
        
         sei(); 
      } // sekunde
      
      if (outputdelay) // Output soll ON sein
      {
         loopstatus |= (1<<AMP_ON);
         PORTB |= (1<<PB0); // ON
         
      }
      else
      {
         loopstatus &= ~(1<<AMP_ON);
         PORTB &= ~(1<<PB0); // OFF
         PORTD &= ~((1 << 3) | (1 << 4)| (1 << 5)| (1 << 6)); // alle relais off
      }
     
      
      
      
 //     PORTB &= ~(1<<PB1); 
#pragma mark get DATA      
      // IRMP
      if (irmp_get_data (&irmp_data))
      {
         // got an IR message, do something
         _delay_ms(25);
         
    //     PORTB |= (1<<PB0);
         protokoll = irmp_data.protocol;
         adresse = irmp_data.address;
         code = irmp_data.command;
         _delay_ms(25);
 
         audio_remote(code);
 
      }
       
        
        
      
      
   }     
         
      
      
      
 
   // return 0;
}
