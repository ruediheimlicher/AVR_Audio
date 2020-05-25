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


static volatile uint8_t SlaveStatus=0x00; //status


void delay_ms(unsigned int ms);
uint16_t EEMEM Brennerlaufzeit;	// Akkumulierte Laufzeit

void delay_ms(unsigned int ms);

uint8_t buerostatus=0x00;

#pragma mark SERVO
// Servo


volatile uint16_t servotaktcounter=0;					//	ISR-counter fuer Servoimpuls-takt
volatile uint16_t servoimpulscounter=0x00;				//	Zaehler fuer Impulsdauer
volatile uint8_t servostatus=0;		//	Status fuer Ablauf
volatile uint8_t servoimpulshold=SERVOHOLD;   // counter fuer Hold-Zeit des Impuls

volatile uint8_t servoposition=SERVOSTART;

volatile uint8_t ServoimpulsdauerSpeicher=0;	//	Speicher  fuer Servoimpulsdauer
volatile uint8_t Potwert=45;
uint8_t servoimpulsNullpunkt=23;
uint8_t servoimpulsSchrittweite=10;
//uint8_t Servoposition[]={23,33,42,50,60};


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

volatile uint8_t isrcounter = 0;

volatile uint8_t outputstatus = 0; // status fuer Ausgang

volatile uint8_t loopstatus = 0; // Status fuer loop

volatile uint8_t exorcounter = 0; // Status fuer loop

volatile uint8_t aktuellerkanal = 0xFF;

volatile uint8_t lastkanal = 0xFF;

volatile uint8_t pauseposition = SERVOSTART;

volatile uint8_t aktiverkanal = 0;
volatile uint8_t neuerkanal = 0;


volatile uint8_t remotechange = 0; // kanalwahl mit remote

uint16_t inputlevel[4] = {0};

// defines loopstatus
#define SEKUNDE         0
#define AMP_ON          1
#define PAUSE           2

uint16_t kanaldelay[4] = {0};
uint16_t kanaldelayA = KANALDELAY;
uint16_t kanaldelayB = KANALDELAY;
uint16_t kanaldelayC = KANALDELAY;
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
         lcd_gotoxy(4,2);
         lcd_puts("rew");
         lcd_puthex(inputstatus);
         lcd_putc(' ');
         lcd_puthex(aktuellerkanal);
         lcd_putc(' ');
         uint8_t neuerkanal = aktuellerkanal;
         // aktiven Kanal vor aktuellerkanal suchen
         if (aktuellerkanal) // noch nicht bei 0
         {
            neuerkanal--;
            while ((neuerkanal) && (!(inputstatus & (1<<(neuerkanal))))) // next kanal ist nicht aktiv
            {
               neuerkanal--; // 
            }
            if ((neuerkanal == 0) && (!(inputstatus & (1<<(neuerkanal))))) // kanal 0 ist nicht aktiv
            {
               lcd_puts("nix0");
               break; // nix
            }
            lcd_puts("tp");
            lcd_puthex(neuerkanal);
            // aktuellen kanal ausschalten
            uint8_t relais = aktuellerkanal+3; // position auf PORTD
            PORTD &= ~(1 << relais);
            kanalstatus &= ~(1<<aktuellerkanal); // aktuellen Kanal entfernen
            // neuen Kanal einschalten
            relais = neuerkanal+3; // position auf PORTD
            outputdelay = OUTPUTDELAY; // outputelay aktualisieren
            PORTD |= (1<< (relais));
            kanalstatus |= (1<<neuerkanal); 
 
            aktuellerkanal = neuerkanal;
          
         }
       }break;
         
      case APPLE_FWD:
      {
         lcd_gotoxy(4,2);
         lcd_puts("fwd");
         lcd_puthex(inputstatus);
         lcd_putc(' ');
         lcd_puthex(aktuellerkanal);
         lcd_putc(' ');
         uint8_t neuerkanal = aktuellerkanal;
         // aktiven Kanal vor aktuellerkanal suchen
          if (aktuellerkanal < 3) 
          {
             neuerkanal++; // next kanal
             while ((neuerkanal < 3) && (!(inputstatus & (1<<(neuerkanal))))) // dieser kanal ist nicht aktiv
             {
                neuerkanal++;
             }
             
             if ((neuerkanal == 3)  && (!(inputstatus & (1<<(neuerkanal)))))// letzter kanal ist nicht aktiv
             {
                lcd_puts("nix ");
                
                break; // nix
             }
             
             lcd_puts("tp");
             lcd_puthex(neuerkanal);
             
          
             // aktuellen kanal ausschalten
             uint8_t relais = aktuellerkanal+3; // position auf PORTD
             PORTD &= ~(1 << relais);
             kanalstatus &= ~(1<<aktuellerkanal); // aktuellen Kanal entfernen
             // neuen Kanal einschalten
             relais = neuerkanal+3; // position auf PORTD
             outputdelay = OUTPUTDELAY; // outputelay aktualisieren
             PORTD |= (1<< (relais));
             kanalstatus |= (1<<neuerkanal); 
             
             aktuellerkanal = neuerkanal;
            
          }
        }break;
         
      case APPLE_PLUS:
      {
         lcd_gotoxy(10,0);
         lcd_puts("+  ");
         loopstatus &= ~(1<<PAUSE); // eventuelle Pause beenden
         if (servoposition < SERVOMAX)
         {
            servoposition++;
            pauseposition = servoposition;
            servoimpulshold = SERVOHOLD; 
         }
         break;
      }
         
      case APPLE_MINUS:
      {
         lcd_gotoxy(10,0);
         lcd_puts("-  ");
         loopstatus &= ~(1<<PAUSE); // eventuelle Pause beenden
         if (servoposition > SERVOMIN)
         {
            servoposition--;
            pauseposition = servoposition;
            servoimpulshold = SERVOHOLD; 
         }

         break;
      }
         
      case APPLE_PLAY:
      {
         lcd_gotoxy(14,1);
         uint8_t relais = aktuellerkanal+3; // position auf PORTD
        // if (kanalstatus & (1<<aktuellerkanal))// aktueller kanal vorhanden
         {
           // if (PORTD & (1<< relais)) // Kanal ist ON, pause ein
            if (loopstatus & (1<<PAUSE))// pause beenden
            {
               loopstatus &= ~(1<<PAUSE);
               servoposition = pauseposition;
               servoimpulshold = SERVOHOLD; 
               lcd_puts("play ");
            }
            else 
            {
               loopstatus |= (1<<PAUSE);
               lcd_puts("pause");
               pauseposition = servoposition;
               servoposition = SERVOMIN; // Volume down
               servoimpulshold = SERVOHOLD; 
            }
         }// if aktuellerkanal
            
          
         break;
      }
         
      case APPLE_MENU:
      {
         lcd_gotoxy(10,0);
         lcd_puts("menu");
         loopstatus ^= (1<<AMP_ON);
         if (loopstatus & (1<<AMP_ON))
         {
            
            lcd_putc('1');
            // ersten aktiven Kanal suchen
            for (int kanal=0;kanal < 4;kanal++)
            {
               if ((inputstatus) & (1<<kanal))
               {
                  neuerkanal = kanal;
                  kanaldelay[kanal] = KANALDELAY;
               }
               else
               {
                  kanaldelay[kanal] = 3;
               }
               
            }
            // neuen Kanal einschalten
            uint8_t relais = neuerkanal+3; // position auf PORTD
            outputdelay = OUTPUTDELAY; // outputelay aktualisieren
            PORTD |= (1<< (relais));
            kanalstatus |= (1<<neuerkanal); 
            //  kanaldelay[neuerkanal] = KANALDELAY;
            lcd_gotoxy(6,1);
            lcd_putc('N');
            lcd_putint1(neuerkanal);
            
            // aktuellen Kanal neu setzen
            aktuellerkanal = neuerkanal;

            
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
/*
void timer0 (void) 
{ 
   // Timer fuer Exp
   TCCR0 |= (1<<CS00);
   //TCCR0 |= (1<<CS01);                  // clock   /8
   //TCCR0 |= (1<<CS01)|(1<<CS02);         // clock   /64
   //TCCR0 |= (1<<CS02)| (1<<CS02);         // clock   /256
   //TCCR0 |= (1<<CS00)|(1<<CS02);         // clock /1024
   
   //TIFR |= (1<<TOV0);                     //Clear TOV0 Timer/Counter Overflow Flag. clear pending interrupts
   TIMSK |= (1<<TOIE0);                     //Overflow Interrupt aktivieren
   //TCNT0 = TIMER0_STARTWERT;               //RŸcksetzen des Timers
   
}

ISR (TIMER0_OVF_vect) 
{ 
//   PORTC ^= (1<<PC5); 
   
   
   if (servostatus & (1<<SERVOCONTROLBIT)) // Impuls on, takt zaehlen
   {
      servotaktcounter++;
      if (servotaktcounter > servoposition) // 30: 1ms 60: 2ms
      {
         servotaktcounter = 0;
         SERVOPORT &= ~(1<<SERVOPIN0); // Impuls OFF
         servostatus &= ~(1<<SERVOCONTROLBIT); // Impuls fertig, wird in ISR1 gesetzt
      }
   }
 
   
}
*/
static void
timer1_init (void)
{
   /*
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
 */  
   OCR1A   =  ((F_CPU /( F_INTERRUPTS )))/2 - 1;                                  // compare value: 1/15000 of CPU frequency
 //  OCR1A   = 534/2;
   TCCR1B  = (1 << WGM12) | (1 << CS10);                                   // switch CTC Mode on, set prescaler to 1
   TIMSK   = 1 << OCIE1A; 
}

ISR(TIMER1_COMPA_vect)                                                             // Timer1 output compare A interrupt service routine, called every 1/15000 sec
{
 //  PORTD ^= (1<<3);
 //  PORTB ^= (1<<7); 
   isrcounter++;
   if ((isrcounter % 2) == 0)
   {
   //PORTC ^= (1<<PC5); 
   irmp_ISR();
   }
 //  irmp_ISR();
   //audio_remote();                                                        // call irmp ISR
   // call other timer interrupt routines...
   //inputstatus = (PINC & 0x0F);
//   PORTC ^= (1<<PC5); 
   // Servotakt 50Hz
   servotaktcounter++;
   if ((servotaktcounter == 300) && servoimpulshold)
   {
      servotaktcounter = 0;
      servostatus |= (1<<SERVOCONTROLBIT); // flag fuer neuen Impuls setzen
      SERVOPORT |= (1<<SERVOPIN0); 
      servoimpulscounter = 0;
      servoimpulshold--;
        
   }
   
   
   if (servostatus & (1<<SERVOCONTROLBIT)) // Impuls on
   {
      servotaktcounter++;
      if (servotaktcounter > servoposition) // 30: 1ms 60: 2ms
      {
         servotaktcounter = 0;
         SERVOPORT &= ~(1<<SERVOPIN0); 
         servostatus &= ~(1<<SERVOCONTROLBIT);
      }
   }
   
   
   timer1counter++;
   if (timer1counter >= 2*F_INTERRUPTS)
   {
      //PORTB ^= (1<<PB6); 
      timer1counter = 0;
      sekundencounter++;
      //toggleB;
      loopstatus |= (1<<SEKUNDE); // sekundentasks in loop aktivieren
      //inputstatus = (PINC & 0x0F); // Status des Eingangsports aufnehmen
   }
   
 
}

#define TIMER2_PRESCALER      (1 << CS21) //| (1 << CS20)

/*
void timer2 (uint8_t wert) 
{ 
   //   TCCR2 |= (1<<CS02);            //8-Bit Timer, Timer clock = system clock/256
   
   //Takt fuer Servo
   //TCCR2 |= (1<<CS21);
   //TCCR2 |= (1<<CS20)|(1<<CS21);   //Takt /64   Intervall 64 us
   TCCR2 = 0;
   TCCR2 |= (1<<WGM21) | TIMER2_PRESCALER;      //   ClearTimerOnCompareMatch CTC
   
   //OC2 akt
   //   TCCR2 |= (1<<COM20);      //   OC2 Pin zuruecksetzen bei CTC
   
   
   //TIFR |= (1<<TOV2);             //Clear TOV2 Timer/Counter Overflow Flag. clear pending interrupts
   TIMSK |= (1<<OCIE2);         //CTC Interrupt aktivieren
   
   //TCNT2 = 0x00;               //Zaehler zuruecksetzen
   
   OCR2 = wert;               //Setzen des Compare Registers auf Servoimpulsdauer
} 

ISR(TIMER2_COMP_vect) // Schaltet Impuls an SERVOPIN0 aus
{
//   PORTC ^= (1<<PC5);
   
   if (servostatus & (1<<SERVOCONTROLBIT)) // Impuls on, takt zaehlen
   {
      servoimpulscounter++;
      if (servoimpulscounter > servoposition) // 30: 1ms 60: 2ms
      {
         servoimpulscounter = 0;
         SERVOPORT &= ~(1<<SERVOPIN0); // Impuls OFF
         servostatus &= ~(1<<SERVOCONTROLBIT); // Impuls fertig, wird in ISR1 gesetzt
      }
   }

}
*/
void slaveinit(void)
{
	LOOPLEDDDR |= (1<<LOOPLED);		//Pin z von PORT D als Ausgang fuer loop-LED
//	DDRB &= ~(1<<PB1);	//Bit 1 von PORT B als Eingang fŸr IR
//	PORTB |= (1<<PB1);	//Pull-up
   
   // Servo
   SERVODDR |= (1<<SERVOPIN0);
   SERVOPORT |= (1<<SERVOPIN0);// LO
   
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
   
  // timer0();
   
 //  timer2(50);
   
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
   uint16_t oc = ((F_CPU /( F_INTERRUPTS ))) - 1;
   lcd_putint12(oc);

   sei ();                                                                 // enable interrupts
   
    while (1)
   {
   //   PORTB ^= (1<<PB7); 
      //Blinkanzeige
      loopcount0++;
      if (loopcount0==0xFFFF)
      {
         //PORTB ^= (1<<PB6);
         loopcount0=0;
         LOOPLEDPORT ^=(1<<LOOPLED);
         //delay_ms(10);
         TastaturCount++;
         //lcd_gotoxy(13,1);
         //lcd_putint(TastaturCount);
         
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
   //      PORTB ^= (1<<PB7); 
         lcd_gotoxy(19,0);
         lcd_putint1(sekundencounter);

   
         lcd_gotoxy(10,1);
         lcd_putc('L');
         lcd_puthex(lastkanal);
         
         lcd_gotoxy(0,2);  
         lcd_putint(servoposition);
         //lcd_putc(' ');
         //lcd_putint12(code);



         loopstatus &= ~(1<<SEKUNDE);

         /* ******************************************* */
         // Inputlevel messen, inputstatus setzen
         /* ******************************************* */
         lcd_gotoxy(0,3);
         inputstatus=0;
         for (int kanal=0;kanal < 3;kanal++)
         {
            inputlevel[kanal] = readKanal(kanal);
            lcd_putint12(inputlevel[kanal]);
            lcd_putc(' ');
            if (inputlevel[kanal] > INPUTLEVEL) // kanal aktiv, level gross genug
            {
               inputstatus |= (1<<kanal);
            }
         }
         
         /* ******************************************* */
         // inputstatus anpassen: 
         /* ******************************************* */   
         
         if (lastkanal < 0xFF) // ein lastkanal festgelegt
         {
            if (inputlevel[lastkanal] >= INPUTLEVEL) // lastkanal: pegel ist noch zu gross
            {
               inputstatus &= ~(1<<lastkanal); // lastkanal entfernen
            }
            else // lastkanal: pegel nicht mehr relevant
           {
              lastkanal = 0xFF;
           }
         }

        
          //lcd_putc(' ');
         
         
         /* 
         lcd_gotoxy(0,2);  
         lcd_puthex(kanaldelay[0]);
         lcd_putc(' ');
         lcd_puthex(kanaldelay[1]);
         lcd_putc(' ');
         lcd_puthex(kanaldelay[2]);
         lcd_putc(' ');
         lcd_puthex(kanaldelay[3]);
          
         
         lcd_gotoxy(16,2);
         lcd_putint(outputdelay);
         */
         
           
         /* ******************************************* */
         // relaisstatus abfragen
         /* ******************************************* */
         lcd_gotoxy(0,0);
         lcd_puts("PD");
         uint8_t d = PIND;
         lcd_puthex((PIND & 0x78)>>3); // bit 3-6
         lcd_putc(' ');
         lcd_puts("IS");
         lcd_puthex(inputstatus);
         
         lcd_gotoxy(16,3);
         lcd_puts("S");
         lcd_putint(servoposition);
         
         /* ******************************************* */
         //Aenderung abfragen
         /* ******************************************* */
         uint8_t change = inputstatus ^ lastinputstatus; 
         
         
#pragma mark CHANGE         
         /* ******************************************* */
         /* ****************  CHANGE   **************** */
         /* ******************************************* */
         
         if (change) // Aenderung: neuer Kanal oder ein kanal weg
         {
            exorcounter++;
            lcd_gotoxy(17,3);
            lcd_putint(exorcounter);
            
 //           lcd_gotoxy(0,1);
 //           lcd_putc('A');
 //           lcd_putint1(aktuellerkanal);
            
            
            uint8_t kanalnew = change & inputstatus; 
            // > 0: Kanal von change ist neu;   0: Kanal von change ist weg
            
            /* ******************************************* */
            // aktuellen Kanal ausschalten, sofern da
            /* ******************************************* */
           
            if (aktuellerkanal < 0xFF) // aktueller kanal vorhanden, ausschalten
            {
               lcd_gotoxy(8,1);
               lcd_puts("weg");
               lcd_putint(aktuellerkanal);
               uint8_t relais = aktuellerkanal+3; // position auf PORTD
               PORTD &= ~(1 << relais);
               kanaldelay[aktuellerkanal] = 5;
               //inputstatus &= ~(1<<aktuellerkanal);
          //     lastkanal = aktuellerkanal; // nach ADC aus inputstatus entfernen
               
               kanalstatus &= ~(1<<aktuellerkanal); // aktuellen Kanal entfernen
               aktuellerkanal = 0xFF;
               lcd_putc(' ');
               lcd_putint(aktuellerkanal);
               
            }
             

            if (inputstatus == 0) // Kein Eingang aktiv
            {
               PORTD &= ~((1 << 3) | (1 << 4)| (1 << 5)| (1 << 6)); // alle relais off
               neuerkanal = 0xFF;
               
               //break;
            }
            else
            {
               
               // neuen aktuellen Kanal suchen
               neuerkanal = 0xFF;
               
               if (kanalnew) // neuer Kanal dazugekommen (kanalnew = change & inputstatus;)
               {
                  for (int kanal=0;kanal < 4;kanal++)
                  {
                     if ((change) & (1<<kanal))
                     {
                        neuerkanal = kanal;
                        kanaldelay[kanal] = KANALDELAY;
                     }
                     else
                     {
                        kanaldelay[kanal] = 3;
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
                        kanaldelay[kanal] = KANALDELAY;
                     }
                     else
                     {
                        kanaldelay[kanal] = 3;
                     }

                  }
                  
               }
               
               /* ******************************************* */
               // neuen Kanal einschalten, aktuellen Kanal neu setzen
               /* ******************************************* */
               
               if (neuerkanal < 0xFF)
               {
                  lastkanal = 0xFF;
                  
                  // neuen Kanal einschalten
                  uint8_t relais = neuerkanal+3; // position auf PORTD
                  outputdelay = OUTPUTDELAY; // outputelay aktualisieren
                  PORTD |= (1<< (relais));
                  kanalstatus |= (1<<neuerkanal); 
                //  kanaldelay[neuerkanal] = KANALDELAY;
                  lcd_gotoxy(6,1);
                  lcd_putc('N');
                  lcd_putint1(neuerkanal);
                  
                  // aktuellen Kanal neu setzen
                  aktuellerkanal = neuerkanal;
               
               }
               
               
              // lcd_gotoxy(8,1);
             //  lcd_puts("             ");
               
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
/*
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
    //                    PORTD &= ~(1<<relais);
                     }
                     
                  }
                  
               } // Input
    */           
               
               
            } // else
            
            // lastinputstatus aktualisieren
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
            
            if (remotechange) // remote hat den atuellen kanal veraendert
            {
               
            }
            //lcd_gotoxy(15,2);
            //lcd_puts("--");
         }
         lcd_gotoxy(3,1);
         lcd_putc('A');
         lcd_putint1(aktuellerkanal);

         
         
         // Output steuern
         if (((inputstatus & 0x0F) > 0) && (loopstatus & (1<<AMP_ON)))// mindestens ein Eingang aktiv
         {
            //loopstatus |= (1<<AMP_ON);
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

         
        
         sei(); 
      } // sekunde
      
      
      
      
      
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
