//
//  defines.h
//  AVR_Audio
//
//  Created by Ruedi Heimlicher on 18.04.2020.
//

#ifndef defines_h
#define defines_h

#define AUDIO_A   0
#define AUDIO_B   1
#define AUDIO_C   2
#define AUDIO_D   3

#define REL_A     3
#define REL_B     4
#define REL_C     5
#define REL_D     6


#define KANALDELAY   20
#define OUTPUTDELAY  30

#define INPUTLEVEL 130 // minimaler inputlevel, verhindert ausschalten bei Pausen


/* remote control address and command codes */

#define APPLE_REW    0x08
#define APPLE_FWD    0x07
#define APPLE_PLUS   0x0B
#define APPLE_MINUS  0x0D
#define APPLE_PLAY   0x04
#define APPLE_MENU   0x02


#define MY_REMOTE   0x916e
#define KEY_OPERATE 0x14
#define KEY_1       0x05
#define KEY_2       0x06
#define KEY_3       0x07
#define KEY_4       0x0c
#define KEY_5       0x0d
#define KEY_6       0x0e
#define KEY_7       0x0f
#define KEY_8       0x1c
#define KEY_9       0x1d
#define KEY_0       0x04
#define KEY_MINUS   0x18
#define KEY_PLUS    0x19
#define KEY_REV     0x02
#define KEY_FWD     0x03
#define KEY_STOP    0x01
#define KEY_PLAY    0x08
#define KEY_REC     0x09
#define KEY_PAUSE   0x0b







// alte defines

#define TWI_PORT      PORTC
#define TWI_PIN      PINC
#define TWI_DDR      DDRC

#define SDAPIN      4 // PORT C
#define SCLPIN      5



#define STARTDELAYBIT   0
#define HICOUNTBIT      1

#define WDTBIT         7


#define BUEROPORT   PORTD      // Ausgang fuer BUERO
#define UHRPIN 0

#define SERVOPORT   PORTC      // Ausgang fuer Servo
#define SERVODDR  DDRC
#define SERVOPIN0 4            // Impuls für Servo
//#define SERVOPIN1 7            // Enable fuer Servo, Active H
#define SERVOCONTROLBIT 0 // statusbit

#define SERVOMIN 10
#define SERVOMAX 120 
#define SERVOMITTE (SERVOMIN + SERVOMAX)/2
#define SERVOSTART 20
// Definitionen Slave Buero
#define UHREIN 0
#define UHRAUS 1


#define LOOPLEDPORT      PORTD
#define LOOPLEDDDR      DDRD

// Define fuer Slave:
#define LOOPLED         7




#define TASTATURPIN      3
#define POTPIN         0
#define BUZZERPIN      0

#define INNEN         0   //   Byte fuer INNENtemperatur
#define AUSSEN         1   //   Byte fuer Aussentemperatur
#define STATUS         3   //   Byte fuer Status
#define BRENNERPIN      2   //   PIN 2 von PORT B als Eingang fuer Brennerstatus



#define TIMER0_STARTWERT   0x40


#endif /* defines_h */
