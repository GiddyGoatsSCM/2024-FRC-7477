// *****************************************************************************
//  File Name    : brampicphoto.c
//  Version      : 1.0
//  Description  : BRAM - Beginner's Robot Autonomous Mobile
//                 PIC Photovore I
//  Author       : RWB test Cain
//  Target       : PICJazz 16F690 Learning Board
//  Compiler     : HI-TECH C PRO for the PIC10/12/16 MCU family V9.60PL5 (Lite)
//  IDE          : Microchip MPLAB IDE v8.30
//  Programmer   : PICKit 2
//  Last Updated : 05 May 2009
// *****************************************************************************
// shit
#include <htc.h>
#include <pic.h>
#include <stdlib.h>
#include <stdio.h>
/*   PIC Configuration Bit:
**   INTIO     - Using Internal RC No Clock
**   WDTDIS    - Wacthdog Timer Disable
**   PWRTEN    - Power Up Timer Enable
**   MCLREN    - Master Clear Enable
**   UNPROTECT - Code Un-Protect
**   UNPROTECT - Data EEPROM Read Un-Protect
**   BORDIS    - Borwn Out Detect Disable
**   IESODIS   - Internal External Switch Over Mode Disable
**   FCMDIS    - Monitor Clock Fail Safe Disable
*/
//__CONFIG(INTIO & WDTDIS & PWRTEN & MCLREN & UNPROTECT \
//  & UNPROTECT & BORDIS & IESODIS & FCMDIS);
__CONFIG(FOSC_INTRCCLK & WDTE_OFF & MCLRE_OFF & CP_OFF);
// Using Internal Clock of 8 MHz
#define FOSC 8000000L
// Define Light Parameter
#define LIGHT_THRESHOLD  30
#define THRESHOLD_VALUE  50
#define MAX_THRESHOLD    180
// Define BRAM Steering
#define MOVE_FORWARD  0
#define TURN_LEFT     1
#define TURN_RIGHT    2
#define ROTATE_LEFT   3
#define ROTATE_RIGHT  4
#define MOVE_BACKWARD 5
#define FULL_STOP     6
// BRAM Debugging Mode, 0 - Debug Off, 1 - Debug On
#define BRAM_DEBUG    1
#if BRAM_DEBUG
  #define BAUD_RATE 9600
#endif
// Delay Function
#define	_delay_us(x) { unsigned char us; \
			         us = (x)/(12000000/FOSC)|1; \
			         while(--us != 0) continue; }
void _delay_ms(unsigned int ms)
{
  unsigned char i;
  do {
    i = 4;
    do {
      _delay_us(164);
    } while(--i);
  } while(--ms);
}
// BRAM UART Debugging Function
#if BRAM_DEBUG
void ansi_cl(void)
{
  // ANSI clear screen: cl=\E[H\E[J
  putchar(27);
  putchar('[');
  putchar('H');
  putchar(27);
  putchar('[');
  putchar('J');
}
void ansi_me(void)
{
  // ANSI turn off all attribute: me=\E[0m
  putchar(27);
  putchar('[');
  putchar('0');
  putchar('m');
}
void uart_init(void)
{
  TRISB5 = 1;          // Set Port B5 and B7 for UART Tx and Rx
  TRISB7 = 0;
  // Baud Rate formula for SYNC=0 (Async), BRG16=0 (8-bit), BRGH=0 (low speed)
  // 0.16% Error for 8 MHz Oscilator Clock. Actual Rate will be 9615.
  // BAUD_RATE = FOSC / (64 x (SPBRG + 1)) 

  SPBRG = (int)(FOSC/(64UL * BAUD_RATE) - 1);
  TXSTA = 0b00100000;  // Async, 8 bit and Enable Transmit (TXEN=1)
  RCSTA = 0b10010000;  // Serial Port Enable, Async,8-bit and Enable Receipt (CREN=1)
  BAUDCTL=0b00000;
}
void putch(unsigned char data)
{
  // Send Data when TXIF bit is ready
  while(!TXIF) continue;
  TXREG = data;
}
unsigned char getch(void) {
  // Get Data when RCIF bit is ready
  while(!RCIF) continue;
  return RCREG;
}
#endif
// BRAM Speed and Steering Functions
void BRAM_speed(unsigned char sp)
{
  // Adjust the PWM CCPR1L register
  CCPR1L=sp;
}
void BRAM_steer(unsigned char steer)
{
  switch(steer) {
    case MOVE_FORWARD:
      RC4=0; RC5=1;  // Right Motor On Forward
      RC6=1; RC7=0;  // Left Motor On Forward
      break;
    case TURN_LEFT:
      RC4=0; RC5=1;  // Right Motor On Forward
      RC6=0; RC7=0;  // Left Motor Off
      break;
    case TURN_RIGHT:
      RC4=0; RC5=0;  // Right Motor Off
      RC6=1; RC7=0;  // Left Motor On Forward
      break;
    case ROTATE_LEFT:
      RC4=0; RC5=1;  // Right Motor On Forward
      RC6=0; RC7=1;  // Left Motor On Reverse
      break;
    case ROTATE_RIGHT:
      RC4=1; RC5=0;  // Right Motor On Reverse
      RC6=1; RC7=0;  // Left Motor On Forward
      break;
    case MOVE_BACKWARD:
      RC4=1; RC5=0;  // Right Motor On Reverse
      RC6=0; RC7=1;  // Left Motor On Reverse
      break;
    case FULL_STOP:
      RC4=0; RC5=0;  // Right Motor Off
      RC6=0; RC7=0;  // Left Motor Off
      break;
  }
}
// BRAM Behaviour Functions
int random_number(void)
{
   unsigned char num;   

   num=(unsigned char) rand();
   return ((num % 300) + 20);
}
void BRAM_AvoidMode(unsigned char bump_sensor)
{
  static unsigned char turn_status = 0;
  turn_status ^= 0x01;
  // Process the Whisker Sensor
  if (bump_sensor > 50) {
    RA5=1;        // Turn On BRAM Head Light
#if BRAM_DEBUG
    printf("Whisker Value: %d\n\r",bump_sensor);
#endif
    // Initial Random Seed Number
    srand(bump_sensor);

    BRAM_steer(MOVE_BACKWARD);
    _delay_ms(150);
    if (bump_sensor <= 120) {
      // Left Bump Switch Range: 0 to 120
      BRAM_steer(ROTATE_RIGHT);
      _delay_ms(random_number());
    } else if (bump_sensor > 120 && bump_sensor <= 130) {
      // Right Bump Switch Range: 120 to 130
      BRAM_steer(ROTATE_LEFT);
      _delay_ms(random_number());
    } else {
      // Left + Right Bump Switch: > 130
      if (turn_status)
        BRAM_steer(ROTATE_LEFT);
      else
        BRAM_steer(ROTATE_RIGHT);
      _delay_ms(random_number());
    }
  }
}
void BRAM_LightFollow(unsigned char ldr_left, unsigned char ldr_right)
{
  int ldr_diff;
#if BRAM_DEBUG
  printf("Left LDR: %d; Right LDR: %d\n\r",ldr_left,ldr_right);
#endif
  // Get the different
  ldr_diff=ldr_left - ldr_right;
  if ((ldr_diff >= -THRESHOLD_VALUE) && (ldr_diff <= THRESHOLD_VALUE)) {
    if ((ldr_left > MAX_THRESHOLD) || (ldr_right > MAX_THRESHOLD)) {
      // Avoid Light
      BRAM_steer(FULL_STOP);
      _delay_ms(200);
      BRAM_steer(MOVE_BACKWARD);
      _delay_ms(200);
      BRAM_steer(ROTATE_LEFT);
      _delay_ms(200);
    } else {
      BRAM_steer(MOVE_FORWARD);       // Just go straight
    }
  } else {
    if (ldr_diff < 0)
      BRAM_steer(TURN_LEFT);
    else
      BRAM_steer(TURN_RIGHT);
  }
}
void main(void)
{
  unsigned char ldr_left;
  unsigned char ldr_right;
  unsigned char speed;
  unsigned char bump_sensor;
  unsigned char halt_status;
  OSCCON=0x70;         // Select 8 MHz internal clock
  TRISA = 0x03;        // Input for RA0 and RA1
  TRISC = 0x07;        // Set RC0,RC1 and RC2 as input others as Output
  ANSEL = 0x71;        // Set PORT AN0, AN4, AN5 and AN6 as analog input
  ANSELH = 0x00;       // Set PORT AN8 to AN11 as Digital I/O
  PORTC = 0x00;        // Turn Off all PORTC
  /* Init PWM for Single Output */
  CCP1CON=0b00001100;  // Single PWM mode; P1A, P1C active-high; P1B, P1D active-high
  CCPR1L=0;            // Start with zero Duty Cycle
  PSTRCON=0b00000100;  // Enable PIC Pulse Steering PWM on RC3 Port
  T2CON=0b00000101;    // Postscale: 1:1, Timer2=On, Prescale = 1:4
  PR2=0x65;            // Frequency: 4.90 kHz
  TMR2=0;              // Start with zero Counter 

  /* Init ADC */
  ADCON1=0b00110000;   // Select the FRC for 8 MHz
  halt_status=0;       // Initial: 0-Stop, 1-Run
  speed=0;             // Initial Speed
#if BRAM_DEBUG
  // Initial PIC16F690 UART
  uart_init();
  // Set the Attribute and Clear Screen
  ansi_me();
  ansi_cl();
  printf("Welcome to Cain Domain Debugging Mode\n\r\n\r");
#endif
  // Blink BRAM's Head Light Twice for Ready
  RA5=1; _delay_ms(100);
  RA5=0; _delay_ms(100);
  RA5=1; _delay_ms(100);
  RA5=0; _delay_ms(100);  

  for(;;) {
    if (RA1 == 0) {           // BRAM Status Switch
      _delay_ms(1);
      if (RA1 == 0) {         // Read again for Simple Debounce
        halt_status ^= 0x01;  // Halt Status
      }
    }
    /* Start Read All the ADC input here */
    // The PWM Motor Speed Control
    ADCON0=0b00000001;        // Select Left justify result. ADC port channel AN0
    GO_DONE=1;	              // Initiate conversion on the channel 0
    while(GO_DONE) continue;   // Wait for conversion done
    speed=ADRESH;
    // Bumped Sensor
    ADCON0=0b00011001;       // Select left justify result. ADC port channel AN6
    GO_DONE=1;	             // Initiate conversion on the channel 6
    while(GO_DONE) continue;  // Wait for ldr_left conversion done
    bump_sensor=ADRESH;      // Read 8 bits MSB, Ignore 2 bits LSB in ADRESL

    // The LDR Sensor
    ADCON0=0b00010001;       // Select left justify result. ADC port channel AN4
    GO_DONE=1;	             // Initiate conversion on the channel 4
    while(GO_DONE) continue;  // Wait for ldr_left conversion done
    ldr_left=ADRESH;         // Read 8 bits MSB, Ignore 2 bits LSB in ADRESL
    ADCON0=0b00010101;       // Select left justify result. ADC port channel AN5
    GO_DONE=1;	             // Initiate conversion on the channel 5
    while(GO_DONE) continue;  // Wait for ldr_right conversion done
    ldr_right=ADRESH;        // Read 8 bits MSB, Ignore 2 bits LSB in ADRESL           

    // BRAM Steering Speed
    BRAM_speed(speed * halt_status);
    RA5=0;                   // Turn Off BRAM Head Light
    // Whisker Sensor
    BRAM_AvoidMode(bump_sensor);    

    // Light Follower
    BRAM_LightFollow(ldr_left,ldr_right);
    _delay_ms(10);
  }
}
/* EOF: brampicphoto.c */
