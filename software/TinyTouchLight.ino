// ===================================================================================
// Project:   TinyTouchLight - Dimmable USB Night Light with Capacitive Touch Control
// Version:   v1.0
// Year:      2021
// Author:    Stefan Wagner
// Github:    https://github.com/wagiminator
// EasyEDA:   https://easyeda.com/wagiminator
// License:   http://creativecommons.org/licenses/by-sa/3.0/
// ===================================================================================
//
// Description:
// ------------
// TinyTouchLight is a dimmable USB night light with capacitive touch control based on
// the ATtiny13A. It plugs into any USB charger or power bank.
// The implementation of the capacitive touch control using charge sharing between the
// touch pad and the internal ADC sample and hold (S/H) capacitor similar to Atmel's
// QTouchADC is based on TinyTouchLib by Tim.
//
// References:
// -----------
// https://github.com/cpldcpu/TinyTouchLib
// http://ww1.microchip.com/downloads/en/Appnotes/doc8497.pdf
//
// Wiring:
// -------
//                              +-\/-+
//           --- RST ADC0 PB5  1|°   |8  Vcc
// TOUCH SHC ------- ADC3 PB3  2|    |7  PB2 ADC1 -------- 
// TOUCH PAD ------- ADC2 PB4  3|    |6  PB1 AIN1 OC0B --- 
//                        GND  4|    |5  PB0 AIN0 OC0A --- LED PWM
//                              +----+
//
// Compilation Settings:
// ---------------------
// Controller:  ATtiny13A
// Core:        MicroCore (https://github.com/MCUdude/MicroCore)
// Clockspeed:  128 kHz internal
// BOD:         BOD disabled
// Timing:      Micros disabled
//
// Leave the rest on default settings. Don't forget to "Burn bootloader"!
// No Arduino core functions or libraries are used. Use the makefile if 
// you want to compile without Arduino IDE.
//
// Fuse settings: -U lfuse:w:0x3b:m -U hfuse:w:0xff:m


// ===================================================================================
// Libraries and Definitions
// ===================================================================================

// Libraries
#include <avr/io.h>             // for gpio
#include <util/delay.h>         // for delays

// Pin definitions
#define LED_PIN       PB0       // pin for LEDs
#define TC_SHC_PIN    PB3       // S/H cap charge/discharge pin (internal)
#define TC_PAD_PIN    PB4       // touch pad sense pin
#define TC_SHC_ADC    3         // ADC3 as S/H cap charge/discharge ADC input
#define TC_PAD_ADC    2         // ADC2 as touch pad sense ADC input

// ===================================================================================
// Touch Control (TC) Implementation (based on TinyTouchLib by Tim)
// ===================================================================================

// TC parameter definitions
#define TC_THRESHOLD_ON   70    // lower values increase sensitivity (70)
#define TC_THRESHOLD_OFF  30    // higher values decrease risk of stuck (30)
#define TC_CHARGETIME     32    // depends on touch pad und series resistor (32)
#define TC_TIMEOUT        255   // to recover from stuck (255)

// TC return values
enum {TC_OFF,TC_ON,TC_PUSH,TC_RELEASE,TC_FAIL};

// TC global variables
uint16_t TC_bias;               // no-touch reference value
uint8_t	 TC_touch;              // "is touched" flag
uint8_t  TC_timer;              // touch timer

// TC get touch pad charge delta
uint8_t TC_getDelta(void) {	
  // Precharge touch pad LOW and S/H cap HIGH
  ADMUX   =  TC_SHC_ADC;              // connect S/H control pin to S/H cap
  PORTB  |=  (1<<TC_SHC_PIN);         // charge S/H cap
  PORTB  &= ~(1<<TC_PAD_PIN);         // prepare discharge touch pad
  DDRB   |=  (1<<TC_PAD_PIN);         // discharge touch pad
  _delay_us(TC_CHARGETIME);           // wait for precharge complete

  // Perform charge sharing between touch pad and S/H cap
  DDRB   &= ~(1<<TC_PAD_PIN);         // float pad input (pull up is off)
  ADMUX   =  (1<<ADLAR) | TC_PAD_ADC; // connect touch pad to S/H and ADC
  ADCSRA |=  (1<<ADSC)  | (1<<ADIF);  // start voltage sampling
  while(!(ADCSRA & (1<<ADIF)));       // wait for sampling complete
  uint8_t dat1 = ADCH;                // read sampling result (voltage)
  
  // Precharge touch pad HIGH and S/H cap low
  ADMUX   =  TC_SHC_ADC;              // connect S/H control pin to S/H cap
  PORTB  &= ~(1<<TC_SHC_PIN);         // discharge S/H cap
  PORTB  |=  (1<<TC_PAD_PIN);         // prepare charge touch pad
  DDRB   |=  (1<<TC_PAD_PIN);         // charge touch pad
  _delay_us(TC_CHARGETIME);           // wait for precharge complete

  // Perform charge sharing between touch pad and S/H cap
  DDRB   &= ~(1<<TC_PAD_PIN);         // float touch pad input
  PORTB  &= ~(1<<TC_PAD_PIN);         // pull up off
  ADMUX   =  (1<<ADLAR) | TC_PAD_ADC; // connect touch pad to S/H and ADC
  ADCSRA |=  (1<<ADSC)  | (1<<ADIF);  // start voltage sampling
  while(!(ADCSRA & (1<<ADIF)));       // wait for sampling complete
  uint8_t dat2 = ADCH;                // read sampling result (voltage)

  // Calculate and return delta
  return dat2-dat1;
}

// TC init
void TC_init(void) {
  ADCSRA   = (1<<ADEN);               // enable ADC, prescaler 2
  DDRB    |= (1<<TC_SHC_PIN);         // set S/H cap control pin as output
  TC_bias  = TC_getDelta()<<8;        // get initial bias
  TC_touch = 0;                       // set initial touch flag
}

// TC sense touch pad
uint8_t TC_sense(void) {
  uint16_t tmp = 0;
  for(uint8_t i=16; i; i--) {
    tmp += TC_getDelta();             // average 16 samples
    _delay_us(100);                   // wait a bit between the samples
  }
  int16_t diff = tmp - (TC_bias>>4);  // difference with bias value
  if(!TC_touch) {                     // not touched previously?
    if(diff > TC_THRESHOLD_ON) {      // new touch detected?
      TC_touch = 1;                   // set "is touched" flag
      TC_timer = 0;                   // reset touch timer
      return TC_PUSH;                 // return "new push"
    }
    TC_bias = (TC_bias - (TC_bias>>6)) + (tmp>>2); // update bias (low pass)
    return TC_OFF;                    // return "still not pushed" 
  }
  else {                              // touched previously?
    if(diff < TC_THRESHOLD_OFF) {     // touch button released?
      TC_touch = 0;                   // clear "is touched" flag
      return TC_RELEASE;              // return "touch pad released"
    }
    if(TC_timer == TC_TIMEOUT) {      // still touched but for too long?
      TC_bias  = TC_getDelta()<<8;    // maybe stuck situation, read new bias
      return TC_FAIL;                 // return "fail"
    }
    TC_timer++;                       // increase timer
    return TC_ON;                     // return "still pushed"
  }
}

// ===================================================================================
// Main Function
// ===================================================================================

// Main function
int main(void) {
  // Local variables
  uint8_t bright = 64;                // current brightness of LEDs 
  uint8_t dir    = 0;                 // current fade direction

  // Setup
  TCCR0A = (1<<COM0A1)                // clear OC0A on compare match, set at TOP
         | (1<<WGM01) | (1<<WGM00);   // fast PWM
  TCCR0B = (1<<CS00);                 // start timer without prescaler
  DDRB   = (1<<LED_PIN);              // set LED pin as output
  ACSR   = (1<<ACD);                  // disable analog comperator
  DIDR0  = 0x1F;                      // disable digital intput buffer 
  TC_init();                          // setup touch control
  
  // Loop
  while(1) {
    uint8_t sense = TC_sense();       // read touch pad
    if(sense == TC_PUSH) dir = !dir;  // change fade direction on new push
    if(sense == TC_ON) {              // fade on touch hold
      if(dir) {                       // fade in?
        if(bright < 196) bright++;    // increase brightness
      }
      else {                          // fade out?
        if(bright) bright--;          // decrease brightness
      }
    }    
    OCR0A = bright;                   // set brightness (PWM)
  }
}
