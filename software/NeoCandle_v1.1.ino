// NeoCandle
//
// Candle simulation using an ATtiny25/45/85 and four NeoPixels.
// The candle can be controlled by an IR remote control.
// It can be set into an automatic mode in which it's been
// turned on or off by the intensity of the ambient light.
// The code uses sleep mode in order to decrease power consumption.
//
//                               +-\/-+
// LDR ----------- A0 (D5) PB5  1|°   |8  Vcc
// NEOPIXELS ----- A3 (D3) PB3  2|    |7  PB2 (D2) A1 ---
// IR RECEIVER --- A2 (D4) PB4  3|    |6  PB1 (D1) ------
//                         GND  4|    |5  PB0 (D0) ------
//                               +----+
//
// RESET pin is used as a weak analog input for the LDR light sensor.
// You don't need to disable the RESET pin as the voltage won't go
// below 40% of Vcc.
//
// Core:    ATtinyCore (https://github.com/SpenceKonde/ATTinyCore)
// Board:   ATtiny25/45/85 (No bootloader)
// Chip:    ATtiny25 or 45 or 85 (depending on your chip)
// Clock:   8 MHz (internal)
// Millis:  disabled
// B.O.D.:  disabled
// Leave the rest on default settings. Don't forget to "Burn bootloader"!
// No Arduino core functions or libraries are used. Use the makefile if 
// you want to compile without Arduino IDE.
//
// The Neopixel implementation was inspired by Josh Levine.
// http://wp.josh.com/2014/05/11/ws2812-neopixels-made-easy/
//
// The simulation code is based on the great work of Mark Sherman.
// https://github.com/carangil/candle
//
// The lightweight pseudo random number generator based on 
// Galois linear feedback shift register is taken from Łukasz Podkalicki.
// https://blog.podkalicki.com/attiny13-pseudo-random-numbers/
//
// The IR receiver implementation (NEC protocol) is based on TinyDecoder
// https://github.com/wagiminator/ATtiny13-TinyDecoder
//
// 2019 - 2021 by Stefan Wagner 
// Project Files (EasyEDA): https://easyeda.com/wagiminator
// Project Files (Github):  https://github.com/wagiminator
// License: http://creativecommons.org/licenses/by-sa/3.0/


// Libraries
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// Pins
#define NEO_PIN       3       // Pin for neopixels
#define IR_PIN        4       // Pin for IR receiver
#define LDR_AP        0       // ADC port of LDR

// IR codes
#define IR_ADDR       0xEF00  // IR device address
#define IR_PWRON      0x03    // IR code for power on
#define IR_PWROFF     0x02    // IR code for power off
#define IR_FAIL       0xFF    // IR fail code

// LDR parameters
#define LDR_DARK      900     // LDR ADC threshold value for dark
#define LDR_BRIGHT    800     // LDR ADC threshold value for bright

// Auto switch off timer
#define AUTOTIMER     (6*3600000)

// Global variables
uint32_t timermillis  = 0;    // timer variable for auto switch off
uint8_t  ldrmode      = 0;    // LDR mode flag: "1" = LDR mode on

// -----------------------------------------------------------------------------
// Neopixel Implementation (8MHz Clock, 800kHz Pixels) inspired by Josh Levine
// -----------------------------------------------------------------------------

#define NEO_PIXELS    4                       // number of pixels in the string
#define NEO_init()    DDRB |= (1<<NEO_PIN)    // set pixel DATA pin as output
#define NEO_latch()   _delay_us(251)          // delay to show shifted colors

// Send a byte to the pixels string (DATA LOW is at least 900ns due to the loop)
void NEO_sendByte(uint8_t byte) {
  for(uint8_t bit=8; bit; bit--, byte<<=1) {  // 8 bits, MSB first
    if(byte & 0x80) asm volatile (            // "1"-bit ?
      "sbi  %[port], %[bit]   \n\t"           // DATA HIGH
      "nop                    \n\t"           // delay:     1 cycle  = 125ns |
      "lpm                    \n\t"           // delay:     3 cycles = 375ns | ~800ns
      "cbi  %[port], %[bit]   \n\t"           // DATA LOW:  2 cycles = 250ns |
      ::
      [port]  "I" (_SFR_IO_ADDR(PORTB)),
      [bit]   "I" (NEO_PIN)
    );
    else asm volatile (                       // "0"-bit ?
      "sbi  %[port], %[bit]   \n\t"           // DATA HIGH
      "nop                    \n\t"           // delay:     1 cycle  = 125ns |
      "cbi  %[port], %[bit]   \n\t"           // DATA LOW:  2 cycles = 250ns | ~400ns
      ::
      [port]  "I" (_SFR_IO_ADDR(PORTB)),
      [bit]   "I" (NEO_PIN)
    );
  }
}

// Switch off all pixels
void NEO_clear(void) {
  cli();
  for(uint8_t i = 3 * NEO_PIXELS; i; i--) NEO_sendByte(0);
  sei();
  NEO_latch();
}

// -----------------------------------------------------------------------------
// Millis Counter Implementation for Timer0
// -----------------------------------------------------------------------------

#define MIL_TOP   (F_CPU/64000UL)-1   // timer TOP value for 1kHz
volatile uint32_t MIL_counter = 0;    // millis counter variable

// Init millis counter
void MIL_init(void) {
  OCR0A  = MIL_TOP;                   // TOP-value for 1kHz
  TCCR0A = (1<<WGM01);                // timer0 CTC mode
  TCCR0B = (1<<CS01)|(1<<CS00);       // start timer0 with prescaler 64
  TIMSK |= (1<<OCIE0A);               // enable output compare match interrupt
}

// Read millis counter
uint32_t MIL_read(void) {
  cli();                              // disable interrupt for atomic read
  uint32_t result = MIL_counter;      // read millis counter
  sei();                              // enable interrupts
  return(result);                     // return millis counter value
}

// Timer0 compare match A interrupt service routine (every millisecond)
ISR(TIM0_COMPA_vect) {
  MIL_counter++;                      // increase millis counter
}

// -----------------------------------------------------------------------------
// IR Receiver Implementation (NEC Protocol) using Timer1
// -----------------------------------------------------------------------------

// IR receiver definitions and macros
#define IR_available()  (~PINB & (1<<IR_PIN))       // return true if IR line is low
#define IR_TOP          ((F_CPU / 1000UL) * 15000UL) / 512000UL
#define IR_9000us       ((F_CPU / 1000UL) *  9000UL) / 512000UL
#define IR_4500us       ((F_CPU / 1000UL) *  4500UL) / 512000UL
#define IR_1687us       ((F_CPU / 1000UL) *  1687UL) / 512000UL
#define IR_562us        ((F_CPU / 1000UL) *   562UL) / 512000UL

// IR global variables
volatile uint8_t IR_duration;             // for storing duration of last burst/pause
volatile uint8_t IR_flag;                 // gets zero in pin change or time over

// IR initialize the receiver
void IR_init(void) {
  DDRB  &= ~(1<<IR_PIN);                  // IR pin as input
  PCMSK |=  (1<<IR_PIN);                  // enable interrupt on IR pin
  GIMSK |=  (1<<PCIE);                    // enable pin change interrupts
  OCR1A  = IR_TOP;                        // timeout causes OCA interrupt
  TCNT1  = 0;                             // reset timer1
  TIMSK |= (1<<OCIE1A);                   // enable output compare match interrupt
}

// IR check if current signal length matches the desired duration
uint8_t IR_checkDur(uint8_t dur) {
  IR_flag = 1;
  if(!IR_duration) return 0;
  uint8_t error = dur >> 3; if (error < 6) error = 6;
  if (IR_duration > dur) return ((IR_duration - dur) < error);
  return ((dur - IR_duration) < error);
}

// IR read data according to NEC protocol
uint8_t IR_read(void) {
  uint32_t data;                                // variable for received data
  uint16_t addr;                                // variable for received address
  if (!IR_available()) return IR_FAIL;          // exit if no signal
  IR_flag = 1;                                  // reset flag
  while(IR_flag);                               // wait for end of start burst
  if (!IR_checkDur(IR_9000us)) return IR_FAIL;  // exit if no start condition
  while(IR_flag);                               // wait for end of start pause
  if (!IR_checkDur(IR_4500us)) return IR_FAIL;  // exit if no start condition
  for (uint8_t i=32; i; i--) {                  // receive 32 bits
    data >>= 1;                                 // LSB first
    while(IR_flag);                             // wait for end of burst
    if (!IR_checkDur(IR_562us)) return IR_FAIL; // exit if burst has incorrect length
    while(IR_flag);                             // wait for end of pause
    if (IR_checkDur(IR_1687us)) data |= 0x80000000;   // bit "0" or "1" depends on pause duration
    else if (!IR_checkDur(IR_562us)) return IR_FAIL;  // exit if it's neither "0" nor "1"
  }
  while(IR_flag);                               // wait for end of final burst
  if (!IR_checkDur(IR_562us)) return IR_FAIL;   // exit if burst has incorrect length
  uint8_t addr1 = data;                         // get first  address byte
  uint8_t addr2 = data >> 8;                    // get second address byte
  uint8_t cmd1  = data >> 16;                   // get first  command byte
  uint8_t cmd2  = data >> 24;                   // get second command byte
  if ((cmd1 + cmd2) < 255) return IR_FAIL;      // if second command byte is not the inverse of the first
  if ((addr1 + addr2) == 255) addr = addr1;     // check if it's extended NEC-protocol ...
  else addr = data;                             // ... and get the correct address
  if (addr != IR_ADDR) return IR_FAIL;          // wrong address
  return cmd1;                                  // return command code
}

// Pin change interrupt service routine
ISR (PCINT0_vect) {
  IR_duration = TCNT1;                    // save timer value
  TCNT1   = 0;                            // reset timer1
  TCCR1   = (1<<CS13)|(1<<CS11);          // start timer1 with prescaler 512
  IR_flag = 0;                            // raise flag
}

// Timer1 compare match A interrupt service routine (timeout)
ISR(TIM1_COMPA_vect) {
  TCCR1   = 0;                            // stop timer1
  TCNT1   = 0;                            // reset timer1
  IR_flag = 0;                            // raise flag
  IR_duration = 0;                        // set duration value to zero
}

// -----------------------------------------------------------------------------
// ADC and LDR Implementation
// -----------------------------------------------------------------------------

// Init ADC for LDR light sensor
void LDR_init(void) {
  ADCSRA = (1<<ADPS2) | (1<<ADPS1);       // set ADC clock prescaler to 64
  ADMUX  = LDR_AP;                        // set LDR port against Vcc
}

// Read LDR sensor value
uint16_t LDR_read(void) {
  PRR &= ~(1<<PRADC);                     // power on ADC
  ADCSRA |= (1<<ADEN) | (1<<ADSC);        // enable ADC and start sampling
  while (ADCSRA & (1<<ADSC));             // wait for sampling complete
  uint16_t result = ADC;                  // read ADC value;
  ADCSRA &= ~(1<<ADEN);                   // disable ADC
  PRR |= (1<<PRADC);                      // power off ADC
  return(result);
}

// Switch on LDR mode
void switchOnLDRmode(void) {
  // Show switching animation
  for (int i=0; i<4; i++) {
    cli();
    for(uint8_t j=0; j<3*NEO_PIXELS; j++)
      (j == ((i << 1) + i + 2)) ? NEO_sendByte(128) : NEO_sendByte(0);
    sei();
    _delay_ms(250);
  }

  // Switch on LDR mode
  ldrmode = 1;
}

// -----------------------------------------------------------------------------
// Standby, Sleep and Watchdog Implementation
// -----------------------------------------------------------------------------

// Reset watchdog timer
void resetWatchdog (void) {
  cli();                                  // timed sequence coming up
  MCUSR = 0;                              // clear various "reset" flags
  WDTCR = (1<<WDCE)|(1<<WDE)|(1<<WDIF);   // allow changes, clear existing interrupt
  WDTCR = (1<<WDIE)|(1<<WDP3)|(1<<WDP0);  // enable watchdog, set interval to 8 seconds
  wdt_reset();                            // pat the dog
  sei();                                  // interrupts are required now
}

// Go to sleep in order to save energy, wake up again by watchdog timer or pin change interrupt
void sleep(void) {
  GIFR  |= (1<<PCIF);                     // clear any outstanding interrupts
  if(ldrmode) resetWatchdog();            // get watchdog ready for LDR mode
  sleep_mode();                           // sleep
}

// Go to standby mode
void gotoStandby(void) {
  NEO_clear();                            // turn off NeoPixels
  
  // Sleep until IR remote power on button is pressed
  while(1) {
    if( (IR_available()) && (IR_read() == IR_PWRON) ) {
      ldrmode = 0;
      timermillis = MIL_read();
      break;
    }
    if(ldrmode && (LDR_read() > LDR_DARK)) break;
    sleep();
  }
}

// Watchdog interrupt service routine
ISR (WDT_vect) {
  wdt_disable();                          // disable watchdog
}

// -----------------------------------------------------------------------------
// Pseudo Random Number Generator (adapted from Łukasz Podkalicki)
// -----------------------------------------------------------------------------

// Start state (any nonzero value will work)
uint16_t rn = 0xACE1;

// Pseudo random number generator
uint16_t prng(uint16_t maxvalue) {
  rn = (rn >> 0x01) ^ (-(rn & 0x01) & 0xB400);
  return(rn % maxvalue);
}

// -----------------------------------------------------------------------------
// Candle Simulation Implementation (adapted from Mark Sherman)
// -----------------------------------------------------------------------------

// Candle simulation parameters
#define MINUNCALM     ( 5 * 256)
#define MAXUNCALM     (60 * 256)
#define UNCALMINC     10
#define MAXDEV        100
#define CANDLEDELAY   25

// Some variables
int16_t centerx = MAXDEV;
int16_t centery = MAXDEV / 2;
int16_t xvel = 0;
int16_t yvel = 0;
uint16_t uncalm =   MINUNCALM;
int16_t uncalmdir = UNCALMINC;
uint8_t cnt = 0;

// Set one candle LED
void setPixel(int val) {
  if (val > 255) val = 255;   
  if (val < 0  ) val = 0;
  uint8_t byte = (uint8_t)val;
  NEO_sendByte(byte >> 1);
  NEO_sendByte(byte >> 2);
  NEO_sendByte(byte >> 5);
}

// Candle simulation
void updateCandle() {
  int movx=0;
  int movy=0;
    
  // Random trigger brightness oscillation, if at least half uncalm
  if (uncalm > (MAXUNCALM / 2)) {
    if (prng(2000) < 5) uncalm = MAXUNCALM * 2;  //occasional 'bonus' wind
  }
   
  // Random poke, intensity determined by uncalm value (0 is perfectly calm)
  movx = prng(uncalm >> 8) - (uncalm >> 9);
  movy = prng(uncalm >> 8) - (uncalm >> 9);
  
  // If reach most calm value, start moving towards uncalm
  if (uncalm < MINUNCALM) uncalmdir =  UNCALMINC;
  
  // If reach most uncalm value, start going towards calm
  if (uncalm > MAXUNCALM) uncalmdir = -UNCALMINC;
  uncalm += uncalmdir;

  // Move center of flame by the current velocity
  centerx += movx + (xvel >> 2);
  centery += movy + (yvel >> 2); 
  
  // Range limits
  if (centerx < -MAXDEV) centerx = -MAXDEV;
  if (centerx >  MAXDEV) centerx =  MAXDEV;
  if (centery < -MAXDEV) centery = -MAXDEV; 
  if (centery >  MAXDEV) centery =  MAXDEV;

  // Counter
  cnt++;
  if (! (cnt & 3)) {
    // Attenuate velocity 1/4 clicks 
    xvel = (xvel * 999) / 1000;
    yvel = (yvel * 999) / 1000;
  }

  // Apply acceleration towards center, proportional to distance from center (spring motion; hooke's law)
  xvel -= centerx;
  yvel -= centery;

  // Set NeoPixels
  cli();
  setPixel(128 - centerx - centery);
  setPixel(128 + centerx - centery);
  setPixel(128 + centerx + centery);
  setPixel(128 - centerx + centery);
  sei();
}

// -----------------------------------------------------------------------------
// Main Function
// -----------------------------------------------------------------------------

int main(void) {
  // Reset watchdog timer
  resetWatchdog();                        // do this first in case WDT fires

  // Local variables
  uint32_t lastmillis = 0;

  // Disable unused peripherals and set sleep mode to save power
  ACSR   =  (1<<ACD);                     // disable analog comperator
  DIDR0  = ~(1<<IR_PIN) & 0x1F;           // disable digital intput buffer except IR pin
  PRR    =  (1<<PRUSI) | (1<<PRADC);      // shut down USI and ADC
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);    // set sleep mode to power down

  // Setup
  NEO_init();                             // init Neopixels
  LDR_init();                             // init ADC for LDR light sensor
  MIL_init();                             // init millis counter
  IR_init();                              // init IR receiver
  sei();                                  // enable global interrupts

  // Loop
  while(1) {
    // Update candle simulation
    if ((MIL_read() - lastmillis) > CANDLEDELAY) {
      updateCandle();
      lastmillis = MIL_read();
    }

    // Check for IR remote signals
    if (IR_available()) {
      uint8_t keypressed = IR_read();
      if (keypressed == IR_PWRON) switchOnLDRmode();
      if (keypressed == IR_PWROFF) {
        ldrmode = 0;
        gotoStandby();
      }
    }

    // Turn off candle in daylight when in LDR mode
    if (ldrmode && (LDR_read() < LDR_BRIGHT)) gotoStandby();

    // Check auto switch off timer when not in LDR mode
    if (!ldrmode && ((MIL_read() - timermillis) > AUTOTIMER)) gotoStandby();
  }
}
