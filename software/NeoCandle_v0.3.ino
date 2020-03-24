// NeoCandle
//
// Candle simulation using an ATtiny85 and four NeoPixels.
// The candle can be controlled by an IR remote control.
// It can be set into an automatic mode in which it's been
// turned on or off by the intensity of the ambient light.
// The code uses sleep mode in order to decrease power consumption.
// 
// The simulation code is based on the great work of Mark Sherman.
//
// 2019 by Stefan Wagner
// Licence: LGPLv3


// libraries
#include <Adafruit_NeoPixel.h>
#include <tiny_IRremote.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include <avr/wdt.h>

// pins
#define NEOPIN        3
#define IRPIN         4
#define LDRPIN        A0

// IR codes
#define IRPWRON       0xFF807F
#define IRPWROFF      0xFF906F

// LDR parameters
#define LDRDARK       900
#define LDRBRIGHT     800

// auto switch off timer
#define AUTOTIMER     (6*3600000)

// candle simulation parameters
#define BRIGHTNESS    128
#define MINUNCALM     (5*256)
#define MAXUNCALM     (60*256)
#define UNCALMINC     10
#define MAXDEV        100
#define CANDLEDELAY   25

// init NeoPixel library
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(4, NEOPIN, NEO_RGB + NEO_KHZ800);

// init IRremote library
IRrecv irrecv(IRPIN);
decode_results results;

// some variables
int centerx=MAXDEV;
int centery=MAXDEV/2;
int xvel=0;
int yvel=0;
int uncalm=MINUNCALM;
int uncalmdir= UNCALMINC;
char cnt=0;

uint32_t keypressed = 0;
uint32_t lastmillis = 0;
uint32_t timermillis;

bool ldrmode = false;


void setup() {
  // do this first in case WDT fires
  resetWatchdog();

  // set pin change interrupt for IR Pin, but don't enable it yet
  PCMSK  = bit (IRPIN);   // interrupt on IR Pin
  
  // initialize NeoPixels
  pixels.begin();
  pixels.setBrightness(BRIGHTNESS);
  pixels.show();

  // start the IR receiver
  irrecv.enableIRIn();

  // start auto switch off timer
  timermillis = millis();
}


void loop() {
  // update candle simulation
  if ((millis() - lastmillis) > CANDLEDELAY) {
    updateCandle();
    lastmillis = millis();
  }

  // check for IR remote signals
  if (irrecv.decode(&results)) {
    keypressed = results.value;
    if (keypressed == IRPWRON) switchOnLDRmode();
    if (keypressed == IRPWROFF) gotoStandby();   
    keypressed = 0;
    irrecv.resume();
  }

  // turn off candle in daylight when in LDR mode
  if (ldrmode && (analogRead(LDRPIN) < LDRBRIGHT)) dayTimeOff();

  // check auto switch off timer when not in LDR mode
  if (!ldrmode && ((millis() - timermillis) > AUTOTIMER)) gotoStandby();
}


void switchOnLDRmode() {
  // show switching animation
  for (int i=0; i<4; i++) {
    pixels.clear();
    pixels.setPixelColor( i, 0,0,255);
    pixels.show();
    delay(250);
  }

  // switch on LDR mode
  ldrmode = true;
}


void dayTimeOff() {
  // turn off NeoPixels
  pixels.clear();
  pixels.show();

  // sleep until darkness or IR remote power on button pressed
  keypressed = 0;
  do {
    sleep();
    if (irrecv.decode(&results)) {
      keypressed = results.value;
      irrecv.resume();
    }
  } while ((analogRead(LDRPIN) < LDRDARK) && (keypressed != IRPWRON));

  // turn off LDR mode if power button was pressed
  if (keypressed == IRPWRON) {
    ldrmode = false;
    timermillis = millis();
  }
}


void gotoStandby() {
  // turn off NeoPixels
  pixels.clear();
  pixels.show();

  // switch off LDR mode
  ldrmode = false;
  
  // sleep until IR remote power on button is pressed
  keypressed = 0; irrecv.resume();
  do {
    sleep();
    if (irrecv.decode(&results)) {
      keypressed = results.value;
      irrecv.resume();
    }
  } while (keypressed != IRPWRON);
  keypressed = 0;

  // reset auto switch off timer
  timermillis = millis();
}


void sleep() {
  // Go to sleep when candle is off in order to save energy.
  // Wake up again after 8 seconds (watchdog timer) or when
  // IR receiver triggers pin change interrupt.
  set_sleep_mode (SLEEP_MODE_PWR_DOWN);
  bitSet (GIFR, PCIF);    // clear any outstanding interrupts
  bitSet (GIMSK, PCIE);   // enable pin change interrupts
  ADCSRA = 0;             // turn off ADC
  power_all_disable ();   // power off ADC, Timer 0 and 1, serial interface
  noInterrupts ();        // timed sequence coming up
  resetWatchdog ();       // get watchdog ready
  sleep_enable ();        // ready to sleep
  interrupts ();          // interrupts are required now
  sleep_cpu ();           // sleep
  bitClear (GIMSK, PCIE); // disable pin change Interrupt                
  sleep_disable ();       // precaution
  power_all_enable ();    // power everything back on
  ADCSRA = bit(ADEN);     // turn ADC back on
  delay(100);             // wait for IR signal to be decoded
}


// watchdog interrupt service routine
ISR (WDT_vect) 
{
   wdt_disable();  // disable watchdog
}


// pin change interrupt service routine
ISR (PCINT0_vect) {
 // Nothing to be done here
}


void resetWatchdog () {
  // clear various "reset" flags
  MCUSR = 0;     
  // allow changes, disable reset, clear existing interrupt
  WDTCR = bit (WDCE) | bit (WDE) | bit (WDIF);
  // set interrupt mode and an interval (WDE must be changed from 1 to 0 here)
  WDTCR = bit (WDIE) | bit (WDP3) | bit (WDP0);    // set WDIE, and 8 seconds delay
  // pat the dog
  wdt_reset();  
}
  

uint32_t powcolor(int val) {
  if (val > 255) val=255;   
  if(val <0) val=0;
  return pixels.Color( val, val/2, val/16);
}


void updateCandle() {
  int movx=0;
  int movy=0;
    
  // random trigger brightness oscillation, if at least half uncalm
  if (uncalm > (MAXUNCALM/2)) {
    if (random(2000)<5) uncalm = MAXUNCALM*2;  //occasional 'bonus' wind
  }
   
  // random poke, intensity determined by uncalm value (0 is perfectly calm)
  movx = random(uncalm>>8) -(uncalm>>9);
  movy = random(uncalm>>8) -(uncalm>>9);
  
  // if reach most calm value, start moving towards uncalm
  if (uncalm < MINUNCALM) uncalmdir=UNCALMINC;
  
  // if reach most uncalm value, start going towards calm
  if (uncalm > MAXUNCALM) uncalmdir=-UNCALMINC;
  uncalm += uncalmdir;

  // move center of flame by the current velocity
  centerx+=movx +(xvel>>2);
  centery+=movy +(yvel>>2); 
  
  // range limits
  if (centerx < -MAXDEV) centerx = -MAXDEV;
  if (centerx > MAXDEV) centerx = MAXDEV;
  if (centery < -MAXDEV) centery = -MAXDEV; 
  if (centery > MAXDEV) centery = MAXDEV;

  // counter
  cnt++;
  if (! (cnt & 3)) {
    //attenuate velocity 1/4 clicks 
    xvel = (xvel *999)/1000;
    yvel = (yvel *999)/1000;
  }

  // apply acceleration towards center, proportional to distance from center (spring motion; hooke's law)
  xvel -= centerx;
  yvel -= centery;

  // set NeoPixels
  pixels.setPixelColor( 0, powcolor(128 - centerx - centery ));
  pixels.setPixelColor( 1, powcolor(128 + centerx - centery ));
  pixels.setPixelColor( 2, powcolor(128 + centerx + centery ));
  pixels.setPixelColor( 3, powcolor(128 - centerx + centery ));
  pixels.show();
}
