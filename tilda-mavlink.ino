// Mavlink Display for Tilda MKE
// Based on Arduino MAVLink test code.

// Pin D18 and D19 used for Serial1 UART
// D19 = Yellow wire to Radio Dongle (Marked TX on diagram)
// D18 = Blue wire to Radio Dongle (Marked RX on diagram)

#include <SPI.h>
#include <glcd.h>
#include <fonts/allFonts.h>
#define LCD_POWER (40u)
#define LCD_BACKLIGHT (35u)

#include "C:\Users\jamesp\Documents\Arduino\libraries\mavlink\common\mavlink.h"        // Mavlink interface

void setup() {
  SerialUSB.begin(57600);
  delay(3000); 
  //while (!SerialUSB) {
  //  ; // wait for serial port to connect. Needed for native USB.
  // TODO: Enhance this to give up after a few seconds! We won't always have USB.
  //}
  SerialUSB.println("USB Serial Console Opened");
  SerialUSB.println("Turning on LCD...");
  pinMode(LCD_POWER, OUTPUT);
  digitalWrite(LCD_POWER, LOW);
  //Turn Backlight On
  pinMode(LCD_BACKLIGHT, OUTPUT);
  digitalWrite(LCD_BACKLIGHT, HIGH);
  //Init LCD
  GLCD.Init(NON_INVERTED); 
  GLCD.SelectFont(System5x7);
  GLCD.print("MavLink Decode v0.1");
  GLCD.display();
  SerialUSB.println("Opening Radio Dongle");
  Serial1.begin(57600);
  tone(DAC0,262,500);
}

void loop() {
  comm_receive();
}

void handle_message(mavlink_message_t *msg, mavlink_status_t *status) {
  switch(msg->msgid)
      {
        case MAVLINK_MSG_ID_VFR_HUD:
          SerialUSB.print("Got a HUD message! Alt=");
          SerialUSB.println(mavlink_msg_vfr_hud_get_alt(msg));
          GLCD.CursorTo(0, 1);
          GLCD.print("Alt: ");
          GLCD.CursorTo(6, 1);
          GLCD.print(mavlink_msg_vfr_hud_get_alt(msg));
          GLCD.display();
          break;
          
        case MAVLINK_MSG_ID_HEARTBEAT:
          mavlink_heartbeat_t hb;
          mavlink_msg_heartbeat_decode(msg, &hb);
          char debugStr[100];

          tone(DAC0,750,100);
          sprintf(debugStr, "HEARTBEAT: AP %x BM: %x: SS: %x MV: %x",hb.autopilot,hb.base_mode,hb.system_status,hb.mavlink_version);
          SerialUSB.println(debugStr);
          break;
          
        default:
         // Serial.println("Unknown msg");
          break;
      }
}

void comm_receive() {

  mavlink_message_t msg;
  mavlink_status_t status;
  
  while(Serial1.available() > 0 ) 
  {
    // Serial.println("reading message");
    uint8_t c = Serial1.read();
    //SerialUSB.print(c,HEX);
    //SerialUSB.print(" ");
    // Try to get a new message
    if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      //SerialUSB.println("END");
      handle_message(&msg, &status);
      
    }
    // And get the next one
  }
}

/*
Tone generator
v1  use timer, and toggle any digital pin in ISR
   funky duration from arduino version
   TODO use FindMckDivisor?
   timer selected will preclude using associated pins for PWM etc.
    could also do timer/pwm hardware toggle where caller controls duration
*/

// frequency (in hertz) and duration (in milliseconds).

// timers TC0 TC1 TC2   channels 0-2 ids 0-2  3-5  6-8     AB 0 1
// use TC1 channel 0 
#define TONE_TIMER TC1
#define TONE_CHNL 0
#define TONE_IRQ TC3_IRQn

// TIMER_CLOCK4   84MHz/128 with 16 bit counter give 10 Hz to 656KHz
//  piano 27Hz to 4KHz

static uint8_t pinEnabled[PINS_COUNT];
static uint8_t TCChanEnabled = 0;
static boolean pin_state = false ;
static Tc *chTC = TONE_TIMER;
static uint32_t chNo = TONE_CHNL;

volatile static int32_t toggle_count;
static uint32_t tone_pin;

void tone(uint32_t ulPin, uint32_t frequency, int32_t duration)
{
    const uint32_t rc = VARIANT_MCK / 256 / frequency; 
    tone_pin = ulPin;
    toggle_count = 0;  // strange  wipe out previous duration
    if (duration > 0 ) toggle_count = 2 * frequency * duration / 1000;
     else toggle_count = -1;

    if (!TCChanEnabled) {
      pmc_set_writeprotect(false);
      pmc_enable_periph_clk((uint32_t)TONE_IRQ);
      TC_Configure(chTC, chNo,
        TC_CMR_TCCLKS_TIMER_CLOCK4 |
        TC_CMR_WAVE |         // Waveform mode
        TC_CMR_WAVSEL_UP_RC ); // Counter running up and reset when equals to RC
  
      chTC->TC_CHANNEL[chNo].TC_IER=TC_IER_CPCS;  // RC compare interrupt
      chTC->TC_CHANNEL[chNo].TC_IDR=~TC_IER_CPCS;
       NVIC_EnableIRQ(TONE_IRQ);
                         TCChanEnabled = 1;
    }
    if (!pinEnabled[ulPin]) {
      pinMode(ulPin, OUTPUT);
      pinEnabled[ulPin] = 1;
    }
    TC_Stop(chTC, chNo);
                TC_SetRC(chTC, chNo, rc);    // set frequency
    TC_Start(chTC, chNo);
}

void noTone(uint32_t ulPin)
{
  TC_Stop(chTC, chNo);  // stop timer
  digitalWrite(ulPin,LOW);  // no signal on pin
}

// timer ISR  TC1 ch 0
void TC3_Handler ( void ) {
  TC_GetStatus(TC1, 0);
  if (toggle_count != 0){
    // toggle pin  TODO  better
    digitalWrite(tone_pin,pin_state= !pin_state);
    if (toggle_count > 0) toggle_count--;
  } else {
    noTone(tone_pin);
  }
}



