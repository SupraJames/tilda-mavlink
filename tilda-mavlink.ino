// Mavlink Display for Tilda MKE
// Based on Arduino MAVLink test code.

// Pin D18 and D19 used for Serial1 UART
// D19 = Yellow wire to Radio Dongle (Marked TX on diagram)
// D18 = Blue wire to Radio Dongle (Marked RX on diagram)

#include "C:\Users\jamesp\Documents\Arduino\libraries\mavlink\common\mavlink.h"        // Mavlink interface

void setup() {
  SerialUSB.begin(57600);    
  while (!SerialUSB) {
    ; // wait for serial port to connect. Needed for native USB
  }
  SerialUSB.println("USB Serial Console Opened");
  SerialUSB.println("Opening Radio Dongle");
  Serial1.begin(57600);
}

void loop() {
  comm_receive();
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
      switch(msg.msgid)
      {
        case MAVLINK_MSG_ID_VFR_HUD:
          SerialUSB.print("Got a HUD message! Alt=");
          SerialUSB.println(mavlink_msg_vfr_hud_get_alt(&msg));
          break;
        case MAVLINK_MSG_ID_HEARTBEAT:
          mavlink_heartbeat_t hb;
          mavlink_msg_heartbeat_decode(&msg, &hb);

          char debugStr[100];
          sprintf(debugStr, "HEARTBEAT: AP %x BM: %x SS: %x MV: %x",hb.autopilot,hb.base_mode,hb.system_status,hb.mavlink_version);
          
          SerialUSB.println(debugStr);
          
          break;
        default:
         // Serial.println("Unknown msg");
          break;
      }
    }
    // And get the next one
  }
}
       
       
       

