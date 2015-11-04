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
  //radioDongle.begin(57600); 
  Serial1.begin(57600);
}

void loop() {

    //int sysid = 20;                   ///< ID 20 for this airplane
    //int compid = MAV_COMP_ID_IMU;     ///< The component sending the message is the IMU, it could be also a Linux process
    //int type = MAV_TYPE_QUADROTOR;   ///< This system is an airplane / fixed wing
 
// Define the system type, in this case an airplane
   // uint8_t system_type = MAV_TYPE_FIXED_WING;
   // uint8_t autopilot_type = MAV_AUTOPILOT_GENERIC;
 
  //  uint8_t system_mode = MAV_MODE_PREFLIGHT; ///< Booting up
  //  uint32_t custom_mode = 0;                 ///< Custom mode, can be defined by user/adopter
  //  uint8_t system_state = MAV_STATE_STANDBY; ///< System ready for flight
    // Initialize the required buffers
  //  mavlink_message_t msg;
  //  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
 
// Pack the message
   // mavlink_msg_heartbeat_pack(sysid,compid, &msg, type, autopilot_type, system_mode, custom_mode, system_state);
 
// Copy the message to the send buffer
  //  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
 
// Send the message with the standard UART send function
// uart0_send might be named differently depending on
// the individual microcontroller / library in use.
     //delay(1000);
    // radioDongle.write(buf, len);
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
      // Handle message
   //SerialUSB.println("END");
      // Serial.print(msg.msgid);
      switch(msg.msgid)
      {
        case MAVLINK_MSG_ID_VFR_HUD:
          SerialUSB.print("Got a HUD message! Alt=");
          SerialUSB.println(mavlink_msg_vfr_hud_get_alt(&msg));
          break;
        case MAVLINK_MSG_ID_HEARTBEAT:
          mavlink_heartbeat_t hb;
          mavlink_msg_heartbeat_decode(&msg, &hb);
          SerialUSB.println("HEARTBEAT");
          
          break;
        default:
         // Serial.println("Unknown msg");
          break;
      }
    }
    // And get the next one
  }
}
       
       
       

