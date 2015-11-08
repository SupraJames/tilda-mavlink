# tilda-mavlink

Mavlink message decoding on the Tilda MKe

Takes mavlink messages from the on board serial UART, decodes them, and displays them on the LCD. The messages can come straight from an autopilot modules, or a telemetry system like the 3DR radios, and clones.

I'm afraid there is not much here yet :)

Currently, the following messages are decoded:

MAVLINK_MSG_ID_HEARTBEAT for system status
MAVLINK_MSG_ID_VFR_HUD for Alt, Speed, Heading, Course etc
MAVLINK_MSG_ID_GPS_RAW_INT for GPS data

This small project is based on the Tilda MKE, an Arduino Due like board with an LCD, from EMF Camp 2014
See https://badge.emfcamp.org/wiki/TiLDA_MKe for more details about the hardware.
