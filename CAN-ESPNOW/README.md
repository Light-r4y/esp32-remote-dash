This directory contains source code for master transceiver wireless(ESPNOW) based on board Wemos ESP32-S2.

CAN Tranceiver use module TJA1050 from China market.

Connection:

CANTr --  ESP

Vcc   --  3.3V

TX    --  PIN40

RX    --  PIN39

GND   --  GND

For power 12V->5V use MP1584

This project can listen CAN bus on my car (LADA Granta) and send to display board RPM, coolant temp (CLT) and Speed value.

![wemos-s2](https://github.com/Light-r4y/esp32-remote-dash/blob/main/media/wemos-s2.jpg)
