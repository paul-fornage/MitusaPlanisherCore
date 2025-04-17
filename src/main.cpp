/*
  ModbusTCP Client for ClearCode Arduino wrapper

  (c)2021 Alexander Emelianov (a.m.emelianov@gmail.com)
  https://github.com/emelianov/modbus-esp8266

  This code is licensed under the BSD New License. See LICENSE.txt for more info.
*/

#include <Ethernet.h>       // Ethernet library v2 is required

#include <ModbusAPI.h>
#include <ModbusTCPTemplate.h>

class ModbusEthernet : public ModbusAPI<ModbusTCPTemplate<EthernetServer, EthernetClient>> {};

const uint16_t REG = 100;               // Modbus Hreg Offset
IPAddress remote(192, 168, 0, 211);  // Address of Modbus Slave device
const int32_t showDelay = 500;   // Show result every n'th mellisecond

bool usingDhcp = false;
byte mac[] = { 0x24, 0x15, 0x10, 0xB0, 0x45, 0xA4 }; // MAC address is ignored but because of C++ types, you still need to give it garbage
IPAddress ip(192, 168, 0, 178); // The IP address will be dependent on your local network
ModbusEthernet mb;               // Declare ModbusTCP instance

void setup() {
  Serial.begin(115200);
  uint32_t timeout = 5000;
  uint32_t startTime = millis();
  while (!Serial && millis() - startTime < timeout)
    continue;

  Ethernet.begin(mac, ip);

  // Make sure the physical link is up before continuing.
  while (Ethernet.linkStatus() == LinkOFF) {
    Serial.println("The Ethernet cable is unplugged...");
    delay(1000);
  }
  mb.client();              // Act as Modbus TCP server
}

bool coil = false;
uint16_t reg1 = 0;
uint16_t reg2 = 0;
uint32_t showLast = 0;
uint16_t sum = 0;

void loop() {
  if (mb.isConnected(remote)) {   // Check if connection to Modbus Slave is established
    mb.readCoil(remote, 100, &coil);  // Initiate Read Hreg from Modbus Slave
    Serial.print("coil 100: ");
    Serial.println(coil ? "True" : "False");
    mb.readHreg(remote, 16, &reg1);  // Initiate Read Hreg from Modbus Slave
    Serial.print("reg 16: ");
    Serial.println(reg1);
    mb.readHreg(remote, 17, &reg2);  // Initiate Read Hreg from Modbus Slave
    Serial.print("reg 17: ");
    Serial.println(reg2);
    sum = reg1+reg2;
    mb.writeHreg(remote, 18, &sum);  // Initiate Read Hreg from Modbus Slave

  } else {
    mb.connect(remote);           // Try to connect if not connected
  }
  delay(10);                     // Pulling interval
  mb.task();                      // Common local Modbus task
}