/*
  ModbusTCP Client for ClearCode Arduino wrapper

  (c)2021 Alexander Emelianov (a.m.emelianov@gmail.com)
  https://github.com/emelianov/modbus-esp8266

  This code is licensed under the BSD New License. See LICENSE.txt for more info.
*/

#include <Arduino.h>
#include "ClearCore.h"
#include "EthernetTcpClient.h"


void setup() {

  ConnectorUsb.PortOpen();
  const uint32_t startTime = millis();
  while (!ConnectorUsb)
    continue;



  // Debug delay to be able to restart motor before program starts
  delay(4000);

  EthernetMgr.Setup();
  const bool dhcpSuccess = EthernetMgr.DhcpBegin();
  while (dhcpSuccess) {
    ConnectorUsb.Send("DHCP Success. IP: ");
    ConnectorUsb.SendLine(EthernetMgr.LocalIp().StringValue());

    EthernetTcpClient client;
    // Start a TCP connection with the server on port 8888.
    if (client.Connect(IpAddress(192, 168, 1, 15), 8888)) {
      while (client.Connected()) {
        while (client.BytesAvailable()) {
          ConnectorUsb.SendChar(static_cast<uint8_t>(client.Read()));
        }
        while (ConnectorUsb.AvailableForRead()) {
          client.Send(static_cast<uint8_t>(ConnectorUsb.CharGet()));
        }
      }
    }
    ConnectorUsb.SendLine("connection ended, retrying");
  }

  ConnectorUsb.SendLine("DHCP shat the bed");


}


void loop() {

}
