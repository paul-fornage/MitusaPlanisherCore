/*
  ModbusTCP Client for ClearCode Arduino wrapper

  (c)2021 Alexander Emelianov (a.m.emelianov@gmail.com)
  https://github.com/emelianov/modbus-esp8266

  This code is licensed under the BSD New License. See LICENSE.txt for more info.
*/

#include <Arduino.h>
#include <StaticVector.h>


#include "ClearCore.h"
#include "EthernetTcpClient.h"

#include "MiTcpMessage.h"

StaticVector<uint8_t, MITCP_MESSAGE_BUFFER_SIZE> InBuffer;

EthernetTcpClient client;

void setup() {

  ConnectorUsb.PortOpen();
  const uint32_t startTime = millis();
  while (!ConnectorUsb)
    continue;

  EthernetMgr.Setup();
  const bool dhcpSuccess = EthernetMgr.DhcpBegin();
  while (dhcpSuccess) {
    ConnectorUsb.Send("DHCP Success. IP: ");
    ConnectorUsb.SendLine(EthernetMgr.LocalIp().StringValue());


    // Start a TCP connection with the server on port 8888.
    if (client.Connect(IpAddress(192, 168, 1, 15), 8888)) {
      while (client.Connected()) {
        while (client.BytesAvailable()) {
          const char in_word = client.Read();
          InBuffer.push_back(in_word);
          if (in_word == '\0') {
            const auto message = MiTcpMessage::decodeMessage(InBuffer);
            if (!message.second) {
              ConnectorUsb.SendLine("Failed to decode message");
            } else {
              ConnectorUsb.SendLine(message.first.to_string().c_str());
            }
            InBuffer.clear();
          }
          if (InBuffer.size() >= MITCP_MESSAGE_BUFFER_SIZE) {
            InBuffer.clear();
            ConnectorUsb.SendLine("Buffer Overflow, received 4096 bytes with no null");
          }
        }
      }
    }
    ConnectorUsb.SendLine("connection ended, retrying");
  }
  ConnectorUsb.SendLine("DHCP shat the bed");


}


void loop() {

}
