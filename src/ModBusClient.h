#ifndef MODBUSCLIENT_H
#define MODBUSCLIENT_H

#include <EthernetTcpClient.h>
#include "HmiReg.h"

class ModBusClient {
public:
    explicit ModBusClient(ClearCore::EthernetTcpClient& tcpClient);

    // Coil operations
    bool readCoil(uint16_t address); // Read a single coil by address
    bool readCoil(const HmiReg<bool>& coilReg); // Read a coil using HmiReg
    
    void writeCoil(uint16_t address, bool value); // Write a single coil by address
    void writeCoil(const HmiReg<bool>& coilReg); // Write a coil using HmiReg

    // Holding register operations
    uint16_t readHoldingRegister(uint16_t address); // Read a single holding register by address
    uint16_t readHoldingRegister(const HmiReg<uint16_t>& reg); // Read using HmiReg

    void writeHoldingRegister(uint16_t address, uint16_t value); // Write a single holding register by address
    void writeHoldingRegister(const HmiReg<uint16_t>& reg); // Write using HmiReg

    // Generalized write for HmiReg<bool> or HmiReg<uint16_t>
    template <typename T>
    void write(const HmiReg<T>& reg);
    template <typename T>
    void read(HmiReg<T> *reg); // Read a coil using HmiReg

private:
    ClearCore::EthernetTcpClient& client;

    // Helpers for Modbus TCP
    void sendRequest(const uint8_t* request, size_t length) const;
    void receiveResponse(uint8_t* response, size_t maxLength) const;
    void buildModbusRequest(uint8_t functionCode, uint16_t address, uint16_t value, uint8_t* request);
};

#endif // MODBUSCLIENT_H