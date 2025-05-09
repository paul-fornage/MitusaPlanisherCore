#include "ModBusClient.h"
#include <stdexcept>
#include <cstring> // for memcpy

ModBusClient::ModBusClient(ClearCore::EthernetTcpClient& tcpClient)
    : client(tcpClient) {}

// Coil - read
bool ModBusClient::readCoil(const uint16_t address) {
    uint8_t request[12];
    uint8_t response[12];
    
    // Build request for function code 0x01 (Read Coils)
    buildModbusRequest(0x01, address, 1, request);
    sendRequest(request, sizeof(request));
    receiveResponse(response, sizeof(response));

    // Extract coil status from the response
    // A valid response has the coil data at a specific byte.
    return response[9] & 1; // Assuming 1 coil queried, mask the LSb
}

bool ModBusClient::readCoil(const HmiReg<bool>& coilReg) {
    return readCoil(coilReg.address);
}

// Coil - write
void ModBusClient::writeCoil(const uint16_t address, const bool value) {
    uint8_t request[12];
    buildModbusRequest(0x05, address, value ? 0xFF00 : 0x0000, request);
    sendRequest(request, sizeof(request));
}

void ModBusClient::writeCoil(const HmiReg<bool>& coilReg) {
    writeCoil(coilReg.address, coilReg.value);
}

// Holding Register - read
uint16_t ModBusClient::readHoldingRegister(const uint16_t address) {
    uint8_t request[12];
    uint8_t response[12];
    
    // Build request for function code 0x03 (Read Holding Registers)
    buildModbusRequest(0x03, address, 1, request);
    sendRequest(request, sizeof(request));
    receiveResponse(response, sizeof(response));

    // Extract holding register value from the response
    // A valid response has the register data starting at byte 9 and 10
    return static_cast<uint16_t>((response[9] << 8) | response[10]);
}

uint16_t ModBusClient::readHoldingRegister(const HmiReg<uint16_t>& reg) {
    return readHoldingRegister(reg.address);
}

// Holding Register - write
void ModBusClient::writeHoldingRegister(uint16_t address, uint16_t value) {
    uint8_t request[12];
    buildModbusRequest(0x06, address, value, request);
    sendRequest(request, sizeof(request));
}

void ModBusClient::writeHoldingRegister(const HmiReg<uint16_t>& reg) {
    writeHoldingRegister(reg.address, reg.value);
}

// Generalized write

void ModBusClient::write(const HmiReg<bool>& reg) {
    writeCoil(reg);
}
void ModBusClient::write(const HmiReg<uint16_t>& reg) {
    write(reg);
}

// Request builder
void ModBusClient::buildModbusRequest(uint8_t functionCode, uint16_t address, uint16_t value, uint8_t* request) {
    // Transaction ID, Protocol ID, Length, Unit ID
    const uint16_t transactionID = 1; // Example Transaction ID
    const uint16_t protocolID = 0;   // Modbus Protocol ID
    const uint16_t length = 6;       // Message Length
    const uint8_t unitID = 1;        // Example Unit ID

    const uint16_t netTransactionID = htons(transactionID);
    const uint16_t netProtocolID = htons(protocolID);
    const uint16_t netLength = htons(length);
    const uint16_t netAddress = htons(address);
    const uint16_t netValue = htons(value);

    memcpy(request, &netTransactionID, 2);
    memcpy(request + 2, &netProtocolID, 2);
    memcpy(request + 4, &netLength, 2);
    request[6] = unitID;
    request[7] = functionCode;
    memcpy(request + 8, &netAddress, 2);
    memcpy(request + 10, &netValue, 2);
}

// Send a Modbus request packet
void ModBusClient::sendRequest(const uint8_t* request, size_t length) const {
    if (!client.Connected()) {
#ifdef USB_PRINT
        USB_PRINT("client not connected");
#endif
        return;
    }
    client.Send(request, length);
    client.Flush();
}

// Receive a Modbus response packet
void ModBusClient::receiveResponse(uint8_t* response, const size_t maxLength) const {
    if (!client.Connected()) {
#ifdef USB_PRINT
        USB_PRINT("client not connected");
#endif
        return;
    }

    const size_t bytesRead = client.Read(response, maxLength);
    if (bytesRead == 0) {
        throw std::runtime_error("No response received from server");
    }
}
