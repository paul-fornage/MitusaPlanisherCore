//
// Created by paulw on 5/6/2025.
//

#ifndef MITCPMESSAGE_H
#define MITCPMESSAGE_H


#define MITCP_MESSAGE_BUFFER_SIZE 256
#define MITCP_PAYLOAD_BUFFER_SIZE 250

#include <StaticVector.h>
#include <ArduinoString.h>
#include <utility>

enum class MessageHeader {
    ReadRequest,
    WriteRequest,
    ReadSuccess,
    WriteSuccess,
    ReadFail,
    WriteFail,
    ExecuteRequest,
    ExecuteSuccess,
    ExecuteFail,
    Broadcast,
    Invalid
};

class MiTcpMessage {
public:
    MessageHeader header;
    String target;
    StaticVector<uint8_t, MITCP_PAYLOAD_BUFFER_SIZE> data; // Adjust capacity as needed for application

    MiTcpMessage(const MessageHeader header, const String& target, const StaticVector<uint8_t, MITCP_PAYLOAD_BUFFER_SIZE>& data = {})
        : header(header), target(target), data(data) {}

    /**
     * Decode a MiTcp message
     * @param bytes
     * @return a pair containing the message and if it succeeded
     */
    static std::pair<MiTcpMessage, bool> decodeMessage(const StaticVector<uint8_t, MITCP_MESSAGE_BUFFER_SIZE>& bytes);

    /**
     * Encode a MiTcpMessage into a vector of bytes.
     * @param message the MiTcp message to be encoded
     * @return a vector of bytes representing the encoded message, and if the operation succeeded
     */
    static std::pair<StaticVector<uint8_t, MITCP_MESSAGE_BUFFER_SIZE>, bool> encodeMessage(const MiTcpMessage& message);

    String to_string() const;
};

#endif // MITCPMESSAGE_H
