#include "MiTcpMessage.h"
#include "ClearCore.h"

#define debugMessage(message) ConnectorUsb.Send("MiTcp Handler message: "); ConnectorUsb.SendLine(message)

// Maps 2-character headers to MessageHeader values
MessageHeader decodeHeader(const String& header) {
    if (header == "Rr") return MessageHeader::ReadRequest;
    if (header == "Wr") return MessageHeader::WriteRequest;
    if (header == "Rs") return MessageHeader::ReadSuccess;
    if (header == "Ws") return MessageHeader::WriteSuccess;
    if (header == "Rf") return MessageHeader::ReadFail;
    if (header == "Wf") return MessageHeader::WriteFail;
    if (header == "Xr") return MessageHeader::ExecuteRequest;
    if (header == "Xs") return MessageHeader::ExecuteSuccess;
    if (header == "Xf") return MessageHeader::ExecuteFail;
    if (header == "Bc") return MessageHeader::Broadcast;
    return MessageHeader::Invalid; // Default fallback for invalid headers
}

std::pair<StaticVector<uint8_t, MITCP_MESSAGE_BUFFER_SIZE>, bool> MiTcpMessage::encodeMessage(const MiTcpMessage& message) {
    StaticVector<uint8_t, MITCP_MESSAGE_BUFFER_SIZE> bytes;

    // Step 1: Add the header (2 bytes)
    String header = "";
    switch (message.header) {
        case MessageHeader::ReadRequest: header = "Rr"; break;
        case MessageHeader::WriteRequest: header = "Wr"; break;
        case MessageHeader::ReadSuccess: header = "Rs"; break;
        case MessageHeader::WriteSuccess: header = "Ws"; break;
        case MessageHeader::ReadFail: header = "Rf"; break;
        case MessageHeader::WriteFail: header = "Wf"; break;
        case MessageHeader::ExecuteRequest: header = "Xr"; break;
        case MessageHeader::ExecuteSuccess: header = "Xs"; break;
        case MessageHeader::ExecuteFail: header = "Xf"; break;
        case MessageHeader::Broadcast: header = "Bc"; break;
        default:
            debugMessage("Invalid message header");
            return std::make_pair(bytes, false);
    }
    bytes.push_back(header[0]);
    bytes.push_back(header[1]);

    // Step 2: Add a placeholder for length (we'll calculate this later)
    bytes.push_back(0); // Single byte length placeholder

    // Step 3: Add target name
    for (size_t i = 0; i < message.target.length(); i++) {
        char c = message.target[i];
        if (!bytes.push_back(static_cast<uint8_t>(c))) {
            debugMessage("Target string too long");
            return std::make_pair(bytes, false);
        }
    }

    // Step 4: Add newline if there is payload data
    if (!message.data.empty()) {
        if (!bytes.push_back('\n')) {
            debugMessage("Failed to add newline before payload");
            return std::make_pair(bytes, false);
        }
        for (size_t i = 0; i < message.data.size(); i++) {
            if (!bytes.push_back(message.data[i])) {
                debugMessage("Payload too large");
                return std::make_pair(bytes, false);
            }
        }
    }

    // Step 5: Null terminator
    if (!bytes.push_back(0)) {
        debugMessage("Failed to add null terminator");
        return std::make_pair(bytes, false);
    }

    // Step 6: Calculate and update the total length
    uint8_t totalLength = static_cast<uint8_t>(bytes.size());
    if (totalLength > 255) { // Since only 1 byte is supported now, max is 255
        debugMessage("Message exceeds maximum supported length");
        return std::make_pair(bytes, false);
    }
    bytes[2] = totalLength; // Update the length placeholder

    return std::make_pair(bytes, true);
}

std::pair<MiTcpMessage, bool> MiTcpMessage::decodeMessage(const StaticVector<uint8_t, MITCP_MESSAGE_BUFFER_SIZE>& bytes) {
    MiTcpMessage message(MessageHeader::Invalid, String(), StaticVector<uint8_t, MITCP_PAYLOAD_BUFFER_SIZE>());

    if (bytes.size() < 5) {
        debugMessage("Message too short");
        return std::make_pair(message, false);
    }

    // Decode header
    const String headerText = String(static_cast<char>(bytes[0])) + static_cast<char>(bytes[1]);
    message.header = decodeHeader(headerText);
    if (message.header == MessageHeader::Invalid) {
        debugMessage("Invalid header: ");
        debugMessage(headerText.c_str());
        return std::make_pair(message, false);
    }

    // Decode length
    const uint8_t length = bytes[2]; // Extract single-byte length
    if (length != bytes.size()) {
        debugMessage("Message length mismatch");
        return std::make_pair(message, false);
    }

    // Find target name and payload
    size_t payloadStartIndex = 0;
    bool hasPayload = false;
    for (size_t i = 4; i < length - 1; i++) {
        if (bytes[i] == '\n') {
            payloadStartIndex = i + 1;
            hasPayload = true;
            break;
        }
    }

    // Extract target name
    const size_t targetEndIndex = hasPayload ? payloadStartIndex - 1 : length - 1;
    for (size_t i = 3; i < targetEndIndex; i++) {
        message.target += static_cast<char>(bytes[i]);
    }

    // Extract the payload if it exists
    if (hasPayload) {
        for (size_t i = payloadStartIndex; i < length - 1; i++) {
            if (!message.data.push_back(bytes[i])) {
                debugMessage("Payload too large during decoding");
                return std::make_pair(message, false);
            }
        }
    }

    return std::make_pair(message, true);
}

String MiTcpMessage::to_string() const {
    String oss;
    oss += "MiTcpMessage: {\n";
    oss += "  Header: ";
    switch (header) {
        case MessageHeader::ReadRequest: oss += "ReadRequest"; break;
        case MessageHeader::WriteRequest: oss += "WriteRequest"; break;
        case MessageHeader::ReadSuccess: oss += "ReadSuccess"; break;
        case MessageHeader::WriteSuccess: oss += "WriteSuccess"; break;
        case MessageHeader::ReadFail: oss += "ReadFail"; break;
        case MessageHeader::WriteFail: oss += "WriteFail"; break;
        case MessageHeader::ExecuteRequest: oss += "ExecuteRequest"; break;
        case MessageHeader::ExecuteSuccess: oss += "ExecuteSuccess"; break;
        case MessageHeader::ExecuteFail: oss += "ExecuteFail"; break;
        case MessageHeader::Broadcast: oss += "Broadcast"; break;
        default: oss += "Invalid"; break;
    }
    oss += "\n  Target: " + target + "\n  Data: [";

    for (size_t i = 0; i < data.size(); i++) {
        char buffer[5];
        sprintf(buffer, "0x%02X", data[i]);
        oss += buffer;
        if (i < data.size() - 1) {
            oss += ", ";
        }
    }
    oss += "]\n}";
    return oss;
}