//
// Created by paulw on 5/12/2025.
//

#include "Message.h"

#include <cstring>

static const char* MessageTexts[] = {
#define DEFINE_MESSAGE(Name, Text) Text,
#include "message_definitions.inc"   // Include the list of messages
#undef DEFINE_MESSAGE
};

const char* GetMessageText(Message::MessageId id) {
    const auto index = static_cast<uint16_t>(id);
    if (index < static_cast<uint16_t>(Message::MessageId::COUNT)) {
        return MessageTexts[index];
    }
    return "Unknown Message";
}

using namespace Message;

inline void MessageClass::set_message(const char* message) {
    strncpy(this->message, message, message_char_len_max);
}

inline void MessageClass::set_message(const char* message, const uint16_t ms) {
    set_message(message);
    this->ms_remaining = ms;
}
inline void MessageClass::set_message(const MessageId message) {
    set_message(GetMessageText(message));
}
inline void MessageClass::set_message(const MessageId message, const uint16_t ms) {
    set_message(GetMessageText(message));
    this->ms_remaining = ms;
}
inline void MessageClass::set_time(const uint16_t ms) {
    this->ms_remaining = ms;
}
inline uint16_t MessageClass::get_time() const {
    return this->ms_remaining;
}
inline uint16_t* MessageClass::get_message_u16() {
    return this->message_as_uint16;
}
inline char* MessageClass::get_message_char() {
    return this->message;
}
void MessageClass::tick() volatile {
    if (this->ms_remaining > 0) {
        this->ms_remaining--;
    }
}