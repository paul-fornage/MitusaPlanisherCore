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
    const uint16_t index = static_cast<uint16_t>(id);
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
inline void MessageClass::set_message(const Message::MessageId message) {
    set_message(GetMessageText(message));
}
inline void MessageClass::set_message(Message::MessageId message, uint16_t ms) {
    set_message(message);
    this->ms_remaining = ms;
}
inline void MessageClass::set_time(uint16_t ms);
inline uint16_t* MessageClass::get_message_u16();
inline char* MessageClass::get_message_char();
void MessageClass::tick() volatile;