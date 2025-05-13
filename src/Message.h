//
// Created by paulw on 5/12/2025.
//

#ifndef MESSAGE_H
#define MESSAGE_H
#include <cstdint>


namespace Message {

    enum class MessageId {
        #define DEFINE_MESSAGE(Name, Text) Name,
        #include "message_definitions.inc"   // Include the list of messages
        #undef DEFINE_MESSAGE
        COUNT // Special entry to get the count
    };

    const char* GetMessageText(MessageId id);




    /**
     * A message to display either on an HMI or LCD screen
     * an incoming message MUST be null terminated, but when read, it might not be.
     * Be careful when printing
     */
    class MessageClass {
    public:
        static constexpr uint16_t message_u16_len_max = 32;
        static constexpr uint16_t message_char_len_max = message_u16_len_max * 2;

        void set_message(const char* message);
        void set_message(const char* message, uint16_t ms);
        void set_message(MessageId message);
        void set_message(MessageId message, uint16_t ms);
        void set_time(uint16_t ms);
        uint16_t get_time() const;
        uint16_t* get_message_u16();
        char* get_message_char();
        void tick() volatile;
        void unlatch_updated();
        bool has_been_updated() const;


    private:
        union {
            char message[message_u16_len_max * 2] = {'\0'};
            uint16_t message_as_uint16[message_u16_len_max];
        };
        volatile uint16_t ms_remaining = 0;
        bool is_updated = true;


    };
}





#endif //MESSAGE_H
