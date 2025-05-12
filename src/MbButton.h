//
// Created by paulw on 5/12/2025.
//

#ifndef MBBUTTON_H
#define MBBUTTON_H
#include <cstdint>


class MbButton{
public:
    explicit MbButton(uint16_t address);
    MbButton(uint16_t address, bool current_state, bool last_state);
    bool is_rising() const;
    bool is_falling() const;
    bool is_changing() const;
    bool get_current_state() const;
    void new_reading(bool new_state);
    void shift_input();
    uint16_t get_address() const;
private:
    bool last_state;
    bool current_state;
    const uint16_t address;
};



#endif //MBBUTTON_H
