//
// Created by paulw on 5/12/2025.
//

#include "mbButton.h"

MbButton::MbButton(const uint16_t address)
    : last_state(false),
        current_state(false),
        address(address) {
}

MbButton::MbButton(const uint16_t address, const bool current_state, const bool last_state)
    : last_state(last_state),
        current_state(current_state),
        address(address) {
}
bool MbButton::is_rising() const {
    return current_state && !last_state;
}
bool MbButton::is_falling() const {
    return !current_state && last_state;
}
bool MbButton::is_changing() const {
    return current_state ^ last_state;
}
bool MbButton::get_current_state() const {
    return current_state;
}
void MbButton::new_reading(const bool new_state) {
    last_state = current_state;
    current_state = new_state;
}
void MbButton::shift_input() {
    last_state = current_state;
}
uint16_t MbButton::get_address() const {
    return address;
}
