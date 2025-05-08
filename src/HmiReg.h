// HmiReg.h
// Created by paulw on 5/8/2025.

#ifndef HMIREG_H
#define HMIREG_H

#include <stdint.h>

template <typename T>
class HmiReg {
public:
    // Constructor to initialize the register address and value
    HmiReg(const uint16_t regAddress, T val) : value(val) {
        address = regAddress;  // Initializes the shared register address
    }

    // Getter and setter for value
    T getValue() const { return value; }
    void setValue(T val) { value = val; }

    // Getter for address
    static uint16_t getAddress() { return address; }

private:
    static uint16_t address; // Static register address specific to the template type
    T value;                 // Value associated with the register
};

template <typename T>
uint16_t HmiReg<T>::address = 0; // Initialize static member

#endif // HMIREG_H