// HmiReg.h
// Created by paulw on 5/8/2025.

#ifndef HMIREG_H
#define HMIREG_H

#include <stdint.h>

template <typename T>
class HmiReg {
public:
    HmiReg(const uint16_t regAddress, T val) : value(val) {
        address = regAddress;  // Initializes the shared register address
    }

    uint16_t address;
    T value;
};


#endif // HMIREG_H