// HmiReg.h
// Created by paulw on 5/8/2025.

#ifndef HMIREG_H
#define HMIREG_H

#include <cstdint>

template <typename T>
class HmiReg {
public:
    HmiReg(const uint16_t regAddress, T val)
        : address(regAddress)
        , value(val)
    {}

    const uint16_t address;
    T value;
};


#endif // HMIREG_H