//
// Created by paulw on 5/13/2025.
//

#include "helpers.h"
#include "Arduino.h"

double steps_to_f64_inch(const uint32_t steps) {
    const double motor_revs = static_cast<double>(steps)/STEPS_PER_REV;
    const double pinion_revs = motor_revs / GEARBOX_RATIO;
    const double teeth = pinion_revs * PINION_TEETH_PER_REV;
    const double inches = teeth / RACK_TEETH_PER_INCH;
    return inches;
}

uint16_t steps_to_hundreths(const uint32_t steps) {
    const double hundreths = steps_to_f64_inch(steps) * 100;
    return static_cast<uint16_t>(round(hundreths));
}

uint16_t steps_per_sec_to_inches_per_minute(const uint32_t steps_per_second) {
    const double inches_per_second = steps_to_f64_inch(steps_per_second);
    const double inches_per_minute = inches_per_second * 60;

    return static_cast<uint16_t>(round(inches_per_minute));
}

uint16_t steps_per_sec_to_hundreths_per_minute(const uint32_t steps_per_second) {
    const double inches_per_second = steps_to_f64_inch(steps_per_second);
    const double inches_per_minute = inches_per_second * 60;
    const double hundreths_per_minute = inches_per_minute * 100;

    return static_cast<uint16_t>(round(hundreths_per_minute));
}

uint32_t f64_inch_to_steps(const double inches) {
    const double teeth = inches * RACK_TEETH_PER_INCH;
    const double pinion_revs = teeth / PINION_TEETH_PER_REV;
    const double motor_revs = pinion_revs * GEARBOX_RATIO;
    const double steps = motor_revs * STEPS_PER_REV;
    return static_cast<uint32_t>(round(steps));
}

uint32_t hundreths_to_steps(const uint16_t hundreths) {
    const double inches = (static_cast<double>(hundreths)) / 100.0;
    return f64_inch_to_steps(inches);
}

uint32_t inches_per_minute_to_steps_per_sec(const uint16_t inches_per_minute) {
    const double inches_per_second = static_cast<double>(inches_per_minute) / 60.0;
    return f64_inch_to_steps(inches_per_second);
}

uint32_t hundreths_per_minute_to_steps_per_sec(const uint16_t hundreths_per_minute) {
    const double inches_per_minute = static_cast<double>(hundreths_per_minute) / 100.0;
    const double inches_per_second = inches_per_minute / 60.0;
    return f64_inch_to_steps(inches_per_second);
}

/**
 * Convert bytes to u32. Big endian, little endian?
 * I don't know, but it works with `u32_to_bytes()`
 *
 * @warning `bytes[offset+3]` needs to be a valid address
 *
 * @param bytes Const; The bytes to turn into a u32
 * @param offset Offset for accessing array
 * @return the u32
 */
uint32_t bytes_to_u32(const uint8_t *bytes, const uint32_t offset) {
    return
        bytes[offset] << 24 |
        bytes[offset + 1] << 16 |
        bytes[offset + 2] << 8 |
        bytes[offset + 3];
}

/**
 * Convert uint32_t into bytes. Big endian, little endian?
 * I don't know, but it works with `bytes_to_u32()`
 *
 * @warning `bytes[offset+3]` needs to be a valid address
 *
 * @param value The u32 to convert
 * @param bytes Where that converted u32 should go
 * @param offset Offset for writing to array
 */
void u32_to_bytes(const uint32_t value, uint8_t *bytes, const uint32_t offset) {
    bytes[offset] = (value >> 24) & 0xFF;
    bytes[offset+1] = (value >> 16) & 0xFF;
    bytes[offset+2] = (value >> 8) & 0xFF;
    bytes[offset+3] = value & 0xFF;
}


/**
 * Convert bytes to uint64_t. Big endian, little endian?
 * I don't know, but it works with `u64_to_bytes()`
 *
 * @warning `bytes[offset+7]` needs to be a valid address
 *
 * @param bytes Const; The bytes to turn into a uint64_t
 * @param offset Offset for accessing array
 * @return the uint64_t
 */
uint64_t bytes_to_u64(const uint8_t *bytes, const uint64_t offset) {
    return
        static_cast<uint64_t>(bytes[offset + 0]) << 56 |
        static_cast<uint64_t>(bytes[offset + 1]) << 48 |
        static_cast<uint64_t>(bytes[offset + 2]) << 40 |
        static_cast<uint64_t>(bytes[offset + 3]) << 32 |
        static_cast<uint64_t>(bytes[offset + 4]) << 24 |
        static_cast<uint64_t>(bytes[offset + 5]) << 16 |
        static_cast<uint64_t>(bytes[offset + 6]) << 8  |
        static_cast<uint64_t>(bytes[offset + 7]);
}

/**
 * Convert uint64_t into bytes. Big endian, little endian?
 * I don't know, but it works with `bytes_to_u64()`
 *
 * @warning `bytes[offset+7]` needs to be a valid address
 *
 * @param value The uint64_t to convert
 * @param bytes Where that converted uint64_t should go
 * @param offset Offset for writing to array
 */
void u64_to_bytes(const uint64_t value, uint8_t *bytes, const uint64_t offset) {
    bytes[offset + 0] = (value >> 56) & 0xFF;
    bytes[offset + 1] = (value >> 48) & 0xFF;
    bytes[offset + 2] = (value >> 40) & 0xFF;
    bytes[offset + 3] = (value >> 32) & 0xFF;
    bytes[offset + 4] = (value >> 24) & 0xFF;
    bytes[offset + 5] = (value >> 16) & 0xFF;
    bytes[offset + 6] = (value >> 8 ) & 0xFF;
    bytes[offset + 7] = value & 0xFF;
}