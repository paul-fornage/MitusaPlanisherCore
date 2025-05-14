//
// Created by paulw on 5/13/2025.
//

#ifndef HELPERS_H
#define HELPERS_H

// TODO: Verify this WAGNER
#define STEPS_PER_REV 800.0
#define GEARBOX_RATIO 100.0 // number of input revs for one output revs
#define RACK_TEETH_PER_INCH 4.0
#define PINION_TEETH_PER_REV 24.0 // teeth on the pinion gear that interfaces with the rack

#include <cstdint>

double steps_to_f64_inch(uint32_t steps);

uint16_t steps_to_hundreths(uint32_t steps);

uint16_t steps_per_sec_to_inches_per_minute(uint32_t steps_per_second);

uint16_t steps_per_sec_to_hundreths_per_minute(uint32_t steps_per_second);

uint32_t f64_inch_to_steps(double inches);

uint32_t hundreths_to_steps(uint16_t hundreths);

uint32_t inches_per_minute_to_steps_per_sec(uint16_t inches_per_minute);

uint32_t hundreths_per_minute_to_steps_per_sec(uint16_t hundreths_per_minute);

void u32_to_bytes(uint32_t value, uint8_t *bytes, uint32_t offset = 0);

uint32_t bytes_to_u32(const uint8_t *bytes, uint32_t offset = 0);

void u64_to_bytes(uint64_t value, uint8_t *bytes, uint32_t offset = 0);

uint64_t bytes_to_u64(const uint8_t *bytes, uint32_t offset = 0);

#endif //HELPERS_H
