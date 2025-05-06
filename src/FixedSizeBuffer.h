//
// Created by paulw on 5/6/2025.
//

#ifndef FIXEDSIZEBUFFER_H
#define FIXEDSIZEBUFFER_H

#include <cstdint>
#include <utility>

class FixedSizeBuffer {
private:
    static constexpr std::size_t BufferSize = 2048; // Fixed size
    char data[BufferSize];                  // Array of int16_t
    std::size_t currentIndex;                  // Tracks the current value index

public:
    // Constructor to initialize the buffer
    FixedSizeBuffer() : currentIndex(0) {}

    /**
     * Append an element
     * @param value the value to push
     * @return false for buffer overflow (not executed)
     */
    bool push(const char value) {
        if (currentIndex >= BufferSize) {return false;}
        data[currentIndex++] = value;
        return true;
    }

    /**
     * Clearcore uses old C++, so no modern Rust-inspired features like result or option
     * @return a pair, first value is the value retrieved, which is only good if the bool is true
     */
    std::pair<char, bool> get(const std::size_t index) const {
        if (index >= currentIndex) {
            return {0, false};
        }
        return {data[index], true};
    }

    /**
     * promise to stay in range
     * @return the value
     */
    char get_unchecked(const std::size_t index) const {
        return data[index];
    }

    // Reset the buffer to its initial empty state
    void reset() {
        currentIndex = 0;
    }

    // Get the current size (number of elements)
    std::size_t size() const {
        return currentIndex;
    }

    // Check if the buffer is empty
    bool isEmpty() const {
        return currentIndex == 0;
    }

    // Check if the buffer is full
    bool isFull() const {
        return currentIndex >= BufferSize;
    }

    // Get the maximum capacity
    static std::size_t capacity() {
        return BufferSize;
    }

    /**
     * Gets the raw data pointer. Be careful
     */
    char* get_data() {
        return data;
    }

    /**
     * Add a null to the end, and then return a pointer.
     * Used for converting to string.
     * Only works as intended if the buffer didn't get a null inserted before the end.
     *
     * @pre Buffer does not contain a null;
     * @pre Buffer must have room for at least one more element;
     * @return Pointer to the beginning of the buffer, with a null inserted after the last element
     */
    char* get_data_null_term() {
        this->push('\0');
        return data;
    }



};

#endif // FIXEDSIZEBUFFER_H