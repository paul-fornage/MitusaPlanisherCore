#ifndef STATIC_VECTOR_H
#define STATIC_VECTOR_H

#include <cstdint>
#include <cstddef>  // size_t

template <typename T, size_t MaxSize>
class StaticVector {
public:
    // Constructor
    StaticVector() : m_size(0) {}

    // Get current size of the vector
    size_t size() const {
        return m_size;
    }

    // Get the maximum capacity of the vector
    constexpr size_t capacity() const {
        return MaxSize;
    }

    // Check if the vector is empty
    bool empty() const {
        return m_size == 0;
    }

    // Access elements (indexing)
    T& operator[](size_t index) {
        return m_data[index];
    }

    const T& operator[](size_t index) const {
        return m_data[index];
    }

    // Add an element to the end of the vector
    bool push_back(const T& value) {
        if (m_size < MaxSize) {
            m_data[m_size++] = value;
            return true;  // Indicates success
        }
        return false;  // Indicates failure due to exceeding size
    }

    // Remove the last element
    bool pop_back() {
        if (m_size > 0) {
            --m_size;
            return true;
        }
        return false;  // Cannot pop because vector is empty
    }

    // Access the last element
    T& back() {
        return m_data[m_size - 1];
    }

    const T& back() const {
        return m_data[m_size - 1];
    }

    // Clear the vector
    void clear() {
        m_size = 0;
    }

    const uint8_t* data() const {
        return reinterpret_cast<const uint8_t*>(m_data);
    }


private:
    T m_data[MaxSize];  // Fixed-size array to store elements
    size_t m_size;      // Number of elements currently in the vector
};

#endif // STATIC_VECTOR_H