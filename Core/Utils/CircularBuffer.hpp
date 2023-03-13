//----------------------------------------------------------------------------------------------------
// File: CircularBuffer.hpp
// Desc: Utility class for a basic circular buffer implementation
//----------------------------------------------------------------------------------------------------
#pragma once

// Standard libraries includes
#include <array>

namespace gtfo {

template<typename T, size_t N>
class CircularBuffer : private std::array<T, N>{
    using Base = std::array<T, N>;
public:
    CircularBuffer() : Base{}, head_{0} {}
    CircularBuffer(const Base& data) : Base(data), head_{0} {}

    T& operator[](const size_t& n) {
        return Base::operator[]((head_ + n) % N);
    }

    void push_front(const T& datum){
        head_ = (head_ + N - 1) % N;
        Base::operator[](head_) = datum;
    }

    void push_back(const T& datum){
        Base::operator[](head_) = datum;
        head_ = (head_ + 1) % N;
    }
private:
    size_t head_;
};

}   // namespace gtfo