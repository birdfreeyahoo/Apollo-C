#include "sint15.h"

sint15::sint15()
{
    value = 0;
}

sint15::sint15(int i)
{
    value = i;

    // Convert from signed 16-bit to signed 15-bit, by moving the sign bit
    value &= ~0x4000;
    value |= (value & 0x8000) >> 1;
}

sint15::sint15(const sint15& i)
{
    value = i.value;
}

sint15& sint15::operator=(const sint15& i)
{
    value = i.value;

    return *this;
}

sint15& sint15::operator=(int i)
{
    value = i;

    // Convert from signed 16-bit to signed 15-bit, by moving the sign bit
    value &= ~0x4000;
    value |= (value & 0x8000) >> 1;

    return *this;
}

sint15& sint15::operator+=(const sint15& i)
{
    *this = *this + i;
    return *this;
}

sint15& sint15::operator+=(int i)
{
    *this = *this + sint15(i);
    return *this;
}

sint15& sint15::operator-=(const sint15& i)
{
    *this = *this - i;
    return *this;
}

sint15& sint15::operator-=(int i)
{
    *this = *this - sint15(i);
    return *this;
}

sint15& sint15::operator*=(const sint15& i)
{
    *this = *this * i;
    return *this;
}

sint15& sint15::operator*=(int i)
{
    *this = *this * sint15(i);
    return *this;
}

sint15& sint15::operator/=(const sint15& i)
{
    *this = *this / i;
    return *this;
}

sint15& sint15::operator/=(int i)
{
    *this = *this / sint15(i);
    return *this;
}

sint15& sint15::operator%=(const sint15& i)
{
    *this = *this % i;
    return *this;
}

sint15& sint15::operator%=(int i)
{
    *this = *this % sint15(i);
    return *this;
}

sint15& sint15::operator++()
{
    *this = *this + 1;
    return *this;
}

sint15& sint15::operator--()
{
    *this = *this - 1;
    return *this;
}

sint15 sint15::operator++(int)
{
    sint15 temp = *this;
    *this = *this + 1;
    return temp;
}

sint15 sint15::operator--(int)
{
    sint15 temp = *this;
    *this = *this - 1;
    return temp;
}

sint15 sint15::operator-() const
{
    return sint15(-value);
}

sint15 sint15::operator+() const
{
    return sint15(+value);
}

sint15 sint15::operator~() const
{
    return sint15(~value);
}

sint15 sint15::operator&(const sint15& i) const
{
    return sint15(value & i.value);
    (unsigned short)(-0x4000)
}

#include <iostream>
#include <cstdint>

#define NEGATIVE_ZERO 0x7FFF

class OnesComplementInt15 {
private:
    uint16_t value;

    uint16_t ones_complement(uint16_t x) {
        return ~x & 0x7FFF;
    }

    // Private constructor to create -0 directly
    OnesComplementInt15(int16_t x, bool negative_zero) : value(NEGATIVE_ZERO) {}

public:
    OnesComplementInt15() : value(0) {}

    OnesComplementInt15(int16_t x) {
        if (x < 0) {
            value = ones_complement(-x);
        } else {
            value = x;
        }
    }

    int16_t to_int16() const {
        if (value == NEGATIVE_ZERO) {
            return 0;
        }
        return (value & 0x4000) ? -ones_complement(value) : value;
    }

    OnesComplementInt15 operator+(const OnesComplementInt15& other) const {
        uint32_t temp = static_cast<uint32_t>(value) + other.value;
        temp = (temp & 0x7FFF) + (temp >> 15);
        return OnesComplementInt15(static_cast<int16_t>(temp));
    }

    OnesComplementInt15 operator-() const {
        if (value == 0) {
            return OnesComplementInt15(0, true); // Construct -0 directly
        }
        return OnesComplementInt15(-to_int16());
    }

    OnesComplementInt15 operator-(const OnesComplementInt15& other) const {
        return *this + (-other);
    }

    // Add other necessary operators and methods as needed
};

int main() {
    OnesComplementInt15 a(0);
    OnesComplementInt15 b = -a;
    std::cout << "0 as int16: " << a.to_int16() << ", -0 as int16: " << b.to_int16() << std::endl;
    std::cout << "0 in one's complement: " << a.value << ", -0 in one's complement: " << b.value << std::endl;

    return 0;
}