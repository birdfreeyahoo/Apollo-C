#ifndef SINT15_H
#define SINT15_H

#include "defines.h"

class sint15
{
public:
    sint15();
    sint15(int);
    sint15(const sint15&);
    sint15& operator=(const sint15&);
    sint15& operator=(int);
    sint15& operator+=(const sint15&);
    sint15& operator+=(int);
    sint15& operator-=(const sint15&);
    sint15& operator-=(int);
    sint15& operator*=(const sint15&);
    sint15& operator*=(int);
    sint15& operator/=(const sint15&);
    sint15& operator/=(int);
    sint15& operator%=(const sint15&);
    sint15& operator%=(int);
    sint15& operator++();
    sint15 operator++(int);
    sint15& operator--();
    sint15 operator--(int);
    sint15 operator-() const;
    sint15 operator+() const;
    sint15 operator~() const;
    sint15 operator&(const sint15&) const;
    sint15 operator&(int) const;
    sint15 operator|(const sint15&) const;
    sint15 operator|(int) const;
    sint15 operator^(const sint15&) const;
    sint15 operator^(int) const;
    sint15 operator<<(const sint15&) const;
    sint15 operator<<(int) const;
    sint15 operator>>(const sint15&) const;
    sint15 operator>>(int) const;
    sint15 operator&&(const sint15&) const;
    sint15 operator&&(int) const;
    sint15 operator||(const sint15&) const;
    sint15 operator||(int) const;
    sint15 operator!() const;
    sint15 operator==(const sint15&) const;
    sint15 operator==(int) const;
    sint15 operator!=(const sint15&) const;
    sint15 operator!=(int) const;
    sint15 operator<(const sint15&) const;
    sint15 operator<(int) const;
    sint15 operator>(const sint15&) const;
    sint15 operator>(int) const;
    sint15 operator<=(const sint15&) const;
    sint15 operator<=(int) const;
    sint15 operator>=(const sint15&) const;
    sint15 operator>=(int) const;
    sint15 operator,(const sint15&) const;
    sint15 operator,(int) const;
    sint15 operator[](const sint15&) const;
    sint15 operator[](int) const;

    private:
    sint16 value;
};

#endif