#ifndef DEF_H
#define DEF_H

#include <Arduino.h>

class Vec{
public:
    float x;
    float y;
    float z;

    Vec()
    {
        x = 0;
        y = 0;
        z = 0;
    }

    Vec operator * (float m)
    {
        Vec v = *this;
        v.x *= m;
        v.y *= m;
        v.z *= m;
        return v;
    }

    Vec& operator *= (float m)
    {
        *this = *this * m;
        return *this;
    }

    Vec& operator += (Vec &v)
    {
        x += v.x;
        y += v.y;
        z += v.z;
        return *this;
    }
    
    float len()
    {
        return sqrt(x*x+y*y+z*z);
    }
};

class Measure {
public:
    Vec accel;
    Vec gyro;
    Vec mag;
    float temperature;
    float pressure;

    Measure()
    {
        temperature = 0;
        pressure = 0;
    }

    Measure operator+= (Measure &m)
    {
        accel += m.accel;
        gyro += m.gyro;
        mag += m.mag;
        temperature += m.temperature;
        pressure += m.pressure;
        return *this;
    }

    Measure operator*= (float m)
    {
        accel *= m;
        gyro *= m;
        mag *= m;
        temperature *= m;
        pressure *= m;
        return *this;
    }
};

#endif
