#ifndef QUATERNION_H
#define QUATERNION_H

#include <math.h>
#include "Arduino.h"

class Quaternion
{
   public:
    float w = 1.0f;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    Quaternion() = default;
    Quaternion(const float w, const float x, const float y, const float z)
        : w(w), x(x), y(y), z(z)
    {
    }
    Quaternion(const float w[3], const float dt);
    Quaternion(const float euler_angle[3]);
    Quaternion(const float rotation_matrix[3][3]);

    float norm(void);
    float invNorm(void);
    void normalize(void);
    void set(float w, float x, float y, float z);

    Quaternion derivative(const float w[3]) const
    {
        return Quaternion(
            0.5 * (w[0] * -this->x + w[1] * -this->y + w[2] * -this->z),
            0.5 * (w[0] * this->w + w[1] * -this->z + w[2] * this->y),
            0.5 * (w[0] * this->z + w[1] * this->w + w[2] * -this->x),
            0.5 * (w[0] * -this->y + w[1] * this->x + w[2] * this->w));
    }

    Quaternion &operator+=(const Quaternion &delta)
    {
        w += delta.w;
        x += delta.x;
        y += delta.y;
        z += delta.z;
        return *this;
    }

    Quaternion inverse(void) const
    {
        return Quaternion(this->w, -this->x, -this->y, -this->z);
    }

    // TODO migrate this to operator * to increase efficiency
    Quaternion &operator*=(const Quaternion &delta)
    {
        float x, y, z, w;

        x = this->x * delta.w + this->y * delta.z - this->z * delta.y +
            this->w * delta.x;
        y = -this->x * delta.z + this->y * delta.w + this->z * delta.x +
            this->w * delta.y;
        z = this->x * delta.y - this->y * delta.x + this->z * delta.w +
            this->w * delta.z;
        w = -this->x * delta.x - this->y * delta.y - this->z * delta.z +
            this->w * delta.w;

        this->x = x;
        this->y = y;
        this->z = z;
        this->w = w;

        return *this;
    }

    Quaternion &operator*=(const float scalar)
    {
        this->w *= scalar;
        this->x *= scalar;
        this->y *= scalar;
        this->z *= scalar;
        return *this;
    }

    Quaternion &operator/=(const float scalar)
    {
        this->w /= scalar;
        this->x /= scalar;
        this->y /= scalar;
        this->z /= scalar;
        return *this;
    }

    void toEulerAngle( float *roll,  float *pitch,
                       float *yaw) const;
};

Quaternion operator+(const Quaternion &lhs, const Quaternion &rhs);
Quaternion operator*(const Quaternion &lhs, const Quaternion &rhs);
Quaternion operator*(const float scalar, const Quaternion &rhs);
Quaternion operator*(const Quaternion &lhs, const float scalar);
Quaternion operator/(const Quaternion &lhs, const float scalar);

#endif