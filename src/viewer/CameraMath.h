// SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: LicenseRef-NvidiaProprietary
//
// NVIDIA CORPORATION, its affiliates and licensors retain all intellectual
// property and proprietary rights in and to this material, related
// documentation and any modifications thereto. Any use, reproduction,
// disclosure or distribution of this material and related documentation
// without an express license agreement from NVIDIA CORPORATION or
// its affiliates is strictly prohibited.

#pragma once

#include <math.h>
#include <vector>

#ifdef __CUDACC__
#    define CUDA_CALLABLE __host__ __device__
#    define CUDA_CALLABLE_DEVICE __device__
#else
#    define CUDA_CALLABLE
#    define CUDA_CALLABLE_DEVICE
#endif

#ifndef UNUSED
#    define UNUSED(x) (void)(x)
#endif

namespace viewer
{

// ----------------------------------------------------------------------------
#define Pi 3.141592653589793f
#define TwoPi (2.0f * Pi)
#define HalfPi (0.5f * Pi)
#define MaxFloat 3.402823466e+38f
#define MaxInt32 0x7fffffff

enum _ZERO
{
    Zero
};
enum _IDENTITY
{
    Identity
};
enum _EMPTY
{
    Empty
};

CUDA_CALLABLE inline float Min(float f0, float f1)
{
    return f0 < f1 ? f0 : f1;
}

CUDA_CALLABLE inline float Max(float f0, float f1)
{
    return f0 > f1 ? f0 : f1;
}

CUDA_CALLABLE inline float Min(float f0, float f1, float f2)
{
    return Min(Min(f0, f1), f2);
}

CUDA_CALLABLE inline float Max(float f0, float f1, float f2)
{
    return Max(Max(f0, f1), f2);
}

CUDA_CALLABLE inline int Min(int i0, int i1)
{
    return i0 < i1 ? i0 : i1;
}

CUDA_CALLABLE inline int Max(int f0, int f1)
{
    return f0 > f1 ? f0 : f1;
}

CUDA_CALLABLE inline size_t Min(size_t i0, size_t i1)
{
    return i0 < i1 ? i0 : i1;
}

CUDA_CALLABLE inline size_t Max(size_t f0, size_t f1)
{
    return f0 > f1 ? f0 : f1;
}

CUDA_CALLABLE inline float Abs(float f)
{
    return f >= 0.0f ? f : -f;
}

CUDA_CALLABLE inline int Abs(int i)
{
    return i >= 0 ? i : -i;
}

/**
 * @brief Clamps a floating point value to the range [a, b].
 * @param f The value to clamp.
 * @param a The minimum value.
 * @param b The maximum value.
 * @return The clamped value.
 */
CUDA_CALLABLE inline float Clamp(float f, float a, float b)
{
    return Min(b, Max(a, f));
}

/**
 * @brief Clamps an integer value to the range [a, b].
 * @param f The value to clamp.
 * @param a The minimum value.
 * @param b The maximum value.
 * @return The clamped value.
 */
CUDA_CALLABLE inline int Clamp(int f, int a, int b)
{
    return Min(b, Max(a, f));
}

/**
 * @brief Computes the sine of an angle in radians.
 */
CUDA_CALLABLE inline float Sin(float f)
{
    return sinf(f);
}

/**
 * @brief Computes the cosine of an angle in radians.
 */
CUDA_CALLABLE inline float Cos(float f)
{
    return cosf(f);
}

/**
 * @brief Computes the arc sine of a value safely clamping the input to the range [-1, 1].
 */
CUDA_CALLABLE inline float Asin(float f)
{
    return asinf(Clamp(f, -1.0f, 1.0f));
}

/**
 * @brief Computes the arc cosine of a value safely clamping the input to the range [-1, 1].
 */
CUDA_CALLABLE inline float Acos(float f)
{
    return acosf(Clamp(f, -1.0f, 1.0f));
}

/**
 * @brief Rounds a floating point value to a specified number of digits.
 */
CUDA_CALLABLE inline float Round(float f, int numDigits)
{
    float p = powf(10.0f, (float)numDigits);
    return f > 0.0f ? floorf(p * f + 0.5f) / p : ceilf(p * f - 0.5f) / p;
}

struct Vec3
{
    CUDA_CALLABLE Vec3() : x(0.0f), y(0.0f), z(0.0f)
    {
    }
    CUDA_CALLABLE Vec3(_ZERO) : x(0.0f), y(0.0f), z(0.0f)
    {
    }
    CUDA_CALLABLE Vec3(float x, float y, float z) : x(x), y(y), z(z)
    {
    }
    CUDA_CALLABLE Vec3(float* f)
    {
        x = f[0];
        y = f[1];
        z = f[2];
    }
    CUDA_CALLABLE Vec3(_IDENTITY, int dim)
    {
        x = dim == 0 ? 1.0f : 0.0f;
        y = dim == 1 ? 1.0f : 0.0f;
        z = dim == 2 ? 1.0f : 0.0f;
    }

    CUDA_CALLABLE float& operator[](unsigned int index)
    {
        return reinterpret_cast<float*>(this)[index];
    }

    CUDA_CALLABLE const float& operator[](unsigned int index) const
    {
        return reinterpret_cast<const float*>(this)[index];
    }

    CUDA_CALLABLE bool operator==(const Vec3& v) const
    {
        return x == v.x && y == v.y && z == v.z;
    }
    CUDA_CALLABLE bool operator!=(const Vec3& v) const
    {
        return x != v.x || y != v.y || z != v.z;
    }

    CUDA_CALLABLE bool operator<(const Vec3& v) const
    { // order dependent!
        if (x < v.x)
            return true;
        if (x > v.x)
            return false;
        if (y < v.y)
            return true;
        if (y > v.y)
            return false;
        return z < v.z;
    }

    CUDA_CALLABLE bool isZero() const
    {
        return x == 0.0 && y == 0.0 && z == 0.0;
    }

    CUDA_CALLABLE void set(float newX, float newY, float newZ)
    {
        x = newX;
        y = newY;
        z = newZ;
    }

    CUDA_CALLABLE Vec3 minimum(const Vec3& v) const
    {
        return Vec3(Min(x, v.x), Min(y, v.y), Min(z, v.z));
    }

    CUDA_CALLABLE Vec3 maximum(const Vec3& v) const
    {
        return Vec3(Max(x, v.x), Max(y, v.y), Max(z, v.z));
    }

    CUDA_CALLABLE Vec3 lerp(const Vec3& v, float s) const
    {
        return *this * (1.0f - s) + v * s;
    }

    CUDA_CALLABLE Vec3 operator-() const
    {
        return Vec3(-x, -y, -z);
    }

    CUDA_CALLABLE Vec3 operator+(const Vec3& v) const
    {
        return Vec3(x + v.x, y + v.y, z + v.z);
    }

    CUDA_CALLABLE Vec3 operator-(const Vec3& v) const
    {
        return Vec3(x - v.x, y - v.y, z - v.z);
    }

    CUDA_CALLABLE void operator+=(const Vec3& v)
    {
        x += v.x;
        y += v.y;
        z += v.z;
    }

    CUDA_CALLABLE void operator-=(const Vec3& v)
    {
        x -= v.x;
        y -= v.y;
        z -= v.z;
    }

    CUDA_CALLABLE void operator*=(float f)
    {
        x *= f;
        y *= f;
        z *= f;
    }

    CUDA_CALLABLE void operator/=(float f)
    {
        x /= f;
        y /= f;
        z /= f;
    }

    CUDA_CALLABLE Vec3 operator*(float f) const
    {
        return Vec3(x * f, y * f, z * f);
    }

    CUDA_CALLABLE Vec3 operator/(float f) const
    {
        return Vec3(x / f, y / f, z / f);
    }

    CUDA_CALLABLE Vec3 multiply(const Vec3& a) const
    {
        return Vec3(x * a.x, y * a.y, z * a.z);
    }

    CUDA_CALLABLE Vec3 diagonalMult(const Vec3& v) const
    {
        return Vec3(x * v.x, y * v.y, z * v.z);
    }

    CUDA_CALLABLE void setMin(const Vec3& v)
    {
        x = x < v.x ? x : v.x;
        y = y < v.y ? y : v.y;
        z = z < v.z ? z : v.z;
    }

    CUDA_CALLABLE void setMax(const Vec3& v)
    {
        x = x > v.x ? x : v.x;
        y = y > v.y ? y : v.y;
        z = z > v.z ? z : v.z;
    }

    CUDA_CALLABLE float magnitude() const
    {
        return sqrtf(x * x + y * y + z * z);
    }

    CUDA_CALLABLE float magnitudeSquared() const
    {
        return x * x + y * y + z * z;
    }

    CUDA_CALLABLE float length() const
    {
        return sqrtf(x * x + y * y + z * z);
    }

    CUDA_CALLABLE float lengthSquared() const
    {
        return x * x + y * y + z * z;
    }

    CUDA_CALLABLE float normalize()
    {
        float m = magnitude();
        if (m > 0.0f)
            *this /= m;
        return m;
    }

    CUDA_CALLABLE Vec3 normalized() const
    {
        Vec3 v = *this;
        v.normalize();
        return v;
    }

    CUDA_CALLABLE float dot(const Vec3& v) const
    {
        return x * v.x + y * v.y + z * v.z;
    }

    CUDA_CALLABLE Vec3 cross(const Vec3& v) const
    {
        return Vec3(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x);
    }

    CUDA_CALLABLE Vec3 perp() const
    {
        if (isZero())
            return Vec3(1.0, 0.0, 0.0);
        Vec3 a(0.0, 0.0, 1.0);
        if (fabs(x) < fabs(y) && fabs(x) < fabs(z))
            a = Vec3(1.0, 0.0, 0.0);
        else if (fabs(y) < fabs(z))
            a = Vec3(0.0, 1.0, 0.0);
        Vec3 b = a.cross(*this);
        b.normalize();
        return b;
    }

    float x, y, z;
};

CUDA_CALLABLE inline Vec3 operator*(float f, const Vec3& v)
{
    return Vec3(f * v.x, f * v.y, f * v.z);
}

struct Quat;

/**
 * @brief Defines a 3x3 matrix via its columns of type Vec3.
 */

struct Mat33
{
    CUDA_CALLABLE Mat33() : column0(Zero), column1(Zero), column2(Zero){};
    CUDA_CALLABLE Mat33(const Vec3& column0, const Vec3& column1, const Vec3& column2)
        : column0(column0), column1(column1), column2(column2)
    {
    }

    CUDA_CALLABLE Mat33(const Quat& q);
    CUDA_CALLABLE Mat33(_IDENTITY)
        : column0(Vec3(1.0f, 0.0f, 0.0f)), column1(Vec3(0.0f, 1.0f, 0.0f)), column2(Vec3(0.0f, 0.0f, 1.0f)){};

    CUDA_CALLABLE Mat33(_ZERO) : column0(Vec3(Zero)), column1(Vec3(Zero)), column2(Vec3(Zero)){};

    CUDA_CALLABLE Mat33(const Vec3& v)
        : column0(Vec3(v.x, 0.0f, 0.0f)), column1(Vec3(0.0f, v.y, 0.0f)), column2(Vec3(0.0f, 0.0f, v.z)){};

    CUDA_CALLABLE Mat33(float a00, float a01, float a02, float a10, float a11, float a12, float a20, float a21, float a22)
        : column0(Vec3(a00, a10, a20)), column1(Vec3(a01, a11, a21)), column2(Vec3(a02, a12, a22))
    {
    }

    CUDA_CALLABLE bool isZero() const
    {
        return column0.isZero() && column1.isZero() && column2.isZero();
    }

    CUDA_CALLABLE Mat33 getTranspose() const
    {
        Vec3 v0(column0.x, column1.x, column2.x);
        Vec3 v1(column0.y, column1.y, column2.y);
        Vec3 v2(column0.z, column1.z, column2.z);
        return Mat33(v0, v1, v2);
    }

    CUDA_CALLABLE Vec3 operator*(const Vec3& v) const
    {
        return column0 * v.x + column1 * v.y + column2 * v.z;
    }

    CUDA_CALLABLE const Mat33 operator*(float scalar) const
    {
        return Mat33(column0 * scalar, column1 * scalar, column2 * scalar);
    }

    CUDA_CALLABLE Mat33 operator*(const Mat33& M) const
    {
        return Mat33(*this * M.column0, *this * M.column1, *this * M.column2);
    }

    CUDA_CALLABLE Vec3& operator[](unsigned int num)
    {
        return (&column0)[num];
    }
    CUDA_CALLABLE const Vec3& operator[](unsigned int num) const
    {
        return (&column0)[num];
    }

    CUDA_CALLABLE float operator()(unsigned int row, unsigned int col) const
    {
        return (*this)[col][row];
    }

    CUDA_CALLABLE float& operator()(unsigned int row, unsigned int col)
    {
        return (*this)[col][row];
    }

    CUDA_CALLABLE const Mat33 operator+(const Mat33& M) const
    {
        return Mat33(column0 + M.column0, column1 + M.column1, column2 + M.column2);
    }

    CUDA_CALLABLE const Mat33 operator-(const Mat33& M) const
    {
        return Mat33(column0 - M.column0, column1 - M.column1, column2 - M.column2);
    }

    CUDA_CALLABLE Mat33& operator+=(const Mat33& M)
    {
        column0 += M.column0;
        column1 += M.column1;
        column2 += M.column2;
        return *this;
    }

    CUDA_CALLABLE Mat33& operator-=(const Mat33& M)
    {
        column0 -= M.column0;
        column1 -= M.column1;
        column2 -= M.column2;
        return *this;
    }

    CUDA_CALLABLE void operator*=(float scalar)
    {
        column0 *= scalar;
        column1 *= scalar;
        column2 *= scalar;
    }

    CUDA_CALLABLE void operator/=(float scalar)
    {
        column0 /= scalar;
        column1 /= scalar;
        column2 /= scalar;
    }

    CUDA_CALLABLE const Mat33 operator-() const
    {
        return Mat33(-column0, -column1, -column2);
    }

    CUDA_CALLABLE bool operator==(const Mat33& M) const
    {
        return column0 == M.column0 && column1 == M.column1 && column2 == M.column2;
    }

    CUDA_CALLABLE bool operator<(const Mat33& M) const
    {
        if (column0 != M.column0)
            return column0 < M.column0;
        if (column1 != M.column1)
            return column1 < M.column1;
        return column2 < M.column2;
    }

    CUDA_CALLABLE float getDeterminant() const
    {
        return column0.dot(column1.cross(column2));
    }
    CUDA_CALLABLE const Mat33 getInverse() const
    {
        const float det = getDeterminant();
        Mat33 inverse;

        if (det != 0.0)
        {
            const float invDet = 1.0f / det;

            inverse.column0.x = invDet * (column1.y * column2.z - column2.y * column1.z);
            inverse.column0.y = invDet * -(column0.y * column2.z - column2.y * column0.z);
            inverse.column0.z = invDet * (column0.y * column1.z - column0.z * column1.y);

            inverse.column1.x = invDet * -(column1.x * column2.z - column1.z * column2.x);
            inverse.column1.y = invDet * (column0.x * column2.z - column0.z * column2.x);
            inverse.column1.z = invDet * -(column0.x * column1.z - column0.z * column1.x);

            inverse.column2.x = invDet * (column1.x * column2.y - column1.y * column2.x);
            inverse.column2.y = invDet * -(column0.x * column2.y - column0.y * column2.x);
            inverse.column2.z = invDet * (column0.x * column1.y - column1.x * column0.y);

            return inverse;
        }
        else
            return Mat33(Identity);
    }

    CUDA_CALLABLE float dot(const Mat33& M) const
    {
        return column0.dot(M.column0) + column1.dot(M.column1) + column2.dot(M.column2);
    }

    CUDA_CALLABLE void createFrame(const Vec3& axis)
    {
        column0 = axis;
        column0.normalize();
        column1 = column0.perp();
        column2 = column0.cross(column1);
    }

    Vec3 column0, column1, column2;
};

CUDA_CALLABLE inline Mat33 operator*(float f, const Mat33& M)
{
    return Mat33(f * M.column0, f * M.column1, f * M.column2);
}

/**
 * @brief Defines a quaternion with the float components x, y, z and w, where x, y and z define the imaginary part and w
 * the real part.
 */
struct Quat
{
    CUDA_CALLABLE Quat() : x(0.0f), y(0.0f), z(0.0f), w(0.0f)
    {
    }
    CUDA_CALLABLE Quat(float x, float y, float z, float w) : x(x), y(y), z(z), w(w)
    {
    }
    CUDA_CALLABLE Quat(_IDENTITY)
    {
        x = y = z = 0.0f;
        w = 1.0f;
    }
    CUDA_CALLABLE Quat(_ZERO)
    {
        x = y = z = 0.0f;
        w = 0.0f;
    }
    CUDA_CALLABLE Quat(const Vec3& v, float w) : x(v.x), y(v.y), z(v.z), w(w)
    {
    }
    CUDA_CALLABLE Quat(float angleRadians, const Vec3& unitAxis)
    {
        const float a = angleRadians * 0.5f;
        const float s = sinf(a);
        w = cosf(a);
        x = unitAxis.x * s;
        y = unitAxis.y * s;
        z = unitAxis.z * s;
    }
    CUDA_CALLABLE Quat(const Mat33& m)
    {
        Vec3 c0 = m.column0.normalized();
        Vec3 c1 = m.column1.normalized();
        Vec3 c2 = m.column2.normalized();

        if (c2.z < 0.0f)
        {
            if (c0.x > c1.y)
            {
                float t = 1.0f + c0.x - c1.y - c2.z;
                *this = Quat(t, c0.y + c1.x, c2.x + c0.z, c1.z - c2.y) * (0.5f / sqrtf(t));
            }
            else
            {
                float t = 1.0f - c0.x + c1.y - c2.z;
                *this = Quat(c0.y + c1.x, t, c1.z + c2.y, c2.x - c0.z) * (0.5f / sqrtf(t));
            }
        }
        else
        {
            if (c0.x < -c1.y)
            {
                float t = 1.0f - c0.x - c1.y + c2.z;
                *this = Quat(c2.x + c0.z, c1.z + c2.y, t, c0.y - c1.x) * (0.5f / sqrtf(t));
            }
            else
            {
                float t = 1.0f + c0.x + c1.y + c2.z;
                *this = Quat(c1.z - c2.y, c2.x - c0.z, c0.y - c1.x, t) * (0.5f / sqrtf(t));
            }
        }
    }
    CUDA_CALLABLE Quat(const Vec3& fromAxis, const Vec3& toAxis)
    {
        Vec3 v0 = fromAxis, v1 = toAxis;
        v0.normalize();
        v1.normalize();
        float dot = v0.dot(v1);
        if (dot >= 1.0f)
        {
            x = y = z = 0.0f;
            w = 1.0f;
        }
        else if (dot <= -1.0f)
        {
            Vec3 p = v0.perp();
            x = p.x;
            y = p.y;
            z = p.z;
            w = 0.0f;
        }
        else
        {
            float s = sqrtf((1.0f + dot) * 2.0f);
            float invs = 1.0f / s;
            Vec3 cross = v0.cross(v1);
            x = cross.x * invs;
            y = cross.y * invs;
            z = cross.z * invs;
            w = s * 0.5f;
            normalize();
        }
    }

    CUDA_CALLABLE bool operator==(const Quat& q) const
    {
        return x == q.x && y == q.y && z == q.z && w == q.w;
    }

    CUDA_CALLABLE Vec3 getBasisVector0() const
    {
        const float x2 = x * 2.0f;
        const float w2 = w * 2.0f;
        return Vec3((w * w2) - 1.0f + x * x2, (z * w2) + y * x2, (-y * w2) + z * x2);
    }

    CUDA_CALLABLE Vec3 getBasisVector1() const
    {
        const float y2 = y * 2.0f;
        const float w2 = w * 2.0f;
        return Vec3((-z * w2) + x * y2, (w * w2) - 1.0f + y * y2, (x * w2) + z * y2);
    }

    CUDA_CALLABLE Vec3 getBasisVector2() const
    {
        const float z2 = z * 2.0f;
        const float w2 = w * 2.0f;
        return Vec3((y * w2) + x * z2, (-x * w2) + y * z2, (w * w2) - 1.0f + z * z2);
    }

    CUDA_CALLABLE Quat operator*(float s) const
    {
        return Quat(x * s, y * s, z * s, w * s);
    }

    CUDA_CALLABLE void operator+=(const Quat& q)
    {
        x += q.x;
        y += q.y;
        z += q.z;
        w += q.w;
    }

    CUDA_CALLABLE Quat operator+(const Quat& q)
    {
        return Quat(x + q.x, y + q.y, z + q.z, w + q.w);
    }

    CUDA_CALLABLE Quat operator-(const Quat& q)
    {
        return Quat(x - q.x, y - q.y, z - q.z, w - q.w);
    }

    CUDA_CALLABLE float magnitude()
    {
        return sqrtf(x * x + y * y + z * z + w * w);
    }
    CUDA_CALLABLE void normalize()
    {
        float len = magnitude();
        if (len > 0.0f)
        {
            float invLen = 1.0f / len;
            x *= invLen;
            y *= invLen;
            z *= invLen;
            w *= invLen;
        }
    }

    CUDA_CALLABLE Quat normalized() const
    {
        Quat q = *this;
        q.normalize();
        return q;
    }

    CUDA_CALLABLE Quat getConjugate() const
    {
        return Quat(-x, -y, -z, w);
    }

    CUDA_CALLABLE Quat getInverse() const
    {
        return Quat(-x, -y, -z, w);
    }

    CUDA_CALLABLE const Vec3 rotate(const Vec3& v) const
    {
        const float vx = 2.0f * v.x;
        const float vy = 2.0f * v.y;
        const float vz = 2.0f * v.z;
        const float w2 = w * w - 0.5f;
        const float dot2 = (x * vx + y * vy + z * vz);
        return Vec3((vx * w2 + (y * vz - z * vy) * w + x * dot2), (vy * w2 + (z * vx - x * vz) * w + y * dot2),
                    (vz * w2 + (x * vy - y * vx) * w + z * dot2));
    }

    CUDA_CALLABLE const Vec3 rotateInv(const Vec3& v) const
    {
        const float vx = 2.0f * v.x;
        const float vy = 2.0f * v.y;
        const float vz = 2.0f * v.z;
        const float w2 = w * w - 0.5f;
        const float dot2 = (x * vx + y * vy + z * vz);
        return Vec3((vx * w2 - (y * vz - z * vy) * w + x * dot2), (vy * w2 - (z * vx - x * vz) * w + y * dot2),
                    (vz * w2 - (x * vy - y * vx) * w + z * dot2));
    }

    CUDA_CALLABLE Quat rotate(const Quat& q) const
    {
        return *this * q;
    }

    CUDA_CALLABLE Quat rotateInv(const Quat& q) const
    {
        return getConjugate() * q;
    }

    CUDA_CALLABLE Quat operator*(const Quat& q) const
    {
        return Quat(w * q.x + q.w * x + y * q.z - q.y * z, w * q.y + q.w * y + z * q.x - q.z * x,
                    w * q.z + q.w * z + x * q.y - q.x * y, w * q.w - x * q.x - y * q.y - z * q.z);
    }

    CUDA_CALLABLE Quat operator+(const Quat& q) const
    {
        return Quat(x + q.x, y + q.y, z + q.z, w + q.w);
    }

    CUDA_CALLABLE inline Quat rotateLinear(const Quat& q, const Vec3& omega)
    {
        Quat res = q;
        res += Quat(omega.x, omega.y, omega.z, 0.0) * q * 0.5;
        res.normalize();
        return res;
    }

    CUDA_CALLABLE inline Quat rotateNonlinear(const Quat& q, const Vec3& omega)
    {
        Quat res = q;
        if (!omega.isZero())
        {
            Vec3 n = omega;
            float ang = n.normalize();
            res = Quat(ang, n) * q;
        }
        return res;
    }

    CUDA_CALLABLE Quat inline project(const Vec3& unitAxis)
    {
        Quat q;
        float s = x * unitAxis.x + y * unitAxis.y + z * unitAxis.z;
        q.x = unitAxis.x * s;
        q.y = unitAxis.y * s;
        q.z = unitAxis.z * s;

        q.w = sqrtf(Max(0.0f, 1.0f - s * s));
        if (w < 0.0f)
        {
            q.w = -q.w;
        }
        return q;
    }

    CUDA_CALLABLE Quat lerp(const Quat& v, float s) const
    {
        float t = 1.0f - s;
        Quat q(x * t + v.x * s, y * t + v.y * s, z * t + v.z * s, w * t + v.w * s);
        q.normalize();
        return q;
    }

    CUDA_CALLABLE Quat slerp(const Quat& v, float t) const
    {
        float cosHalfTheta = x * v.x + y * v.y + z * v.z + w * v.w;
        if (fabs(cosHalfTheta) >= 1.0f)
            return *this;

        bool reverse = false;
        if (cosHalfTheta < 0.0f)
        {
            reverse = true;
            cosHalfTheta = -cosHalfTheta;
        }

        float halfTheta = Acos(cosHalfTheta);
        float sinHalfTheta = sqrtf(1.0f - cosHalfTheta * cosHalfTheta);
        float a = 1.0f - t;
        float b = t;

        if (fabs(sinHalfTheta) > 0.001f)
        {
            a = Sin((1.0f - t) * halfTheta) / sinHalfTheta;
            b = Sin(t * halfTheta) / sinHalfTheta;
        }

        if (!reverse)
            return Quat(a * x + b * v.x, a * y + b * v.y, a * z + b * v.z, a * w + b * v.w);
        else
            return Quat(a * x - b * v.x, a * y - b * v.y, a * z - b * v.z, a * w - b * v.w);
    }

    CUDA_CALLABLE Vec3 toEulerAngles() const
    {
        Quat qn = this->normalized();
        Vec3 angles(Zero);

        // First rotation around X
        float sinr_cosp = 2.0f * (qn.w * qn.x + qn.y * qn.z);
        float cosr_cosp = 1.0f - 2.0f * (qn.x * qn.x + qn.y * qn.y);
        angles.x = atan2f(sinr_cosp, cosr_cosp);

        // Second rotation around Y
        float sinp = 2.0f * (qn.w * qn.y - qn.z * qn.x);
        if (fabsf(sinp) >= 1.0f)
            angles.y = copysignf(HalfPi, sinp);
        else
            angles.y = asinf(sinp);

        // Third rotation around Z
        float siny_cosp = 2.0f * (qn.w * qn.z + qn.x * qn.y);
        float cosy_cosp = 1.0f - 2.0f * (qn.y * qn.y + qn.z * qn.z);
        angles.z = atan2f(siny_cosp, cosy_cosp);
        return angles;
    }

    CUDA_CALLABLE static Quat fromEulerAngles(const Vec3& angles)
    {
        Quat q;

        float cy = cosf(angles.x * 0.5f);
        float sy = sinf(angles.x * 0.5f);
        float cp = cosf(angles.y * 0.5f);
        float sp = sinf(angles.y * 0.5f);
        float cr = cosf(angles.z * 0.5f);
        float sr = sinf(angles.z * 0.5f);

        q.w = cr * cp * cy + sr * sp * sy;
        q.z = sr * cp * cy - cr * sp * sy;
        q.y = cr * sp * cy + sr * cp * sy;
        q.x = cr * cp * sy - sr * sp * cy;

        return q;
    }

    float x, y, z, w;
};


CUDA_CALLABLE inline Quat operator*(float f, const Quat& q)
{
    return Quat(f * q.x, f * q.y, f * q.z, f * q.w);
}


CUDA_CALLABLE inline Mat33::Mat33(const Quat& q)
{
    float x = q.x, y = q.y, z = q.z, w = q.w;
    float x2 = x + x, y2 = y + y, z2 = z + z;
    float xx = x2 * x, yy = y2 * y, zz = z2 * z;
    float xy = x2 * y, xz = x2 * z, xw = x2 * w;
    float yz = y2 * z, yw = y2 * w, zw = z2 * w;
    column0 = Vec3(1.0f - yy - zz, xy + zw, xz - yw);
    column1 = Vec3(xy - zw, 1.0f - xx - zz, yz + xw);
    column2 = Vec3(xz + yw, yz - xw, 1.0f - xx - yy);
}


} // namespace viewer
