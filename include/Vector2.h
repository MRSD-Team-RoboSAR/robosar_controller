/*
 * RVO Library
 * vector2.h
 *
 * Copyright 2008 University of North Carolina at Chapel Hill
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Please send all bug reports to <geom@cs.unc.edu>.
 *
 * The authors may be contacted via:
 *
 * Jur van den Berg
 * Dept. of Computer Science
 * 201 S. Columbia St.
 * Frederick P. Brooks, Jr. Computer Science Bldg.
 * Chapel Hill, N.C. 27599-3175
 * United States of America
 *
 * <http://gamma.cs.unc.edu/RVO/>
 */

/*! \file vector2.h Contains the Vector2 class; a two-dimensional vector, and some operations on a vector. */
#ifndef __VECTOR_H__
#define __VECTOR_H__

#include <iostream>
#include <cmath>
namespace RVO {
  /*! Defines a two-dimensional vector and its operations. */
  class Vector2 {

  private:
    /*! The x-component of the vector. */
    float _x;
    /*! The y-component of the vector. */
    float _y;

  public:
    /*! \returns The x-component of the vector. */
    inline float x() const { return _x; }
    /*! \returns The y-component of the vector. */
    inline float y() const { return _y; }

    /*! Constructs a null vector. */
    inline Vector2() { _x = 0; _y = 0; }
    /*! Copy constructor.
        \param q The vector to be copied into the new vector. */
    inline Vector2(const Vector2& q) { _x = q.x(); _y = q.y(); }
    /*! Constructor.
        \param x The x-component of the new vector.
        \param y The y-component of the new vector. */
    inline Vector2(float x, float y) { _x = x; _y = y; }

    /*! Unary minus.
        \returns The negation of the vector.  */
    inline Vector2 operator-() const { return Vector2(-_x, -_y); }
    /*! Unary plus.
        \returns A reference to the vector.  */
    inline const Vector2& operator+() const { return (*this); }

    /*! Dot product.
        \param q The right hand side vector
        \returns The dot product of the lhs vector and the rhs vector.  */
    inline float operator*(const Vector2& q) const { return _x * q.x() + _y * q.y(); }
    /*! Scalar product.
        \param a The right hand side scalar
        \returns The scalar product of the lhs vector and the rhs scalar.  */
    inline Vector2 operator*(float a) const { return Vector2(_x * a, _y * a); }
    /*! Scalar division.
        \param a The right hand side scalar
        \returns The scalar division of the lhs vector and the rhs scalar.  */
    inline Vector2 operator/(float a) const { float inv_a = 1.0f/a; return Vector2(_x * inv_a, _y * inv_a); }
    /*! Vector addition.
        \param q The right hand side vector
        \returns The sum of the lhs vector and the rhs vector.  */
    inline Vector2 operator+(const Vector2& q) const { return Vector2(_x + q.x(), _y + q.y()); }
    /*! Vector subtraction.
        \param q The right hand side vector
        \returns The vector difference of the lhs vector and the rhs vector.  */
    inline Vector2 operator-(const Vector2& q) const { return Vector2(_x - q.x(), _y - q.y()); }

    /*! Vector equality.
        \param q The right hand side vector
        \returns True if the lhs vector and the rhs vector are equal. False otherwise.  */
    inline bool operator==(const Vector2& q) const { return (_x == q.x() && _y == q.y()); }
    /*! Vector inequality.
        \param q The right hand side vector
        \returns True if the lhs vector and the rhs vector are not equal. False otherwise.  */
    inline bool operator!=(const Vector2& q) const { return (_x != q.x() || _y != q.y()); }

    /*! The operator multiplies the vector by a scalar.
        \param a The scalar
        \returns A reference to the vector.  */
    inline Vector2& operator*=(float a) { _x *= a; _y *= a; return *this; }
    /*! The operator divides the vector by a scalar.
        \param a The scalar
        \returns A reference to the vector.  */
    inline Vector2& operator/=(float a) { float inv_a = 1.0f/a; _x *= inv_a; _y *= inv_a; return *this; }
    /*! The operator adds an rhs vector to the vector.
        \param q The right hand side vector
        \returns A reference to the vector.  */
    inline Vector2& operator+=(const Vector2& q) { _x += q.x(); _y += q.y(); return *this; }
    /*! The operator subtracts an rhs vector from the vector.
        \param q The right hand side vector
        \returns A reference to the vector.  */
    inline Vector2& operator-=(const Vector2& q) { _x -= q.x(); _y -= q.y(); return *this; }
  };
}

/*! Scalar multiplication.
    \param a The left hand side scalar
    \param q The right hand side vector
    \returns The scalar multiplication of the lhs scalar and the rhs vector.  */
inline RVO::Vector2 operator*(float a, const RVO::Vector2& q) { return RVO::Vector2(a * q.x(), a * q.y()); }

/*! Writes a vector to the standard output.
    \param os The output stream
    \param q The vector
    \returns The standard output.  */
inline std::ostream& operator<<(std::ostream& os, const RVO::Vector2& q) {
  //os << "(" << q.x() << "," << q.y() << ")";
  os << q.x() << " " << q.y();
  return os;
}

/*! \param q A vector
    \returns The squared absolute value of the vector.  */
inline float absSq(const RVO::Vector2& q) { return q*q; }
/*! \param q A vector
    \returns The absolute value of the vector.  */
inline float abs(const RVO::Vector2& q) { return std::sqrt(absSq(q)); }
/*! \param q A vector
    \returns The normalized vector.  */
inline RVO::Vector2 norm(const RVO::Vector2& q) { return q / abs(q); }
/*! \param p A point
    \param q A point
    \returns The normal vector to the line segment pq. */
inline RVO::Vector2 normal(const RVO::Vector2& p, const RVO::Vector2& q) { return norm(RVO::Vector2(q.y() - p.y(), -(q.x() - p.x()))); }
/*! \param q A vector
    \returns The angle the vector makes with the positive x-axis. Is in the range [-PI, PI]. */
inline float atan(const RVO::Vector2& q) { return std::atan2(q.y(), q.x()); }
/*! \param p A vector
    \param q A vector
    \returns Returns the determinant of the 2x2 matrix formed by using p as the upper row and q as the lower row. */
inline float det(const RVO::Vector2& p, const RVO::Vector2& q) { return p.x()*q.y() - p.y()*q.x(); }

inline float euc_dist(const RVO::Vector2 v1, const RVO::Vector2 v2) {
    float x1 = v1.x();
    float y1 = v1.y();
    float x2 = v2.x();
    float y2 = v2.y();
    return abs(sqrt(x1*x1)-sqrt(x2*x2)) + abs(sqrt(y1*y1)-sqrt(y2*y2));
}
#endif