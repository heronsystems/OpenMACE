#ifndef HELPER_PI_H
#define HELPER_PI_H

#define _USE_MATH_DEFINES

#include <cmath>
#include <cstddef>

namespace mace
{
namespace math
{

/** Modifies the given angle to translate it into the [0,2pi[ range.
  * \note Take care of not instancing this template for integer numbers, since
 * it only works for float, double and long double.
  * \sa wrapToPi, wrapTo2Pi, unwrap2PiSequence
  */
template <class T>
inline T wrapTo2Pi(T a)
{
    bool was_neg = a < 0;
    a = fmod(a, static_cast<T>(2.0 * M_PI));
    if (was_neg) a += static_cast<T>(2.0 * M_PI);

        return a;
}

/** Modifies the given angle to translate it into the ]-pi,pi] range.
  * \note Take care of not instancing this template for integer numbers, since
 * it only works for float, double and long double.
  * \sa wrapTo2Pi, wrapToPiInPlace, unwrap2PiSequence
  */
template <class T>
inline T wrapToPi(T a)
{
        return wrapTo2Pi(a + static_cast<T>(M_PI)) - static_cast<T>(M_PI);
}


/** Computes the shortest angular increment (or distance) between two planar
 * orientations, such that it is constrained to [-pi,pi] and is correct for
 * any combination of angles (e.g. near +-pi)
  * Examples: angDistance(0,pi) -> +pi; angDistance(pi,0) -> -pi;
  *           angDistance(-3.1,3.1) -> -0.08; angDistance(3.1,-3.1) -> +0.08;
  * \note Take care of not instancing this template for integer numbers, since
 * it only works for float, double and long double.
  * \sa wrapToPi, wrapTo2Pi
  */
template <class T>
inline T angDistance(T from, T to)
{
        wrapTo2Pi(from);
        wrapTo2Pi(to);
        T d = to - from;
        if (d > M_PI)
                d -= 2. * M_PI;
        else if (d < -M_PI)
                d += 2. * M_PI;
        return d;
}

template <class T>
inline T compassToPolarBearing(const T &value)
{
    T newValue = -(value - M_PI_2);
    return wrapTo2Pi(newValue);
}

template <class T>
inline T polarToCompassBearing(const T &value)
{
    T newValue = -value + M_PI_2;
    return wrapTo2Pi(newValue);
}


template <class T>
inline T correctBearing(const T &value)
{
    return fmod((value * 180.0/M_PI) + 360.0,360.0);
}

template <class T>
inline T reverseBearing(const T &value)
{
    T newValue = value + M_PI;
    return wrapTo2Pi(newValue);
}


/**
 * @brief Position::convertDegreesToRadians
 * @param degrees
 * @return
 */
inline double convertDegreesToRadians(const double &degrees)
{
    double radians = degrees * (M_PI/180.0);
    return radians;
}

/**
 * @brief Position::convertRadiansToDegrees
 * @param radians
 * @return
 */
inline double convertRadiansToDegrees(const double &radians)
{
    double degrees = radians * (180.0/M_PI);
    return degrees;
}

}  // End of math namespace
}  // End of namespace

#endif

