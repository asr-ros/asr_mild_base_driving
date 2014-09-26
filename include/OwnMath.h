// this is a -*- C++ -*- file

// -- BEGIN LICENSE BLOCK ----------------------------------------------
//
// You received this file as part of MCA2
// Modular Controller Architecture Version 2
//
// Copyright (C) FZI Forschungszentrum Informatik Karlsruhe
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
//
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file OwnMath.h
 *
 * \author  Bernd Gaﬂmann
 * \date 14.04.00
 *
 * \brief Mathematical functions
 *
 * Defines often used mathematical functions with are not part of the
 * GNU math library. math.h/math166.h/rt_math.h is included.
 *
 * If the macro MATH_USE_PRECISE_OPS is defined, the standard precise versions
 * of sin() and cos() are called. Otherwise fast versions are used, which use
 * table lookup in the range [-2pi,2pi].
 */
//----------------------------------------------------------------------

#ifndef _OwnMath_h_
#define _OwnMath_h_
//----------------------------------------------------------------------
// Includes
//----------------------------------------------------------------------
#include "KernelMath.h"

#ifdef _SYSTEM_WIN32_
# define M_E  2.7182818284590452354 /* e */
# define M_LOG2E 1.4426950408889634074 /* log_2 e */
# define M_LOG10E 0.43429448190325182765 /* log_10 e */
# define M_LN2  0.69314718055994530942 /* log_e 2 */
# define M_LN10  2.30258509299404568402 /* log_e 10 */
# define M_PI  3.14159265358979323846 /* pi */
# define M_PI_2  1.57079632679489661923 /* pi/2 */
# define M_PI_4  0.78539816339744830962 /* pi/4 */
# define M_1_PI  0.31830988618379067154 /* 1/pi */
# define M_2_PI  0.63661977236758134308 /* 2/pi */
# define M_2_SQRTPI 1.12837916709551257390 /* 2/sqrt(pi) */
# define M_SQRT2 1.41421356237309504880 /* sqrt(2) */
# define M_SQRT1_2 0.70710678118654752440 /* 1/sqrt(2) */
#endif

#include <stdlib.h>
#include <math.h>

double FastSin(double x);
double FastCos(double x);

//--------------------------------------------------------------------------------------------------
//                                Mathematical Function
//--------------------------------------------------------------------------------------------------



/*! Evaluates \c a fmod(\c b) and returns the result (0 <= result < b).
 */
inline double Mod(double a, double b)
{
  double c = fmod(a, b);
  return c >= 0 ? c : c + b;
}

/*! Evaluates \c a fmod(\c b) and returns the result (0 <= result < b).
 */
inline double Mod(double a, int b)
{
  return Mod(a, double(b));
}

/*! Evaluates \c a mod(\c b) and returns the result (0 <= result < b).
 */
inline int Mod(int a, int b)
{
  int c = a % b;
  return c >= 0 ? c : c + b;
}

/*! Evaluates \c a mod(\c b) and returns the result (0 <= result < b).
 */
inline long int Mod(long int a, long int b)
{
  int c = a % b;
  return c >= 0 ? c : c + b;
}

/*! Returns the sign (1 or -1) of \c a. The sign of zero here is defined as 1!
 */
inline int Sgn(double a)
{
  return a >= 0 ? 1 : -1;
}

/*! Returns the sign (1 or -1) of \c a. The sign of zero here is defined as 1!
 */
inline int Sgn(long double a)
{
  return a >= 0 ? 1 : -1;
}

/*! Returns the sign (1, -1 or 0) of \c a. The sign of zero here is defined as 0!
 */
inline int Sgn0(double a)
{
  return a > 0 ? 1 : a < 0 ? -1 : 0;
}

/*! Returns the sign (1, -1 or 0) of \c a. The sign of zero here is defined as 0!
 */
inline int Sgn0(long double a)
{
  return a > 0 ? 1 : a < 0 ? -1 : 0;
}

/*! Returns the sign (1 or -1) of \c a. The sign of zero here is defined as 1!
 */
inline int Sgn(long a)
{
  return a >= 0 ? 1 : -1;
}

/*! Returns the sign (1, -1 or 0) of \c a. The sign of zero here is defined as 0!
 */
inline int Sgn0(long a)
{
  return a > 0 ? 1 : a < 0 ? -1 : 0;
}

/*! Converts the angle \c a from radiant to degree and returns the result.
 */
inline double Rad2Deg(double a)
{
  static double factor = 180 / M_PI;
  return a*factor;
}

/*! Converts the angle \c a from degree to radiant and returns the result.
 */
inline double Deg2Rad(double a)
{
  static double factor = M_PI / 180;
  return a*factor;
}

/*! return the \a angle (in rad) normalized to [0, 2*M_PI) */
inline double NormalizeAngleUnsigned(const double angle)
{
  return Mod(angle, 2*M_PI);
};

/*! return the \a angle (in rad) normalized to [-M_PI, M_PI) */
inline double NormalizeAngleSigned(const double angle)
{
  return Mod(angle + M_PI, 2*M_PI) - M_PI;
};


/*! return the \a angle (in degree) normalized to [-180, 180) */
//@Zacharias: inherente Annahme angle>-180
inline double NormalizeAngleInDegreeSigned(const double angle)
{
  return Mod(angle + 180, 360) - 180;
};

/*! returns a power b for int values */
inline int Pow(int a, int b)
{
  int ret;
  for (ret = 1;b > 0;--b)
    ret *= a;
  return ret;
}
/*! Rounds a double value with a specified precision.*/
inline double Round(double value, int precision = 0)
{
  double fact = pow(1e1, precision);
  return floor(((value*fact) + 0.5)) / fact;
}

/*! Performs a ceil for  a double value with a specified precision.*/
inline double Ceil(double value, int precision = 0)
{
  double fact = pow(1e1, precision);
  return ceil(value*fact) / fact;
}

/*! Performs a floor for  a double value with a specified precision.*/
inline double Floor(double value, int precision = 0)
{
  double fact = pow(1e1, precision);
  return floor(value*fact) / fact;
}

/*! Adds 2 integer arrays. The result is stored in a.*/
inline int* Add(int *a, int*b, int size)
{
  int i;
  for (i = 0;i < size;i++)
    a[i] += b[i];
  return a;
}

/*! Adds 2 integer arrays. The result is stored in c.*/
inline int* Sum(int *a, int*b, int*c, int size)
{
  int i;
  for (i = 0;i < size;i++)
    c[i] = a[i] + b[i];
  return c;
}

/*! Divides each element ao the array a with b. The resut is stored in a..*/
inline int* Div(int* a, int b, int size)
{
  int i;
  for (i = 0;i < size;i++)
    a[i] /= b;
  return a;
}

/*! Divides each element ao the array a with b. The resut is stored in c.*/
inline int* Div(int* a, int b, int*c, int size)
{
  int i;
  for (i = 0;i < size;i++)
    c[i] = a[i] / b;
  return c;
}

/*! returns the square of a value */
template<typename T>
inline T Sqr(T a)
{
  return a*a;
}

/*! return the square root of a value */
inline double Sqrt(double a)
{
  return sqrt(a);
}

/*!
  returns uniformly distributed random value
  from intervall [0,1]
 */
inline double RandomUniformAbs()
{
  return float(rand()) / RAND_MAX;
}

/*!
  returns uniformly distributed random value
  from intervall [-1,1]
 */
inline double RandomUniform()
{
  return 2*float(rand()) / RAND_MAX - 1;
}

/*!
  returns a normal distributed random value
with specified variance. Mean values is 0.
 */
inline double RandomNormal(double variance)
{
  double r1 = (float((rand()) + 1) / (float(RAND_MAX) + 1)); // intervall ]0,1] wegen log(0)
  double r2 = float(rand()) / RAND_MAX;    // intervall [0,1]
#ifdef MATH_USE_TABLE_LOOKUP
  return variance*sqrt(-2*log(r1))*FastSin(2*M_PI*r2);
#else
  return variance*sqrt(-2*log(r1))*sin(2*M_PI*r2);
#endif
  // ‹brigens ergibt (man beachte den cosinus anstelle des sinus)
  // eine weitere Zufallsvariable, die von der ersten unabh‰ngig ist:
  // sqrt(-2*variance*log(r1))*cos(2*M_PI*r2);
}


//! An integer pow-function.
/*! Calculates \f$ b^e , b \ge 0, e \ge 0 \f$
 *  \param base the base (b)
 *  \param exponent the exponent (e)
 *  \return pow(base,exponent) or 0, if base or exponent are < 0
 */
inline int ipow(int base, int exponent)
{
  if (base <= 0)
    return 0;
  if (exponent < 0)
    return 0;
  else
  {
    int ret = 1;
    for (int i = 1; i <= exponent; i++)
      ret *= base;
    return ret;
  }
}

//! calculates log_2, rounded upward
/*! \param value an integer value
 *  \returns log_2(value), rounded upward
 */
inline int log2upw(int value)
{
  int res = 1;
  while (value > 2)
  {
    value = value / 2;
    res++;
  }
  return res;
}

//! calculates log_8, rounded upward
/*! \param value an integer value
 *  \returns log_8(value), rounded upward
 */
inline int log8upw(int value)
{
  int res = 1;
  while (value > 8)
  {
    value = value / 8;
    res++;
  }
  return res;
}

/*!
  sincos function
  x angle in rad
  sx, cx the return values contain the sinus and the cosinus of x
 */
inline void SinCos(const double x, double &sx, double &cx)
{
#if defined _SYSTEM_LINUX_ && ! defined _SYSTEM_DARWIN_
  sincos(x, &sx, &cx);
#else
  sx = sin(x);
  cx = cos(x);
#endif
}

class tSinCosLookupTable
{
public:
  tSinCosLookupTable() {
    int i = 0;
    double x = 0;
    double increment = 2. * M_PI / double(cTABLE_SIZE);
    for (; i < cTABLE_SIZE; ++i, x += increment) {
      m_sin_table[i] = ::sin(x);
      m_cos_table[i] = ::cos(x);
    }
  }

  double Sin(int index) {
    return m_sin_table[index & cTABLE_INDEX_MASK];
  }

  double Cos(int index) {
    return m_cos_table[index & cTABLE_INDEX_MASK];
  }

  static const int cTABLE_SIZE = 16384;
  static const int cTABLE_INDEX_MASK;
  static const double cSCALE_FACTOR;

private:
  double m_sin_table[cTABLE_SIZE];
  double m_cos_table[cTABLE_SIZE];
};

extern tSinCosLookupTable sin_cos_lookup_table;

inline double FastSin(double x)
{
  double scaled_x = sin_cos_lookup_table.cSCALE_FACTOR * x;
  double table_index = ::floor(scaled_x);
  double y1 = sin_cos_lookup_table.Sin(int(table_index));
  double y2 = sin_cos_lookup_table.Sin(int(table_index) + 1);
  return y1 + (y2 - y1) * (scaled_x - table_index);
}

inline double FastCos(double x)
{
  double scaled_x = sin_cos_lookup_table.cSCALE_FACTOR * x;
  double table_index = ::floor(scaled_x);
  double y1 = sin_cos_lookup_table.Cos(int(table_index));
  double y2 = sin_cos_lookup_table.Cos(int(table_index) + 1);
  return y1 + (y2 - y1) * (scaled_x - table_index);
}

/*!
  FloatIsEqual function
  compares float values by testing against epsilon neighbourhood
  \param a a float or double value
  \param b a float or double value
  \param epsilon minimum difference, defines epsilon neighbourhood
  \returns true if |a-b| < epsilon, else false
 */
inline bool FloatIsEqual(const double a, const double b, const double epsilon = 1e-10)
{
  return fabs(a -b) < epsilon;
}


#endif
