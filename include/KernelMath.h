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
/*!\file
 *
 * \author  Bernd Gaﬂmann
 * \date 14.04.00
 *
 * \brief Mathematical functions
 *
 * Defines often used mathematical functions.
 */
//----------------------------------------------------------------------
#ifndef _KernelMath_h_
#define _KernelMath_h_

//----------------------------------------------------------------------
// non MCA Includes - include with <>
// MCA Includes - include with ""
//----------------------------------------------------------------------
#include <limits.h>

//----------------------------------------------------------------------
//                                Mathematical Function
//----------------------------------------------------------------------

/*! Evaluates the maximum of \c a and \c b and returns the result.
 */
template <class T>
inline T Max(T a, T b)
{
  return a > b ? a : b;
}


/*! Evaluates the maximum of \c a and \c b and returns the result.
 */
inline double Max(double a, double b)
{
  return a > b ? a : b;
}

/*! Evaluates the maximum of \c a and \c b and returns the result.
 */
inline double Max(double a, long b)
{
  return a > b ? a : b;
}

/*! Evaluates the maximum of \c a and \c b and returns the result.
 */
inline double Max(double a, unsigned long b)
{
  return a > b ? a : b;
}

/*! Evaluates the maximum of \c a and \c b and returns the result.
 */
inline double Max(double a, int b)
{
  return a > b ? a : b;
}

/*! Evaluates the maximum of \c a and \c b and returns the result.
 */
inline double Max(double a, unsigned b)
{
  return a > b ? a : b;
}

/*! Evaluates the maximum of \c a and \c b and returns the result.
 */
inline double Max(long a, double b)
{
  return a > b ? a : b;
}

/*! Evaluates the maximum of \c a and \c b and returns the result.
 */
inline double Max(unsigned long a, double b)
{
  return a > b ? a : b;
}

/*! Evaluates the maximum of \c a and \c b and returns the result.
 */
inline double Max(int a, double b)
{
  return a > b ? a : b;
}

/*! Evaluates the maximum of \c a and \c b and returns the result.
 */
inline double Max(unsigned a, double b)
{
  return a > b ? a : b;
}

/*! Evaluates the maximum of \c a and \c b and returns the result.
 */
inline int Max(int a, int b)
{
  return a > b ? a : b;
}

#if 1
//#ifdef _SYSTEM_DARWIN_
/*! Evaluates the maximum of \c a and \c b and returns the result.
 */
inline unsigned long Max(unsigned long a, int b)
{
  // !!!Be Carefull size_t will be casted to unsigned!!!
  return b < 0 ? a : Max(unsigned(a), unsigned(b));
}

/*! Evaluates the maximum of \c a and \c b and returns the result.
 */
inline unsigned long Max(int a, unsigned long b)
{
  // !!!Be Carefull size_t will be casted to unsigned!!!
  return a < 0 ? b : Max(unsigned(a), unsigned(b));
}

/*! Evaluates the maximum of \c a and \c b and returns the result.
 */
inline unsigned long Max(unsigned a, unsigned long b)
{
  return a > b ? a : b;
}
#endif

/*! Evaluates the maximum of \c a and \c b and returns the result.
 */
inline unsigned Max(unsigned a, unsigned b)
{
  return a > b ? a : b;
}

/*! Evaluates the maximum of \c a and \c b and returns the result.
 */
inline unsigned Max(int a, unsigned b)
{
  return a < 0 ? b : Max(unsigned(a), b);
}

/*! Evaluates the maximum of \c a and \c b and returns the result.
 */
inline unsigned Max(unsigned a, int b)
{
  return Max(b, a);
}

/*! Evaluates the maximum of \c a,\c b and \c c and returns the result.
 */
inline double Max(double a, double b, double c)
{
  return Max(a, Max(b, c));
}

/*! Evaluates the maximum of \c a,\c b,\c c and \c d and returns the result.
 */
template<typename A, typename B, typename C, typename D, typename R>
inline R Max(A a, B b, C c, D d)
{
  return Max(a, Max(b, Max(c, d)));
}

/*! Evaluates the minimum of \c a and \c b and returns the result.
 */
template <class T>
inline T Min(T a, T b)
{
  return a < b ? a : b;
}

/*! Evaluates the minimum of \c a and \c b and returns the result.
 */
inline double Min(double a, double b)
{
  return a < b ? a : b;
}

/*! Evaluates the minimum of \c a and \c b and returns the result.
 */
inline double Min(double a, long b)
{
  return a < b ? a : b;
}

/*! Evaluates the minimum of \c a and \c b and returns the result.
 */
inline double Min(double a, int b)
{
  return a < b ? a : b;
}

/*! Evaluates the minimum of \c a and \c b and returns the result.
 */
inline double Min(double a, unsigned long b)
{
  return a < b ? a : b;
}

/*! Evaluates the minimum of \c a and \c b and returns the result.
 */
inline double Min(double a, unsigned b)
{
  return a < b ? a : b;
}

/*! Evaluates the minimum of \c a and \c b and returns the result.
 */
inline double Min(long a, double b)
{
  return a < b ? a : b;
}

/*! Evaluates the minimum of \c a and \c b and returns the result.
 */
inline double Min(int a, double b)
{
  return a < b ? a : b;
}

/*! Evaluates the minimum of \c a and \c b and returns the result.
 */
inline double Min(unsigned long a, double b)
{
  return a < b ? a : b;
}

/*! Evaluates the minimum of \c a and \c b and returns the result.
 */
inline double Min(unsigned a, double b)
{
  return a < b ? a : b;
}

/*! Evaluates the minimum of \c a and \c b and returns the result.
 */
inline int Min(int a, int b)
{
  return a < b ? a : b;
}

/*! Evaluates the minimum of \c a and \c b and returns the result.
 */
inline void* Min(void* a, void* b)
{
  return a < b ? a : b;
}

/*! Evaluates the minimum of \c a and \c b and returns the result.
 */
inline unsigned Min(unsigned a, unsigned b)
{
  return a < b ? a : b;
}

/*! Evaluates the minimum of \c a and \c b and returns the result.
 */
inline int Min(int a, unsigned int b)
{
  return b > INT_MAX ? a : (Min(int(b), a));
}

/*! Evaluates the minimum of \c a and \c b and returns the result.
 */
inline int Min(unsigned int a, int b)
{
  return Min(b, a);
}

/*! Evaluates the minimum of \c a, \c b and \c c and returns the result.
 */
inline double Min(double a, double b, double c)
{
  return Min(a, Min(b, c));
}

/*! Evaluates the minimum of \c a, \c b, \c c and \c d and returns the result.
*/
template<typename A, typename B, typename C, typename D, typename R>
inline R Min(A a, B b, C c, D d)
{
  return Min(a, Min(b, Min(c, d)));
}

/*! Calculate the absolute value
 */
template<typename T>
inline T Abs(T x)
{
  return (x > 0 ? x : -x);
}

#endif
