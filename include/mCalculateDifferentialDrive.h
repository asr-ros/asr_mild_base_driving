// this is a -*- C++ -*- file
//----------------------------------------------------------------------
/*!\file
 *
 * \author   Daniel Pathmaperuma
 * \date    27.03.03
 *
 * \brief   Contains mCalculateDifferentialDrive
 *
 * \b mCalculateDifferentialDrive
 *
 * contains mCalculateDifferentialDrive
 *
 */
//----------------------------------------------------------------------

#ifndef _CalculateDifferentialDrive_h_
#define _CalculateDifferentialDrive_h_
//----------------------------------------------------------------------
// non MCA Includes - include with <>
// MCA Includes - include with ""
//----------------------------------------------------------------------
#include "OwnMath.h"
#include <ros/time.h>
#include <ros/duration.h>
#include <nav_msgs/Odometry.h>

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//! This Module calculates a given speed, radius and angle_speed to two single velocities for a DifferentialDrive and back
/*!

 How does controll work?

  - if speed or radius are nonzero,we consider the vehicle should drive a curve.
    for the calculations, we take ONLY RADIUS AND SPEED, NOT ANGLE_SPEED !!
   If radius is zero output speed is set to 0

 - if speed and radius are both zero but
    angle_speed is not zero, we consider the vehicle
    should turn without changing the position. Therefore the speed of the left
    wheel is the inverse of the right wheel speed

 - negative radius turns right

 - negaitve angle-speed turns right
    \f[  vl = \frac{v}{-r} * (\frac{wd}{2} - r)    \f]
    \f[
      vr = \frac{v}{-r} * (\frac{-wd}{2} - r)
  \f]
  or in case of using angular_speed:
  \f[    vr =  \frac {av}{2} * wd        \f]
  \f[    vl =  \frac {-av}{2} * wd        \f]


   How does sense work?

there are different cases to be considered
 - there are no turning wheels
   -# in that case, by definition we want
     the speed, radius and angle_speed to be 0

 - both wheels are turning at the same rate
  -# the radius is set to 1+E09
    -# the angle_speed is set to 0
    -# the speed is set to the speed of one wheel

 - one wheel is turning at the negative speed of the other
 ( v_left = - v_right)
 -# the speed is set to 0
  -# the radius is set to 0
  -# the angle_speed is set acording to the turning speed

 - all other cases
  -# now at last, we catched all special cases
     we calculate the speed and the radius via the "Strahlensatz"
     then set the angle_speed as calculatet out of the radius and speed

      \f[ v=\frac {vl * -r} {\frac{wd}{2} - r}                          \f]
   \f[ r=\frac{wd}{2} * \frac{ \frac{vl}{vr} +1} {\frac {vl}{vr} -1} \f]
      \f[ av = 2* asin(\frac {\frac{v}{2}} {-r})                      \f]


   r  = radius;
   v  = speed;
   av = angle speed;
   wd = wheel distance;
   vl = speed left;
   vr = speed right;


*/
class mCalculateDifferentialDrive
{
public:
  /*!
    Anonymous enumeration typ which contains the indices of the
    controller inputs.
  */
  DESCR(static, mCalculateDifferentialDrive, ci_description, 4, Natural, cDATA_VECTOR_END_MARKER);
  enum
  {
    eCI_VELOCITY,          /*!< velocity of the vehicle in mm/s  */
    eCI_RADIUS,            /*!< 1/radius of the vehicle in mm    */
    eCI_ANGULAR_VELOCITY,  /*!< the angular velocity of the vehicle
            in RAD (0 - 2pi, negative = turn to right*/
    eCI_DIMENSION       /*!< Endmarker and Dimension */
  };

  /*!
    Anonymous enumeration typ which contains the indices of the
    controller outputs.
  */
  DESCR(static, mCalculateDifferentialDrive, co_description, 4, Natural, cDATA_VECTOR_END_MARKER);
  enum
  {
    eCO_VELOCITY_LEFT,  /*!< velocity of the left wheel  */
    eCO_VELOCITY_RIGHT, /*!< velocity of the right wheel */
    eCO_DIMENSION       /*!< Endmarker and Dimension */
  };

  /*!
    Anonymous enumeration typ which contains the indices of the
    sensor inputs.
  */
  DESCR(static, mCalculateDifferentialDrive, si_description, 4, Natural, cDATA_VECTOR_END_MARKER);
  enum
  {
    eSI_IMPULSES_LEFT,  /*!< impulses of the left wheel  */
    eSI_IMPULSES_RIGHT, /*!< impulses of the right wheel */
    eSI_IMPULSES_TIME_STAMP_SEC, /*!< timestamp (secs) of above impuls values */
    eSI_IMPULSES_TIME_STAMP_USEC,/*!< timestamp (usecs) of above impuls values */
    eSI_DIMENSION /*!< Endmarker and Dimension */
  };

  /*! \param parent the parent
   *  \param wheel_distance of used vehicle in mm
   *  \param name The module name
   *  \param fixit whether to use FixIt() or not
   */
  mCalculateDifferentialDrive(float _wheel_distance,
                                int max_encoder,
                                float impulses_per_mm_left,
                                float impulses_per_mm_right,
                                float speed_curve_diff);
  /*!
   */
  virtual ~mCalculateDifferentialDrive();
  /*!
   */
  virtual void Control();
  /*!
    if angle_speed is zero, we consider the vehicle should drive straigt,
    therefore both wheels get the same speed

    if angle_speed is not zero, but radius is, we consider the vehicle
    should turn without changing the position. Therefore the speed of the left
    wheel is the inverse of the right wheel speed

    in all other cases, we consider the vehicle should drive a curve.
    for the calculations, we take ONLY RADIUS AND SPEED, NOT ANGLE_SPEED !!
  */
  virtual void Sense();
  /*!
    there are different cases to be considered
    1.
    there are no turning wheels
    -> in that case, by definition we want
    the speed, radius and angle_speed to be 0
    2.
    both wheels are turning at the same rate
    the radius is set to 1+E09
    the angle_speed is set to 0
    -> the speed is set to the speed of one wheel
    3.
    one wheel is turning at the negative speed of the other
    v_left = - v_right
    -> the speed is set to 0
    the radius is set to 0
    the angle_speed is set acording to the turning speed
    4.
    all other cases
    now at last, we catched all special cases
    we calculate the speed and the radius via the "Strahlensatz"
    then set the angle_speed as calculatet out of the radius and speed

  */
  virtual void Exception(tExceptionType type);
private:
  float wheel_distance;
  ros::Time last_encoder_time_stamp;
  float ticks_left;
  float ticks_right;
  float ticks_left_old;
  float ticks_right_old;

  float velocity_left;
  float velocity_right;

  // parameters
  float impulses_per_mm_left;
  float impulses_per_mm_right;
  int   max_encoder;
  float speed_curve_diff;

};

#endif
