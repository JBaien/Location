#ifndef TIMOO_DRIVER_TIMER_H
#define TIMOO_DRIVER_TIMER_H

#include <climits>
#include <math.h>
#include <sys/time.h>
#include <iostream>
#include <chrono>
#include <cstdint>
#include <limits>
#include <iomanip>

namespace timoo {
namespace driver {
namespace base {


//////////////////////////////////////////////////
/// Duration
//////////////////////////////////////////////////

inline void normalizeSecNSecSigned(int64_t& sec, int64_t& nsec)
{
  int64_t nsec_part = nsec;
  int64_t sec_part = sec;

  while (nsec_part > 1000000000L)
  {
    nsec_part -= 1000000000L;
    ++sec_part;
  }
  while (nsec_part < 0)
  {
    nsec_part += 1000000000L;
    --sec_part;
  }

  if (sec_part < INT_MIN || sec_part > INT_MAX)
    throw std::runtime_error("Duration is out of dual 32-bit range");

  sec = sec_part;
  nsec = nsec_part;
}

inline void normalizeSecNSecSigned(int32_t& sec, int32_t& nsec)
{
  int64_t sec64 = sec;
  int64_t nsec64 = nsec;

  normalizeSecNSecSigned(sec64, nsec64);

  sec = (int32_t)sec64;
  nsec = (int32_t)nsec64;
}

/**
 * \brief Base class for Duration implementations.  Provides storage, common functions and operator overloads.
 * This should not need to be used directly.
 */
template<class T>
class DurationBase
{
public:
  int32_t sec, nsec;
  DurationBase() : sec(0), nsec(0) { }
  DurationBase(int32_t _sec, int32_t _nsec);
  explicit DurationBase(double t){fromSec(t);};
  ~DurationBase() {}
  T operator+(const T &rhs) const;
  T operator-(const T &rhs) const;
  T operator-() const;
  T operator*(double scale) const;
  T& operator+=(const T &rhs);
  T& operator-=(const T &rhs);
  T& operator*=(double scale);
  bool operator==(const T &rhs) const;
  inline bool operator!=(const T &rhs) const { return !(*static_cast<const T*>(this) == rhs); }
  bool operator>(const T &rhs) const;
  bool operator<(const T &rhs) const;
  bool operator>=(const T &rhs) const;
  bool operator<=(const T &rhs) const;
  double toSec() const { return (double)sec + 1e-9*(double)nsec; };
  int64_t toNSec() const {return (int64_t)sec*1000000000ll + (int64_t)nsec;  };
  T& fromSec(double t);
  T& fromNSec(int64_t t);
  bool isZero();
};

/**
 * \brief Duration representation for use with the Time class.
 *
 * ros::DurationBase provides most of its functionality.
 */
class Duration : public DurationBase<Duration>
{
public:
  Duration()
  : DurationBase<Duration>()
  { }

  Duration(int32_t _sec, int32_t _nsec)
  : DurationBase<Duration>(_sec, _nsec)
  {}

  explicit Duration(double t) { fromSec(t); }

  /**
   * \brief sleep for the amount of time specified by this Duration.  If a signal interrupts the sleep, resleeps for the time remaining.
   */
  bool sleep() const;
};

extern const Duration DURATION_MAX;
extern const Duration DURATION_MIN;


//
// DurationBase template member function implementation
//
template<class T>
DurationBase<T>::DurationBase(int32_t _sec, int32_t _nsec)
: sec(_sec), nsec(_nsec)
{
  normalizeSecNSecSigned(sec, nsec);
}

template<class T>
T& DurationBase<T>::fromSec(double d)
{
#ifdef HAVE_TRUNC
  sec  = (int32_t)trunc(d);
#else
  // (morgan: why doesn't win32 provide trunc? argh. hacked this together
  // without much thought. need to test this conversion.
  if (d >= 0.0)
    sec = (int32_t)floor(d);
  else
    sec = (int32_t)floor(d) + 1;
#endif
  nsec = (int32_t)((d - (double)sec)*1000000000);
  return *static_cast<T*>(this);
}

template<class T>
T& DurationBase<T>::fromNSec(int64_t t)
{
  sec  = (int32_t)(t / 1000000000);
  nsec = (int32_t)(t % 1000000000);

  normalizeSecNSecSigned(sec, nsec);

  return *static_cast<T*>(this);
}

template<class T>
T DurationBase<T>::operator+(const T &rhs) const
{
  return T(sec + rhs.sec, nsec + rhs.nsec);
}

template<class T>
T DurationBase<T>::operator*(double scale) const
{
  return T(toSec() * scale);
}

template<class T>
T DurationBase<T>::operator-(const T &rhs) const
{
  return T(sec - rhs.sec, nsec - rhs.nsec);
}

template<class T>
T DurationBase<T>::operator-() const
{
  return T(-sec , -nsec);
}

template<class T>
T& DurationBase<T>::operator+=(const T &rhs)
{
  *this = *this + rhs;
  return *static_cast<T*>(this);
}

template<class T>
T& DurationBase<T>::operator-=(const T &rhs)
{
  *this += (-rhs);
  return *static_cast<T*>(this);
}

template<class T>
T& DurationBase<T>::operator*=(double scale)
{
  fromSec(toSec() * scale);
  return *static_cast<T*>(this);
}

template<class T>
bool DurationBase<T>::operator<(const T &rhs) const
{
  if (sec < rhs.sec)
    return true;
  else if (sec == rhs.sec && nsec < rhs.nsec)
    return true;
  return false;
}

template<class T>
bool DurationBase<T>::operator>(const T &rhs) const
{
  if (sec > rhs.sec)
    return true;
  else if (sec == rhs.sec && nsec > rhs.nsec)
    return true;
  return false;
}

template<class T>
bool DurationBase<T>::operator<=(const T &rhs) const
{
  if (sec < rhs.sec)
    return true;
  else if (sec == rhs.sec && nsec <= rhs.nsec)
    return true;
  return false;
}

template<class T>
bool DurationBase<T>::operator>=(const T &rhs) const
{
  if (sec > rhs.sec)
    return true;
  else if (sec == rhs.sec && nsec >= rhs.nsec)
    return true;
  return false;
}

template<class T>
bool DurationBase<T>::operator==(const T &rhs) const
{
  return sec == rhs.sec && nsec == rhs.nsec;
}

template<class T>
bool DurationBase<T>::isZero()
{
  return sec == 0 && nsec == 0;
}



//////////////////////////////////////////////////
/// Time
//////////////////////////////////////////////////


inline void normalizeSecNSec(uint64_t& sec, uint64_t& nsec)
{
  uint64_t nsec_part = nsec % 1000000000UL;
  uint64_t sec_part = nsec / 1000000000UL;

  if (sec_part > UINT_MAX)
    throw std::runtime_error("Time is out of dual 32-bit range");

  sec += sec_part;
  nsec = nsec_part;
}

inline void normalizeSecNSec(uint32_t& sec, uint32_t& nsec)
{
  uint64_t sec64 = sec;
  uint64_t nsec64 = nsec;

  normalizeSecNSec(sec64, nsec64);

  sec = (uint32_t)sec64;
  nsec = (uint32_t)nsec64;
}

inline void normalizeSecNSecUnsigned(int64_t& sec, int64_t& nsec)
{
  int64_t nsec_part = nsec;
  int64_t sec_part = sec;

  while (nsec_part >= 1000000000L)
  {
    nsec_part -= 1000000000L;
    ++sec_part;
  }
  while (nsec_part < 0)
  {
    nsec_part += 1000000000L;
    --sec_part;
  }

  if (sec_part < 0 || sec_part > INT_MAX)
    throw std::runtime_error("Time is out of dual 32-bit range");

  sec = sec_part;
  nsec = nsec_part;
}

/**
 * \brief Base class for Time implementations.  Provides storage, common functions and operator overloads.
 * This should not need to be used directly.
 */
template<class T, class D>
class TimeBase
{
public:
  uint32_t sec, nsec;

  TimeBase() : sec(0), nsec(0) { }
  TimeBase(uint32_t _sec, uint32_t _nsec) : sec(_sec), nsec(_nsec)
  {
    normalizeSecNSec(sec, nsec);
  }
  explicit TimeBase(double t) { fromSec(t); }
  ~TimeBase() {}
  D operator-(const T &rhs) const;
  T operator+(const D &rhs) const;
  T operator-(const D &rhs) const;
  T& operator+=(const D &rhs);
  T& operator-=(const D &rhs);
  bool operator==(const T &rhs) const;
  inline bool operator!=(const T &rhs) const { return !(*static_cast<const T*>(this) == rhs); }
  bool operator>(const T &rhs) const;
  bool operator<(const T &rhs) const;
  bool operator>=(const T &rhs) const;
  bool operator<=(const T &rhs) const;

  inline double toSec()  const { return (double)sec + 1e-9*(double)nsec; };
  inline T& fromSec(double t) { sec = (uint32_t)floor(t); nsec = (uint32_t)round((t-sec) * 1e9);  return *static_cast<T*>(this);}

  uint64_t toNSec() const {return (uint64_t)sec*1000000000ull + (uint64_t)nsec;  }
  T& fromNSec(uint64_t t);

  inline bool isZero() const { return sec == 0 && nsec == 0; }
  inline bool is_zero() const { return isZero(); }
};


/**
 * \brief Time representation.  May either represent wall clock time or ROS clock time.
 *
 * ros::TimeBase provides most of its functionality.
 */
class Time : public TimeBase<Time, Duration>
{
public:
  Time()
  : TimeBase<Time, Duration>()
  {}

  Time(uint32_t _sec, uint32_t _nsec)
  : TimeBase<Time, Duration>(_sec, _nsec)
  {}

  explicit Time(double t) { fromSec(t); }


  inline static Time now() {
    Time t;
    auto now = std::chrono::high_resolution_clock::now();
    auto nanos = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();

    t.sec = static_cast<uint32_t>(nanos * 1e-9);
    t.nsec = static_cast<uint32_t>(nanos % static_cast<int64_t>(1e9));

    return t;
  };

};


inline std::ostream &operator <<(std::ostream &os, const Time &rhs){
  os << rhs.sec << "." << std::setw(9) << std::setfill('0') << rhs.nsec;
  return os;
}

template<class T, class D>
T& TimeBase<T, D>::fromNSec(uint64_t t)
{
  sec  = (int32_t)(t / 1000000000);
  nsec = (int32_t)(t % 1000000000);

  normalizeSecNSec(sec, nsec);

  return *static_cast<T*>(this);
}

template<class T, class D>
D TimeBase<T, D>::operator-(const T &rhs) const
{
  return D((int32_t)sec -  (int32_t)rhs.sec,
                  (int32_t)nsec - (int32_t)rhs.nsec); // carry handled in ctor
}

template<class T, class D>
T TimeBase<T, D>::operator-(const D &rhs) const
{
  return *static_cast<const T*>(this) + ( -rhs);
}

template<class T, class D>
T TimeBase<T, D>::operator+(const D &rhs) const
{
  int64_t sec_sum  = (int64_t)sec  + (int64_t)rhs.sec;
  int64_t nsec_sum = (int64_t)nsec + (int64_t)rhs.nsec;

  // Throws an exception if we go out of 32-bit range
  normalizeSecNSecUnsigned(sec_sum, nsec_sum);

  // now, it's safe to downcast back to uint32 bits
  return T((uint32_t)sec_sum, (uint32_t)nsec_sum);
}

template<class T, class D>
T& TimeBase<T, D>::operator+=(const D &rhs)
{
  *this = *this + rhs;
  return *static_cast<T*>(this);
}

template<class T, class D>
T& TimeBase<T, D>::operator-=(const D &rhs)
{
  *this += (-rhs);
  return *static_cast<T*>(this);
}

template<class T, class D>
bool TimeBase<T, D>::operator==(const T &rhs) const
{
  return sec == rhs.sec && nsec == rhs.nsec;
}

template<class T, class D>
bool TimeBase<T, D>::operator<(const T &rhs) const
{
  if (sec < rhs.sec)
    return true;
  else if (sec == rhs.sec && nsec < rhs.nsec)
    return true;
  return false;
}

template<class T, class D>
bool TimeBase<T, D>::operator>(const T &rhs) const
{
  if (sec > rhs.sec)
    return true;
  else if (sec == rhs.sec && nsec > rhs.nsec)
    return true;
  return false;
}

template<class T, class D>
bool TimeBase<T, D>::operator<=(const T &rhs) const
{
  if (sec < rhs.sec)
    return true;
  else if (sec == rhs.sec && nsec <= rhs.nsec)
    return true;
  return false;
}

template<class T, class D>
bool TimeBase<T, D>::operator>=(const T &rhs) const
{
  if (sec > rhs.sec)
    return true;
  else if (sec == rhs.sec && nsec >= rhs.nsec)
    return true;
  return false;
}


} // namespace base
} // namespace driver
} // namespace timoo



#endif // TIMOO_DRIVER_TIMER_H