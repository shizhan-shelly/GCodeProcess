// Copyright 2016 Fangling Software Co., Ltd. All Rights Reserved.
// Author: shizhan-shelly@hotmail.com (Zhan Shi)

#ifndef MATH_H__
#define MATH_H__

#include <math.h>

#include "Circle.h"

#ifndef MAX
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#endif

#ifndef MIN
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif

namespace math {

static const double INFINITESIMAL = 0.000001;
static const double PI = 3.14159265358979323846264338327950288419716939937510;
static const double _1p4PI = 0.78539816;
static const double _1p2PI = 1.57079633;
static const double _3p4PI = 2.35619449;
static const double _5p4PI = 3.92699081;
static const double _3p2PI = 4.71238898;
static const double _7p4PI = 5.49778714;
static const double _2PI = 6.28318531;
static const double EP = 1e-5;

inline bool IsZero(double x) {
  return fabs(x) < INFINITESIMAL;
}

inline double fpmin(double x, double y) {
  return x < y ? x : y;
}

inline double fpmax(double x, double y) {
  return x > y ? x : y;
}

inline bool IsEqual(double x, double y) {
  return fabs(x - y) < fpmax(INFINITESIMAL * fpmin(fabs(x), fabs(y)), INFINITESIMAL);
}

// return x > y
inline bool IsGreater(double x, double y) {
  return x > y + fpmax(INFINITESIMAL * fpmin(fabs(x), fabs(y)), INFINITESIMAL);
}

// return x < y
inline bool IsLesser(double x, double y) {
  return x < y - fpmax(INFINITESIMAL * fpmin(fabs(x), fabs(y)), INFINITESIMAL);
} 

inline double dist(double x1, double y1, double x2, double y2) {
  return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

inline double sqr(double x) {
  return x * x;
}

/*
 * Calculate intersector point of two circles.
 * [in] circle1, first circle
 * [in] circle2, second circle
 * [out] point1, first intersector point
 * [out] point2, second intersector point
 * @return: 1, no intersector
 *          2, outer
 *          3, Inscribed
 *          4, two intersectors
 *          5, one included by other circle(concentric circle)
 */
int CircleIntersector(const Circle &circle1, const Circle &circle2,
    D_Point &point1, D_Point &point2);

} // namespace math

#endif // MATH_H__
