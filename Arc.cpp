// Copyright 2017 Fangling Software Co., Ltd. All Rights Reserved.
// Author: shizhan-shelly@hotmail.com (Zhan Shi)

#include "Arc.h"

#include "mymath.h"

namespace math {

Arc::Arc(double center_x, double center_y, double radius,
         double start_x, double start_y, double end_x, double end_y,
         Clockwise clockwise) : Circle(center_x, center_y, radius)
                              , start_x_(start_x)
                              , start_y_(start_y)
                              , end_x_(end_x)
                              , end_y_(end_y)
                              , clockwise_(clockwise) {}

void Arc::Set(double center_x, double center_y, double radius,
    double start_x, double start_y, double end_x, double end_y,
    Clockwise clockwise) {

  radius_ = radius;
  center_x_ = center_x;
  center_y_ = center_y;
  start_x_ = start_x;
  start_y_ = start_y;
  end_x_ = end_x;
  end_y_ = end_y;
  clockwise_ = clockwise;
}

int Arc::Calc(double &x, double &y, double length_to_end) {
  double arc_length = ArcLength();
  if (IsLesser(arc_length, length_to_end) == 1 || IsZero(arc_length) == 1) {
    return -1;
  }
  // According to the Cosine theorem: a * a + b * b - c * c = 2 * a * b * consin
  // Get two equations with two angles: angle_to_end and angle_to_start.
  double angle_to_end = length_to_end / radius_;
  double angle_to_start = (arc_length - length_to_end) / radius_;
  angle_to_end = clockwise_ == CCW ? angle_to_end : -angle_to_end;
  angle_to_start = clockwise_ == CCW ? angle_to_start : -angle_to_start;
  if (IsZero(angle_to_start)) {
    x = start_x_;
    y = start_y_;
    return 0;
  }
  double cosine1 = cos(angle_to_end);
  double cosine2 = cos(angle_to_start);
  double a1 = end_x_ - center_x_;
  double b1 = end_y_ - center_y_;
  double a2 = start_x_ - center_x_;
  double b2 = start_y_ - center_y_;
  double c1 = sqr(radius_) * cosine1 + end_x_ * center_x_ + end_y_ * center_y_ - sqr(center_x_) - sqr(center_y_);
  double c2 = sqr(radius_) * cosine2 + start_x_ * center_x_ + start_y_ * center_y_ - sqr(center_x_) - sqr(center_y_);
  if (IsEqual(a2 * b1, b2 * a1) == 1 || IsZero(b1)) {
    x = center_x_ + (start_x_ - center_x_) * cos(angle_to_start)
        - (start_y_ - center_y_) * sin(angle_to_start);

    y = center_y_ + (start_x_ - center_x_) * sin(angle_to_start)
        + (start_y_ - center_y_) * cos(angle_to_start);

    return 1;
  }
  x = (b1 * c2 - b2 * c1) / (a2 * b1 - b2 * a1);
  y = (c1 - a1 * x) / b1;
  return 1;
}

double Arc::ArcLength() {
  double a = math::dist(start_x_, start_y_, center_x_, center_y_);
  double b = math::dist(end_x_, end_y_, center_x_, center_y_);
  double c = math::dist(start_x_, start_y_, end_x_, end_y_);
  double cosine = (a * a + b * b - c * c) / (2 * a * b);
  double cross_product_z =
      (start_x_ - center_x_) * (end_y_ - center_y_) -
      (start_y_ - center_y_) * (end_x_ - center_x_);

  double arc_angle = 0;
  if (clockwise_ == CW) {
    arc_angle = cross_product_z < 0 ? acos(cosine) : 2 * math::PI - acos(cosine);
  } else {
    arc_angle = cross_product_z > 0 ? acos(cosine) : 2 * math::PI - acos(cosine);
  }

  return radius_ * arc_angle;
}

}
