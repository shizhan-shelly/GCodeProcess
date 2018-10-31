// Copyright 2016 Fangling Software Co., Ltd. All Rights Reserved.
// Author: shizhan-shelly@hotmail.com (Zhan Shi)

#include "Line.h"

#include <cassert>

#include "mymath.h"

namespace math {

Line::Line(double start_x, double start_y, double end_x, double end_y, double min, double max)
    : min_(min), max_(max) {
  assert(!IsEqual(start_x, end_x) || !IsEqual(start_y, end_y));
  Calc(start_x, start_y, end_x, end_y);
}

void Line::Set(double start_x, double start_y, double end_x, double end_y,
               double min, double max) {

  assert(!IsEqual(start_x, end_x) || !IsEqual(start_y, end_y));
  min_ = min;
  max_ = max;
  Calc(start_x, start_y, end_x, end_y);
}

void Line::Calc(double start_x, double start_y, double end_x, double end_y) {
  if (IsEqual(start_x, end_x)) {
    a_ = 1.;
    b_ = 0.;
    c_ = -start_x;
  } else {
    double k = (end_y - start_y) / (end_x - start_x);
    double b = start_y - k * start_x;
    a_ = k;
    b_ = -1.;
    c_ = b;
  }
}

bool Line::Calc(double x, double &y) {
  if (IsEqual(b_, 0.)) {
    return false;
  }
  if (IsLesser(x, min_) || !IsLesser(x, max_)) {
    return false;
  }
  y = (-c_ - a_ * x) / b_;
  return true;
}

bool Line::Calc(double &x, double &y,
                double start_x, double start_y,
                double end_x, double end_y,
                double length_to_end) {


  double length = dist(start_x, start_y, end_x, end_y);
  if (length_to_end > length || IsZero(length) == 1) {
    return false;
  }
  if (IsZero(b_) == 1) {
    x = start_x;
    y = start_y < end_y ? end_y - length_to_end : end_y + length_to_end;
    return true;
  }
  if (start_y < end_y) {
    x = IsEqual(start_x, end_x) ? start_x : end_x - (end_x - start_x) * length_to_end / length;
  } else if (start_y > end_y) {

    x = IsEqual(start_x, end_x) ? start_x : start_x + (end_x - start_x) * (length - length_to_end) / length;
  } else {
    x = start_x < end_x ? end_x - length_to_end : end_x + length_to_end;
  }
  y = (-c_ - a_ * x) / b_;
  return true;
}

} // namespace math
