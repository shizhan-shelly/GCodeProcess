// Copyright 2017 Fangling Software Co., Ltd. All Rights Reserved.
// Author: shizhan-shelly@hotmail.com (Zhan Shi)

#ifndef ARC_H__
#define ARC_H__

#include "Circle.h"

namespace math {

typedef enum _Clockwise {
  CW,
  CCW,

} Clockwise;

class Arc : public Circle {
 public:
  Arc() : clockwise_(CCW) {}
  Arc(double center_x, double center_y, double radius,
      double start_x, double start_y, double end_x, double end_y,
      Clockwise clockwise = CCW);

  void Set(double center_x, double center_y, double radius,
      double start_x, double start_y, double end_x, double end_y,
      Clockwise clockwise);

  // Calculate the special point(x, y) in the arc, its distance to the end
  // point of arc is length_to_end.
  // return: -1, failed to get the special point as the error parameter.
  //         0, start point.
  //         1, successful to get the point.
  int Calc(double &x, double &y, double length_to_end);

  double ArcLength();

  Clockwise clockwise() const {
    return clockwise_;
  }

 private:
  double start_x_;
  double start_y_;
  double end_x_;
  double end_y_;
  Clockwise clockwise_;

}; // class Arc

} // namespace math

#endif // ARC_H__
