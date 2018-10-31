// Copyright 2017 Fangling Software Co., Ltd. All Rights Reserved.
// Author: shizhan-shelly@hotmail.com (Zhan Shi)

#ifndef CIRCLE_H__
#define CIRCLE_H__

#include "D_Point.h"

namespace math {

class Circle {
 public:
  Circle();
  Circle(double center_x, double center_y, double &radius);

  D_Point circle_center() const {
    return D_Point(center_x_, center_y_);
  }

  double radius() const {
    return radius_;
  }

 protected:
  double radius_;
  double center_x_;
  double center_y_;

}; // class Circle

} // namespace math

#endif // CIRCLE_H__
