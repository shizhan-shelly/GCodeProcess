// Copyright 2016 Fangling Software Co., Ltd. All Rights Reserved.
// Author: shizhan-shelly@hotmail.com (Zhan Shi)

#ifndef D_POINT_H__
#define D_POINT_H__

class D_Point {
 public:
  D_Point(double x = 0., double y = 0.);

  double x() const {
    return x_;
  }

  double y() const {
    return y_;
  }

  static double Distance(D_Point p1, D_Point p2);

  static bool IsSamePoint(D_Point p1, D_Point p2);

 private:
  double x_;
  double y_;

}; // class D_Point

#endif // D_POINT_H__
