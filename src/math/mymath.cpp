// Copyright 2018 Fangling Software Co., Ltd. All Rights Reserved.
// Author: shizhan-shelly@hotmail.com (Zhan Shi)

#include "mymath.h"

#include <assert.h>

namespace math {

int CircleIntersector(const Circle &circle1, const Circle &circle2,
                      D_Point &point1, D_Point &point2) {

  double xc1 = circle1.circle_center().x();
  double yc1 = circle1.circle_center().y();
  double r1 = circle1.radius();
  double xc2 = circle2.circle_center().x();
  double yc2 = circle2.circle_center().y();
  double r2 = circle2.radius();

  assert(r1 > 0. && r2 > 0.);

  double center_dis = dist(xc1, yc1, xc2, yc2);

  if (IsLesser(r1 + r2 , center_dis)) {
    return 1;
  }
  if (IsLesser(center_dis, fabs(r1 - r2)) || IsZero(center_dis)) {
    return 5;
  }
  int rtn = 0;
  if (IsEqual(center_dis, fabs(r1 - r2))) {
    rtn = 3;
  } else if (IsEqual(center_dis, fabs(r1 + r2))) {
    rtn = 2;
  } else {
    rtn = 4;
  }

  double cos_x = (xc2 - xc1) / center_dis;
  double sin_x = (yc2 - yc1) / center_dis;

  double cos_a = (pow(r1,2) + pow(center_dis, 2) - pow(r2, 2)) / (2 * r1 * center_dis);
  double sin_a = sqrt(1 - pow(cos_a, 2));

  point1 = D_Point(xc1 + r1 * (cos_x * cos_a - sin_x * sin_a),
                   yc1 + r1 * (sin_x * cos_a + cos_x * sin_a));

  point2 = D_Point(xc1 + r1 * (cos_x * cos_a + sin_x * sin_a),
                   yc1 + r1 * (sin_x * cos_a - cos_x * sin_a));

  return rtn;
}

} // namespace math
