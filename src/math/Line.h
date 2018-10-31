// Copyright 2016 Fangling Software Co., Ltd. All Rights Reserved.
// Author: shizhan-shelly@hotmail.com (Zhan Shi)

#ifndef LINE_H__
#define LINE_H__

namespace math {

class Line {
 public:
  Line(double start_x, double start_y, double end_x, double end_y, double min, double max);
  Line() {}

  void Set(double start_x, double start_y, double end_x, double end_y, double min, double max);

  // 根据x求直线的y, 若x超出范围，返回false
  // 若直线垂直于x轴，返回false
  bool Calc(double x, double &y);

  bool Calc(double &x, double &y,
            double start_x, double start_y,
            double end_x, double end_y,
            double length_to_end);

 private:
  // the limit is [min_, max_)
  double min_;
  double max_;
  // the line function is a_ * x + b_ * y + c_ = 0
  double a_;
  double b_;
  double c_;

  // 计算a_, b_, c_参数
  void Calc(double start_x, double start_y, double end_x, double end_y);
};

} // namespace math

#endif // LINE_H__
