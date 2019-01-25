// Copyright 2017 Fangling Software Co., Ltd. All Rights Reserved.
// Author: shizhan-shelly@hotmail.com (Zhan Shi)

#ifndef GCODERESTRUCT_H__
#define GCODERESTRUCT_H__

#include <string>
#include <vector>

class GCodeRestruct {
 public:
  GCodeRestruct() {}
  ~GCodeRestruct() {}

  void GetHyperthermProcessParameter(double diameter,
      double &hole_speed, double &lead_in_speed, double &hole_kerf,
      double &asynchronous_stop, int &disable_ahc, double &US);

  void GetKjellbergProcessParameter(double hole_diameter,
      double &hole_speed, double &lead_in_speed, double &overburn_speed,
      double &hole_kerf, double &asynchronous_stop,
      double &US, double &PA);

 private:
  std::string GetReferDiameter(double cur_diameter,
      std::vector<std::string> selectable_diameter) const;

}; // class GCodeRestruct

extern GCodeRestruct theApp;

#endif // GCODERESTRUCT_H__
