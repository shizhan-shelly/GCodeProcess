// Copyright 2017 Fangling Software Co., Ltd. All Rights Reserved.
// Author: shizhan-shelly@hotmail.com (Zhan Shi)

#include "GCodeRestruct.h"

#include "math\mymath.h"

GCodeRestruct theApp;

void GCodeRestruct::GetHyperthermProcessParameter(double diameter,
    double &hole_speed, double &lead_in_speed, double &hole_kerf,
    double &asynchronous_stop, int &disable_ahc, double &US) {

  hole_kerf = 0.5;
  hole_speed = 1000;
  lead_in_speed = 1000;
  asynchronous_stop = -0.053;
  disable_ahc = 1;
  US = 3.1415926 * diameter / 4;
}

void GCodeRestruct::GetKjellbergProcessParameter(double hole_diameter,
    double &hole_speed, double &lead_in_speed, double &overburn_speed,
    double &hole_kerf,
    double &US, double &PA) {

  double cutting_kerf_hole = 1.0;
  double cutting_kerf_quality = 2.0;
  double cutting_speed_hole = 800;
  double cutting_speed_quality = 1000;
  double thickness = 10;
  double down_slope = 0.2;

  double N2 = hole_diameter / thickness;
  double N8 = cutting_kerf_hole;
  double N17 = cutting_kerf_quality;
  double N18 = N17 - N8;
  double N19 = ((N2 - 1) * N18) + N8;
  double N20 = ((0.8 - 1) * N18) + N8;
  if (N2 < 0.8) {
    hole_kerf = N20;
  } else {
    if (N2 > 2) {
      hole_kerf = N17;
    } else {
      hole_kerf = N19;
    }
  }
  double N3 = cutting_speed_hole;
  double N4 = cutting_speed_quality;
  double N5 = N4- N3;
  double N6 = ((0.8 - 1) * N5) + N3;
  double O2 = ((N2 - 1) * N5) + N3;
  if (N2 < 0.8) {
    hole_speed = N6;
  } else {
    if (N2 > 2) {
      hole_speed = N4;
    } else {
      hole_speed = O2;
    }
  }
  down_slope *= 1000; // unit: ms
  double DS = (down_slope / 60) * (hole_speed / 1000);
  double D33 = down_slope + 50;
  US = DS + (1.5 * cutting_kerf_hole);
  PA = (hole_speed / 60) * (D33 / 1000);

  lead_in_speed = hole_speed;
  overburn_speed = hole_speed;
}

std::string GCodeRestruct::GetReferDiameter(double cur_diameter,
    std::vector<std::string> selectable_diameter) const {

  // selectable_diameter vector has already been sorted by ASC.
  for (size_t i = 0; i < selectable_diameter.size(); i++) {
    if (selectable_diameter[i].find('*') == std::string::npos &&
        !math::IsLesser(atof(selectable_diameter[i].c_str()), cur_diameter)) {

      return selectable_diameter[i];
    }
  }
  return "*";
}
