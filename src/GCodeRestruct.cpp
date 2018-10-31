// Copyright 2017 Fangling Software Co., Ltd. All Rights Reserved.
// Author: shizhan-shelly@hotmail.com (Zhan Shi)

#include "GCodeRestruct.h"

GCodeRestruct theApp;

void GCodeRestruct::GetHyperthermProcessParameter(double diameter,
    double &hole_speed, double &lead_in_speed, double &hole_kerf,
    double &asynchronous_stop, int &disable_ahc, double &US) {

  hole_kerf = 0.5;
  hole_speed = 1000;
  lead_in_speed = 1000;
  asynchronous_stop = -0.053;
  disable_ahc = 0;
  US = 3.1415926 * diameter / 4;
}
