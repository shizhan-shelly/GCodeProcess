// Copyright 2016 Fangling Software Co., Ltd. All Rights Reserved.
// Author: shizhan-shelly@hotmail.com (Zhan Shi)

#include "CircleHoleCodeRebuild.h"

#include <assert.h>

CircleHoleCodeRebuild::CircleHoleCodeRebuild() : HoleCodeRebuild() {}

CircleHoleCodeRebuild::~CircleHoleCodeRebuild() {}

void CircleHoleCodeRebuild::RebuildCircleHoleCode(
    std::vector<GCodeStruct> &rebuild_codes,
    const std::vector<GCodeStruct> &g_code, size_t circle_index,
    double kerf_hole, double speed_hole,
    double lead_in_speed, double over_burn_speed,
    double US, double asynchronous_stop) {

  std::vector<GCodeStruct>::const_iterator iter = g_code.begin()
      + circle_index;

  if (iter->Name != G02 && iter->Name != G03) {
    assert(false);
    return ;
  }
  GCodeStruct code_array = g_code[circle_index];
  code_array.Name = G00;
  code_array.X0 = rebuild_codes.empty() ? LeadInPoint(g_code, circle_index).x() : rebuild_codes.back().X;
  code_array.Y0 = rebuild_codes.empty() ? LeadInPoint(g_code, circle_index).y() : rebuild_codes.back().Y;
  code_array.X = iter->I;
  code_array.Y = iter->J + kerf_hole / 2;
  rebuild_codes.push_back(code_array);
  if (lead_in_type_ == LineLeadIn) {
    code_array.Name = G00;
    code_array.X0 = code_array.X;
    code_array.Y0 = code_array.Y;
    code_array.X = iter->Name == G03 ? iter->I - kerf_hole / 2 : iter->I + kerf_hole / 2;
    code_array.Y = iter->J + kerf_hole / 2;
    rebuild_codes.push_back(code_array);
  }
  code_array.Name = iter->Name == G03 ? G41 : G42;
  code_array.OmitKerf = false;
  code_array.KerfValue = kerf_hole / 2;
  rebuild_codes.push_back(code_array);
  code_array.Name = M07;
  rebuild_codes.push_back(code_array);
  code_array.Name = M80;
  rebuild_codes.push_back(code_array);
  code_array.Name = M46;
  rebuild_codes.push_back(code_array);
  if (lead_in_type_ == LineLeadIn) {
    code_array.Name= G01;
    code_array.OmitF = false;
    code_array.F = lead_in_speed;
    code_array.X0 = code_array.X;
    code_array.Y0 = code_array.Y;
    code_array.X = code_array.X0;
    code_array.Y = code_array.Y0 - iter->R;
    rebuild_codes.push_back(code_array);
    if (iter->Name == G02) {
      code_array.Name = G02;
      code_array.X0 = code_array.X;
      code_array.Y0 = code_array.Y;
      code_array.X = code_array.X0 - kerf_hole / 2;
      code_array.Y = code_array.Y0 - kerf_hole / 2;
      code_array.I = code_array.X0 - kerf_hole / 2;
      code_array.J = code_array.Y0;
    } else {
      code_array.Name = G03;
      code_array.X0 = code_array.X;
      code_array.Y0 = code_array.Y;
      code_array.X = code_array.X0 + kerf_hole / 2;
      code_array.Y = code_array.Y0 - kerf_hole / 2;
      code_array.I = code_array.X0 + kerf_hole / 2;
      code_array.J = code_array.Y0;
    }
    rebuild_codes.push_back(code_array);
  } else {
    if (iter->Name == G02) {
      code_array.Name = G02;
    } else {
      code_array.Name = G03;
    }
    code_array.OmitF = false;
    code_array.F = lead_in_speed;
    code_array.X0 = code_array.X;
    code_array.Y0 = code_array.Y;
    code_array.X = code_array.X0;
    code_array.Y = code_array.Y0 - iter->R - kerf_hole / 2;
    code_array.I = code_array.X0;
    code_array.J = code_array.Y0 - iter->R / 2 - kerf_hole / 4;
    rebuild_codes.push_back(code_array);
  }

  double arc_angle = GetOverburnArcAngle(US, iter->R);

  if (iter->Name == G02) {
    code_array.Name = G02;
    code_array.X0 = code_array.X;
    code_array.Y0 = code_array.Y;
    code_array.I = iter->I;
    code_array.J = iter->J;
    code_array.OmitF = false;
    code_array.F = speed_hole;
    rebuild_codes.push_back(code_array);

    code_array.Name = G02;
    code_array.X = code_array.X0 - iter->R * sin(arc_angle);
    code_array.Y = code_array.Y0 + iter->R * (1 - cos(arc_angle));
    code_array.OmitF = false;
    code_array.F = over_burn_speed;
    rebuild_codes.push_back(code_array);

    code_array.Name = M50;
    code_array.OmitAsynchronousStop = false;
    code_array.AsynchronousStop = asynchronous_stop;
    rebuild_codes.push_back(code_array);
  } else {
    code_array.Name = G03;
    code_array.X0 = code_array.X;
    code_array.Y0 = code_array.Y;
    code_array.I = iter->I;
    code_array.J = iter->J;
    code_array.OmitF = false;
    code_array.F = speed_hole;
    rebuild_codes.push_back(code_array);

    code_array.Name = G03;
    code_array.X = code_array.X0 + iter->R * sin(arc_angle);
    code_array.Y = code_array.Y0 + iter->R * (1 - cos(arc_angle));
    code_array.OmitF = false;
    code_array.F = over_burn_speed;
    rebuild_codes.push_back(code_array);

    code_array.Name = M50;
    code_array.OmitAsynchronousStop = false;
    code_array.AsynchronousStop = asynchronous_stop;
    rebuild_codes.push_back(code_array);
  }
  code_array.Name = M47;
  rebuild_codes.push_back(code_array);
  code_array.Name = M08;
  rebuild_codes.push_back(code_array);
  code_array.Name = G40;
  rebuild_codes.push_back(code_array);
}
