// Copyright 2017 Fangling Software Co., Ltd. All Rights Reserved.
// Author: shizhan-shelly@hotmail.com (Zhan Shi)

#include "HyperthermCircleHoleCodeRebuild.h"

#include <assert.h>

#include "math/Arc.h"
#include "math/mymath.h"

HyperthermCircleHoleCodeRebuild::HyperthermCircleHoleCodeRebuild() : HoleCodeRebuild() {}

HyperthermCircleHoleCodeRebuild::~HyperthermCircleHoleCodeRebuild() {}

void HyperthermCircleHoleCodeRebuild::RebuildCircleHoleCode(
    std::vector<GCodeStruct> &rebuild_codes,
    const std::vector<GCodeStruct> &g_code, size_t circle_index,
    double hole_kerf, double hole_speed,
    double lead_in_speed, double over_burn_speed,
    double US, double asynchronous_stop, int disable_ahc) {

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
  code_array.Y = iter->J + hole_kerf / 2;
  rebuild_codes.push_back(code_array);
  code_array.Name = iter->Name == G03 ? G41 : G42;
  code_array.OmitKerf = false;
  code_array.KerfValue = hole_kerf / 2;
  rebuild_codes.push_back(code_array);
  code_array.Name = M07;
  rebuild_codes.push_back(code_array);
  code_array.Name = M80;
  rebuild_codes.push_back(code_array);
  if (disable_ahc != 0) {
    code_array.Name = M46;
    rebuild_codes.push_back(code_array);
  }
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
  code_array.Y = code_array.Y0 - iter->R - hole_kerf / 2;
  code_array.I = code_array.X0;
  code_array.J = code_array.Y0 - iter->R / 2 - hole_kerf / 4;
  rebuild_codes.push_back(code_array);

  double arc_angle = GetOverburnArcAngle(US, iter->R);
  //double close_arc_dis = (hole_speed / 60.0) * asynchronous_stop;
  double close_arc_dis = ((hole_speed / 60.0) * asynchronous_stop) * iter->R / (iter->R - hole_kerf / 2);

  if (iter->Name == G02) {
    if (math::IsLesser(close_arc_dis, 0)) {
      code_array.Name = G02;
      code_array.X0 = code_array.X;
      code_array.Y0 = code_array.Y;
      code_array.I = iter->I;
      code_array.J = iter->J;
      code_array.OmitF = false;
      code_array.F = hole_speed;
      math::Arc arc(code_array.I, code_array.J, code_array.R,
          code_array.X0, code_array.Y0, code_array.X, code_array.Y, math::CW);

      if (arc.Calc(code_array.X, code_array.Y, fabs(close_arc_dis)) == -1) {
        rebuild_codes.push_back(code_array);
      } else {
        rebuild_codes.push_back(code_array);
        code_array.Name = M29;
        code_array.X0 = code_array.X;
        code_array.Y0 = code_array.Y;
        rebuild_codes.push_back(code_array);
        code_array.Name = G02;
        code_array.X = iter->I;
        code_array.Y = iter->J - iter->R;
        rebuild_codes.push_back(code_array);
      }
    } else {
      code_array.Name = G02;
      code_array.X0 = code_array.X;
      code_array.Y0 = code_array.Y;
      code_array.I = iter->I;
      code_array.J = iter->J;
      code_array.OmitF = false;
      code_array.F = hole_speed;
      rebuild_codes.push_back(code_array);
    }
    code_array.Name = G02;
    code_array.X0 = code_array.X;
    code_array.Y0 = code_array.Y;
    code_array.X = code_array.X0 - iter->R * sin(arc_angle);
    code_array.Y = code_array.Y0 + iter->R * (1 - cos(arc_angle));
    code_array.OmitF = false;
    code_array.F = over_burn_speed;
    rebuild_codes.push_back(code_array);
  } else {
    if (math::IsLesser(close_arc_dis, 0)) {
      code_array.Name = G03;
      code_array.X0 = code_array.X;
      code_array.Y0 = code_array.Y;
      code_array.I = iter->I;
      code_array.J = iter->J;
      code_array.OmitF = false;
      code_array.F = hole_speed;
      math::Arc arc(code_array.I, code_array.J, code_array.R,
          code_array.X0, code_array.Y0, code_array.X, code_array.Y, math::CCW);

      if (arc.Calc(code_array.X, code_array.Y, fabs(close_arc_dis)) == -1) {
        rebuild_codes.push_back(code_array);
      } else {
        rebuild_codes.push_back(code_array);
        code_array.Name = M29;
        code_array.X0 = code_array.X;
        code_array.Y0 = code_array.Y;
        rebuild_codes.push_back(code_array);
        code_array.Name = G03;
        code_array.X = iter->I;
        code_array.Y = iter->J - iter->R;
        rebuild_codes.push_back(code_array);
      }
    } else {
      code_array.Name = G03;
      code_array.X0 = code_array.X;
      code_array.Y0 = code_array.Y;
      code_array.I = iter->I;
      code_array.J = iter->J;
      code_array.OmitF = false;
      code_array.F = hole_speed;
      rebuild_codes.push_back(code_array);
    }
    code_array.Name = G03;
    code_array.X0 = code_array.X;
    code_array.Y0 = code_array.Y;
    code_array.X = code_array.X0 + iter->R * sin(arc_angle);
    code_array.Y = code_array.Y0 + iter->R * (1 - cos(arc_angle));
    code_array.OmitF = false;
    code_array.F = over_burn_speed;
    rebuild_codes.push_back(code_array);
  }
  if (disable_ahc != 0) {
    code_array.Name = M47;
    rebuild_codes.push_back(code_array);
  }
  code_array.Name = M08;
  rebuild_codes.push_back(code_array);
  code_array.Name = G40;
  rebuild_codes.push_back(code_array);
}

void HyperthermCircleHoleCodeRebuild::ModifyCircleHoleCode(
    std::vector<GCodeStruct> &rebuild_codes,
    const std::vector<GCodeStruct> &g_code, size_t circle_index,
    double hole_kerf, double hole_speed,
    double lead_in_speed, double over_burn_speed,
    double asynchronous_stop) {

  std::vector<GCodeStruct>::iterator iter = rebuild_codes.end() - 1;
  for (; iter > rebuild_codes.begin(); iter--) {
    if (iter->Name != G01 && iter->Name != G02 && iter->Name != G03) {
      break;
    }
    iter->OmitF = false;
    iter->F = lead_in_speed;
  }

  std::vector<GCodeStruct>::const_iterator circle_iter = g_code.begin()
      + circle_index;

  if (circle_iter->Name != G02 && circle_iter->Name != G03) {
    assert(false);
    return ;
  }
  GCodeStruct code_array = g_code[circle_index];
  code_array.OmitF = false;
  code_array.F = hole_speed;

  double close_arc_dis = ((hole_speed / 60.0) * asynchronous_stop) * circle_iter->R / (circle_iter->R - hole_kerf / 2);
  if (math::IsLesser(close_arc_dis, 0)) {
    math::Arc arc(code_array.I, code_array.J, code_array.R,
        code_array.X0, code_array.Y0, code_array.X, code_array.Y,
        circle_iter->Name == G02 ? math::CW : math::CCW);

    if (arc.Calc(code_array.X, code_array.Y, fabs(close_arc_dis)) == -1) {
      rebuild_codes.push_back(code_array);
    } else {
      rebuild_codes.push_back(code_array);
      code_array.Name = M29;
      code_array.X0 = code_array.X;
      code_array.Y0 = code_array.Y;
      rebuild_codes.push_back(code_array);
      code_array.Name = circle_iter->Name;
      code_array.X = circle_iter->X;
      code_array.Y = circle_iter->Y;
      rebuild_codes.push_back(code_array);
    }
  } else {
    rebuild_codes.push_back(code_array);
  }
  for (size_t i = circle_index + 1; i < g_code.size(); i++) {
    code_array = g_code[i];
    if (code_array.Name == G01 || code_array.Name == G02 || code_array.Name == G03) {
      code_array.OmitF = false;
      code_array.F = over_burn_speed;
    }
    rebuild_codes.push_back(code_array);
    if (code_array.Name == G00) {
      break;
    }
  }
}
