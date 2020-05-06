// Copyright 2020 Fangling Software Co., Ltd. All Rights Reserved.
// Author: shizhan-shelly@hotmail.com (Zhan Shi)

#include "HyperthermWaistHoleCodeRebuild.h"

#include <assert.h>

#include "math/mymath.h"

using namespace math;

HyperthermWaistHoleCodeRebuild::HyperthermWaistHoleCodeRebuild()
    : HoleCodeRebuild() {}

HyperthermWaistHoleCodeRebuild::~HyperthermWaistHoleCodeRebuild() {}

void HyperthermWaistHoleCodeRebuild::RebuildWaistHoleCode(
    std::vector<GCodeStruct> &rebuild_codes,
    const std::vector<GCodeStruct> &g_code, size_t waist_index,
    double kerf_hole, double speed_hole,
    double lead_in_speed, double over_burn_speed,
    double US, double asynchronous_stop, int disable_ahc) {

  std::vector<GCodeStruct>::const_iterator iter = g_code.begin() + waist_index;
  D_Point start_point = D_Point(iter->X0, iter->Y0);
  std::vector<GCodeStruct> arc_codes;
  for (; iter != g_code.end(); iter++) {
    if (iter->Name == G02 || iter->Name == G03) {
      arc_codes.push_back(*iter);
    }
    if (D_Point::IsSamePoint(start_point, D_Point(iter->X, iter->Y))) {
      break;
    }
  }
  if (arc_codes.size() != 2) {
    assert(false);
    return ;
  }
  RebuildCode(rebuild_codes, g_code, waist_index, arc_codes[0], arc_codes[1],
      kerf_hole, speed_hole, lead_in_speed, over_burn_speed, US,
      asynchronous_stop, disable_ahc);

}

void HyperthermWaistHoleCodeRebuild::RebuildCode(
    std::vector<GCodeStruct> &rebuild_codes,
    const std::vector<GCodeStruct> &g_code, size_t waist_index,
    const GCodeStruct &first_arc, const GCodeStruct &second_arc,
    double kerf_hole, double speed_hole,
    double lead_in_speed, double over_burn_speed,
    double US, double asynchronous_stop, int disable_ahc) {

  GCodeStruct code_array = g_code[waist_index];
  code_array.Name = G00;
  code_array.X0 = rebuild_codes.empty() ? LeadInPoint(g_code, waist_index).x() : rebuild_codes.back().X;
  code_array.Y0 = rebuild_codes.empty() ? LeadInPoint(g_code, waist_index).y() : rebuild_codes.back().Y;
  code_array.X = first_arc.I;
  code_array.Y = first_arc.J + kerf_hole / 2;
  rebuild_codes.push_back(code_array);
  code_array.Name = first_arc.Name == G02 ? G42 : G41;
  code_array.OmitKerf = false;
  code_array.KerfValue = kerf_hole / 2;
  rebuild_codes.push_back(code_array);
  code_array.Name = M07;
  rebuild_codes.push_back(code_array);
  code_array.Name = M80;
  rebuild_codes.push_back(code_array);
  if (disable_ahc != 0) {
    code_array.Name = M46;
    rebuild_codes.push_back(code_array);
  }
  if (first_arc.Name == G02) {
    code_array.Name = G02;
  } else {
    code_array.Name = G03;
  }
  code_array.OmitF = false;
  code_array.F = lead_in_speed;
  code_array.X0 = code_array.X;
  code_array.Y0 = code_array.Y;
  code_array.X = first_arc.X;
  code_array.Y = first_arc.Y - first_arc.R - kerf_hole / 2;
  code_array.I = (code_array.X0 + code_array.X) / 2;
  code_array.J = (code_array.Y0 + code_array.Y) / 2;
  rebuild_codes.push_back(code_array);
  if (first_arc.Name == G02) {
    code_array.Name = G01;
    code_array.OmitF = false;
    code_array.F = speed_hole;
    code_array.X0 = first_arc.X;
    code_array.Y0 = first_arc.Y;
    code_array.X = second_arc.X0;
    code_array.Y = second_arc.Y0;
    rebuild_codes.push_back(code_array);
    code_array.Name = G02;
    code_array.X0 = second_arc.X0;
    code_array.Y0 = second_arc.Y0;
    code_array.X = second_arc.X;
    code_array.Y = second_arc.Y;
    code_array.I = second_arc.I;
    code_array.J = second_arc.J;
    rebuild_codes.push_back(code_array);
    code_array.Name = G01;
    code_array.X0 = second_arc.X;
    code_array.Y0 = second_arc.Y;
    code_array.X = first_arc.X0;
    code_array.Y = first_arc.Y0;
    rebuild_codes.push_back(code_array);
    code_array.Name = G02;
    code_array.X0 = first_arc.X0;
    code_array.Y0 = first_arc.Y0;
    code_array.X = first_arc.X;
    code_array.Y = first_arc.Y;
    code_array.I = first_arc.I;
    code_array.J = first_arc.J;
    rebuild_codes.push_back(code_array);
  } else {
    code_array.Name = G01;
    code_array.OmitF = false;
    code_array.F = speed_hole;
    code_array.X0 = first_arc.X;
    code_array.Y0 = first_arc.Y;
    code_array.X = second_arc.X0;
    code_array.Y = second_arc.Y0;
    rebuild_codes.push_back(code_array);
    code_array.Name = G03;
    code_array.X0 = second_arc.X0;
    code_array.Y0 = second_arc.Y0;
    code_array.X = second_arc.X;
    code_array.Y = second_arc.Y;
    code_array.I = second_arc.I;
    code_array.J = second_arc.J;
    rebuild_codes.push_back(code_array);
    code_array.Name = G01;
    code_array.X0 = second_arc.X;
    code_array.Y0 = second_arc.Y;
    code_array.X = first_arc.X0;
    code_array.Y = first_arc.Y0;
    rebuild_codes.push_back(code_array);
    code_array.Name = G03;
    code_array.X0 = first_arc.X0;
    code_array.Y0 = first_arc.Y0;
    code_array.X = first_arc.X;
    code_array.Y = first_arc.Y;
    code_array.I = first_arc.I;
    code_array.J = first_arc.J;
    rebuild_codes.push_back(code_array);
  }
  std::vector<GCodeStruct> overburn_arc = OverburnArcCodes(first_arc,
      over_burn_speed, US, asynchronous_stop, disable_ahc);

  rebuild_codes.insert(rebuild_codes.end(), overburn_arc.begin(),
      overburn_arc.end());

  code_array = rebuild_codes.back();
  if (disable_ahc != 0) {
    code_array.Name = M47;
    rebuild_codes.push_back(code_array);
  }
  code_array.Name = M08;
  rebuild_codes.push_back(code_array);
  code_array.Name = G40;
  rebuild_codes.push_back(code_array);
}

std::vector<GCodeStruct> HyperthermWaistHoleCodeRebuild::OverburnArcCodes(
    const GCodeStruct &arc_code, double over_burn_speed,
    double US, double asynchronous_stop, int disable_ahc) {

  std::vector<GCodeStruct> arc_codes;
  if (arc_code.Name != G02 && arc_code.Name != G03) {
    assert(false);
    return arc_codes;
  }
  double arc_angle = GetOverburnArcAngle(US, arc_code.R);

  GCodeStruct code_array = arc_code;
  code_array.Name = M50;
  code_array.OmitAsynchronousStop = false;
  code_array.AsynchronousStop = asynchronous_stop;
  arc_codes.push_back(code_array);
  if (arc_code.Name == G02) {
    code_array.Name = G02;
    code_array.X0 = code_array.X;
    code_array.Y0 = code_array.Y;
    code_array.X = code_array.X0 - arc_code.R * sin(arc_angle);
    code_array.Y = code_array.Y0 + arc_code.R * (1 - cos(arc_angle));
    code_array.OmitF = false;
    code_array.F = over_burn_speed;
    arc_codes.push_back(code_array);
  } else if (arc_code.Name == G03) {
    code_array.Name = G03;
    code_array.X0 = code_array.X;
    code_array.Y0 = code_array.Y;
    code_array.X = code_array.X0 + arc_code.R * sin(arc_angle);
    code_array.Y = code_array.Y0 + arc_code.R * (1 - cos(arc_angle));
    code_array.OmitF = false;
    code_array.F = over_burn_speed;
    arc_codes.push_back(code_array);
  }
  return arc_codes;
}

void HyperthermWaistHoleCodeRebuild::ModifyWaistHoleCode(
    std::vector<GCodeStruct> &rebuild_codes,
    const std::vector<GCodeStruct> &g_code,
    size_t waist_start, size_t waist_code_count,
    double kerf_hole, double speed_hole,
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

  GCodeStruct code_array;
  for (size_t i = 0; i < waist_code_count; i++) {
    code_array = g_code[waist_start + i];
    code_array.OmitF = false;
    code_array.F = speed_hole;
    rebuild_codes.push_back(code_array);
  }

  code_array.Name = M50;
  code_array.OmitAsynchronousStop = false;
  code_array.AsynchronousStop = asynchronous_stop;
  rebuild_codes.push_back(code_array);

  for (size_t i = waist_start + waist_code_count; i < g_code.size(); i++) {
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
