// Copyright 2016 Fangling Software Co., Ltd. All Rights Reserved.
// Author: shizhan-shelly@hotmail.com (Zhan Shi)

#include "WaistHoleCodeRebuild.h"

#include <assert.h>

#include "math/D_Point.h"
#include "math/mymath.h"

WaistHoleCodeRebuild::WaistHoleCodeRebuild() : HoleCodeRebuild() {}

WaistHoleCodeRebuild::~WaistHoleCodeRebuild() {}

void WaistHoleCodeRebuild::RebuildWaistHoleCode(
    std::vector<GCodeStruct> &rebuild_codes,
    const std::vector<GCodeStruct> &g_code, size_t waist_index,
    double kerf_hole, double speed_hole,
    double lead_in_speed, double over_burn_speed,
    double US, double PA) {

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
  if (math::IsEqual(arc_codes[0].Y0, arc_codes[0].Y)) {
    if (arc_codes[0].Name == G02) {
      if (arc_codes[0].X0 < arc_codes[0].X) {
        RebuildCode(rebuild_codes, g_code, waist_index, arc_codes[0], arc_codes[1], kerf_hole, speed_hole, lead_in_speed, over_burn_speed, US, PA);
      } else {
        RebuildCode(rebuild_codes, g_code, waist_index, arc_codes[1], arc_codes[0], kerf_hole, speed_hole, lead_in_speed, over_burn_speed, US, PA);
      }
    } else {
      if (arc_codes[0].X0 > arc_codes[0].X) {
        RebuildCode(rebuild_codes, g_code, waist_index, arc_codes[0], arc_codes[1], kerf_hole, speed_hole, lead_in_speed, over_burn_speed, US, PA);
      } else {
        RebuildCode(rebuild_codes, g_code, waist_index, arc_codes[1], arc_codes[0], kerf_hole, speed_hole, lead_in_speed, over_burn_speed, US, PA);
      }
    }
  } else if (arc_codes[0].Y0 < arc_codes[0].Y) {
    RebuildCode(rebuild_codes, g_code, waist_index, arc_codes[0], arc_codes[1], kerf_hole, speed_hole, lead_in_speed, over_burn_speed, US, PA);
  } else {
    RebuildCode(rebuild_codes, g_code, waist_index, arc_codes[1], arc_codes[0], kerf_hole, speed_hole, lead_in_speed, over_burn_speed, US, PA);
  }
}

void WaistHoleCodeRebuild::RebuildCode(
    std::vector<GCodeStruct> &rebuild_codes,
    const std::vector<GCodeStruct> &g_code, size_t waist_index,
    const GCodeStruct &first_arc, const GCodeStruct &second_arc,
    double kerf_hole, double speed_hole,
    double lead_in_speed, double over_burn_speed,
    double US, double PA) {

  GCodeStruct code_array = g_code[waist_index];
  code_array.Name = G00;
  code_array.X0 = rebuild_codes.empty() ? LeadInPoint(g_code, waist_index).x() : rebuild_codes.back().X;
  code_array.Y0 = rebuild_codes.empty() ? LeadInPoint(g_code, waist_index).y() : rebuild_codes.back().Y;
  code_array.X = first_arc.I;
  code_array.Y = first_arc.J;
  rebuild_codes.push_back(code_array);
  if (lead_in_type_ == LineLeadIn) {
    code_array.Name = G00;
    code_array.X0 = code_array.X;
    code_array.Y0 = code_array.Y;
    code_array.X = first_arc.Name == G03 ? first_arc.I - kerf_hole / 2 : first_arc.I + kerf_hole / 2;
    code_array.Y = first_arc.J;
    rebuild_codes.push_back(code_array);
  }
  code_array.Name = first_arc.Name == G02 ? G42 : G41;
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
    code_array.Name = G01;
    code_array.OmitF = false;
    code_array.F = lead_in_speed;
    code_array.X0 = code_array.X;
    code_array.Y0 = code_array.Y;
    code_array.X = first_arc.Name == G03 ? first_arc.X0 - kerf_hole / 2 : first_arc.X0 + kerf_hole / 2;
    code_array.Y = first_arc.Y0 + kerf_hole / 2;
    rebuild_codes.push_back(code_array);
    if (first_arc.Name == G02) {
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
    if (first_arc.Name == G02) {
      code_array.Name = G02;
    } else {
      code_array.Name = G03;
    }
    code_array.OmitF = false;
    code_array.F = lead_in_speed;
    code_array.X0 = code_array.X;
    code_array.Y0 = code_array.Y;
    code_array.X = first_arc.X0;
    code_array.Y = first_arc.Y0;
    code_array.I = (code_array.X0 + code_array.X) / 2;
    code_array.J = (code_array.Y0 + code_array.Y) / 2;
    rebuild_codes.push_back(code_array);
  }
  if (first_arc.Name == G02) {
    code_array.Name = G02;
    code_array.OmitF = false;
    code_array.F = speed_hole;
    code_array.X0 = first_arc.X0;
    code_array.Y0 = first_arc.Y0;
    code_array.X = first_arc.X;
    code_array.Y = first_arc.Y;
    code_array.I = first_arc.I;
    code_array.J = first_arc.J;
    rebuild_codes.push_back(code_array);
    code_array.Name = G01;
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
  } else {
    code_array.Name = G03;
    code_array.OmitF = false;
    code_array.F = speed_hole;
    code_array.X0 = first_arc.X0;
    code_array.Y0 = first_arc.Y0;
    code_array.X = first_arc.X;
    code_array.Y = first_arc.Y;
    code_array.I = first_arc.I;
    code_array.J = first_arc.J;
    rebuild_codes.push_back(code_array);
    code_array.Name = G01;
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
  }
  std::vector<GCodeStruct> overburn_arc = OverburnArcCodes(first_arc,
      over_burn_speed, US, PA);

  rebuild_codes.insert(rebuild_codes.end(), overburn_arc.begin(),
      overburn_arc.end());

  code_array = rebuild_codes.back();
  code_array.Name = M47;
  rebuild_codes.push_back(code_array);
  code_array.Name = M08;
  rebuild_codes.push_back(code_array);
  code_array.Name = G40;
  rebuild_codes.push_back(code_array);
}

std::vector<GCodeStruct> WaistHoleCodeRebuild::OverburnArcCodes(
    const GCodeStruct &arc_code, double over_burn_speed,
    double US, double PA) {

  std::vector<GCodeStruct> arc_codes;
  if (arc_code.Name != G02 && arc_code.Name != G03) {
    assert(false);
    return arc_codes;
  }
  double tan_angle0;
  if (math::IsEqual(arc_code.Y0, arc_code.J)) {
    tan_angle0 = 0;
  } else if (arc_code.Y0 < arc_code.J) {
    tan_angle0 = fabs(arc_code.X0 - arc_code.I) / fabs(arc_code.Y0 - arc_code.J);
  } else {
    if (!math::IsEqual(arc_code.X0, arc_code.I)) {
      tan_angle0 = fabs(arc_code.Y0 - arc_code.J) / fabs(arc_code.X0 - arc_code.I);
    } else {
      tan_angle0 = 0;
    }
  }
  double angle0 = atan(tan_angle0);
  double angle1 = US > PA ? angle0 + GetOverburnArcAngle(US - PA, arc_code.R) : 0;
  double angle2 = angle0 + GetOverburnArcAngle(US, arc_code.R);

  GCodeStruct code_array = arc_code;
  if (arc_code.Name == G02) {
    if (angle1 > 0) {
      if (arc_code.Y0 < arc_code.J) {
        code_array.X = arc_code.X0 - (arc_code.R * sin(angle1) - arc_code.R * sin(angle0));
        code_array.Y = arc_code.Y0 + (arc_code.R * cos(angle0) - arc_code.R * cos(angle1));
      } else {
        code_array.X = arc_code.X0 + (arc_code.R * cos(angle0) - arc_code.R * cos(angle1));
        code_array.Y = arc_code.Y0 + (arc_code.R * sin(angle1) - arc_code.R * sin(angle0));
      }
      code_array.OmitF = false;
      code_array.F = over_burn_speed;
      arc_codes.push_back(code_array);
      code_array.X0 = code_array.X;
      code_array.Y0 = code_array.Y;
    }
    code_array.Name = M29;
    arc_codes.push_back(code_array);
    if (arc_code.Y0 < arc_code.J) {
      code_array.Name = G02;
      code_array.X = code_array.X0 - (arc_code.R * sin(angle2) - arc_code.R * sin(angle1));
      code_array.Y = code_array.Y0 + (arc_code.R * cos(angle1) - arc_code.R * cos(angle2));
    } else {
      code_array.Name = G02;
      code_array.X = code_array.X0 + (arc_code.R * cos(angle1) - arc_code.R * cos(angle2));
      code_array.Y = code_array.Y0 + (arc_code.R * sin(angle2) - arc_code.R * sin(angle1));
    }
    code_array.OmitF = false;
    code_array.F = over_burn_speed;
    arc_codes.push_back(code_array);
  } else if (arc_code.Name == G03) {
    if (angle1 > 0) {
      if (arc_code.Y0 < arc_code.J) {
        code_array.X = arc_code.X0 + (arc_code.R * sin(angle1) - arc_code.R * sin(angle0));
        code_array.Y = arc_code.Y0 + (arc_code.R * cos(angle0) - arc_code.R * cos(angle1));
      } else {
        code_array.X = arc_code.X0 - (arc_code.R * cos(angle0) - arc_code.R * cos(angle1));
        code_array.Y = arc_code.Y0 + (arc_code.R * sin(angle1) - arc_code.R * sin(angle0));
      }
      code_array.OmitF = false;
      code_array.F = over_burn_speed;
      arc_codes.push_back(code_array);
      code_array.X0 = code_array.X;
      code_array.Y0 = code_array.Y;
    }
    code_array.Name = M29;
    arc_codes.push_back(code_array);
    if (arc_code.Y0 < arc_code.J) {
      code_array.Name = G03;
      code_array.X = code_array.X0 + (arc_code.R * sin(angle2) - arc_code.R * sin(angle1));
      code_array.Y = code_array.Y0 + (arc_code.R * cos(angle1) - arc_code.R * cos(angle2));
    } else {
      code_array.Name = G03;
      code_array.X = code_array.X0 - (arc_code.R * cos(angle1) - arc_code.R * cos(angle2));
      code_array.Y = code_array.Y0 + (arc_code.R * sin(angle2) - arc_code.R * sin(angle1));
    }
    code_array.OmitF = false;
    code_array.F = over_burn_speed;
    arc_codes.push_back(code_array);
  }
  return arc_codes;
}
