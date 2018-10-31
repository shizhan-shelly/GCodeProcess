// Copyright 2016 Fangling Software Co., Ltd. All Rights Reserved.
// Author: shizhan-shelly@hotmail.com (Zhan Shi)

#include "HoleCodeRebuild.h"

#include "math/mymath.h"

HoleCodeRebuild::HoleCodeRebuild() : lead_in_type_(ArcLeadIn) {}

HoleCodeRebuild::~HoleCodeRebuild() {}

double HoleCodeRebuild::GetOverburnArcAngle(double arc_length,
                                            double radius) {

  if (math::IsZero(radius)) {
    return 0.;
  }
  return arc_length / radius;
}

D_Point HoleCodeRebuild::LeadInPoint(const std::vector<GCodeStruct> &g_code,
                                     size_t begin_index) {

  std::vector<GCodeStruct>::const_iterator ptr = g_code.begin() + begin_index;
  std::vector<GCodeStruct>::const_iterator iter = g_code.begin();
  for (; ptr > g_code.begin(); ptr--) {
    if (ptr->Name == G00) {
      iter = ptr;
      break;
    }
  }
  return D_Point(iter->X0, iter->Y0);
}

D_Point HoleCodeRebuild::LeadOutPoint(const std::vector<GCodeStruct> &g_code,
                                      size_t begin_index) {

  std::vector<GCodeStruct>::const_iterator ptr = g_code.begin() + begin_index;
  std::vector<GCodeStruct>::const_iterator iter = g_code.end() - 1;
  for (; ptr < g_code.end(); ptr++) {
    if (ptr->Name == G00) {
      iter = ptr;
      break;
    }
  }
  return D_Point(iter->X, iter->Y);
}

void HoleCodeRebuild::RebuildLeadOutCode(
    std::vector<GCodeStruct> &rebuild_codes,
    const std::vector<GCodeStruct> &g_code, size_t begin_index) {

  std::vector<GCodeStruct>::const_iterator ptr = g_code.begin() + begin_index;
  for (; ptr != g_code.end(); ptr++) {
    if (ptr->Name == G00) {
      break;
    }
  }
  if (ptr == g_code.end()) {
    return ;
  }
  GCodeStruct code_array;
  code_array.Name = G00;
  code_array.X0 = rebuild_codes.back().X;
  code_array.Y0 = rebuild_codes.back().Y;
  code_array.X = ptr->X;
  code_array.Y = ptr->Y;
  rebuild_codes.push_back(code_array);
}
