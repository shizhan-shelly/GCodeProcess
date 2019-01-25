// Copyright 2016 Fangling Software Co., Ltd. All Rights Reserved.
// Author: shizhan-shelly@hotmail.com (Zhan Shi)

#ifndef GCODE_WAISTHOLECODEREBUILD_H__
#define GCODE_WAISTHOLECODEREBUILD_H__

#include "HoleCodeRebuild.h"

class WaistHoleCodeRebuild : public HoleCodeRebuild {
 public:
  WaistHoleCodeRebuild();
  ~WaistHoleCodeRebuild();

  void RebuildWaistHoleCode(std::vector<GCodeStruct> &rebuild_codes,
      const std::vector<GCodeStruct> &g_code, size_t waist_index,
      double kerf_hole, double speed_hole,
      double lead_in_speed, double over_burn_speed,
      double US, double asynchronous_stop);

 private:
  void RebuildCode(std::vector<GCodeStruct> &rebuild_codes,
      const std::vector<GCodeStruct> &g_code, size_t waist_index,
      const GCodeStruct &first_arc, const GCodeStruct &second_arc,
      double kerf_hole, double speed_hole,
      double lead_in_speed, double over_burn_speed,
      double US, double asynchronous_stop);

  std::vector<GCodeStruct> OverburnArcCodes(
      const GCodeStruct &arc_code, double over_burn_speed,
      double US, double asynchronous_stop);

}; // class WaistHoleCodeRebuild

#endif // GCODE_WAISTHOLECODEREBUILD_H__
