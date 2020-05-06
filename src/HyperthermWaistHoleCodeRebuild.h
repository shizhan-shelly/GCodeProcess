// Copyright 2020 Fangling Software Co., Ltd. All Rights Reserved.
// Author: shizhan-shelly@hotmail.com (Zhan Shi)

#ifndef GCODE_HYPERTHERMWAISTHOLECODEREBUILD_H__
#define GCODE_HYPERTHERMWAISTHOLECODEREBUILD_H__

#include "HoleCodeRebuild.h"

class HyperthermWaistHoleCodeRebuild : public HoleCodeRebuild {
 public:
  HyperthermWaistHoleCodeRebuild();
  ~HyperthermWaistHoleCodeRebuild();

  void RebuildWaistHoleCode(std::vector<GCodeStruct> &rebuild_codes,
      const std::vector<GCodeStruct> &g_code, size_t waist_index,
      double kerf_hole, double speed_hole,
      double lead_in_speed, double over_burn_speed,
      double US, double asynchronous_stop, int disable_ahc);

  void ModifyWaistHoleCode(std::vector<GCodeStruct> &rebuild_codes,
      const std::vector<GCodeStruct> &g_code,
      size_t waist_start, size_t waist_code_count,
      double kerf_hole, double speed_hole,
      double lead_in_speed, double over_burn_speed,
      double asynchronous_stop);

 private:
  void RebuildCode(std::vector<GCodeStruct> &rebuild_codes,
      const std::vector<GCodeStruct> &g_code, size_t waist_index,
      const GCodeStruct &first_arc, const GCodeStruct &second_arc,
      double kerf_hole, double speed_hole,
      double lead_in_speed, double over_burn_speed,
      double US, double asynchronous_stop, int disable_ahc);

  std::vector<GCodeStruct> OverburnArcCodes(
      const GCodeStruct &arc_code, double over_burn_speed,
      double US, double asynchronous_stop, int disable_ahc);

}; // class HyperthermWaistHoleCodeRebuild

#endif // GCODE_HYPERTHERMWAISTHOLECODEREBUILD_H__
