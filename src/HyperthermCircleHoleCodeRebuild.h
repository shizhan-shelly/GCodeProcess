// Copyright 2016 Fangling Software Co., Ltd. All Rights Reserved.
// Author: shizhan-shelly@hotmail.com (Zhan Shi)

#ifndef GCODE_HYPERTHERMCIRCLEHOLECODEREBUILD_H__
#define GCODE_HYPERTHERMCIRCLEHOLECODEREBUILD_H__

#include "HoleCodeRebuild.h"

class HyperthermCircleHoleCodeRebuild : public HoleCodeRebuild {
 public:
  HyperthermCircleHoleCodeRebuild();
  ~HyperthermCircleHoleCodeRebuild();

  void RebuildCircleHoleCode(std::vector<GCodeStruct> &rebuild_codes,
      const std::vector<GCodeStruct> &g_code, size_t circle_index,
      double hole_kerf, double hole_speed,
      double lead_in_speed, double over_burn_speed,
      double US, double asynchronous_stop, int disable_ahc);

  void ModifyCircleHoleCode(std::vector<GCodeStruct> &rebuild_codes,
      const std::vector<GCodeStruct> &g_code, size_t circle_index,
      double hole_kerf, double hole_speed,
      double lead_in_speed, double over_burn_speed,
      double asynchronous_stop);

}; // class HyperthermCircleHoleCodeRebuild

#endif // GCODE_HYPERTHERMCIRCLEHOLECODEREBUILD_H__
