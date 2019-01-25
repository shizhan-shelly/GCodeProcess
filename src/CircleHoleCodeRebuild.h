// Copyright 2016 Fangling Software Co., Ltd. All Rights Reserved.
// Author: shizhan-shelly@hotmail.com (Zhan Shi)

#ifndef GCODE_CIRCLEHOLECODEREBUILD_H__
#define GCODE_CIRCLEHOLECODEREBUILD_H__

#include "HoleCodeRebuild.h"

class CircleHoleCodeRebuild : public HoleCodeRebuild {
 public:
  CircleHoleCodeRebuild();
  ~CircleHoleCodeRebuild();

  void RebuildCircleHoleCode(std::vector<GCodeStruct> &rebuild_codes,
      const std::vector<GCodeStruct> &g_code, size_t circle_index,
      double kerf_hole, double speed_hole,
      double lead_in_speed, double over_burn_speed,
      double US, double asynchronous_stop);

}; // class CircleHoleCodeRebuild

#endif // GCODE_CIRCLEHOLECODEREBUILD_H__