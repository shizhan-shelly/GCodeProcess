// Copyright 2016 Fangling Software Co., Ltd. All Rights Reserved.
// Author: shizhan-shelly@hotmail.com (Zhan Shi)

#ifndef GCODE_HOLECODEREBUILD_H__
#define GCODE_HOLECODEREBUILD_H__

#include <vector>

#include "GCodeDefinition.h"
#include "math/D_Point.h"

typedef enum _LeadInType {
  LineLeadIn,
  ArcLeadIn,

} LeadInType;

class HoleCodeRebuild {
 public:
  HoleCodeRebuild();
  ~HoleCodeRebuild();

  void RebuildLeadOutCode(std::vector<GCodeStruct> &rebuild_codes,
      const std::vector<GCodeStruct> &g_code, size_t begin_index);

 protected:
  double GetOverburnArcAngle(double arc_length, double radius);

  D_Point LeadInPoint(const std::vector<GCodeStruct> &g_code, size_t begin_index);
  D_Point LeadOutPoint(const std::vector<GCodeStruct> &g_code, size_t begin_index);

  LeadInType lead_in_type_;

}; // class HoleCodeRebuild

#endif // GCODE_HOLECODEREBUILD_H__
