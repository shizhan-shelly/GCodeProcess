// Copyright 2017 Fangling Software Co., Ltd. All Rights Reserved.
// Author: shizhan-shelly@hotmail.com (Zhan Shi)

#ifndef GCODE_CURVEFITTING_SIMPLELINEARFITTING_H__
#define GCODE_CURVEFITTING_SIMPLELINEARFITTING_H__

#include <vector>

#include "GCodeDefinition.h"

#define Gfilesize 0x400000

extern char text_wholefile[];

void FitSmallLine(GCodeStruct *Ptr, double min_fit_length);

class SimpleLinearFitting {
 public:
  SimpleLinearFitting();
  ~SimpleLinearFitting();

  void FitSmallLine(std::vector<GCodeStruct> &g_codes, double min_fit_length);

 private:
  char *text_whole_file_;

}; // class SimpleLinearFitting

#endif // GCODE_CURVEFITTING_SIMPLELINEARFITTING_H__
