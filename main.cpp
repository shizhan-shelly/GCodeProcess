// Copyright 2016 Fangling Software Co., Ltd. All Rights Reserved.
// Author: shizhan-shelly@hotmail.com (Zhan Shi)

#include <iostream>

#include "GCodeParse.h"
#include "GCodeProcess.h"
#include "kerf/Kerf.h"

int main(int argc, char *argv[]) {
  //double cutting_kerf_quality = 2.0;
  //double cutting_speed_quality = 5000;
  //double thickness = 1.5;
  //GCodeProcess code_process;
  //code_process.GCodeRebuild(argv[1], cutting_kerf_quality, cutting_speed_quality,
  //thickness);

  //Kerf kerf;
  //kerf.SetKerfValue(0.95);
  //kerf.GKerfProc("d:\\cutfile.txt", "cutfilekerf.txt");

  std::vector<std::string> code_lines;
  std::vector<GCodeStruct> g_code;
  std::vector<GCodeStruct> process_code;

  GCodeParse parse;
  parse.ReadGCode("d:\\test.txt", code_lines);
  parse.ParseGCode(code_lines, g_code);

  GCodeProcess code_process;
  code_process.BreakArcPocess(g_code, process_code, 5, 4500, true);

  parse.GenerateGCode(process_code, code_lines);
  parse.WriteGCode("result.txt", code_lines);
  return 0;
}
