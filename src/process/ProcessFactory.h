// Copyright 2017 Fangling Software Co., Ltd. All Rights Reserved.
// Author: shizhan-shelly@hotmail.com (Zhan Shi)

#ifndef GCODE_PROCESS_PROCESSFACTORY_H__
#define GCODE_PROCESS_PROCESSFACTORY_H__

#include <vector>

#include "AbstractProcess.h"

class ProcessFactory {
 public:
  ProcessFactory();
  virtual ~ProcessFactory();

  AbstractProcess *CreateProcess(ProcessType process_type,
      const std::vector<GCodeStruct> &g_codes);

  void SetDefaultParam();

}; // class ProcessFactory

#endif // GCODE_PROCESS_PROCESSFACTORY_H__
