// Copyright 2017 Fangling Software Co., Ltd. All Rights Reserved.
// Author: shizhan-shelly@hotmail.com (Zhan Shi)

#include "ProcessFactory.h"

#include "PlasmaProcess.h"

ProcessFactory::ProcessFactory() {}

ProcessFactory::~ProcessFactory() {}

AbstractProcess *ProcessFactory::CreateProcess(ProcessType process_type,
    const std::vector<GCodeStruct> &g_codes) {

  AbstractProcess *process = NULL;
  if (g_codes.empty()) {
    return process;
  }
  switch (process_type) {
   case Plasma1:
   case Plasma2:
    process = new PlasmaProcess();
    break;
   default:
    return NULL;
  }
  for (size_t i = 0; i < g_codes.size(); i++) {
    if (!process->CodeParse(g_codes[i])) {
      delete process;
      process = NULL;
      break;
    }
  }
  return process;
}
