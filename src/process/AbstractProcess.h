// Copyright 2017 Fangling Software Co., Ltd. All Rights Reserved.
// Author: shizhan-shelly@hotmail.com (Zhan Shi)

#ifndef GCODE_PROCESS_ABSTRACTPROCESS_H__
#define GCODE_PROCESS_ABSTRACTPROCESS_H__

#include "GCodeDefinition.h"
#include "ProcessVariable.h"

class AbstractProcess {
 public:
  AbstractProcess() {}
  virtual ~AbstractProcess() {}

  virtual AbstractProcess *clone() = 0;

  virtual bool CodeParse(const GCodeStruct &g_code) = 0;

  double GetVariableValue(VariableType type) const;

}; // class AbstractProcess

#endif // GCODE_PROCESS_ABSTRACTPROCESS_H__
