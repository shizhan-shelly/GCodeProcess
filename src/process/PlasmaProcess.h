// Copyright 2017 Fangling Software Co., Ltd. All Rights Reserved.
// Author: shizhan-shelly@hotmail.com (Zhan Shi)

#ifndef GCODE_PROCESS_PLASMAPROCESS_H__
#define GCODE_PROCESS_PLASMAPROCESS_H__

#include "AbstractProcess.h"

class PlasmaProcess : public AbstractProcess {
 public:
  PlasmaProcess();
  ~PlasmaProcess();

  virtual AbstractProcess *clone();

  PlasmaProcess(const PlasmaProcess &other);

  bool CodeParse(const GCodeStruct &g_code);

  double plasma_set_arc_voltage_;
  double plasma_pierce_time_;
  double plasma_pierce_height_factor_;
  double plasma_cut_height_;
  double plasma_transfer_height_factor_;
  double plasma_cut_height_delay_;
  double plasma_kerf_detect_reacquire_time_;
  double plasma_mode_select_;
  double plasma_arc_current_;
  double plasma_AVC_delay_;

}; // class PlasmaProcess

#endif // GCODE_PROCESS_PLASMAPROCESS_H__
