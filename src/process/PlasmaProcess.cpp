// Copyright 2017 Fangling Software Co., Ltd. All Rights Reserved.
// Author: shizhan-shelly@hotmail.com (Zhan Shi)

#include "PlasmaProcess.h"

PlasmaProcess::PlasmaProcess() {}

PlasmaProcess::~PlasmaProcess() {}

AbstractProcess *PlasmaProcess::clone() {
  return new PlasmaProcess(*this);
}

PlasmaProcess::PlasmaProcess(const PlasmaProcess &other) {
  plasma_set_arc_voltage_ = other.plasma_set_arc_voltage_;
  plasma_pierce_time_ = other.plasma_pierce_time_;
  plasma_pierce_height_factor_ = other.plasma_pierce_height_factor_;
  plasma_cut_height_ = other.plasma_cut_height_;
  plasma_transfer_height_factor_ = other.plasma_transfer_height_factor_;
  plasma_cut_height_delay_ = other.plasma_cut_height_delay_;
  plasma_kerf_detect_reacquire_time_ = other.plasma_kerf_detect_reacquire_time_;
  plasma_mode_select_ = other.plasma_mode_select_;
  plasma_arc_current_ = other.plasma_arc_current_;
  plasma_AVC_delay_ = other.plasma_AVC_delay_;
}

bool PlasmaProcess::CodeParse(const GCodeStruct &g_code) {
  if (g_code.Name != G59) {
    return false;
  }
  switch (g_code.VariableType) {
   case 600:
   case 625:
    plasma_set_arc_voltage_ = g_code.VariableValue;
    break;
   case 601:
   case 626:
    plasma_pierce_time_ = g_code.VariableValue;
    break;
   case 602:
   case 627:
    plasma_pierce_height_factor_ = g_code.VariableValue;
    break;
   case 603:
   case 628:
    plasma_cut_height_ = g_code.VariableValue;
    break;
   case 604:
   case 629:
    plasma_transfer_height_factor_ = g_code.VariableValue;
    break;
   case 605:
   case 630:
    plasma_cut_height_delay_ = g_code.VariableValue;
    break;
   case 606:
   case 631:
    plasma_kerf_detect_reacquire_time_ = g_code.VariableValue;
    break;
   case 607:
   case 632:
    plasma_mode_select_ = g_code.VariableValue;
    break;
   case 608:
   case 633:
    plasma_arc_current_ = g_code.VariableValue;
    break;
   case 613:
   case 638:
    plasma_AVC_delay_ = g_code.VariableValue;
    break;
   default:
    return false;
  }
  return true;
}
