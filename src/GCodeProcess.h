// Copyright 2016 Fangling Software Co., Ltd. All Rights Reserved.
// Author: shizhan-shelly@hotmail.com (Zhan Shi)

#ifndef GCODE_GCODEPROCESS_H__
#define GCODE_GCODEPROCESS_H__

#include <map>
#include <vector>

#include "GCodeDefinition.h"

/**
 * G Code Restructuring Algorithm for Smart Focus Incise
 */
class GCodeProcess {
 public:
	GCodeProcess();
	~GCodeProcess();

  /**
   *target: Find circle and waist-shaped hole.
   *@param [in]: g_code, source g code to be process.
   *@param [in/out]: circle_shape, code's line number of circle
   *@param [in/out]: waist_shape, first code's line number of waist-shape hole 
   */
  void GCodeInvestigate(const std::vector<GCodeStruct> &g_code,
                        std::vector<size_t> &circle_shape,
                        std::map<size_t, size_t> &waist_shape);

  void GCodeRebuild(const std::string &file_name,
      double cutting_kerf_quality, double cutting_kerf_hole,
      double cutting_speed_quality, double cutting_speed_hole,
      double thickness, double down_slope);

  void GCodeRebuildHypertherm(const std::string &file_name,
      double outside_contour_cut_speed, double thickness);

  void BreakArcPocess(const std::vector<GCodeStruct> &g_code,
                      std::vector<GCodeStruct> &process_code,
                      double break_arc_time, double global_speed, bool F_forbid);

  void ForbidTHCProcess(const std::vector<GCodeStruct> &g_code,
                        std::vector<GCodeStruct> &process_code,
                        double forbid_thc_distance,
                        double forbid_thc_speed_percent,
                        double global_speed, bool F_forbid);

 private:
  /**
   * Find out the circle-shape code's line number and waist-shape code's line
   * number from one section of cut code.
   */
  void ClosedShapeIntercept(const std::vector<GCodeStruct> &cut_code,
                            std::vector<size_t> &circle_shape,
                            std::map<size_t, size_t> &waist_shape);

  std::vector<GCodeStruct> ClosedShapeMotionCode(
      const std::vector<GCodeStruct> &cut_code,
      const std::vector<size_t> &trace_index);

  std::vector<size_t> GetMotionIndex(const std::vector<GCodeStruct> &cut_code);

  bool IsMotionCode(const GCodeStruct &g_code);

  bool IsCirCleShape(const std::vector<GCodeStruct> &closed_shape);

  bool IsWaistShape(const std::vector<GCodeStruct> &closed_shape);

  double GetWaistHoleRadius(const std::vector<GCodeStruct> &g_code,
      size_t begin_index, size_t code_count);

  bool IsInsideContour(const std::vector<GCodeStruct> &g_code,
      size_t begin_index, size_t code_count);

  bool IsSmallHole(double radius, double thickness);

  bool IsSmallHoleHypertherm(double radius, double thickness);

  void OutsideContourSpeedRebuild(GCodeStruct &g_code, double cutting_speed);

  // Only before process small hole contour, pop up the lead-in code of inside
  // until the G00 code. Only after process small hole contour, throw away the
  // lead-out code of inside until the G00 code.
  void PreRebuild(std::vector<GCodeStruct> &rebuild_codes);
  void PostRebuild(const std::vector<GCodeStruct> &g_code,
      size_t begin_index, size_t &next_index);

  void CloseArcVolProcess(std::vector<GCodeStruct> &segment_code,
                          double advance_dis);

  double kerf_hole_;
  double speed_hole_;
  double lead_in_speed_;
  double overburn_speed_;
  double US_;
  double PA_;

}; // class GCodeProcess

#endif // GCODE_GCODEPROCESS_H__
