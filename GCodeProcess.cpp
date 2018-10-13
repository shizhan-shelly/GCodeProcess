// Copyright 2016 Fangling Software Co., Ltd. All Rights Reserved.
// Author: shizhan-shelly@hotmail.com (Zhan Shi)

#include "GCodeProcess.h"

#include "GCodeParse.h"
#include "CircleHoleCodeRebuild.h"
#include "HyperthermCircleHoleCodeRebuild.h"
#include "WaistHoleCodeRebuild.h"
#include "math/Arc.h"
#include "math/D_Point.h"
#include "math/Line.h"
#include "math/mymath.h"
#include "GCodeRestruct.h"

extern GCodeRestruct theApp;

GCodeProcess::GCodeProcess() {}

GCodeProcess::~GCodeProcess() {}

void GCodeProcess::GCodeRebuild(const std::string &file_name,
    double cutting_kerf_quality, double cutting_kerf_hole,
    double cutting_speed_quality, double cutting_speed_hole,
    double thickness, double down_slope,
    std::vector<std::string> &code_lines) {

  GCodeParse parse;
  std::vector<std::string> src_lines;
  parse.ReadGCode(file_name, src_lines);
  std::vector<GCodeStruct> g_code;
  parse.ParseGCode(src_lines, g_code);

  std::vector<size_t> circle_shape;
  std::map<size_t, size_t> waist_shape;
  GCodeInvestigate(g_code, circle_shape, waist_shape);

  std::vector<size_t>::iterator circle_iter = circle_shape.begin();
  std::map<size_t, size_t>::iterator waist_iter = waist_shape.begin();

  std::vector<GCodeStruct> rebuild_codes;
  for (size_t i = 0; i < g_code.size(); i++) {
    if (circle_iter != circle_shape.end()) {
      if (g_code[i].LineNoInTotalFile == *circle_iter) {
        if (IsInsideContour(g_code, *circle_iter, 1) &&
            IsSmallHole(g_code[i].R, thickness)) {

          PreRebuild(rebuild_codes);
          Calculate(cutting_kerf_quality, cutting_kerf_hole, cutting_speed_quality,
              cutting_speed_hole, g_code[i].R * 2, thickness, down_slope, kerf_hole_,
              speed_hole_, US_, PA_);

          CircleHoleCodeRebuild circle_build;
          circle_build.RebuildCircleHoleCode(rebuild_codes,
              g_code, g_code[i].LineNoInTotalFile, kerf_hole_, speed_hole_,
              speed_hole_, speed_hole_,
              US_, PA_);

          circle_build.RebuildLeadOutCode(rebuild_codes, g_code, *circle_iter);
          PostRebuild(g_code, *circle_iter, i);
          circle_iter++;
          continue;
        }
        circle_iter++;
      }
    }
    if (waist_iter != waist_shape.end()) {
      if (g_code[i].LineNoInTotalFile == waist_iter->first) {
        double radius = GetWaistHoleRadius(g_code, waist_iter->first, waist_iter->second);
        if (IsInsideContour(g_code, waist_iter->first, waist_iter->second) &&
            IsSmallHole(radius, thickness)) {

          PreRebuild(rebuild_codes);
          Calculate(cutting_kerf_quality, cutting_kerf_hole, cutting_speed_quality,
              cutting_speed_hole, 2 * radius, thickness, down_slope, kerf_hole_,
              speed_hole_, US_, PA_);

          WaistHoleCodeRebuild waist_build;
          waist_build.RebuildWaistHoleCode(rebuild_codes,
              g_code, g_code[i].LineNoInTotalFile, kerf_hole_, speed_hole_,
              speed_hole_, speed_hole_,
              US_, PA_);

          waist_build.RebuildLeadOutCode(rebuild_codes, g_code, waist_iter->first);
          PostRebuild(g_code, waist_iter->first, i);
          waist_iter++;
          continue;
        }
        waist_iter++;
      }
    }
    OutsideContourSpeedRebuild(g_code[i], cutting_speed_quality);
    rebuild_codes.push_back(g_code[i]);
  }

  parse.GenerateGCode(rebuild_codes, code_lines);
}

void GCodeProcess::GCodeRebuildHypertherm(const std::string &file_name,
    double outside_contour_cut_speed, double thickness) {

  GCodeParse parse;
  std::vector<std::string> src_lines;
  parse.ReadGCode(file_name, src_lines);
  std::vector<GCodeStruct> g_code;
  parse.ParseGCode(src_lines, g_code);

  std::vector<size_t> circle_shape;
  std::map<size_t, size_t> waist_shape;
  GCodeInvestigate(g_code, circle_shape, waist_shape);

  std::vector<size_t>::iterator circle_iter = circle_shape.begin();
  std::map<size_t, size_t>::iterator waist_iter = waist_shape.begin();

  std::vector<GCodeStruct> rebuild_codes;
  for (size_t i = 0; i < g_code.size(); i++) {
    if (circle_iter != circle_shape.end() && g_code[i].LineNoInTotalFile == *circle_iter) {
      if (IsInsideContour(g_code, *circle_iter, 1)) {
        double hole_kerf;
        double hole_speed;
        double lead_in_speed;
        double asynchronous_stop;
        int disable_ahc;
        double US;
        theApp.GetHyperthermProcessParameter(g_code[i].R * 2,
            hole_speed, lead_in_speed, hole_kerf,
            asynchronous_stop, disable_ahc, US);

        HyperthermCircleHoleCodeRebuild circle_build;
        if (IsSmallHoleHypertherm(g_code[i].R, thickness)) {
          PreRebuild(rebuild_codes);
          circle_build.RebuildCircleHoleCode(rebuild_codes,
              g_code, g_code[i].LineNoInTotalFile,
              hole_kerf, hole_speed,
              lead_in_speed, hole_speed,
              US, asynchronous_stop, disable_ahc);

          circle_build.RebuildLeadOutCode(rebuild_codes, g_code, *circle_iter);
          PostRebuild(g_code, *circle_iter, i);
        } else {
          circle_build.ModifyCircleHoleCode(rebuild_codes,
              g_code, *circle_iter,
              hole_kerf, hole_speed, lead_in_speed, hole_speed,
              asynchronous_stop);

          PostRebuild(g_code, *circle_iter, i);
        }
        circle_iter++;
        continue;
      }
      circle_iter++;
    }
    OutsideContourSpeedRebuild(g_code[i], outside_contour_cut_speed);
    rebuild_codes.push_back(g_code[i]);
  }

  std::vector<std::string> code_lines;
  parse.GenerateGCode(rebuild_codes, code_lines);
  parse.WriteGCode(file_name, code_lines);
}

void GCodeProcess::GCodeInvestigate(const std::vector<GCodeStruct> &g_code,
                                    std::vector<size_t> &circle_shape,
                                    std::map<size_t, size_t> &waist_shape) {

  std::vector<std::vector<GCodeStruct> > cut_code;
  GCodeParse::SplitCutCode(g_code, cut_code);

  circle_shape.clear();
  waist_shape.clear();
  for (size_t i = 0; i < cut_code.size(); i++) {
    ClosedShapeIntercept(cut_code[i], circle_shape, waist_shape);
  }
}

void GCodeProcess::ClosedShapeIntercept(const std::vector<GCodeStruct> &cut_code,
                                        std::vector<size_t> &circle_shape,
                                        std::map<size_t, size_t> &waist_shape) {

  std::vector<size_t> motion_index = GetMotionIndex(cut_code);
  if (motion_index.empty()) {
    return;
  }

	std::vector<size_t> trace_index;
  for (size_t i = 0; i < motion_index.size(); i++) {
    trace_index.push_back(motion_index[i]);
    std::vector<GCodeStruct> closed_shape = ClosedShapeMotionCode(cut_code,
	                                                                trace_index);

    if (!closed_shape.empty()) {
      if (IsCirCleShape(closed_shape)) {
        circle_shape.push_back(closed_shape.front().LineNoInTotalFile);
	    } else if (IsWaistShape(closed_shape)) {
        waist_shape.insert(std::make_pair(closed_shape.front().LineNoInTotalFile,
            closed_shape.size()));

	    }
	    trace_index.clear();
	  }
  }
}

std::vector<GCodeStruct> GCodeProcess::ClosedShapeMotionCode(
    const std::vector<GCodeStruct> &cut_code,
    const std::vector<size_t> &trace_index) {

  if (cut_code.empty() || trace_index.empty()) {
    return std::vector<GCodeStruct>();
  }

  std::vector<GCodeStruct> closed_shape;
  D_Point end_point = D_Point(cut_code[trace_index.back()].X,
                              cut_code[trace_index.back()].Y);

  for (size_t i = trace_index.size() - 1; i >= 0; i--) {
    D_Point start_point = D_Point(cut_code[trace_index[i]].X0,
                                  cut_code[trace_index[i]].Y0);

    if (D_Point::IsSamePoint(start_point, end_point)) {
      for (size_t j = i; j < trace_index.size(); j++) {
        closed_shape.push_back(cut_code[trace_index[j]]);
			}
	    return closed_shape;
		}
    if (i == 0) {
      break;
    }
  }
  return closed_shape;
}

std::vector<size_t> GCodeProcess::GetMotionIndex(
	  const std::vector<GCodeStruct> &cut_code) {

  std::vector<size_t> motion_index;
  for (size_t i = 0; i < cut_code.size(); i++) {
    if (IsMotionCode(cut_code[i])) {
      motion_index.push_back(i);
    }
  }
  return motion_index;
}

bool GCodeProcess::IsMotionCode(const GCodeStruct &g_code) {
  return g_code.Name == G00 || g_code.Name == G01 || g_code.Name == G02 ||
      g_code.Name == G03;

}

bool GCodeProcess::IsCirCleShape(const std::vector<GCodeStruct> &closed_shape) {
	if (closed_shape.size() != 1) {
		return false;
	}
  return closed_shape.front().Name == G02 || closed_shape.front().Name == G03;
}

bool GCodeProcess::IsWaistShape(const std::vector<GCodeStruct> &closed_shape) {
	if (closed_shape.size() < 4) {
		return false;
	}
	unsigned short direction_name;
  double first_line_len = 0.;
  double second_line_len = 0.;
  double waist_arc_length = 0.;
	std::vector<GCodeStruct>::const_iterator iter = closed_shape.begin();
  if (iter->Name == G01) { // begin of line
    while (iter->Name == G01) {
      first_line_len += iter->Length;
      iter++;
      if (iter == closed_shape.end()) {
        return false;
      }
    }
    if (iter->Name != G02 && iter->Name != G03) {
      return false;
    } else {
      direction_name = iter->Name;
      waist_arc_length = iter->Length;
      iter++;
      if (iter == closed_shape.end()) {
        return false;
      }
    }
    if (iter->Name != G01) {
      return false;
    } else {
      while (iter->Name == G01) {
        second_line_len += iter->Length;
        iter++;
        if (iter == closed_shape.end()) {
          return false;
        }
      }
    }
    if (iter->Name != G02 && iter->Name != G03) {
      return false;
    } else {
      if (iter->Name != direction_name ||
          !math::IsEqual(iter->Length, waist_arc_length)) {

        return false;
      } else {
        iter++;
        if (iter == closed_shape.end()) {
          if (!math::IsEqual(first_line_len, second_line_len)) {
            return false;
          }
        } else {
          if (iter->Name != G01) {
            return false;
          } else {
            while (iter->Name == G01) {
              first_line_len += iter->Length;
              iter++;
              if (iter == closed_shape.end()) {
                return math::IsEqual(first_line_len, second_line_len);
              }
            }
            return false;
          }
        }
      }
    }
	} else { // begin of half-circle
    if (iter->Name != G02 && iter->Name != G03) {
      return false;
    } else {
      direction_name = iter->Name;
      waist_arc_length = iter->Length;
      iter++;
      if (iter == closed_shape.end()) {
        return false;
      }
    }
    if (iter->Name != G01) {
      return false;
    } else {
      while (iter->Name == G01) {
        first_line_len += iter->Length;
        iter++;
        if (iter == closed_shape.end()) {
          return false;
        }
      }
    }
    if (iter->Name != G02 && iter->Name != G03) {
      return false;
    } else {
      if (iter->Name != direction_name ||
          !math::IsEqual(iter->Length, waist_arc_length)) {

        return false;
      } else {
        iter++;
        if (iter == closed_shape.end()) {
          return false;
        }
      }
    }
    if (iter->Name != G01) {
      return false;
    } else {
      while (iter->Name == G01) {
        second_line_len += iter->Length;
        iter++;
        if (iter == closed_shape.end()) {
          return math::IsEqual(first_line_len, second_line_len);
        }
      }
      return false;
    }
  }
	return true;
}

void GCodeProcess::Calculate(double cutting_kerf_quality, double cutting_kerf_hole,
    double cutting_speed_quality, double cutting_speed_hole, 
    double hole_diameter, double thickness, double down_slope,
    double &cutting_kerf_hole_r, double &cutting_speed_hole_r,
    double &US, double &PA) {

  double N2 = hole_diameter / thickness;
  double N8 = cutting_kerf_hole;
  double N17 = cutting_kerf_quality;
  double N18 = N17 - N8;
  double N19 = ((N2 - 1) * N18) + N8;
  double N20 = ((0.8 - 1) * N18) + N8;
  if (N2 <0.8) {
    cutting_kerf_hole_r = N20;
  } else {
    if (N2 > 2) {
      cutting_kerf_hole_r = N17;
    } else {
      cutting_kerf_hole_r = N19;
    }
  }
  double N3 = cutting_speed_hole;
  double N4 = cutting_speed_quality;
  double N5 = N4- N3;
  double N6 = ((0.8 - 1) * N5) + N3;
  double O2 = ((N2 - 1) * N5) + N3;
  if (N2 < 0.8) {
    cutting_speed_hole_r = N6;
  } else {
    if (N2 > 2) {
      cutting_speed_hole_r = N4;
    } else {
      cutting_speed_hole_r = O2;
    }
  }
  down_slope *= 1000; // unit: ms
  double DS = (down_slope / 60) * (cutting_speed_hole_r / 1000);
  double D33 = down_slope + 50;
  US = DS + (1.5 * cutting_kerf_hole);
  PA = (cutting_speed_hole_r / 60) * (D33 / 1000);
}

double GCodeProcess::GetWaistHoleRadius(const std::vector<GCodeStruct> &g_code,
    size_t begin_index, size_t code_count) {

  for (size_t i = begin_index; i < begin_index + code_count; i++) {
    if (i >= g_code.size()) {
      return 0.;
    }
    if (g_code[i].Name == G02 || g_code[i].Name == G03) {
      return g_code[i].R;
    }
  }
  return 0.;
}

bool GCodeProcess::IsInsideContour(const std::vector<GCodeStruct> &g_code,
    size_t begin_index, size_t code_count) {

  std::vector<GCodeStruct>::const_iterator iter = g_code.begin() + begin_index;
  for (; iter < g_code.begin() + begin_index + code_count; iter++) {
    if (iter == g_code.end()) {
      return false;
    }
    if (iter->Name == G02 && iter->KerfDir == 2
        || iter->Name == G03 && iter->KerfDir == 1) {

      return true;
    }
  }
  return false;
}

bool GCodeProcess::IsSmallHole(double radius, double thickness) {
  static const double accuracy = 0.01;
  double diameter = 2 * radius;
  return math::IsLesser(diameter, 2 * thickness + accuracy) && math::IsGreater(diameter, 0.8 * thickness - accuracy)
      || math::IsEqual(diameter, 2 * thickness + accuracy) || math::IsEqual(diameter, 0.8 * thickness - accuracy);

}

bool GCodeProcess::IsSmallHoleHypertherm(double radius, double thickness) {
  static const double accuracy = 0.01;
  double diameter = 2 * radius;
  return math::IsLesser(diameter, 2.5 * thickness + accuracy) && math::IsGreater(diameter, thickness - accuracy)
      || math::IsEqual(diameter, 2.5 * thickness + accuracy) || math::IsEqual(diameter, thickness - accuracy);

}

void GCodeProcess::PreRebuild(std::vector<GCodeStruct> &rebuild_codes) {
  std::vector<GCodeStruct>::iterator iter = rebuild_codes.end() - 1;
  std::vector<GCodeStruct>::iterator ptr = rebuild_codes.begin();
  while (iter > rebuild_codes.begin()) {
    if (iter->Name == G00) {
      ptr = iter;
      break;
    }
    iter--;
  }
  rebuild_codes.erase(ptr, rebuild_codes.end());
}

void GCodeProcess::PostRebuild(const std::vector<GCodeStruct> &g_code,
    size_t begin_index, size_t &next_index) {

  next_index = g_code.size() - 1;
  for (size_t i = begin_index; i < g_code.size(); i++) {
    if (g_code[i].Name == G00) {
      next_index = i;
      break;
    }
  }
}

void GCodeProcess::OutsideContourSpeedRebuild(GCodeStruct &g_code,
                                              double cutting_speed) {

  if (g_code.Name == G01 || g_code.Name == G02 || g_code.Name == G03) {
    g_code.OmitF = false;
    g_code.F = cutting_speed;
  }
}

void GCodeProcess::CloseArcVolProcess(std::vector<GCodeStruct> &segment_code,
                                      double advance_dis) {

  double remain_length = advance_dis;
  std::vector<GCodeStruct>::iterator i = segment_code.end() - 1;
  for (double sum_length = 0.; i > segment_code.begin(); i--) {
    remain_length = advance_dis - sum_length;
    if (!math::IsLesser(i->Length, remain_length)) {
      break;
    }
    sum_length += i->Length;
  }
  if (i == segment_code.begin()) {
    return ;
  }
  double x;
  double y;
  if (i->Name == G01) {
    math::Line line(i->X0, i->Y0, i->X, i->Y, MIN(i->X0, i->X), MAX(i->X0, i->X));
    if (!line.Calc(x, y, i->X0, i->Y0, i->X, i->Y, remain_length)) {
      return ;
    }
  } else if (i->Name == G02 || i->Name == G03) {
    math::Arc arc(i->I, i->J, i->R, i->X0, i->Y0, i->X, i->Y,
        i->Name == G02 ? math::CW : math::CCW);

    if (arc.Calc(x, y, remain_length) == -1) {
      return ;
    }
  } else {
    return ;
  }
  GCodeStruct insert_code = *i;
  i->X = x;
  i->Y = y;
  insert_code.X0 = i->X;
  insert_code.Y0 = i->Y;
  GCodeStruct M29_code = *i;
  M29_code.Name = M29;
  M29_code.X0 = i->X;
  M29_code.Y0 = i->Y;
  i = segment_code.insert(i + 1, insert_code);
  segment_code.insert(i, M29_code);
}

void GCodeProcess::BreakArcPocess(const std::vector<GCodeStruct> &g_code,
                                  std::vector<GCodeStruct> &process_code,
                                  double break_arc_time, double cut_speed) {

  if (math::IsEqual(break_arc_time, 0)) {
    process_code = g_code;
    return ;
  }
  double break_arc_dis = cut_speed * break_arc_time / 60;
  std::vector<GCodeStruct> segment_code;
  process_code.clear();
  std::vector<GCodeStruct>::const_iterator iter = g_code.begin();
  while (iter != g_code.end()) {
    if (iter->Name != G00) {
      segment_code.push_back(*iter);
    } else {
      if (!segment_code.empty()) {
        CloseArcVolProcess(segment_code, break_arc_dis);
        process_code.insert(process_code.end(), segment_code.begin(), segment_code.end());
        segment_code.clear();
      }
      process_code.push_back(*iter);
    }
    iter++;
    if (iter->Name == M02) {
      break;
    }
  }
  if (!segment_code.empty()) {
    CloseArcVolProcess(segment_code, break_arc_dis);
    process_code.insert(process_code.end(), segment_code.begin(), segment_code.end());
    segment_code.clear();
  }
}
