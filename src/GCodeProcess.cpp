// Copyright 2016 Fangling Software Co., Ltd. All Rights Reserved.
// Author: shizhan-shelly@hotmail.com (Zhan Shi)

#include "GCodeProcess.h"

#include "GCodeParse.h"
#include "CircleHoleCodeRebuild.h"
#include "HyperthermCircleHoleCodeRebuild.h"
#include "HyperthermWaistHoleCodeRebuild.h"
#include "WaistHoleCodeRebuild.h"
#include "math/Arc.h"
#include "math/D_Point.h"
#include "math/Line.h"
#include "math/mymath.h"
#include "GCodeRestruct.h"

extern GCodeRestruct theApp;

using namespace math;

GCodeProcess::GCodeProcess() {}

GCodeProcess::~GCodeProcess() {}

void GCodeProcess::GCodeRebuild(const std::string &file_name,
    double cutting_kerf_quality, double cutting_speed_quality,
    double thickness) {

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
          theApp.GetKjellbergProcessParameter(g_code[i].R * 2, speed_hole_,
              lead_in_speed_, overburn_speed_, kerf_hole_, asynchronous_stop_,
              US_, PA_);

          CircleHoleCodeRebuild circle_build;
          circle_build.RebuildCircleHoleCode(rebuild_codes,
              g_code, g_code[i].LineNoInTotalFile, kerf_hole_, speed_hole_,
              lead_in_speed_, overburn_speed_,
              US_, asynchronous_stop_);

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
          theApp.GetKjellbergProcessParameter(radius * 2, speed_hole_,
              lead_in_speed_, overburn_speed_, kerf_hole_, asynchronous_stop_,
              US_, PA_);

          WaistHoleCodeRebuild waist_build;
          waist_build.RebuildWaistHoleCode(rebuild_codes,
              g_code, g_code[i].LineNoInTotalFile, kerf_hole_, speed_hole_,
              lead_in_speed_, overburn_speed_,
              US_, asynchronous_stop_);

          waist_build.RebuildLeadOutCode(rebuild_codes, g_code, waist_iter->first);
          PostRebuild(g_code, waist_iter->first, i);
          waist_iter++;
          continue;
        }
        waist_iter++;
      }
    }
    OutsideContourKerfRebuild(g_code[i], cutting_kerf_quality);
    OutsideContourSpeedRebuild(g_code[i], cutting_speed_quality);
    rebuild_codes.push_back(g_code[i]);
  }

  std::vector<std::string> code_lines;
  parse.GenerateGCode(rebuild_codes, code_lines);
  parse.WriteGCode(file_name, code_lines);
}

void GCodeProcess::GCodeRebuildHypertherm(const std::string &file_name,
    double outside_contour_kerf, double outside_contour_cut_speed,
    double thickness) {

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
    if (waist_iter != waist_shape.end() && g_code[i].LineNoInTotalFile == waist_iter->first) {
      double radius = GetWaistHoleRadius(g_code, waist_iter->first, waist_iter->second);
      if (IsInsideContour(g_code, waist_iter->first, waist_iter->second)) {
        theApp.GetHyperthermProcessParameter(radius * 2,
            speed_hole_, lead_in_speed_, kerf_hole_,
            asynchronous_stop_, disable_ahc_, US_);

        HyperthermWaistHoleCodeRebuild waist_build;
        if (IsSmallHoleHypertherm(radius, thickness)) {
          PreRebuild(rebuild_codes);
          waist_build.RebuildWaistHoleCode(rebuild_codes,
              g_code, g_code[i].LineNoInTotalFile, kerf_hole_, speed_hole_,
              lead_in_speed_, speed_hole_,
              US_, asynchronous_stop_, disable_ahc_);

          waist_build.RebuildLeadOutCode(rebuild_codes, g_code, waist_iter->first);
          PostRebuild(g_code, waist_iter->first, i);
        } else {
          waist_build.ModifyWaistHoleCode(rebuild_codes,
              g_code, waist_iter->first, waist_iter->second,
              kerf_hole_, speed_hole_, lead_in_speed_, speed_hole_,
              asynchronous_stop_);

          PostRebuild(g_code, waist_iter->first, i);
        }
        waist_iter++;
        continue;
      }
      waist_iter++;
    }
    OutsideContourKerfRebuild(g_code[i], outside_contour_kerf);
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
  SplitCutCode(g_code, cut_code);

  circle_shape.clear();
  waist_shape.clear();
  for (size_t i = 0; i < cut_code.size(); i++) {
    ClosedShapeIntercept(cut_code[i], circle_shape, waist_shape);
  }
}

void GCodeProcess::SplitCutCode(const std::vector<GCodeStruct> &g_code,
    std::vector<std::vector<GCodeStruct> > &cut_code) {

  cut_code.clear();
  for (size_t i = 0; i < g_code.size(); i++) {
    if (g_code[i].Name == M07) {
      std::vector<GCodeStruct> section_codes;
      size_t j = i + 1;
      while (j < g_code.size()) {
        if (g_code[j].Name == M08) {
          break;
        }
        section_codes.push_back(g_code[j]);
        j++;
      }
      cut_code.push_back(section_codes);
      i = j;
    }
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
          !IsEqual(iter->Length, waist_arc_length)) {

        return false;
      } else {
        iter++;
        if (iter == closed_shape.end()) {
          if (!IsEqual(first_line_len, second_line_len)) {
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
                return IsEqual(first_line_len, second_line_len);
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
          !IsEqual(iter->Length, waist_arc_length)) {

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
          return IsEqual(first_line_len, second_line_len);
        }
      }
      return false;
    }
  }
	return true;
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
  return IsLesser(diameter, 2 * thickness + accuracy) && IsGreater(diameter, 0.8 * thickness - accuracy)
      || IsEqual(diameter, 2 * thickness + accuracy) || IsEqual(diameter, 0.8 * thickness - accuracy);

}

bool GCodeProcess::IsSmallHoleHypertherm(double radius, double thickness) {
  static const double accuracy = 0.01;
  double diameter = 2 * radius;
  return IsLesser(diameter, 2.5 * thickness + accuracy) && IsGreater(diameter, thickness - accuracy)
      || IsEqual(diameter, 2.5 * thickness + accuracy) || IsEqual(diameter, thickness - accuracy);

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
    if (g_code.OmitF) {
      g_code.OmitF = false;
      g_code.F = cutting_speed;
    }
  }
}

void GCodeProcess::OutsideContourKerfRebuild(GCodeStruct &g_code,
                                             double cutting_kerf) {

  if (g_code.Name == G41 || g_code.Name == G42) {
    g_code.OmitKerf = false;
    g_code.KerfValue = cutting_kerf / 2;
  }
}

void GCodeProcess::CloseArcVolProcess(std::vector<GCodeStruct> &segment_code,
                                      double advance_dis) {

  double remain_length = advance_dis;
  std::vector<GCodeStruct>::iterator i = segment_code.end() - 1;
  for (double sum_length = 0.; i > segment_code.begin(); i--) {
    remain_length = advance_dis - sum_length;
    if (!IsLesser(i->Length, remain_length)) {
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
                                  double break_arc_time,
                                  double global_speed, bool F_forbid) {

  if (IsEqual(break_arc_time, 0)) {
    process_code = g_code;
    return ;
  }
  double break_arc_dis = global_speed * break_arc_time / 60;
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

void GCodeProcess::ForbidTHCProcess(const std::vector<GCodeStruct> &g_code,
                                    std::vector<GCodeStruct> &process_code,
                                    double forbid_thc_distance,
                                    double forbid_thc_speed_percent,
                                    double global_speed, bool F_forbid) {

  if (forbid_thc_distance > 0) {
    for (size_t i = 0; i < g_code.size(); i++) {
      GCodeStruct cur_code = g_code[i];
      double x;
      double y;
      double x2;
      double y2;
      if (cur_code.Name == G01) {
        math::Line line(cur_code.X0, cur_code.Y0,
            cur_code.X, cur_code.Y,
            MIN(cur_code.X0, cur_code.X),
            MAX(cur_code.X0, cur_code.X));

        if (IsGreater(cur_code.Length, 2.0 * forbid_thc_distance)) {
          line.Calc(x, y, cur_code.X0, cur_code.Y0, cur_code.X, cur_code.Y, forbid_thc_distance); // calc the end point at disable THC
          line.Calc(x2, y2, cur_code.X0, cur_code.Y0, cur_code.X, cur_code.Y, cur_code.Length - forbid_thc_distance); // calc the start point at enable THC
        } else {
          GCodeStruct M46_code = cur_code;
          M46_code.Name = M46;
          M46_code.X = M46_code.X0;
          M46_code.Y = M46_code.Y0;
          process_code.push_back(M46_code);
          process_code.push_back(cur_code);
          continue;
        }
      } else if (cur_code.Name == G02 || cur_code.Name == G03) {
        math::Arc arc(cur_code.I, cur_code.J, cur_code.R,
            cur_code.X0, cur_code.Y0, cur_code.X, cur_code.Y,
            cur_code.Name == G02 ? math::CW : math::CCW);

        if (IsGreater(cur_code.Length, 2.0 * forbid_thc_distance)) {
          arc.Calc(x, y,  forbid_thc_distance); // calc the end point at disable THC
          arc.Calc(x2, y2, cur_code.Length - forbid_thc_distance); // calc the start point at enable THC
        } else {
          GCodeStruct M46_code = cur_code;
          M46_code.Name = M46;
          M46_code.X = M46_code.X0;
          M46_code.Y = M46_code.Y0;
          process_code.push_back(M46_code);
          process_code.push_back(cur_code);
          continue;
        }
      } else {
        process_code.push_back(cur_code);
        continue;
      }
      GCodeStruct insert_code = cur_code;
      insert_code.X = x2;
      insert_code.Y = y2;
      process_code.push_back(insert_code);

      insert_code.X0 = x2;
      insert_code.Y0 = y2;
      insert_code.X = x2;
      insert_code.Y = y2;
      insert_code.Name = M47;
      process_code.push_back(insert_code);

      insert_code = cur_code;
      insert_code.X0 = x2;
      insert_code.Y0 = y2;
      insert_code.X = x;
      insert_code.Y = y;
      process_code.push_back(insert_code);

      insert_code.X0 = x;
      insert_code.Y0 = y;
      insert_code.X = x;
      insert_code.Y = y;
      insert_code.Name = M46;
      process_code.push_back(insert_code);

      insert_code = cur_code;
      insert_code.X0 = x;
      insert_code.Y0 = y;
      process_code.push_back(insert_code);
    }
  } else {
    process_code = g_code;
  }
}
