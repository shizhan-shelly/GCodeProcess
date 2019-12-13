// Copyright 2016 Fangling Software Co., Ltd. All Rights Reserved.
// Author: shizhan-shelly@hotmail.com (Zhan Shi)

#include "GCodeParse.h"

#include <fstream>
#include <string>
#include <cctype>

#include "math/Arc.h"
#include "math/Circle.h"
#include "math/D_Point.h"
#include "math/Line.h"
#include "math/mymath.h"

using namespace math;

static const double FOOT_METRIC_FACTOR = 25.4;

GCodeParse::GCodeParse() : foot_metric_(MetricSystem)
                         , relative_absolute_(Relative)
                         , m07_m08_flag_(false) {}

GCodeParse::~GCodeParse() {}

bool GCodeParse::ReadGCode(const std::string &file_name,
                           std::vector<std::string> &code_lines) {

  std::ifstream in_file(file_name.c_str());
  if (!in_file) {
    return false;
  }
  std::string line_text = "";
  while (getline(in_file, line_text)) {
    if (line_text.empty()) {
      continue;
    }
    ToUppercase(line_text);
    code_lines.push_back(TrimCodeLine(line_text));
  }

  in_file.close();
  in_file.clear();

  return true;
}

void GCodeParse::ParseGCode(const std::vector<std::string> &code_lines,
                            std::vector<GCodeStruct> &g_code) {

  g_code.clear();

  double start_X = 0.;
  double start_Y = 0.;

  refer_point_.x = 0.;
  refer_point_.y = 0.;

  GCodeStruct code_array;
  code_array.KerfDir = 0;
  code_array.ScaleFactor = 1;
  code_array.RotateAngle = 0;
  code_array.HorizonMirror = 0;
  code_array.VerticalMirror = 0;
  code_array.ShowLine = 0;
  code_array.LineNoInTotalFile = 0;
  for (size_t i = 0; i < code_lines.size(); i++) {
    code_array.X0 = code_array.X = start_X;
    code_array.Y0 = code_array.Y = start_Y;
    code_array.OmitKerf = true;
    code_array.KerfValue = 0;
    code_array.OmitF = true;
    code_array.F = 0;
    code_array.R = 0;
    code_array.Length = 0;
    code_array.Delay = 0.;
    code_array.VariableType = 0;
    code_array.VariableValue = 0.;
    code_array.AsynchronousStop = 0.;
    code_array.OmitAsynchronousStop = true;
    code_array.remark = "";

    ParseCodeLine(code_lines[i], code_array);
    if ((code_array.Name == G01 || code_array.Name == G02 || code_array.Name == G03)
        && !m07_m08_flag_) {

      GCodeStruct m07 = code_array;
      m07.Name = M07;
      m07_m08_flag_ = true;
      g_code.push_back(m07);
      code_array.ShowLine++;
      code_array.LineNoInTotalFile++;
    }
    if (code_array.Name == G00 && m07_m08_flag_) {
      GCodeStruct m08 = code_array;
      m08.Name = M08;
      m07_m08_flag_ = false;
      g_code.push_back(m08);
      code_array.ShowLine++;
      code_array.LineNoInTotalFile++;
    }
    g_code.push_back(code_array);
    start_X = code_array.X;
    start_Y = code_array.Y;
    code_array.ShowLine++;
    code_array.LineNoInTotalFile++;

    if (code_array.Name == M02) {
      break;
    }
    // TODO: do the error check of above, if error, revert and break here.
  }
  CheckAppendM02(g_code);
}

int GCodeParse::GetCodeValue(const std::string &code_line,
                             const std::string &match,
                             double &argument) {

  std::size_t pos = code_line.find(match);
  if (pos == std::string::npos) {
    return -1;
  }
  std::size_t end_pos = code_line.find_first_not_of("+-0123456789.e", pos + 1);
  std::string value = end_pos != std::string::npos ?
      code_line.substr(pos + 1, end_pos - pos -1) : code_line.substr(pos + 1);

  char *endptr;
  argument = strtod(value.c_str(), &endptr);
  return *endptr == '\0' ? 1 : 0;
}

int GCodeParse::GetCodeValue(const std::string &code_line,
                             const std::string &match,
                             int &argument) {

  std::size_t pos = code_line.find(match);
  if (pos == std::string::npos) {
    return -1;
  }
  std::size_t end_pos = code_line.find_first_not_of("+-0123456789", pos + 1);
  std::string value = end_pos != std::string::npos ?
      code_line.substr(pos + 1, end_pos - pos -1) : code_line.substr(pos + 1);

  char *endptr;
  argument = strtol(value.c_str(), &endptr, 10);
  return *endptr == '\0' ? 1 : 0;
}

double GCodeParse::GetArcRadius(const GCodeStruct &arc_code) {
  return sqrt(pow(arc_code.X - arc_code.I, 2) +
              pow(arc_code.Y - arc_code.J, 2));

}

double GCodeParse::GetArcAngle(const GCodeStruct &arc_code) {
  D_Point start_point = D_Point(arc_code.X0, arc_code.Y0);
  D_Point end_point = D_Point(arc_code.X, arc_code.Y);
  D_Point center_point = D_Point(arc_code.I, arc_code.J);

  double a = D_Point::Distance(start_point, center_point);
  double b = D_Point::Distance(end_point, center_point);
  double c = D_Point::Distance(start_point, end_point);
  double cosine = (a * a + b * b - c * c) / (2 * a * b);
  double cross_product_z =
      (arc_code.X0 - arc_code.I) * (arc_code.Y - arc_code.J) -
      (arc_code.Y0 - arc_code.J) * (arc_code.X - arc_code.I);

  if (arc_code.Name == G02) {
    return cross_product_z < 0 ? math::ACOS(cosine) : 2 * PI - math::ACOS(cosine);
  } else if (arc_code.Name == G03) {
    return cross_product_z > 0 ? math::ACOS(cosine) : 2 * PI - math::ACOS(cosine);
  } else {
    return 0.;
  }
}

double GCodeParse::GetGCodeLength(const GCodeStruct &g_code) {
  if (g_code.Name == G00 || g_code.Name == G01) {
    return sqrt(pow(g_code.X - g_code.X0, 2)
        + pow(g_code.Y - g_code.Y0, 2));

  } else if (g_code.Name == G02 || g_code.Name == G03) {
    return GetArcRadius(g_code) * GetArcAngle(g_code);
  } else {
    return 0.;
  }
}

void GCodeParse::CalculateArcAngle(GCodeStruct &g_code) {
  if (g_code.Name != G02 && g_code.Name != G03) {
    return ;
  }
  g_code.StartAngle = atan2(g_code.Y0 - g_code.J, g_code.X0 - g_code.I);
  g_code.EndAngle = atan2(g_code.Y - g_code.J, g_code.X - g_code.I);

  while (g_code.StartAngle > _2PI) {
    g_code.StartAngle -= _2PI;
  }
  while (g_code.StartAngle < EP) {
    g_code.StartAngle += _2PI;
  }
  if (fabs(g_code.StartAngle) < EP) {
    g_code.StartAngle = 0;
  }
  while (g_code.EndAngle > _2PI) {
    g_code.EndAngle -= _2PI;
  }
  while (g_code.EndAngle < EP) {
    g_code.EndAngle += _2PI;
  }
  if (fabs(g_code.EndAngle) < EP) {
    g_code.EndAngle = 0;
  }

  if (g_code.Name == G02) {
    if (IsEqual(g_code.X, g_code.X0) && IsEqual(g_code.Y, g_code.Y0)
        || IsEqual(g_code.StartAngle, g_code.EndAngle)) {

      g_code.StartAngle = g_code.EndAngle + _2PI;
    } else if (IsLesser(g_code.StartAngle, g_code.EndAngle)) {
      g_code.StartAngle += _2PI;
    }

  }  else if (g_code.Name == G03) {
    if (IsEqual(g_code.X, g_code.X0) && IsEqual(g_code.Y, g_code.Y0)
        || IsEqual(g_code.StartAngle, g_code.EndAngle)) {

      g_code.EndAngle = g_code.StartAngle + _2PI;
    } else if (IsLesser(g_code.StartAngle, g_code.EndAngle)) {
      g_code.EndAngle += _2PI;
    }

  }
}

int GCodeParse::CalculateArcCenter(GCodeStruct &g_code) {
  // Circle do NOT calculate center.
  if (IsEqual(g_code.X0, g_code.X) && IsEqual(g_code.Y0, g_code.Y)) {
    return 0;
  }
  double r1 = dist(g_code.X0, g_code.Y0, g_code.I, g_code.J);
  double r2 = dist(g_code.X, g_code.Y, g_code.I, g_code.J);
  D_Point new_center;
  double s_to_e = dist(g_code.X0, g_code.Y0, g_code.X, g_code.Y);
  if (IsGreater(s_to_e, fabs(r1 + r2))) {
    return -1;
  } else if (IsEqual(s_to_e, fabs(r1 + r2))) {
    new_center = D_Point((g_code.X0 + g_code.X) / 2, (g_code.Y0 + g_code.Y) / 2);
  } else {
    D_Point point1;
    D_Point point2;
    double expect_radius = fabs(r1 + r2) / 2;
    int rtn = CircleIntersector(Circle(g_code.X0, g_code.Y0, expect_radius),
                                Circle(g_code.X, g_code.Y, expect_radius),
                                point1, point2);

    if (rtn == 2 || rtn == 3) {
      new_center = point1;
    } else if (rtn == 4) {
      double start_of_intersector1 = atan2(g_code.Y0 - point1.y(), g_code.X0 - point1.x());
      double end_of_intersector1 = atan2(g_code.Y - point1.y(), g_code.X - point1.x());
      double delt;
      if (g_code.Name == G02) {
        if (start_of_intersector1 < end_of_intersector1) {
          start_of_intersector1 += _2PI;
        }
        delt  = start_of_intersector1 - end_of_intersector1;
      } else {
        if (end_of_intersector1 < start_of_intersector1) {
          end_of_intersector1 += _2PI;
        }
        delt  = end_of_intersector1 - start_of_intersector1;
      }
      double arc_angle = GetArcAngle(g_code);
      if (arc_angle < PI) {
        if (delt < PI) {
          new_center = point1;
        } else {
          new_center = point2;
        }
      } else {
        if (delt > PI) {
          new_center = point1;
        } else {
          new_center = point2;
        }
      }
    } else {
      return -1;
    }
  }
  static const double ARC_CENTER_ADJUST_ACCURACY = 0.1;
  if (dist(g_code.I, g_code.J, new_center.x(), new_center.y()) > ARC_CENTER_ADJUST_ACCURACY) {
    return -1;
  }
  g_code.I = new_center.x();
  g_code.J = new_center.y();
  return 0;
}

int GCodeParse::CalculateArcEnd(GCodeStruct &g_code) {
  double r1 = dist(g_code.X0, g_code.Y0, g_code.I, g_code.J);
  double r2 = dist(g_code.X, g_code.Y, g_code.I, g_code.J);
  D_Point new_end = D_Point(r1 * (g_code.X - g_code.I) / r2 + g_code.I,
                            r1 * (g_code.Y - g_code.J) / r2 + g_code.J);

  static const double ARC_RADIUS_DIFF = 0.5;
  if (dist(g_code.X, g_code.Y, new_end.x(), new_end.y()) > ARC_RADIUS_DIFF) {
    return -1;
  }
  g_code.X = new_end.x();
  g_code.Y = new_end.y();
  return 0;
}

bool GCodeParse::WriteGCode(const std::string &file_name,
                            const std::vector<std::string> &contents) {

  std::ofstream out_file(file_name.c_str(), std::ios::out);
  if (!out_file) {
    return false;
  }
  for (size_t i = 0; i < contents.size(); i++) {
    out_file << contents[i];
  }
  out_file.close();
  out_file.clear();
  return true;
}

void GCodeParse::GenerateGCode(const std::vector<GCodeStruct> &g_code,
                               std::vector<std::string> &code_lines) {

  code_lines.clear();
  for (size_t i = 0; i < g_code.size(); i++) {
    char GCODE[100];
    switch (g_code[i].Name) {
     case G00:
      sprintf(GCODE, "G00 X%.3lf Y%.3lf\n",
          relative_absolute_ == Absolute ? g_code[i].X : g_code[i].X - g_code[i].X0,
          relative_absolute_ == Absolute ? g_code[i].Y : g_code[i].Y - g_code[i].Y0);

      break;
     case G01:
      if (g_code[i].OmitF) {
        sprintf(GCODE, "G01 X%.3lf Y%.3lf\n",
            relative_absolute_ == Absolute ? g_code[i].X : g_code[i].X - g_code[i].X0,
            relative_absolute_ == Absolute ? g_code[i].Y : g_code[i].Y - g_code[i].Y0);

      } else {
        sprintf(GCODE, "G01 X%.3lf Y%.3lf F%.3lf\n",
            relative_absolute_ == Absolute ? g_code[i].X : g_code[i].X - g_code[i].X0,
            relative_absolute_ == Absolute ? g_code[i].Y : g_code[i].Y - g_code[i].Y0,
            g_code[i].F);

      }
      break;
     case G02:
      if (g_code[i].OmitF) {
        sprintf(GCODE, "G02 X%.3lf Y%.3lf I%.3lf J%.3lf\n",
            relative_absolute_ == Absolute ? g_code[i].X : g_code[i].X - g_code[i].X0,
            relative_absolute_ == Absolute ? g_code[i].Y : g_code[i].Y - g_code[i].Y0,
            relative_absolute_ == Absolute ? g_code[i].I : g_code[i].I - g_code[i].X0,
            relative_absolute_ == Absolute ? g_code[i].J : g_code[i].J - g_code[i].Y0);

      } else {
        sprintf(GCODE, "G02 X%.3lf Y%.3lf I%.3lf J%.3lf F%.3lf\n",
            relative_absolute_ == Absolute ? g_code[i].X : g_code[i].X - g_code[i].X0,
            relative_absolute_ == Absolute ? g_code[i].Y : g_code[i].Y - g_code[i].Y0,
            relative_absolute_ == Absolute ? g_code[i].I : g_code[i].I - g_code[i].X0,
            relative_absolute_ == Absolute ? g_code[i].J : g_code[i].J - g_code[i].Y0,
            g_code[i].F);

      }
      break;
     case G03:
      if (g_code[i].OmitF) {
        sprintf(GCODE, "G03 X%.3lf Y%.3lf I%.3lf J%.3lf\n",
            relative_absolute_ == Absolute ? g_code[i].X : g_code[i].X - g_code[i].X0,
            relative_absolute_ == Absolute ? g_code[i].Y : g_code[i].Y - g_code[i].Y0,
            relative_absolute_ == Absolute ? g_code[i].I : g_code[i].I - g_code[i].X0,
            relative_absolute_ == Absolute ? g_code[i].J : g_code[i].J - g_code[i].Y0);

      } else {
        sprintf(GCODE, "G03 X%.3lf Y%.3lf I%.3lf J%.3lf F%.3lf\n",
            relative_absolute_ == Absolute ? g_code[i].X : g_code[i].X - g_code[i].X0,
            relative_absolute_ == Absolute ? g_code[i].Y : g_code[i].Y - g_code[i].Y0,
            relative_absolute_ == Absolute ? g_code[i].I : g_code[i].I - g_code[i].X0,
            relative_absolute_ == Absolute ? g_code[i].J : g_code[i].J - g_code[i].Y0,
            g_code[i].F);

      }
      break;
     case G04:
      sprintf(GCODE, "G04 P%.3lf\n", g_code[i].Delay);
      break;
     case G40:
      sprintf(GCODE, "G40\n");
      break;
     case G41:
      if (g_code[i].OmitKerf) {
        sprintf(GCODE, "G41\n");
      } else {
        sprintf(GCODE, "G41 K%.3lf\n", g_code[i].KerfValue);
      }
      break;
     case G42:
      if (g_code[i].OmitKerf) {
        sprintf(GCODE, "G42\n");
      } else {
        sprintf(GCODE, "G42 K%.3lf\n", g_code[i].KerfValue);
      }
      break;
     case G20:
      sprintf(GCODE, "G20\n");
      break;
     case G21:
      sprintf(GCODE, "G21\n");
      break;
     case G90:
      sprintf(GCODE, "G90\n");
      break;
     case G91:
      sprintf(GCODE, "G91\n");
      break;
     case G92:
      sprintf(GCODE, "G92 X%.3lf Y%.3lf\n", refer_point_.x, refer_point_.y);
      break;
     case G99:
      sprintf(GCODE, "G99 X%.3lf Y%.3lf I%.3lf J%.3lf\n",
          g_code[i].ScaleFactor, g_code[i].RotateAngle,
          g_code[i].HorizonMirror, g_code[i].VerticalMirror);

      break;
     case G59:
      sprintf(GCODE, "G59 V%d F%.3lf\n",
          g_code[i].VariableType, g_code[i].VariableValue);

      break;
     case GGG:
      sprintf(GCODE, "%s\n", g_code[i].remark.c_str());
      break;
     case M00:
      sprintf(GCODE, "M00\n");
      break;
     case M02:
      sprintf(GCODE, "M02\n");
      break;
     case M07:
      sprintf(GCODE, "M07\n");
      break;
     case M08:
      sprintf(GCODE, "M08\n");
      break;
     case M09:
      sprintf(GCODE, "M09\n");
      break;
     case M10:
      sprintf(GCODE, "M10\n");
      break;
     case M11:
      sprintf(GCODE, "M11\n");
      break;
     case M12:
      sprintf(GCODE, "M12\n");
      break;
     case M17:
      sprintf(GCODE, "M17\n");
      break;
     case M18:
      sprintf(GCODE, "M18\n");
      break;
     case M28:
      sprintf(GCODE, "M28\n");
      break;
     case M29:
      sprintf(GCODE, "M29\n");
      break;
     case M36:
      sprintf(GCODE, "M36 T%d\n", g_code[i].ProcessType);
      break;
     case M46:
      sprintf(GCODE, "M46\n");
      break;
     case M47:
      sprintf(GCODE, "M47\n");
      break;
     case M50:
      if (g_code[i].OmitAsynchronousStop) {
        sprintf(GCODE, "M50\n");
      } else {
        sprintf(GCODE, "M50H%.3lf\n", g_code[i].AsynchronousStop);
      }
      break;
     case M51:
      sprintf(GCODE, "M51\n");
      break;
     case M80:
      sprintf(GCODE, "M80\n");
      break;
     default:
      sprintf(GCODE, "\n");
      break;
    }
    code_lines.push_back(GCODE);
    if (g_code[i].Name == M02) {
      break;
    }
  }
}

void GCodeParse::CalculateGCode(std::vector<GCodeStruct> &g_code) {
  for (size_t i = 0; i < g_code.size(); i++) {
    switch (g_code[i].Name) {
     case G00:
     case G01:
      g_code[i].R = 0;
      g_code[i].Length = GetGCodeLength(g_code[i]);
      g_code[i].StartAngle = atan2(g_code[i].Y - g_code[i].Y0, g_code[i].X - g_code[i].X0);
      break;
     case G02:
     case G03:
      g_code[i].R = GetArcRadius(g_code[i]);
      g_code[i].Length = GetGCodeLength(g_code[i]);
      CalculateArcAngle(g_code[i]);
      break;
     default:
      g_code[i].R = 0;
      g_code[i].Length = 0;
      break;
    }
  }
}

bool GCodeParse::AsynchronousStopGCode(const std::vector<GCodeStruct> &g_code,
                                       std::vector<GCodeStruct> &new_code,
                                       bool F_forbid, double global_speed,
                                       double arc_radius_diff) {

  new_code.clear();
  for (size_t i = 0; i < g_code.size(); i++) {
    GCodeStruct cur_code = g_code[i];
    if (cur_code.Name == G02 || cur_code.Name == G03) {
      double r1 = dist(cur_code.X0, cur_code.Y0, cur_code.I, cur_code.J);
      double r2 = dist(cur_code.X, cur_code.Y, cur_code.I, cur_code.J);
      if (IsGreater(fabs(r1 - r2), arc_radius_diff)) {
        return false;
      }
      double angle = GetArcAngle(cur_code);
      if (!IsGreater(cur_code.R - cur_code.R * cos(angle / 2), 0)) {
        cur_code.Name = G01;
        new_code.push_back(cur_code);
      } else if (CalculateArcCenter(cur_code) != 0) {
        if (CalculateArcEnd(cur_code) != 0) {
          return false;
        }
        GCodeStruct G01_code = cur_code;
        G01_code.Name = G01;
        G01_code.X0 = cur_code.X;
        G01_code.Y0 = cur_code.Y;
        G01_code.X = g_code[i].X;
        G01_code.Y = g_code[i].Y;
        new_code.push_back(cur_code);
        new_code.push_back(G01_code);
      } else {
        new_code.push_back(cur_code);
      }
      continue;
    }
    if (cur_code.Name == M50 && !cur_code.OmitAsynchronousStop) {
      if (new_code.empty()) {
        return false;
      }
      double asynchronous_length = F_forbid ? fabs(cur_code.AsynchronousStop) * global_speed / 60
                                            : fabs(cur_code.AsynchronousStop) * new_code.back().F / 60;

      double x;
      double y;
      if (new_code.back().Name == G01) {
        math::Line line(new_code.back().X0, new_code.back().Y0,
            new_code.back().X, new_code.back().Y,
            MIN(new_code.back().X0, new_code.back().X),
            MAX(new_code.back().X0, new_code.back().X));

        if (!line.Calc(x, y, new_code.back().X0, new_code.back().Y0, new_code.back().X, new_code.back().Y, asynchronous_length)) {
          return false;
        }
      } else if (new_code.back().Name == G02 || new_code.back().Name == G03) {
        math::Arc arc(new_code.back().I, new_code.back().J, new_code.back().R,
            new_code.back().X0, new_code.back().Y0, new_code.back().X, new_code.back().Y,
            new_code.back().Name == G02 ? math::CW : math::CCW);

        if (arc.Calc(x, y, asynchronous_length) == -1) {
          return false;
        }
      } else {
        return false;
      }
      GCodeStruct insert_code = new_code.back();
      new_code.back().X = x;
      new_code.back().Y = y;
      insert_code.X0 = new_code.back().X;
      insert_code.Y0 = new_code.back().Y;
      GCodeStruct M29_code = new_code.back();
      M29_code.Name = M29;
      M29_code.X0 = new_code.back().X;
      M29_code.Y0 = new_code.back().Y;
      new_code.push_back(M29_code);
      new_code.push_back(insert_code);
      new_code.push_back(cur_code);
      new_code.back().OmitAsynchronousStop = true;
      continue;
    }
    new_code.push_back(cur_code);
  }
  return true;
}

void GCodeParse::ToUppercase(std::string &line) {
  std::string transform = line;
  for (size_t i = 0; i < line.size(); i++) {
    transform[i] = toupper(line[i]);
  }
  line = transform;
}

std::string GCodeParse::TrimCodeLine(const std::string &line) {
  std::string blank = " \t";
  size_t begin = line.find_first_not_of(blank);
  if (begin == std::string::npos) {
    return "";
  }
  size_t end = line.find_last_not_of(blank);
  return line.substr(begin, end - begin + 1);
}

void GCodeParse::CheckAppendM02(std::vector<GCodeStruct> &g_code) {
  if (g_code.back().Name != M02) {
    GCodeStruct code_array = g_code.back();
    code_array.Name = M02;
    code_array.X0 = code_array.X;
    code_array.Y0 = code_array.Y;
    code_array.R = 0;
    code_array.AngleRatio = 0;
    code_array.Length = 0;
    code_array.OmitKerf = true;
    code_array.KerfValue = 0;
    code_array.OmitF = true;
    code_array.F = 0;
    code_array.Delay = 0.;
    code_array.ShowLine++;
    code_array.LineNoInTotalFile++;
    g_code.push_back(code_array);
  }
}

void GCodeParse::ParseSecondary(GCodeStruct &g_code) {
  switch (g_code.Name) {
   case G20:
    foot_metric_ = FootSystem;
    break;
   case G21:
    foot_metric_ = MetricSystem;
    break;
   case G90:
    relative_absolute_ = Absolute;
    break;
   case G91:
    relative_absolute_ = Relative;
    break;
   case G92:
    refer_point_.x = g_code.X;
    refer_point_.y = g_code.Y;
    break;
   case G40:
    g_code.KerfDir = 0;
    break;
   case G41:
    g_code.KerfDir = 1;
    break;
   case G42:
    g_code.KerfDir = 2;
    break;
   case G00:
   case G01:
    g_code.R = 0;
    g_code.Length = GetGCodeLength(g_code);
    g_code.StartAngle = atan2(g_code.Y - g_code.Y0, g_code.X - g_code.X0);
    break;
   case G02:
   case G03:
    g_code.R = GetArcRadius(g_code);
    g_code.Length = GetGCodeLength(g_code);
    CalculateArcAngle(g_code);
    break;
   case M07:
    if (!m07_m08_flag_) {
      m07_m08_flag_ = true;
    }
    break;
   case M08:
    if (m07_m08_flag_) {
      m07_m08_flag_ = false;
    }
    break;
   default:
    break;
  }
}

/**
 * return: 0 Unknown
 *         1 G, M code or default
 */
int GCodeParse::ParseCodeLine(const std::string &code_line,
    GCodeStruct &g_code) {

  int gm_id = -1;
  int type = RecognizeGMType(code_line, gm_id);
  if (type == 0 || type == 1) {
    ID2CodeName(type, gm_id, g_code);
  } else if (type == 3) { // comments
    g_code.Name = GGG;
  }
  if (g_code.Name == GGG) {
    ParseRemark(code_line, g_code);
    return 0;
  }
  ParseCodeArguments(code_line, g_code);
  ParseSecondary(g_code);
  return 1;
}

//return: 0 G
//        1 M
//        2 Same as last G or M code(Default)
//        3 Comments
int GCodeParse::RecognizeGMType(const std::string &code_line, int &gm_id) {
  size_t pos_r = code_line.find_first_of("(%T");
  size_t pos_g = code_line.find('G');
  size_t pos_m = code_line.find('M');
  size_t pos = pos_g != std::string::npos ? pos_g : pos_m;

  if (pos_r != std::string::npos) { // Comments
    return 3;
  }
  if (pos != std::string::npos) { // G or M code
    size_t id_end = code_line.find_first_not_of("0123456789", pos + 1);
    std::string value = id_end != std::string::npos ?
        code_line.substr(pos + 1, id_end - pos - 1) : code_line.substr(pos + 1);

    char *endptr;
    gm_id = strtol(value.c_str(), &endptr, 10);
    return pos_g != std::string::npos ? 0 : 1;
  } else {
    return 2;
  }
}

void GCodeParse::ID2CodeName(int gm_type, int gm_id, GCodeStruct &g_code) {
  if (gm_type == 0) { // G code
    switch (gm_id) {
     case 0: g_code.Name = G00; break;
     case 1: g_code.Name = G01; break;
     case 2: g_code.Name = G02; break;
     case 3: g_code.Name = G03; break;
     case 4: g_code.Name = G04; break;
     case 20:
     case 70: g_code.Name = G20; break;
     case 21:
     case 71: g_code.Name = G21; break;
     case 26: g_code.Name = G26; break;
     case 27: g_code.Name = G27; break;
     case 28: g_code.Name = G28; break;
     case 40: g_code.Name = G40; break;
     case 41: g_code.Name = G41; break;
     case 42: g_code.Name = G42; break;
     case 59: g_code.Name = G59; break;
     case 90: g_code.Name = G90; break;
     case 91: g_code.Name = G91; break;
     case 92: g_code.Name = G92; break;
     case 99: g_code.Name = G99; break;
     default: g_code.Name = GGG; break;
    }
  } else if (gm_type == 1) { // M code
    switch (gm_id) {
     case 2:
     case 30: g_code.Name = M02; break;
     case 0: g_code.Name = M00; break;
     case 7: g_code.Name = M07; break;
     case 17: g_code.Name = M17; break;
     case 8: g_code.Name = M08; break;
     case 18: g_code.Name = M18; break;
     case 9: g_code.Name = M09; break;
     case 10: g_code.Name = M10; break;
     case 11: g_code.Name = M11; break;
     case 12: g_code.Name = M12; break;
     case 28: g_code.Name = M28; break;
     case 29: g_code.Name = M29; break;
     case 36: g_code.Name = M36; break;
     case 46: g_code.Name = M46; break;
     case 47: g_code.Name = M47; break;
     case 50: g_code.Name = M50; break;
     case 51: g_code.Name = M51; break;
     case 80: g_code.Name = M80; break;
     default: g_code.Name = GGG; break;
    }
  }
}

void GCodeParse::ParseRemark(const std::string &code_line,
    GCodeStruct &g_code) {

  if (g_code.Name == GGG) {
    g_code.remark = code_line;
  }
}

void GCodeParse::ParseCodeArguments(const std::string &code_line,
                                    GCodeStruct &g_code) {

  /// g_code is NOT GGG unknown code.
  ParseXArgument(code_line, g_code);
  ParseYArgument(code_line, g_code);
  ParseUArgument(code_line, g_code);
  ParseVArgument(code_line, g_code);
  ParseIArgument(code_line, g_code);
  ParseJArgument(code_line, g_code);
  ParseRArgument(code_line, g_code);
  ParseFArgument(code_line, g_code);
  ParseKArgument(code_line, g_code);
  ParsePArgument(code_line, g_code);
  ParseTArgument(code_line, g_code);
  ParseHArgument(code_line, g_code);
}

void GCodeParse::ParseXArgument(const std::string &code_line,
                                GCodeStruct &g_code) {

  double value = 0.;
  if (GetCodeValue(code_line, "X", value) != 1) {
    return;
  }
  if (g_code.Name == G99) {
    g_code.ScaleFactor = value;
  } else if (g_code.Name == G41 || g_code.Name == G42) {
    value = foot_metric_ == MetricSystem ? value : value * FOOT_METRIC_FACTOR;
    g_code.KerfValue = value;
    g_code.OmitKerf = false;
  } else if (g_code.Name == G92) {
    value = foot_metric_ == MetricSystem ? value : value * FOOT_METRIC_FACTOR;
    g_code.X = value;
  } else {
    value = foot_metric_ == MetricSystem ? value : value * FOOT_METRIC_FACTOR;
    g_code.X = relative_absolute_ == Absolute ? value : g_code.X0 + value;
  }
}

void GCodeParse::ParseYArgument(const std::string &code_line,
                                GCodeStruct &g_code) {

  double value = 0.;
  if (GetCodeValue(code_line, "Y", value) != 1) {
    return;
  }
  if (g_code.Name == G99) {
    g_code.RotateAngle = value;
  } else if (g_code.Name == G92) {
    value = foot_metric_ == MetricSystem ? value : value * FOOT_METRIC_FACTOR;
    g_code.Y = value;
  } else {
    value = foot_metric_ == MetricSystem ? value : value * FOOT_METRIC_FACTOR;
    g_code.Y = relative_absolute_ == Absolute ? value : g_code.Y0 + value;
  }
}

void GCodeParse::ParseUArgument(const std::string &code_line,
                                GCodeStruct &g_code) {

  double value = 0.;
  if (GetCodeValue(code_line, "U", value) != 1) {
    return;
  }
  value = foot_metric_ == MetricSystem ? value : value * FOOT_METRIC_FACTOR;
  g_code.X = g_code.X0 + value;
}

void GCodeParse::ParseVArgument(const std::string &code_line,
                                GCodeStruct &g_code) {

  double value = 0.;
  if (GetCodeValue(code_line, "V", value) != 1) {
    return;
  }
  if (g_code.Name == G59) {
    g_code.VariableType = static_cast<int>(value);
  } else {
    value = foot_metric_ == MetricSystem ? value : value * FOOT_METRIC_FACTOR;
    g_code.Y = g_code.Y0 + value;
  }
}

void GCodeParse::ParseIArgument(const std::string &code_line,
                                GCodeStruct &g_code) {

  double value = 0.;
  if (GetCodeValue(code_line, "I", value) != 1) {
    value = 0.;
  }
  if (g_code.Name == G99) {
    g_code.HorizonMirror = value;
  } else if (g_code.Name == G02 || g_code.Name == G03) {
    value = foot_metric_ == MetricSystem ? value : value * FOOT_METRIC_FACTOR;
    g_code.I = relative_absolute_ == Absolute ? value : g_code.X0 + value;
  }
}

void GCodeParse::ParseJArgument(const std::string &code_line,
                                GCodeStruct &g_code) {

  double value = 0.;
  if (GetCodeValue(code_line, "J", value) != 1) {
    value = 0.;
  }
  if (g_code.Name == G99) {
    g_code.VerticalMirror = value;
  } else if (g_code.Name == G02 || g_code.Name == G03) {
    value = foot_metric_ == MetricSystem ? value : value * FOOT_METRIC_FACTOR;
    g_code.J = relative_absolute_ == Absolute ? value : g_code.Y0 + value;
  }
}

void GCodeParse::ParseRArgument(const std::string &code_line,
                                GCodeStruct &g_code) {

}

void GCodeParse::ParseFArgument(const std::string &code_line,
                                GCodeStruct &g_code) {

  double value = 0.;
  if (GetCodeValue(code_line, "F", value) != 1) {
    return;
  }
  if (g_code.Name == G59) {
    g_code.VariableValue = value;
  } else {
    value = foot_metric_ == MetricSystem ? value : value * FOOT_METRIC_FACTOR;
    g_code.F = value;
    g_code.OmitF = false;
  }
}

void GCodeParse::ParseKArgument(const std::string &code_line,
                                GCodeStruct &g_code) {

  double value = 0.;
  if (GetCodeValue(code_line, "K", value) != 1) {
    return;
  }
  value = foot_metric_ == MetricSystem ? value : value * FOOT_METRIC_FACTOR;
  g_code.KerfValue = value;
  g_code.OmitKerf = false;
}

void GCodeParse::ParsePArgument(const std::string &code_line,
                                GCodeStruct &g_code) {

  double value = 0.;
  if (GetCodeValue(code_line, "P", value) != 1) {
    return;
  }
  if (g_code.Name == G04) {
    g_code.Delay = value < 0 ? 0 : value * 0.01; // time unit: s
  }
}

void GCodeParse::ParseTArgument(const std::string &code_line,
                                GCodeStruct &g_code) {

  int value = 0;
  if (GetCodeValue(code_line, "T", value) != 1) {
    return;
  }
  if (g_code.Name == M36) {
    g_code.ProcessType = value < 0 ? 0 : value;
  }
}

void GCodeParse::ParseHArgument(const std::string &code_line,
                                GCodeStruct &g_code) {

  double value = 0.;
  if (GetCodeValue(code_line, "H", value) != 1) {
    return;
  }
  if (g_code.Name == M50) {
    g_code.AsynchronousStop = value; // unit:s
    g_code.OmitAsynchronousStop = false;
  }
}
