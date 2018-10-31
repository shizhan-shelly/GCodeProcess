// Copyright 2016 Fangling Software Co., Ltd. All Rights Reserved.
// Author: shizhan-shelly@hotmail.com (Zhan Shi)

#ifndef KERF_KERF_H__
#define KERF_KERF_H__

#include <vector>

#include "../GCodeDefinition.h"

namespace GCode {

class Kerf {
 public:
  Kerf();
  ~Kerf();

  void SetKerfValue(double kerf_value);

  void SetKerfHandlerMode(int handler_mode);

  void GKerfProc(const std::string &noKerfFile, const std::string &KerfFile);

 private:
  double param_kerf;
  int handler_mode_;
  std::vector<GCodeStruct> GfileFloatKerf; // ¨®D??¡¤¨¬¦Ì???¦Ì?D¨ª?D??¡ä¨²??
  std::vector<GCodeStruct> GfileFloatNoKerf; // ??¨®D??¡¤¨¬¦Ì???¦Ì?D¨ª?D??¡ä¨²??

  void CircleCheFen(std::vector<GCodeStruct> &GCodeArry);
  int IgnoreLittleLine(std::vector<GCodeStruct> &GCodeArry);

  void whole_circle_mark(GCodeStruct &PGCode);
  void whole_circle_mark_clear(GCodeStruct &pGcode);
  int is_whole_circle_marked (GCodeStruct &PGCode);
  int is_arc_radius_vaild_for_kerf(int kerf_dir, double kerf_value, const GCodeStruct &PGCode);
  int is_g_code_angle_change_small(GCodeStruct &pPreviousLine, GCodeStruct &pNextLine);
  int is_arc_in_fit_to_line_accuracy(double R, double alpha);

  void calculate_arc_length_radius_and_angle(GCodeStruct &PGCode);
  int calculate_arc_center(GCodeStruct &PGCode, double r1, double r2);
  int calculate_arc_end(GCodeStruct &PGCode, double &xe, double &ye);
  int calibrate_gcode_position(std::vector<GCodeStruct> &GFileFloatKerf);
  int calibrate_arc(std::vector<GCodeStruct> &GCodeArry);
  int Cal_Length_Angle_R(std::vector<GCodeStruct> &GCodeArry, int WithKerf);
  int check_gcode_position(std::vector<GCodeStruct> &GCodeArry);
  int check_gcode_before_kerf();
  int check_gcode_after_kerf();
  int calibrate_whole_circle();
  double calculate_g_code_angle_change(GCodeStruct &pPreviousLine, GCodeStruct &pNextLine);
  double GetRadius(GCodeStruct &pGcode);
  double GetTangent(GCodeStruct &pGcode, int StartOrEnd);
  int GetAddKerfGCode(GCodeStruct &pNoKerfG,
      GCodeStruct &AddKerfGCode, double kerfvalue, int dir);

  int AddOrTrunc(GCodeStruct &pPreviousLine, GCodeStruct &pNextLine,
      GCodeStruct &pAddLine, int dir);

  int Setupkerf(GCodeStruct &pGcode, double &dx, double &dy,
      double kerfvlaue, int dir);

  int Canclekerf(GCodeStruct &pGcode, double &dx, double &dy,
      double kerfvlaue, int dir);

  void gcode_set_start_pos(GCodeStruct &pGcode, double xs, double ys);
  void gcode_set_end_pos(GCodeStruct &pGcode, double xe, double ye);
  void gcode_set_IJ_pos(GCodeStruct &pGcode, double xi, double yi);
  void gcode_set_name(GCodeStruct &pGcode, unsigned short gcode_name);
  void gcode_get_next_kerf_buf(std::vector<GCodeStruct>::iterator &PtrDst,
                               std::vector<GCodeStruct>::iterator &PtrSrc);
  void gcode_set_next_kerf(std::vector<GCodeStruct>::iterator &PtrDst,
                           std::vector<GCodeStruct>::iterator &PtrSrc,
                           unsigned short Name, double Xe, double Ye);

  bool gcode_set_end_pos_until_G01G02G03(std::vector<GCodeStruct>::iterator &Ptr,
      double xe, double ye);

  bool kerf_get_previous_line_gcode(const std::vector<GCodeStruct> &g_code,
                                    std::vector<GCodeStruct>::iterator &Ptr,
                                    std::vector<GCodeStruct>::iterator &PtrPreviousLine);

  int kerf_line_and_line_cut_off(std::vector<GCodeStruct>::iterator &PtrDst,
                                 std::vector<GCodeStruct>::iterator &PtrSrc,
                                 GCodeStruct &AddKerfGCode);
  int kerf_line_and_circle_cut_off(std::vector<GCodeStruct>::iterator &PtrDst,
                                   std::vector<GCodeStruct>::iterator &PtrSrc,
                                   GCodeStruct &AddKerfGCode);
  int kerf_circle_and_circle_cut_off(std::vector<GCodeStruct>::iterator &PtrDst,
                                     std::vector<GCodeStruct>::iterator &PtrSrc,
                                     GCodeStruct &AddKerfGCode);

  void kerf_insert_line(std::vector<GCodeStruct>::iterator &PtrDst,
                        std::vector<GCodeStruct>::iterator &PtrSrc,
                        GCodeStruct &AddKerfGCode);
  void kerf_insert_arc(int arc_name, std::vector<GCodeStruct>::iterator &PtrDst,
                       std::vector<GCodeStruct>::iterator &PtrSrc,
                       GCodeStruct &AddKerfGCode);
  void kerf_direct_connect(std::vector<GCodeStruct>::iterator &PtrDst,
                           std::vector<GCodeStruct>::iterator &PtrSrc,
                           GCodeStruct &AddKerfGCode);
  void kerf_just_set_up(std::vector<GCodeStruct>::iterator &PtrDst,
                        std::vector<GCodeStruct>::iterator &PtrSrc,
                        GCodeStruct &AddKerfGCode);

  void kerf_no_need(std::vector<GCodeStruct>::iterator &PtrDst,
                    std::vector<GCodeStruct>::iterator &PtrSrc);
  int kerf_for_line(std::vector<GCodeStruct>::iterator &PtrDst,
                    std::vector<GCodeStruct>::iterator &PtrSrc,
                    char &kerf_on, double kerf_value);
  int kerf_for_arc(std::vector<GCodeStruct>::iterator &PtrDst,
                   std::vector<GCodeStruct>::iterator &PtrSrc,
                   char &kerf_on, double kerf_value);

  int g2kerf(std::vector<GCodeStruct> &DesKerFile,
             std::vector<GCodeStruct> &NoKerfFile);

}; // class Kerf

} // namespace GCode

#endif // KERF_KERF_H__
