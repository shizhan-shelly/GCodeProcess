// Copyright 2016 Fangling Software Co., Ltd. All Rights Reserved.
// Author: shizhan-shelly@hotmail.com (Zhan Shi)

#ifndef GCODE_GCODEPARSE_H__
#define GCODE_GCODEPARSE_H__

#include <vector>

#include "GCodeDefinition.h"

class GCodeParse {
 public:
  GCodeParse();
  ~GCodeParse();

  bool ReadGCode(const std::string &file_name,
      std::vector<std::string> &code_lines);

  bool WriteGCode(const std::string &file_name,
      const std::vector<std::string> &contents);

  void ParseGCode(const std::vector<std::string> &code_lines,
                  std::vector<GCodeStruct> &g_code);

  void GenerateGCode(const std::vector<GCodeStruct> &g_code,
                     std::vector<std::string> &code_lines);

  void CalculateGCode(std::vector<GCodeStruct> &g_code);

  bool AsynchronousStopGCode(const std::vector<GCodeStruct> &g_code,
                             std::vector<GCodeStruct> &new_code,
                             bool F_forbid, double global_speed,
                             double arc_radius_diff);

 private:
  /**
   * @return: 1, success to get the value following "match" and transform it
   *             correctly.
   *          0, get the value following "match" but fail to transform it.
   *          -1, no value be found.
   */
  int GetCodeValue(const std::string &code_line, const std::string &match,
                   double &argument);

  int GetCodeValue(const std::string &code_line, const std::string &match,
                   int &argument);

  double GetArcRadius(const GCodeStruct &arc_code);

  double GetArcAngle(const GCodeStruct &arc_code);

  double GetGCodeLength(const GCodeStruct &g_code);

  void CalculateArcAngle(GCodeStruct &g_code);

  /// Arc g code modify its center point's coordinate(I, J).
  /// User must make sure that it's G02 or G03.
  /// @return: 0, successful
  ///         -1, error
  int CalculateArcCenter(GCodeStruct &g_code);

  /// Arc g code modify its end point's coordinate(X, Y).
  /// User must make sure that it's G02 or G03.
  /// @return: 0, successful
  ///         -1, error
  int CalculateArcEnd(GCodeStruct &g_code);

  void ToUppercase(std::string &line);
  std::string TrimCodeLine(const std::string &line);

  // Check whether there is "M02" in the back of g code array,
  // if no, append the M02 g code in the last.
  void CheckAppendM02(std::vector<GCodeStruct> &g_code);

  /**
   * return: 0 G
   *         1 M
   *         2 Same as last G or M code(Default)
   *         3 Comments
   */
  int RecognizeGMType(const std::string &code_line, int &gm_id);

  // gm_type: 0, G code; 1: M code
  void ID2CodeName(int gm_type, int gm_id, GCodeStruct &g_code);
  void ParseCodeArguments(const std::string &code_line, GCodeStruct &g_code);
  void ParseXArgument(const std::string &code_line, GCodeStruct &g_code);
  void ParseUArgument(const std::string &code_line, GCodeStruct &g_code);
  void ParseYArgument(const std::string &code_line, GCodeStruct &g_code);
  void ParseVArgument(const std::string &code_line, GCodeStruct &g_code);
  void ParseIArgument(const std::string &code_line, GCodeStruct &g_code);
  void ParseJArgument(const std::string &code_line, GCodeStruct &g_code);
  void ParseRArgument(const std::string &code_line, GCodeStruct &g_code);
  void ParseFArgument(const std::string &code_line, GCodeStruct &g_code);
  void ParseKArgument(const std::string &code_line, GCodeStruct &g_code);
  void ParsePArgument(const std::string &code_line, GCodeStruct &g_code);
  void ParseTArgument(const std::string &code_line, GCodeStruct &g_code);
  void ParseHArgument(const std::string &code_line, GCodeStruct &g_code);

  /**
   * This function will parse a g code object if it is G or M code.
   * return: 0 comments
   *        -1 Unknown
   *         1 G, M code or default
   */
  int ParseCodeLine(const std::string &code_line, GCodeStruct &g_code);

  void ParseSecondary(GCodeStruct &g_code);

  FootMetricSystem foot_metric_;
  RelorAbsCoordinate relative_absolute_;
  bool m07_m08_flag_;

}; // class GCodeParse

#endif // GCODE_GCODEPARSE_H__
