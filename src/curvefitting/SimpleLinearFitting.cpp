// Copyright 2017 Fangling Software Co., Ltd. All Rights Reserved.
// Author: shizhan-shelly@hotmail.com (Zhan Shi)

#include "SimpleLinearFitting.h"

#include <memory.h>

#include "../math/mymath.h"

char text_wholefile[Gfilesize];

void FitSmallLine(GCodeStruct *Ptr, double min_fit_length) {
  double TotalLen, Len;
  GCodeStruct *start_line, *next_line, *new_start_line, *new_next_line;
  char *fit_flag;

  // In F2000 system, MINFITLENGTH is set with following calculate formula:
  // double MINFITLENGTH = AllPara_New.SystemPara.MaxCutSpeed/60*INTERPOLATION_T/2;
  double MINFITLENGTH = min_fit_length;
  if (MINFITLENGTH > 1.0) MINFITLENGTH = 1;

  //小线段拟合
  start_line = next_line = Ptr;
  fit_flag = text_wholefile;
	while(start_line->Name!=M02)
	{
    *(fit_flag++) = 0;
		//合并小线段，在规定的精度和步数内
		next_line = start_line + 1; 				
		TotalLen = start_line->Length;
		if (start_line->Name == G01) 	
    {
			while (TotalLen < MINFITLENGTH && next_line->Name==G01)
			{
				TotalLen = sqrt(pow(start_line->X0-next_line->X, 2) + pow(start_line->Y0-next_line->Y, 2));	//实际拟合后的长度
				next_line++;
			}
			if (TotalLen >= MINFITLENGTH && next_line->Name == G01)		//判断下次拟合的长度
			{
				new_start_line = next_line;
				new_next_line = new_start_line + 1;
				Len = new_start_line->Length;
				if (Len < MINFITLENGTH && new_start_line->Name == G01)
				{
					while (Len < MINFITLENGTH && new_next_line->Name == G01)
					{
						Len = sqrt(pow(new_start_line->X0-new_next_line->X, 2) + pow(new_start_line->Y0-new_next_line->Y, 2)); //实际拟合后的长度
						new_next_line++;
					}
				}
				if (Len < MINFITLENGTH && Len > 0)					//下次拟合达不到规定的长度
					next_line = new_next_line;
			}
    }
		next_line--;
		if(next_line > start_line)
		{
      memset(fit_flag, 1, (next_line - start_line) * sizeof(char));
      fit_flag += (next_line - start_line);

		  start_line->X = next_line->X;
		  start_line->Y = next_line->Y;
      start_line = next_line;
		}
		start_line++;
	}
  // 整合拟合后的结构体数组
  start_line = Ptr;
  new_start_line = Ptr;
  fit_flag = text_wholefile;
  while (start_line->Name != M02) {
    if (*(fit_flag++) != 0) {
      start_line++;
      continue;
    }
    *(new_start_line++) = *(start_line++);
  }
  *new_start_line = *start_line; // 拷贝M02
}

static const int G_FILE_SIZE = 0x400000;

SimpleLinearFitting::SimpleLinearFitting() {
  text_whole_file_ = new char[G_FILE_SIZE];
  memset(text_whole_file_, 0, sizeof(char) * G_FILE_SIZE);
}

SimpleLinearFitting::~SimpleLinearFitting() {
  delete [] text_whole_file_;
}

void SimpleLinearFitting::FitSmallLine(std::vector<GCodeStruct> &g_codes,
                                       double min_fit_length) {

  double TotalLen, Len;
  std::vector<GCodeStruct>::iterator start_line, next_line, new_start_line, new_next_line;
  char *fit_flag;

  // In F2000 system, MINFITLENGTH is set with following calculate formula:
  // double MINFITLENGTH = AllPara_New.SystemPara.MaxCutSpeed/60*INTERPOLATION_T/2;
  double MINFITLENGTH = min_fit_length;
  if (MINFITLENGTH > 1.0) MINFITLENGTH = 1;

  //小线段拟合
  start_line = next_line = g_codes.begin();
  fit_flag = text_whole_file_;
	while(start_line->Name != M02)
	{
    *(fit_flag++) = 0;
		//合并小线段，在规定的精度和步数内
		next_line = start_line + 1;
		TotalLen = start_line->Length;
		if (start_line->Name == G01)
    {
			while (TotalLen < MINFITLENGTH && next_line->Name==G01)
			{
				TotalLen = sqrt(pow(start_line->X0-next_line->X, 2) + pow(start_line->Y0-next_line->Y, 2));	//实际拟合后的长度
				next_line++;
			}
			if (!math::IsLesser(TotalLen, MINFITLENGTH) && next_line->Name == G01)		//判断下次拟合的长度
			{
				new_start_line = next_line;
				new_next_line = new_start_line + 1;
				Len = new_start_line->Length;
				if (Len < MINFITLENGTH && new_start_line->Name == G01)
				{
					while (Len < MINFITLENGTH && new_next_line->Name == G01)
					{
						Len = sqrt(pow(new_start_line->X0-new_next_line->X, 2) + pow(new_start_line->Y0-new_next_line->Y, 2)); //实际拟合后的长度
						new_next_line++;
					}
				}
				if (Len < MINFITLENGTH && Len > 0) //下次拟合达不到规定的长度
					next_line = new_next_line;
			}
    }
		next_line--;
		if(next_line > start_line)
		{
      memset(fit_flag, 1, (next_line - start_line) * sizeof(char));
      fit_flag += (next_line - start_line);

		  start_line->X = next_line->X;
		  start_line->Y = next_line->Y;
      start_line = next_line;
		}
		start_line++;
	}
  // 整合拟合后的结构体数组
  start_line = g_codes.begin();
  new_start_line = g_codes.begin();
  fit_flag = text_whole_file_;
  while (start_line->Name != M02) {
    if (*(fit_flag++) != 0) {
      start_line++;
      continue;
    }
    *(new_start_line++) = *(start_line++);
  }
  *new_start_line = *start_line; // 拷贝M02
}
