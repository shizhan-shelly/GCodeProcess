// Copyright 2016 Fangling Software Co., Ltd. All Rights Reserved.
// Author: shizhan-shelly@hotmail.com (Zhan Shi)

#include "Kerf.h"

#include "../GCodeParse.h"
#include "../curvefitting/SimpleLinearFitting.h"
#include "../math/mymath.h"

using math::INFINITESIMAL;
using math::PI;
using math::_1p2PI;
using math::_2PI;
using math::EP;
using math::IsEqual;
using math::IsZero;
using math::IsGreater;
using math::IsLesser;
using math::sqr;

typedef enum _ERR_G_CODE {
  ErrNo,                      // 正常
  ErrGCode,                   // 获取上一行需加割缝的G代码时获取不到
  ERR_G_CODE_POSITION,        // G代码绝对位置偏差过大
  ERR_G_CODE_WHOLE_CIRCLE,    // 整圆出错，无法校正
  ERR_G_CODE_ARC_R_TOO_SMALL, // 圆弧半径相对割缝半径过小
  ERR_G_CODE_ARC_CENTER,      // 圆弧精度误差较大，圆心无法校正
  ERR_G_CODE_KERF_CUT_OFF,    // 截断时无法计算出交点
  ERR_G_CODE_ErrG0203,        // 圆弧错误
  ERR_G_CODE_ErrGLarge,       // 数组空间不足

} ERR_G_CODE;

typedef enum _kerfstatus {
	NOKERF,  						//未建立 补偿
	G41KERF,  						//遇到G41，但补偿尚没有建立状态 
	G42KERF,  						//遇到G42,但补偿尚未建立状态
	JUSTSETG41KERF,					//刚刚建立 G41补偿
	JUSTSETG42KERF,					//刚刚建立 g42补偿
	SETTEDG41KERF, 					//已经建立起G41补偿
	SETTEDG42KERF, 					//已经建立起G42补偿
	G40KERF  						//撤消补偿状态
} kerfstatus;

/***************************************************************************************
定义割缝最小角度阈值

kerf = 10mm
    angle = 1度， L = 0.174mm
    angle = 3度， L = 0.523mm

kerf = 5mm
    angle = 1度， L = 0.087mm
    angle = 3度， L = 0.261mm

kerf = 3mm
    angle = 1度， L = 0.052mm
    angle = 3度， L = 0.157mm
***************************************************************************************/
double kerf_small_angle_threshold = 0.01745*3;
double curr_angle_change;

double arc_radius_accuracy = 0.5;                        //圆弧半径误差允许范围
double arc_fit_to_line_accuracy = 0;                   //圆弧拟合成直线精度误差
double arc_center_adjust_accuracy = 0.1;                 //圆心调整精度误差

size_t GfileTotalRows = 80000;

double myatan2(double y, double x) {
  double ang = 0.;
  ang = (y != 0 || x != 0) ? atan2(y, x) : 0;
  if (ang < 0) {
    ang += _2PI;
  }
  return ang;
}

double maxfabs(double a, double b) {
  return fabs(a) > fabs(b) ? fabs(a) : fabs(b);
}

double minfabs(double a, double b) {
  return fabs(a) > fabs(b) ? fabs(b) : fabs(a);
}

double dist(double x, double y, double x1, double y1) {
  return sqrt((x - x1) * (x - x1) + (y - y1) * (y - y1));
}

int CalcRoots(double a, double b, double c, double *root1, double *root2) {
  if (IsZero(a)) {
    if (IsZero(b)) {
      return IsZero(c) ? -1 : 0;
    } else {
      *root1 = -c / b;
      return 1;
    }
  } else {
    double delta = b * b - 4 * a * c;
    
    if (IsLesser(delta, 0.)) { // no root
      return 0;
    } else if (IsZero(delta)) {
      *root1 = -b / (2 * a);
      return 1;
    } else {
      delta = sqrt(delta);
      *root1 = (-b + delta) / (2 * a);
      *root2 = (-b - delta) / (2 * a);
      return 2;
    }
  }
}
//lead_type 0--交点靠近圆弧引入点 1--交点靠近圆弧引出点
/**********************************************************/
int if_in_arc_ccw(double ang_temp,double ang_str,double ang_end,int lead_type)
{
	/*ang_temp用来看是否在圆弧内，已经将角度变为+但360度附近还是*/
	/*要注意，参考画圆部分*/
	/*如果str>end则temp>str||temp<end即可*/
	/*如果str<end则str<temp<end即可*/
	/*
  if (ang_str<0) ang_str+=6.28;
	if (ang_end<0) ang_end+=6.28;
	if (ang_temp<0) ang_temp+=6.28;
	if (ang_str>ang_end)
	{
		if (ang_temp>ang_str||ang_temp<ang_end)  return 0;
	}
	else if (ang_str<ang_end)
	{
		if (ang_temp>=ang_str&&ang_temp<=ang_end)  return 0;
	}
	else
		return 0;
	
	return 1;
  */
	if (IsLesser(ang_str ,0.)) {
    ang_str += 2. * PI;
  }
  if (IsLesser(ang_end ,0.)) {
    ang_end += 2. * PI;
  }
  if (IsLesser(ang_temp, 0.)) {
    ang_temp += 2. * PI;
  }
  if (IsEqual(ang_str, ang_end)) {
    ang_end += 2. * PI;
    if(ang_temp>ang_str && lead_type==1)
      ang_temp += 2. * PI;
  }
  if (IsGreater(ang_str, ang_end)) {
    if (IsGreater(ang_temp, ang_str) || IsLesser(ang_temp, ang_end)) {
      return 0;
    }
  } else if (IsLesser(ang_str, ang_end)) {
    if (!IsLesser(ang_temp, ang_str) && !IsGreater(ang_temp, ang_end)) {
      return 0;
    }
  } else {
    return 0;
  }

  return 1;
}
//lead_type 0--交点靠近圆弧引入点 1--交点靠近圆弧引出点
int if_in_arc_cw(double ang_temp,double ang_str,double ang_end,int lead_type)
{
	/*ang_temp用来看是否在圆弧内，已经将角度变为+但360度附近还是*/
	/*要注意，参考画圆部分*/
	/*如果str>end则str>temp>end即可*/
	/*如果str<end则temp<str||temp>end即可*/
	/*
	if (ang_str<0) ang_str+=_2PI;
	if (ang_end<0) ang_end+=_2PI;
	if (ang_temp<0) ang_temp+=_2PI;
	
	if (ang_str>ang_end)
	{
		if (ang_temp<=ang_str&&ang_temp>=ang_end)  return 0;
	}
	else if (ang_str<ang_end)
	{
		if (ang_temp<ang_str||ang_temp>ang_end)  return 0;
	}
	else
		return 0;
	
	return 1;*/
	if (IsLesser(ang_str, 0.)) {
    ang_str += 2. * PI;
  }
  if (IsLesser(ang_end, 0.)) {
    ang_end += 2. * PI;
  }
  if (IsLesser(ang_temp, 0.)) {
    ang_temp += 2. * PI;
  }
  if (IsEqual(ang_str, ang_end)) {
    ang_end -= 2. * PI;
    if(ang_temp<ang_str&& lead_type==1)
      ang_temp -= 2. * PI;
  }
  if (IsGreater(ang_str, ang_end)) {
    if (!IsGreater(ang_temp, ang_str) && !IsLesser(ang_temp, ang_end)) {
      return 0;
    }
  } else if (IsLesser(ang_str, ang_end)) {
    if (IsLesser(ang_temp, ang_str) || IsGreater(ang_temp, ang_end)) {
      return 0;
    }
  } else {
    return 0;
  }

  return 1;
}

/*
	x0,y0 start(x,y)
	x1,y1 end(x,y)
	cx,cy end(x,y)
	px,py test(x,y)		//逆时针圆弧
*/
/**********************************************************************************************************
函数功能: 判断点是否在逆时针的圆弧上, 注意，该函数只能用来判断逆时针的圆弧，如果圆弧是顺时针的，则需要调换起点和末点的坐标

参数:
    x0,y0 : 逆时针圆弧的起点坐标
    x1,y1 : 逆时针圆弧的末点坐标
    cx,cy : 逆时针圆弧的圆心坐标

    px,py : 测试点的坐标

返回值:
        1 : 点在圆弧上
        0 : 点不在圆弧上
**********************************************************************************************************/
int if_point_in_arc(double x0,double y0, double x1,double y1, double cx,double cy,double px,double py)
{
	double ang_str,ang_end,ang_pt;
	double d_t1,d_t2,d_t3,d_t4;
	d_t1 = x1-cx;     // 由终点相对起点坐标转换成终点相对圆心坐标
  d_t2 = y1-cy;
  d_t3 = x0-cx;   // 由圆心相对起点坐标转换成起点相对圆心坐标
  d_t4 = y0-cy;
	ang_end = myatan2(d_t2,d_t1);  
	ang_str = myatan2(d_t4, d_t3);
	d_t3 = px-cx;   
  d_t4 = py-cy;
	ang_pt = myatan2(d_t4, d_t3);

    while(ang_str<0)
    ang_str+=_2PI;
    while(ang_str>_2PI)
    ang_str-=_2PI;

    while(ang_end<0)
    ang_end+=_2PI;
    while(ang_end>_2PI)
    ang_end-=_2PI;
    
    while(ang_pt<0)
    ang_pt+=_2PI;
    while(ang_pt>_2PI)
    ang_pt-=_2PI;
 	/*2014.6.16if(ang_end<ang_str) {
     ang_end += _2PI;
    }
    d_t1 = (ang_end-ang_pt)*(ang_pt-ang_str);
	if(d_t1>=0)
		return 1;
	else
		return 0;
		*/
    d_t1 = ang_end-ang_str;
    if(d_t1 <= EP)
    	d_t1 += _2PI;

   	d_t2 = ang_pt-ang_str;
    if(d_t2<=0)
    	d_t2+=_2PI;
    if(d_t1>=d_t2||fabs(ang_pt-ang_end)<=0.00002||fabs(ang_pt-ang_str)<=0.00002)
    return 1;
    else
    return 0;
	
}

/**********************************************************************************************************
函数功能: 判断点是否在圆弧上

参数:
    px,py : 测试点的坐标
    
    arc_gcode_name: 圆弧G代码指令 G02:顺时针圆弧  G03:逆时针圆弧
    x0,y0 : 圆弧的起点坐标
    x1,y1 : 圆弧的末点坐标
    cx,cy : 圆弧的圆心坐标

返回值:
        1 : 点在圆弧上
        0 : 点不在圆弧上
**********************************************************************************************************/
int is_point_in_arc (double px,double py, int arc_gcode_name, double x0,double y0, double x1,double y1, double cx,double cy)
{
    if (arc_gcode_name == G02)
    {
        return if_point_in_arc (x1, y1, x0, y0, cx, cy, px, py);
    }
    else
    {
        return if_point_in_arc (x0, y0, x1, y1, cx, cy, px, py);
    }
}

/*
int if_in_line(double ang1x,double ang1y,double ang2x,double ang2y,double ang3x,double ang3y)
{
	double R,R2,R3;

	R=pow(ang2x-ang3x,2)+pow(ang2y-ang3y,2);
	R2=pow(ang1x-ang2x,2)+pow(ang1y-ang2y,2);
	R3=pow(ang1x-ang3x,2)+pow(ang1y-ang3y,2);

	if (R>R2&&R>R3)
		return 0;
	else
		return 1;
}*/
/******************************************************************************************
函数功能:判断点是否在线段范围内，
注意    :该函数不判断当前点是否在直线上

参数: 
    pointx,pointy : 测试点的坐标
    
    x0,y0 : 线段的起点坐标
    x,y   : 线段的末点坐标

返回值:
    1  : 点在线段范围内
    0  : 点不在线段范围内
******************************************************************************************/
int point_in_line(double pointx,double pointy,double x0,double y0,double x,double y)
{
	double R,R2;
	R = (x-pointx)*(pointx - x0);
	R2 =(y-pointy)*(pointy-y0);
	if(!IsLesser(R, 0) && !IsLesser(R2,0))  //点在线段上
		return 1;
	else
		return 0;
	
}

int if_in_line_line(double ang1x,double ang1y,double ang2x,double ang2y,double ang3x,double ang3y)
{
	double R,R2,R3;

	R=pow(ang2x-ang3x,2)+pow(ang2y-ang3y,2);
	R2=pow(ang1x-ang2x,2)+pow(ang1y-ang2y,2);
	R3=pow(ang1x-ang3x,2)+pow(ang1y-ang3y,2);

	if (R>R2&&R>R3)
		return 0;
	else if (R2>R3)
	       return 0;
	else
		return 1;
}

/*
返回值 :
  0 平行
  1 有交点
*/
/***********************************************************************************************
函数功能: 判断两条直线位置关系

参数:
    x0,y0 : 第一条线段的起点
    x1,y1 : 第一条线段的末点
    
    x2,y2 : 第二条线段的起点
    x3,y3 : 第二条线段的末点
    
    InterX, InterY : 交点坐标 (只有在返回值为1时才有效，交点坐标不一定在线段上，还需要进一步判断)

返回值:
    0 : 平行
    1 : 有交点
    -1: 错误
***********************************************************************************************/
int TwoLineIsIntersect(double   x0,   double   y0,   double   x1,   double   y1,   double   x2,   double   y2,   double   x3,   double   y3,   double   *InterX,   double   *InterY)   
{ //两条线段是否相交X0X1   AND   X1X2   
  //double   x,   y;   
  #define Min(v0,v1) ((v0>v1) ? v1 : v0)
  #define Max(v0,v1) ((v0>v1) ? v0 : v1)
  double   Minx01   =   Min(x0,   x1);   
  double   Miny01   =   Min(y0,   y1);   
  double   Minx23   =   Min(x2,   x3);   
  double   Miny23   =   Min(y2,   y3);   
  double   Maxx01   =   Max(x0,   x1);   
  double   Maxy01   =   Max(y0,   y1);   
  double   Maxx23   =   Max(x2,   x3);   
  double   Maxy23   =   Max(y2,   y3);   
  double   k1;   
  double   k2;   
  //double   Den;
	if(fabs(x0-x1)>=fabs(y0-y1)&&fabs(x2-x3)>=fabs(y2-y3))
	{
		k1 = (y0-y1)/(x0-x1);
		k2 = (y2-y3)/(x2-x3);
		if(IsEqual(k1,k2))
		{
			if(IsEqual(x1,x2)&&IsEqual(y1,y2))
			{
				*InterX = x1;
				*InterY = y1;
				return 1;
			}
			else
				return 0;
		}
		
	}
	else if(fabs(x0-x1)<=fabs(y0-y1)&&fabs(x2-x3)<=fabs(y2-y3))
	{
		k1 = (x0-x1)/(y0-y1);
		k2 = (x2-x3)/(y2-y3);
		if(IsEqual(k1,k2))
		{
			if(IsEqual(x1,x2)&&IsEqual(y1,y2))
			{
				*InterX = x1;
				*InterY = y1;
				return 1;
			}
			else
				return 0;
		}
		
	}

	//else
	{
		if(IsZero(x1-x0))
		{
			*InterX = x1;
			if(IsZero(x3-x2))
				return 0;
			
			k1 = (y3-y2)/(x3-x2);
			*InterY = y2+k1*(*InterX-x2);
			return 1;
		}
		else if(IsZero(x3-x2))
		{
			*InterX = x2;
			if(IsZero(x1-x0))
				return 0;
			
			k1 = (y1-y0)/(x1-x0);
			*InterY = y0+k1*(*InterX-x0);
			return 1;
		}
		else
		{
			k1 = (y0-y1)/(x0-x1);
			k2 = (y2-y3)/(x2-x3);
			*InterX = (y2-y0 +k1*x0-k2*x2)/(k1-k2);
			//*InterY = y0+k1*(*InterX-x0);
			*InterY  = (k1*k2*(x2-x0)+k2*y0-k1*y2)/(k2-k1);
			return 1;
		}
	}
	return -1;
}

/*
	返回值:
		0-没有交点，即相离
		1-一个交点，即相切
		2-二两个交点，即相交
		
*/
/************************************************************************************************
函数功能: 计算线段与圆弧之间的位置关系

参数: 
    x0,y0  : 线段起点坐标
    x,y    : 线段末点坐标
    xc0,yc0: 圆弧圆心坐标
    Radius : 圆弧半径
    InterX1,InterY1: 线段所在的直线与圆弧的第一个交点 (返回值为1或2时有效)
    InterX2,InterY2: 线段所在的直线与圆弧的第二个交点
 (返回值为1或2时有效)

返回值:
    0 : 没有交点，相离
    1 : 有一个交点，相切
    2 : 有两个交点，相交
************************************************************************************************/
int LineCircleIntersect(double x0,double y0,double x,double y, double xc0,double yc0,double Radius,double *InterX1,double *InterY1,double *InterX2,double *InterY2)
{
	int res=0;
	double k1=1,temp1;
	double x_intersect1 = 0,y_intersect1 = 0;
	double x_intersect2 = 0,y_intersect2 = 0;
	double a,b,c,bb4ac;//ax*x+bx+c=0
	/*x=(-b+-sqrt(b*b-4ac))/2a*/
	/*先判断b*b-4ac>0?*/
	/*>0:直线与园相交*/
	/*=0:直线与园相切*/
	/*<0:直线与园相离即外割逢*/

	//if(fabs(x-x0)>fabs(y-y0))
	if (IsGreater (fabs(x-x0), fabs(y-y0)))
	{
		k1 = (y-y0)/(x-x0);
		//y = k1*x-k1*x0+y0代入圆方程,求解X
		temp1 = y0 - k1*x0;
		a = 1+k1*k1;
		b = -2*xc0+2*k1*(temp1-yc0);
		c = pow(xc0,2)  + pow(temp1-yc0,2)-pow(Radius,2);
              bb4ac = pow(b,2)-4*a*c;

		//if(bb4ac>0.1) //相交
		if (IsGreater (bb4ac, 0))
		{
			res = 2;
			*InterX1 = (-b+sqrt(bb4ac))/2/a;
			*InterX2 = (-b-sqrt(bb4ac))/2/a;
			*InterY1 = k1*(*InterX1)+temp1;
			*InterY2 = k1*(*InterX2)+temp1;
		}
		else if (IsLesser (bb4ac, 0))
		//else if(bb4ac<0.001)  //相离
		{
			res = 0;
		}
		else 
		{
			res = 1;
			*InterX1 = *InterX2 = (-b)/2/a;
			*InterY1 = *InterY2 = k1*(*InterX2)+temp1;
		}
	}
	else
	{
		k1 = (x-x0)/(y-y0);
		//y = k1*x-k1*x0+y0代入圆方程,求解X
		temp1 = x0 - k1*y0;
		a = 1+k1*k1;
		b = -2*yc0+2*k1*(temp1-xc0);
		c = pow(yc0,2)  + pow(temp1-xc0,2)-pow(Radius,2);
              bb4ac = pow(b,2)-4*a*c;

		//if(bb4ac>0.1) //相交
		if (IsGreater (bb4ac, 0))
		{
			res = 2;
			*InterY1= (-b+sqrt(bb4ac))/2/a;
			*InterY2 = (-b-sqrt(bb4ac))/2/a;
			*InterX1 = k1*(*InterY1)+temp1;
			*InterX2 = k1*(*InterY2)+temp1;
		}
		//else if(bb4ac<0.001)  //相离
		else if (IsLesser (bb4ac, 0))
		{
			res = 0;
		}
		else 
		{
			res = 1;
			*InterY1 = *InterY2 = (-b)/2/a;
			*InterX1 = *InterX2 = k1*(*InterY2)+temp1;
		}
	}
	return res;

}

int circle_intersect(Circle_t A, Circle_t B, Point_t *ia, Point_t *ib) {
  double a, b, c, d, k, aa, bb, cc, dd, drt;
  if (IsEqual(A.x, B.x) && IsEqual(A.y, B.y)) {
    return 5; // 同心圆
  }
  dd = dist(A.x, A.y, B.x, B.y);
  if (A.r + B.r + EP < dd) {
    return 1;  // 相离
  }
  a = B.x - A.x;
  b = B.y - A.y;
  c = B.r;
  k = A.r;
  d = sqr(c) - sqr(k) - sqr(a) - sqr(b);

  aa = 4 * sqr(a) + 4 * sqr(b);
  bb = 4 * b * d;
  cc = sqr(d) - 4 * sqr(a) *sqr(k);

  drt = sqr(bb) - 4 * aa * cc;
  if (drt < 0) {
    return 5; // 无解
  }
  drt = sqrt(drt);
  ia->y = (-bb + drt) / 2 / aa;
  ib->y = (-bb - drt) / 2 / aa;
  if (fabs(a) < EP) {
    ia->x = sqrt(sqr(k) - sqr(ia->y));
    ib->x = -ia->x;
  } else {
    ia->x = (2 * b * ia->y + d) / -2 / a;
    ib->x = (2 * b * ib->y + d) / -2 / a;
  }
  ia->x += A.x;
  ia->y += A.y;
  ib->x += A.x;
  ib->y += A.y;
  if (fabs(ia->y - ib->y) < EP) {
    if (fabs(A.r + B.r - dd) < EP) {
      return 2; // 外切一个交点
    }
    if (fabs(dd - (maxfabs(A.r, B.r) - minfabs(A.r, B.r))) < EP) {
      return 3; // 内切一个交点
    }
  }
  return 4; // 2个交点
}

int circle_intersect2(Circle_t A, Circle_t B, Point_t *ia, Point_t *ib)
{
	 double dd;
	 double cos_x,sin_x,cos_a,sin_a;
	 int res;
    if (IsEqual(A.x,B.x)&&IsEqual(A.y,B.y))//同心
        return 5;
    dd = dist (A.x, A.y, B.x, B.y);
    if (A.r + B.r + EP < dd||dd<EP)  //相离
        return 1;//没有交点
    if((fabs(A.r-B.r)-EP)>dd)  //相含
    	return 1; //included by another circle. 

    cos_x = (B.x-A.x)/dd;
    sin_x = (B.y-A.y)/dd;
   
    cos_a = (pow(A.r,2)+pow(dd,2)-pow(B.r,2))/(2*A.r*dd)  ;
    sin_a = sqrt(1-pow(cos_a,2));

    if(dd>=(fabs(A.r-B.r)-EP)&&dd<=(fabs(A.r-B.r)+EP))// 内切一个点
	{
		res=3;	
	}
	else if(dd>=(fabs(A.r+B.r)-EP)&&dd<=(fabs(A.r+B.r)+EP))// 外切一个点
	{
		res=2;
	}
	else
	{
		res=4;
	}
	ia->x = A.x + A.r*(cos_x*cos_a-sin_x*sin_a);
  ia->y = A.y + A.r*(sin_x*cos_a+cos_x*sin_a);
  ib->x = A.x + A.r*(cos_x*cos_a+sin_x*sin_a);
  ib->y = A.y + A.r*(sin_x*cos_a-cos_x*sin_a);
    
	return res;
}

/*
返回值 :
	1,5-相离
	2-外切
	3-内切
	4-两个交点
*/
int CircleIntersector(double x1, double y1, double r1,
                      double x2, double y2, double r2, 
                      double *ix1, double *iy1, 
                      double *ix2, double *iy2) {

  double dd = dist (x1, y1, x2, y2);
  double a;
  double b;
  double c;
  double p;
  double q;
  int rtn;
  
  Circle_t A, B;
  Point_t PointA, PointB;
  if (IsLesser(r1 + r2, dd)) {
    return 1; // 相离
  }

  if (IsLesser(dd, fabs(r1 - r2)) || (IsEqual(x1, x2) && IsEqual(y1, y2) && IsEqual(r1, r2))) {
    return 5; // 相含
  }

  a = 2. * r1 * (x1 - x2);
  b = 2. * r1 * (y1 - y2);
  c = sqr(r2) - sqr(r1) - sqr(x1 - x2) - sqr(y1 - y2);
  p = sqr(a) + sqr(b);
  q = -2. * a * c;
  
  if((1 - sqr(-q / p / 2))<0) // 几乎相切或相交，但实际相离的情况
		return 1;
  // 相切
  if (IsEqual(dd, fabs(r1 - r2)) || IsEqual(dd, r1 + r2)) {
    if(IsEqual(dd, fabs(r1 - r2)))
    {
        if(r1>r2)
        {
            *ix1 = *ix2 = x2+r2*(x2-x1)/(r1-r2);
            *iy1 = *iy2 = y2+r2*(y2-y1)/(r1-r2);
        }
        else
        {
            *ix1 = *ix2 = x1+r1*(x1-x2)/(r2-r1);
            *iy1 = *iy2 = y1+r1*(y1-y2)/(r2-r1);
        }
        return 3;
    }
    else
    {
        *ix1 = *ix2 = x1+r1*(x2-x1)/(r1+r2);
        *iy1 = *iy2 = y1+r1*(y2-y1)/(r1+r2);
        return 2;
    }
    
   /* c1 = -q / p / 2.;
    s1 = sqrt(1 - sqr(c1));
    *ix1 =  r1 * c1 + x1;
    *iy1 =  r1 * s1 + y1;
    if (!IsEqual(sqr(*ix1 - x2) + sqr(*iy1 - y2), sqr(r2))) {
      *iy1 = -r1 * s1 + y1;
    }
    *ix2  = *ix1;
    *iy2 = *iy1;
    return IsEqual(dd, fabs(r1 - r2)) ? 3 : 2;*/
  }
  A.x = x1;
  A.y = y1;
  A.r = r1;
  B.x = x2;
  B.y = y2;
  B.r = r2;
  rtn = circle_intersect(A,B,&PointA,&PointB);
  if(rtn ==1)
  {
    //同心圆
    return 5;
  }
  else if(rtn==4)//两个交点
  {
      *ix1 = PointA.x;
      *iy1 = PointA.y;
      *ix2 = PointB.x;
      *iy2 = PointB.y;
      return 4;
  }
  else if(rtn==2)//外切一个交点
  {
      *ix1 = PointA.x;
      *iy1 = PointA.y;
      *ix2 = PointB.x;
      *iy2 = PointB.y;
      return 2;
  }
  else if(rtn ==3 )//内切一个交点
  {
      *ix1 = PointA.x;
      *iy1 = PointA.y;
      *ix2 = PointB.x;
      *iy2 = PointB.y;
      return 3;
  }
  else
  {
      //fault
      return 1;
  
  }
}

/*G代码G02 G03 的半径到相对坐标变换
    Name:    G02 或 G03
    R:            圆的半径，如果大于0，则为小于180度的圆弧，如果小于0，则为大于180的圆弧
    StartP:  G02或G03的起点绝对坐标 
    EndP :   G02或G03的终点绝对坐标 
    *ResP:   返回要求的圆心绝对坐标
*/
unsigned int RToIJ(int Name, double R, Point_t StartP, Point_t EndP, Point_t *ResP)
{
	Point_t PtA,PtB;
	Circle_t A,B;
	unsigned int res;
	double x,y,hx,StartAOfCircleA,EndAOfCircleA;
	
	x = sqrt((EndP.x-StartP.x)*(EndP.x-StartP.x)+(EndP.y-StartP.y)*(EndP.y-StartP.y));
	y = 2*fabs(R);
	if (IsGreater (x, y))		//两点之间的距离超过了圆的直径，返回错误
	{
		return 1;			
	}
	else if (IsEqual (x, y))	//两点之间的距离等于圆的直径，圆心坐标唯一确定
	{
		ResP->x = (StartP.x + EndP.x) / 2;
		ResP->y = (StartP.y + EndP.y) / 2;
		return 0;				//圆心确定，返回
	}

	  
	  if(x>(y+EP))
		  return 1;
      A.x = StartP.x;
      A.y = StartP.y;
      B.r =  A.r = fabs(R);
      B.x = EndP.x;
      B.y  = EndP.y;

      res = circle_intersect2(A,B,&PtA,&PtB);
       if (res==2||res==3) 
       {
             *ResP = PtA;                  
       }
       else if(res==4)
       {

             StartAOfCircleA = myatan2(StartP.y-PtA.y,StartP.x-PtA.x);
             EndAOfCircleA = myatan2(EndP.y-PtA.y,EndP.x-PtA.x);
            if(Name == G02) //G02,顺圆
			{
				if(R>0)
				{
					/*
					//＜１８０°圆弧
					if(((CircleA.x - hx)*(x-hx)<=0) && ((CircleA.y - hy)*(y-hy)<=0))
					{
                         *ResP = CircleA;//  所求圆心
					} 
					else
					{
						*ResP = CircleB;
					}
				   	*/
				   	if(StartAOfCircleA < EndAOfCircleA)
				   	StartAOfCircleA += _2PI;
				   	hx = StartAOfCircleA - EndAOfCircleA;
				   	
				   	if(hx<(PI-EP))
				   	{
				   		*ResP = PtA;
				   	}
				   	else if(hx<(PI+EP))
				   	{
				   		return 0; //错误，两个圆弧大小都是180度的圆弧
				   	}
				   	else
				   	{
				   		*ResP = PtB;
				   	}
				}
				else 
				{
					//>１８０°圆弧
				/*	if(((CircleA.x - hx)*(x-hx)>=0) && ((CircleA.y - hy)*(y-hy)>=0))
					{
                         *ResP = CircleA;//  所求圆心
					} 
					else
					{
						*ResP = CircleB;
					}*/
					if(StartAOfCircleA < EndAOfCircleA)
				   	StartAOfCircleA += _2PI;
				   	hx = StartAOfCircleA - EndAOfCircleA;
				   	
				   	if(hx>(PI+EP))
				   	{
				   		*ResP = PtA;
				   	}
				   	else if(hx>(PI-EP))
				   	{
				   		return 0; //错误，两个圆弧大小都是180度的圆弧
				   	}
				   	else
				   	{
				   		*ResP = PtB;
				   	}
				}
             // else 
               //   return 1;
			}
			else  //G03 , 逆圆
			{
				if(R<0)
				{
					/*
					//>１８０°圆弧
					if(((CircleA.x - hx)*(x-hx)<=0) && ((CircleA.y - hy)*(y-hy)<=0))
					{
                         *ResP = CircleA;//  所求圆心
					} 
					else
					{
						*ResP = CircleB;
					}*/
				   	if(EndAOfCircleA < StartAOfCircleA)
				   	EndAOfCircleA += _2PI;
				   	hx = EndAOfCircleA - StartAOfCircleA;
				   	
				   	if(hx>(PI+EP))
				   	{
				   		*ResP = PtA;
				   	}
				   	else if(hx>(PI-EP))
				   	{
				   		return 0; //错误，两个圆弧大小都是180度的圆弧
				   	}
				   	else
				   	{
				   		*ResP = PtB;
				   	}
				}
				else
				{
					//<１８０°圆弧
					/*if(((CircleA.x - hx)*(x-hx)>=0) && ((CircleA.y - hy)*(y-hy)>=0))
					{
                         *ResP = CircleA;//  所求圆心
					} 
					else
					{
						*ResP = CircleB;
					}*/
					if(EndAOfCircleA < StartAOfCircleA)
				   	EndAOfCircleA += _2PI;
				   	hx = EndAOfCircleA - StartAOfCircleA;
				   	
				   	if(hx<(PI-EP))
				   	{
				   		*ResP = PtA;
				   	}
				   	else if(hx<(PI+EP))
				   	{
				   		return 0; //错误，两个圆弧大小都是180度的圆弧
				   	}
				   	else
				   	{
				   		*ResP = PtB;
				   	}
				}

			}
       }
       else
		   return 1;
  // ResP->x = ResP->x - StartP.x;
  // ResP->y = ResP->y - StartP.y;
  return 0;     
}

namespace GCode {

Kerf::Kerf(): param_kerf(0.)
            , handler_mode_(0) {}

Kerf::~Kerf() {}

void Kerf::SetKerfValue(double kerf_value) {
  param_kerf = kerf_value;
}

void Kerf::SetKerfHandlerMode(int handler_mode) {
  handler_mode_ = handler_mode;
}

void Kerf::CircleCheFen(std::vector<GCodeStruct> &GCodeArry)
{
	double StartAngle,EndAngle,StepAngle,CurrentAngle;
	double d_t1,d_t2,d_t3,d_t4;
  std::vector<GCodeStruct> GCodeCheFen;
  std::vector<GCodeStruct>::iterator GCodeArryPtrSrc = GCodeArry.begin();
  std::vector<GCodeStruct>::iterator GCodeArryCheFen = GCodeCheFen.begin();
	
	while(GCodeArryPtrSrc->Name!=M02)
	{
    GCodeCheFen.push_back(*GCodeArryPtrSrc);
		if(GCodeArryPtrSrc->Name==G02||GCodeArryPtrSrc->Name==G03)
		{
			if(GCodeArryPtrSrc->R>50000)
			{
				d_t1 = GCodeArryPtrSrc->X-GCodeArryPtrSrc->I;     // 由终点相对起点坐标转换成终点相对圆心坐标
  			d_t2 = GCodeArryPtrSrc->Y-GCodeArryPtrSrc->J;
  			d_t3 = GCodeArryPtrSrc->X0 -GCodeArryPtrSrc->I;   // 由圆心相对起点坐标转换成起点相对圆心坐标
    		d_t4 = GCodeArryPtrSrc->Y0 -GCodeArryPtrSrc->J;
    		StartAngle= myatan2(d_t4, d_t3);
    		EndAngle= myatan2(d_t2, d_t1);
	    		
  		  //StepAngle = 0.00087-0.00076*((GCodeArryPtrSrc->R-50000)/100000000);
  		  if(GCodeArryPtrSrc->R>500000)
  		  {
  			  //StepAngle = 5/GCodeArryPtrSrc->R;// 1-0.1 2-0.2
  			  StepAngle = 2*sqrt(2*0.1*GCodeArryPtrSrc->R-0.1*0.1)/GCodeArryPtrSrc->R;// pricise 0.1mm
  		  }
  		  else
  			  StepAngle = 2*((2*sqrt((100*GCodeArryPtrSrc->R)*(GCodeArryPtrSrc->R)-(GCodeArryPtrSrc->R*10-1)*(GCodeArryPtrSrc->R*10-1)))/GCodeArryPtrSrc->R/10);// 1-0.1 2-0.2
  		  //StepAngle = arccos(1-0.1/GCodeArryPtrSrc->R);
  		  if(GCodeArryPtrSrc->Name==G02)
  		  {
  			  if(StartAngle<=EndAngle+EP)
  				  StartAngle+=_2PI;
  		  }
  		  else if(GCodeArryPtrSrc->Name==G03)
  		  {
  			  if(StartAngle>=EndAngle+EP)
  				  StartAngle-=_2PI;
  		  }
  		  CurrentAngle = StartAngle;
  		  do{
  			  if(GCodeArryPtrSrc==GCodeArry.begin())
  			  {
  				  GCodeArryCheFen->X0=0;
  				  GCodeArryCheFen->Y0=0;
  			  }
  			  else
  			  *GCodeArryCheFen=*(GCodeArryCheFen-1);
  			  GCodeArryCheFen->Name = G01;
  			  GCodeArryCheFen->ShowLine = GCodeArryPtrSrc->ShowLine;
  			  //if(GCodeArryPtrSrc->F < 0.83)
  			  GCodeArryCheFen->F=0;
  			  GCodeArryCheFen->X0 = (GCodeArryCheFen-1)->X;
  			  GCodeArryCheFen->Y0 = (GCodeArryCheFen-1)->Y;
  			  if(GCodeArryPtrSrc->Name==G02)
  			  {
  				  d_t2 = CurrentAngle-EndAngle;
  				  if(d_t2>=_2PI)
  					  d_t2-=_2PI;
  				  else if(d_t2<=-_2PI)
  					  d_t2+=_2PI;
  				  if(d_t2<StepAngle)
  				  {
  					  GCodeArryCheFen->X = GCodeArryPtrSrc->X;
  					  GCodeArryCheFen->Y = GCodeArryPtrSrc->Y;
  					  CurrentAngle = EndAngle;
  				  }
  				  else
  				  {
  					  CurrentAngle -= StepAngle;
    				  if(CurrentAngle<=-_2PI)
    					  CurrentAngle-=_2PI;
    				  d_t2 = fabs(CurrentAngle-EndAngle);
    				  if(d_t2>=_2PI)
    					  d_t2-=_2PI;
    				  else if(d_t2<=-_2PI)
    					  d_t2+=_2PI;
    				  if(d_t2<=StepAngle)
    				  {
    					  GCodeArryCheFen->X = GCodeArryPtrSrc->X;
    					  GCodeArryCheFen->Y = GCodeArryPtrSrc->Y;
    					  CurrentAngle = EndAngle;
    				  }
    				  else
    				  {
    					  GCodeArryCheFen->X = GCodeArryPtrSrc->I+GCodeArryPtrSrc->R*cos(CurrentAngle);
    					  GCodeArryCheFen->Y = GCodeArryPtrSrc->J+GCodeArryPtrSrc->R*sin(CurrentAngle);
    				  }
  				  }
    				
  			  }
  			  else
  			  {
  				  d_t2 = EndAngle-CurrentAngle;
  				  if(d_t2>=_2PI)
  					  d_t2-=_2PI;
  				  else if(d_t2<=-_2PI)
  					  d_t2+=_2PI;
  				  if(d_t2<StepAngle)
  				  {
  					  GCodeArryCheFen->X = GCodeArryPtrSrc->X;
  					  GCodeArryCheFen->Y = GCodeArryPtrSrc->Y;
  					  CurrentAngle = EndAngle;
  				  }
  				  else
  				  {
    				  CurrentAngle += StepAngle;
    				  if(CurrentAngle>=_2PI)
    				  CurrentAngle-=_2PI;
    				  d_t2 = fabs(CurrentAngle-EndAngle);
    				  if(d_t2>=_2PI)
    					  d_t2-=_2PI;
    				  else if(d_t2<=-_2PI)
    					  d_t2+=_2PI;
    				  if(d_t2<=StepAngle)
    				  {
    					  GCodeArryCheFen->X = GCodeArryPtrSrc->X;
    					  GCodeArryCheFen->Y = GCodeArryPtrSrc->Y;
    					  CurrentAngle = EndAngle;
    				  }
    				  else
    				  {
    					  GCodeArryCheFen->X = GCodeArryPtrSrc->I + GCodeArryPtrSrc->R*cos(CurrentAngle);
    					  GCodeArryCheFen->Y = GCodeArryPtrSrc->J + GCodeArryPtrSrc->R*sin(CurrentAngle);
    				  }
  				  }
  			  }
  			  GCodeArryCheFen++;
        } while(CurrentAngle != EndAngle); 
		    GCodeArryCheFen--;
			}
		}
		GCodeArryPtrSrc++;
		GCodeArryCheFen++;
		
	}
	GCodeArryCheFen->Name = M02;	
  GCodeArry = GCodeCheFen;
}

int Kerf::IgnoreLittleLine(std::vector<GCodeStruct> &GCodeArry) {
	std::vector<GCodeStruct>::iterator GCodeArryPtrSrc = GCodeArry.begin();
	std::vector<GCodeStruct>::iterator GCodeArryLittelLine = GfileFloatKerf.begin();
  //GfileFloatTemp;

	int res=0; 
  double AccLength=0;

	while(GCodeArryPtrSrc->Name!=M02)
	{
		if(GCodeArryPtrSrc->Length<0.1&&AccLength<0.5&&(GCodeArryPtrSrc->X0!=0||GCodeArryPtrSrc->Y0!=0)&&(GCodeArryPtrSrc->Name>=G00&&GCodeArryPtrSrc->Name<=G03)&&GCodeArryPtrSrc>GCodeArry.begin()&&GCodeArryPtrSrc->Name==G01&&(GCodeArryPtrSrc-1)->Name==G01)
		{
		    AccLength += GCodeArryPtrSrc->Length;

			GCodeArryLittelLine--;
			
			GCodeArryLittelLine->X=(GCodeArryPtrSrc)->X;
			GCodeArryLittelLine->Y=(GCodeArryPtrSrc)->Y;
			//GCodeArryPtrSrc++;
			res = 1;
			
		}
		else
		{
		  AccLength = 0;
			*GCodeArryLittelLine=*GCodeArryPtrSrc;
			
		}
		GCodeArryLittelLine++;
		GCodeArryPtrSrc++;
	}
	GCodeArryLittelLine->Name=M02;
	
	GCodeArryLittelLine = GfileFloatKerf.begin();//GfileFloatTemp;
	GCodeArryPtrSrc = GCodeArry.begin();
	while(GCodeArryLittelLine->Name!=M02)
	{
		*GCodeArryPtrSrc++=*GCodeArryLittelLine++;
	}
	GCodeArryPtrSrc->Name=M02;
	return res;
}

/*
函数功能：计算各段长度，直线斜率和圆弧起始终止角度
参数:
	WithKerf : 0 未加过割缝的代码
             1 已加过割缝的代码
 return :
    1 -- source don't have G code
    2 --
*/
int Kerf::Cal_Length_Angle_R(std::vector<GCodeStruct> &GCodeArry, int WithKerf) {
	double little_arc=5,little_arc_spd_level=0;
	int FirstCurve=0; //是不是第一次遇到曲线，即G代码第一次遇到G00,01,02,03,26,27,28
	double LastEndTangent=0,CurrentStartTangent=0,CurrentEndTangent=0;
	int CheckKerf=0,IsKerf=0;
	std::vector<GCodeStruct>::iterator Ptr = GCodeArry.begin();

	if(Ptr->Name == M02)
	  return 1;

	while(Ptr->Name !=M02)
	{
		if((Ptr->Name==G00) ||(Ptr->Name==G01)||(Ptr->Name==G26)||(Ptr->Name==G27)||(Ptr->Name==G28) )
		{
			Ptr->Length = sqrt((Ptr->X-Ptr->X0)*(Ptr->X-Ptr->X0)+(Ptr->Y-Ptr->Y0)*(Ptr->Y-Ptr->Y0));
			Ptr->StartAngle=  myatan2(Ptr->Y-Ptr->Y0, Ptr->X-Ptr->X0);//XYToAngle(Ptr->X-Ptr->X0,Ptr->Y-Ptr->Y0,d_t1);
			Ptr->R = 0;
		}
		else if((Ptr->Name==G02) ||(Ptr->Name==G03) )
		{
			calculate_arc_length_radius_and_angle (*Ptr);
		}
		else
		{
			Ptr->Length = 0;
			Ptr->R = 0;
		}

		Ptr++;
	}
	return 0;
}

/*********************************************************************************************
函数功能: 计算圆弧的起始角度和末角度
*********************************************************************************************/
void Kerf::calculate_arc_length_radius_and_angle (GCodeStruct &PGCode) {
	double d_t1,d_t2,d_t3,d_t4;
	
	d_t1 = PGCode.X-PGCode.I;	  					// 由终点相对起点坐标转换成终点相对圆心坐标
	d_t2 = PGCode.Y-PGCode.J;
	d_t3 = PGCode.X0-PGCode.I;   					// 由圆心相对起点坐标转换成起点相对圆心坐标
	d_t4 = PGCode.Y0-PGCode.J;
	PGCode.R = sqrt(d_t3*d_t3+d_t4*d_t4);  		//半径
	PGCode.StartAngle= myatan2(d_t4, d_t3);
	PGCode.EndAngle= myatan2(d_t2, d_t1);
	
	while(PGCode.StartAngle>_2PI)
		PGCode.StartAngle -=_2PI;
	while(PGCode.StartAngle<EP)
		PGCode.StartAngle +=_2PI;
	if(fabs(PGCode.StartAngle)<EP)
		PGCode.StartAngle = 0;
	while(PGCode.EndAngle>_2PI)
		PGCode.EndAngle -=_2PI;
	while(PGCode.EndAngle<EP)
		PGCode.EndAngle +=_2PI;
	if(fabs(PGCode.EndAngle)<EP)
		PGCode.EndAngle = 0;

	if(PGCode.Name==G02)	 			//顺时针
	{
		if ((IsEqual (PGCode.X, PGCode.X0) && IsEqual (PGCode.Y, PGCode.Y0)) || IsEqual (PGCode.StartAngle, PGCode.EndAngle))
		{
			PGCode.StartAngle=PGCode.EndAngle+_2PI;
		}
		else
		{
			if (IsLesser (PGCode.StartAngle, PGCode.EndAngle))
				PGCode.StartAngle += _2PI;
		}
	}
	else //G03			//逆时针
	{
		if ((IsEqual (PGCode.X, PGCode.X0) && IsEqual (PGCode.Y, PGCode.Y0)) || IsEqual (PGCode.StartAngle, PGCode.EndAngle))
		{
			PGCode.EndAngle = PGCode.StartAngle+_2PI;
		}
		else
		{
			if (IsGreater (PGCode.StartAngle, PGCode.EndAngle))
				PGCode.EndAngle += _2PI;
		}
	}

	PGCode.Length = PGCode.R*fabs(PGCode.StartAngle - PGCode.EndAngle);		//计算弧长
}

/**********************************************************************************
函数功能: 根据给定的圆弧半径，重新计算圆心坐标

返回值:
	 0: 计算成功
	-1: 计算出错

注意:
    计算出的圆心偏差不在精度范围内，则报错，精度取r1,r2的差值与arc_accuracy/10之间的最大值
    优先进行圆心校正，因为相比末点校正，圆心校正不会增加G01代码段。同时也要保证圆心校正的精度。
    圆心校正的缺点是:当末点与起点接近时，圆心校正的误差会增大。所以取arc_accuracy/10的精度作为圆心校正的精度
**********************************************************************************/
static double re_r1, re_r2;
int Kerf::calculate_arc_center(GCodeStruct &PGCode, double r1, double r2) {
	double radius = fabs(r1+r2)/2;
	Point_t StartP, EndP, ResP;
  int ret;
  double new_arc_center_err;

  //确定圆心精度, 取arc_center_adjust_accuracy与r1,r2差值之间的最大值
  double accuracy = MAX(arc_center_adjust_accuracy, fabs (r1-r2));

	if (IsEqual(PGCode.X0, PGCode.X) && IsEqual(PGCode.Y0, PGCode.Y))				//整圆直接返回，无需校正
		return 0;
	
	//1. 计算圆弧为优弧还是劣弧
	calculate_arc_length_radius_and_angle(PGCode);
	if (IsGreater (fabs(PGCode.StartAngle - PGCode.EndAngle), PI))
		radius = -radius;

	//2. 重新计算圆弧圆心坐标
	StartP.x = PGCode.X0;
	StartP.y = PGCode.Y0;
	EndP.x   = PGCode.X;
	EndP.y   = PGCode.Y;
	ret = RToIJ (PGCode.Name, radius, StartP, EndP, &ResP);
	if (ret != 0)
		return -1;

	re_r1 = dist (PGCode.X0, PGCode.Y0, ResP.x, ResP.y);
	re_r2 = dist (PGCode.X, PGCode.Y, ResP.x, ResP.y);

  new_arc_center_err = dist(PGCode.I, PGCode.J, ResP.x, ResP.y);

  if (new_arc_center_err > arc_center_adjust_accuracy)
    return -1;                                                   //校正后误差过大

  //arc_info.arc_center_adjust_count++;
  //arc_info.arc_center_adjust_differ = new_arc_center_err;
  //if (IsLesser (arc_info.arc_max_center_err, new_arc_center_err))
  //  arc_info.arc_max_center_err = new_arc_center_err;

	PGCode.I = ResP.x;
	PGCode.J = ResP.y;
	return 0;
}

/**********************************************************************************
函数功能: 根据给定圆弧起点，圆心和弧度，计算圆弧末点

返回值:
	 0: 计算成功
	-1: 计算出错

注意:
    (考虑半径较大时，角度的误差，是否有必要重新计算一下圆心?)
**********************************************************************************/
int Kerf::calculate_arc_end(GCodeStruct &PGCode, double &xe, double &ye)
{
  double r1, r2;

  calculate_arc_length_radius_and_angle(PGCode);
  r1 = dist(PGCode.X0, PGCode.Y0, PGCode.I, PGCode.J);
  r2 = dist(PGCode.X, PGCode.Y, PGCode.I, PGCode.J);
  xe = r1 * (PGCode.X - PGCode.I) / r2 + PGCode.I;
  ye = r1 * (PGCode.Y - PGCode.J) / r2 + PGCode.J;

  //arc_info.arc_endpoint_adjust_differ = dist (PGCode.X, PGCode.Y, xe, ye);
  if (dist(PGCode.X, PGCode.Y, xe, ye) > arc_radius_accuracy)
    return -1;

  //arc_info.arc_endpoint_adjust_count++;
	return 0;	
}

/*********************************************************************************
函数功能: 检查g代码位置是否有偏差，偏差过大则报错提示

参数: 
	Ptr: G代码结构体数组指针

返回值:
	 0: 正常
	-1: 偏差过大
	-2: 无法修正

说明:
	当前行代码的末点坐标和后一行代码的起点坐标偏差在1mm以上，则直接报错
	偏差在1mm以内，则进行修正，修正时必须确保是在运动指令上，否则,直接报错。
	另外，优先在线段上修正坐标，一定不能在整圆上修正坐标，否则可能导致
	整圆变成小圆弧。如果是两个连续的整圆，坐标有偏差，则直接报错
	(注意如果是在G02G03上修正坐标，则会对圆弧精度产生影响,并且如果圆弧起末点
	 偏差不大，有可能导致小圆弧变成整圆或者整圆变成小圆弧!
	 所以，坐标位置检查后需要进行圆弧精度检查修正),	
*********************************************************************************/
static GCodeStruct err_gcode, err_gcode_next;
int Kerf::calibrate_gcode_position(std::vector<GCodeStruct> &GCodeArry) {
	#define IS_MOVE_GCODE(Name) (Name == G00 || Name == G01 || Name == G02 || Name == G03 || Name == G26 || Name == G27 || Name == G28)
	#define IS_ARC_GCODE(Name)	(Name == G02 || Name == G03)
	#define IS_LINE_GCODE(Name) (Name == G00 || Name == G01 || Name == G26 || Name == G27 || Name == G28)
	
	std::vector<GCodeStruct>::iterator PtrNext;
	double err_x, err_y;
	float MaxErrLength = 1.0;						//最大误差 1mm

  std::vector<GCodeStruct>::iterator Ptr = GCodeArry.begin();

	PtrNext = Ptr + 1;
	while (Ptr->Name != M02 && PtrNext->Name != M02)
	{
		//检查G代码当前行起点坐标和下一行末点坐标是否连续
    if (!(IsEqual (Ptr->X, PtrNext->X0) && IsEqual (Ptr->Y, PtrNext->Y0)))
    {
			err_x = Ptr->X - PtrNext->X0;
			err_y = Ptr->Y - PtrNext->Y0;
			err_gcode = *Ptr;			//debug use
			err_gcode_next = *PtrNext;	//debug use
			if (IsGreater (fabs (err_x), MaxErrLength) || IsGreater (fabs (err_y), MaxErrLength))		//横向或者纵向误差超过1mm，则报错
			{
				return -1;
			}
			else 
			{
				if (IS_LINE_GCODE (Ptr->Name))
				{
					Ptr->X = PtrNext->X0;
					Ptr->Y = PtrNext->Y0;
				}
				else if (IS_LINE_GCODE (PtrNext->Name))
				{
					PtrNext->X0 = Ptr->X;
					PtrNext->Y0 = Ptr->Y;
				}
				else if (IS_ARC_GCODE (Ptr->Name) && 
					     !(IsLesser (fabs(Ptr->X0-Ptr->X), 2*MaxErrLength) && IsLesser (fabs(Ptr->Y0-Ptr->Y), 2*MaxErrLength)))  		//非整圆或者小圆弧
				{
					Ptr->X = PtrNext->X0;
					Ptr->Y = PtrNext->Y0;
				}
				else if (IS_ARC_GCODE (PtrNext->Name) && 
					     !(IsLesser (fabs(PtrNext->X0-PtrNext->X), 2*MaxErrLength) && IsLesser (fabs(PtrNext->Y0-PtrNext->Y), 2*MaxErrLength)))  //非整圆或者小圆弧
				{
					PtrNext->X0 = Ptr->X;
					PtrNext->Y0 = Ptr->Y;
				}
				else
					return -2;
			}
    }
		else 
		{
			Ptr->X = PtrNext->X0;						//重新赋值，保证G代码位置绝对一致
			Ptr->Y = PtrNext->Y0;
		}

		Ptr++;
		PtrNext = Ptr + 1;
	}

	return 0;
}

/*********************************************************************************
函数功能: 校正圆弧

返回值:
	 0: 正常
	-1: 偏差过大，无法校正

优先校正圆心，圆心无法校正时，校正末点
*********************************************************************************/
int Kerf::calibrate_arc(std::vector<GCodeStruct> &GCodeArry) {
	double r1, r2;
	double xe, ye;
	double alpha;

  std::vector<GCodeStruct>::iterator Ptr = GCodeArry.begin();
	while (Ptr->Name != M02)
	{
		if (Ptr->Name == G02 || Ptr->Name == G03)
		{
			r1 = sqrt ((Ptr->X0 - Ptr->I) * (Ptr->X0 - Ptr->I) + (Ptr->Y0 - Ptr->J) * (Ptr->Y0 - Ptr->J));		//计算起点相对圆心半径
			r2 = sqrt ((Ptr->X - Ptr->I) * (Ptr->X - Ptr->I) + (Ptr->Y - Ptr->J) * (Ptr->Y - Ptr->J));			//计算末点相对圆心半径

            //1. 检查半径误差是否过大
            if (IsGreater (fabs (r1 - r2), arc_radius_accuracy))
            {
                return -1;                                      //误差过大
            }

            //2. 检查是否能拟合成小线段
            calculate_arc_length_radius_and_angle (*Ptr);
            alpha = (Ptr->Name == G02) ? (Ptr->StartAngle - Ptr->EndAngle) : (Ptr->EndAngle - Ptr->StartAngle);
            if (is_arc_in_fit_to_line_accuracy (Ptr->R, alpha) == 0)
            {
                Ptr->Name = G01;				//拟合成小线段
            }//3. 校准圆弧
            else if (calculate_arc_center (*Ptr, r1, r2) != 0)    //先校正圆心
            {
                if (calculate_arc_end (*Ptr, xe, ye) != 0)	//圆心无法校正，则校正末点
                {
                    return -1;
                }
                //插入一段G01补偿修正圆弧
                {
                    GCodeStruct insert_code = *Ptr;
                    Ptr = GCodeArry.insert(Ptr + 1, insert_code);
                    (Ptr-1)->X = xe;                //圆弧末点重置
                    (Ptr-1)->Y = ye;
                    Ptr->Name = G01;        //插入G01连接
                    Ptr->X0 = xe;
                    Ptr->Y0 = ye;
                    Ptr->R = 0;
                }
            }
		}
		Ptr++;
	}

	return 0;
}

/*********************************************************************************
函数功能: 检查g代码位置是否有偏差
返回值 :
    0 : 没有偏差
    >0: G代码偏差个数
*********************************************************************************/
int Kerf::check_gcode_position(std::vector<GCodeStruct> &GCodeArry)
{
  int err_position_cnt = 0;

  std::vector<GCodeStruct>::iterator Ptr = GCodeArry.begin();
	std::vector<GCodeStruct>::iterator PtrNext = Ptr + 1;
	while (Ptr->Name != M02 && PtrNext->Name != M02)
	{
		//检查G代码当前行起点坐标和下一行末点坐标是否连续
    if (!(IsEqual (Ptr->X, PtrNext->X0) && IsEqual (Ptr->Y, PtrNext->Y0)))
    {
        err_position_cnt++;
    }
		Ptr++;
		PtrNext = Ptr + 1;
	}

	return err_position_cnt;
}

/**************************************************************************************************
函数功能: 标记整圆
**************************************************************************************************/
void Kerf::whole_circle_mark(GCodeStruct &PGCode)
{
	//计算圆
	if (PGCode.Name == G02 || PGCode.Name == G03)
	{
		if (IsEqual(PGCode.X0, PGCode.X) && IsEqual(PGCode.Y0, PGCode.Y))
			PGCode.AngleRatio = 1;
		else
			PGCode.AngleRatio = 0;
	}
}

/**************************************************************************************************
函数功能: 清除整圆标记
**************************************************************************************************/
void Kerf::whole_circle_mark_clear(GCodeStruct &pGcode)
{
	if (pGcode.Name == G02 || pGcode.Name == G03)
	{
		pGcode.AngleRatio = 0;
	}
}

/**************************************************************************************************
函数功能: 判断整圆标记是否存在

返回值 :
	1 : 是整圆
	0 : 不是整圆
**************************************************************************************************/
int Kerf::is_whole_circle_marked (GCodeStruct &PGCode)
{
	if ((PGCode.Name == G02 || PGCode.Name == G03) && IsEqual (PGCode.AngleRatio, 1))
	{
		 return 1;
	}

	return 0;
}

/**************************************************************************************************
函数功能: 检查圆弧加割缝半径是否过小

参数:
	kerf_dir  : 割缝方向 G41:左割缝，G42:右割缝
    kerf_value: 割缝值
    GCodePtr  : 圆弧代码

返回值:
     0: 正常
    -1: 圆弧半径过小
**************************************************************************************************/
int Kerf::is_arc_radius_vaild_for_kerf (int kerf_dir, double kerf_value, const GCodeStruct &PGCode) {
  if ((PGCode.Name == G02 && kerf_dir == G42) ||
      (PGCode.Name == G03 && kerf_dir == G41)) {

      if (sqrt(pow(PGCode.X0-PGCode.I,2)+pow(PGCode.Y0-PGCode.J,2)) < kerf_value)
          return -1;
  }
    
  return 0;
}

/**********************************************************************************
检查加割缝前的G代码
返回值 :  
	ErrNo                      : 代码正常
	ERR_G_CODE_ErrG0203        : 圆弧错误

**********************************************************************************/
int Kerf::check_gcode_before_kerf()
{
  Cal_Length_Angle_R(GfileFloatNoKerf, 0);
  if (theApp.GetProfileInt("CommSet", "FitOption", 1) == 1) {
    double min_fit_length = atof(theApp.GetProfileString("CommSet", "MinFitLength", "0.1"));
    SimpleLinearFitting line_fit;
    line_fit.FitSmallLine(GfileFloatNoKerf, min_fit_length);
    Cal_Length_Angle_R(GfileFloatNoKerf, 0);
  }

  return ErrNo;
}

/**********************************************************************************
检查加割缝后的G代码
   
返回值 :  
	ErrNo                      : 代码正常
	ERR_G_CODE_POSITION	       : G代码绝对位置偏差过大
	ERR_G_CODE_WHOLE_CIRCLE    : 整圆补偿出错, 可以调节:kerf_small_angle_threshold
	ERR_G_CODE_ErrG0203        : 圆弧错误

**********************************************************************************/
int Kerf::check_gcode_after_kerf()
{
	//1. 检查并修正整圆
	if (calibrate_whole_circle() != 0)
		return ERR_G_CODE_WHOLE_CIRCLE;

  //2. 检查圆弧精度并校正圆弧
  if (calibrate_arc(GfileFloatKerf) != 0)
    return ERR_G_CODE_ErrG0203;

  //3. 检查g代码精度
  if (check_gcode_position(GfileFloatKerf) != 0)
    return ERR_G_CODE_POSITION;

	return ErrNo;
}

/**************************************************************************************************
函数功能: 检查并恢复整圆

参数:
	Ptr: G代码结构体指针

返回值 :
	0 : 正常恢复
	-1: 无法恢复整圆

说明: 
	该函数用于校正加割缝后变成小圆弧的整圆。加割缝前对整圆进行标记，
	加割缝，该函数会检查整圆标记是否有效，如果整圆标记有效，并且圆弧的角度小于5度，则认为
	整圆变成了小圆弧，插入一段小线段对整圆进行修正。
**************************************************************************************************/
int Kerf::calibrate_whole_circle()
{
	std::vector<GCodeStruct>::iterator PtrNext;
	static double angle_change = 0;
	double small_arc_angle = 3.0/180.0*PI;							//阈值角度为3度
	int err_whole_circle = 0;
	
  std::vector<GCodeStruct>::iterator Ptr = GfileFloatKerf.begin();
	PtrNext = Ptr + 1;
	while (Ptr->Name != M02 && PtrNext->Name != M02)
	{
		PtrNext = Ptr + 1;
		if (is_whole_circle_marked (*Ptr) == 1)						//整圆
		{
			calculate_arc_length_radius_and_angle(*Ptr);			//计算圆弧角度

			//整圆补偿后，圆弧角度小于5度，则认为补偿出错，插入一段小线段恢复整圆
			angle_change = fabs (Ptr->StartAngle - Ptr->EndAngle);
      if (IsLesser (angle_change, small_arc_angle) == 1)
      {
          //优先在G01段插入小线段
          if ((Ptr-1)->Name == G01 || ((Ptr+1)->Name != G01 && ((Ptr-1)->Name == G02 || (Ptr-1)->Name == G03)))
          {
              //在前一行插入小线段
              {
                  //插入小线段
                  GCodeStruct insert_code = *(Ptr+1);
                  Ptr = GfileFloatKerf.insert(Ptr, insert_code);
                  Ptr->Name = G01;
                  //圆弧末点修正
                  (Ptr+1)->X0 = (Ptr+1)->X;
                  (Ptr+1)->Y0 = (Ptr+1)->Y;
              }
          }
          else if ((Ptr+1)->Name == G01 || (Ptr+1)->Name == G02 || (Ptr+1)->Name == G03)
          {
              //在后一行插入小线段
              {
                  //插入小线段
                  GCodeStruct insert_code = *Ptr;
                  Ptr = GfileFloatKerf.insert(Ptr+1, insert_code);
                  (Ptr)->Name = G01;
                  //圆弧末点修正
                  (Ptr-1)->X = (Ptr-1)->X0;
                  (Ptr-1)->Y = (Ptr-1)->Y0;
              }
          }
          else
              return -1;          //前后都是非切割段，报错
      }
		}

		Ptr++;
		PtrNext = Ptr + 1;
	}

	return 0;
}

double Kerf::GetRadius(GCodeStruct &pGcode)
{
	return sqrt(pow(pGcode.X - pGcode.I, 2) + pow(pGcode.Y - pGcode.J, 2));
}

/*
	StartOrEnd:
	 0- startpoint
	 1 -endpoint
*/
double Kerf::GetTangent(GCodeStruct &pGcode, int StartOrEnd)
{
	double Tangent;
	if(pGcode.Name==G01)
	{
		Tangent = myatan2(pGcode.Y-pGcode.Y0, pGcode.X-pGcode.X0);
	}
	else if(pGcode.Name==G02)
	{
		if(StartOrEnd==0)
			Tangent = myatan2(pGcode.Y0-pGcode.J, pGcode.X0-pGcode.I)-_1p2PI;
		else
			Tangent = myatan2(pGcode.Y-pGcode.J, pGcode.X-pGcode.I)-_1p2PI;
	}
	else if(pGcode.Name==G03)
	{
		if(StartOrEnd==0)
			Tangent = myatan2(pGcode.Y0-pGcode.J, pGcode.X0-pGcode.I)+_1p2PI;
		else
			Tangent = myatan2(pGcode.Y-pGcode.J, pGcode.X-pGcode.I)+_1p2PI;
	}
	return Tangent;
}

/*************************************************************************************************************
函数功能: 计算前后代码段的角度变化
*************************************************************************************************************/
double Kerf::calculate_g_code_angle_change(GCodeStruct &pPreviousLine, GCodeStruct &pNextLine)
{
	double AngleChange=0;

	AngleChange = GetTangent(pNextLine,0)-GetTangent(pPreviousLine,1);

	while(AngleChange>(PI))
	{
		AngleChange-=2*PI;
	}
	while(AngleChange<-PI)
	{
		AngleChange+=2*PI;
	}

  return AngleChange;
}

/*************************************************************************************************************
函数功能: 判断角度变化是否很小

返回值:
    1 : 是
    0 : 否

说明
    角度变化很小时，交点计算会不准确，辅助判断交点是否存在
*************************************************************************************************************/
int Kerf::is_g_code_angle_change_small(GCodeStruct &pPreviousLine, GCodeStruct &pNextLine)
{
  double AngleChange=0;

	AngleChange = calculate_g_code_angle_change(pPreviousLine, pNextLine);
  curr_angle_change = AngleChange;

  if (IsLesser (fabs (AngleChange), kerf_small_angle_threshold) == 1)
    return 1;

  return 0;
}

/*********************************************************************
判断圆弧是否在指定的精度范围内，以便将圆弧拟合成直线

参数:
    R : 圆弧半径
    alpha : 圆心角 [0 ~ 2PI)
    accuracy : 精度

返回值:
    0 : 是
    -1: 否

判定条件
    劣弧 : 半径 - 弦心距 < 指定精度
    优弧 : 半径 + 弦心距 < 指定精度
*********************************************************************/
int Kerf::is_arc_in_fit_to_line_accuracy (double R, double alpha)
{
    double L;                               //弦心距

    L = R * cos (alpha / 2);
    if (IsGreater (R - L, arc_fit_to_line_accuracy))
        return -1;

    return 0;
}

/*
    对未有增加割缝补偿的G 代码未补偿后的G代码
    只对G01, G02,G03进行计算
    kerfvalue < 0为左补偿，
    kerfvalue>0为右补偿
    返回值 的Name==M02为错误补偿 
*/
/***************************************************************************************************
函数功能: 计算给定G代码加割缝之后的路径

参数:
    pNoKerfG     : 待加割缝的G代码
    AddKerfGCode : 加割缝之后的G代码
    kerfvalue    : 待加的割缝值
    dir          : 割缝方向 G41:左割缝，G42:右割缝

返回值:
    0 : 正常
    1 : 给定割缝值为0
***************************************************************************************************/
int Kerf::GetAddKerfGCode(GCodeStruct &pNoKerfG, GCodeStruct &AddKerfGCode,double kerfvalue,int dir)
{
	//GCodeStruct KerfGCode;
	double KerfX=0,KerfY=0;
	double LastTangent, NextTangent;
	AddKerfGCode.Name = M02;
	if(fabs(kerfvalue)<EP)
		return 1;

	AddKerfGCode.AngleRatio = pNoKerfG.AngleRatio;
	if(pNoKerfG.Name==G01)
	{
		LastTangent = myatan2(pNoKerfG.Y-pNoKerfG.Y0, pNoKerfG.X-pNoKerfG.X0);
		if(dir==G41)
		{
			KerfX = (kerfvalue)*cos(LastTangent+_1p2PI);
			KerfY = (kerfvalue)*sin(LastTangent+_1p2PI);
		}
		else
		{
			KerfX = (kerfvalue)*cos(LastTangent -_1p2PI);
			KerfY = (kerfvalue)*sin(LastTangent - _1p2PI);
		}
		AddKerfGCode = pNoKerfG;
		AddKerfGCode.X0 += KerfX;
		AddKerfGCode.Y0 += KerfY;
		AddKerfGCode.X  += KerfX;
		AddKerfGCode.Y  += KerfY;
		
	}
	else if(pNoKerfG.Name==G02)
	{
		LastTangent = myatan2(pNoKerfG.Y0-pNoKerfG.J, pNoKerfG.X0-pNoKerfG.I)-_1p2PI;
		NextTangent = myatan2(pNoKerfG.Y-pNoKerfG.J, pNoKerfG.X-pNoKerfG.I)-_1p2PI;
		
		if(dir==G41)
		{
			KerfX = (kerfvalue)*cos(LastTangent+_1p2PI);
			KerfY = (kerfvalue)*sin(LastTangent+_1p2PI);	
		}
		else
		{
			KerfX = (kerfvalue)*cos(LastTangent-_1p2PI);
			KerfY = (kerfvalue)*sin(LastTangent-_1p2PI);
		}
		AddKerfGCode = pNoKerfG;
		AddKerfGCode.R -= kerfvalue;
		AddKerfGCode.X0 += KerfX;
		AddKerfGCode.Y0 += KerfY;

		if(dir==G41)
		{
			KerfX = (kerfvalue)*cos(NextTangent+_1p2PI);
			KerfY = (kerfvalue)*sin(NextTangent+_1p2PI);	
		}
		else
		{
			KerfX = (kerfvalue)*cos(NextTangent-_1p2PI);
			KerfY = (kerfvalue)*sin(NextTangent-_1p2PI);
		}
		
		AddKerfGCode.X  += KerfX;
		AddKerfGCode.Y  += KerfY;
		AddKerfGCode.I = pNoKerfG.I;
		AddKerfGCode.J = pNoKerfG.J;
		
		
		
	}
	else if(pNoKerfG.Name==G03)
	{
		LastTangent = myatan2(pNoKerfG.Y0-pNoKerfG.J, pNoKerfG.X0-pNoKerfG.I)+_1p2PI;
		NextTangent = myatan2(pNoKerfG.Y-pNoKerfG.J, pNoKerfG.X-pNoKerfG.I)+_1p2PI;
		if(dir==G41)
		{
			KerfX = (kerfvalue)*cos(LastTangent+_1p2PI);
			KerfY = (kerfvalue)*sin(LastTangent+_1p2PI);
		}
		else
		{
			KerfX = (kerfvalue)*cos(LastTangent-_1p2PI);
			KerfY = (kerfvalue)*sin(LastTangent-_1p2PI);
		}
		AddKerfGCode = pNoKerfG;
		AddKerfGCode.R += kerfvalue;
		AddKerfGCode.X0 += KerfX;
		AddKerfGCode.Y0 += KerfY;
		if(dir==G41)
		{
			KerfX = (kerfvalue)*cos(NextTangent+_1p2PI);
			KerfY = (kerfvalue)*sin(NextTangent+_1p2PI);
		}
		else
		{
			KerfX = (kerfvalue)*cos(NextTangent-_1p2PI);
			KerfY = (kerfvalue)*sin(NextTangent-_1p2PI);
		}
		AddKerfGCode.X  += KerfX;
		AddKerfGCode.Y  += KerfY;
		AddKerfGCode.I = pNoKerfG.I;
		AddKerfGCode.J = pNoKerfG.J;
	}
	return 0;
}

/*************************************************************************************************************
函数功能: 判断两条G代码轨迹之间的位置关系

参数:
    pPreviousLine : 前一条G代码轨迹
    pNextLine     : 下一条G代码轨迹
    pAddLine      : 返回位置关系
                    pAddLine->Name = M02 : 两条G代码轨迹之间需要截断
                    pAddLine->Name = G01 : 两条G代码轨迹之间需要插入G01直线
                    pAddLine->Name = G02 : 两条G代码轨迹之间需要插入G02圆弧
                    pAddLine->Name = G03 : 两条G代码轨迹之间需要插入G03圆弧
                    pAddLine->Name = M00 : 两条G代码轨迹之间平等相接，不需要截断或者插入
    kerfvalue     : 割缝值大小
    dir           : 割缝方向  G41:左割缝, G42:右割缝

返回值:
    无效

note:
    前一条线的末端切线角度 beta，后一条线的初端切线角度alpha
	alpha-beta>0 左补偿为截取，右补偿为增加G03
	alpha-beta<0 左补偿为增加G02，右补偿为截取
	alpha-beta ~=0 则根据以原则截取或增加不直线段 
	kerfvalue < 0为左补偿，
	kerfvalue>0为右补偿
	pPreviousLine未补偿的前一句G 代码
	pNextLine 未补偿的后一句G 代码
	AddLine补偿的G代码码, Name==G01(补偿的是小直线 ),G02(左补偿),G03(左补偿),M02(截取了，没有补偿)
*************************************************************************************************************/
int Kerf::AddOrTrunc(GCodeStruct &pPreviousLine,GCodeStruct &pNextLine,GCodeStruct &pAddLine, int dir)
{
	double AngleChange=0;
	//long InterTrunc;
	
  //debug_stop (pNextLine);

	AngleChange = GetTangent(pNextLine,0)-GetTangent(pPreviousLine,1);

	while(AngleChange>(PI))
	{
		AngleChange-=2*PI;
	}
	while(AngleChange<-PI)
	{
		AngleChange+=2*PI;
	}
	if(IsLesser( fabs( fabs(AngleChange)-PI ),0.017))//1//1degree
	{
		if(dir == G41) //左补偿
		{
			if ((pNextLine.Name == G01 && pPreviousLine.Name == G02) || pNextLine.Name == G02)
				pAddLine.Name = M02;			//截断
			else
				pAddLine.Name = G02;
		}
		else
		{
			if ((pNextLine.Name == G01 && pPreviousLine.Name == G03) || pNextLine.Name == G03)
				pAddLine.Name = M02;			//截断
			else
				pAddLine.Name = G03;
		}
	}
	else if(IsLesser(fabs(AngleChange),0.0055)) ///0.3度，kerf = 10mm时，起末点最大偏差0.055mm
	{
		pAddLine.Name = M00;
	}
	/*else if(IsLesser(fabs(AngleChange),0.035))
	{
		pAddLine.Name = G01;
	}*/
	else if(dir == G41) //左补偿
	{
		if(AngleChange>0)
			pAddLine.Name = M02;
		else
		{	
			if(IsLesser(fabs(AngleChange),0.085))
				pAddLine.Name = G01;
			else
				pAddLine.Name = G02;
		}
	}
	else
	{
		if(AngleChange<0)
			pAddLine.Name = M02;
		else
		{
			if(IsLesser(fabs(AngleChange),0.017))
				pAddLine.Name = G01;
			else
				pAddLine.Name = G03;
		}
	}
  return 0;
}

int Kerf::Setupkerf(GCodeStruct &pGcode, double &dx,double &dy, double kerfvlaue, int dir)
{
	double QieXianAngle1;
	if(pGcode.Name==G01 ||pGcode.Name==G00)
	{
		QieXianAngle1 = myatan2(pGcode.Y-pGcode.Y0, pGcode.X-pGcode.X0);
	}
	else if(pGcode.Name==G02 )
	{
		QieXianAngle1 = myatan2(pGcode.Y0-pGcode.J,pGcode.X0-pGcode.I)-_1p2PI;	
	}
	else if(pGcode.Name==G03 )
	{
		QieXianAngle1 = myatan2(pGcode.Y0-pGcode.J,pGcode.X0-pGcode.I)+_1p2PI;
	}
	if(dir==G41)
	{
		dx = kerfvlaue*cos(QieXianAngle1+_1p2PI);
		dy = kerfvlaue*sin(QieXianAngle1+_1p2PI);
	}
	else
	{
		dx = kerfvlaue*cos(QieXianAngle1-_1p2PI);
		dy = kerfvlaue*sin(QieXianAngle1-_1p2PI);
	}
  return 0;
}

int Kerf::Canclekerf(GCodeStruct &pGcode, double &dx, double &dy, double kerfvlaue, int dir)
{
	double QieXianAngle1;
	if(pGcode.Name==G01||pGcode.Name==G00 )
	{
		QieXianAngle1 = myatan2(pGcode.Y-pGcode.Y0,pGcode.X-pGcode.X0);
	}
	else if(pGcode.Name==G02 )
	{
		QieXianAngle1 = myatan2(pGcode.Y-pGcode.J,pGcode.X-pGcode.I)-_1p2PI;	
	}
	else if(pGcode.Name==G03 )
	{
		QieXianAngle1 = myatan2(pGcode.Y-pGcode.J,pGcode.X-pGcode.I)+_1p2PI;
	}
	if(dir==G41)
	{
		dx = kerfvlaue*cos(QieXianAngle1-_1p2PI);
		dy = kerfvlaue*sin(QieXianAngle1-_1p2PI);
	}
	else
	{
		dx = kerfvlaue*cos(QieXianAngle1+_1p2PI);
		dy = kerfvlaue*sin(QieXianAngle1+_1p2PI);
	}
  return 0;
}

/*************************************************************************************
设定G代码的起始坐标
*************************************************************************************/
void Kerf::gcode_set_start_pos(GCodeStruct &pGcode, double xs, double ys) {
  pGcode.X0 = xs;
  pGcode.Y0 = ys;
}

/*************************************************************************************
设定G代码的末点坐标
*************************************************************************************/
void Kerf::gcode_set_end_pos(GCodeStruct &pGcode, double xe, double ye) {
  pGcode.X = xe;
  pGcode.Y = ye;
}

/*************************************************************************************
设定G代码的圆心坐标
*************************************************************************************/
void Kerf::gcode_set_IJ_pos(GCodeStruct &pGcode, double xi, double yi) {
  pGcode.I = xi;
  pGcode.J = yi;
}

/*************************************************************************************
设置G代码名称
*************************************************************************************/
void Kerf::gcode_set_name(GCodeStruct &pGcode, unsigned short gcode_name) {
  pGcode.Name = gcode_name;
}

/*************************************************************************************
获取下一个割缝G代码的缓冲区
起点默认设置为上行代码的末点
*************************************************************************************/
void Kerf::gcode_get_next_kerf_buf(std::vector<GCodeStruct>::iterator &PtrDst,
                                   std::vector<GCodeStruct>::iterator &PtrSrc) {

  PtrDst++;
  *PtrDst = *PtrSrc;
  PtrDst->X0 = (PtrDst-1)->X;
  PtrDst->Y0 = (PtrDst-1)->Y;
}

/*************************************************************************************
设置下一个割缝G代码
*************************************************************************************/
void Kerf::gcode_set_next_kerf(std::vector<GCodeStruct>::iterator &PtrDst,
                               std::vector<GCodeStruct>::iterator &PtrSrc,
                               unsigned short Name, double Xe, double Ye) {

  gcode_get_next_kerf_buf(PtrDst, PtrSrc);
  gcode_set_name(*PtrDst, Name);
  gcode_set_end_pos(*PtrDst, Xe, Ye);
}

/*************************************************************************************
函数功能: 向前设定G代码的末点坐标,直到遇到G01G02G03停止

参数:
	Ptr: 割缝数组指针
	xe,ye: 末点坐标

说明: 中间遇到G41/G42, 或者起点末点坐标不相等时，则返回错误
*************************************************************************************/
bool Kerf::gcode_set_end_pos_until_G01G02G03(
    std::vector<GCodeStruct>::iterator &Ptr,
    double xe, double ye) {

	while (1)
	{
		if (Ptr->Name == G01 || Ptr->Name == G02 || Ptr->Name == G03)
			break;
		
		if (Ptr->Name == G41 || Ptr->Name == G42)
      return false;

		if (!IsEqual (Ptr->X0, Ptr->X) || !IsEqual (Ptr->Y0, Ptr->Y))
			return false;

		Ptr->X0 = Ptr->X = xe;
		Ptr->Y0 = Ptr->Y = ye;

		Ptr--;
    if (Ptr == GfileFloatKerf.begin())
			return false;
	}

	gcode_set_end_pos(*Ptr, xe, ye);
	return true;
}

/*************************************************************************************
函数功能: 获取上一条有效的需要加割缝的G代码指令

返回值: 
	有效的G代码指针
  NULL: 无效
*************************************************************************************/
bool Kerf::kerf_get_previous_line_gcode(const std::vector<GCodeStruct> &g_code,
                                        std::vector<GCodeStruct>::iterator &Ptr,
                                        std::vector<GCodeStruct>::iterator &PtrPreviousLine) {

	PtrPreviousLine = Ptr;
	while (1)
	{
		if (PtrPreviousLine->Name == G01 || PtrPreviousLine->Name == G02 || PtrPreviousLine->Name == G03)
			break;

		if (PtrPreviousLine->Name == G41 || PtrPreviousLine->Name == G42)
			return false;
		
		PtrPreviousLine--;
    if (PtrPreviousLine == g_code.begin())
			return false;
	}

	return true;
}

/*************************************************************************************
函数功能: 直线和直线截断处理

参数:	
	PtrDst : 加完割缝的上一行代码，
	PtrSrc : 待加割缝的原始代码
	AddKerfGCode: PtrSrc加割缝后的代码

返回值:
	0 : 计算正常
	-1: 截断交点不在线段上，报错
	-2: 截断计算不出交点
	-3: 代码有误，非两条直线
	
*************************************************************************************/
int Kerf::kerf_line_and_line_cut_off(std::vector<GCodeStruct>::iterator &PtrDst,
                                     std::vector<GCodeStruct>::iterator &PtrSrc,
                                     GCodeStruct &AddKerfGCode)
{
	int res;
	double deltax, deltay;					//交点
	int point_in_line1, point_in_line2; 	//点是否在直线上
	int handler_mode = handler_mode_;				//处理方式 0: 报错，1: 过切
  std::vector<GCodeStruct>::iterator PtrKerfPrevious;

	//获取加割缝的上一行G代码
	if (!kerf_get_previous_line_gcode (GfileFloatKerf, PtrDst, PtrKerfPrevious) || PtrKerfPrevious->Name != G01 || AddKerfGCode.Name != G01)
		return -3;							//错误，不是直线与直线
	
	res = TwoLineIsIntersect(PtrKerfPrevious->X0, PtrKerfPrevious->Y0, PtrKerfPrevious->X, PtrKerfPrevious->Y, AddKerfGCode.X0, AddKerfGCode.Y0,AddKerfGCode.X, AddKerfGCode.Y, &deltax,&deltay);
	if (res == 1)
	{
		point_in_line1 = point_in_line (deltax, deltay, PtrKerfPrevious->X0, PtrKerfPrevious->Y0, PtrKerfPrevious->X, PtrKerfPrevious->Y);
		point_in_line2 = point_in_line (deltax, deltay, AddKerfGCode.X0, AddKerfGCode.Y0,AddKerfGCode.X, AddKerfGCode.Y);
		
		if (point_in_line1 == 1 && point_in_line2 == 1) 						//交点在两条线段上
		{
			if (gcode_set_end_pos_until_G01G02G03 (PtrDst, deltax, deltay) == NULL)
				return -3;

			gcode_set_next_kerf (PtrDst, PtrSrc, PtrSrc->Name, AddKerfGCode.X, AddKerfGCode.Y);
		}
		else if (is_g_code_angle_change_small (*PtrKerfPrevious, AddKerfGCode))      //角度变化很小，系统计算精度不够，此时插入小线段
		{
			gcode_set_next_kerf (PtrDst, PtrSrc, G01, AddKerfGCode.X0, AddKerfGCode.Y0);
			gcode_set_next_kerf (PtrDst, PtrSrc, PtrSrc->Name, AddKerfGCode.X, AddKerfGCode.Y);
		}
		else if (handler_mode == 0) 											//截断处理时交点不同时在两条线上，则为异常状态，报错
		{
			return -1;
		}
		else if (handler_mode == 1) 										    //过切处理，保留加割缝后的线段，增加小线段
		{
			if (point_in_line1 == 1)										    //交点在前一条线上
			{
				//(1). 如果交点在前一条线上，则从交点处截断第一条线，第二条线长度截断为0，从交点插入G01到第二条线末点
				if (gcode_set_end_pos_until_G01G02G03 (PtrDst, deltax, deltay) == NULL)
					return -3;

				gcode_set_next_kerf (PtrDst, PtrSrc, G01, AddKerfGCode.X, AddKerfGCode.Y);
			}
			else if (point_in_line2 == 1)											//交点在后一条线上
			{
				//(2). 如果交点在第二条线上，则第一条线的长度截断为0，从第一条线的起点插入G01到交点处
				if (gcode_set_end_pos_until_G01G02G03 (PtrDst, deltax, deltay) == NULL)
					return -3;

				gcode_set_next_kerf (PtrDst, PtrSrc, PtrSrc->Name, AddKerfGCode.X, AddKerfGCode.Y);
			}
			else
			{
				//(3). 如果交点既不在前一条上，也不再当前线上，则两段线长度都截断为0，增加G01连接起点和末点
				if (gcode_set_end_pos_until_G01G02G03 (PtrDst, AddKerfGCode.X, AddKerfGCode.Y) == NULL)
					return -3;			
			}
		}		
		else
			return -2;		
	}		
	else if (res == 0)
	{
		gcode_set_next_kerf (PtrDst, PtrSrc, G01, AddKerfGCode.X, AddKerfGCode.Y);
		//Assert("line line res=0");
	}
	else
	{
		gcode_set_next_kerf (PtrDst, PtrSrc, G01, AddKerfGCode.X, AddKerfGCode.Y);
		//Assert("line line no joint");
	}											

	return 0;
}

/*************************************************************************************
函数功能: 直线和圆弧的截断处理

参数:	
	PtrDst : 加完割缝的上一行代码，
	PtrSrc : 待加割缝的原始代码
	AddKerfGCode: PtrSrc加割缝后的代码

返回值:
	0 : 计算正常
	-1: 截断交点不在线段上，报错
	-2: 截断位置关系判断错误
	-3: 代码有误，非直线和圆弧
	-4: 未知错误
	
	
*************************************************************************************/
int Kerf::kerf_line_and_circle_cut_off(std::vector<GCodeStruct>::iterator &PtrDst,
                                       std::vector<GCodeStruct>::iterator &PtrSrc,
                                       GCodeStruct &AddKerfGCode)
{
	int res;
	double InterX1, InterY1, InterX2, InterY2;
	int point1_in_line, point1_in_circle;
	int point2_in_line, point2_in_circle;
	int point1_in_line_and_circle, point2_in_line_and_circle;
	int pt_in_line, pt_in_circle;
	int point1_is_valid;					//交点1有效标志
	int point_in_position;					//点所在位置标志，0,点不在两条线上 1: 点在前一条线上  2: 点在后一条线上 3: 点同在两条线上
	int handler_mode = handler_mode_;				//处理方式 0: 报错，1: 过切
	GCodeStruct *PtrLine, *PtrCircle;
  std::vector<GCodeStruct>::iterator PtrKerfPrevious;

  if (!kerf_get_previous_line_gcode (GfileFloatKerf, PtrDst, PtrKerfPrevious)) {
    return -3;	//错误，代码不是直线与圆弧
  }
	if (PtrKerfPrevious->Name == G01 && (AddKerfGCode.Name == G02 || AddKerfGCode.Name == G03))
	{
		PtrLine = &(*PtrKerfPrevious);
		PtrCircle = &AddKerfGCode;
	}
	else if ((PtrKerfPrevious->Name == G02 || PtrKerfPrevious ->Name == G03) && AddKerfGCode.Name == G01)
	{
		PtrLine = &AddKerfGCode;
		PtrCircle = &(*PtrKerfPrevious);
	}
	else
		return -3;	//错误，代码不是直线与圆弧
	
	res = LineCircleIntersect (PtrLine->X0, PtrLine->Y0, PtrLine->X, PtrLine->Y, PtrCircle->I, PtrCircle->J, GetRadius (*PtrCircle), &InterX1, &InterY1, &InterX2, &InterY2);

	if(res==0)		//截断情况下，直线与圆没有交点，则插入小直线, 这种情况是由于精度丢失导致，没有问题，减小平等相接(M00)的判断精度，则这种情况会减少
	{
		gcode_set_next_kerf (PtrDst, PtrSrc, G01, AddKerfGCode.X0, AddKerfGCode.Y0);
		gcode_set_next_kerf (PtrDst, PtrSrc, PtrSrc->Name, AddKerfGCode.X, AddKerfGCode.Y);
		//Assert("C/L no joint");
	}
	else														//直线与圆有两个交点
	{
		if(res==1)	//截断情况下，直线与圆没有只有一个交点，也是由于精度问题导致，按照有2个交点的方式处理
		{
			//Assert("C/L have one point");
		}

		//判断点是否在直线上并且在圆弧上
		point1_in_line	 = point_in_line (InterX1, InterY1, PtrLine->X0, PtrLine->Y0, PtrLine->X, PtrLine->Y);
		point1_in_circle = is_point_in_arc(InterX1, InterY1, PtrCircle->Name, PtrCircle->X0, PtrCircle->Y0, PtrCircle->X, PtrCircle->Y, PtrCircle->I, PtrCircle->J);
		point2_in_line	 = point_in_line (InterX2, InterY2, PtrLine->X0, PtrLine->Y0, PtrLine->X, PtrLine->Y);
		point2_in_circle = is_point_in_arc(InterX2, InterY2, PtrCircle->Name, PtrCircle->X0, PtrCircle->Y0, PtrCircle->X, PtrCircle->Y, PtrCircle->I, PtrCircle->J);
		point1_in_line_and_circle = (point1_in_line == 1 && point1_in_circle == 1) ? 1 : 0;
		point2_in_line_and_circle = (point2_in_line == 1 && point2_in_circle == 1) ? 1 : 0;

		if ((point1_in_line_and_circle == 1 && point2_in_line_and_circle == 0) || 
			(point1_in_line_and_circle == 0 && point2_in_line_and_circle == 1) ||
			(point1_in_line_and_circle == 1 && point2_in_line_and_circle == 1 && (res == 1)))
		{																		//有一组有效的交点
			if (point2_in_line_and_circle == 1)
			{
				InterX1 = InterX2;				//第二个交点有效
				InterY1 = InterY2;
			}

			if (!gcode_set_end_pos_until_G01G02G03 (PtrDst, InterX1, InterY1))
				return -3;

			gcode_set_next_kerf (PtrDst, PtrSrc, PtrSrc->Name, AddKerfGCode.X, AddKerfGCode.Y);
		}
		else if (handler_mode == 1) 											//加割缝后交点不同在两条线上，过切处理
		{
			point1_is_valid = 1;
			//1. 判断有效的交点位置  交点不同时在两条g代码上，或者两点同在G代码上，选择最近的点作为交点，
			if (dist (InterX2, InterY2, PtrSrc->X0, PtrSrc->Y0) < dist (InterX1, InterY1, PtrSrc->X0, PtrSrc->Y0))
			{
				InterX1 = InterX2;												//选择距离原始交点近的点为有效交点
				InterY1 = InterY2;
				point1_is_valid = 0;
			}

			//2. 判断有效交点的位置，在前一条线上还是在后一条线上
			point_in_position = 0;
			pt_in_line = (point1_is_valid == 1 ? point1_in_line : point2_in_line);		//点在直线上
			pt_in_circle = (point1_is_valid == 1 ? point1_in_circle : point2_in_circle);	//点在圆弧上
			if (pt_in_line == 1 && pt_in_circle == 1)
			{
				point_in_position = 3;										//交点同在两条线上
			}
			else if (pt_in_line == 1)
			{
				if (PtrDst->Name == G01)
					point_in_position = 1;									//交点在第一条线上
				else
					point_in_position = 2;									//交点在第二条线上
			}
			else if (pt_in_circle == 1)
			{
				if (PtrDst->Name == G02 || PtrDst->Name == G03)
					point_in_position = 1;									//交点在第一条线上
				else
					point_in_position = 2;									//交点在第二条线上
			}
			else
			{
				point_in_position = 0;										//交点不在两条线上
			}

			if (point_in_position == 3)			//交点同在两条线上
			{
				//(1). 两点同在直线和圆弧上，则取有效交点截断
				if (!gcode_set_end_pos_until_G01G02G03 (PtrDst, InterX1, InterY1))
					return -3;

				gcode_set_next_kerf (PtrDst, PtrSrc, PtrSrc->Name, AddKerfGCode.X, AddKerfGCode.Y);
			}
			else if (point_in_position == 2)	//交点在第二条线上
			{
				//(2). 如果交点在第二条线上，则第一条线的长度截断为0，从第一条线的起点插入G01到交点处
				//第一条线截断时，先判断一下是否为整圆，如果是整圆，则应该是精度问题导致判断不准的，直接以该点作为正常的交点
				if ((PtrKerfPrevious->Name == G02 || PtrKerfPrevious->Name == G03) && is_whole_circle_marked (*PtrKerfPrevious) == 1)
				{
					if (!gcode_set_end_pos_until_G01G02G03 (PtrDst, InterX1, InterY1))
						return -3;

					gcode_set_next_kerf (PtrDst, PtrSrc, PtrSrc->Name, AddKerfGCode.X, AddKerfGCode.Y);
					//Assert ("circle lack of accuracy");
				}
				else
				{
					if (!gcode_set_end_pos_until_G01G02G03 (PtrDst, InterX1, InterY1))
						return -3;
					gcode_set_name(*PtrDst, G01);
					gcode_set_next_kerf (PtrDst, PtrSrc, PtrSrc->Name, AddKerfGCode.X, AddKerfGCode.Y);
				}
			}
			else if (point_in_position == 1)	//交点在第一条线上
			{
				//(3). 如果交点在前一条线上，则从交点处截断第一条线，第二条线长度截断为0，从交点插入G01到第二条线末点
				if (!gcode_set_end_pos_until_G01G02G03 (PtrDst, InterX1, InterY1))
					return -3;

				gcode_set_next_kerf (PtrDst, PtrSrc, G01, AddKerfGCode.X, AddKerfGCode.Y);
				gcode_set_start_pos (*PtrDst, InterX1, InterY1);
			}
			else								//交点不在两条线上 
			{
				//(4). 如果交点既不在前一条上，也不再当前线上，则两段线长度都截断为0，增加G01连接起点和末点
				if ((PtrKerfPrevious->Name == G02 || PtrKerfPrevious->Name == G03) && is_whole_circle_marked (*PtrKerfPrevious) == 1)
				{
					if (!gcode_set_end_pos_until_G01G02G03 (PtrDst, InterX1, InterY1))
						return -3;

					gcode_set_next_kerf (PtrDst, PtrSrc, G01, AddKerfGCode.X, AddKerfGCode.Y);
					gcode_set_start_pos (*PtrDst, InterX1, InterY1);
					//Assert ("circle lack of accuracy");
				}
				else
				{
					if (!gcode_set_end_pos_until_G01G02G03 (PtrDst, AddKerfGCode.X, AddKerfGCode.Y))
						return -3;
					gcode_set_name (*PtrDst, G01);
				}
			}
		}
		else
			return -1;
	}

	return 0;
}

/*************************************************************************************
函数功能: 圆弧和圆弧的截断处理

参数:	
	PtrPtrDst : 加完割缝的上一行代码，
	PtrSrc    : 待加割缝的原始代码
	AddKerfGCode   : PtrSrc加割缝后的代码

返回值:
	0 : 计算正常
	-1: 截断交点不在线段上，报错
	-2: 截断位置关系判断错误
	-3: 代码有误，非直线和圆弧
	-4: 未知错误
	
*************************************************************************************/
int Kerf::kerf_circle_and_circle_cut_off(std::vector<GCodeStruct>::iterator &PtrDst,
                                         std::vector<GCodeStruct>::iterator &PtrSrc,
                                         GCodeStruct &AddKerfGCode)
{
	int res;
	double InterX1, InterY1, InterX2, InterY2;		//交点坐标
	int point1_in_circle1, point1_in_circle2;		//点在圆弧上位置标志
	int point2_in_circle1, point2_in_circle2;		//点在圆弧上位置标志
	int point1_in_circle, point2_in_circle;			//点在圆弧上位置标志
	int point1_is_valid;							//有效交点标志
	int point_in_position;							//有效交点位置标志
	int pt_in_circle1, pt_in_circle2;				//点在圆弧上位置标志
	int handler_mode = handler_mode_;				//处理方式 0: 报错，1: 过切
  std::vector<GCodeStruct>::iterator PtrKerfPrevious, Ptrtmp;

	if (!kerf_get_previous_line_gcode (GfileFloatKerf, PtrDst, PtrKerfPrevious) || !((PtrKerfPrevious->Name == G02 || PtrKerfPrevious->Name == G03) && (AddKerfGCode.Name == G02 || AddKerfGCode.Name == G03)))
		return -3;	//错误，代码不是圆弧与圆弧

	res = CircleIntersector (PtrKerfPrevious->I, PtrKerfPrevious->J, GetRadius (*PtrKerfPrevious), AddKerfGCode.I, AddKerfGCode.J, GetRadius(AddKerfGCode), &InterX1, &InterY1, &InterX2, &InterY2);

	if(res == 1 || res == 5 || res == 2 || res == 3) 	        			//相离
	{
        if (is_g_code_angle_change_small (*PtrKerfPrevious, AddKerfGCode) || handler_mode == 1)
        {
          gcode_set_next_kerf (PtrDst, PtrSrc, G01, AddKerfGCode.X0, AddKerfGCode.Y0);
          gcode_set_next_kerf (PtrDst, PtrSrc, PtrSrc->Name, AddKerfGCode.X, AddKerfGCode.Y);
        }
        else
		    return -1;
	}
	else      																//相切或者两个交点
	{	
		//判断点是否在圆弧上
		point1_in_circle1 = is_point_in_arc(InterX1, InterY1, PtrKerfPrevious->Name,  PtrKerfPrevious->X0,  PtrKerfPrevious->Y0,  PtrKerfPrevious->X,  PtrKerfPrevious->Y,  PtrKerfPrevious->I,  PtrKerfPrevious->J);
		point1_in_circle2 = is_point_in_arc(InterX1, InterY1, AddKerfGCode.Name, AddKerfGCode.X0, AddKerfGCode.Y0, AddKerfGCode.X, AddKerfGCode.Y, AddKerfGCode.I, AddKerfGCode.J);
		point2_in_circle1 = is_point_in_arc(InterX2, InterY2, PtrKerfPrevious->Name,  PtrKerfPrevious->X0,  PtrKerfPrevious->Y0,  PtrKerfPrevious->X,  PtrKerfPrevious->Y,  PtrKerfPrevious->I,  PtrKerfPrevious->J);
		point2_in_circle2 = is_point_in_arc(InterX2, InterY2, AddKerfGCode.Name, AddKerfGCode.X0, AddKerfGCode.Y0, AddKerfGCode.X, AddKerfGCode.Y, AddKerfGCode.I, AddKerfGCode.J);
		point1_in_circle = (point1_in_circle1 == 1 && point1_in_circle2 == 1) ? 1 : 0;
		point2_in_circle = (point2_in_circle1 == 1 && point2_in_circle2 == 1) ? 1 : 0;
		
		if ((point1_in_circle == 1 && point2_in_circle == 0) || 
			(point1_in_circle == 0 && point2_in_circle == 1))		            //只有一个有效交点
		{																		//加割缝后，点同时在直线和圆弧上，正确状态
			if (point2_in_circle == 1)
			{
				InterX1 = InterX2;												//第二个交点有效
				InterY1 = InterY2;
			}

			if (!gcode_set_end_pos_until_G01G02G03 (PtrDst, InterX1, InterY1))
				return -3;

			gcode_set_next_kerf (PtrDst, PtrSrc, PtrSrc->Name, AddKerfGCode.X, AddKerfGCode.Y);
		}
        else if (is_g_code_angle_change_small (*PtrKerfPrevious, AddKerfGCode))      //角度变化很小，系统计算精度不够，此时插入小线段
        {
            gcode_set_next_kerf (PtrDst, PtrSrc, G01, AddKerfGCode.X0, AddKerfGCode.Y0);
            gcode_set_next_kerf (PtrDst, PtrSrc, PtrSrc->Name, AddKerfGCode.X, AddKerfGCode.Y);
        }
		else if (handler_mode == 1)												//加割缝后交点不同在两条线上，过切处理
		{
			point1_is_valid = 1;
			//1. 判断有效的交点位置  交点不同时在两条g代码上，或者两点同在G代码上，选择最近的点作为交点，
			if (dist (InterX2, InterY2, PtrSrc->X0, PtrSrc->Y0) < dist (InterX1, InterY1, PtrSrc->X0, PtrSrc->Y0))
			{
				InterX1 = InterX2;												//选择距离原始交点近的点为有效交点
				InterY1 = InterY2;
				point1_is_valid = 0;
			}

			//2. 判断有效交点的位置，在前一条线上还是在后一条线上
			point_in_position = 0;
			pt_in_circle1 = (point1_is_valid == 1 ? point1_in_circle1 : point2_in_circle1);		//点在第一个圆弧上
			pt_in_circle2 = (point1_is_valid == 1 ? point1_in_circle2 : point2_in_circle2);		//点在第二个圆弧上
			if (pt_in_circle1 == 1 && pt_in_circle1 == 1)
			{
				point_in_position = 3;										//交点同在两条线上
			}
			else if (pt_in_circle1 == 1)
			{
				point_in_position = 1;										//交点在第一条线上
			}
			else if (pt_in_circle2 == 1)
			{
				point_in_position = 2;										//交点在第二条线上
			}
			else
			{
				point_in_position = 0;										//交点不在两条线上
			}

			if (point_in_position == 3)			//交点同在两条线上
			{
				//(1). 点同在圆弧上，则取有效交点截断
				if (!gcode_set_end_pos_until_G01G02G03 (PtrDst, InterX1, InterY1))
					return -3;

				gcode_set_next_kerf (PtrDst, PtrSrc, PtrSrc->Name, AddKerfGCode.X, AddKerfGCode.Y);
			}
			else if (point_in_position == 2)	//交点在第二条线上
			{
				//(2). 如果交点在第二条线上，则第一条线的长度截断为0，从第一条线的起点插入G01到交点处
				//第一条线截断时，先判断一下是否为整圆，如果是整圆，则应该是精度问题导致判断不准的，直接以该点作为正常的交点
				if (is_whole_circle_marked (*PtrKerfPrevious) != 1)
				{

					if (!gcode_set_end_pos_until_G01G02G03 (PtrDst, InterX1, InterY1))
						return -3;
					gcode_set_name (*PtrDst, G01);

					gcode_set_next_kerf (PtrDst, PtrSrc, PtrSrc->Name, AddKerfGCode.X, AddKerfGCode.Y);
				}
				else
				{	//精度问题导致判断不准，
					if (!gcode_set_end_pos_until_G01G02G03 (PtrDst, InterX1, InterY1))
						return -3;

					gcode_set_next_kerf (PtrDst, PtrSrc, PtrSrc->Name, AddKerfGCode.X, AddKerfGCode.Y);
					//Assert ("circle lack of accuracy");
				}
			}
			else if (point_in_position == 1)	//交点在第一条线上
			{
				//(3). 如果交点在前一条线上，则从交点处截断第一条线，第二条线长度截断为0，从交点插入G01到第二条线末点
				if (gcode_set_end_pos_until_G01G02G03 (PtrDst, InterX1, InterY1) == NULL)
					return -3;

				gcode_set_next_kerf (PtrDst, PtrSrc, G01, AddKerfGCode.X, AddKerfGCode.Y);			//插入G01
				gcode_set_start_pos (*PtrDst, InterX1, InterY1);
			}
			else								//交点不在两条线上 
			{
				//(4). 如果交点既不在前一条上，也不再当前线上，则两段线长度都截断为0，增加G01连接起点和末点
				if (is_whole_circle_marked (*PtrKerfPrevious) != 1)
				{
					if (!gcode_set_end_pos_until_G01G02G03 (PtrDst, AddKerfGCode.X, AddKerfGCode.Y))
						return -3;
					gcode_set_name(*PtrDst, G01);
				}
				else
				{	//精度问题导致判断不准
					if (!gcode_set_end_pos_until_G01G02G03 (PtrDst, InterX1, InterY1))
						return -3;

					gcode_set_next_kerf (PtrDst, PtrSrc, G01, AddKerfGCode.X, AddKerfGCode.Y);			//插入G01
					gcode_set_start_pos (*PtrDst, InterX1, InterY1);
					//Assert ("circle lack of accuracy");
				}
			}
		}
		else
			return -1;
	}

	return 0;
}

/*************************************************************************************
函数功能: 割缝插入直线
*************************************************************************************/
void Kerf::kerf_insert_line(std::vector<GCodeStruct>::iterator &PtrDst,
                            std::vector<GCodeStruct>::iterator &PtrSrc,
                            GCodeStruct &AddKerfGCode)
{
	gcode_set_next_kerf (PtrDst, PtrSrc, G01, AddKerfGCode.X0, AddKerfGCode.Y0);
	gcode_set_next_kerf (PtrDst, PtrSrc, PtrSrc->Name, AddKerfGCode.X, AddKerfGCode.Y);
}

/*************************************************************************************
函数功能: 割缝插入圆弧
*************************************************************************************/
void Kerf::kerf_insert_arc(int arc_name, std::vector<GCodeStruct>::iterator &PtrDst,
                           std::vector<GCodeStruct>::iterator &PtrSrc,
                           GCodeStruct &AddKerfGCode)
{
	gcode_set_next_kerf     (PtrDst, PtrSrc, arc_name, AddKerfGCode.X0, AddKerfGCode.Y0);
	gcode_set_IJ_pos        (*PtrDst, PtrSrc->X0, PtrSrc->Y0);			//设置圆心坐标
	whole_circle_mark_clear (*PtrDst);									//割缝产生的代码，取消整圆标记

	gcode_set_next_kerf     (PtrDst, PtrSrc, PtrSrc->Name, AddKerfGCode.X, AddKerfGCode.Y);
}

/*************************************************************************************
函数功能: 割缝直连
*************************************************************************************/
void Kerf::kerf_direct_connect(std::vector<GCodeStruct>::iterator &PtrDst,
                               std::vector<GCodeStruct>::iterator &PtrSrc,
                               GCodeStruct &AddKerfGCode)
{
	//平等相接时，插入小线段
	if (!IsEqual (AddKerfGCode.X0, PtrDst->X) && !IsEqual (AddKerfGCode.Y0, PtrDst->Y))
	{
		gcode_set_next_kerf (PtrDst, PtrSrc, G01, AddKerfGCode.X0, AddKerfGCode.Y0);
	}

	gcode_set_next_kerf (PtrDst, PtrSrc, PtrSrc->Name, AddKerfGCode.X, AddKerfGCode.Y);
}

/*************************************************************************************
函数功能: 割缝初次建立
*************************************************************************************/
void Kerf::kerf_just_set_up(std::vector<GCodeStruct>::iterator &PtrDst,
                            std::vector<GCodeStruct>::iterator &PtrSrc,
                            GCodeStruct &AddKerfGCode)
{
	gcode_set_next_kerf (PtrDst, PtrSrc, PtrSrc->Name, AddKerfGCode.X, AddKerfGCode.Y);
}

/*************************************************************************************
函数功能: 不需要加割缝
*************************************************************************************/
void Kerf::kerf_no_need(std::vector<GCodeStruct>::iterator &PtrDst,
                        std::vector<GCodeStruct>::iterator &PtrSrc)
{
  gcode_set_next_kerf (PtrDst, PtrSrc, PtrSrc->Name,
      PtrDst->X+(PtrSrc->X-PtrSrc->X0),
      PtrDst->Y+(PtrSrc->Y-PtrSrc->Y0));

}

/*************************************************************************************
函数功能: 对直线加割缝处理

返回值:
	ErrNo: 没有错误
*************************************************************************************/
int Kerf::kerf_for_line(std::vector<GCodeStruct>::iterator &PtrDst,
                        std::vector<GCodeStruct>::iterator &PtrSrc,
                        char &kerf_on, double kerf_value)
{
	GCodeStruct AddKerf, GTemp;
  std::vector<GCodeStruct>::iterator PtrPreviousLine;
	int kerf_dir;

	kerf_dir = G40;
	if (kerf_on == JUSTSETG41KERF || kerf_on == SETTEDG41KERF)
	{
		kerf_dir = G41;
	}
	else if (kerf_on == JUSTSETG42KERF || kerf_on == SETTEDG42KERF)
	{
		kerf_dir = G42;
	}

	if(kerf_on == JUSTSETG41KERF ||kerf_on == JUSTSETG42KERF)
	{
		if(kerf_on == JUSTSETG42KERF)
		{
			kerf_on = SETTEDG42KERF;
		}
		else
		{
			kerf_on = SETTEDG41KERF;
		}
		GetAddKerfGCode(*PtrSrc, AddKerf, kerf_value, kerf_dir);                          //割缝第一次建立，直接计算加割缝后的位置即可，无需计算与上一条线的位置关系
		kerf_just_set_up (PtrDst, PtrSrc, AddKerf);
	}
	else if(kerf_on == SETTEDG41KERF||kerf_on == SETTEDG42KERF)  			//仅仅确定当前行的起点坐标，终点坐标采用补偿后的末点坐标
	{
		if(kerf_on == SETTEDG42KERF)
		{
			kerf_dir = G42;   
		}
		else
		{
			kerf_dir = G41;
		}
		if (!kerf_get_previous_line_gcode (GfileFloatNoKerf, PtrSrc-1, PtrPreviousLine)) //获取上一行需加割缝的G代码
			return ErrGCode;
		
		AddOrTrunc(*PtrPreviousLine, *PtrSrc, GTemp, kerf_dir);       			//计算前一条线段和当前线段的位置关系
		GetAddKerfGCode(*PtrSrc, AddKerf, kerf_value, kerf_dir);                      	//计算当前线段加割缝之后的位置
			
		if(GTemp.Name==M02)          			//前面一行和当前行需要截取
		{
			if(PtrPreviousLine->Name==G01)  //case G01 level 2                                 //前一行代码为G01
			{
				if (kerf_line_and_line_cut_off (PtrDst, PtrSrc, AddKerf) != 0)
					return ERR_G_CODE_KERF_CUT_OFF;	
			}
			else if(PtrPreviousLine->Name==G02||PtrPreviousLine->Name==G03) //case G01 level 2,上行G02或G03，当前行G01
			{
				if (kerf_line_and_circle_cut_off (PtrDst, PtrSrc, AddKerf) != 0)
				{
					return ERR_G_CODE_KERF_CUT_OFF;
				}
			}
		}
		else if(GTemp.Name==G01) 				//前面一行和当前行需要插入一条小直线 
		{
			kerf_insert_line (PtrDst, PtrSrc, AddKerf);
		}
		else if(GTemp.Name==G02)				//插入G02
		{
			kerf_insert_arc (G02, PtrDst, PtrSrc, AddKerf);
		}
		else if(GTemp.Name==G03)				//插入G03
		{
			kerf_insert_arc (G03, PtrDst, PtrSrc, AddKerf);
		}
		else if(GTemp.Name==M00) 				//前面一行和当前行平等相接，不需要插入或截取
		{
			kerf_direct_connect (PtrDst, PtrSrc, AddKerf);
		}
	}
	else
	{
		kerf_no_need (PtrDst, PtrSrc);
	}

	return ErrNo;
}

/*************************************************************************************
函数功能: 对圆弧加割缝处理

返回值: 
	ErrNo : 割缝正常
	ERR_G_CODE_ARC_R_TOO_SMALL: 圆弧半径过小，无法加割缝
*************************************************************************************/
int Kerf::kerf_for_arc(std::vector<GCodeStruct>::iterator &PtrDst,
                       std::vector<GCodeStruct>::iterator &PtrSrc,
                       char &kerf_on, double kerf_value)
{
	GCodeStruct AddKerf, GTemp;
  std::vector<GCodeStruct>::iterator PtrPreviousLine;
	int kerf_dir;
	
	kerf_dir = G40;
    if (kerf_on == JUSTSETG41KERF || kerf_on == SETTEDG41KERF)
    {
		kerf_dir = G41;
    }
	else if (kerf_on == JUSTSETG42KERF || kerf_on == SETTEDG42KERF)
	{
		kerf_dir = G42;
	}
	
	whole_circle_mark (*PtrSrc);															//检查整圆并标记
	if (is_arc_radius_vaild_for_kerf (kerf_dir, kerf_value, *PtrSrc) != 0)				//检查圆弧半径是否过小，
	{
		return ERR_G_CODE_ARC_R_TOO_SMALL;     
	}
	
	if(kerf_on == JUSTSETG41KERF || kerf_on == JUSTSETG42KERF)					//割缝初次建立
	{
		if(kerf_on == JUSTSETG42KERF)
		{
			kerf_on = SETTEDG42KERF;
		}
		else
		{
			kerf_on = SETTEDG41KERF;
		}
		
		GetAddKerfGCode(*PtrSrc, AddKerf, kerf_value, kerf_dir);
		kerf_just_set_up (PtrDst, PtrSrc, AddKerf);
	}
	else if(kerf_on == SETTEDG41KERF || kerf_on == SETTEDG42KERF)  				//割缝已经建立，计算当前行的起点坐标及上一行的末点坐标.
	{
		if (!kerf_get_previous_line_gcode (GfileFloatNoKerf, PtrSrc-1, PtrPreviousLine)) //获取上一行需加割缝的G代码
			return ErrGCode;
		
		AddOrTrunc(*PtrPreviousLine, *PtrSrc, GTemp, kerf_dir);				//判断前后两条线的位置关系
		GetAddKerfGCode(*PtrSrc, AddKerf, kerf_value, kerf_dir);						//计算当前线的割缝位置
		
		if(GTemp.Name==M02)  															//前面一行和当前行需要截断
		{
			if(PtrPreviousLine->Name==G01)
			{
				if (kerf_line_and_circle_cut_off (PtrDst, PtrSrc, AddKerf) != 0)
				{
					return ERR_G_CODE_KERF_CUT_OFF;
				}
			}
			else if(PtrPreviousLine->Name==G02 || PtrPreviousLine->Name==G03)
			{	
				if (kerf_circle_and_circle_cut_off (PtrDst, PtrSrc, AddKerf) != 0)
				{
					return ERR_G_CODE_KERF_CUT_OFF;
				}
			}							
		}
		else if(GTemp.Name==G01) 				//前面一行和当前行需要插入一条小直线 
		{
			kerf_insert_line (PtrDst, PtrSrc, AddKerf);
		}
		else if(GTemp.Name==G02)				//插入G02
		{
			kerf_insert_arc (G02, PtrDst, PtrSrc, AddKerf);
		}
		else if(GTemp.Name==G03)				//插入G03
		{
			kerf_insert_arc (G03, PtrDst, PtrSrc, AddKerf);
		}
		else if(GTemp.Name==M00) 				//前面一行和当前行平等相接，不需要插入或截取
		{
			kerf_direct_connect (PtrDst, PtrSrc, AddKerf);
		}
	}
	else
	{
		kerf_no_need (PtrDst, PtrSrc);
	}

	return ErrNo;
}

/*************************************************************************************
函数功能: 割缝补偿

参数:
	handler_mode_ : 
		0 - 严格检查加割缝时的位置关系，有问题则报错
		1 - 截断时无有效交点存在，则过切处理
		
返回值:
    ErrNo                      : 正常
    ERR_G_CODE_POSITION        : G代码绝对位置偏差过大
    ERR_G_CODE_WHOLE_CIRCLE    : 整圆出错，无法校正
    ERR_G_CODE_ARC_R_TOO_SMALL : 圆弧半径过小
    ERR_G_CODE_ARC_CENTER      : 圆弧精度误差较大，圆心无法校正
    ERR_G_CODE_ErrG0203        : 圆弧错误
*************************************************************************************/
int Kerf::g2kerf(std::vector<GCodeStruct> &DesKerFile,
                 std::vector<GCodeStruct> &NoKerfFile) {

  std::vector<GCodeStruct>::iterator GCodeArryPtrSrc = NoKerfFile.begin();
  std::vector<GCodeStruct>::iterator GCodeArryPtrDes = DesKerFile.begin();
	GCodeStruct GTemp1;
	char kerf_on=0;  // 1-建立割缝 
	double deltax,deltay;
	int res,i;
	unsigned short LastGName,LastKerf=0,RowNum=0;
	unsigned short Kerf_G01 = 0;						//补偿 G00 or G01 标记
	double kerf = param_kerf;

  res = check_gcode_before_kerf();
  if (res != ErrNo) {
    return res;
  }
  if (GfileFloatNoKerf.size() > GfileTotalRows)
    return ERR_G_CODE_ErrGLarge;							//数组空间不足

  GCodeArryPtrSrc = GfileFloatNoKerf.begin();
 	
	if((kerf <= EP) && (kerf >= -EP))//if no kerf or zero kerf
	{
		while(GCodeArryPtrSrc->Name!=M02)
		{
			*GCodeArryPtrDes++ = *GCodeArryPtrSrc++;
		}

		*GCodeArryPtrDes = *GCodeArryPtrSrc;
		return ErrNo;
	}
    
  GCodeArryPtrSrc = GfileFloatNoKerf.begin();
  GCodeArryPtrDes = GfileFloatKerf.begin();
  *GCodeArryPtrDes = *GCodeArryPtrSrc;
	while(GCodeArryPtrSrc->Name !=M02)
	{
		if (GCodeArryPtrSrc->Name == M07)				
		{
			Kerf_G01 = 1;									//M07之后的补G01
		}
		else if (GCodeArryPtrSrc->Name == M08 || GCodeArryPtrSrc->Name == G00 || GCodeArryPtrSrc->Name == M02)
		{
			Kerf_G01 = 0;									//M08 G00 M02 之后的补G00
		}
	
		if(GCodeArryPtrSrc->Name==G41||GCodeArryPtrSrc->Name==G42 )
		{
			kerf = GCodeArryPtrSrc->OmitKerf ? param_kerf : GCodeArryPtrSrc->KerfValue;
			LastGName = GCodeArryPtrSrc->Name;
			if(kerf_on!=SETTEDG41KERF&&kerf_on!=SETTEDG42KERF)
			{
				i=0;
				do {
					
					GTemp1 = *(GCodeArryPtrSrc+i);
					i++;
					if(GTemp1.Name==M02||GTemp1.Name==G00||GTemp1.Name==G01 ||GTemp1.Name==G02 || GTemp1.Name==G03)
					break;
				} while(1);
				
				
				if(GTemp1.Name!=M02)
				{
					Setupkerf(GTemp1, deltax, deltay, kerf, LastGName);
					GCodeArryPtrDes++;
					*GCodeArryPtrDes = *GCodeArryPtrSrc;
					if (Kerf_G01 == 1)
						GCodeArryPtrDes->Name = G01;
					else
						GCodeArryPtrDes->Name = G00;  //增加一段空走线
					GCodeArryPtrDes->X0 = (GCodeArryPtrDes-1)->X;
					GCodeArryPtrDes->Y0 = (GCodeArryPtrDes-1)->Y;
					GCodeArryPtrDes->X = GCodeArryPtrDes->X0+deltax;
					GCodeArryPtrDes->Y = GCodeArryPtrDes->Y0+deltay;
				}
				if(LastGName==G41)
					kerf_on = JUSTSETG41KERF;
				else if(LastGName==G42)
					kerf_on = JUSTSETG42KERF;
			}
		}
		else if(GCodeArryPtrSrc->Name==G40)  //取消补偿
		{
			if(kerf_on==SETTEDG41KERF||kerf_on==SETTEDG42KERF)
			{
				if(kerf_on == SETTEDG41KERF)
					LastGName = G41;
				else
					LastGName = G42;
				i=0;
				do {
					GTemp1 = *(GCodeArryPtrDes-i);
					i++;
				} while(GTemp1.ShowLine>1&&GTemp1.Name!=G01  &&GTemp1.Name!=G02 && GTemp1.Name!=G03);

//				Canclekerf(&GTemp1, &deltax, &deltay, kerf, LastGName);
				GCodeArryPtrDes++;
				*GCodeArryPtrDes = *GCodeArryPtrSrc;
				if (Kerf_G01 == 1)
					GCodeArryPtrDes->Name = G01;
				else
					GCodeArryPtrDes->Name = G00;  //增加一段空走线
				GCodeArryPtrDes->X0 = (GCodeArryPtrDes-1)->X;
				GCodeArryPtrDes->Y0 = (GCodeArryPtrDes-1)->Y;
//				GCodeArryPtrDes->X = GCodeArryPtrDes->X0+deltax;
//				GCodeArryPtrDes->Y = GCodeArryPtrDes->Y0+deltay;

                //取消割缝增加的代码末点设为剩余代码的起点 (引出线是小线段时，割缝太大导致引出线补偿没了，使用Canclekerf函数计算末点会出错)
/*
                i = 0;
                while (1)
                {
                    GTemp1 = *(GCodeArryPtrSrc + i);
                    if (GTemp1.Name == G00 || GTemp1.Name == G01 || GTemp1.Name == G02 || GTemp1.Name == G03 ||
                        GTemp1.Name == G26 || GTemp1.Name == G27 || GTemp1.Name == G28 || GTemp1.Name == M02)
                    {
                        break;
                    }
                    i++;
                }
                GCodeArryPtrDes->X = GTemp1.X0;                     //G40割缝取消末点剩余代码的起点
                GCodeArryPtrDes->Y = GTemp1.Y0;
*/
				GCodeArryPtrDes->X = GCodeArryPtrSrc->X0;
				GCodeArryPtrDes->Y = GCodeArryPtrSrc->Y0;


				kerf_on = NOKERF;
				LastKerf = NOKERF;
			}
		}
		else if(GCodeArryPtrSrc->Name==G01)  								// case G01 level 1
		{
			res = kerf_for_line (GCodeArryPtrDes, GCodeArryPtrSrc, kerf_on, kerf);
			if (res != ErrNo)
				return res;
		}
		else if( GCodeArryPtrSrc->Name==G02 || GCodeArryPtrSrc->Name==G03)  //case G02/G03 level 1
		{
			res = kerf_for_arc (GCodeArryPtrDes, GCodeArryPtrSrc, kerf_on, kerf);
			if (res != ErrNo)
				return res;
		}
		else
		{
      if(GCodeArryPtrSrc == GfileFloatNoKerf.begin())
			{
				*GCodeArryPtrDes = *GCodeArryPtrSrc;	//起始行
			}
			else
			{
				if (GCodeArryPtrSrc->Name != GGG)		//是有效的G代码
				{
					kerf_no_need (GCodeArryPtrDes, GCodeArryPtrSrc);
				}
			}
		}
		if((GCodeArryPtrDes->Name>=G00&&GCodeArryPtrDes->Name<=G03)||(GCodeArryPtrDes->Name>=G26&&GCodeArryPtrDes->Name<=G28))
		{
				RowNum++;
				GCodeArryPtrDes->PierceHoleNum = RowNum;
		}
		GCodeArryPtrSrc++;
	}
	GCodeArryPtrDes++;
	GCodeArryPtrDes->Name = M02;

  //检查代码是否连续，不连续则报错
  return check_gcode_after_kerf();
}

void Kerf::GKerfProc(const std::string &noKerfFile, const std::string &KerfFile) {
  GCodeParse parse;
  std::vector<std::string> code_lines;
  parse.ReadGCode(noKerfFile, code_lines);
  parse.ParseGCode(code_lines, GfileFloatNoKerf);

  GfileFloatKerf = std::vector<GCodeStruct>(GfileTotalRows);
  kerf_rtn_ = g2kerf(GfileFloatKerf, GfileFloatNoKerf);
  if (kerf_rtn_ != ErrNo)  {
    return ;
  }

  if (theApp.GetProfileInt("CommSet", "FitOption", 1) == 1) {
    parse.CalculateGCode(GfileFloatKerf);
    double min_fit_length = atof(theApp.GetProfileString("CommSet", "MinFitLength", "0.1"));
    SimpleLinearFitting line_fit;
    line_fit.FitSmallLine(GfileFloatKerf, min_fit_length);
  }

  code_lines.clear();
  parse.GenerateGCode(GfileFloatKerf, code_lines);
  parse.WriteGCode(KerfFile, code_lines);
}

} // namespace GCode
