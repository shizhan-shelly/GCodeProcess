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
  ErrNo,                      // ����
  ErrGCode,                   // ��ȡ��һ����Ӹ���G����ʱ��ȡ����
  ERR_G_CODE_POSITION,        // G�������λ��ƫ�����
  ERR_G_CODE_WHOLE_CIRCLE,    // ��Բ�����޷�У��
  ERR_G_CODE_ARC_R_TOO_SMALL, // Բ���뾶��Ը��뾶��С
  ERR_G_CODE_ARC_CENTER,      // Բ���������ϴ�Բ���޷�У��
  ERR_G_CODE_KERF_CUT_OFF,    // �ض�ʱ�޷����������
  ERR_G_CODE_ErrG0203,        // Բ������
  ERR_G_CODE_ErrGLarge,       // ����ռ䲻��

} ERR_G_CODE;

typedef enum _kerfstatus {
	NOKERF,  						//δ���� ����
	G41KERF,  						//����G41����������û�н���״̬ 
	G42KERF,  						//����G42,��������δ����״̬
	JUSTSETG41KERF,					//�ոս��� G41����
	JUSTSETG42KERF,					//�ոս��� g42����
	SETTEDG41KERF, 					//�Ѿ�������G41����
	SETTEDG42KERF, 					//�Ѿ�������G42����
	G40KERF  						//��������״̬
} kerfstatus;

/***************************************************************************************
��������С�Ƕ���ֵ

kerf = 10mm
    angle = 1�ȣ� L = 0.174mm
    angle = 3�ȣ� L = 0.523mm

kerf = 5mm
    angle = 1�ȣ� L = 0.087mm
    angle = 3�ȣ� L = 0.261mm

kerf = 3mm
    angle = 1�ȣ� L = 0.052mm
    angle = 3�ȣ� L = 0.157mm
***************************************************************************************/
double kerf_small_angle_threshold = 0.01745*3;
double curr_angle_change;

double arc_radius_accuracy = 0.5;                        //Բ���뾶�������Χ
double arc_fit_to_line_accuracy = 0;                   //Բ����ϳ�ֱ�߾������
double arc_center_adjust_accuracy = 0.1;                 //Բ�ĵ����������

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
//lead_type 0--���㿿��Բ������� 1--���㿿��Բ��������
/**********************************************************/
int if_in_arc_ccw(double ang_temp,double ang_str,double ang_end,int lead_type)
{
	/*ang_temp�������Ƿ���Բ���ڣ��Ѿ����Ƕȱ�Ϊ+��360�ȸ�������*/
	/*Ҫע�⣬�ο���Բ����*/
	/*���str>end��temp>str||temp<end����*/
	/*���str<end��str<temp<end����*/
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
//lead_type 0--���㿿��Բ������� 1--���㿿��Բ��������
int if_in_arc_cw(double ang_temp,double ang_str,double ang_end,int lead_type)
{
	/*ang_temp�������Ƿ���Բ���ڣ��Ѿ����Ƕȱ�Ϊ+��360�ȸ�������*/
	/*Ҫע�⣬�ο���Բ����*/
	/*���str>end��str>temp>end����*/
	/*���str<end��temp<str||temp>end����*/
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
	px,py test(x,y)		//��ʱ��Բ��
*/
/**********************************************************************************************************
��������: �жϵ��Ƿ�����ʱ���Բ����, ע�⣬�ú���ֻ�������ж���ʱ���Բ�������Բ����˳ʱ��ģ�����Ҫ��������ĩ�������

����:
    x0,y0 : ��ʱ��Բ�����������
    x1,y1 : ��ʱ��Բ����ĩ������
    cx,cy : ��ʱ��Բ����Բ������

    px,py : ���Ե������

����ֵ:
        1 : ����Բ����
        0 : �㲻��Բ����
**********************************************************************************************************/
int if_point_in_arc(double x0,double y0, double x1,double y1, double cx,double cy,double px,double py)
{
	double ang_str,ang_end,ang_pt;
	double d_t1,d_t2,d_t3,d_t4;
	d_t1 = x1-cx;     // ���յ�����������ת�����յ����Բ������
  d_t2 = y1-cy;
  d_t3 = x0-cx;   // ��Բ������������ת����������Բ������
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
��������: �жϵ��Ƿ���Բ����

����:
    px,py : ���Ե������
    
    arc_gcode_name: Բ��G����ָ�� G02:˳ʱ��Բ��  G03:��ʱ��Բ��
    x0,y0 : Բ�����������
    x1,y1 : Բ����ĩ������
    cx,cy : Բ����Բ������

����ֵ:
        1 : ����Բ����
        0 : �㲻��Բ����
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
��������:�жϵ��Ƿ����߶η�Χ�ڣ�
ע��    :�ú������жϵ�ǰ���Ƿ���ֱ����

����: 
    pointx,pointy : ���Ե������
    
    x0,y0 : �߶ε��������
    x,y   : �߶ε�ĩ������

����ֵ:
    1  : �����߶η�Χ��
    0  : �㲻���߶η�Χ��
******************************************************************************************/
int point_in_line(double pointx,double pointy,double x0,double y0,double x,double y)
{
	double R,R2;
	R = (x-pointx)*(pointx - x0);
	R2 =(y-pointy)*(pointy-y0);
	if(!IsLesser(R, 0) && !IsLesser(R2,0))  //�����߶���
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
����ֵ :
  0 ƽ��
  1 �н���
*/
/***********************************************************************************************
��������: �ж�����ֱ��λ�ù�ϵ

����:
    x0,y0 : ��һ���߶ε����
    x1,y1 : ��һ���߶ε�ĩ��
    
    x2,y2 : �ڶ����߶ε����
    x3,y3 : �ڶ����߶ε�ĩ��
    
    InterX, InterY : �������� (ֻ���ڷ���ֵΪ1ʱ����Ч���������겻һ�����߶��ϣ�����Ҫ��һ���ж�)

����ֵ:
    0 : ƽ��
    1 : �н���
    -1: ����
***********************************************************************************************/
int TwoLineIsIntersect(double   x0,   double   y0,   double   x1,   double   y1,   double   x2,   double   y2,   double   x3,   double   y3,   double   *InterX,   double   *InterY)   
{ //�����߶��Ƿ��ཻX0X1   AND   X1X2   
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
	����ֵ:
		0-û�н��㣬������
		1-һ�����㣬������
		2-���������㣬���ཻ
		
*/
/************************************************************************************************
��������: �����߶���Բ��֮���λ�ù�ϵ

����: 
    x0,y0  : �߶��������
    x,y    : �߶�ĩ������
    xc0,yc0: Բ��Բ������
    Radius : Բ���뾶
    InterX1,InterY1: �߶����ڵ�ֱ����Բ���ĵ�һ������ (����ֵΪ1��2ʱ��Ч)
    InterX2,InterY2: �߶����ڵ�ֱ����Բ���ĵڶ�������
 (����ֵΪ1��2ʱ��Ч)

����ֵ:
    0 : û�н��㣬����
    1 : ��һ�����㣬����
    2 : ���������㣬�ཻ
************************************************************************************************/
int LineCircleIntersect(double x0,double y0,double x,double y, double xc0,double yc0,double Radius,double *InterX1,double *InterY1,double *InterX2,double *InterY2)
{
	int res=0;
	double k1=1,temp1;
	double x_intersect1 = 0,y_intersect1 = 0;
	double x_intersect2 = 0,y_intersect2 = 0;
	double a,b,c,bb4ac;//ax*x+bx+c=0
	/*x=(-b+-sqrt(b*b-4ac))/2a*/
	/*���ж�b*b-4ac>0?*/
	/*>0:ֱ����԰�ཻ*/
	/*=0:ֱ����԰����*/
	/*<0:ֱ����԰���뼴����*/

	//if(fabs(x-x0)>fabs(y-y0))
	if (IsGreater (fabs(x-x0), fabs(y-y0)))
	{
		k1 = (y-y0)/(x-x0);
		//y = k1*x-k1*x0+y0����Բ����,���X
		temp1 = y0 - k1*x0;
		a = 1+k1*k1;
		b = -2*xc0+2*k1*(temp1-yc0);
		c = pow(xc0,2)  + pow(temp1-yc0,2)-pow(Radius,2);
              bb4ac = pow(b,2)-4*a*c;

		//if(bb4ac>0.1) //�ཻ
		if (IsGreater (bb4ac, 0))
		{
			res = 2;
			*InterX1 = (-b+sqrt(bb4ac))/2/a;
			*InterX2 = (-b-sqrt(bb4ac))/2/a;
			*InterY1 = k1*(*InterX1)+temp1;
			*InterY2 = k1*(*InterX2)+temp1;
		}
		else if (IsLesser (bb4ac, 0))
		//else if(bb4ac<0.001)  //����
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
		//y = k1*x-k1*x0+y0����Բ����,���X
		temp1 = x0 - k1*y0;
		a = 1+k1*k1;
		b = -2*yc0+2*k1*(temp1-xc0);
		c = pow(yc0,2)  + pow(temp1-xc0,2)-pow(Radius,2);
              bb4ac = pow(b,2)-4*a*c;

		//if(bb4ac>0.1) //�ཻ
		if (IsGreater (bb4ac, 0))
		{
			res = 2;
			*InterY1= (-b+sqrt(bb4ac))/2/a;
			*InterY2 = (-b-sqrt(bb4ac))/2/a;
			*InterX1 = k1*(*InterY1)+temp1;
			*InterX2 = k1*(*InterY2)+temp1;
		}
		//else if(bb4ac<0.001)  //����
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
    return 5; // ͬ��Բ
  }
  dd = dist(A.x, A.y, B.x, B.y);
  if (A.r + B.r + EP < dd) {
    return 1;  // ����
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
    return 5; // �޽�
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
      return 2; // ����һ������
    }
    if (fabs(dd - (maxfabs(A.r, B.r) - minfabs(A.r, B.r))) < EP) {
      return 3; // ����һ������
    }
  }
  return 4; // 2������
}

int circle_intersect2(Circle_t A, Circle_t B, Point_t *ia, Point_t *ib)
{
	 double dd;
	 double cos_x,sin_x,cos_a,sin_a;
	 int res;
    if (IsEqual(A.x,B.x)&&IsEqual(A.y,B.y))//ͬ��
        return 5;
    dd = dist (A.x, A.y, B.x, B.y);
    if (A.r + B.r + EP < dd||dd<EP)  //����
        return 1;//û�н���
    if((fabs(A.r-B.r)-EP)>dd)  //�ຬ
    	return 1; //included by another circle. 

    cos_x = (B.x-A.x)/dd;
    sin_x = (B.y-A.y)/dd;
   
    cos_a = (pow(A.r,2)+pow(dd,2)-pow(B.r,2))/(2*A.r*dd)  ;
    sin_a = sqrt(1-pow(cos_a,2));

    if(dd>=(fabs(A.r-B.r)-EP)&&dd<=(fabs(A.r-B.r)+EP))// ����һ����
	{
		res=3;	
	}
	else if(dd>=(fabs(A.r+B.r)-EP)&&dd<=(fabs(A.r+B.r)+EP))// ����һ����
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
����ֵ :
	1,5-����
	2-����
	3-����
	4-��������
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
    return 1; // ����
  }

  if (IsLesser(dd, fabs(r1 - r2)) || (IsEqual(x1, x2) && IsEqual(y1, y2) && IsEqual(r1, r2))) {
    return 5; // �ຬ
  }

  a = 2. * r1 * (x1 - x2);
  b = 2. * r1 * (y1 - y2);
  c = sqr(r2) - sqr(r1) - sqr(x1 - x2) - sqr(y1 - y2);
  p = sqr(a) + sqr(b);
  q = -2. * a * c;
  
  if((1 - sqr(-q / p / 2))<0) // �������л��ཻ����ʵ����������
		return 1;
  // ����
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
    //ͬ��Բ
    return 5;
  }
  else if(rtn==4)//��������
  {
      *ix1 = PointA.x;
      *iy1 = PointA.y;
      *ix2 = PointB.x;
      *iy2 = PointB.y;
      return 4;
  }
  else if(rtn==2)//����һ������
  {
      *ix1 = PointA.x;
      *iy1 = PointA.y;
      *ix2 = PointB.x;
      *iy2 = PointB.y;
      return 2;
  }
  else if(rtn ==3 )//����һ������
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

/*G����G02 G03 �İ뾶���������任
    Name:    G02 �� G03
    R:            Բ�İ뾶���������0����ΪС��180�ȵ�Բ�������С��0����Ϊ����180��Բ��
    StartP:  G02��G03������������ 
    EndP :   G02��G03���յ�������� 
    *ResP:   ����Ҫ���Բ�ľ�������
*/
unsigned int RToIJ(int Name, double R, Point_t StartP, Point_t EndP, Point_t *ResP)
{
	Point_t PtA,PtB;
	Circle_t A,B;
	unsigned int res;
	double x,y,hx,StartAOfCircleA,EndAOfCircleA;
	
	x = sqrt((EndP.x-StartP.x)*(EndP.x-StartP.x)+(EndP.y-StartP.y)*(EndP.y-StartP.y));
	y = 2*fabs(R);
	if (IsGreater (x, y))		//����֮��ľ��볬����Բ��ֱ�������ش���
	{
		return 1;			
	}
	else if (IsEqual (x, y))	//����֮��ľ������Բ��ֱ����Բ������Ψһȷ��
	{
		ResP->x = (StartP.x + EndP.x) / 2;
		ResP->y = (StartP.y + EndP.y) / 2;
		return 0;				//Բ��ȷ��������
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
            if(Name == G02) //G02,˳Բ
			{
				if(R>0)
				{
					/*
					//����������Բ��
					if(((CircleA.x - hx)*(x-hx)<=0) && ((CircleA.y - hy)*(y-hy)<=0))
					{
                         *ResP = CircleA;//  ����Բ��
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
				   		return 0; //��������Բ����С����180�ȵ�Բ��
				   	}
				   	else
				   	{
				   		*ResP = PtB;
				   	}
				}
				else 
				{
					//>��������Բ��
				/*	if(((CircleA.x - hx)*(x-hx)>=0) && ((CircleA.y - hy)*(y-hy)>=0))
					{
                         *ResP = CircleA;//  ����Բ��
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
				   		return 0; //��������Բ����С����180�ȵ�Բ��
				   	}
				   	else
				   	{
				   		*ResP = PtB;
				   	}
				}
             // else 
               //   return 1;
			}
			else  //G03 , ��Բ
			{
				if(R<0)
				{
					/*
					//>��������Բ��
					if(((CircleA.x - hx)*(x-hx)<=0) && ((CircleA.y - hy)*(y-hy)<=0))
					{
                         *ResP = CircleA;//  ����Բ��
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
				   		return 0; //��������Բ����С����180�ȵ�Բ��
				   	}
				   	else
				   	{
				   		*ResP = PtB;
				   	}
				}
				else
				{
					//<��������Բ��
					/*if(((CircleA.x - hx)*(x-hx)>=0) && ((CircleA.y - hy)*(y-hy)>=0))
					{
                         *ResP = CircleA;//  ����Բ��
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
				   		return 0; //��������Բ����С����180�ȵ�Բ��
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
				d_t1 = GCodeArryPtrSrc->X-GCodeArryPtrSrc->I;     // ���յ�����������ת�����յ����Բ������
  			d_t2 = GCodeArryPtrSrc->Y-GCodeArryPtrSrc->J;
  			d_t3 = GCodeArryPtrSrc->X0 -GCodeArryPtrSrc->I;   // ��Բ������������ת����������Բ������
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
�������ܣ�������γ��ȣ�ֱ��б�ʺ�Բ����ʼ��ֹ�Ƕ�
����:
	WithKerf : 0 δ�ӹ����Ĵ���
             1 �Ѽӹ����Ĵ���
 return :
    1 -- source don't have G code
    2 --
*/
int Kerf::Cal_Length_Angle_R(std::vector<GCodeStruct> &GCodeArry, int WithKerf) {
	double little_arc=5,little_arc_spd_level=0;
	int FirstCurve=0; //�ǲ��ǵ�һ���������ߣ���G�����һ������G00,01,02,03,26,27,28
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
��������: ����Բ������ʼ�ǶȺ�ĩ�Ƕ�
*********************************************************************************************/
void Kerf::calculate_arc_length_radius_and_angle (GCodeStruct &PGCode) {
	double d_t1,d_t2,d_t3,d_t4;
	
	d_t1 = PGCode.X-PGCode.I;	  					// ���յ�����������ת�����յ����Բ������
	d_t2 = PGCode.Y-PGCode.J;
	d_t3 = PGCode.X0-PGCode.I;   					// ��Բ������������ת����������Բ������
	d_t4 = PGCode.Y0-PGCode.J;
	PGCode.R = sqrt(d_t3*d_t3+d_t4*d_t4);  		//�뾶
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

	if(PGCode.Name==G02)	 			//˳ʱ��
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
	else //G03			//��ʱ��
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

	PGCode.Length = PGCode.R*fabs(PGCode.StartAngle - PGCode.EndAngle);		//���㻡��
}

/**********************************************************************************
��������: ���ݸ�����Բ���뾶�����¼���Բ������

����ֵ:
	 0: ����ɹ�
	-1: �������

ע��:
    �������Բ��ƫ��ھ��ȷ�Χ�ڣ��򱨴�����ȡr1,r2�Ĳ�ֵ��arc_accuracy/10֮������ֵ
    ���Ƚ���Բ��У������Ϊ���ĩ��У����Բ��У����������G01����Ρ�ͬʱҲҪ��֤Բ��У���ľ��ȡ�
    Բ��У����ȱ����:��ĩ�������ӽ�ʱ��Բ��У����������������ȡarc_accuracy/10�ľ�����ΪԲ��У���ľ���
**********************************************************************************/
static double re_r1, re_r2;
int Kerf::calculate_arc_center(GCodeStruct &PGCode, double r1, double r2) {
	double radius = fabs(r1+r2)/2;
	Point_t StartP, EndP, ResP;
  int ret;
  double new_arc_center_err;

  //ȷ��Բ�ľ���, ȡarc_center_adjust_accuracy��r1,r2��ֵ֮������ֵ
  double accuracy = MAX(arc_center_adjust_accuracy, fabs (r1-r2));

	if (IsEqual(PGCode.X0, PGCode.X) && IsEqual(PGCode.Y0, PGCode.Y))				//��Բֱ�ӷ��أ�����У��
		return 0;
	
	//1. ����Բ��Ϊ�Ż������ӻ�
	calculate_arc_length_radius_and_angle(PGCode);
	if (IsGreater (fabs(PGCode.StartAngle - PGCode.EndAngle), PI))
		radius = -radius;

	//2. ���¼���Բ��Բ������
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
    return -1;                                                   //У����������

  //arc_info.arc_center_adjust_count++;
  //arc_info.arc_center_adjust_differ = new_arc_center_err;
  //if (IsLesser (arc_info.arc_max_center_err, new_arc_center_err))
  //  arc_info.arc_max_center_err = new_arc_center_err;

	PGCode.I = ResP.x;
	PGCode.J = ResP.y;
	return 0;
}

/**********************************************************************************
��������: ���ݸ���Բ����㣬Բ�ĺͻ��ȣ�����Բ��ĩ��

����ֵ:
	 0: ����ɹ�
	-1: �������

ע��:
    (���ǰ뾶�ϴ�ʱ���Ƕȵ����Ƿ��б�Ҫ���¼���һ��Բ��?)
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
��������: ���g����λ���Ƿ���ƫ�ƫ������򱨴���ʾ

����: 
	Ptr: G����ṹ������ָ��

����ֵ:
	 0: ����
	-1: ƫ�����
	-2: �޷�����

˵��:
	��ǰ�д����ĩ������ͺ�һ�д�����������ƫ����1mm���ϣ���ֱ�ӱ���
	ƫ����1mm���ڣ����������������ʱ����ȷ�������˶�ָ���ϣ�����,ֱ�ӱ���
	���⣬�������߶����������꣬һ����������Բ���������꣬������ܵ���
	��Բ���СԲ���������������������Բ��������ƫ���ֱ�ӱ���
	(ע���������G02G03���������꣬����Բ�����Ȳ���Ӱ��,�������Բ����ĩ��
	 ƫ����п��ܵ���СԲ�������Բ������Բ���СԲ��!
	 ���ԣ�����λ�ü�����Ҫ����Բ�����ȼ������),	
*********************************************************************************/
static GCodeStruct err_gcode, err_gcode_next;
int Kerf::calibrate_gcode_position(std::vector<GCodeStruct> &GCodeArry) {
	#define IS_MOVE_GCODE(Name) (Name == G00 || Name == G01 || Name == G02 || Name == G03 || Name == G26 || Name == G27 || Name == G28)
	#define IS_ARC_GCODE(Name)	(Name == G02 || Name == G03)
	#define IS_LINE_GCODE(Name) (Name == G00 || Name == G01 || Name == G26 || Name == G27 || Name == G28)
	
	std::vector<GCodeStruct>::iterator PtrNext;
	double err_x, err_y;
	float MaxErrLength = 1.0;						//������ 1mm

  std::vector<GCodeStruct>::iterator Ptr = GCodeArry.begin();

	PtrNext = Ptr + 1;
	while (Ptr->Name != M02 && PtrNext->Name != M02)
	{
		//���G���뵱ǰ������������һ��ĩ�������Ƿ�����
    if (!(IsEqual (Ptr->X, PtrNext->X0) && IsEqual (Ptr->Y, PtrNext->Y0)))
    {
			err_x = Ptr->X - PtrNext->X0;
			err_y = Ptr->Y - PtrNext->Y0;
			err_gcode = *Ptr;			//debug use
			err_gcode_next = *PtrNext;	//debug use
			if (IsGreater (fabs (err_x), MaxErrLength) || IsGreater (fabs (err_y), MaxErrLength))		//���������������1mm���򱨴�
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
					     !(IsLesser (fabs(Ptr->X0-Ptr->X), 2*MaxErrLength) && IsLesser (fabs(Ptr->Y0-Ptr->Y), 2*MaxErrLength)))  		//����Բ����СԲ��
				{
					Ptr->X = PtrNext->X0;
					Ptr->Y = PtrNext->Y0;
				}
				else if (IS_ARC_GCODE (PtrNext->Name) && 
					     !(IsLesser (fabs(PtrNext->X0-PtrNext->X), 2*MaxErrLength) && IsLesser (fabs(PtrNext->Y0-PtrNext->Y), 2*MaxErrLength)))  //����Բ����СԲ��
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
			Ptr->X = PtrNext->X0;						//���¸�ֵ����֤G����λ�þ���һ��
			Ptr->Y = PtrNext->Y0;
		}

		Ptr++;
		PtrNext = Ptr + 1;
	}

	return 0;
}

/*********************************************************************************
��������: У��Բ��

����ֵ:
	 0: ����
	-1: ƫ������޷�У��

����У��Բ�ģ�Բ���޷�У��ʱ��У��ĩ��
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
			r1 = sqrt ((Ptr->X0 - Ptr->I) * (Ptr->X0 - Ptr->I) + (Ptr->Y0 - Ptr->J) * (Ptr->Y0 - Ptr->J));		//����������Բ�İ뾶
			r2 = sqrt ((Ptr->X - Ptr->I) * (Ptr->X - Ptr->I) + (Ptr->Y - Ptr->J) * (Ptr->Y - Ptr->J));			//����ĩ�����Բ�İ뾶

            //1. ���뾶����Ƿ����
            if (IsGreater (fabs (r1 - r2), arc_radius_accuracy))
            {
                return -1;                                      //������
            }

            //2. ����Ƿ�����ϳ�С�߶�
            calculate_arc_length_radius_and_angle (*Ptr);
            alpha = (Ptr->Name == G02) ? (Ptr->StartAngle - Ptr->EndAngle) : (Ptr->EndAngle - Ptr->StartAngle);
            if (is_arc_in_fit_to_line_accuracy (Ptr->R, alpha) == 0)
            {
                Ptr->Name = G01;				//��ϳ�С�߶�
            }//3. У׼Բ��
            else if (calculate_arc_center (*Ptr, r1, r2) != 0)    //��У��Բ��
            {
                if (calculate_arc_end (*Ptr, xe, ye) != 0)	//Բ���޷�У������У��ĩ��
                {
                    return -1;
                }
                //����һ��G01��������Բ��
                {
                    GCodeStruct insert_code = *Ptr;
                    Ptr = GCodeArry.insert(Ptr + 1, insert_code);
                    (Ptr-1)->X = xe;                //Բ��ĩ������
                    (Ptr-1)->Y = ye;
                    Ptr->Name = G01;        //����G01����
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
��������: ���g����λ���Ƿ���ƫ��
����ֵ :
    0 : û��ƫ��
    >0: G����ƫ�����
*********************************************************************************/
int Kerf::check_gcode_position(std::vector<GCodeStruct> &GCodeArry)
{
  int err_position_cnt = 0;

  std::vector<GCodeStruct>::iterator Ptr = GCodeArry.begin();
	std::vector<GCodeStruct>::iterator PtrNext = Ptr + 1;
	while (Ptr->Name != M02 && PtrNext->Name != M02)
	{
		//���G���뵱ǰ������������һ��ĩ�������Ƿ�����
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
��������: �����Բ
**************************************************************************************************/
void Kerf::whole_circle_mark(GCodeStruct &PGCode)
{
	//����Բ
	if (PGCode.Name == G02 || PGCode.Name == G03)
	{
		if (IsEqual(PGCode.X0, PGCode.X) && IsEqual(PGCode.Y0, PGCode.Y))
			PGCode.AngleRatio = 1;
		else
			PGCode.AngleRatio = 0;
	}
}

/**************************************************************************************************
��������: �����Բ���
**************************************************************************************************/
void Kerf::whole_circle_mark_clear(GCodeStruct &pGcode)
{
	if (pGcode.Name == G02 || pGcode.Name == G03)
	{
		pGcode.AngleRatio = 0;
	}
}

/**************************************************************************************************
��������: �ж���Բ����Ƿ����

����ֵ :
	1 : ����Բ
	0 : ������Բ
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
��������: ���Բ���Ӹ��뾶�Ƿ��С

����:
	kerf_dir  : ��췽�� G41:���죬G42:�Ҹ��
    kerf_value: ���ֵ
    GCodePtr  : Բ������

����ֵ:
     0: ����
    -1: Բ���뾶��С
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
���Ӹ��ǰ��G����
����ֵ :  
	ErrNo                      : ��������
	ERR_G_CODE_ErrG0203        : Բ������

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
���Ӹ����G����
   
����ֵ :  
	ErrNo                      : ��������
	ERR_G_CODE_POSITION	       : G�������λ��ƫ�����
	ERR_G_CODE_WHOLE_CIRCLE    : ��Բ��������, ���Ե���:kerf_small_angle_threshold
	ERR_G_CODE_ErrG0203        : Բ������

**********************************************************************************/
int Kerf::check_gcode_after_kerf()
{
	//1. ��鲢������Բ
	if (calibrate_whole_circle() != 0)
		return ERR_G_CODE_WHOLE_CIRCLE;

  //2. ���Բ�����Ȳ�У��Բ��
  if (calibrate_arc(GfileFloatKerf) != 0)
    return ERR_G_CODE_ErrG0203;

  //3. ���g���뾫��
  if (check_gcode_position(GfileFloatKerf) != 0)
    return ERR_G_CODE_POSITION;

	return ErrNo;
}

/**************************************************************************************************
��������: ��鲢�ָ���Բ

����:
	Ptr: G����ṹ��ָ��

����ֵ :
	0 : �����ָ�
	-1: �޷��ָ���Բ

˵��: 
	�ú�������У���Ӹ�����СԲ������Բ���Ӹ��ǰ����Բ���б�ǣ�
	�Ӹ�죬�ú���������Բ����Ƿ���Ч�������Բ�����Ч������Բ���ĽǶ�С��5�ȣ�����Ϊ
	��Բ�����СԲ��������һ��С�߶ζ���Բ����������
**************************************************************************************************/
int Kerf::calibrate_whole_circle()
{
	std::vector<GCodeStruct>::iterator PtrNext;
	static double angle_change = 0;
	double small_arc_angle = 3.0/180.0*PI;							//��ֵ�Ƕ�Ϊ3��
	int err_whole_circle = 0;
	
  std::vector<GCodeStruct>::iterator Ptr = GfileFloatKerf.begin();
	PtrNext = Ptr + 1;
	while (Ptr->Name != M02 && PtrNext->Name != M02)
	{
		PtrNext = Ptr + 1;
		if (is_whole_circle_marked (*Ptr) == 1)						//��Բ
		{
			calculate_arc_length_radius_and_angle(*Ptr);			//����Բ���Ƕ�

			//��Բ������Բ���Ƕ�С��5�ȣ�����Ϊ������������һ��С�߶λָ���Բ
			angle_change = fabs (Ptr->StartAngle - Ptr->EndAngle);
      if (IsLesser (angle_change, small_arc_angle) == 1)
      {
          //������G01�β���С�߶�
          if ((Ptr-1)->Name == G01 || ((Ptr+1)->Name != G01 && ((Ptr-1)->Name == G02 || (Ptr-1)->Name == G03)))
          {
              //��ǰһ�в���С�߶�
              {
                  //����С�߶�
                  GCodeStruct insert_code = *(Ptr+1);
                  Ptr = GfileFloatKerf.insert(Ptr, insert_code);
                  Ptr->Name = G01;
                  //Բ��ĩ������
                  (Ptr+1)->X0 = (Ptr+1)->X;
                  (Ptr+1)->Y0 = (Ptr+1)->Y;
              }
          }
          else if ((Ptr+1)->Name == G01 || (Ptr+1)->Name == G02 || (Ptr+1)->Name == G03)
          {
              //�ں�һ�в���С�߶�
              {
                  //����С�߶�
                  GCodeStruct insert_code = *Ptr;
                  Ptr = GfileFloatKerf.insert(Ptr+1, insert_code);
                  (Ptr)->Name = G01;
                  //Բ��ĩ������
                  (Ptr-1)->X = (Ptr-1)->X0;
                  (Ptr-1)->Y = (Ptr-1)->Y0;
              }
          }
          else
              return -1;          //ǰ���Ƿ��и�Σ�����
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
��������: ����ǰ�����εĽǶȱ仯
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
��������: �жϽǶȱ仯�Ƿ��С

����ֵ:
    1 : ��
    0 : ��

˵��
    �Ƕȱ仯��Сʱ���������᲻׼ȷ�������жϽ����Ƿ����
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
�ж�Բ���Ƿ���ָ���ľ��ȷ�Χ�ڣ��Ա㽫Բ����ϳ�ֱ��

����:
    R : Բ���뾶
    alpha : Բ�Ľ� [0 ~ 2PI)
    accuracy : ����

����ֵ:
    0 : ��
    -1: ��

�ж�����
    �ӻ� : �뾶 - ���ľ� < ָ������
    �Ż� : �뾶 + ���ľ� < ָ������
*********************************************************************/
int Kerf::is_arc_in_fit_to_line_accuracy (double R, double alpha)
{
    double L;                               //���ľ�

    L = R * cos (alpha / 2);
    if (IsGreater (R - L, arc_fit_to_line_accuracy))
        return -1;

    return 0;
}

/*
    ��δ�����Ӹ�첹����G ����δ�������G����
    ֻ��G01, G02,G03���м���
    kerfvalue < 0Ϊ�󲹳���
    kerfvalue>0Ϊ�Ҳ���
    ����ֵ ��Name==M02Ϊ���󲹳� 
*/
/***************************************************************************************************
��������: �������G����Ӹ��֮���·��

����:
    pNoKerfG     : ���Ӹ���G����
    AddKerfGCode : �Ӹ��֮���G����
    kerfvalue    : ���ӵĸ��ֵ
    dir          : ��췽�� G41:���죬G42:�Ҹ��

����ֵ:
    0 : ����
    1 : �������ֵΪ0
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
��������: �ж�����G����켣֮���λ�ù�ϵ

����:
    pPreviousLine : ǰһ��G����켣
    pNextLine     : ��һ��G����켣
    pAddLine      : ����λ�ù�ϵ
                    pAddLine->Name = M02 : ����G����켣֮����Ҫ�ض�
                    pAddLine->Name = G01 : ����G����켣֮����Ҫ����G01ֱ��
                    pAddLine->Name = G02 : ����G����켣֮����Ҫ����G02Բ��
                    pAddLine->Name = G03 : ����G����켣֮����Ҫ����G03Բ��
                    pAddLine->Name = M00 : ����G����켣֮��ƽ����ӣ�����Ҫ�ضϻ��߲���
    kerfvalue     : ���ֵ��С
    dir           : ��췽��  G41:����, G42:�Ҹ��

����ֵ:
    ��Ч

note:
    ǰһ���ߵ�ĩ�����߽Ƕ� beta����һ���ߵĳ������߽Ƕ�alpha
	alpha-beta>0 �󲹳�Ϊ��ȡ���Ҳ���Ϊ����G03
	alpha-beta<0 �󲹳�Ϊ����G02���Ҳ���Ϊ��ȡ
	alpha-beta ~=0 �������ԭ���ȡ�����Ӳ�ֱ�߶� 
	kerfvalue < 0Ϊ�󲹳���
	kerfvalue>0Ϊ�Ҳ���
	pPreviousLineδ������ǰһ��G ����
	pNextLine δ�����ĺ�һ��G ����
	AddLine������G������, Name==G01(��������Сֱ�� ),G02(�󲹳�),G03(�󲹳�),M02(��ȡ�ˣ�û�в���)
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
		if(dir == G41) //�󲹳�
		{
			if ((pNextLine.Name == G01 && pPreviousLine.Name == G02) || pNextLine.Name == G02)
				pAddLine.Name = M02;			//�ض�
			else
				pAddLine.Name = G02;
		}
		else
		{
			if ((pNextLine.Name == G01 && pPreviousLine.Name == G03) || pNextLine.Name == G03)
				pAddLine.Name = M02;			//�ض�
			else
				pAddLine.Name = G03;
		}
	}
	else if(IsLesser(fabs(AngleChange),0.0055)) ///0.3�ȣ�kerf = 10mmʱ����ĩ�����ƫ��0.055mm
	{
		pAddLine.Name = M00;
	}
	/*else if(IsLesser(fabs(AngleChange),0.035))
	{
		pAddLine.Name = G01;
	}*/
	else if(dir == G41) //�󲹳�
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
�趨G�������ʼ����
*************************************************************************************/
void Kerf::gcode_set_start_pos(GCodeStruct &pGcode, double xs, double ys) {
  pGcode.X0 = xs;
  pGcode.Y0 = ys;
}

/*************************************************************************************
�趨G�����ĩ������
*************************************************************************************/
void Kerf::gcode_set_end_pos(GCodeStruct &pGcode, double xe, double ye) {
  pGcode.X = xe;
  pGcode.Y = ye;
}

/*************************************************************************************
�趨G�����Բ������
*************************************************************************************/
void Kerf::gcode_set_IJ_pos(GCodeStruct &pGcode, double xi, double yi) {
  pGcode.I = xi;
  pGcode.J = yi;
}

/*************************************************************************************
����G��������
*************************************************************************************/
void Kerf::gcode_set_name(GCodeStruct &pGcode, unsigned short gcode_name) {
  pGcode.Name = gcode_name;
}

/*************************************************************************************
��ȡ��һ�����G����Ļ�����
���Ĭ������Ϊ���д����ĩ��
*************************************************************************************/
void Kerf::gcode_get_next_kerf_buf(std::vector<GCodeStruct>::iterator &PtrDst,
                                   std::vector<GCodeStruct>::iterator &PtrSrc) {

  PtrDst++;
  *PtrDst = *PtrSrc;
  PtrDst->X0 = (PtrDst-1)->X;
  PtrDst->Y0 = (PtrDst-1)->Y;
}

/*************************************************************************************
������һ�����G����
*************************************************************************************/
void Kerf::gcode_set_next_kerf(std::vector<GCodeStruct>::iterator &PtrDst,
                               std::vector<GCodeStruct>::iterator &PtrSrc,
                               unsigned short Name, double Xe, double Ye) {

  gcode_get_next_kerf_buf(PtrDst, PtrSrc);
  gcode_set_name(*PtrDst, Name);
  gcode_set_end_pos(*PtrDst, Xe, Ye);
}

/*************************************************************************************
��������: ��ǰ�趨G�����ĩ������,ֱ������G01G02G03ֹͣ

����:
	Ptr: �������ָ��
	xe,ye: ĩ������

˵��: �м�����G41/G42, �������ĩ�����겻���ʱ���򷵻ش���
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
��������: ��ȡ��һ����Ч����Ҫ�Ӹ���G����ָ��

����ֵ: 
	��Ч��G����ָ��
  NULL: ��Ч
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
��������: ֱ�ߺ�ֱ�߽ضϴ���

����:	
	PtrDst : ���������һ�д��룬
	PtrSrc : ���Ӹ���ԭʼ����
	AddKerfGCode: PtrSrc�Ӹ���Ĵ���

����ֵ:
	0 : ��������
	-1: �ضϽ��㲻���߶��ϣ�����
	-2: �ضϼ��㲻������
	-3: �������󣬷�����ֱ��
	
*************************************************************************************/
int Kerf::kerf_line_and_line_cut_off(std::vector<GCodeStruct>::iterator &PtrDst,
                                     std::vector<GCodeStruct>::iterator &PtrSrc,
                                     GCodeStruct &AddKerfGCode)
{
	int res;
	double deltax, deltay;					//����
	int point_in_line1, point_in_line2; 	//���Ƿ���ֱ����
	int handler_mode = handler_mode_;				//����ʽ 0: ����1: ����
  std::vector<GCodeStruct>::iterator PtrKerfPrevious;

	//��ȡ�Ӹ�����һ��G����
	if (!kerf_get_previous_line_gcode (GfileFloatKerf, PtrDst, PtrKerfPrevious) || PtrKerfPrevious->Name != G01 || AddKerfGCode.Name != G01)
		return -3;							//���󣬲���ֱ����ֱ��
	
	res = TwoLineIsIntersect(PtrKerfPrevious->X0, PtrKerfPrevious->Y0, PtrKerfPrevious->X, PtrKerfPrevious->Y, AddKerfGCode.X0, AddKerfGCode.Y0,AddKerfGCode.X, AddKerfGCode.Y, &deltax,&deltay);
	if (res == 1)
	{
		point_in_line1 = point_in_line (deltax, deltay, PtrKerfPrevious->X0, PtrKerfPrevious->Y0, PtrKerfPrevious->X, PtrKerfPrevious->Y);
		point_in_line2 = point_in_line (deltax, deltay, AddKerfGCode.X0, AddKerfGCode.Y0,AddKerfGCode.X, AddKerfGCode.Y);
		
		if (point_in_line1 == 1 && point_in_line2 == 1) 						//�����������߶���
		{
			if (gcode_set_end_pos_until_G01G02G03 (PtrDst, deltax, deltay) == NULL)
				return -3;

			gcode_set_next_kerf (PtrDst, PtrSrc, PtrSrc->Name, AddKerfGCode.X, AddKerfGCode.Y);
		}
		else if (is_g_code_angle_change_small (*PtrKerfPrevious, AddKerfGCode))      //�Ƕȱ仯��С��ϵͳ���㾫�Ȳ�������ʱ����С�߶�
		{
			gcode_set_next_kerf (PtrDst, PtrSrc, G01, AddKerfGCode.X0, AddKerfGCode.Y0);
			gcode_set_next_kerf (PtrDst, PtrSrc, PtrSrc->Name, AddKerfGCode.X, AddKerfGCode.Y);
		}
		else if (handler_mode == 0) 											//�ضϴ���ʱ���㲻ͬʱ���������ϣ���Ϊ�쳣״̬������
		{
			return -1;
		}
		else if (handler_mode == 1) 										    //���д��������Ӹ�����߶Σ�����С�߶�
		{
			if (point_in_line1 == 1)										    //������ǰһ������
			{
				//(1). ���������ǰһ�����ϣ���ӽ��㴦�ضϵ�һ���ߣ��ڶ����߳��Ƚض�Ϊ0���ӽ������G01���ڶ�����ĩ��
				if (gcode_set_end_pos_until_G01G02G03 (PtrDst, deltax, deltay) == NULL)
					return -3;

				gcode_set_next_kerf (PtrDst, PtrSrc, G01, AddKerfGCode.X, AddKerfGCode.Y);
			}
			else if (point_in_line2 == 1)											//�����ں�һ������
			{
				//(2). ��������ڵڶ������ϣ����һ���ߵĳ��Ƚض�Ϊ0���ӵ�һ���ߵ�������G01�����㴦
				if (gcode_set_end_pos_until_G01G02G03 (PtrDst, deltax, deltay) == NULL)
					return -3;

				gcode_set_next_kerf (PtrDst, PtrSrc, PtrSrc->Name, AddKerfGCode.X, AddKerfGCode.Y);
			}
			else
			{
				//(3). �������Ȳ���ǰһ���ϣ�Ҳ���ٵ�ǰ���ϣ��������߳��ȶ��ض�Ϊ0������G01��������ĩ��
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
��������: ֱ�ߺ�Բ���Ľضϴ���

����:	
	PtrDst : ���������һ�д��룬
	PtrSrc : ���Ӹ���ԭʼ����
	AddKerfGCode: PtrSrc�Ӹ���Ĵ���

����ֵ:
	0 : ��������
	-1: �ضϽ��㲻���߶��ϣ�����
	-2: �ض�λ�ù�ϵ�жϴ���
	-3: �������󣬷�ֱ�ߺ�Բ��
	-4: δ֪����
	
	
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
	int point1_is_valid;					//����1��Ч��־
	int point_in_position;					//������λ�ñ�־��0,�㲻���������� 1: ����ǰһ������  2: ���ں�һ������ 3: ��ͬ����������
	int handler_mode = handler_mode_;				//����ʽ 0: ����1: ����
	GCodeStruct *PtrLine, *PtrCircle;
  std::vector<GCodeStruct>::iterator PtrKerfPrevious;

  if (!kerf_get_previous_line_gcode (GfileFloatKerf, PtrDst, PtrKerfPrevious)) {
    return -3;	//���󣬴��벻��ֱ����Բ��
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
		return -3;	//���󣬴��벻��ֱ����Բ��
	
	res = LineCircleIntersect (PtrLine->X0, PtrLine->Y0, PtrLine->X, PtrLine->Y, PtrCircle->I, PtrCircle->J, GetRadius (*PtrCircle), &InterX1, &InterY1, &InterX2, &InterY2);

	if(res==0)		//�ض�����£�ֱ����Բû�н��㣬�����Сֱ��, ������������ھ��ȶ�ʧ���£�û�����⣬��Сƽ�����(M00)���жϾ��ȣ���������������
	{
		gcode_set_next_kerf (PtrDst, PtrSrc, G01, AddKerfGCode.X0, AddKerfGCode.Y0);
		gcode_set_next_kerf (PtrDst, PtrSrc, PtrSrc->Name, AddKerfGCode.X, AddKerfGCode.Y);
		//Assert("C/L no joint");
	}
	else														//ֱ����Բ����������
	{
		if(res==1)	//�ض�����£�ֱ����Բû��ֻ��һ�����㣬Ҳ�����ھ������⵼�£�������2������ķ�ʽ����
		{
			//Assert("C/L have one point");
		}

		//�жϵ��Ƿ���ֱ���ϲ�����Բ����
		point1_in_line	 = point_in_line (InterX1, InterY1, PtrLine->X0, PtrLine->Y0, PtrLine->X, PtrLine->Y);
		point1_in_circle = is_point_in_arc(InterX1, InterY1, PtrCircle->Name, PtrCircle->X0, PtrCircle->Y0, PtrCircle->X, PtrCircle->Y, PtrCircle->I, PtrCircle->J);
		point2_in_line	 = point_in_line (InterX2, InterY2, PtrLine->X0, PtrLine->Y0, PtrLine->X, PtrLine->Y);
		point2_in_circle = is_point_in_arc(InterX2, InterY2, PtrCircle->Name, PtrCircle->X0, PtrCircle->Y0, PtrCircle->X, PtrCircle->Y, PtrCircle->I, PtrCircle->J);
		point1_in_line_and_circle = (point1_in_line == 1 && point1_in_circle == 1) ? 1 : 0;
		point2_in_line_and_circle = (point2_in_line == 1 && point2_in_circle == 1) ? 1 : 0;

		if ((point1_in_line_and_circle == 1 && point2_in_line_and_circle == 0) || 
			(point1_in_line_and_circle == 0 && point2_in_line_and_circle == 1) ||
			(point1_in_line_and_circle == 1 && point2_in_line_and_circle == 1 && (res == 1)))
		{																		//��һ����Ч�Ľ���
			if (point2_in_line_and_circle == 1)
			{
				InterX1 = InterX2;				//�ڶ���������Ч
				InterY1 = InterY2;
			}

			if (!gcode_set_end_pos_until_G01G02G03 (PtrDst, InterX1, InterY1))
				return -3;

			gcode_set_next_kerf (PtrDst, PtrSrc, PtrSrc->Name, AddKerfGCode.X, AddKerfGCode.Y);
		}
		else if (handler_mode == 1) 											//�Ӹ��󽻵㲻ͬ���������ϣ����д���
		{
			point1_is_valid = 1;
			//1. �ж���Ч�Ľ���λ��  ���㲻ͬʱ������g�����ϣ���������ͬ��G�����ϣ�ѡ������ĵ���Ϊ���㣬
			if (dist (InterX2, InterY2, PtrSrc->X0, PtrSrc->Y0) < dist (InterX1, InterY1, PtrSrc->X0, PtrSrc->Y0))
			{
				InterX1 = InterX2;												//ѡ�����ԭʼ������ĵ�Ϊ��Ч����
				InterY1 = InterY2;
				point1_is_valid = 0;
			}

			//2. �ж���Ч�����λ�ã���ǰһ�����ϻ����ں�һ������
			point_in_position = 0;
			pt_in_line = (point1_is_valid == 1 ? point1_in_line : point2_in_line);		//����ֱ����
			pt_in_circle = (point1_is_valid == 1 ? point1_in_circle : point2_in_circle);	//����Բ����
			if (pt_in_line == 1 && pt_in_circle == 1)
			{
				point_in_position = 3;										//����ͬ����������
			}
			else if (pt_in_line == 1)
			{
				if (PtrDst->Name == G01)
					point_in_position = 1;									//�����ڵ�һ������
				else
					point_in_position = 2;									//�����ڵڶ�������
			}
			else if (pt_in_circle == 1)
			{
				if (PtrDst->Name == G02 || PtrDst->Name == G03)
					point_in_position = 1;									//�����ڵ�һ������
				else
					point_in_position = 2;									//�����ڵڶ�������
			}
			else
			{
				point_in_position = 0;										//���㲻����������
			}

			if (point_in_position == 3)			//����ͬ����������
			{
				//(1). ����ͬ��ֱ�ߺ�Բ���ϣ���ȡ��Ч����ض�
				if (!gcode_set_end_pos_until_G01G02G03 (PtrDst, InterX1, InterY1))
					return -3;

				gcode_set_next_kerf (PtrDst, PtrSrc, PtrSrc->Name, AddKerfGCode.X, AddKerfGCode.Y);
			}
			else if (point_in_position == 2)	//�����ڵڶ�������
			{
				//(2). ��������ڵڶ������ϣ����һ���ߵĳ��Ƚض�Ϊ0���ӵ�һ���ߵ�������G01�����㴦
				//��һ���߽ض�ʱ�����ж�һ���Ƿ�Ϊ��Բ���������Բ����Ӧ���Ǿ������⵼���жϲ�׼�ģ�ֱ���Ըõ���Ϊ�����Ľ���
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
			else if (point_in_position == 1)	//�����ڵ�һ������
			{
				//(3). ���������ǰһ�����ϣ���ӽ��㴦�ضϵ�һ���ߣ��ڶ����߳��Ƚض�Ϊ0���ӽ������G01���ڶ�����ĩ��
				if (!gcode_set_end_pos_until_G01G02G03 (PtrDst, InterX1, InterY1))
					return -3;

				gcode_set_next_kerf (PtrDst, PtrSrc, G01, AddKerfGCode.X, AddKerfGCode.Y);
				gcode_set_start_pos (*PtrDst, InterX1, InterY1);
			}
			else								//���㲻���������� 
			{
				//(4). �������Ȳ���ǰһ���ϣ�Ҳ���ٵ�ǰ���ϣ��������߳��ȶ��ض�Ϊ0������G01��������ĩ��
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
��������: Բ����Բ���Ľضϴ���

����:	
	PtrPtrDst : ���������һ�д��룬
	PtrSrc    : ���Ӹ���ԭʼ����
	AddKerfGCode   : PtrSrc�Ӹ���Ĵ���

����ֵ:
	0 : ��������
	-1: �ضϽ��㲻���߶��ϣ�����
	-2: �ض�λ�ù�ϵ�жϴ���
	-3: �������󣬷�ֱ�ߺ�Բ��
	-4: δ֪����
	
*************************************************************************************/
int Kerf::kerf_circle_and_circle_cut_off(std::vector<GCodeStruct>::iterator &PtrDst,
                                         std::vector<GCodeStruct>::iterator &PtrSrc,
                                         GCodeStruct &AddKerfGCode)
{
	int res;
	double InterX1, InterY1, InterX2, InterY2;		//��������
	int point1_in_circle1, point1_in_circle2;		//����Բ����λ�ñ�־
	int point2_in_circle1, point2_in_circle2;		//����Բ����λ�ñ�־
	int point1_in_circle, point2_in_circle;			//����Բ����λ�ñ�־
	int point1_is_valid;							//��Ч�����־
	int point_in_position;							//��Ч����λ�ñ�־
	int pt_in_circle1, pt_in_circle2;				//����Բ����λ�ñ�־
	int handler_mode = handler_mode_;				//����ʽ 0: ����1: ����
  std::vector<GCodeStruct>::iterator PtrKerfPrevious, Ptrtmp;

	if (!kerf_get_previous_line_gcode (GfileFloatKerf, PtrDst, PtrKerfPrevious) || !((PtrKerfPrevious->Name == G02 || PtrKerfPrevious->Name == G03) && (AddKerfGCode.Name == G02 || AddKerfGCode.Name == G03)))
		return -3;	//���󣬴��벻��Բ����Բ��

	res = CircleIntersector (PtrKerfPrevious->I, PtrKerfPrevious->J, GetRadius (*PtrKerfPrevious), AddKerfGCode.I, AddKerfGCode.J, GetRadius(AddKerfGCode), &InterX1, &InterY1, &InterX2, &InterY2);

	if(res == 1 || res == 5 || res == 2 || res == 3) 	        			//����
	{
        if (is_g_code_angle_change_small (*PtrKerfPrevious, AddKerfGCode) || handler_mode == 1)
        {
          gcode_set_next_kerf (PtrDst, PtrSrc, G01, AddKerfGCode.X0, AddKerfGCode.Y0);
          gcode_set_next_kerf (PtrDst, PtrSrc, PtrSrc->Name, AddKerfGCode.X, AddKerfGCode.Y);
        }
        else
		    return -1;
	}
	else      																//���л�����������
	{	
		//�жϵ��Ƿ���Բ����
		point1_in_circle1 = is_point_in_arc(InterX1, InterY1, PtrKerfPrevious->Name,  PtrKerfPrevious->X0,  PtrKerfPrevious->Y0,  PtrKerfPrevious->X,  PtrKerfPrevious->Y,  PtrKerfPrevious->I,  PtrKerfPrevious->J);
		point1_in_circle2 = is_point_in_arc(InterX1, InterY1, AddKerfGCode.Name, AddKerfGCode.X0, AddKerfGCode.Y0, AddKerfGCode.X, AddKerfGCode.Y, AddKerfGCode.I, AddKerfGCode.J);
		point2_in_circle1 = is_point_in_arc(InterX2, InterY2, PtrKerfPrevious->Name,  PtrKerfPrevious->X0,  PtrKerfPrevious->Y0,  PtrKerfPrevious->X,  PtrKerfPrevious->Y,  PtrKerfPrevious->I,  PtrKerfPrevious->J);
		point2_in_circle2 = is_point_in_arc(InterX2, InterY2, AddKerfGCode.Name, AddKerfGCode.X0, AddKerfGCode.Y0, AddKerfGCode.X, AddKerfGCode.Y, AddKerfGCode.I, AddKerfGCode.J);
		point1_in_circle = (point1_in_circle1 == 1 && point1_in_circle2 == 1) ? 1 : 0;
		point2_in_circle = (point2_in_circle1 == 1 && point2_in_circle2 == 1) ? 1 : 0;
		
		if ((point1_in_circle == 1 && point2_in_circle == 0) || 
			(point1_in_circle == 0 && point2_in_circle == 1))		            //ֻ��һ����Ч����
		{																		//�Ӹ��󣬵�ͬʱ��ֱ�ߺ�Բ���ϣ���ȷ״̬
			if (point2_in_circle == 1)
			{
				InterX1 = InterX2;												//�ڶ���������Ч
				InterY1 = InterY2;
			}

			if (!gcode_set_end_pos_until_G01G02G03 (PtrDst, InterX1, InterY1))
				return -3;

			gcode_set_next_kerf (PtrDst, PtrSrc, PtrSrc->Name, AddKerfGCode.X, AddKerfGCode.Y);
		}
        else if (is_g_code_angle_change_small (*PtrKerfPrevious, AddKerfGCode))      //�Ƕȱ仯��С��ϵͳ���㾫�Ȳ�������ʱ����С�߶�
        {
            gcode_set_next_kerf (PtrDst, PtrSrc, G01, AddKerfGCode.X0, AddKerfGCode.Y0);
            gcode_set_next_kerf (PtrDst, PtrSrc, PtrSrc->Name, AddKerfGCode.X, AddKerfGCode.Y);
        }
		else if (handler_mode == 1)												//�Ӹ��󽻵㲻ͬ���������ϣ����д���
		{
			point1_is_valid = 1;
			//1. �ж���Ч�Ľ���λ��  ���㲻ͬʱ������g�����ϣ���������ͬ��G�����ϣ�ѡ������ĵ���Ϊ���㣬
			if (dist (InterX2, InterY2, PtrSrc->X0, PtrSrc->Y0) < dist (InterX1, InterY1, PtrSrc->X0, PtrSrc->Y0))
			{
				InterX1 = InterX2;												//ѡ�����ԭʼ������ĵ�Ϊ��Ч����
				InterY1 = InterY2;
				point1_is_valid = 0;
			}

			//2. �ж���Ч�����λ�ã���ǰһ�����ϻ����ں�һ������
			point_in_position = 0;
			pt_in_circle1 = (point1_is_valid == 1 ? point1_in_circle1 : point2_in_circle1);		//���ڵ�һ��Բ����
			pt_in_circle2 = (point1_is_valid == 1 ? point1_in_circle2 : point2_in_circle2);		//���ڵڶ���Բ����
			if (pt_in_circle1 == 1 && pt_in_circle1 == 1)
			{
				point_in_position = 3;										//����ͬ����������
			}
			else if (pt_in_circle1 == 1)
			{
				point_in_position = 1;										//�����ڵ�һ������
			}
			else if (pt_in_circle2 == 1)
			{
				point_in_position = 2;										//�����ڵڶ�������
			}
			else
			{
				point_in_position = 0;										//���㲻����������
			}

			if (point_in_position == 3)			//����ͬ����������
			{
				//(1). ��ͬ��Բ���ϣ���ȡ��Ч����ض�
				if (!gcode_set_end_pos_until_G01G02G03 (PtrDst, InterX1, InterY1))
					return -3;

				gcode_set_next_kerf (PtrDst, PtrSrc, PtrSrc->Name, AddKerfGCode.X, AddKerfGCode.Y);
			}
			else if (point_in_position == 2)	//�����ڵڶ�������
			{
				//(2). ��������ڵڶ������ϣ����һ���ߵĳ��Ƚض�Ϊ0���ӵ�һ���ߵ�������G01�����㴦
				//��һ���߽ض�ʱ�����ж�һ���Ƿ�Ϊ��Բ���������Բ����Ӧ���Ǿ������⵼���жϲ�׼�ģ�ֱ���Ըõ���Ϊ�����Ľ���
				if (is_whole_circle_marked (*PtrKerfPrevious) != 1)
				{

					if (!gcode_set_end_pos_until_G01G02G03 (PtrDst, InterX1, InterY1))
						return -3;
					gcode_set_name (*PtrDst, G01);

					gcode_set_next_kerf (PtrDst, PtrSrc, PtrSrc->Name, AddKerfGCode.X, AddKerfGCode.Y);
				}
				else
				{	//�������⵼���жϲ�׼��
					if (!gcode_set_end_pos_until_G01G02G03 (PtrDst, InterX1, InterY1))
						return -3;

					gcode_set_next_kerf (PtrDst, PtrSrc, PtrSrc->Name, AddKerfGCode.X, AddKerfGCode.Y);
					//Assert ("circle lack of accuracy");
				}
			}
			else if (point_in_position == 1)	//�����ڵ�һ������
			{
				//(3). ���������ǰһ�����ϣ���ӽ��㴦�ضϵ�һ���ߣ��ڶ����߳��Ƚض�Ϊ0���ӽ������G01���ڶ�����ĩ��
				if (gcode_set_end_pos_until_G01G02G03 (PtrDst, InterX1, InterY1) == NULL)
					return -3;

				gcode_set_next_kerf (PtrDst, PtrSrc, G01, AddKerfGCode.X, AddKerfGCode.Y);			//����G01
				gcode_set_start_pos (*PtrDst, InterX1, InterY1);
			}
			else								//���㲻���������� 
			{
				//(4). �������Ȳ���ǰһ���ϣ�Ҳ���ٵ�ǰ���ϣ��������߳��ȶ��ض�Ϊ0������G01��������ĩ��
				if (is_whole_circle_marked (*PtrKerfPrevious) != 1)
				{
					if (!gcode_set_end_pos_until_G01G02G03 (PtrDst, AddKerfGCode.X, AddKerfGCode.Y))
						return -3;
					gcode_set_name(*PtrDst, G01);
				}
				else
				{	//�������⵼���жϲ�׼
					if (!gcode_set_end_pos_until_G01G02G03 (PtrDst, InterX1, InterY1))
						return -3;

					gcode_set_next_kerf (PtrDst, PtrSrc, G01, AddKerfGCode.X, AddKerfGCode.Y);			//����G01
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
��������: ������ֱ��
*************************************************************************************/
void Kerf::kerf_insert_line(std::vector<GCodeStruct>::iterator &PtrDst,
                            std::vector<GCodeStruct>::iterator &PtrSrc,
                            GCodeStruct &AddKerfGCode)
{
	gcode_set_next_kerf (PtrDst, PtrSrc, G01, AddKerfGCode.X0, AddKerfGCode.Y0);
	gcode_set_next_kerf (PtrDst, PtrSrc, PtrSrc->Name, AddKerfGCode.X, AddKerfGCode.Y);
}

/*************************************************************************************
��������: ������Բ��
*************************************************************************************/
void Kerf::kerf_insert_arc(int arc_name, std::vector<GCodeStruct>::iterator &PtrDst,
                           std::vector<GCodeStruct>::iterator &PtrSrc,
                           GCodeStruct &AddKerfGCode)
{
	gcode_set_next_kerf     (PtrDst, PtrSrc, arc_name, AddKerfGCode.X0, AddKerfGCode.Y0);
	gcode_set_IJ_pos        (*PtrDst, PtrSrc->X0, PtrSrc->Y0);			//����Բ������
	whole_circle_mark_clear (*PtrDst);									//�������Ĵ��룬ȡ����Բ���

	gcode_set_next_kerf     (PtrDst, PtrSrc, PtrSrc->Name, AddKerfGCode.X, AddKerfGCode.Y);
}

/*************************************************************************************
��������: ���ֱ��
*************************************************************************************/
void Kerf::kerf_direct_connect(std::vector<GCodeStruct>::iterator &PtrDst,
                               std::vector<GCodeStruct>::iterator &PtrSrc,
                               GCodeStruct &AddKerfGCode)
{
	//ƽ�����ʱ������С�߶�
	if (!IsEqual (AddKerfGCode.X0, PtrDst->X) && !IsEqual (AddKerfGCode.Y0, PtrDst->Y))
	{
		gcode_set_next_kerf (PtrDst, PtrSrc, G01, AddKerfGCode.X0, AddKerfGCode.Y0);
	}

	gcode_set_next_kerf (PtrDst, PtrSrc, PtrSrc->Name, AddKerfGCode.X, AddKerfGCode.Y);
}

/*************************************************************************************
��������: �����ν���
*************************************************************************************/
void Kerf::kerf_just_set_up(std::vector<GCodeStruct>::iterator &PtrDst,
                            std::vector<GCodeStruct>::iterator &PtrSrc,
                            GCodeStruct &AddKerfGCode)
{
	gcode_set_next_kerf (PtrDst, PtrSrc, PtrSrc->Name, AddKerfGCode.X, AddKerfGCode.Y);
}

/*************************************************************************************
��������: ����Ҫ�Ӹ��
*************************************************************************************/
void Kerf::kerf_no_need(std::vector<GCodeStruct>::iterator &PtrDst,
                        std::vector<GCodeStruct>::iterator &PtrSrc)
{
  gcode_set_next_kerf (PtrDst, PtrSrc, PtrSrc->Name,
      PtrDst->X+(PtrSrc->X-PtrSrc->X0),
      PtrDst->Y+(PtrSrc->Y-PtrSrc->Y0));

}

/*************************************************************************************
��������: ��ֱ�߼Ӹ�촦��

����ֵ:
	ErrNo: û�д���
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
		GetAddKerfGCode(*PtrSrc, AddKerf, kerf_value, kerf_dir);                          //����һ�ν�����ֱ�Ӽ���Ӹ����λ�ü��ɣ������������һ���ߵ�λ�ù�ϵ
		kerf_just_set_up (PtrDst, PtrSrc, AddKerf);
	}
	else if(kerf_on == SETTEDG41KERF||kerf_on == SETTEDG42KERF)  			//����ȷ����ǰ�е�������꣬�յ�������ò������ĩ������
	{
		if(kerf_on == SETTEDG42KERF)
		{
			kerf_dir = G42;   
		}
		else
		{
			kerf_dir = G41;
		}
		if (!kerf_get_previous_line_gcode (GfileFloatNoKerf, PtrSrc-1, PtrPreviousLine)) //��ȡ��һ����Ӹ���G����
			return ErrGCode;
		
		AddOrTrunc(*PtrPreviousLine, *PtrSrc, GTemp, kerf_dir);       			//����ǰһ���߶κ͵�ǰ�߶ε�λ�ù�ϵ
		GetAddKerfGCode(*PtrSrc, AddKerf, kerf_value, kerf_dir);                      	//���㵱ǰ�߶μӸ��֮���λ��
			
		if(GTemp.Name==M02)          			//ǰ��һ�к͵�ǰ����Ҫ��ȡ
		{
			if(PtrPreviousLine->Name==G01)  //case G01 level 2                                 //ǰһ�д���ΪG01
			{
				if (kerf_line_and_line_cut_off (PtrDst, PtrSrc, AddKerf) != 0)
					return ERR_G_CODE_KERF_CUT_OFF;	
			}
			else if(PtrPreviousLine->Name==G02||PtrPreviousLine->Name==G03) //case G01 level 2,����G02��G03����ǰ��G01
			{
				if (kerf_line_and_circle_cut_off (PtrDst, PtrSrc, AddKerf) != 0)
				{
					return ERR_G_CODE_KERF_CUT_OFF;
				}
			}
		}
		else if(GTemp.Name==G01) 				//ǰ��һ�к͵�ǰ����Ҫ����һ��Сֱ�� 
		{
			kerf_insert_line (PtrDst, PtrSrc, AddKerf);
		}
		else if(GTemp.Name==G02)				//����G02
		{
			kerf_insert_arc (G02, PtrDst, PtrSrc, AddKerf);
		}
		else if(GTemp.Name==G03)				//����G03
		{
			kerf_insert_arc (G03, PtrDst, PtrSrc, AddKerf);
		}
		else if(GTemp.Name==M00) 				//ǰ��һ�к͵�ǰ��ƽ����ӣ�����Ҫ������ȡ
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
��������: ��Բ���Ӹ�촦��

����ֵ: 
	ErrNo : �������
	ERR_G_CODE_ARC_R_TOO_SMALL: Բ���뾶��С���޷��Ӹ��
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
	
	whole_circle_mark (*PtrSrc);															//�����Բ�����
	if (is_arc_radius_vaild_for_kerf (kerf_dir, kerf_value, *PtrSrc) != 0)				//���Բ���뾶�Ƿ��С��
	{
		return ERR_G_CODE_ARC_R_TOO_SMALL;     
	}
	
	if(kerf_on == JUSTSETG41KERF || kerf_on == JUSTSETG42KERF)					//�����ν���
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
	else if(kerf_on == SETTEDG41KERF || kerf_on == SETTEDG42KERF)  				//����Ѿ����������㵱ǰ�е�������꼰��һ�е�ĩ������.
	{
		if (!kerf_get_previous_line_gcode (GfileFloatNoKerf, PtrSrc-1, PtrPreviousLine)) //��ȡ��һ����Ӹ���G����
			return ErrGCode;
		
		AddOrTrunc(*PtrPreviousLine, *PtrSrc, GTemp, kerf_dir);				//�ж�ǰ�������ߵ�λ�ù�ϵ
		GetAddKerfGCode(*PtrSrc, AddKerf, kerf_value, kerf_dir);						//���㵱ǰ�ߵĸ��λ��
		
		if(GTemp.Name==M02)  															//ǰ��һ�к͵�ǰ����Ҫ�ض�
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
		else if(GTemp.Name==G01) 				//ǰ��һ�к͵�ǰ����Ҫ����һ��Сֱ�� 
		{
			kerf_insert_line (PtrDst, PtrSrc, AddKerf);
		}
		else if(GTemp.Name==G02)				//����G02
		{
			kerf_insert_arc (G02, PtrDst, PtrSrc, AddKerf);
		}
		else if(GTemp.Name==G03)				//����G03
		{
			kerf_insert_arc (G03, PtrDst, PtrSrc, AddKerf);
		}
		else if(GTemp.Name==M00) 				//ǰ��һ�к͵�ǰ��ƽ����ӣ�����Ҫ������ȡ
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
��������: ��첹��

����:
	handler_mode_ : 
		0 - �ϸ���Ӹ��ʱ��λ�ù�ϵ���������򱨴�
		1 - �ض�ʱ����Ч������ڣ�����д���
		
����ֵ:
    ErrNo                      : ����
    ERR_G_CODE_POSITION        : G�������λ��ƫ�����
    ERR_G_CODE_WHOLE_CIRCLE    : ��Բ�����޷�У��
    ERR_G_CODE_ARC_R_TOO_SMALL : Բ���뾶��С
    ERR_G_CODE_ARC_CENTER      : Բ���������ϴ�Բ���޷�У��
    ERR_G_CODE_ErrG0203        : Բ������
*************************************************************************************/
int Kerf::g2kerf(std::vector<GCodeStruct> &DesKerFile,
                 std::vector<GCodeStruct> &NoKerfFile) {

  std::vector<GCodeStruct>::iterator GCodeArryPtrSrc = NoKerfFile.begin();
  std::vector<GCodeStruct>::iterator GCodeArryPtrDes = DesKerFile.begin();
	GCodeStruct GTemp1;
	char kerf_on=0;  // 1-������� 
	double deltax,deltay;
	int res,i;
	unsigned short LastGName,LastKerf=0,RowNum=0;
	unsigned short Kerf_G01 = 0;						//���� G00 or G01 ���
	double kerf = param_kerf;

  res = check_gcode_before_kerf();
  if (res != ErrNo) {
    return res;
  }
  if (GfileFloatNoKerf.size() > GfileTotalRows)
    return ERR_G_CODE_ErrGLarge;							//����ռ䲻��

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
			Kerf_G01 = 1;									//M07֮��Ĳ�G01
		}
		else if (GCodeArryPtrSrc->Name == M08 || GCodeArryPtrSrc->Name == G00 || GCodeArryPtrSrc->Name == M02)
		{
			Kerf_G01 = 0;									//M08 G00 M02 ֮��Ĳ�G00
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
						GCodeArryPtrDes->Name = G00;  //����һ�ο�����
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
		else if(GCodeArryPtrSrc->Name==G40)  //ȡ������
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
					GCodeArryPtrDes->Name = G00;  //����һ�ο�����
				GCodeArryPtrDes->X0 = (GCodeArryPtrDes-1)->X;
				GCodeArryPtrDes->Y0 = (GCodeArryPtrDes-1)->Y;
//				GCodeArryPtrDes->X = GCodeArryPtrDes->X0+deltax;
//				GCodeArryPtrDes->Y = GCodeArryPtrDes->Y0+deltay;

                //ȡ��������ӵĴ���ĩ����Ϊʣ��������� (��������С�߶�ʱ�����̫���������߲���û�ˣ�ʹ��Canclekerf��������ĩ������)
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
                GCodeArryPtrDes->X = GTemp1.X0;                     //G40���ȡ��ĩ��ʣ���������
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
				*GCodeArryPtrDes = *GCodeArryPtrSrc;	//��ʼ��
			}
			else
			{
				if (GCodeArryPtrSrc->Name != GGG)		//����Ч��G����
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

  //�������Ƿ��������������򱨴�
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
