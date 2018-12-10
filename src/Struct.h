#ifndef STRUCT_H
#define STRUCT_H

#include <stdio.h>

enum camera_type
{
	GUIDANCE = 0,
	X3 = 1
};
enum Marker_type
{
	OLDMARKER = 0,
	NEWMARKER = 1

};
struct GPSPos
{
	float B; /* unit: deg*/
	float L;
	float H;
	GPSPos(float _B = 0.0, float _L = 0.0, float _H = 0.0)
	{
		B = _B;
		L = _L;
		H = _H;
	}
};

struct ECEFPos
{
	float X; /* unit: m*/
	float Y;
	float Z;
	ECEFPos(float _X = 0.0, float _Y = 0.0, float _Z = 0.0)
	{
		X = _X;
		Y = _Y;
		Z = _Z;
	}
};

struct localPos
{
	float X; /* unit: m*/
	float Y;
	float Z;
	localPos(float _X = 0.0, float _Y = 0.0, float _Z = 0.0)
	{
		X = _X;
		Y = _Y;
		Z = _Z;
	}
};

struct MyTime
{
	int min;
	int sec;
	int msec;
	MyTime(int _min = 0, int _sec = 0, int _msec = 0)
	{
		min = _min;
		sec = _sec;
		msec = _msec;
	}
};

struct COMMONTIME /* 通用时间定义 */
{
	unsigned short Year;
	unsigned short Month;
	unsigned short Day;
	unsigned short Hour;
	unsigned short Minute;
	double Second;

	COMMONTIME(unsigned short _Year = 0, unsigned short _Month = 0, unsigned short _Day = 0,
			   unsigned short _Hour = 0, unsigned short _Minute = 0, double _Second = 0.0)
	{
		Year = _Year;
		Month = _Month;
		Day = _Day;
		Hour = _Hour;
		Minute = _Minute;
		Second = _Second;
	}
};

struct GPSTIME /* GPS时间定义 */
{
	unsigned short Week;
	double SecOfWeek;

	GPSTIME()
	{
		Week = 0;
		SecOfWeek = 0.0;
	}
};

struct MJDTIME /* 简化儒略日 */
{
	int Days;
	double FracDay;

	MJDTIME()
	{
		Days = 0;
		FracDay = 0.0;
	}
};

/* 年积日结构体定义*/
struct DOYTIME
{
	short Year;
	short Doy;
	double Fracday;

	DOYTIME()
	{
		Year = 0;
		Doy = 0;
		Fracday = 0.0;
	}
};

#endif // STRUCT_H
