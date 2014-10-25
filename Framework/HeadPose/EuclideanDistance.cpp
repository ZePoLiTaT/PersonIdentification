#include "EuclideanDistance.h"
#include <math.h>
#include <cstdarg>

EuclideanDistance::EuclideanDistance()
{
}


EuclideanDistance::~EuclideanDistance()
{
}

//float EuclideanDistance::computeSegment(const Location* loc, ...) const
//{
//	//for (int i = 0; i<)
//	va_list args;
//	va_start(args, loc);
//
//	//while (*fmt != '\0') {
//	//	if (*fmt == 'd') {
//	//		int i = va_arg(args, int);
//	//		std::cout << i << '\n';
//	//	}
//	//	else if (*fmt == 'c') {
//	//		// note automatic conversion to integral type
//	//		int c = va_arg(args, int);
//	//		std::cout << static_cast<char>(c) << '\n';
//	//	}
//	//	else if (*fmt == 'f') {
//	//		double d = va_arg(args, double);
//	//		std::cout << d << '\n';
//	//	}
//	//	++fmt;
//	//}
//
//	va_end(args);
//
//	return 0;
//}

float EuclideanDistance::compute(const Location &ji, const Location &je) const
{
	float eucdist = 0;

	float xdif = ( je.x - ji.x );
	float ydif = ( je.y - ji.y );
	float zdif = ( je.z - ji.z );

	eucdist = sqrt(xdif*xdif + ydif *ydif + zdif *zdif);
	return eucdist;
}