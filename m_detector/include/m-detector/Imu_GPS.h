#ifndef IMU_GPS_H
#define IMU_GPS_H


double rad(double d);
//获得距离差
void GetDirectDistance(double L0, double B0,double H0, double L, double B,double H,double *x,double *y,double *z);





#endif