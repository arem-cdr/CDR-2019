#ifndef MATH_PRECALC_H
#define MATH_PRECALC_H


#define PI 3.1416

float cos_precalc(int angle);
float sin_precalc(int angle);

double racine(long int a);
double pow(long int a, long int b);
double abs(double a, double b);

double diff_angle(double angle1, double angle2);


double borne_angle_d(double angle);
double borne_angle_r(double angle);
int signe(float entier);
float min(float f1, float f2);
double recup_angle_entre_trois_points_213(double x1,double y1,double x2,double y2,double x3,double y3);

#endif