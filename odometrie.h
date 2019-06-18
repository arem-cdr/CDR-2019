#ifndef ODOMETRIE_H
#define ODOMETRIE_H


void init_odometrie(void);
void actualise_position(void);
double get_x_actuel(void);
double get_y_actuel(void);
double get_angle(void);

void setEmplacementDepartViolet();
void setEmplacementDepartJaune();

void setEmplacementTest();
#endif
