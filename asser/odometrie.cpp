#include "mbed.h"
#include "odometrie.h"
#include "hardware.h"
#include "reglages.h"
#include "math_precalc.h"

double x_actuel;
double y_actuel;
double angle; // angle du robot dans le repère absolue

// redefinition de variable globale défini autre part dans le projet.
extern Serial pc;
extern double X_init = 0.0;
extern double Y_init = 0.0;
extern double ANGLE_init = 0.0;

long int nbr_tick_D_prec;
long int nbr_tick_G_prec;
   
// initialisation des variables de l'odométrie, il faut appeler la fonction quand le robot avant que le robot se déplace.
void init_odometrie()
{
	x_actuel = X_init;
	y_actuel = Y_init;
	angle = ANGLE_init;
	nbr_tick_D_prec=0;
	nbr_tick_G_prec=0;
}

void actualise_position()
{
    /*
    on suppose les valeurs de vd et vg constantes pendant t, la trajectoire decrite par le robot est alors un cercle
    */

	/*------recuperation de la rotation des roues---------*/
	long int nbr_tick_D=get_nbr_tick_D();
	long int nbr_tick_G=get_nbr_tick_G();
	
	//calcul du nombre de tick
	long int nbr_tick_D_actuel=nbr_tick_D-nbr_tick_D_prec;
	long int nbr_tick_G_actuel=nbr_tick_G-nbr_tick_G_prec;
	
	//sauvegarde
	nbr_tick_D_prec=nbr_tick_D;
	nbr_tick_G_prec=nbr_tick_G;
	
	double dep_roue_G = nbr_tick_G_actuel * DISTANCE_PAR_TICK_G; // deplacement des roues
    double dep_roue_D = nbr_tick_D_actuel * DISTANCE_PAR_TICK_D;
	
	
	/*------calcul de la trajectoire---------*/
	
    // determination du cercle décrit par la trajectoire et de la vitesse du robot sur ce cercle
    if (dep_roue_G != dep_roue_D){
        
    	double R = 0; // rayon du cercle decrit par la trajectoire
	    double d = 0; // vitesse du robot
        double cx = 0; // position du centre du cercle decrit par la trajectoire
        double cy = 0;
        
        R = ECART_ROUE / 2 * (dep_roue_D + dep_roue_G) / (dep_roue_D - dep_roue_G); // rayon du cercle
        cx = x_actuel - R * sin(angle);
        cy = y_actuel + R * cos(angle);
        d = (dep_roue_G + dep_roue_D) / 2;

        // mise à jour des coordonnées du robot
        if (dep_roue_G + dep_roue_D != 0){
            angle += d / R;
        }
        else{
            angle += dep_roue_D * 2 / ECART_ROUE;
        }
        
    	angle = borne_angle_r(angle);

        x_actuel = cx + R * sin(angle);
        y_actuel = cy - R * cos(angle);
    }
    else if (dep_roue_G == dep_roue_D){ // cas où la trajectoire est une parfaite ligne droite
        x_actuel +=dep_roue_G * cos(angle);
        y_actuel +=dep_roue_D * sin(angle);
    }
        
    //printf("tick d : %d, tick g : %d, x : %lf, y : %lf. angle : %lf\n", nbr_tick_D, nbr_tick_G, x_actuel, y_actuel, angle*180/PI);//debug
    
} 



double get_x_actuel()
{
	return x_actuel;
}

double get_y_actuel()
{
	return y_actuel;
}

double get_angle()
{
	return angle*180/PI;
}

void setEmplacementDepartViolet()
{
	X_init = 22500.0; //22500
	Y_init = 43500.0; // 45000 
	ANGLE_init = -90.0*PI/180.0;
}


void setEmplacementDepartJaune()
{
	X_init = 277500;
	Y_init = 43500;
	ANGLE_init = -90.0*PI/180.0;
}

void setEmplacementTest()
{
	X_init = 30000;
	Y_init = 30000;
	ANGLE_init = 0.0;
}