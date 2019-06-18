#ifndef DEPLACEMENT_H
#define DEPLACEMENT_H

/**
 ****************************************************************************
 * @file    deplacement.h
 * @author  Guillaume Chauvon && Corentin Courtot #l'assert
 * @version V0.2.3
 * @date    22/05/2019
 * @brief   Implementation file for the deplacement of the robot
 ****************************************************************************
 *This software has been developed to be used in AREM's robot for the cdfr 2019
 **/
 
/* Includes ------------------------------------------------------------------*/
 

#include "BrasPousser.h"
#include "Pompe.h"
#include "AnalyseArcLib.h"
/* Constants -------------------------------------------------------------------*/

#define A_GAUCHE 1
#define A_DROITE 0
#define TAILLE_TAB 250

/* structures-----------------------------------------------------------------------*/

struct Coordonnees
{
            double x; 
            double y;
};





/* Class BrasPousser ------------------------------------------------------------------*/

class deplacement{
    public:
        deplacement();
        void initialisation(void);//init
        
        void rotation_rel(double angle_vise); // rotation relative angle positif pour tourner vers la gauche
        void rotation_abs(double angle_vise); //rotation absolue du robot
        void rotation_rel_pente(double angle_vise);
        void rotation_abs_pente(double angle_vise);
        
        void commande_vitesse(float vitesse_G, float vitesse_D); //set_PWM avec des vitesses flottantes.
        
        void vitesse_nulle_D(int zero); //coupe moteur droit
        void vitesse_nulle_G(int zero); //coupe moteur gauche
        void arreterRobot();
        void marche_arriere(int distance); // ligne droite en marche arriere, argument entier NEGATIF
        void ligne_droite_basique(long int distance); // ligne droite en marche avant, argument entier POSITIF
        
        void ligne_droite(long int distance, double x, double y, double cap);
        
        void asservissement(void); // asservissement en vitesse à ne pas utiliser avec les fonctions de déplacement séquentiel
        void printftab(void);  
        void test(void);
        void changement_consigne(int cons_D, int cons_G);
         
        void poussette(float temps); // set PWM 150 pendant 1.5s 
        void arc(Coordonnees p1, Coordonnees p2, int sens);//p2 point final p1 point intermediaire
        int cercle(Coordonnees a,Coordonnees b, Coordonnees c);
        double int_ext_cercle(double x, double y);
        void va_au_point(double x,double y, double cap);
        double recup_angle_entre_trois_points_213(double x1,double y1,double x2,double y2,double x3,double y3);
        void pente(long int distance, float vitesse, double angle_a_tourner);
        void pente_combo(double angle_pente, BrasPousser brasPousserGauche, BrasPousser brasPousserDroit, Pompe pompe);
        
        void evitementArc(double x, double y, double cap);    
        void evitementRectangle(double x, double y, double cap);
        
    private:
        float consigne;
        int consigne_D;
        int consigne_G;
        float somme_erreur_D;
        float somme_erreur_G;
        float erreur_precedente_D;
        float erreur_precedente_G;
        float erreur_glissee_D[5];
        float erreur_glissee_G[5];
        int compteur_glisse;
        float Kp_D;
        float Ki_D;
        float Kd_D;
        float Kp_G;
        float Ki_G;
        float Kd_G;
        long int tick_prec_D;
        long int tick_prec_G;
        float tab_cmd_D[TAILLE_TAB];
        float tab_cmd_G[TAILLE_TAB];
        float vtab_D[TAILLE_TAB];
        float vtab_G[TAILLE_TAB];
        float erreur_tab_G[TAILLE_TAB];
        float erreur_tab_D[TAILLE_TAB];
        float somme_erreur_tab_G[TAILLE_TAB];
        float somme_erreur_tab_D[TAILLE_TAB];
        float c_D[TAILLE_TAB];
        float c_G[TAILLE_TAB];
        int dix_ms;
        int consigne_tab[20][2];
        int compteur_asser;
        double somme_y;
        double point[3];
};

Coordonnees pointIntermediaire();
Coordonnees pointFinale();

Position PositionPos1();
Position PositionPos2();
Position PositionPos3();
#endif