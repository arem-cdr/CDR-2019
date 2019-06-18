#ifndef BRAS_H
#define BRAS_H

#include "mbed.h"
#include <Servo.h>

#define ACTIVATIONBRASGAUCHE -90  //position du bras gauche pour pousser le palet bleu
#define ACTIVATIONBRASDROIT   33 //position du bras droit pour pousser le palet bleu
#define DESACTIVERBRASGAUCHE  -5  //position du bras gauche pour le ranger
#define DESACTIVERBRASDROIT -48   //position du bras gauche pour le ranger

#define GAUCHE 0 //pour commander le bras gauche du robot
#define DROIT 1  //pour commander le bras droit du robot

class Bras {
    
public:
Bras(PinName cmd, int identifiant); //constructeur 
void setAngle(float angle); //deplace le servo jusqu'a l'angle choisit entre -90 et 90 environ 
float getAngle(); //récupère la valeur de l'angle 
void init(); //vérifie que le servo peut bouger correctement 
void activerBras(); //active le bras pour pousser le palet
void desactiverBras(); //desactive le bras pour le ranger 


private: 
Servo servo;
int id; // bras gauche 0, bras droit 1

};

#endif
