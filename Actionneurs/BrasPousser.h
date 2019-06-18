#ifndef BRASPOUSSER_H
#define BRASPOUSSER_H
/**
 ****************************************************************************
 * @file    BrasPousser.h
 * @author  Jimmy MAINGAM
 * @version V0.0.1
 * @date    08/04/2019
 * @brief   Implementation file for the servos
 ****************************************************************************
 *This software has been developed to be used in AREM's robot for the cdfr 2019
 **/
 
/* Includes ------------------------------------------------------------------*/
 
#include "mbed.h"
#include <Servo.h>

/* Constants -------------------------------------------------------------------*/

#define ACTIVATIONBRASPALETGAUCHE -5 //valeur du servo gauche  pour pousser les palets
#define ACTIVATIONBRASPALETDROIT   -81 //valeur du servo droit  pour pousser les palets
#define DESACTIVERBRASPALETGAUCHE  -75 //valeur du servo gauche  pour ranger le bras
#define DESACTIVERBRASPALETDROIT -1 //valeur du servo gauche  pour ranger le bras

#define GAUCHE 0 //pour commander le bras gauche du robot
#define DROIT 1 //pour commander le bras droit du robot

/* Class BrasPousser ------------------------------------------------------------------*/

class BrasPousser {
    
public:
BrasPousser(PinName cmd, int identifiant);//constructeur
void setAngle(float angle); //deplace le servo jusqu'a l'angle choisit entre -90 et 90 environ 
void init(); //vérifie que le servo peut bouger correctement et est bien branché
void activerBras(); //active le bras pour pousser les  palets par terre
void desactiverBras(); //desactive le bras pour le ranger 


private: 
Servo servo;
int id; // bras gauche 0, bras droit 1

};

void activerDeuxBras(BrasPousser brasPousserGauche, BrasPousser brasPousserDroit);
void desactiverDeuxBras(BrasPousser brasPousserGauche, BrasPousser brasPousserDroit);

#endif