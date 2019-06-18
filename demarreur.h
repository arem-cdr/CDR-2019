#ifndef DEMARREUR_H
#define DEMARREUR_H
/**
 ****************************************************************************
 * @file    demarreur.h
 * @author  Jimmy MAINGAM
 * @version V0.0.1
 * @date    04/04/2019
 * @brief   Implementation file to start the robot
 ****************************************************************************
 *This software has been developed to be used in AREM's robot for the cdfr 2019
 **/
 
/* Includes ------------------------------------------------------------------*/
  
#include "mbed.h"
 
 
/* Class demarreur ------------------------------------------------------------------*/
 
class Demarreur{ 
    
public:
Demarreur(PinName cmd);//constructeur
bool getState();//récupère state pour savoir l'état du démarreur

private: 
DigitalIn pin;
bool state; //état du demarreur false à l'arret et true en action
};
 
 
 #endif