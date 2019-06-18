#ifndef POMPE_H
#define POMPE_H
/**
 ****************************************************************************
 * @file    Ecran.h
 * @author  Jimmy MAINGAM
 * @version V0.0.1
 * @date    13/04/2019
 * @brief   Implementation file to communicate with the screen 
 ****************************************************************************
 *This software has been developed to be used in AREM's robot for the cdfr 2019
 **/
 
/* Includes ------------------------------------------------------------------*/
 
#include "mbed.h"
 
/* Class Pompe ------------------------------------------------------------------*/

class Pompe{ 
    
public:
Pompe(PinName cmd);//constructeur
void init(); //init pour vérifier les branchement avec l'écran
void activer(); //active la pompe pour récupérer le goldenium
void desactiver(); //desactive la pompe pour le faire tomber dans la balance
bool getState(); //renvoie l'état de la pompe à toute instant

private: 
DigitalOut pin;
bool state; //false pompe à l'arret, true pompe active 
};
 
 
 #endif