#ifndef STRATEGIE_H
#define STRATEGIE_H
/**
 ****************************************************************************
 * @file    Strategie.h
 * @author  Jimmy MAINGAM
 * @version V0.0.1
 * @date    04/05/2019
 * @brief   Implementation file to communicate with the screen 
 ****************************************************************************
 *This software has been developed to be used in AREM's robot for the cdfr 2019
 **/
 
 /* Includes ------------------------------------------------------------------*/
 
#include "mbed.h"
#include "hardware.h"
#include "odometrie.h"
#include "reglages.h"
#include "deplacement.h"
#include "BrasPousser.h"
#include "Bras.h"
#include "Pompe.h"
#include "demarreur.h"
#include "Ecran.h"
#include "AnalyseDistance.h"
#include "pinMap.hpp"

/* Constants -------------------------------------------------------------------*/

#define TEMPSMATCH 99 
#define ARRET 0
#define ARC 2
#define RECTANGLE 1
/* Global Variables ---------------------------------------------------------*/   



/* Public Functions ------------------------------------------------------------------*/

void strategieHomologationViolet(deplacement robot, Pompe pompe, Demarreur demarreur, Bras brasGauche, Bras brasDroit, BrasPousser brasPousserGauche,  BrasPousser brasPousserDroit);
void strategieHomologationJaune(deplacement robot, Pompe pompe, Demarreur demarreur, Bras brasGauche, Bras brasDroit, BrasPousser brasPousserGauche,  BrasPousser brasPousserDroit);

void strategieClassiqueViolet(deplacement robot, Pompe pompe, Demarreur demarreur, Bras brasGauche, Bras brasDroit, BrasPousser brasPousserGauche,  BrasPousser brasPousserDroit);
void strategieClassiqueJaune(deplacement robot, Pompe pompe, Demarreur demarreur, Bras brasGauche, Bras brasDroit, BrasPousser brasPousserGauche,  BrasPousser brasPousserDroit);

void strategieRCVAViolet(deplacement robot, Pompe pompe, Demarreur demarreur, Bras brasGauche, Bras brasDroit, BrasPousser brasPousserGauche,  BrasPousser brasPousserDroit);
void strategieRCVAJaune(deplacement robot, Pompe pompe, Demarreur demarreur, Bras brasGauche, Bras brasDroit, BrasPousser brasPousserGauche,  BrasPousser brasPousserDroit);

void strategieHumiliationViolet(deplacement robot, Pompe pompe, Demarreur demarreur, Bras brasGauche, Bras brasDroit, BrasPousser brasPousserGauche,  BrasPousser brasPousserDroit);
void strategieHumiliationJaune(deplacement robot, Pompe pompe, Demarreur demarreur, Bras brasGauche, Bras brasDroit, BrasPousser brasPousserGauche,  BrasPousser brasPousserDroit);

void updateAndWriteScore(char n1, char n2, char n3);

void arretSystem(deplacement robot, Pompe pompe, Bras brasGauche, Bras brasDroit, BrasPousser brasPousserGauche,  BrasPousser brasPousserDroit);

void LectureI2CCarteCapteur(deplacement robot); // a remplacer par un include de la lib I2C d'Antoine

#endif

