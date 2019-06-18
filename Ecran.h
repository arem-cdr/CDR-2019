#ifndef ECRAN_H
#define ECRAN_H
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
#include "hardware.h"
#include "odometrie.h"
#include "reglages.h"
#include "deplacement.h"
#include "BrasPousser.h"
#include "Bras.h"
#include "Pompe.h"
#include "demarreur.h"
#include "Ecran.h"
#include "Strategie.h"



/* Constants -------------------------------------------------------------------*/

#define TAILLE 11 //taille du buffer 
#define TAILLEBUFFERCAPTEURS 32 // taille de la variable bufferCapteurs

/* Global Variables ---------------------------------------------------------*/   

extern char bufferRequest[11]; //buffer pour le traitement
extern volatile char buffer[11]; //buffer pour la requête
extern Serial serial; //liaison uart entre la carte Maître et l'écran 
extern Serial pc;// debug PC
extern char bufferCapteurs[TAILLEBUFFERCAPTEURS];
extern bool ready;
extern bool couleur; //0 Violet 1 Jaune


/* Public Functions ------------------------------------------------------------------*/

void resetBuffer(); // vide le buffer 
void writeCapteurs(); // écrit un buffer sur le port série de l'écran 
void write(char bufferWrite[]); //écrit un buffer de taille quelconque
void copyBuffer(); //recopie le buffer dans le bufferRequest 
void executeOrderInit(deplacement robot, Pompe pompe, Demarreur demarreur, Bras brasGauche, Bras brasDroit, BrasPousser brasPousserGauche,  BrasPousser brasPousserDroit); //traite la requete de l'écran et fait l'action voulue
void blink(); // la led blink quand on recoit un ordre
void read(); //read les requêtes de l'écran (fonction bloquante)
void afficherEcran(int events); //read les messages en provenance de l'écran et applique les requêtes en utilisant executeOrderInit() 
void lancerTimerEcran(); //write un message sur l'UART de l'écran pour lancer le timer de match de l'écran

#endif

/*exemple d'utilisation -------------------------------------------------------------------------*/
/*
int main() { 
    while(1)
    {
        serial.scanf("%s", buffer);
        blink();
        copyBuffer();
        resetBuffer();
        executeOrderInit();
    } 
}*/
