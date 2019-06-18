#include "BrasPousser.h"


BrasPousser::BrasPousser(PinName cmd, int identifiant): servo(cmd), id(identifiant) {
    if(id == DROIT)
    {
        servo.position(DESACTIVERBRASPALETDROIT);
    }
    else if (id == GAUCHE)
    {
         servo.position(DESACTIVERBRASPALETGAUCHE);
    }
}


void BrasPousser::setAngle(float angle) {
    servo.position(angle);
}


void BrasPousser::init() {
    if(id == GAUCHE)
    {
        servo.position(ACTIVATIONBRASPALETGAUCHE);
        wait_ms(1000);
        servo.position(DESACTIVERBRASPALETGAUCHE); 
        wait_ms(1000);  
    }   
    if(id == DROIT)
    {
        servo.position(ACTIVATIONBRASPALETDROIT);
        wait_ms(1000);
        servo.position(DESACTIVERBRASPALETDROIT); 
        wait_ms(1000);  
    }  
}


void BrasPousser::activerBras()
{
    if(id == GAUCHE)
    {
        servo.position(ACTIVATIONBRASPALETGAUCHE);
        //wait_ms(1000);
    }   
    if(id == DROIT)
    {
        servo.position(ACTIVATIONBRASPALETDROIT);
        //wait_ms(1000); 
    }     
}

void BrasPousser::desactiverBras()
{
    if(id == GAUCHE)
    {
        servo.position(DESACTIVERBRASPALETGAUCHE);
        //wait_ms(1000);
    }   
    if(id == DROIT)
    {
        servo.position(DESACTIVERBRASPALETDROIT);
        //wait_ms(1000); 
    }    
}

void activerDeuxBras(BrasPousser brasPousserGauche, BrasPousser brasPousserDroit)
{
    brasPousserGauche.activerBras();
    brasPousserDroit.activerBras();
    wait(1);
}


void desactiverDeuxBras(BrasPousser brasPousserGauche, BrasPousser brasPousserDroit)
{
    brasPousserGauche.desactiverBras();
    brasPousserDroit.desactiverBras();
    wait(1);
}