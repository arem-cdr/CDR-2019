#include "Bras.h"


Bras::Bras(PinName cmd, int identifiant): servo(cmd), id(identifiant) {
     if(id == DROIT)
    {
        servo.position(DESACTIVERBRASDROIT);
    }
    else if (id == GAUCHE)
    {
         servo.position(DESACTIVERBRASGAUCHE);
    }
}


void Bras::setAngle(float angle) {
    servo.position(angle);
}

float Bras::getAngle() {
    return servo.read();
}


void Bras::init() {
    if(id == GAUCHE)
    {
        servo.position(ACTIVATIONBRASGAUCHE);
        wait_ms(1000);
        servo.position(DESACTIVERBRASGAUCHE); 
        wait_ms(1000);  
    }   
    if(id == DROIT)
    {
        servo.position(ACTIVATIONBRASDROIT);
        wait_ms(1000);
        servo.position(DESACTIVERBRASDROIT); 
        wait_ms(1000);  
    }  
}


void Bras::activerBras()
{
    if(id == GAUCHE)
    {
        servo.position(ACTIVATIONBRASGAUCHE);
        wait_ms(1000);
    }   
    if(id == DROIT)
    {
        servo.position(ACTIVATIONBRASDROIT);
        wait_ms(1000); 
    }     
}

void Bras::desactiverBras()
{
    if(id == GAUCHE)
    {
        servo.position(DESACTIVERBRASGAUCHE);
        wait_ms(1000);
    }   
    if(id == DROIT)
    {
        servo.position(DESACTIVERBRASDROIT);
        wait_ms(1000); 
    }    
}