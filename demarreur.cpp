#include "demarreur.h"

Demarreur::Demarreur(PinName cmd): pin(cmd, PullDown){
    state = pin.read();   
}

bool Demarreur::getState(){
    wait_ms(50);
    state = pin.read(); 
    return state;  
}