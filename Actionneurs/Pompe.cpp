#include "Pompe.h"


Pompe::Pompe(PinName cmd): pin(cmd) {
    state = 0;
    pin.write(0);
    
}

void Pompe::init()
{
    pin.write(1);
    wait_ms(2000);
    pin.write(0);
}

void Pompe::activer()
{
    pin.write(1);  
    state = true; 
}

void Pompe::desactiver()
{
    pin.write(0);  
    state = false;  
}

bool Pompe::getState()
{
    return state;   
}

