#define _POSIX_C_SOURCE 199309L
#include "mbed.h"
#include "reglages.h"
#include "hardware.h"
#include "DevSPI.h"
#include "Ecran.h"
#include "XNucleoIHM02A1.h"

// PWM_MAX est définit dans réglage;
bool moteurs_arret = false;
bool init_shield = false;

XNucleoIHM02A1 *x_nucleo_ihm02a1; //Création d'une entité pour la carte de contôle des pas à pas // Les valeurs à rentrer dépendent de l'alimentation du moteur.
L6470_init_t init[L6470DAISYCHAINSIZE] = {
/* First Motor. */
    {
        4.08,                           /* Motor supply voltage in V. */
        200,                           /* Min number of steps per revolution for the motor. */
        7.5,                           /* Max motor phase voltage in A. */
        7,                          /* Max motor phase voltage in V. */
        0,                         /* Motor initial speed [step/s]. */
        500,                         /* Motor acceleration [step/s^2] (comment for infinite acceleration mode). */
        1500.0,                        /* Motor deceleration [step/s^2] (comment for infinite deceleration mode). */
        1500.0,                         /* Motor maximum speed [step/s]. */
        0.0,                           /* Motor minimum speed [step/s]. */
        602.7,                         /* Motor full-step speed threshold [step/s]. */
        4.06,                          /* Holding kval [V]. */
        4.06,                          /* Constant speed kval [V]. */
        4.06,                          /* Acceleration starting kval [V]. */
        4.06,                          /* Deceleration starting kval [V]. */
        61.52,                         /* Intersect speed for bemf compensation curve slope changing [step/s]. */
        392.1569e-6,                   /* Start slope [s/step]. */
        643.1372e-6,                   /* Acceleration final slope [s/step]. */
        643.1372e-6,                   /* Deceleration final slope [s/step]. */
        0,                             /* Thermal compensation factor (range [0, 15]). */
        4.5*1000*1.10,            /* Ocd threshold [ma] (range [375 ma, 6000 ma]). */
        4.9*1000*1.00,            /* Stall threshold [ma] (range [31.25 ma, 4000 ma]). */
        StepperMotor::STEP_MODE_1_128, /* Step mode selection. */
        0xFF,                          /* Alarm conditions enable. */
        0x2E88                         /* Ic configuration. */
    },
 
    /* Second Motor. */
    {
        4.08,                           /* Motor supply voltage in V. */
        200,                           /* Min number of steps per revolution for the motor. */
        7.5,                           /* Max motor phase voltage in A. */
        7,                          /* Max motor phase voltage in V. */
        0,                         /* Motor initial speed [step/s]. */
        490,                         /* Motor acceleration [step/s^2] (comment for infinite acceleration mode). */
        1500.0,                         /* Motor deceleration [step/s^2] (comment for infinite deceleration mode). */
        1500.0,                         /* Motor maximum speed [step/s]. */
        0.0,                           /* Motor minimum speed [step/s]. */
        602.7,                         /* Motor full-step speed threshold [step/s]. */
        4.06,                          /* Holding kval [V]. */
        4.06,                          /* Constant speed kval [V]. */
        4.06,                           /* Acceleration starting kval [V]. */
        4.06,                          /* Deceleration starting kval [V]. */
        61.52,                         /* Intersect speed for bemf compensation curve slope changing [step/s]. */
        392.1569e-6,                   /* Start slope [s/step]. */
        643.1372e-6,                   /* Acceleration final slope [s/step]. */
        643.1372e-6,                   /* Deceleration final slope [s/step]. */
        0,                             /* Thermal compensation factor (range [0, 15]). */
        4.5*1000*1.10,            /* Ocd threshold [ma] (range [375 ma, 6000 ma]). */
        4.9*1000*1.00,             /* Stall threshold [ma] (range [31.25 ma, 4000 ma]). */
        StepperMotor::STEP_MODE_1_128, /* Step mode selection. */
        0xFF,                          /* Alarm conditions enable. */
        0x2E88                         /* Ic configuration. */
    }
};

L6470 **motors; //Instance des moteurs

DigitalOut led(LED2);
//Serial pc(USBTX, USBRX); // tx, rx //la liaison serie pc-robot fut redéfini autre part.
DevSPI dev_spi(D11, D12, D3);


//Connections codeuses
//Nucleo 401re
InterruptIn ENCAL(D9);
InterruptIn ENCAJ(D8);
InterruptIn ENCBL(D6);
InterruptIn ENCBJ(D5);


volatile long encoderValueA = 0; //nombre de tics sur l'encodeur A
volatile long encoderValueB = 0; //nombre de tics sur l'encodeur B

void init_hardware()
{
    pc.baud(2000000); //Initialisation de l'USART pc 

    /* Initializing Motor Control Expansion Board. */
    if (init_shield == false)
    {
        x_nucleo_ihm02a1 = new XNucleoIHM02A1(&init[0], &init[1], A4, A5, D4, A2, &dev_spi);
        motors = x_nucleo_ihm02a1->get_components();
        init_shield = true ;
        
        ENCAL.mode(PullUp); //Initialisation des codeuses, on active la resistance de pull
        ENCAJ.mode(PullUp);
        ENCBL.mode(PullUp);
        ENCBJ.mode(PullUp);
    
        ENCAL.rise(&updateEncoderA); // on lit les tics de codeuses sur chaque changement de front, et on appelle les fonctions de comptage de tics.
        ENCAL.fall(&updateEncoderA);
        ENCAJ.rise(&updateEncoderA);
        ENCAJ.fall(&updateEncoderA);
    
        ENCBL.rise(&updateEncoderB);
        ENCBL.fall(&updateEncoderB);
        ENCBJ.rise(&updateEncoderB);
        ENCBJ.fall(&updateEncoderB);
    }
    encoderValueA = 0;
    encoderValueB = 0;

}

void set_PWM_moteur_D(int PWM) // Pour faire tourner les moteurs, ça vient du hello world du shield.
{
    if (!moteurs_arret) {
        if (PWM > PWM_MAX) {
            motors[0]->prepare_run(StepperMotor::BWD, PWM_MAX); //BWD = backward , FWD = forward , la vitesse doit etre positive
        } else if (PWM <-PWM_MAX) {
            motors[0]->prepare_run(StepperMotor::FWD, PWM_MAX);
        } else if (PWM > 0) {
            motors[0]->prepare_run(StepperMotor::BWD, PWM);
        } else if (PWM < 0) {
            motors[0]->prepare_run(StepperMotor::FWD, -PWM);
        } else if (PWM == 0) {
            motors[0]->prepare_run(StepperMotor::BWD, 0);
        }
    } else {
        motors[0]->prepare_hard_hiz(); //mode haute impédence pour pouvoir déplacer le robot à la main
    }
    x_nucleo_ihm02a1->perform_prepared_actions();
}

void set_PWM_moteur_G(int PWM)
{

    if (!moteurs_arret) {
        if (PWM > PWM_MAX) {
            motors[1]->prepare_run(StepperMotor::FWD, PWM_MAX);
        } else if (PWM <-PWM_MAX) {
            motors[1]->prepare_run(StepperMotor::BWD, PWM_MAX);
        } else if (PWM > 0) {
            motors[1]->prepare_run(StepperMotor::FWD, PWM);
        } else if (PWM < 0) {
            motors[1]->prepare_run(StepperMotor::BWD, -PWM);
        } else if (PWM == 0) {
            motors[1]->prepare_run(StepperMotor::BWD, 0);
        }
    } else {
        motors[1]->prepare_hard_hiz(); //mode haute impédence pour pouvoir déplacer le robot à la main
    }
    x_nucleo_ihm02a1->perform_prepared_actions();
}


long int get_nbr_tick_D() 
{
    return encoderValueA;
}

long int get_nbr_tick_G()
{
    return encoderValueB;
}

void attente_synchro() //non utilisé
{
    //structute du temps d'attente de l'asservissement 10ms
    wait(0.010);
}

void motors_stop() //coupe les moteurs et les rends libres.
{
    moteurs_arret=1;
    motors[0]->prepare_hard_hiz(); //mode haute impédence pour pouvoir déplacer le robot à la main
    motors[1]->prepare_hard_hiz(); 
    x_nucleo_ihm02a1->perform_prepared_actions();
}

void motors_on() // il faut activer les moteurs pour qu'il puisse recevoir des commandes PWM. 
{
    moteurs_arret=0;
}


void allumer_del()
{
    led = 1;
}

void eteindre_del()
{
    led = 0;
}

void delay_ms()
{
}

void allumer_autres_del()
{
}

void eteindre_autres_del()
{
}
void toggle_autres_del() {}

void set_all_led()
{

}

//Il s'agit des fonctions appelées sur interruptions pour compter les ticks des codeuses, merci Internet

volatile int lastEncodedA = 0;
long lastencoderValueA = 0;
int lastMSBA = 0;
int lastLSBA = 0;

void updateEncoderA()
{
    int MSBA = ENCAL.read(); //MSB = most significant bit
    int LSBA = ENCAJ.read(); //LSB = least significant bit

    int encodedA = (MSBA << 1) |LSBA; //converting the 2 pin value to single number
    int sumA  = (lastEncodedA << 2) | encodedA; //adding it to the previous encoded value

    if(sumA == 0b1101 || sumA == 0b0100 || sumA == 0b0010 || sumA == 0b1011) encoderValueA ++;
    if(sumA == 0b1110 || sumA == 0b0111 || sumA == 0b0001 || sumA == 0b1000) encoderValueA --;

    lastEncodedA = encodedA; //store this value for next time
}


volatile int lastEncodedB = 0;
long lastencoderValueB = 0;
int lastMSBB = 0;
int lastLSBB = 0;

void updateEncoderB()
{
    int MSBB = ENCBL.read(); //MSB = most significant bit
    int LSBB = ENCBJ.read(); //LSB = least significant bit

    int encodedB = (MSBB << 1) |LSBB; //converting the 2 pin value to single number
    int sumB  = (lastEncodedB << 2) | encodedB; //adding it to the previous encoded value

    if(sumB == 0b1101 || sumB == 0b0100 || sumB == 0b0010 || sumB == 0b1011) encoderValueB ++;
    if(sumB == 0b1110 || sumB == 0b0111 || sumB == 0b0001 || sumB == 0b1000) encoderValueB --;

    lastEncodedB = encodedB; //store this value for next time
}

// permet de lire les tics de codeuses sur la liaison série.

void debugEncoder()
{
    printf("tick_D : %ld, tick_G : %ld\n", get_nbr_tick_D(),get_nbr_tick_G());
}
long int get_position_G(){
    /* Getting the current position. */
    long int position = motors[1]->get_position();
    return position;
    /* Printing to the console. */
    //printf("--> Getting the current position: %d\r\n", position);
    
}
long int get_position_D(){
    /* Getting the current position. */
    long int position = motors[0]->get_position();
    
    /* Printing to the console. */
    //printf("--> Getting the current position: %d\r\n", position);
    return position;
    
}

void bouton(){
    DigitalIn depart(USER_BUTTON);
    while (depart){
        //printf("attente\n");    
    }
}
