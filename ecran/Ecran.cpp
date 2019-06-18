#include "Ecran.h" 


event_callback_t afficherEcranCallback;
volatile char buffer[TAILLE] = ""; 
char bufferRequest[TAILLE]="";
char bufferCapteurs[TAILLEBUFFERCAPTEURS] = "";
Serial serial(PC_10, PC_11,115200);
DigitalOut myled(LED2);
Serial pc(SERIAL_TX, SERIAL_RX);
bool ready = false;
extern bool newOrder;
extern int typeEvitement;
//extern deplacement robot;
extern Pompe pompe;
extern Demarreur demarreur;
extern Bras brasGauche;
extern Bras brasDroit;
extern BrasPousser brasPousserGauche;
extern BrasPousser brasPousserDroit;

//buffers pour les tests 
const char pomp[TAILLE] = "TESpompe\n\r";
const char serbg[TAILLE] = "TESserbg\n\r";
const char serbd[TAILLE] = "TESserbd\n\r";
const char serpg[TAILLE] = "TESserpg\n\r";
const char serpd[TAILLE] = "TESserpd\n\r";
const char rg180[TAILLE] = "TESrg180\n\r";
const char rd180[TAILLE] = "TESrd180\n\r";
const char motav[TAILLE] = "TESmotav\n\r";
const char motar[TAILLE] = "TESmotar\n\r";

//buffers pour le match
//stratégie coté jaune
const char stratJauneHomologation[TAILLE] = "MATJhomo\n\r";
const char stratJauneClassique[TAILLE] = "MATJclas\n\r";
const char stratJauneRCVA[TAILLE] = "MATJrcva\n\r";
const char stratJauneHumiliation[TAILLE] = "MATJhumi\n\r";
//stratégie coté violet
const char stratVioletHomologation[TAILLE] = "MATVhomo\n\r";
const char stratVioletClassique[TAILLE] = "MATVclas\n\r";
const char stratVioletRCVA[TAILLE] = "MATVrcva\n\r";
const char stratVioletHumiliation[TAILLE] = "MATVhumi\n\r";

//buffer pour la démo ST
const char demstm32[TAILLE] = "DEMstm32\n\r";


void resetBuffer()
{
    for(int i = 0; i<TAILLE ; i++)
    {
        buffer[i] = '0';   
    }  
}

void blink()
{
      myled = 1; // LED is ON
      wait(0.2); // 200 ms
      myled = 0; // LED is OFF   
}

void writeCapteurs()
{
    serial.printf( "%s\n", bufferCapteurs); 
    //pc.printf( "%s\n", bufferCapteurs); 
}

void write(char bufferWrite[])
{
   serial.printf( "%s\n", bufferWrite); 
   //pc.printf( "%s\n", buffer);  
}
void read()
{
    serial.scanf("%s", buffer);
}


void afficherEcran(int events)
{
    //pc.printf("Reception ecran : %s \n\r", buffer);
    //pc.printf("Reception ecran : %s \n\r", bufferRequest);
    //On réactive la lecture après le callback = acquitter l'interruption
    copyBuffer();
    newOrder = true;
    //executeOrderInit();
    //resetBuffer();
    serial.read((uint8_t*)buffer, TAILLE, afficherEcranCallback, SERIAL_EVENT_RX_COMPLETE);
    
}

void copyBuffer()
{
    for(int i=0; i<TAILLE; i++)
    {
        bufferRequest[i] = buffer[i];  
    }
    //pc.printf("%s\n\r",bufferRequest);
}

void executeOrderInit(deplacement robot, Pompe pompe, Demarreur demarreur, Bras brasGauche, Bras brasDroit, BrasPousser brasPousserGauche,  BrasPousser brasPousserDroit)
{
    if(strcmp(bufferRequest, pomp) == 0)
    {
        pc.printf("Pompe\n\r");
        pompe.init();
        return;
    }
    else if(strcmp(bufferRequest, serbg) == 0)
    {
        pc.printf("Servo bras gauche\n\r");
        brasGauche.init();
        return;
    }
    else if(strcmp(bufferRequest, serbd) == 0)
    {
        pc.printf("Servo bras droit\n\r");
        brasDroit.init();
        return;
    }
    else if(strcmp(bufferRequest, serpg) == 0)
    {
        pc.printf("Servo palet gauche\n\r");
        brasPousserGauche.init();
        return;
    }
    else if(strcmp(bufferRequest, serpd) == 0)
    {
        pc.printf("Servo palet droit\n\r");
        brasPousserDroit.init();
        return;
    }
    else if(strcmp(bufferRequest, rg180) == 0)
    {
        pc.printf("Rotation de 180 par la gauche\n\r");
        robot.rotation_rel(180);
        return;
    }
    else if(strcmp(bufferRequest, rd180) == 0)
    {
        pc.printf("Rotation de 180 par la droite\n\r");
        robot.rotation_rel(-180);
        return;
    }
    else if(strcmp(bufferRequest, motav) == 0)
    {
        pc.printf("Avancer\n\r");
        //robot.initialisation();
        robot.ligne_droite_basique(15000);
        return;
    }
    else if(strcmp(bufferRequest, motar) == 0)
    {
        pc.printf("Reculer\n\r");
        robot.marche_arriere(-15000);
        return;
    }
    //Match jaune stratégie 
    else if(strcmp(bufferRequest, stratJauneHomologation) == 0)
    {
        pc.printf("Vous avez choisit la strategie homologation et vous etes l'equipe jaune\n\r");
        typeEvitement = ARRET;
        couleur = 1;
        setEmplacementDepartJaune();
        init_odometrie();
        ready = true;
        strategieHomologationJaune(robot, pompe, demarreur, brasGauche, brasDroit, brasPousserGauche,  brasPousserDroit);
        return;
    }
    else if(strcmp(bufferRequest, stratJauneClassique) == 0)
    {
        pc.printf("Vous avez choisit la strategie classique et vous etes l'equipe jaune\n\r");
        typeEvitement = ARC;
        couleur = 1;
        setEmplacementDepartJaune();
        init_odometrie();
        ready = true;
        strategieClassiqueJaune(robot, pompe, demarreur, brasGauche, brasDroit, brasPousserGauche,  brasPousserDroit);
        return;
    }
    else if(strcmp(bufferRequest, stratJauneRCVA) == 0)
    {
        pc.printf("Vous avez choisit la strategie qui va aneantir RCVA et vous etes l'equipe jaune\n\r");
        couleur = 1;
        typeEvitement = ARC;
        setEmplacementDepartJaune();
        init_odometrie();
        ready = true;
        strategieRCVAJaune(robot, pompe, demarreur, brasGauche, brasDroit, brasPousserGauche,  brasPousserDroit);
        return;
    }
    else if(strcmp(bufferRequest, stratJauneHumiliation) == 0)
    {
        pc.printf("Vous avez choisit la strategie qui va humilier votre adversaire  et vous etes l'equipe jaune\n\r");
        typeEvitement = ARC;
        couleur = 1;
        setEmplacementDepartJaune();
        init_odometrie();
        ready = true;
        strategieHumiliationJaune(robot, pompe, demarreur, brasGauche, brasDroit, brasPousserGauche,  brasPousserDroit);
        return;
    }
    //Match violet stratégie 
    else if(strcmp(bufferRequest, stratVioletHomologation) == 0)
    {
        pc.printf("Vous avez choisit la strategie homologation et vous etes l'equipe violette\n\r");
        typeEvitement = ARRET;
        couleur = 0;
        setEmplacementDepartViolet();
        init_odometrie();
        ready = true;
        strategieHomologationViolet(robot, pompe, demarreur, brasGauche, brasDroit, brasPousserGauche, brasPousserDroit);
        return;
    }
    else if(strcmp(bufferRequest, stratVioletClassique) == 0)
    {
        pc.printf("Vous avez choisit la strategie classique et vous etes l'equipe violette\n\r");
        typeEvitement = ARC;
        couleur = 0;
        setEmplacementDepartViolet();
        init_odometrie();
        ready = true;
        strategieClassiqueViolet(robot, pompe, demarreur, brasGauche, brasDroit, brasPousserGauche, brasPousserDroit);
        return;
    }
    else if(strcmp(bufferRequest, stratVioletRCVA) == 0)
    {
        pc.printf("Vous avez choisit la strategie qui va aneantir RCVA et vous etes l'equipe violette\n\r");
        typeEvitement = ARC;
        couleur = 0;
        setEmplacementDepartViolet();
        init_odometrie();
        ready = true;
        strategieRCVAViolet(robot, pompe, demarreur, brasGauche, brasDroit, brasPousserGauche,  brasPousserDroit);
        return;
    }
    else if(strcmp(bufferRequest, stratVioletHumiliation) == 0)
    {
        pc.printf("Vous avez choisit la strategie qui va humilier votre adversaire  et vous etes l'equipe violette\n\r");
        typeEvitement = ARC;
        couleur = 0;
        setEmplacementDepartViolet();
        init_odometrie();
        ready = true;
        strategieHumiliationViolet(robot, pompe, demarreur, brasGauche, brasDroit, brasPousserGauche,  brasPousserDroit);
        return;
    }
    //démonstration
    else if(strcmp(bufferRequest, demstm32) == 0)
    {
        pc.printf("Demonstration pour le 18 Avril avec STMicroelectronics\n\r");
        ready = true;
        return;
    }
    else
    {
        pc.printf("%s\n\r",bufferRequest);
        return;   
    }
}

void lancerTimerEcran()
{
     serial.printf( "start\n");  
}
