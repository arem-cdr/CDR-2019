#include "Strategie.h"

char bufferScore[3]={'0','0','0'}; 

bool typeMovement = DEPLACEMENT_AVANT;
int typeEvitement = ARRET;

extern int distanceUltrasonGauche;
extern int distanceUltrasonDroit;
extern int distanceUltrasonArriere;

extern int distanceTOF1;
extern int distanceTOF2;
extern int distanceTOF3;
extern int distanceTOF4;
extern int distanceTOF5;


extern bool stopCapteurs;


Timeout tempsArretMvt;



void updateAndWriteScore(char n1, char n2, char n3)
{
    bufferScore[0] = n1;
    bufferScore[1] = n2;
    bufferScore[2] = n3;
    write(bufferScore);
}

void LectureI2CCarteCapteur(deplacement robot)
{
    traitementBufferCapteurs();
    if(AnalyseDistance(distanceUltrasonGauche, distanceUltrasonDroit, distanceUltrasonArriere, typeMovement) == ROBOT)
    {
            if( ((distanceUltrasonGauche <= 300)  || (distanceUltrasonDroit <= 300)) && (typeMovement == DEPLACEMENT_AVANT) )
            {
                    stopCapteurs = true;           
            }
            else if ( (distanceUltrasonArriere <= 300) && (typeMovement == DEPLACEMENT_ARRIERE) )
            {
                    stopCapteurs = true;
            }
            else 
            {
                    stopCapteurs = false;
            }
    }
    else
    {
        stopCapteurs = false;   
    }
}

void arretSystem()
{
    Pompe pompe(PIN_POMPE);
    Demarreur demarreur(PIN_DEMARREUR);
    Bras brasGauche(PIN_SERVO_BRAS_GAUCHE, GAUCHE);
    Bras brasDroit(PIN_SERVO_BRAS_DROIT, DROIT);
    BrasPousser brasPousserGauche(PIN_SERVO_PALETS_GAUCHE, GAUCHE);
    BrasPousser brasPousserDroit(PIN_SERVO_PALETS_DROIT,DROIT);
    deplacement robot;
    
    robot.initialisation();
    robot.arreterRobot();
    pompe.desactiver();
}

/*---------Debut Strategie homologation violet ------------------------------------------------------*/


void strategieHomologationViolet(deplacement robot, Pompe pompe, Demarreur demarreur, Bras brasGauche, Bras brasDroit, BrasPousser brasPousserGauche,  BrasPousser brasPousserDroit)
{
    write(bufferScore);
    Timer tempsMatch; // timer en seconde
    int etapeMatch = 0;
    while(demarreur.getState() == false)
    {
        //on attend
        //pc.printf("On attend le depart du Robot Thanos\n\r");
    }
    tempsArretMvt.attach(&arretSystem, TEMPSMATCH);
    robot.initialisation();
    lancerTimerEcran();
    tempsMatch.start();
    typeEvitement = ARRET;
    while(tempsMatch  <= TEMPSMATCH)
    {   
        switch (etapeMatch) {

            case 0 : /* On tourne pour aller vers le terrain adverse et on avance jusqu'au milieu du terrain */ 
            {
                typeMovement = DEPLACEMENT_AVANT;
                robot.rotation_rel(90);
                robot.ligne_droite_basique(127500);
                etapeMatch++;
                
            break;
            }
            case 1 : /* On se met en position pour pousser les palets */ 
            {
                robot.rotation_rel(90);
                robot.ligne_droite_basique(60000);
                robot.rotation_rel(90);
                brasPousserDroit.activerBras();
                brasPousserGauche.activerBras();
                etapeMatch++;
            
            break;
            }
            case 2 : /* On pousse la zone de chaos dans les cases */ 
            {
                robot.ligne_droite_basique(110000);
                updateAndWriteScore('0', '1', '0');
                etapeMatch++;
            
            break;
            }
            case 3 : /* On se met en position pour faire des rectangles */ 
            {
                typeMovement = DEPLACEMENT_ARRIERE;
                robot.marche_arriere(-10000);
                brasPousserDroit.desactiverBras();
                brasPousserGauche.desactiverBras();
                etapeMatch++;
            
            break;
            }
            case 4 : /* On fait un rectangle arriere  */ 
            {
                
                robot.marche_arriere(-50000);
                robot.rotation_rel(-90);
                robot.marche_arriere(-50000);
                robot.rotation_rel(-90);
                robot.marche_arriere(-50000);
                robot.rotation_rel(-90);
                robot.marche_arriere(-50000);
                //robot.rotation_rel(180);
                etapeMatch++;
            
            break;
            }
            case 5 : /* On pousse le palet vert ou le vert et les deux bleus dans la balance en passant par la pente */ 
            {
                typeMovement = DEPLACEMENT_AVANT;
                robot.ligne_droite_basique(50000);
                robot.rotation_rel(90);
                robot.ligne_droite_basique(50000);
                robot.rotation_rel(90);
                robot.ligne_droite_basique(50000);
                robot.rotation_rel(90);
                robot.ligne_droite_basique(50000);
                robot.rotation_rel(90);
                etapeMatch++;
            
            break;
            }
            
            default:
            {
                robot.vitesse_nulle_D(0);
                robot.vitesse_nulle_G(0);
                motors_stop();
            }
        }  
    }
    
    //Arrêter les moteurs
    robot.vitesse_nulle_D(0);
    robot.vitesse_nulle_G(0);
    motors_stop();
    while(1);
}

/*-------Fin Stratégie homologation violet------------------------------------------------------------*/


/*-------Debut Strategie homologation jaune-----------------------------------------------------------*/


void strategieHomologationJaune(deplacement robot, Pompe pompe, Demarreur demarreur, Bras brasGauche, Bras brasDroit, BrasPousser brasPousserGauche,  BrasPousser brasPousserDroit)
{
    write(bufferScore);
    Timer tempsMatch; // timer en seconde
    int etapeMatch = 0;
    while(demarreur.getState() == false)
    {
        //on attend
        //pc.printf("On attend le depart du Robot Thanos\n\r");
    }
    tempsArretMvt.attach(&arretSystem, TEMPSMATCH);
    robot.initialisation();
    lancerTimerEcran();
    tempsMatch.start();
    typeEvitement = ARRET;
    while(tempsMatch  <= TEMPSMATCH)
    {   
        switch (etapeMatch) {
            

            case 0 : /* On tourne pour aller vers le terrain adverse et on avance jusqu'au milieu du terrain */ 
            {
                robot.rotation_rel(-90);
                robot.ligne_droite_basique(127500);
                etapeMatch++;
                
            break;
            }
            case 1 : /* On se met en position pour pousser les palets */ 
            {
                robot.rotation_rel(-90);
                robot.ligne_droite_basique(60000);
                robot.rotation_rel(-90);
                brasPousserDroit.activerBras();
                brasPousserGauche.activerBras();
                etapeMatch++;
            
            break;
            }
            case 2 : /* On pousse la zone de chaos dans les cases */ 
            {
                robot.ligne_droite_basique(110000);
                updateAndWriteScore('0', '1', '0');
                etapeMatch++;
            
            break;
            }
            case 3 : /* On se met en position pour faire des rectangles */ 
            {
                robot.marche_arriere(-10000);
                brasPousserDroit.desactiverBras();
                brasPousserGauche.desactiverBras();
                etapeMatch++;
            
            break;
            }
            case 4 : /* On fait un rectangle arriere  */ 
            {
                
                robot.marche_arriere(-50000);
                robot.rotation_rel(90);
                robot.marche_arriere(-50000);
                robot.rotation_rel(90);
                robot.marche_arriere(-50000);
                robot.rotation_rel(90);
                robot.marche_arriere(-50000);
                //robot.rotation_rel(180);
                etapeMatch++;
            
            break;
            }
            case 5 : /* On pousse le palet vert ou le vert et les deux bleus dans la balance en passant par la pente */ 
            {
                robot.ligne_droite_basique(50000);
                robot.rotation_rel(-90);
                robot.ligne_droite_basique(50000);
                robot.rotation_rel(-90);
                robot.ligne_droite_basique(50000);
                robot.rotation_rel(-90);
                robot.ligne_droite_basique(50000);
                robot.rotation_rel(-90);
                etapeMatch++;
            
            break;
            }
            
            default:
            {
                robot.vitesse_nulle_D(0);
                robot.vitesse_nulle_G(0);
                motors_stop();
            }
        }  
    }
    
    //Arrêter les moteurs
    robot.vitesse_nulle_D(0);
    robot.vitesse_nulle_G(0);
    motors_stop();
    while(1);
}

/*Fin Strategie homologation jaune -------------------------------------------------------------------*/


/*Debut Strategie classique violet -------------------------------------------------------------------*/


void strategieClassiqueViolet(deplacement robot, Pompe pompe, Demarreur demarreur, Bras brasGauche, Bras brasDroit, BrasPousser brasPousserGauche,  BrasPousser brasPousserDroit)
{
    write(bufferScore);
    Timer tempsMatch; // timer en seconde
    int etapeMatch = 0;
    while(demarreur.getState() == false)
    {
        //on attend
        //pc.printf("On attend le depart du Robot Thanos\n\r");
    }
    tempsArretMvt.attach(&arretSystem, TEMPSMATCH);
    robot.initialisation();//check si modif
    lancerTimerEcran();
    tempsMatch.start();
    while(tempsMatch  <= TEMPSMATCH)
    {   
        switch (etapeMatch) {

            case 0 : /* On active l'expérience */ 
            {
                typeMovement = DEPLACEMENT_AVANT;
                robot.ligne_droite_basique(26500);
                robot.poussette(300);
                updateAndWriteScore('0', '4', '0');
                etapeMatch++;
                
            break;
            }
            case 1 : /* On pousse le palet bleu de la pente de l'accélérateur */ 
            {
                typeMovement = DEPLACEMENT_ARRIERE;
                robot.marche_arriere(-11200);
                typeMovement = DEPLACEMENT_AVANT;
                /*robot.rotation_rel(90);
                robot.ligne_droite_basique(130000);*/
                robot.va_au_point(160000,23000,0);
                brasDroit.activerBras();
                robot.ligne_droite_basique(14000);
                brasDroit.desactiverBras();
                updateAndWriteScore('0', '5', '0');
                etapeMatch++;
            
            break;
            }
            case 2 : /* On récupère le goldenium */ 
            {
                robot.va_au_point(223000,30000,-90);
                //robot.ligne_droite_basique(54500);
                //robot.rotation_rel(-90);
                pompe.activer();
                robot.poussette(1500);
                typeMovement = DEPLACEMENT_ARRIERE;
                robot.marche_arriere(-10000);
                typeMovement = DEPLACEMENT_AVANT;
                updateAndWriteScore('0', '8', '0');
                etapeMatch++;
            
            break;
            }
            case 3 : /* On met le goldenium dans la balance */ 
            {
                typeEvitement = ARRET;
                robot.va_au_point(130700,116000,90);
                robot.ligne_droite_basique(15000);
                /*robot.rotation_rel(-90);
                robot.ligne_droite_basique(93000);
                robot.rotation_rel(-90);
                robot.ligne_droite_basique(104000);*/
                robot.poussette(1500);
                pompe.desactiver();
                wait(2);
                updateAndWriteScore('1', '0', '4');
                etapeMatch++;
            
            break;
            }
            case 4 : /* On pousse la zone de chaos dans la zone du redium  */ 
            {
                typeMovement = DEPLACEMENT_ARRIERE;
                robot.marche_arriere(-8000);
                typeMovement = DEPLACEMENT_AVANT;
                robot.rotation_abs(-134);
                brasPousserGauche.activerBras();
                brasPousserDroit.activerBras();
                robot.ligne_droite_basique(116000);
                updateAndWriteScore('1', '2', '4');
                etapeMatch++;
            
            break;
            }
            case 5 : /* On pousse le palet vert ou le vert et les deux bleus dans la balance en passant par la pente */ 
            {
                brasPousserGauche.desactiverBras();
                brasPousserDroit.desactiverBras();
                typeMovement = DEPLACEMENT_ARRIERE;
                robot.marche_arriere(-8000);
                typeMovement = DEPLACEMENT_AVANT;
                robot.va_au_point(18000,178000,0);
                robot.ligne_droite_basique(5000);
                brasPousserGauche.activerBras();
                brasPousserDroit.activerBras();
                etapeMatch++;
            
            break;
            }
            case 6 : /* On redescant de la pente */ 
            {
                robot.pente_combo(0, brasPousserGauche,brasPousserDroit,pompe);
                updateAndWriteScore('1', '3', '4');
                robot.vitesse_nulle_D(0);
                robot.vitesse_nulle_G(0);
                etapeMatch++;
            
            break;
            }
            default:
            {
                robot.vitesse_nulle_D(0);
                robot.vitesse_nulle_G(0);
                motors_stop();
                pompe.desactiver();
            }
        }  
    }
    
    //Arrêter les moteurs
    robot.vitesse_nulle_D(0);
    robot.vitesse_nulle_G(0);
    motors_stop();
    pompe.desactiver();
    while(1);
}


/*-------- Fin Strategie classique violet -----------------------------------------------*/


/*------- Debut Strategie classique Jaune -----------------------------------------------*/


void strategieClassiqueJaune(deplacement robot, Pompe pompe, Demarreur demarreur, Bras brasGauche, Bras brasDroit, BrasPousser brasPousserGauche,  BrasPousser brasPousserDroit)
{
    write(bufferScore);
    Timer tempsMatch; // timer en seconde
    int etapeMatch = 0;
    while(demarreur.getState() == false)
    {
        //on attend
        //pc.printf("On attend le depart du Robot Thanos\n\r");
    }
    tempsArretMvt.attach(&arretSystem, 100.0);
    robot.initialisation();
    lancerTimerEcran();
    tempsMatch.start();
    while(tempsMatch  <= TEMPSMATCH)
    {   
        switch (etapeMatch) {

            case 0 : /* On active l'expérience */ 
            {
                typeMovement = DEPLACEMENT_AVANT;
                robot.ligne_droite_basique(26500);
                robot.poussette(300);
                updateAndWriteScore('0', '4', '0');
                etapeMatch++;
                
            break;
            }
            case 1 : /* On pousse le palet bleu de la pente de l'accélérateur */ 
            {
                typeMovement = DEPLACEMENT_ARRIERE;
                robot.marche_arriere(-11200);
                typeMovement = DEPLACEMENT_AVANT;
                /*robot.rotation_rel(90);
                robot.ligne_droite_basique(130000);*/
                robot.va_au_point(300000-160000,23000,180);
                brasGauche.activerBras();
                robot.ligne_droite_basique(14000);
                brasGauche .desactiverBras();
                updateAndWriteScore('0', '5', '0');
                etapeMatch++;
            
            break;
            }
            case 2 : /* On récupère le goldenium */ 
            {
                robot.va_au_point(300000-223000,30000,-90);
                //robot.ligne_droite_basique(54500);
                //robot.rotation_rel(-90);
                pompe.activer();
                robot.poussette(1500);
                typeMovement = DEPLACEMENT_ARRIERE;
                robot.marche_arriere(-10000);
                typeMovement = DEPLACEMENT_AVANT;
                updateAndWriteScore('0', '8', '0');
                etapeMatch++;
            
            break;
            }
            case 3 : /* On met le goldenium dans la balance */ 
            {
                typeEvitement = ARRET;
                robot.va_au_point(300000-130700,116000,90);
                robot.ligne_droite_basique(15000);
                /*robot.rotation_rel(-90);
                robot.ligne_droite_basique(93000);
                robot.rotation_rel(-90);
                robot.ligne_droite_basique(104000);*/
                robot.poussette(1500);
                pompe.desactiver();
                wait(2);
                updateAndWriteScore('1', '0', '4');
                etapeMatch++;
            
            break;
            }
            case 4 : /* On pousse la zone de chaos dans la zone du redium  */ 
            {
                typeMovement = DEPLACEMENT_ARRIERE;
                robot.marche_arriere(-8000);
                typeMovement = DEPLACEMENT_AVANT;
                robot.rotation_abs(-44);
                brasPousserGauche.activerBras();
                brasPousserDroit.activerBras();
                robot.ligne_droite_basique(116000);
                updateAndWriteScore('1', '2', '4');
                etapeMatch++;
            
            break;
            }
            case 5 : /* On pousse le palet vert ou le vert et les deux bleus dans la balance en passant par la pente */ 
            {
                brasPousserGauche.desactiverBras();
                brasPousserDroit.desactiverBras();
                typeMovement = DEPLACEMENT_ARRIERE;
                robot.marche_arriere(-8000);
                typeMovement = DEPLACEMENT_AVANT;
                robot.va_au_point(300000-18000,178000,180);
                robot.ligne_droite_basique(5000);
                brasPousserGauche.activerBras();
                brasPousserDroit.activerBras();
                etapeMatch++;
            
            break;
            }
            case 6 : /* On redescant de la pente */ 
            {
                robot.pente_combo(180, brasPousserGauche,brasPousserDroit,pompe);
                updateAndWriteScore('1', '3', '4');
                robot.vitesse_nulle_D(0);
                robot.vitesse_nulle_G(0);
                etapeMatch++;
            
            break;
            }
            default:
            {
                robot.vitesse_nulle_D(0);
                robot.vitesse_nulle_G(0);
                motors_stop();
                pompe.desactiver();
            }
        }  
    }
    
    //Arrêter les moteurs
    robot.vitesse_nulle_D(0);
    robot.vitesse_nulle_G(0);
    motors_stop();
    pompe.desactiver();
    while(1);
}

/*--------- Fin Strategie classique jaune --------------------------------------------*/


/*--------- Debut Strategie RCVA violet ----------------------------------------------*/


void strategieRCVAViolet(deplacement robot, Pompe pompe, Demarreur demarreur, Bras brasGauche, Bras brasDroit, BrasPousser brasPousserGauche,  BrasPousser brasPousserDroit)
{
    write(bufferScore);
    Timer tempsMatch; // timer en seconde
    int etapeMatch = 0;
    while(demarreur.getState() == false)
    {
        //on attend
        //pc.printf("On attend le depart du Robot Thanos\n\r");
    }
    tempsArretMvt.attach(&arretSystem, TEMPSMATCH);
    robot.initialisation();
    lancerTimerEcran();
    tempsMatch.start();
    typeEvitement = ARRET;
}


/*-------- Fin Strategie  RCVA violet -----------------------------------------------*/


/*-------- Debut Strategie RCVA jaune -----------------------------------------------*/

void strategieRCVAJaune(deplacement robot, Pompe pompe, Demarreur demarreur, Bras brasGauche, Bras brasDroit, BrasPousser brasPousserGauche,  BrasPousser brasPousserDroit)
{
    write(bufferScore);
    Timer tempsMatch; // timer en seconde
    int etapeMatch = 0;
    while(demarreur.getState() == false)
    {
        //on attend
        //pc.printf("On attend le depart du Robot Thanos\n\r");
    }
    tempsArretMvt.attach(&arretSystem, TEMPSMATCH);
    robot.initialisation();
    lancerTimerEcran();
    tempsMatch.start();
    typeEvitement = ARRET;
}


/*-------- Fin Strategie RCVA jaune --------------------------------------------------*/

/*-------- Debut Strategie humiliation  violet ---------------------------------------*/

void strategieHumiliationViolet(deplacement robot, Pompe pompe, Demarreur demarreur, Bras brasGauche, Bras brasDroit, BrasPousser brasPousserGauche,  BrasPousser brasPousserDroit)
{
    write(bufferScore);
    Timer tempsMatch; // timer en seconde
    int etapeMatch = 0;
    while(demarreur.getState() == false)
    {
        //on attend
        //pc.printf("On attend le depart du Robot Thanos\n\r");
    }
    tempsArretMvt.attach(&arretSystem, TEMPSMATCH);
    robot.initialisation();
    lancerTimerEcran();
    tempsMatch.start();
    typeEvitement = ARRET;
}


/*--------- Fin Strategie humiliation violet ----------------------------------------*/


/*-------- Debut Strategie humiliation jaune ----------------------------------------*/

void strategieHumiliationJaune(deplacement robot, Pompe pompe, Demarreur demarreur, Bras brasGauche, Bras brasDroit, BrasPousser brasPousserGauche,  BrasPousser brasPousserDroit)
{
    write(bufferScore);
    Timer tempsMatch; // timer en seconde
    int etapeMatch = 0;
    while(demarreur.getState() == false)
    {
        //on attend
        //pc.printf("On attend le depart du Robot Thanos\n\r");
    }
    tempsArretMvt.attach(&arretSystem, TEMPSMATCH);
    robot.initialisation();
    lancerTimerEcran();
    tempsMatch.start();
    typeEvitement = ARRET;
}


/*--------- Fin Strategie humiliation jaune -----------------------------------------*/




