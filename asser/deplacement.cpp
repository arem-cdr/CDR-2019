#include "deplacement.h"
#include "Strategie.h"
#include "mbed.h"
#include "odometrie.h"
#include "hardware.h"
#include "math_precalc.h"
#include "reglages.h"
#include "Ecran.h"
#include "AnalyseArcLib.h"


extern Serial pc;
extern bool detectionUltrasons;
bool stopCapteurs = false;
extern int typeEvitement;

extern int distanceUltrasonGauche;
extern int distanceUltrasonDroit;
extern int distanceUltrasonArriere;

extern double posFinalRobotX;
extern double posFinalRobotY;

extern double posInterRobotX;
extern double posInterRobotY;

extern Position Pos1;
extern Position Pos2;
extern Position Pos3;

bool evitementValider = false; 

Position PositionPos1()
{
    Position P; 
    P.X = Pos1.X * 100;
    P.Y = Pos1.Y * 100;
    P.Cap = Pos1.Cap;
    return P;
}

Position PositionPos2()
{
    Position P;
    P.X = Pos2.X * 100;
    P.Y = Pos2.Y * 100;
    P.Cap = Pos2.Cap;
    return P;
}
Position PositionPos3()
{
    Position P; 
    P.X = Pos3.X * 100;
    P.Y = Pos3.Y * 100;
    P.Cap = Pos3.Cap;
    return P;
}




Coordonnees pointIntermediaire()
{
    Coordonnees P1;
    double x_init = get_x_actuel();
    double y_init = get_y_actuel();
    P1.x = posInterRobotX *100 - x_init;
    P1.y = posInterRobotY *100 - y_init;
    return P1;
}

Coordonnees pointFinale()
{
    Coordonnees P1;
    double x_init = get_x_actuel();
    double y_init = get_y_actuel();
    P1.x = posFinalRobotX *100 - x_init;
    P1.y = posFinalRobotY *100 - y_init;
    return P1;
}

deplacement::deplacement(){
    consigne_D = 0;
    consigne_G = 0;
    somme_erreur_D = 0;
    somme_erreur_G = 0;
    erreur_precedente_D = 0;
    erreur_precedente_G = 0;
    compteur_asser =0;
    somme_y=0;
    
    for (int k =0; k<5;k++){
        erreur_glissee_D[k] = 0;
        erreur_glissee_G[k] = 0;
    }
    compteur_glisse = 0;
    
    Kp_D = 1.5;//1
    Ki_D = 0.12;//0.15
    Kd_D = 0.5;//1
    
    Kp_G = 1;//1
    Ki_G = 0.13;//0.15
    Kd_G = 1.2;//1
    
    tick_prec_D=0;
    tick_prec_G = 0;
    dix_ms = 0;
    for (int k =0; k<TAILLE_TAB;k++){
        tab_cmd_G[k]=0;
        tab_cmd_D[k]=0;
        vtab_G[k]=0;
        vtab_D[k]=0;
        c_D[k]=0;
        c_G[k]=0;
    }
    consigne_tab[0][0]=0;
    consigne_tab[0][1]=0;
    
    consigne_tab[1][0]=10;
    consigne_tab[1][1]=10;
    
    consigne_tab[2][0]=20;
    consigne_tab[2][1]=20;
    
    consigne_tab[3][0]=30;
    consigne_tab[3][1]=30;
    
    consigne_tab[4][0]=40;
    consigne_tab[4][1]=40;
    
   /* consigne_tab[5][0]=3*5;
    consigne_tab[5][1]=3*5;
    
    consigne_tab[6][0]=3*6;
    consigne_tab[6][1]=3*6;
    
    consigne_tab[7][0]=3*7;
    consigne_tab[7][1]=3*7;
    
    consigne_tab[8][0]=3*8;
    consigne_tab[8][1]=3*8;
    
    consigne_tab[9][0]=3*9;
    consigne_tab[9][1]=3*9;
    
    consigne_tab[10][0]=3*10;
    consigne_tab[10][1]=3*10;
    
    consigne_tab[11][0]=3*11;
    consigne_tab[11][1]=3*11;
    
    consigne_tab[12][0]=3*12;
    consigne_tab[12][1]=3*12;
    
    consigne_tab[13][0]=3*13;
    consigne_tab[13][1]=3*13;
    
    consigne_tab[14][0]=3*14;
    consigne_tab[14][1]=3*14;
    
    consigne_tab[15][0]=0;
    consigne_tab[15][1]=0;
    
    consigne_tab[16][0]=0;
    consigne_tab[16][1]=0;
    
    consigne_tab[17][0]=0;
    consigne_tab[17][1]=0;
    
    consigne_tab[18][0]=0;
    consigne_tab[18][1]=0;
    
    consigne_tab[19][0]=0;
    consigne_tab[19][1]=0;*/
}

void deplacement::arreterRobot()
{
      vitesse_nulle_D(0); 
      vitesse_nulle_G(0); 
}

void deplacement::commande_vitesse(float vitesse_G,float vitesse_D){ //fonction pour commander les moteurs sans avoir à utiliser set_PWM
    
    int sens_G=signe(vitesse_G);
    int sens_D=signe(vitesse_D);
    double vitesse_local_G=abs(vitesse_G);
    double vitesse_local_D=abs(vitesse_D);
    
    if(abs(vitesse_G) > 900){
        vitesse_local_G=900;
    }
    if(abs(vitesse_G)<10){
        vitesse_local_G=10;
    }
    if(abs(vitesse_D) > 900){
        vitesse_local_D=900;
    }
    if(abs(vitesse_D)< 10){
        vitesse_local_D=10;
    
    }
    ;
    int VG_int = (int) vitesse_local_G*sens_G*COEFF_MOTEUR_G;
    int VD_int = (int) vitesse_local_D*sens_D*COEFF_MOTEUR_D;
    float VG_f = vitesse_local_G*sens_G*COEFF_MOTEUR_G;
    float VD_f = vitesse_local_D*sens_D*COEFF_MOTEUR_D;
    float centieme_D = (VD_f-VD_int)*1000;
    float centieme_G = (VG_f-VG_int)*1000;
    if ((rand()%1000) < centieme_G){
        VG_int+=1;
    }
    if ((rand()%1000) < centieme_D){
        VD_int+=1;
    }
    //printf("vitesseG : %f, vitesseD : %f, %d, %d", VG_f, VD_f, VG_int, VD_int);
    set_PWM_moteur_G(VG_int);//le branchements des moteurs est à vérifier ( fonctionne dans l'état actuel du robots
    set_PWM_moteur_D(VD_int);//
}
void deplacement::vitesse_nulle_G(int zero){
    if(zero == 0){
        set_PWM_moteur_G(0);
    }
}
void deplacement::vitesse_nulle_D(int zero){
    if(zero == 0){
        set_PWM_moteur_D(0);
    }
}
void deplacement::marche_arriere(int distance)
{
    somme_y=0;
    // le robot avance en ligne droite sur une distance donnée, à la vitesse voulue (entre 0 et 900)
    motors_on();
    actualise_position();
    double x_ini = get_x_actuel();
    double y_ini = get_y_actuel();
    double angle_vise_deg = get_angle();
    double angle_vise=angle_vise_deg*3.1416/180;
    double angle = get_angle();
    
    double x_local_ini = x_ini*cos(angle_vise) + y_ini*sin(angle_vise);
    double y_local_ini = y_ini*cos(angle_vise) - x_ini*sin(angle_vise);
    
    double x_actuel = get_x_actuel();
    double y_actuel = get_y_actuel();
    
    
    double x_local = x_actuel*cos(angle_vise) + y_actuel*sin(angle_vise)-x_local_ini;
    double y_local = y_actuel*cos(angle_vise) - x_actuel*sin(angle_vise)-y_local_ini;
    
    //long int y_local_prec = y_local;
    float vitesse_G;
    float vitesse_D;
    
    angle = get_angle();
    float Kip=0;
    float Kpp= 0.05 ;
    float Kdp= 10;
    while (distance-x_local<0){
             if(detectionUltrasons)
            {
                detectionUltrasons = false;
                //writeCapteurs();
                LectureI2CCarteCapteur(*this);
            }
            vitesse_G = (distance-x_local)/70;
            vitesse_D = vitesse_G;
            if(vitesse_G >400){
                vitesse_G=400;
                vitesse_D=400;
            }
            if (vitesse_G<-400){
                vitesse_G=-400;
                vitesse_D=-400;
            }
            
            angle = get_angle();
            
            vitesse_G = vitesse_G  - Kpp*y_local + Kdp * diff_angle(angle_vise_deg, angle) + Kip*somme_y;
            vitesse_D = vitesse_D  + Kpp*y_local - Kdp * diff_angle(angle_vise_deg, angle) - Kip*somme_y;
            //consigne_D = vitesse_D;
            //consigne_G = vitesse_G;
            if (stopCapteurs == true)
            {
                vitesse_nulle_D(0);
                vitesse_nulle_G(0);
            }
            else
            {
                commande_vitesse(vitesse_G,vitesse_D);
            }
            actualise_position();
            x_actuel = get_x_actuel();
            y_actuel = get_y_actuel();
            somme_y+=y_actuel;
            //y_local_prec = y_local;
            x_local = x_actuel*cos(angle_vise) + y_actuel*sin(angle_vise)-x_local_ini;
            y_local = y_actuel*cos(angle_vise) - x_actuel*sin(angle_vise)-y_local_ini;
            if (compteur_asser==150){
                compteur_asser=0;
                //printf("%lf\n",get_y_actuel());
            }
            compteur_asser++;
            //printf("   VG : %f  VD : %f ; x_local : %lf, y_local : %lf, angle_vise : %f\n",vitesse_G,vitesse_D, x_local,y_local, angle);// sqrt((x_ini - x_actuel)*(x_ini - x_actuel) + (y_ini - y_actuel)*(y_ini - y_actuel)), y_actuel, (x_actuel - x_ini)*(x_actuel - x_ini) + (y_actuel - y_ini)*(y_actuel - y_ini));
    }
    //printf("x : %d, y : %d, angle : %f\n", get_x_actuel(), get_y_actuel(),get_angle());
    rotation_abs(angle_vise_deg);
    //printf("x : %d, y : %d, angle : %f\n", get_x_actuel(), get_y_actuel(),get_angle());
}


void deplacement::ligne_droite_basique(long int distance)
{
    somme_y=0;
    // le robot avance en ligne droite sur une distance donnée, à la vitesse voulue (entre 0 et 900)
    motors_on();
    actualise_position();
    double x_ini = get_x_actuel();
    double y_ini = get_y_actuel();
    double angle_vise_deg = get_angle();
    double angle_vise=angle_vise_deg*3.1416/180;
    double angle = get_angle();
    
    double x_local_ini = x_ini*cos(angle_vise) + y_ini*sin(angle_vise);
    double y_local_ini = y_ini*cos(angle_vise) - x_ini*sin(angle_vise);
    
    double x_actuel = get_x_actuel();
    double y_actuel = get_y_actuel();
    
    
    double x_local = x_actuel*cos(angle_vise) + y_actuel*sin(angle_vise)-x_local_ini;
    double y_local = y_actuel*cos(angle_vise) - x_actuel*sin(angle_vise)-y_local_ini;
    
    //long int y_local_prec = y_local;
    float vitesse_G;
    float vitesse_D;
    
    angle = get_angle();
    float Kip=0;
    float Kpp= 0.05 ;
    float Kdp= 10;
    while (distance-x_local>0){
            
            if(detectionUltrasons)
            {
                detectionUltrasons = false;
                evitementValider = false;
                //writeCapteurs();
                LectureI2CCarteCapteur(*this);
            }
            vitesse_G = (distance-x_local)/70;
            vitesse_D = vitesse_G;
            if(vitesse_G >400){
                vitesse_G=400;
                vitesse_D=400;
            }
            if (vitesse_G<-400){
                vitesse_G=-400;
                vitesse_D=-400;
            }
            
            angle = get_angle();
            
            vitesse_G = vitesse_G  + Kpp*y_local + Kdp * diff_angle(angle_vise_deg, angle) + Kip*somme_y;
            vitesse_D = vitesse_D  - Kpp*y_local - Kdp * diff_angle(angle_vise_deg, angle) - Kip*somme_y;
            //consigne_D = vitesse_D;
            //consigne_G = vitesse_G;
            if (typeEvitement == ARRET)
            {
                if(stopCapteurs == true)
                {
                    evitementValider = true;
                    vitesse_nulle_D(0);
                    vitesse_nulle_G(0);
                }   
                else
                {
                    commande_vitesse(vitesse_G,vitesse_D);
                }   
            }  
            actualise_position();
            x_actuel = get_x_actuel();
            y_actuel = get_y_actuel();
            commande_vitesse(vitesse_G,vitesse_D);
            somme_y+=y_actuel;
            //y_local_prec = y_local;
            x_local = x_actuel*cos(angle_vise) + y_actuel*sin(angle_vise)-x_local_ini;
            y_local = y_actuel*cos(angle_vise) - x_actuel*sin(angle_vise)-y_local_ini;
            if (compteur_asser==150){
                compteur_asser=0;
                //printf("%lf\n",get_y_actuel());
            }
            compteur_asser++;
            //printf("   VG : %f  VD : %f ; x_local : %d, y_local : %d, angle_vise : %f",vitesse_G,vitesse_D, x_local,y_local, angle_vise_deg);// sqrt((x_ini - x_actuel)*(x_ini - x_actuel) + (y_ini - y_actuel)*(y_ini - y_actuel)), y_actuel, (x_actuel - x_ini)*(x_actuel - x_ini) + (y_actuel - y_ini)*(y_actuel - y_ini));
    }
    //printf("x : %d, y : %d, angle : %f\n", get_x_actuel(), get_y_actuel(),get_angle());
    rotation_abs(angle_vise_deg);
    //printf("x : %d, y : %d, angle : %f\n", get_x_actuel(), get_y_actuel(),get_angle());
}


void deplacement::ligne_droite(long int distance, double x, double y, double cap)
{
    somme_y=0;
    // le robot avance en ligne droite sur une distance donnée, à la vitesse voulue (entre 0 et 900)
    motors_on();
    actualise_position();
    double x_ini = get_x_actuel();
    double y_ini = get_y_actuel();
    double angle_vise_deg = get_angle();
    double angle_vise=angle_vise_deg*3.1416/180;
    double angle = get_angle();
    
    double x_local_ini = x_ini*cos(angle_vise) + y_ini*sin(angle_vise);
    double y_local_ini = y_ini*cos(angle_vise) - x_ini*sin(angle_vise);
    
    double x_actuel = get_x_actuel();
    double y_actuel = get_y_actuel();
    
    
    double x_local = x_actuel*cos(angle_vise) + y_actuel*sin(angle_vise)-x_local_ini;
    double y_local = y_actuel*cos(angle_vise) - x_actuel*sin(angle_vise)-y_local_ini;
    
    //long int y_local_prec = y_local;
    float vitesse_G;
    float vitesse_D;
    
    angle = get_angle();
    float Kip=0;
    float Kpp= 0.05 ;
    float Kdp= 10;
    while (distance-x_local>0){
            
            if(detectionUltrasons)
            {
                detectionUltrasons = false;
                evitementValider = false;
                //writeCapteurs();
                LectureI2CCarteCapteur(*this);
            }
            vitesse_G = (distance-x_local)/70;
            vitesse_D = vitesse_G;
            if(vitesse_G >400){
                vitesse_G=400;
                vitesse_D=400;
            }
            if (vitesse_G<-400){
                vitesse_G=-400;
                vitesse_D=-400;
            }
            
            angle = get_angle();
            
            vitesse_G = vitesse_G  + Kpp*y_local + Kdp * diff_angle(angle_vise_deg, angle) + Kip*somme_y;
            vitesse_D = vitesse_D  - Kpp*y_local - Kdp * diff_angle(angle_vise_deg, angle) - Kip*somme_y;
            //consigne_D = vitesse_D;
            //consigne_G = vitesse_G;
            if (typeEvitement == ARRET)
            {
                if(stopCapteurs == true)
                {
                    evitementValider = true;
                    vitesse_nulle_D(0);
                    vitesse_nulle_G(0);
                }   
                else
                {
                    commande_vitesse(vitesse_G,vitesse_D);
                }   
            }  
            else if(typeEvitement == RECTANGLE)
            {
                if(stopCapteurs == true)
                {
                    evitementRectangle(x, y, cap);
                    return;
                }
                else
                {
                    commande_vitesse(vitesse_G,vitesse_D);
                }
            }
            else
            {
                if(stopCapteurs == true)
                {
                    evitementArc(x, y, cap);
                    return;
                }
                else
                {
                    commande_vitesse(vitesse_G,vitesse_D);
                }
            }
            actualise_position();
            x_actuel = get_x_actuel();
            y_actuel = get_y_actuel();
            somme_y+=y_actuel;
            //y_local_prec = y_local;
            x_local = x_actuel*cos(angle_vise) + y_actuel*sin(angle_vise)-x_local_ini;
            y_local = y_actuel*cos(angle_vise) - x_actuel*sin(angle_vise)-y_local_ini;
            if (compteur_asser==150){
                compteur_asser=0;
                //printf("%lf\n",get_y_actuel());
            }
            compteur_asser++;
            //printf("   VG : %f  VD : %f ; x_local : %d, y_local : %d, angle_vise : %f",vitesse_G,vitesse_D, x_local,y_local, angle_vise_deg);// sqrt((x_ini - x_actuel)*(x_ini - x_actuel) + (y_ini - y_actuel)*(y_ini - y_actuel)), y_actuel, (x_actuel - x_ini)*(x_actuel - x_ini) + (y_actuel - y_ini)*(y_actuel - y_ini));
    }
    //printf("x : %d, y : %d, angle : %f\n", get_x_actuel(), get_y_actuel(),get_angle());
    rotation_abs(angle_vise_deg);
    //printf("x : %d, y : %d, angle : %f\n", get_x_actuel(), get_y_actuel(),get_angle());
}

void deplacement::rotation_rel(double angle_vise)
{
    // rotation de angle_vise
    motors_on();
    double vitesse=180;
    int sens;
    double angle = get_angle();
    angle_vise+=angle;
    borne_angle_d(angle_vise);
    if (diff_angle(angle,angle_vise)<=0){
        sens = -1;
        //printf("negatif\n");
    }
    else{
        sens = 1;
        
        //printf("positif\n");
    }
    //printf("diff : %lf ",diff_angle(angle,angle_vise));
    while ((sens*diff_angle(angle,angle_vise)>0) || abs(diff_angle(angle,angle_vise))>100)
    {
        actualise_position();
        angle = get_angle();
        vitesse=3*sens*abs(diff_angle(angle,angle_vise));
        if (vitesse > 90){
            vitesse = 90;
        }
        if (vitesse < -90){
            vitesse = -90;
        }
        commande_vitesse(-vitesse,vitesse);
        if (compteur_asser==150){
                compteur_asser=0;
                //printf("%lf\n",get_y_actuel());
            }
        compteur_asser++;
        //printf("vitesse : %lf ", vitesse);
    }
    //printf("\ndiff2 : %lf ",diff_angle(angle,angle_vise));
    //printf(" x et y recu : %lf, %ld. distance parcourue : %ld ", sqrt((x_ini - x_actuel)*(x_ini - x_actuel) + (y_ini - y_actuel)*(y_ini - y_actuel)), y_actuel, (x_actuel - x_ini)*(x_actuel - x_ini) + (y_actuel - y_ini)*(y_actuel - y_ini));
    //consigne_D = 0;
    //consigne_G = 0;
    vitesse_nulle_G(0);
    vitesse_nulle_D(0);
    wait(0.1);
    motors_stop();
}


void deplacement::rotation_abs(double angle_vise)
{
    actualise_position();
    //printf("bite");
    double angle_rel = borne_angle_d(angle_vise-get_angle());
    rotation_rel(angle_rel);
}

void deplacement::asservissement(){
    long int tick_D = get_nbr_tick_D();
    long int tick_G = get_nbr_tick_G();
    
    long int tick_D_passe = tick_D-tick_prec_D;
    long int tick_G_passe = tick_G-tick_prec_G;
    
    tick_prec_D=tick_D;
    tick_prec_G=tick_G;
    
    float vitesse_codeuse_D = tick_D_passe;
    float vitesse_codeuse_G = tick_G_passe;
    
    float erreur_D = (float) consigne_D - (float) vitesse_codeuse_D;
    float erreur_G = (float) consigne_G - (float) vitesse_codeuse_G;
    
    if (compteur_glisse == 5)
        compteur_glisse = 0;
    
    if (compteur_glisse == -1)
    {
        compteur_glisse = 0;
        for (int i = 0; i<5; i++){
            erreur_glissee_D[compteur_glisse] = erreur_D;
            erreur_glissee_G[compteur_glisse] = erreur_G;
        }
    }
    
    erreur_glissee_D[compteur_glisse] = erreur_D;
    erreur_glissee_G[compteur_glisse] = erreur_G;
    compteur_glisse++;
    
    erreur_D = erreur_glissee_D[0];
    erreur_G = erreur_glissee_G[0];
    for (int i=1; i<5; i++)
    {
         erreur_D += erreur_glissee_D[i];
         erreur_G += erreur_glissee_G[i];
    }
    
    erreur_D = erreur_D/5.0f;
    erreur_G = erreur_G/5.0f; // erreur est maintenant la moyenne des 5 erreurs prec
    
    somme_erreur_D += erreur_D;
    somme_erreur_G += erreur_G;
    
    float delta_erreur_D = erreur_D-erreur_precedente_D;
    float delta_erreur_G = erreur_G-erreur_precedente_G;
    
    erreur_precedente_G = erreur_G;
    erreur_precedente_D = erreur_D;
    
    float cmd_D = Kp_D*erreur_D+Ki_D*somme_erreur_D + Kd_D*delta_erreur_D;
    float cmd_G = Kp_G*erreur_G+Ki_G*somme_erreur_G + Kd_G*delta_erreur_G;
    
    if (cmd_G <0){
        cmd_G = 0;
    }
    if (cmd_G > 500){
        cmd_G = 500;
    }
    if (cmd_D <0){
        cmd_D = 0;
    }
    if (cmd_D > 500){
        cmd_D = 500;
    }
    c_D[dix_ms]=consigne_D;
    c_G[dix_ms]=consigne_G;
    //printf("%d\n",c[i]);
    tab_cmd_D[dix_ms] = cmd_D;
    tab_cmd_G[dix_ms] = cmd_G;
    vtab_D[dix_ms] = vitesse_codeuse_D;
    vtab_G[dix_ms] = vitesse_codeuse_G;
    commande_vitesse(cmd_G,cmd_D);
    dix_ms++;
    //printf("%d\n",i);
    //printf("tick : %ld cmd : %f,erreur : %f, somme_erreur : %f\n",tick_D_passe ,cmd_D,erreur_D, somme_erreur_D);
    //printf("%f,%f\n",cmd_G,cmd_D);
    //printf("oui");
}

void deplacement::printftab(){

    for (int j =0;j<TAILLE_TAB;j++){
        if(j==500)
            bouton();
        printf("%f,%f,%f,%f,%f,%f\n",tab_cmd_G[j],10*vtab_G[j],10*c_D[j],tab_cmd_D[j],10*vtab_D[j],10*c_G[j]);
    }
        /*if (j<5)
            printf("%f,%f,%f,%f,%f\n",tab_cmd_G[j],10*vtab_G[j],10*c[j],tab_cmd_D[j],10*vtab_D[j]);
        else
            printf("%f,%f,%f,%f,%f\n",tab_cmd_G[j],2*(vtab_G[j]+vtab_G[j-1]+vtab_G[j-2]+vtab_G[j-3]+vtab_G[j-4]),10*c[j],tab_cmd_D[j],2*(vtab_D[j]+vtab_D[j-1]+vtab_D[j-2]+vtab_D[j-3]+vtab_D[j-4]));
    }*/
    
    /*for (int j =0;j<TAILLE_TAB;j++){
        printf("%f,%f,%d\n",2*(vtab_G[j]+vtab_G[j-1]+vtab_G[j-2]+vtab_G[j-3]+vtab_G[j-4]), 2*(vtab_D[j]+vtab_D[j-1]+vtab_D[j-2]+vtab_D[j-3]+vtab_D[j-4]), j);
    }*/
}

void deplacement::test(){
    Timer t;
    t.start();
    for (int i =0;i<5 ;i++){
        changement_consigne(consigne_tab[i][0], consigne_tab[i][1]);
        while(t.read()<0.5f){
            //actualise_positio n();
        }
        //printf("t.read() : %f\n",t.read());
        //printf("consigne_D : %ld, consigne_G : %ld\n",consigne_D,consigne_G);
        t.reset();
    }
}

void deplacement::changement_consigne(int cons_D, int cons_G){
    consigne_D = cons_D;
    consigne_G = cons_G;
    compteur_glisse = -1;   
}


void deplacement::poussette(float temps){
    motors_on();
    commande_vitesse(100,100);
    wait_ms(temps);
    vitesse_nulle_G(0);
    vitesse_nulle_D(0);
    motors_stop();
}

void deplacement::initialisation(){
    init_odometrie();
    init_hardware();
    srand(time(NULL));
    motors_on();
}


void deplacement::arc(Coordonnees p1, Coordonnees p2, int sens){
    actualise_position();
    float vitesse_G;
    float vitesse_D;
    double x_ini = get_x_actuel();
    double y_ini = get_y_actuel();
    double angle_vise_deg = get_angle();
    double angle_vise=angle_vise_deg*3.1416/180.0;
    double angle = get_angle();
    
    double x_local_ini = x_ini*cos(angle_vise) + y_ini*sin(angle_vise);
    double y_local_ini = y_ini*cos(angle_vise) - x_ini*sin(angle_vise);
    
    double x_actuel = get_x_actuel();
    double y_actuel = get_y_actuel();
    
    
    double x_local = x_actuel*cos(angle_vise) + y_actuel*sin(angle_vise)-x_local_ini;
    double y_local = y_actuel*cos(angle_vise) - x_actuel*sin(angle_vise)-y_local_ini;
    /*double p1x = p1.x*cos(angle_vise) + p1.y*sin(angle_vise)-x_local_ini;
    double p2x = p2.x*cos(angle_vise) + p2.y*sin(angle_vise)-x_local_ini;
    double p1y = p1.y*cos(angle_vise) - p1.x*sin(angle_vise)-y_local_ini;
    double p2y = p2.y*cos(angle_vise) - p2.x*sin(angle_vise)-y_local_ini;*/
    
    Coordonnees p0;
    p0.x = x_local;
    p0.y = y_local;
    /*p1.x = p1x;
    p2.x = p2x;
    p1.y = p1y;
    p2.y = p2y;*/
    
    cercle(p0,p1,p2);
    double xc = point[0];
    double yc = point[1];
    double Rc = point[2];
    double angle_tan;
    
    double angle_a_parcourir = recup_angle_entre_trois_points_213(xc,yc,p0.x,p0.y,p2.x,p2.y);
    printf("angle_a_parcourirrir : %lf\n",angle_a_parcourir);
        
    if (angle_a_parcourir >0 && sens == A_DROITE){
            angle_a_parcourir -=360.0 ;
    }
    if (angle_a_parcourir <0 && sens == A_GAUCHE){
            angle_a_parcourir +=360;
    }
    
    if (angle_a_parcourir >= 0)
    {
        if (p0.y != yc){
            angle_tan = -atan((p0.x-xc)/(p0.y-yc))*180.0/3.1416;
            if (xc<0){
                angle_tan -=180.0;
            }
        }
        else{
            angle_tan = 90;
            if (xc<0){
                angle_tan -=180.0;
            }
        }
    }
    else
    {
        if (p0.y != yc){
            angle_tan = atan((p0.x-xc)/(p0.y-yc))*180.0/3.1416;
            if (xc<0){
                angle_tan+=180.0;
            }
        }
        else{
            angle_tan = -90;
            if (xc<0){
                angle_tan+=180.0;
            }
        }
            
    }
    
    //printf("angle_tan : %lf\n",angle_tan);
    rotation_rel(angle_tan);
    actualise_position();
    x_actuel = get_x_actuel();
    y_actuel = get_y_actuel();
    angle = get_angle();
    x_local = x_actuel*cos(angle_vise) + y_actuel*sin(angle_vise)-x_local_ini;
    y_local = y_actuel*cos(angle_vise) - x_actuel*sin(angle_vise)-y_local_ini;
    //printf("xc: %lf, yc : %lf, Rc: %lf,vg: %lf,vd : %lf\n",xc,yc,Rc,150.0,150.0*(Rc + ECART_ROUE/2.0)/(Rc-ECART_ROUE/2));
    //printf("x_loc : %lf, y_loc : %lf, angle : %lf,angle_tan : %lf,angle_a_parcourir : %lf\n",x_local,y_local,angle,angle_tan,angle_a_parcourir);
    motors_on();
    
    
    if (angle_a_parcourir < 0){
        double mon_angle_ini = recup_angle_entre_trois_points_213(xc,yc,x_local,y_local,p2.x,p2.y);
        double mon_angle = recup_angle_entre_trois_points_213(xc,yc,x_local,y_local,p2.x,p2.y);
        double mon_angle_prec;
        if (mon_angle >= 0){
            mon_angle -=360.0;
            mon_angle_ini -=360.0;   
        }
        mon_angle_prec = mon_angle;
        while (!( mon_angle_prec + mon_angle < -359.9 && abs(mon_angle - mon_angle_prec) > 100)){
            vitesse_D = abs(15 * mon_angle);
            if (vitesse_D>300){
                vitesse_D = 300;
            }
            vitesse_G = vitesse_D*(Rc-ECART_ROUE/2.0)/(Rc + ECART_ROUE/2.0);
            double correction = int_ext_cercle(x_local,y_local);
            double correction_en_cours = correction*0.035 - 3*diff_angle(borne_angle_d(angle_tan-angle_vise_deg),angle);
            if (correction_en_cours>50){
                correction_en_cours = 50;
            }
            if (correction_en_cours < -50){
                correction_en_cours = -50;
            }
            vitesse_D= vitesse_D-correction_en_cours;
            vitesse_G= vitesse_G+correction_en_cours;
            
            //printf("angle _tan : %lf, angle : %lf  , diff_angle : %lf , borne_angle : %lf\n",angle_tan, angle, diff_angle(borne_angle_d(angle_tan-angle_vise_deg),angle),borne_angle_d(angle_tan-angle_vise_deg));
            
            if(detectionUltrasons)
            {
                detectionUltrasons = false;
                evitementValider = false;
                //writeCapteurs();
                LectureI2CCarteCapteur(*this);
            }
            if(stopCapteurs == true)
            {
                 evitementValider = true;
                 vitesse_nulle_D(0);
                 vitesse_nulle_G(0);
            }   
            else
            {
                commande_vitesse(vitesse_G,vitesse_D);
            }     
            actualise_position();
            x_actuel = get_x_actuel();
            y_actuel = get_y_actuel();
            angle = get_angle();
            x_local = x_actuel*cos(angle_vise) + y_actuel*sin(angle_vise)-x_local_ini;
            y_local = y_actuel*cos(angle_vise) - x_actuel*sin(angle_vise)-y_local_ini;
            mon_angle_prec = mon_angle;
            mon_angle = recup_angle_entre_trois_points_213(xc,yc,x_local,y_local,p2.x,p2.y);
            
            
            if (mon_angle >= 0){
                mon_angle -=360.0;   
            }
            
            if (y_local-yc>=0){//gauche du cercle
                if (y_local != yc){
                    if (x_local-xc>= 0){//haut gauche
                        angle_tan = -atan((x_local-xc)/(y_local-yc))*180.0/3.1416 + 180;
                    }
                    else{//haut droite
                        angle_tan = -atan((x_local-xc)/(y_local-yc))*180.0/3.1416 - 180;
                    }
                }
                else{
                    if (x_local-xc>= 0){//haut gauche
                        angle_tan = 90;
                    }
                    else{//haut droite
                        angle_tan = -90;
                    }
                }
            }
            else{//partie droite
                if (x_local-xc>= 0){// haut droite
                    angle_tan = -atan((x_local-xc)/(y_local-yc))*180.0/3.1416;
                }
                else{ //bas droite
                    angle_tan = -atan((x_local-xc)/(y_local-yc))*180.0/3.1416;
                }
            }
            if (xc<0){
                angle_tan+=180.0;
            }
        }
    }
    if (angle_a_parcourir>= 0){
        double mon_angle_ini = recup_angle_entre_trois_points_213(xc,yc,x_local,y_local,p2.x,p2.y);
        double mon_angle = recup_angle_entre_trois_points_213(xc,yc,x_local,y_local,p2.x,p2.y);
        double mon_angle_prec;
        if (mon_angle <= 0){
            mon_angle +=360.0;
            mon_angle_ini +=360.0;
        }
        mon_angle_prec = mon_angle;
        //printf(" test :: %lf - %lf >< %lf ET %lf - %lf >< %lf, boolen : %d\n",mon_angle_ini,mon_angle,angle_a_parcourir,mon_angle_ini,mon_angle_prec,angle_a_parcourir,!(((mon_angle_ini - mon_angle) > angle_a_parcourir) && ((mon_angle_ini - mon_angle_prec) <= angle_a_parcourir))); 
        //while(!(((mon_angle_ini - mon_angle) > angle_a_parcourir) && ((mon_angle_ini - mon_angle_prec) >= angle_a_parcourir))){
        while (!( mon_angle_prec + mon_angle > 359.9 && abs(mon_angle - mon_angle_prec) > 100)){
            
            //printf(" test :: %lf - %lf >< %lf ET %lf + %lf >< %lf, boolen : %d\n",mon_angle_ini,mon_angle,angle_a_parcourir,mon_angle_ini,mon_angle_prec,angle_a_parcourir,!(((mon_angle_ini - mon_angle) > angle_a_parcourir) && ((mon_angle_ini - mon_angle_prec) <= angle_a_parcourir))); 
            vitesse_G = 15 * mon_angle;
            if (abs(vitesse_G)>300){
                vitesse_G = 300;
            }
            vitesse_D = vitesse_G*(Rc-ECART_ROUE/2.0)/(Rc + ECART_ROUE/2.0);
            double correction = int_ext_cercle(x_local,y_local);
            double correction_en_cours = correction*0.035 - 3*diff_angle(borne_angle_d(angle_tan+angle_vise_deg),angle);
            if (correction_en_cours>50){
                correction_en_cours = 50;
            }
            if (correction_en_cours < -50){
                correction_en_cours = -50;
            }
            vitesse_D= vitesse_D+correction_en_cours;
            vitesse_G= vitesse_G-correction_en_cours;
            //printf("%lf , %lf ,%lf ,%lf, correction : %lf, vitesse_D : %f, vitesse_G : %f\n",angle_tan , angle,borne_angle_d(angle_tan+angle_vise_deg) ,diff_angle(borne_angle_d(angle_tan+angle_vise_deg),angle),correction,vitesse_D,vitesse_G);
            if(detectionUltrasons)
            {
                detectionUltrasons = false;
                evitementValider = false;
                //writeCapteurs();
                LectureI2CCarteCapteur(*this);
            }
           if(stopCapteurs == true)
           {
                evitementValider = true;
                vitesse_nulle_D(0);
                vitesse_nulle_G(0);
           }   
           else
           {
                commande_vitesse(vitesse_G,vitesse_D);
           }    
            actualise_position();
            x_actuel = get_x_actuel();
            y_actuel = get_y_actuel();
            angle = get_angle();
            x_local = x_actuel*cos(angle_vise) + y_actuel*sin(angle_vise)-x_local_ini;
            y_local = y_actuel*cos(angle_vise) - x_actuel*sin(angle_vise)-y_local_ini;
            mon_angle_prec = mon_angle;
            mon_angle = recup_angle_entre_trois_points_213(xc,yc,x_local,y_local,p2.x,p2.y);
            if (mon_angle <= 0){
                mon_angle +=360.0;   
            }
            if (y_local-yc>=0){//gauche du cercle
                if (y_local != yc){
                    if (x_local-xc>= 0){//haut gauche
                        angle_tan = -atan((x_local-xc)/(y_local-yc))*180.0/3.1416;
                    }
                    else{//haut droite
                        angle_tan = -atan((x_local-xc)/(y_local-yc))*180.0/3.1416;
                    }
                }
                else{
                    if (x_local-xc>= 0){//haut gauche
                        angle_tan = -90;
                    }
                    else{//haut droite
                        angle_tan = 90;
                    }
                }
            }
            else{//partie droite
                if (x_local-xc>= 0){// haut droite
                    angle_tan = -atan((x_local-xc)/(y_local-yc))*180.0/3.1416 - 180.0 ;
                }
                else{ //bas droite
                    angle_tan = -atan((x_local-xc)/(y_local-yc))*180.0/3.1416 + 180.0 ;
                }
            }
            
            
        }
        //printf(" test :: %lf - %lf >< %lf ET %lf + %lf >< %lf, boolen : %d\n",mon_angle_ini,mon_angle,angle_a_parcourir,mon_angle_ini,mon_angle_prec,angle_a_parcourir,!(((mon_angle_ini - mon_angle) > angle_a_parcourir) && ((mon_angle_ini - mon_angle_prec) <= angle_a_parcourir))); 

    }

    //printf("distance : %lf ",recup_angle_entre_trois_points_213(xc,yc,x_local,y_local,p2.x,p2.y));
    //printf("x_loc : %lf, y_loc : %lf\n",x_local,y_local);
    vitesse_nulle_D(0);
    vitesse_nulle_G(0);
    wait(0.2);
    motors_stop();
    
}


int deplacement::cercle(Coordonnees a,Coordonnees b, Coordonnees c){
    double ax2 = pow(a.x,2);
    double ay2 = pow(a.y,2);
    double bx2 = pow(b.x,2);
    double by2 = pow(b.y,2);
    double cx2 = pow(c.x,2);
    double cy2 = pow(c.y,2);
    double pente_a=0;
    double ord_b;
    double pente_A=0;
    double ord_B;
    double xc;
    double yc;
    double Rc;
    if (a.y == b.y && b.y == c.y){
        printf("Les points sont alignes1\n");
        return 0;
    }
    if (b.y-a.y !=0){
        pente_a = -1*(b.x-a.x)/(b.y-a.y);
        printf("a : %lf ",pente_a);
        ord_b = (by2-ay2+bx2-ax2)/(2*(b.y-a.y));
        printf("b : %lf ",ord_B);

    }
    else{
        cercle(c,a,b);
        return 0;
     
    }
    if (c.y-a.y !=0){
        pente_A = -1*(c.x-a.x)/(c.y-a.y);
        printf("A : %lf ",pente_A);
        ord_B = (cy2-ay2+cx2-ax2)/(2*(c.y-a.y));
        printf("B : %lf ",ord_B);

    }
    else{
        cercle(b,c,a);
        return 0;
    }
    
    
    if (pente_a-pente_A!=0){
        xc = (ord_B-ord_b)/(pente_a-pente_A);
        yc = pente_a*xc+ord_b;
        Rc = pow((pow(xc-a.x,2)+pow(yc-a.y,2)),0.5);
        point[0] = xc;
        point[1] = yc;
        point[2] = Rc;
        printf("xc : %f, yc : %f, Rc : %f\n",xc,yc,Rc);
    }
    else{
        printf("Les points sont alignes2\n");
        return 0;
    }
    
    
    return 0;
}
double deplacement::int_ext_cercle(double x, double y){
    double xc= point[0];
    double yc= point[1];
    double Rc= point[2];
    double rayon = pow((pow(xc-x,2)+pow(yc-y,2)),0.5);
    return Rc-rayon;
    
}

void deplacement::va_au_point(double x,double y, double cap){
    actualise_position();
    double angle = get_angle();
    double x_robot = get_x_actuel();
    double y_robot = get_y_actuel();
    double x_projete = 10000.0*cos(angle*3.1416/180.0)+x_robot;
    double y_projete = 10000.0*sin(angle*3.1416/180.0)+y_robot;
    printf("angle ::: %lf\n",recup_angle_entre_trois_points_213(x_robot,y_robot,x,y,x_projete,y_projete));
    rotation_rel(recup_angle_entre_trois_points_213(x_robot,y_robot,x,y,x_projete,y_projete));
    //printf("oui\n");
    actualise_position();
    angle = get_angle();
    x_robot = get_x_actuel();
    y_robot = get_y_actuel();
    double distance = pow(pow(x-x_robot,2)+pow(y-y_robot,2),0.5);
    printf("distance : %lf\n",distance);
    ligne_droite(distance,  x, y, cap);
    actualise_position();
    angle = get_angle();
    x_robot = get_x_actuel();
    y_robot = get_y_actuel();
    rotation_abs(cap);
    
    
}

double deplacement::recup_angle_entre_trois_points_213(double x1,double y1,double x2,double y2,double x3,double y3){
    double x13 = x3-x1;
    double y13 = y3-y1;
    double x12 = x2-x1;
    double y12 = y2-y1;
    double norme12 = pow(x12*x12+y12*y12,0.5);
    double norme13 = pow(x13*x13+y13*y13,0.5);
    double ux = x13/norme13;
    double uy = y13/norme13;
    double wx = x12/norme12;
    double wy = y12/norme12;
    //printf("u : %lf,%lf ,v : %lf,%lf ,w : %lf,%lf\n",ux,uy,-uy,ux,wx,wy);
    double prod_scal = x12*x13+y12*y13;
    double cos_angle = prod_scal/(norme12*norme13);
    double sin_angle;
    if (uy!=0){
        sin_angle = -1*(wx - cos_angle*ux)/uy;
    }
    else{
        sin_angle = (wy - cos_angle*uy)/ux;
    }
    //printf("cos : %lf sin : %lf\n",cos_angle,sin_angle);
    if (sin_angle >=0){
        return acos(prod_scal/(norme12*norme13))*180.0/3.1416;
    }
    else{
        return -acos(prod_scal/(norme12*norme13))*180.0/3.1416;
    }
}
   
   
void deplacement::pente(long int distance, float vitesse, double angle_a_tourner)
{
    Timer time;
    time.start();
    somme_y=0;
    // le robot avance en ligne droite sur une distance donnée, à la vitesse voulue (entre 0 et 900)
    motors_on();
    actualise_position();
    double x_ini = get_x_actuel();
    double y_ini = get_y_actuel();
    double angle_vise_deg = get_angle();
    double angle_vise=angle_vise_deg*3.1416/180;
    double angle = get_angle();
    
    double x_local_ini = x_ini*cos(angle_vise) + y_ini*sin(angle_vise);
    double y_local_ini = y_ini*cos(angle_vise) - x_ini*sin(angle_vise);
    
    double x_actuel = get_x_actuel();
    double y_actuel = get_y_actuel();
    
    
    double x_local = x_actuel*cos(angle_vise) + y_actuel*sin(angle_vise)-x_local_ini;
    double y_local = y_actuel*cos(angle_vise) - x_actuel*sin(angle_vise)-y_local_ini;
    
    //long int y_local_prec = y_local;
    float vitesse_G;
    float vitesse_D;
    
    angle = get_angle();
    while (distance-x_local>0){
            
            vitesse_G = vitesse+y_local*0.02;
            vitesse_D = vitesse-y_local*0.02;
            
            
            commande_vitesse(vitesse_G,vitesse_D);
            actualise_position();
            x_actuel = get_x_actuel();
            y_actuel = get_y_actuel();
            somme_y+=y_actuel;
            //y_local_prec = y_local;
            x_local = x_actuel*cos(angle_vise) + y_actuel*sin(angle_vise)-x_local_ini;
            y_local = y_actuel*cos(angle_vise) - x_actuel*sin(angle_vise)-y_local_ini;
            
            //printf("   VG : %f  VD : %f ; x_local : %lf, y_local : %lf, angle_vise : %f\n",vitesse_G,vitesse_D, x_local,y_local, angle_vise_deg);// sqrt((x_ini - x_actuel)*(x_ini - x_actuel) + (y_ini - y_actuel)*(y_ini - y_actuel)), y_actuel, (x_actuel - x_ini)*(x_actuel - x_ini) + (y_actuel - y_ini)*(y_actuel - y_ini));
    }
    //printf("x : %lf, y : %lf, angle : %f\n", get_x_actuel(), get_y_actuel(),get_angle());
    rotation_abs_pente(angle_a_tourner);
    //printf("x : %d, y : %d, angle : %f\n", get_x_actuel(), get_y_actuel(),get_angle());
}

void deplacement::rotation_rel_pente(double angle_vise)
{
    // rotation de angle_vise
    motors_on();
    double vitesse=180;
    int sens;
    double angle = get_angle();
    angle_vise+=angle;
    borne_angle_d(angle_vise);
    if (diff_angle(angle,angle_vise)<=0){
        sens = -1;
        //printf("negatif\n");
    }
    else{
        sens = 1;
        
        //printf("positif\n");
    }
    //printf("diff : %lf ",diff_angle(angle,angle_vise));
    while ((sens*diff_angle(angle,angle_vise)>0) || abs(diff_angle(angle,angle_vise))>100)
    {
        actualise_position();
        angle = get_angle();
        vitesse=0.5*sens*abs(diff_angle(angle,angle_vise));
        
        commande_vitesse(-vitesse,vitesse);
        if (compteur_asser==150){
                compteur_asser=0;
                //printf("%lf\n",get_y_actuel());
            }
        compteur_asser++;
        //printf("vitesse : %lf ", vitesse);
    }
    //printf("\ndiff2 : %lf ",diff_angle(angle,angle_vise));
    //printf(" x et y recu : %lf, %ld. distance parcourue : %ld ", sqrt((x_ini - x_actuel)*(x_ini - x_actuel) + (y_ini - y_actuel)*(y_ini - y_actuel)), y_actuel, (x_actuel - x_ini)*(x_actuel - x_ini) + (y_actuel - y_ini)*(y_actuel - y_ini));
    //consigne_D = 0;
    //consigne_G = 0;
    vitesse_nulle_G(0);
    vitesse_nulle_D(0);
    
}

void deplacement::rotation_abs_pente(double angle_vise)
{
    actualise_position();
    //printf("bite");
    double angle_rel = borne_angle_d(angle_vise-get_angle());
    rotation_rel_pente(angle_rel);
}

void deplacement::pente_combo(double angle_pente, BrasPousser brasPousserGauche, BrasPousser brasPousserDroit, Pompe pompe){
    activerDeuxBras(brasPousserGauche, brasPousserDroit);
    motors_on();
    pente(15000,50,angle_pente);
    pente(10000,30,angle_pente);
    pente(52500,100,angle_pente);
    desactiverDeuxBras(brasPousserGauche, brasPousserDroit);
    commande_vitesse(80,80);
    wait(1.5);
    vitesse_nulle_D(0);
    vitesse_nulle_G(0);
    //hardHiz();
    pompe.desactiver();
    wait(3);
    rotation_rel(-90);
    wait(4);
    vitesse_nulle_D(0);
    vitesse_nulle_G(0);
}

void deplacement::evitementArc(double x, double y, double cap) 
{
        pc.printf("ARC\n\r");
        if((distanceUltrasonGauche == 1100) && (distanceUltrasonDroit != 1100) && (evitementValider == false))
        {
            if(lancerUnEvitementParArcGauche(distanceUltrasonGauche,x,y) == SANS_OBSTACLE)
            {
                evitementValider = true;
                actualise_position();
                Coordonnees P1 = pointIntermediaire();
                Coordonnees P2 = pointFinale();
                arc(P1,P2, A_GAUCHE);
            }
            else if(lancerUnEvitementParArcDroit(distanceUltrasonDroit,x,y) == SANS_OBSTACLE)  
            {
                evitementValider = true;
                actualise_position();
                Coordonnees P1 = pointIntermediaire();
                Coordonnees P2 = pointFinale();
                arc(P1,P2, A_DROITE);
            } 
        }
        if((distanceUltrasonGauche != 1100) && (distanceUltrasonDroit == 1100) && (evitementValider == false))
        {
            if(lancerUnEvitementParArcDroit(distanceUltrasonDroit,x,y) == SANS_OBSTACLE)  
            {
                evitementValider = true;
                actualise_position();
                Coordonnees P1 = pointIntermediaire();
                Coordonnees P2 = pointFinale();
                arc(P1,P2, A_DROITE);
            } 
            else if(lancerUnEvitementParArcGauche(distanceUltrasonGauche,x,y) == SANS_OBSTACLE)
            {
                evitementValider = true;
                actualise_position();
                Coordonnees P1 = pointIntermediaire();
                Coordonnees P2 = pointFinale();
                arc(P1,P2, A_GAUCHE);
            }
        }
        if((distanceUltrasonGauche != 1100) && (distanceUltrasonDroit != 1100) && (evitementValider == false))
        {
            if(lancerUnEvitementParArcGauche(distanceUltrasonGauche,x,y) == SANS_OBSTACLE)
            {
                evitementValider = true;
                actualise_position();
                Coordonnees P1 = pointIntermediaire();
                Coordonnees P2 = pointFinale();
                arc(P1,P2, A_GAUCHE);
            }
            else if(lancerUnEvitementParArcDroit(distanceUltrasonDroit,x,y) == SANS_OBSTACLE)  
            {
                evitementValider = true;
                actualise_position();
                Coordonnees P1 = pointIntermediaire();
                Coordonnees P2 = pointFinale();
                arc(P1,P2, A_DROITE);
            }   
        }
        else
        {
            evitementValider = true;
            vitesse_nulle_D(0);
            vitesse_nulle_G(0);
        }
    va_au_point(x,y,cap);
}



void deplacement::evitementRectangle(double x, double y, double cap) 
{
        pc.printf("Rectangle \n\r");
        if((distanceUltrasonGauche == 1100) && (distanceUltrasonDroit != 1100) && (evitementValider == false))
        {
            if(lancerUnEvitementParRectangleGauche(x,y,cap) == SANS_OBSTACLE)
            {
                evitementValider = true;
                actualise_position();
                Position P1 = PositionPos1();
                Position P2 = PositionPos2();
                Position P3 = PositionPos3();
                va_au_point(P1.X, P1.Y, P1.Cap);
                va_au_point(P2.X, P2.Y, P2.Cap);
                va_au_point(P3.X, P3.Y, P3.Cap); 
                
            }
            else if(lancerUnEvitementParRectangleDroit(x,y,cap) == SANS_OBSTACLE)  
            {
                evitementValider = true;
                actualise_position();
                Position P1 = PositionPos1();
                Position P2 = PositionPos2();
                Position P3 = PositionPos3();
                va_au_point(P1.X, P1.Y, P1.Cap);
                va_au_point(P2.X, P2.Y, P2.Cap);
                va_au_point(P3.X, P3.Y, P3.Cap); 
            } 
        }
        if((distanceUltrasonGauche != 1100) && (distanceUltrasonDroit == 1100) && (evitementValider == false))
        {
            if(lancerUnEvitementParRectangleDroit(x,y,cap) == SANS_OBSTACLE)  
            {
                evitementValider = true;
                actualise_position();
                Position P1 = PositionPos1();
                Position P2 = PositionPos2();
                Position P3 = PositionPos3();
                va_au_point(P1.X, P1.Y, P1.Cap);
                va_au_point(P2.X, P2.Y, P2.Cap);
                va_au_point(P3.X, P3.Y, P3.Cap); 
            } 
            else if(lancerUnEvitementParRectangleGauche(x,y,cap) == SANS_OBSTACLE)
            {
                evitementValider = true;
                actualise_position();
                Position P1 = PositionPos1();
                Position P2 = PositionPos2();
                Position P3 = PositionPos3();
                va_au_point(P1.X, P1.Y, P1.Cap);
                va_au_point(P2.X, P2.Y, P2.Cap);
                va_au_point(P3.X, P3.Y, P3.Cap); 
            }
        }
        if((distanceUltrasonGauche != 1100) && (distanceUltrasonDroit != 1100) && (evitementValider == false))
        {
            if(lancerUnEvitementParRectangleGauche(x,y,cap) == SANS_OBSTACLE)
            {
                evitementValider = true;
                actualise_position();
                Position P1 = PositionPos1();
                Position P2 = PositionPos2();
                Position P3 = PositionPos3();
                va_au_point(P1.X, P1.Y, P1.Cap);
                va_au_point(P2.X, P2.Y, P2.Cap);
                va_au_point(P3.X, P3.Y, P3.Cap); 
            }
            else if(lancerUnEvitementParRectangleDroit(x, y, cap) == SANS_OBSTACLE)  
            {
                evitementValider = true;
                actualise_position();
                Position P1 = PositionPos1();
                Position P2 = PositionPos2();
                Position P3 = PositionPos3();
                va_au_point(P1.X, P1.Y, P1.Cap);
                va_au_point(P2.X, P2.Y, P2.Cap);
                va_au_point(P3.X, P3.Y, P3.Cap); 
            }   
        }
        else
        {
            evitementValider = true;
            vitesse_nulle_D(0);
            vitesse_nulle_G(0);
        }
}