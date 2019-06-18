#ifndef HARDWARE_H
#define HARDWARE_H


void init_hardware(void);
void set_PWM_moteur_D(int PWM);
void set_PWM_moteur_G(int PWM);

void set_step_moteur_D(int steps);


long int get_nbr_tick_D(void);
long int get_nbr_tick_G(void);
void attente_synchro(void);

void allumer_del(void);
void eteindre_del(void);

void allumer_autres_del(void);
void eteindre_autres_del(void);
void toggle_autres_del(void);

void motors_on(void);
void motors_stop(void);

void updateEncoderA(void);
void updateEncoderB(void);

long int get_position_D();
long int get_position_G();
void debugEncoder(); //printf les ticks de codeuse
void bouton(); //attente de l'appuie sur le bouton bleu

#endif
