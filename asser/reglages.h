//position initiale du robot
#define X_INIT 0
#define Y_INIT 0
#define THETA_INIT 0

//propre a chaque robot
#define ECART_ROUE 24870 // a augmenter si l'angle reel est plus grand que l'angle vise //31190
#define DISTANCE_PAR_TICK_D 8.43 // si le robot va trop loin, à augmenter//8.5
#define DISTANCE_PAR_TICK_G 8.43




//correction mécanique
#define COEFF_MOTEUR_D 1.00 //1.085
#define COEFF_MOTEUR_G 0.97   //1.10

//contraintes mecaniques
#define PWM_MAX 900  //PWM maximal, à cette valeur le robot est à sa vitesse maximale admissible

