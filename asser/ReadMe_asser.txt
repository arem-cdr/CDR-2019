ReadMe_asser

Le principe de déplacement du robot utilisé lors de la Coupe de France de Robotique 2019 est d'utilisé de l'odométrie pour faire du suivi de trajectoire.

Pour cela nous disposions de 2 encodeurs incrémentals branché dans l'alignement des roues moteurs pour suivre les déplacements du robot.
L'idée général est donc de récupérer les tics des encodeurs sur intéruption pour actualiser la position du robot au sein du programme.
Ainsi grâce à cette position, nous pouvons faire du suivi de trajectoire : 
	-ligne_droite
	-rotation
	-arc de cercle
	
Notre code est composé de 6 fichiers principaux : odométrie.cpp/odométrie.h ; hardware.cpp/hardware.h ; deplacement.cpp/deplacement.h

Dans hardware.cpp/.h : il y a les fonctions permettant initialiser les ports, de configurer le shield, de configurer l'alimentation des moteurs, de compter les tics des encodeurs.
Dans odométrie.cpp/.h : il y a la fonction d'odométrie qui actualise la position du robot (x,y,angle).
Dans deplacement.cpp/.h : il y a les fonctions de suivi de trajectoires. 
	Pour cela, nous avons créé une classe deplacement. Il faut donc créer une entité deplacement robot pour accéder au fonction de déplacement, qui sont pour les plus basiques
	ligne_droite_basique(distance) où distance représente la distance en ligne droite à parcourir et distance = 1000 = 1cm.
	rotation_rel(angle) angle en degré, rotation rel, angle positif pour des rotations par la gauche.
	rotation_abs(angle) fais tourner le robot jusqu'à un angle du repert absolu qui est créer au départ du robot.

La fonction commande_vitesse(vitesse_G, vitesse_D) permet de faire tourner les moteurs à une vitesse float, avec une vitesse minimale, maximale. 
