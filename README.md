# Raquette musicale


# Chaîne d'acquisition 

Capteur piezo (onde de flexion de la raquette)----> détection impact, essai de CNN pour retirer informations (rotation, 
                                                      impact sans rotation), algorithme pour détecter quel côté est frappé
IMU MPU6050 6dof                               ----> caractérisation topspin, tape, amorti


# CODE pour la carte esp32 s3 embarquée

Le code que j'ai partagé est celui de la carte esp32 s3.  Il y a deux tâches : mesure adc très rapide pour captation précise des informations (toutes les 0.8ms) et de mesure accéléromètre (toutes les 3 ms) + envoi udp wifi si un impact est détecté.

Dans la mesure adc disque piézo il y a la captation d'impact. Soient un modèle cnn est inséré (commenté car fonctionne mal) et un algorithme de détection de côté qui le remplace (non commenté). Dès qu'un impact est détecté je prends une photo de 4 fois 0.8ms, soit 3.2ms de signal piézo qui est ensuite passé dans le modèle cnn/algo.

Le réseau de neurone, à ce stade du stage, pour traiter les mesures d'accéléromètre est toujours sur python et n'est pas mis sur la carte.

# Code python
Ce code n'est pas partagé. J'ai fait de nombreux traitements statistiques et plusieurs versions de réception wifi,  entrainements pytorch faciles à reproduire.  

# Video démo 

Une video de démo est présentée dans ce github

# Présentation Slides 

Une présentation est disponible pour avoir un aperçu de la raquette et des résultats.
