# Raquettemusicale
Raquette connectée et sonification de geste sportif


Le code que j'ai partagé est celui de la carte esp32 s3.  Il y a deux tâches : mesure adc très rapide pour captation précise des informations (toutes les 0.8ms) et de mesure accéléromètre (toutes les 3 ms) + envoi udp wifi si un impact est détecté.

Dans la mesure adc disque piézo il y a la captation d'impact. Soient un modèle cnn est inséré (commenté car fonctionne mal) et un algorithme de détection de côté qui le remplace (non commenté). Dès qu'un impact est détecté je prends une photo de 4 fois 0.8ms, soit 3.2ms de signal piézo qui est ensuite passé dans le modèle cnn/algo.

Le réseau de neurone, à ce stade du stage, pour traiter les mesures d'accéléromètre est toujours sur python et n'est pas mis sur la carte.
