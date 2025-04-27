# Introduction
Essai pour un programme STM32 avec une architecture C++.\
Inclus FreeRtos, un driver USB et UART avec un envoi géré par queue.

## Démarrage
Une fois téléchargé le programme manque les librairies de base pour compiler.\
Il faut ouvrir le fichier .ioc puis cliquer sur "Project->Generate Code".\
Cela va générer les fichiers de librairies.

!!!Warning Il ne faut SURTOUT PAS toucher aux fichiers dans les dossiers des librairies Drivers, Middlewares et USB_DEVICE!!! Si c'est le cas alors merci de les supprimer du .gitignore

## Point de départ
Comme nous utilisons CubeMX pour générer les librairies standards, le point de départ est le fichier Core/Src/main.c\
Comme le projet a pour but d'être codé en C++ alors le vrai point de départ du programme a été déplacé dans le fichier Client/Sketch.cpp

## Permettre la compilation en C++
STM32 propose de compiler en C++ cependant il ne propose pas d'architecture C++ de base.\
Pour programmer en C++ je vous conseille suivre ce [tutoriel youtube](https://www.youtube.com/watch?v=9syJpWDqj88).\
Dans la logique il faut :
- Vérifier que le projet est bien en C++\
    clic droit sur le nom du projet, en bas de la liste si le choix "Convert to C" est noté c'est que c'est bon
- créer un nouveau dossier avec le code C++\
    clic droit sur le nom du projet, New->Source Folder
- Créer un fichier Sketch.cpp et y inclure main.h

A partir de là il faut linker les fonctions du main.c vers Sketch.cpp\
Ajouter une fonction setup() avant la boucle infini puis une fonction loop() dans la boucle infini.\
Si on utilise l'architecture freertos alors il faut rajouter une fonction par task créé.\
Le prototype de toutes les fonctions créés doit être mit dans le main.h

!!!Warning Il faut bien mettre tout le code ajouté dans les fichiers main.c et le main.h entre les balises BEGIN et END, sinon à toute modification dans CubeMX vos modifications seront supprimés!

Options recommandées:\
Mettre à jour les propriétés du projet, click droit sur le nom du projet->Propriété
- Dans C/C++ Build->Settings
    - Mettre à jour la version de compilation\
        - MCU/MPU GCC Compiler->General\
            Mettre le dernier GNU (pour moi c'est GNU18)
        - MCU/MPU G++ Compiler->General\
            Mettre le dernier GNU (pour moi c'est GNU++20)
    - Modifier le niveau d'optimisation\
        - MCU/MPU GCC Compiler->Optimization\
            Optimize for size (-Os)
        - MCU/MPU G++ Compiler->Optimization\
            Optimize for size (-Os)

