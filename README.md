## LARM-challenge-3

### Descriptif :
Dépot de notre projet en UV Logiciels et Applications pour la Robotique Mobile (LARM)
Ce package dépend de mb-tbot6. Il faut s'assurer qu'il est correctememt installé sur la machine.  
  
Challenge 3 : Le robot se déplace dans un espace clos en évitant les obstacles. Il fait une carte de son environnement par SLAM. Il détecte des bouteilles de Nuka Cola et de Nuka Cherry quand il passe devant. Il ralentit et se met face à une bouteille détectée pour un temps afin de s'assurer que c'est bien une bouteille. Il met un message dans le topic `/detection` en précisant le type de la bouteille (Nuka Cola ou Nuka Cherry) et sa position estimée. Une fois qu'il a détecté pour sûr une bouteille, le robot pose un marqueur sur la carte pour repérer son emplacement.  
Reste à faire : Lorsqu'il passe devant une bouteille qu'il a détecté auparavant, il la reconnait d'après les positions enregistrées en mémoire. La visualisation sur Rviz2 devrait aussi permettre de publier des `/goal_pose` pour envoyer le robot dans une position précise.  

### Auteurs :
Kim Luxembourger et Nathan Simon (machine data)

### Packages :
grp_data

### Dépendances :
pkg_tbot  
  
Pour installer, en résumé :  
Créer un directory (`mkdir`) qui deviendra votre workspace et cloner notre package dedans (`clone`) :  
`mkdir my_ws`  
`cd my_ws`  
Suivre le tuto https://ceri-num.gitbook.io/uv-larm/tuto.-01-kick-off/ros-basics pour installer le pkg-tbot (mb6-tbot) nécessaire.  
Puis cloner notre package dans votre workspace :  
`git clone https://github.com/kiim29/LARM-kim-nathan`  

### Puis dans votre workspace, à chaque modification de code :
`colcon build`  pour compiler les packages  
`source install/setup.bash` pour sourcer 

### Launch :
Pour lancer la simulation :  
`ros2 launch grp_data chall3_simulation.launch.py`  
Elle démarre gazebo pour simuler le robot et rviz2 pour une visualisation partielle. On peut voir le robot éviter les obstacles dans Gazebo et faire une carte de l'arène virtuelle dans rviz2.  
  
Pour lancer le robot réel :  
`ros2 launch grp_data chall3_tbot.launch.py`  
On peut voir le robot se déplacer et éviter les obstacles dans l'arène réelle. Ce launch démarre le robot, le laser scan, la caméra, la détection de bouteilles, etc... Pour la détection des bouteilles, nous avons choisi de fonctionner avec trois seuils de couleurs pour chaque bouteille (noir, rouge et blanc pour le Nuka Cola; orange, rouge et blanc pour le Nuka Cherry) correspondant aux couleurs de la bouteille, de l'étiquette et du nom sur l'étiquette. Cela nous donne 3 masques que l'on peut dilater et faire se recouper pour trouver une étiquette, et donc une bouteille.  
  
Pour lancer la visualisation :  
`ros2 launch grp_data chall3_visualize.launch.py`  
La visualisation fonctionne pour le robot réel. Elle permet un visuel sur les relevés laser et sur la caméra du robot. L'image signale les objets repérés par le robot (sans les derniers traitements pour éviter les faux positifs) en les indiquant avec une marque rouge. Elle ouvre aussi un nouveau terminal pour la commande teleop qui permet le contrôle à distance.  
Elle affiche aussi la carte `/map` avec les obstacles et éventuellement les marqueurs de position des bouteilles.  
  
Pour repérer les bouteilles :  
`ros2 topic echo detection`  
Cette commande permet l'écoute du topic `/detection`. Elle signale lorsqu'une bouteille est repérée par le robot avec sa localisation et sa distance approximative.  





## LARM-challenge-2

### Descriptif :
Dépot de notre projet en UV Logiciels et Applications pour la Robotique Mobile (LARM)
Ce package dépend de mb-tbot6. Il faut s'assurer qu'il est correctememt installé sur la machine.  
  
Challenge 2 : le robot se déplace dans un espace clos en évitant les obstacles. Il fait une carte de son environnement par un SLAM. Il détecte des bouteilles de Nuka Cola quand il passe devant.

### Auteurs :
Kim Luxembourger et Nathan Simon (machine data)

### Packages :
grp_data

### Dependances :
pkg_tbot  
  
En résumé : Créer un directory (`mkdir`) qui deviendra votre workspace et cloner notre package dedans (`clone`) :  
`mkdir my_ws`  
`cd my_ws`  
Suivre le tuto https://ceri-num.gitbook.io/uv-larm/tuto.-01-kick-off/ros-basics pour installer le pkg-tbot (mb6-tbot) nécessaire.  
Puis cloner notre package dans votre workspace :  
`git clone https://github.com/kiim29/LARM-kim-nathan`  

### Puis dans votre workspace :
`colcon build`  pour compiler les packages  
`source install/setup.bash` pour sourcer

### Launch :
Pour lancer la simulation :  
`ros2 launch grp_data chall2_simulation.launch.py`  
Elle démarre gazebo pour simuler le robot et rviz2 pour une visualisation partielle. On peut voir le robot éviter les obstacles dans Gazebo et faire une carte de l'arène virtuelle dans rviz2.  
  
Pour lancer le robot réel :  
`ros2 launch grp_data chall2_tbot.launch.py`  
On peut voir le robot se déplacer et éviter les obstacles dans l'arène réelle.  
  
Pour lancer la visualisation :  
`ros2 launch grp_data chall2_visualize.launch.py`  
La visualisation fonctionne pour le robot réel. Elle permet un visuel sur les relevés laser et sur la caméra du robot. L'image signale les objets repérés par le robot en les indiquant avec une marque rouge. Elle ouvre aussi un nouveau terminal pour la commande teleop qui permet le contrôle à distance.  
  
Pour repérer les bouteilles :  
`ros2 topic echo detection`  
Cette commande permet l'écoute du topic `/detection`. Elle signale lorsqu'une bouteille est repérée par le robot avec sa localisation et sa distance approximative.  
  
  

## LARM-challenge-1

### Descriptif :
Dépot de notre projet en UV Logiciels et Applications pour la Robotique Mobile (LARM)
Ce package dépend de mb-tbot6. Il faut s'assurer qu'il est correctememt installé sur la machine.
Challenge 1 : le robot se déplace dans un espace clos en évitant les obstacles. La visualisation sur rviz2 montre la caméra et le laser.

### Auteurs :
Kim Luxembourger (machine data) et Nathan Simon

### Packages :
grp_data

### Dependances :
pkg_tbot  
  
En résumé : Créer un directory (`mkdir`) qui deviendra votre workspace et cloner notre package dedans (`clone`) :  
`mkdir my_ws`  
`cd my_ws`  
Suivre le tuto https://ceri-num.gitbook.io/uv-larm/tuto.-01-kick-off/ros-basics pour installer le pkg-tbot (mb6-tbot) nécessaire.  
Puis cloner notre package dans votre workspace :  
`git clone https://github.com/kiim29/LARM-kim-nathan`  

### Puis dans votre workspace :
`colcon build`  pour compiler les packages  
`source install/setup.bash`

### Launch :
Pour lancer la simulation :  
`ros2 launch grp_data simulation.launch.py`  
Elle démarre gazebo pour simuler le robot et rviz2 pour une visualisation partielle.  
  
Pour lancer le robot réel :  
`ros2 launch grp_data tbot.launch.py`  
  
Pour lancer la visualisation :  
`ros2 launch grp_data visualize.launch.py`  
La visualisation fonctionne pour la simulation ou pour le robot réel. Elle permet un visuel sur les relevés laser et sur la caméra du robot. Elle ouvre aussi un nouveau terminal pour la commande teleop qui permet le contrôle à distance.  