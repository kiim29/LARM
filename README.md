# LARM-challenge

### Descriptif :
Dépot de notre projet en UV Logiciels et Applications pour la Robotique Mobile (LARM)
Ce package dépend de mb-tbot6. Il faut s'assurer qu'il est correctememt installé sur la machine.
Challenge 1 : le robot se déplace dans un espace clos en évitant les obstacles. La visualisation sur rviz2 montre la caméra et le laser.

### Auteurs :
Kim Luxembourger (machine data)
Nathan Simon

### Packages :
pkg_data

### Dependances :
pkg_tbot  
  
En résumé : Créer un directory (`mkdir`) qui deviendra votre workspace et cloner notre package dedans (`clone`) :  
`mkdir my_ws`  
`cd my_ws`  
Suivre le tuto https://ceri-num.gitbook.io/uv-larm/tuto.-01-kick-off/ros-basics pour installer le pkg-tbot (mb6-tbot) nécessaire.  
`git clone https://github.com/kiim29/LARM-kim-nathan`  

### Puis dans votre workspace :
`colcon build`  pour compiler les package
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