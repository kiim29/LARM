# LARM-challenge

### Descriptif
Dépot de notre projet en UV Logiciels et Applications pour la Robotique Mobile (LARM)
Ce package dépend de mb-tbot6. Il faut s'assurer qu'il est correctememt installé sur la machine.
Chalenge 1 : le robot se déplace dans un espace clos en évitant les obstacles. La visualisation sur rviz2 montre la caméra et le laser.

### Auteurs :
Kim Luxembourger
Nathan Simon

### Packages :
pkg_data

### Launch :
Pour lancer la simulation :
ros2 launch grp_data simulation.launch.py
Pour lancer le robot réel :
ros2 launch grp_data tbot.launch.py
Pour lancer la visualisation :
ros2 launch grp_data visualize.launch.py