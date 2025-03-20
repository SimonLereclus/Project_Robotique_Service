# CE PROJET EN RESUME
### Fait par Simon LERECLUS et Florian BEAUVAIS

Le projet à besoin d'une installation ROS Jazzy fonctionelle

# INSTALL INSTRUCTIONS

créee un nouveau workspace ros2_ws si non existant.

`cd ros2_ws/src
git clone git@github.com:SimonLereclus/Project_Robotique_Service.git`

Le dossier devrais se trouvé à ros2_ws/src/tiago_pick_and_place après le clone

Déplace le fichier `pick_and_place.world` dans `~/ros2_ws/src/br2_gazebo_worlds/worlds`

`cd ~/ros2_ws
colcon build
source ~/.bashrc`

# LAUNCH INSTRUCTIONS

## ETAPE 1

`ros2 launch tiago_gazebo tiago_gazebo.launch.py is_public_sim:=True world_name:=pick_and_place`

## ETAPE 2

`ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/key_vel`
### Pour pouvoir se déplacé vers le cube manuscrit

## ETAPE 3

`ros2 launch tiago_pick_and_place plan2.launch.py use_sim_time:=True`

# REFERENCES ET BIBLIOGRAPHIE

le projet et basé sur les sources suivantes :

ROS JAZZY : https://gitlab.com/f2m2robserv/jazzy-ros-ynov/

lenet : https://gitlab.com/f2m2robserv/lenet#lenet-5-in-9-lines-of-code-using-keras

Générer par chat GPT : fonction `detect_and_draw_square` 

