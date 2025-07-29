# MiniProject1
Pasos para iniciarlo.

source /opt/ros/humble/setup.bash

git clone https://github.com/TnTivanTnT/MiniProject1.git
cd WS
source install/setup.bash

1º terminal, para ir actualizando cambios:
colcon build

2º terminal, iniciar turtlesim:
ros2 run turtlesim turtlesim_node

3º terminal, ejecuta el programa python:
ros2 run Escritor NodoEscritor
