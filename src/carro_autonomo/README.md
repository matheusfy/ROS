# Inicialização com apenas um robo (Tarefa incompleta)

## Primeiro passo: fazer um robo (mestre) desviar dos objetos colocados no ambiente.
## Segundo passo: Adicionar o seguidor e fazer desviar dos objetos do ambiente enquanto vai em direção do mestre.

- roscore
- roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch (inicializa mestre em um mundo vazio)

-Script/
 |
 |-> turtle_pose.py (arquivo que faz o robo ir ao ponto definido pelo usuário)
 |-> lidar.py (Arquivo que irá conter as funções para fazer o mestre chegar ate o ponto definido pelo usuario enquanto desvia dos objetos)




## Atividade  odom e icp:
Desafio 2 comandos para funcionar atividade:
- roscore
- roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch (Deve ser inserido alguns objetos no mundo)
- rosrun turtlebot3_teleop turtlebot3_teleop_key (comando awsd para o carrinho andar e girar)
- rosrun carro_autonomo turtle_icp.py (Funciona depois que o carro já está em andamento)
