# Nesta pasta contém os arquivos para o desafio do seguidor.

## Para fazer o seguidor seguir o mestre, execute os seguintes comandos:

- 1ª > roscore
- 2ª > roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch 
- 3ª > rosrun turtlebot3_teleop turtlebot3_teleop_key (inicia topico que envia comandos de velocidade usando as teclas A,W,S,D)
- 4ª > rosrun meu_pacote swarm_control_tb3.py ( Se inscreve no topico de velocidade e publica no mestre as informações das teclas)
- 5ª > rosrun meu_pacote seguidor.py (inicia o topico que publica informação de velocidade no seguidor e com base na posição do mestre, corrige a direção do seguidor).
