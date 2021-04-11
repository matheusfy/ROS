# Trabalhando com ROS

##Comandos:
-> gedit ~/.bashrc (para localizar o modelo exportado e alterar entre burguer, waffle e waffle_pi)

-> catkin_make (para dar um build nos arquivos novos criado dentro dos pacotes. *caso haja alguma dependência dentro dos pacotes faltando irá ocorrer erro)
---
##Estruturação dos arquivos:
src:

/meu_pacote - Contém arquivos para trabalhar com um seguidor controlando o Mestre usando as Teclas A,W,S,D

/carro_autonomo - Os arquivos para fazer o Mestre desviar de obstáculos. (Próximo passo fazer o seguidor chegar no local que em que o mestre estiver, desviando de obstaculos)

Restante dos arquivos foram fornecidos pelo professor para dar suporte nas atividades. 
