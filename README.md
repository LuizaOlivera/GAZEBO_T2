GAZEBO_T2 - Trabalho com TurtleBot3 no ROS 2

Autora: Luiza Olivera das Neves

Video Youtube:
https://youtu.be/5FV1mmwaqmU 

Resumo:
Este projeto tem como objetivo simular um robô TurtleBot3 se deslocando até pontos específicos dentro de um ambiente controlado. A simulação foi feita com ROS 2 e Gazebo, utilizando comandos de movimentação baseados em posições previamente definidas.

Sobre o funcionamento:
O robô parte sempre de uma posição inicial e se move até um ponto de interesse no mapa. Após chegar ao destino, ele retorna ao ponto de origem antes de iniciar o próximo trajeto. Esse processo se repete quatro vezes, com caminhos diferentes para cada missão.

Os movimentos são baseados em cálculos simples de distância e orientação, considerando a posição atual (odometria). Todos os caminhos foram definidos manualmente, e o controle foi feito por meio do tópico /cmd_vel.

Como utilizar:
- Copie os arquivos para o seu workspace ROS 2
- Compile com colcon build
- Rode o Gazebo com o cenário desejado
- Execute o script de controle

Observações:
Este projeto não utiliza sensores para evitar obstáculos. As rotas foram planejadas de forma a contornar os blocos do cenário. Como limitação, se o ambiente mudar ou surgir um novo obstáculo, o robô não consegue se adaptar.

Comentários finais:
A proposta foi concluída com sucesso. O robô completou os quatro trajetos previstos, retornando corretamente após cada missão.

