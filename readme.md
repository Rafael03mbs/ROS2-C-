ROS 2 Motion Planning with MoveIt 2

Descrição Geral

Este repositório reúne experiências e implementações de robótica em ROS 2 e C++, com foco na configuração de manipuladores robóticos, planeamento de movimentos e utilização do MoveIt 2.

O projeto explora a integração entre a descrição do robô, a configuração do ambiente de planeamento e a execução de tarefas de movimento, incluindo exemplos com o MoveIt Task Constructor (MTC).

Funcionalidades Principais

* MoveIt 2: configuração e utilização de ferramentas de planeamento de movimentos para manipuladores robóticos.
* MoveIt Task Constructor: exploração de planeamento de tarefas através de etapas modulares.
* Robot Description: descrição e organização dos componentes do robô através de pacotes ROS 2.
* Motion Planning Configuration: configuração dos componentes necessários à integração do robô com o MoveIt 2.
* ROS 2 Development: organização de pacotes e utilização de ficheiros de lançamento para executar os exemplos.

Estrutura do Projeto

Diretório / Ficheiro	Descrição
moveit_lab_1/	Exemplos e experiências de configuração e utilização do MoveIt.
mtc_tutorial/	Implementação de um exemplo baseado no MoveIt Task Constructor.
so_arm_description/	Pacote associado à descrição do manipulador robótico.
so_arm_moveit_config/	Configuração do MoveIt para o manipulador.
correr.txt	Comando de lançamento do exemplo MTC.

Tecnologias Utilizadas

* C++
* ROS 2
* MoveIt 2
* MoveIt Task Constructor
* URDF / Robot Description
* RViz 2
* colcon

Pré-requisitos

* Ubuntu compatível com a distribuição ROS 2 utilizada.
* ROS 2 instalado e configurado.
* MoveIt 2 e dependências correspondentes aos pacotes do projeto.
* colcon e ferramentas de compilação.

Compilação e Execução

1. Criar o workspace

mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/Rafael03mbs/ROS2-C-.git
cd ..

2. Instalar dependências

source /opt/ros/$ROS_DISTRO/setup.bash
rosdep install --from-paths src --ignore-src -r -y

3. Compilar o projeto

colcon build

4. Carregar o workspace

source install/setup.bash

5. Executar o exemplo MTC

ros2 launch mtc_tutorial mtc_demo.launch.py

O comando inicia o exemplo de planeamento de tarefas definido no pacote mtc_tutorial.

Nota: a distribuição ROS 2, as dependências do MoveIt 2 e os restantes requisitos devem corresponder à configuração utilizada no projeto.

Objetivo

Este projeto serve como ambiente de experimentação e aprendizagem no desenvolvimento de aplicações robóticas com ROS 2 e C++, explorando a configuração de manipuladores e as ferramentas de planeamento de movimentos disponibilizadas pelo MoveIt 2.

⸻

Autor: Rafael Batista da Silva