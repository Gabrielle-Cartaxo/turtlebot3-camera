# Turtlebot teleoperado pt2

### Passo 1: Instalação de Dependências

Certifique-se de ter instalado as seguintes dependências:

- **ROS2**: Certifique-se de ter o ROS2 instalado corretamente em seu sistema. Você pode seguir as instruções de instalação do [site oficial do ROS2](https://index.ros.org/doc/ros2/Installation/).

- **Python Packages**: Se você está usando Python para desenvolver o nó publicador, você pode precisar instalar pacotes como `cv2`.

```bash
pip install opencv-python
```

- **Webots ROS2 Package**: Para testar o funcionamento desse módulo, estou usando o Webots para simulação. Se quiser replicar, também instale o pacote `webots_ros2_turtlebot` de acordo com as instruções fornecidas na documentação.

### Passo 2: Iniciar o Rosbridge WebSocket Server

Abra um terminal e inicie o Rosbridge WebSocket Server:

```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.py
```

### Passo 3: Iniciar o Simulador Webots com o Turtlebot

Abra um segundo terminal e inicie o simulador Webots com o Turtlebot:

```bash
ros2 launch webots_ros2_turtlebot robot_launch.py
```

### Passo 4: Executar o Publisher Python

Abra um terceiro terminal, navegue até o diretório do seu pacote ROS2 e execute o nó publicador Python:

```bash
publisher.py
```

### Passo 5: Abrir o Arquivo HTML no Navegador

Abra um navegador da web e carregue o arquivo `index.html` que está na pasta src.

Com esses passos, você deverá ser capaz de executar o projeto e controlar o robô simulado através da interface HTML, garantindo que todas as dependências estejam instaladas corretamente e que o ambiente ROS2 esteja configurado e funcionando adequadamente.
