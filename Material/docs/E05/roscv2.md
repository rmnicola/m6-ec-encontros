---
title: Rosbridge
sidebar_position: 4
sidebar_class_name: autoestudo
slug: /rosbridge
---

# Rosbridge

:::warning Em construção

Esse material atual é um esboço rápido e será revisado. Por enquanto, estude as
implementações dadas como exemplo.

:::

## 1. O que é o rosbridge

O [rosbridge](https://github.com/RobotWebTools/rosbridge_suite) é uma interface
de JSON para ROS. Com ele, é possível usar um servidor de websockets que mapeia
toda a sua rede ROS local para ficar disponível em um endpoint com uma API em
json. Para instalar o ros bridge, rode:

```bash
sudo apt install ros-humble-rosbridge-suite
```

Para rodar o servidor websocket, rode:

```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

## 2. Utilizando o rosbridge para interfaces web

Para interagir com o servidor, basta criar um arquivo html simples com um
script inline que invoca a `roslibjs`, que é uma API em Javascript que se
comunica com os websockets do servidor.

:::tip Dica

Além da `roslibjs`, existe também a `jrosbridge`, para Java; a `roslibpy`, para
Python; e a `roslibrust`, para Rust.

:::

A seguir, crie um arquivo e coloque o conteúdo abaixo nele.

```html showLineNumbers title="bridge_test.html"
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8" />

<script type="text/javascript" src="https://cdn.jsdelivr.net/npm/eventemitter2@6.4.9/lib/eventemitter2.min.js"></script>
<script type="text/javascript" src="https://cdn.jsdelivr.net/npm/roslib@1/build/roslib.min.js"></script>

<script type="text/javascript" type="text/javascript">
  // Connecting to ROS
  // -----------------

  var ros = new ROSLIB.Ros({
    url : 'ws://localhost:9090'
  });

  ros.on('connection', function() {
    console.log('Connected to websocket server.');
  });

  ros.on('error', function(error) {
    console.log('Error connecting to websocket server: ', error);
  });

  ros.on('close', function() {
    console.log('Connection to websocket server closed.');
  });

  // Publishing a Topic
  // ------------------

  var cmdVel = new ROSLIB.Topic({
    ros : ros,
    name : '/turtle1/cmd_vel',
    messageType : 'geometry_msgs/Twist'
  });

  var twist = new ROSLIB.Message({
    linear : {
      x : 0.1,
      y : 0.2,
      z : 0.3
    },
    angular : {
      x : -0.1,
      y : -0.2,
      z : -0.3
    }
  });
  cmdVel.publish(twist);

  // Subscribing to a Topic
  // ----------------------

  var listener = new ROSLIB.Topic({
    ros : ros,
    name : '/chatter',
    messageType : 'std_msgs/String'
  });

  listener.subscribe(function(message) {
    console.log('Received message on ' + listener.name + ': ' + message.data);
    listener.unsubscribe();
  });
</script>
</head>

<body>
  <h1>Simple roslib Example</h1>
  <p>Check your Web Console for output.</p>
</body>
</html>
```

Agora, rode em outro terminal um comando para enviar mensagens no tópico
`/chatter`:

```bash
ros2 run demo_nodes_cpp talker
```

:::tip Dica

O código html acima inclui em seu js inline a configuração de um publisher para
o tópico `/turtle1/cmd_vel`. Fica como desafio você entender com o que ele está
interagindo e testar a implementação.

:::

## 3. Transmititndo imagens com o ROS

De modo similar, para transmitir imagens em tempo real utilizando o ROS é muito
simples usando o rosbridge. Segue o código html com JS inline para receber o
vídeo em tempo real.

```html showLineNumbers title="imagens.html"
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8" />

<!-- ROS libraries -->
<script type="text/javascript" src="https://cdn.jsdelivr.net/npm/eventemitter2@6.4.9/lib/eventemitter2.min.js"></script>
<script type="text/javascript" src="https://cdn.jsdelivr.net/npm/roslib@1/build/roslib.min.js"></script>

<script type="text/javascript">
  var ros = new ROSLIB.Ros({
    url : 'ws://localhost:9090'
  });

  ros.on('connection', function() {
    console.log('Connected to websocket server.');
  });

  ros.on('error', function(error) {
    console.log('Error connecting to websocket server: ', error);
  });

  ros.on('close', function() {
    console.log('Connection to websocket server closed.');
  });

  // Topic to receive video frames
  var videoTopic = new ROSLIB.Topic({
    ros : ros,
    name : '/video_frames',
    messageType : 'sensor_msgs/CompressedImage'
  });

  // Function to handle incoming video frames
  videoTopic.subscribe(function(message) {
    var img = document.getElementById('videoStream');
    img.src = 'data:image/jpeg;base64,' + message.data;
  });

  window.onload = function() {
    // Subscribe to video frames once
    videoTopic.subscribe();
  };
</script>
</head>
<body>
  <h1>Real-time Video Stream from ROS2 Topic</h1>
  <img id="videoStream" alt="Video Stream" style="width: 640px; height: 480px;" />
</body>
</html>
```

Para testar esse código, precisaremos criar um publisher em Python capaz de ler
imagens da webcam utilizando o openCV e enviá-las em um tópico ROS.

```python showLineNumbers title="sender.py"
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2

class WebcamPublisher(Node):
    def __init__(self):
        super().__init__('webcam_publisher')
        self.publisher_ = self.create_publisher(CompressedImage, '/video_frames', 10)
        self.timer = self.create_timer(0.02, self.timer_callback)  # Publish every 0.1 seconds (10 Hz)
        self.cap = cv2.VideoCapture(0)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            # Encode frame as JPEG
            _, buffer = cv2.imencode('.jpg', frame)
            msg = CompressedImage()
            msg.format = "jpeg"
            msg.data = buffer.tobytes()
            self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    webcam_publisher = WebcamPublisher()
    rclpy.spin(webcam_publisher)
    webcam_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Para rodar o código, use:

```bash
python3 sender.py
```

