---
title: Setup do turtlebot
sidebar_position: 1
sidebar_class_name: autoestudo
slug: /setupturtle
---

# Setup do Turtlebot3

:::warning ALERTA

Esse autoestudo ainda está em uma versão antiga.

Ele **VAI** ser atualizado.

Pode ler à vontade, pois o conteúdo ainda está correto. No entanto, vai
melhorar consideravelmente e também **vai mudar o sistema operacional sugerido
(troca do server para o ubuntu normal)**

É isso, tchau! =*

:::

## 1. Instalando o sistema operacional no raspberry pi

Antes de mais nada, baixe e instale o [raspberry pi
imager](https://www.raspberrypi.com/software/)

Após instalado, abra o software e comece a configurar o sistema operacional a
ser inserido no cartão SD da raspberry. O primeiro passo é definir o sistema
operacional. Para isso, clique no botão destacado na imagem abaixo:
![rpi1](/img/rpi1.png)

Agora precisamos escolher o sistema operacional. Selecione a opção "Other
general-purpose OS"

![rpi2](/img/rpi2.png)

Entre os sistemas operacionais que aparecem, selecione o Ubuntu:

![rpi3](/img/rpi3.png)

Selecione a versão "Ubuntu Server 22.04 LTS (64-bit)":

![rpi4](/img/rpi4.png)

A seguir, precisamos definir qual cartão SD será utilizado. Para isso, clique
no botão "Choose storage":

![rpi5](/img/rpi5.png)

Na nova tela, selecione o cartão SD que quer utilizar:

![rpi6](/img/rpi6.png)

Antes de gravarmos o sistema operacional, vamos definir algumas configurações
necessárias para uma configuração do tipo headless (instalação sem monitor e
teclado para a raspberry). Para isso, clique no ícone de engrenagem:

![rpi7](/img/rpi7.png)

Aqui você deve:
1. Configurar o hostname do dispositivo (evite o padrão raspberry ou
   raspberrypi, pois a nossa rede tem muitos dispositivos e é possível que haja
   outros dispositivos com o mesmo hostname se mantivermos o padrão)
2. Ativar o SSH
3. Configurar o usuário e a senha (evite o padrão aqui também)
4. Configure o SSID e senha para acesso ao Wifi
![rpi8](/img/rpi8.png)

Pronto! Agora podemos clicar em "Write" e aguardar até que o processo esteja
concluído.
![rpi9](/img/rpi9.png)

## 2. Conectando-se ao raspberry pi

Para conectar-se ao raspberry pi, há duas alternativas:

1. Conectar-se utilizando mouse, teclado e monitor; e
2. Conectar-se utilizando ssh para uma solução headless.

Neste tutorial, vamos focar na solução headless. Para utilizar o raspberry pi
por ssh, será necessário conectar-se a ele utilizando o seu IP local ou o seu
hostname. Abra um terminal e rode:

```bash
ssh <usuário>@<hostname-pi>
```

Onde `<usuário>` é o nome do usuário configurado na raspberry e `<hostname-pi>`
é o nome do hostname configurado. Caso esteja tentando executar esse comando em
um sistema Linux, certifique-se de que o `avahi-daemon` está instalado e
rodando (impossível em WSL) e execute uma versão modificada do comando acima:

```bash
ssh <usuário>@<hostname-pi>.local
```

Se tudo der certo, você deve ver uma mensagem perguntando se você deseja
adicionar o hostname à sua lista de hosts conhecidos. A partir daí, basta
digitar sua senha e utilizar o terminal do raspberry.

### 2.1. Alternativa: configurando SSH por USB

Para os casos em que a rede local utilizada não possui funcionalidade de DNS,
existe uma alternativa que envolve configurar a porta USB do raspberry como
porta Ethernet e realizar a conexão por ssh utilizando um cabo USB. Para isso,
siga [esse tutorial](https://desertbot.io/blog/ssh-into-pi-zero-over-usb)


## 3. Instalando o ROS Humble

Antes de mais nada, garanta que o sistema operacional está com todos os seus
pacotes atualizados e com as definições do banco de dados do apt mais novas.
Para garantir essas características, rode:

```bash
sudo apt update && sudo apt upgrade
```

Se você receber uma mensagem de que o pacote `dpkg` não foi atualizado com
sucesso, rode:

```bash
sudo apt-get --with-new-pkgs upgrade dpkg
```

Você vai precisar reiniciar o raspberry antes de continuar.

Siga o tutorial disponível
[aqui](https://github.com/rmnicola/m6-ec-encontro1#instalando-o-ros2-humble)

## 4. Instalando os pacotes do Turtlebot3

Para instalar os pacotes do turtlebot3, basta rodar o seguinte comando:

```bash
sudo apt install ros-humble-turtlebot3*
```

## 5. Compilando o pacote do LIDAR LDS-02

Vamos precisar criar um workspace ROS para compilar o pacote necessário para
utilizar o LIDAS LDS-02. Rode: 

```bash
mkdir -p ~/turtlebot3_ws/src && cd ~/turtlebot3_ws/src
```

Agora vamos clonar o repositório do firmware do LIDAR com:

```bash
git clone -b ros2-devel https://github.com/ROBOTIS-GIT/ld08_driver.git
```

Voltamos para a pasta do workspace com:

```bash
cd ~/turtlebot3_ws/
```

E finalmente vamos compilar o pacote com:

```bash
colcon build --symlink-install
```

Após compilar o pacote, configuramos o nosso `bashrc` para dar `source` no
arquivo de instalação do pacote com:

```bash
echo 'source ~/turtlebot3_ws/install/setup.bash' >> ~/.bashrc
```

Também precisamos especificar o modelo do LIDAR em uma variável de ambiente.
Rode:

```bash
echo 'export LDS_MODEL=LDS-02' >> ~/.bashrc
```

Lembre-se que para poder utilizar essas mudanças sem reiniciar o terminal ainda
precisa rodar:

```bash
source ~/.bashrc
```

## 6. Setup do OpenCR

Antes de mais nada, precisamos atualizar as configurações para a porta USB do
Raspberry. Para isso, vamos rodar:

```bash
sudo cp `ros2 pkg prefix turtlebot3_bringup`/share/turtlebot3_bringup/script/99-turtlebot3-cdc.rules /etc/udev/rules.d/
```

Seguido por:

```bash
sudo udevadm control --reload-rules
```

E por fim:

```bash
sudo udevadm trigger
```

A seguir, precisamos adicionar a arquitetura `armhf` às definições do `dpkg`.
Rode:

```bash
sudo dpkg --add-architecture armhf
```

Para que essa alteração entre em efeito, precisamos rodar:

```bash
sudo apt update
```

Agora podemos instalar a biblioteca `libc6` para a arquitetura `armhf` com:

```bash
sudo apt install libc6:armhf
```

Conecte a OpenCR no Raspberry utilizando a porta USB e verifique em que porta
ela está utilizando:
 
```bash
ls /dev/ttyACM*
```

O mais provável é que ele esteja conectado na porta `/dev/ttyACM0`, então
seguiremos com os comandos para essa porta. Precisamos agora adicionar algumas
variáveis de ambiente para compilar e fazer upload do firmware. Rode:

```bash
echo 'export OPENCR_PORT=/dev/ttyACM0' >> ~/.bashrc
```

E: 

```bash
echo 'export OPENCR_MODEL=burger' >> ~/.bashrc
```

Agora vamos baixar o firmware do repositório do github com:

```bash
wget https://github.com/ROBOTIS-GIT/OpenCR-Binaries/raw/master/turtlebot3/ROS2/latest/opencr_update.tar.bz2
```

Vamos descompactar a pasta com:

```bash
tar -xvf ./opencr_update.tar.bz2
```

Finalmente estamos prontos para fazer o upload do firmware. Para isso,
precisamos entrar na pasta que acabamos de descompactar:

```bash
cd ~/opencr_update
```

E o comando final é para fazer o upload do firmware:

```bash
./update.sh $OPENCR_PORT $OPENCR_MODEL.opencr
```

## 7. Comandando o Turtlebot3

Para comandar o turtlebot, primeiro precisamos setar a variável de ambiente que
define qual o modelo do robô utilizado:

```bash
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc && source ~/.bashrc
```

Em um terminal, rode o comando para iniciar a operação do pacote do turtlebot3:

```bash
ros2 launch turtlebot3_bringup robot.launch.py
```

Em outro terminal, rode o pacote teleop:

```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

### 7.1. Por ssh

Para comandar o robô por ssh, basta conectar-se utilizando o comando:

```bash
ssh <usuário>@<hostname>
```

E, a partir daí, seguir os passos da seção anterior.

### 7.2. Por rede

Para comandar o robô por rede, é necessário definir o mesmo `ROS_DOMAIN_ID`
para todos os computadores envolvidos. Essa variável de ambiente define quais
portas `UDP` o ROS utilizará para sua comunicação em rede. Se os dois
computadores se encontram na mesma rede e estão configurados com o mesmo
`ROS_DOMAIN_ID`, e comunicação entre os seus nós é automática.

Para setar a variável de ambiente, use:

```bash
echo 'export ROS_DOMAIN_ID=<Valor-numérico-de-0-a-232> #TURTLEBOT3' >> ~/.bashrc
```

Lembre-se de dar `source` no `bashrc`:

```bash
source ~/.bashrc
```
