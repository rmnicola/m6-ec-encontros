---
title: Workspaces ROS
sidebar_position: 3
sidebar_class_name: autoestudo
slug: /workspaces
---

import Admonition from '@theme/Admonition';

Quando trabalhamos com Python, aprendemos a criar pacotes utilizando o `pip`.

Quando trabalhamos em javascript, aprendemos a gerenciar pacotes utilizando o
`npm`.

Quando trabalhamos em rust, utilizamos o `cargo`. 

Com ROS não é diferente. Ele oferece os conceitos de **workspaces** e
**pacotes**. Para isso, é preciso utilizar o gerenciador de pacotes do ROS, o
`colcon`. 

## 1. Definições básicas

### 1.1. Pacote ROS

*Por que criamos pacotes?*

Essa é uma pergunta fundamental para qualquer linguagem, não só para o
desenvolvimento em ROS e essa pergunta também nos leva a um apontamento que
define **o que é um pacote** através da sua **função** em um projeto.

Projetos que envolvem código muitas vezes tem uma complexidade elevada e
envolvem o trabalho de muitas pessoas. Nós criamos pacotes para que seja
possível desacoplar essa complexidade. O que significa desacoplar a
complexidade? Significa que um desenvolvedor que precisa desenvolver um
frontend não precisa se preocupar em saber como implementar um sistema de
deteção de objetos em imagens. Significa que podemos criar unidades de código
que podem interagir entre si e diminuir a carga cognitiva necessária para criar
um sistema complexo.

Os pacotes em ROS servem justamente para isso: diminuir a carga cognitiva
necessária para criar aplicações robóticas. Que implementar um algoritmo de
localização e mapeamento simultâneo? Tem um pacote de alguém que gastou muito
tempo nesse problema que você pode usar sem precisar gastar muito tempo. Que
utilizar filtro de kalmann? Tem um pacote de alguém que gastou muito tempo
nesse problema... Quer transferir imagens em tempo real através do ROS? Tem um
pacote...

Não preciso nem dizer que espera-se, portanto, que você saiba interagir com um
pacote, criando-os e usando-os sempre que precisar.

**Instalando pacotes ROS**

Para instalar pacotes ROS, temos dois caminhos distintos:

1. Baixando o código fonte e utilizando o `colcon` para compilar o pacote; ou
2. Instalando o pacote utilizando o `apt`.

O caminho que vamos usar na maior parte das vezes é a instalação utilizando o
gerenciador de pacotes do Ubuntu, o `apt`. Para isso, você deve saber o padrão
de nomenclatura para pacotes disponíveis no `apt`:

```bash
ros-<distro-ros>-<pacote>
```

Aqui, o `distro-ros` descreve a versão do ROS que está sendo utilizada e o
`pacote` é o nome do pacote. Um exemplo é o pacote
`ros-humble-turtlebot3-messages`. Aqui temos um pacote para o `ROS 2 Humble` e
o nome do pacote é o `turtlebot3-messages`. Esse pacote está dentro do
`workspace` do `turtlebot3` e define os tipos de mensagens que são necessárias
para interagir com o robô turtlebot3.

**Criando pacotes ROS**

Para criar pacotes ROS, precisamos primeiro entender a unidade que serve para
organizar conjuntos de pacotes ROS, o `workspace`. Então, vamos ver o que é um
workspace.


### 1.2. Workspace ROS

Um workspace ROS nada mais é do que um **diretório** onde é possível encontrar
**um ou mais pacotes ROS**. A utilidade dessa organização é que com um
workspace é possível instalar vários pacotes de uma vez só e também cada
workspace conta com um script de setup similar ao do ROS em si, mas esse serve
para configurar as definições de executáveis dos pacotes contidos no workspace.

**Exemplo**

Os pacotes relacionados ao Turtlebot (o robô utilizado no módulo) estão
organizados em um workspace. Nesse workspace podemos encontrar, entre outras
coisas:

* O pacote que define as mensages básicas para interagir com o turtlebot;
* Um pacote que implementa o algoritmo de mapeamento e navegação simultânea do
  turtlebot;
* Arquivos de configuração que definem a cadeia cinemática (oi Geraldo s2) do
  robô.

Uma das principais vantagens de se ter todos os pacotes relacionados a um mesmo
projeto aglomerados em um único workspace é o fato de que é possível **compilar
todos os pacotes de um workspace de uma vez só**. Mais para frente vamos ver
como fazer isso, mas trata-se de uma vantagem muito grande para quem quer
entregar um projeto complexo que é composto por muitos pacotes e conseguir
fazer o setup do projeto em um único comando.

Beleza, mas como podemos criar um workspace? Vamos ver exatamente como se faz
isso na próxima seção, em que vamos tratar sobre o setup do seu primeiro
projeto ROS.

## 2. Criando seu primeiro projeto

### 2.1. Criando um workspace

O primeiro passo para criar o seu primeiro projeto ROS é criar um workspace.
Para criar um workspace ROS, basta criar um diretório qualquer e, dentro dele,
adicionar um subdiretório chamado `src`, que é onde vão ficar os nossos pacotes.

:::tip dica

O argumento `-p` do comando `mkdir` faz com que seja possível criar todos os
diretórios no caminho até o diretório final. No exemplo abaixo queremos criar
`meu_workspace/src`, mas o diretório `meu_workspace` ainda não existe. Com esse
argumento é possível criar os dois diretórios ao mesmo tempo.

:::

```bash
mkdir -p meu_workspace/src
```

### 2.2. Adicionando um pacote de exemplo

Ok, mas e os pacotes? Bom, como ainda não temos nenhum pacote, vamos testar
nosso workspace pegando um pacote de exemplo disponibilizado pelos criadores do
ROS.

Entre no diretório do seu workspace e no subdiretório `src`:

```bash
cd meu_workspace/src
```

E vamos clonar o repositório que guarda o pacote de exemplo com:

```bash
git clone https://github.com/ros/ros_tutorials.git -b humble 
```

É muito comum um pacote ROS ter dependências. Para garantir que temos as
dependências necessárias para nosso pacote, precisamos rodar o `rosdep`. Para
isso, é necessário garantir que você tem o `rosdep` instalado. Use:

```bash 
sudo apt install python3-rosdep
```

Após a instalação, precisamos configurar o rosdep. Para isso, usaremos dois
comandos em sequência: 

```bash 
sudo rosdep init
```

```bash
rosdep update
```

Volte para o diretório raíz do seu workspace e, logo em seguida, rode o
seguinte comando para baixar todas as dependências:

```bash
rosdep install -i --from-path src --rosdistro humble -y
```

Pronto! Esse comando já vai checar todos os seus pacotes e instalar as
depedências automaticamente. A seguir, vamos compilar o nosso pacote usando:

```bash 
colcon build
```
:::caution Aviso

Para executar o comando abaixo você precisa, antes, instalar o seguinte pacote:

```bash
sudo apt install python3-colcon-common-extensions
```

Caso tenha utilizado meu script `ros-install` com todas as opções selecionadas,
esse pacote já vai ter sido instalado.

:::

Ao rodar esse comando, nosso diretório mudou. Vamos dar uma olhada em como ele
está? Lembre-se que antes ele só tinha o diretório `src` com nosso pacote
exemplo antes.

```bash
.
├── build
│  ├── COLCON_IGNORE
│  └── turtlesim
├── install
│  ├── _local_setup_util_ps1.py
│  ├── _local_setup_util_sh.py
│  ├── COLCON_IGNORE
│  ├── local_setup.bash
│  ├── local_setup.ps1
│  ├── local_setup.sh
│  ├── local_setup.zsh
│  ├── setup.bash
│  ├── setup.ps1
│  ├── setup.sh
│  ├── setup.zsh
│  └── turtlesim
├── log
│  ├── build_2024-04-23_19-34-44
│  ├── build_2024-04-23_20-05-36
│  ├── COLCON_IGNORE
│  ├── latest -> latest_build
│  └── latest_build -> build_2024-04-23_20-05-36
└── src
   └── ros_tutorials
```

Note que surgiram as pastas `install`, `build` e `log` em seu workspace. Caso
queira subir seu ws para um repositório, não esqueça de adicionar essas pastas
ao `.gitignore`!

Para poder rodar o seu pacote, precisamos agora apenas dar `source` no script
de configuração do workspace. Fazemos isso com: 

```bash
source install/local_setup.bash
```

:::tip Dica

A diferença entre o `local_setup` e o `setup` é que o setup local foi feito
para configurar apenas os executáveis apenas do workspace local (o diretório em
que você se encontra atualmente). A versão `setup` foi feita para configurar
workspaces instalados globalmente.

:::

Pronto! Seu workspace ROS está configurado e com um pacote funcional. Mas, como
vamos testar? Você pode ter notado que o pacote compilado pelo comando `colcon
build` foi o `turtlesim`. Nós já temos esse pacote instalado, mas agora estamos
usando uma versão local. Beleza, mas como vamos saber se estamos de fato usando
a versão local ou a do sistema? Simples, vamos colocar uma mensagem especial no
`turtle_teleop_key`! =D

:::tip Dica

Note que o turtlesim é um pacote inteiramente feito em `C++`. Nós vamos usar
majoritariamente Python nesse módulo, mas se quiser ver como é por dentro um
pacote ROS em C++, a hora é agora.

:::

Para fazer isso, vamos modificar o arquivo
`meu_workspace/src/ros_tutorials/turtlesim/tutorials/teleop_turtle_key.cpp`.

O que vamos mexer? Pesquise por `Reading from keyboard` no arquivo. Você deve
ver o seguinte bloco de `puts`:

```cpp
    puts("Reading from keyboard");
    puts("---------------------------");
    puts("Use arrow keys to move the turtle.");
    puts("Use G|B|V|C|D|E|R|T keys to rotate to absolute orientations. 'F' to cancel a rotation.");
    puts("'Q' to quit.");
```

Vamos mudar para:

```cpp
    puts("Turtlesim da massa");
    puts("---------------------------");
    puts("Agora o turtlesim eh nosso!!");
    puts("Use G|B|V|C|D|E|R|T keys to rotate to absolute orientations. 'F' to cancel a rotation.");
    puts("'Q' to quit.");
```

Para conseguir rodar esse código, vamos precisar compilar o workspace
novamente, portanto rode `colcon build`.

Agora, vamos rodar nossa versão modificada com

```bash
ros2 run turtlesim turtle_teleop_key
```

Se tudo deu certo, você viu a nossa mensagem modificada.

Vamos para o próximo passo, que é a criação e um pacote nosso usando Python?

### 2.3. Criando o seu primeiro pacote

Volte para o diretório `src` do seu workspace, pois está na hora de criarmos um
pacote novo usando o comando `ros2 pkg create`. Precisamos definir duas coisas
para rodar esse comando: o nome do pacote e o tipo de pacote a ser criado (C++
ou Python)

```bash
ros2 pkg create --build-type ament_python ola_mundo
```

:::tip Curiosidade

Na transição do ROS1 para o ROS2 houve uma mudança no sistema de gerenciamento
de pacotes (de catkin mudou para colcon). O motivo disso foi para que fosse
possível criar pacotes ROS utilizando Python com mais facilidade. O efeito é
que hoje podemos escolher se vamos criar um pacote em C++ ou em Python
utilizando o `colcon`.

:::

Dessa vez, o pacote será criado sem nenhum script para ser rodado. Vamos
configurar tudo na mão. Vamos entrar no diretório criado pelo comando com:

```bash
cd ola_mundo
```

Como é a estrutura de diretórios aqui dentro?

```bash
.
├── ola_mundo
│  └── __init__.py
├── package.xml
├── resource
│  └── ola_mundo
├── setup.cfg
├── setup.py
└── test
   ├── test_copyright.py
   ├── test_flake8.py
   └── test_pep257.py
```

Percebeu algo? Sim, isso é exatamente a mesma estrutura encontrada em [módulo
Python](https://docs.python.org/3/tutorial/modules.html). Portanto, o que
precisamos fazer é adicionar nossos arquivos `.py` dentro do subdiretório
`ola_mundo` (onde tem um `__init__.py`). Vamos criar um arquivo simples?

Abra seu editor de texto favorito, crie um arquivo chamado "teste.py" e
adicione o seguinte:

```python showLineNumbers title="ola_mundo.py"
def main():
    print("Ola, mundo!")

if __name__ == "__main__":
    main()
```

Volte para a raíz do seu workspace e rode `colcon build` para compilar seu
pacote. Não se esqueça também de dar `source` no `local_setup`, pois dessa vez
temos um pacote novo sendo definido.

Beleza, mas como podemos testar nosso pacote? Assim como fizemos com o
`turtle_teleop_key`, precisamos testar algum **executável** do nosso pacote.
Existe um comando chamado `ros2 pkg executables` que lista para a gente todos
os executáveis de um pacote ROS. Para testar o comando, rode:

```bash
ros2 pkg executables turtlesim
```

Na sua tela você deve ter visto algo assim:

```bash
turtlesim draw_square
turtlesim mimic
turtlesim turtle_teleop_key
turtlesim turtlesim_node
```

Beleza, agora rode:

```bash
ros2 pkg executables ola_mundo
```

E... nada. Por quê? Simples! Nós ainda não configuramos nenhum executável.

Para fazer isso, vamos precisar mexer nos metadados do nosso pacote. Volte para
o diretório raíz do `ola_mundo` e encontre o arquivo `package.xml`. Nele, você
deve editar os campos `<name>`, `<version>`, `<description>`, `<maintainer>` e
`<license>`. Aqui, você também pode adicionar dependências de execução
(controladas pelo `rosdep`). Exemplo:

```bash
<exec_depend>rclpy</exec_depend>
<exec_depend>std_msgs</exec_depend>
```

Essas mudanças não são obrigatórias, mas é uma convenção que todos que criam
pacotes seguem. A seguir, vamos mexer no arquivo `setup.py`. Deixe-o assim:

```python showLineNumbers
from setuptools import find_packages, setup

package_name = 'ola_mundo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
# highlight-next-line
    maintainer='seu-nome',
# highlight-next-line
    maintainer_email='seu-email',
# highlight-next-line
    description='sua-descrição',
# highlight-next-line
    license='CC0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
# highlight-next-line
            "ola = ola_mundo.teste:main",
        ],
    },
)
```

Note essa parte:

```python
'console_scripts': [
    "ola = ola_mundo.teste:main",
],
```

Aqui é onde você define os **entry points** do seu pacote. No caso, há um
script de console chamado `ola`, que aciona o arquivo `teset` dentro do pacote
`ola_mundo`, especificamente a função `main`.

Agora, volte para a raíz do seu workspace e rode `colcon build`. A seguir,
vamos ver se o nosso executável aparece na lista de executáveis:

```bash
ros2 pkg executables ola_mundo
```

Output:
```bash
ola_mundo ola
```

Legal, isso significa que podemos rodar nosso script com:

```bash
ros2 run ola_mundo ola
```

Pronto! Está tudo configurado e você agora tem um pacote que você mesmo
configurou!

## 3. Criando launch files 

<Admonition 
    type="info" 
    title="Autoestudo">

[Criando launch files em
ROS2](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Creating-Launch-Files.html)
</Admonition>

Um launch file nada mais é do que um script em Python que automatiza o processo
de rodar diversos nós/pacotes. É muito comum que um sistema complexo
desenvolvido em ROS seja composto por diversos pacotes diferentes, uma ou mais
workspaces e diversos nós. Seria tedioso ter que toda vez rodar cada um desses
nós um por um. A solução? Criar uma ferramenta capaz de automatizar tudo isso.

Vamos usar a mesma workspace dos exemplos anteriores e agora só adicionar uma
pasta chamada `launch`:

```bash
cd my_package #voltando para a pasta raíz do WS
mkdir launch
```

Beleza, agora vamos entrar na pasta e criar lá dentro um arquivo chamado
`test_launch.py`. Neste arquivo, vamos adicionar o seguinte:

```python showLineNumbers
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim',
            output='screen'
        ),
        Node(
            package='turtlesim',
            executable='turtle_teleop_key',
            name='teleop',
            prefix = 'gnome-terminal --',
            output='screen'
        )
    ])
```

Note que esse launcher lança dois nós: o `turtlesim_node` e o
`turtle_teleop_key`. Note, também, que esse launchfile depende do
`gnome-terminal`. Isso significa que ele só vai funcionar se você tiver esse
emulador de terminal instalado!. Vamos rodar esse launchfile e ver o que ele
faz:

```bash
ros2 launch test_launch.py
```

Se tudo deu certo, você conseguiu seguir o exemplo básico do turtlesim sem
precisar de duas sessões de terminal.

### 3.1. Integrando launch files a pacotes ROS

<Admonition 
    type="info" 
    title="Autoestudo">

[Integrando launch files a pacotes
ROS2](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-system.html)
</Admonition>

Agora vamos integrar essa launch file a nosso pacote? Para fazer isso, vamos
precisar mexer um pouco na estrutura do nosso pacote. Antes de mais nada, entre
na pasta do nosso pacote:

```bash
cd src/my_package
```

Aqui vamos precisar que a pasta siga a seguinte estrutura:

```bash
├── launch
│   ├── __pycache__
│   │   ├── test_launch.cpython-310.pyc
│   │   └── turtlesim_mimic_launch.cpython-310.pyc
│   └── test_launch.py
├── my_package
│   ├── __init__.py
│   └── my_node.py
├── package.xml
├── resource
│   └── my_package
├── setup.cfg
├── setup.py
└── test
    ├── test_copyright.py
    ├── test_flake8.py
    └── test_pep257.py

5 directories, 12 files
```

O que precisamos fazer é editar o arquivo `setup.py` para que o sistema inclua
a pasta `launch` dentro dos executáveis do nosso pacote. Para isso, vamos fazer
as seguintes alterações ao nosso arquivo:

```python showLineNumbers title="setup.py"
#highlight-start
import os
from glob import glob
from setuptools import find_packages, setup
#highlight-end

package_name = 'my_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
# highlight-next-line
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rmnicola',
    maintainer_email='rodrigo.nicola0@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_node = my_package.my_node:main'
        ],
    },
)
```

Pronto, basta usar o `colcon build` para recompilar o seu pacote e deve ser
possível rodar o launch file com:

```bash
ros2 launch my_package test_launch.py
```
