---
title: Workspaces ROS
sidebar_position: 3
sidebar_class_name: autoestudo
slug: /workspaces
---

import Admonition from '@theme/Admonition';

## 1. Criando um workspace em ROS 

O workspace de ROS é basicamente uma pasta onde se concentram um ou mais pacotes 
ROS. Ela é útil pois podemos juntar vários pacotes e compilá-los com apenas um
comando e também dar apenas um `source` para adicionar os comandos de
run/launch ao nosso sistema. 

Para criar um workspace ROS, basta criar uma pasta qualquer e, dentro dela,
adicionar uma subpasta `src`, que é onde vão ficar os nossos pacotes.

```bash
mkdir -p meu_workspace/src
```

Após isso, vamos entrar em nossa pasta `src` e adicionar um pacote de exemplo
para testarmos nosso workspace.

```bash 
cd meu_workspace/src
git clone https://github.com/ros/ros_tutorials.git -b humble 
```

Pacotes ROS costumeiramente tem dependências para serem executados. Para
garantir que temos as dependências necessárias para nosso pacote, precisamos
rodar o `rosdep`. Em nossa instalação, o `rosdep` não foi adicionado por
padrão. Precisamos, portanto, instalá-lo:

```bash 
sudo apt install python3-rosdep
```

Após a instalação, precisamos configurar o rosdep. Para isso, usaremos dois
comandos em sequência: 

```bash 
sudo rosdep init
rosdep update
```

Feito isso, basta resolver nossas dependências com: 

```bash 
cd meu_workspace #voltando para a pasta raíz do ws
rosdep install -i --from-path src --rosdistro humble -y
```

Pronto! Esse comando já vai checar todos os seus pacotes e instalar as
depedências automaticamente.

A seguir, vamos compilar o nosso pacote usando:

:::caution Aviso

Para executar o comando abaixo você precisa, antes, instalar o seguinte pacote:

```bash
sudo apt install python3-colcon-common-extensions
```

:::

```bash 
colcon build
```

Pode ser que apareça um warning sobre o fim da vida do `setuptools`, mas está
tudo bem. Caso não esteja, basta forçar a instalação da versão `58.2.0` com:

```bash 
pip install setuptools==58.2.0
```

Note que surgiram as pastas `install`, `build` e `log` em seu workspace. Caso
queira subir seu ws para um repositório, não esqueça de adicionar essas pastas
ao `.gitignore`!

Para poder rodar o seu pacote, precisamos agora apenas dar `source` no script
de configuração do workspace. Fazemos isso com: 

```bash
source install/local_setup.bash #se estiver usando zsh, mude para setup.zsh
```

Pronto! Seu workspace ROS está configurado e com um pacote funcionar. Vamos
criar o nosso próprio pacote agora?

## 2. Criando pacotes em ROS

### 2.1. Criando um pacote pré-preenchido

Para criar nosso pacote, vamos utilizar o seguinte comando:

```bash 
ros2 pkg create --build-type ament_python --node-name my_node my_package
```

Como nosso pacote já veio com um exemplo, vamos só precisar dar build nele para
poder rodar alguma coisa dele:

```bash
colcon build
```

Como nosso ws tem dois pacotes, às vezes podemos querer especificar para
compilar apenas um (ou mais) pacotes em vez de compilar tudo o que está lá.
Podemos fazer isso com: 

```bash
colcon build --packages-select my_package
```

Após compilado, precisamos dar source no script de setup do WS:

```bash
source install/local_setup.bash #se estiver usando zsh, mude para setup.zsh
```

Pronto! Quer rodar seu pacote? Use:

```bash
ros2 run my_package my_node
```

### 2.2. Criando um pacote vazio e preenchendo com o seu nó

Estando dentro da pasta `src` do seu workspace ROS, rode:

```bash
ros2 pkg create --build-type ament_python ola_mundo
```

Dessa vez, o pacote será criado sem nenhum script para ser rodado. Vamos
configurar tudo na mão. Primeiro, crie um arquivo dentro da subpasta teste:

```bash
cd ola_mundo/ola_mundo
touch ola.py
```

Vamos preencher esse arquivo com:

```python showLineNumbers title="ola_mundo.py"
def main():
    print("Ola, mundo!")

if __name__ == "__main__":
    main()
```

Agora, vamos configurar os metadados do pacote para conseguir rodar usando o
`ros2 run`. Primeiro, vamos mexer no arquivo `package.xml`, localizado na raíz
do projeto `teste`. Nele, você deve editar os campos `<name>`, `<version>`,
`<description>`, `<maintainer>` e `<license>`. Aqui, você também pode
adicionar dependências de execução (controladas pelo `rosdep`). Exemplo:

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
            "ola = ola_mundo.ola:main",
        ],
    },
)
```

Note essa parte:
```python
'console_scripts': [
    "ola = ola_mundo.ola:main",
],
```

Aqui é onde você define os entry points do seu pacote. No caso, há um script de
console chamado `ola`, que aciona o arquivo `ola` dentro do pacote `ola_mundo`,
especificamente a função `main`.

Agora, volte para a raíz do seu workspace e rode `colcon build` e de `source`
no script de setup do workspace. A seguir, rode:

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

## 3. Integrando launch files a pacotes ROS

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
