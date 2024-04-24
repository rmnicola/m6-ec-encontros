---
title: Launchfiles ROS
sidebar_position: 2
sidebar_class_name: autoestudo
slug: /launchfiles
---

# Launch files em ROS

Agora que já fizemos algumas coisas usando ROS, você deve ter percebido um
padrão um tanto chato na execução de pacotes ROS - eles são muitas vezes
**bastante** fragmentados! Quer um exemplo? Beleza, vamos ver o que precisamos
rodar para fazer uma simples tartaruga se mover na tela:

```bash
ros2 run turtlesim turtlesim_node
```

E, em outro terminal, precisamos rodar:

```bash
ros2 run turtlesim turtle_teleop_key
```

E, acredite, o problema fica **muito** pior quando começamos a mapear e navegar
em ambientes de forma autônoma.

Qual a solução? **Launch files**

## 1. Criando launch files 

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

