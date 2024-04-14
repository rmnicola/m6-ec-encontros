---
title: ROS
sidebar_position: 2
sidebar_class_name: autoestudo
slug: /ros
---

import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

# Instalação do ROS

Para instalar o ROS, podemos seguir de duas formas distintas:

1. A forma fácil, usando meu script de instalação automática; ou
2. A forma convencional, configurando os repositórios apt e instalando tudo na
   mão.

## 1. Script de instalação automática

Antes de mais nada, garanta que o git está instalado com:
```bash
sudo apt install git -y
```

Para facilitar o processo de instalação do ROS 2 Humble, é possível utilizar 
um script feito por um bom samaritano. Para tal, clone o repositório onde esta
o script:

```bash
git clone https://github.com/rmnicola/Scripts.git
cd Scripts
```

Embora seja possível rodar os scripts individualmente, sugiro que usem o script
de instalação localizado na pasta raíz do repositório. Ele é responsável por
buscar todos os arquivos `sh` dentro do repositório e criar um link simbólico
para a pasta `/usr/local/bin`. Isso significa que o sistema passa a tratar
esses scripts como arquivos binários. Sendo assim, não é mais necessário fazer:

```bash
./Scripts/General/install-ros.sh 
```

Bastando, em vez disso, fazer:

```bash
install-ros 
```

Como fazer isso? Basta executar como admin o script de instalação:

```bash
cd Scripts 
sudo ./install.sh
```

Após a instalação, não se esqueça de adicionar `source
/opt/ros/humble/setup.xxx` ao arquivo de configuração do seu shell,
substituindo o `xxx` pela extensão adequada. (mais comuns bash e zsh). Veja
abaixo:

<Tabs defaultValue="bash" values={[
        {label: 'Bash', value: 'bash'},
        {label: 'Zsh', value: 'zsh'},
        {label: 'Zsh com a minha config.', value: 'zsh-meu'},
  ]}>

<TabItem value="bash">

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

</TabItem>

<TabItem value="zsh">

```bash
echo "source /opt/ros/humble/setup.zsh" >> ~/.zshrc
```

</TabItem>

<TabItem value="zsh-meu">

```bash
echo "source /opt/ros/humble/setup.zsh" >> '$ZODTDIR'/.zshrc
```

</TabItem>
</Tabs>

## 2. Na raça

<div style={{ textAlign: 'center' }}>
    <iframe 
        style={{
            display: 'block',
            margin: 'auto',
            width: '100%',
            height: '50vh',
        }}
        src="https://www.youtube.com/embed/Dt1x4NBPp-Y" 
        frameborder="0" 
        allowFullScreen>
    </iframe>
</div>

Para instalar o ROS, precisamos adicionar novos repositórios ao apt, pois o ROS não se encontra nos repositórios padrão do Ubuntu. Para isso começaremos garantindo que o repositório `universe` está habilitado. Rode:

```bash
sudo apt-add-repository universe
```

A seguir, precisamos baixar uma chave GPG e adicioná-la ao keyring do sistema para poder validar o repositório que vamos adicionar. Rode:
```bash
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

Agora precisamos adicionar o repositório à lista de repositórios. Use:
```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

Como fizemos alterações nos repositórios do apt, precisamos atualizar seu banco de dados novamente. Rode:

```bash
sudo apt update
```

Pronto! Agora estamos finalmente prontos para instalar a nossa distribuição de ROS. Para facilitar nossa vida, vamos escolher a versão do pacote mais completa, assim não precisaremos nos preocupar se os exemplos e pacotes que vamos precisar já estarão instalados ou não. Rode:

```bash
sudo apt install ros-humble-desktop
```

Essa instalação vai demorar alguns minutos, então tenha paciência =)

Falta apenas uma coisa para termos o poder do ROS em nossas mãos: por padrão, o ROS não adiciona automaticamente todos os executáveis e variáveis de ambiente ao nosso sistema, mas existe um script que faz todo esse setup para nós. Como ninguém tem tempo de ficar dando source nesse script toda vez, rodem esse comando para garantir que tudo vai estar configurado sempre que você abrir o terminal do WSL:

:::caution Aviso
 
O comando abaixo vai reconfigurar seu bashrc para que, da proxima vez, ele
consiga identificar os comandos do ROS. Caso não queira reiniciar o terminal,
rode:

```bash
source ~./bashrc
```
 
:::

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

Perfeito! Agora estamos prontos para trabalhar com o ROS2 Humble. Vamos testar?

Abra dois terminais e, para cada um deles vamos rodar um comando. Para o terminal 1:

```bash
ros2 run demo_nodes_cpp talker
```

Para o terminal 2:
```bash
ros2 run demo_nodes_cpp listener
```

Se tudo deu certo, você acabou de ver dois processos totalmente independentes conversando através da interface de comunicação do ROS. Legal, né? Para fechar as instruções necessárias, vamos apenas aprender a rodar nosso exemplo.
