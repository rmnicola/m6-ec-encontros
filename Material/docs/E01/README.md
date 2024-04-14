---
title: E1 - Introdução ao ROS
sidebar_position: 5
slug: /e1
---

# Configurando e usando o ROS

**Encontro em 17/04/2024**

## 1. Para que/quem serve esse encontro?

Esse encontro serve para o estudante que acabou de instalar o ROS e ainda não
sabe configurá-lo ou utilizá-lo.

## 2. Objetivos

1. Apresentar o ROS;
2. Configuração do ROS e realização de testes simples;
3. Apresentar o conceito de workspaces e como criá-las;
4. Explorar os conceitos de tópicos e nós em ROS; e
5. Realizar uma atividade usando o ROS em rede e o turtlesim.

## 3. Roteiro 

### 3.1. Apresentando e configurando o ROS

*Obs: o encontro começa aos 15 min pois deve-se realizar a daily sempre no
começo da instrução*

**Atividade: relembrando o MQTT** (15 - 30 min)

Os alunos devem separar-se em grupos, discutir e separar **5 características**
do MQTT e concluir respondendo à pergunta:

* Por que é vantajoso usar MQTT para dispositivos IoT?

A atividade deve ser feita em 15 minutos e registrada em uma planilha
compartilhada pelo professor.

**Comparando MQTT com ROS** (30 - 45 min)

Utilizando as características descritas pelos alunos, o professor guiará uma
discussão sobre em que sentidos o ROS e o MQTT são parecidos e em que sentido
eles são diferentes. Ao final dessa comparação, a seguinte pergunta deverá ser
respondida:

* Por que é vantajoso usar o ROS para desenvolvimento de robôs?

**Configurando o ROS** (45 - 60 min)

O professor demonstrará como configurar o ROS para que seja possível utilizá-lo
em um ambiente de desenvolvimento em Ubuntu 22.04. Ainda nessa exposição, serão
utilizadas as rotinas de teste do ROS e será criado o primeiro workspace do
módulo.

**Pausa para dúvidas** (60 - 70 min)

Pausa de 10 minutos para esclarecimento de dúvidas e garantir que todos estão
na mesma etapa.

### 3.2. Entendendo nós, tópicos e tipos de mensagens

**Criando nós e tópicos** (70 - 80 min)

Voltando para a atividade expositiva, o professor irá utilizar o workspace
criado para desenvolvimento de exemplos de subscribers e publishers utilizando
a biblioteca de ROS2 para Python.

**Turtlesim e tipos de mensagens** (80 - 90 min)

A atividade a seguir será a modificação de um publisher para interagir com o
turtlesim. Para isso, utilizaremos o rqt para investigar o tipo de mensagem que
deve ser enviada para que seja possível mover a tartaruga na tela.

**Atividade: ROS domain ID e controlando a tartaruga do colega** (90 - 115 min)

Os estudantes devem se dividir em duplas e pesquisar para que serve o ROS
domain ID e implementar um sistema em que seja possível um estudante movimentar
a tartaruga do turtlesim do outro.

**Call to action** (115 - 120 min)

O professor vai apresentar a atividade ponderada que deverá ser entregue até o
fim da semana: desenhando com o turtlesim. Para isso, há alguns conceitos de
ROS que não foram formalmente apresentados mas podem ser úteis no desafio:

* Serviços
* Ações

A provocação é para que tentem desvendar o que são esses dois conceitos e como
eles tem relação com o turtlesim.

Após isso, os estudantes estão liberados para o desenvolvimento do projeto.

## 4. Slides 

<div style={{ textAlign: 'center' }}>
    <iframe 
        style={{
            display: 'block',
            margin: 'auto',
            width: '100%',
            height: '50vh',
        }}
        src="https://slides.com/rodrigomangoninicola/m6-ec-encontros/embed#/encontro1"
        frameborder="0" 
        allowFullScreen>
    </iframe>
</div>
