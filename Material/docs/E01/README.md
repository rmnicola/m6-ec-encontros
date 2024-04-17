---
title: E1 - Introdução ao ROS
sidebar_position: 5
slug: /e1
---

# Configurando e usando o ROS

**Encontro em 17/04/2024**

<img 
  src="https://media1.tenor.com/m/GNGJxpwR6nQAAAAC/hibito-nanba-nanba-hibito.gif"
  alt="Nanba hibito"
  style={{ 
    display: 'block',
    marginLeft: 'auto',
    maxHeight: '70vh',
    marginRight: 'auto'
  }} 
/>
<br/>

> "Here comes PD, Pretty Dog"
>
> — Nanba Hibito (Space Brothers)


## 1. Para que/quem serve esse encontro?

Esse encontro serve para o aluno que:

* Está com dúvidas para configurar o Ubuntu;
* Ainda não sabe utilizar o ROS;
* Tem duvidas sobre o motivo pelo qual vamos utilizar o ROS.

## 2. Objetivos

1. Configurar o Ubuntu;
1. Apresentar o ROS;
2. Configuração do ROS e realização de testes simples;
3. Apresentar o conceito de workspaces e como criá-las; e
4. Explorar os conceitos de tópicos e nós em ROS.

## 3. Roteiro 

### 3.1. Configurando o Ubuntu e introdução ao ROS

*Obs: o encontro começa aos 15 min pois deve-se realizar a daily sempre no
começo da instrução*

**Demonstração: configuração do Ubuntu** (15 - 45 min)

A demonstração feita será de configuração do Ubuntu. Nesse momento, o professor
deve conversar com os alunos sobre algumas das ferramentas comuns utilizadas no
Linux. Além disso, será instalado o ROS e feito o teste inicial da instalação.

**Questão: relembrando o MQTT** (45 - 75 min)

Utilizando uma ferramenta de questionário ao vivo, o professor apresentará uma
série de perguntas de multipla escolha sobre o MQTT. O objetivo é fazer com que
os alunos relembrem o protocolo e considerem suas vantagens e desvantagens.

**Comparando MQTT com ROS** (75 - 90 min)

Após discussão e revisão do MQTT, serão apresentadas as características do ROS
e a comparação com o MQTT ficará evidente. Após isso, será feita uma discussão
aberta sobre as vantagens de se utilizar o ROS.

### 3.2. Entendendo nós, tópicos e tipos de mensagens

**Demonstração: criando um workspace** (90 - 100 min)

O professor retoma a demonstração criando um workspace, que é a maneira
sugerida para gerenciar projetos utilizando o ROS.

**Demonstração: criando um publisher e um subscriber** (100 - 110 min)

Modificação do workspace criado para apresentação de um publisher e um
subscriber simples.

**Turtlesim e tipos de mensagens** (110 - 120 min)

Para finalizar o encontro, o professor apresenta o turtlesim para os estudantes
e apresenta o desafio proposto na ponderada. Para isso, o aluno deve investigar
o nó turtlesim utilizando o **rqt** (professor apresenta em sala) para entender
como interagir com a simulação. Alguns dos conceitos que podem ser úteis para o
estudo do aluno são:

* Tipos de mensagens em ROS
* Ações
* Serviços
* Publishers e subscribers utilizando orientação à objetos

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
