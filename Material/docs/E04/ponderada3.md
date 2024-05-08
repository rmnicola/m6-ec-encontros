---
title: Turtlebot teleoperado
sidebar_position: 2
sidebar_class_name: ponderada
slug: /ponderada3
---

# Turtlebot teleoperado

**Atividade com prazo de entrega até 20/05**

## 1. Objetivo

Fazer o setup e interagir com o turtlebot, compreendendo os conceitos básico
para uso do ROS em rede e dos pacotes para interação com o robô.

## 2. Enunciado

Para essa atividade, deve-se desenvolver um script que seja capaz de fazer a
leitura de teclas pressionadas pelo usuário e, utilizando um publisher no
tópico adequado, provocar a movimentação do robô. Para isso, deve-se
desenvolver dois componentes principais:

1. Uma interface de usuário (pode ser feita em terminal). Essa interface deve
   ser capaz de detectar em tempo real os botões pressionados pelo usuário e
   dar um feedback da velocidade do robô em tempo real.
2. Um nó de ROS 2 capaz de comandar o robô utilizando o tópico adequado e que
   seja capaz de verificar se o robô está inicializado e disponível para
   receber suas mensagens antes de enviá-las.

Para efeito de avaliação:

Tanto faz se a comprovação da funcionalidade for feita usando o robô real ou
simulado.

## 3. Padrão de entrega

:::warning ATENÇÃO

Esses são os critérios mínimos para que eu considere a atividade como entregue.
Fique atento, pois o não cumprimento de qualquer um desses critérios pode, no
melhor dos casos, gerar um desconto de nota e, no pior deles, invalidar a
atividade.

:::

1. A atividade deve ser feita em um repositório **aberto** no Github. Seu link
deve ser fornecido no card da adalove;
2. No README do repositório deve ter instruções claras de como instalar e rodar o
sistema criado, comandos em blocos de código e uma expliação sucinta do que
fazem;
3. Ainda no README, deve haver um vídeo gravado demonstrando plenamente o
funcionamento do sistema criado;

## 4. Padrão de qualidade

### 4.1. Interface de usuário (até 5,0 pontos)

<table>
  <tr>
    <th>Supera<br/>(4,0 - 5,0)</th>
    <th>Atende<br/>(3,0 - 4,0)</th>
    <th>Quase lá<br/>(2,0 - 3,0)</th>
    <th>Insuficiente<br/>(1,0 - 2,0)</th>
    <th>Desalinhado<br/>(0,0 - 1,0)</th>
  </tr>
  <tr>
    <td>A interface permite ao usuário comandar o robô por botões, exibe o
    status do robô em tempo real e oferece a funcionalidade de parada por botão
    de emergência.</td>
    <td>Além de comandar o robô em tempo real, a interface exibe a velocidade
    do robô em todos os eixos em tempo real.</td>
    <td>Interface capaz de detectar em tempo real botões pressionados pelo usuário.</td>
    <td>Interface capaz de detectar comandos enviados pelo usuário, mas não em
    tempo real/por botões pressionados.</td>
    <td>Entrega fora de contexto.</td>
  </tr>
</table>

### 4.2. Nó ROS 2 (até 5,0 pontos)

<table>
  <tr>
    <th>Supera<br/>(4,0 - 5,0)</th>
    <th>Atende<br/>(3,0 - 4,0)</th>
    <th>Quase lá<br/>(2,0 - 3,0)</th>
    <th>Insuficiente<br/>(1,0 - 2,0)</th>
    <th>Desalinhado<br/>(0,0 - 1,0)</th>
  </tr>
  <tr>
    <td>Além da funcionalidade básica de controle e monitoramente de
    velocidade, o nó ainda conta com um serviço que é responsável por parar o
    robô e matar o processo de operação. Deve-se criar o cliente e o servidor
    para esse serviço.</td>
    <td>O nó é capaz de publicar mensagens de comando e se subscreve no tópico
    adequado para fazer a leitura das velocidades do turtlebot</td>
    <td>O nó é capaz de publicar mensagens corretamente no tópico de comando do
    turtlebot.</td>
    <td>O nó é inicializado corretamente, mas não há interação correta com os
    tópicos e serviços adequados.</td>
    <td>Entrega fora de contexto.</td>
  </tr>
</table>
