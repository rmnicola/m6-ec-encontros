---
title: Turtlebot teleoperado pt. 2
sidebar_position: 1
sidebar_class_name: ponderada
slug: /ponderada4
---

# Turtlebot teleoperado pt. 2

**Atividade com prazo de entrega até 27/05**

## 1. Objetivo

Incrementar o sistema do turtlebot teleoperado para incluir conceitos de
streaming de imagens.

## 2. Enunciado

Para essa atividade, deve-se utilizar a entrega da última ponderada (Turtlebot
teleoperado pt. 1) e incrementá-lo com uma interface de usuário capaz de exibir
a imagem vista por uma câmera em tempo real. Para isso, deve-se:

1. Revisar a implementação original do turtlebot teleoperado;
2. Modificar a interface de usuário para que exiba a imagem transmitida; e
3. Criar o código necessário para transferir a imagem em tempo real.

Para efeito de avaliação:

Tanto faz se a comprovação da funcionalidade for feita usando o robô real ou
simulado. Se escolher testar com o ambiente simulado, pode fazer a transmissão
de um arquivo de vídeo qualquer. Não há o requisito específico de enviar
imagens vistas pela câmera embarcada no robô.

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

### 4.1. Reavaliação da parte 1 (até 3,0 pontos)

Se você não tirou 10 na ponderada anterior, aqui está a chance para considerar
o feedback dado e melhorar sua implementação. Como essa ponderada usa a outra
com base, vou considerar novamente o que está no padrão de qualidade de lá.

### 4.2. Adequação interface de usuário (até 3,0 pontos)

<table>
  <tr>
    <th>Supera<br/>(2,5 - 3,0)</th>
    <th>Atende<br/>(2,0 - 2,5)</th>
    <th>Quase lá<br/>(1,5 - 2,0)</th>
    <th>Insuficiente<br/>(0,5 - 1,5)</th>
    <th>Desalinhado<br/>(0,0 - 0,5)</th>
  </tr>
  <tr>
    <td>Além da funcionalidade básica do sistema, a interface ainda apresenta a
    informação da latência estimada do processo de processamento e transmissão
    de imagens para cada frame.</td>
    <td>A interface de usuário foi refeita, criando botões para as
    funcionalidades de comando, exibições do status de operação e a imagem
    transmitida em tempo real.</td>
    <td>A interface agora conta com a imagem em tempo real, mas ainda em
    terminal e com a imagem sendo exibida em uma janela separada.</td>
    <td>A interface de usuário não apresentou as mudanças determinadas,
    mantendo a interface original.</td>
    <td>Entrega fora de contexto.</td>
  </tr>
</table>

### 4.3. Nó ROS 2 + transmissão de imagens (até 4,0 pontos)

<table>
  <tr>
    <th>Supera<br/>(3,0 - 4,0)</th>
    <th>Atende<br/>(2,5 - 3,0)</th>
    <th>Quase lá<br/>(1,5 - 2,5)</th>
    <th>Insuficiente<br/>(0,5 - 1,5)</th>
    <th>Desalinhado<br/>(0,0 - 0,5)</th>
  </tr>
  <tr>
    <td>Além da transmissão da imagem, o sistema ainda consegue estimar a
    enviar a latência adicionada no processo de aquisição, processamento e
    envio da imagem</td>
    <td>A imagem é transmitida e recebida utilizando qualquer tecnologia (e.g.
    tópico ROS, websocket, porta UDP).</td>
    <td>A transmissão da imagem é feita, mas não há um tratamento adequado do
    recebimento para integrar a imagem na interface de usuário.</td>
    <td>O nó responsável pela interação com o turtlebot não teve alterações
    significativas.</td>
    <td>Entrega fora de contexto.</td>
  </tr>
</table>
