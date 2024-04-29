---
title: Desenhando com o turtlesim
sidebar_position: 5
sidebar_class_name: ponderada
slug: /ponderada2
---

# Desenhando com o turtlesim

**Atividade com prazo de entrega até 02/05**

## 1. Objetivo

O objetivo da atividade é garantir o aprendizado dos conceitos básicos do ROS.
Esses conceitos serão necessários para interagir com o turtlebot nas próximas
etapas de projeto.

## 2. Enunciado

Vamos desenhar com o turtlesim? Nessa atividade, você vai precisar criar um
projeto ROS que interaja com a tartaruga do turtlesim de modo que, ao final da
execução do script, a tela do turtlesim apresente um desenho. Qual desenho?
Você é o Bob Ross do sistema ROS (tradução: use sua criatividade).

Alguns requisitos pedidos:

* O script deve sempre fazer o mesmo desenho;
* O script deve fazer o spawn e kill de ao menos uma tartaruga (pesquise como
  fazer isso); e
* O script deve utilizar o penUp e penDown ao menos uma vez (pesquise,
  pesquise).

Além dos requisitos descritos acima, vai receber nota máxima apenas a entrega
que esteja formatada como um pacote ROS.

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

### 4.1. Implementação do publisher (até 9,0 pontos)

**[0,0 - 1,0]**
O repositório apresentado está fora do contexto da atividade

**[1,0 - 4,0]**
O repositório apresentado contem um script em ROS que é capaz de minimamente
interagir com o turtlesim. No entanto, não há uma implementação que atenda aos
requisitos apresentados no enunciado. Há movimentação da tartaruga, mas nenhum
desenho inteligível é feito e/ou o script tem um resultado diferente toda vez
que roda.

**[4,0 - 7,0]**
O repositório apresenta um script em ROS que, sempre que executado, interage
com o turtlesim e cria sempre o mesmo desenho. Ainda há um ou mais requisitos
não atendidos (spawn/kill, penUp/penDown).

**[7,0 - 9,0]**
O repositório apresenta um script ROS que faz sempre o mesmo desenho e atende
aos requisitos especificados.

### 4.2. Pacote ROS (até 1,0 ponto)

**[0,0 - 0,5]**
O repositório apresenta um pacote ROS configurado, mas os entry points não
foram configurados adequadamente, o que faz com que não seja possível rodar o
script criado usando o comando `ros2 run`

**[0,5 - 1,0]**
O repositório apresenta um pacote ROS devidamente configurado. Após a
instalação do pacote, é possível rodar o script utilizando um comando `ros2
run`
