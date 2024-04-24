---
title: O5 - Setup do turtlebot
sidebar_position: 11
slug: /o5
---

# Setup do turtlebot

**Encontro em 03/05/2024**

## 1. Resultados desejados

### 1.1. Objetivos estabelecidos

Fazer o setup do turtlebot3 é uma etapa fundamental para o desenvolvimento do
projeto. Sendo assim, o objetivo desse encontro é que os alunos aprendam a
configurar o robô para operar e se comunicar através do ROS.

Um objetivo secundário é o aprofundamento do conhecimento a respeito de Linux e
das diferenças entre um dispositivo com arquitetura x86 de outro com
arquitetura arm.

### 1.2. Questões centrais do encontro

1. Qual a diferença entre o Ubuntu do meu computador e o Ubuntu que deve ser
   instalado no robô?
2. Como funciona o SSH e por quê ele vai ser importante para operar e
   configurar o robô (e servidores em geral)?
3. Por quê há duas placas no turtlebot e para que cada uma delas serve?
4. Qual a relação entre os pacotes do turtlebot3 e o workspace desenvolvido
   durante os autoestudos do módulo?
5. Por quê é tão difícil de fazer o turtlebot3 se comunicar na rede do Inteli?
   O que há de diferente nela?

### 1.3. O que o aluno deve compreender?

* O aluno deve compreender que o **target** de compilação de um programa dita
  onde ele pode ser instalado e como, ainda mais se há diferença de
  arquiteturas. Sendo assim, não é possível instalar o Ubuntu em uma máquina
  x86 e só transferir o cartão SD para o robô.
* O aluno deve compreender a importância do protocolo SSH e saber usá-lo para
  acessar sistemas que operam "sem cabeça".
* O aluno deve compreender a diferença entre um microcontrolador e um
  microprocessador e o motivo pelo qual um sistema robótico pode precisar
  implementar ambos no mesmo sistema.
* O aluno deve compreender que os pacotes do turtlebot3 estão dispostos em um
  workspace similar ao desenvolvido por ele nos autoestudos.
* O aluno deve compreender as diferenças de protocolo existentes para redes
  enterprise e como isso pode impactar em um dispositivo de baixo consumo como
  o raspberry pi.

### 1.4. O que o aluno deve ser capaz de fazer

Uma única frase: configurar o turtlebot3 com consciência e não só seguindo um
passo a passo sem usar o pensamento crítico.

## 2. Evidências de aprendizado

### 2.1. Evidências de desempenho

**Atuação no desenvolvimento de projeto**

Será possível aferir se o aluno absorveu os conhecimentos e competências
esperados através do seu desempenho no desenvolvimento do projeto. Um aluno que
não conseguiu acompanhar essa instrução dificilmente será capaz de interagir
com o robô e provavelmente se esquivará dessas tarefas.

## 3. Plano de aprendizagem

**Daily** (0 - 15 min)

**Discussão - apresentação das questões centrais do encontro** (15 - 25 min)

O professor deve apresentar as questões centrais do encontro para que o aluno
esteja ciente de seus objetivos de aprendizagem.

**Expositiva - o que é o turtlebot3 e overview do setup** (25 - 55 min)

O professor deve apresentar o turtlebot 3 e comentar sobre algumas de suas
escolhas de projeto. Após isso, deve ser apresentado um overview do processo de
setup do robô, comentando sobre pontos importantes que possam causar dúvidas
para os alunos.

**Atividade em grupo - configuração do turtlebot 3** (55 - 100 min)

O professor acompanhará os grupos enquanto configuram o turtlebot 3 e testam o
controle tanto por ssh quanto por configuração do ROS em rede. Nos 5 minutos
finais, o professor interrompe a atividade para conversar novamente sobre as
etapas do setup e aferir o progresso de cada grupo. Dependendo do resultado
aqui, essa atividade terá sua duração estendida ou a próxima atividade irá
começar.

**Atividade em grupo - controlando o turtlebot 3** (100 - 115 min)

A atividade se divide em duas etapas:

1. Controlar o robô acessando a raspberry por ssh; e
2. Controlar o robô usando outro dispositivo, que envia mensagens de comando
   através da infraestrutura do ROS.

**Encerramento - contextualizando os resultados do dia** (115 - 120 min)

O professor deve contextualizar os resultados da atividade feita com o projeto
e o que se espera do sprint 2:

* Basta só utilizar o teleop_key para o entregável dessa sprint?
* Como é possível ir além?
* Como se reguardar para uma possível falha de rede no dia da apresentação?

