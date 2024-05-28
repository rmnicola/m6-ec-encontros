---
title: E8 - MLP e torch
sidebar_position: 21
slug: /e8
---

# Multi-layer perceptrons

**Encontro em 28/05/2024**

## 1. Resultados desejados

### 1.1. Objetivos estabelecidos

Seguindo a linha de introdução à deep learning, o objetivo deste encontro é
apresentar uma ferramenta capaz de classificar problemas que não são
linearmente separáveis e aprofundar alguns conhecimentos a respeito de deep
learning.

### 1.2. Quais são as questões centrais do encontro?

1. Como as redes neurais aprendem? O que é backpropagation e qual a relação com
   o forward pass?
2. O que são e como definir as funções de perda? Por quê se usa tanto a
   entropia cruzada?
3. O que são as funções de ativação e por quê elas existem? O que é ReLU e por
   quê é a função de ativação mais utilizada?
4. Como construir uma rede de perceptrons com mais de um perceptron? Ela é
   capaz de resolver problemas que não são linearmente separáveis?

### 1.3. O que o aluno deve ser capaz de fazer?

O aluno deve ser capaz de:

* Implementar um multi layer perceptron em Python puro, sem utilizar frameworks
  ou bibliotecas de deep learning.
* Implementar algoritmos de backpropagation, forward pass, uma implementação de
  função de perda simples e com uma função de ativação degrau.

## 2. Plano de aprendizagem

**Daily** (0 - 15 min)

**Expositivo: conceitos de deep learning** (15 - 45 min)

Apresentação dos conceitos:

* Backpropagation
* Forward pass
* Função de perda
* Função de ativação

**Análise de implementação: perceptron** (45 - 60 min)

Apresentação da solução da implementação de um perceptron simples (aula
passada). Nessa implementação já tem o forward pass e a função de ativação por
degrau.

**Implementação: função de perda e backpropagation** (60 - 80 min)

Tempo para alunos desenvolverem em cima do perceptron apresentado os algoritmos
de treinamento por backpropagation. Para isso, deve-se também desenvolver o
algoritmo de cálculo da função de perda. Para testar a implementação, deve-se
treinar o perceptron para representar as funções lógicas NAND e OR.

**Discussão: o problema do XOR** (80 - 95 min)

Por quê o perceptron não consegue representar um XOR? Solução por lógica e por
deep learning.

**Implementação: Multi-layer perceptron** (95 - 115 min)

Tempo para os alunos desenvolverem o multi-layer perceptron para resolver o
problema do XOR.

**Fechamento** (115 - 120 min)

Apresentação da solução final e discussão dos próximos passos a respeito de
deep learning.

## 3. Slides 

<div style={{ textAlign: 'center' }}>
    <iframe 
        style={{
            display: 'block',
            margin: 'auto',
            width: '100%',
            height: '50vh',
        }}
        src="https://slides.com/rodrigomangoninicola/m6-ec-encontros/embed#/encontro8"
        frameborder="0" 
        allowFullScreen>
    </iframe>
</div>

