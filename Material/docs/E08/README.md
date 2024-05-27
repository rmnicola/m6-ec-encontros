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
4. 

### 1.3. O que o aluno deve compreender?

## 3. Plano de aprendizagem

**Daily** (0 - 15 min)

**Discussão: apresentação das questões centrais** (15 - 25 min)

*O que significa um sistema ser controlado?*

O professor irá discutir abertamente as questões centrais envolvidas no
encontro, apresentando o problema de controle de sistemas de forma intuitiva.

**Expositivo: sistemas de controle e o domínio dos complexos** (25 - 50 min)

*O que é controle de malha aberta e de malha fechada*
*Como e por que modelar o comportamento de sistemas dinâmicos*
*O que é um controlador? Ele tem dinâmica?*

Contando com a exposição mais aprofundada dos autoestudos sobre o assunto, o
professor deve contextualizar a próxima etapa da instrução utilizando alguns
slides explicativos sobre o papel da transformada de Laplace na modelagem de
sistemas e como um sistema de malha fechado pode ser representado no domínio
dos complexos.

**Atividade: interagindo com o PID** (50 - 90 min)

* Qual é a técnica de controle que vai me servir em praticamente todos os casos
  em que preciso controlar um sistema linear? *
* Qual o papel de cada um dos termos do controlador PID? Como cada um deles
  influencia na dinâmica do sistema? *

Utilizando um simulador simplificado, os alunos devem interagir com cada
componente do controlador PID e observar o comportamento do sistema quando um
componente é adicionado ou tirado e quando uma constante aumenta ou diminui.

A atividade deve ser dividida em dois momentos:

1. Interação com o PID (20 min)
2. Discussão do que descobriu com o seu grupo (20 min)

**Atividade: discutindo as conclusões* (90 - 110 min)

O professor guiará uma discussão em grupo sobre as conclusões do experimento
sobre PID. As perguntas que guiarão o debate serão:

* Qual é a técnica de controle que vai me servir em praticamente todos os casos
  em que preciso controlar um sistema linear? *
* Qual o papel de cada um dos termos do controlador PID? Como cada um deles
  influencia na dinâmica do sistema? *

**Expositivo: identificação de sistemas** (110 - 120 min)

* Como definir o modelo matemático que rege o comportamento de um sistema
  dinâmico através de ensaios experimentais? *

Para finalizar o material do encontro, o professor apresentará a resposta
temporal padrão de sistemas de segunda ordem como uma forma de avaliar sistemas
através de experimentação, sem precisar modelar precisamente o seu
comportamento.

## 4. Slides 

<div style={{ textAlign: 'center' }}>
    <iframe 
        style={{
            display: 'block',
            margin: 'auto',
            width: '100%',
            height: '50vh',
        }}
        src="https://slides.com/rodrigomangoninicola/m6-ec-encontros/embed#/encontro3"
        frameborder="0" 
        allowFullScreen>
    </iframe>
</div>

