---
title: E3 - Sistemas de controle
sidebar_position: 10
slug: /e3
---

# Sistemas de controle

**Encontro em 02/05/2024**

## 1. Resultados desejados

### 1.1. Objetivos estabelecidos

É um dos requisitos de um Engenheiro de Computação a capacidade de compreender
como podemos controlar de forma precisa atuadores (e.g. fazer com que um motor
gire exatamente 180 graus). O principal objetivo de aprendizagem deste encontro
é a compreensão dos fenômenos e ferramentas matemáticas e computacionais
envolvidas para que esse controle de atuadores seja possível.

Um objetivo secundário ligado ao projeto é perceber que o robô que eles
utilizarão só é capaz de funcionar de forma controlada por causa da aplicação
dos conceitos vistos nesse encontro.

### 1.2. Quais são as questões centrais do encontro?

1. Como eu posso fazer um sistema móvel (e.g. robô) se mover de forma precisa
   **toda vez**? O que significa um sistema ser controlado?
2. O que é controle de malha aberta? E controle de malha fechada? Como esse
   conceito influencia na eficácia do controlador e no custo de implementação?
4. Como eu posso modelar matematicamente o comportamento desses sistemas? Qual
   a relação disso com equações diferenciais e o domínio dos números complexos?
5. O que exatamente um "controlador" faz? Ele tem dinâmica? Se sim, como essa
   dinâmica influencia no comportamento do sistema?
6. Qual é a técnica de controle que vai me servir em praticamente todos os
   casos em que preciso controlar um sistema linear?
7. Qual o papel de cada um dos termos do controlador PID? Como cada um deles
   influencia na dinâmica do sistema?
8. Como definir o modelo matemático que rege o comportamento de um sistema
   dinâmico através de ensaios experimentais?

### 1.3. O que o aluno deve compreender?

* O aluno deve saber a diferença entre controle de malha aberta e malha
  fechada, elencar os prós e contras de cada tipo de controle e quando deve-se
  usar cada um;
* O aluno deve compreender que os componentes básicos para fazer com que um
sistema se movimente podem ser descritos matematicamente por equações
dinâmicas;
* O aluno deve entender o processo através do qual é possível compor sistemas
dinâmicos complexos pela concatenação do comportamento dinâmico dos seus
subsistemas e entender que a ferramenta matemática que permite esse tipo de
composição de forma relativamente indolor é a transformada de Laplace;
* O aluno deve entender que um controlador nada mais é do que um sistema
dinâmico que atua na diferença entre o objetivo do sistema e o seu estado
atual (em outras palavras, o erro). Essa compreensão é fundamental para
construir a próxima compreensão; 
* O aluno deve compreender que ajustar um controlador pode ser um processo de
tentativa e erro **ou** um algoritmo de otimização de parâmetros de uma equação
matemática. Ser capaz de fazer a segunda opção é o que tipicamente separa um
engenheiro de um hobbysta;
* O aluno deve dominar de forma intuitiva os conceitos por trás do controlador
  PID e como cada um dos seus componentes influencia no comportamento do
  sistema. Com esse conhecimento;
* O aluno deve entender que não existe um controlador perfeito, mas sim um
  controlador adequado. A resposta temporal de um sistema deve ser informada
  pelos requisitos de projeto, servindo como métrica de adequação do
  controlador; e, por fim
* O aluno deve conhecer o processo através do qual é possível aproximar a
  equação dinâmica que rege um sistema através de ensaios com entradas
  conhecidas (e.g. função heaviside).

### 1.4. O que o aluno deve ser capaz de fazer?

Uma pequena quebra de protocolo é conveniente aqui, pois a seguir será
delineado aquilo que **não** se espera que o aluno seja capaz de fazer após
apenas um encontro de sistemas de controle:

* **Não** se espera que o aluno seja capaz de modelar sistemas complexos
  utilizando massas, molas e amortecedores; e
* **Não** se espera que o aluno seja capaz de projetar um controlador do zero,
  utilizando ferramentas como lugar das raízes e otimizando os pólos e zeros de
  malha fechada do sistema;

O que se espera, na verdade, é um tanto mais singelo:

* Espera-se que o aluno seja capaz de considerar aspectos de controle ao criar
  integrações com sistemas controlados;
* Espera-se que o aluno seja capaz de inspecionar a resposta temporal de um
  sistema e ser capaz de discutir sobre a sua estabilidade; e
* Espera-se que o aluno seja capaz de discutir sobre decisões básicas de
  controle tendo em vista os objetivos e recursos de projeto.

## 2. Evidências de aprendizado

### 2.1. Evidências de desempenho

**Perguntas no padrão ENADE**

Como esse é um conteúdo obrigatório para cursos de Engenharia de Computação,
podemos contar com a régua de avaliação do ENADE para aferir se o conhecimento
adquirido pelo aluno foi suficiente. O aluno deve conseguir compreender o
contexto fornecido pelas perguntas nesse padrão e raciocinar sobre a resposta
utilizando as ferramentas matemáticas que lhes foram fornecidas no decorrer do
curso.

**Questionários abertos em sala de aula**

Muitas vezes é interessante medir a absorção do conteúdo em ambientes com menos
pressão do que uma avaliação formal, para essa finalidade pode ser útil
utilizar o ambiente de sala de aula para um debate aberto sobre perguntas
relacionadas à sistemas de controle. Aqui é bastante importante estimular a
participação de todos os alunos sem julgamento, mas sim correções oportunas.

**Questões objetivas**

A tomada de decisão a respeito de implementação de controladores pode ser
avaliada através de cenários hipotéticos dispostos no formato de questões de
multipla escolha com justificativa. Aqui vai ser possível verificar se o aluno
é capaz de participar de forma ativa de construtiva da tomada de decisão na
implementação de um sistema de controle.

### 2.2. Outras evidências

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
