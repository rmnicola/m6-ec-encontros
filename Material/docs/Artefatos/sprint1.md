---
title: Sprint 1
sidebar_position: 1
sidebar_class_name: artefato
slug: /sprint1
---

# Artefatos Sprint 1 (Computação)

## 1. Simulador MQTT

### 1.1. Enunciado

O grupo deverá implementar um cliente MQTT que se comunica com um broker
público. O intuito é a capacidade de simular a comunicação gerada pelos
dispositivos do parceiro, demonstrando a compatibilidade com o protocolo de
comunicação por ele utilizado assim como as especificações dos componentes em
si. Testes deverão ser especificados, e seus resultados devem ser registrados
para mostrar o correto funcionamento deste primeiro protótipo.

**Tecnologias sugeridas**

* [Paho MQTT](https://eclipse.dev/paho/)
* [Lib paho-mqtt para Python](https://pypi.org/project/paho-mqtt/)
* [Lib paho-mqtt para Go](https://github.com/eclipse/paho.mqtt.golang)

### 1.2. Padrão de entrega

:::warning

O padrão de entrega do artefato contempla uma série de requisitos para que a
atividade seja considerada concluída. Embora não haja atribuição direta de
pontuação aos itens aqui descritos, entenda-se que o **não cumprimento de
qualquer dos requisitos** pode, no melhor dos casos, acarretar em um **desconto
de nota** e, no pior caso, **invalidar a entrega por completo**. Fique atento!

:::

1. O código fonte da solução deve estar disponível no repositório do grupo no
   Github (na branch `main`), em um diretório denominado `src`. É provável que
   a sua solução não seja monolítica. Se esse for o caso, os módulos que compõe
   o seu código-fonte devem ser divididos em subdiretórios.
2. Os testes desenvolvidos devem estar disponíveis no diretório `tests` e,
   quando possível, aplicados de forma automatizada. Os testes realizados sem
   automatização devem ter seu procedimento de execução e resultados claramente
   descritos em um arquivo contido dentro do diretório de testes.
3. As instruções para que o parceiro possa executar o projeto devem estar
   claramente discriminadas na documentação, com um link no README para que a
   seção possa ser prontamente acessada.
4. O projeto em seu estado atual deve estar disponível em um release do Github
   cujo nome deve incluir a numeração da sprint (e.g. release-sprint-1).
5. As questões centrais de desenvolvimento da sprint e os testes devem ser
   apresentadas de forma clara durante o review com o parceiro.

### 1.3. Padrão de qualidade

**[0,0 - 3,0]**
A implementação do simulador apresenta inconsistências técnicas
e/ou está substancialmente desalinhada com os requisitos especificados pelo
parceiro.

**[3,0 - 5,0]**
Implementação aparentemente correta de um simulador de dispositivos IoT, mas
sem evidências de funcionamento e adequação às especificações do parceiro
através de testes objetivos.

**[5,0 - 7,0]**
Simulador de dispositivos IoT funcional e com testes bem definidos para
evidenciar seu funcionamento e adequação, porém apresenta falhas em testes (de
forma intermitente ou não). Os testes propostos refletem ao menos parcialmente
os requisitos não funcionais especificados para essa etapa do projeto.

**[7,0 - 9,0]**
Simulador funcional com testes bem definidos e que refletem os requisitos não
funcionais do projeto. O sistema passa em todos os testes de forma consistente.

**[9,0 - 10,0]**
Além de um simulador comprovadamente funcional, o sistema desenvolvido conta
com abstrações para que seja possível adequar o simulador para outros
dispositivos IoT (diferentes daqueles especificados pelo parceiro) sem a
necessidade de um refatoramento substancial.

## 2. Documentação 

### 2.1. Enunciado

O grupo deverá especificar requisitos funcionais, não funcionais e restrições,
testes relacionados e propor uma arquitetura inicial para o projeto. Pelo menos
um requisito deve estar relacionado à segurança e pelo menos um requisito deve
estar relacionado à escalabilidade.

### 2.2. Padrão de entrega

:::warning

O padrão de entrega do artefato contempla uma série de requisitos para que a
atividade seja considerada concluída. Embora não haja atribuição direta de
pontuação aos itens aqui descritos, entenda-se que o não cumprimento de
qualquer dos requisitos pode, no melhor dos casos, acarretar em um desconto de
nota e, no pior caso, invalidar a entrega por completo. Fique atento!

:::

1. O texto desenvolvido pelo grupo deve estar disponível em uma página estática
   gerada pelo framework Docusaurus. Para tal, deve haver um diretório no
   repositório denominado `docs`, onde ficará a raíz do Docusaurus;
2. A documentação da sprint 1 deve estar inteiramente contida em uma seção
   denominada `Sprint 1`. Cada um dos artefatos deve ter sua descrição contida
   em uma página ou subseção dentro da seção `Sprint 1`.
3. As figuras utilizadas no documento devem sempre ser referenciadas no texto,
   com descrições textuais que estimulem a coesão entre o que é apresentado
   visualmente e o restante do texto; e 
4. Todas as figuras utilizadas na documentação devem estar salvas dentro do
   diretório `docs`, em um subdiretório chamado `static`.

### 2.3. Padrão de qualidade

#### 2.3.1 Qualidade textual (até 5,0 pontos)

**[0,0 - 1,0]**
A documentação desenvolvida foge do contexto do projeto/está consideravelmente
incompleta.

**[1,0 - 2,0]**
A documentação desenvolvida aborda todos os temas obrigatórios (artefatos), mas
não apresenta-os de forma coesa, de modo que não seja possível vislumbrar o
objetivo conjunto do projeto. O documento apresenta uma linguagem
consistentemente inadequada para um documento de cunho técnico e/ou apresenta
mais de um estilo textual de forma dissonante (colcha de retalhos).

**[2,0 - 4,0]**
Documentação aborda todos os temas e apresenta-os com alguma coesão, dando um
entendimento ao menos parcial do projeto como um todo. Há poucas ou nenhuma
ocorrência de linguagem inadequada e o texto apresenta alguma consistência de
estilo.

**[4,0 - 5,0]**
Documentação completa e coesa do projeto desenvolvido. A linguagem utilizada é
condizente com o tipo de documento produzido.

#### 2.3.2 Requisitos (até 3,0 pontos)

**[0,0 - 0,5]**
Os requisitos levantados estão fora de contexto e/ou não refletem os objetivos
do parceiro e do projeto.

**[0,5 - 1,5]**
Os requisitos estão contextualizados com o objetivo do projeto, mas há poucas
ou nenhuma métrica e/ou planejamento de teste para aferir se foram atingidos.

**[1,5 - 2,5]**
Requisitos contextualizados e com métricas aferíveis em todos os requisitos não
funcionais. Falta detalhamento no planejamento dos testes.

**[2,5 - 3,0]**
Requisitos contextualizados, com métricas bem definidas e com o planejamento de
testes devidamente documentado e coerente com os requisitos.


#### 2.3.3 Instruções para execução (até 2,0 pontos)

**[0,0 - 0,5]**
Há pouca ou nenhuma instrução para execução do projeto em sua versão atual.

**[0,5 - 1,0]**
As instruções estão presentes e abordam de modo geral o que deve ser feito para
a execução do projeto, porém faltam detalhes que não podem ser facilmente
inferidos pelo leitor. As instruções apresentam uma falta de exemplos com
comandos claros para execução. Não há informações sobre compatibilidade do
software com versões de linguagens de programação, bibliotecas ou sistemas
operacionais.

**[1,0 - 1,5]**
Instruções completas e com comandos detalhados de execução, porém com poucos ou
nenhum dos exemplos mais comuns de utilização. Informações incompletas sobre
compatibilidade.

**[1,5 - 2,0]**
Instruções completas, com exemplos mais comuns, comandos detalhados de execução
e informações robustas de compatiblidade.
