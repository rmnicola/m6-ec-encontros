---
title: Sprint 2
sidebar_position: 3
sidebar_class_name: artefato
slug: /sprint2
---

# Artefatos Sprint 1

Esse é o sprint em que será necessário apresentar a primeira versão da **prova
de conceito**. Os artefatos orbitam em torno dessa implementação. O que o
sistema deve ser capaz de fazer ao final dessa sprint?

1. Comunicação por rede local com o robô;
2. Interface simplificada (terminal ou UI simples) para que um operador possa
   mover o robô à distância;
3. Operação limitada a quando o operador tem o robo dentro de seu campo de
   visão. Sem câmeras por enquanto.
4. Deve haver um sinal de emergência que o operador é capaz de enviar usando o
   ROS para que o robô pare imediatamente o que está fazendo.

Quais são os entregáveis?

* Robô operado à distância; e
* Documentação.

## 1. Robô operado à distância

### 1.1. Enunciado

Neste sprint, o grupo é desafiado a desenvolver e apresentar a primeira versão
da prova de conceito para um robô operado à distância. A comunicação com o robô
deve ser realizada através de uma rede local utilizando o ROS (Robot Operating
System). O operador deve ser capaz de controlar o robô utilizando uma interface
de usuário simplificada, que pode ser um terminal básico ou uma interface
gráfica simples, conforme a escolha da equipe. Importante destacar que, nesta
fase, a operação do robô deve ser realizada enquanto o operador mantém o robô
dentro de seu campo de visão direto, uma vez que o uso de câmeras não será
implementado até uma sprint futura. Além disso, deve ser implementado um
mecanismo de emergência eficaz, permitindo que o operador envie um comando de
parada imediata ao robô através do ROS em caso de qualquer eventualidade ou
risco observado.

### 1.2. Padrão de entrega

:::warning

O padrão de entrega do artefato exige a completa aderência aos requisitos
listados. A falta de cumprimento de qualquer um dos requisitos pode resultar em
penalidades na nota ou até mesmo na rejeição total do artefato entregue. É
essencial que todos os detalhes sejam meticulosamente seguidos.

:::

1. O código-fonte da solução deve estar integralmente disponível no repositório
   do grupo no GitHub, na branch `main`, dentro de um diretório chamado `src`.
       Para soluções compostas por múltiplos módulos, estes devem ser
       organizados em subdiretórios apropriados.
2. Os testes desenvolvidos para verificar a funcionalidade do código devem
   estar disponíveis no diretório `tests`. Testes automatizados são
   preferíveis, mas os procedimentos e resultados de testes manuais devem ser
   claramente documentados.
3. As instruções detalhadas para a execução do projeto devem ser facilmente
   acessíveis na documentação, com um link direto no arquivo README para
   facilitar a consulta.
4. A versão mais recente do projeto deve estar disponível como um release no
   GitHub, nomeado adequadamente com a referência à sprint atual, por exemplo,
   `release-sprint-1`.
5. As principais questões de desenvolvimento abordadas durante a sprint e os
   resultados dos testes devem ser claramente apresentados durante a revisão
   com os parceiros do projeto.

### 1.3. Padrão de qualidade

#### 1.3.1. Teleoperação (até 6,0 pontos)

**[0,0 - 1,0]**  
A teleoperação é ineficaz ou não funcionando, com falhas críticas que impedem
qualquer forma de controle remoto.

**[1,0 - 3,0]**  
O controle remoto do robô é possível, mas apresenta resposta lenta ou
inconsistente, limitando a eficácia da operação.

**[3,0 - 5,0]**  
A teleoperação é funcional com resposta estável e controle eficaz, mas ainda há
margem para refinamento na precisão e na reatividade.

**[5,0 - 6,0]**  
O controle remoto é excepcionalmente fluido e preciso, permitindo operações
detalhadas e reativas em tempo real.

#### 1.3.2. Interface de usuário (até 2,0 pontos)

**[0,0 - 0,5]**  
A interface é extremamente básica ou difícil de usar, afetando negativamente a
operação do robô.

**[0,5 - 1,5]**  
A interface é funcional, mas pode ser mais intuitiva ou detalhada para melhorar
a experiência do usuário.

**[1,5 - 2,0]**  
A interface é bem projetada, intuitiva e facilita o controle eficaz e imediato
do robô.

#### 1.3.3. Sistema de emergência (até 2,0 pontos)

**[0,0 - 0,5]**  
O sistema de emergência é inexistente ou falha ao ser acionado, não oferecendo
segurança adequada.

**[0 ,5 - 1,5]**  
O sistema de emergência funciona, mas pode ser mais rápido ou mais acessível
durante a operação.

**[1,5 - 2,0]**  
O sistema de emergência é altamente eficiente, com ativação imediata e fácil
acessibilidade, garantindo máxima segurança durante a operação.

## 2. Documentação 

### 2.1. Enunciado

O grupo deve registrar e analisar o desenvolvimento da primeira versão da prova
de conceito para o robô operado à distância. Este registro deve incluir as
decisões de design, desafios enfrentados e soluções adotadas.

### 2.2. Padrão de entrega

:::warning

O padrão de entrega do artefato inclui requisitos essenciais para a conclusão
da atividade. Embora não seja atribuída uma pontuação direta para cada item, o
não cumprimento pode resultar em uma redução da nota ou, em casos extremos, na
rejeição total do artefato entregue. Atenção é crucial.

:::

1. O texto desenvolvido pelo grupo deve estar disponível em uma página estática
   gerada pelo framework Docusaurus. A estrutura do repositório deve incluir um
   diretório denominado `docs`, que servirá como raiz para o Docusaurus.
2. A documentação da sprint 2 deve ser composta e organizada integralmente na
   seção `Sprint 2`. Cada artefato e sua respectiva descrição devem estar
   contidos em uma página ou subseção dentro desta seção.
3. As figuras utilizadas na documentação devem ser claramente referenciadas e
   descritas no texto para garantir uma conexão visual e contextual clara com o
   conteúdo documentado.
4. Todas as imagens empregadas na documentação devem ser armazenadas dentro do
   diretório `docs`, especificamente em um subdiretório chamado `static`.

### 2.3. Padrão de qualidade

#### 2.3.1 Qualidade textual (até 4,0 pontos)

:::warning

A qualidade textual avalia o conteúdo documentado durante a sprint corrente.
Defeitos textuais remanescentes de sprints anteriores também podem ser
penalizados pelo professor orientador.

:::

**[0,0 - 1,0]**
A documentação está fora de contexto ou significativamente incompleta.

**[1,0 - 2,0]**
A documentação cobre todos os artefatos obrigatórios, mas apresenta-os de forma
incoesa, dificultando a visão do objetivo global do projeto. O documento
utiliza uma linguagem inadequada para um contexto técnico ou exibe
inconsistência estilística.

**[2,0 - 3,0]**
A documentação cobre todos os temas necessários com alguma coesão, oferecendo
uma visão parcialmente integrada do projeto. A linguagem é adequada com mínimas
falhas, mantendo uma consistência de estilo.

**[3,0 - 4,0]**
A documentação é completa, coesa e detalhada, refletindo todos os aspectos do
projeto desenvolvido. A linguagem é apropriada e técnica, adequada ao tipo de
documento produzido.

#### 2.3.2 Metodologia (até 4,0 pontos)

**[0,0 - 1,0]**
A metodologia descrita é vaga ou praticamente inexistente, com falhas críticas
que impedem a compreensão do processo de desenvolvimento.

**[1,0 - 2,0]**
A metodologia é descrita, mas de forma superficial ou incompleta. Falta clareza
nos processos e nas etapas de desenvolvimento, e há pouca descrição das
técnicas e ferramentas utilizadas.

**[2,0 - 3,0]**
A metodologia está bem explicada e detalhada para a maioria das etapas de
desenvolvimento, com descrição adequada das técnicas e ferramentas utilizadas.
Algumas pequenas omissões podem estar presentes, mas não comprometem a
compreensão geral.

**[3,0 - 4,0]**
A metodologia é detalhadamente descrita, incluindo todas as etapas do
desenvolvimento do projeto. As técnicas e ferramentas são explicadas com
precisão, permitindo uma compreensão completa e replicação do processo.

#### 2.3.3 Instruções para execução (até 2,0 pontos)

**[0,0 - 0,5]**
Instruções para execução do projeto são escassas ou ausentes, deixando muitas
questões sem resposta.

**[0,5 - 1,0]**
As instruções de execução são básicas e gerais, faltando detalhes essenciais e
exemplos práticos para facilitar a compreensão.

**[1,0 - 1,5]**
Instruções detalhadas e claras são fornecidas, com alguns exemplos práticos. A
documentação inclui a maior parte das informações necessárias, mas pode ser
ligeiramente incompleta em relação à compatibilidade.

**[1,5 - 2,0]**
Instruções completas e detalhadas, incluindo exemplos comuns e comandos
específicos para execução. Informações detalhadas sobre compatibilidade com
diferentes linguagens de programação, bibliotecas e sistemas operacionais são
fornecidas.
