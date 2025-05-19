---
title: Sprint 3
sidebar_position: 4
sidebar_class_name: artefato
slug: /sprint3
---

# Artefatos Sprint 3

Para um desenvolvimento verdadeiramente agil, a sprint 3 deve ter apenas duas
entregas obrigatórias:

1. Implementação técnica; e
2. Documentação

Para isso, considerem o que pode ser implementado de modo que, ao final de cada
sprint, exista **um sistema funcional** com **clara evolução** com relação à
sprint passada. Percebam que essa evolução não pode ser algo desacoplado da
motivação do projeto. Sendo assim, toda e qualquer evolução deve ser
justificada levando em consideração a proposta de valor e as personas
desenvolvidos pelo grupo na sprint 1. Reconsiderar o escopo do projeto quando
necessário é mais do que permitido, **é recomendável**. No entanto, essa
mudança deve ser **claramente explicada** na documentação do projeto. Caso
contrário, será considerado que o novo escopo está fora do contexto definido
pelo grupo.

Como forma de auxiliar no processo de tomada de decisão,
considerem as tarefas que podem estar no backlog de projeto (por favor, usem o
bom senso para decidir se: 1. o que está aqui é o suficiente para o projeto de
vocês e 2. se o que está aqui sequer se encaixa no projeto de vocês):

* Teste e validação da locomoção do robô/drone
* Treinamento do sistema de visão computacional
* Transmissão de imagens em tempo real
* Sistemas de mensageria em tempo real (e.g. ROS)
* Interface de usuário
* Armazenamento de dados de inferência e/ou modelo e/ou imagens
* API para interface com o(s) sistema(s) de armazenamento de dados
* Deploy do modelo de visão computacional (hospedagem e armazenamento)
* Esteira de desenvolvimento para o modelo de visão

Notem que o que está acima são **exemplos** que podem ou não se encaixar no
projeto de vocês. Além disso, a maioria dessas tarefas é complexa demais para
ser apenas um item de backlog. Por fim, notem também que a maioria dessas
tarefas permite implementações incrementais (e.g. interface de usuário primeiro
em terminal, depois em um web app; modelo de visão usando apenas detecção da
YOLO, depois modelos de segmentação ou classificação)

## 1. Padrão de entrega

### 1.1. Implementação técnica/código

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

### 1.2. Documentação

:::warning

O padrão de entrega do artefato inclui requisitos essenciais para a conclusão
da atividade. Embora não seja atribuída uma pontuação direta para cada item, o
não cumprimento pode resultar em uma redução da nota ou, em casos extremos, na
rejeição total do artefato entregue. Atenção é crucial.

:::

1. O texto desenvolvido pelo grupo deve estar disponível em uma página estática
   gerada pelo framework Docusaurus. A estrutura do repositório deve incluir um
   diretório denominado `docs`, que servirá como raiz para o Docusaurus.
2. A documentação da sprint 3 deve ser composta e organizada integralmente na
   seção `Sprint 3`. Cada artefato e sua respectiva descrição devem estar
   contidos em uma página ou subseção dentro desta seção.
3. As figuras utilizadas na documentação devem ser claramente referenciadas e
   descritas no texto para garantir uma conexão visual e contextual clara com o
   conteúdo documentado.
4. Todas as imagens empregadas na documentação devem ser armazenadas dentro do
   diretório `docs`, especificamente em um subdiretório chamado `static`.

## 2. Padrão de qualidade

### 2.1. Implementação técnica (peso 5)

**[0,0 - 1,0]**  
Implementação fora do contexto do projeto ou insuficiente.

**[1,0 - 6,0]**  
A implementação feita pelo grupo está dentro do contexto do projeto, mas há
falhas como:
* O sistema apresenta **falhas de funcionamento**;
* Não ficou completamente claro como essa evolução do sistema pode **trazer
valor** para o parceiro;
* A implementação apresentada não está inteiramente contida no **repositório do
grupo**;
* O sistema **evoluiu pouco** com relação à entrega anterior;

**[6,0 - 9,0]**  
A entrega é relevante e apresenta evolução clara, mas ainda não está perfeita.
Alguns exemplos do que pode estar faltando:
* O sistema está funcional, mas ainda há **bugs pontuais** que afetam as principais funcionalidades;
* Código com **gambiarra**, sem usar boas práticas de arquitetura e desenvolvimento de software;
* Pequenas partes do sistema ficaram de fora do **release**;
* A **apresentação** ao parceiro não foi **clara** por não mostrar o sistema
funcional ou por não deixar clara a proposta de valor da implementação;

**[9,0 - 10,0]**  
A implementação feita pelo grupo:
* Contem um sistema **funcional**;
* Entrega **valor** para o parceiro;
* Está inteiramente contida no repositório do grupo **em um release**; 
* Apresenta uma **evolução significativa** sobre a última entrega;
* Foi **apresentada ao parceiro** de forma clara, permitindo o seu feedback.

### 2.2. Documentação (peso 3)

**[0,0 - 1,0]**
A documentação está fora de contexto ou significativamente incompleta.

**[1,0 - 6,0]**
A documentação apresentada está dentro do contexto do projeto, mas há falhas
graves como:
* Não apresenta **instruções de uso** do sistema;
* Apresenta o sistema de forma incompleta;
* Está **mal escrita**, com problemas claros de coesão e/ou frequentes erros
gramaticais;

**[6,0 - 9,0]**
A documentação está minimamente bem escrita, apresenta instruções de uso e
apresenta todos os novos elementos do sistema, mas ainda há imperfeições.
Exemplos:
* Não está suficientemente claro o **escopo** da entrega;
* A **proposta de valor** da entrega não está clara;
* Há falhas nas **instruções de uso** apresentadas;
* Apresenta elementos textuais/não-textuais desnecessários, se tornando
**prolixa** e difícil de ler;
* O texto **não está coeso**. Cada seção tem uma forma de escrita distinta e/ou
não há uma sequência lógica de uma seção para a outra;
* Não apresenta de forma clara quando há **mudanças de escopo** no projeto;

**[9,0 - 10,0]**
A documentação apresentada:
* Define claramente o **escopo** da entrega da sprint;
* Explica as **decisões arquiteturais** de implementação do sistema;
* Apresenta de forma clara as **instruções de uso** da versão atual do sistema;
* Deixa clara a **proposta de valor** da entrega;
* Apresenta claramente as **mudanças de escopo** propostas (quando cabível);
* Utiliza **português correto**, prezando pela **coesão textual**;
* É **concisa** na medida certa.
