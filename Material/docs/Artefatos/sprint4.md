---
title: Sprint 4
sidebar_position: 5
sidebar_class_name: artefato
slug: /sprint4
---

# Artefatos Sprint 4

A sprint 4 conta com a seguinte estrutura de entregáveis:

1. Implementação técnica; e
2. Documentação, sendo que aqui é obrigatório ter dois resultados em
   particular:

    2.1. Análise financeira do projeto; e
    2.2. Relatório de testes com o usuário.

Ao final desta sprint, deve existir **um sistema funcional** com **clara
evolução** com relação à sprint passada. Percebam que essa evolução não pode
ser algo desacoplado da motivação do projeto. Sendo assim, toda e qualquer
evolução deve ser justificada levando em consideração a proposta de valor e as
personas desenvolvidos pelo grupo na sprint 1. Reconsiderar o escopo do projeto
quando necessário é mais do que permitido, **é recomendável**. No entanto, essa
mudança deve ser **claramente explicada** na documentação do projeto. Caso
contrário, será considerado que o novo escopo está fora do contexto definido
pelo grupo.

A **Análise financeira** deve fornecer insumos para o estudo da viabilidade do
*projeto final* (**NÃO da prova de conceito**). Sendo assim, deve-se estimar:
* Tempo de desenvolvimento e/ou implantação e suas despesas relacionadas;
* Custo com funcionários e infraestrutura;
* Investimentos em equipamentos e seu tempo de duração;
* Custo de manutenção;
* Resultados esperados (por economia e/ou novas fontes de renda);

Para isso, pode ser necessário validar algumas informações com o parceiro de
projeto. Sabendo que nem sempre estas informações vão existir ou serem
disponibilizadas, entende-se que a *estimativa* do grupo pode tomar outras
fontes como forma de embasamento. O que é imprescindível é que **haja um
embasamento** para a estimativa. Este aspecto será considerado na avaliação do
projeto.

O **Relatório de testes** deve contemplar:
* A revisão dos requisitos funcionais e não funcionais, caso seja necessário;
* O planejamento detalhado dos testes, separando com clareza aqueles que
dependem de validação do usuário daqueles que tratam-se de testes objetivos de
validação de métricas;
* Relatórios de implementação de cada um dos testes executados e os seus
resultados; e
* Uma análise crítica dos resultados obtidos, discorrendo sobre a adequação do
sistema desenvolvido e possíveis pontos de melhoria.

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
2. A documentação da sprint 4 deve ser composta e organizada integralmente na
   seção `Sprint 4`. Cada artefato e sua respectiva descrição devem estar
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

#### 2.2.1. Documentação base (até 4,0 pontos)

**[0,0 - 0,5]**
A documentação está fora de contexto ou significativamente incompleta.

**[0,5 - 2,0]**
A documentação apresentada está dentro do contexto do projeto, mas há falhas
graves como:
* Não apresenta **instruções de uso** do sistema;
* Apresenta o sistema de forma incompleta;
* Está **mal escrita**, com problemas claros de coesão e/ou frequentes erros
gramaticais;

**[2,0 - 3,5]**
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

**[3,5 - 4,0]**
A documentação apresentada:
* Define claramente o **escopo** da entrega da sprint;
* Explica as **decisões arquiteturais** de implementação do sistema;
* Apresenta de forma clara as **instruções de uso** da versão atual do sistema;
* Deixa clara a **proposta de valor** da entrega;
* Apresenta claramente as **mudanças de escopo** propostas (quando cabível);
* Utiliza **português correto**, prezando pela **coesão textual**;
* É **concisa** na medida certa.

#### 2.2.2. Análise financeira (até 3,0 pontos)

**[0,0 - 0,5]**
A análise financeira está fora de contexto ou significativamente incompleta.

**[0,5 - 1,5]**
A análise financeira apresentada não torna clara a proposta de valor do projeto
por um ou mais dos motivos abaixo:
* Os resultados esperados não são realistas e/ou não tem base;
* Os custos/investimentos/despesas estimados não são realistas e/ou não tem
base;
* O cronograma apresentado não é realista e/ou não tem base; 

**[1,5 - 2,5]**
A análise financeira é sólida, mas há falhas conceituais e/ou inconsistências
tais como:
* Confusão entre custos, investimentos e despesas;
* Não diferencia claramente as etapas de p&d e implantação dos custos
operacionais;
* Algumas estimativas apresentadas não são verossímeis (sem grande impacto no
resultado da análise);

**[2,5 - 3,0]**
A análise financeira apresentada:
* Contém uma estimativa embasada do **cronograma** de desenvolvimento e/ou
implantação do projeto;
* Diferencia claramente a **previsão de custos, investimentos e despesas**,
oferecendo justificativas embasadas para cada um deles;
* Apresenta uma composição de **equipe de desenvolvimento e operacional** com
uma *justificativa* para cada um dos integrantes;
* Demonstra claramente o valor do projeto através dos resultados esperados;

#### 2.2.3. Relatório de testes

**[0,0 - 0,5]**
O relatório de testes está fora de contexto ou significativamente incompleta.

**[0,5 - 1,5]**
Os testes propostos apresentam falhas significativas, como:
* Não contemplam os requisitos do projeto adequadamente;
* Falta clareza em seu planejamento;
* Não há evidências de sua execução conforme o planejado;

**[1,5 - 2,5]**
Os testes propostos são coerentes com os requisitos de projeto e foram
executados conforme o planejado. No entanto, há problemas a serem resolvidos,
como:
* A análise dos resultados é rasa, limitando-se a constatar se o projeto atende
ou não os requisitos sem que haja uma discussão das causas e consequências
envolvidas;
* O plano de melhorias é genérico e/ou pouco realista;
* Nem todos os testes apresentam evidências claras de sua execução;
* Os testes não foram feitos com usuários que atendem o perfil descrito pelas
personas do projeto;

**[2,5 - 3,0]**
O relatório de testes contempla:
* A revisão dos requisitos do projeto;
* Um plano de testes com usuários com um roteiro bem definido e uma expectativa
clara do que cada teste deve avaliar;
* Um plano de testes para validação de métricas de sistema;
* O relatório completo de execução dos testes;
* Uma análise coerente que aponta aspectos de adequação do sistema aos
requisitos propostos;
* Melhorias planejadas a partir da análise;
