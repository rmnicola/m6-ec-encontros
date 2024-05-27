---
title: Sprint 4
sidebar_position: 5
sidebar_class_name: artefato
slug: /sprint4
---

Para a sprint 4, espera-se o desenvolvimento de três aspectos do projeto:

1. O sistema de visão computacional/inteligência artificial;
2. O backend e o banco de dados capaz de armazenar os dados coletados; e
3. Implementação de pelo menos um teste de funcionalidade e três testes de
   validação para requisitos não funcionais (pode revisar os requisitos
   escritos na sprint 1).

Sendo assim, os entregáveis se dividem em:

* Sistema de visão computacional e backend (peso 4);
* Implementação dos testes (peso 2); e
* Documentação (peso 2).

## 1. Sistema de visão computacional e backend

Agora que você tem um sistema capaz de permitir a um usuário pilotar o robô à
distância com transmissão de imagens em tempo real, vamos começar a coletar e
processar dados para auxiliar o operador do sistema. Para isso, será necessário
treinar e implementar um modelo de visão computacional, modelar e criar um
banco de dados para armazenar essas informações e uma API para que seja
possível acessar esses dados para posterior análise.

### 1.1. Padrão de entrega

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

### 1.2. Padrão de qualidade

#### 1.2.1. Modelo de visão computacional (até 4,0 pontos)

<table>
  <tr>
    <th>Supera<br/>(3,5 - 4,0)</th>
    <th>Atende<br/>(3,0 - 3,5)</th>
    <th>Quase lá<br/>(2,0 - 3,0)</th>
    <th>Insuficiente<br/>(0,5 - 2,0)</th>
    <th>Desalinhado<br/>(0,0 - 0,5)</th>
  </tr>
  <tr>
    <td>Além de ter sido ajustado para o problema do parceiro, o modelo foi
    validado com uma aplicação em tempo real e capturando imagens reais através
    da câmera do robô.</td>
    <td>O modelo implementado apresenta performance aceitável e passou pelo
    processo de fine tuning para que faça sentido no contexto do projeto.
    Alternativamente, o modelo foi treinado do zero com um dataset condizente
    com o problema do parceiro.</td>
    <td>O modelo implementado apresenta performance (latência) condizente com
    os objetivos do projeto, porém ele não está ajustado para o contexto do
    parceiro.</td>
    <td>Há um modelo de visão computacional implementado, porém ele apresenta
    resultados e/ou performance que não condizem com os objetivos do
    projeto.</td>
    <td>Entrega fora de contexto ou não entregou.</td>
  </tr>
</table>

#### 1.2.2. Banco de dados (até 2,0 pontos)

<table>
  <tr>
    <th>Atende I<br/>(1,0 - 2,0)</th>
    <th>Atende II<br/>(1,0 - 2,0)</th>
    <th>Quase lá I<br/>(0,5 - 1,0)</th>
    <th>Quase lá II<br/>(0,5 - 1,0)</th>
    <th>Desalinhado<br/>(0,0 - 0,5)</th>
  </tr>
  <tr>
    <td>O banco de dados implementado é coerente com o armazenamento de
    documentos e apresenta uma modelagem clara para os dados que devem ser
    inseridos nele.</td>
    <td>Há um banco de dados relacional implementado com normalização até 2NF
    no mínimo.</td>
    <td>O banco de dados não relacional foi implementado corretamente, mas sem
    qualquer preocupaçõa com a modelagem dos documentos nele inseridos.</td>
    <td>Há um banco de dados relacional implementado, mas ele não apresenta
    normalização alguma nas tabelas criadas.</td>
    <td>Entrega fora de contexto ou não entregou.</td>
  </tr>
</table>

#### 1.2.3. API e Backend (até 4,0 pontos)

<table>
  <tr>
    <th>Supera<br/>(3,5 - 4,0)</th>
    <th>Atende<br/>(3,0 - 3,5)</th>
    <th>Quase lá<br/>(2,0 - 3,0)</th>
    <th>Insuficiente<br/>(0,5 - 2,0)</th>
    <th>Desalinhado<br/>(0,0 - 0,5)</th>
  </tr>
  <tr>
    <td>Além do backend implementado com as rotas necessária para GET, POST e
    DELETE, o grupo ainda fez e documentou testes de cada uma das rotas
    utilizando ferramentas como Postman ou Insomnia.</td>
    <td>O backend foi implementado e possui as rotas necessárias para
    implementar as funcionalidades do sistema. Ele está documentado e utiliza
    os verbos adequados do HTTP, de acordo com a funcionalidade esperada (GET
    para obter dados, POST para enviar dados, DELETE para deletar dados, apenas
    para citar alguns).</td>
    <td>O backend foi implementado e possui as rotas necessárias para
    implementar as funcionalidades do sistema. Contudo, não existe nenhuma
    documentação das rotas. Todas elas estão implementadas utilizando apenas o
    verbo POST do HTTP.</td>
    <td>Existiu o início da construção da aplicação backend, mas ela não foi
    além de um template de código com a implementação de algumas rotas, como a
    rota de hello-world a uma rota de echo.</td>
    <td>Entrega fora de contexto ou não entregou.</td>
  </tr>
</table>

## 2. Testes

Deve-se implementar ao menos:

* Um teste de funcionalidade (requisito funcional) relacionado à operação do
  robô; e
* Três testes de validação de requisitos não funcionais.

### 2.1. Padrão de entrega

### 2.2. Padrão de qualidade

#### 2.2.1. Teste de funcionalidade (até 6,0 pontos)

<table>
  <tr>
    <th>Supera<br/>(5,0 - 6,0)</th>
    <th>Atende<br/>(4,0 - 5,0)</th>
    <th>Quase lá<br/>(2,5 - 4,0)</th>
    <th>Insuficiente<br/>(0,5 - 2,5)</th>
    <th>Desalinhado<br/>(0,0 - 0,5)</th>
  </tr>
  <tr>
    <td>Além do teste coerente e relatório detalhado, o teste foi feito com ao
    menos 4 usuários diferentes.</td>
    <td>O teste implementado é consistente com a funcionalidade e o roteiro e o
    relatório do teste foi detalhado, com anotações detalhando as interações do
    usuário com o sistema e conclusões sólidas que levam à propostas melhoria
    do sistema desenvolvido.</td>
    <td>O teste implementado está consistente com a funcionalidade descrita e o
    roteiro de teste definido, porém faltam detalhes no relatório de execução
    dos testes e/ou nas conclusões tiradas a partir dos testes.</td>
    <td>Há inconsistências significativas entre o teste realizado e a descrição
    de funcionalidade feita e/ou o roteiro de teste definido.</td>
    <td>Entrega fora de contexto ou não entregou.</td>
  </tr>
</table>

#### 2.2.2. Validação de RNFs (até 4,0 pontos)

<table>
  <tr>
    <th>Supera<br/>(3,5 - 4,0)</th>
    <th>Atende<br/>(2,0 - 3,5)</th>
    <th>Quase lá<br/>(1,5 - 2,5)</th>
    <th>Insuficiente<br/>(0,5 - 1,5)</th>
    <th>Desalinhado<br/>(0,0 - 0,5)</th>
  </tr>
  <tr>
    <td>Além dos testes estarem de acordo com os RNFs apresentados, o grupo
    implementou melhorias aos sistemas que não atendiam os requisitos descritos
    de modo a melhorar suas métricas (caso todos os RNFs passem de primeira,
    deve-se melhorar a métrica de ao menos um dos sistemas medidos)</td>
    <td>Todos os testes estão de acordo com os RNFs relacionados.</td>
    <td>Ao menos dois dos três testes apresentados verdadeiramente medem os
    RNFs descritos. Isso significa apresentar claramente uma descrição
    detalhada dos testes feitos e um relatório de sua execução.</td>
    <td>Os testes apresentados possuem inconsistências significativas técnicas
    e/ou não medem verdadeiramente os RNFs descritos.</td>
    <td>Entrega fora de contexto ou não entregou.</td>
  </tr>
</table>

## 3. Documentação 

Registro do processo de desenvolvimento dos artefatos da sprint. Os itens
obrigatórios para uma boa documentação são:

1. Manter uma **qualidade textual** adequada, sem cometer erros graves de
   português e nem utilizar uma linguagem completamente inadequada.
2. Apresentar de forma clara as **instruções para execução** do projeto. Esse
   item é importante para que o parceiro consiga utilizar sua solução.
3. Descrever a **metodologia** de desenvolvimento do projeto. Aqui deve ficar
   claro quais foram as principais decisões de desenvolvimento e **como** vocês
   implementaram o sistema. Pense que o cliente pode decidir integrar o projeto
   de vocês no sistema dele e isso só é possível se ele for capaz de
   compreender completamente como o sistema foi construído.

### 3.1. Padrão de entrega

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

### 3.2. Padrão de qualidade

#### 3.2.1 Qualidade textual (até 2,0 pontos)

:::warning

A qualidade textual avalia o conteúdo documentado durante a sprint corrente.
Defeitos textuais remanescentes de sprints anteriores também podem ser
penalizados pelo professor orientador.

:::

<table>
  <tr>
    <th>Atende<br/>(1,5 - 2,0)</th>
    <th>Quase lá<br/>(1,0 - 1,5)</th>
    <th>Insuficiente<br/>(0,5 - 1,0)</th>
    <th>Desalinhado<br/>(0,0 - 0,5)</th>
  </tr>
  <tr>
    <td>Texto correto, coeso e com fluidez. O grupo demonstrou preocupação em
    apresentar os conceitos da forma mais didática possível, escolhendo
    cuidadosamente quando e como utilizar tabelas e imagens.</td>
    <td>Texto razoavelmente bem escrito, com poucos problemas de adequação da
    linguagem ou erros gramaticais.</td>
    <td>O texto escrito tem problemas graves de linguagem, apresentando erros
    gramaticais graves e constantes e/ou apresenta linguagem inadequada para um
    texto técnico.</td>
    <td>Entrega fora de contexto ou não entregou.</td>
  </tr>
</table>

#### 3.2.2 Instruções para execução (até 2,0 pontos)

<table>
  <tr>
    <th>Supera<br/>(1,75 - 2,0)</th>
    <th>Atende<br/>(1,25 - 1,75)</th>
    <th>Quase lá<br/>(0,75 - 1,25)</th>
    <th>Insuficiente<br/>(0,25 - 0,75)</th>
    <th>Desalinhado<br/>(0,0 - 0,25)</th>
  </tr>
  <tr>
    <td>Instruções completas e apresentadas de forma amigável ao usuário. Ficou
    claro que o grupo considerou genuinamente as dúvidas que podem surgir e a
    melhor forma de apresentá-las (imagens, vídeos, texto, trechos de
    código)</td>
    <td>Todas as principais ações do usuário foram contempladas no guia de
    execução do sistema.</td>
    <td>As instruções para execução do projeto estão presentes, mas ainda
    bastante incompletas. O usuário vai precisar procurar informações por fora
    para conseguir utilizar o sistema.</td>
    <td>Instruções escassas ou ausentes, de modo que o usuário não consiga
    extrair praticamente nada do guia.</td>
    <td>Entrega fora de contexto ou não entregou.</td>
  </tr>
</table>

#### 3.2.2 Metodologia (até 6,0 pontos)
<table>
  <tr>
    <th>Supera<br/>(5,5 - 6,0)</th>
    <th>Atende<br/>(4,5 - 5,5)</th>
    <th>Quase lá<br/>(2,5 - 4,5)</th>
    <th>Insuficiente<br/>(0,5 - 2,5)</th>
    <th>Desalinhado<br/>(0,0 - 0,5)</th>
  </tr>
  <tr>
    <td>Não só todas as etapas foram descritas de forma precisa e sem omitir
    etapas, como ficou claro que o grupo se preocupou em utilizar recursos para
    facilitar a compreensão da metodologia. Imagens, vídeos, trechos de código
    foram usados para as partes mais complexas.</td>
    <td>Todas as etapas do desenvolvimento foram descritas com precisão e de
    forma completa.</td>
    <td>Quase todas as etapas foram descritas com precisão e de forma completa.
    No entanto, ainda tem um ou mais etapas mal definidas, de modo que ainda
    não seja possível reproduzir o projeto apenas com a leitura dessa
    seção</td>
    <td>As etapas de desenvolvimento foram descritas de forma superficial ou
    incompleta. Falta clareza nos processos e etapas de desenvolvimento. É
    impossível reproduzir o projeto apenas seguindo as etapas descritas no
    texto.</td>
    <td>Entrega fora de contexto ou não entregou.</td>
  </tr>
</table>
