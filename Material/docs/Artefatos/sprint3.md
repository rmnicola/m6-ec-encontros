---
title: Sprint 3
sidebar_position: 4
sidebar_class_name: artefato
slug: /sprint3
---

O sprint 3 é marcado por duas principais modificações ao projeto:

1. Uma nova **interface de usuário**, agora adicionando ao usuário a
   possibilidade de ver em tempo real o feed da câmera acoplada ao robô.
2. A criação de uma **análise financeira** para que o parceiro saiba quanto vai
   custar a **prova de conceito** e também a **versão final** do projeto.

Os entregáveis se dividem em:

* Análise Financeira (peso 2);
* Evolução da interface de usuário (peso 4);
* Documentação (peso 2).

## 1. Análise financeira

Agora que já foi possível implementar a versão mínimamente viável do projeto,
estamos aptos a prever o custo de implementação do projeto considerando duas
situações:

1. A PoC que o grupo está desenvolvendo; e
2. A versão final do projeto, considerando a operação sem nenhuma das
   simplificações impostas pelo kit robótico educacional.

### 1.1. Padrão de entrega

1. O texto desenvolvido pelo grupo deve estar disponível em uma página estática
   gerada pelo framework Docusaurus. Para tal, deve haver um diretório no
   repositório denominado docs, onde ficará a raíz do Docusaurus;
2. A documentação da sprint 3 deve estar inteiramente contida em uma seção
   denominada Sprint 3. Cada um dos artefatos deve ter sua descrição contida em
   uma página ou subseção dentro da seção Sprint 3; e
3. As figuras utilizadas no documento devem sempre ser referenciadas no texto,
   com descrições textuais que estimulem a coesão entre o que é apresentado
   visualmente e o restante do texto.

### 1.2. Padrão de qualidade

#### 1.2.1 Análise da PoC (até 5,0 pontos)

<table>
  <tr>
    <th>Supera<br/>(4,5 - 5,0)</th>
    <th>Atende<br/>(3,5 - 4,5)</th>
    <th>Quase lá<br/>(2,5 - 3,5)</th>
    <th>Insuficiente<br/>(0,5 - 2,5)</th>
    <th>Desalinhado<br/>(0,0 - 0,5)</th>
  </tr>
  <tr>
    <td>Além de uma análise financeira que atende a todos os requisitos, o
    grupo considerou onde e como pode fazer alterações para diminuir o valor da
    proposta, definindo claramente um valor mínimo aceitável para a venda do
    projeto.</td>
    <td>Análise com fontes, principais custos delineados e com valores
    verossímeis. Além disso, o grupo considerou o lucro desejado e a incidência
    correta de impostos (deve apresentar CNAE considerado).</td>
    <td>A análise feita apresenta claramente os principais custos de projeto,
    definindo claramente a fonte utilizada para cada uma das informações
    apresentadas. Os valores apresentados são verossímeis.</td>
    <td>A análise é rasa, apresentando números que aparecem sem uma
    justificativa e ignoram as boas práticas da análise financeira.</td>
    <td>Entrega fora de contexto ou não entregou.</td>
  </tr>
</table>

#### 1.2.2. Análise do projeto final (até 5,0 pontos)

<table>
  <tr>
    <th>Supera<br/>(4,5 - 5,0)</th>
    <th>Atende<br/>(3,5 - 4,5)</th>
    <th>Quase lá<br/>(2,5 - 3,5)</th>
    <th>Insuficiente<br/>(0,5 - 2,5)</th>
    <th>Desalinhado<br/>(0,0 - 0,5)</th>
  </tr>
  <tr>
    <td>Além de uma análise financeira que atende a todos os requisitos, o
    grupo considerou onde e como pode fazer alterações para diminuir o valor da
    proposta, definindo claramente um valor mínimo aceitável para a venda do
    projeto.</td>
    <td>Análise com fontes, principais custos delineados e com valores
    verossímeis. Além disso, o grupo considerou o lucro desejado e a incidência
    correta de impostos (deve apresentar CNAE considerado).</td>
    <td>A análise feita apresenta claramente os principais custos de projeto,
    definindo claramente a fonte utilizada para cada uma das informações
    apresentadas. Os valores apresentados são verossímeis.</td>
    <td>A análise é rasa, apresentando números que aparecem sem uma
    justificativa e ignoram as boas práticas da análise financeira.</td>
    <td>Entrega fora de contexto ou não entregou.</td>
  </tr>
</table>

## 2. Evolução da interface de usuário

A evolução da interface de usuário deve contemplar o seguinte:

1. Alteração da interface de usuário considerando o feedback do parceiro e
   adicionando o feed de vídeo em tempo real;
2. Aprimoramento do sistema de segurança;
3. Implementação do software para transmissão de vídeo em tempo real; e
4. Sugestão de gráficos a partir das informações coletadas pelo sistema
   (sugestões de BI).

### 2.1. Padrão de entrega

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

### 2.2. Padrão de qualidade

#### 2.2.1. Interface de usuário (até 3,0 pontos)

<table>
  <tr>
    <th>Supera<br/>(2,5 - 3,0)</th>
    <th>Atende<br/>(2,0 - 2,5)</th>
    <th>Quase lá<br/>(1,0 - 2,0)</th>
    <th>Insuficiente<br/>(0,5 - 1,0)</th>
    <th>Desalinhado<br/>(0,0 - 0,5)</th>
  </tr>
  <tr>
    <td>Além da funcionalidade básica do sistema, a interface ainda apresenta a
    informação da latência estimada do processo de processamento e transmissão
    de imagens para cada frame.</td>
    <td>Interface completa, incluindo o feed de imagem transmitido em tempo
    real.</td>
    <td>A transmissão de vídeo é feita, mas de forma que não se integra com a
    tela do programa, mas sim em outra tela que abre ao iniciar o
    processo.</td>
    <td>A interface de usuário apresenta formas para o usuário comendar o robô
    em tempo real, incluindo um botão de parada emergencial que atua
    imediatamente e apresentação do status do robô em tempo real. Não há, no
    entanto, nada implementado para o vídeo</td>
    <td>Entrega fora de contexto ou não entregou.</td>
  </tr>
</table>

#### 2.2.2. Sistema de segurança (até 3,0 pontos)

<table>
  <tr>
    <th>Supera<br/>(2,5 - 3,0)</th>
    <th>Atende<br/>(2,0 - 2,5)</th>
    <th>Quase lá<br/>(1,0 - 2,0)</th>
    <th>Insuficiente<br/>(0,5 - 1,0)</th>
    <th>Desalinhado<br/>(0,0 - 0,5)</th>
  </tr>
  <tr>
    <td>O sistema é capaz de detectar a direção onde está o obstáculo e, assim,
    permite que o usuário envie comandos para que o robô se mova para longe do
    obstáculo, mas trava o movimento do robô em direção ao obstáculo.</td>
    <td>O sistema não só envia um alerta para o usuário quando há um objeto
    próximo, como faz com que o robô não possa mais se mover através de
    comandos de velocidade quando há um obstáculo muito próximo.</td>
    <td>Além do botão de emergência funcional, a interface ainda apresenta
    alertas para o usuário quando a leitura do LiDAR acusa um objeto muito
    próximo do robô (e.g. uma parede)</td>
    <td>O botão de emergência foi integrado à interface de usuário e modificado
    para agir como um serviço ROS. O cliente envia para o robô um pedido de
    parada e a resposta é um status de encerramento do processo do
    turtlebot.</td>
    <td>Entrega fora de contexto ou não entregou.</td>
  </tr>
</table>

#### 2.2.3. Transmissão de imagens (até 3,0 pontos)

<table>
  <tr>
    <th>Supera<br/>(2,5 - 3,0)</th>
    <th>Atende<br/>(2,0 - 2,5)</th>
    <th>Quase lá<br/>(1,0 - 2,0)</th>
    <th>Insuficiente<br/>(0,5 - 1,0)</th>
    <th>Desalinhado<br/>(0,0 - 0,5)</th>
  </tr>
  <tr>
    <td>Além da transmissão da imagem, o sistema ainda consegue estimar a
    enviar a latência adicionada no processo de aquisição, processamento e
    envio da imagem</td>
    <td>A imagem é transmitida e recebida utilizando qualquer tecnologia (e.g.
    tópico ROS, websocket, porta UDP).</td>
    <td>A transmissão da imagem é feita, mas não há um tratamento adequado do
    recebimento para integrar a imagem na interface de usuário.</td>
    <td>Há um sistema criado capaz de interagir com o ROS e comandar o robô
    utilizando o tópico de velocidades, receber feedback como subscriber de um
    tópico e um serviço para realizar as paradas de emergência. Não há, no
    entanto, um sistema específico para a transmissão de imagens.</td>
    <td>Entrega fora de contexto ou não entregou.</td>
  </tr>
</table>

#### 2.2.4. Gráficos sugeridos (até 1,0 ponto)

<table>
  <tr>
    <th>Atende<br/>(0,75 - 1,0)</th>
    <th>Quase lá<br/>(0,5 - 0,75)</th>
    <th>Insuficiente<br/>(0,25 - 0,5)</th>
    <th>Desalinhado<br/>(0,0 - 0,25)</th>
  </tr>
  <tr>
    <td>Além da descrição clara do que estará disponível em sua API, o grupo
    apresentou sugestões de gráficos para o cliente implementar em sua
    plataforma de BI</td>
    <td>O grupo apresentou de forma clara os dados que ficarão disponíveis em
    sua API, mas não há sugestões de gráficos que o parceiro pode implementar
    em sua ferramenta de BI</td>
    <td>Existe uma análise dos dados armazenados pelo sistema e que ficarão
    disponíveis em uma API, mas ela está incompleta.</td>
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
2. A documentação da sprint 3 deve ser composta e organizada integralmente na
   seção `Sprint 3`. Cada artefato e sua respectiva descrição devem estar
   contidos em uma página ou subseção dentro desta seção.
3. As figuras utilizadas na documentação devem ser claramente referenciadas e
   descritas no texto para garantir uma conexão visual e contextual clara com o
   conteúdo documentado.
4. Todas as imagens empregadas na documentação devem ser armazenadas dentro do
   diretório `docs`, especificamente em um subdiretório chamado `static`.

### 3.2. Padrão de qualidade

#### 3.2.1 Qualidade textual (até 4,0 pontos)

:::warning

A qualidade textual avalia o conteúdo documentado durante a sprint corrente.
Defeitos textuais remanescentes de sprints anteriores também podem ser
penalizados pelo professor orientador.

:::

<table>
  <tr>
    <th>Supera<br/>(3,5 - 4,0)</th>
    <th>Atende<br/>(2,5 - 3,5)</th>
    <th>Quase lá<br/>(1,5 - 2,5)</th>
    <th>Insuficiente<br/>(0,5 - 1,5)</th>
    <th>Desalinhado<br/>(0,0 - 0,5)</th>
  </tr>
  <tr>
    <td>Texto correto, coeso e com fluidez. O grupo demonstrou preocupação em
    apresentar os conceitos da forma mais didática possível, escolhendo
    cuidadosamente quando e como utilizar tabelas e imagens.</td>
    <td>Texto revisado e com quase nenhuma ocorrência de linguagem inadequada
    ou erros gramaticais. O texto é conciso, ligando de forma adequada os temas
    apresentados com fluidez.</td>
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

#### 3.2.2 Metodologia (até 4,0 pontos)
<table>
  <tr>
    <th>Supera<br/>(3,5 - 4,0)</th>
    <th>Atende<br/>(2,5 - 3,5)</th>
    <th>Quase lá<br/>(1,5 - 2,5)</th>
    <th>Insuficiente<br/>(0,5 - 1,5)</th>
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
