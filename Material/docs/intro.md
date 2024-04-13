---
title: Introdução
sidebar_position: 1
slug: /
---

# Robótica móvel e visão computacional


<img 
  src="https://media1.tenor.com/m/2Fpxuz_VTKcAAAAC/mutta-nanba-nanba-mutta.gif"
  alt="Muta says hi" 
  style={{ 
    display: 'block',
    marginLeft: 'auto',
    maxHeight: '30vh',
    marginRight: 'auto'
  }} 
/>
<br/>

> "本気の失敗には、価値がある"
>
> "An earnest mistake has value."
>
> — Nanba Mutta (Space Brothers)

*Um erro cometido com esforço tem valor*. 

É com esse pensamento que dou as boas vindas ao material de computação do
**módulo 6** do curso de **Engenharia de Computação** do Inteli. Esse é o lugar
em que você vai encontrar tudo o que precisa saber sobre os desafios
relacionados a esse módulo, cujo tema é **robótica móvel e visão
computacional**. O que significa ter **tudo** o que você precisa saber?
Significa que aqui você vai encontrar:

* A descrição geral do módulo e **especificações do metaprojeto**;
* Um descritivo detalhado dos **artefatos** de cada sprint, assim como seu **padrão de qualidade**;
* Guias para **configurar** o seu **sistema** com as **ferramentas** sugeridas para o desafio do módulo;
* Todo o **material** dos **encontros de orientação**, assim como os seus **autoestudos**;
* Todo o **material** dos **encontros de programação**, assim como os seus **autoestudos**; e
* Os **enunciados** detalhados das **atividades ponderadas** de **computação** propostas no decorrer do módulo.

O objetivo dessa seção em que estamos agora é dar a descrição geral do módulo e
te ensinar a encontrar todas as outras informações prometidas acima. Sendo
assim, essa seção subdivide-se em:

1. Descrição do módulo e assuntos abordados;
2. Organização do material;
3. Descrição dos entregáveis por sprint; e
4. Bibliografia.

Antes de seguirmos em frente com o conteúdo principal dessa seção, vou
aproveitar que tenho sua atenção para fazer um **anúncio** de **extrema
importância**. Se liga, que logo abaixo vem o admonition:

:::danger

Esse material **NÃO** substitui de forma alguma o uso da **Adalove**. Você
**DEVE** entrar na Adalove com frequência e **REGISTRAR O SEU PROGRESSO**.
Entendeu? Ainda não? Pera aí que vou desenhar:

<img 
  src="img/assets/aviso-adalove.png"
  alt="ACESSE A ADALOVE" 
  style={{ 
    display: 'block',
    marginLeft: 'auto',
    maxHeight: '40vh',
    marginRight: 'auto'
  }} 
/>

:::

Beleza, agora podemos seguir em frente.

## 1. Descrição do módulo e assuntos abordados

### 1.1. Ementa do módulo

:::warning

Essa é a **ementa oficial** do módulo. O texto é sucinto e formal pois ele faz
parte de um documento **apresentado ao MEC** no momento em que o curso é
reconhecido. A ementa é um dos itens que **obrigatoriamente** deve ser
**mostrado aos alunos**. O que estou fazendo agora? Mostrando a ementa do
módulo para vocês. Sem mais dúvidas, certo?

:::

Este módulo aborda o desenvolvimento de um sistema integrando robôs móveis
controlados de forma remota e visão computacional. Para tal, são apresentados
os assuntos de simulação de sistemas eletromecânicos para aplicações de
robótica, estruturas de dados (grafos e filas), sistemas de controle (discretos
e contínuos), transformadas de Laplace e Fourier, transmissão de dados com
baixa latência e alto throughput, conceitos básicos de sistemas operacionais,
desenvolvimento de sistemas com falha segura, visão computacional clássica
(filtros de convolução e correlação cruzada), redes neurais artificiais e redes
neurais convolucionais. A solução proposta pelos estudantes deverá integrar
conceitos de metadesign, e apresentar uma discussão dos aspectos éticos
relacionados à utilização de robôs. Os estudantes também deverão avaliar os
impactos econômicos dessa inserção, abordando os temas de novas oportunidades
de negócios com Veículo Guiado Automático (AGV  - Automated Guided Vehicle),
além do estudo da geração de leads (estudo de possíveis clientes) e modelos de
mercado. Além disso, a coluna dorsal do módulo é a relação entre ética e
conhecimento, seja na sua produção, negociação ou implementação. Nesse sentido,
é tratada  a construção de argumentos, técnicas de negociação e gestão do
conhecimento. Ao final do módulo, os estudantes deverão ser capazes de
desenvolver projetos que envolvem a junção de controle automático de sistemas
dinâmicos eletromecânicos, aplicações práticas de redes neurais para visão
computacional e requisitos de comunicação em soft real time através de meta
sistemas operacionais.

### 1.2. Descrição do projeto

<img 
  src="https://satchiikoma.files.wordpress.com/2013/02/here-he-comes-to-save-the-day.jpg"
  alt="Hibito sendo puxado pelo rover" 
  style={{ 
    display: 'block',
    marginLeft: 'auto',
    maxHeight: '60vh',
    marginRight: 'auto'
  }} 
/>
<br/>

O projeto deste módulo tem como objetivo o uso de um robô móvel para navegar em
ambientes cujo **acesso** para seres humanos seja **difícil** ou
**impossível**. O robô deve ser **operado à distância**, enviando um **vídeo em
tempo real** para o operador. O sistema deve ser capaz de **processar as
imagens** geradas, detectando e enviando ao operador situações que podem ser de
seu interesse (e.g. detectar falhas estruturas, princípios de incêndio,
detritos)

Para entender o valor do projeto proposto nesse módulo, acho fundamental
respondermos algumas perguntas:

1. O que é um robô móvel?
2. Por quê substituir um humano por um robô?
3. Como um humano pode operar um robô à distância?
4. Qual o ganho de fazer o robô identificar padrões e analisar as imagens antes
   de enviar ao operador humano?

Como estamos firmemente na **era das IAs generativas**, vou perguntar
diretamente ao nosso amigo **Gepeto** e ver se a resposta é razoável. Tá aqui o
[link do
chat](https://chat.openai.com/share/387ec7a8-4023-4bbb-a070-17d4ab1c82ca) que
não me deixa mentir.

**O que é um robô móvel?**

> Um robô móvel é basicamente uma máquina equipada com rodas ou outro tipo de
> locomoção que lhe permite mover-se autonomamente, ou seja, sem precisar de
> controle humano direto. Ele é programado para realizar tarefas específicas,
> como transportar itens em uma fábrica, fazer limpezas em ambientes domésticos
> ou até explorar locais que seriam perigosos ou inacessíveis para humanos, como
> o fundo do oceano ou outros planetas.
> 
> Esses robôs possuem sensores e sistemas computacionais que lhes permitem
> entender e interagir com o ambiente ao redor. Eles podem detectar obstáculos,
> seguir rotas mapeadas e até responder a mudanças no ambiente, adaptando-se para
> continuar realizando suas funções de maneira eficiente. Essa capacidade de
> navegação e adaptação faz dos robôs móveis ferramentas valiosas em muitas
> áreas, incluindo indústria, pesquisa científica e vida cotidiana.

Agora, vou fazer o que vocês **sempre deveriam fazer** ao receber uma resposta
de uma IA generativa: vou usar o meu **senso crítico** e **checar** para ver se
tem alguma coisa errada nessa resposta.

Primeiro, vamos desempacotar a resposta do GPT e identificar os pontos
principais:

1. Um robô móvel é uma máquina capaz de se locomover;
2. O robô móvel é autônomo e não precisa de controle humano;
3. Ele é usado em tarefas específicas, geralmente para tarefas simples; e
4. Ele possui sistemas computacionais e sensores para "ver" e responder ao
   ambiente em que está inserido.

Agora, vamos fazer a nossa pesquisa para validar esses pontos. Fazendo uma
[pesquisa simples](https://www.igi-global.com/dictionary/?p=Mobile+Robot) pelo
termo "Mobile robot" em uma ferramenta de definição de termos científicos
baseada em diversos artigos quase que imediatamente corrobora os pontos **1, 3
e 4**. O ponto **2**, no entanto, só é corroborado por artigos que mencionam de
forma específica o termo "autonomous mobile robot", ou seja, há motivos para
imaginar que **nem todo robô móvel é autônomo**. Uma segunda pesquisa, agora no
google, pelo termo "operated mobile robot" nos mostra diversos exemplos de
robôs móveis operados por seres humanos. Um exemplo é o uso de **drones de
resgate**:

<img 
  src="https://sn4.scholastic.com/content/dam/classroom-magazines/sn4/issues/2022-23/011623/drones-to-the-rescue/SN4-12-011623-P04-DronesToTheRescue-TN_.jpg"
  alt="Drone de resgate" 
  style={{ 
    display: 'block',
    marginLeft: 'auto',
    maxHeight: '30vh',
    marginRight: 'auto'
  }} 
/>
<br/>


Pronto! Agora usamos o GPT de forma consciente e temos nossa definição de robô
móvel.

**Por quê substituir um humano por um robô?**

> Substituir um humano por um robô móvel é vantajoso principalmente porque robôs
> podem realizar tarefas repetitivas, perigosas ou que exigem precisão constante
> sem cansaço, erro ou risco para a saúde humana. Eles aumentam a eficiência e
> segurança nos processos, trabalhando ininterruptamente e em condições que
> seriam adversas ou nocivas para pessoas, como ambientes com substâncias tóxicas
> ou extremos de temperatura.

Essa resposta é bem menos opinionada que a última, portanto não há muito o que
verificar. A única afirmação que tipicamente mereceria uma citação é de que
**robôs móveis aumentam a eficiência e segurança nos processos**. Uma pesquisa
rápida no google scholar me levou a [esse
artigo](https://www.sciencedirect.com/science/article/abs/pii/S0301479723021291)
que corrobora a afirmação ao menos para o caso de descarte de detritos de
obras.

**Como um humano pode operar um robô à distância?**

> Um humano pode operar um robô à distância usando controles remotos ou
> interfaces digitais que permitem enviar comandos através de conexões sem fio
> como Wi-Fi, Bluetooth ou mesmo via internet. Essas interfaces podem variar
> desde simples joysticks até softwares sofisticados instalados em computadores
> ou aplicativos em smartphones. Além disso, algumas interfaces incluem telas que
> mostram vídeo em tempo real do ponto de vista do robô, oferecendo ao operador
> uma visão detalhada do ambiente ao redor do robô, o que facilita a navegação e
> a execução de tarefas específicas com maior precisão.

Acho que vocês já entenderam o que precisam fazer agora, né? Verifique você
mesmo se a resposta acima é verdadeira.

**Qual o ganho de fazer o robô identificar padrões e analisar as imagens antes
de enviar ao operador humano?**

> O ganho de fazer o robô identificar padrões e analisar imagens antes de enviar
> ao operador humano é significativo, pois permite uma operação mais eficiente e
> autônoma. Primeiramente, reduz a quantidade de dados que precisam ser
> transmitidos ao operador, enviando apenas informações relevantes e filtradas, o
> que é crucial em ambientes com limitações de largura de banda ou conexões
> instáveis. Além disso, a análise prévia pelo robô possibilita respostas mais
> rápidas a situações críticas, pois o sistema pode identificar e reagir a
> elementos ou alterações importantes no ambiente sem atraso na comunicação
> humana. Isso também diminui a carga cognitiva sobre o operador, permitindo que
> ele se concentre em decisões mais complexas e estratégicas, aumentando assim a
> segurança e a eficácia das operações.

**Conclusão**

Agora que respondemos às perguntas propostas, podemos concluir que:

1. Vamos usar um mecanismo capaz de se locomover para entrar em espaços que
   podem ser nocivos ao ser humano;
2. O que faz com que possamos eliminar um risco para a pessoa que executava
   aquela tarefa ao substituí-la por um robô;
3. Para que esse operador possa pilotar o robô à distância, precisamos que uma
   imagem seja transmitida em tempo real; e
4. Ao pré-processar essa imagem e identificar padrões antes de mostrar para o
   operador, podemos tirar a sua carga cognitiva e garantir que as situações de
   interesse sejam sempre detectadas.

### 1.3. Assuntos abordados

Segue uma lista dos principais assuntos que vamos abordar:

* Estruturas de dados e algoritmos;
* Sistemas de controle;
* Conceitos de robótica móvel;
* Processamento e transmissão de sinais;
* Visão computacional clássica;
* Fundamentos de aprendizado profundo;
* Redes neurais; e
* Visão computacional moderna.


## 2. Organização do material

Vamos conversar sobre como esse material foi organizado? Isso vai ser
importante para você não se perder por aqui e conseguir identificar rapidamente
quais seções vão ser as mais importantes para você o mais rápido possível (e
quais **valem nota** também =O). Vamos começar falando dos **ícones** que vocês
vão ver por aqui.

### 2.1. Glossário de ícones

O objetivo desse material é facilitar sua vida e diminuir a distância entre
você e as informações necessárias para que você faça um projeto espetacular.
Para isso, é essencial que as informações aqui dispostas sejam muito fáceis de
categorizar só batendo o olho. Qual a melhor forma de fazer isso? Usando ícones
padronizados. Em cada seção de conteúdo, você vai ver um **ícone à esquerda do
título da seção**. Aqui estão os ícones que você vai encontrar e o que eles
significam:

<div style={{ display: 'flex', justifyContent: 'center', alignItems: 'center', margin: '0' }}>
    <table>
        <tr>
            <th>Ícone</th>
            <th>Descrição</th>
        </tr>
        <tr>
            <td style={{textAlign: "center", verticalAlign: "middle", height: "10vh"}}>
                <img src="icons/artefato.svg" style={{height: "5vh", maxWidth: "75%"}}/>
            </td>
            <td style={{textAlign: "left", verticalAlign: 'top', paddingTop: '3.5vh' }}>
                Significa que a seção vai conter o detalhamento de um ou mais
                **artefatos** do metaprojeto.
            </td>
        </tr>
        <tr>
            <td style={{textAlign: "center", verticalAlign: "middle", height: "10vh"}}>
                <img src="icons/autoestudo.svg" style={{height: "4vh"}}/>
            </td>
            <td style={{textAlign: "left", verticalAlign: 'top', paddingTop: '3.5vh' }}>
                Significa que a seção é um autoestudo **obrigatório**.
            </td>
        </tr>
        <tr>
            <td style={{textAlign: "center", verticalAlign: "middle", height: "10vh"}}>
                <img src="icons/opcional.svg" style={{height: "4vh"}}/>
            </td>
            <td style={{textAlign: "left", verticalAlign: 'top', paddingTop: '3.5vh' }}>
                Significa que a seção é um autoestudo **opcional**.
            </td>
        </tr>
        <tr>
            <td style={{textAlign: "center", verticalAlign: "middle", height: "10vh"}}>
                <img src="icons/ponderada.svg" style={{height: "4vh"}}/>
            </td>
            <td style={{textAlign: "left", verticalAlign: 'top', paddingTop: '3.5vh' }}>
                Significa que a seção descreve o enunciado de uma atividade que
                **vale nota**.
            </td>
        </tr>
    </table>
</div>

### 2.2. Seções do material

Existem quatro grandes seções que dividem esse material. São elas:

1. Introdução;
2. Artefatos;
3. Setup das ferramentas; e
4. Conteúdo dos encontros

A seção de **introdução** é, bem, essa aqui. Ela é a página inicial do material
e deve ser o ponto de partida para qualquer um que não tenha visto esse
material antes. Tendo dito isso, se você nunca leu esse material antes e não
está lendo isso, não tem muito o que eu possa fazer =-(

A seção dos **artefatos** contem uma descrição **detalhada** de todos os
artefatos do metaprojeto que competem à area de computação. 

A seção de **setup das ferramentas** contem todos os autoestudos que não são
exatamente o cerne do módulo, mas são tutoriais úteis para que você possa
configurar seu ambiente de trabalho. Aqui você vai encontrar coisas como
tutoriais para instalação de sistemas operacionais, ferramentas/bibliotecas,
seu repositório no github e até mesmo indicações de cursos/vídeos para aprender
as linguagens principais vistas por aqui.

A seção de **conteúdo dos encontros** é, na verdade, um conjunto de seções.
Elas aparecem por último no menu que fica aqui à esquerda (`<<`). Você vai
conseguir perceber que se trata de uma seção de material do encontro pois todas
elas tem "EXX" ou "OXX" em seu título, sendo o `XX` substituído por algum
número. Por exemplo: `E1 - Modelos de comunicação e MQTT`.

O que é esse barulho, você está ouvindo? Ah, é só mais um **ALERTA DE EXTREMA
IMPORTÂNCIA** chegando:

:::warning

O nosso modelo de ensino é muito inovador e tem um foco muito grande em
desenvolver competências. Qual é o custo disso? Não existe possibilidade alguma
de somente o tempo em sala de aula ser suficiente para abordar todo o conteúdo.
Então aí vem o alerta:

**SE VOCÊ NÃO FIZER OS AUTOESTUDOS ANTES DO ENCONTRO VOCÊ VAI FICAR BOIANDO**

Sério, vai mesmo. Nesse módulo não teremos **nenhuma** aula 100% expositiva.
**Faça os autoestudos**. Tá sem tempo, deixou a coisa acumular? Segue o
preceito do **feito é melhor que perfeito**. Dá uma lida, veja só os vídeos,
mas faça alguma coisa pelo menos. Combinado? Combinado.

:::


## 3. Descrição dos entregáveis

A descrição completa e precisa dos artefatos você vai conseguir encontrar na
[Adalove](https://adalove.inteli.edu.br) ou nas seções de artefatos desse
material. No entanto, eu sei o que acontece quando vocês encontram um bloco de
texto maior do que 10 linhas:

<img 
  src="https://media1.tenor.com/m/Zuar_NDXOxMAAAAC/mutta-nanba-nanba-mutta.gif"
  alt="Muta is not amused"
  style={{ 
    display: 'block',
    marginLeft: 'auto',
    maxHeight: '30vh',
    marginRight: 'auto'
  }} 
/>
<br/>

E eu também sei que é muito importante que **todos** tenham a capacidade de
explicar de forma simples o que precisa ser alcançado em cada sprint. Por isso,
eu vou escrever de forma **clara** e com uma linguagem **acessível** o que
vocês precisam fazer em cada um dos sprints.

### 3.1. [Sprint 1](/sprint1) 

Aqui vocês precisam **entender o problema**. Não, isso não significa sair
fazendo matriz de risco, personas e user stories sem nem pensar. Isso significa
seguir uma **série** de **etapas investigativas** para que o **escopo** do seu
projeto **reflita** uma **necessidade** real de **pessoas**:

1. Entenda as **pessoas** e a sua dor. Todo o resto emana disso. Se você não se
   esforçar para entender **para quem** e **por quê** você está fazendo o que
   está fazendo, o seu trabalho corre um **grande risco** de ser **inútil**.
2. Estude o **negócio** e monte sua **proposta de valor**. Agora que você
   conhece as pessoas, pode pensar no ambiente em que estão inseridas e o que
   significa valor nesse ambiente. Aqui é que você deixa executivos felizes.
3. Sem **perder de vista** o que fez nas etapas anteriores, monte o **escopo**
   do seu projeto. Defina os seus **requisitos funcionais** e **não
   funcionais**. Defina **métricas** para validar o que significa
   **sucesso** ou **fracasso**. Sem fazer essa etapa do projeto, você vai
   depender de *feeling* para saber se está indo no caminho certo. Você
   **não quer** depender de *feeling*.
4. Pesquise as tecnologias envolvidas e monte uma **proposta de arquitetura**.
   Sim, você vai mudar ela. Não, isso não significa que você não deve se
   esforçar para fazer uma arquitetura que faça sentido já nessa etapa do
   projeto. *Um erro cometido com esforço tem valor*. Se você fizer essa etapa
   de qualquer jeito, seu projeto vai virar uma bagunça.

### 3.2. [Sprint 2](/sprint2) 

Monte a sua primeira versão da **prova de conceito**. Já? Já. O que isso
significa? Que um **usuário** tem que ser capaz de **operar o robô** à
distância. Faça o **mínimo** para que essa funcionalidade seja **viável**.

Todo começo de projeto costuma ter mais perguntas do que respostas. Um erro
cometido com esforço tem valor pois ele traz respostas. Conclusão? Se esforce e
erre mais rápido. 

Nessa sprint, vocês vão precisar aprender a usar o **robô**, o **ROS** e
**configurar a comunicação** para que um usuário possa pilotar o seu robô.
Precisa de câmera? Ainda não. Precisa de visão computacional? Ainda não.

### 3.3. [Sprint 3](/sprint3) 

Agora que tiveram a primeira **interação do usuário com o sistema**, tá na hora
de organizar as informações que foi possível extrair desse teste e **melhorar o
projeto**. Como? Pensando melhor na **interface de usuário** e adicionando o
**feed de imagem de uma câmera**.

Aqui também vamos voltar para a **proposta de negócios** e montar uma **análise
financeira**. Por que agora e não antes? Pois agora que vocês já montaram um
MVP, vão poder estimar com muito mais propriedade o custo de implementação e
operação do **projeto final**.

### 3.4. [Sprint 4](/sprint4) 

Duas palavras: **visão computacional**. Chegou a hora. Procure um dataset
relevante (idealmente fornecido pelo parceiro) e treine um modelo para detectar
um padrão que seja útil para o operador do sistema.

Aqui também vamos voltar para os **requisitos**. Lembra que falei que você
precisava definir bem os requisitos para conseguir discernir o que é
**fracasso** e **sucesso**? Pois bem, agora é a hora de bolar e implementar os
**roteiros de testes** que vão trazer à tona se o seu projeto está encaminhado
para atender os requisitos levantados.

### 3.5. [Sprint 5](/sprint5) 

Tudo ao mesmo tempo o tempo todo. Pode parecer que o sprint 5 são duas semanas
de férias após a conclusão dos requisitos técnicos de um projeto, mas na
verdade o sprint 5 é uma **oportunidade** de melhorar **tudo**. A **análise de
negócios** não reflete mais o projeto após mudanças de rumo? Modifique. Os
**testes** ainda não estão dando os resultados esperados? Melhore sua
implementação. Sua **análise financeira** ainda não reflete o sistema final?
Refaça.

Tem muita coisa para fazer e é por isso que sugere-se que a implementação
técnica já esteja finalizada. Não menospreze o esforço necessário para fazer
ajustes finos em um projeto.

## 4. Bibliografia

:::warning

Sou eu novamente. Só vim avisar que mostrar a bibliografia do módulo para vocês
também é **obrigatório** de acordo com o MEC. O que estou fazendo aqui? Estou
mostrando a bibliografia para vocês.

:::

O material disponibilizado nessa página deve ser o suficiente para que você
seja capaz de entregar o projeto proposto, mas não se engane; não há a menor
chance do material aqui contido ser tão detalhado quanto um livro a respeito
dos diversos assuntos abordados nesse módulo. Sendo assim, essa seção apresenta
quais foram os livros utilizados para gerar o conteúdo focado dessa página.
Quer se aprofundar em algum assunto que viu no módulo? procure pelo livro aqui.

### 4.1. Bibliografia básica

[CASTRUCCI, P. de L.; BITTAR, A.; SALES, R. M. Controle automático. 2. ed. Rio
de Janeiro: LTC,
2018.](https://integrada.minhabiblioteca.com.br/#/books/9788521635628)

* O livro aborda os principais conceitos e técnicas em controle automático, como
sistemas de controle de malha aberta e fechada, controle PID, projeto de
controladores, análise de resposta transitória e estabilidade e controle
adaptativo. Além disso, o livro é atualizado para refletir as últimas
tendências em controle automático, tornando-se uma leitura obrigatória para
pessoas interessadas nessa área.

[DINIZ, P. S. R. Processamento digital de sinais: projeto e análise de
sistemas. 2. ed. Porto Alegre: Bookman,
2014.](https://integrada.minhabiblioteca.com.br/#/books/9788582601242)

* Este livro abrange uma ampla gama de tópicos em processamento de sinais, desde
fundamentos de análise espectral e transformadas de Fourier, até filtros
digitais, técnicas de modulação, equalização de canal, compressão de sinal e
processamento adaptativo. Além disso, o livro inclui exemplos práticos de
implementação de algoritmos.

[LAMBERT, K. A. Fundamentos de Python: estruturas de dados. 1. ed. São Paulo:
Cengage Learning,
2022.](https://integrada.minhabiblioteca.com.br/#/books/9786555584288)

* O autor apresenta as principais estruturas de dados em Python, incluindo
  listas, tuplas, dicionários e conjuntos. O livro é muito útil para aqueles
  que desejam compreender como essas estruturas funcionam em Python e como
  usá-las de forma eficiente em diferentes situações de programação. Com muitos
  exemplos práticos e exercícios, o livro é uma ferramenta valiosa para o
  aprendizado da linguagem de programação  Python.

[LATHI, B. P. Sinais e sistemas lineares. 2. ed. Porto Alegre: Bookman,
2008.](https://integrada.minhabiblioteca.com.br/#/books/9788577803910)

* Este o livro é uma fonte rica para estudantes de engenharia, pois seu
  conteúdo apresenta de forma clara e didática conceitos fundamentais de sinais
  e sistemas lineares, com o objetivo de trazer ferramentas matemáticas no
  domínio contínuo e discreto para que os estudantes possam modelar e manipular
  sistemas de controle clássicos e modernos.

[MORAES, D. de. Metaprojeto: o design do design. São Paulo: Blucher,
2010.](https://integrada.minhabiblioteca.com.br/#/books/9788521216308)

* O metaprojeto, como observado neste livro, é uma alternativa posta ao design,
  contrapondo os limites da metodologia convencional, ao se colocar como etapa
  prévia de reflexão e suporte ao desenvolvimento do projeto em um cenário
  mutante e complexo. Nesse sentido, o metaprojeto, enquanto metodologia da
  complexidade, pode ser considerado o projeto do projeto, ou melhor, o design
  do design.

[NALON, J. A. Introdução ao processamento digital de sinais. Rio de Janeiro:
LTC, 2009.](https://integrada.minhabiblioteca.com.br/books/978-85-216-2615-2)

* Este livro trata de conteúdo importante para estudantes de engenharia, pois
  aborda tópicos importantes como séries de Fourier, transformadas de Fourier
  entre outros, fundamentais em diversas áreas da engenharia, com explicações
  claras e exemplos práticos. 

[PICHETTI, R. F.; JUNIOR, C. A. C.; ALVES, J. V. da S.; et al. Computação
gráfica e processamento de imagens. Porto Alegre: SAGAH,
2022.](https://integrada.minhabiblioteca.com.br/#/books/9786556903088)

* Os autores exploram os principais conceitos e técnicas em computação gráfica
  e processamento de imagens, incluindo a teoria da cor, transformações
  geométricas, filtragem e reconhecimento de padrões. Com exemplos práticos, o
  livro é uma ferramenta para aprimorar as habilidades, com aplicações em
  jogos, animação, design gráfico, visão computacional e processamento de
  imagens.

[RUSSELL, S. J.; NORVIG, P. Inteligência artificial: uma abordagem moderna. 4.
ed. Rio de Janeiro: LTC,
2022.](https://integrada.minhabiblioteca.com.br/#/books/9788595159495)

* Este livro abrange tópicos como aprendizado de máquina, processamento de
  linguagem natural e visão computacional, fornecendo uma visão abrangente e
  prática, sendo uma referência essencial para os estudantes interessados em
  explorar as aplicações modernas da IA.

[SCHIAVINI, J. M.; MARANGONI, E. Marketing digital e sustentável. Porto Alegre:
SAGAH, 2019.](https://integrada.minhabiblioteca.com.br/#/books/9786581739034)

* O crescimento do marketing digital gera implicações nas estratégias e
  conceitos atuais de marketing. Esse livro aborda quais são os novos conceitos
  e definições advindas do marketing digital e as novas tendências da área.

[SHAW, Z. A. Aprenda Python 3 do jeito certo: uma introdução muito simples ao
incrível mundo dos computadores e da codificação. Rio de Janeiro: Alta Books,
2019.](https://integrada.minhabiblioteca.com.br/#/books/9788550809205)

* É um livro altamente recomendado para aqueles que desejam aprender Python de
  forma prática e acessível. Com muitos exemplos e exercícios, o livro é
  voltado para iniciantes e oferece uma base sólida em Python 3, cobrindo os
  principais conceitos e recursos da linguagem. Além disso, o livro é
  atualizado regularmente para refletir as mudanças mais recentes na linguagem
  Python.

[SILVA, F. M.; LEITE, M. C. D.; OLIVEIRA, D. B. Paradigmas de programação.
Porto Alegre: SAGAH,
2019.](https://integrada.minhabiblioteca.com.br/#/books/9788533500426)

* O livro possui uma abordagem didática e clara, os autores exploram os
  principais paradigmas de programação, incluindo programação funcional,
  programação orientada a objetos e programação concorrente. O livro é
  especialmente útil para aqueles que desejam compreender as diferenças entre
  esses paradigmas e quando aplicá-los em diferentes situações de programação.

[SILVA, R. F. da. Deep Learning. São Paulo: Platos Soluções Educacionais S.A.,
2021.](https://integrada.minhabiblioteca.com.br/#/books/9786589881520)

* O livro aborda de forma clara e concisa os conceitos de redes neurais e deep
  learning, que são essenciais para a construção de sistemas autônomos. Além
  disso, o livro apresenta exemplos práticos e uma abordagem ética, o que é
  fundamental em um campo em constante evolução e que tem grande impacto na
  sociedade.

[SORDI, J. O. Administração da informação: fundamentos e práticas para uma nova
gestão do conhecimento. São Paulo: Saraiva,
2015.](https://integrada.minhabiblioteca.com.br/#/books/9788502634817)

* Considerando o contexto informacional atual, este livro descreve estratégias
  práticas que as organizações utilizam para a gestão de seu conhecimento. 

[TAJRA, S. F. Comunicação e negociação: conceitos e práticas organizacionais.
São Paulo: Saraiva,
2014.](https://integrada.minhabiblioteca.com.br/#/books/9788536511054)

* Este livro tem como objetivo apresentar técnicas de comunicação que ajudam
  lideranças a construírem acordos, estabelecerem uma cultura de respeito
  dentro das organizações e a fomentar processos claros e alinhados aos
  objetivos dos colaboradores e da organização.

[ZILL, D. G. Matemática avançada para engenharia. 3. ed. Porto Alegre: Bookman,
2011. v. 1. ](https://integrada.minhabiblioteca.com.br/books/9788577804771)

* Este livro é essencial para estudantes de engenharia, pois oferece uma visão
  clara e concisa dos principais conceitos matemáticos necessários para a
  prática da profissão. O volume 1 aborda Equações Diferenciais Ordinárias,
  Transformadas de Laplace, entre outros tópicos, de forma didática e objetiva.

### 4.2. Bibliografia complementar

[BANIN, S. L. Python 3: conceitos e aplicações: uma abordagem didática. São
Paulo: Érica,
2018.](https://integrada.minhabiblioteca.com.br/#/books/9788536530253)

* O livro aborda tópicos como tipos de dados, estruturas de controle, funções,
  arquivos, módulos e muito mais. O livro é especialmente útil para aqueles que
  desejam compreender a lógica da programação em Python e aplicá-la em projetos
  práticos, como desenvolvimento web, científico, de jogos e automação de
  tarefas.

[BROWN, T. Design thinking: uma metodologia poderosa para decretar o fim das
velhas ideias. Rio de Janeiro: Alta Books,
2020.](https://integrada.minhabiblioteca.com.br/#/books/9788550814377)

* Esta obra apresenta as emoções, a mentalidade e os métodos necessários para
  elaborar o design de um produto a uma experiência ou estratégia, de modo
  inovador. Este livro é um guia para líderes criativos que buscam propor o
  design thinking em todas as facetas de suas organizações, produtos ou
  serviços para descobrir novas alternativas para os negócios e para a
  sociedade como um todo.

[CASAS, A. L. L. Marketing digital. 1. ed. Barueri: Atlas,
2022.](https://integrada.minhabiblioteca.com.br/#/books/9786559771103)

* Livro de referência no estudo do Marketing brasileiro, apresenta novidades e
  discute os mais modernos assuntos da área, como storytelling, economia de
  compartilhamento, neuromarketing e content marketing.

[TURCHI, S. R. Estratégia de marketing digital e e-commerce. 2. ed. São Paulo:
Atlas, 2018.](https://integrada.minhabiblioteca.com.br/#/books/9788597015409)

* A obra tem por objetivo apresentar, de forma acessível, uma visão abrangente
  sobre as diversas estratégias de marketing digital e e-commerce, por meio da
  exposição de artigos, ideias, pesquisas e também casos de grandes empresas,
  multinacionais e pequenas e médias empresas.

[ZEITHAML, V. A.; PARASURAMAN, A. A excelência em serviços: como superar as
expectativas e garantir a satisfação completa de seus clientes. 1. ed. São
Paulo: Saraiva,
2014.](https://integrada.minhabiblioteca.com.br/#/books/9788502225572)

* Serviços de qualidade superior tem se revelado uma estratégia competitiva
  vencedora e um maior desafio para as empresas Nessa obra os autores
  desenvolvem uma ferramenta que permite mensurar de forma qualitativa e
  quantitativamente o grau de satisfação dos clientes, considerando as cinco
  dimensões da qualidade dos serviços: tangibilidade, confiabilidade,
  responsividade, segurança e empatia.

[CENGEL, Y. A. Equações diferenciais. Porto Alegre: AMGH,
2014.](https://integrada.minhabiblioteca.com.br/books/9788580553499)

* O livro é uma fonte fundamental para estudantes de engenharia, abordando de
  forma clara e abrangente os principais tópicos e aplicações das equações
  diferenciais. É uma ferramenta valiosa para compreender e resolver problemas
  complexos em diversas áreas da matemática aplicada e engenharia.

[COSTA, R. L. da; ANTÓNIO, N. S. Aprendizagem organizacional: ferramenta no
processo de mudança. Coimbra: Conjuntura Actual, 2017.
](https://integrada.minhabiblioteca.com.br/#/books/9789896942601)

* A gestão do conhecimento depende de uma organização comprometida com o
  aprendizado constante. Esta obra descreve esse processo apontando para os
  diferenciais competitivos de uma boa estratégia de aprimoramento e mudança
  organizacional. 

[DORF, R. C.; BISHOP, R. H. Sistemas de controle modernos. 13. ed. Rio de
Janeiro: LTC,
2018.](https://integrada.minhabiblioteca.com.br/#/books/9788521635147)

* O livro Sistemas de Controle Modernos é uma leitura fundamental para
  estudantes e profissionais de engenharia que desejam aprender sobre sistemas
  de controle. Os autores exploram os principais conceitos e técnicas em
  sistemas de controle modernos, incluindo sistemas de controle de malha aberta
  e fechada, controle PID, projeto de controladores, análise de resposta
  transitória e estabilidade, controle adaptativo e muito mais.

[FRIGERI, S. R.; JUNIOR, C. A. C.; ROMANINI, A. Computação gráfica. Porto
Alegre: SAGAH:
2018.](https://integrada.minhabiblioteca.com.br/#/books/9788595026889)

* Os autores exploram as principais técnicas em computação gráfica, incluindo
  modelagem geométrica, iluminação, shading, texturização e animação. Com
  exemplos, o livro é uma ferramenta valiosa para quem busca aprimorar suas
  habilidades em computação gráfica e aplicá-las em projetos práticos, como
  desenvolvimento de jogos, simulações e animações.

[LEWICKI, R. J.; SAUNDERS, D. M.; BARRY, B. Fundamentos de negociação. 5. ed.
Porto Alegre: AMGH,
2014.](https://integrada.minhabiblioteca.com.br/#/books/9788580553864)

* O livro ensina estratégias para negociações complexas e difíceis, bem como
  técnicas para lidar com diferentes tipos de negociadores, aborda as emoções e
  os comportamentos envolvidos na negociação, fornecendo uma compreensão mais
  profunda do processo de negociação.

[MARTIN, R. C. Desenvolvimento ágil limpo: de volta às origens. Rio de Janeiro:
Alta Books, 2020.
](https://integrada.minhabiblioteca.com.br/#/books/9788550816890)

* Esta obra traz uma revisão do manifesto ágil e a forma como ele é utilizado
  na entrega de valor com o projeto. Ele traz uma visão do framework Scrum e
  outras ferramentas ágeis do ponto de vista dos desenvolvedores, refletindo
  sua experiência ao longo dos anos e como este manifesto foi revisitado.

[MILANI, A. M. P. et al. Visualização de dados. 1. ed. Porto Alegre: SAGAH,
2020.](https://integrada.minhabiblioteca.com.br/#/books/9786556900278)

* Este livro mostra diferentes formas de processamento, análise e apresentação
  de dados para que o leitor esteja apto a utilizá-los da forma mais proveitosa
  possível. Para tal, discute-se desde a história desse campo até as mais
  modernas técnicas e abordagens que permitem a elaboração de visualizações de
  dados claras, úteis e atualizadas.

[ORSINI, L. Q. Curso de circuitos elétricos. 2. ed. São Paulo: Blucher, 2004.
v. 2. ](https://integrada.minhabiblioteca.com.br/books/9788521215264)

* O livro é uma ferramenta essencial para estudar séries e transformadas de
  Fourier, oferecendo explicações claras e exemplos práticos. Sua abordagem
  abrangente e sua segunda edição atualizada o tornam uma fonte valiosa para a
  compreensão aprofundada desses tópicos fundamentais em circuitos elétricos.

[READE, D. V.; ROCHA, M.; OLIVEIRA, S. L. I. de; CHERNIOGLO, A. Marketing B2B.
São Paulo: Saraiva,
2015.](https://integrada.minhabiblioteca.com.br/reader/books/978-85-02-63884-6)

* Este volume da coleção Marketing em Tempos Modernos é focado no mercado B2B e
  na sua crescente globalização e complexidade. A obra aborda a importância do
  bom relacionamento entre as empresas e seus clientes e fornecedores, pois
  tais parcerias estratégicas permitem a criação de diferenciais competitivos,
  fundamentais para o sucesso de toda a cadeia de suprimentos da qual essas
  organizações fazem parte. 

[SOBRAL, W. S. Design de interfaces: introdução. São Paulo: Saraiva,
2019.](https://integrada.minhabiblioteca.com.br/#/books/9788536532073)

* Esta obra aborda o que são interfaces e sua evolução ao longo do tempo, até
  chegarem à Interface Homem-Computador (IHC). Propõe o estudo das fases de um
  projeto, incluindo métodos de construção e documentação. Discorre sobre as
  estratégias de prototipagem adotadas no desenvolvimento do design e comenta a
  lista de princípios para orientar o designer durante esse processo criativo.
