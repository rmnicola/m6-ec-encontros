---
title: Introdução
sidebar_position: 1
slug: /
---

# Infraestrutura de Cidades Inteligentes

<img 
  src="https://www.icegif.com/wp-content/uploads/2022/09/icegif-78.gif"
  alt="Gon says hi =D" 
  style={{ 
    display: 'block',
    marginLeft: 'auto',
    maxHeight: '20vh',
    marginRight: 'auto'
  }} 
/>
<br/>

Olá! Seja bem-vindo à página que abriga o conteúdo de computação do módulo 9 do
curso de Engenharia de Computação do Inteli. Aqui você vai encontrar tudo o que
precisa para compreender os desafios relacionados à hiperconectividade aplicada
a cidades inteligentes e atender aos requisitos do projeto do módulo. Esta
seção em específico serve para você entender como esse material foi organizado
e ter uma visão geral do módulo e do metaprojeto. Sendo assim, aqui você poderá
consultar:

* Descrição geral do módulo e especificação do metaprojeto;
* Organização do material;
* Principais assuntos abordados;
* Artefatos e itens de entrega por sprint; e
* Bibliografia.

Antes de seguirmos em frente com o conteúdo principal dessa seção, vou
aproveitar que tenho sua atenção para fazer um **anúncio** de **extrema
importância**. Se liga, que logo abaixo vem o admonition:

:::danger

Esse material **NÃO** substitui de forma alguma o uso da **Adalove**. Você
**DEVE** entrar na Adalove com frequência e **REGISTRAR O SEU PROGRESSO**.
Entendeu? Ainda não? Pera aí que vou desenhar:

<img 
  src="img/aviso-adalove.png"
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

Este módulo foca no desenvolvimento de um sistema avançado, projetado para
atender às exigências de conectividade em cidades inteligentes. Abordaremos
tópicos fundamentais como modelos de comunicação em redes, incluindo as camadas
OSI e TCP/IP, além de protocolos como MQTT. Exploraremos tanto bancos de dados
relacionais quanto não relacionais, enfatizando sua aplicabilidade e
otimização. A segurança da informação será um pilar central, garantindo a
proteção de dados em ambientes de grande volume de informações. Além disso,
discutiremos sistemas de processamento de eventos complexos para garantir
escalabilidade ao sistema, além de abordar aspectos de business intelligence,
apresentando ferramentas essenciais para análise e tomada de decisão
estratégica. Por fim, abordaremos formatos de arquivo para armazenamento de
longo prazo de grandes volumes de dados, essenciais para sua gestão eficiente.
Aspectos focados no urbanismo centrado no usuário são apresentados, focando nos
impactos que eles causam no meio ambiente e como as transformações digitais
impactam em outros setores de políticas de educação ambiental, representados
por indicadores de governança ambiental, social e corporativa (ESG). Ainda
dentro deste escopo, os estudantes devem considerar aspectos de
hiperconectividade no contexto das culturas afro-brasileira, africana e
indígena, representados pelas minorias periféricas, que também são usuários dos
sistemas projetados. Ainda nessa linha, o projeto desenvolvido deve apresentar
soluções para o desafio de lidar com direitos humanos e suas vulnerabilidades
no contexto organizacional. Ao concluir este módulo, o estudante estará
capacitado a projetar e implementar sistemas robustos, capazes de gerenciar
grandes volumes de requisições de maneira segura e eficiente no desenvolvimento
de soluções para Smart Cities.

### 1.2. Descrição do projeto

O nosso modelo gira em torno do projeto, então nada mais justo do que
investirmos alguns minutos para definir de forma clara o que faremos nesse
módulo:

Neste módulo vamos trabalhar com a *parte invisível* dos projetos de IoT.
Sempre se fala muito de como instrumentar, configurar e criar dispositivos
capazes de se comunicar com outros, mas pouco se fala no que acontece quando
temos **cetenas de milhares** ou até mesmo **MILHÕES** de dispositivos enviando
requisições de forma constante dentro de uma mesma infraestrutura. O que
acontece é mais ou menos isso aqui:

<img 
  src="https://media1.tenor.com/m/qg324pNzm50AAAAC/server-is-fine-burn.gif"
  alt="Servidor pegando fogo"
  style={{ 
    display: 'block',
    marginLeft: 'auto',
    maxHeight: '30vh',
    marginRight: 'auto'
  }} 
/>
<br/>

O objetivo desse módulo e do projeto é resolver esse problema. Como? Criando
uma infraestrutura capaz de suportar todas essas requisições **com segurança**.
Basicamente vamos falar de escalabilidade e segurança no contexto de internet
das coisas em uma escala compatível com cidades inteligentes.

### 1.3. Assuntos abordados

Segue uma lista dos principais assuntos que vamos abordar:

* Modelos de comunicação em rede (OSI TCP/IP);
* Protocolo MQTT;
* Bancos de dados relacionais e não relacionais;
* Segurança da informação;
* Sistemas de processamento de eventos complexos por filas;
* Business Intelligence; e
* Armazenamento de arquivos a longo prazo.

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

<div style={{ display: 'flex', justifyContent: 'center', alignItems: 'center',
margin: '0' }}>
    <table>
        <tr>
            <th>Ícone</th>
            <th>Descrição</th>
        </tr>
        <tr>
            <td style={{textAlign: "center", verticalAlign: "middle"}}>
                <img src="icons/artefato.svg"/>
            </td>
            <td style={{textAlign: "left", verticalAlign: 'top', paddingTop:
            '25px' }}>
                Significa que a seção vai conter o detalhamento de um ou mais
                **artefatos** do metaprojeto.
            </td>
        </tr>
        <tr>
            <td style={{textAlign: "center", verticalAlign: "middle"}}>
                <img src="icons/autoestudo.svg"/>
            </td>
            <td style={{textAlign: "left", verticalAlign: 'top', paddingTop:
            '25px' }}>
                Significa que a seção é um autoestudo **obrigatório**.
            </td>
        </tr>
        <tr>
            <td style={{textAlign: "center", verticalAlign: "middle"}}>
                <img src="icons/opcional.svg"/>
            </td>
            <td style={{textAlign: "left", verticalAlign: 'top', paddingTop:
            '25px' }}>
                Significa que a seção é um autoestudo **opcional**.
            </td>
        </tr>
        <tr>
            <td style={{textAlign: "center", verticalAlign: "middle"}}>
                <img src="icons/ponderada.svg"/>
            </td>
            <td style={{textAlign: "left", verticalAlign: 'top', paddingTop:
            '25px' }}>
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

A seção de **introdução** é, bem, essa aqui. Ela é a página inicial do material e
deve ser o ponto de partida para qualquer um que não tenha visto esse material
antes. Tendo dito isso, se você nunca leu esse material antes e não está lendo
isso, não tem muito o que eu possa fazer =-(

A seção dos **artefatos** contem uma descrição **detalhada** de todos os artefatos
do metaprojeto que competem à area de computação. Aqui você não vai encontrar a
descrição dos artefatos de UX ou Negócios. Para isso, visite a
[Adalove](https://adalove.inteli.edu.br/). Na subseção de artefatos dentro da
seção de introdução (de novo, é essa aqui em que estamos agora =D) você vai
encontrar um resumo de cada sprint e um link para a página que detalha os
artefatos.

A seção de **setup das ferramentas** contem todos os autoestudos que não são
exatamente o cerne do módulo, mas são tutoriais úteis para que você possa
configurar seu ambiente de trabalho. Aqui você vai encontrar coisas como
tutoriais para instalação de sistemas operacionais, ferramentas/bibliotecas,
seu repositório no github e até mesmo indicações de cursos/vídeos para aprender
as linguagens principais vistas por aqui.

A seção de **conteúdo dos encontros** é, na verdade, um conjunto de seções.
Elas aparecem por último no menu que fica aqui à esquerda (`<<`). Você vai
conseguir perceber que se trata de uma seção de material do encontro pois todas
elas tem "EXX" em seu título, sendo o `XX` substituído por algum número. Por
exemplo: `E1 - Modelos de comunicação e MQTT`.

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


## 3. Artefatos

### 3.1. [Sprint 1](/sprint1) 

Aqui você vai ter que investir um tempo para entender de fato o problema
apresentado pelo parceiro e começar a desenhar a sua solução. A essa altura, já
deve ser algo que você está acostumado a fazer em projetos. Lembre-se que a
ferramenta e as tecnologias **nunca** precedem a necessidade do
cliente/parceiro. Se você já decidiu o que usar antes de conversar com o
parceiro, seu projeto começou mal.

O que você vai precisar entregar na parte técnica de computação? Um simulador
de dispositivos MQTT que consiga enviar dados que se adequem às especificações
dos dispositivos do parceiro. Os itens de entrega são:

* Canvas de proposta de valor;
* Análise PESTEL;
* TAM, SAM e SOM;
* Matriz de risco e plano de contingência;
* Plano de comunicação;
* Análise financeira do projeto;
* Personas;
* Mapa de jornada de usuário;
* Requisitos funcionais e não funcionais;
* Planejamento de validação dos requisitos definidos;
* Simulador de dispositivos MQTT; e
* Testes e validação do simulador.

### 3.2. [Sprint 2](/sprint2) 

Já não deve ser novidades para vocês também o fato de que prezamos sempre por
um **desenvolvimento em espiral** de verdade e não uma **cascata ágil**. Sendo
assim, já nessa sprint vamos entregar uma versão completa do projeto. O que
isso significa? Significa que seu projeto já vai ter que contemplar:

* Um broker MQTT implementado com funcionalidades básicas de segurança
  (autenticação e autorização);
* Um sistema de armazenamento de dados em banco de dados relacional (sem
  nenhuma mudança ou novidade com relação ao que vocês usaram no passado); e
* Um dashboard simples para que seja possível que um time de negócios ou data
  analytics consiga acompanhar os principais indicadores dos dispositivos
  conectados.

Aqui também surgem duas novas demandas: o foco em **desenvolvimento de testes**
e **correção dos problemas** encontrados na versão anterior do projeto pelo
professor orientador.

A lista de itens de entrega pode ser vista abaixo:
* Análise de impacto ético;
* Broker MQTT com autenticação e autorização de acesso;
* Implementação de armazenamento de dados em um DB estruturado;
* Versão inicial do dashboard;
* Validação dos sistemas de autenticação e autorização do broker MQTT; e
* Correção dos issues registrados relativos à sprint passada. 


### 3.3. [Sprint 3](/sprint3) 

Se tudo deu certo no seu sprint 2, temos agora um sistema completo, mas ainda
**muito longe** de atender aos requisitos de **segurança** e escalabilidade. A
partir de agora, vamos pinçar os subsistemas criados e ir melhorando aos poucos
sua implementação. No sprint 3, vamos focar na **segurança** do **banco de
dados** e também na elaboração de um **dashboard** mais completo, levando em
consideração o que o professor orientador e o parceiro observaram a respeito do
dashboard anterior e também adicionando autenticação de usuários.

Os itens de entrega são:
* Planejamento de hiperconectividade para cidades inteligentes;
* Implementação de armazenamento não estruturado com segurança;
* Evolução do dashboard (solução sob medida e autenticação);
* Validação da segurança do sistema de armazenamento;
* Validação do sistema de autenticação do dashboard; e 
* Correção dos issues registrados relativos à sprint passada. 

### 3.4. [Sprint 4](/sprint4) 

Agora que já temos um sistema com um bom dashboard e com segurança comprovada
na interface de usuário, armazenamento e transmissão de dados, está na hora de
focar no problema da **escalabilidade**. Para isso, vamos adiconar ao sistema
um componente que nos permita **processar eventos complexos de forma
escalável**. Os itens de entrega são:

* Implementação e integração de sistema de processamento de eventos complexos;
* Validação dos requisitos não funcionais de escalabilidade; e
* Correção dos issues registrados relativos à sprint passada. 

### 3.5. [Sprint 5](/sprint5) 

Se você chegou até aqui, significa que você já tem um projeto com toda a sua
funcionalidade principal funcionando (talvez não esteja impecável, mas tem que
funcionar). Se essa descrição não se aplica ao seu projeto, o nome desse sprint
vai ser **catch up**. Se não, aqui você vai fazer o **refinamento e correção de
erros**. Os itens de entrega são:

* Ajuste fino dos sistemas e sua integração;
* Correção dos issues registrados durante todo o módulo;
* Apresentação final do projeto; e
* Documentação completa.

## 4. Bibliografia

### 4.1. Bibliografia básica

ARRUDA, A. J. V. Design e inovação social. São Paulo: Blucher, 2017.
https://integrada.minhabiblioteca.com.br/#/books/9788580392647

Este livro é uma obra importante que estabelece a conexão entre design e sua função social, explorando como o design pode ser utilizado como ferramenta para promover a inovação social e o desenvolvimento sustentável. Ao apresentar estudos de caso e teorias, a obra ajuda a entender como o design pode transcender a estética e funcionar como agente transformador da sociedade.

BARBIERI, Carlos. Governança de dados. Rio de Janeiro: Editora Alta Books, 2020.
https://integrada.minhabiblioteca.com.br/#/books/9788550815435

O livro aborda tópicos essenciais como histórico dos dados, governança de dados, gestão prática, políticas e padrões, visão Big Data e Normal Data, leis de proteção de dados, ética nos dados, metadados, qualidade dos dados, agilidade, CDO, PMBOK e DMBoK, dados como ativos organizacionais, e a importância do gerenciamento.

BATISTA, Claudia R.; ULBRICHT, Vania R.; FADEL, Luciane M. Design para acessibilidade e inclusão. São Paulo: Blucher, 2017. E-book. ISBN 9788580393040.
https://integrada.minhabiblioteca.com.br/#/books/9788580393040

O livro é um recurso indispensável, já que aborda um aspecto crucial do design contemporâneo: a acessibilidade. A obra articula métodos e práticas para garantir que os produtos sejam não apenas funcionais e atraentes, mas também acessíveis a todos, com abordagem inclusiva no design, ampliando nosso entendimento da responsabilidade social do design.

CHIAVENATO, I. Administração de recursos humanos: gestão humana, fundamentos básicos. 9. ed. Barueri: Atlas, 2022.
https://integrada.minhabiblioteca.com.br/#/books/9786559771233

Este livro aborda as transformações da administração dos recursos humanos que antes eram meros recursos organizacionais, agora são inseridas nas empresas como valores humanos, munidas de competências e conhecimentos, aptas a se encaixarem em um contexto complexo e mutável e a aliarem produtividade, qualidade e competitividade, atuando como parceiras das organizações.

FOROUZAN, B. A. Comunicação de dados e redes de computadores. Porto Alegre: AMGH, 2010.
https://integrada.minhabiblioteca.com.br/#/books/9788563308474

O livro abrange tópicos essenciais como comunicação de dados e redes, camada física, camada de enlace de dados, camada de rede, camada de transporte, camada de aplicação e segurança. O curso fornecerá uma compreensão aprofundada dos princípios, protocolos e tecnologias relacionados às redes de computadores, preparando os estudantes para lidar com os desafios da engenharia de redes e comunicação de dados.

GASALLA, J. M. A nova gestão de pessoas: o talento executivo. São Paulo: Saraiva, 2007.
https://integrada.minhabiblioteca.com.br/#/books/9788502099852

A Nova Gestão de Pessoas é um compêndio do conhecimento útil na gestão de recursos humanos: é uma obra que combina o rigor e, ao mesmo tempo, a boa pedagogia, o que faz com que o leitor entenda com facilidade temas complexos. Ao considerar que cada vez mais a gestão de pessoas assume uma dimensão crítica para o sucesso das empresas, o livro traz informações essenciais para a gestão de pessoas.

LIMA, A. F. de. Design de produto. São Paulo: Platos Soluções Educacionais S.A., 2021.
https://integrada.minhabiblioteca.com.br/#/books/9786589965701

Este livro é uma ótima referência para pesquisa, proporcionando insights teóricos e práticos essenciais no campo do design de produtos, oferecendo uma perspectiva atualizada e inovadora sobre o assunto, abordando desde os princípios fundamentais até a aplicação no mercado contemporâneo.

MARRAS, J. P. Administração de recursos humanos. 15. ed. São Paulo: Saraiva, 2016.
https://integrada.minhabiblioteca.com.br/#/books/9788547201098

O best seller de Jean Pierre Marras chega à sua 15ª edição, com conteúdo revisto, atualizado e ampliado, pronto para servir como livro-texto para estudantes e como base para profissionais da área de administração de recursos humanos.

MARTINS, Júlio S. et al. Processamentos de linguagem natural. Porto Alegre: SAGAH, 2020.
https://integrada.minhabiblioteca.com.br/#/books/9786556900575

Este livro oferece um aprofundamento em inteligência artificial e processamento de linguagem natural. Também são cobertas habilidades importantes, como redação técnica para a criação de manuais, guias de instrução e documentação de API. A abordagem integrada do livro ajudará a entender como os sistemas de linguagem natural são construídos, testados e usados, oferecendo uma base sólida.

PRESSMAN, Roger S.; MAXIM, Bruce R. PRESSMAN, Roger S. Engenharia de software: uma abordagem profissional. 9. ed. Porto Alegre: AMGH, 2021.
https://integrada.minhabiblioteca.com.br/#/books/9786558040118

O livro fornece conhecimento abrangente sobre modelos de processo de software, conceitos de design, garantia de qualidade de software, gerenciamento de projetos e tendências emergentes em engenharia de software, essenciais para estudantes de engenharia.

SILVA, M. L. da. Administração de departamento de pessoal. 15. ed. São Paulo: Érica, 2017.
https://integrada.minhabiblioteca.com.br/#/books/9788536529967

Obra atualizada em conformidade com a legislação trabalhista brasileira até junho de 2017, trata dos direitos e das obrigações de empregados e empregadores e traz como novidade as obrigações fiscais e trabalhistas antes da implantação do eSocial, enfatizando o novo cenário proposto após a implantação desse sistema, prevista para janeiro de 2018.

SIPSER, M. Introdução à Teoria da Computação: Trad. 2ª ed. norte-americana. São Paulo: Cengage Learning Brasil, 2007.
https://integrada.minhabiblioteca.com.br/#/books/9788522108862

Traz os tópicos fundamentais da Teoria da Computação, como autômatos, computabilidade e complexidade. O livro proporciona uma compreensão sólida dos princípios matemáticos subjacentes à ciência da computação, preparando os alunos para enfrentar os desafios relacionados à computabilidade e análise de complexidade em projetos de engenharia.

ULRICH, D. A transformação do RH: construindo os recursos humanos de fora para dentro. Porto Alegre: Bookman, 2011.
https://integrada.minhabiblioteca.com.br/#/books/9788577808434

Essa obra defende que o maior desafio dos profissionais do RH na atualidade é ajudar suas respectivas organizações a ter sucesso. Concentrar-se na função externa do RH é fundamental para isso, entendendo o que clientes e investidores precisam e como o RH pode contribuir para o sucesso da empresa.

### 4.2. Bibliografia complementar

ARAÚJO NETO, Antônio Palmeira de. Governança de dados. São Paulo: Platos Soluções Educacionais, 2021.
https://integrada.minhabiblioteca.com.br/#/books/9786589881476

Este livro é fundamental para entender os princípios e práticas da governança efetiva de dados. Aborda temas como qualidade de dados, metadados, privacidade e conformidade regulatória, oferecendo uma visão abrangente sobre como gerenciar e utilizar os dados de forma estratégica e segura.

DIAS, A. S. Processamento de linguagem natural. São Paulo: Platos Soluções Educacionais S.A., 2021.
https://integrada.minhabiblioteca.com.br/#/books/9786589881995

O livro traz uma compreensão dos conceitos e técnicas fundamentais do PLN, como tokenização, análise morfológica, modelos de linguagem e processamento de texto. Com uma abordagem prática e exemplos reais, é uma referência para os estudantes que desejam explorar o processamento automatizado da linguagem e suas diversas aplicações no mundo moderno.

DIVERIO, T. A.; MENEZES, P. B. Teoria da computação: máquinas universais e computabilidade. 3. ed. Porto Alegre: Bookman, 2009.
https://integrada.minhabiblioteca.com.br/#/books/9788577808311

Este livro é essencial para compreender os fundamentos da computação teórica, explorando temas como autômatos, linguagens formais, máquinas de Turing e computabilidade, fornecendo uma base sólida para estudantes e pesquisadores interessados em entender os limites e possibilidades da computação.

LESKO, J. Design industrial: guia de materiais e fabricação. 2. ed. São Paulo: Blucher, 2012.
https://integrada.minhabiblioteca.com.br/#/books/9788521206576

Este livro, em sua segunda edição, oferece uma visão abrangente dos materiais e processos de fabricação empregados na indústria do design. Ao desvendar a complexidade desses elementos, a obra permite uma compreensão mais profunda de como as escolhas de materiais e métodos de fabricação impactam o produto final.

MARRAS, J. P. Administração de recursos humanos. 15. ed. São Paulo: Saraiva, 2016.
https://integrada.minhabiblioteca.com.br/#/books/9788547201098

O best seller de Jean Pierre Marras chega à sua 15ª edição, com conteúdo revisto, atualizado e ampliado, pronto para servir como livro-texto para estudantes e como base para profissionais da área de administração de recursos humanos.

RUSSELL, S. J.; NORVIG, P. Inteligência Artificial: Uma Abordagem Moderna. Porto Alegre: Grupo GEN, 2022.
https://integrada.minhabiblioteca.com.br/#/books/9788595159495

Este livro abrange tópicos como aprendizado de máquina, processamento de linguagem natural e visão computacional, fornecendo uma visão abrangente e prática, sendo uma referência essencial para os estudantes interessados em explorar as aplicações modernas da IA.

SILVA, M. L. da. Administração de departamento de pessoal. 15. ed. São Paulo: Érica, 2017.
https://integrada.minhabiblioteca.com.br/#/books/9788536529967

Obra atualizada em conformidade com a legislação trabalhista brasileira até junho de 2017, trata dos direitos e das obrigações de empregados e empregadores e traz como novidade as obrigações fiscais e trabalhistas antes da implantação do eSocial, enfatizando o novo cenário proposto após a implantação desse sistema, prevista para janeiro de 2018.

SOARES, M. M. Metodologia de ergodesign para o design de produtos: uma abordagem centrada no humano. São Paulo: Blucher, 2021.
https://integrada.minhabiblioteca.com.br/#/books/9786555061659

O livro aborda a importância de considerar a ergonomia na concepção do design, garantindo a eficácia, segurança e conforto dos produtos, enfatizando uma abordagem centrada no humano, que é crucial para a criação de produtos verdadeiramente úteis e agradáveis de usar.

SOUZA, D. C de; SOARES, J. A.; SILVA, F. R. da; et al. Gerenciamento de Redes de Computadores. Porto Alegre: SAGAH, 2021.
https://integrada.minhabiblioteca.com.br/#/books/9786556901411

O livro é fundamental para entender os princípios e práticas do gerenciamento eficiente de redes. Abrange conceitos como monitoramento, segurança, protocolos de rede e soluções de problemas, oferecendo uma visão abrangente e atualizada das melhores práticas nessa área em constante evolução.

VETORAZZO, A. S. Engenharia de software. Porto Alegre: SAGAH, 2018.
https://integrada.minhabiblioteca.com.br/#/books/9788595026780

O livro abrange tópicos como processos de desenvolvimento, requisitos, modelagem, testes e gerenciamento de projetos. Com uma abordagem prática e exemplos, é uma referência para os estudantes que desejam adquirir conhecimentos sólidos e atualizados sobre as melhores práticas na área de desenvolvimento de software.
