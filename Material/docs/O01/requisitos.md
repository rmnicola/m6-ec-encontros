---
title: Criando requisitos
sidebar_position: 4
sidebar_class_name: autoestudo
slug: /requisitos
---

import Admonition from '@theme/Admonition';

# Criando requisitos

## 1. Requisitos funcionais

O que o seu programa **faz**? Essa é a simples pergunta que deve ser respondida
pelos seus requisitos **funcionais**. Essa pergunta **sempre** vai ser guiada
pelo usuário do seu sistema. Ainda não fez o dever de casa de identificar
**quem** é o seu usuário final? Volte duas casas. Já **decidiu** que tecnologia
quer utilizar antes mesmo de ler esse texto ou definir quem é o usuário? Vá
para a cadeia. Só volte aqui quando fizer as coisas na **ordem certa**.

Opa, voltou? Legal. Vamos voltar a falar de requisitos funcionais com uma
definição um pouco mais formal:

Requisitos funcionais são **especificações** detalhadas que descrevem as
**funções**, **características** e **interações** que um sistema ou software
deve possuir. Eles definem o que o sistema deve fazer em resposta a entradas
específicas ou em condições determinadas. Em outras palavras, os requisitos
funcionais descrevem as funcionalidades que o sistema deve oferecer para
atender às necessidades e expectativas dos usuários e dos stakeholders.

**Exemplo**

Para um sistema de vendas online, um requisito funcional pode ser:

"O sistema deve permitir que os usuários adicionem produtos ao carrinho de
compras e realizem o checkout para finalizar a compra".

### 1.1. Como Definir Requisitos Funcionais

Se você está aqui, é pois já sabe **quem é** o seu usuário. Não sabe ainda?
Como você chegou até aqui!? Volte duas casas!

Voltou? Calma que ainda não acabou. Já fez a sua análise de negócios? Não? Como
você quer definir requisitos funcionais de um projeto que você ainda nem sabe
se tem **valor**, é **viável** e se encaixa no **modelo de negócios** do seu
cliente? Volte uma casa!

Opa, já fez o dever de casa? Então tenho uma boa notícia; você já tem **tudo**
o que precisa para definir os requisitos funcionais. Se você fez **user
stories** bem feitas e que cobrem bem o escopo de utilização das suas personas,
significa que é só sentar com o seu grupo e analisar essas user stories. Os
requisitos funcionais mais óbvios vão estar logo ali, quase dando um beijo em
vocês de tão na cara que eles estavam. Quer um exemplo? Então toma um user
story em formato de [haiku](https://edtl.fcsh.unl.pt/encyclopedia/haiku):

```bash
Operador atento,
Explora vãos com destreza —
Tubos sem segredo.
```

:::tip

<img 
  src="https://64.media.tumblr.com/7ee84119b2a167d95b1f19810b908a4d/tumblr_inline_nqkkc1pfiN1qeyssh_500.gifv"
  alt="Apo shitposting"
  style={{ 
    display: 'block',
    marginLeft: 'auto',
    maxHeight: '20vh',
    marginRight: 'auto'
  }} 
/>
<br/>

Formatar os seus user stories como haikus não é uma boa prática. Só estou
fazendo uma graça pra arrancar um sorrisinho seu durante esse autoestudo

:::

Um bom user story deve responder **quem** é o usuário, **qual** a sua
**necessidade** e qual seu **objetivo**. O meu haiku não é tão claro, então não
é um bom user story (mas tem o valor do entretenimento). No entanto, podemos
extrair essas informações para nosso exemplo:

* **Quem**: operador atento;
* **Necessidade**: explorar vãos com destreza
* **Para quê**: ...tubos sem segredo? (Ok, vou adicionar uma info a mais aqui:
  verificar tubulações que ele **não consegue acessar**)

A partir disso, podemos focar na exploração **com destreza** para definir como um requisito funcional:

```
O sistema deve permitir a teleoperação de um robô móvel de forma responsiva.
```

Viu? Tem muita informação nesses user stories. Vai olhar para eles com cuidado!

Beleza, mas tudo vai sair dos user stories? Idealmente, sim. O problema é que
isso dependeria de user stories **perfeitos** que falam sobre **todas** as
necessidades do usuário. Nem o usuário sabe todas as necessidades do usuário.
Sendo assim, use o seu **bom senso** para cobrir funcionalidades que você
consegue inferir que são necessárias. Exemplo? *O sistema deve ter uma forma de
controle de acesso*. Pode ser que isso não apareça no seu estudo de usuário,
mas é absolutamente necessário para a segurança do sistema.

Sério? Só bom senso. Não exatamente. Nessa primeira etapa sim, é bom senso e
estimativa. A partir do momento em que você tiver um protótipo funcional, você
precisa **testar** esse protótipo com os olhos bem atentos, pois outros
requisitos funcionais ainda não mapeados podem começar a se manifestar.

## 2. Requisitos não funcionais

<Admonition 
    type="info" 
    title="Autoestudo">

[Requisitos não
funcionais](https://www.altexsoft.com/blog/non-functional-requirements/)

<div style={{ textAlign: 'center' }}>
    <iframe 
        style={{
            display: 'block',
            margin: 'auto',
            width: '100%',
            height: '50vh',
        }}
        src="https://www.youtube.com/embed/fc-5HJPBZMQ" 
        frameborder="0" 
        allowFullScreen>
    </iframe>
</div>

</Admonition>

O artigo acima aborda os requisitos não funcionais (RNFs) em software e
sistemas. Ele pode ser resumido da seguinte maneira: 

- **Introdução**: Assim como uma motocicleta tem características além de
simplesmente se mover de um ponto A para um ponto B, um software também tem
requisitos que não estão diretamente relacionados à sua função principal, mas
são essenciais para atender às necessidades do usuário
final.

- **Definição**: Os requisitos não funcionais descrevem as capacidades
operacionais do sistema e tentam melhorar sua funcionalidade. Eles abordam
aspectos como velocidade, segurança, confiabilidade e integridade dos
dados.

- **Diferença com Requisitos Funcionais**: Enquanto os requisitos funcionais
definem o que um produto deve fazer (por exemplo, um mensageiro deve permitir a
edição de mensagens), os requisitos não funcionais especificam os atributos de
qualidade do
sistema.

- **Tipos de Requisitos Não Funcionais**:
  - **Performance e Escalabilidade**: Refere-se à rapidez com que o sistema
retorna resultados e como essa performance muda com cargas de trabalho
maiores.
  - **Portabilidade e Compatibilidade**: Aborda em quais hardwares, sistemas
operacionais e navegadores o software funciona e se ele é compatível com outras
aplicações.
  - **Confiabilidade, Manutenibilidade e Disponibilidade**: Estes tratam da
frequência de falhas críticas, o tempo necessário para corrigir problemas e a
probabilidade de o sistema estar acessível a qualquer
momento.
  - **Segurança**: Garante que todos os dados dentro do sistema sejam protegidos
contra ataques maliciosos ou acesso não
autorizado.
  - **Localização**: Define quão bem um sistema se alinha com o contexto do
mercado local, incluindo idiomas, leis, moedas, entre
outros.
  - **Usabilidade**: Aborda a facilidade de uso do produto.

- **Recomendações Gerais**:
  - Tornar os RNFs mensuráveis e testáveis.
  - Definir requisitos para componentes do sistema e não apenas para produtos
inteiros.
  - Vincular RNFs a objetivos de negócios.
  - Considerar limitações de terceiros e arquitetônicas.
  - Consultar padrões e guias existentes.

### 2.1. Requisitos não funcionais na robótica

Para definir RNFs para um projeto de robótica, deve-se começar por entender o 
contexto em que o robô será usado. Isso pode incluir ambientes como casas, 
escritórios, lojas ou espaços públicos. A seguir, deve-se considerar os 
seguintes questionamentos:

- **Performance e Escalabilidade:** 
  - Quão rápido o robô deve responder às solicitações do usuário?
  - Qual é a capacidade máxima de usuários simultâneos que o chatbot e o LLM
podem atender?

- **Portabilidade e Compatibilidade:** 
  - Em quais superfícies ou terrenos o robô deve operar? 
  - O robô precisa ser compatível com outros sistemas ou dispositivos?

- **Confiabilidade e Disponibilidade:** 
  - Qual é a taxa aceitável de falhas do robô?
  - O robô precisa funcionar 24/7 ou em horários específicos?

- **Segurança:** 
  - Como os dados do usuário serão protegidos?
  - Existem medidas para evitar que o robô seja hackeado ou manipulado?

- **Usabilidade:** 
  - Quão intuitivo é para os usuários interagirem com o robô e o chatbot?
  - O robô e o chatbot devem suportar múltiplos idiomas ou dialetos?

- **Manutenibilidade:** 
  - Com que facilidade o robô pode ser reparado ou atualizado?
  - Como as atualizações do software do chatbot e LLM serão implementadas?

- **Autonomia:** 
  - Qual é a duração da bateria do robô?
  - O robô pode se auto-carregar ou precisa ser carregado manualmente?
