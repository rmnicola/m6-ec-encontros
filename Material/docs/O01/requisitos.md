---
title: Criando requisitos
sidebar_position: 4
sidebar_class_name: autoestudo
slug: /requisitos
---

import Admonition from '@theme/Admonition';

# Criando requisitos

## 1. Requisitos funcionais

Requisitos funcionais são especificações detalhadas que descrevem as funções,
características e interações que um sistema ou software deve possuir. Eles
definem o que o sistema deve fazer em resposta a entradas específicas ou em
condições determinadas. Em outras palavras, os requisitos funcionais descrevem
as funcionalidades que o sistema deve oferecer para atender às necessidades e
expectativas dos usuários e dos
stakeholders.

Por exemplo, para um sistema de vendas online, um requisito funcional pode ser:
"O sistema deve permitir que os usuários adicionem produtos ao carrinho de
compras e realizem o checkout para finalizar a
compra".

### 1.1. Como Definir Requisitos Funcionais

1. **Entendimento do Negócio:** Antes de tudo, é crucial entender o domínio do
negócio e os objetivos do projeto. Isso fornece um contexto para a definição dos
requisitos.
 
2. **Reuniões e Workshops:** Organize sessões de brainstorming com stakeholders,
incluindo gerentes de projeto, desenvolvedores, designers e, claro,
representantes dos usuários finais.

3. **Use Casos e Histórias de Usuários:** Descreva cenários específicos de como
os usuários interagirão com o sistema. Isso ajuda a visualizar as
funcionalidades necessárias e a identificar possíveis lacunas.

4. **Prototipagem:** Crie protótipos ou mockups do sistema para visualizar como
ele funcionará. Isso pode ajudar a identificar requisitos adicionais e a obter
feedback antecipado.

### 1.2. Importância do Cliente na Definição de Requisitos Funcionais

O cliente, seja ele um usuário final ou um representante de uma organização
cliente, é fundamental na definição de requisitos funcionais. Aqui estão as
razões:

1. **Perspectiva do Usuário:** O cliente oferece uma perspectiva única, a do
usuário. Eles podem fornecer insights sobre o que realmente precisam e esperam
do sistema.

2. **Validação:** Ao envolver o cliente no processo de definição, é possível
validar os requisitos à medida que são desenvolvidos, garantindo que o sistema
atenda às suas necessidades.

3. **Priorização:** O cliente pode ajudar a priorizar requisitos com base em sua
importância e valor para o negócio.

4. **Redução de Riscos:** Ao entender e documentar corretamente os requisitos do
cliente desde o início, reduz-se o risco de retrabalho, custos adicionais e
insatisfação do cliente no final do projeto.

5. **Relacionamento e Confiança:** Envolver o cliente no processo fortalece o
relacionamento e constrói confiança, pois eles veem que suas opiniões e
necessidades são valorizadas.

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

### 2.2. Criação de Testes para Validar RNFs de um robô de serviço

- **Testes de Performance:** 
  - Simule várias cargas de trabalho no chatbot e LLM para garantir que eles
possam lidar com múltiplas solicitações simultaneamente.
  - Teste a velocidade de movimento e resposta do robô em diferentes cenários.

- **Testes de Portabilidade:** 
  - Faça o robô operar em diferentes superfícies e ambientes para verificar sua
adaptabilidade.

- **Testes de Confiabilidade:** 
  - Execute o robô continuamente por longos períodos para identificar possíveis
falhas ou problemas de desempenho.

- **Testes de Usabilidade:** 
  - Realize testes com usuários reais para obter feedback sobre a interface e a
experiência do usuário.
  - Avalie a precisão e relevância das respostas do chatbot e LLM.

- **Testes de Manutenibilidade:** 
  - Implemente atualizações fictícias para o software do chatbot e LLM e
verifique a facilidade e eficácia do processo.

- **Testes de Autonomia:** 
  - Monitore a duração da bateria do robô em diferentes condições de uso.
  - Teste o processo de carregamento, seja manual ou autônomo.
