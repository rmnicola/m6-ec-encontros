---
title: Prova 2
sidebar_position: 36
slug: /hon-ki-no-shippai
unlisted: true
---

# P2 Módulo 6 EC 2024

<img 
  src="https://64.media.tumblr.com/236f136f7878c4599ca846d2745e1f7e/tumblr_mxt1qu2mzO1src1c6o1_500.gif"
  alt="Pretty dog" 
  style={{ 
    display: 'block',
    marginLeft: 'auto',
    maxHeight: '30vh',
    marginRight: 'auto'
  }} 
/>
<br/>

## 1. Enunciado

Você foi contratado para desenvolver a versão inicial de um sistema de detecção
de emoções para a cerimônia da Bola de Ouro da PIFA de 2024. Este evento de
prestígio reúne as maiores estrelas do futebol mundial, e capturar suas reações
e emoções ao vivo é uma das ideias para tornar o evento mais interessante para
as audiências das novas gerações.

Nesta prova de conceito, sua tarefa é identificar todas as faces presentes no
primeiro plano de um vídeo de teste. O objetivo é garantir que apenas as faces
reais sejam detectadas, eliminando ao máximo a ocorrência de falsos positivos,
que poderiam comprometer a integridade dos dados e a eficácia do sistema.

:::warning IMPORTANTE

Para baixar o vídeo de teste, utilize [esse
link](https://drive.google.com/uc?export=download&id=18is4Ww5klq5FX2NuuLKNO3pItOJas43b)

Caso decida utilizar HAAR cascade, baixe os arquivos xml [desse
link](https://github.com/opencv/opencv/tree/master/data/haarcascades)

:::

Para que sua entrega seja aceita, há algumas condições **não negociáveis**:

1. Como a PIFA é uma organização mundialmente conhecida por sua transparência e
   lisura, foi solicitado que a sua prova de conceito seja entregue em um
   **repositório ABERTO** no Github para que seja possível que outros
   especialistas possam **avaliar** o código desenvolvido;
2. Em seu repositório deve haver **instruções claras** de como instalar e
   executar o sistema desenvolvido e uma amostra do **vídeo de exemplo com as
   faces detectadas**.
3. Além da entrega técnica feita, o README do repositório deve contar com
   respostas a algumas perguntas técnicas que vão auxiliar o desenvolvimento
   das próximas etapas do sistema.
4. A PIFA também é mundialmente conhecida por sua organização impecável dos
   eventos e regras do esporte. Portanto, há um **prazo não negociável** para
   entrega da prova de conceito. Este prazo é até o dia 14/06/2024 às 17h00.

## 2. Perguntas técnicas

### 2.1. 

Descreva de maneira concisa (um parágrafo no máximo) o funcionamento do método
de detecção escolhido.

### 2.2 

Considere as seguintes alternativas para resolver o problema de detecção de
faces:
* HAAR Cascade
* CNN
* NN Linear
* Filtros de correlação cruzada

Classifique-os (coloque em ordem) em termos de viabilidade técnica (se é
possível resolver o problema), facilidade de implementação e versatilidade da
solução. Justifique sua classificação.

### 2.3. 

Considerando as mesmas alternativas acima, faça uma nova classificação
considerando a viabilidade técnica para detecção de emoções através da imagem
de uma face.

### 2.4.

A solução apresentada ou qualquer outra das que foram listadas na questão 2.2.
tem a capacidade de considerar variações de um frame para outro (e.g. perceber
que em um frame a pessoa está feliz e isso influenciar na detecção do próximo
frame)? Se não, quais alterações poderiam ser feitas para que isso seja
possível?

### 2.5. (BONUS - não vale nada)

Quem ganha a bola de ouro 2024?

## 3. Padrão de qualidade

### 3.1. Perguntas técnicas (até 1,0 ponto cada)
<table>
  <tr>
    <th>Atende<br/>(0,8 - 1,0)</th>
    <th>Quase lá<br/>(0,3 - 0,8)</th>
    <th>Insuficiente<br/>(0,0 - 0,3)</th>
  </tr>
  <tr>
    <td>A resposta está correta e permite ao leitor entender claramente o seu
    impacto na sequência do projeto a ser desenvolvido. </td>
    <td>Resposta tecnicamente correta, mas falta contextualização com o tema da
    atividade.</td>
    <td>Resposta tecnicamente incorreta ou incompleta.</td>
  </tr>
</table>

### 3.2. Prova de conceito (até 6,0 pontos)

<table>
  <tr>
    <th>Supera<br/>(5,0 - 6,0)</th>
    <th>Atende<br/>(4,0 - 5,0)</th>
    <th>Quase lá<br/>(2,0 - 4,0)</th>
    <th>Insuficiente<br/>(0,5 - 2,0)</th>
    <th>Desalinhado<br/>(0,0 - 0,5)</th>
  </tr>
  <tr>
    <td>Além de detectar faces utilizando o método escolhido, há uma estratégia
    clara implementada para evitar falsos positivos (dica: se for usar haar
    cascade, considere utilizar um segundo cascade para validar a
    detecção)</td>
    <td>O sistema é capaz de detectar faces e anotar um retângulo (ou bounding
    box) em cada frame, gerando um novo vídeo com as "anotações" feitas.</td>
    <td>Há um sistema de detecção de faces, mas ele ainda não funciona
    perfeitamente (detecta faces onde não existe/não detecta faces que
    existem)</td>
    <td>O sistema desenvolvido é capaz de manipular o arquivo de vídeo,
    extraindo todos os frames. No entanto, não há nada sendo detectado ou
    anotado.</td>
    <td>Entrega fora de contexto.</td>
  </tr>
</table>

