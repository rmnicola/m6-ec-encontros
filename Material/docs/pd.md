---
title: Prova 1
sidebar_position: 15
slug: /here-comes-pd-pretty-dog
unlisted: true
---

# P1 Módulo 6 EC 2024

<img 
  src="https://media1.tenor.com/m/S0hKfhXsX7AAAAAC/apo-space-brothers.gif"
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

Você é um engenheiro de computação recém-contratado pela RoboTech Solutions,
uma empresa inovadora que desenvolve tecnologias de ponta para robótica. A
RoboTech está focada em criar sistemas de controle avançados para robôs, usados
em várias indústrias, desde a manufatura até serviços domésticos. Como parte do
seu treinamento inicial, você foi designado para um projeto que envolve o
desenvolvimento de uma interface de linha de comando (CLI) para controlar um
simulador de robô tartaruga, o Turtlesim, utilizando o ROS2 (Robot Operating
System 2).

O objetivo do projeto é criar uma aplicação que permita aos operadores enviar
comandos de movimento para a tartaruga no Turtlesim. Esses comandos devem ser
enfileirados e executados sequencialmente, garantindo que cada comando seja
concluído antes que o próximo seja iniciado. Você utilizará a estrutura de
dados deque para gerenciar essa fila de comandos de forma eficiente. Seu papel
é essencial para garantir que o sistema de controle funcione de maneira
ordenada e eficaz, proporcionando uma experiência de uso fluida para os
operadores.

### 1.1. Deque

Para essa atividade, você deve utilizar uma implementação de fila em Python.
Sugere-se o uso do deque. O `deque` (Double-Ended Queue) é uma estrutura de
dados altamente eficiente disponível no módulo `collections` do Python. Ele
permite a inserção e remoção de elementos tanto no início quanto no final da
fila, com complexidade O(1) para ambas as operações. Isso o torna uma escolha
excelente para situações onde é necessário adicionar ou remover elementos em
ambas as extremidades da fila de maneira rápida.

**Instalação**

Não precisa instalar, o módulo `collections` já vem por padrão no Python.

**Exemplo de uso**

```python showLineNumbers title="test_deque.py"
from collections import deque

# Criação de um deque vazio
dq = deque()

# Adicionando elementos no final
dq.append('A')
dq.append('B')
dq.append('C')
print(f"Deque após append: {dq}")

# Adicionando elementos no início
dq.appendleft('X')
dq.appendleft('Y')
print(f"Deque após appendleft: {dq}")

# Removendo elementos do final
dq.pop()
print(f"Deque após pop: {dq}")

# Removendo elementos do início
dq.popleft()
print(f"Deque após popleft: {dq}")

# Rotacionando o deque
dq.rotate(1)
print(f"Deque após rotate(1): {dq}")

# Verificando o comprimento do deque
print(f"Tamanho do deque: {len(dq)}")
```

**Mais informações**

Quer saber mais sobre o deque? O melhor lugar é na [documentação
oficial](https://docs.python.org/3/library/collections.html#collections.deque)

### 1.2. Desafio

Para resolver o problema descrito no enunciado, você vai precisar implementar
dois sistemas distintos:

#### 1.2.1 CLI

Uma CLI capaz de ler os comandos dados pelo usuário no seguinte formato:

```bash
vx vy vtheta tempo_em_ms
```

Em que `vx` é a velocidade da tartaruga em x, `vy` é a velocidade da tartaruga
em y, `vtheta` é a velocidade angular da tartaruga em torno do eixo z e
`tempo_em_ms` indica o tempo que a tartaruga deve ficar executando o comando.

Abaixo um exemplo de comando válido:

```bash
0.0 1.0 0.0 1000
```

O comando acima deve mover a tartaruga com vy = 1.0 durante 1000ms.

Alternativamente, a CLI pode receber os comandos no formato de argumentos de
linha de comando. Exemplo:

```bash
python3 cli.py --vx 0.0 --vy 1.0 --vt 0.0 -t 1000
```

Fica também a seu critério criar valores padrão para os argumentos, podendo
assim simplificar o comando acima para:

```bash
python3 cli.py --vy 1.0 -t 1000 # Valores de vx e vt são 0.0 por padrão
```

#### 1.2.2 Publisher e fila de comandos

O publisher deve ser um nó ROS2 (não precisa ser um pacote) que recebe os
comandos enviados pelo usuário e os armazena em uma estrutura do tipo fila.
Sempre que a tartaruga não estiver executando um comando, o nó deve buscar um
novo comando na frente da fila e removê-lo para que possa executá-lo. Quando
não houver mais nenhum comando na fila para executar, o nó envia a velocidade 0
para a tartaruga (para mantê-la parada).

## 2. Padrão de entrega

:::warning

Esses são os critérios mínimos para que eu considere a atividade como entregue.
Fique atento, pois o não cumprimento de qualquer um desses critérios pode, no
melhor dos casos, gerar um desconto de nota e, no pior deles, invalidar a
atividade.

:::

1. A atividade deve ser feita em um repositório **aberto** no github. Seu link
   deve ser fornecido na resposta do forms da prova;
2. No README do repositório deve ter instruções claras de como instalar e rodar
   o sistema criado, comandos em blocos de código e uma expliação sucinta do
   que fazem;
3. Ainda no README, deve haver um vídeo/imagens demonstrando plenamente o
   funcionamento do sistema criado;
4. O prazo para a entrega desta atividade é até o dia 17/05/2024 às 16h00

## 3. Padrão de qualidade

### 3.1. CLI (até 3,0 pontos)

<table>
  <tr>
    <th>Atende<br/>(2,0 - 3,0)</th>
    <th>Quase lá<br/>(1,0 - 2,0)</th>
    <th>Insuficiente<br/>(0,5 - 1,0)</th>
    <th>Desalinhado<br/>(0,0 - 0,5)</th>
  </tr>
  <tr>
    <td>O sistema permite que o usuário envie o seu comando como argumentos de
    linha de comando, o que torna a operação não bloqueante do sistema
    possível (sugestão: utilize
    [argparse](https://docs.python.org/3/library/argparse.html) e separe o
    código da CLI do publisher/fila, com a CLI publicando os comandos do
    usuário em um tópico da sua escolha)</td>
    <td>Interface recebe comandos do usuário e verifica se está no padrão
    esperado, exibindo uma mensagem de ajuda quando o usuário tentar enviar um
    comando inválido.</td>
    <td>A interface implementada permite que o usuário envie comandos no
    formato especificado, mas não verifica se o comando enviado está dentro do
    padrão esperado ou não.</td>
    <td>Entrega fora de contexto.</td>
  </tr>
</table>

### 3.2. Publisher e fila (até 7,0 pontos)

<table>
  <tr>
    <th>Supera<br/>(6,0 - 7,0)</th>
    <th>Atende<br/>(4,0 - 6,0)</th>
    <th>Quase lá<br/>(2,0 - 4,0)</th>
    <th>Insuficiente<br/>(1,0 - 2,0)</th>
    <th>Desalinhado<br/>(0,0 - 1,0)</th>
  </tr>
  <tr>
    <td>A execução dos comandos é feita de uma maneira que, mesmo quando o
    sistema está executando um comando, é possível adicionar um novo comando à
    fila (novamente sugiro utilizar um timer para isso em vez do sleep).</td>
    <td>O nó é capaz de buscar comandos na frente da fila e executa-os pelo
    tempo especificado (sugestão de utilizar um
    [timer](https://design.ros2.org/articles/clock_and_time.html) e reenviar o
    comando periódicamente até o tempo definido pelo usuário).</td>
    <td>Há uma fila capaz de armazenar os comandos recebidos e enviá-los em
    ordem para o tópico de interação com a tartaruga, mas o sistema não garante
    que a tartaruga executa o comando pelo tempo definido pelo usuário.</td>
    <td>O nó criado inicializa um publisher corretamente e é capaz de interagir
    com o `cmd_vel` do turtlesim. No entanto, não há nenhuma implementação de
    fila de comandos.</td>
    <td>Entrega fora de contexto.</td>
  </tr>
</table>
