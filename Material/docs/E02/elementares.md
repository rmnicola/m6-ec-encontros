---
title: Estruturas elementares
sidebar_position: 3
sidebar_class_name: autoestudo
slug: /elementares
---

# Estruturas elementares

As estruturas de dados elementares são aquelas que utilizam ponteiros de
maneira simples. Tipicamente elas empregam bastante o uso de [localidade
espacial](https://medium.com/@adamzerner/spatial-and-temporal-locality-for-dummies-b080f2799dd)
e não permitem o uso de múltiplos tipos de variáveis em uma mesma estrutura. As
estruturas elementares utilizam como base o **array**.

## 1. Arrays

Arrays são a forma mais simples de armazenar coleções ordenadas de elementos.
Embora haja muitas implementações de array nas diferentes linguagens de
programação, vamos focar na implementação de arrays em C, pois trata-se do
array "canônico". Em C, um array nada mais é do que um ponteiro que aponta para
o começo de uma sequência de elementos ordenados.

<img 
  src="https://media.geeksforgeeks.org/wp-content/uploads/20230302091959/Arrays-in-C.png"
  alt="O array" 
  style={{ 
    display: 'block',
    marginLeft: 'auto',
    maxHeight: '30vh',
    marginRight: 'auto'
  }} 
/>
<br/>

Na imagem acima, podemos ver um array definido da seguinte forma: `[2, 4, 8,
12, 16, 18]`. Se atribuirmos essa sequência a uma variável `v`, o que de fato
está sendo armazenado na variável é um ponteiro que aponta para a posição `0`
do array. Quer uma prova? Compile e rode o programa abaixo:

```C title="array_test.c"
#include <stdio.h>

int main() {
    int v[] = {2, 4, 8, 12, 16, 18};
    puts("**** Prova 1: v == &v[0] ****");
    printf("v \t= %p\n", v);
    printf("&v[0] \t= %p\n", &v[0]);
    puts("\n**** Prova 2: *(v+i) == v[i] ****");
    printf("*v \t= %d\n", *v);
    printf("v[0] \t= %d\n", v[0]);
    printf("*(v+2) \t= %d\n", *(v+2));
    printf("v[2] \t= %d\n", v[2]);
    puts("\n**** Bonus: i[v] == v[i] == *(v+i) == *(i+v) ****");
    printf("*(v+1) \t= %d\n", *(v+1));
    printf("*(1+v) \t= %d\n", *(1+v));
    printf("v[1] \t= %d\n", v[1]);
    printf("1[v] \t= %d\n", 1[v]);
    return 0;
}
```

:::tip Dica

Para rodar o programa acima, lembre-se de usar o `gcc`. 

```bash
gcc array_test.c -o a.out
./a.out
```

:::

O que exatamente esse teste prova?

1. Os elementos de um array são **contíguos**; e
2. Acessar elementos de um array com `array[índice]` nada mais é do que açúcar
   de sintaxe para [aritmética de
   ponteiros](https://www.tutorialspoint.com/cprogramming/c_pointer_arithmetic.htm)

Arrays são **rápidos** e **não precisam usar o heap**, mas isso tras limitações
que vamos discutir em outro momento. Por enquanto, vamos explorar duas
estruturas básicas que podem ser feitas usando apenas arrays: as pilhas e as
filas.

## 2. Pilhas

<img 
  src="https://miro.medium.com/v2/resize:fit:720/format:webp/1*tQ9Y11OdaMnhXwbfCF-edA.gif"
  alt="Pilhas" 
  style={{ 
    display: 'block',
    marginLeft: 'auto',
    maxHeight: '30vh',
    marginRight: 'auto'
  }} 
/>
<br/>

Não há muito o que ser dito a respeito de pilhas e filas além do fato de que
tratam-se de sequências de valores ordenados (assim como os arrays) que se
tornam únicos pelas regras estabelecidas na hora de **adicionar ou remover
elementos**. Uma **pilha** obriga a inserção de novos elementos em seu **topo**
e a remoção de elementos também a partir do seu **topo** (tal qual uma
**pilha** de pratos). O nome desse tipo de estrutura é **LIFO** (Last In First
Out). O código abaixo exemplifica uma implementação de pilha simples em C:

```C showLineNumbers title="simple_stack.c"
#include <stdio.h>

#define MAX_SIZE 100  // Definindo o tamanho máximo da pilha

// Estrutura para a pilha
typedef struct {
    int items[MAX_SIZE];
    int top;
    int bottom;
} Stack;

// Declarando uma pilha global
Stack s;

// Função para inicializar a pilha
void initStack() {
    s.top = -1;  // Inicializa o topo da pilha como -1 indicando que está vazia
    s.bottom = 0;  // A base da pilha sempre será 0
}

// Função para adicionar elementos na pilha
void push(int item) {
    if (s.top == MAX_SIZE - 1) {
        printf("Stack overflow\n");
        return;
    }
    s.items[++s.top] = item;  // Incrementa o topo e adiciona o item
    printf("Pushed %d to the stack\n", item);
}

// Função para remover elementos da pilha
int pop() {
    if (s.top == -1) {
        printf("Stack underflow\n");
        return -1;  // Retorna -1 para indicar que a pilha está vazia
    }
    printf("Popped %d from the stack\n", s.items[s.top]);
    return s.items[s.top--];  // Retorna o item e decrementa o topo
}

// Função para exibir o conteúdo atual da pilha
void displayStack() {
    if (s.top == -1) {
        printf("Stack is empty\n");
        return;
    }
    printf("Stack contents: ");
    for (int i = s.top; i >= s.bottom; --i) {
        printf("%d ", s.items[i]);
    }
    printf("\n");
}

// Programa principal para testar as funções da pilha
int main() {
    initStack();  // Inicializa a pilha
    displayStack();  // Mostra a pilha (vazia inicialmente)
    push(10);  // Adiciona elementos à pilha
    displayStack();  // Mostra a pilha
    push(20);
    displayStack();  // Mostra a pilha
    push(30);
    displayStack();  // Mostra a pilha
    pop();  // Remove elementos da pilha
    displayStack();  // Mostra a pilha
    pop();
    displayStack();  // Mostra a pilha
    return 0;
}
```

### 2.1. Onde pilhas são utilizadas?

Exemplo fácil - a **pilha/stack** do seu programa. Vai aí uma leitura
adicional:

:::tip Leitura Adicional

[Como funciona o stack?](https://www.geeksforgeeks.org/introduction-to-stack-memory/)

:::

## 3. Filas

<img 
  src="https://miro.medium.com/v2/resize:fit:640/format:webp/1*DSEFgwCum5ewVAfBMtXYOg.gif"
  alt="Filas" 
  style={{ 
    display: 'block',
    marginLeft: 'auto',
    maxHeight: '30vh',
    marginRight: 'auto'
  }} 
/>
<br/>

Se as **pilhas** permitem apenas adição e remoção de elementos a partir do seu
**topo**, as **filas** trabalham utilizando o conceito **FIFO** (First In First
Out). Isso significa que novos elementos são adicionados ao **final da fila** e
os elementos sempre são removidos do **começo da fila**. Segue uma
implementação simples em C:

```c showLineNumbers title="simple_queue.c"
#include <stdio.h>

#define MAX_SIZE 100  // Definindo o tamanho máximo da fila

// Estrutura para a fila
typedef struct {
    int items[MAX_SIZE];
    int head;
    int tail;
} Queue;

// Declarando uma fila global
Queue q;

// Função para inicializar a fila
void initQueue() {
    q.head = 0;
    q.tail = -1;
}

// Função para adicionar elementos na fila
void enqueue(int item) {
    if (q.tail == MAX_SIZE - 1) {
        printf("Queue overflow\n");
        return;
    }
    q.items[++q.tail] = item;  // Incrementa a cauda e adiciona o item
    printf("Enqueued %d to the queue\n", item);
}

// Função para remover elementos da fila
int dequeue() {
    if (q.head > q.tail) {
        printf("Queue underflow\n");
        return -1;  // Retorna -1 para indicar que a fila está vazia
    }
    printf("Dequeued %d from the queue\n", q.items[q.head]);
    return q.items[q.head++];  // Retorna o item e incrementa a cabeça
}

// Função para exibir o conteúdo atual da fila
void displayQueue() {
    if (q.head > q.tail) {
        printf("Queue is empty\n");
        return;
    }
    printf("Queue contents: ");
    for (int i = q.head; i <= q.tail; i++) {
        printf("%d ", q.items[i]);
    }
    printf("\n");
}

// Programa principal para testar as funções da fila
int main() {
    initQueue();  // Inicializa a fila
    displayQueue();  // Mostra a fila (vazia inicialmente)
    enqueue(10);  // Adiciona elementos à fila
    displayQueue();  // Mostra a fila
    enqueue(20);
    displayQueue();  // Mostra a fila
    enqueue(30);
    displayQueue();  // Mostra a fila
    dequeue();  // Remove elementos da fila
    displayQueue();  // Mostra a fila
    dequeue();
    displayQueue();  // Mostra a fila
    return 0;
}
```

### 3.1. Onde as filas são utilizadas?

O papel das filas é absolutamente fundamental para processamento de dados em
alta escala. Veja o vídeo abaixo sobre o assunto:

:::tip Vídeo complementar

<div style={{ textAlign: 'center' }}>
    <iframe 
        style={{
            display: 'block',
            margin: 'auto',
            width: '100%',
            height: '50vh',
        }}
        src="https://www.youtube.com/embed/W4_aGb_MOls"
        frameborder="0" 
        allowFullScreen>
    </iframe>
</div>
<br />

:::

**psst...Ei... só um adendo**: o ROS utiliza fila de mensagens também, tá?

## 4. Filas e Pilhas em Python

Só para ficar registrado, é possível implementar filas e pilhas em Python
utilizando a estrutura **deque** (Double Ended Queue - se pronuncia `deck`).

Vamos ver como fica?

```python showLineNumbers title="deque_stack.py"
from collections import deque

# Criando e utilizando a pilha
stack = deque()
stack.append(10)  # Push
stack.append(20)  # Push
stack.append(30)  # Push

print("Stack contents:", list(stack))

stack.pop()  # Pop
print("Stack after pop:", list(stack))
```

E para a fila:

```python showLineNumbers title="deque_queue.py"
from collections import deque

# Criando e utilizando a fila
queue = deque()
queue.append(10)  # Enqueue
queue.append(20)  # Enqueue
queue.append(30)  # Enqueue

print("Queue contents:", list(queue))

queue.popleft()  # Dequeue
print("Queue after dequeue:", list(queue))
```

Mais fácil, né? É para isso que servem as linguagens de alto nível.
