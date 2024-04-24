---
title: Listas
sidebar_position: 4
sidebar_class_name: autoestudo
slug: /listas
---

import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

# Estruturas de dados array-like (ou listas)

## 1. O problema dos arrays

O problema dos arrays já pode ter ficado claro para os mais observadores: o
array **não cresce**. Seja uma implementação de array na stack ou no heap, não
é possível modificar dinâmicamente o tamanho de um array. O que isso significa?
Que você pode até aproveitar a performance dos arrays... **se** souber
exatamente o tamanho que precisa ter sua estrutura. Essa condição de contorno
pode ser razoável em diversoso momentos, mas na maioria dos casos é bem difícil
saber o tamanho que precisamos para cada estrutura de nosso código. É por isso
que uma solução clássica para esse problema é a **lista ligada**.

## 2. Listas ligadas

<img 
  src="https://miro.medium.com/v2/resize:fit:720/format:webp/1*v9uMEKfoRPHe1KUlLRH0RA.gif"
  alt="Listas ligadas" 
  style={{ 
    display: 'block',
    marginLeft: 'auto',
    maxHeight: '60vh',
    marginRight: 'auto'
  }} 
/>
<br/>

Diferente das estruturas que chamei de *elementares* na seção anterior, a lista
ligada já te **obriga** a utilizar ponteiros e a alocar memória de forma
dinâmica. Não se sente confortável com esses dois conceitos? Sugiro parar e dar
uma estudada antes de voltar para essa seção.

Vamos ver uma implementação simples de uma lista ligada em C?

```c showLineNumbers title="linked_list.c"
#include <stdio.h>
#include <stdlib.h>

// Definindo a estrutura para um nó na lista ligada
typedef struct Node {
    int data;
    struct Node* next;
} Node;

// Função para criar um novo nó
Node* createNode(int data) {
    Node* newNode = (Node*) malloc(sizeof(Node));
    if (newNode == NULL) {
        printf("Error creating a new node.\n");
        exit(0);
    }
    newNode->data = data;
    newNode->next = NULL;
    return newNode;
}

// Função para adicionar um elemento ao final da lista
void append(Node** head, int data) {
    Node* newNode = createNode(data);
    if (*head == NULL) { // Se a lista está vazia
        *head = newNode;
    } else {
        Node* temp = *head;
        while (temp->next != NULL) {
            temp = temp->next;
        }
        temp->next = newNode;
    }
}

// Função para remover um elemento da lista
void deleteNode(Node** head, int key) {
    Node* temp = *head, *prev = NULL;
    // Se o nó a ser removido é o cabeçalho da lista
    if (temp != NULL && temp->data == key) {
        *head = temp->next;
        free(temp);
        return;
    }
    // Procurar o nó a ser removido, mantendo o registro do nó anterior
    while (temp != NULL && temp->data != key) {
        prev = temp;
        temp = temp->next;
    }
    // Se a chave não foi encontrada na lista
    if (temp == NULL) return;
    // Desvincular o nó da lista
    prev->next = temp->next;
    free(temp);
}

// Função para imprimir todos os elementos da lista
void display(Node* head) {
    Node* temp = head;
    while (temp != NULL) {
        printf("%d -> ", temp->data);
        temp = temp->next;
    }
    printf("NULL\n");
}

// Função principal para testar as operações da lista ligada
int main() {
    Node* head = NULL; // Inicializando a lista como vazia

    append(&head, 10);
    append(&head, 20);
    append(&head, 30);
    display(head);

    deleteNode(&head, 20);
    display(head);

    return 0;
}
```

Note que, para cada elemento (nó), deve-se alocar memória na heap. Esse
processo é **muito lento**. Ou seja, a lista ligada resolve o problema de
espaço, mas cria um novo: o problema da **lentidão**. Eu não quero só contar
para vocês que a lista ligada é lenta, vamos ver na prática! Eis aqui duas
implementações de um programa ~inútil~ simples. A ideia é criar uma estrutura
de lista de **100 mil** elementos e ir de elemento em elemento,
incrementando-o. Segue o código em C abaixo para essa implementação usando
listas ligadas e arrays:

<Tabs>
<TabItem value="llist" label="Lista ligada" default>

```c showLineNumbers title="increment_llist.c"
#include <stdio.h>
#include <stdlib.h>

// Definindo a estrutura para um nó na lista ligada
typedef struct Node {
    int data;
    struct Node* next;
} Node;

// Função para criar um novo nó
Node* createNode(int data) {
    Node* newNode = (Node*) malloc(sizeof(Node));
    if (newNode == NULL) {
        printf("Memory allocation failed.\n");
        exit(0);
    }
    newNode->data = data;
    newNode->next = NULL;
    return newNode;
}

// Função para adicionar um elemento no final da lista
void append(Node** head, int data) {
    Node* newNode = createNode(data);
    if (*head == NULL) {
        *head = newNode;
    } else {
        Node* temp = *head;
        while (temp->next != NULL) {
            temp = temp->next;
        }
        temp->next = newNode;
    }
}

// Função para incrementar todos os elementos da lista em uma unidade
void incrementAll(Node* head) {
    Node* current = head;
    while (current != NULL) {
        current->data += 1;
        current = current->next;
    }
}

// Função para liberar a memória da lista
void freeList(Node* head) {
    Node* current = head;
    Node* next;
    while (current != NULL) {
        next = current->next;
        free(current);
        current = next;
    }
}

// Função principal para testar as operações da lista ligada
int main() {
    Node* head = NULL;
    const int size = 100000; // Por exemplo, 100.000 para demonstração

    // Preenchendo a lista
    for (int i = 0; i < size; i++) {
        append(&head, i);
    }

    // Incrementando todos os elementos
    incrementAll(head);

    // Liberando a lista
    freeList(head);

    return 0;
}
```

</TabItem>

<TabItem value="array" label="Array">

```c showLineNumbers title="increment_array.c"
#include <stdio.h>
#include <stdlib.h>

// Função para incrementar todos os elementos do array
void incrementArray(int *array, int size) {
    for (int i = 0; i < size; i++) {
        array[i] += 1;
    }
}

int main() {
    int size = 100000000; // Tamanho do array, 100 milhões de elementos
    int *array = (int *)malloc(size * sizeof(int));

    if (array == NULL) {
        printf("Memory allocation failed.\n");
        return 1; // Retorna erro se a alocação falhar
    }

    // Inicializando o array
    for (int i = 0; i < size; i++) {
        array[i] = i;
    }

    // Incrementando todos os elementos do array
    incrementArray(array, size);

    // Liberando a memória alocada para o array
    free(array);

    return 0;
}
```
</TabItem>
</Tabs>

Agora, vamos compilar e rodar o programa usando o comando `time`, do Linux:

**Resultado lista ligada**

```bash
./a.out  14.54s user 0.00s system 99% cpu 14.567 total
```

**Resultado array**

```bash
./a.out  0.39s user 0.14s system 99% cpu 0.527 total
```

Ou seja, o mesmo programa sendo feito com um array foi **quase 40 vezes** mais
rápido que a versão com a lista ligada.

Quer saber de um segredo? Eu menti para vocês. A versão do código com arrays
está trabalhando com **100 milhões** de elementos, contra **100 mil** da lista
ligada. Se configurarmos a versão de array para trabalhar apenas com 100 mil
elementos, o resultado passa a ser esse:

```bash
./a.out  0.00s user 0.00s system 83% cpu 0.002 total
```

É... eu não consigo nem medir o quão mais rápida é a versão usando arrays nesse
caso. A diferença de performance entre o array e a lista ligada é
**assustadora**. Esse é o poder da **localidade espacial** ou, colocando de
outra forma, essa é a [lentidão do heap](https://youtu.be/ioJkA7Mw2-U).

<img 
  src="./gifs/array_linked.gif"
  alt="lista ligada vs array" 
  style={{ 
    display: 'block',
    marginLeft: 'auto',
    maxHeight: '30vh',
    marginRight: 'auto'
  }} 
/>
<br/>

Qual o efeito disso? É meio difícil ver algum programa que precisa de
performance usar a lista ligada. A questão é que o problema original continua:
o que fazer com um array que precisa crescer? A resposta pode ser mais simples
do que parece: destrói ele e cria de novo, com mais memória. Essa é a resposta
do **ArrayList**.

## 3. ArrayList

Para explicar o motivo de existir do ArrayList e como ele resolve o problema do
array sem ser lento como a lista ligada, vou usar o **excelente** vídeo feito
pelo canal **Core Dumped** sobre o assunto:

:::warning Autoestudo

<div style={{ textAlign: 'center' }}>
    <iframe 
        style={{
            display: 'block',
            margin: 'auto',
            width: '100%',
            height: '50vh',
        }}
        src="https://www.youtube.com/embed/xFMXIgvlgcY" 
        frameborder="0" 
        allowFullScreen>
    </iframe>
</div>

:::

O que nos resta aqui são duas coisas:

1. Eu mostrar uma implementação de array list em C; e
2. Eu rodar o benchmark que fiz ali em cima, agora usando array lists.

Vou fazer as duas coisas de uma vez só, usando o código do benchmark como
exemplo de array list em C.

```c showLineNumbers title="array_list.c"
#include <stdio.h>
#include <stdlib.h>

typedef struct {
    int *array;
    int size;
    int capacity;
} ArrayList;

ArrayList createArrayList(int initialCapacity) {
    ArrayList list;
    list.array = (int *)malloc(initialCapacity * sizeof(int));
    if (list.array == NULL) {
        fprintf(stderr, "Failed to allocate memory.\n");
        exit(1);
    }
    list.size = 0;
    list.capacity = initialCapacity;
    return list;
}

void addElement(ArrayList *list, int element) {
    if (list->size == list->capacity) {
        list->capacity *= 2;  // Dobrar a capacidade quando necessário
        list->array = (int *)realloc(list->array, list->capacity * sizeof(int));
        if (list->array == NULL) {
            fprintf(stderr, "Failed to reallocate memory.\n");
            exit(1);
        }
    }
    list->array[list->size++] = element;
}

void incrementAll(ArrayList *list) {
    for (int i = 0; i < list->size; i++) {
        list->array[i]++;
    }
}

void displayArrayList(const ArrayList *list) {
    printf("ArrayList contents: ");
    for (int i = 0; i < list->size; i++) {
        printf("%d ", list->array[i]);
    }
    printf("\n");
}

int main() {
    int initialSize = 100000000; // 100 milhões de elementos
    ArrayList list = createArrayList(initialSize);

    // Preenchendo o ArrayList
    for (int i = 0; i < initialSize; i++) {
        addElement(&list, i);
    }

    // Incrementando todos os elementos
    incrementAll(&list);

    // Liberando a memória alocada para o ArrayList
    free(list.array);

    return 0;
}
```

E, ao rodar esse código com o `time`, temos:

```bash
./a.out  0.57s user 0.11s system 99% cpu 0.681 total
```

O que é um resultado **muito próximo** da implementação com array puro. O
motivo disso é que, para todos os efeitos, ele **é** um array puro. A sacada do
array list é concentrar a perda de performance acarretada por uma **alocação
dinâmica** somente no momento da criação da estrutura e quando precisa mudar de
tamanho.

Essa estrutura é tão boa que a maioria das linguagens usa ela como padrão de
estrutura "array-like", mas ela tem limitações. A principal delas é:

**O Array list sacrifica a capacidade de crescer ocupando espaços fragmentados
da memória e de ter nós que podem ter tamanhos gigantescos para encontrar um
meio termo entre performance e capacidade de crescimento**. Ou seja, ainda
existe lugar no mundo para listas ligadas (pense em uma lista de músicas, por
exemplo - oi spotify).

<img 
  src="https://static.javatpoint.com/core/images/arraylist-implementation-in-java.png"
  alt="Array lists crescendo" 
  style={{ 
    display: 'block',
    marginLeft: 'auto',
    maxHeight: '60vh',
    marginRight: 'auto'
  }} 
/>
<br/>

## 4. A lista do Python

Para finalizar nosso passeio pelo mundo dos array-likes, vamos conversar um
pouco sobre a **lista** de Python. Ela claramente não é performática como um
array list (ou não haveria necessidade do **numpy** forçar o uso dos seus
**arrays** para melhorar a performance), mas a performance dela também não é
absolutamente patética (como é o caso da lista ligada). O motivo disso é que o
Python implementa um array de tamanho dinâmico similar ao ArrayList, mas ele
precisa fazer uma modificação:

**Cada elemento da lista de Python precisa guardar um ponteiro que aponta para
onde fica o dado daquele "nó"**. Por quê? Simples. Olha só um exemplo clássico
de lista em Python:

```python
l = ["oi", 42, True, 3.14, ["oi", "tchau"]]
```

Notou algo? Sim, é uma **bagunça** de tipos de dados. Em Python isso é bem
resolvido pois a implementação de **tipo de variável como objeto** é uma das
bases que sustenta a linguagem, mas o fato é que a implementação disso em C
fatalmente incorre no uso de ponteiros. Não qualquer ponteiro, mas ponteiros
para **void**. Vamos dar uma olhada em uma implementação de array dinâmico
similar à de Python, mas extremamente simplificada?

```c showLineNumbers title="pythonlike_list.c"
#include <stdio.h>
#include <stdlib.h>

// Definições de tipos para os tipos de dados armazenados
typedef enum {
    INT, FLOAT, CHAR
} DataType;

// Estrutura para armazenar dados e tipo de dados
typedef struct {
    void *data;
    DataType type;
} Element;

// Estrutura de ArrayList
typedef struct {
    Element *array;
    int size;
    int capacity;
} ArrayList;

ArrayList createArrayList(int initialCapacity) {
    ArrayList list;
    list.array = (Element *)malloc(initialCapacity * sizeof(Element));
    if (list.array == NULL) {
        fprintf(stderr, "Failed to allocate memory.\n");
        exit(1);
    }
    list.size = 0;
    list.capacity = initialCapacity;
    return list;
}

void addElement(ArrayList *list, void *data, DataType type) {
    if (list->size == list->capacity) {
        list->capacity *= 2;
        list->array = (Element *)realloc(list->array, list->capacity * sizeof(Element));
        if (list->array == NULL) {
            fprintf(stderr, "Failed to reallocate memory.\n");
            exit(1);
        }
    }
    list->array[list->size].data = data;
    list->array[list->size].type = type;
    list->size++;
}

void displayArrayList(const ArrayList *list) {
    printf("ArrayList contents:\n");
    for (int i = 0; i < list->size; i++) {
        switch (list->array[i].type) {
            case INT:
                printf("%d (int)\n", *(int *)list->array[i].data);
                break;
            case FLOAT:
                printf("%f (float)\n", *(float *)list->array[i].data);
                break;
            case CHAR:
                printf("%c (char)\n", *(char *)list->array[i].data);
                break;
        }
    }
}

int main() {
    ArrayList list = createArrayList(10);

    int a = 10;
    float b = 3.14f;
    char c = 'x';

    addElement(&list, &a, INT);
    addElement(&list, &b, FLOAT);
    addElement(&list, &c, CHAR);

    displayArrayList(&list);

    // Liberando a memória alocada para o ArrayList
    free(list.array);

    return 0;
}
```

Rode o código acima, brinque com ele e tente entender intuitivamente o que é um
ponteiro que aponta para **void**. Essa é sua lição de casa =D
