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
