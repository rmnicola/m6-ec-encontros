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
