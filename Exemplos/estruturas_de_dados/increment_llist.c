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
