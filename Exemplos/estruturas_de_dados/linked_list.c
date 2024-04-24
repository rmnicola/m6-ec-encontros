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
