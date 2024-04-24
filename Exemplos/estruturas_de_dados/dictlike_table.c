#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TABLE_SIZE 10

typedef enum {
    INT, FLOAT, CHAR
} ValueType;

typedef struct {
    ValueType type;
    union {
        int i;
        float f;
        char c;
    } data;
} Value;

typedef struct node {
    char* key;
    Value value;
    struct node* next;
} Node;

Node* hashTable[TABLE_SIZE];

// Função de hash
unsigned int hash(const char *key) {
    unsigned long int value = 0;
    unsigned int i = 0;
    unsigned int key_len = strlen(key);
    for (; i < key_len; ++i) {
        value = value * 37 + key[i];
    }
    value = value % TABLE_SIZE;
    return value;
}

// Inserir ou atualizar um par chave-valor no hash map
void insert(char* key, Value value) {
    unsigned int index = hash(key);
    Node* newNode = (Node*)malloc(sizeof(Node));
    newNode->key = strdup(key);
    newNode->value = value;
    newNode->next = NULL;

    if (hashTable[index] == NULL) {
        hashTable[index] = newNode;
    } else {
        Node* current = hashTable[index];
        while (current->next != NULL && strcmp(current->key, key) != 0) {
            current = current->next;
        }
        if (strcmp(current->key, key) == 0) {
            current->value = value;
            free(newNode->key);
            free(newNode);
        } else {
            current->next = newNode;
        }
    }
}

// Buscar um valor pela chave
Value search(char* key) {
    unsigned int index = hash(key);
    Node* current = hashTable[index];
    while (current != NULL) {
        if (strcmp(current->key, key) == 0) {
            return current->value;
        }
        current = current->next;
    }
    // Retornar um tipo especial indicando não encontrado
    Value notFound;
    notFound.type = INT; // Default type
    notFound.data.i = -1; // Indicando não encontrado
    return notFound;
}

// Função para imprimir o hash map
void printValue(Value value) {
    switch(value.type) {
        case INT: printf("%d", value.data.i); break;
        case FLOAT: printf("%f", value.data.f); break;
        case CHAR: printf("%c", value.data.c); break;
    }
}

void printHashMap() {
    for (int i = 0; i < TABLE_SIZE; ++i) {
        Node* current = hashTable[i];
        if (current == NULL) {
            printf("Index %d: (null)\n", i);
        } else {
            printf("Index %d:", i);
            while (current != NULL) {
                printf(" (%s: ", current->key);
                printValue(current->value);
                printf(")");
                current = current->next;
            }
            printf("\n");
        }
    }
}

int main() {
    Value intValue = {.type = INT, .data.i = 100};
    Value floatValue = {.type = FLOAT, .data.f = 123.456};
    Value charValue = {.type = CHAR, .data.c = 'A'};

    insert("key1", intValue);
    insert("key2", floatValue);
    insert("key3", charValue);

    printf("Value for 'key1': ");
    printValue(search("key1"));
    printf("\nValue for 'key2': ");
    printValue(search("key2"));
    printf("\nValue for 'key3': ");
    printValue(search("key3"));
    printf("\n");

    printHashMap(); // Mostrar o estado do hash map

    return 0;
}
