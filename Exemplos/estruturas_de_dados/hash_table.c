#include <stdio.h>
#include <string.h>

#define TABLE_SIZE 100

typedef struct {
    char* key;
    int value;
} Entry;

Entry hashTable[TABLE_SIZE];

// Função de hash simples
unsigned int hash(const char *key) {
    unsigned long int value = 0;
    unsigned int i = 0;
    unsigned int key_len = strlen(key);

    // Convertendo a chave para um inteiro
    for (; i < key_len; ++i) {
        value = value * 37 + key[i];
    }

    // Garantir que o valor esteja dentro dos limites da tabela
    value = value % TABLE_SIZE;

    return value;
}

// Inserir chave-valor na hash table
void insert(char* key, int value) {
    unsigned int index = hash(key);

    hashTable[index].key = key;
    hashTable[index].value = value;
}

// Buscar um valor pela chave na hash table
int search(char* key) {
    unsigned int index = hash(key);
    
    if (strcmp(hashTable[index].key, key) == 0) {
        return hashTable[index].value;
    } else {
        return -1; // Assume-se que -1 indica "não encontrado"
    }
}

int main() {
    char* keys[] = {"key1", "key2", "key3", "key4"};
    int values[] = {10, 20, 30, 40};

    for (int i = 0; i < 4; i++) {
        insert(keys[i], values[i]);
    }

    // Buscando valores
    printf("O valor de 'key2' é %d\n", search("key2"));
    printf("O valor de 'key4' é %d\n", search("key4"));

    return 0;
}
