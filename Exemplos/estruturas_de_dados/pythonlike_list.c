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
