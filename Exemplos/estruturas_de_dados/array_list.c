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

