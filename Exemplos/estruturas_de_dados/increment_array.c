#include <stdio.h>
#include <stdlib.h>

// Função para incrementar todos os elementos do array
void incrementArray(int *array, int size) {
    for (int i = 0; i < size; i++) {
        array[i] += 1;
    }
}

int main() {
    int size = 100000; // Tamanho do array, 100 milhões de elementos
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
