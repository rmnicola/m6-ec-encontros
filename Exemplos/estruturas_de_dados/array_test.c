#include <stdio.h>

int main() {
    int v[] = {2, 4, 8, 12, 16, 18};
    puts("**** Prova 1: v == &v[0] ****");
    printf("v \t= %p\n", v);
    printf("&v[0] \t= %p\n", &v[0]);
    puts("\n**** Prova 2: *(v+i) == v[i] ****");
    printf("*v \t= %d\n", *v);
    printf("v[0] \t= %d\n", v[0]);
    printf("*(v+2) \t= %d\n", *(v+2));
    printf("v[2] \t= %d\n", v[2]);
    puts("\n**** Bonus: i[v] == v[i] == *(v+i) == *(i+v) ****");
    printf("*(v+1) \t= %d\n", *(v+1));
    printf("*(1+v) \t= %d\n", *(1+v));
    printf("v[1] \t= %d\n", v[1]);
    printf("1[v] \t= %d\n", 1[v]);
    return 0;
}
