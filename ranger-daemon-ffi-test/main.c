#include <stdlib.h>
#include <stdio.h>
#include <HsFFI.h>

#include <ranger_types.h>
#include <Ranger/CApi_stub.h>

int main(int argc, char *argv[]) {
    int i;

    hs_init(&argc, &argv);

    for (i = 0; i < 5; i++) {
        printf("%d\n", foo(100*i));
    }

    hs_exit();

    return 0;
};
