#include "datamatrix.h"
#include <stdio.h>

int main(int argc, char **argv)
{
    if (argc != 2) return 2;
    FILE *input = fopen(argv[1], "rb");
    if (!input) return 2;
    uint8_t modules[100];
    size_t count = fread(modules, 1, sizeof(modules), input);
    int extra = fgetc(input);
    fclose(input);
    if (count != sizeof(modules) || extra != EOF) return 2;
    BarcodeResult result;
    int length = datamatrix_decode_grid10(modules, &result);
    if (length <= 0) return 1;
    for (int index = 0; index < length; index++) printf("%02X", (unsigned char)result.data[index]);
    printf("\n");
    return 0;
}
