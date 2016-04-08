#include <stdio.h>
#include <stdlib.h>
#include <bmpfile.h>

int main(int argc, char **argv)
{
    bmpfile_t *bmp;
    int i, j;
    char* infilename;
    FILE* infile;
    char* outfile;
    int width;
    int height;
    int depth;
    unsigned char red, green, blue; // 8-bits each
    //unsigned char pixel[3]; // 24-bits per pixel

    if (argc < 6) {
        printf("Usage: %s infile width height depth outfile.\n", argv[0]);
        exit(EXIT_FAILURE);
    }

    infilename = argv[1];
    outfile = argv[5];

    infile = fopen(infilename, "rb");
    if (NULL == infile) {
        perror("Couldn't read infile");
        exit(EXIT_FAILURE);
    }

    width = atoi(argv[2]);
    height = atoi(argv[3]);
    depth = atoi(argv[4]);

    // should be depth/8 at 16-bit depth, but 32-bit depth works better
    char buffer[height * width * 3];
    printf("depth: %d\n", depth);
    if (fread(&buffer, 1, height * width * 3, infile) != height * width * 3) {
        fputs("infile dimensions don't match the size you supplied\n", stderr);
    }

    if ((bmp = bmp_create(width, height, depth)) == NULL) {
        printf("Invalid depth value: '%d'. Try 1, 4, 8, 16, 24, or 32.\n", depth);
        exit(EXIT_FAILURE);
    }

    for (i = 0; i < width; ++i) {
        for (j = 0; j < height; ++j) {

            red     = buffer[(width * j + i) * 3 + 0];
            green   = buffer[(width * j + i) * 3 + 1];
            blue    = buffer[(width * j + i) * 3 + 2];

            rgb_pixel_t bpixel = {blue, green, red, 0};
            bmp_set_pixel(bmp, i, j, bpixel);
        }
    }

    bmp_save(bmp, outfile);
    bmp_destroy(bmp);

    return 0;
}
