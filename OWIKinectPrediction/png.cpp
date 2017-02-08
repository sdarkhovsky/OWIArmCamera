#ifndef _WINDOWS
    #include <unistd.h>
#endif
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#define DEBUGGING

#ifdef DEBUGGING
#define PNG_DEBUG 3
#include <png.h>

#include "point_cloud.h"

bool write_png_file(const char* file_name, ais::c_point_cloud& point_cloud)
{
    int x, y, i;

    png_byte color_type = PNG_COLOR_TYPE_RGBA;
    png_byte bit_depth = 8;

    int width = point_cloud.points[0].size();
    int height = point_cloud.points.size();

    png_structp png_ptr;
    png_infop info_ptr;

    /* create file */
    FILE *fp = fopen(file_name, "wb");
    if (!fp)
        return false;

    /* initialize stuff */
    png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);

    if (!png_ptr)
        return false;

    info_ptr = png_create_info_struct(png_ptr);
    if (!info_ptr)
        return false;

    if (setjmp(png_jmpbuf(png_ptr)))
        return false;

    png_init_io(png_ptr, fp);


    /* write header */
    if (setjmp(png_jmpbuf(png_ptr)))
        return false;

    png_set_IHDR(png_ptr, info_ptr, width, height,
        bit_depth, color_type, PNG_INTERLACE_NONE,
        PNG_COMPRESSION_TYPE_BASE, PNG_FILTER_TYPE_BASE);

    png_write_info(png_ptr, info_ptr);


    /* write bytes */
    if (setjmp(png_jmpbuf(png_ptr)))
        return false;

    png_bytep * row_pointers;
    row_pointers = (png_bytep*)malloc(sizeof(png_bytep) * height);
    for (y = 0; y<height; y++)
        row_pointers[y] = (png_byte*)malloc(png_get_rowbytes(png_ptr, info_ptr));

    for (y = 0; y<height; y++) {
        png_byte* row = row_pointers[y];
        for (x = 0; x<width; x++) {
            png_byte* ptr = &(row[x * 4]);

            for (i = 0; i < 3; i++) 
                ptr[i] = point_cloud.points[y][x].Clr(i);
            ptr[3] = 255;
        }
    }

    png_write_image(png_ptr, row_pointers);


    /* end write */
    if (setjmp(png_jmpbuf(png_ptr)))
        return false;

    png_write_end(png_ptr, NULL);

    /* cleanup heap allocation */
    for (y = 0; y<height; y++)
        free(row_pointers[y]);
    free(row_pointers);

    fclose(fp);

    return true;
}
#endif


