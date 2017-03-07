#ifndef _WINDOWS
    #include <unistd.h>
#endif
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#include "png_visualize.h"

#define PNG_DEBUG 3
#include <png.h>

#include "point_cloud.h"

namespace ais {

    bool png_visualize_prepare(FILE *fp, int width, int height, png_structp* png_ptrptr, png_infop* info_ptrptr) {

        png_byte color_type = PNG_COLOR_TYPE_RGBA;
        png_byte bit_depth = 8;

        png_structp png_ptr;
        png_infop info_ptr;

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

        *png_ptrptr = png_ptr;
        *info_ptrptr = info_ptr;
        return true;
    }

    bool png_visualize_complete(png_structp png_ptr, png_bytep * row_pointers) {

        png_write_image(png_ptr, row_pointers);

        /* end write */
        if (setjmp(png_jmpbuf(png_ptr)))
            return false;

        png_write_end(png_ptr, NULL);

        return true;
    }


    inline void set_image_pixel(c_point_cloud_point& cloud_point, png_byte* pixel_ptr) {
        Vector3f Clr;
        get_cloud_point_rendering_color(cloud_point, Clr);
        pixel_ptr[0] = Clr(0);
        pixel_ptr[1] = Clr(1);
        pixel_ptr[2] = Clr(2);
        pixel_ptr[3] = 255;
    }

    bool png_visualize_point_cloud_correspondence(const char* file_name, vector <c_point_correspondence>& map, c_point_cloud& src_point_cloud, c_point_cloud& tgt_point_cloud) {
        int x, y, i, j;
        png_structp png_ptr;
        png_infop info_ptr;
        bool result = true;
        int src_x, src_y, tgt_x, tgt_y;
        Vector3i line_clr(0, 0, 255);

        int src_width = src_point_cloud.points[0].size();
        int src_height = src_point_cloud.points.size();
        int tgt_width = tgt_point_cloud.points[0].size();
        int tgt_height = tgt_point_cloud.points.size();
        int gap_width = 10;

        if (src_height != tgt_height)
            return false;

        int width = src_width + gap_width + tgt_width;
        int height = src_height;

        /* create file */
        FILE *fp = fopen(file_name, "wb");
        if (!fp)
            return false;

        if (!png_visualize_prepare(fp, width, height, &png_ptr, &info_ptr))
            return false;

        png_bytep * row_pointers;
        row_pointers = (png_bytep*)malloc(sizeof(png_bytep) * height);
        if (!row_pointers)
            goto Cleanup;
        for (y = 0; y < height; y++) {
            row_pointers[y] = (png_byte*)malloc(png_get_rowbytes(png_ptr, info_ptr));
            if (!row_pointers[y])
                goto Cleanup;
        }

        // copy images with edge and corner marks
        int tgt_img_start = src_width + gap_width;
        for (y = 0; y < height; y++) {
            png_byte* row = row_pointers[y];
            for (x = 0; x < width; x++) {
                png_byte* ptr = &(row[x * 4]);

                if (x < src_width) {
                    set_image_pixel(src_point_cloud.points[y][x], ptr);
                }
                else {
                    if (x < tgt_img_start) {
                        for (i = 0; i < 3; i++)
                            ptr[i] = 0;
                        ptr[3] = 255;
                    }
                    else {
                        set_image_pixel(tgt_point_cloud.points[y][x - tgt_img_start], ptr);
                    }
                }
            }
        }

        for (auto it = map.begin(); it != map.end(); it++) {
            Vector2i start = it->src_uv;
            Vector2i end = it->tgt_uv + Vector2i(0, tgt_img_start);
            int x_len = end(1) - start(1);
            Vector2i dir = (end - start);

            // the line connecting start and end)
            for (j = 0; j < x_len; j++) {
                Vector2i pnt = start + j*dir/x_len;
                png_byte* row = row_pointers[pnt(0)];
                png_byte* ptr = &(row[pnt(1) * 4]);
                for (i = 0; i < 3; i++)
                    ptr[i] = line_clr(i);
                ptr[3] = 255;
            }
        }

        png_visualize_complete(png_ptr, row_pointers);

    Cleanup:
        /* cleanup heap allocation */
        if (row_pointers) {
            for (y = 0; y < height; y++) {
                if (row_pointers[y])
                    free(row_pointers[y]);
            }
            free(row_pointers);
        }

        fclose(fp);

        return result;
    }

    bool png_visualize_point_cloud(const char* file_name, c_point_cloud& point_cloud)
    {
        int x, y, i;
        png_structp png_ptr;
        png_infop info_ptr;
        bool result = true;

        int width = point_cloud.points[0].size();
        int height = point_cloud.points.size();

        /* create file */
        FILE *fp = fopen(file_name, "wb");
        if (!fp)
            return false;

        if (!png_visualize_prepare(fp, width, height, &png_ptr, &info_ptr))
            return false;

        png_bytep * row_pointers;
        row_pointers = (png_bytep*)malloc(sizeof(png_bytep) * height);
        if (!row_pointers) 
            goto Cleanup;
        for (y = 0; y < height; y++) {
            row_pointers[y] = (png_byte*)malloc(png_get_rowbytes(png_ptr, info_ptr));
            if (!row_pointers[y])
                goto Cleanup;
        }

        for (y = 0; y < height; y++) {
            png_byte* row = row_pointers[y];
            for (x = 0; x < width; x++) {
                png_byte* ptr = &(row[x * 4]);

                set_image_pixel(point_cloud.points[y][x], ptr);
            }
        }

        png_visualize_complete(png_ptr, row_pointers);

        Cleanup:
        /* cleanup heap allocation */
        if (row_pointers) {
            for (y = 0; y < height; y++) {
                if (row_pointers[y])
                    free(row_pointers[y]);
            }
            free(row_pointers);
        }

        fclose(fp);

        return result;
    }
}