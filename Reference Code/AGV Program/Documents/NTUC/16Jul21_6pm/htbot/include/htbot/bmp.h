#ifndef _BMP_H_  // prevent recursive inclusion
#define _BMP_H_

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#define BMP_HEADER_SIZE 54
#define DIB_HEADER_SIZE 40

#pragma pack(push)  // save the original data alignment
#pragma pack(1)     // Set data alignment to 1 byte boundary

// uint16_t is a 16-bit unsigned integer
// uint32_t is a 32-bit unsigned integer

// Correct values for the header
#define MAGIC_VALUE         0x4D42 
#define NUM_PLANE           1
#define COMPRESSION         0
#define NUM_COLORS          0
#define IMPORTANT_COLORS    0
#define BITS_PER_PIXEL      24
#define BITS_PER_BYTE       8

typedef struct {
    uint16_t type;              // Magic identifier: 0x4d42
    uint32_t size;              // File size in bytes
    uint16_t reserved1;         // Not used
    uint16_t reserved2;         // Not used
    uint32_t offset;            // Offset to image data in bytes from beginning of file
    uint32_t dib_header_size;   // DIB Header size in bytes
    int32_t  width_px;          // Width of the image
    int32_t  height_px;         // Height of image
    uint16_t num_planes;        // Number of color planes
    uint16_t bits_per_pixel;    // Bits per pixel
    uint32_t compression;       // Compression type
    uint32_t image_size_bytes;  // Image size in bytes
    int32_t  x_resolution_ppm;  // Pixels per meter
    int32_t  y_resolution_ppm;  // Pixels per meter
    uint32_t num_colors;        // Number of colors
    uint32_t important_colors;  // Important colors
} BMPHeader;

#pragma pack(pop)  // restore the previous pack setting

typedef struct {
    BMPHeader header;
    unsigned char* data;
} BMPImage;

BMPImage* read_bmp(FILE* fp, char** error);
bool      check_bmp_header(BMPHeader* bmp_header, FILE* fp);
bool      write_bmp(FILE* fp, BMPImage* image, char** error);
void      free_bmp(BMPImage* image);
BMPImage* crop_bmp(BMPImage* image, int x, int y, int w, int h, char** error);

long _get_file_size(FILE *fp);
int _get_image_size_bytes(BMPHeader *bmp_header);
int _get_image_row_size_bytes(BMPHeader *bmp_header);
int _get_bytes_per_pixel(BMPHeader  *bmp_header);
int _get_padding(BMPHeader *bmp_header);
int _get_position_x_row(int x, BMPHeader *bmp_header);
bool _check(bool condition, char **error, const char *error_message);
char *_string_duplicate(const char *string);
void loadNoGoZone(void);

#endif  /* bmp.h */
