//#pragma pack(push,2)

#include <ros/ros.h>
#include <stdio.h>
#include <string.h>  // for strlen, strcopy
#include <stdlib.h>  // for malloc

/* Windows 3.x bitmap file header */
/*
typedef struct {
    char         filetype[2];   // magic - always 'B' 'M' 
    unsigned int filesize;
    short        reserved1;
    short        reserved2;
    unsigned int dataoffset;    // offset in bytes to actual bitmap data 
} file_header;
*/
/* Windows 3.x bitmap full header, including file header */
/*
typedef struct {
    file_header  fileheader;
    unsigned int headersize;
    int          width;
    int          height;
    short        planes;
    short        bitsperpixel;  // we only support the value 24 here 
    unsigned int compression;   // we do not support compression 
    unsigned int bitmapsize;
    int          horizontalres;
    int          verticalres;
    unsigned int numcolors;
    unsigned int importantcolors;
} bitmap_header;
*/
typedef struct {
    uint16_t type;              // Magic identifier: 0x4d42
    uint32_t size;              // File size in bytes
    uint16_t reserved1;         // Not used
    uint16_t reserved2;         // Not used
    uint32_t offset;            // Offset to image data in bytes from beginning of file
    uint32_t dib_header_size;   // DIB Header size in bytes
    int32_t  width;          // Width of the image
    int32_t  height;         // Height of image
    uint16_t planes;        // Number of color planes
    uint16_t bitsperpixel;    // Bits per pixel
    uint32_t compression;       // Compression type
    uint32_t bitmapsize;  // Image size in bytes
    int32_t  x_resolution_ppm;  // Pixels per meter
    int32_t  y_resolution_ppm;  // Pixels per meter
    uint32_t numcolors;        // Number of colors
    uint32_t important_colors;  // Important colors
} bitmap_header;
//#pragma pack(pop)

int main(int argc, char **argv) {

    //variable dec:
    FILE *fp;
    bitmap_header* hp;
    int n;
    char *data;


    //Open input file:
    fp = fopen("/home/rac/catkin_ws/src/htbot/maps/test.bmp", "r");
    if(fp==NULL){
    	ROS_INFO("-------- BMP : Failed to Open Test.bmp ----------------");
			return -1;
    }


    //Read the input file headers:
    hp=(bitmap_header*)malloc(sizeof(bitmap_header));
    if(hp==NULL) {
    	ROS_INFO("-------- BMP : Failed to allocate memory to file headers ----------------");
			return -1;
		}
		rewind(fp);
    n=fread(hp, sizeof(bitmap_header), 1, fp);
    if(n<1){
    	ROS_INFO("-------- BMP : Failed to Failed to read File Header ----------------");
    }

		// Image Header Info
		printf("---%x----",&hp[0]);
		ROS_INFO("--- BMP : Width=%d. Height=%d. Plane=%d. BitPerPixel=%d. BitMapSize=%d. numclr=%d. type=%x. Size=%x -----",hp->width,hp->height,hp->planes,hp->bitsperpixel,hp->bitmapsize,hp->numcolors,hp->type,hp->size);

    //Read the data of the image:
    data = (char*)malloc(sizeof(char)*hp->bitmapsize);
    if(data==NULL){
    	ROS_INFO("-------- BMP : Failed to allocate memory to Image Data ----------------");
    }

    fseek(fp,sizeof(char)*hp->offset,SEEK_SET);
    n=fread(data,sizeof(char),hp->bitmapsize, fp);
    if(n<1){
    	ROS_INFO("-------- BMP : Failed to Read Image Data ----------------");
    }

    fclose(fp);
    free(hp);
    free(data);
    return 0;
}
