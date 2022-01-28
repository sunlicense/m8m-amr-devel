#ifndef PGM_H
#define PGM_H
	  
/*max size of an image*/
#define MAX 1600
  
/*  
#define LOW_VALUE 0 
#define HIGH_VALUE 255
*/
  
/*RGB color struct with integral types*/
typedef struct {
	unsigned char red;
	unsigned char green;
	unsigned char blue;
}RGB_INT;
  
typedef struct  
{
	int maxVal;
	int width;
	int height;
	RGB_INT data[MAX][MAX];
} PGMImage;
	  
//typedef struct PGMstructure PGMImage;
    
/***prototypes**********************************************************/
/***********************************************************************/
  
void getPGMfile (char filename[], PGMImage *img);
void save(PGMImage *img);
  
#endif

