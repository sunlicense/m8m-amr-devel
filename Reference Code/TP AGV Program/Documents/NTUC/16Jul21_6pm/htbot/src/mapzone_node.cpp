/*
 * This node is to service command request from websocket 
 */
/* 	History
*		Date Modified : 3.12.2014
*		Changes :
*/

//#define ODROID
#define SSMC
//#define RAC

#include <ros/ros.h>
#include <stdio.h>
#include <math.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/poll.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <netdb.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <arpa/inet.h> 
#include <fcntl.h>
#include <signal.h>
#include <string>
#include <sys/time.h>
#include <signal.h>
#include "htbot/move.h"
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include "boost/algorithm/string.hpp"
#include <tf/transform_datatypes.h>
#include <stdint.h>
#include <sensor_msgs/LaserScan.h>
#include "htbot/pgm.h"
#include "htbot/bmp.h"

#define MAX_RANGE 6.0
#define MIN_RANGE 0.05
#define	MAX_IDX	350 //356 //350 //240 //180 //360 //666 //500 //720
#define START_IDX 66 //66
#define RIGHT_IDX 133 
#define LEFT_IDX 178
#define END_IDX 290 //284
#define DEPTH 1.0
#define WINDOW_WIDTH 2.5
#define DEGPIDX 1.0 //0.5 //0.36 //0.25
#define RADPIDX 0.01745 //0.00873 //0.00628 //0.00436
#define CIDX 175 //120 //90 //180 // 333 //250 //360  // total index for 240 (180) degree scan is 666

#define DWIDTH 0.3  //
#define DBOX 2.0
#define DFRONT 0.02  //0.1
#define PI 3.14159
#define PI2 6.28319
#define DM 8.530766 //9.926 // dm = atan(DWIDTH / DBOX) * (180.0 / PI);  // degree
#define DMIDX 9 //28 // (int)(dm / DEGPIDX)
#define DG  45 // dg = atan(DWIDTH / DWIDTH) * (180.0 / PI);  // degree
#define DGIDX 45 //125 // (int)(dg / DEGPIDX)
#define LASEROFFSET 0.03

#define DEG2RAD(x) ((x)*ANGLE_RES*3.14159/180.0)
#define RAD2DEG(x) ((x)*180.0/3.14159)
#define DISTGT350MM  0.6
#define MAX_MAPDATA 4465000
#define NOGOZONE 55


using namespace std;

//struct sockaddr_in serv_addr; 
//int sock, csock, reuse = 1, blocking=1;
//char recBuf[40];
//char *sendBuff= new char[65536];
//int FMPortNumber,FMRobotNum;
//string FMAddress;
ros::Subscriber pose_sub;
ros::Subscriber map_sub;
ros::Publisher robot_pub;
ros::Publisher scan_pub;
ros::Publisher pose_pub;
//sensor_msgs::LaserScan scan_msg;
geometry_msgs::Pose pose_msg;
double posex,posey,poserz,poserw;
boost::mutex publish_mutex_;
char* fmaddress;
struct timeval trcv, tsnd;
int map_width,map_height,map_size;
//char map_data[MAX_MAPDATA];  //
char *map_data;  //
double laser_data[MAX_IDX+10];
double yawp;
double Resolution,Window;
uint32_t lseq;
bool poseflag, mapflag,mapcallbackflag;
double MapObsRate,MapOffsetX,MapOffsetY,ressq;
int clearObs,lookforObs,cidx,lidx,ridx,lmidx,rmidx;
double refr,lrange,dm,dg,dt,dtt,od, tod,rod,lod,lrd;
double refrr,refll,rtod,ltod;
std::string NoGoZoneFile;


// prototype
void poseCallback(const geometry_msgs::Pose::ConstPtr& msg);
void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
void publish_robot_pos();
void generateObsMap();

/* 
 * Return the number of bytes per pixel.
 *  
 * Precondition:
 *     - the header must have the number of bits per pixel.
 */
int _get_bytes_per_pixel(BMPHeader  *bmp_header)
{
    return bmp_header->bits_per_pixel / BITS_PER_BYTE;
}

/*
 * Return the position of the pixel x from the beginning of a row.
 */ 
int _get_position_x_row(int x, BMPHeader *bmp_header)
{
    return x * _get_bytes_per_pixel(bmp_header);
}

/* 
 * Return the size of an image row in bytes.
 *  
 * - Precondition: the header must have the width of the image in pixels.
 */
int _get_image_row_size_bytes(BMPHeader *bmp_header)
{
    int bytes_per_row_without_padding = bmp_header->width_px * _get_bytes_per_pixel(bmp_header);
    return bytes_per_row_without_padding + _get_padding(bmp_header);
}

/*
 * Return size of padding in bytes.
 */ 
int _get_padding(BMPHeader *bmp_header)
{
    return (4 - (bmp_header->width_px * _get_bytes_per_pixel(bmp_header)) % 4) % 4;
}


/* 
 * Return the size of the image in bytes.
 */
int _get_image_size_bytes(BMPHeader *bmp_header)
{
    return _get_image_row_size_bytes(bmp_header) * bmp_header->height_px;
}

/* 
 * Return the size of the file.
 */
long _get_file_size(FILE *fp)
{   
    // Get current file position
    long current_position = ftell(fp);
    if (current_position == -1)
    {
        return -1;
    }
    // Set file position to the end
    if (fseek(fp, 0, SEEK_END) != 0)
    {
        return -2;
    }
    // Get current file position (now at the end)
    long file_size = ftell(fp);
    if (file_size == -1)
    {
        return -3;
    }
    // Restore previous file position
    if (fseek(fp, current_position, SEEK_SET) != 0)
    {
        return -4;
    }

    return file_size;
}

/*
 * Test if the BMPHeader is consistent with itself and the already open image file.
 * 
 * Return: true if and only if the given BMPHeader is valid.
 */
bool check_bmp_header(BMPHeader* bmp_header, FILE* fp)
{
    /*
    A header is valid if:
    1. its magic number is 0x4d42,
    2. image data begins immediately after the header data (header->offset == BMP HEADER SIZE),
    3. the DIB header is the correct size (DIB_HEADER_SIZE),
    4. there is only one image plane,
    5. there is no compression (header->compression == 0),
    6. num_colors and  important_colors are both 0,
    7. the image has 24 bits per pixel,
    8. the size and image_size_bytes fields are correct in relation to the bits,
       width, and height fields and in relation to the file size.
    */
    return
        bmp_header->type == MAGIC_VALUE
        && bmp_header->offset == BMP_HEADER_SIZE
        && bmp_header->dib_header_size == DIB_HEADER_SIZE
        && bmp_header->num_planes == NUM_PLANE
        && bmp_header->compression == COMPRESSION
        && bmp_header->num_colors == NUM_COLORS && bmp_header->important_colors == IMPORTANT_COLORS
        && bmp_header->bits_per_pixel == BITS_PER_PIXEL
        && bmp_header->size == _get_file_size(fp) && bmp_header->image_size_bytes == _get_image_size_bytes(bmp_header);
}

/*
 * Read a BMP image from an already open file.
 * 
 * - Postcondition: it is the caller's responsibility to free the memory
 *   for the error message and the returned image.
 * 
 * - Return: the image as a BMPImage on the heap.
 */
BMPImage *read_bmp(FILE *fp)
{    
    BMPImage *image = (BMPImage*)malloc(sizeof(*image));
		if (image == NULL) {
			ROS_INFO("-------- MapZone : Not enough memory for BMPImage -----------");
			return NULL;
		}
    // Read header
    rewind(fp);
    int num_read = fread(&image->header, sizeof(image->header), 1, fp);
		if (num_read != 1) {
			ROS_INFO("-------- MapZone : Cannot read Image header -----------");
			return NULL;
		}
    // Check header
    bool is_valid_header = check_bmp_header(&image->header, fp);
		if (!is_valid_header) {
			ROS_INFO("-------- MapZone : Invalid BMP file -----------");
			return NULL;
		}
    // Allocate memory for image data
    image->data = (unsigned char*)malloc(sizeof(*image->data) * image->header.image_size_bytes);
		if (image->data == NULL) {
			ROS_INFO("-------- MapZone : Not enough memory for Image Data -----------");
			return NULL;
		}
    // Read image data
    num_read = fread(image->data, image->header.image_size_bytes, 1, fp);
		if (num_read != 1) {
			ROS_INFO("-------- MapZone : Cannot read Image Data -----------");
			return NULL;
		}
    return image;
}

void loadNoGoZone(void)
{
	int width,height,yw,xp,yp,id;
	BMPHeader *bh;
	unsigned char bb,gg,rr;
	BMPImage *bimg;
	FILE *fp;
  
	fp = fopen(NoGoZoneFile.c_str(), "r");
  if(fp==NULL){
  	ROS_INFO("\n-------- BMP : Failed to Open NoGoZoneFile ----------------\n");
		return;
  }
	bimg = read_bmp(fp);
	bh = &bimg->header;
	map_width = bh->width_px;
	map_height = bh->height_px;
	yw = _get_image_row_size_bytes(bh);
	//xp = _get_position_x_row(0,bh);
	yp = 0;

	if ((map_width  > MAX) || (map_height  > MAX))
	{
		ROS_INFO("\n ***** MapZone : ERROR - image too big for current image structure ****** \n");
		return;
	}
	ROS_INFO("\n--------- MapZone : Width=%d. Height=%d --------------\n",map_width,map_height);
	map_data = (char*)malloc(sizeof(char) * ((map_width * map_height)+1));
  for (int i=0;i<map_height;i++) {
  	// Iterate image's rows
    for (int j = 0; j < map_width; j++) {
    	// Iterate image's pixels
			xp = _get_position_x_row(j, &bimg->header);
			bb = bimg->data[yp + xp];
			gg = bimg->data[yp + xp + 1];
			rr = bimg->data[yp + xp + 2];
			id = j + (i * map_width);
			if ((bb < 0x20) && (gg < 0x20) && (rr > 0xf0)) {
				//ROS_INFO("\n---- BMP : x=%d. y=%d : bb=%x. gg=%x. rr=%x. id=%d ---------\n",j,i,bb,gg,rr,id);
				map_data[id] = NOGOZONE;
			} else {
				map_data[id] = 0;
			}
  	}
		yp += yw;        
	}
	mapflag = true;
	ROS_INFO("\n-----------MapZone : Done reading BMP file. -------------------\n");
}


/*Gets an ascii pgm image file, store as a color pgm.*/
void getPGMfile (const char filename[], PGMImage *img)
{
	FILE *in_file;
	char ch;
	int row, col, type;
	int ch_int;
	int id,dat,ret;
	in_file = fopen(filename, "r");
	//in_file = fopen("/home/racnys/catkin_ws/src/htbot/maps/docmap.pgm", "r");
	if (in_file == NULL)
	{
		fprintf(stderr, "Error: Unable to open file %s\n\n", filename);
		exit(8);
	}
  
	printf("\nReading image file: %s\n", filename);
    
	/*determine pgm image type (only type three can be used)*/
	ch = getc(in_file);
	if(ch != 'P')
	{
		printf("ERROR(1): Not valid pgm/ppm file type\n");
		exit(1);
	}
	ch = getc(in_file);
	/*convert the one digit integer currently represented as a character to
		an integer(48 == '0')*/
	type = ch - 48;
	printf("\n PGM Type : %d\n",type);
	if((type != 2) && (type != 3) && (type != 5) && (type != 6))
	{
		printf("ERROR(2): Not valid pgm/ppm file type\n");
		exit(1);
	}
  //printf("\n Here 1\n");
	while(getc(in_file) != '\n');             /* skip to end of line*/
	//printf("\n Here 2\n");
	while (getc(in_file) == '#')              /* skip comment lines */
	{
		while (getc(in_file) != '\n');          /* skip to end of comment line */
	}
	//printf("\n Here 3\n");
	/*there seems to be a difference between color and b/w.  This line is needed
		by b/w but doesn't effect color reading...*/
	fseek(in_file, -1, SEEK_CUR);             /* backup one character*/
	//printf("\n Here 4\n");
	ret = fscanf(in_file,"%d", &((*img).width));
	ret = fscanf(in_file,"%d", &((*img).height));
	ret = fscanf(in_file,"%d", &((*img).maxVal));
	map_width = (*img).width;
	map_height = (*img).height;
	printf("\n width  = %d",(*img).width);
	printf("\n height = %d",(*img).height);
	printf("\n maxVal = %d",(*img).maxVal);
	printf("\n");
	//printf("\n Here A\n");
	if (((*img).width  > MAX) || ((*img).height  > MAX))
	{
		printf("\n\n***ERROR - image too big for current image structure***\n\n");
		exit(1);
	}
	 
	if(type == 2) /*uncompressed ascii file (B/W)*/
	{
		for (row=(*img).height-1; row >=0; row--)
			for (col=0; col< (*img).width; col++)
			{
				ret = fscanf(in_file,"%d", &ch_int);
				(*img).data[row][col].red = ch_int;
				(*img).data[row][col].green = ch_int;
				(*img).data[row][col].blue = ch_int;
			}
	}
	else if(type == 3) /*uncompressed ascii file (color)*/
	{
		for (row=(*img).height-1; row >=0; row--)
			for (col=0; col< (*img).width; col++)
			{
	  	  
				ret = fscanf(in_file,"%d", &ch_int);
				((*img).data[row][col].red) = (unsigned char)ch_int;
	       
				ret = fscanf(in_file,"%d", &ch_int);
				((*img).data[row][col].green) = (unsigned char)ch_int;
	  	  
				ret = fscanf(in_file,"%d", &ch_int);
				((*img).data[row][col].blue) = (unsigned char)ch_int;
			}
	}
	else if(type == 5) /*compressed file (B/W)*/
	/*note: this type remains untested at this time...*/
	{
		printf("\n Type 5 PGM file\n");
		while(getc(in_file) != '\n'); /*skip to end of line*/
	  
		for (row=(*img).height-1; row >=0; row--)
			for (col=0; col< (*img).width; col++)
			{
				ch = getc(in_file);
				(*img).data[row][col].red = ch;
				(*img).data[row][col].green = ch;
				(*img).data[row][col].blue = ch;
				//printf("\n Data at %d, %d, %d",row,col,ch);
				if (ch < -30) {
					id = col + ((map_height - row - 1) * map_width);
					map_data[id] = ch;
					//printf("\n Unknown at %d, %d, %d. id=%d\n",row,col,ch,id);
				}
			}
	}
	  
	else if(type == 6) /*compressed file (color)*/
	{
		while(getc(in_file) != '\n'); /*skip to end of line*/
	  
		for (row=(*img).height-1; row >=0; row--)
			for (col=0; col< (*img).width; col++)
			{
				(*img).data[row][col].red = getc(in_file);
				(*img).data[row][col].green = getc(in_file);
				(*img).data[row][col].blue = getc(in_file);
			}
	}
	  
	fclose(in_file);
	
	//id = 58 + (110 * map_width);
	//dat = map_data[id];
	//ROS_INFO("Map : id = %d. Data(80,80) = %d",id,dat);
	//id = 160 + (120 * map_width);
	//dat = map_data[id];
	//ROS_INFO("Map : id = %d. Data(160,120) = %d",id,dat);
	
	mapflag = true;
	ROS_INFO("\n-----------MapZone : Done reading PGM file. -------------------");
}

void poseCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
	boost::mutex::scoped_lock lock(publish_mutex_);
	posex = msg->position.x;
	posey = msg->position.y;	
	poserz = msg->orientation.z;
	poserw = msg->orientation.w;
	//ROS_INFO("posex=%.3f",posex);
	poseflag = true;
}

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	double x, y, z,rx,ry,rz,rw;
	int id,dat;
	boost::mutex::scoped_lock lock(publish_mutex_);
	ROS_INFO("--------------MapZone : Start map callback -----------------------");
	map_width = msg->info.width;
	map_height = msg->info.height;
	x = msg->info.origin.position.x;
	y = msg->info.origin.position.y;
	z = msg->info.origin.position.z;
	rx = msg->info.origin.orientation.x;
	ry = msg->info.origin.orientation.y;
	rz = msg->info.origin.orientation.z;
	rw = msg->info.origin.orientation.w;

	/*
	map_size = msg->data.size();
	//memcpy(map_data, msg->data, size);
	for (int i=0;i<map_size;i++) {
		map_data[i] = (char)msg->data[i];
	}
	id = 80 + ((map_height - 80 - 1) * map_width);
	//dat = msg->data[id];
	dat = map_data[id];
	ROS_INFO("Map : Width = %d. Height = %d. Size of Data = %d. Data(80,80) = %d",map_width,map_height,map_size,dat);
	//ROS_INFO("Map : x=%.3f. y=%.3f. z=%.3f. rx=%.3f. ry=%.3f. rz=%.3f. rw=%.3f. ",x,y,z,rx,ry,rz,rw);
	mapflag = true;
	*/
	ROS_INFO("--------------MapZone : Start map callback -----------------------");
	mapcallbackflag = true;
}

void publish_robot_pos()
{
	boost::mutex::scoped_lock lock(publish_mutex_);
	htbot::move mv;
	mv.x = posex;
	mv.y = posey;
	robot_pub.publish(mv);
	return;
}

/*
void readCmdFromFM() {

}

void sendInfoToFM() {

}
*/

void publish_pose() {
	pose_msg.position.x = 1.7422;
	pose_msg.position.y = 2.5862;
	pose_msg.position.z = 0.0;
	pose_msg.orientation.x = 0.0;
	pose_msg.orientation.y = 0.0;
	pose_msg.orientation.z = -0.0049;
	pose_msg.orientation.w = 1.0000;
	pose_pub.publish(pose_msg);
}

void generateObsMap() {
	int pcx,pcy,widx;
	int x1,x2,y1,y2,id,dat,lidx;
	double an,anl,dist,xan;
	ros::NodeHandle nx;
	tf::Quaternion qp(0.0,0.0,poserz,poserw);
	yawp = tf::getYaw(qp);
	int leftside; // 1=left side of robot axis. 0 = right side
	int rightcount,leftcount,BackObs1;
	sensor_msgs::LaserScan scan_msg;
	// clear laser data
	for (int i=0;i<MAX_IDX;i++) {
		laser_data[i] = MAX_RANGE;
	}
	// cells of robot pose
	//ROS_INFO("Here A");
	BackObs1 = 0;
	nx.getParam("BackObs1",BackObs1);
	pcx = (int)((posex+MapOffsetX) / Resolution);
	pcy = (int)((posey+MapOffsetY) / Resolution);
	//pcy = map_height - pcy;
	widx = (int)(Window / Resolution);
	//widx = (int)(WINDOW_WIDTH / Resolution);
	x1 = pcx - widx;
	x2 = pcx + widx;
	if (x1 < 0) {
		x1 = 0;
	}
	if (x2 > map_width) {
		x2 = map_width;
	}
	y1 = pcy - widx;
	y2 = pcy + widx;
	if (y1 < 0) {
		y1 = 0;
	}
	if (y2 > map_height) {
		y2 = map_height;
	}
	//ROS_INFO(" ================ Start Cycle =======================");
	//ROS_INFO("x=%.3f. y=%.3f. Res=%.3f. widx=%d. x1=%d. x2=%d. y1=%d. y2=%d",posex,posey,Resolution,widx,x1,x2,y1,y2);
	for (int r=y1;r<y2;r++) {
		for (int c=x1;c<x2;c++) {
			//id = c + ((map_height - r - 1) * map_width);
			id = c + (r * map_width);
			dat = map_data[id];
			//if (dat > 80) {
			//if (dat < -30) {
			//ROS_INFO("Map A : dat=%d. row=%d. col=%d",dat,r,c);
			if (dat == NOGOZONE) {
				//ROS_INFO("-------- MapZone B : dat=%d. row=%d. col=%d. id=%d ---------------------",dat,r,c,id);
				// obs. calculate angle of obs.
				if ((r < pcy) && (c > pcx)) {
					// left-top of robot. 2Q. angle is positive rad. ##top-right positive
					// bottom right
					if (c > pcx) {
						//an = 3.1416 - atan((pcy - r)*1.0/(c - pcx)*1.0);
						an = - atan((pcy - r)*1.0/(c - pcx)*1.0);
					}
					//ROS_INFO("-----Mapzone : BR an=%.3f. col=%d. row=%d------------",an,r,c);
				} else {
					if ((r >= pcy) && (c > pcx)) {
						// top-right of robot. 1Q angle is negative rad. ###bot-right. neg
						if (c > pcx) {
							//an = -atan((r - pcy)*1.0/(c - pcx)*1.0);
							//an = -(3.1416-atan((r - pcy)*1.0/(c - pcx)*1.0));		
							an = atan((r - pcy)*1.0/(c - pcx)*1.0);
						}
						//ROS_INFO("-----Mapzone : TR an=%.3f. col=%d. row=%d-----------",an,c,r);
					}	else {
						if ((r >= pcy) && (c <= pcx)) {
							// bottom-right of robot. 4Q. angle is negative rad. ### bot-left. neg
							// top right
							if (pcx > c) {
								//an = -3.1416 + atan((r - pcy)*1.0/(pcx - c)*1.0);
								//an = -atan((r - pcy)*1.0/(pcx - c)*1.0);		
								an = 3.1416 - atan((r - pcy)*1.0/(pcx - c)*1.0);
							}
							//ROS_INFO("bl. an=%.3f. px=%d. py=%d",an,pcx,pcy);
							//ROS_INFO("-----Mapzone : TL an=%.3f. px=%d. py=%d-------------",an,pcx,pcy);
						} else {
							// bottom-left of robot. 3Q. angle is positive rad. ##top-left. pos
							if (pcx > c) {
								//an = 3.1416 - atan((pcy-r)*1.0/(pcx - c)*1.0);
								//an = atan((pcy-r)*1.0/(pcx - c)*1.0);		
								an = -(3.1416 - atan((pcy-r)*1.0/(pcx - c)*1.0));		
							}
							//ROS_INFO("tl. an=%.3f. px=%d. py=%d",an,pcx,pcy);
							//ROS_INFO("-----Mapzone : BL an=%.3f. px=%d. py=%d------------",an,pcx,pcy);
						}
					}		
				}
				// angle/index of obs reference to laser axis
				if (an < 0.0) {
					an = PI2 + an;
				}
				if (yawp < 0.0) {
					yawp = PI2 + yawp;
				}
				anl = an - yawp;
				if (anl > PI) {
					anl = anl - PI2;
				}
				if (anl < -PI) {
					anl = PI2 + anl;
				}
				//ROS_INFO("------------ MapZone : dat=%d. r=%d. c=%d. anl=%.3f. yawp=%.3f---------",dat,r,c,anl,yawp);
				//if ((anl > 1.5709) || (anl < -1.5709)) {
				//if ((anl > 2.0923) || (anl < -2.0923)) {
				if ((anl > 3.05433) || (anl < -3.05433)) {
				//if ((anl > 3.107) || (anl < -3.107)) {
				//if ((anl > 2.8) || (anl < -2.8)) {
					// infront of robot only
					continue;
				}
				lidx = CIDX + (int)(anl / RADPIDX);
				if (lidx >= MAX_IDX) {
					continue;
				}
				//ROS_INFO("\n------------ MapZone : dat=%d. r=%d. c=%d. anl=%.3f. yawp=%.3f. lidx=%d---------\n",dat,r,c,anl,yawp,lidx);
				// dist of obs to laser. assume map axis is pointing along left to right of map
				//dist = sqrt((r-pcy)*(r-pcy)*0.0025 + (c-pcx)*(c-pcx)*0.0025);
				dist = sqrt((r-pcy)*(r-pcy)*ressq + (c-pcx)*(c-pcx)*ressq);
				//ROS_INFO("***** obs. dist=%.3f. idx=%d  ******",dist,lidx);
				if (dist < laser_data[lidx]) {
					laser_data[lidx] = dist;
				}
			}
		}
	}
	// publis laser scan
	//ROS_INFO("Here B");
	scan_msg.ranges.resize(MAX_IDX);
	//scan_msg.header.seq = lseq++;
	//if (lseq > UINT32_MAX) {
	//	lseq = 0;
	//}
	scan_msg.header.stamp = ros::Time::now();
	scan_msg.header.frame_id = "/maplaser";
	scan_msg.angle_min = -3.05433; //-3.107; //-3.05433; //-2.0923; //-1.5708;
  scan_msg.angle_max = 3.05433; //3.107; //3.05433; //2.0923; //1.5708;
	scan_msg.angle_increment = 6.10865 / MAX_IDX; // 4.1846 / MAX_IDX;  //3.1416 / MAX_IDX;
  scan_msg.time_increment = (1.0 / MapObsRate) / (MAX_IDX);
  scan_msg.range_min = MIN_RANGE;
  scan_msg.range_max = MAX_RANGE;
	//if (lookforObs == 1) {
	cidx = CIDX;
	rmidx = cidx - DMIDX;
	lmidx = cidx + DMIDX;
	ridx = cidx - DGIDX;
	lidx = cidx + DGIDX;
	od = 0.0; // obs dist in front of robot
	tod = 0.0;
	//}
	rightcount = 0;
	leftcount = 0;
	for (int i=0;i<MAX_IDX;i++) {
		scan_msg.ranges[i] = MAX_RANGE;
		leftside = 0;  // assume right side		
		/*
		if (BackObs1 > 0) {
			if (i<START_IDX) {
				laser_data[i] = (DEPTH / cos((i+2)*RADPIDX)) ;
			} else {
				if ((i>=START_IDX) && (i < 88)) {
					laser_data[i] = (WINDOW_WIDTH / sin((i+2)*RADPIDX)) ;
				} else {
					if ((i>=88) && (i < RIGHT_IDX)) {
						laser_data[i] = (WINDOW_WIDTH / cos((i-88)*RADPIDX)) ;
					} else {
						if ((i>=LEFT_IDX) && (i < 268)) {
							laser_data[i] = (WINDOW_WIDTH / sin((i-178)*RADPIDX)) ;
						} else {
							if ((i>=268) && (i < END_IDX)) {
								laser_data[i] = (WINDOW_WIDTH / cos((i-268)*RADPIDX)) ;
							} else {
								laser_data[i] = (DEPTH / sin((i-268)*RADPIDX)) ;
							}
						}
					}
				}
			}
		}
		*/
		scan_msg.ranges[i] = laser_data[i];
		lrd = laser_data[i];
		//if (lookforObs == 1) {
			if ((i >= ridx) && (i <= rmidx)) {
				dt = (cidx-i) * RADPIDX;  // rad
				dtt = sin(dt);
				if (dtt != 0.0) {
					refr = DWIDTH / sin(dt);
				} else {
					ROS_INFO("Error. Div by Zero. A. i=%d",i);
					refr = MAX_RANGE;
				}
				leftside = 0;
			}	
			if ((i > rmidx) && (i <= cidx)) {
				dt = (cidx-i) * RADPIDX;
				dtt = cos(dt);
				if (dtt != 0.0) {
					refr = (DBOX) / cos(dt);
				} else {
					ROS_INFO("Error. Div by Zero. B. i=%d",i);
					refr = MAX_RANGE;
				}				
				leftside = 0;
			}	
			if ((i > cidx) && (i <= lmidx)) {
				dt = (i - cidx) * RADPIDX;
				dtt = cos(dt);
				if (dtt != 0.0) {
					refr = (DBOX) / cos(dt);
				} else {
					ROS_INFO("Error. Div by Zero. C. i=%d",i);
					refr = MAX_RANGE;
				}	
				leftside = 1;
				//refr = (DBOX) / cos(dt);
			}
			if ((i >= lmidx) && (i <= lidx)) {
				dt = (i - cidx) * RADPIDX;
				dtt = sin(dt);
				if (dtt != 0.0) {
					refr = DWIDTH / sin(dt);
				} else {
					ROS_INFO("Error. Div by Zero. D. i=%d",i);
					refr = MAX_RANGE;
				}
				leftside = 1;
				//refr = DWIDTH / sin(dt);
			}
			if ((refr > 0.0) && (lrd < refr)) {  // obs in box
				tod = lrd * cos(dt);
				if ((tod < od) || (od == 0.0)) {
					od = tod;
				}
				if (leftside == 0) {  // right side
					if (lrd < DISTGT350MM) {
						rightcount++;
					}
				} else {
					if (lrd < DISTGT350MM) {
						leftcount++;
					}					
				}
			}
		//}
	}
	//if (lookforObs == 1) {
	//od = od - DWIDTH;
	if (od > 0.0) {
		nx.setParam("MapObsDist",od);	
	} else {
		nx.setParam("MapObsDist",5.0);
	}
	//}
	//ROS_INFO("Here C");
	nx.setParam("MapDirectionToSearch",(rightcount - leftcount));
	scan_pub.publish(scan_msg);
	//ROS_INFO("Here D");
	//if (i<START_IDX) {
	//	scan_msg.ranges[i] = (DEPTH / cos((i+5)*RADPIDX)) ;
	//} else {
	//	if (i>END_IDX) {
	//		scan_msg.ranges[i] = (DEPTH / cos((MAX_IDX-i)*RADPIDX)) ;
	//	} else {
	//		scan_msg.ranges[i] = laser_data[i];
	//	}
	//}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "MapZoneNode");
  ros::NodeHandle n;
	char buf [30];
	int ret,readyflag,cnt;
	bool ready;	
	PGMImage* pgmimg = (PGMImage*)malloc(sizeof(PGMImage));
	ready = false;
	poseflag = false;
	mapflag = false;
	mapcallbackflag = false;
	//ros::Rate loop_rate(MapObsRate);
	pose_sub = n.subscribe<geometry_msgs::Pose>("/robot_pose", 1 ,poseCallback);
	//map_sub = n.subscribe<nav_msgs::OccupancyGrid>("/map", 1 ,mapCallback);
	scan_pub = n.advertise<sensor_msgs::LaserScan>("mapscan", 100);
	//pose_pub = n.advertise<geometry_msgs::Pose>("/robot_pose", 10);
	
	n.param("Resolution",Resolution,0.05);
	n.param("Window",Window,1.5);
	n.param("MapOffsetX",MapOffsetX,0.0);
	n.param("MapOffsetY",MapOffsetY,0.0);
	n.param("MapObsRate",MapObsRate,20.0);
	n.getParam("NoGoZoneFile",NoGoZoneFile);	
	ROS_INFO("MapObsRate=%.3f. OffsetX=%.3f. OffsetY=%.3f. NoGoZoneFile=%s",MapObsRate,MapOffsetX,MapOffsetY,NoGoZoneFile.c_str());
	
	ressq = Resolution * Resolution;
	ros::Rate loop_rate(MapObsRate);

	loadNoGoZone();
	//getPGMfile ("/home/rac/catkin_ws/src/htbot/maps/docmap_obs.pgm", pgmimg);
	sleep(1);
	//ROS_INFO("----------- MapZone : Point A ---------------");
	//while(true) {
	//	if (mapcallbackflag) {
	//		break;
	//	}
	//}
	//tf::Quaternion tt(0.0,0.0,0.9998,-0.0214);
	//double yawptt = tf::getYaw(tt);
	//tf::Quaternion ss(0.0,0.0,0.0076,1.0000);
	//double yawpss = tf::getYaw(ss);
	//tf::Quaternion s1(0.0,0.0,-0.7114,0.7028);
	//double yawps1 = tf::getYaw(s1);
	//ROS_INFO("\n\n--------MapZone Test Angle : ref=%.3f. stn3=%.3f.stn1=%.3f-------------\n\n",yawptt,yawpss,yawps1);
	cnt = 0;
	while(true) {
		n.getParam("RobotReady",readyflag);
		if (readyflag == 7) {
			break;
		}
		sleep(1);
		cnt++;
		if (cnt > 10) {
			break;
		}
	}
	
	ROS_INFO("----------- MapZone : Starting ---------------");
	while (true) {  	  	 			
		//sendInfoToFM();
		//readCmdFromFM();
		//publish_pose();
		if (poseflag && mapflag) {			
			generateObsMap();
			if (!ready) {
				ROS_INFO(" -------- mapzone Node : Node is ready ----------");
				ready = true;
			}
		}
		ros::spinOnce();	
  	loop_rate.sleep();
  }
	
  //ros::spin();

  return 0;
}



