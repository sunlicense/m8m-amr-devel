#include <ros/ros.h>
#include <signal.h>
#include <termios.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include "htbot/status.h"

#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include "string.h"
#include <linux/input.h>

char cvt_ev_char(int);
void debug_rcvd_event(struct input_event *);
#define MAX_IN_SIZE 30

ros::Publisher barcode_pub;
htbot::status bcmsg;

void publish(std::string bc)
{
	bcmsg.msg = bc;
	barcode_pub.publish(bcmsg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "BarCode Reader Node");

  ros::NodeHandle n;
	std::string s;
  //ros::Publisher barcode_pub;
	//htbot::status bcmsg;
  barcode_pub = n.advertise<htbot::status>("barcode", 1000);

	struct input_event ev[64]; /* each scan causes more than one event...get them all */
	int fd = -1; /* file descriptor for the scanner device */
	int bytes_read = 0; /* number of bytes read by the read function */
	int i; /* loop variable */
	char scanner_in[30]; /* set this bigger if scanned input is more than 30 characters */
	char scan_char; /* this holds one char at a time...will be concatenated into scanner_in[] */
	int wr_ptr = 0; /* points into scanner_in so we know where to put the next scan_char */
	int count;

	if ((fd = open("/dev/sensors/bar", O_RDONLY)) < 0) {
		ROS_INFO("Error opening file descriptor - do you have sufficient permission?");
		return -1;
	}
	ROS_INFO("Starting BarCode Node");
	count = 0;
	while (ros::ok()) {
		bytes_read = read(fd, &ev, sizeof(struct input_event) * 64);
		if (bytes_read < 0) {
			printf("ERROR: can't properly read from device\n");
			return -1;
		}
		//ROS_INFO("BarCode Node Loop");
		for (i=0; i < (int) (bytes_read / sizeof(struct input_event)); i++) {
			/* Look for a "key press" event */
			if ((ev[i].type == EV_KEY) && (ev[i].value == 1)) {
				if (ev[i].code != KEY_LEFTSHIFT) {
					scan_char = cvt_ev_char(ev[i].code); /* Extract the character from the event */
					if (ev[i].code != KEY_ENTER) {
						scanner_in[wr_ptr++] = scan_char;
					}
					else {						
						scanner_in[wr_ptr] = '\0';
						s.assign(scanner_in,wr_ptr);
						ROS_INFO("Input from Scanner: %s",s.c_str());						
						n.setParam("Password",s);
						publish(s);
						wr_ptr = 0;
						//count = 3;
						//sleep(5);						
					}
				} /* if (ev[i].code ...) */
			} /* if ((ev[i].type.....)) */
		} /* for (i=0...) */
	} /* while (1) */
	close(fd);
	return 0;
} /* main */

char cvt_ev_char(int foo) {
	char bar;

	switch (foo) {
		case KEY_0: bar = '0'; break;
		case KEY_1: bar = '1'; break;
		case KEY_2: bar = '2'; break;
		case KEY_3: bar = '3'; break;
		case KEY_4: bar = '4'; break;
		case KEY_5: bar = '5'; break;
		case KEY_6: bar = '6'; break;
		case KEY_7: bar = '7'; break;
		case KEY_8: bar = '8'; break;
		case KEY_9: bar = '9'; break;
		case KEY_A: bar = 'A'; break;
		case KEY_B: bar = 'B'; break;
		case KEY_C: bar = 'C'; break;
		case KEY_D: bar = 'D'; break;
		case KEY_E: bar = 'E'; break;
		case KEY_F: bar = 'F'; break;
		case KEY_G: bar = 'G'; break;
		case KEY_H: bar = 'H'; break;
		case KEY_I: bar = 'I'; break;
		case KEY_J: bar = 'J'; break;
		case KEY_K: bar = 'K'; break;
		case KEY_L: bar = 'L'; break;
		case KEY_M: bar = 'M'; break;
		case KEY_N: bar = 'N'; break;
		case KEY_O: bar = 'O'; break;
		case KEY_P: bar = 'P'; break;
		case KEY_Q: bar = 'Q'; break;
		case KEY_R: bar = 'R'; break;
		case KEY_S: bar = 'S'; break;
		case KEY_T: bar = 'T'; break;
		case KEY_U: bar = 'U'; break;
		case KEY_V: bar = 'V'; break;
		case KEY_W: bar = 'W'; break;
		case KEY_X: bar = 'X'; break;
		case KEY_Y: bar = 'Y'; break;
		case KEY_Z: bar = 'Z'; break;
		case KEY_ENTER: bar = '\n'; break;
		default: bar = '?';
	}
	return bar;
}

void debug_rcvd_event(struct input_event *ev) {
	char type_str[15];
	
	switch (ev->type) {
		case EV_SYN: strcpy(type_str, "EV_SYN"); break;
		case EV_KEY: strcpy(type_str, "EV_KEY"); break;
		case EV_REL: strcpy(type_str, "EV_REL"); break;
		case EV_ABS: strcpy(type_str, "EV_ABS"); break;
		case EV_MSC: strcpy(type_str, "EV_MSC"); break;
		case EV_SW: strcpy(type_str, "EV_SW"); break;
		case EV_LED: strcpy(type_str, "EV_LED"); break;
		case EV_SND: strcpy(type_str, "EV_SND"); break;
		case EV_REP: strcpy(type_str, "EV_REP"); break;
		case EV_FF: strcpy(type_str, "EV_FF"); break;
		case EV_PWR: strcpy(type_str, "EV_PWR"); break;
		case EV_MAX: strcpy(type_str, "EV_MAX"); break;
		case EV_FF_STATUS: strcpy(type_str, "EV_FF_STATUS"); break;
		default: strcpy(type_str, "UNK");
	}
}

