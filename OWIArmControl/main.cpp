#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <string.h>
#include "owi_interface.h"
//#include <unistd.h>
#include <time.h>

#include "owi_arm_control.h"

// to build: g++ main.cpp owi_interface.cpp `pkg-config --libs libusb-1.0` -g -o OWIArmControl

void usage()
{
   fprintf(stderr,"Usage: OWIArmControl stop    to stop all components\n");
   fprintf(stderr,"       OWIArmControl base_joint <state>\n");
   fprintf(stderr,"       OWIArmControl shoulder <state>\n");
   fprintf(stderr,"       OWIArmControl elbow <state>\n");
   fprintf(stderr,"       OWIArmControl wrist <state>\n");
   fprintf(stderr,"       OWIArmControl gripper_joint <state>\n");
   fprintf(stderr,"       OWIArmControl led <state>\n");
   fprintf(stderr,"       where state is 0 for stop, 1 and 2 to move in a direction or turn on led\n");
}	

int main(int ac, char **av)
{
    owi_interface owi;
    if(ac < 2)
    {
				usage();
        return 1;
    }

		if (owi.open_device() < 0)
		{
			return 1;
		}
	
		if (!strcmp(av[1], "stop"))
		{
			owi.stop();
			owi.send_command();
			return 0;
		}

    if(ac != 3)
    {
				usage();
        return 1;
    }

		owi.set_joint_state(av[1], (unsigned char)strtol(av[2],NULL,16));
		owi.send_command();

//		sleep(1);
		struct timespec ts;
		ts.tv_sec = 0;
		ts.tv_nsec = OWI_COMMAND_DURATION_MILLISECONDS*1000000;
		nanosleep(&ts, NULL);

		owi.stop();
		owi.send_command();

		return 0;
}

