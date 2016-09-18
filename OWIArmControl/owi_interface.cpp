#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <string.h>

#include "owi_interface.h"

#define EP_INTR	(1 | LIBUSB_ENDPOINT_IN)

#define ARM_VENDOR       0x1267
#define ARM_PRODUCT      0
#define CMD_DATALEN      3

bool verbose = false;

libusb_device * owi_interface::find_arm(libusb_device **devs)
{
	libusb_device *dev;
	int i = 0;

	while ((dev = devs[i++]) != NULL) {
		struct libusb_device_descriptor desc;
		int r = libusb_get_device_descriptor(dev, &desc);
		if (r < 0) {
			fprintf(stderr, "failed to get device descriptor");
			return NULL;
		}
		if(desc.idVendor == ARM_VENDOR &&
		   desc.idProduct == ARM_PRODUCT)
		  {
		    return dev;
		  }
	}
	return NULL;
}

void owi_interface::close_device()
{
		if (devh != NULL)
		{
	    libusb_close(devh);
			devh = NULL;
		}

		if (devs != NULL)
		{
			libusb_free_device_list(devs, 1);
			devs = NULL;
		}

		if (libusb_initialized)
		{
			libusb_exit(NULL);
			libusb_initialized = false;
		}
}

int owi_interface::open_device()
{
    libusb_device *dev;
		int r;
		ssize_t cnt;

		close_device();

		r = libusb_init(NULL);
		if (r < 0)
	  {
			fprintf(stderr, "failed to initialize libusb\n");
		  return r;
	  }

		libusb_initialized = true;

	  libusb_set_debug(NULL,2);

		cnt = libusb_get_device_list(NULL, &devs);
		if (cnt < 0)
			return (int) cnt;
    dev=find_arm(devs);
    if(!dev)
    {
	    fprintf(stderr, "Robot Arm not found\n");
	    return -1;
    }

    r = libusb_open(dev,&devh);
    if(r!=0)
    {
	    fprintf(stderr, "Error opening device\n");
       	    libusb_free_device_list(devs, 1);
             libusb_exit(NULL);
	    return -1;
    }
		return r;	
}

/*
see http://notbrainsurgery.livejournal.com/38622.html
This is USB device with vendor id 0x1267 and product id 0. The device is controlled by 3 byte commands sent via USB control transfers. Command format is byte0, byte1, byte2. Each byte controls a group of arm features. All motors could be controlled independently. Most commands start action which is continued until next action is signalled. Byte '00' universally used as stop action.

Bit numbering in byte:

7 (most significant) 	6 	5 	4 	3 	2 	1 	0 (least significant)


Meaning of bits in each byte:

Byte 0

First byte controls grip, shoulder and wrist. Bits are assigned as following:

Bit numbers 	Controls 	Bit combinations
0,1 	Grip 	00-do not move, 01-close, 10-open
2,3 	Wrist 	00-do not move, 01-move up, 10-move down
4,5 	Elbow 	00-do not move, 01-move up, 10-move down
6,7 	Shoulder 	00-do not move, 01-move up, 10-move down


Sample commands, activating single motor:

Byte 0 	Command
00 	stop grip, wrist, elbow and shoulder movement
01 	grip close
02 	grip open
04 	wrist up
08 	wrist down
10 	elbow up
20 	elbow down
40 	shoulder up
80 	shoulder down


Sample commands activating 2 motors simultaneously:

Byte 0 	Command
11 	grip close and elbow up
82 	grip open and shoulder down


Byte1

Second byte controls base.
Bits are assigned as following:

Bit numbers 	Controls 	Bit combinations
0,1 	Base 	00-do not move, 01-rotate clockwise, 10-rotate counter-clocwise


Sample commands:

Byte 1 	Command
00 	stop base rotation
01 	rotate base clockwise
02 	rotate base counter-clocwise


Byte 2

Third byte controls LED light inside the grip.
Bits are assigned as following:

Bit numbers 	Controls 	Bit combinations
0 	LED 	0-off, 1-on


Sample commands:

Byte 2 	Command
00 	LED off
01 	LED on
*/

/**
 * \param name: The name of the joint or LED
 * \param state: 0 - stop the joint action, or turn LED off 
 *             	 1 - depends on the joint: close grip or move up or clockwise or turn on LED
 *               2 - depends on the joint: open grip or move down or counter-clockwise
 */
void owi_interface::set_joint_state(const std::string& name, unsigned char state)
{

    for (JointStateMap::iterator it = joint_state_map.begin(); it != joint_state_map.end(); ++it)
    {
      if (it->first == name)
      {
        it->second = state;
				return;
			}
    }
}

int owi_interface::send_command()
{
		int r;
    unsigned char cmd[3];
    cmd[0] = cmd[1] = cmd[2] = 0;

    for (JointStateMap::iterator it = joint_state_map.begin(); it != joint_state_map.end(); ++it)
    {
			if (it->first == BASE_JOINT_NAME)
			{
				cmd[1]=it->second;
			}
			else
			if (it->first == LED_NAME)
			{
				cmd[2]=it->second;
			}
			else
			{
				int shift = 0;
				unsigned char mask = 0;
				if (it->first == SHOULDER_NAME)
				{
					shift = 6;
					mask = 0x3f;
				}
				else
				if (it->first == ELBOW_NAME)
				{
					shift = 4;
					mask = 0xcf;
				}
				else
				if (it->first == WRIST_NAME)
				{
					shift = 2;
          mask = 0xf3;
				}
				else
				if (it->first == GRIPPER_JOINT_NAME)
				{
					shift = 0;
          mask = 0xfc;
				}

				cmd[0]= (cmd[0] & mask) | (it->second << shift);
			}
    }

		if (devh == NULL)
		{
	      fprintf(stderr, "send_command err: device not opened\n");
				return -1;
		}
	
		if (verbose)
		{
		  fprintf(stderr, "Sending %02X %02X %02X\n",
			      (int)cmd[0],
			      (int)cmd[1],
			      (int)cmd[2]
		  );
		}

	  	r = libusb_control_transfer(devh,
	                              0x40, //uint8_t 	bmRequestType,
	                              6, //uint8_t 	bRequest,
	                              0x100, //uint16_t 	wValue,
	                              0,//uint16_t 	wIndex,
	                              cmd,
	                              CMD_DATALEN,
	                              0	 
	  	);
	  
	  	if(r != CMD_DATALEN)
	  	{
	      fprintf(stderr, "Write err %d\n",r);
	  	}

		if (verbose)
		{
		  fprintf(stderr, "Done\n");
		}

		return r;
}

void owi_interface::stop()
{
    for (JointStateMap::iterator it = joint_state_map.begin(); it != joint_state_map.end(); ++it)
    {
      it->second = 0;
		}
}

unsigned char owi_interface::get_joint_state(const std::string& name)
{
    for (JointStateMap::iterator it = joint_state_map.begin(); it != joint_state_map.end(); ++it)
    {
      if (it->first == name)
      {
        return it->second;
			}
		}
		return 0;
}
