#ifndef OWI_INTERFACE_H
#define OWI_INTERFACE_H

#include <string>
#include <list>
#include <map>
#include <libusb-1.0/libusb.h>

#define BASE_JOINT_NAME "base_joint"
#define SHOULDER_NAME "shoulder"
#define ELBOW_NAME "elbow"
#define WRIST_NAME "wrist"
#define GRIPPER_JOINT_NAME "gripper_joint"
#define LED_NAME "led"

class owi_interface
{
public:
	owi_interface() 
 	{ 
		devh = NULL;
		devs = NULL;
		
    joint_state_map[BASE_JOINT_NAME] = 0;
    joint_state_map[SHOULDER_NAME] = 0;
    joint_state_map[ELBOW_NAME] = 0;
		joint_state_map[WRIST_NAME] = 0;
		joint_state_map[GRIPPER_JOINT_NAME] = 0;
		joint_state_map[LED_NAME] = 0;
		libusb_initialized = false;
  }

	~owi_interface() 
	{
		close_device();
	}

	int open_device();
	void close_device();
	void set_joint_state(const std::string& name, unsigned char state);
	unsigned char get_joint_state(const std::string& name);

	int send_command();
	void stop();

private:
	bool libusb_initialized;
	libusb_device * find_arm(libusb_device **devs);

  typedef std::map<std::string, unsigned char > JointStateMap;
	JointStateMap joint_state_map;

	libusb_device **devs;
  struct libusb_device_handle *devh;
};
#endif
