#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <random>
#include <algorithm>
#include "winsockserver/client_shared.h"

void execute_command(std::string cmd)
{
	int status = std::system(cmd.c_str());
	if (status)
	{
		std::cout << "command failed:" << cmd << std::endl;
		exit(1);
	}
	else
	{
		std::cout << "command:" << cmd << std::endl;
	}
}

std::vector<std::string> get_joint_commands();
bool socket_send_recv(const char* send_message, int send_message_len, const char* hostname="192.168.1.100");

enum CAPTURE_SRC_TYPE { CAMERA_CAPTURE_SRC, KINECT_DEPTH_CAPTURE_SRC};


// Run the program with administrator privileges as sending a commmand 
// to the OWI Arm USB controller requires to send data to its USB device:
// sudo ./OWIArmCamera <num_samples_to_generate> <capture_src> 
// where capture_src's value is one of CAPTURE_SRC_TYPE
// for example, for the Kinect capture of 3 images:  sudo ./OWIArmCamera 3 1
int main(int argc, char** argv)
{
    int status = 0;
	std::string cmdSuffix, capture_file_name, cmd, message_header;
    int num_samples=2;
	int capture_src=CAMERA_CAPTURE_SRC;

	std::vector<std::string> joint_commands = get_joint_commands();	
	int joint_commands_count = joint_commands.size();
    

    std::default_random_engine generator;
    std::uniform_int_distribution<> distrib(0, joint_commands_count-1);
/*
    for (int i=0; i < 30; i++)
	{
		std::cout << joint_commands[distrib(generator)] << std::endl;
	}

	exit(1);
*/
	if (argc > 1) num_samples=std::atoi(argv[1]);
	if (argc > 2) capture_src=std::atoi(argv[2]);
    for (int i=0; i < num_samples; i++)
	{
		switch(capture_src) {
			case KINECT_DEPTH_CAPTURE_SRC:
			{
				capture_file_name = "img" + std::to_string(i) + cmdSuffix;
	            message_header = KINECT_CAPTURE_MESSAGE;
				std::string send_message = message_header + capture_file_name;
				socket_send_recv(send_message.c_str(), send_message.size());
				break;
			}
			case CAMERA_CAPTURE_SRC:
			default:
			{
				capture_file_name = "img" + std::to_string(i) + cmdSuffix;
				cmd = "./CameraCapture/CameraCapture 1 ./TrainingSamples/" + capture_file_name;
				execute_command(cmd);
			}
		}

		if (i >= num_samples-1)
			break;
		
		// randomly select the move and describe it in the suffix
 		int icmd = distrib(generator);
		
		std::string cmd = "./OWIArmControl/OWIArmControl " + joint_commands[icmd]; 
		execute_command(cmd);
		cmdSuffix = " " + joint_commands[icmd];
		std::replace( cmdSuffix.begin(), cmdSuffix.end(), ' ', '_'); 
	}

    message_header = END_CAPTURE_MESSAGE;
	std::string send_message = message_header;
	socket_send_recv(send_message.c_str(), send_message.size());

	return status;
}
