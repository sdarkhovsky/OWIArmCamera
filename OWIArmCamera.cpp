#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <random>
#include <algorithm>

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

std::string joint_name[] = 
{
//	"base_joint1", 
	"shoulder 1", 
//	"elbow 1", 
//	"wrist1", 
//	"gripper_joint1",
//	"base_joint1", 
	"shoulder 2", 
//	"elbow 2", 
//	"wrist1", 
//	"gripper_joint1"

};

// Run the program with administrator privileges as sending a commmand 
// to the OWI Arm USB controller requires to send data to its USB device:
// sudo ./OWIArmCamera <num_samples_to_generate>
int main(int argc, char** argv)
{
    int status = 0;
	std::string cmdCameraCapture, cmdOWIArmControl, cmdSuffix;
	std::string cmdCameraCapturePrefix = "./CameraCapture/CameraCapture 1 ./TrainingSamples/img";
    int num_samples=2;
	int joint_name_count = sizeof(joint_name)/sizeof(*joint_name);
    
    std::default_random_engine generator;
    std::uniform_int_distribution<> distrib(0, joint_name_count-1);
/*
    for (int i=0; i < 30; i++)
	{
		std::cout << joint_name[distrib(generator)] << std::endl;
	}

	exit(1);
*/
	if (argc > 1) num_samples=std::atoi(argv[1]);
    for (int i=0; i < num_samples; i++)
	{
        cmdCameraCapture = cmdCameraCapturePrefix + std::to_string(i) + cmdSuffix + ".png";
		execute_command(cmdCameraCapture);

		if (i >= num_samples-1)
			break;
		
		// randomly select the move and describe it in the suffix
 		int joint = distrib(generator);
		
		cmdOWIArmControl = "./OWIArmControl/OWIArmControl " + joint_name[joint]; 
		execute_command(cmdOWIArmControl);
		cmdSuffix = " " + joint_name[joint];
		std::replace( cmdSuffix.begin(), cmdSuffix.end(), ' ', '_'); 
	}
	return status;
}
