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

std::vector<std::string> get_joint_commands();

// Run the program with administrator privileges as sending a commmand 
// to the OWI Arm USB controller requires to send data to its USB device:
// sudo ./OWIArmCamera <num_samples_to_generate>
int main(int argc, char** argv)
{
    int status = 0;
	std::string cmdCameraCapture, cmdOWIArmControl, cmdSuffix;
	std::string cmdCameraCapturePrefix = "./CameraCapture/CameraCapture 1 ./TrainingSamples/img";
    int num_samples=2;

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
    for (int i=0; i < num_samples; i++)
	{
        cmdCameraCapture = cmdCameraCapturePrefix + std::to_string(i) + cmdSuffix + ".png";
		execute_command(cmdCameraCapture);

		if (i >= num_samples-1)
			break;
		
		// randomly select the move and describe it in the suffix
 		int icmd = distrib(generator);
		
		cmdOWIArmControl = "./OWIArmControl/OWIArmControl " + joint_commands[icmd]; 
		execute_command(cmdOWIArmControl);
		cmdSuffix = " " + joint_commands[icmd];
		std::replace( cmdSuffix.begin(), cmdSuffix.end(), ' ', '_'); 
	}
	return status;
}
