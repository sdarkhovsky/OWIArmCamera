#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <random>

void execute_command(std::string cmd)
{
	int status = std::system(cmd.c_str());
	if (status)
	{
		std::cout << "command failed:" << cmd;
		exit(1);
	}
}

std::string joint_name[] = {"base_joint", "shoulder", "elbow", "wrist", "gripper_joint"};

int main(int argc, char** argv)
{
    int status = 0;
	std::string cmdCameraCapture, cmdOWIArmControl, cmdSuffix;
	std::string cmdCameraCapturePrefix = "./CameraCapture/CameraCapture 1 ./TrainingSamples/img";
    int num_samples=10;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> joint_dis(0, 4);
    std::uniform_int_distribution<> direction_dis(1, 2);
    
    for (int i=0; i < num_samples; i++)
	{
        cmdCameraCapture = cmdCameraCapturePrefix + std::to_string(i) + cmdSuffix + ".png";
		execute_command(cmdCameraCapture);
		if (i == 0) continue;
		
		// randomly select the move and describe it in the suffix
 		int joint = joint_dis(gen);
		int direction = direction_dis(gen);
		
		cmdOWIArmControl = "OWIArmControl " + joint_name[joint] + " " + std::to_string(direction); 
		execute_command(cmdOWIArmControl);
		cmdSuffix = joint_name[joint] + "_" + std::to_string(direction);
	}
	return status;
}
