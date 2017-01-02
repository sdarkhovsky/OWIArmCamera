#include <vector>
#include <string>

std::vector<std::string> get_joint_commands()
{ 
	std::vector<std::string> joint_commands;
	//joint_commands.push_back("base_joint 1");
 	joint_commands.push_back("shoulder 1");
	joint_commands.push_back("elbow 1");
	//joint_commands.push_back("wrist 1");
	//joint_commands.push_back("gripper_joint 1");

	//joint_commands.push_back("base_joint 2");
 	joint_commands.push_back("shoulder 2");
	joint_commands.push_back("elbow 2");
	//joint_commands.push_back("wrist 2");
	//joint_commands.push_back("gripper_joint 2");

	return joint_commands;
};

