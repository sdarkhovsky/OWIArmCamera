#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>

#include <algorithm>
#include <functional>
#include <vector>
#include <dirent.h>
#include <sys/types.h>

#if 0
#include "opencv2/highgui/highgui.hpp"
//#include <opencv2/core/utility.hpp>
//#include "opencv2/core/cuda.hpp"
//#include "opencv2/highgui.hpp"
//#include "opencv2/objdetect.hpp"
//#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#endif 

#include <unistd.h>

#include "ais.h"
#include "../OWIArmControl/owi_arm_control.h"

#include "logger.hpp"

std::vector<std::string> get_joint_commands();

namespace ais {
	extern c_ais g_ais;
}

// found at http://www.cplusplus.com/forum/unices/3548/
// read_directory()
//   Return an ASCII-sorted vector of filename entries in a given directory.
//   If no path is specified, the current working directory is used.
//
//   Always check the value of the global 'errno' variable after using this
//   function to see if anything went wrong. (It will be zero if all is well.)
//
int get_file_number(std::string a)
{
	if (a.size()>3) a = a.substr(3);

	std::size_t found = a.find("_");
	if (found >= 0) a = a.substr(0,found);

	return std::stoi(a);
}

// returns -1 if no command is present in the file name
int get_owi_command_from_file_name(std::string& file_name) {

    std::size_t found1 = file_name.find("_");
    std::size_t found2 = file_name.find(".");
    std::string cmd_name;
    int owi_command = -1;
    
	std::vector<std::string> joint_commands = get_joint_commands();
	
	for (auto& s : joint_commands)
	{
		std::replace( s.begin(), s.end(), ' ', '_'); 
	}    

    if (found1 >= 0 && found2 >= 0 && found2 > found1) cmd_name=file_name.substr(found1+1,found2-found1-1);
    for (size_t icmd=0; icmd < joint_commands.size(); icmd++)
    {
        if (joint_commands[icmd]==cmd_name)
        {
            owi_command = icmd;
            break;
        }
    }
    return owi_command;
}

std::vector <std::string> read_directory( const std::string& path = std::string() )
{
  std::vector <std::string> result;
  dirent* de;
  DIR* dp;
  errno = 0;
  dp = opendir( path.empty() ? "." : path.c_str() );
  if (dp)
    {
    while (true)
      {
      errno = 0;
      de = readdir( dp );
      if (de == NULL) break;
	  std::string s(de->d_name);
	  if (s == "." || s == "..")
		continue;
      result.push_back(s);
      }
    closedir( dp );

	struct {
	    bool operator()(std::string a, std::string b)
	    {   
			int a1 = get_file_number(a);
			int b1 = get_file_number(b);
	        return a1 < b1;
	    }   
	} sort_by_names;

    std::sort( result.begin(), result.end(), sort_by_names );
    }
  return result;
}

/*
command line: OWIKinectPrediction [<training_samples_directory>]   # Without terminating slash. 
by default the training images are searched at <this_executable_path>/../TrainingSamples
May need: export LD_LIBRARY_PATH=/usr/local/lib before running the program
*/
int main(int argc, char** argv)
{
    int result = 0;
	std::string training_samples_directory;

	if (argc == 1)
	{
		char buf[500]; 
		if (readlink("/proc/self/exe", buf, sizeof(buf)) > 0)
		{
			std::string sbuf = buf;
			std::size_t found = sbuf.find_last_of("/\\");
			training_samples_directory = sbuf.substr(0,found+1) + "../TrainingSamples";
		}
	}	
	else
	if (argc == 2)
		training_samples_directory = argv[1];
	else
	{
		printf("usage: %s [<training_samples_directory>]   (without terminating slash)", argv[0]);
		exit(1);
	}

	std::vector <std::string> img_files = read_directory(training_samples_directory);

 	for (std::vector<std::string>::iterator it = img_files.begin() ; it != img_files.end(); ++it)
	{
		// the image is taken after the command
		cur_time = (double)(OWI_COMMAND_DURATION_MILLISECONDS*get_file_number(*it));

		std::string img_path =  training_samples_directory + "/" + *it;
        
		c_kinect_image kinect_img;
        c_point_cloud point_cloud;
        
        if (!kinect_img.read_file( img_path, point_cloud ))
		{
			std::cout << "can't open image file: "  << *it << std::endl;
			continue;
        }
        
        g_ais.world.add_observation(point_cloud );
        
        int owi_cmd = get_owi_command_from_file_name(*it);
        if (owi_cmd >= 0) {
            // todo: add the command event 
            /*
                std::vector<double> param_value;
                param_value.push_back(owi_cmd);
                // the image is taken after the command
                double cmd_time=cur_time - (double)OWI_COMMAND_DURATION_MILLISECONDS;
                ais::g_ais.history.add_event(ais::c_event(cmd_time, ais::ACTUATOR_COMMAND_EVENT, param_value));
            */            
        }

        g_ais.world.predict();
	}

#if defined(LOGGING) && 0
//		ais::g_ais.history.print();
#endif

	return result;
}
