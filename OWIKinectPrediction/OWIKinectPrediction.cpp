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

	std::vector<std::string> joint_commands = get_joint_commands();
	
	for (auto& s : joint_commands)
	{
		std::replace( s.begin(), s.end(), ' ', '_'); 
	}

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
        
		c_kinect_image orig_img;
        if (!orig_img.read_file(img_path))
		{
			std::cout << "can't open image file: "  << *it << std::endl;
			continue;
        }

		if (!prev_feat_img.empty())
		{
			// add history record for the g_command
			std::size_t found1 = (*it).find("_");
			std::size_t found2 = (*it).find(".");
			std::string cmd_name;

			if (found1 >= 0 && found2 >= 0 && found2 > found1) cmd_name=(*it).substr(found1+1,found2-found1-1);
			for (size_t icmd=0; icmd < joint_commands.size(); icmd++)
			{
				if (joint_commands[icmd]==cmd_name)
				{
					std::vector<double> param_value;
					param_value.push_back(icmd);
					// the image is taken after the command
					double cmd_time=cur_time - (double)OWI_COMMAND_DURATION_MILLISECONDS;
					ais::g_ais.history.add_event(ais::c_event(cmd_time, ais::ACTUATOR_COMMAND_EVENT, param_value));

					break;
				}
			}

			cv::Mat diff_feat_img;
			cv::absdiff(feat_img,prev_feat_img, diff_feat_img);
/*
			cv::imshow("diff_feat_img", diff_feat_img);
			cv::waitKey(0);
*/

			// find convex hull for major connected components
			std::vector<cv::Vec2i> component_areas;
			cv::Mat labelImage;
			get_connected_components(diff_feat_img, labelImage, component_areas);
            int max_num_components=1;
			for (int area_label=1; area_label < std::min(max_num_components+1, (int)component_areas.size()); area_label++)
			{
				cv::Mat component_img = labelImage;
				for(int r = 0; r < labelImage.rows; ++r){
					for(int c = 0; c < labelImage.cols; ++c){
						int label = labelImage.at<int>(r, c);
						if (label != area_label)
							component_img.at<int>(r, c) = 0;
					 }
				}

				cv::Mat component_hull;
				calc_convexHull(component_img, component_hull);
				cv::Mat feat_event;
				cv::bitwise_and(feat_img, component_hull, feat_event);
/*
				cv::imshow("feat_event", feat_event);
				cv::waitKey(0);
*/
				calc_event_mc_and_orientation(feat_event, cur_time);
			}
		}

		prev_feat_img = feat_img;

		// interpret (correctly classify) the observed events by comparing them with the predicted events 
		ais::interpret_observed_events_and_update_prediction_map(cur_time);
	}

#if defined(LOGGING) && 0
		ais::g_ais.history.print();
#endif

	return result;
}

