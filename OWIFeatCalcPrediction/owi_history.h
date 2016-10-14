#ifndef OWI_HISTORY_H
#define OWI_HISTORY_H

#include <vector>
#include <string>

//#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

class owi_history
{
public:
	std::vector <double> event_orientations;
	std::vector <cv::Point> event_centers;
	std::vector <double> event_times;
	std::vector <int> joint_commands;
};

#endif
