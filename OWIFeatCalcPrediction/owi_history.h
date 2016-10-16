#ifndef OWI_HISTORY_H
#define OWI_HISTORY_H

#include <vector>
#include <string>

//#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

class b_event
{
public:
	b_event(double _time, cv::Point _center, double _orientation)
	{
		time = _time;
		center = _center;
		orientation = _orientation;
	}

	double time;
	cv::Point center;
	double orientation;
};

class g_command_event
{
public:
	g_command_event(double _time, int _joint_command)
	{
		time = _time;
		joint_command = _joint_command;
	}
	double time;
	int joint_command;
};

class owi_history
{
public:
	std::vector <b_event> b_events;
	std::vector <g_command_event> g_events;
};

#endif
