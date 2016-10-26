#ifndef AIS_EVENT_H
#define AIS_EVENT_H

#include <vector>
#include <string>

// see the notebook #12 for more information

namespace ais {

enum EVENT_TYPE { B_EVENT, GC_EVENT };

class event
{
public:
	event(double _time, EVENT_TYPE _event_type)
	{
		time = _time;
		event_type = _event_type;
	}
	virtual bool compare_events(event& another_event) =0;

	double time;
	EVENT_TYPE event_type;
};


//#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

class b_event : event
{
public:
	b_event(double _time, cv::Point _center, double _orientation) : event(_time, B_EVENT)
	{
		center = _center;
		orientation = _orientation;
	}
	bool compare_events(event& another_event);

	cv::Point center;
	double orientation;
};

class g_event : event
{
public:
	g_event(double _time, int _joint_command) : event(_time, GC_EVENT)
	{
		joint_command = _joint_command;
	}
	bool compare_events(event& another_event);

	int joint_command;
};


}

#endif
